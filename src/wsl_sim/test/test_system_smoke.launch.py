# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
End-to-end smoke test: the real warehouse launch, headless.

Proves that Gazebo starts, the robot spawns, every sensor publishes plausible
data with the right frames, the TF tree has single authorities, and the robot
drives forward, backward and turns both ways. Requires an OpenGL-capable
renderer (Mesa llvmpipe works), hence opt-in via -DWSL_SIM_TESTS=ON.
"""

import math
import os
import time
import unittest

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
import tf2_ros


@pytest.mark.launch_test
def generate_test_description():
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wsl_sim'), 'launch', 'warehouse.launch.py')),
        launch_arguments={'headless': 'true', 'rviz': 'false', 'slam': 'false',
                          'perception': 'true'}.items())
    return LaunchDescription([system, launch_testing.actions.ReadyToTest()])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestSystemSmoke(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('smoke_test', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        cls.messages = {}
        cls.tf_publishers = {}

        def store(topic):
            return lambda msg: cls.messages.setdefault(topic, []).append(msg)

        # Reliable: a best-effort subscriber loses most 920 KB camera frames to
        # DDS fragment drops (measured 2.4 of 20 Hz), which would test the
        # transport rather than the simulator.
        for topic, msg_type in [('/lidar/points', PointCloud2), ('/camera/image_raw', Image),
                                ('/camera/camera_info', CameraInfo), ('/imu/data', Imu),
                                ('/odom', Odometry)]:
            cls.node.create_subscription(msg_type, topic, store(topic), 10)

        def record_tf_authority(msg, info=None):
            for t in msg.transforms:
                cls.tf_publishers.setdefault((t.header.frame_id, t.child_frame_id), set())
        cls.node.create_subscription(TFMessage, '/tf', record_tf_authority, 100)
        cls.cmd_vel = cls.node.create_publisher(Twist, '/cmd_vel', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout_s, what):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(f'timed out after {timeout_s} s waiting for {what}')

    def spin_sim(self, sim_seconds, timeout_s=120.0):
        start = self.node.get_clock().now()
        self.spin_until(
            lambda: (self.node.get_clock().now() - start) >= Duration(seconds=sim_seconds),
            timeout_s, f'{sim_seconds} s of simulated time')

    def rate_hz(self, topic):
        stamps = [Time.from_msg(m.header.stamp).nanoseconds * 1e-9 for m in self.messages[topic]]
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])

    def test_1_sensors(self):
        topics = ['/lidar/points', '/camera/image_raw', '/camera/camera_info', '/imu/data', '/odom']
        # Startup includes the Gazebo server boot and spawning.
        self.spin_until(lambda: all(len(self.messages.get(t, [])) >= 2 for t in topics),
                        180.0, 'all sensor topics')
        self.messages.clear()
        self.spin_sim(5.0)

        cloud = self.messages['/lidar/points'][-1]
        self.assertEqual(cloud.header.frame_id, 'lidar_link')
        self.assertEqual((cloud.height, cloud.width), (16, 900))
        xyz = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'))
        finite = xyz[np.all(np.isfinite(xyz), axis=1)]
        self.assertGreater(len(finite), 5000)
        ranges = np.linalg.norm(finite, axis=1)
        self.assertGreaterEqual(ranges.min(), 0.29)
        self.assertLessEqual(ranges.max(), 30.01)
        # Lowest ring (-15 deg) hits the floor 0.66 m below the sensor at ~2.5 m.
        self.assertAlmostEqual(np.percentile(finite[:, 2], 1), -0.66, delta=0.05)
        # The spawn point is open space, nothing is within 2 m: a closer return is a self-hit.
        self.assertGreater(ranges.min(), 2.0, 'LiDAR sees the robot')

        image = self.messages['/camera/image_raw'][-1]
        self.assertEqual(image.header.frame_id, 'camera_optical_frame')
        self.assertEqual((image.width, image.height, image.encoding), (640, 480, 'rgb8'))
        pixels = np.frombuffer(image.data, dtype=np.uint8)
        self.assertGreater(pixels.std(), 5.0, 'camera image is uniform (rendering failed?)')
        info = self.messages['/camera/camera_info'][-1]
        self.assertEqual(info.header.frame_id, 'camera_optical_frame')
        # fx = (w / 2) / tan(hfov / 2) with hfov = 1.2 rad.
        self.assertAlmostEqual(info.k[0], 320.0 / math.tan(0.6), delta=1.0)

        imu = self.messages['/imu/data'][-1]
        self.assertEqual(imu.header.frame_id, 'imu_link')
        self.assertAlmostEqual(imu.linear_acceleration.z, 9.81, delta=0.2)
        self.assertLess(abs(imu.angular_velocity.z), 0.02)

        self.assertAlmostEqual(self.rate_hz('/lidar/points'), 10.0, delta=1.0)
        self.assertAlmostEqual(self.rate_hz('/camera/image_raw'), 20.0, delta=2.0)
        self.assertAlmostEqual(self.rate_hz('/imu/data'), 100.0, delta=10.0)
        self.assertAlmostEqual(self.rate_hz('/odom'), 50.0, delta=5.0)

    def test_2_tf_tree(self):
        self.spin_until(
            lambda: self.tf_buffer.can_transform('odom', 'camera_optical_frame', Time()),
            60.0, 'odom -> camera_optical_frame')
        lidar = self.tf_buffer.lookup_transform('base_footprint', 'lidar_link', Time())
        self.assertAlmostEqual(lidar.transform.translation.x, 0.0, places=3)
        self.assertAlmostEqual(lidar.transform.translation.z, 0.66, places=3)
        # Without SLAM nothing may publish map -> odom; odometry is the only dynamic root edge.
        self.assertIn(('odom', 'base_footprint'), self.tf_publishers)
        self.assertFalse(any(parent == 'map' for parent, _ in self.tf_publishers))

    def drive(self, linear, angular, sim_seconds):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        start = self.node.get_clock().now()
        self.spin_until(lambda: bool(self.messages.get('/odom')), 30.0, '/odom')
        odom0 = self.messages['/odom'][-1].pose.pose
        while (self.node.get_clock().now() - start) < Duration(seconds=sim_seconds):
            self.cmd_vel.publish(twist)
            self.spin_sim(0.1)
        self.cmd_vel.publish(Twist())
        self.spin_sim(1.0)
        odom1 = self.messages['/odom'][-1].pose.pose
        dyaw = math.atan2(math.sin(yaw_of(odom1.orientation) - yaw_of(odom0.orientation)),
                          math.cos(yaw_of(odom1.orientation) - yaw_of(odom0.orientation)))
        heading = yaw_of(odom0.orientation)
        dx = odom1.position.x - odom0.position.x
        dy = odom1.position.y - odom0.position.y
        forward = dx * math.cos(heading) + dy * math.sin(heading)
        return forward, dyaw

    def test_3_driving(self):
        forward, dyaw = self.drive(0.5, 0.0, 3.0)
        self.assertGreater(forward, 1.0)
        self.assertLess(abs(dyaw), 0.05)

        backward, dyaw = self.drive(-0.3, 0.0, 2.0)
        self.assertLess(backward, -0.3)
        self.assertLess(abs(dyaw), 0.05)

        _, dyaw = self.drive(0.0, 0.5, 2.0)  # +z: counter-clockwise, left
        self.assertGreater(dyaw, 0.5)
        _, dyaw = self.drive(0.0, -0.5, 2.0)
        self.assertLess(dyaw, -0.5)

        forward, dyaw = self.drive(0.4, 0.3, 2.0)
        self.assertGreater(forward, 0.3)
        self.assertGreater(dyaw, 0.2)
