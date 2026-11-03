# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Black-box test of perception_node: real executable, real TF, synthetic clouds."""

import os
import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from vision_msgs.msg import Detection3DArray

CONFIG = os.path.join(get_package_share_directory('wsl_perception'), 'config', 'perception.yaml')
NODE = os.path.join(
    get_package_prefix('wsl_perception'), 'lib', 'wsl_perception', 'perception_node')


@pytest.mark.launch_test
def generate_test_description():
    perception = launch_ros.actions.Node(
        package='wsl_perception', executable='perception_node', name='perception',
        parameters=[CONFIG], output='screen')
    # LiDAR 0.66 m above base_footprint, as on the robot.
    static_tf = launch_ros.actions.Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--z', '0.66', '--frame-id', 'base_footprint', '--child-frame-id', 'lidar_link'])
    return launch.LaunchDescription([perception, static_tf, launch_testing.actions.ReadyToTest()])


def synthetic_scene():
    """Floor grid 0.66 m below the sensor plus a 0.6 x 1.0 x 1.0 m box 3 m ahead."""
    xs, ys = np.meshgrid(np.arange(-6, 6, 0.1), np.arange(-6, 6, 0.1))
    floor = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, -0.66)], axis=1)
    by, bz = np.meshgrid(np.arange(-0.5, 0.5, 0.04), np.arange(0.0, 1.0, 0.04))
    face = np.stack([np.full(by.size, 3.0), by.ravel(), bz.ravel() - 0.66], axis=1)
    return np.vstack([floor, face]).astype(np.float32)


class TestPerceptionNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('perception_test_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_detects_single_obstacle(self):
        detections = []
        self.node.create_subscription(
            Detection3DArray, '/perception/clusters', detections.append, 10)
        publisher = self.node.create_publisher(
            PointCloud2, '/lidar/points', qos_profile_sensor_data)
        points = synthetic_scene()

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not detections:
            header = Header(frame_id='lidar_link', stamp=self.node.get_clock().now().to_msg())
            publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertTrue(detections, 'no /perception/clusters message received')
        result = detections[-1]
        self.assertEqual(result.header.frame_id, 'base_footprint')
        self.assertEqual(len(result.detections), 1)
        bbox = result.detections[0].bbox
        self.assertAlmostEqual(bbox.center.position.x, 3.0, delta=0.1)
        self.assertAlmostEqual(bbox.center.position.y, 0.0, delta=0.1)
        # Ground removal keeps the box from merging with the floor.
        self.assertLess(bbox.size.x, 0.3)

    def test_rejects_inconsistent_runtime_reconfiguration(self):
        # Atomic: both values reach the callback together, so the cross-field
        # check sees min_range_m > max_range_m and neither value is applied.
        client = self.node.create_client(
            SetParametersAtomically, '/perception/set_parameters_atomically')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = SetParametersAtomically.Request(parameters=[Parameter(
            name='min_range_m',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=8.0))])
        request.parameters.append(Parameter(
            name='max_range_m',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=5.0)))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        result = future.result().result
        self.assertFalse(result.successful)
        self.assertIn('max_range_m', result.reason)


class TestInvalidStartupConfiguration(unittest.TestCase):

    def run_node(self, *params):
        args = [NODE, '--ros-args', '-r', '__node:=perception_invalid']
        for p in params:
            args += ['-p', p]
        return subprocess.run(args, capture_output=True, text=True, timeout=30)

    def test_out_of_range_parameter_fails_fast(self):
        result = self.run_node('voxel_leaf_size_m:=-0.1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('voxel_leaf_size_m', result.stderr + result.stdout)

    def test_inconsistent_parameters_fail_fast(self):
        result = self.run_node('min_range_m:=5.0', 'max_range_m:=2.0')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('max_range_m must be > min_range_m', result.stderr + result.stdout)
