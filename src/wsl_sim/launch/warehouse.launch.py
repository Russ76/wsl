# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Full WSL system: Gazebo warehouse, robot, ROS-Gazebo bridge, perception, SLAM, RViz.

    ros2 launch wsl_sim warehouse.launch.py [headless:=true] [slam:=false] ...

Startup ordering needs no sleeps: ros_gz_sim `create` waits for the world's
create service and for the latched robot_description; the perception node and
RTAB-Map wait for TF and data with bounded timeouts and warn while waiting.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    ('world', 'warehouse',
     'World in wsl_sim/worlds (file name without .sdf; must match the SDF world name).'),
    ('headless', 'false', 'Run only the Gazebo server; sensors still render offscreen.'),
    ('rviz', 'true', 'Start RViz.'),
    ('perception', 'true', 'Start the point-cloud perception node.'),
    ('slam', 'true', 'Start RTAB-Map. RViz uses map as fixed frame if true, odom otherwise.'),
    ('ground_truth', 'false', 'Publish evaluation-only ground truth on /ground_truth/odom.'),
    ('x', '-12.0', 'Spawn x [m] in the world frame.'),
    ('y', '9.0', 'Spawn y [m] in the world frame.'),
    ('yaw', '0.0', 'Spawn yaw [rad] in the world frame.'),
    ('lidar_channels', '16', 'Vertical LiDAR channels.'),
    ('lidar_horizontal_samples', '900', 'LiDAR samples per revolution.'),
    ('lidar_rate_hz', '10', 'LiDAR rotation rate [Hz].'),
    ('camera_rate_hz', '20', 'Camera frame rate [Hz].'),
    ('odom_wheel_radius_scale', '1.0', 'Odometry wheel radius / true radius (calibration error).'),
    ('odom_wheel_separation_scale', '1.0', 'Odometry wheel separation / true separation.'),
]

ROBOT_ARGS = [
    'lidar_channels', 'lidar_horizontal_samples', 'lidar_rate_hz', 'camera_rate_hz',
    'odom_wheel_radius_scale', 'odom_wheel_separation_scale', 'ground_truth',
]


def generate_launch_description():
    sim_share = get_package_share_directory('wsl_sim')
    arg = LaunchConfiguration
    sim_time = {'use_sim_time': True}

    xacro_command = ['xacro ', PathJoinSubstitution(
        [get_package_share_directory('wsl_description'), 'urdf', 'wsl_amr.urdf.xacro'])]
    for name in ROBOT_ARGS:
        xacro_command += [f' {name}:=', arg(name)]
    robot_description = ParameterValue(Command(xacro_command), value_type=str)

    gz_args = [
        PathJoinSubstitution([sim_share, 'worlds', [arg('world'), '.sdf']]),
        PythonExpression(["' -r -s --headless-rendering' if '", arg('headless'),
                          "' == 'true' else ' -r'"]),
    ]
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description}, sim_time])

    # z = 0: base_footprint is on the floor and the wheels are tangent to it, so the
    # 126 kg robot does not drop. From 2 cm it lands at 0.4 m/s with 0.7 mm penetration.
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', arg('world'), '-name', 'wsl_amr', '-topic', 'robot_description',
                   '-x', arg('x'), '-y', arg('y'), '-z', '0.0', '-Y', arg('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
        parameters=[{'config_file': os.path.join(sim_share, 'config', 'bridge.yaml')}])

    ground_truth_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ground_truth_bridge',
        output='screen', condition=IfCondition(arg('ground_truth')),
        parameters=[{'config_file': os.path.join(sim_share, 'config', 'bridge_ground_truth.yaml')}])

    perception = Node(
        package='wsl_perception', executable='perception_node', name='perception', output='screen',
        condition=IfCondition(arg('perception')),
        parameters=[os.path.join(get_package_share_directory('wsl_perception'),
                                 'config', 'perception.yaml'), sim_time])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wsl_slam'), 'launch', 'slam.launch.py')),
        condition=IfCondition(arg('slam')))

    rviz = Node(
        package='rviz2', executable='rviz2', output='log', condition=IfCondition(arg('rviz')),
        arguments=['-d', os.path.join(sim_share, 'rviz', 'warehouse.rviz'),
                   '-f', PythonExpression(["'map' if '", arg('slam'), "' == 'true' else 'odom'"])],
        parameters=[sim_time])

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=description)
         for name, default, description in ARGUMENTS] +
        [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(sim_share, 'models')),
         # Parent of the package share, so package://wsl_description/... resolves in Gazebo.
         AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(
             get_package_share_directory('wsl_description'))),
         gazebo, robot_state_publisher, spawn, bridge, ground_truth_bridge, perception, slam, rviz])
