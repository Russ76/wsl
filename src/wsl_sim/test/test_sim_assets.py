# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Consistency checks across the world, the asset models, the bridge and the robot model."""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions

SIM_DIR = os.environ.get('WSL_SIM_DIR', os.path.join(os.path.dirname(__file__), '..'))
WORLD = os.path.join(SIM_DIR, 'worlds', 'warehouse.sdf')


def load_bridge(name):
    with open(os.path.join(SIM_DIR, 'config', name)) as f:
        return yaml.safe_load(f)


def model_uris(root):
    return [inc.find('uri').text for inc in root.iter('include')]


def test_world_name_matches_file_name():
    # warehouse.launch.py passes the file stem to ros_gz_sim create as -world.
    assert ET.parse(WORLD).getroot().find('world').get('name') == 'warehouse'


def test_all_model_includes_resolve():
    pending = list(model_uris(ET.parse(WORLD).getroot()))
    seen = set()
    while pending:
        uri = pending.pop()
        assert uri.startswith('model://'), uri
        name = uri[len('model://'):]
        if name in seen:
            continue
        seen.add(name)
        model_dir = os.path.join(SIM_DIR, 'models', name)
        assert os.path.isfile(os.path.join(model_dir, 'model.config')), name
        sdf = ET.parse(os.path.join(model_dir, 'model.sdf')).getroot()
        pending.extend(model_uris(sdf))
    assert seen == {'wsl_rack', 'wsl_rack_row', 'wsl_pallet_stack', 'wsl_pillar'}


def test_include_names_are_unique():
    names = [inc.find('name').text for inc in ET.parse(WORLD).getroot().iter('include')]
    assert len(names) == len(set(names))


def test_world_has_required_systems():
    plugins = {p.get('name') for p in ET.parse(WORLD).getroot().iter('plugin')}
    assert {'gz::sim::systems::Physics', 'gz::sim::systems::Sensors',
            'gz::sim::systems::Imu', 'gz::sim::systems::SceneBroadcaster'} <= plugins


@pytest.mark.parametrize('config', ['bridge.yaml', 'bridge_ground_truth.yaml'])
def test_bridge_entries_are_unidirectional(config):
    entries = load_bridge(config)
    assert len({e['ros_topic_name'] for e in entries}) == len(entries)
    for entry in entries:
        assert entry['direction'] in ('GZ_TO_ROS', 'ROS_TO_GZ'), entry
        expected = 'ROS_TO_GZ' if entry['ros_topic_name'] == '/cmd_vel' else 'GZ_TO_ROS'
        assert entry['direction'] == expected, entry


def test_ground_truth_is_isolated():
    main = load_bridge('bridge.yaml')
    gt = load_bridge('bridge_ground_truth.yaml')
    assert not any('ground_truth' in e['ros_topic_name'] or 'ground_truth' in e['gz_topic_name']
                   for e in main)
    assert all(e['ros_topic_name'].startswith('/ground_truth/') for e in gt)


def test_bridged_gazebo_topics_exist_on_the_robot():
    """A typo in either file would silently produce a bridge with no data."""
    urdf = os.path.join(
        get_package_share_directory('wsl_description'), 'urdf', 'wsl_amr.urdf.xacro')
    robot = ET.fromstring(xacro.process_file(urdf).toxml())
    tags = ('topic', 'odom_topic', 'tf_topic', 'camera_info_topic')
    robot_topics = {el.text for tag in tags for el in robot.iter(tag)}
    lidar = robot.find('.//sensor[@name="lidar"]/topic').text
    robot_topics.add(lidar + '/points')  # gpu_lidar publishes the cloud on <topic>/points
    for entry in load_bridge('bridge.yaml') + load_bridge('bridge_ground_truth.yaml'):
        if entry['gz_topic_name'] != '/clock':
            assert entry['gz_topic_name'] in robot_topics, entry['gz_topic_name']


def test_robot_meshes_resolve_in_gazebo_and_rviz():
    """package:// is converted to model://<package>/ for Gazebo; RViz uses the ament index."""
    share = get_package_share_directory('wsl_description')
    urdf = os.path.join(share, 'urdf', 'wsl_amr.urdf.xacro')
    meshes = {mesh.get('filename')
              for mesh in ET.fromstring(xacro.process_file(urdf).toxml()).iter('mesh')}
    assert meshes
    for uri in meshes:
        assert uri.startswith('package://wsl_description/'), uri
        assert os.path.isfile(os.path.join(share, uri[len('package://wsl_description/'):])), uri

    launch_file = os.path.join(SIM_DIR, 'launch', 'warehouse.launch.py')
    description = PythonLaunchDescriptionSource(launch_file).get_launch_description(LaunchContext())
    paths = [perform_substitutions(LaunchContext(), entity.value)
             for entity in description.entities
             if isinstance(entity, AppendEnvironmentVariable)
             and perform_substitutions(LaunchContext(), entity.name) == 'GZ_SIM_RESOURCE_PATH']
    assert os.path.dirname(share) in paths


@pytest.mark.parametrize('model', ['wsl_rack', 'wsl_pallet_stack', 'wsl_pillar'])
def test_static_assets_use_a_single_collision_envelope(model):
    """Per-part collision boxes made physics the bottleneck (RTF 0.51 vs 0.99)."""
    sdf = ET.parse(os.path.join(SIM_DIR, 'models', model, 'model.sdf')).getroot()
    assert len(list(sdf.iter('collision'))) == 1
    assert len(list(sdf.iter('visual'))) >= 2
