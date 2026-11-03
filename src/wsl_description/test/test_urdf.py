# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Structural and physical checks on the expanded WSL AMR model."""

import math
import os
import shutil
import struct
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
from urdf_parser_py.urdf import Box, Mesh, Sphere, URDF

URDF_DIR = os.environ.get(
    'WSL_DESCRIPTION_URDF_DIR',
    os.path.join(os.path.dirname(__file__), '..', 'urdf'))
MODEL = os.path.join(URDF_DIR, 'wsl_amr.urdf.xacro')
PACKAGE_DIR = os.path.dirname(URDF_DIR)
MESH_DIR = os.path.join(PACKAGE_DIR, 'meshes', 'rbsummit_steel')

WHEELS = ('front_left', 'front_right', 'back_left', 'back_right')
ALL_LINKS = {
    'base_footprint', 'base_link', 'lidar_link', 'camera_link', 'camera_optical_frame',
    'imu_link', *(f'{wheel}_wheel_link' for wheel in WHEELS),
}
ALL_JOINTS = {
    'base_joint', 'lidar_joint', 'camera_joint', 'camera_optical_joint', 'imu_joint',
    *(f'{wheel}_wheel_joint' for wheel in WHEELS),
}
WHEEL_RADIUS = 0.127
WHEEL_BASE = 0.445
WHEEL_SEPARATION = 0.409
LIDAR_HEIGHT = 0.66
LIDAR_LOWEST_RING_RAD = math.radians(15.0)
# Traction direction per wheel: the axis of the roller on the ground.
ROLLER_AXIS = {'front_left': (1, -1), 'front_right': (1, 1), 'back_left': (1, 1),
               'back_right': (1, -1)}


def expand(**args):
    mappings = {k: str(v) for k, v in args.items()}
    return xacro.process_file(MODEL, mappings=mappings).toprettyxml(indent='  ')


@pytest.fixture(scope='module')
def urdf_xml():
    return expand()


@pytest.fixture(scope='module')
def robot(urdf_xml):
    return URDF.from_xml_string(urdf_xml)


def rpy_to_matrix(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def mesh_path(filename):
    prefix = 'package://wsl_description/'
    assert filename.startswith(prefix), filename
    return os.path.join(PACKAGE_DIR, filename[len(prefix):])


def stl_vertices(path):
    with open(path, 'rb') as f:
        data = f.read()
    count = struct.unpack('<I', data[80:84])[0]
    assert len(data) == 84 + 50 * count, f'{path} is not a binary STL'
    record = np.dtype([('normal', '<3f4'), ('vertices', '<9f4'), ('attribute', '<u2')])
    return np.frombuffer(data, record, count=count, offset=84)['vertices'].reshape(-1, 3)


def dae_vertices(path):
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    assert root.find('c:asset/c:up_axis', ns).text == 'Z_UP'
    assert float(root.find('c:asset/c:unit', ns).get('meter')) == 1.0
    arrays = root.findall('.//c:library_geometries//c:source[1]/c:float_array', ns)
    return np.vstack([np.fromstring(a.text, sep=' ').reshape(-1, 3) for a in arrays])


def top_cover_visual(robot):
    return next(v for v in robot.link_map['base_link'].visuals
                if isinstance(v.geometry, Mesh) and 'top_cover' in v.geometry.filename)


def chassis_collision(robot):
    return next(c for c in robot.link_map['base_link'].collisions if isinstance(c.geometry, Box))


def test_tree_structure(robot):
    assert robot.get_root() == 'base_footprint'
    assert {link.name for link in robot.links} == ALL_LINKS
    assert {joint.name for joint in robot.joints} == ALL_JOINTS
    for joint in robot.joints:
        if joint.name.endswith('wheel_joint'):
            assert joint.type == 'continuous'
            assert list(joint.axis) == [0, 1, 0]
        else:
            assert joint.type == 'fixed', f'{joint.name} should be fixed'


def test_inertia_is_physical(robot):
    """Each inertia tensor must be positive definite and obey the triangle inequality."""
    for link in robot.links:
        if link.inertial is None:
            continue
        assert link.inertial.mass > 0.0, link.name
        i = link.inertial.inertia
        tensor = np.array([[i.ixx, i.ixy, i.ixz], [i.ixy, i.iyy, i.iyz], [i.ixz, i.iyz, i.izz]])
        eig = np.linalg.eigvalsh(tensor)
        assert np.all(eig > 0.0), f'{link.name}: inertia not positive definite {eig}'
        a, b, c = eig
        assert a + b >= c and a + c >= b and b + c >= a, \
            f'{link.name}: violates triangle inequality'


def test_total_mass_is_plausible(robot):
    total = sum(link.inertial.mass for link in robot.links if link.inertial)
    assert 125.0 < total < 127.0


def test_wheels_touch_ground(robot):
    base_joint = robot.joint_map['base_joint']
    axle_height = base_joint.origin.xyz[2]
    assert axle_height == pytest.approx(WHEEL_RADIUS)
    for wheel in WHEELS:
        link = robot.link_map[f'{wheel}_wheel_link']
        assert link.collisions[0].geometry.radius == pytest.approx(axle_height)
        assert robot.joint_map[f'{wheel}_wheel_joint'].origin.xyz[2] == pytest.approx(0.0)


def test_wheel_layout_is_symmetric(robot):
    origins = {w: robot.joint_map[f'{w}_wheel_joint'].origin.xyz for w in WHEELS}
    for wheel, (x, y, _) in origins.items():
        assert x == pytest.approx(WHEEL_BASE / 2 if 'front' in wheel else -WHEEL_BASE / 2)
        assert y == pytest.approx(WHEEL_SEPARATION / 2 if 'left' in wheel
                                  else -WHEEL_SEPARATION / 2)


def test_only_wheels_touch_ground(robot):
    axle_height = robot.joint_map['base_joint'].origin.xyz[2]
    chassis = chassis_collision(robot)
    bottom = axle_height + chassis.origin.xyz[2] - chassis.geometry.size[2] / 2
    assert 0.06 < bottom < 0.07, f'chassis bottom at {bottom} m'


def test_centre_of_mass_cannot_tip_under_drive_acceleration(robot):
    axle_height = robot.joint_map['base_joint'].origin.xyz[2]
    masses = 0.0
    positions = np.zeros(3)
    parents = {j.child: j for j in robot.joints}
    for link in robot.links:
        if link.inertial is None:
            continue
        origin = np.array(link.inertial.origin.xyz)
        name = link.name
        while name != 'base_link':
            joint = parents[name]
            origin = rpy_to_matrix(joint.origin.rpy) @ origin + np.array(joint.origin.xyz)
            name = joint.parent
        masses += link.inertial.mass
        positions += link.inertial.mass * origin
    com = positions / masses
    assert abs(com[0]) < 0.01 and abs(com[1]) < 0.01, f'centre of mass off centre: {com}'
    height = axle_height + com[2]
    max_drive_acceleration = 1.0
    tip_acceleration = 9.81 * (WHEEL_SEPARATION / 2) / height
    assert tip_acceleration > 5.0 * max_drive_acceleration, f'CoM {height} m above ground'


def test_visual_meshes_resolve_and_are_never_collision(robot):
    for link in robot.links:
        for collision in link.collisions:
            assert not isinstance(collision.geometry, Mesh), f'{link.name} collides with a mesh'
        for visual in link.visuals:
            if isinstance(visual.geometry, Mesh):
                assert os.path.isfile(mesh_path(visual.geometry.filename)), \
                    visual.geometry.filename


def test_mesh_licence_ships_with_the_meshes():
    with open(os.path.join(MESH_DIR, 'LICENSE')) as f:
        assert 'Robotnik Automation S.L.' in f.read()


def test_chassis_collision_matches_mesh(robot):
    vertices = stl_vertices(os.path.join(MESH_DIR, 'rbsummit_steel_chassis.stl'))
    chassis = chassis_collision(robot)
    centre = np.array(chassis.origin.xyz)
    half = np.array(chassis.geometry.size) / 2
    np.testing.assert_allclose(centre - half, vertices.min(axis=0), atol=2e-4)
    np.testing.assert_allclose(centre + half, vertices.max(axis=0), atol=2e-4)


def test_wheel_geometry_matches_mesh(robot):
    for wheel in WHEELS:
        link = robot.link_map[f'{wheel}_wheel_link']
        vertices = dae_vertices(mesh_path(link.visuals[0].geometry.filename))
        assert np.hypot(vertices[:, 0], vertices[:, 2]).max() == pytest.approx(WHEEL_RADIUS,
                                                                               abs=1e-3)
        np.testing.assert_allclose(vertices.mean(axis=0), 0.0, atol=0.01)


def test_roller_direction_on_ground_matches_friction_direction(robot, urdf_xml):
    """A wheel mirrored in the URDF would look right and drive sideways."""
    fdir = {}
    for gazebo in ET.fromstring(urdf_xml).iter('gazebo'):
        element = gazebo.find('.//fdir1')
        if element is not None:
            assert element.get('{http://gazebosim.org/schema}expressed_in') == 'base_footprint'
            fdir[gazebo.get('reference')] = [float(v) for v in element.text.split()]
    for wheel, (x, y) in ROLLER_AXIS.items():
        link = f'{wheel}_wheel_link'
        assert fdir[link] == [x, y, 0.0]
        vertices = dae_vertices(mesh_path(robot.link_map[link].visuals[0].geometry.filename))
        lowest = vertices[vertices[:, 2] < vertices[:, 2].min() + 0.006][:, :2]
        lowest = lowest - lowest.mean(axis=0)
        ridge = np.linalg.eigh(lowest.T @ lowest)[1][:, 1]
        assert abs(ridge @ np.array([x, y]) / math.sqrt(2)) > 0.95, f'{wheel}: rollers {ridge}'


def test_top_cover_does_not_z_fight(robot):
    """The cover must be clear of the recess floor and coplanar with no chassis face."""
    chassis_top = stl_vertices(os.path.join(MESH_DIR, 'rbsummit_steel_chassis.stl'))[:, 2].max()
    cover_z = top_cover_visual(robot).origin.xyz[2]
    assert cover_z >= chassis_top - 1e-5
    recess_floor = 0.38062
    assert cover_z + 0.0015 - recess_floor >= 0.003


def test_no_helper_visuals(robot):
    assert not robot.link_map['base_footprint'].visuals


def test_every_stl_visual_has_a_colour(robot):
    for link in robot.links:
        for visual in link.visuals:
            is_dae = isinstance(visual.geometry, Mesh) and visual.geometry.filename.endswith('.dae')
            assert is_dae or visual.material is not None, f'{link.name}: no material'


def test_camera_optical_frame_convention(robot):
    """Optical z must point along base x, optical x along base -y, optical y along base -z."""
    rot = rpy_to_matrix(robot.joint_map['camera_optical_joint'].origin.rpy)
    np.testing.assert_allclose(rot @ [0, 0, 1], [1, 0, 0], atol=1e-9)
    np.testing.assert_allclose(rot @ [1, 0, 0], [0, -1, 0], atol=1e-9)
    np.testing.assert_allclose(rot @ [0, 1, 0], [0, 0, -1], atol=1e-9)


def test_lidar_height(robot):
    axle_height = robot.joint_map['base_joint'].origin.xyz[2]
    lidar_z = robot.joint_map['lidar_joint'].origin.xyz[2]
    assert axle_height + lidar_z == pytest.approx(LIDAR_HEIGHT)


def test_lidar_lowest_ring_clears_the_robot(robot):
    """Every downward ray must pass above the highest point of the body at its radius."""
    lidar = np.array(robot.joint_map['lidar_joint'].origin.xyz)
    chassis = chassis_collision(robot)
    half = np.array(chassis.geometry.size) / 2
    corners_x = np.abs(lidar[0]) + half[0]
    corner_radius = math.hypot(corners_x, half[1])
    top = top_cover_visual(robot).origin.xyz[2] + 0.0015
    drop = corner_radius * math.tan(LIDAR_LOWEST_RING_RAD)
    assert lidar[2] - drop > top, f'ring is at {lidar[2] - drop} m over the corner, cover {top} m'

    camera = np.array(robot.joint_map['camera_joint'].origin.xyz)
    radius = math.hypot(camera[0] + 0.015 - lidar[0], 0.05)
    assert lidar[2] - radius * math.tan(LIDAR_LOWEST_RING_RAD) > camera[2] + 0.015


def test_camera_is_outside_the_chassis(robot):
    camera = robot.joint_map['camera_joint'].origin.xyz
    chassis = chassis_collision(robot)
    back_face = camera[0] - 0.015
    assert back_face > chassis.geometry.size[0] / 2
    assert chassis.origin.xyz[2] - chassis.geometry.size[2] / 2 < camera[2] \
        < chassis.origin.xyz[2] + chassis.geometry.size[2] / 2


def test_imu_is_inside_the_chassis_on_the_rotation_axis(robot):
    imu = robot.joint_map['imu_joint'].origin.xyz
    chassis = chassis_collision(robot)
    assert imu[0] == pytest.approx(0.0) and imu[1] == pytest.approx(0.0)
    assert imu[2] == pytest.approx(chassis.origin.xyz[2])


def find_plugin(root, name):
    return [p for p in root.iter('plugin') if p.get('name') == name]


def test_sensor_frames_match_links(urdf_xml):
    root = ET.fromstring(urdf_xml)
    links = {link.get('name') for link in root.iter('link')}
    sensors = list(root.iter('sensor'))
    assert {s.get('name') for s in sensors} == {'lidar', 'camera', 'imu'}
    for sensor in sensors:
        assert sensor.find('gz_frame_id').text in links


def test_lidar_horizontal_scan_does_not_overlap(urdf_xml):
    horizontal = ET.fromstring(urdf_xml).find('.//sensor[@name="lidar"]//horizontal')
    samples = int(horizontal.find('samples').text)
    span = float(horizontal.find('max_angle').text) - float(horizontal.find('min_angle').text)
    assert span == pytest.approx(2 * math.pi * (samples - 1) / samples)


def test_drive_joints_exist_and_are_published(urdf_xml, robot):
    root = ET.fromstring(urdf_xml)
    drive = find_plugin(root, 'gz::sim::systems::MecanumDrive')[0]
    states = find_plugin(root, 'gz::sim::systems::JointStatePublisher')[0]
    for wheel in WHEELS:
        joint = f'{wheel}_wheel_joint'
        assert drive.find(f'{wheel}_joint').text == joint
        assert joint in {e.text for e in states.findall('joint_name')}
    assert drive.find('topic').text == '/wsl/cmd_vel'
    assert drive.find('odom_topic').text == '/wsl/odom'
    assert drive.find('tf_topic').text == '/wsl/tf'
    assert drive.find('frame_id').text == 'odom'
    assert drive.find('child_frame_id').text == 'base_footprint'


def test_drive_limits_use_the_names_the_harmonic_plugin_reads(urdf_xml):
    """Per-axis limit names are ignored silently by gz-sim 8, leaving the drive unlimited."""
    drive = find_plugin(ET.fromstring(urdf_xml), 'gz::sim::systems::MecanumDrive')[0]
    assert float(drive.find('max_acceleration').text) == pytest.approx(1.0)
    assert float(drive.find('min_acceleration').text) == pytest.approx(-1.0)
    assert float(drive.find('max_velocity').text) == pytest.approx(1.5)
    assert float(drive.find('min_velocity').text) == pytest.approx(-1.5)
    tags = {element.tag for element in drive}
    assert not {tag for tag in tags if '_linear_' in tag or '_angular_' in tag}


def test_odometry_calibration_scales(robot):
    root = ET.fromstring(expand(odom_wheel_radius_scale=0.98, odom_wheel_separation_scale=1.05))
    drive = find_plugin(root, 'gz::sim::systems::MecanumDrive')[0]
    assert float(drive.find('wheel_radius').text) == pytest.approx(WHEEL_RADIUS * 0.98)
    assert float(drive.find('wheel_separation').text) == pytest.approx(WHEEL_SEPARATION * 1.05)
    assert float(drive.find('wheelbase').text) == pytest.approx(WHEEL_BASE * 1.05)


def test_ground_truth_can_be_disabled():
    name = 'gz::sim::systems::OdometryPublisher'
    assert len(find_plugin(ET.fromstring(expand()), name)) == 1
    assert len(find_plugin(ET.fromstring(expand(ground_truth='false')), name)) == 0


def test_wheels_collide_as_spheres(robot):
    for wheel in WHEELS:
        geometry = robot.link_map[f'{wheel}_wheel_link'].collisions[0].geometry
        assert isinstance(geometry, Sphere)


@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='check_urdf not installed')
def test_check_urdf(urdf_xml, tmp_path):
    path = tmp_path / 'wsl_amr.urdf'
    path.write_text(urdf_xml)
    result = subprocess.run(['check_urdf', str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
