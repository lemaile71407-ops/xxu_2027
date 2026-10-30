#!/usr/bin/env python3
"""Check generated SDF/URDF mass balance, transforms and navigation envelope."""
import argparse
from datetime import date
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def transform(values):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    result[:3, 3] = values[:3]
    return result


def validate(folder):
    geometry = json.loads((folder / 'geometry.json').read_text())
    properties = json.loads((folder / 'physical_properties.json').read_text())
    model = ET.parse(folder / 'model.sdf').getroot().find('model')
    robot = ET.parse(folder / 'robot.urdf').getroot()
    urdf_poses = {'base_footprint': np.eye(4)}
    pending = list(robot.findall('joint'))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent not in urdf_poses:
                continue
            origin = joint.find('origin')
            values = [float(v) for v in (origin.get('xyz') + ' ' + origin.get('rpy')).split()]
            urdf_poses[joint.find('child').get('link')] = urdf_poses[parent] @ transform(values)
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError('URDF contains unresolved parents or a cycle')
    poses = {}
    for link in model.findall('link'):
        pose = link.find('pose')
        value = transform([float(v) for v in pose.text.split()])
        if pose.get('relative_to'):
            value = poses[pose.get('relative_to')] @ value
        poses[link.get('name')] = value
        np.testing.assert_allclose(value, urdf_poses[link.get('name')], atol=1e-10)
    items = []
    for link in model.findall('link'):
        pose = poses[link.get('name')]
        inertial = link.find('inertial')
        mass = float(inertial.findtext('mass'))
        com = np.array([float(v) for v in inertial.findtext('pose').split()])[:3]
        values = {v.tag: float(v.text) for v in inertial.find('inertia')}
        tensor = np.array([[values['ixx'], values['ixy'], values['ixz']],
                           [values['ixy'], values['iyy'], values['iyz']],
                           [values['ixz'], values['iyz'], values['izz']]])
        eig = np.linalg.eigvalsh(tensor)
        assert mass > 0 and eig[0] > 0 and eig[-1] <= eig[:2].sum() + 1e-8
        rotation = pose[:3, :3]
        items.append((mass, pose[:3, 3] + rotation @ com, rotation @ tensor @ rotation.T))
        for sensor in link.findall('sensor'):
            sensor_pose = transform([float(v) for v in sensor.findtext('pose').split()])
            frame = sensor.findtext('plugin/frame') or sensor.findtext('gz_frame_id')
            np.testing.assert_allclose(pose @ sensor_pose, urdf_poses[frame], atol=1e-10)
    mass = sum(item[0] for item in items)
    com = sum(item[0] * item[1] for item in items) / mass
    tensor = np.zeros((3, 3))
    for m, c, inertia in items:
        delta = c - com
        tensor += inertia + m * (np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta))
    expected_com = np.array(properties.get('operating_com_m', properties['native_com_m']))
    expected_com[2] += geometry['base_height']
    np.testing.assert_allclose(mass, properties.get('operating_mass_kg', properties['native_mass_kg']), atol=1e-9)
    np.testing.assert_allclose(com, expected_com, atol=1e-9)
    np.testing.assert_allclose(tensor, properties['corrected_inertia_kg_m2'], atol=1e-8)
    assert geometry['robot_radius'] >= geometry['measured_xy_envelope_radius'] + .01999
    assert len(geometry['wheels']) == 4
    assert len(model.findall("joint[@type='revolute']")) == 4
    assert len(model.findall("joint[@type='fixed']")) == 1
    assert len(model.findall(".//sensor[@type='custom']")) == 1
    assert len(model.findall(".//sensor[@type='imu']")) == 1
    assert 'secondary' not in ET.tostring(model, encoding='unicode')
    assert 'secondary' not in ET.tostring(robot, encoding='unicode')
    assert properties['active_lidar_count'] == 1
    assert not list((folder / 'meshes').glob('*secondary*'))
    for mesh in model.findall('.//mesh/uri'):
        assert (folder / mesh.text).is_file(), mesh.text
    report = dict(date=str(date.today()), result='passed', scope='Generated folded SDF/URDF and envelope; not a real-vehicle calibration',
                  sdf_urdf_link_pose_agreement=True, sensor_optical_imu_frame_agreement=True,
                  active_lidar_count=1, secondary_model_frames_and_meshes_removed=True,
                  total_mass_kg=mass, com_error_m=float(np.linalg.norm(com - expected_com)),
                  inertia_error_kg_m2=float(np.linalg.norm(tensor - properties['corrected_inertia_kg_m2'])),
                  positive_physical_link_inertias=True, navigation_radius_m=geometry['robot_radius'],
                  collision_hulls=len(model.findall("link[@name='base_link']/collision")))
    (folder / 'static_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_folder', type=Path)
    validate(parser.parse_args().model_folder)
