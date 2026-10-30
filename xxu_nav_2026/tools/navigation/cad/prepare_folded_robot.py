#!/usr/bin/env python3
"""Build the fixed folded vehicle from saved CAD meshes and mass properties.

Source CAD stays local. Mesh omissions, contact approximations and corrections
are recorded in the generated manifest; none are claimed as real measurements.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
import trimesh

from convert_ega_sentry import material_for

C = np.array([[1., 0, 0], [0, 0, -1.], [0, 1., 0]])


def text(parent, tag, value):
    el = ET.SubElement(parent, tag)
    el.text = value if isinstance(value, str) else ' '.join(f'{v:.12g}' for v in value)
    return el


def inertia_matrix(values):
    # SW stores positive products of inertia, opposite the tensor off-diagonal.
    x, y, z = values[9:12]
    return np.diag(values[6:9]) - np.array([[0, x, y], [x, 0, z], [y, z, 0]])


def parallel(mass, delta):
    return mass * (np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta))


def add_inertial(link, mass, center, tensor):
    eigen = np.linalg.eigvalsh(tensor)
    if mass <= 0 or eigen[0] <= 0 or eigen[-1] > eigen[:2].sum() + 1e-7:
        raise ValueError(f'Nonphysical inertia for {link.get("name")}: {eigen}')
    el = ET.SubElement(link, 'inertial')
    text(el, 'mass', str(mass))
    text(el, 'pose', [*center, 0, 0, 0])
    it = ET.SubElement(el, 'inertia')
    for name, val in zip(['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'],
                         [*tensor.diagonal(), tensor[0, 1], tensor[0, 2], tensor[1, 2]]):
        text(it, name, str(val))


def prepare(source, decoded, reader, upstream, output, cover_mode='closed'):
    sys.path.insert(0, str(reader / 'src'))
    from sldprt.blocks import read_blocks

    main = next(source.rglob('底盘总装V2-哨ver-0517.SLDASM'))
    data = main.read_bytes()
    metadata = json.loads((decoded / 'sentry_decode.json').read_text())
    if metadata['source_sha256'] != hashlib.sha256(data).hexdigest():
        raise ValueError('Decoded geometry belongs to a different CAD assembly')
    blocks = read_blocks(data)
    tree = ET.fromstring(next(b.inflate() for b in blocks if b.name == 'swXmlContents/COMPINSTANCETREE'))
    local = lambda el: el.tag.rsplit('}', 1)[-1]
    models = {el.get('id'): el for el in tree.iter() if local(el) == 'swModel'}
    configs = next(el for el in tree if local(el) == 'swConfigurationList')
    cfg = next((el for el in configs if el.get('swMostRecentConfiguration') == 'YES'), configs[0])
    instances = {}

    def walk(mid, parent=np.eye(4), path=''):
        model = models[mid]
        for el in model:
            if local(el) != 'swReference' or el.get('swSuppressed') == 'YES' or el.get('swHidden') == 'YES':
                continue
            world = np.array([float(v) for v in el.get('swTransform').split()]).reshape(4, 4) @ parent
            name = f"{el.get('swName')}-{el.get('swReferenceNumber')}@{model.get('swName')}"
            full = f'{path}/{name}' if path else name
            instances[full] = (el.get('swName'), world)
            if el.get('swModelRef') in models:
                walk(el.get('swModelRef'), world, full)
    walk(cfg.get('swModelRef'))

    def mass_props(filename):
        p = next(source.rglob(filename))
        for b in read_blocks(p.read_bytes()):
            if b.name != 'docProps/ISolidWorksInformation.xml':
                continue
            if not b.verify():
                raise ValueError(f'Mass property CRC failed: {filename}')
            for el in ET.fromstring(b.inflate()).iter():
                if el.tag == 'property' and el.get('name') == 'SW-MassProp-Config-0':
                    return np.array([float(v) for v in ''.join(el.itertext()).split(',')])
        raise ValueError(f'Missing native mass properties: {filename}')

    output.mkdir(parents=True, exist_ok=True)
    meshes = output / 'meshes'
    meshes.mkdir(exist_ok=True)
    arrays = np.load(decoded / 'sentry_components_mm.npz')
    wheels = {}
    sensors = {}
    for path, (name, world) in instances.items():
        if name == '全向轮13滚子-0517':
            center = C @ (world[3, :3] + np.array([0, 0, -0.0115]) @ world[:3, :3])
            label = ('f' if center[0] > 0 else 'r') + ('l' if center[1] > 0 else 'r')
            axis = C @ world[2, :3]
            if axis @ center < 0:
                axis = -axis
            angle = np.arctan2(axis[1], axis[0])
            rotation = Rotation.from_euler('xyz', [0, np.pi / 2, angle]).as_matrix()
            wheels[path] = dict(name=f'wheel_{label}', center=center, rotation=rotation,
                                radius=0.08, width=0.03, axis=axis)
        elif name == 'mid360激光雷达-0517':
            frame = 'livox_lidar' if not sensors else 'livox_lidar_secondary'
            sensors[path] = dict(name=frame + '_housing', frame=frame,
                                 center=C @ world[3, :3], rotation=C @ world[:3, :3].T @ C.T,
                                 world=world)
    if len(wheels) != 4 or len(sensors) != 2:
        raise ValueError(f'Unexpected wheel/sensor layout: {len(wheels)}, {len(sensors)}')
    # The saved assembly has two lidars; the real vehicle has only the
    # original navigation lidar. Keep both source records for mass correction.
    source_sensors = sensors
    sensors = {path: item for path, item in source_sensors.items()
               if item['frame'] == 'livox_lidar'}
    removed_sensors = {path: item for path, item in source_sensors.items()
                       if path not in sensors}
    scenes = {'base_link': trimesh.Scene()}
    for item in [*wheels.values(), *sensors.values()]:
        scenes[item['name']] = trimesh.Scene()
    collision_groups = {}
    all_points = []
    skipped_reference = []
    cover_paths = []
    for part in metadata['parts']:
        path = part['name']
        is_cover = path.rsplit('/', 1)[-1].startswith('雷达保护罩-')
        if path in removed_sensors:
            continue
        removed_mount = any(prefix.split('/')[0] == path.split('/')[0]
                            for prefix in removed_sensors)
        if is_cover and (cover_mode == 'removed' or removed_mount):
            cover_paths.append(path)
            continue
        points = arrays[f'v{part["index"]}'].astype(float) @ C.T / 1000
        faces = arrays[f'f{part["index"]}']
        mesh = trimesh.Trimesh(points, faces, process=False)
        if path.rsplit('/', 1)[-1].startswith('定位模块UW01-'):
            bodies = mesh.split(only_watertight=False)
            bodies = [body for body in bodies if max(body.extents) < .1]
            if not bodies:
                raise ValueError('Reference filtering removed the UWB receiver')
            mesh = trimesh.util.concatenate(bodies)
        if path.rsplit('/', 1)[-1].startswith('全向轮范围-'):
            skipped_reference.append(path)
            continue
        all_points.append(mesh.vertices)
        owner = next((item for prefix, item in wheels.items() if path.startswith(prefix + '/')), None)
        if owner is None:
            owner = sensors.get(path)
        if is_cover and cover_mode == 'transparent':
            owner = next(item for prefix, item in sensors.items()
                         if prefix.split('/')[0] == path.split('/')[0])
        link_name = owner['name'] if owner else 'base_link'
        if owner:
            mesh.vertices = (mesh.vertices - owner['center']) @ owner['rotation']
        else:
            top = path.split('/')[0]
            group = top if top.startswith(('折叠双pitch', 'mid360快拆')) else 'lower_chassis'
            collision_groups.setdefault(group, []).append(mesh.vertices.copy())
        mesh = trimesh.graph.smooth_shade(mesh, angle=np.radians(20))
        label, color, metallic = material_for(path)
        if is_cover and cover_mode == 'transparent':
            label, color, metallic = 'transparent_cover', [180, 205, 220, 70], 0.0
        mesh.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(
            name=label, baseColorFactor=color, metallicFactor=metallic, roughnessFactor=.65,
            doubleSided=True, alphaMode='BLEND' if is_cover and cover_mode == 'transparent' else 'OPAQUE'))
        scenes[link_name].add_geometry(mesh, geom_name=f'part_{part["index"]}')
    points = np.vstack(all_points)
    for wheel in wheels.values():
        vertices = np.vstack([mesh.vertices for mesh in scenes[wheel['name']].geometry.values()])
        wheel['radius'] = float(np.linalg.norm(vertices[:, :2], axis=1).max())
        wheel['width'] = float(np.ptp(vertices[:, 2]))
    for sensor in sensors.values():
        vertices = np.vstack([mesh.vertices for mesh in scenes[sensor['name']].geometry.values()])
        # Livox manual v1.2, pp. 12/14/19: O is 47 mm above the base;
        # +X points away from the connector (CAD +X points toward it).
        sensor['optical_offset'] = np.array([0., 0., vertices[:, 2].min() + .047])
        sensor['optical_rotation'] = Rotation.from_euler('z', np.pi).as_matrix()
        sensor['imu_offset'] = sensor['optical_offset'] + sensor['optical_rotation'] @ np.array([.011, .02329, -.04412])
    base_height = max(w['radius'] - w['center'][2] for w in wheels.values())
    bounds = np.array([points.min(0), points.max(0)])
    radius = float(np.linalg.norm(points[:, :2], axis=1).max())
    for name, scene in scenes.items():
        # Consolidate render batches while preserving each material and every
        # saved triangle. Hundreds of tiny CAD draw calls are unnecessary.
        materials = {}
        for mesh in scene.geometry.values():
            materials.setdefault(mesh.visual.material.name, []).append(mesh)
        scene = trimesh.Scene()
        for label, chunks in materials.items():
            mesh = trimesh.util.concatenate(chunks)
            mesh.visual = trimesh.visual.TextureVisuals(material=chunks[0].visual.material)
            scene.add_geometry(mesh, geom_name=label)
        scenes[name] = scene
        scene.export(meshes / f'{name}.glb')
    for old in meshes.glob('livox_lidar*.glb'):
        if old.stem not in scenes:
            old.unlink()

    root_props = mass_props(main.name)
    sensor_props = mass_props('mid360激光雷达-0517.SLDPRT')
    wheel_props = mass_props('全向轮13滚子-0517.SLDASM')
    total_mass = root_props[5]
    total_center = C @ root_props[:3]
    corrected = inertia_matrix(root_props)
    # The two purchased lidar CAD files override mass but retain enormous
    # impossible cached inertias. Remove precisely those tensors, preserving
    # their masses/COM and all parallel-axis terms already in the root tensor.
    sensor_size = np.array([.065, .065, .060])
    sensor_inertia = sensor_props[5] / 12 * np.diag([
        sensor_size[1]**2 + sensor_size[2]**2,
        sensor_size[0]**2 + sensor_size[2]**2,
        sensor_size[0]**2 + sensor_size[1]**2])
    for item in source_sensors.values():
        r = item['world'][:3, :3].T
        corrected -= r @ inertia_matrix(sensor_props) @ r.T
        corrected += C.T @ item['rotation'] @ sensor_inertia @ item['rotation'].T @ C
        item['mass'] = float(sensor_props[5])
        item['com'] = C @ sensor_props[:3]
        item['inertia'] = sensor_inertia
    corrected = C @ corrected @ C.T
    source_center = total_center.copy()
    removed_components = []
    for item in removed_sensors.values():
        removed_components.append((item['mass'], item['center'] + item['rotation'] @ item['com'],
                                   item['rotation'] @ item['inertia'] @ item['rotation'].T))
    if cover_paths:
        prop = mass_props('雷达保护罩-0517.SLDPRT')
        for path in cover_paths:
            world = instances[path][1]
            rotation = C @ world[:3, :3].T
            center = C @ (prop[:3] @ world[:3, :3] + world[3, :3])
            removed_components.append((prop[5], center, rotation @ inertia_matrix(prop) @ rotation.T))
    if removed_components:
        new_mass = total_mass - sum(x[0] for x in removed_components)
        new_center = (total_mass * total_center - sum(m * c for m, c, _ in removed_components)) / new_mass
        corrected += parallel(total_mass, total_center - new_center)
        for m, c, tensor in removed_components:
            corrected -= tensor + parallel(m, c - new_center)
        total_mass, total_center = new_mass, new_center
    for item in wheels.values():
        item['mass'] = float(wheel_props[5])
        # Wheel tensor is nearly axisymmetric; preserve its CAD principal
        # values about the measured wheel centre in the generated axle frame.
        item['com'] = np.zeros(3)
        item['inertia'] = np.diag(wheel_props[6:9])
    children = [*wheels.values(), *sensors.values()]
    body_mass = total_mass - sum(item['mass'] for item in children)
    body_center = (total_mass * total_center - sum(
        item['mass'] * (item['center'] + item['rotation'] @ item['com']) for item in children)) / body_mass
    body_inertia = corrected + parallel(total_mass, total_center - body_center)
    for item in children:
        com = item['center'] + item['rotation'] @ item['com']
        body_inertia -= item['rotation'] @ item['inertia'] @ item['rotation'].T + parallel(item['mass'], com - body_center)

    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name='ega_folded')
    text(model, 'self_collide', 'false')
    body = ET.SubElement(model, 'link', name='base_link')
    text(body, 'pose', [0, 0, base_height, 0, 0, 0])
    add_inertial(body, body_mass, body_center, body_inertia)
    links = {'base_link': body}
    urdf = ET.Element('robot', name='ega_folded')
    ET.SubElement(urdf, 'link', name='base_footprint')

    def urdf_link(name, mass, com, tensor):
        link = ET.SubElement(urdf, 'link', name=name)
        inert = ET.SubElement(link, 'inertial')
        ET.SubElement(inert, 'origin', xyz=' '.join(map(str, com)), rpy='0 0 0')
        ET.SubElement(inert, 'mass', value=str(mass))
        ET.SubElement(inert, 'inertia', **dict(zip(
            ['ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'], map(str,
            [*tensor.diagonal(), tensor[0, 1], tensor[0, 2], tensor[1, 2]]))))
        visual = ET.SubElement(link, 'visual')
        geom = ET.SubElement(visual, 'geometry')
        ET.SubElement(geom, 'mesh', filename=f'package://xxu_bringup/models/ega_folded/meshes/{name}.glb')
        return link

    def urdf_joint(name, parent, child, xyz, rpy, joint_type='fixed'):
        j = ET.SubElement(urdf, 'joint', name=name, type=joint_type)
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))
        if joint_type == 'continuous':
            ET.SubElement(j, 'axis', xyz='0 0 1')
            ET.SubElement(j, 'limit', effort='5', velocity='30')
        return j

    urdf_link('base_link', body_mass, body_center, body_inertia)
    urdf_joint('footprint_joint', 'base_footprint', 'base_link', [0, 0, base_height], [0, 0, 0])
    upstream_model = ET.parse(upstream / 'models/sentry/model.sdf').getroot().find('model')
    for item in children:
        name = item['name']
        rpy = Rotation.from_matrix(item['rotation']).as_euler('xyz')
        item['rpy'] = rpy
        link = ET.SubElement(model, 'link', name=name)
        text(link, 'pose', [*item['center'], *rpy]).set('relative_to', 'base_link')
        add_inertial(link, item['mass'], item['com'], item['inertia'])
        links[name] = link
        moving = name.startswith('wheel_')
        joint = ET.SubElement(model, 'joint', name=name + '_joint', type='revolute' if moving else 'fixed')
        text(joint, 'parent', 'base_link')
        text(joint, 'child', name)
        if moving:
            axis = ET.SubElement(joint, 'axis')
            text(axis, 'xyz', [0, 0, 1])
            lim = ET.SubElement(axis, 'limit')
            for tag, val in [('lower', '-1e16'), ('upper', '1e16'), ('effort', '5'), ('velocity', '30')]:
                text(lim, tag, val)
            dyn = ET.SubElement(axis, 'dynamics')
            text(dyn, 'damping', '0.001')
            collision = ET.SubElement(link, 'collision', name=name + '_contact')
            cyl = ET.SubElement(ET.SubElement(collision, 'geometry'), 'cylinder')
            text(cyl, 'radius', str(item['radius']))
            text(cyl, 'length', str(item['width']))
            friction = ET.SubElement(ET.SubElement(ET.SubElement(collision, 'surface'), 'friction'), 'ode')
            text(friction, 'mu', '0.05')
            text(friction, 'mu2', '2.0')
            text(friction, 'fdir1', '0 0 1')
            plugin = ET.SubElement(model, 'plugin', filename='gz-sim-joint-controller-system', name='gz::sim::systems::JointController')
            text(plugin, 'joint_name', name + '_joint')
            text(plugin, 'topic', '/' + name + '/cmd_vel')
        else:
            for sensor in upstream_model.find("link[@name='livox_lidar']").findall('sensor'):
                sensor = copy.deepcopy(sensor)
                if sensor.get('type') == 'custom':
                    sensor.set('name', 'RGLLidar_' + name)
                    text(sensor, 'pose', [*item['optical_offset'], 0, 0, np.pi])
                    plugin = sensor.find('plugin')
                    plugin.remove(plugin.find('pattern_preset_path'))
                    text(plugin, 'pattern_preset', 'Livox Mid360')
                    plugin.find('topic').text = '/rm_sim/lidar/points'
                    plugin.find('frame').text = item['frame']
                    plugin.find('range/min').text = '0.1'
                    plugin.find('range/max').text = '40'
                else:
                    sensor.set('name', name + '_imu')
                    text(sensor, 'pose', [*item['imu_offset'], 0, 0, np.pi])
                    sensor.find('topic').text = '/livox/imu'
                    sensor.find('frame').text = item['frame'] + '_imu'
                    text(sensor, 'gz_frame_id', item['frame'] + '_imu')
                link.append(sensor)
        urdf_link(name, item['mass'], item['com'], item['inertia'])
        if moving:
            collision = ET.SubElement(urdf.find(f"link[@name='{name}']"), 'collision')
            ET.SubElement(ET.SubElement(collision, 'geometry'), 'cylinder',
                          radius=str(item['radius']), length=str(item['width']))
        urdf_joint(name + '_joint', 'base_link', name, item['center'], rpy, 'continuous' if moving else 'fixed')
        if not moving:
            for frame, offset in [(item['frame'], item['optical_offset']),
                                  (item['frame'] + '_imu', item['imu_offset'])]:
                ET.SubElement(urdf, 'link', name=frame)
                urdf_joint(frame + '_frame_joint', name, frame, offset, [0, 0, np.pi])
    for name, link in links.items():
        visual = ET.SubElement(link, 'visual', name=name + '_visual')
        text(ET.SubElement(ET.SubElement(visual, 'geometry'), 'mesh'), 'uri', f'meshes/{name}.glb')

    collision_files = set()
    collision_triangle_counts = {}
    self_filter_hulls = []
    for i, (group, chunks) in enumerate(collision_groups.items()):
        vertices = np.vstack(chunks)
        if max(np.ptp(vertices, axis=0)) < .025:
            continue
        try:
            hull = trimesh.convex.convex_hull(vertices)
            if len(hull.faces) > 200:
                hull = hull.simplify_quadric_decimation(face_count=200, aggression=0)
        except Exception as error:
            raise ValueError(f'Could not build collision group {group}') from error
        if hull.volume < 1e-9:
            continue
        self_filter_hulls.append(ConvexHull(hull.vertices).equations.tolist())
        filename = f'collision_{i}.stl'
        collision_files.add(filename)
        collision_triangle_counts[filename] = len(hull.faces)
        hull.export(meshes / filename)
        collision = ET.SubElement(body, 'collision', name=f'body_collision_{i}')
        text(ET.SubElement(ET.SubElement(collision, 'geometry'), 'mesh'), 'uri', 'meshes/' + filename)
        ucollision = ET.SubElement(urdf.find("link[@name='base_link']"), 'collision')
        ET.SubElement(ET.SubElement(ucollision, 'geometry'), 'mesh', filename=f'package://xxu_bringup/models/ega_folded/meshes/{filename}')
    for old in meshes.glob('collision_*.stl'):
        if old.name not in collision_files:
            old.unlink()
    js = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system', name='gz::sim::systems::JointStatePublisher')
    text(js, 'topic', 'joint_states')
    for item in wheels.values():
        text(js, 'joint_name', item['name'] + '_joint')
    pose = ET.SubElement(model, 'plugin', filename='gz-sim-pose-publisher-system', name='gz::sim::systems::PosePublisher')
    for tag in ['publish_link_pose', 'publish_model_pose', 'use_pose_vector_msg']:
        text(pose, tag, 'true')
    text(pose, 'update_frequency', '100')
    ET.SubElement(urdf, 'link', name='lio_base_sensor')
    urdf_joint('lio_sensor_joint', 'base_link', 'lio_base_sensor', [0, 0, 0], [0, 0, 0])
    ET.indent(sdf)
    ET.indent(urdf)
    ET.ElementTree(sdf).write(output / 'model.sdf', encoding='utf-8', xml_declaration=True)
    ET.ElementTree(urdf).write(output / 'robot.urdf', encoding='utf-8', xml_declaration=True)
    config = ET.Element('model')
    text(config, 'name', 'EGA fixed folded sentry')
    text(config, 'version', '1.0')
    text(config, 'sdf', 'model.sdf').set('version', '1.9')
    text(config, 'description', 'User folded CAD with four driven omni wheels and one fixed Mid360 sensor; secondary lidar removed, mounting hardware retained; uncalibrated contact and actuator model.')
    ET.indent(config)
    ET.ElementTree(config).write(output / 'model.config', encoding='utf-8', xml_declaration=True)
    primary = next(iter(sensors.values()))
    primary_rpy = Rotation.from_matrix(primary['rotation'] @ primary['optical_rotation']).as_euler('xyz')
    geometry = dict(base_height=base_height, collision_height=float(bounds[1, 2] + base_height), visual_bounds_m=bounds.tolist(),
                    require_stopped_goal=True,
                    measured_xy_envelope_radius=radius, robot_radius=float(np.ceil((radius + .02) * 1000) / 1000),
                    primary_sensor=dict(offset=(primary['center'] + primary['rotation'] @ primary['optical_offset']).tolist(),
                                        imu_offset=(primary['center'] + primary['rotation'] @ primary['imu_offset']).tolist(),
                                        rpy=primary_rpy.tolist()),
                    self_filter=dict(frame='lio_base_sensor', hull_planes=self_filter_hulls, tolerance=.005),
                    wheels=[dict(name=w['name'], center=w['center'].tolist(), axis=w['axis'].tolist(),
                                 radius=w['radius'], width=w['width']) for w in wheels.values()],
                    max_wheel_speed=30., max_wheel_acceleration=80.)
    (output / 'geometry.json').write_text(json.dumps(geometry, indent=2) + '\n')
    manifest = dict(source_assembly=main.name, source_sha256=metadata['source_sha256'], configuration=cfg.get('swName'),
                    fixed_folded_configuration=True, native_mass_kg=float(root_props[5]),
                    native_com_m=source_center.tolist(), operating_mass_kg=float(total_mass),
                    operating_com_m=total_center.tolist(), corrected_inertia_kg_m2=corrected.tolist(),
                    lidar_cover_mode=cover_mode, removed_covers=cover_paths,
                    active_lidar_count=len(sensors), source_lidar_count=len(source_sensors),
                    removed_lidars=[dict(source_instance=path, mass_kg=item['mass'])
                                    for path, item in removed_sensors.items()],
                    removed_lidar_mount_hardware='retained',
                    real_lidar_cover_handling_verified=False,
                    inertia_correction='Subtract two nonphysical lidar cached tensors, replace with 65x65x60 mm box tensors; retain CAD mass, COM and parallel-axis terms.',
                    rejected_raw_inertia_kg_m2=inertia_matrix(root_props).tolist(),
                    rejected_raw_inertia_frame='SolidWorks saved model frame (Y-up)',
                    mesh_coverage=metadata['coverage'], omitted_reference_meshes=skipped_reference,
                    contact_model='CAD physical wheel envelope cylinders (reference range bodies excluded); anisotropic Coulomb friction mu=0.05, mu2=2, axle direction; initial uncalibrated values',
                    actuator_model='Ideal joint velocity controllers with wheel speed/acceleration limits; no identified motor/current/power model',
                    sensor_origin='Livox manual v1.2: optical O 47 mm above CAD bottom, +X away from connector; IMU at [11,23.29,-44.12] mm in optical frame. Mounts from saved CAD transforms; no real extrinsic calibration.',
                    sensor_origin_source='https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf',
                    real_vehicle_mass_verified=False, real_friction_verified=False,
                    collision_model='Four convex groups: lower chassis, folded mechanism, two lidar mounts; quality-preserving decimation targeting 200 faces without forcing the target; wheel cylinders',
                    collision_triangle_counts=collision_triangle_counts,
                    primary_lidar='livox_lidar')
    (output / 'physical_properties.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(mass=total_mass, base_height=base_height, robot_radius=geometry['robot_radius'],
                          size_m=(bounds[1] - bounds[0]).tolist(), meshes=len(scenes),
                          collision_hulls=len(body.findall('collision'))), indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source', 'decoded', 'reader', 'upstream', 'output']:
        parser.add_argument(name, type=Path)
    parser.add_argument('--cover-mode', choices=['closed', 'removed', 'transparent'], default='closed')
    args = parser.parse_args()
    prepare(args.source, args.decoded, args.reader, args.upstream, args.output, args.cover_mode)
