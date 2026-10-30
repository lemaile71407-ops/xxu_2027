#!/usr/bin/env python3
"""Add deterministic, matte arena materials to the supplied RMUL field.

The STL's existing raised paint/zone geometry supplies the boundaries. This
palette is a simulation reference, not official artwork. Collision and Nav2
assets are read-only. COLLADA carries the original triangles, UVs and materials.
"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image


PALETTE = {
    'pvc_floor': ('#343b43', 0.025, 5.0),
    'pvc_platform': ('#5c6570', 0.025, 5.0),
    'pvc_red': ('#b82e3b', 0.025, 5.0),
    'pvc_blue': ('#235fba', 0.025, 5.0),
    'pvc_control': ('#555849', 0.025, 5.0),
    'paint_white': ('#dedfd8', 0.025, 6.0),
    'paint_yellow': ('#ddba4b', 0.025, 6.0),
    'panel_graphite': ('#282e35', 0.035, 7.0),
    'ramp_grip': ('#3e454d', 0.020, 4.0),
    'base_dark': ('#1d2329', 0.020, 4.0),
}
STL_DTYPE = np.dtype([
    ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
COLLADA = 'http://www.collada.org/2005/11/COLLADASchema'


def read_triangles(path):
    raw = path.read_bytes()
    count = struct.unpack_from('<I', raw, 80)[0]
    if not count or len(raw) != 84+count*50:
        raise ValueError('Expected a complete binary STL')
    triangles = np.frombuffer(raw, dtype=STL_DTYPE, offset=84)['vertices'].astype(float)
    if not np.isfinite(triangles).all() or not np.allclose(
            [triangles.min((0, 1)), triangles.max((0, 1))],
            [[0, 0, -0.1], [12.6, 8.6, 0.42]], atol=1e-5):
        raise ValueError('Material regions are specific to the supplied RMUL field')
    return raw, triangles


def classify(triangles):
    centers = triangles.mean(1)
    normals = np.cross(triangles[:, 1]-triangles[:, 0],
                       triangles[:, 2]-triangles[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-20)
    labels = []
    for (x, y, z), normal in zip(centers, normals):
        team = 'pvc_red' if x < 6.3 else 'pvc_blue'
        edge = x < 0.31 or x > 12.29 or y < 0.31 or y > 8.29
        if z < -0.005 or normal[2] < -0.5:
            label = 'base_dark'
        elif normal[2] > 0.98:
            if abs(z) < 0.0005:
                label = 'pvc_floor'
            elif (abs(z-0.001) < 0.0004 or abs(z-0.003) < 0.0004
                  or abs(z-0.201) < 0.0004):
                label = team
            elif (abs(z-0.002) < 0.0004
                  or abs(z-0.202) < 0.0004 or abs(z-0.403) < 0.0004):
                label = 'paint_white'
            elif abs(z-0.010) < 0.0004:
                label = 'pvc_control'
            elif abs(z-0.011) < 0.0004:
                label = 'paint_yellow'
            elif edge and z > 0.41:
                label = 'paint_white'
            else:
                label = 'panel_graphite' if edge else 'pvc_platform'
        elif normal[2] > 0.15 and not edge:
            label = 'ramp_grip'
        else:
            label = 'panel_graphite'
        labels.append(label)
    return np.array(labels), normals


def texture(path, color, seed):
    rgb = np.array([int(color[i:i+2], 16) for i in (1, 3, 5)], dtype=float)
    rng = np.random.default_rng(seed)
    width, height = (2048, 1398) if path.stem == 'pvc_floor' else (256, 256)
    fine = rng.normal(0, 1.5, (height, width))
    coarse = rng.normal(0, 0.8, (32, 32)).astype(np.float32)
    coarse = np.asarray(Image.fromarray(coarse).resize((width, height), Image.Resampling.BICUBIC))
    grain = fine+coarse
    pixels = np.clip(rgb+grain[:, :, None], 0, 255).astype(np.uint8)
    if path.stem == 'pvc_floor':
        x = (np.arange(width)+0.5)/width*12.6
        y = (np.arange(height)+0.5)/height*8.6
        vertical = ((np.abs(x-0.42) <= 0.025) | (np.abs(x-12.18) <= 0.025))[None, :]
        horizontal = ((np.abs(y-0.42) <= 0.025) | (np.abs(y-8.18) <= 0.025))[:, None]
        boundary = ((vertical & ((y >= 0.42) & (y <= 8.18))[:, None]) |
                    (horizontal & ((x >= 0.42) & (x <= 12.18))[None, :]))
        pixels[boundary] = [222, 223, 216]
    Image.fromarray(pixels).save(path)


def write_collada(path, triangles, labels, normals):
    ET.register_namespace('', COLLADA)

    def element(parent, tag, text=None, **attributes):
        child = ET.SubElement(parent, f'{{{COLLADA}}}{tag}', attributes)
        if text is not None:
            child.text = text
        return child

    def values(array):
        return ' '.join(format(float(value), '.9g') for value in array.flat)

    root = ET.Element(f'{{{COLLADA}}}COLLADA', version='1.4.1')
    asset = element(root, 'asset')
    element(asset, 'created', '2026-10-18T00:00:00Z')
    element(asset, 'modified', '2026-10-18T00:00:00Z')
    element(asset, 'unit', name='meter', meter='1')
    element(asset, 'up_axis', 'Z_UP')
    images = element(root, 'library_images')
    effects = element(root, 'library_effects')
    materials = element(root, 'library_materials')
    geometries = element(root, 'library_geometries')
    scenes = element(root, 'library_visual_scenes')
    scene = element(scenes, 'visual_scene', id='Arena', name='Arena')

    for name in sorted(set(labels)):
        image = element(images, 'image', id=f'{name}_image', name=f'{name}_image')
        element(image, 'init_from', f'../materials/textures/{name}.png')
        effect = element(effects, 'effect', id=f'{name}_effect')
        profile = element(effect, 'profile_COMMON')
        surface = element(element(profile, 'newparam', sid=f'{name}_surface'), 'surface', type='2D')
        element(surface, 'init_from', f'{name}_image')
        sampler = element(element(profile, 'newparam', sid=f'{name}_sampler'), 'sampler2D')
        element(sampler, 'source', f'{name}_surface')
        element(sampler, 'wrap_s', 'WRAP')
        element(sampler, 'wrap_t', 'WRAP')
        phong = element(element(profile, 'technique', sid='common'), 'phong')
        element(element(phong, 'emission'), 'color', '0 0 0 1')
        element(element(phong, 'ambient'), 'color', '0.3 0.3 0.3 1')
        element(element(phong, 'diffuse'), 'texture', texture=f'{name}_sampler', texcoord='UVMap')
        specular, shininess = PALETTE[name][1:]
        element(element(phong, 'specular'), 'color', f'{specular} {specular} {specular} 1')
        element(element(phong, 'shininess'), 'float', str(shininess))
        material = element(materials, 'material', id=f'{name}_material', name=name)
        element(material, 'instance_effect', url=f'#{name}_effect')

        subset = triangles[labels == name]
        face_normals = normals[labels == name]
        positions = subset.reshape(-1, 3)
        normal_values = np.repeat(face_normals, 3, axis=0)
        uv = np.empty((len(subset), 3, 2))
        for index, (face, normal) in enumerate(zip(subset, face_normals)):
            axis = int(np.argmax(np.abs(normal)))
            coordinates = [1, 2] if axis == 0 else [0, 2] if axis == 1 else [0, 1]
            uv[index] = (face[:, :2]/[12.6, 8.6] if name == 'pvc_floor'
                         else face[:, coordinates]/0.30)
        geometry = element(geometries, 'geometry', id=f'{name}_geometry', name=name)
        mesh = element(geometry, 'mesh')
        for suffix, data, parameters in [
                ('positions', positions, ['X', 'Y', 'Z']),
                ('normals', normal_values, ['X', 'Y', 'Z']),
                ('uv', uv.reshape(-1, 2), ['S', 'T'])]:
            source_id = f'{name}_{suffix}'
            source = element(mesh, 'source', id=source_id)
            element(source, 'float_array', values(data), id=f'{source_id}_array', count=str(data.size))
            accessor = element(element(source, 'technique_common'), 'accessor',
                               source=f'#{source_id}_array', count=str(len(data)), stride=str(data.shape[1]))
            for parameter in parameters:
                element(accessor, 'param', name=parameter, type='float')
        vertices = element(mesh, 'vertices', id=f'{name}_vertices')
        element(vertices, 'input', semantic='POSITION', source=f'#{name}_positions')
        faces = element(mesh, 'triangles', count=str(len(subset)), material=f'{name}_symbol')
        element(faces, 'input', semantic='VERTEX', source=f'#{name}_vertices', offset='0')
        element(faces, 'input', semantic='NORMAL', source=f'#{name}_normals', offset='1')
        element(faces, 'input', semantic='TEXCOORD', source=f'#{name}_uv', offset='2', set='0')
        indices = np.repeat(np.arange(len(positions)), 3)
        element(faces, 'p', ' '.join(map(str, indices)))
        node = element(scene, 'node', id=f'{name}_node', name=name)
        instance = element(node, 'instance_geometry', url=f'#{name}_geometry')
        binding = element(element(instance, 'bind_material'), 'technique_common')
        material_binding = element(binding, 'instance_material', symbol=f'{name}_symbol', target=f'#{name}_material')
        element(material_binding, 'bind_vertex_input', semantic='UVMap', input_semantic='TEXCOORD', input_set='0')
    element(element(root, 'scene'), 'instance_visual_scene', url='#Arena')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def apply_materials(model_directory):
    mesh = model_directory/'meshes/rmul_2027_m_zup.stl'
    raw, triangles = read_triangles(mesh)
    labels, normals = classify(triangles)
    textures = model_directory/'materials/textures'
    textures.mkdir(parents=True, exist_ok=True)
    for index, name in enumerate(sorted(set(labels))):
        texture(textures/f'{name}.png', PALETTE[name][0], 20261018+index)
    visual = model_directory/'meshes/rmul_2027_visual.dae'
    write_collada(visual, triangles, labels, normals)
    sdf_path = model_directory/'model.sdf'
    sdf = ET.parse(sdf_path)
    field_link = sdf.getroot().find("model/link[@name='field']")
    field_visual = field_link.find("visual[@name='field_visual']")
    if field_visual is None:
        field_visual = field_link.find("visual[@name='field']")
    if field_visual is None:
        raise ValueError('Expected the supplied field visual')
    field_visual.set('name', 'field_visual')
    field_visual.find('geometry/mesh/uri').text = 'meshes/rmul_2027_visual.dae'
    material = field_visual.find('material')
    if material is not None:
        field_visual.remove(material)
    sdf.write(sdf_path, encoding='utf-8', xml_declaration=True)
    summary = {
        'style': 'matte PVC / painted panels / red-blue zones / existing raised markings',
        'official_artwork': False,
        'source_mesh_sha256': hashlib.sha256(raw).hexdigest(),
        'visual_triangles': len(triangles),
        'groups': {name: int(np.count_nonzero(labels == name)) for name in sorted(set(labels))},
        'collision_mesh': 'meshes/rmul_2027_m_zup.stl',
        'navigation_map_modified': False,
        'contact_friction_modified': False,
        'lidar_material_response_calibrated': False,
        'floor_boundary_width_m': 0.05,
    }
    (model_directory/'materials/appearance.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(apply_materials(args.model_directory), indent=2))
