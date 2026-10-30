#!/usr/bin/env python3
"""Offline CAD visibility estimate; does not change ROS or Gazebo configuration.

Requires numpy, scipy, trimesh, matplotlib and g++. Ray helper is built in a
temporary directory. Coordinates: x forward, y left, z above the floor.
"""
import argparse
import ctypes
from datetime import datetime
import hashlib
import json
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation


def pose(text):
    a = np.array([float(x) for x in (text or '0 0 0 0 0 0').split()])
    m = np.eye(4)
    m[:3, :3] = Rotation.from_euler('xyz', a[3:]).as_matrix()
    m[:3, 3] = a[:3]
    return m


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--reuse-grid', action='store_true', help='Reuse this output directory grid for plotting or new live evidence')
    parser.add_argument('--captured-cloud', type=Path, help='Optional NPZ of raw sensor-frame points for passive simulation comparison')
    args = parser.parse_args()
    folder, out = args.model, args.output
    out.mkdir(parents=True, exist_ok=True)
    geometry = json.loads((folder / 'geometry.json').read_text())
    properties = json.loads((folder / 'physical_properties.json').read_text())
    model = ET.parse(folder / 'model.sdf').getroot().find('model')
    origin = np.array(geometry['primary_sensor']['offset'])
    origin[2] += geometry['base_height']
    rotation = Rotation.from_euler('xyz', geometry['primary_sensor']['rpy']).as_matrix()
    pieces, inputs = [], [folder / name for name in ['geometry.json', 'model.sdf', 'physical_properties.json']]
    for link in model.findall('link'):
        # RGL excludes its own housing link. Its optical surfaces must not be
        # treated as opaque triangles enclosing the virtual emission origin.
        if link.get('name') == 'livox_lidar_housing':
            continue
        local = pose(link.findtext('pose'))
        if link.find('pose').get('relative_to') == 'base_link':
            local[2, 3] += geometry['base_height']
        for visual in link.findall('visual'):
            uri = visual.findtext('geometry/mesh/uri')
            if not uri:
                continue
            path = folder / uri
            inputs.append(path)
            mesh = trimesh.load(path, force='scene').to_geometry()
            mesh.apply_transform(local @ pose(visual.findtext('pose')))
            pieces.append(mesh)
    mesh = trimesh.util.concatenate(pieces)
    footprint = mesh.vertices[ConvexHull(mesh.vertices[:, :2]).vertices, :2]
    print(f'Loaded {len(mesh.faces):,} full-detail triangles', flush=True)
    helper = Path(__file__).with_name('mesh_raytrace.cpp')
    with tempfile.TemporaryDirectory(prefix='xxu-blindspots-') as temp:
        library = Path(temp) / 'ray.so'
        subprocess.run(['g++', '-O3', '-std=c++17', '-shared', '-fPIC', '-fopenmp', str(helper), '-o', str(library)], check=True)
        lib = ctypes.CDLL(str(library))
        fp = np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')
        dp = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        lib.initialize.argtypes = [fp, ctypes.c_int]
        lib.trace.argtypes = [dp, dp, ctypes.c_int, dp]

        def trace(directions, start=origin):
            directions = np.ascontiguousarray(directions, dtype=np.float64)
            result = np.empty(len(directions))
            lib.trace(np.ascontiguousarray(start, dtype=np.float64), directions, len(result), result)
            return result

        # Independent known-distance cube checks: outside, inside, misses,
        # negative directions and parallel-to-box-axis directions.
        cube = trimesh.creation.box(extents=[2, 2, 2])
        lib.initialize(np.ascontiguousarray(cube.triangles, dtype=np.float32), len(cube.faces))
        test = trace([[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0]], np.zeros(3))
        assert np.allclose(test, 1), test
        test = trace([[-1, 0, 0], [1, 0, 0], [0, 1, 0]], np.array([2., 0, 0]))
        assert np.isclose(test[0], 1) and np.isinf(test[1:]).all(), test
        triangles = np.ascontiguousarray(mesh.triangles, dtype=np.float32)
        lib.initialize(triangles, len(triangles))
        # Cross-check against trimesh's independent rtree implementation.
        rng = np.random.default_rng(20261018)
        check_dirs = rng.normal(size=(12, 3))
        check_dirs /= np.linalg.norm(check_dirs, axis=1)[:, None]
        ours = trace(check_dirs)
        locations, ray_ids, _ = mesh.ray.intersects_location(np.tile(origin, (12, 1)), check_dirs, multiple_hits=False)
        reference = np.full(12, np.inf)
        reference[ray_ids] = np.linalg.norm(locations - origin, axis=1)
        assert np.array_equal(np.isfinite(ours), np.isfinite(reference))
        assert np.allclose(ours[np.isfinite(ours)], reference[np.isfinite(reference)], atol=2e-5)
        print('Known-distance and independent ray checks passed', flush=True)

        angles = np.deg2rad(np.arange(360))
        radii = np.round(np.arange(.54, 6.001, .02), 4)
        obstacle_heights = [.05, .10, .15, .20, geometry['collision_height'], .50]
        z = np.unique(np.r_[np.arange(0, .5001, .005), obstacle_heights, geometry['collision_height'] + .02])
        minimum = np.full((len(radii), 360), np.inf)
        minimum_fov = minimum.copy()
        minimum_scan = minimum.copy()
        minimum_scan_fov = minimum.copy()
        scan_max = geometry['collision_height'] + .02
        if args.reuse_grid:
            previous = json.loads((out / 'results.json').read_text())
            for path in inputs:
                key = str(path.relative_to(folder.parents[3]))
                assert hashlib.sha256(path.read_bytes()).hexdigest() == previous['sources_sha256'][key], f'Stale grid: {key}'
            saved = np.load(out / 'visibility_grid.npz')
            assert np.array_equal(saved['radii_m'], radii)
            minimum = saved['minimum_physical_height_m']
            minimum_scan = saved['minimum_navigation_height_m']
            minimum_fov = saved['minimum_fov_only_height_m']
            minimum_scan_fov = saved['minimum_navigation_fov_only_height_m']
        for i, radius in ([] if args.reuse_grid else enumerate(radii)):
            xyz = np.empty((len(z), 360, 3))
            xyz[:, :, 0] = radius * np.cos(angles)
            xyz[:, :, 1] = radius * np.sin(angles)
            xyz[:, :, 2] = z[:, None]
            delta = xyz - origin
            distance = np.linalg.norm(delta, axis=2)
            directions = delta / distance[:, :, None]
            sensor_dirs = directions @ rotation
            elevation = np.rad2deg(np.arcsin(np.clip(sensor_dirs[:, :, 2], -1, 1)))
            fov = (elevation >= -7) & (elevation <= 52) & (distance >= .1)
            blocked = trace(directions.reshape(-1, 3)).reshape(len(z), 360) < distance - 1e-5
            heights = np.broadcast_to(z[:, None], fov.shape)
            band = (z[:, None] >= .1) & (z[:, None] <= scan_max)
            for dest, mask in [(minimum, fov & ~blocked), (minimum_fov, fov),
                               (minimum_scan, fov & ~blocked & band), (minimum_scan_fov, fov & band)]:
                dest[i] = np.min(np.where(mask, heights, np.inf), axis=0)
            if i % 50 == 0:
                print(f'Computed columns through {radius:.2f} m', flush=True)
        # Actual preset directions are its +Z columns: the loader's uniform
        # convention maps pitch=90deg to horizontal rays without axis remapping.
        pattern = folder.parents[2] / 'rm_sim_26/external/RGLGazeboPlugin/lidar_patterns/LivoxMid360.mat3x4f'
        a = np.fromfile(pattern, dtype='<f4').reshape(-1, 3, 4)
        dirs = a[:, :, 2].astype(float)
        elev = np.rad2deg(np.arcsin(np.clip(dirs[:, 2], -1, 1)))
        all_dirs = dirs @ rotation.T
        preset_hits = trace(all_dirs)
        preset = dict(ray_count=len(a), alternating_frames=40, cycle_sim_seconds=4.0,
                      sensor_elevation_degrees=[float(elev.min()), float(elev.max())],
                      outside_official_vertical_fov_percent=float(100*np.mean((elev < -7) | (elev > 52))),
                      body_intercepted_percent=float(100*np.mean(np.isfinite(preset_hits))),
                      caveat='Offline mesh intersections; does not verify Gazebo fixed-joint entity-ignore behavior or sensor-link optics.')
        inputs.append(pattern)
        live = None
        if args.captured_cloud:
            capture = np.load(args.captured_cloud)
            sensor_points = capture['points'].astype(float)
            live_distance = np.linalg.norm(sensor_points, axis=1)
            directions = (sensor_points / live_distance[:, None]) @ rotation.T
            intersections = trace(directions)
            live = dict(topic='/rm_sim/lidar/points', frame='livox_lidar', point_count=len(directions),
                sim_stamp_start=float(capture['stamps'][0]), sim_stamp_end=float(capture['stamps'][-1]),
                frames=len(capture['stamps']), mesh_tolerance_m=.005,
                returns_beyond_first_vehicle_surface_count=int(np.sum(intersections < live_distance-.005)),
                returns_beyond_first_vehicle_surface_percent=float(100*np.mean(intersections < live_distance-.005)),
                returns_matching_vehicle_surface_count=int(np.sum(np.abs(intersections-live_distance) <= .005)),
                snapshot_file='raw_points_snapshot.npz', snapshot_sha256=hashlib.sha256(args.captured_cloud.read_bytes()).hexdigest(),
                explanation='Passive snapshot. Returns beyond an opaque CAD surface reveal a simulation/model visibility mismatch; do not interpret environment-dependent empty scan bins as blind angles.')
            if args.captured_cloud.resolve() != (out / 'raw_points_snapshot.npz').resolve():
                shutil.copyfile(args.captured_cloud, out / 'raw_points_snapshot.npz')

    summaries = []
    for height in obstacle_heights:
        row = dict(obstacle_height_m=height, rings=[])
        for r in [.54, .76, 1., 2., 3., 5., 6.]:
            i = int(np.argmin(abs(radii-r)))
            row['rings'].append(dict(radius_from_base_center_m=float(radii[i]),
                physical_blind_percent=float(100*np.mean(minimum[i] > height+1e-9)),
                navigation_blind_percent=float(100*np.mean(minimum_scan[i] > height+1e-9)),
                fov_only_navigation_blind_percent=float(100*np.mean(minimum_scan_fov[i] > height+1e-9))))
        # Weighted column area in the sampled annulus (not percentage of rays).
        mask = radii <= 3
        row['blind_area_fraction_053_to_301m'] = float(np.average(np.mean(minimum_scan[mask] > height+1e-9, axis=1), weights=radii[mask]))
        summaries.append(row)
    direction_table = []
    for name, deg in [('front', 0), ('left', 90), ('rear', 180), ('right', 270)]:
        vals = {}
        for height in [.15, .20, geometry['collision_height']]:
            good = minimum_scan[:, deg] <= height+1e-9
            vals[str(height)] = dict(first_visible_center_distance_m=float(radii[np.flatnonzero(good)[0]]) if good.any() else None,
                                    blind_rings_at_or_beyond_1m=int(np.sum(~good & (radii >= 1))))
        direction_table.append(dict(direction=name, angle_degrees=deg, heights=vals))
    np.savez_compressed(out / 'visibility_grid.npz', angles_degrees=np.arange(360), radii_m=radii,
                        minimum_physical_height_m=minimum, minimum_navigation_height_m=minimum_scan,
                        minimum_fov_only_height_m=minimum_fov, minimum_navigation_fov_only_height_m=minimum_scan_fov)
    sha = {str(p.relative_to(folder.parents[3])): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs}
    result = dict(date=datetime.now(ZoneInfo('Asia/Shanghai')).date().isoformat(), kind='offline_geometric_estimate_not_real_vehicle_test',
        configuration=dict(active_lidar_count=properties['active_lidar_count'], cover_mode=properties['lidar_cover_mode'],
            optical_origin_above_floor_m=origin.tolist(), sensor_rpy_degrees=np.rad2deg(geometry['primary_sensor']['rpy']).tolist(),
            official_fov_degrees=[-7,52], lidar_min_range_m=.1,
            scan_height_band_m=[.1, scan_max], navigation_center_exclusion_radius_m=geometry['robot_radius']),
        method=dict(triangles=len(mesh.faces), azimuth_step_degrees=1, radial_step_m=.02, vertical_step_m=.005,
            radial_centers_m=[float(radii[0]),float(radii[-1])], threshold_accuracy='Discrete samples; narrow angular gaps can be missed; no beam/reflectivity probability model.',
            obstacle='Opaque vertical face covering heights from ground to H, evaluated at a radial column; any unblocked eligible height counts as geometrically detectable.',
            independent_checks='Known-distance cube and 12 seeded rays compared with trimesh rtree, passed.',
            omissions='Sensor housing link excluded as in RGL; no ground slope, environment occlusion, rolling motion, sunlight, reflectivity, finite per-frame sampling or scan/detection persistence modeled.',
            incomplete_mesh_counts={k:properties['mesh_coverage'][k] for k in ['references','placed','no_cached_mesh','suppressed']}),
        near_field=dict(center_excluded_disk_area_m2=float(np.pi*geometry['robot_radius']**2),
            convex_projected_body_envelope_area_m2=float(ConvexHull(footprint).volume),
            excluded_area_outside_convex_body_envelope_m2=float(np.pi*geometry['robot_radius']**2-ConvexHull(footprint).volume),
            straight_axis_gap_from_body_bounds_m=dict(front=float(geometry['robot_radius']-mesh.bounds[1,0]),
                rear=float(geometry['robot_radius']+mesh.bounds[0,0]),left=float(geometry['robot_radius']-mesh.bounds[1,1]),
                right=float(geometry['robot_radius']+mesh.bounds[0,1]))),
        obstacle_summaries=summaries, cardinal_directions=direction_table, simulation_pattern=preset, passive_live_observation=live,
        sources_sha256=sha,
        official_source='https://www.livoxtech.com/jp/mid-360/specs')
    (out / 'results.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    # Scientific plot: floor plan, forward up and left on the left.
    from matplotlib.colors import ListedColormap
    fig, axes = plt.subplots(1, 2, figsize=(13, 6), constrained_layout=True)
    for ax, height, title in zip(axes, [.15, geometry['collision_height']], ['15 cm obstacle', '26.6 cm obstacle']):
        # 0 visible; 1 body shadow; 2 outside FOV; 3 center exclusion.
        rr, aa = np.meshgrid(np.r_[radii-.01, radii[-1]+.01], np.deg2rad(np.arange(361)-.5), indexing='ij')
        classes = np.where(minimum_scan <= height+1e-9, 0, np.where(minimum_scan_fov <= height+1e-9, 1, 2))
        ax.pcolormesh(-rr*np.sin(aa), rr*np.cos(aa), classes, cmap=ListedColormap(['#6cc5a8','#d8786e','#e6c66a']),
                      vmin=0, vmax=2, shading='flat', rasterized=True)
        ax.add_patch(plt.Circle((0,0),geometry['robot_radius'],color='#9da7b5',zorder=3))
        ax.fill(-footprint[:,1],footprint[:,0],color='#29394b',zorder=4)
        ax.scatter(-origin[1], origin[0], color='#ffec99',edgecolors='#26364a',s=65,zorder=5)
        ax.set(xlim=(-3,3),ylim=(-3,3),aspect='equal',title=title,xlabel='Left  <----  metres  ---->  Right',ylabel='Rear  <----  metres  ---->  Front')
        ax.grid(alpha=.18)
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color=c,label=l) for c,l in [('#6cc5a8','Geometrically visible'),('#d8786e','Vehicle shadow'),('#e6c66a','Outside eligible FOV'),('#9da7b5','Scan exclusion'),('#29394b','Vehicle envelope')]],loc='lower center',ncol=5,bbox_to_anchor=(.5,-.06))
    fig.suptitle('Single MID360 / cover removed / current navigation height band\nCAD estimate; official FOV; 1 degree / 2 cm / 5 mm grid',fontsize=14)
    fig.savefig(out/'blindspots.png',dpi=170,bbox_inches='tight')
    plt.close(fig)
    print(json.dumps(dict(near_field=result['near_field'],summaries=summaries,cardinal_directions=direction_table,preset=preset),indent=2),flush=True)


if __name__ == '__main__':
    main()
