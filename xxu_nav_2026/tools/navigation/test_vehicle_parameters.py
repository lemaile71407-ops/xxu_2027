#!/usr/bin/env python3
"""Check resolved CAD navigation parameters after sourcing the ROS workspace.

These checks resolve both shared profiles and vehicle overrides, without
starting nodes. Runtime navigation still needs the separate RMUL regression.
"""
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import yaml

from xxu_bringup.launch_helpers import (
    RobotGeometryParameters, RobotProfile, lio_preprocessing, nav2_parameters,
)


SHARE = Path(__file__).resolve().parents[2] / 'src/xxu_bringup'
GEOMETRY = SHARE / 'models/ega_folded/geometry.json'
NAV_PROFILE = GEOMETRY.parent / 'navigation_profile.yaml'
if not NAV_PROFILE.is_file():
    NAV_PROFILE = GEOMETRY.parent / 'navigation_profile.candidate.yaml'
LIO_PROFILE = GEOMETRY.parent / 'lio_profile.yaml'
if not LIO_PROFILE.is_file():
    LIO_PROFILE = GEOMETRY.parent / 'lio_profile.candidate.yaml'


class VehicleParametersTest(unittest.TestCase):
    def test_autonomous_mapping_resolves_selected_vehicle(self):
        launch_file = SHARE.parent / 'xxu_slam_toolbox/launch/autonomous_mapping.launch.py'
        spec = importlib.util.spec_from_file_location('autonomous_mapping', launch_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        radius = json.loads(GEOMETRY.read_text())['robot_radius']
        for fake in ['true', 'false']:
            for geometry in ['', str(GEOMETRY)]:
                with self.subTest(fake=fake, geometry=geometry):
                    nodes = []

                    def capture_node(**kwargs):
                        nodes.append(kwargs)
                        return Node(**kwargs)

                    with patch.object(module, 'Node', side_effect=capture_node), \
                            patch.object(module, 'get_package_share_directory', return_value=str(SHARE)), \
                            patch.object(module, 'FindPackageShare', side_effect=lambda name: TextSubstitution(text=str(SHARE.parent / name))), \
                            patch.object(module, 'default_map_save_path', return_value='/tmp/xxu_test_map'):
                        description = module.generate_launch_description()
                    declarations = [action for action in description.entities
                                    if isinstance(action, DeclareLaunchArgument)]
                    self.assertIn('robot_geometry_file', [action.name for action in declarations])
                    context = LaunchContext()
                    context.launch_configurations.update(
                        use_fake_frame=fake, robot_geometry_file=geometry)
                    for action in declarations:
                        action.execute(context)
                    controller = next(node for node in nodes if node['package'] == 'nav2_controller')
                    selected = controller['parameters'][0]
                    after = yaml.safe_load(Path(selected.perform(context)).read_text())
                    before = self.resolve('exploration', fake)
                    if not geometry:
                        self.assertEqual(after, before)
                        continue
                    for name in ['local_costmap', 'global_costmap']:
                        self.assertEqual(after[name][name]['ros__parameters']['robot_radius'], radius)
                    points = json.loads(after['collision_monitor']['ros__parameters']['EmergencyStop']['points'])
                    for a, b in zip(points, points[1:] + points[:1]):
                        edge_distance = abs(a[0] * b[1] - a[1] * b[0]) / math.dist(a, b)
                        self.assertGreaterEqual(edge_distance, radius + .03 - 1e-5)

    def resolve(self, mode, fake, geometry_file='', *, candidate=False):
        with tempfile.TemporaryDirectory() as folder:
            if candidate:
                geometry_copy = Path(folder) / 'geometry.json'
                geometry_copy.write_text(Path(geometry_file).read_text())
                (Path(folder) / 'navigation_profile.yaml').write_text(
                    NAV_PROFILE.read_text())
                geometry_file = geometry_copy
            _, selected, _ = nav2_parameters(
                str(SHARE / f'config/nav2_{mode}.yaml'), 'true', fake,
                str(SHARE), mode=mode, robot_geometry_file=str(geometry_file))
            return yaml.safe_load(Path(selected.perform(LaunchContext())).read_text())

    def assert_clearance(self, before, after, radius):
        points = json.loads(after['collision_monitor']['ros__parameters']['EmergencyStop']['points'])
        outer_stop = max(math.hypot(*p) for p in points)
        # All octagon edges must still enclose the vehicle plus 3 cm.
        for a, b in zip(points, points[1:] + points[:1]):
            edge_distance = abs(a[0] * b[1] - a[1] * b[0]) / math.dist(a, b)
            self.assertGreaterEqual(edge_distance, radius + .03 - 1e-5)
        for name in ['local_costmap', 'global_costmap']:
            old = before[name][name]['ros__parameters']
            new = after[name][name]['ros__parameters']
            inflation = new['inflation_layer']['inflation_radius']
            self.assertEqual(new['robot_radius'], radius)
            self.assertGreaterEqual(inflation, outer_stop + new['resolution'] - 1e-5)
            self.assertGreaterEqual(inflation, old['inflation_layer']['inflation_radius'])
            old_clearance = old['inflation_layer']['inflation_radius'] - old['robot_radius']
            self.assertGreaterEqual(inflation - radius, old_clearance - 1e-9)

    def test_selected_vehicle_keeps_clearance_in_all_profiles(self):
        radius = json.loads(GEOMETRY.read_text())['robot_radius']
        for mode in ['navigation', 'exploration']:
            for fake in ['true', 'false']:
                with self.subTest(mode=mode, fake=fake):
                    before = self.resolve(mode, fake)
                    after = self.resolve(mode, fake, GEOMETRY, candidate=True)
                    # The profile may tighten arrival tolerance, never relax it.
                    old_controller = before['controller_server']['ros__parameters']
                    new_controller = after['controller_server']['ros__parameters']
                    self.assertLessEqual(new_controller['goal_checker']['xy_goal_tolerance'],
                                         old_controller['goal_checker']['xy_goal_tolerance'])
                    if mode == 'navigation':
                        self.assert_clearance(before, after, radius)
                        self.assertEqual(new_controller['goal_checker']['xy_goal_tolerance'], .06)
                        controller = new_controller['FollowPath']
                        smoother = after['velocity_smoother']['ros__parameters']
                        self.assertEqual([controller['vx_min'], controller['vx_max'], controller['vy_max']],
                                         [-.5, .5, .5])
                        self.assertEqual(smoother['max_velocity'][:2], [.5, .5])
                        self.assertEqual(smoother['min_velocity'][:2], [-.5, -.5])
                        self.assertEqual(after['local_costmap']['local_costmap']['ros__parameters']
                                         ['inflation_layer']['inflation_radius'], .85)
                    else:
                        self.assertEqual(new_controller['FollowPath'], old_controller['FollowPath'])
                        self.assertEqual(after['velocity_smoother'], before['velocity_smoother'])
                        for name in ['local_costmap', 'global_costmap']:
                            self.assertEqual(after[name][name]['ros__parameters']['inflation_layer'],
                                             before[name][name]['ros__parameters']['inflation_layer'])

    def test_no_vehicle_leaves_source_untouched(self):
        source = str(SHARE / 'config/nav2_navigation.yaml')
        self.assertEqual(RobotGeometryParameters(source, '').perform(LaunchContext()), source)

    def test_large_vehicle_and_explicit_extra_clearance(self):
        for radius, explicit_inflation in [(1.5, .55), (.533, 1.2), (.2, .55)]:
            with self.subTest(radius=radius, explicit_inflation=explicit_inflation):
                with tempfile.TemporaryDirectory() as folder:
                    folder = Path(folder)
                    before = self.resolve('navigation', 'true')
                    for name in ['local_costmap', 'global_costmap']:
                        before[name][name]['ros__parameters']['inflation_layer']['inflation_radius'] = explicit_inflation
                    source = folder / 'params.yaml'
                    source.write_text(yaml.safe_dump(before))
                    geometry = folder / 'geometry.json'
                    geometry.write_text(json.dumps({'robot_radius': radius}))
                    (folder / 'navigation_profile.yaml').write_text(
                        'navigation:\n  minimum_inflation_radius: 0.55\n')
                    output = RobotGeometryParameters(str(source), str(geometry)).perform(LaunchContext())
                    self.assert_clearance(before, yaml.safe_load(Path(output).read_text()), radius)

    def test_vehicle_profile_never_raises_stricter_source_limits(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            before = self.resolve('navigation', 'true')
            controller = before['controller_server']['ros__parameters']
            controller['FollowPath'].update(vx_min=-.2, vx_max=.2, vy_max=.2)
            controller['goal_checker']['xy_goal_tolerance'] = .04
            smoother = before['velocity_smoother']['ros__parameters']
            smoother['max_velocity'][:2] = [.2, .2]
            smoother['min_velocity'][:2] = [-.2, -.2]
            source = folder / 'params.yaml'
            source.write_text(yaml.safe_dump(before))
            geometry = folder / 'geometry.json'
            geometry.write_text(json.dumps({'robot_radius': .533}))
            (folder / 'navigation_profile.yaml').write_text(
                NAV_PROFILE.read_text())
            output = RobotGeometryParameters(str(source), str(geometry)).perform(LaunchContext())
            after = yaml.safe_load(Path(output).read_text())
            self.assertEqual(after['controller_server'], before['controller_server'])
            self.assertEqual(after['velocity_smoother'], before['velocity_smoother'])

    def test_missing_profile_keeps_existing_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            geometry_file = Path(folder) / 'geometry.json'
            geometry_file.write_text(GEOMETRY.read_text())
            for mode in ['navigation', 'exploration']:
                with self.subTest(mode=mode):
                    before = self.resolve(mode, 'true')
                    after = self.resolve(mode, 'true', geometry_file)
                    old = before['controller_server']['ros__parameters']
                    new = after['controller_server']['ros__parameters']
                    self.assertEqual(new['FollowPath'], old['FollowPath'])
                    self.assertEqual(new['goal_checker']['xy_goal_tolerance'],
                                     old['goal_checker']['xy_goal_tolerance'])
                    self.assertEqual(after['velocity_smoother'], before['velocity_smoother'])
                    for name in ['local_costmap', 'global_costmap']:
                        self.assertEqual(after[name][name]['ros__parameters']['inflation_layer'],
                                         before[name][name]['ros__parameters']['inflation_layer'])

    def test_simulator_lio_overlay_preserves_explicit_override_precedence(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            geometry_file = folder / 'geometry.json'
            geometry = json.loads(GEOMETRY.read_text())
            geometry_file.write_text(json.dumps(geometry))
            model_lio = folder / 'lio_profile.yaml'
            model_lio.write_text(LIO_PROFILE.read_text())
            profile = RobotProfile('candidate', str(geometry_file), geometry)
            base_config = str(SHARE / 'config/rm_sim_lio.yaml')
            with patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
                actions = lio_preprocessing(
                    'rm_sim', use_sim_time='true', config_file=base_config,
                    profile=profile, lio_overrides={'laser_point_cov': .002})
            parameters = actions[-1]['parameters']
            self.assertEqual(parameters[:2], [base_config, str(model_lio)])
            self.assertEqual(parameters[-1]['laser_point_cov'], .002)
            self.assertEqual(parameters[-1]['planar_z'], geometry['base_height'])


if __name__ == '__main__':
    unittest.main()
