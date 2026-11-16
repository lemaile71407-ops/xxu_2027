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
import xml.etree.ElementTree as ET
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import yaml

from xxu_bringup.launch_helpers import (
    RobotGeometryParameters, RobotProfile, nav2_parameters,
)
from xxu_bringup.sensor_launch import sensor_preprocessing


SHARE = Path(__file__).resolve().parents[2] / 'src/xxu_bringup'
SIM_SHARE = Path(__file__).resolve().parents[2] / 'src/xxu_sim_bringup'
GEOMETRY = SIM_SHARE / 'models/ega_folded/geometry.json'
NAV_PROFILE = GEOMETRY.parent / 'navigation_profile.yaml'
if not NAV_PROFILE.is_file():
    NAV_PROFILE = GEOMETRY.parent / 'navigation_profile.candidate.yaml'


class VehicleParametersTest(unittest.TestCase):
    def test_spin_profile_preserves_collision_and_stopped_goal_configuration(self):
        from xxu_bringup.navigation_parameters import continuous_spin_parameters
        source = yaml.safe_load((SHARE / 'config/nav2_navigation.yaml').read_text())
        source['controller_server']['ros__parameters']['FollowPath']['motion_model'] = 'Omni'
        before = yaml.safe_dump(source)
        after = continuous_spin_parameters(source)
        self.assertEqual(yaml.safe_dump(source), before)
        for key in ('collision_monitor', 'local_costmap', 'global_costmap'):
            self.assertEqual(after[key], source[key])
        for key in ('goal_checker', 'progress_checker', 'odom_topic'):
            self.assertEqual(after['controller_server']['ros__parameters'].get(key),
                             source['controller_server']['ros__parameters'].get(key))
        self.assertEqual(after['velocity_smoother']['ros__parameters']['max_velocity'][:2],
                         source['velocity_smoother']['ros__parameters']['max_velocity'][:2])

    def test_spin_profile_rejects_nonholonomic_and_out_of_range_requests(self):
        from xxu_bringup.navigation_parameters import continuous_spin_parameters
        source = yaml.safe_load((SHARE / 'config/nav2_navigation.yaml').read_text())
        source['controller_server']['ros__parameters']['FollowPath']['motion_model'] = 'DiffDrive'
        with self.assertRaises(ValueError):
            continuous_spin_parameters(source)
        source['controller_server']['ros__parameters']['FollowPath']['motion_model'] = 'Omni'
        for rate in (0., .6, -.6, float('nan'), float('inf')):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                continuous_spin_parameters(source, rate=rate)

    def test_interval_sampling_changes_only_temporary_imu_model(self):
        from ament_index_python.packages import get_package_share_directory
        spec = importlib.util.spec_from_file_location(
            'rm_sim_interval', SIM_SHARE / 'launch/rm_sim_gazebo.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        source = SIM_SHARE / 'models/ega_folded/model.sdf'
        original = source.read_bytes()
        upstream = get_package_share_directory('rm_sim_26')
        with tempfile.TemporaryDirectory() as folder:
            _, direct, _ = module.prepare_assets(upstream, folder, 'rgl', robot_model='ega_folded')
            before = ET.parse(direct).find(".//sensor[@type='imu']")
            _, averaged, _ = module.prepare_assets(
                upstream, folder, 'rgl', robot_model='ega_folded', imu_sampling='interval_mean')
            after = ET.parse(averaged).find(".//sensor[@type='imu']")
            self.assertEqual(float(before.findtext('update_rate')), 100.)
            self.assertEqual(float(after.findtext('update_rate')), 1000.)
            for old, new in zip(before.findall('.//noise/stddev'), after.findall('.//noise/stddev')):
                self.assertAlmostEqual(float(new.text), float(old.text) * math.sqrt(10))
                new.text = old.text
            after.find('update_rate').text = before.findtext('update_rate')
            self.assertEqual(ET.tostring(before), ET.tostring(after))
        self.assertEqual(source.read_bytes(), original)

    def test_external_mode_excludes_both_amcl_and_gicp(self):
        spec = importlib.util.spec_from_file_location('nav_modes', SHARE / 'launch/navigation.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update(enable_localization='true', localization_mode='external')
        description = module.generate_launch_description()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        packages = []

        def visit(actions):
            for action in actions:
                if action.condition is not None and not action.condition.evaluate(context):
                    continue
                if isinstance(action, Node):
                    packages.append(perform_substitutions(context, normalize_to_list_of_substitutions(
                        action.node_package)))
                elif isinstance(action, GroupAction):
                    visit(action.get_sub_entities())

        visit(description.entities)
        self.assertEqual(packages.count('nav2_map_server'), 1)
        self.assertEqual(packages.count('nav2_controller'), 1)
        self.assertNotIn('nav2_amcl', packages)
        self.assertNotIn('xxu_gicp_localization', packages)

    def test_navigation_servers_can_be_constructed_without_localization(self):
        spec = importlib.util.spec_from_file_location(
            'navigation_launch', SHARE / 'launch/navigation.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with patch.object(module, 'Node', side_effect=lambda **kw: kw):
            servers = module.navigation_server_nodes('/nav.yaml', [], module.interfaces())
            map_server = module.map_server_node('/nav.yaml', '/map.yaml', 'true')
        self.assertEqual(len(servers), 8)
        self.assertNotIn('nav2_amcl', {node['package'] for node in servers})
        self.assertNotIn('xxu_gicp_localization', {node['package'] for node in servers})
        self.assertEqual(map_server['package'], 'nav2_map_server')

    def test_sensor_preprocessing_can_be_constructed_without_an_estimator(self):
        profile = RobotProfile('ega_folded', str(GEOMETRY), json.loads(GEOMETRY.read_text()))
        for platform in ['real', 'rm_sim']:
            with self.subTest(platform=platform), \
                    patch('xxu_bringup.sensor_launch.Node', side_effect=lambda **kw: kw):
                actions = sensor_preprocessing(
                    platform, use_sim_time='true', profile=profile if platform == 'rm_sim' else None)
                self.assertTrue(actions)
                self.assertTrue(all(action['package'] == 'xxu_bringup' for action in actions))

    def test_autonomous_mapping_resolves_selected_vehicle(self):
        launch_file = SHARE.parent / 'xxu_slam_toolbox/launch/autonomous_mapping.launch.py'
        spec = importlib.util.spec_from_file_location('autonomous_mapping', launch_file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        radius = json.loads(GEOMETRY.read_text())['robot_radius']
        for geometry in ['', str(GEOMETRY)]:
            with self.subTest(geometry=geometry):
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
                context.launch_configurations.update(robot_geometry_file=geometry)
                for action in declarations:
                    action.execute(context)
                controller = next(node for node in nodes if node['package'] == 'nav2_controller')
                selected = controller['parameters'][0]
                after = yaml.safe_load(Path(selected.perform(context)).read_text())
                before = self.resolve('exploration')
                if not geometry:
                    self.assertEqual(after, before)
                    continue
                for name in ['local_costmap', 'global_costmap']:
                    self.assertEqual(after[name][name]['ros__parameters']['robot_radius'], radius)
                points = json.loads(after['collision_monitor']['ros__parameters']['EmergencyStop']['points'])
                for a, b in zip(points, points[1:] + points[:1]):
                    edge_distance = abs(a[0] * b[1] - a[1] * b[0]) / math.dist(a, b)
                    self.assertGreaterEqual(edge_distance, radius + .03 - 1e-5)

    def resolve(self, mode, geometry_file='', *, candidate=False):
        with tempfile.TemporaryDirectory() as folder:
            if candidate:
                geometry_copy = Path(folder) / 'geometry.json'
                geometry_copy.write_text(Path(geometry_file).read_text())
                (Path(folder) / 'navigation_profile.yaml').write_text(
                    NAV_PROFILE.read_text())
                geometry_file = geometry_copy
            _, selected, _ = nav2_parameters(
                str(SHARE / f'config/nav2_{mode}.yaml'), 'true',
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
            with self.subTest(mode=mode):
                before = self.resolve(mode)
                after = self.resolve(mode, GEOMETRY, candidate=True)
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
                    before = self.resolve('navigation')
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
            before = self.resolve('navigation')
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

    def test_stopped_goal_yaw_check_requires_explicit_profile(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            geometry = folder / 'geometry.json'
            geometry.write_text(GEOMETRY.read_text())
            source = folder / 'params.yaml'
            params = self.resolve('navigation')
            source.write_text(yaml.safe_dump(params))
            profile = folder / 'navigation_profile.yaml'

            def checker():
                output = RobotGeometryParameters(str(source), str(geometry)).perform(LaunchContext())
                return yaml.safe_load(Path(output).read_text())['controller_server']['ros__parameters']['goal_checker']

            self.assertEqual(checker()['yaw_goal_tolerance'], math.pi)
            profile.write_text('navigation:\n  goal_yaw_tolerance: 0.2\n')
            opted_in = checker()
            self.assertEqual(opted_in['plugin'], 'nav2_controller::StoppedGoalChecker')
            self.assertEqual(opted_in['yaw_goal_tolerance'], .2)
            self.assertEqual(opted_in['rot_stopped_velocity'], .05)

            params['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance'] = .1
            source.write_text(yaml.safe_dump(params))
            self.assertEqual(checker()['yaw_goal_tolerance'], .1)
            profile.write_text('navigation:\n  goal_yaw_tolerance: 3.2\n')
            with self.assertRaisesRegex(ValueError, 'goal_yaw_tolerance'):
                checker()

    def test_missing_profile_keeps_existing_defaults(self):
        with tempfile.TemporaryDirectory() as folder:
            geometry_file = Path(folder) / 'geometry.json'
            geometry_file.write_text(GEOMETRY.read_text())
            for mode in ['navigation', 'exploration']:
                with self.subTest(mode=mode):
                    before = self.resolve(mode)
                    after = self.resolve(mode, geometry_file)
                    old = before['controller_server']['ros__parameters']
                    new = after['controller_server']['ros__parameters']
                    self.assertEqual(new['FollowPath'], old['FollowPath'])
                    self.assertEqual(new['goal_checker']['xy_goal_tolerance'],
                                     old['goal_checker']['xy_goal_tolerance'])
                    self.assertEqual(after['velocity_smoother'], before['velocity_smoother'])
                    for name in ['local_costmap', 'global_costmap']:
                        self.assertEqual(after[name][name]['ros__parameters']['inflation_layer'],
                                         before[name][name]['ros__parameters']['inflation_layer'])

    def test_rmul_entry_selects_only_zlio_and_composes_one_frontend(self):
        spec = importlib.util.spec_from_file_location(
            'rmul_navigation', SIM_SHARE / 'launch/rmul_navigation.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        declaration = next(action for action in description.entities
                           if isinstance(action, DeclareLaunchArgument)
                           and action.name == 'localization_backend')
        context = LaunchContext()
        declaration.execute(context)
        self.assertEqual(context.launch_configurations['localization_backend'], 'zlio_tdt')
        context.launch_configurations['localization_backend'] = 'amcl'
        with self.assertRaises(RuntimeError):
            declaration.execute(context)
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            settings = folder / 'config.yaml'
            settings.write_text('{}')
            context.launch_configurations.update(
                enable_localization='true', robot_model='ega_folded', prior_map=str(folder),
                **{key: str(settings) for key in
                   ('frontend_config', 'backend_config', 'external_map', 'external_nav2_params')},
                **{key: 'false' for key in ('gazebo_gui', 'rviz', 'dynamic_test_vehicle', 'idle_scan')},
                **{key: '' for key in ('field_model', 'rgl_plugin_dir', 'robot_geometry_file')},
                spawn_x='2', spawn_y='2', spawn_yaw='0', lidar_mode='rgl', imu_sampling='direct')
            with patch.object(module, 'get_package_share_directory',
                              side_effect=lambda name: str(SIM_SHARE if name == 'xxu_sim_bringup' else SHARE)), \
                    patch.object(module, 'PythonLaunchDescriptionSource',
                                 wraps=module.PythonLaunchDescriptionSource) as constructed:
                actions = module.external_setup(context)
            from launch.actions import IncludeLaunchDescription
            includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
            sources = [call.args[0] for call in constructed.call_args_list]
            self.assertEqual(sum(path.endswith('/bringup.launch.py') for path in sources), 1, sources)
            self.assertEqual(sum(path.endswith('/rm_sim_gazebo.launch.py') for path in sources), 1)
            self.assertEqual(sum(path.endswith('/zlio_base_adapter.launch.py') for path in sources), 1)
            simulation = next(action for action, path in zip(includes, sources)
                              if path.endswith('/rm_sim_gazebo.launch.py'))
            self.assertNotIn('enable_lio', dict(simulation.launch_arguments))
            self.assertNotIn('enable_zlio_timed', dict(simulation.launch_arguments))

    def test_simulation_constructs_sensors_without_a_second_estimator(self):
        spec = importlib.util.spec_from_file_location(
            'rm_sim_sensors', SIM_SHARE / 'launch/rm_sim_gazebo.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        for action in module.generate_launch_description().entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        context.launch_configurations.update(lidar_mode='gpu_lidar')
        nodes = []

        def capture(**kwargs):
            nodes.append(kwargs)
            return Node(**kwargs)

        robot = '''<robot name="probe"><link name="base_link"/><link name="lio_base_sensor"/>
          <joint name="sensor" type="fixed"><parent link="base_link"/>
          <child link="lio_base_sensor"/><origin xyz="0 0 .0845"/></joint></robot>'''
        with patch.object(module, 'prepare_assets', return_value=('/world', '/model', robot)), \
                patch.object(module, 'Node', side_effect=capture), \
                patch('xxu_bringup.sensor_launch.Node', side_effect=capture):
            module.launch_setup(context)
        self.assertFalse({'frontend', 'backend', 'small_point_lio'} &
                         {node['package'] for node in nodes})
        self.assertEqual(sum(node['name'] == 'gimbal_imu_compensator' for node in nodes), 1)
        self.assertEqual(sum(node['name'] == 'pointcloud_motion_compensator' for node in nodes), 1)
        self.assertNotIn('robot_state_publisher', {node['package'] for node in nodes})
        statics = [node for node in nodes if node.get('executable') == 'static_transform_publisher']
        self.assertEqual(len(statics), 1)
        arguments = dict(zip(statics[0]['arguments'][::2], statics[0]['arguments'][1::2]))
        self.assertEqual(arguments['--frame-id'], 'base_link')
        self.assertEqual(arguments['--child-frame-id'], 'lio_base_sensor')


if __name__ == '__main__':
    unittest.main()
