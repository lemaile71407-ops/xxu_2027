#!/usr/bin/env python3
"""Resolve the RMUL mapping launch entries after sourcing the ROS workspace."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.events import Shutdown
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters, normalize_parameters
import yaml

from test_vehicle_parameters import SHARE, SIM_SHARE, active_nodes
from xxu_bringup.mapping_waypoints import Waypoint, load_waypoints_csv, yaw_to_quaternion


ROUTE = SHARE / 'waypoints/rmul_2027_mapping_loop.csv'


def load_launch(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve_arguments(description, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def run_shutdown(actions, context):
    event = Shutdown()
    for entity in actions:
        if isinstance(entity, RegisterEventHandler) and entity.event_handler.matches(event):
            for cleanup in entity.event_handler.handle(event, context):
                cleanup.execute(context)


class MappingWaypointsTest(unittest.TestCase):
    def test_csv_parsing(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.csv'
            path.write_text('# Comment\n\nname,x,y,yaw\nstart,0,0,0\n  # Another comment\n'
                            '\n"left, north",-0.4,4.8,1.57\n')
            self.assertEqual(load_waypoints_csv(path), [
                Waypoint('start', 0.0, 0.0, 0.0),
                Waypoint('left, north', -0.4, 4.8, 1.57),
            ])

    def test_invalid_csv(self):
        header = 'name,x,y,yaw\n'
        cases = ('', 'x,y,yaw,name\nstart,0,0,0\n', header, header + 'wp,1,2\n',
                 header + 'wp,1,2,3,4\n', header + ',1,2,3\n',
                 header + 'wp,NaN,2,0\n', header + 'wp,1,inf,0\n',
                 header + 'wp,1,2,invalid\n')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'route.csv'
            for contents in cases:
                with self.subTest(contents=contents), self.assertRaises(ValueError):
                    path.write_text(contents)
                    load_waypoints_csv(path)

    def test_quaternion(self):
        self.assertEqual(yaw_to_quaternion(0.0), (0.0, 0.0, 0.0, 1.0))
        for angle in (math.pi / 2, -math.pi / 2, math.pi):
            x, y, z, w = yaw_to_quaternion(angle)
            self.assertEqual((x, y), (0.0, 0.0))
            self.assertAlmostEqual(2.0 * math.atan2(z, w), angle)
            self.assertAlmostEqual(z * z + w * w, 1.0)

    def test_rmul_route_is_closed(self):
        waypoints = load_waypoints_csv(ROUTE)
        self.assertEqual(len(waypoints), 11)
        self.assertTrue(all(math.isfinite(value) for waypoint in waypoints
                            for value in (waypoint.x, waypoint.y, waypoint.yaw)))
        for waypoint in (waypoints[0], waypoints[-1]):
            self.assertEqual((waypoint.x, waypoint.y), (0.0, 0.0))


class MappingLaunchTest(unittest.TestCase):
    def test_simulation_scan_height_parameters(self):
        module = load_launch(SIM_SHARE / 'launch/rm_sim_gazebo.launch.py')
        geometry = json.loads((SIM_SHARE / 'models/ega_folded/geometry.json').read_text())
        cases = (
            ('ega_folded', '', geometry['collision_height'] + 0.02, geometry['base_height']),
            ('ega_folded', '0.28', 0.28, geometry['base_height']),
            ('ega_folded', '0.22', 0.22, geometry['base_height']),
            ('upstream', '', 0.50, 0.0),
            ('upstream', '0.28', 0.28, 0.0),
        )
        for model, maximum, ground_height, base_height in cases:
            with self.subTest(robot_model=model, scan_max_height=maximum), tempfile.TemporaryDirectory() as folder:
                lidar_mode = 'rgl' if model == 'ega_folded' else 'gpu_lidar'
                for library in ('libRobotecGPULidar.so', 'libRGLServerPluginInstance.so',
                                'libRGLServerPluginManager.so'):
                    (Path(folder) / library).touch()
                overrides = {'scan_max_height': maximum} if maximum else {}
                context = resolve_arguments(module.generate_launch_description(), robot_model=model,
                                            lidar_mode=lidar_mode, rgl_plugin_dir=folder, **overrides)
                with patch.object(module, 'Node', wraps=Node) as nodes:
                    actions = module.launch_setup(context)
                run_shutdown(actions, context)
                options = next(call.kwargs for call in nodes.call_args_list
                               if call.kwargs['name'] == 'pointcloud_to_laserscan')
                scan = options['parameters'][-1]
                self.assertAlmostEqual(scan['min_height'], 0.10 - base_height)
                self.assertAlmostEqual(scan['max_height'], ground_height - base_height)

    def test_invalid_scan_height_parameters(self):
        module = load_launch(SIM_SHARE / 'launch/rm_sim_gazebo.launch.py')
        for minimum, maximum, field in (
            ('0.10', '0.09', 'scan_max_height'), ('nan', '', 'scan_min_height'),
            ('invalid', '', 'scan_min_height'), ('0.10', 'inf', 'scan_max_height'),
            ('0.10', 'invalid', 'scan_max_height'),
        ):
            with self.subTest(minimum=minimum, maximum=maximum):
                context = resolve_arguments(module.generate_launch_description(),
                                            scan_min_height=minimum, scan_max_height=maximum)
                with self.assertRaisesRegex(ValueError, field):
                    module.launch_setup(context)

    def test_navigation_tf_only_startup(self):
        module = load_launch(SHARE / 'launch/navigation.launch.py')
        options = {}

        def capture(**kwargs):
            node = Node(**kwargs)
            options[node] = kwargs
            return node

        with patch.object(module, 'Node', side_effect=capture):
            description = module.generate_launch_description()
        context = resolve_arguments(description, use_map_server='false',
                                    enable_localization='false', wait_for_tf_only='true')
        active = {options[node].get('name', options[node]['executable']): options[node]
                  for node in active_nodes(description.entities, context)}
        self.assertNotIn('map_server', active)
        self.assertNotIn('lifecycle_manager_localization', active)
        manager = active['lifecycle_manager_navigation']
        startup = active['nav2_navigation_startup']
        manager_params = evaluate_parameters(context, normalize_parameters(manager['parameters']))[0]
        startup_params = evaluate_parameters(context, normalize_parameters(startup['parameters']))[0]
        self.assertIs(manager_params['autostart'], False)
        self.assertIs(startup_params['require_localization'], False)
        self.assertIs(startup_params['idle_scan_only'], False)

    def test_mapping_expansion(self):
        module = load_launch(SIM_SHARE / 'launch/rmul_mapping.launch.py')
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            (directory / 'base.yaml').write_text(
                'prior_map: {enabled: true}\nuwb: {enabled: true}\n'
                'output: {tf: {publish_predictions: false, max_prediction_age: 0.25}}\n')
            configs = directory / 'configs'
            configs.mkdir()
            (configs / 'middle.yaml').write_text(
                'extends: ../base.yaml\nstartup_hint: {enabled: true}\n')
            backend = configs / 'backend.yaml'
            backend.write_text('extends: middle.yaml\nloop: {enabled: false}\n')
            frontend = directory / 'lio.yaml'
            frontend.write_text('{}')
            output = str(directory / 'maps')
            for auto_waypoints in ('true', 'false'):
                with self.subTest(auto_waypoints=auto_waypoints), \
                        patch.object(module, 'PythonLaunchDescriptionSource',
                                     wraps=module.PythonLaunchDescriptionSource) as sources, \
                        patch.object(module, 'Node', wraps=Node) as nodes:
                    description = module.generate_launch_description()
                    context = resolve_arguments(
                        description, frontend_config=str(frontend), backend_config=str(backend),
                        nav2_params=str(SHARE / 'config/nav2_mapping.yaml'),
                        save_directory=output, auto_waypoints=auto_waypoints,
                        auto_save_map='false', scan_max_height='0.28',
                        waypoints_file=str(ROUTE) if auto_waypoints == 'true' else '')
                    expansion = next(entity for entity in description.entities
                                     if isinstance(entity, OpaqueFunction))
                    actions = expansion.execute(context)
                    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
                    args = {Path(call.args[0]).name: dict(action.launch_arguments)
                            for call, action in zip(sources.call_args_list, includes, strict=True)}
                    self.assertCountEqual(args, ('rm_sim_gazebo.launch.py', 'bringup.launch.py',
                                                 'zlio_base_adapter.launch.py', 'navigation.launch.py'))
                    self.assertEqual(args['bringup.launch.py']['mode'], 'mapping')
                    temporary = Path(args['bringup.launch.py']['backend_config'])
                    merged = yaml.safe_load(temporary.read_text())
                    self.assertNotIn('extends', merged)
                    for key in ('prior_map', 'uwb', 'startup_hint'):
                        self.assertIs(merged[key]['enabled'], False)
                    self.assertIs(merged['loop']['enabled'], True)
                    self.assertEqual(merged['trajectory']['save_directory'], output)
                    self.assertIs(merged['trajectory']['save_global_pcd'], True)
                    self.assertAlmostEqual(merged['trajectory']['map_voxel_resolution'], 0.05)
                    for key in ('enabled', 'publish_runtime_map', 'publish_paths'):
                        self.assertIs(merged['visualization'][key], True)
                    self.assertEqual(merged['output']['tf'],
                                     {'publish_predictions': True, 'max_prediction_age': 0.25})
                    self.assertEqual(args['rm_sim_gazebo.launch.py']['require_localization'], 'false')
                    self.assertEqual(args['rm_sim_gazebo.launch.py']['scan_max_height'], '0.28')
                    self.assertEqual(args['navigation.launch.py']['use_map_server'], 'false')
                    self.assertEqual(args['navigation.launch.py']['wait_for_tf_only'], 'true')
                    self.assertEqual(nodes.call_count, int(auto_waypoints == 'true'))
                    if auto_waypoints == 'true':
                        runner = nodes.call_args.kwargs
                        self.assertEqual(runner['executable'], 'waypoint_mapping_runner.py')
                        params = evaluate_parameters(context, normalize_parameters(runner['parameters']))[0]
                        self.assertEqual(params['waypoints_file'], str(ROUTE))
                        self.assertIs(params['auto_save_map'], False)

                    run_shutdown(actions, context)
                    self.assertFalse(temporary.exists())

            (directory / 'base.yaml').write_text('extends: configs/backend.yaml\n')
            with self.assertRaisesRegex(ValueError, 'inheritance cycle'):
                module.setup(context)

            (directory / 'base.yaml').write_text('prior_map: {enabled: false}\n')
            override_context = resolve_arguments(
                module.generate_launch_description(),
                frontend_config=str(frontend), backend_config=str(backend),
                nav2_params=str(SHARE / 'config/nav2_mapping.yaml'),
                waypoints_file=str(ROUTE), map_voxel_resolution='0.10')
            with patch.object(module, 'PythonLaunchDescriptionSource',
                              wraps=module.PythonLaunchDescriptionSource):
                override_actions = module.setup(override_context)
            override_include = next(a for a in override_actions
                                    if isinstance(a, IncludeLaunchDescription)
                                    and dict(a.launch_arguments).get('mode') == 'mapping')
            override_cfg = yaml.safe_load(Path(dict(override_include.launch_arguments)['backend_config']).read_text())
            self.assertAlmostEqual(override_cfg['trajectory']['map_voxel_resolution'], 0.10)
            run_shutdown(override_actions, override_context)

            for bad in ('0.0', '-0.1', 'nan', 'invalid'):
                bad_ctx = resolve_arguments(
                    module.generate_launch_description(),
                    frontend_config=str(frontend), backend_config=str(backend),
                    nav2_params=str(SHARE / 'config/nav2_mapping.yaml'),
                    waypoints_file=str(ROUTE), map_voxel_resolution=bad)
                with self.subTest(map_voxel_resolution=bad), self.assertRaisesRegex(ValueError, 'map_voxel_resolution'):
                    module.setup(bad_ctx)


if __name__ == '__main__':
    unittest.main()
