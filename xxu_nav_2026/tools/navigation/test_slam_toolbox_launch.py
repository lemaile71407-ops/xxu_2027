#!/usr/bin/env python3
"""Check mapping launch parameters after sourcing Jazzy and the workspace."""

import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.utilities import LifecycleEventManager
import yaml


SLAM_SHARE = Path(__file__).resolve().parents[2] / "src/xxu_slam_toolbox"
NAV2_SERVERS = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
    "collision_monitor",
)


def expand_parameters(launch_file, arguments):
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    parameters = {}

    def capture_process(process, context):
        command = [perform_substitutions(context, part) for part in process.cmd]
        files = [
            Path(value) for flag, value in zip(command, command[1:])
            if flag == "--params-file"
        ]
        # launch_ros emits a tuple tag for the lifecycle manager's node list.
        parameters[Path(command[0]).name] = [
            (path, yaml.full_load(path.read_text())) for path in files
        ]

    def visit(entity):
        # Expand the delayed actions immediately so the check needs no running nodes.
        children = entity.actions if isinstance(entity, TimerAction) else entity.visit(context)
        for child in children or ():
            visit(child)

    source = PythonLaunchDescriptionSource(str(launch_file))
    with patch.object(ExecuteProcess, "execute", autospec=True, side_effect=capture_process), \
            patch.object(LifecycleEventManager, "setup_lifecycle_manager"):
        visit(source.get_launch_description(context))
    return parameters


class SlamToolboxLaunchTest(unittest.TestCase):
    def test_launch_clock(self):
        default_params = SLAM_SHARE / "config/slam_toolbox_mapping.yaml"
        for launch_name in ("mapping", "autonomous_mapping"):
            for clock in (None, "false", "true"):
                for custom in (False, True):
                    with (
                        self.subTest(launch=launch_name, clock=clock, custom=custom),
                        tempfile.TemporaryDirectory() as directory,
                        patch.object(tempfile, "tempdir", directory),
                        patch.dict(os.environ, XXU_MAP_DIR=directory),
                    ):
                        self.check_launch(
                            launch_name, clock, default_params, custom, Path(directory))

    def check_launch(self, launch_name, clock, default_params, custom, directory):
        expected = clock != "false"
        arguments = {} if clock is None else {"use_sim_time": clock}
        source = default_params
        source_data = yaml.safe_load(source.read_text())
        if custom:
            source = directory / "custom_slam.yaml"
            source_data["slam_toolbox"]["ros__parameters"].update(
                use_sim_time=not expected, transform_publish_period=0.125)
            source_data["/**"] = {"ros__parameters": {"use_sim_time": not expected}}
            source.write_text(yaml.safe_dump(source_data))
            arguments["slam_params_file"] = str(source)

        nodes = expand_parameters(SLAM_SHARE / "launch" / f"{launch_name}.launch.py", arguments)
        slam_files = nodes["async_slam_toolbox_node"]
        rewritten_path, rewritten = slam_files[0]
        self.assertNotEqual(rewritten_path.resolve(), source.resolve())
        for section in source_data:
            source_data[section]["ros__parameters"]["use_sim_time"] = expected
            self.assertIs(rewritten[section]["ros__parameters"]["use_sim_time"], expected)
        self.assertEqual(rewritten, source_data)
        self.assert_clock(slam_files[1][1], "/slam_toolbox", expected)
        self.assert_clock(nodes["map_autosaver.py"][-1][1], "/map_autosaver", expected)
        self.assert_clock(nodes["rviz2"][-1][1], f"/rviz2_{launch_name}", expected)

        if launch_name == "autonomous_mapping":
            for server in NAV2_SERVERS:
                self.assert_clock(nodes[server][0][1], server, expected)
            for server, costmap in (("controller_server", "local_costmap"),
                                    ("planner_server", "global_costmap")):
                self.assert_clock(nodes[server][0][1][costmap], costmap, expected)
            self.assert_clock(nodes["lifecycle_manager"][-1][1],
                              "/lifecycle_manager_navigation", expected)
            self.assert_clock(nodes["explore"][-1][1], "/explore_node", expected)

    def assert_clock(self, data, node, expected):
        parameters = data[node] if node in data else data["/**"]
        self.assertIs(parameters["ros__parameters"]["use_sim_time"], expected)

    def test_default_yaml_leaves_clock_to_launch(self):
        for filename, section in (("slam_toolbox_mapping.yaml", "slam_toolbox"),
                                  ("explore_lite.yaml", "/**")):
            with self.subTest(file=filename):
                data = yaml.safe_load((SLAM_SHARE / "config" / filename).read_text())
                self.assertNotIn("use_sim_time", data[section]["ros__parameters"])


if __name__ == "__main__":
    unittest.main()
