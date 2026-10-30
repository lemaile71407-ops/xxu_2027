#!/usr/bin/env python3
"""Check static ROS configuration consumers against the integration contract.

Offline: python3 tools/navigation/check_interfaces.py (PyYAML, no ROS needed).
Changing a launch boundary override does not rewrite static YAML consumers.
"""

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


def leaves(value, path=()):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from leaves(child, (*path, str(key)))
    elif isinstance(value, list):
        for i, child in enumerate(value):
            yield from leaves(child, (*path, str(i)))
    else:
        yield path, value


def topic_names(value, path=()):
    for key, child in value.items():
        if "name" in child:
            yield ".".join((*path, key)), child["name"]
        else:
            yield from topic_names(child, (*path, key))


def expected_reference(filename, path):
    """Describe each consumer's meaning, independent of its configured value."""
    key = path[-1].rsplit(".", 1)[-1]
    owner = path[0]
    rules = {
        "scan_topic": "topic:localization.scan", "topic": "topic:localization.scan",
        "deskewed_cloud_topic": "topic:localization.deskewed_cloud",
        "cmd_vel_in_topic": "topic:navigation.smoothed_output",
        "cmd_vel_out_topic": "topic:navigation.collision_output",
        "state_topic": "topic:nav2.collision_state",
        "polygon_pub_topic": "topic:nav2.emergency_stop",
        "local_costmap_topic": "topic:nav2.local_costmap_raw",
        "global_costmap_topic": "topic:nav2.global_costmap_raw",
        "local_footprint_topic": "topic:nav2.local_footprint",
        "footprint_topic": "topic:nav2.local_footprint",
        "global_footprint_topic": "topic:nav2.global_footprint",
        "costmap_topic": "topic:system.map", "costmap_updates_topic": "topic:system.map_updates",
        "odom_frame": "frame:odom", "odom_frame_id": "frame:odom", "local_frame": "frame:odom",
        "map_frame": "frame:map", "global_frame_id": "frame:map",
        "base_frame": "frame:ground_base", "robot_base_frame": "frame:navigation_base",
        "base_frame_id": "frame:ground_base" if owner == "amcl" else "frame:navigation_base",
        "global_frame": "frame:odom" if owner == "local_costmap" else "frame:map",
        "odom_topic": "topic:navigation.odometry" if filename == "nav2_fake_profile.yaml" else "topic:localization.odometry",
    }
    if key in ("lidar_topic", "imu_topic", "lidar_frame"):
        real = filename == "xxu_mid360_gimbal_compensated.yaml"
        raw = filename == "xxu_gazebo_mid360.yaml"
        return {
            "lidar_topic": "topic:radar.pointcloud_compensated" if real else
                           "topic:simulation.lidar_shaped" if raw else "topic:simulation.lidar_compensated",
            "imu_topic": "topic:radar.imu_compensated" if real else
                         "topic:simulation.imu" if raw else "topic:simulation.imu_compensated",
            "lidar_frame": "frame:gazebo_lidar" if raw else "frame:lidar_compensated",
        }[key]
    return rules.get(key)


def check(workspace, contract_path=None):
    src = workspace / "src"
    contract_path = contract_path or src / "xxu_bringup/config/robot_interfaces.yaml"
    contract = yaml.safe_load(contract_path.read_text())
    topics = dict(topic_names(contract["topics"]))
    references = {"topic:" + key: name for key, name in topics.items()}
    references.update({"frame:" + key: name for key, name in contract["frames"].items()})
    errors = []
    checked = 0
    files = [src / "xxu_bringup/config" / name for name in (
        "nav2_navigation.yaml", "nav2_exploration.yaml", "nav2_fake_profile.yaml", "rm_sim_lio.yaml")]
    files += [src / "xxu_slam_toolbox/config" / name for name in (
        "slam_toolbox_mapping.yaml", "explore_lite.yaml")]
    files += sorted((src / "small_point_lio/config").glob("xxu*.yaml"))
    files += sorted((src / "xxu_bringup/rviz").glob("*.rviz"))
    files += sorted((src / "xxu_slam_toolbox/rviz").glob("*.rviz"))
    for file in files:
        data = yaml.safe_load(file.read_text())
        for path, value in leaves(data):
            if not isinstance(value, str):
                continue
            key = path[-1].lower()
            rviz_topic = file.suffix == ".rviz" and key == "value" and "topic" in path[-2].lower()
            is_frame = "frame" in key
            is_topic = "topic" in key or rviz_topic
            if not (is_topic or is_frame) or value == "<Fixed Frame>":
                continue
            if file.suffix == ".rviz":
                if is_frame:
                    expected = references["frame:map"]
                else:
                    # The display's class/name defines its semantic topic role.
                    parent = data
                    for part in path[:-2]:
                        parent = parent[int(part)] if isinstance(parent, list) else parent[part]
                    name = parent.get("Name", "")
                    kind = parent.get("Class", "").rsplit("/", 1)[-1]
                    role = {"LaserScan": "localization.scan", "RobotModel": "system.robot_description",
                            "ParticleCloud": "nav2.particle_cloud", "SetInitialPose": "nav2.initialpose",
                            "PointStamped": "nav2.lookahead", "MarkerArray": "nav2.frontiers"}.get(kind)
                    if kind == "Map":
                        role = {"Global Costmap": "nav2.global_costmap", "Local Costmap": "nav2.local_costmap"}.get(name, "system.map")
                        if path[-2] == "Update Topic":
                            role = {"nav2.global_costmap": "nav2.global_costmap_updates",
                                    "nav2.local_costmap": "nav2.local_costmap_updates", "system.map": "system.map_updates"}[role]
                    if kind == "Path":
                        role = "nav2.local_plan" if name == "Controller Local Plan" else "nav2.plan"
                    expected = topics.get(role)
            else:
                expected = references.get(expected_reference(file.name, path))
            checked += 1
            normalized = value if is_frame else "/" + value.lstrip("/")
            if expected is None or normalized != expected:
                errors.append(f"{file.relative_to(workspace)}:{'.'.join(path)}: {value!r}; expected {expected!r}")
    # Resolve every platform sensor-chain endpoint, catching misspelled references.
    sensor = yaml.safe_load((src / "xxu_bringup/config/sensor_chain.yaml").read_text())
    for path, value in leaves(sensor):
        if isinstance(value, str) and value.startswith("@"):
            checked += 1
            if value[1:] not in references:
                errors.append(f"sensor_chain.yaml:{'.'.join(path)}: unknown {value}")
    # CAD models are static consumers too. A valid launch/bridge contract must
    # agree with the actual Gazebo endpoints and the published URDF frames.
    for geometry_file in sorted((src / "xxu_bringup/models").glob("*/geometry.json")):
        geometry = json.loads(geometry_file.read_text())
        sdf_file = geometry_file.with_name("model.sdf")
        urdf_file = geometry_file.with_name("robot.urdf")
        files.extend([sdf_file, urdf_file])
        sdf, urdf = ET.parse(sdf_file), ET.parse(urdf_file)

        def compare(label, actual, expected):
            nonlocal checked
            checked += 1
            if label.endswith((".topic", ".command")):
                actual = ["/" + value.lstrip("/") if value is not None else None for value in actual]
            if actual != expected:
                errors.append(f"{geometry_file.parent.relative_to(workspace)}:{label}: {actual!r}; expected {expected!r}")

        for wheel in geometry["wheels"]:
            controllers = [plugin for plugin in sdf.findall(".//plugin")
                           if plugin.findtext("joint_name") == wheel["name"] + "_joint"
                           and "JointController" in plugin.get("name", "")]
            actual = [plugin.findtext("topic") for plugin in controllers]
            compare(wheel["name"] + ".command", actual,
                    [topics["simulation.wheel_command"].format(wheel=wheel["name"])])
        rgl = [plugin for plugin in sdf.findall(".//plugin")
               if "RGLServerPluginInstance" in plugin.get("name", "")]
        compare("lidar.topic", [p.findtext("topic") for p in rgl], [topics["simulation.rm_lidar"]])
        compare("lidar.frame", [p.findtext("frame") for p in rgl], [contract["frames"]["rm_lidar"]])
        compare("imu.topic", [s.findtext("topic") for s in sdf.findall(".//sensor[@type='imu']")],
                [topics["radar.imu"]])
        joints = [p.findtext("topic") for p in sdf.findall(".//plugin")
                  if "JointStatePublisher" in p.get("name", "")]
        compare("joint_states.topic", joints, [topics["gimbal.joint_states"]])
        urdf_links = {link.get("name") for link in urdf.findall("link")}
        for role in ("navigation_base", "ground_base", "lidar_compensated", "rm_lidar"):
            frame = contract["frames"][role]
            compare("urdf.frame." + role, frame in urdf_links, True)
    return checked, files, errors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--contract", type=Path)
    args = parser.parse_args()
    try:
        count, files, errors = check(args.workspace.resolve(), args.contract)
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError, yaml.YAMLError) as exc:
        print(f"Interface check failed: {exc}", file=sys.stderr)
        return 1
    for error in errors:
        print(error, file=sys.stderr)
    print(f"{count} interface bindings checked in {len(files)} static configs/models and sensor_chain.yaml; {len(errors)} errors")
    return bool(errors)


if __name__ == "__main__":
    sys.exit(main())
