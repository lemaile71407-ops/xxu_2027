"""Chassis-independent rigid sensor TF, preprocessing and optional base bridge.

Does not start hardware drivers, LIO/backend, Nav2 or chassis actuation.
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from xxu_bringup.sensor_launch import load_rigid_mount, rigid_sensor_actions


def setup(context):
    lc = lambda name: LaunchConfiguration(name).perform(context)
    mount = load_rigid_mount(lc("mount_file"))  # Validate before creating any action.
    if lc("odom_frame") in {mount["frames"][key] for key in ("base", "sensor", "lidar")}:
        raise ValueError("odom_frame must be distinct from rigid mount frames")
    actions = rigid_sensor_actions(
        mount, use_sim_time=ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        publish_mount_tf=lc("publish_mount_tf") == "true")
    if lc("start_base_adapter") == "true":
        share = Path(get_package_share_directory("xxu_bringup"))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/zlio_base_adapter.launch.py")),
            launch_arguments={
                "use_sim_time": lc("use_sim_time"),
                "sensor_frame": mount["frames"]["sensor"],
                "base_frame": mount["frames"]["base"],
                **{name: lc(name) for name in ("source_child_frame", "odom_frame", "input_topic",
                                              "output_topic", "publish_deskewed_cloud")},
            }.items()))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("xxu_bringup"))
    defaults = {
        "mount_file": str(share / "config/rigid_mount.pending.yaml"),
        "source_child_frame": "imu", "odom_frame": "odom",
        "input_topic": "/odometry", "output_topic": "/odom",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
        *(DeclareLaunchArgument(name, default_value=value, choices=["true", "false"])
          for name, value in {"use_sim_time": "false", "publish_mount_tf": "true",
                              "start_base_adapter": "true", "publish_deskewed_cloud": "false"}.items()),
        OpaqueFunction(function=setup),
    ])
