"""Construct platform sensor preprocessing and LIO actions."""
from pathlib import Path

from launch_ros.actions import Node
import yaml

from .interfaces import interfaces


def sensor_chain_defaults(platform):
    """Return the platform's calibration and node defaults from the installed YAML."""
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory("xxu_bringup"))
    return yaml.safe_load((share / "config/sensor_chain.yaml").read_text())[platform]


def lio_preprocessing(platform, *, use_sim_time, config_file, profile=None,
                      pointcloud_overrides=None, imu_overrides=None,
                      pointcloud_remappings=None, imu_remappings=None,
                      lio_overrides=None, conditions=None):
    """Construct the platform's preprocessing and estimator from one definition."""
    definitions = sensor_chain_defaults(platform)
    contract = interfaces()
    overrides = {"pointcloud": dict(pointcloud_overrides or {}), "imu": dict(imu_overrides or {})}
    remaps = {"pointcloud": pointcloud_remappings or {}, "imu": imu_remappings or {}}
    conditions = conditions or {}
    geometry = profile.geometry if profile else {}
    if platform == "rm_sim":
        overrides["pointcloud"].update(self_filter_geometry_file=profile.geometry_file if profile else "",
                                        use_joint_interpolation=not bool(geometry))
        overrides["imu"]["fixed_sensor"] = bool(geometry)
    if geometry:
        sensor = geometry["primary_sensor"]
        overrides["pointcloud"].update(sensor_offset=sensor["offset"], sensor_rpy=sensor["rpy"])
        overrides["imu"].update(sensor_offset=sensor["imu_offset"], sensor_rpy=sensor["rpy"])
    actions = []
    for key, definition in definitions.items():
        parameters = {"use_sim_time": use_sim_time, **definition["parameters"], **overrides.get(key, {})}
        # Config references are semantic contract keys, not a second copy of names.
        parameters = {name: contract.frame(value[7:]) if isinstance(value, str) and value.startswith("@frame:")
                      else value for name, value in parameters.items()}
        mappings = {name: contract.topic(value[7:]) if value.startswith("@topic:") else value
                    for name, value in definition.get("remappings", {}).items()}
        mappings.update(remaps.get(key, {}))
        actions.append(Node(package="xxu_bringup", executable=definition["executable"],
                            name=definition["name"], output="screen", parameters=[parameters],
                            remappings=list(mappings.items()), condition=conditions.get(key)))
    parameters = dict(lio_overrides or {})
    if geometry:
        parameters["planar_z"] = geometry["base_height"]
    # The simulator's LIO YAML already owns use_sim_time. Preserve its exact
    # parameter layering; the other platforms expose it as a launch override.
    files = [config_file]
    if platform == "rm_sim" and geometry:
        model_lio_file = Path(profile.geometry_file).with_name('lio_profile.yaml')
        if model_lio_file.is_file():
            files.append(str(model_lio_file))
    if platform != "rm_sim":
        parameters["use_sim_time"] = use_sim_time
    if parameters:
        files.append(parameters)
    actions.append(Node(package="small_point_lio", executable="small_point_lio_node",
                        name="small_point_lio", output="screen", parameters=files,
                        condition=conditions.get("lio")))
    return actions
