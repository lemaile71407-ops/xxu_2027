"""Resolve vehicle profiles and chassis parameters."""
import json
import math
from pathlib import Path

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from .interfaces import interfaces


class RobotProfile:
    """A CAD vehicle profile; the upstream simulator is the empty profile."""

    def __init__(self, name, geometry_file="", geometry=None):
        self.name = name
        self.geometry_file = geometry_file
        self.geometry = geometry or {}


def robot_profile(robot_model, bringup_share):
    """Select installed model assets by directory, with no vehicle-name branches."""
    if robot_model == "upstream":
        return RobotProfile(robot_model)
    if Path(robot_model).name != robot_model or robot_model in ("", ".", ".."):
        raise ValueError(f"Invalid robot model name: {robot_model!r}")
    path = Path(bringup_share) / "models" / robot_model / "geometry.json"
    geometry = json.loads(path.read_text())
    # CAD profiles describe a rigid sensor and four individually driven wheels.
    for key in ("primary_sensor", "wheels", "robot_radius", "base_height", "collision_height"):
        if key not in geometry:
            raise ValueError(f"Robot geometry {path} is missing {key}")
    return RobotProfile(robot_model, str(path), geometry)


class RobotGeometryFile(Substitution):
    """Resolve a model profile after launch argument values are available."""

    def __init__(self, robot_model, bringup_share, *, sim_backend=None):
        super().__init__()
        self.robot_model = normalize_to_list_of_substitutions(robot_model)
        self.bringup_share = normalize_to_list_of_substitutions(bringup_share)
        self.sim_backend = (normalize_to_list_of_substitutions(sim_backend)
                            if sim_backend is not None else None)

    def perform(self, context):
        if self.sim_backend is not None and perform_substitutions(context, self.sim_backend) != "rm_sim_26":
            return ""
        return robot_profile(perform_substitutions(context, self.robot_model),
                             perform_substitutions(context, self.bringup_share)).geometry_file


def chassis_parameters(profile):
    """Map CAD axle directions to the existing chassis controller convention."""
    geometry = profile.geometry
    wheels = geometry["wheels"]
    if len(wheels) != 4 or len({wheel["name"] for wheel in wheels}) != 4:
        raise ValueError("A CAD chassis profile must contain four distinct wheels")
    contract = interfaces()
    for wheel in wheels:
        ax, ay = wheel["axis"][:2]
        if not math.isclose(math.hypot(ax, ay), 1.0, abs_tol=1e-9):
            raise ValueError("CAD wheel axes must be unit vectors in the XY plane")
    return {
        "wheel_radii": [wheel["radius"] for wheel in wheels],
        "wheel_x": [wheel["center"][0] for wheel in wheels],
        "wheel_y": [wheel["center"][1] for wheel in wheels],
        "drive_direction_angle": [math.atan2(wheel["axis"][0], -wheel["axis"][1])
                                  for wheel in wheels],
        "wheel_command_sign": -1.0,
        "max_wheel_speed": geometry["max_wheel_speed"],
        "max_wheel_acceleration": geometry["max_wheel_acceleration"],
        "wheel_command_topics": [contract.topic("simulation.wheel_command", wheel=wheel["name"]) for wheel in wheels],
        "cmd_vel_topic": contract.topic("simulation.safe_velocity"),
        "expected_frame_id": contract.frame("navigation_base"),
        "accept_empty_frame_id": False,
        "require_fresh_stamp": True,
        "timeout": 0.3,
        "publish_rate": 100.0,
    }
