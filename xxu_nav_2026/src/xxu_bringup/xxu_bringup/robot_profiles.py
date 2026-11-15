"""Resolve vehicle geometry profiles independently of simulator actuation."""
import json
from pathlib import Path

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


class RobotProfile:
    """A vehicle geometry profile; empty profile skips model overrides."""

    def __init__(self, name, geometry_file="", geometry=None):
        self.name = name
        self.geometry_file = geometry_file
        self.geometry = geometry or {}


def robot_profile(robot_model, package_share):
    """Select installed model geometry metadata by directory."""
    if robot_model == "upstream":
        return RobotProfile(robot_model)
    if Path(robot_model).name != robot_model or robot_model in ("", ".", ".."):
        raise ValueError(f"Invalid robot model name: {robot_model!r}")
    path = Path(package_share) / "models" / robot_model / "geometry.json"
    geometry = json.loads(path.read_text())
    for key in ("primary_sensor", "wheels", "robot_radius", "base_height", "collision_height"):
        if key not in geometry:
            raise ValueError(f"Robot geometry {path} is missing {key}")
    return RobotProfile(robot_model, str(path), geometry)


class RobotGeometryFile(Substitution):
    """Resolve a model profile after launch argument values are available."""

    def __init__(self, robot_model, package_share, *, sim_backend=None):
        super().__init__()
        self.robot_model = normalize_to_list_of_substitutions(robot_model)
        self.package_share = normalize_to_list_of_substitutions(package_share)
        self.sim_backend = (normalize_to_list_of_substitutions(sim_backend)
                            if sim_backend is not None else None)

    def perform(self, context):
        if self.sim_backend is not None and perform_substitutions(context, self.sim_backend) != "rm_sim_26":
            return ""
        return robot_profile(perform_substitutions(context, self.robot_model),
                             perform_substitutions(context, self.package_share)).geometry_file
