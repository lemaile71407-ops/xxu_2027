"""Simulation-only vehicle controller parameter mapping."""

import math

from xxu_bringup.interfaces import interfaces


def chassis_parameters(profile):
    """Map CAD axle directions to the Gazebo chassis controller convention."""
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
        "drive_direction_angle": [
            math.atan2(wheel["axis"][0], -wheel["axis"][1]) for wheel in wheels
        ],
        "wheel_command_sign": -1.0,
        "max_wheel_speed": geometry["max_wheel_speed"],
        "max_wheel_acceleration": geometry["max_wheel_acceleration"],
        "wheel_command_topics": [
            contract.topic("simulation.wheel_command", wheel=wheel["name"])
            for wheel in wheels
        ],
        "cmd_vel_topic": contract.topic("simulation.safe_velocity"),
        "expected_frame_id": contract.frame("navigation_base"),
        "accept_empty_frame_id": False,
        "require_fresh_stamp": True,
        "timeout": 0.3,
        "publish_rate": 100.0,
    }
