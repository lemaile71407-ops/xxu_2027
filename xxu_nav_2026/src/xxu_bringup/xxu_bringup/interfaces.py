"""Robot interface data, independent of ROS launch and Nav2."""

from pathlib import Path

import yaml


class InterfaceContract:
    """Named topics and frames shared by all integration launch files."""

    def __init__(self, data):
        if data.get("version") != 1:
            raise ValueError("Unsupported robot interface contract version")
        self.data = data

    def topic(self, key, *, relative=False, **fields):
        value = self.data["topics"]
        for part in key.split("."):
            value = value[part]
        name = value["name"].format(**fields)
        if not isinstance(name, str) or not name.startswith("/"):
            raise ValueError(f"Interface topic {key} must be absolute")
        return name.lstrip("/") if relative else name

    def frame(self, key):
        return self.data["frames"][key]


def interfaces():
    """Load the installed contract; launch arguments can override its defaults."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("xxu_bringup")) / "config/robot_interfaces.yaml"
    return InterfaceContract(yaml.safe_load(path.read_text()))
