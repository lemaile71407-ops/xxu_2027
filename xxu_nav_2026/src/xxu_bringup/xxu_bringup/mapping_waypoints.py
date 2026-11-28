"""Parse map-frame mission waypoints without a ROS runtime."""

import csv
from dataclasses import dataclass
import math
from pathlib import Path


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


def load_waypoints_csv(path):
    path = Path(path)
    with path.open(encoding='utf-8', newline='') as stream:
        numbered_lines = (
            (number, line) for number, line in enumerate(stream, start=1)
            if line.strip() and not line.lstrip().startswith('#')
        )
        _, header = next(numbered_lines, (0, ''))
        if next(csv.reader([header])) != ['name', 'x', 'y', 'yaw']:
            raise ValueError(f'{path}: expected CSV header name,x,y,yaw')
        waypoints = []
        for number, line in numbered_lines:
            try:
                name, x, y, yaw = next(csv.reader([line], strict=True))
                coordinates = (float(x), float(y), float(yaw))
            except (csv.Error, ValueError) as error:
                raise ValueError(f'{path}:{number}: expected name,x,y,yaw') from error
            if not name.strip() or not all(map(math.isfinite, coordinates)):
                raise ValueError(f'{path}:{number}: expected a name and finite x, y, yaw')
            waypoints.append(Waypoint(name.strip(), *coordinates))
    if not waypoints:
        raise ValueError(f'{path}: expected at least one waypoint')
    return waypoints


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
