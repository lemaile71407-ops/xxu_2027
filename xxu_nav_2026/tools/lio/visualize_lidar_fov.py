#!/usr/bin/env python3
"""Interactive 3D + 2D LiDAR FOV and post-processing visualizer for ROS 2 bags.

Visualizes:
  1. 3D view (VTK): raw LiDAR point cloud, theoretical sensor FOV envelope
     (Livox Mid-360 -7 deg to +52 deg), filtered/processed point cloud,
     robot footprint, and 2D LaserScan projection ring.
  2. 2D view (Matplotlib): top-down XY projection of raw vs. height/range
     filtered points, 2D LaserScan (/scan or simulated pointcloud_to_laserscan)
     contour, and visible free-space FOV polygon.

Usage:
  source xxu_nav_2026/install/setup.bash  # needed if bag uses livox_ros_driver2/CustomMsg
  python3 xxu_nav_2026/tools/lio/visualize_lidar_fov.py \\
      --bag xxu_nav_2026/bags/mid360s_corridor_test_20261115_1845 \\
      --max-frames 120 --stride 2
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle, Polygon, Rectangle
import vtk
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
from vtkmodules.util import numpy_support


# ---------------------------------------------------------------------------
# Data structures & ROS 2 bag loading
# ---------------------------------------------------------------------------

@dataclass
class FrameData:
    timestamp_s: float
    points_xyz: np.ndarray          # (N, 3) float32
    intensity: np.ndarray           # (N,) float32
    scan_xy: Optional[np.ndarray]   # (M, 2) float32 from recorded /scan if present
    processed_xyz: Optional[np.ndarray]  # (K, 3) float32 from recorded processed topic if present


def _detect_storage_id(bag_path: Path) -> str:
    meta = bag_path / "metadata.yaml"
    if meta.is_file():
        with meta.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        info = data.get("rosbag2_bagfile_information", {})
        storage_id = info.get("storage_identifier")
        if storage_id:
            return str(storage_id)
    if any(bag_path.glob("*.mcap")):
        return "mcap"
    return "sqlite3"


def _decode_pointcloud2(msg) -> Tuple[np.ndarray, np.ndarray]:
    """Vectorized PointCloud2 -> (xyz float32[N,3], intensity float32[N])."""
    n_pts = int(msg.width) * int(msg.height)
    if n_pts == 0 or not msg.data:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)

    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ("x", "y", "z")):
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)

    step = int(msg.point_step)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    usable = (len(raw) // step) * step
    if usable == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    raw = raw[:usable].reshape(-1, step)

    def _read_f32(name: str) -> np.ndarray:
        off = int(field_map[name].offset)
        return raw[:, off:off + 4].view(np.float32).reshape(-1)

    xyz = np.column_stack((_read_f32("x"), _read_f32("y"), _read_f32("z"))).astype(np.float32)

    intensity = np.zeros((len(xyz),), dtype=np.float32)
    for key in ("intensity", "reflectivity"):
        if key in field_map:
            f = field_map[key]
            off = int(f.offset)
            if f.datatype == 7:  # FLOAT32
                intensity = raw[:, off:off + 4].view(np.float32).reshape(-1).copy()
            elif f.datatype == 2:  # UINT8
                intensity = raw[:, off].astype(np.float32)
            break

    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid], intensity[valid]


def _decode_custom_msg(msg) -> Tuple[np.ndarray, np.ndarray]:
    """Decode livox_ros_driver2/msg/CustomMsg."""
    pts = msg.points
    if not pts:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0,), dtype=np.float32)
    arr = np.array([(p.x, p.y, p.z, float(p.reflectivity)) for p in pts], dtype=np.float32)
    xyz = arr[:, :3]
    intensity = arr[:, 3]
    valid = np.isfinite(xyz).all(axis=1) & ((xyz[:, 0] != 0.0) | (xyz[:, 1] != 0.0) | (xyz[:, 2] != 0.0))
    return xyz[valid], intensity[valid]


def _decode_laserscan(msg) -> np.ndarray:
    """Decode sensor_msgs/msg/LaserScan to (M, 2) XY array."""
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    if ranges.size == 0:
        return np.zeros((0, 2), dtype=np.float32)
    angles = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment
    valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a))).astype(np.float32)


def load_bag_frames(
    bag_path: Path,
    cloud_topic: Optional[str] = None,
    processed_topic: Optional[str] = None,
    scan_topic: Optional[str] = None,
    start_sec: float = 0.0,
    duration_sec: float = 0.0,
    stride: int = 1,
    max_frames: int = 150,
) -> Tuple[List[FrameData], Dict[str, str]]:
    """Load point cloud frames (and optional recorded scan/processed topics) from a ROS 2 bag."""
    storage_id = _detect_storage_id(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if not topic_types:
        raise RuntimeError(f"No topics found in bag: {bag_path}")

    # Auto-detect cloud topic
    if cloud_topic is None:
        candidates = [
            "/livox/lidar",
            "/livox/lidar_compensated",
            "/cloud_deskewed",
            "/mid360/points_navigation",
            "/points",
        ]
        for c in candidates:
            if c in topic_types:
                cloud_topic = c
                break
        if cloud_topic is None:
            for name, mtype in topic_types.items():
                if mtype in ("sensor_msgs/msg/PointCloud2", "livox_ros_driver2/msg/CustomMsg"):
                    cloud_topic = name
                    break
    if cloud_topic is None or cloud_topic not in topic_types:
        raise RuntimeError(f"Cloud topic '{cloud_topic}' not found. Available: {list(topic_types.keys())}")

    if processed_topic is None and "/mid360/points_navigation" in topic_types and cloud_topic != "/mid360/points_navigation":
        processed_topic = "/mid360/points_navigation"
    if scan_topic is None and "/scan" in topic_types:
        scan_topic = "/scan"

    selected_topics = [cloud_topic]
    if processed_topic and processed_topic in topic_types:
        selected_topics.append(processed_topic)
    else:
        processed_topic = None
    if scan_topic and scan_topic in topic_types:
        selected_topics.append(scan_topic)
    else:
        scan_topic = None

    msg_classes: Dict[str, type] = {}
    for tname in selected_topics:
        mtype = topic_types[tname]
        try:
            msg_classes[tname] = get_message(mtype)
        except Exception as exc:
            raise RuntimeError(
                f"Cannot import message type '{mtype}' for topic '{tname}'. "
                f"If using livox_ros_driver2/CustomMsg, run: source xxu_nav_2026/install/setup.bash"
            ) from exc

    reader.set_filter(rosbag2_py.StorageFilter(topics=selected_topics))

    frames: List[FrameData] = []
    first_ts_ns: Optional[int] = None
    cloud_msg_idx = 0
    latest_scan_xy: Optional[np.ndarray] = None
    latest_processed_xyz: Optional[np.ndarray] = None

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()
        if first_ts_ns is None:
            first_ts_ns = ts_ns
        rel_s = (ts_ns - first_ts_ns) * 1e-9

        if rel_s < start_sec:
            continue
        if duration_sec > 0.0 and (rel_s - start_sec) > duration_sec:
            break

        msg = deserialize_message(data, msg_classes[topic])
        if topic == scan_topic:
            latest_scan_xy = _decode_laserscan(msg)
            continue
        if topic == processed_topic:
            p_xyz, _ = _decode_pointcloud2(msg)
            latest_processed_xyz = p_xyz
            continue

        if topic == cloud_topic:
            if cloud_msg_idx % max(1, stride) != 0:
                cloud_msg_idx += 1
                continue
            cloud_msg_idx += 1

            if topic_types[topic] == "livox_ros_driver2/msg/CustomMsg":
                xyz, intensity = _decode_custom_msg(msg)
            else:
                xyz, intensity = _decode_pointcloud2(msg)

            frames.append(
                FrameData(
                    timestamp_s=rel_s,
                    points_xyz=xyz,
                    intensity=intensity,
                    scan_xy=None if latest_scan_xy is None else latest_scan_xy.copy(),
                    processed_xyz=None if latest_processed_xyz is None else latest_processed_xyz.copy(),
                )
            )
            if len(frames) % 20 == 0 or len(frames) == max_frames:
                print(f"  loading frames: {len(frames)}/{max_frames} (t={rel_s:.1f}s)", flush=True)
            if len(frames) >= max_frames:
                break

    if not frames:
        raise RuntimeError(f"No frames loaded from topic '{cloud_topic}' in {bag_path}")

    meta = {
        "cloud_topic": cloud_topic,
        "cloud_type": topic_types[cloud_topic],
        "processed_topic": processed_topic or "(computed offline)",
        "scan_topic": scan_topic or "(projected offline)",
    }
    return frames, meta


class LiveTopicSource:
    """Subscribe to live ROS 2 LiDAR point cloud (and optional /scan) into a ring buffer."""

    def __init__(
        self,
        cloud_topic: Optional[str] = None,
        scan_topic: Optional[str] = "/scan",
        live_type: str = "auto",
        max_frames: int = 100,
    ) -> None:
        from collections import deque
        import threading
        import time
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = Node("lidar_fov_visualizer")
        self._lock = threading.Lock()
        self._buffer: deque = deque(maxlen=max(10, max_frames))
        self._latest_scan_xy: Optional[np.ndarray] = None
        self._t0 = time.monotonic()
        self._seq = 0

        # Discover topic & type
        topic = cloud_topic or "/livox/lidar"
        mtype = None
        if live_type in ("CustomMsg", "PointCloud2"):
            mtype = (
                "livox_ros_driver2/msg/CustomMsg"
                if live_type == "CustomMsg"
                else "sensor_msgs/msg/PointCloud2"
            )
        else:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                tmap = {n: types[0] for n, types in self.node.get_topic_names_and_types() if types}
                if cloud_topic and cloud_topic in tmap:
                    mtype = tmap[cloud_topic]
                    break
                if not cloud_topic:
                    for cand in ("/livox/lidar", "/livox/lidar_compensated", "/cloud_deskewed", "/mid360/points_navigation"):
                        if cand in tmap:
                            topic = cand
                            mtype = tmap[cand]
                            break
                if mtype:
                    break
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if mtype is None:
                mtype = "livox_ros_driver2/msg/CustomMsg" if "livox" in topic else "sensor_msgs/msg/PointCloud2"

        msg_cls = get_message(mtype)
        is_custom = mtype == "livox_ros_driver2/msg/CustomMsg"

        def _on_cloud(msg) -> None:
            xyz, intensity = _decode_custom_msg(msg) if is_custom else _decode_pointcloud2(msg)
            rel_s = time.monotonic() - self._t0
            with self._lock:
                scan_copy = None if self._latest_scan_xy is None else self._latest_scan_xy.copy()
                self._buffer.append(
                    FrameData(
                        timestamp_s=rel_s,
                        points_xyz=xyz,
                        intensity=intensity,
                        scan_xy=scan_copy,
                        processed_xyz=None,
                    )
                )
                self._seq += 1

        self.node.create_subscription(msg_cls, topic, _on_cloud, qos_profile_sensor_data)

        tmap = {n: types[0] for n, types in self.node.get_topic_names_and_types() if types}
        active_scan = None
        if scan_topic and scan_topic in tmap and tmap[scan_topic] == "sensor_msgs/msg/LaserScan":
            scan_cls = get_message("sensor_msgs/msg/LaserScan")
            self.node.create_subscription(
                scan_cls,
                scan_topic,
                lambda msg: self._on_scan(msg),
                qos_profile_sensor_data,
            )
            active_scan = scan_topic

        self.meta = {
            "cloud_topic": f"{topic} [LIVE]",
            "cloud_type": mtype,
            "processed_topic": "(computed live)",
            "scan_topic": active_scan or "(projected live)",
        }

        self._stop = False
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def _on_scan(self, msg) -> None:
        xy = _decode_laserscan(msg)
        with self._lock:
            self._latest_scan_xy = xy

    def _spin_loop(self) -> None:
        while not self._stop and self._rclpy.ok():
            self._rclpy.spin_once(self.node, timeout_sec=0.05)

    def snapshot(self) -> Tuple[List[FrameData], int]:
        with self._lock:
            return list(self._buffer), self._seq

    def close(self) -> None:
        if getattr(self, "_closed", False):
            return
        self._closed = True
        self._stop = True
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self.node.destroy_node()
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ---------------------------------------------------------------------------
# Point cloud processing (transform, self-filter, height slice, 2D scan)
# ---------------------------------------------------------------------------

def _euler_zyx_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float32)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float32)
    return rz @ ry @ rx


@dataclass
class SelfFilterGeometry:
    """Convex-hull + wheel-cylinder self-filter loaded from geometry.json."""
    tolerance: float
    hulls: List[np.ndarray]          # list of (P, 4) float32 [a, b, c, d]
    wheels: List[Tuple[np.ndarray, np.ndarray, float, float]]  # (center, axis, half_w, r2)

    @classmethod
    def from_file(cls, path: Path) -> "SelfFilterGeometry":
        import json
        data = json.loads(path.read_text(encoding="utf-8"))
        spec = data["self_filter"]
        tol = float(spec["tolerance"])
        hulls = [np.asarray(planes, dtype=np.float32) for planes in spec.get("hull_planes", [])]
        wheels = []
        for w in data.get("wheels", []):
            center = np.asarray(w["center"], dtype=np.float32)
            axis = np.asarray(w["axis"], dtype=np.float32)
            half_w = float(w["width"]) / 2.0 + tol
            r = float(w["radius"]) + tol
            wheels.append((center, axis, half_w, r * r))
        return cls(tolerance=tol, hulls=hulls, wheels=wheels)

    def inside_mask(self, xyz: np.ndarray) -> np.ndarray:
        """Return boolean mask of points inside robot hull planes or wheel cylinders."""
        inside = np.zeros((xyz.shape[0],), dtype=bool)
        if xyz.shape[0] == 0:
            return inside
        for hull in self.hulls:
            # ax + by + cz + d <= tolerance for all planes of this convex hull
            vals = xyz @ hull[:, :3].T + hull[:, 3]
            inside |= np.all(vals <= self.tolerance, axis=1)
        for center, axis, half_w, r2 in self.wheels:
            delta = xyz - center
            axial = delta @ axis
            radial2 = np.sum(delta * delta, axis=1) - axial * axial
            inside |= (np.abs(axial) <= half_w) & (radial2 <= r2)
        return inside


@dataclass
class ProcessingParams:
    sensor_z: float = 0.42
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    min_height: float = 0.05
    max_height: float = 0.50
    range_min: float = 0.35
    range_max: float = 12.0
    self_filter_box_enabled: bool = True
    self_length_x: float = 0.58
    self_width_y: float = 0.48
    self_filter_z_max: float = 0.45
    height_filter_3d: bool = False
    scan_bins: int = 360
    accumulate_frames: int = 1
    self_geometry: Optional[SelfFilterGeometry] = None


def process_frames(
    frames: List[FrameData],
    index: int,
    params: ProcessingParams,
) -> Dict[str, np.ndarray]:
    """Transform, accumulate, filter, and project to 2D LaserScan."""
    start_idx = max(0, index - max(1, params.accumulate_frames) + 1)
    xyz_list = [frames[i].points_xyz for i in range(start_idx, index + 1) if frames[i].points_xyz.size > 0]
    int_list = [frames[i].intensity for i in range(start_idx, index + 1) if frames[i].points_xyz.size > 0]

    if not xyz_list:
        empty3 = np.zeros((0, 3), dtype=np.float32)
        empty1 = np.zeros((0,), dtype=np.float32)
        return {
            "raw_xyz": empty3,
            "raw_intensity": empty1,
            "proc_xyz": empty3,
            "scan_slice_xyz": empty3,
            "scan_xy": np.zeros((0, 2), dtype=np.float32),
            "recorded_scan_xy": None,
            "scan_angles": empty1,
            "scan_ranges": empty1,
            "scan_valid": np.zeros((0,), dtype=bool),
            "n_raw": 0,
            "n_after_self": 0,
            "n_after_range": 0,
            "n_after_height": 0,
            "self_mode": "3D包围盒",
        }

    raw_xyz = np.vstack(xyz_list)
    raw_int = np.concatenate(int_list)

    rot = _euler_zyx_matrix(
        math.radians(params.roll_deg),
        math.radians(params.pitch_deg),
        math.radians(params.yaw_deg),
    )
    transformed_xyz = raw_xyz @ rot.T
    transformed_xyz[:, 2] += params.sensor_z

    r_xy = np.hypot(transformed_xyz[:, 0], transformed_xyz[:, 1])
    if params.self_geometry is not None:
        in_self_body = params.self_geometry.inside_mask(transformed_xyz)
        self_mode = "凸包+轮体"
    elif params.self_filter_box_enabled:
        half_x = 0.5 * params.self_length_x
        half_y = 0.5 * params.self_width_y
        in_self_body = (
            (np.abs(transformed_xyz[:, 0]) < half_x)
            & (np.abs(transformed_xyz[:, 1]) < half_y)
            & (transformed_xyz[:, 2] >= -0.05)
            & (transformed_xyz[:, 2] <= params.self_filter_z_max)
        )
        self_mode = f"3D盒 {params.self_length_x:.2f}×{params.self_width_y:.2f}m"
    else:
        in_self_body = np.zeros((transformed_xyz.shape[0],), dtype=bool)
        self_mode = "仅半径R_min"
    mask_self = (~in_self_body) & (r_xy >= params.range_min)
    mask_range = mask_self & (r_xy <= params.range_max)
    mask_height = mask_range & (transformed_xyz[:, 2] >= params.min_height) & (transformed_xyz[:, 2] <= params.max_height)

    proc_3d_xyz = transformed_xyz[mask_height if params.height_filter_3d else mask_range]
    scan_slice_xyz = transformed_xyz[mask_height]

    # 2D LaserScan projection (pointcloud_to_laserscan equivalent)
    n_bins = max(36, int(params.scan_bins))
    angles = np.linspace(-math.pi, math.pi, n_bins, endpoint=False, dtype=np.float32)
    scan_ranges = np.full((n_bins,), np.inf, dtype=np.float32)

    if scan_slice_xyz.shape[0] > 0:
        pt_angles = np.arctan2(scan_slice_xyz[:, 1], scan_slice_xyz[:, 0])
        pt_ranges = np.hypot(scan_slice_xyz[:, 0], scan_slice_xyz[:, 1])
        bin_idx = np.floor((pt_angles + math.pi) / (2.0 * math.pi) * n_bins).astype(np.int32)
        bin_idx = np.clip(bin_idx, 0, n_bins - 1)
        np.minimum.at(scan_ranges, bin_idx, pt_ranges)

    scan_valid = np.isfinite(scan_ranges) & (scan_ranges >= params.range_min) & (scan_ranges <= params.range_max)
    scan_xy = np.column_stack((
        scan_ranges[scan_valid] * np.cos(angles[scan_valid]),
        scan_ranges[scan_valid] * np.sin(angles[scan_valid]),
    )).astype(np.float32)

    # Prefer recorded /scan if user didn't accumulate and /scan exists
    recorded_scan = frames[index].scan_xy
    return {
        "raw_xyz": transformed_xyz,
        "raw_intensity": raw_int,
        "proc_xyz": proc_3d_xyz,
        "scan_slice_xyz": scan_slice_xyz,
        "scan_xy": scan_xy,
        "recorded_scan_xy": recorded_scan,
        "scan_angles": angles,
        "scan_ranges": scan_ranges,
        "scan_valid": scan_valid,
        "n_raw": int(transformed_xyz.shape[0]),
        "n_after_self": int(np.count_nonzero(mask_self)),
        "n_after_range": int(np.count_nonzero(mask_range)),
        "n_after_height": int(scan_slice_xyz.shape[0]),
        "self_mode": self_mode,
    }


# ---------------------------------------------------------------------------
# 3D VTK Widget
# ---------------------------------------------------------------------------

def _numpy_to_vtk_polydata(xyz: np.ndarray, scalars: Optional[np.ndarray] = None) -> vtk.vtkPolyData:
    poly = vtk.vtkPolyData()
    if xyz.shape[0] == 0:
        return poly
    pts = vtk.vtkPoints()
    pts.SetData(numpy_support.numpy_to_vtk(np.ascontiguousarray(xyz, dtype=np.float32), deep=True))
    poly.SetPoints(pts)

    n = xyz.shape[0]
    verts = vtk.vtkCellArray()
    conn = np.column_stack((np.ones(n, dtype=np.int64), np.arange(n, dtype=np.int64))).ravel()
    verts.SetCells(n, numpy_support.numpy_to_vtkIdTypeArray(conn, deep=True))
    poly.SetVerts(verts)

    if scalars is not None and scalars.shape[0] == n:
        arr = numpy_support.numpy_to_vtk(np.ascontiguousarray(scalars, dtype=np.float32), deep=True)
        arr.SetName("scalar")
        poly.GetPointData().SetScalars(arr)
    return poly


class Lidar3DWidget(QVTKRenderWindowInteractor):
    """Hardware-accelerated VTK 3D LiDAR FOV + Point Cloud viewer."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.renderer = vtk.vtkRenderer()
        self.renderer.SetBackground(0.07, 0.09, 0.13)
        self.GetRenderWindow().AddRenderer(self.renderer)

        style = vtk.vtkInteractorStyleTrackballCamera()
        self.SetInteractorStyle(style)

        self._init_static_scene()
        self._init_dynamic_actors()
        self._vtk_initialized = False
        self.reset_camera("iso")

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if not self._vtk_initialized:
            self._vtk_initialized = True
            self.Initialize()
            self.GetRenderWindow().Render()

    def _init_static_scene(self) -> None:
        # Ground grid (XY plane at z=0)
        grid = self._build_grid_actor(size=16.0, step=1.0)
        self.renderer.AddActor(grid)

        # Coordinate axes at base_link origin
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(0.7, 0.7, 0.7)
        axes.SetShaftTypeToCylinder()
        axes.AxisLabelsOff()
        self.renderer.AddActor(axes)

        # Robot footprint / self-filter wireframe box
        self.robot_cube = vtk.vtkCubeSource()
        self.robot_cube.SetXLength(0.64)
        self.robot_cube.SetYLength(0.64)
        self.robot_cube.SetZLength(0.45)
        self.robot_cube.SetCenter(0.0, 0.0, 0.225)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(self.robot_cube.GetOutputPort())
        self.robot_actor = vtk.vtkActor()
        self.robot_actor.SetMapper(mapper)
        self.robot_actor.GetProperty().SetRepresentationToWireframe()
        self.robot_actor.GetProperty().SetColor(0.95, 0.75, 0.25)
        self.robot_actor.GetProperty().SetLineWidth(1.5)
        self.renderer.AddActor(self.robot_actor)

        # Livox Mid-360 3D FOV envelope (-7 deg to +52 deg, 360 deg horizontal)
        self.fov_actor = self._build_fov_actor(radius=6.0, elev_min_deg=-7.0, elev_max_deg=52.0)
        self.renderer.AddActor(self.fov_actor)

    def _build_grid_actor(self, size: float = 16.0, step: float = 1.0) -> vtk.vtkActor:
        pts = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        half = size / 2.0
        idx = 0
        vals = np.arange(-half, half + 1e-4, step)
        for v in vals:
            pts.InsertNextPoint(-half, float(v), 0.0)
            pts.InsertNextPoint(half, float(v), 0.0)
            lines.InsertNextCell(2, [idx, idx + 1])
            idx += 2
            pts.InsertNextPoint(float(v), -half, 0.0)
            pts.InsertNextPoint(float(v), half, 0.0)
            lines.InsertNextCell(2, [idx, idx + 1])
            idx += 2
        poly = vtk.vtkPolyData()
        poly.SetPoints(pts)
        poly.SetLines(lines)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.22, 0.26, 0.34)
        actor.GetProperty().SetLineWidth(1.0)
        return actor

    def _build_fov_actor(self, radius: float, elev_min_deg: float, elev_max_deg: float) -> vtk.vtkActor:
        """Build Livox Mid-360 conical FOV shell (-7 deg to +52 deg)."""
        pts = vtk.vtkPoints()
        polys = vtk.vtkCellArray()
        n_az = 48
        elevs = np.linspace(math.radians(elev_min_deg), math.radians(elev_max_deg), 8)
        azs = np.linspace(0.0, 2.0 * math.pi, n_az, endpoint=False)

        # Origin vertex at index 0
        pts.InsertNextPoint(0.0, 0.0, 0.0)
        for el in elevs:
            ce, se = math.cos(el), math.sin(el)
            for az in azs:
                pts.InsertNextPoint(radius * ce * math.cos(az), radius * ce * math.sin(az), radius * se)

        # Bottom cone & top cone fans from origin
        for j in range(n_az):
            jn = (j + 1) % n_az
            polys.InsertNextCell(3, [0, 1 + j, 1 + jn])
            top_base = 1 + (len(elevs) - 1) * n_az
            polys.InsertNextCell(3, [0, top_base + jn, top_base + j])

        # Outer spherical shell strips
        for i in range(len(elevs) - 1):
            r0 = 1 + i * n_az
            r1 = 1 + (i + 1) * n_az
            for j in range(n_az):
                jn = (j + 1) % n_az
                polys.InsertNextCell(4, [r0 + j, r0 + jn, r1 + jn, r1 + j])

        poly = vtk.vtkPolyData()
        poly.SetPoints(pts)
        poly.SetPolys(polys)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.20, 0.65, 0.95)
        actor.GetProperty().SetOpacity(0.10)
        actor.GetProperty().SetEdgeVisibility(True)
        actor.GetProperty().SetEdgeColor(0.25, 0.70, 0.95)
        return actor

    def _init_dynamic_actors(self) -> None:
        # Raw cloud actor (colored by Z)
        self.raw_mapper = vtk.vtkPolyDataMapper()
        lut = vtk.vtkLookupTable()
        lut.SetHueRange(0.66, 0.0)  # blue -> cyan -> green -> yellow -> red
        lut.SetTableRange(-0.2, 2.0)
        lut.Build()
        self.raw_mapper.SetLookupTable(lut)
        self.raw_mapper.SetScalarRange(-0.2, 2.0)
        self.raw_actor = vtk.vtkActor()
        self.raw_actor.SetMapper(self.raw_mapper)
        self.raw_actor.GetProperty().SetPointSize(2.0)
        self.renderer.AddActor(self.raw_actor)

        # Processed cloud actor (bright emerald green)
        self.proc_mapper = vtk.vtkPolyDataMapper()
        self.proc_mapper.ScalarVisibilityOff()
        self.proc_actor = vtk.vtkActor()
        self.proc_actor.SetMapper(self.proc_mapper)
        self.proc_actor.GetProperty().SetColor(0.15, 0.95, 0.55)
        self.proc_actor.GetProperty().SetPointSize(4.0)
        self.renderer.AddActor(self.proc_actor)

        # 2D LaserScan ring in 3D (coral red)
        self.scan_mapper = vtk.vtkPolyDataMapper()
        self.scan_mapper.ScalarVisibilityOff()
        self.scan_actor = vtk.vtkActor()
        self.scan_actor.SetMapper(self.scan_mapper)
        self.scan_actor.GetProperty().SetColor(1.0, 0.30, 0.30)
        self.scan_actor.GetProperty().SetPointSize(5.5)
        self.renderer.AddActor(self.scan_actor)

    def update_scene(
        self,
        raw_xyz: np.ndarray,
        proc_xyz: np.ndarray,
        scan_xy: np.ndarray,
        sensor_z: float,
        roll_deg: float,
        pitch_deg: float,
        yaw_deg: float,
        scan_z: float,
        show_raw: bool,
        show_proc: bool,
        show_scan3d: bool,
        show_fov: bool,
        self_length_x: float = 0.58,
        self_width_y: float = 0.48,
        self_z_max: float = 0.45,
        show_self_box: bool = True,
    ) -> None:
        self.robot_cube.SetXLength(max(0.05, self_length_x))
        self.robot_cube.SetYLength(max(0.05, self_width_y))
        z_len = max(0.05, self_z_max + 0.05)
        self.robot_cube.SetZLength(z_len)
        self.robot_cube.SetCenter(0.0, 0.0, 0.5 * (self_z_max - 0.05))
        self.robot_actor.SetVisibility(1 if show_self_box else 0)

        self.fov_actor.SetPosition(0.0, 0.0, sensor_z)
        self.fov_actor.SetOrientation(roll_deg, pitch_deg, yaw_deg)
        self.fov_actor.SetVisibility(1 if show_fov else 0)

        self.raw_actor.SetVisibility(1 if show_raw else 0)
        if show_raw:
            scalars = raw_xyz[:, 2] if raw_xyz.shape[0] > 0 else None
            self.raw_mapper.SetInputData(_numpy_to_vtk_polydata(raw_xyz, scalars))

        self.proc_actor.SetVisibility(1 if show_proc else 0)
        if show_proc:
            self.proc_mapper.SetInputData(_numpy_to_vtk_polydata(proc_xyz))

        self.scan_actor.SetVisibility(1 if show_scan3d else 0)
        if show_scan3d and scan_xy.shape[0] > 0:
            scan_3d = np.column_stack((scan_xy, np.full((scan_xy.shape[0],), scan_z, dtype=np.float32)))
            self.scan_mapper.SetInputData(_numpy_to_vtk_polydata(scan_3d))
        elif show_scan3d:
            self.scan_mapper.SetInputData(_numpy_to_vtk_polydata(np.zeros((0, 3), dtype=np.float32)))

        if self._vtk_initialized:
            self.GetRenderWindow().Render()

    def reset_camera(self, view: str = "iso") -> None:
        cam = self.renderer.GetActiveCamera()
        cam.SetFocalPoint(0.0, 0.0, 0.4)
        if view == "top":
            cam.SetPosition(0.0, 0.0, 14.0)
            cam.SetViewUp(1.0, 0.0, 0.0)
        elif view == "side":
            cam.SetPosition(0.0, -12.0, 1.5)
            cam.SetViewUp(0.0, 0.0, 1.0)
        else:  # iso
            cam.SetPosition(-8.0, -8.0, 6.5)
            cam.SetViewUp(0.0, 0.0, 1.0)
        self.renderer.ResetCameraClippingRange()
        if self._vtk_initialized:
            self.GetRenderWindow().Render()

    def grab_qimage(self) -> QtGui.QImage:
        self.GetRenderWindow().Render()
        w2if = vtk.vtkWindowToImageFilter()
        w2if.SetInput(self.GetRenderWindow())
        w2if.SetInputBufferTypeToRGB()
        w2if.ReadFrontBufferOff()
        w2if.Update()
        vtk_img = w2if.GetOutput()
        w, h, _ = vtk_img.GetDimensions()
        if w == 0 or h == 0:
            return QtGui.QImage()
        arr = numpy_support.vtk_to_numpy(vtk_img.GetPointData().GetScalars())
        arr = arr.reshape(h, w, 3)[::-1, :, :].copy()
        return QtGui.QImage(arr.data, w, h, w * 3, QtGui.QImage.Format_RGB888).copy()


# ---------------------------------------------------------------------------
# 2D Matplotlib Widget (Post-processed point cloud & LaserScan FOV)
# ---------------------------------------------------------------------------

class Lidar2DWidget(FigureCanvas):
    """Top-down 2D view of filtered point cloud and 2D LaserScan FOV."""

    def __init__(self, parent=None):
        self.fig = Figure(figsize=(6, 6), facecolor="#12161f")
        super().__init__(self.fig)
        self.setParent(parent)
        self.ax = self.fig.add_subplot(111)
        self.fig.subplots_adjust(left=0.09, right=0.97, top=0.92, bottom=0.08)

    def update_plot(
        self,
        raw_xyz: np.ndarray,
        proc_xyz: np.ndarray,
        scan_xy: np.ndarray,
        scan_angles: np.ndarray,
        scan_ranges: np.ndarray,
        scan_valid: np.ndarray,
        range_max: float,
        min_height: float,
        max_height: float,
        frame_idx: int,
        total_frames: int,
        timestamp_s: float,
        recorded_scan_xy: Optional[np.ndarray] = None,
        self_length_x: float = 0.58,
        self_width_y: float = 0.48,
    ) -> None:
        ax = self.ax
        ax.clear()
        ax.set_facecolor("#161b26")
        lim = max(4.0, min(range_max, 12.0))
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal", adjustable="box")

        # Range rings
        for r in (1.0, 2.0, 3.0, 5.0, 8.0, 10.0):
            if r <= lim:
                ax.add_patch(Circle((0, 0), r, fill=False, edgecolor="#2c3548", linestyle="--", linewidth=0.8))
                ax.text(r * 0.707, r * 0.707, f"{r:.0f}m", color="#6c7a96", fontsize=7.5)

        ax.axhline(0.0, color="#2c3548", linewidth=0.8)
        ax.axvline(0.0, color="#2c3548", linewidth=0.8)

        # Free-space / FOV visibility polygon from 2D scan
        vis_r = np.where(scan_valid, scan_ranges, lim)
        poly_x = vis_r * np.cos(scan_angles)
        poly_y = vis_r * np.sin(scan_angles)
        poly_xy = np.column_stack((poly_x, poly_y))
        ax.add_patch(Polygon(poly_xy, closed=True, facecolor="#1f6feb", alpha=0.14, edgecolor="none", label="2D FOV"))

        # Raw cloud XY footprint (subsampled for crisp rendering)
        if raw_xyz.shape[0] > 0:
            step = max(1, raw_xyz.shape[0] // 8000)
            ax.scatter(raw_xyz[::step, 0], raw_xyz[::step, 1], s=1.5, c="#6e7f9e", alpha=0.35, label="Raw XY")

        # Height & range filtered points
        if proc_xyz.shape[0] > 0:
            ax.scatter(proc_xyz[:, 0], proc_xyz[:, 1], s=4.0, c="#26e38c", alpha=0.85, label="Filtered Cloud")

        # 2D LaserScan hits
        if scan_xy.shape[0] > 0:
            ax.scatter(scan_xy[:, 0], scan_xy[:, 1], s=10.0, c="#ff5252", zorder=5, label="2D LaserScan")

        if recorded_scan_xy is not None and recorded_scan_xy.shape[0] > 0:
            ax.scatter(recorded_scan_xy[:, 0], recorded_scan_xy[:, 1], s=14.0, marker="x", c="#ffd166", zorder=6, label="Bag /scan")

        # Robot footprint
        ax.add_patch(Rectangle((-0.5 * self_length_x, -0.5 * self_width_y), self_length_x, self_width_y, fill=False, edgecolor="#f2be42", linewidth=1.5))
        ax.arrow(0.0, 0.0, 0.45, 0.0, head_width=0.12, head_length=0.12, fc="#f2be42", ec="#f2be42", zorder=7)

        cov_pct = 100.0 * float(np.count_nonzero(scan_valid)) / max(1, scan_valid.size)
        ax.set_title(
            f"2D Post-Processed View (z∈[{min_height:.2f}, {max_height:.2f}]m) | "
            f"Frame {frame_idx + 1}/{total_frames} ({timestamp_s:.2f}s)",
            color="#e6edf3",
            fontsize=10,
        )
        ax.set_xlabel("X [m]", color="#9aa7bd", fontsize=9)
        ax.set_ylabel("Y [m]", color="#9aa7bd", fontsize=9)
        ax.tick_params(colors="#9aa7bd", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#30363d")

        stats = (
            f"Raw pts: {raw_xyz.shape[0]}\n"
            f"Filtered: {proc_xyz.shape[0]}\n"
            f"Scan hits: {np.count_nonzero(scan_valid)}/{scan_valid.size} ({cov_pct:.1f}%)"
        )
        ax.text(
            0.02, 0.98, stats, transform=ax.transAxes, va="top", ha="left",
            color="#e6edf3", fontsize=8.5,
            bbox=dict(boxstyle="round,pad=0.35", facecolor="#0d1117", edgecolor="#30363d", alpha=0.85),
        )
        ax.legend(loc="upper right", fontsize=8, facecolor="#0d1117", edgecolor="#30363d", labelcolor="#e6edf3")
        self.draw_idle()


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------

class LidarFovMainWindow(QtWidgets.QMainWindow):
    def __init__(
        self,
        frames: List[FrameData],
        meta: Dict[str, str],
        params: ProcessingParams,
        live_source: Optional[LiveTopicSource] = None,
    ):
        super().__init__()
        self.frames = frames
        self.meta = meta
        self.params = params
        self.live_source = live_source
        self._live_seq = -1
        self.current_idx = 0
        self.playing = live_source is not None

        self.setWindowTitle(f"LiDAR FOV & PointCloud Visualizer — {meta['cloud_topic']}")
        self.resize(1520, 860)
        self.setStyleSheet("background-color: #0d1117; color: #e6edf3;")

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._on_timer_step)

        self._build_ui()
        if self.live_source is not None:
            self.btn_play.setText("暂停")
            self.timer.start(100)
        self._refresh()

    def closeEvent(self, event) -> None:
        if self.live_source is not None:
            self.live_source.close()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Header info & live filtering pipeline explanation
        info = QtWidgets.QLabel(
            f"Cloud: {self.meta['cloud_topic']} ({self.meta['cloud_type']})   |   "
            f"Processed: {self.meta['processed_topic']}   |   Scan: {self.meta['scan_topic']}   |   "
            f"Frames: {len(self.frames)}"
        )
        info.setStyleSheet("color: #8b949e; font-size: 12px; padding: 2px 4px;")
        root.addWidget(info)

        self.lbl_pipeline = QtWidgets.QLabel("")
        self.lbl_pipeline.setStyleSheet(
            "background-color: #161b22; color: #79c0ff; font-size: 12px; "
            "padding: 4px 8px; border: 1px solid #30363d; border-radius: 4px;"
        )
        root.addWidget(self.lbl_pipeline)

        # Main 3-column splitter: Raw 3D | Filtered 3D | 2D LaserScan
        splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        def _wrap_panel(title: str, factory) -> Tuple[QtWidgets.QWidget, QtWidgets.QWidget]:
            box = QtWidgets.QWidget(splitter)
            lay = QtWidgets.QVBoxLayout(box)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(2)
            lbl = QtWidgets.QLabel(title)
            lbl.setStyleSheet("color: #c9d1d9; font-weight: bold; font-size: 12px; padding: 2px 6px; background: #161b22;")
            lay.addWidget(lbl)
            w = factory(box)
            lay.addWidget(w, stretch=1)
            return box, w

        box_raw, self.view3d_raw = _wrap_panel("① 原始 3D 雷达视野 (Raw Cloud + Mid-360 FOV)", Lidar3DWidget)
        box_proc, self.view3d_proc = _wrap_panel("② 过滤后 3D 点云 (自车体/距离/高度切片后)", Lidar3DWidget)
        box_2d, self.view2d = _wrap_panel("③ 2D 处理后画面 (LaserScan + 可见视野)", Lidar2DWidget)
        splitter.addWidget(box_raw)
        splitter.addWidget(box_proc)
        splitter.addWidget(box_2d)
        splitter.setSizes([520, 520, 520])
        root.addWidget(splitter, stretch=1)

        # Playback controls
        play_bar = QtWidgets.QHBoxLayout()
        self.btn_play = QtWidgets.QPushButton("播放")
        self.btn_play.clicked.connect(self._toggle_play)
        self.btn_prev = QtWidgets.QPushButton("上一帧")
        self.btn_prev.clicked.connect(lambda: self._step_frame(-1))
        self.btn_next = QtWidgets.QPushButton("下一帧")
        self.btn_next.clicked.connect(lambda: self._step_frame(1))

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, max(0, len(self.frames) - 1))
        self.slider.valueChanged.connect(self._on_slider_changed)

        self.lbl_frame = QtWidgets.QLabel("0 / 0")
        self.btn_save = QtWidgets.QPushButton("保存截图")
        self.btn_save.clicked.connect(self._on_save_screenshot)

        for w in (self.btn_play, self.btn_prev, self.btn_next):
            w.setStyleSheet("background-color: #21262d; border: 1px solid #30363d; padding: 4px 10px; border-radius: 4px;")
        self.btn_save.setStyleSheet("background-color: #1f6feb; border: none; padding: 4px 12px; border-radius: 4px;")

        play_bar.addWidget(self.btn_play)
        play_bar.addWidget(self.btn_prev)
        play_bar.addWidget(self.btn_next)
        play_bar.addWidget(self.slider, stretch=1)
        play_bar.addWidget(self.lbl_frame)
        play_bar.addWidget(self.btn_save)
        root.addLayout(play_bar)

        # Filter & camera controls
        ctrl_bar = QtWidgets.QHBoxLayout()

        def _make_dspin(label: str, val: float, lo: float, hi: float, step: float, cb) -> QtWidgets.QDoubleSpinBox:
            lbl = QtWidgets.QLabel(label)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setSingleStep(step)
            sp.setDecimals(2)
            sp.setValue(val)
            sp.setStyleSheet("background-color: #161b22; border: 1px solid #30363d; padding: 2px;")
            sp.valueChanged.connect(cb)
            ctrl_bar.addWidget(lbl)
            ctrl_bar.addWidget(sp)
            return sp

        self.sp_zmin = _make_dspin("Z_min(m):", self.params.min_height, -2.0, 3.0, 0.05, self._on_param_changed)
        self.sp_zmax = _make_dspin("Z_max(m):", self.params.max_height, -1.0, 5.0, 0.05, self._on_param_changed)
        self.sp_rmin = _make_dspin("R_min(m):", self.params.range_min, 0.0, 3.0, 0.05, self._on_param_changed)
        self.sp_rmax = _make_dspin("R_max(m):", self.params.range_max, 1.0, 40.0, 0.5, self._on_param_changed)
        self.sp_sz = _make_dspin("Sensor_Z(m):", self.params.sensor_z, -1.0, 2.0, 0.02, self._on_param_changed)
        self.sp_roll = _make_dspin("Roll(°):", self.params.roll_deg, -180.0, 180.0, 1.0, self._on_param_changed)
        self.sp_pitch = _make_dspin("Pitch(°):", self.params.pitch_deg, -180.0, 180.0, 1.0, self._on_param_changed)
        self.sp_sx = _make_dspin("车长(m):", self.params.self_length_x, 0.10, 2.5, 0.02, self._on_param_changed)
        self.sp_sy = _make_dspin("车宽(m):", self.params.self_width_y, 0.10, 2.5, 0.02, self._on_param_changed)
        self.sp_szmax = _make_dspin("Self_Zmax(m):", self.params.self_filter_z_max, 0.05, 2.0, 0.05, self._on_param_changed)
        self.cb_selfbox = QtWidgets.QCheckBox("自车体盒")
        self.cb_selfbox.setChecked(self.params.self_filter_box_enabled)
        self.cb_selfbox.stateChanged.connect(self._on_param_changed)
        ctrl_bar.addWidget(self.cb_selfbox)

        ctrl_bar.addWidget(QtWidgets.QLabel("累积帧:"))
        self.sp_acc = QtWidgets.QSpinBox()
        self.sp_acc.setRange(1, 20)
        self.sp_acc.setValue(self.params.accumulate_frames)
        self.sp_acc.setStyleSheet("background-color: #161b22; border: 1px solid #30363d; padding: 2px;")
        self.sp_acc.valueChanged.connect(self._on_param_changed)
        ctrl_bar.addWidget(self.sp_acc)

        self.cb_raw = QtWidgets.QCheckBox("原始3D")
        self.cb_raw.setChecked(True)
        self.cb_proc = QtWidgets.QCheckBox("过滤3D")
        self.cb_proc.setChecked(True)
        self.cb_h3d = QtWidgets.QCheckBox("3D切高度")
        self.cb_h3d.setChecked(self.params.height_filter_3d)
        self.cb_h3d.stateChanged.connect(self._on_param_changed)
        self.cb_scan3d = QtWidgets.QCheckBox("2D投影环")
        self.cb_scan3d.setChecked(True)
        self.cb_fov = QtWidgets.QCheckBox("视锥包络")
        self.cb_fov.setChecked(True)
        for cb in (self.cb_raw, self.cb_proc, self.cb_scan3d, self.cb_fov):
            cb.stateChanged.connect(self._refresh)
            ctrl_bar.addWidget(cb)
        ctrl_bar.addWidget(self.cb_h3d)

        for label, mode in (("等轴", "iso"), ("俯视", "top"), ("侧视", "side")):
            b = QtWidgets.QPushButton(label)
            b.setStyleSheet("background-color: #21262d; border: 1px solid #30363d; padding: 3px 8px; border-radius: 4px;")
            b.clicked.connect(lambda _=False, m=mode: (self.view3d_raw.reset_camera(m), self.view3d_proc.reset_camera(m)))
            ctrl_bar.addWidget(b)

        root.addLayout(ctrl_bar)

    def _on_param_changed(self) -> None:
        self.params.min_height = self.sp_zmin.value()
        self.params.max_height = self.sp_zmax.value()
        self.params.range_min = self.sp_rmin.value()
        self.params.range_max = self.sp_rmax.value()
        self.params.sensor_z = self.sp_sz.value()
        self.params.roll_deg = self.sp_roll.value()
        self.params.pitch_deg = self.sp_pitch.value()
        self.params.self_length_x = self.sp_sx.value()
        self.params.self_width_y = self.sp_sy.value()
        self.params.self_filter_z_max = self.sp_szmax.value()
        self.params.self_filter_box_enabled = self.cb_selfbox.isChecked()
        self.params.accumulate_frames = self.sp_acc.value()
        self.params.height_filter_3d = self.cb_h3d.isChecked()
        self._refresh()

    def _on_slider_changed(self, value: int) -> None:
        self.current_idx = value
        self._refresh()

    def _step_frame(self, delta: int) -> None:
        nxt = (self.current_idx + delta) % len(self.frames)
        self.slider.setValue(nxt)

    def _toggle_play(self) -> None:
        self.playing = not self.playing
        self.btn_play.setText("暂停" if self.playing else "播放")
        if self.playing:
            self.timer.start(100)
        else:
            self.timer.stop()

    def _on_timer_step(self) -> None:
        if self.live_source is not None:
            buf, seq = self.live_source.snapshot()
            if buf and seq != self._live_seq:
                self._live_seq = seq
                self.frames = buf
                self.slider.blockSignals(True)
                self.slider.setRange(0, len(self.frames) - 1)
                self.current_idx = len(self.frames) - 1
                self.slider.setValue(self.current_idx)
                self.slider.blockSignals(False)
                self._refresh()
        else:
            self._step_frame(1)

    def save_screenshot(self, path: str) -> None:
        pixmap = self.grab()
        painter = QtGui.QPainter(pixmap)
        for view in (self.view3d_raw, self.view3d_proc):
            vtk_img = view.grab_qimage()
            if not vtk_img.isNull():
                top_left = view.mapTo(self, QtCore.QPoint(0, 0))
                painter.drawImage(top_left, vtk_img.scaled(view.size(), QtCore.Qt.IgnoreAspectRatio, QtCore.Qt.SmoothTransformation))
        painter.end()
        pixmap.save(path)

    def _on_save_screenshot(self) -> None:
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存截图", "lidar_fov_view.png", "PNG (*.png)")
        if path:
            self.save_screenshot(path)

    def _refresh(self) -> None:
        idx = self.current_idx
        self.lbl_frame.setText(f"{idx + 1} / {len(self.frames)}")
        res = process_frames(self.frames, idx, self.params)

        n_scan = int(np.count_nonzero(res["scan_valid"]))
        n_bins = int(res["scan_valid"].size)
        h3d_txt = f"3D切高度Z∈[{self.params.min_height:.2f},{self.params.max_height:.2f}]" if self.params.height_filter_3d else "3D高度不切"
        self.lbl_pipeline.setText(
            f"过滤流程: 原始 {res['n_raw']} 点  →  "
            f"①自车体[{res['self_mode']}]&近场(r<{self.params.range_min:.2f}m) 剩 {res['n_after_self']}  →  "
            f"②远场(r>{self.params.range_max:.1f}m) 剩 {res['n_after_range']} [{h3d_txt}]  →  "
            f"③2D高度切片(Z∈[{self.params.min_height:.2f}, {self.params.max_height:.2f}]m) 剩 {res['n_after_height']}  →  "
            f"④2D Scan {n_scan}/{n_bins}"
        )

        # Panel 1: Raw 3D FOV (raw cloud + FOV cone + optional overlay)
        self.view3d_raw.update_scene(
            raw_xyz=res["raw_xyz"],
            proc_xyz=res["proc_xyz"],
            scan_xy=res["scan_xy"],
            sensor_z=self.params.sensor_z,
            roll_deg=self.params.roll_deg,
            pitch_deg=self.params.pitch_deg,
            yaw_deg=self.params.yaw_deg,
            scan_z=self.params.min_height,
            show_raw=self.cb_raw.isChecked(),
            show_proc=False,
            show_scan3d=False,
            show_fov=self.cb_fov.isChecked(),
            self_length_x=self.params.self_length_x,
            self_width_y=self.params.self_width_y,
            self_z_max=self.params.self_filter_z_max,
            show_self_box=self.params.self_filter_box_enabled,
        )
        # Panel 2: Filtered 3D Cloud (only filtered obstacle points + 2D scan ring)
        self.view3d_proc.update_scene(
            raw_xyz=res["raw_xyz"],
            proc_xyz=res["proc_xyz"],
            scan_xy=res["scan_xy"],
            sensor_z=self.params.sensor_z,
            roll_deg=self.params.roll_deg,
            pitch_deg=self.params.pitch_deg,
            yaw_deg=self.params.yaw_deg,
            scan_z=self.params.min_height,
            show_raw=False,
            show_proc=self.cb_proc.isChecked(),
            show_scan3d=self.cb_scan3d.isChecked(),
            show_fov=False,
            self_length_x=self.params.self_length_x,
            self_width_y=self.params.self_width_y,
            self_z_max=self.params.self_filter_z_max,
            show_self_box=self.params.self_filter_box_enabled,
        )
        self.view2d.update_plot(
            raw_xyz=res["raw_xyz"],
            proc_xyz=res["scan_slice_xyz"],
            scan_xy=res["scan_xy"],
            scan_angles=res["scan_angles"],
            scan_ranges=res["scan_ranges"],
            scan_valid=res["scan_valid"],
            range_max=self.params.range_max,
            min_height=self.params.min_height,
            max_height=self.params.max_height,
            frame_idx=idx,
            total_frames=len(self.frames),
            timestamp_s=self.frames[idx].timestamp_s,
            recorded_scan_xy=res["recorded_scan_xy"],
            self_length_x=self.params.self_length_x,
            self_width_y=self.params.self_width_y,
        )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Interactive 3D + 2D LiDAR FOV & point cloud processing visualizer")
    p.add_argument("--bag", default=None, help="Path to ROS 2 bag directory (mcap or sqlite3)")
    p.add_argument("--live", action="store_true", help="Subscribe to live ROS 2 LiDAR topic instead of reading a bag")
    p.add_argument("--live-type", choices=("auto", "CustomMsg", "PointCloud2"), default="auto", help="Live cloud message type")
    p.add_argument("--cloud-topic", default=None, help="PointCloud2 or CustomMsg topic (auto-detected if omitted)")
    p.add_argument("--processed-topic", default=None, help="Optional recorded processed PointCloud2 topic")
    p.add_argument("--scan-topic", default=None, help="Optional recorded LaserScan topic")
    p.add_argument("--start-sec", type=float, default=0.0, help="Start offset in seconds")
    p.add_argument("--duration-sec", type=float, default=0.0, help="Duration to load in seconds (0 = until max-frames)")
    p.add_argument("--stride", type=int, default=1, help="Load every N-th cloud frame")
    p.add_argument("--max-frames", type=int, default=150, help="Maximum frames to load into memory")
    p.add_argument("--min-height", type=float, default=0.05, help="LaserScan min height [m]")
    p.add_argument("--max-height", type=float, default=0.50, help="LaserScan max height [m]")
    p.add_argument("--range-min", type=float, default=0.35, help="Min range [m]")
    p.add_argument("--range-max", type=float, default=12.0, help="Max range [m]")
    p.add_argument("--sensor-z", type=float, default=0.42, help="Sensor Z height above base_link [m]")
    p.add_argument("--roll-deg", type=float, default=0.0, help="Sensor roll [deg]")
    p.add_argument("--pitch-deg", type=float, default=0.0, help="Sensor pitch [deg]")
    p.add_argument("--yaw-deg", type=float, default=0.0, help="Sensor yaw [deg]")
    p.add_argument("--accumulate", type=int, default=1, help="Number of frames to accumulate")
    p.add_argument("--geometry-file", default=None, help="Optional geometry.json for convex-hull + wheel self-filter")
    p.add_argument("--height-filter-3d", action="store_true", help="Also apply Z_min..Z_max height slicing to 3D filtered view")
    p.add_argument("--snapshot-out", default=None, help="Render first frame to PNG and exit (for headless check)")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    if not args.live and not args.bag:
        print("Error: specify either --bag <path> or --live", file=sys.stderr)
        return 1

    live_source: Optional[LiveTopicSource] = None
    if args.live:
        live_source = LiveTopicSource(
            cloud_topic=args.cloud_topic,
            scan_topic=args.scan_topic or "/scan",
            live_type=args.live_type,
            max_frames=args.max_frames,
        )
        meta = live_source.meta
        frames = [
            FrameData(
                timestamp_s=0.0,
                points_xyz=np.zeros((0, 3), dtype=np.float32),
                intensity=np.zeros((0,), dtype=np.float32),
                scan_xy=None,
                processed_xyz=None,
            )
        ]
        print(f"Subscribed live: {meta}")
    else:
        bag_path = Path(args.bag).expanduser().resolve()
        if not bag_path.exists():
            print(f"Error: bag path not found: {bag_path}", file=sys.stderr)
            return 1

        frames, meta = load_bag_frames(
            bag_path=bag_path,
            cloud_topic=args.cloud_topic,
            processed_topic=args.processed_topic,
            scan_topic=args.scan_topic,
            start_sec=args.start_sec,
            duration_sec=args.duration_sec,
            stride=args.stride,
            max_frames=args.max_frames,
        )
        print(f"Loaded {len(frames)} frames from {bag_path.name} ({meta})")

    self_geom = None
    if args.geometry_file:
        gpath = Path(args.geometry_file).expanduser().resolve()
        if not gpath.is_file():
            print(f"Error: geometry file not found: {gpath}", file=sys.stderr)
            return 1
        self_geom = SelfFilterGeometry.from_file(gpath)

    params = ProcessingParams(
        sensor_z=args.sensor_z,
        roll_deg=args.roll_deg,
        pitch_deg=args.pitch_deg,
        yaw_deg=args.yaw_deg,
        min_height=args.min_height,
        max_height=args.max_height,
        range_min=args.range_min,
        range_max=args.range_max,
        height_filter_3d=args.height_filter_3d,
        accumulate_frames=args.accumulate,
        self_geometry=self_geom,
    )

    app = QtWidgets.QApplication(sys.argv)
    win = LidarFovMainWindow(frames, meta, params, live_source=live_source)
    if live_source is not None:
        app.aboutToQuit.connect(live_source.close)
    win.show()

    if args.snapshot_out:
        QtCore.QTimer.singleShot(600, lambda: (win.save_screenshot(args.snapshot_out), win.close()))

    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
