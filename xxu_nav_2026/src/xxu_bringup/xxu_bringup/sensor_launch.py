"""Construct platform sensor preprocessing and LIO actions."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

from launch_ros.actions import Node
import yaml

from .interfaces import interfaces


def _pose(xyz, rpy):
    """Homogeneous pose as (translation, rotation), using URDF Rz Ry Rx."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return list(xyz), [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                       [-sp, cp*sr, cp*cr]]


def _compose(a, b):
    t, r = a
    u, s = b
    return ([t[i] + sum(r[i][j]*u[j] for j in range(3)) for i in range(3)],
            [[sum(r[i][k]*s[k][j] for k in range(3)) for j in range(3)] for i in range(3)])


def _static_action(parent, child, pose):
    xyz, r = pose
    pitch = math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0]))
    if math.hypot(r[0][0], r[1][0]) > 1.e-10:
        roll, yaw = math.atan2(r[2][1], r[2][2]), math.atan2(r[1][0], r[0][0])
    else:
        roll, yaw = math.atan2(-r[1][2], r[1][1]), 0.0
    arguments = ['--frame-id', parent, '--child-frame-id', child]
    for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), xyz + [roll, pitch, yaw]):
        arguments.extend(['--' + name, str(value)])
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name='mount_' + child, arguments=arguments,
                parameters=[{'use_sim_time': False}], output='screen')


def navigation_static_actions(robot_description, base, sensors, *, use_sim_time):
    """Collapse only fixed URDF chains into direct base-to-sensor edges.

    Mechanical joints remain in the simulation model, not in navigation TF.
    A moving sensor must use the existing analytical joint compensation;
    this function refuses to freeze a moving joint into a static transform.
    """
    root = ET.fromstring(robot_description)
    joints = {j.find('child').get('link'): j for j in root.findall('joint')}
    actions = []
    for sensor in dict.fromkeys(sensors):
        chain, child, seen = [], sensor, set()
        while child != base:
            if child in seen or child not in joints:
                raise ValueError(f'No unique fixed mount from {base} to {sensor}')
            seen.add(child)
            joint = joints[child]
            if joint.get('type') != 'fixed':
                raise ValueError(f'Moving mount for {sensor}: {joint.get("name")}')
            origin = joint.find('origin')
            vector = lambda key: [float(v) for v in (origin.get(key, '0 0 0') if origin is not None else '0 0 0').split()]
            chain.append(_pose(vector('xyz'), vector('rpy')))
            child = joint.find('parent').get('link')
        pose = _pose([0., 0., 0.], [0., 0., 0.])
        for part in reversed(chain):
            pose = _compose(pose, part)
        actions.append(_static_action(base, sensor, pose))
    return actions


def load_rigid_mount(path):
    """Load measured rigid extrinsics independently of CAD/chassis kinematics.

    The virtual sensor origin is the physical IMU origin. Point clouds and IMU
    vectors are both expressed in this frame before entering an identity-
    extrinsic LIO frontend. No gimbal state or lever-arm differentiation is used.
    """
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get("version") != 1:
        raise ValueError("Rigid mount requires version: 1")
    if data.get("mounting") != "rigid":
        raise ValueError("This entry requires mounting: rigid")
    if data.get("calibration_confirmed") is not True:
        raise ValueError("Rigid mount extrinsics are pending; measure them and set calibration_confirmed: true")

    def vector(value, name):
        if (not isinstance(value, list) or len(value) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
            raise ValueError(f"{name} must contain three measured finite numbers")
        return [float(v) for v in value]

    frames = data.get("frames", {})
    if not isinstance(frames, dict):
        raise ValueError("Rigid mount frames must be a mapping")
    reserved_frames = {"map", "odom", "base_footprint"}
    for key in ("base", "sensor", "lidar", "imu_source"):
        value = frames.get(key)
        if (not isinstance(value, str) or not value or value.startswith("/")
                or any(c.isspace() for c in value)):
            raise ValueError(f"frames.{key} must be a nonempty TF frame without a leading slash or whitespace")
        if value in reserved_frames:
            raise ValueError(f"frames.{key} cannot use reserved navigation TF frame: {value}")
    if len({frames[key] for key in ("base", "sensor", "lidar")}) != 3:
        raise ValueError("Rigid mount base, sensor and lidar frames must be distinct")
    for key in ("base_to_sensor", "sensor_to_lidar"):
        transform = data.get(key)
        if not isinstance(transform, dict):
            raise ValueError(f"Missing measured transform {key}")
        data[key] = {axis: vector(transform.get(axis), f"{key}.{axis}") for axis in ("xyz", "rpy")}
    data["sensor_from_imu_rpy"] = vector(data.get("sensor_from_imu_rpy"), "sensor_from_imu_rpy")
    scale = data.get("imu_acceleration_scale")
    if type(scale) not in (int, float) or not math.isfinite(scale) or scale <= 0:
        raise ValueError("imu_acceleration_scale must be a positive conversion to m/s^2")
    topics = data.get("topics", {})
    if not isinstance(topics, dict):
        raise ValueError("Rigid mount topics must be a mapping")
    for key in ("points_in", "points_out", "imu_in", "imu_out"):
        value = topics.get(key)
        if (not isinstance(value, str) or not value.startswith("/") or len(value) < 2
                or any(c.isspace() for c in value)):
            raise ValueError(f"topics.{key} must be an absolute ROS topic")
    if len(set(topics[key] for key in ("points_in", "points_out", "imu_in", "imu_out"))) != 4:
        raise ValueError("Rigid mount input/output topics must be distinct (no republishing loops)")
    return data


def rigid_sensor_actions(mount, *, use_sim_time, publish_mount_tf=True, self_filter_geometry_file=""):
    """Build the same rigid extrinsics into static TF and preprocessing."""
    frames = mount["frames"]
    pointcloud_parameters = {
        "use_sim_time": False, "target_frame": frames["sensor"],
        "expected_source_frame": frames["lidar"],
        "use_point_timestamps": False, "use_joint_interpolation": False,
    }
    if self_filter_geometry_file:
        geometry_path = Path(self_filter_geometry_file)
        if not geometry_path.is_file():
            raise ValueError(f"self_filter_geometry_file must be an existing file: {geometry_path}")
        # The C++ filter checks that the geometry frame matches target_frame.
        pointcloud_parameters["self_filter_geometry_file"] = str(geometry_path)
    actions = []
    if publish_mount_tf:
        sensor = _pose(**mount['base_to_sensor'])
        lidar = _compose(sensor, _pose(**mount['sensor_to_lidar']))
        actions += [_static_action(frames['base'], frames['sensor'], sensor),
                    _static_action(frames['base'], frames['lidar'], lidar)]
    # Fixed TF is identical for every acquisition time, so one lookup per cloud
    # preserves all point-time fields without the dynamic TF group limit.
    actions.append(Node(
        package="xxu_bringup", executable="pointcloud_frame_republisher",
        name="rigid_pointcloud_transform", output="screen", parameters=[pointcloud_parameters],
        remappings=[("points_in", mount["topics"]["points_in"]),
                    ("points_out", mount["topics"]["points_out"])]))
    actions.append(Node(
        package="xxu_bringup", executable="gimbal_imu_compensator",
        name="rigid_imu_transform", output="screen", parameters=[{
            "use_sim_time": False, "target_frame": frames["sensor"],
            "fixed_sensor": True, "sensor_offset": [0.0, 0.0, 0.0],
            "expected_source_frame": frames["imu_source"],
            "sensor_rpy": mount["sensor_from_imu_rpy"], "joint_position_offset": 0.0,
            "input_acceleration_scale": float(mount["imu_acceleration_scale"]),
        }], remappings=[("imu_in", mount["topics"]["imu_in"]),
                        ("imu_out", mount["topics"]["imu_out"])]))
    return actions


def sensor_chain_defaults(platform):
    """Return the platform's calibration and node defaults from the installed YAML."""
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory("xxu_bringup"))
    return yaml.safe_load((share / "config/sensor_chain.yaml").read_text())[platform]


def sensor_preprocessing(platform, *, use_sim_time, profile=None,
                         pointcloud_overrides=None, imu_overrides=None,
                         pointcloud_remappings=None, imu_remappings=None,
                         conditions=None):
    """Construct platform preprocessing without choosing a localization backend."""
    definitions = sensor_chain_defaults(platform)
    contract = interfaces()
    overrides = {"pointcloud": dict(pointcloud_overrides or {}), "imu": dict(imu_overrides or {})}
    remaps = {"pointcloud": pointcloud_remappings or {}, "imu": imu_remappings or {}}
    conditions = conditions or {}
    geometry = profile.geometry if profile else {}
    if platform == "rm_sim":
        overrides["pointcloud"].update(self_filter_geometry_file=profile.geometry_file if profile else "",
                                        use_point_timestamps=not bool(geometry),
                                        use_joint_interpolation=not bool(geometry))
        overrides["imu"]["fixed_sensor"] = bool(geometry)
    if geometry:
        sensor = geometry["primary_sensor"]
        overrides["pointcloud"].update(sensor_offset=sensor["offset"], sensor_rpy=sensor["rpy"])
        overrides["imu"].update(sensor_offset=sensor["imu_offset"], sensor_rpy=sensor["rpy"])
    actions = []
    for key, definition in definitions.items():
        parameters = {"use_sim_time": use_sim_time, **definition["parameters"], **overrides.get(key, {})}
        # Acquisition stamps drive compensation; pending expiry uses steady time.
        # Neither preprocessor needs a high-rate /clock subscription.
        if definition["executable"] in ("pointcloud_frame_republisher", "gimbal_imu_compensator"):
            parameters["use_sim_time"] = False
        # Config references are semantic contract keys, not a second copy of names.
        parameters = {name: contract.frame(value[7:]) if isinstance(value, str) and value.startswith("@frame:")
                      else value for name, value in parameters.items()}
        mappings = {name: contract.topic(value[7:]) if value.startswith("@topic:") else value
                    for name, value in definition.get("remappings", {}).items()}
        mappings.update(remaps.get(key, {}))
        actions.append(Node(package="xxu_bringup", executable=definition["executable"],
                            name=definition["name"], output="screen", parameters=[parameters],
                            remappings=list(mappings.items()), condition=conditions.get(key)))
    return actions
