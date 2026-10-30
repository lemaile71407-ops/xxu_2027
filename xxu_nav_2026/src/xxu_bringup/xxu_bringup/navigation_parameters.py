"""Nav2 parameter construction and vehicle navigation geometry."""
import json
import math
from pathlib import Path
import tempfile

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch.substitutions import PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml
import yaml

from .interfaces import interfaces


class RobotGeometryParameters(Substitution):
    """Apply a vehicle envelope and an optional, mode-specific navigation profile."""
    def __init__(self, source, geometry_file, *, mode="navigation"):
        super().__init__()
        self.source = normalize_to_list_of_substitutions(source)
        self.geometry_file = normalize_to_list_of_substitutions(geometry_file)
        self.output = None
        self.mode = mode

    def perform(self, context):
        geometry_file = perform_substitutions(context, self.geometry_file)
        if not geometry_file:
            return perform_substitutions(context, self.source)
        if self.output:
            return self.output
        geometry = json.loads(Path(geometry_file).read_text())
        profile_file = Path(geometry_file).with_name('navigation_profile.yaml')
        profiles = yaml.safe_load(profile_file.read_text()) if profile_file.is_file() else {}
        if profiles is not None and not isinstance(profiles, dict):
            raise ValueError('Vehicle navigation profiles must be a mapping')
        profile = (profiles or {}).get(self.mode, {})
        if not isinstance(profile, dict):
            raise ValueError('Vehicle navigation mode settings must be a mapping')
        for key, value in profile.items():
            if key not in {'minimum_inflation_radius', 'translation_velocity_limit', 'goal_xy_tolerance'}:
                raise ValueError(f'Unknown vehicle navigation setting: {key}')
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f'Invalid vehicle navigation setting: {key}')
        radius = float(geometry['robot_radius'])
        if not math.isfinite(radius) or not 0 < radius < 2:
            raise ValueError('Invalid vehicle navigation radius')
        source = perform_substitutions(context, self.source)
        params = yaml.safe_load(Path(source).read_text())
        # Circumscribe the vehicle plus 3 cm; octagon edges must not cut into it.
        stop_radius = (radius + .03) / math.cos(math.pi / 8)
        for name in ['local_costmap', 'global_costmap']:
            ros = params[name][name]['ros__parameters']
            if profile:
                inflation = ros['inflation_layer']
                old_inflation = float(inflation['inflation_radius'])
                clearance = max(0., old_inflation - float(ros['robot_radius']))
                # Preserve surface clearance and cover the stop polygon plus
                # at least one soft costmap cell, only for an opted-in profile.
                inflation['inflation_radius'] = max(
                    old_inflation, radius + clearance,
                    float(profile.get('minimum_inflation_radius', 0.)),
                    stop_radius + float(ros['resolution']))
            ros['robot_radius'] = radius
            ros.pop('footprint', None)
        polygon = [[round(stop_radius * math.cos(i * math.pi / 4), 5),
                    round(stop_radius * math.sin(i * math.pi / 4), 5)] for i in range(8)]
        params['collision_monitor']['ros__parameters']['EmergencyStop']['points'] = json.dumps(polygon)
        controller = params['controller_server']['ros__parameters'] if profile else None
        if 'translation_velocity_limit' in profile:
            limit = float(profile['translation_velocity_limit'])
            follow = controller['FollowPath']
            follow['vx_max'] = min(follow['vx_max'], limit)
            follow['vx_min'] = max(follow['vx_min'], -limit)
            follow['vy_max'] = min(follow['vy_max'], limit)
            smoother = params['velocity_smoother']['ros__parameters']
            for axis in (0, 1):
                smoother['max_velocity'][axis] = min(smoother['max_velocity'][axis], limit)
                smoother['min_velocity'][axis] = max(smoother['min_velocity'][axis], -limit)
        if 'goal_xy_tolerance' in profile:
            checker = controller['goal_checker']
            checker['xy_goal_tolerance'] = min(
                checker['xy_goal_tolerance'], float(profile['goal_xy_tolerance']))
        if geometry.get('require_stopped_goal', False):
            checker = params['controller_server']['ros__parameters']['goal_checker']
            checker.update(plugin='nav2_controller::StoppedGoalChecker', stateful=False,
                           yaw_goal_tolerance=math.pi, trans_stopped_velocity=.05,
                           rot_stopped_velocity=.05)
        with tempfile.NamedTemporaryFile(mode='w', prefix='xxu_robot_nav_', suffix='.yaml', delete=False) as handle:
            yaml.safe_dump(params, handle)
            self.output = handle.name
        return self.output


def nav2_parameters(source_file, use_sim_time, use_fake_frame, bringup_share,
                    *, mode, extra_rewrites=None, robot_geometry_file=None):
    """Resolve base YAML plus the explicitly selected fake-frame profile."""
    contract = interfaces()
    base_frame = PythonExpression([
        "'", contract.frame("fake_navigation_base"), "' if '", use_fake_frame,
        "' == 'true' else '", contract.frame("navigation_base"), "'",
    ])
    rewrites = {"use_sim_time": use_sim_time}
    for key in (
        "bt_navigator.ros__parameters.robot_base_frame",
        "local_costmap.local_costmap.ros__parameters.robot_base_frame",
        "global_costmap.global_costmap.ros__parameters.robot_base_frame",
        "behavior_server.ros__parameters.robot_base_frame",
        "collision_monitor.ros__parameters.base_frame_id",
    ):
        rewrites[key] = base_frame
    rewrites.update(extra_rewrites or {})
    configured = RewrittenYaml(source_file=source_file, root_key="",
                               param_rewrites=rewrites, convert_types=True)
    profile = yaml.safe_load((Path(bringup_share) / "config/nav2_fake_profile.yaml").read_text())
    fake_rewrites = dict(profile["common"])
    fake_rewrites.update(profile[mode])
    fake_rewrites = {key: value if isinstance(value, str) else json.dumps(value)
                     for key, value in fake_rewrites.items()}
    for key in (
        "bt_navigator.ros__parameters.default_nav_to_pose_bt_xml",
        "bt_navigator.ros__parameters.default_nav_through_poses_bt_xml",
    ):
        fake_rewrites[key] = PathJoinSubstitution([bringup_share, "config", fake_rewrites[key]])
    fake_params = RewrittenYaml(source_file=configured, root_key="",
                                param_rewrites=fake_rewrites, convert_types=True)
    selected = PythonExpression([
        "'", fake_params, "' if '", use_fake_frame,
        "' == 'true' else '", configured, "'",
    ])
    if robot_geometry_file is not None:
        selected = RobotGeometryParameters(selected, robot_geometry_file, mode=mode)
    return configured, selected, base_frame
