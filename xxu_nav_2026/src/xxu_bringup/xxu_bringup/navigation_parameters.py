"""Nav2 parameter construction and vehicle navigation geometry."""
import json
import copy
import math
from pathlib import Path
import tempfile

from launch import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from nav2_common.launch import RewrittenYaml
import yaml

from .interfaces import interfaces


def continuous_spin_parameters(source, *, rate=0.5):
    """Explicit Omni-only trial; retain collision, freshness and stopping limits."""
    if not math.isfinite(rate) or not 0 < abs(rate) <= 0.5:
        raise ValueError('Continuous spin trial requires 0 < abs(rate) <= 0.5 rad/s')
    params = copy.deepcopy(source)
    controller = params['controller_server']['ros__parameters']
    follow = controller['FollowPath']
    if follow['motion_model'] != 'Omni':
        raise ValueError('Continuous spin requires MPPI Omni')
    if 'ContinuousSpinCritic' not in follow['critics']:
        follow['critics'].append('ContinuousSpinCritic')
    follow['ContinuousSpinCritic'] = {
        'enabled': True, 'target_angular_velocity': rate, 'cost_weight': 64.0,
        'release_distance': 0.65, 'transition_distance': 0.35}
    follow['PathAngleCritic']['enabled'] = False
    follow['TwirlingCritic']['enabled'] = False
    follow['wz_max'] = abs(rate)
    follow['visualize'] = True
    smoother = params['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][2] = abs(rate)
    smoother['min_velocity'][2] = -abs(rate)
    return params


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
            if key not in {'minimum_inflation_radius', 'translation_velocity_limit',
                           'goal_xy_tolerance', 'goal_yaw_tolerance'}:
                raise ValueError(f'Unknown vehicle navigation setting: {key}')
            if not math.isfinite(float(value)) or float(value) <= 0:
                raise ValueError(f'Invalid vehicle navigation setting: {key}')
            if key == 'goal_yaw_tolerance' and float(value) > math.pi:
                raise ValueError('Invalid vehicle navigation setting: goal_yaw_tolerance')
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
        if 'goal_yaw_tolerance' in profile:
            checker = controller['goal_checker']
            checker['yaw_goal_tolerance'] = min(
                checker['yaw_goal_tolerance'], float(profile['goal_yaw_tolerance']))
        if geometry.get('require_stopped_goal', False):
            checker = params['controller_server']['ros__parameters']['goal_checker']
            yaw_tolerance = (checker['yaw_goal_tolerance']
                             if 'goal_yaw_tolerance' in profile else math.pi)
            checker.update(plugin='nav2_controller::StoppedGoalChecker', stateful=False,
                           yaw_goal_tolerance=yaw_tolerance, trans_stopped_velocity=.05,
                           rot_stopped_velocity=.05)
        with tempfile.NamedTemporaryFile(mode='w', prefix='xxu_robot_nav_', suffix='.yaml', delete=False) as handle:
            yaml.safe_dump(params, handle)
            self.output = handle.name
        return self.output


def nav2_parameters(source_file, use_sim_time, bringup_share=None,
                    *, mode="navigation", extra_rewrites=None, robot_geometry_file=None):
    """Resolve Nav2 parameters for the standard robot base frame."""
    del bringup_share
    contract = interfaces()
    base_frame = contract.frame("navigation_base")
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
    selected = configured
    if robot_geometry_file is not None:
        selected = RobotGeometryParameters(selected, robot_geometry_file, mode=mode)
    return configured, selected, base_frame
