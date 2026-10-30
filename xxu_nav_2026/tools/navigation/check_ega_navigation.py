"""Run isolated RMUL/EGA navigation and check real stopping error.

Source install/setup.zsh and select unused ROS_DOMAIN_ID=73 plus a unique
GZ_PARTITION starting with xxu_vehicle_check_. Only this process group is stopped.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Path as NavPath
from ament_index_python.packages import get_package_share_directory


def parking_evidence(history, now, sim_time):
    """Require fresh observations of a stationary body, not just zero commands."""
    window = {name: [row for row in history.get(name, []) if now - .75 <= row[0] <= now]
              for name in ('truth', 'odom', 'safe', 'wheels')}
    result = {'passed': False, 'samples': {name: len(rows) for name, rows in window.items()}}
    for rows in window.values():
        if (len(rows) < 3 or now - rows[-1][0] > .5
                or not 0 <= sim_time - rows[-1][1] <= .5
                or rows[-1][1] - rows[0][1] < .25
                or not all(math.isfinite(value) for row in rows for value in row[2])):
            return result
    origin = window['truth'][0][2]
    result['truth_translation_m'] = max(math.dist(row[2][:2], origin[:2]) for row in window['truth'])
    result['truth_rotation_rad'] = max(abs(math.atan2(math.sin(row[2][2] - origin[2]),
                                                    math.cos(row[2][2] - origin[2])))
                                       for row in window['truth'])
    result['odom_translation_mps'] = max(math.hypot(*row[2][:2]) for row in window['odom'])
    result['odom_rotation_rps'] = max(abs(row[2][2]) for row in window['odom'])
    result['passed'] = (
        result['truth_translation_m'] <= .005 and result['truth_rotation_rad'] <= .01
        and result['odom_translation_mps'] <= .02 and result['odom_rotation_rps'] <= .05
        and max(abs(value) for row in window['safe'] for value in row[2]) < 1e-6
        and max(abs(value) for row in window['wheels'] for value in row[2]) < 1e-3)
    return result


def lio_overlay(folder, lio_profile):
    """Expose a private bringup share tree to the launched child only."""
    source = Path(get_package_share_directory('xxu_bringup'))
    prefix = folder / 'lio_overlay'
    share = prefix / 'share/xxu_bringup'
    share.mkdir(parents=True)
    for item in source.iterdir():
        target = share / item.name
        if item.name == 'launch':
            target.mkdir()
            for launch_file in item.iterdir():
                if launch_file.is_file():
                    (target / launch_file.name).symlink_to(launch_file)
        elif item.name != 'models':
            target.symlink_to(item, target_is_directory=item.is_dir())
    models = share / 'models'
    models.mkdir()
    for item in (source / 'models').iterdir():
        target = models / item.name
        if item.name == 'ega_folded':
            target.mkdir()
            for asset in item.iterdir():
                if asset.name != 'lio_profile.yaml':
                    (target / asset.name).symlink_to(asset, target_is_directory=asset.is_dir())
            (target / 'lio_profile.yaml').write_bytes(lio_profile.read_bytes())
        else:
            target.symlink_to(item, target_is_directory=item.is_dir())
    index = prefix / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True)
    (index / 'xxu_bringup').write_text('')
    (prefix / 'lib').mkdir()
    (prefix / 'lib/xxu_bringup').symlink_to(source.parent.parent / 'lib/xxu_bringup',
                                         target_is_directory=True)
    return prefix


def main():
    p = argparse.ArgumentParser()
    p.add_argument('name')
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--tolerance', type=float, default=.12)
    p.add_argument('--params')
    p.add_argument('--spin', type=float)
    p.add_argument('--geometry')
    p.add_argument('--profile', type=Path,
                   help='Opt-in profile copied beside a temporary geometry file; defaults stay unchanged')
    p.add_argument('--lio-profile', type=Path,
                   help='Opt-in LIO overlay exposed to the isolated child only')
    p.add_argument('--goals', default='[[6.59,4.44],[2,2],[3,2],[2,3],[2,2]]')
    args = p.parse_args()
    if Path(args.name).name != args.name:
        p.error('name must be a single directory name')
    if not math.isfinite(args.tolerance) or args.tolerance <= 0:
        p.error('tolerance must be finite and positive')
    goals = json.loads(args.goals)
    if not goals or any(len(xy) != 2 or not all(math.isfinite(float(x)) for x in xy) for xy in goals):
        p.error('goals must contain finite XY pairs')
    folder = args.output_dir.resolve() / args.name
    folder.mkdir(parents=True, exist_ok=False)
    if os.environ.get('ROS_DOMAIN_ID') != '73' or not os.environ.get('GZ_PARTITION', '').startswith('xxu_vehicle_check_'):
        p.error('use isolated ROS_DOMAIN_ID=73 and a unique GZ_PARTITION=xxu_vehicle_check_...')
    if args.profile:
        geometry_source = Path(args.geometry) if args.geometry else Path(
            get_package_share_directory('xxu_bringup')) / 'models/ega_folded/geometry.json'
        geometry_dir = folder / 'candidate_geometry'
        geometry_dir.mkdir()
        (geometry_dir / 'geometry.json').write_bytes(geometry_source.read_bytes())
        (geometry_dir / 'navigation_profile.yaml').write_bytes(args.profile.read_bytes())
        args.geometry = str(geometry_dir / 'geometry.json')
    cmd = ['ros2', 'launch', 'xxu_bringup', 'rmul_navigation.launch.py',
           'gazebo_gui:=false', 'rviz:=false']
    if args.params:
        cmd.append('nav2_params_file:=' + args.params)
    if args.spin is not None:
        cmd.append('gyro_spin_rate:=' + str(args.spin))
    if args.geometry:
        cmd.append('robot_geometry_file:=' + args.geometry)
    report = {'name': args.name, 'command': cmd, 'domain': 73,
              'gz_partition': os.environ['GZ_PARTITION'], 'goals': [],
              'physical_xy_tolerance_m': args.tolerance, 'expected_goal_count': len(goals)}
    if args.profile:
        report['profile_sha256'] = hashlib.sha256(args.profile.read_bytes()).hexdigest()
    child_env = dict(os.environ)
    if args.lio_profile:
        prefix = lio_overlay(folder, args.lio_profile.resolve())
        child_env['AMENT_PREFIX_PATH'] = str(prefix) + os.pathsep + child_env.get('AMENT_PREFIX_PATH', '')
        report['lio_profile_sha256'] = hashlib.sha256(args.lio_profile.read_bytes()).hexdigest()
    log = (folder / 'launch.log').open('w')
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True, env=child_env)
    rclpy.init()
    n = rclpy.create_node('vehicle_regression_observer')
    buf = Buffer()
    listener = TransformListener(buf, n)
    current = {'clock': 0., 'truth': None, 'healthy': False, 'safe': [0., 0., 0.],
               'nav': [0., 0., 0.], 'scan_min': None, 'odom': None,
               'collision': None, 'wheel_max': 0.}
    counts = {}
    parking_history = {name: deque(maxlen=1000) for name in ('truth', 'odom', 'safe', 'wheels')}
    rows = []
    start = time.monotonic()
    def yaw(q):
        return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    def cb(topic, m):
        counts[topic] = counts.get(topic, 0) + 1
        evidence_name = None
        if isinstance(m, Clock):
            current['clock'] = m.clock.sec + m.clock.nanosec / 1e9
        elif isinstance(m, TFMessage):
            for t in m.transforms:
                if t.child_frame_id == 'sentry_bot':
                    a = t.transform.translation
                    current['truth'] = [a.x, a.y]
                    current['truth_yaw'] = yaw(t.transform.rotation)
                    evidence_name, values = 'truth', [a.x, a.y, current['truth_yaw']]
                    stamp = t.header.stamp
        elif isinstance(m, Odometry):
            t = m.twist.twist
            current['odom'] = [t.linear.x, t.linear.y, t.angular.z]
            current['odom_yaw'] = yaw(m.pose.pose.orientation)
            current['odom_stamp'] = m.header.stamp.sec + m.header.stamp.nanosec / 1e9
            evidence_name, values, stamp = 'odom', current['odom'], m.header.stamp
        elif isinstance(m, TwistStamped):
            t = m.twist
            current['safe' if topic.endswith('safe') else 'nav'] = [t.linear.x, t.linear.y, t.angular.z]
            if topic.endswith('safe'):
                evidence_name, values, stamp = 'safe', current['safe'], m.header.stamp
        elif isinstance(m, Bool):
            current['healthy'] = m.data
        elif isinstance(m, CollisionMonitorState):
            current['collision'] = {'polygon': m.polygon_name, 'action': m.action_type}
        elif isinstance(m, JointState):
            current['wheel_max'] = max((abs(v) for k, v in zip(m.name, m.velocity) if 'wheel' in k), default=0.)
            if any('wheel' in name for name in m.name) and m.velocity:
                evidence_name, values, stamp = 'wheels', [current['wheel_max']], m.header.stamp
        elif isinstance(m, LaserScan):
            current['scan_min'] = min((r for r in m.ranges if math.isfinite(r)), default=None)
        elif isinstance(m, NavPath):
            current['plan'] = [[p.pose.position.x,p.pose.position.y] for p in m.poses[::max(1,len(m.poses)//20)]]
        if evidence_name is not None:
            parking_history[evidence_name].append((time.monotonic(), stamp.sec + stamp.nanosec / 1e9, list(values)))
    items = [('/clock', Clock), ('/rm_sim/ground_truth', TFMessage),
             ('/odom', Odometry), ('/scan', LaserScan),
             ('/rm_sim/cmd_vel_safe', TwistStamped), ('/cmd_vel_nav', TwistStamped),
             ('/cmd_vel_watchdog/healthy', Bool), ('/collision_monitor_state', CollisionMonitorState),
             ('/joint_states', JointState), ('/plan', NavPath)]
    subs = [n.create_subscription(typ, topic, lambda m, t=topic: cb(t, m), qos_profile_sensor_data)
            for topic, typ in items]
    action = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=.05)
    def wait(future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            spin(.05)
        if not future.done():
            raise TimeoutError('service/action timeout')
        return future.result()
    def pose():
        try:
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform.translation
            return [t.x, t.y]
        except Exception:
            return None
    def save():
        (folder / 'result.json').write_text(json.dumps(report, indent=2))
    try:
        ready = False
        end = time.monotonic() + 100
        client = n.create_client(GetState, '/bt_navigator/get_state')
        while time.monotonic() < end and proc.poll() is None:
            spin(.5)
            if client.service_is_ready() and action.server_is_ready() and pose():
                state = wait(client.call_async(GetState.Request()), 3)
                if state.current_state.id == 3 and counts.get('/scan', 0) > 3 and current['truth']:
                    ready = True
                    break
        report['ready'] = ready
        if not ready:
            raise RuntimeError('navigation failed to become ready')
        for node, names in [('/local_costmap/local_costmap', ['robot_radius', 'inflation_layer.inflation_radius']),
                            ('/controller_server', ['goal_checker.xy_goal_tolerance', 'FollowPath.vx_max']),
                            ('/small_point_lio', ['laser_point_cov']),
                            ('/fake_vel_transform', ['spin_speed']),
                            ('/collision_monitor', ['EmergencyStop.points'])]:
            c = n.create_client(GetParameters, node + '/get_parameters')
            if not c.wait_for_service(timeout_sec=3):
                raise RuntimeError('missing parameters: ' + node)
            values = wait(c.call_async(GetParameters.Request(names=names)), 3).values
            report.setdefault('parameters', {})[node] = {name: (v.double_value if v.type == 3 else v.string_value) for name, v in zip(names, values)}
        print(args.name, 'ready', json.dumps(report['parameters']), flush=True)
        for xy in goals:
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = map(float, xy)
            goal.pose.pose.orientation.w = 1.
            handle = wait(action.send_goal_async(goal), 5)
            if not handle.accepted:
                raise RuntimeError('goal rejected')
            result = handle.get_result_async()
            goal_start = time.monotonic()
            sim_start = current['clock']
            initial = list(current['truth'])
            last_truth = list(initial)
            last_progress = current['clock']
            g = {'target': xy, 'start_truth': initial, 'emergency_samples': 0,
                 'safe_peak': 0., 'wheel_peak': 0., 'scan_min': None, 'stalled': False}
            next_sample = time.monotonic()
            while not result.done() and time.monotonic() - goal_start < 105:
                spin(.05)
                if time.monotonic() < next_sample:
                    continue
                next_sample += .2
                truth = current['truth']
                if truth and math.dist(truth, last_truth) > .04:
                    last_truth = list(truth)
                    last_progress = current['clock']
                if current['collision'] and current['collision']['polygon'] == 'EmergencyStop':
                    g['emergency_samples'] += 1
                g['safe_peak'] = max(g['safe_peak'], math.hypot(*current['safe'][:2]))
                g['wheel_peak'] = max(g['wheel_peak'], current['wheel_max'])
                v = current['scan_min']
                if v is not None:
                    g['scan_min'] = min(g['scan_min'] or v, v)
                rows.append({'wall_s': time.monotonic() - start, 'goal': xy,
                             **json.loads(json.dumps(current)), 'map_pose': pose()})
                try:
                    rows[-1]['map_fake_yaw'] = yaw(buf.lookup_transform('map','gimbal_yaw_fake',rclpy.time.Time()).transform.rotation)
                except Exception:
                    pass
                if current['clock'] - last_progress > 12:
                    g['stalled'] = True
                    break
                if proc.poll() is not None:
                    raise RuntimeError('launch exited')
            completed = result.done()
            if not completed:
                wait(handle.cancel_goal_async(), 5)
                spin(1.)
            else:
                g['status'] = result.result().status
            g['wall_seconds'] = time.monotonic() - goal_start
            g['simulation_seconds'] = current['clock'] - sim_start
            g['truth_at_result'] = list(current['truth'])
            g['truth_error_m'] = math.dist(current['truth'], xy)
            g['map_at_result'] = pose()
            spin(2.)
            g['settled_truth'] = list(current['truth'])
            g['settled_error_m'] = math.dist(current['truth'], xy)
            g['safe_after_stop'] = list(current['safe'])
            g['odom_after_stop'] = current['odom']
            g['wheel_after_stop'] = current['wheel_max']
            g['parking_evidence'] = parking_evidence(parking_history, time.monotonic(), current['clock'])
            report['goals'].append(g)
            save()
            print(args.name, 'goal', json.dumps(g), flush=True)
            if g.get('status') != 4:
                break
        report['counts'] = counts
        report['real_time_factor'] = current['clock'] / (time.monotonic() - start)
        report['passed'] = len(report['goals']) == len(goals) and all(
            g.get('status') == 4 and not g['stalled']
            and max(g['truth_error_m'], g['settled_error_m']) <= args.tolerance
            and max(map(abs, g['safe_after_stop'])) < 1e-6
            and g['wheel_after_stop'] < 1e-3
            and g['parking_evidence']['passed']
            for g in report['goals'])
    except Exception as e:
        report['error'] = repr(e)
        print(args.name, 'ERROR', repr(e), flush=True)
    finally:
        save()
        (folder / 'samples.json').write_text(json.dumps(rows))
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)
        n.destroy_node()
        rclpy.try_shutdown()
        log.close()
    return 0 if report.get('passed') and not report.get('error') else 1


if __name__ == '__main__':
    raise SystemExit(main())
