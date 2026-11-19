#!/usr/bin/env python3
"""Record live simulation inputs for an offline navigation benchmark."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone

import yaml
from rclpy.executors import ExternalShutdownException


REQUIRED = {
    '/mid360/livox_points_compensated': 'sensor_msgs/msg/PointCloud2',
    '/imu_lio_compensated': 'sensor_msgs/msg/Imu',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
    '/clock': 'rosgraph_msgs/msg/Clock',
}
OPTIONAL = ['/rm_sim/lidar/points', '/mid360/livox_points', '/livox/imu', '/rm_sim/ground_truth',
            '/cmd_vel_nav', '/backend_node/localization_state', '/localization/odometry', '/odom']


def inspect_bag(bag):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosgraph_msgs.msg import Clock

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    stats = {}
    clock_previous = None
    clock_first = None
    clock_resets = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in REQUIRED:
            continue
        item = stats.setdefault(topic, {'count': 0, 'first_ns': stamp, 'last_ns': stamp,
                                        'max_wall_gap_s': 0.0})
        if item['count']:
            item['max_wall_gap_s'] = max(item['max_wall_gap_s'], (stamp - item['last_ns']) / 1e9)
        item['last_ns'] = stamp
        item['count'] += 1
        if topic == '/clock':
            clock = deserialize_message(data, Clock).clock
            value = clock.sec * 1000000000 + clock.nanosec
            if clock_first is None:
                clock_first = value
            if clock_previous is not None and value < clock_previous:
                clock_resets += 1
            clock_previous = value
    for item in stats.values():
        item['span_wall_s'] = (item['last_ns'] - item['first_ns']) / 1e9
    return {'topics': stats, 'clock_backwards_count': clock_resets,
            'clock_span_s': (clock_previous - clock_first) / 1e9 if clock_first is not None else None}


def check_topics():
    result = subprocess.run(['ros2', 'topic', 'list', '-t', '--no-daemon', '--spin-time', '5'], text=True,
                            capture_output=True, timeout=15, check=True)
    lines = set(result.stdout.splitlines())
    return [f'{topic} [{kind}]' for topic, kind in REQUIRED.items()
            if f'{topic} [{kind}]' not in lines]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path,
                        help='new output directory; never overwritten')
    parser.add_argument('--duration', type=float, default=3600,
                        help='recording wall seconds including startup (maximum and default: 3600)')
    parser.add_argument('--min-recorded-span', type=float, default=3600,
                        help='minimum required clock and input span in seconds (default: 3600)')
    parser.add_argument('--min-free-gib', type=float, default=5,
                        help='stop if free disk drops below this many GiB (default: 5)')
    parser.add_argument('--watch-pid', type=int, help='stop if this navigation supervisor exits')
    args = parser.parse_args()
    if args.duration <= 0 or args.min_recorded_span <= 0 or args.min_free_gib <= 0:
        parser.error('duration, min-recorded-span and min-free-gib must be positive')
    if args.duration > 3600:
        parser.error('recording duration cannot exceed one hour')
    args.output = args.output.resolve()
    if args.output.exists():
        parser.error('output already exists')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    missing = check_topics()
    if missing:
        print('Required topics unavailable or incompatible:', *missing, sep='\n  ', file=sys.stderr)
        return 2
    if shutil.disk_usage(args.output.parent).free < args.min_free_gib * 2**30:
        print('Insufficient free disk space', file=sys.stderr)
        return 2
    args.output.mkdir()
    bag = args.output / 'bag'
    manifest = {
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'requested_wall_seconds': args.duration,
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'git_head': subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip(),
        'git_status': subprocess.run(['git', 'status', '--short'], capture_output=True, text=True).stdout.splitlines(),
        'topics': list(REQUIRED) + OPTIONAL,
        'clock_note': 'Bag receive timestamps use wall time; /clock is recorded separately.',
    }
    (args.output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    cmd = ['ros2', 'bag', 'record', '--storage', 'mcap', '--storage-preset-profile', 'fastwrite',
           '--disable-keyboard-controls', '--output', str(bag), '--topics', *manifest['topics']]
    stopped = None
    start = time.monotonic()
    with (args.output / 'recorder.log').open('w') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while time.monotonic() - start < args.duration:
                if proc.poll() is not None:
                    stopped = f'recorder exited early ({proc.returncode})'
                    break
                if args.watch_pid is not None and not Path(f'/proc/{args.watch_pid}').exists():
                    stopped = 'navigation supervisor exited'
                    break
                if shutil.disk_usage(args.output).free < args.min_free_gib * 2**30:
                    stopped = 'free disk threshold reached'
                    break
                time.sleep(min(1, max(0, args.duration - (time.monotonic() - start))))
        except (KeyboardInterrupt, ExternalShutdownException):
            stopped = 'interrupted by user'
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
                stopped = 'recorder did not stop cleanly'
    manifest.update(elapsed_wall_seconds=round(time.monotonic() - start, 3),
                    ended_utc=datetime.now(timezone.utc).isoformat(),
                    recorder_exit=proc.returncode, stopped=stopped)
    metadata = bag / 'metadata.yaml'
    if metadata.exists():
        info = yaml.safe_load(metadata.read_text())['rosbag2_bagfile_information']
        manifest['bag_duration_seconds'] = info['duration']['nanoseconds'] / 1e9
        manifest['message_counts'] = {x['topic_metadata']['name']: x['message_count']
                                      for x in info['topics_with_message_count']}
    missing_data = [topic for topic in REQUIRED if manifest.get('message_counts', {}).get(topic, 0) == 0]
    manifest['missing_required_data'] = missing_data
    if metadata.exists() and not missing_data:
        try:
            manifest['inspection'] = inspect_bag(bag)
        except Exception as exc:
            manifest['inspection_error'] = f'{type(exc).__name__}: {exc}'
    failures = []
    inspection = manifest.get('inspection', {})
    if inspection:
        for topic in ('/mid360/livox_points_compensated', '/imu_lio_compensated', '/clock'):
            if inspection['topics'].get(topic, {}).get('span_wall_s', 0) < args.min_recorded_span:
                failures.append(f'{topic} receive-time span below {args.min_recorded_span}s')
        if inspection['clock_span_s'] is None or inspection['clock_span_s'] < args.min_recorded_span:
            failures.append(f'/clock simulated-time span below {args.min_recorded_span}s')
        if inspection['clock_backwards_count']:
            failures.append('/clock moved backwards')
    manifest['minimum_required_span_seconds'] = args.min_recorded_span
    manifest['verification_failures'] = failures
    manifest['verification_note'] = ('Counts and gaps do not verify sensor content, valid TF, '
                                     'or representative movement; review inspection before benchmark use.')
    (args.output / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    print(f"Output: {args.output}\nBag duration: {manifest.get('bag_duration_seconds', 'unknown')} s"
          f"\nMissing topics: {missing_data}\nVerification failures: {failures}")
    return 1 if (stopped or proc.returncode != 0 or missing_data or failures
                 or 'inspection_error' in manifest or not metadata.exists()) else 0


if __name__ == '__main__':
    raise SystemExit(main())
