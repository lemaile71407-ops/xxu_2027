#!/usr/bin/env python3
"""Short, isolated rosbag player resource probe; not a navigation benchmark."""
import argparse
import csv
import os
from pathlib import Path
import signal
import subprocess
import time

import psutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--domain', type=int, default=191)
    args = parser.parse_args()
    if args.duration <= 0 or not 0 <= args.domain <= 232:
        parser.error('duration must be positive and domain between 0 and 232')
    args.output.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.domain)
    cmd = ['ros2', 'bag', 'play', str(args.bag.resolve()), '--disable-keyboard-controls',
           '--playback-duration', str(args.duration), '--topics',
           '/mid360/livox_points_compensated', '/imu_lio_compensated', '/clock']
    with (args.output / 'player.log').open('w') as log, (args.output / 'process.csv').open('w', newline='') as out:
        writer = csv.writer(out)
        writer.writerow(['elapsed_s', 'pid', 'name', 'cpu_percent_one_core', 'rss_bytes',
                         'pss_bytes', 'uss_bytes', 'threads', 'mem_available_bytes'])
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        root = psutil.Process(proc.pid)
        prior = {}
        start = time.monotonic()
        try:
            while proc.poll() is None:
                now = time.monotonic()
                try:
                    children = root.children(recursive=True)
                except psutil.NoSuchProcess:
                    break
                for p in [root, *children]:
                    try:
                        key = (p.pid, p.create_time())
                        cpu = sum(p.cpu_times()[:2])
                        previous = prior.get(key)
                        pct = 100 * (cpu - previous[1]) / (now - previous[0]) if previous else ''
                        prior[key] = (now, cpu)
                        mem = p.memory_full_info()
                        writer.writerow([round(now - start, 3), p.pid, p.name(),
                                         round(pct, 3) if pct != '' else '', mem.rss,
                                         getattr(mem, 'pss', ''), getattr(mem, 'uss', ''),
                                         p.num_threads(), psutil.virtual_memory().available])
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
                out.flush()
                time.sleep(max(0, 1 - (time.monotonic() - now)))
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
    print(f'player_exit_code={proc.returncode}; output={args.output}')
    return 0 if proc.returncode == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
