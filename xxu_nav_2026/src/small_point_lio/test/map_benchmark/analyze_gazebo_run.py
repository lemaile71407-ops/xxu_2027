#!/usr/bin/env python3
"""Analyze one recorded LIO/Gazebo-truth run without fitting a trajectory.

Inputs are a timestamped samples.csv from the read-only observer and the
navigation checker result.json. Only the final two stationary seconds fix the
world/odom yaw and translation; truth is interpolated to each unique odom stamp.
"""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path
import statistics


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values, q):
    values = sorted(values)
    if not values:
        return None
    x = (len(values) - 1) * q
    lo, hi = math.floor(x), math.ceil(x)
    return values[lo] + (values[hi] - values[lo]) * (x - lo)


def analyze(samples_path, result_path):
    with samples_path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    result = json.loads(result_path.read_text())
    truth, odom, resources = {}, {}, []
    first_moving_sim = None
    for row in rows:
        if row['phase'] == 'moving' and first_moving_sim is None:
            first_moving_sim = float(row['sim_s'])
        if row['truth_stamp']:
            t = float(row['truth_stamp'])
            truth[t] = (float(row['truth_x']), float(row['truth_y']), float(row['truth_yaw']))
        if row['odom_stamp']:
            t = float(row['odom_stamp'])
            odom[t] = (float(row['odom_x']), float(row['odom_y']), float(row['odom_yaw']),
                       float(row['odom_vx']), float(row['odom_vy']), float(row['odom_wz']), row['phase'])
        if row['cpu_seconds'] and row['rss_bytes']:
            resources.append((float(row['wall_s']), float(row['cpu_seconds']), int(row['rss_bytes'])))
    if first_moving_sim is None:
        raise ValueError('No movement phase was recorded')
    stamps = sorted(truth)
    if len(stamps) < 3:
        raise ValueError('Not enough truth samples')

    def at_time(t):
        index = bisect.bisect_left(stamps, t)
        if index < len(stamps) and stamps[index] == t:
            return truth[t]
        if index == 0 or index == len(stamps):
            return None
        a, b = stamps[index - 1], stamps[index]
        if b - a > .5:
            return None
        f = (t - a) / (b - a)
        x, y, angle = truth[a]
        X, Y, target_angle = truth[b]
        return (x + f*(X-x), y + f*(Y-y), wrap(angle + f*wrap(target_angle-angle)))

    pairs = []
    for stamp, state in sorted(odom.items()):
        target = at_time(stamp)
        if target is not None:
            pairs.append((stamp, state, target))
    warm = [p for p in pairs if p[1][6] == 'warmup' and first_moving_sim - 8 <= p[0] < first_moving_sim]
    alignment = [p for p in warm if p[0] >= first_moving_sim - 2]
    moving = [p for p in pairs if p[1][6] == 'moving']
    if len(warm) < 3 or len(alignment) < 3:
        raise ValueError('Insufficient timestamp-aligned stationary observations')
    heading = [wrap(p[2][2]-p[1][2]) for p in alignment]
    theta = math.atan2(sum(math.sin(x) for x in heading), sum(math.cos(x) for x in heading))
    c, s = math.cos(theta), math.sin(theta)
    tx = statistics.median(p[2][0] - (c*p[1][0] - s*p[1][1]) for p in alignment)
    ty = statistics.median(p[2][1] - (s*p[1][0] + c*p[1][1]) for p in alignment)
    errors, heading_errors = [], []
    for _, state, target in moving:
        x, y, angle = state[:3]
        errors.append(math.hypot(c*x-s*y+tx-target[0], s*x+c*y+ty-target[1]))
        heading_errors.append(abs(wrap(angle+theta-target[2])))
    def rms(values):
        return math.sqrt(statistics.mean(x*x for x in values)) if values else None
    truth_origin, odom_origin = warm[0][2][:2], warm[0][1][:2]
    truth_warmup = [truth[t] for t in stamps if first_moving_sim-8 <= t < first_moving_sim]
    truth_yaw_origin = truth_warmup[0][2] if truth_warmup else warm[0][2][2]
    truth_speed, truth_yaw_rate = [], []
    moving_truth = [(t, truth[t]) for t in stamps if t >= first_moving_sim]
    for (ta,a),(tb,b) in zip(moving_truth,moving_truth[1:]):
        dt=tb-ta
        if 0.05 <= dt <= .5:
            truth_speed.append(math.hypot(b[0]-a[0],b[1]-a[1])/dt)
            truth_yaw_rate.append(abs(wrap(b[2]-a[2]))/dt)
    cpu = None
    if len(resources) >= 2 and resources[-1][0] > resources[0][0]:
        cpu = 100*(resources[-1][1]-resources[0][1])/(resources[-1][0]-resources[0][0])
    return {
        'ready': result.get('ready'), 'navigation_passed': result.get('passed', False),
        'error': result.get('error'), 'goals': [
            {'target':g['target'],'status':g.get('status'),'stalled':g.get('stalled'),
             'truth_error_m':g['truth_error_m'],'settled_error_m':g['settled_error_m'],
             'parking_passed':g['parking_evidence']['passed'],
             'parking_odom_translation_mps':g['parking_evidence'].get('odom_translation_mps'),
             'parking_odom_rotation_rps':g['parking_evidence'].get('odom_rotation_rps')}
            for g in result.get('goals', [])],
        'sample_counts': {'recorded':len(rows),'unique_truth':len(truth),'unique_odom':len(odom),
                          'warm_paired':len(warm),'moving_paired':len(moving),'resource':len(resources)},
        'warmup': {'observed_sim_seconds':warm[-1][0]-warm[0][0],
                   'truth_max_translation_m':max(math.dist(p[2][:2],truth_origin) for p in warm),
                   'truth_max_yaw_change_rad':max(abs(wrap(p[2]-truth_yaw_origin)) for p in truth_warmup),
                   'odom_max_translation_m':max(math.dist(p[1][:2],odom_origin) for p in warm),
                   'odom_velocity_xy_rms_mps':rms([math.hypot(p[1][3],p[1][4]) for p in warm]),
                   'odom_angular_rms_radps':rms([p[1][5] for p in warm])},
        'moving': {'xy_rmse_m':rms(errors),'xy_p95_m':percentile(errors,.95),
                   'yaw_abs_p95_rad':percentile(heading_errors,.95),
                   'truth_speed_peak_mps':max(truth_speed) if truth_speed else None,
                   'truth_yaw_rate_peak_radps':max(truth_yaw_rate) if truth_yaw_rate else None},
        'resources': {'mean_process_cpu_percent_one_core':cpu,
                      'sampled_peak_process_rss_bytes':max((p[2] for p in resources),default=None)},
        'alignment': {'method':'yaw and translation from median stationary last 2s; truth interpolated to odom timestamps, max truth gap 0.5s',
                      'yaw_rad':theta,'translation_xy_m':[tx,ty]},
        'timing': {'real_time_factor':result.get('real_time_factor'),'warmup_sim_seconds':result.get('warmup',{}).get('sim_seconds')},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=Path, required=True)
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.write_text(json.dumps(analyze(args.samples, args.result), indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
