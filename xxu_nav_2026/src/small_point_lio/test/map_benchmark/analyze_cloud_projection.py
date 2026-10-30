#!/usr/bin/env python3
"""Compare world XY projections using truth and aligned odom at cloud times.

This measures pose-induced disagreement for the same points, not map thickness
or scan-internal distortion. Uses scipy for quaternion interpolation.
"""
import argparse
import json
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def pose_at(samples, timestamp, max_gap):
    samples = samples[np.argsort(samples[:, 0])]
    _, indices = np.unique(samples[:, 0], return_index=True)
    samples = samples[indices]
    i = np.searchsorted(samples[:, 0], timestamp, side='right')
    if i == 0 or i == len(samples):
        return None
    a, b = samples[i-1], samples[i]
    if b[0] - a[0] > max_gap:
        return None
    f = (timestamp-a[0])/(b[0]-a[0])
    pos = a[1:4]*(1-f) + b[1:4]*f
    rot = Slerp([a[0], b[0]], Rotation.from_quat([a[4:8], b[4:8]]))([timestamp])[0]
    return pos, rot


def analyze(data, alignment):
    yaw = alignment['yaw_rad']
    c, s = np.cos(yaw), np.sin(yaw)
    xy_rotation = np.array([[c, -s], [s, c]])
    translation = np.asarray(alignment['translation_xy_m'])
    distances, frame_rms = [], []
    for idx, timestamp in enumerate(data['stamps']):
        truth = pose_at(data['truth'], timestamp, .05)
        odom = pose_at(data['odom'], timestamp, .15)
        pts = data['points'][data['offsets'][idx]:data['offsets'][idx+1]]
        if truth is None or odom is None or len(pts) == 0:
            continue
        truth_xy = (truth[1].apply(pts) + truth[0])[:, :2]
        odom_xy = (odom[1].apply(pts) + odom[0])[:, :2] @ xy_rotation.T + translation
        d = np.linalg.norm(truth_xy - odom_xy, axis=1)
        distances.append(d)
        frame_rms.append(float(np.sqrt(np.mean(d*d))))
    if not distances:
        raise ValueError('No frames bracketed by valid truth and odometry')
    all_distances = np.concatenate(distances)
    return {'input_frames': len(data['stamps']), 'paired_frames': len(frame_rms),
            'paired_points': len(all_distances), 'xy_point_rmse_m': float(np.sqrt(np.mean(all_distances**2))),
            'xy_point_p95_m': float(np.percentile(all_distances, 95)),
            'frame_rmse_median_m': float(np.median(frame_rms)),
            'frame_rmse_max_m': max(frame_rms),
            'max_truth_gap_s': .05, 'max_odom_gap_s': .15}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cloud', required=True)
    p.add_argument('--alignment', required=True)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    with np.load(args.cloud) as cloud, open(args.alignment) as f:
        result = analyze(cloud, json.load(f)['alignment'])
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)
        f.write('\n')
