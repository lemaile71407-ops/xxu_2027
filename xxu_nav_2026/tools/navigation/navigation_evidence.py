"""Pure acceptance checks shared by navigation runs and offline regressions."""
import math


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
    if not result['passed']:
        result['diagnostic_window'] = {
            name: [{'age_s': now - row[0], 'source_stamp_s': row[1], 'values': row[2]}
                   for row in history.get(name, []) if now - .75 <= row[0] <= now]
            for name in ('truth', 'odom', 'source_odom', 'safe', 'wheels')}
    return result


def stationary_velocity_evidence(history, start, end):
    """Check every raw/adapted odometry sample, not only the final parking slice."""
    result = {'passed': False, 'translation_limit_mps': .02, 'rotation_limit_rps': .05}
    streams_passed = []
    for name in ('source_odom', 'odom'):
        rows = [row for row in history.get(name, []) if start <= row[1] <= end]
        stream = {'passed': False, 'samples': len(rows)}
        result[name] = stream
        if len(rows) < 3 or not all(math.isfinite(value) for row in rows for value in row[2]):
            streams_passed.append(False)
            continue
        times = [row[1] for row in rows]
        gaps = [b - a for a, b in zip(times, times[1:])]
        stream.update(first_source_s=times[0], last_source_s=times[-1],
                      max_source_gap_s=max(gaps),
                      translation_peak_mps=max(math.hypot(*row[2][:2]) for row in rows),
                      rotation_peak_rps=max(abs(row[2][2]) for row in rows))
        stream['passed'] = (0 <= times[0] - start <= .5 and 0 <= end - times[-1] <= .5
                            and min(gaps) > 0 and max(gaps) <= .5
                            and stream['translation_peak_mps'] <= .02
                            and stream['rotation_peak_rps'] <= .05)
        if not stream['passed']:
            stream['exceedances'] = [
                {'source_stamp_s': row[1], 'values': row[2]}
                for row in rows if math.hypot(*row[2][:2]) > .02 or abs(row[2][2]) > .05][:20]
        streams_passed.append(stream['passed'])
    result['passed'] = all(streams_passed)
    return result


def localization_stability_evidence(samples, start, end, translation_limit=.12, rotation_limit=.2):
    """Evaluate a stationary ROS-time window; missing/stale TF cannot count as stable."""
    result = {'passed': False, 'start_ros_s': start, 'end_ros_s': end,
              'required_duration_s': end - start, 'samples': len(samples),
              'translation_limit_m': translation_limit, 'rotation_limit_rad': rotation_limit}
    if len(samples) < 3 or end <= start:
        return result
    if any(row.get('trusted') is not True for row in samples):
        result['reason'] = 'localization not continuously trusted'
        return result
    times = [row['clock'] for row in samples]
    gaps = [times[0] - start, end - times[-1]] + [b - a for a, b in zip(times, times[1:])]
    result['max_sample_gap_s'] = max(gaps)
    if (min(gaps) < -1e-6 or max(gaps) > .5
            or any(row.get('map') is None for row in samples)):
        return result
    values = [value for row in samples for value in [row['clock'], row['map_stamp'], row['truth_stamp'],
                                                    *row['map'], *row['truth']]]
    if not all(math.isfinite(value) for value in values):
        return result
    result['max_tf_age_s'] = max(row['clock'] - row['map_stamp'] for row in samples)
    result['max_truth_age_s'] = max(row['clock'] - row['truth_stamp'] for row in samples)
    if (any(not 0 <= row['clock'] - row['map_stamp'] <= .5 for row in samples)
            or any(not 0 <= row['clock'] - row['truth_stamp'] <= .5 for row in samples)
            or any(b['map_stamp'] < a['map_stamp'] for a, b in zip(samples, samples[1:]))):
        return result
    for name in ('map', 'truth'):
        origin = samples[0][name]
        result[name + '_translation_extent_m'] = max(math.dist(row[name][:2], origin[:2]) for row in samples)
        result[name + '_rotation_extent_rad'] = max(abs(math.remainder(row[name][2] - origin[2], 2 * math.pi))
                                                   for row in samples)
    result['max_map_truth_xy_difference_m'] = max(math.dist(row['map'][:2], row['truth'][:2]) for row in samples)
    result['max_map_truth_yaw_difference_rad'] = max(abs(math.remainder(row['map'][2] - row['truth'][2], 2 * math.pi))
                                                   for row in samples)
    result['passed'] = (result['map_translation_extent_m'] <= translation_limit
                        and result['map_rotation_extent_rad'] <= rotation_limit
                        and result['max_map_truth_xy_difference_m'] <= translation_limit
                        and result['max_map_truth_yaw_difference_rad'] <= rotation_limit
                        and result['truth_translation_extent_m'] <= .005
                        and result['truth_rotation_extent_rad'] <= .01)
    return result
