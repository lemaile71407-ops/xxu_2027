#!/usr/bin/env python3
"""Repeat both yaw routes with fresh simulations; every attempt must pass.

Run from the ROS workspace after sourcing install/setup.bash. Explicit inputs
are mandatory; this runner does not select a different map, backend or profile.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import uuid

import yaml


def apply_sample_noise_profile(config, profile):
    """Override only declared IMU sample variances; keep the capture config intact."""
    names = {'gyro_variance', 'accel_variance'}
    if not isinstance(profile, dict) or set(profile) != names:
        raise ValueError('noise profile must contain exactly gyro_variance and accel_variance')
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) or value <= 0 for value in profile.values()):
        raise ValueError('sample variances must be finite positive numbers in SI units')
    if not isinstance(config, dict) or not isinstance(config.get('filter'), dict):
        raise ValueError('frontend config must contain a filter mapping')
    if not names <= set(config['filter']):
        raise ValueError('frontend config must already declare both sample variances')
    return {**config, 'filter': {**config['filter'], **profile}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--backend-executable', type=Path, required=True,
                        help='Expected backend binary; checked against the package selected by ROS')
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--observe-seconds', type=float, default=10.)
    parser.add_argument('--frontend-noise-profile', type=Path,
                        help='Explicit simulation IMU sample-variance profile; preserves the source config')
    parser.add_argument('--launch-arg', action='append', default=[])
    args = parser.parse_args()
    if args.repeats < 2 or not math.isfinite(args.observe_seconds) or args.observe_seconds < 10:
        parser.error('require at least two repetitions and ten ROS seconds of observation')
    from ament_index_python.packages import get_package_prefix
    selected = Path(get_package_prefix('backend')) / 'lib/backend/backend_node'
    if selected.resolve() != args.backend_executable.resolve():
        parser.error(f'ROS selects {selected}, not the explicitly requested backend')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    inputs = [Path(__file__), Path(__file__).with_name('check_ega_navigation.py'),
              Path(__file__).with_name('navigation_evidence.py'), args.profile, selected]
    inputs.append(Path(get_package_prefix('frontend')) / 'lib/frontend/zlio_node')
    if args.frontend_noise_profile:
        indices = [i for i, arg in enumerate(args.launch_arg) if arg.partition(':=')[0] == 'frontend_config']
        if len(indices) != 1:
            parser.error('noise profile requires exactly one frontend_config launch argument')
        original = Path(args.launch_arg[indices[0]].partition(':=')[2])
        try:
            config = apply_sample_noise_profile(yaml.safe_load(original.read_text()),
                                                yaml.safe_load(args.frontend_noise_profile.read_text()))
        except (OSError, ValueError, yaml.YAMLError) as error:
            parser.error(str(error))
        generated = output / 'frontend.yaml'
        generated.write_text(yaml.safe_dump(config, sort_keys=False))
        inputs.extend([original, args.frontend_noise_profile])
        args.launch_arg[indices[0]] = 'frontend_config:=' + str(generated)
    for argument in args.launch_arg:
        value = argument.partition(':=')[2]
        path = Path(value)
        if path.is_file():
            inputs.append(path)
            if argument.partition(':=')[0] == 'external_map':
                image = Path(yaml.safe_load(path.read_text())['image'])
                inputs.append(image if image.is_absolute() else path.parent / image)
        elif path.is_dir():
            inputs.extend(p for p in path.rglob('*') if p.is_file())
    hashes = {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
    report = {'passed': False, 'repeats': args.repeats, 'observe_ros_seconds': args.observe_seconds,
              'input_sha256': hashes, 'attempts': []}
    summary = output / 'summary.json'
    summary.write_text(json.dumps(report, indent=2) + '\n')
    for repetition in range(1, args.repeats + 1):
        for route, poses in [('direct180', [[2, 2, math.pi]]),
                             ('via90', [[2, 2, math.pi / 2], [2, 2, math.pi]])]:
            name = f'{route}_run{repetition:02}'
            environment = dict(os.environ, ROS_DOMAIN_ID='73',
                               GZ_PARTITION='xxu_vehicle_check_yaw_' + uuid.uuid4().hex)
            command = [sys.executable, str(Path(__file__).with_name('check_ega_navigation.py')),
                       name, '--output-dir', str(output), '--external-localization',
                       '--estimator-node', '/zlio', '--estimator-parameter', 'config_file',
                       '--profile', str(args.profile.resolve()), '--poses', json.dumps(poses),
                       '--settle-seconds', '2', '--localization-stability-seconds', str(args.observe_seconds)]
            for argument in args.launch_arg:
                command.extend(['--launch-arg', argument])
            with (output / f'{name}.console.log').open('w') as console:
                run = subprocess.run(command, env=environment, stdout=console, stderr=subprocess.STDOUT)
            result_path = output / name / 'result.json'
            result = json.loads(result_path.read_text()) if result_path.exists() else {}
            unchanged = all(path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest
                            for name_, digest in hashes.items() for path in [Path(name_)])
            attempt = {'name': name, 'returncode': run.returncode, 'result': str(result_path),
                       'passed': run.returncode == 0 and result.get('passed') is True and unchanged
                                 and not result.get('external_localization_fault') and not result.get('error'),
                       'inputs_unchanged': unchanged, 'fault': result.get('external_localization_fault'),
                       'error': result.get('error')}
            report['attempts'].append(attempt)
            summary.write_text(json.dumps(report, indent=2) + '\n')
            print(json.dumps(attempt), flush=True)
            if not unchanged:
                return 1
    report['passed'] = all(attempt['passed'] for attempt in report['attempts'])
    summary.write_text(json.dumps(report, indent=2) + '\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
