#!/usr/bin/env python3
"""Explicitly build recorded diagnostics without changing backend CMake targets.

Requires an existing backend build with backend_relocalization_test configured.
An optional solver object selects an isolated candidate; omitting it uses the
existing backend core. Failures remain failures, including accepted differences.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-dir', type=Path, required=True)
    parser.add_argument('--solver-object', type=Path)
    parser.add_argument('--experimental', action='store_true',
                        help='Explicitly select the integrated blended_adaptive candidate')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    build = args.build_dir.resolve()
    folder = Path(__file__).resolve().parent
    source = folder / 'registration_test.cpp'
    flags_file = build / 'CMakeFiles/backend_core.dir/flags.make'
    link_file = build / 'test/CMakeFiles/backend_relocalization_test.dir/link.txt'
    inputs = [source, flags_file, link_file, build / 'libbackend_core.a',
              *sorted((folder / 'data').iterdir())]
    if args.solver_object:
        args.solver_object = args.solver_object.resolve()
        inputs.append(args.solver_object)
    for path in inputs:
        if not path.is_file():
            parser.error(f'missing input: {path}')

    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in inputs}
    flags = {}
    for line in flags_file.read_text().splitlines():
        if ' = ' in line:
            name, value = line.split(' = ', 1)
            flags[name] = shlex.split(value)
    recipe = shlex.split(link_file.read_text())
    obj = output / 'registration_test.o'
    subprocess.run([recipe[0], *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'],
                    *flags['CXX_FLAGS'], '-c', str(source), '-o', str(obj)],
                   cwd=build, check=True)
    link = [arg for arg in recipe if not arg.endswith('.cpp.o')]
    objects = [str(obj)]
    if args.solver_object:
        objects.insert(0, str(args.solver_object))
    link[1:1] = objects
    binary = output / 'registration_diagnostics'
    link[link.index('-o') + 1] = str(binary)
    subprocess.run(link, cwd=build / 'test', check=True)
    with (output / 'tests.log').open('w') as log:
        result = subprocess.run([str(binary), '--gtest_output=xml:' + str(output / 'tests.xml')],
                                stdout=log, stderr=subprocess.STDOUT,
                                env=os.environ | {'XXU_REGISTRATION_EXPERIMENTAL': '1' if args.experimental else '0'})
    unchanged = all(hashlib.sha256(path.read_bytes()).hexdigest() == hashes[str(path)]
                    for path in inputs)
    report = {'returncode': result.returncode, 'inputs_unchanged': unchanged,
              'experimental_blended_adaptive': args.experimental,
              'solver_object': str(args.solver_object) if args.solver_object else None,
              'input_sha256': hashes}
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report | {'input_sha256': 'see result.json'}))
    return result.returncode if unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
