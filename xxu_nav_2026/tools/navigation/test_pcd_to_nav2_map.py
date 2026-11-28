#!/usr/bin/env python3
"""Offline tests for PCD decoding and Nav2 occupancy map export."""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

import pcd_to_nav2_map as converter


def pcd_header(encoding, points, fields='x y z', sizes='8 8 8', types='F F F', counts='1 1 1'):
    count_line = f'COUNT {counts}\n' if counts is not None else ''
    # VIEWPOINT is deliberately nonidentity. The samples are already in map coordinates.
    return (f'# test cloud\nVERSION .7\nFIELDS {fields}\nSIZE {sizes}\nTYPE {types}\n'
            f'{count_line}WIDTH {points}\nHEIGHT 1\nVIEWPOINT 10 20 30 0 0 0 1\n'
            f'POINTS {points}\nDATA {encoding}\n').encode('ascii')


def read_pgm(path):
    with path.open('rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('Expected a P5 PGM image')
        width, height = map(int, stream.readline().split())
        if stream.readline().strip() != b'255':
            raise ValueError('Expected an 8-bit PGM image')
        return np.frombuffer(stream.read(), dtype=np.uint8).reshape(height, width)


class PcdToNav2MapTest(unittest.TestCase):
    def setUp(self):
        temporary_dir = tempfile.TemporaryDirectory()
        self.addCleanup(temporary_dir.cleanup)
        self.directory = Path(temporary_dir.name)
        self.pcd_path = self.directory / 'map.pcd'

    def write_xyz(self, points):
        self.pcd_path.write_bytes(pcd_header('ascii', len(points)))
        if points:
            with self.pcd_path.open('ab') as stream:
                np.savetxt(stream, points)

    def run_cli(self, output, *options, pcd_path=None):
        if pcd_path is None:
            pcd_path = self.pcd_path
        with redirect_stdout(io.StringIO()):
            converter.main(['--pcd', str(pcd_path), '--output', str(output), *options])

    def assert_cli_error(self, options, message, *, required=True):
        output = self.directory / 'rejected'
        args = ['--pcd', str(self.pcd_path), '--output', str(output)] if required else []
        errors = io.StringIO()
        with redirect_stderr(errors), self.assertRaises(SystemExit) as caught:
            converter.main(args + options)
        self.assertEqual(caught.exception.code, 2)
        self.assertIn(message, errors.getvalue())
        self.assertFalse(output.with_suffix('.pgm').exists())
        self.assertFalse(output.with_suffix('.yaml').exists())

    def test_ascii_named_fields_and_default_count(self):
        header = pcd_header('ascii', 2, 'z label x y', '4 2 8 4', 'F U F F', counts=None)
        self.pcd_path.write_bytes(header + b'0.25 42 -1.0000000005 2.5\n-0.5 4 3.5 -2\n')
        np.testing.assert_array_equal(converter.read_pcd(self.pcd_path),
                                      [[-1.0000000005, 2.5, 0.25], [3.5, -2, -0.5]])

    def test_binary_interleaved_multi_value_field_and_padding(self):
        header = pcd_header('binary', 2, 'z normal x _ y', '4 4 8 1 4',
                            'F F F U F', '1 3 1 2 1')
        row = struct.Struct('<4fd2xf')
        records = row.pack(0.25, 0, 0, 1, -1.0000000005, 2.5)
        records += row.pack(-0.5, 1, 0, 0, 3.5, -2)
        self.pcd_path.write_bytes(header + records)
        np.testing.assert_array_equal(converter.read_pcd(self.pcd_path),
                                      [[-1.0000000005, 2.5, 0.25], [3.5, -2, -0.5]])

    def test_binary_compressed_field_planes_and_lzf_back_reference(self):
        header = pcd_header('binary_compressed', 2, 'z normal x y', '4 4 8 4',
                            'F F F F', '1 3 1 1')
        z_plane = struct.pack('<2f', 0.25, -0.5)
        xy_planes = struct.pack('<2d2f', -1.0000000005, 3.5, 2.5, -2)
        # The normal plane is a zero literal followed by an overlapping 23-byte match.
        compressed = b'\x07' + z_plane + b'\x00\x00\xe0\x0e\x00' + b'\x17' + xy_planes
        self.pcd_path.write_bytes(header + struct.pack('<II', len(compressed), 56) + compressed)
        np.testing.assert_array_equal(converter.read_pcd(self.pcd_path),
                                      [[-1.0000000005, 2.5, 0.25], [3.5, -2, -0.5]])

    def test_rejects_truncated_binary_and_invalid_lzf(self):
        self.pcd_path.write_bytes(pcd_header('binary', 1) + b'\x00' * 23)
        with self.assertRaisesRegex(ValueError, 'Binary PCD payload size'):
            converter.read_pcd(self.pcd_path)
        self.pcd_path.write_bytes(pcd_header('binary_compressed', 1)
                                  + struct.pack('<II', 2, 24) + b'\x20\x00')
        with self.assertRaisesRegex(ValueError, 'Invalid LZF back-reference'):
            converter.read_pcd(self.pcd_path)

    def test_height_band_ground_offset_and_point_threshold(self):
        self.write_xyz([
            (0, 0, 1.0), (3, 2, 2.0),
            (0.25, 0.25, 1.10), (0.50, 0.25, 1.50),
            (1.25, 0.25, 1.09), (1.50, 0.25, 1.25), (1.75, 0.25, 1.51),
            (2.25, 0.25, 0.25), (2.50, 0.25, 0.40),
        ])
        output = self.directory / 'band'
        self.run_cli(output, '--ground-z', '1', '--resolution', '1', '--padding', '0',
                     '--min-points-per-cell', '2')
        pixels = read_pgm(output.with_suffix('.pgm'))
        self.assertEqual(pixels.shape, (3, 4))
        np.testing.assert_array_equal(np.argwhere(pixels == 0), [[2, 0]])
        np.testing.assert_array_equal(pixels[2], [0, 254, 254, 254])

    def test_pixel_alignment_yaml_and_output_name(self):
        self.write_xyz([(-2.25, -1.75, 0.0), (0.25, 0.25, 0.0),
                        (-1.75, 0.25, 0.20), (0.0, -1.5, 0.30)])
        output = self.directory / 'nested' / 'map.v1'
        for suffix in ('', '.yaml', '.pgm'):
            with self.subTest(suffix=suffix):
                self.run_cli(Path(str(output) + suffix), '--resolution', '0.5', '--padding', '1')
                pgm = Path(str(output) + '.pgm')
                metadata = yaml.safe_load(Path(str(output) + '.yaml').read_text(encoding='utf-8'))
                self.assertEqual(metadata, {
                    'image': pgm.name, 'mode': 'trinary', 'resolution': 0.5,
                    'origin': [-3.25, -2.75, 0.0], 'negate': 0,
                    'occupied_thresh': 0.65, 'free_thresh': 0.196,
                })
                pixels = read_pgm(pgm)
                self.assertEqual(pixels.shape, (8, 9))
                np.testing.assert_array_equal(np.argwhere(pixels == 0), [[1, 3], [5, 6]])
                self.assertEqual(pixels[4, 4], 254)
                self.assertEqual(pixels[0, 4], 205)
                self.assertEqual(pixels[4, 0], 205)

    def test_keyframes_clear_locally_and_preserve_occupied_cells(self):
        self.write_xyz([(0, 0, 0), (3, 3, 0), (0.375, 0.125, 0.25), (2.625, 2.625, 0.25)])
        keyframes = self.directory / 'keyframes.txt'
        keyframes.write_text('# backend_map 1\n# index timestamp tx ty tz qx qy qz qw\n'
                             '0 120 0.125 0.125 0 0 0 0 1\n'
                             '1 121 -0.625 0.125 0 0 0 0 1\n', encoding='utf-8')
        output = self.directory / 'keyframed'
        self.run_cli(output, '--resolution', '0.25', '--padding', '0.5', '--keyframes', str(keyframes))
        pixels = read_pgm(output.with_suffix('.pgm'))
        metadata = yaml.safe_load(output.with_suffix('.yaml').read_text(encoding='utf-8'))
        origin_x, origin_y, _ = metadata['origin']
        resolution = metadata['resolution']

        def pixel(x, y):
            col = math.floor((x - origin_x) / resolution)
            row = pixels.shape[0] - 1 - math.floor((y - origin_y) / resolution)
            return pixels[row, col]

        self.assertEqual(pixel(-0.625, 0.125), 254)
        self.assertEqual(pixel(0.125, 0.125), 254)
        self.assertEqual(pixel(0.125, 0.375), 254)
        self.assertEqual(pixel(0.375, 0.125), 0)
        self.assertEqual(pixel(2.625, 2.625), 0)
        self.assertEqual(pixel(1.625, 1.625), 205)
        self.assertEqual(pixel(3.125, 3.125), 205)

    def test_p5_fallback_and_no_obstacle_returns(self):
        self.write_xyz([(0, 0, 0)])
        output = self.directory / 'fallback'
        with patch.object(converter, 'cv2', None):
            self.run_cli(output, '--padding', '0')
        self.assertEqual(output.with_suffix('.pgm').read_bytes(), b'P5\n1 1\n255\n\xfe')

    def test_invalid_cli_arguments(self):
        self.assert_cli_error([], 'required', required=False)
        for options, message in (
            (['--resolution', '0'], '--resolution'),
            (['--resolution', 'nan'], '--resolution'),
            (['--padding', '-1'], '--padding'),
            (['--ground-tolerance', '-0.1'], '--ground-tolerance'),
            (['--ground-tolerance', 'nan'], '--ground-tolerance'),
            (['--ground-clear-radius', '-0.1'], '--ground-clear-radius'),
            (['--ground-clear-radius', 'inf'], '--ground-clear-radius'),
            (['--min-z', '0.6'], '--min-z'),
            (['--ground-z', 'inf'], 'Height limits'),
            (['--min-points-per-cell', '0'], '--min-points-per-cell'),
        ):
            with self.subTest(options=options):
                self.assert_cli_error(options, message)

    def test_fit_ground_plane_crops_tilted_tunnels_symmetrically(self):
        # Tilted floor z = 0.01*x - 0.01*y - 0.10 across a 10m arena, plus two
        # 0.25m-high tunnel roofs on opposite ends that would diverge under constant ground-z.
        xs, ys = np.meshgrid(np.linspace(0.0, 10.0, 21), np.linspace(0.0, 8.0, 17))
        floor_pts = [(float(x), float(y), 0.01 * x - 0.01 * y - 0.10)
                     for x, y in zip(xs.ravel(), ys.ravel())]
        # Tunnel 1 at (1.5, 7.5): floor z = -0.160, roof at +0.25 -> z = +0.090
        # Tunnel 2 at (8.5, 0.5): floor z = -0.020, roof at +0.25 -> z = +0.230
        roof1 = (1.5, 7.5, 0.01 * 1.5 - 0.01 * 7.5 - 0.10 + 0.25)
        roof2 = (8.5, 0.5, 0.01 * 8.5 - 0.01 * 0.5 - 0.10 + 0.25)
        wall = (5.0, 4.0, 0.01 * 5.0 - 0.01 * 4.0 - 0.10 + 0.18)
        self.write_xyz(floor_pts + [roof1, roof2, wall])

        def cell_value(yaml_path, x, y):
            meta = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
            pixels = read_pgm(yaml_path.with_suffix('.pgm'))
            ox, oy, _ = meta['origin']
            res = meta['resolution']
            col = math.floor((x - ox) / res)
            row = pixels.shape[0] - 1 - math.floor((y - oy) / res)
            return int(pixels[row, col])

        std_yaml = self.directory / 'std.yaml'
        self.run_cli(std_yaml, '--fit-ground-plane', '--min-z', '0.10', '--max-z', '0.34', '--resolution', '0.5')
        self.assertEqual(cell_value(std_yaml, 1.5, 7.5), 0)
        self.assertEqual(cell_value(std_yaml, 8.5, 0.5), 0)
        self.assertEqual(cell_value(std_yaml, 5.0, 4.0), 0)

        clr_yaml = self.directory / 'clr.yaml'
        self.run_cli(clr_yaml, '--fit-ground-plane', '--min-z', '0.10', '--max-z', '0.22', '--resolution', '0.5')
        self.assertEqual(cell_value(clr_yaml, 1.5, 7.5), 254)
        self.assertEqual(cell_value(clr_yaml, 8.5, 0.5), 254)
        self.assertEqual(cell_value(clr_yaml, 5.0, 4.0), 0)

    def test_free_from_ground_clears_observed_floor_without_clearing_obstacle_interior(self):
        # Floor points at (0.25, 0.25) and (1.75, 0.25); obstacle wall at (3.25, 0.25);
        # high/unvisited interior around (4.75, 4.75) has no ground points (z=0.80 > max_z).
        self.write_xyz([
            (0.25, 0.25, 0.0),
            (1.75, 0.25, 0.01),
            (3.25, 0.25, 0.25),
            (4.75, 4.75, 0.80),
        ])
        out = self.directory / 'ground_free.yaml'
        self.run_cli(out, '--free-from-ground', '--ground-tolerance', '0.05',
                     '--ground-clear-radius', '0.15', '--resolution', '0.5', '--padding', '0.5')
        meta = yaml.safe_load(out.read_text(encoding='utf-8'))
        pixels = read_pgm(out.with_suffix('.pgm'))
        ox, oy, _ = meta['origin']
        res = meta['resolution']

        def pixel(x, y):
            return int(pixels[pixels.shape[0] - 1 - math.floor((y - oy) / res),
                              math.floor((x - ox) / res)])

        self.assertEqual(pixel(0.25, 0.25), 254)
        self.assertEqual(pixel(1.75, 0.25), 254)
        self.assertEqual(pixel(3.25, 0.25), 0)
        self.assertEqual(pixel(4.75, 4.75), 205)

    def test_empty_and_nonfinite_point_clouds(self):
        self.write_xyz([])
        self.assert_cli_error([], 'point cloud is empty')
        self.write_xyz([(np.nan, 0, 0), (0, np.inf, 0)])
        self.assert_cli_error([], 'no finite XYZ points')

    def test_real_rotating_map(self):
        pcd = Path(__file__).resolve().parents[2] / 'maps' / 'rmul_rotating_20261112' / 'map.pcd'
        if not pcd.is_file():
            self.skipTest('Repository smoke-test PCD is absent')
        output = self.directory / 'rotating.yaml'
        self.run_cli(output, pcd_path=pcd)
        metadata = yaml.safe_load(output.read_text(encoding='utf-8'))
        self.assertEqual(metadata['image'], 'rotating.pgm')
        pixels = read_pgm(self.directory / metadata['image'])
        self.assertEqual(set(np.unique(pixels)), {0, 205, 254})
        self.assertEqual(metadata['resolution'], 0.05)
        self.assertEqual(metadata['origin'][2], 0.0)


if __name__ == '__main__':
    unittest.main()
