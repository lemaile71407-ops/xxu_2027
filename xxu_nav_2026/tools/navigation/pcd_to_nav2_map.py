#!/usr/bin/env python3
"""Export a Nav2 occupancy map from a ZLIO/TDT point cloud in map coordinates."""

import argparse
import math
from pathlib import Path
import struct

import numpy as np
import yaml

try:
    import cv2
except ImportError:
    cv2 = None


XYZ_FIELDS = ('x', 'y', 'z')
UNKNOWN = 205
FREE = 254
OCCUPIED = 0
KEYFRAME_CLEAR_RADIUS_M = 0.5


def _read_pcd_header(stream):
    header = {}
    for line in stream:
        tokens = line.decode('ascii').partition('#')[0].split()
        if not tokens:
            continue
        key, *values = tokens
        if key in header:
            raise ValueError(f'Duplicate PCD header entry {key}')
        header[key] = values
        if key == 'DATA':
            break

    required = {'FIELDS', 'SIZE', 'TYPE', 'POINTS', 'DATA'}
    missing = required - header.keys()
    if missing:
        raise ValueError(f'Missing PCD header entries: {", ".join(sorted(missing))}')
    names = header['FIELDS']
    sizes = list(map(int, header['SIZE']))
    types = header['TYPE']
    counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
    if not len(names) == len(sizes) == len(types) == len(counts):
        raise ValueError('PCD FIELDS, SIZE, TYPE and COUNT lengths must agree')
    if len(header['POINTS']) != 1 or len(header['DATA']) != 1:
        raise ValueError('PCD POINTS and DATA must each have one value')
    point_count = int(header['POINTS'][0])
    if point_count <= 0:
        raise ValueError('PCD point cloud is empty or has an invalid POINTS count')

    fields = []
    for name, size, kind, count in zip(names, sizes, types, counts):
        supported_sizes = {'F': (4, 8), 'I': (1, 2, 4, 8), 'U': (1, 2, 4, 8)}
        if count <= 0 or size not in supported_sizes.get(kind, ()):
            raise ValueError(f'Invalid PCD field layout for {name}')
        fields.append((name, size, kind, count))
    for axis in XYZ_FIELDS:
        if names.count(axis) != 1:
            raise ValueError(f'PCD must have exactly one {axis} field')
        _, _, kind, count = fields[names.index(axis)]
        if kind != 'F' or count != 1:
            raise ValueError(f'PCD {axis} must be a scalar float32 or float64 field')
    return fields, point_count, header['DATA'][0]


def _lzf_decompress(payload, expected_size):
    output = bytearray()
    cursor = 0
    while cursor < len(payload):
        control = payload[cursor]
        cursor += 1
        if control < 32:
            length = control + 1
            end = cursor + length
            if end > len(payload) or len(output) + length > expected_size:
                raise ValueError('Invalid LZF literal run')
            output.extend(payload[cursor:end])
            cursor = end
            continue

        length = (control >> 5) + 2
        if length == 9:
            if cursor >= len(payload):
                raise ValueError('Truncated LZF match length')
            length += payload[cursor]
            cursor += 1
        if cursor >= len(payload):
            raise ValueError('Truncated LZF match offset')
        distance = ((control & 31) << 8) + payload[cursor] + 1
        cursor += 1
        if distance > len(output) or len(output) + length > expected_size:
            raise ValueError('Invalid LZF back-reference')
        # Copying past the end of the existing bytes repeats the same pattern.
        start = len(output) - distance
        pattern = output[start:start + min(length, distance)]
        repeats, remainder = divmod(length, len(pattern))
        output.extend(pattern * repeats + pattern[:remainder])

    if len(output) != expected_size:
        raise ValueError('LZF output size differs from the PCD size header')
    return output


def _read_ascii_xyz(stream, fields, point_count):
    offsets = {}
    formats = {}
    column = 0
    for name, size, _, count in fields:
        if name in XYZ_FIELDS:
            offsets[name] = column
            formats[name] = f'<f{size}'
        column += count
    dtype = [(axis, formats[axis]) for axis in XYZ_FIELDS]
    rows = np.loadtxt(stream, dtype=dtype, usecols=[offsets[axis] for axis in XYZ_FIELDS],
                      ndmin=1)
    if rows.size != point_count:
        raise ValueError(f'PCD POINTS is {point_count}, but found {rows.size} ASCII rows')
    return {axis: rows[axis] for axis in XYZ_FIELDS}


def _read_binary_xyz(payload, fields, point_count, compressed):
    if compressed:
        # PCL stores contiguous field planes and omits alignment padding.
        fields = [field for field in fields if field[0] != '_']
    point_size = sum(size * count for _, size, _, count in fields)
    expected_size = point_size * point_count
    if compressed:
        if len(payload) < 8:
            raise ValueError('Truncated binary_compressed PCD size header')
        compressed_size, uncompressed_size = struct.unpack_from('<II', payload)
        if compressed_size != len(payload) - 8 or uncompressed_size != expected_size:
            raise ValueError('binary_compressed PCD sizes do not match the field layout or payload')
        payload = _lzf_decompress(memoryview(payload)[8:], expected_size)
    elif len(payload) != expected_size:
        raise ValueError('Binary PCD payload size does not match POINTS and the field layout')

    xyz = {}
    offset = 0
    for name, size, _, count in fields:
        if name in XYZ_FIELDS:
            xyz[name] = np.ndarray(point_count, dtype=f'<f{size}', buffer=payload,
                                   offset=offset, strides=(size if compressed else point_size,))
        offset += size * count * (point_count if compressed else 1)
    return xyz


def read_pcd(path):
    """Read XYZ as an (N, 3) float64 array without applying the PCD VIEWPOINT."""
    with path.open('rb') as stream:
        fields, point_count, encoding = _read_pcd_header(stream)
        if encoding == 'ascii':
            xyz = _read_ascii_xyz(stream, fields, point_count)
        elif encoding in ('binary', 'binary_compressed'):
            xyz = _read_binary_xyz(stream.read(), fields, point_count,
                                   compressed=encoding == 'binary_compressed')
        else:
            raise ValueError(f'Unsupported PCD DATA encoding: {encoding}')
    return np.column_stack([xyz[axis].astype(np.float64, copy=False) for axis in XYZ_FIELDS])


def read_keyframe_xy(path):
    """Read XY translations from the Z-Localization backend map index."""
    positions = []
    with path.open(encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            fields = line.partition('#')[0].split()
            if not fields:
                continue
            try:
                if len(fields) != 9 or int(fields[0]) < 0:
                    raise ValueError('expected index timestamp tx ty tz qx qy qz qw')
                x, y = map(float, fields[2:4])
                if not math.isfinite(x) or not math.isfinite(y):
                    raise ValueError('XY must be finite')
            except ValueError as error:
                raise ValueError(f'{path}:{line_number}: invalid keyframe row ({error})') from error
            positions.append((x, y))
    if not positions:
        raise ValueError(f'{path}: keyframe file is empty')
    return np.array(positions)


def _clear_keyframe_neighborhoods(image, keyframe_xy, origin, resolution):
    height, width = image.shape
    radius = KEYFRAME_CLEAR_RADIUS_M
    for x, y in keyframe_xy:
        low = np.maximum(0, np.floor((np.array([x, y]) - radius - origin) / resolution))
        high = np.minimum([width - 1, height - 1],
                          np.floor((np.array([x, y]) + radius - origin) / resolution))
        first_col, first_y = low.astype(np.intp)
        last_col, last_y = high.astype(np.intp)
        cols = np.arange(first_col, last_col + 1)
        rows = np.arange(last_y, first_y - 1, -1)
        dx = origin[0] + (cols + 0.5) * resolution - x
        dy = origin[1] + (rows + 0.5) * resolution - y
        within_radius = dy[:, None]**2 + dx[None, :]**2 <= radius**2
        neighborhood = image[height - 1 - last_y:height - first_y, first_col:last_col + 1]
        neighborhood[within_radius] = FREE

    # Include each containing cell even when its center is outside the radius.
    cols, y_cells = np.floor((keyframe_xy - origin) / resolution).astype(np.intp).T
    image[height - 1 - y_cells, cols] = FREE


def rasterize(points, resolution, lower_z, upper_z, padding, min_points_per_cell,
              keyframe_xy=None):
    """Return a top-down PGM array and the map-frame XY of the lower-left corner."""
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        raise ValueError('PCD point cloud has no finite XYZ points')
    cloud_min = points[:, :2].min(axis=0)
    cloud_max = points[:, :2].max(axis=0)
    bounds_min, bounds_max = cloud_min, cloud_max
    if keyframe_xy is not None:
        bounds_min = np.minimum(bounds_min, keyframe_xy.min(axis=0))
        bounds_max = np.maximum(bounds_max, keyframe_xy.max(axis=0))

    origin = bounds_min - padding
    with np.errstate(over='ignore', invalid='ignore'):
        last_cell = np.floor((bounds_max - origin) / resolution)
        padded_extent = np.ceil((bounds_max + padding - origin) / resolution)
        grid_size = np.maximum(last_cell + 1, padded_extent)
    if not np.isfinite(grid_size).all() or np.any(grid_size > np.iinfo(np.int32).max):
        raise ValueError('Map dimensions exceed the supported range')
    width, height = map(int, grid_size)
    image = np.full((height, width), UNKNOWN, dtype=np.uint8)

    if keyframe_xy is None:
        first_col, first_y = np.floor((cloud_min - origin) / resolution).astype(np.intp)
        last_col, last_y = np.floor((cloud_max - origin) / resolution).astype(np.intp)
        image[height - 1 - last_y:height - first_y, first_col:last_col + 1] = FREE
    else:
        _clear_keyframe_neighborhoods(image, keyframe_xy, origin, resolution)

    in_band = (points[:, 2] >= lower_z) & (points[:, 2] <= upper_z)
    cols, y_cells = np.floor((points[in_band, :2] - origin) / resolution).astype(np.intp).T
    indices = (height - 1 - y_cells) * width + cols
    if min_points_per_cell > 1:
        indices, counts = np.unique(indices, return_counts=True)
        indices = indices[counts >= min_points_per_cell]
    image.flat[indices] = OCCUPIED
    return image, origin


def write_nav2_map(output, image, origin, resolution):
    if output.suffix.lower() in ('.yaml', '.pgm'):
        output = output.with_suffix('')
    image_path = output.with_name(output.name + '.pgm')
    yaml_path = output.with_name(output.name + '.yaml')
    output.parent.mkdir(parents=True, exist_ok=True)
    if cv2 is None:
        height, width = image.shape
        with image_path.open('wb') as stream:
            stream.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            stream.write(image.tobytes())
    elif not cv2.imwrite(str(image_path), image):
        raise OSError(f'Could not write PGM image {image_path}')
    yaml_path.write_text(yaml.safe_dump({
        'image': image_path.name,
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }, sort_keys=False), encoding='utf-8')
    return image_path, yaml_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', required=True, type=Path, help='Input map.pcd path')
    parser.add_argument('--output', required=True, type=Path, help='Output prefix or .yaml/.pgm path')
    parser.add_argument('--resolution', type=float, default=0.05, help='Meters per cell (default: 0.05)')
    parser.add_argument('--min-z', type=float, default=0.10, help='Lower obstacle height above ground (default: 0.10 m)')
    parser.add_argument('--max-z', type=float, default=0.50, help='Upper obstacle height above ground (default: 0.50 m)')
    parser.add_argument('--ground-z', type=float, default=0.0, help='Ground Z in the map frame (default: 0.0 m)')
    parser.add_argument('--padding', type=float, default=1.0, help='Map padding (default: 1.0 m)')
    parser.add_argument('--min-points-per-cell', type=int, default=1,
                        help='Minimum in-band points to mark a cell occupied (default: 1)')
    parser.add_argument('--keyframes', type=Path,
                        help=f'Backend keyframes.txt for local free clearing ({KEYFRAME_CLEAR_RADIUS_M} m radius)')
    args = parser.parse_args(argv)

    if not math.isfinite(args.resolution) or args.resolution <= 0:
        parser.error('--resolution must be finite and positive')
    if not math.isfinite(args.padding) or args.padding < 0:
        parser.error('--padding must be finite and nonnegative')
    lower_z = args.ground_z + args.min_z
    upper_z = args.ground_z + args.max_z
    if not (math.isfinite(lower_z) and math.isfinite(upper_z) and args.min_z <= args.max_z):
        parser.error('Height limits must be finite and --min-z must be <= --max-z')
    if args.min_points_per_cell < 1:
        parser.error('--min-points-per-cell must be at least 1')

    try:
        points = read_pcd(args.pcd)
        keyframe_xy = read_keyframe_xy(args.keyframes) if args.keyframes is not None else None
        image, origin = rasterize(points, args.resolution, lower_z, upper_z, args.padding,
                                  args.min_points_per_cell, keyframe_xy)
        image_path, yaml_path = write_nav2_map(args.output, image, origin, args.resolution)
    except (OSError, ValueError, MemoryError) as error:
        parser.error(str(error))
    print(f'Wrote {image_path} and {yaml_path} ({image.shape[1]} x {image.shape[0]} cells)')


if __name__ == '__main__':
    main()
