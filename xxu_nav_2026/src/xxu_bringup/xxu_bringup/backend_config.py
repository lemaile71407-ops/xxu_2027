"""Resolve inherited Z-Localization backend YAML configurations."""

from pathlib import Path

import yaml


def merge_mapping(base, overrides):
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge_mapping(base[key], value)
        else:
            base[key] = value
    return base


def load_backend_params(path, ancestors=()):
    resolved = Path(path).resolve()
    if resolved in ancestors:
        raise ValueError(f'backend_config inheritance cycle: {resolved}')
    params = yaml.safe_load(resolved.read_text(encoding='utf-8')) or {}
    if not isinstance(params, dict):
        raise ValueError(f'backend_config must be a mapping: {resolved}')
    parent = params.pop('extends', None)
    if parent:
        parent_path = resolved.parent / parent
        if not parent_path.is_file():
            raise ValueError(f'backend_config parent must be an existing file: {parent_path}')
        params = merge_mapping(load_backend_params(parent_path, ancestors=(*ancestors, resolved)), params)
    return params
