"""Semantic comparison keys for JSON settings; never use for provenance hashes.

Whole-valued floats and integers describe the same numeric setting. This pure
projection changes comparison keys only, preserving original actions/evidence.
"""
import json
import math


def canonical_settings(value, _path='$'):
    if value is None or type(value) in (str, bool, int):
        return value
    if type(value) is float:
        if not math.isfinite(value):
            raise ValueError(f'Nonfinite numeric setting at {_path}')
        return int(value) if value.is_integer() else value
    if isinstance(value, list):
        return [canonical_settings(item, f'{_path}[{index}]') for index, item in enumerate(value)]
    if isinstance(value, dict):
        if not all(isinstance(key, str) for key in value):
            raise ValueError(f'Settings object keys must be strings at {_path}')
        return {key: canonical_settings(item, f'{_path}.{key}') for key, item in value.items()}
    raise ValueError(f'Unsupported settings value at {_path}')


def settings_key(value):
    """Stable semantic identity, with no rounding of nonintegral numbers."""
    return json.dumps(canonical_settings(value), sort_keys=True, separators=(',', ':'), allow_nan=False)
