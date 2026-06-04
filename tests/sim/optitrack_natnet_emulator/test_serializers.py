# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: re-exposes optitrack.natnet.emulator unit tests for pytest tests/."""

import importlib.util
from pathlib import Path

import pytest

_repo_root = Path(__file__).resolve().parents[3]
_pkg_test = _repo_root / "simulation/isaac-sim/extensions/optitrack.natnet.emulator/test"
_real_file = _pkg_test / "test_serializers.py"

_spec = importlib.util.spec_from_file_location("_optitrack_natnet_serializers", _real_file)
_real = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_real)

for _name in dir(_real):
    if _name.startswith("test_"):
        globals()[_name] = pytest.mark.unit(getattr(_real, _name))
