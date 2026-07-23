# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Put the extension root on sys.path for direct pytest test/ runs.

`pytest tests/` (CI) collects this dir directly via the `sim` entry in
tests/colcon_unit_test_packages.yaml — no proxy files.
"""

import sys
from pathlib import Path

_EXT_ROOT = Path(__file__).resolve().parents[1]
if str(_EXT_ROOT) not in sys.path:
    sys.path.insert(0, str(_EXT_ROOT))


def pytest_configure(config):
    # Mirror the `unit` marker from tests/pytest.ini so direct `pytest test/`
    # runs don't emit PytestUnknownMarkWarning and `-m unit` works here too.
    config.addinivalue_line(
        "markers", "unit: Fast hermetic unit test (no Docker stack)."
    )
