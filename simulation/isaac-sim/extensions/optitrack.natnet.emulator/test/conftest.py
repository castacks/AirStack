# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Make the emulator package importable when running these tests directly.

The emulator is an Isaac Sim extension, not a pip-installed package, so a direct
``pytest test/`` (or ``colcon test``) needs the extension root on ``sys.path``.
pytest auto-loads this conftest before collecting any test in this directory, so
the test modules themselves stay free of ``sys.path`` boilerplate. (Runs via
``pytest tests/`` use the proxies under ``tests/`` instead, which set this up in
``tests/conftest.py``.)
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
