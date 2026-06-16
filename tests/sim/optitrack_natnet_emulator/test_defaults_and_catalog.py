# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: registers optitrack.natnet.emulator default/catalog unit tests for pytest tests/."""

from conftest import register_unit_tests, repo_path

register_unit_tests(
    globals(),
    repo_path("simulation/isaac-sim/extensions/optitrack.natnet.emulator/test"),
    "test_defaults.py",
    "test_server_catalog.py",
)
