# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: re-exposes optitrack.natnet.emulator interface config-model unit tests."""

from conftest import reexport_unit_tests, repo_path

reexport_unit_tests(
    globals(),
    repo_path("simulation/isaac-sim/extensions/optitrack.natnet.emulator/test"),
    "test_interface_config.py",
)
