# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Proxy: re-exposes optitrack.natnet.emulator Commit 3 target-resolution tests."""

from conftest import reexport_unit_tests, repo_path

reexport_unit_tests(
    globals(),
    repo_path("simulation/isaac-sim/extensions/optitrack.natnet.emulator/test"),
    "test_target_resolution.py",
)
