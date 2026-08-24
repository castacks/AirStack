# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Import smoke tests for the hello_module fixture package.

This fixture lives under ``tests/``, so broad pytest recursion collects it directly
(unlike real co-located colcon unit tests, which are injected via
``colcon_unit_test_packages.yaml``). It is marked ``unit`` by hand and made
self-sufficient: the package import root goes on ``sys.path`` here.
"""
import sys
from pathlib import Path

import pytest

pytestmark = pytest.mark.unit

_PKG_ROOT = Path(__file__).resolve().parents[1]
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))


def test_package_imports():
    import hello_module

    assert hello_module.__version__


def test_node_module_imports_when_rclpy_available():
    pytest.importorskip("rclpy")
    from hello_module import hello_node

    assert callable(hello_node.main)
