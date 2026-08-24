# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""ROS 2 package.xml metadata contract.

Every first-party package.xml in the three ROS workspace trees
(``robot/ros_ws/src``, ``gcs/ros_ws/src``, ``common/ros_packages``) must carry
real metadata:

* a non-empty ``<description>`` with no TODO placeholder;
* ``<maintainer>`` entries with a real name and email — no ``todo@todo.todo``,
  ``user@todo.todo``, or ``example.com`` placeholders left over from
  ``ros2 pkg create``;
* a non-empty ``<license>`` tag.

The license VALUE is deliberately not asserted: first-party packages are
BSD-3-Clause-Clear, but vendored third-party packages (mav_comm from ETH
Zurich, the rqt-derived GUI plugins, ...) keep their upstream licenses.

Git submodules (vdb_mapping, vdb_mapping_ros2, rviz_polygon_selection_tool,
...) are excluded — their metadata belongs to their own repos — as are the
gitignored module overlay directories (``robot/ros_ws/src/modules/``).
"""
import re
import xml.etree.ElementTree as ET

import pytest

from harness.discovery import TESTS_DIR

pytestmark = pytest.mark.unit

REPO = TESTS_DIR.parent
WS_TREES = ("robot/ros_ws/src", "gcs/ros_ws/src", "common/ros_packages")

# Substrings that mark a maintainer name/email as a ros2-pkg-create leftover.
PLACEHOLDER_PATTERNS = ("todo", "example.com")
# Directory names never containing first-party package sources.
SKIP_DIR_NAMES = {"modules", "build", "install", "log", "__pycache__"}


def _submodule_paths():
    """Relative paths of git submodules, parsed from .gitmodules."""
    gitmodules = REPO / ".gitmodules"
    if not gitmodules.is_file():
        return []
    return re.findall(r"^\s*path\s*=\s*(\S+)", gitmodules.read_text(), re.M)


def _package_xmls():
    submodules = tuple(_submodule_paths())
    found = []
    for tree in WS_TREES:
        root = REPO / tree
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("package.xml")):
            rel = path.relative_to(REPO).as_posix()
            if any(rel.startswith(sub + "/") for sub in submodules):
                continue  # submodule: metadata is owned by its own repo
            if any(part in SKIP_DIR_NAMES for part in path.parts):
                continue  # module overlays / colcon artifacts
            found.append(path)
    return found


def _pkg_ids():
    return [p.relative_to(REPO).as_posix() for p in _package_xmls()]


def test_workspace_trees_have_packages():
    assert _package_xmls(), (
        f"no package.xml found under {WS_TREES} — workspace layout changed?"
    )


@pytest.mark.parametrize("package_xml", _package_xmls(), ids=_pkg_ids())
class TestPackageMetadata:

    def test_description_is_real(self, package_xml):
        root = ET.parse(package_xml).getroot()
        desc = root.find("description")
        assert desc is not None, f"{package_xml}: missing <description>"
        text = "".join(desc.itertext()).strip()
        assert text, f"{package_xml}: <description> is empty"
        assert "todo" not in text.lower(), (
            f"{package_xml}: <description> is a TODO placeholder ({text!r}) — "
            "write one real sentence about what the package does"
        )

    def test_maintainers_are_real(self, package_xml):
        root = ET.parse(package_xml).getroot()
        maintainers = root.findall("maintainer")
        assert maintainers, f"{package_xml}: no <maintainer> tag"
        for tag in maintainers:
            name = (tag.text or "").strip()
            email = (tag.get("email") or "").strip()
            assert name and email, (
                f"{package_xml}: <maintainer> needs both a name and an "
                f"email attribute (got name={name!r}, email={email!r})"
            )
            for value in (name, email):
                hits = [p for p in PLACEHOLDER_PATTERNS
                        if p in value.lower()]
                assert not hits, (
                    f"{package_xml}: maintainer {name!r} <{email}> looks "
                    f"like a ros2-pkg-create placeholder (matched "
                    f"{hits!r}) — put a real maintainer here"
                )

    def test_license_is_non_empty(self, package_xml):
        root = ET.parse(package_xml).getroot()
        licenses = root.findall("license")
        assert licenses, f"{package_xml}: no <license> tag"
        for tag in licenses:
            text = (tag.text or "").strip()
            # Value deliberately unchecked: vendored packages keep upstream
            # licenses; only emptiness/TODO is a contract violation.
            assert text, f"{package_xml}: <license> tag is empty"
            assert "todo" not in text.lower(), (
                f"{package_xml}: <license> is a TODO placeholder ({text!r})"
            )
