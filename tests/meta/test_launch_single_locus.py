# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Single-locus wiring lint (RFC #379 §4.1).

What this lint enforces, precisely: no ``<remap>`` (XML) and no
``remappings=`` (Python launch) in any launch file outside
``stacks/*/launch/``, except the shrinking grandfather allowlist. Module
launch files declare their topic endpoints as ``<arg>``s with canonical
defaults and apply them to their own node locally via ``<set_remap>`` — that
is the sanctioned pattern and is deliberately NOT matched by this lint's
regex. Cross-module REWIRING (pointing one module at another's non-canonical
topic) lives only in stack entry files, as include args.

The rule lands mid-migration (wrap form, P5-E1), so the launch files that
carried remaps at freeze time are grandfathered in
``launch_lint_allowlist.txt``. The allowlist only shrinks:

* rule 1 — no remap outside ``stacks/*/launch/`` except allowlisted files;
* rule 2 — every allowlist entry must still exist AND still contain a remap
  (once E2/E3 flatten a file's wiring away, its line must be deleted);
* rule 3 — stack launch files must carry ``description=`` on every ``<arg>``
  they declare (they are the wiring document — args must be self-describing).

Scan scope: ``*.xml`` / ``*.py`` under any ``launch/`` directory inside
``robot/``, ``gcs/``, ``common/``, and ``simulation/``.
"""
import re
import xml.etree.ElementTree as ET

import pytest

from harness.discovery import TESTS_DIR

pytestmark = pytest.mark.unit

REPO = TESTS_DIR.parent
ALLOWLIST_PATH = TESTS_DIR / "meta" / "launch_lint_allowlist.txt"
SCAN_ROOTS = ("robot", "gcs", "common", "simulation")

# <remap ...> in XML launch; remappings=[...] in Python launch.
_REMAP_RE = re.compile(r"<remap\b|remappings\s*=")


def _scanned_launch_files():
    """Every *.xml / *.py under a launch/ dir inside the scan roots."""
    files = []
    for root in SCAN_ROOTS:
        base = REPO / root
        if not base.is_dir():
            continue
        for suffix in ("xml", "py"):
            for path in base.rglob(f"*.{suffix}"):
                if "launch" in path.parent.parts:
                    files.append(path)
    return sorted(files)


def _has_remap(path):
    try:
        return bool(_REMAP_RE.search(path.read_text(errors="replace")))
    except OSError:
        return False


def _allowlist():
    entries = []
    for line in ALLOWLIST_PATH.read_text().splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            entries.append(line)
    return entries


def test_no_remaps_outside_stack_launch_files():
    """Rule 1: remaps live in stacks/*/launch/ — everything else is frozen."""
    allowed = set(_allowlist())
    offenders = []
    for path in _scanned_launch_files():
        rel = path.relative_to(REPO).as_posix()
        if _has_remap(path) and rel not in allowed:
            offenders.append(rel)
    assert not offenders, (
        "Launch files outside stacks/*/launch/ contain <remap>/remappings= "
        "but are not in the frozen allowlist "
        f"({ALLOWLIST_PATH.relative_to(REPO)}):\n  "
        + "\n  ".join(offenders)
        + "\nCross-module wiring belongs in the stack entry launch file "
        "(RFC #379 §4 single-locus rule; see "
        ".agents/skills/write-launch-file/SKILL.md). The allowlist only "
        "shrinks — do not add new entries."
    )


def test_allowlist_entries_exist_and_still_have_remaps():
    """Rule 2 (shrink-only): stale allowlist lines must be deleted."""
    problems = []
    for rel in _allowlist():
        path = REPO / rel
        if not path.is_file():
            problems.append(f"{rel}: file no longer exists — delete its "
                            "allowlist line")
        elif not _has_remap(path):
            problems.append(f"{rel}: no longer contains a remap — its wiring "
                            "moved (good!); delete its allowlist line so the "
                            "file stays remap-free")
    assert not problems, (
        f"Stale entries in {ALLOWLIST_PATH.relative_to(REPO)}:\n  "
        + "\n  ".join(problems)
    )


def _stack_launch_files():
    stacks = REPO / "stacks"
    if not stacks.is_dir():
        return []
    return sorted(
        p for p in stacks.glob("*/launch/*.launch.xml")
        if not p.parts[len(stacks.parts)].startswith(".")
    )


def test_stack_launch_args_have_descriptions():
    """Rule 3: stack entry files are the wiring document — every <arg> they
    declare needs a description=. Vacuously true while stack files declare no
    args (wrap form passes include args instead)."""
    stack_files = _stack_launch_files()
    assert stack_files, "no stack launch files found under stacks/*/launch/"
    offenders = []
    for path in stack_files:
        tree = ET.parse(path)  # also asserts well-formed XML
        # Only top-level <arg> children of <launch> are declarations;
        # <arg> inside <include> passes a value and needs no description.
        for arg in tree.getroot().findall("arg"):
            if not arg.get("description"):
                offenders.append(
                    f"{path.relative_to(REPO).as_posix()}: "
                    f"<arg name=\"{arg.get('name')}\"> missing description="
                )
    assert not offenders, (
        "Stack launch files must describe every declared <arg>:\n  "
        + "\n  ".join(offenders)
    )
