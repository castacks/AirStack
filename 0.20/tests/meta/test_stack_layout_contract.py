# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Stack folder anatomy contract (RFC #379 §3, RFC #385 §1).

Every stack directory under ``stacks/`` (ignoring ``.external``, the gitignored
home of fetched third-party stack repos) must ship the four committed anatomy
files:

* ``modules.repos`` — YAML with a top-level ``airstack_compat`` semver range
  (sibling of ``repositories:``; vcstool ignores it, our tooling reads it);
* ``launch/`` — at least one ``*.launch.xml`` entry point;
* ``docker-compose.yaml`` — valid YAML (a documented stub until module pins
  arrive);
* ``README.md`` — non-trivial.

``wiring.md`` is NOT required yet (bootstrap: it is generated from the first
validated wiring-snapshot run), but when present it must carry the
machine-readable ``wiring-graph-v1`` trailer that the drift check reads.

Split stacks (RFC #380 §2) extend the anatomy: a stack with **two or more**
launch entry points is a split stack and MUST carry a ``bridge.yaml``
explicitly listing every topic/service/action crossing the machine boundary
(its schema and the control/trajectory placement hard gate are covered by
``tests/meta/test_bridge_contract.py``).
"""
import re

import pytest
import yaml

from harness.discovery import TESTS_DIR

pytestmark = pytest.mark.unit

REPO = TESTS_DIR.parent
STACKS_DIR = REPO / "stacks"


def _stack_dirs():
    if not STACKS_DIR.is_dir():
        return []
    return sorted(
        d for d in STACKS_DIR.iterdir()
        if d.is_dir() and not d.name.startswith(".")
    )


def _stack_ids():
    return [d.name for d in _stack_dirs()]


def test_stacks_dir_has_reference_stacks():
    assert _stack_dirs(), (
        f"{STACKS_DIR} has no stack folders — trunk ships reference stacks "
        "(full_default, ...)"
    )


@pytest.mark.parametrize("stack", _stack_dirs(), ids=_stack_ids())
class TestStackAnatomy:

    def test_modules_repos(self, stack):
        path = stack / "modules.repos"
        assert path.is_file(), f"{stack.name}: missing modules.repos"
        data = yaml.safe_load(path.read_text())
        assert isinstance(data, dict), (
            f"{stack.name}: modules.repos did not parse to a YAML mapping"
        )
        compat = data.get("airstack_compat")
        assert isinstance(compat, str) and compat.strip(), (
            f"{stack.name}: modules.repos needs a top-level airstack_compat "
            "semver-range string (the trunk range the stack was tested "
            "against — RFC #379 §3)"
        )
        assert "repositories" in data, (
            f"{stack.name}: modules.repos needs a repositories: key "
            "(vcstool format; {} when the stack pins no modules)"
        )

    def test_launch_entry_points(self, stack):
        launch_dir = stack / "launch"
        assert launch_dir.is_dir(), f"{stack.name}: missing launch/ dir"
        entries = list(launch_dir.glob("*.launch.xml"))
        assert entries, (
            f"{stack.name}: launch/ has no *.launch.xml entry point "
            "(unsplit stacks have exactly stack.launch.xml)"
        )

    def test_docker_compose_is_valid_yaml(self, stack):
        path = stack / "docker-compose.yaml"
        assert path.is_file(), f"{stack.name}: missing docker-compose.yaml"
        data = yaml.safe_load(path.read_text())
        assert data is not None, (
            f"{stack.name}: docker-compose.yaml is empty — keep at least "
            "'services: {}' plus the explanatory comments"
        )

    def test_readme_non_trivial(self, stack):
        path = stack / "README.md"
        assert path.is_file(), f"{stack.name}: missing README.md"
        text = path.read_text().strip()
        assert len(text) >= 200, (
            f"{stack.name}: README.md is trivial ({len(text)} chars) — state "
            "what the stack is for, how to run it, and its known limits"
        )

    def test_split_stack_requires_bridge(self, stack):
        """Two or more entry points = a split stack = an explicit bridge.yaml
        (RFC #380 §2: the bridge list IS the split, readable in source)."""
        entries = sorted((stack / "launch").glob("*.launch.xml"))
        if len(entries) < 2:
            pytest.skip(f"{stack.name}: unsplit stack "
                        f"({len(entries)} entry point)")
        assert (stack / "bridge.yaml").is_file(), (
            f"{stack.name}: {len(entries)} launch entry points "
            f"({', '.join(e.name for e in entries)}) but no bridge.yaml — a "
            "split stack must declare every topic/service/action crossing "
            "the machine boundary explicitly (RFC #380 §2); generate the "
            "router config from it with tools/gen_dds_router.py"
        )

    def test_wiring_md_trailer_when_present(self, stack):
        """wiring.md is optional (bootstrap) but must be machine-readable."""
        path = stack / "wiring.md"
        if not path.exists():
            pytest.skip(f"{stack.name}: wiring.md not committed yet "
                        "(generated by the first wiring-snapshot run)")
        import wiring_snapshot as ws
        try:
            graph = ws.extract_graph_from_md(path.read_text())
        except ValueError as exc:
            pytest.fail(
                f"{stack.name}: wiring.md lacks a parseable wiring-graph-v1 "
                f"trailer ({exc}) — regenerate it from a wiring-snapshot "
                "run, never hand-edit"
            )
        assert graph.get("nodes") is not None, (
            f"{stack.name}: wiring.md trailer has no nodes key — regenerate "
            "it from a wiring-snapshot run"
        )

    def test_wiring_md_title_names_this_stack(self, stack):
        """wiring.md's H1 must name the stack dir it lives in (a copied
        baseline that still says its source stack's name is a lie — found
        the hard way when full_droan_cpu shipped titled full_default)."""
        path = stack / "wiring.md"
        if not path.exists():
            pytest.skip(f"{stack.name}: wiring.md not committed yet "
                        "(generated by the first wiring-snapshot run)")
        title = path.read_text(encoding="utf-8").splitlines()[0].strip()
        assert title.startswith("# "), (
            f"{stack.name}: wiring.md must start with an H1 title line "
            f"(got {title!r})"
        )
        assert title.endswith(f" {stack.name}") or title == f"#{stack.name}", (
            f"{stack.name}: wiring.md H1 title must end with the stack dir "
            f"name (got {title!r}) — regenerate via the wiring-snapshot run "
            "for THIS stack, don't copy another stack's baseline"
        )


@pytest.mark.parametrize("stack", _stack_dirs(), ids=_stack_ids())
def test_no_dispatcher_include(stack):
    """A stack entry file must never include the dispatcher.

    robot.launch.xml is the DISPATCHER that includes the stack entry file
    when AIRSTACK_STACK_DIR is set; a stack entry that wraps it recurses
    infinitely (robot_1/robot_1/... namespace explosion — found the hard way
    by the asm_optitrack test_stack).
    """
    for entry in sorted((stack / "launch").glob("*.launch.xml")):
        text = entry.read_text(encoding="utf-8")
        # strip XML comments so prose warnings about the rule don't trip it
        uncommented = re.sub(r"<!--.*?-->", "", text, flags=re.DOTALL)
        assert "robot.launch.xml" not in uncommented, (
            f"{stack.name}/{entry.name} includes robot.launch.xml — the "
            "dispatcher includes stack entries, never the reverse "
            "(infinite recursion)"
        )
