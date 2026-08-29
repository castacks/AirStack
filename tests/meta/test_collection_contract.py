# Copyright (c) 2024 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Contract tests for co-located unit-test collection.

Unit-test source lives outside ``tests/``, so ``conftest.pytest_configure`` appends it to
the collection args when ``collection_is_broad`` says the command line did not narrow the
run. Get that wrong in the permissive direction and a narrowed run drags in every unit
test; get it wrong the other way and CI silently runs none of them.

These live under ``tests/`` on purpose. Co-located, they would stop being collected at
the same moment they stopped guarding anything — here plain recursion finds them, so a
broken guard makes them run and fail.
"""
import re
from pathlib import Path

import pytest

from harness.discovery import (  # noqa: E402 — pytest adds tests/ to sys.path
    TESTS_DIR,
    collection_is_broad,
    repo_path,
    unit_test_files,
)

# Not co-located, so `_is_unit_item` will not mark it — the one place the mark is
# written by hand.
pytestmark = pytest.mark.unit

_REPO = TESTS_DIR.parent


@pytest.mark.parametrize(
    "cwd, args",
    [
        (_REPO, ["tests/"]),                    # CI, and the documented commands
        (_REPO, ["tests"]),
        (_REPO, ["./tests/"]),
        (_REPO, [str(TESTS_DIR)]),
        (TESTS_DIR, ["."]),                     # testpaths, i.e. `airstack test`
        (TESTS_DIR, [str(TESTS_DIR)]),
    ],
)
def test_broad_invocations_collect_unit_tests(cwd, args):
    assert collection_is_broad(args, cwd) is True


@pytest.mark.parametrize(
    "cwd, args",
    [
        (TESTS_DIR, ["system"]),
        (TESTS_DIR, ["system/test_liveliness.py"]),
        (TESTS_DIR, ["system/test_liveliness.py::TestLiveliness::test_x"]),
        (_REPO, ["tests/system/test_sensors.py"]),
        (_REPO, ["tests/integration"]),
        (_REPO, ["robot/ros_ws/src/sensors/lidar_point_cloud_filter/test/test_validation_core.py"]),
        (_REPO, ["."]),                         # never recurse over the repo on host
        (_REPO, [""]),
        (_REPO, ["tests/", ""]),
        (_REPO, ["tests/", "tests/system"]),
        (_REPO, []),
    ],
)
def test_narrowed_invocations_do_not(cwd, args):
    assert collection_is_broad(args, cwd) is False


def test_ci_invocation_is_broad():
    """The shared system harness path must permit co-located injection.

    This catches the original bug: CI ran `pytest tests/`, which the guard classified
    as narrowed. The CPU workflow executes the injected tests with ``-m unit``;
    mark-scoped system runs may intentionally deselect them after safe collection.
    """
    workflow = repo_path(".github", "workflows", "system-tests.yml").read_text()
    match = re.search(r"^\s*pytest\s+(\S+)", workflow, re.M)
    assert match, "no `pytest <path>` invocation found in system-tests.yml"
    assert collection_is_broad([match.group(1)], _REPO), (
        f"system-tests.yml runs `pytest {match.group(1)}`, which does not collect "
        "co-located unit tests"
    )


def test_ci_empty_args_do_not_emit_an_empty_positional():
    """Guard the mapfile bug that changed bare `/pytest` into `pytest tests/ ""`."""
    workflow = repo_path(".github", "workflows", "system-tests.yml").read_text()
    assert "sys.stdout.write" in workflow
    assert "print('\\\\n'.join(shlex.split" not in workflow
    assert "Refusing an empty pytest argument" in workflow


def test_cpu_unit_workflow_uses_the_broad_harness_path():
    workflow = repo_path(".github", "workflows", "unit-tests.yml").read_text()
    match = re.search(r"^\s*run:\s+pytest\s+(\S+)", workflow, re.M)
    assert match, "no `pytest <path>` invocation found in unit-tests.yml"
    assert collection_is_broad([match.group(1)], _REPO)


def test_automatic_pr_gates_are_fast_and_repeat_on_updates():
    system = repo_path(".github", "workflows", "system-tests.yml").read_text()
    unit = repo_path(".github", "workflows", "unit-tests.yml").read_text()
    trigger = "types: [opened, synchronize, reopened]"
    assert trigger in system
    assert trigger in unit
    assert "args = ['-m', 'build_packages']" in system
    assert "runs-on: ubuntu-latest" in unit
    assert "run: pytest tests/ -m unit" in unit


def test_report_uses_the_revision_that_was_actually_tested():
    workflow = repo_path(".github", "workflows", "system-tests.yml").read_text()
    assert "tested_sha: ${{ steps.identity.outputs.tested_sha }}" in workflow
    assert "test-results-${{ steps.identity.outputs.tested_sha }}" in workflow
    assert "test-results-${{ needs.run-tests.outputs.tested_sha }}" in workflow
    assert "Number('${{ needs.run-tests.outputs.pr_number }}')" in workflow


def test_pr_head_check_is_finalized_after_metrics():
    workflow = repo_path(".github", "workflows", "system-tests.yml").read_text()
    assert workflow.index("- name: Finalize check on PR head") > workflow.index(
        "- name: Fail on report integrity error"
    )
    assert "ref: ${{ needs.run-tests.outputs.tested_sha }}" in workflow
    assert "conclusion: '${{ job.status }}'" not in workflow


def test_cross_run_baseline_uses_supported_download_inputs():
    workflow = repo_path(".github", "workflows", "system-tests.yml").read_text()
    explicit = workflow.split(
        "- name: Download baseline results (manual, explicit run ID)", 1
    )[1].split("- name:", 1)[0]
    assert "github-token:" in explicit
    assert 'pattern: "test-results-*"' in explicit
    assert "name_is_regexp:" not in explicit


def test_injection_actually_produced_items(request):
    """Every discovered unit-test file contributed at least one collected item.

    Catches breakage below the guard — YAML drift, a glob change, an import error that
    turns a module into a collection error rather than tests.
    """
    if not getattr(request.config, "airstack_unit_tests_injected", False):
        pytest.skip("narrowed run — co-located tests are not injected by design")

    collected = {Path(str(item.path)).resolve() for item in request.session.items}
    missing = [f for f in unit_test_files() if f.resolve() not in collected]
    assert not missing, "discovered but not collected: " + ", ".join(
        str(f.relative_to(_REPO)) for f in missing
    )
