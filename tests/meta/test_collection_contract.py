# Copyright (c) 2024 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
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

from conftest import repo_path  # noqa: E402 — pytest adds tests/ to sys.path
from harness.discovery import TESTS_DIR, collection_is_broad, unit_test_files

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
        (_REPO, ["."]),
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
        (_REPO, ["tests/integration/natnet"]),
        (_REPO, ["simulation/isaac-sim/extensions/optitrack.natnet.emulator/test/test_frames.py"]),
    ],
)
def test_narrowed_invocations_do_not(cwd, args):
    assert collection_is_broad(args, cwd) is False


def test_ci_invocation_is_broad():
    """The command system-tests.yml runs must collect unit tests.

    This is the test that would have caught the original bug: CI ran `pytest tests/`,
    which the guard classified as a narrowing run, so no unit test ever executed in CI.
    """
    workflow = repo_path(".github", "workflows", "system-tests.yml").read_text()
    match = re.search(r"^\s*pytest\s+(\S+)", workflow, re.M)
    assert match, "no `pytest <path>` invocation found in system-tests.yml"
    assert collection_is_broad([match.group(1)], _REPO), (
        f"system-tests.yml runs `pytest {match.group(1)}`, which does not collect "
        "co-located unit tests"
    )


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
