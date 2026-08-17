"""Unit-test discovery: which packages have unit tests and where their files live.

Driven by ``tests/colcon_unit_test_packages.yaml``. ``conftest.pytest_configure`` adds
``unit_test_files()`` to the pytest run whenever ``collection_is_broad`` says the command
line did not narrow the run, and ``pytest_itemcollected`` marks each of those items
``unit`` via ``_is_unit_item``. ament lint tests are excluded here — they run under
``colcon test`` (linter skip is in package pytest config; see
``colcon_test_robot_command``).
"""
import os
import shlex
from pathlib import Path

import yaml

AIRSTACK_ROOT = os.environ.get("AIRSTACK_ROOT", str(Path(__file__).resolve().parents[2]))
COLCON_UNIT_TEST_PACKAGES_YAML = (
    Path(AIRSTACK_ROOT) / "tests" / "colcon_unit_test_packages.yaml"
)

# The tests/ tree, derived from this file rather than AIRSTACK_ROOT so the guard always
# agrees with the conftest that is actually running.
TESTS_DIR = Path(__file__).resolve().parents[1]


def repo_path(*parts: str) -> Path:
    """Resolve a path relative to the repo root (``AIRSTACK_ROOT``).

    Single source of truth for cross-tree paths — no test or hook should hardcode
    ``Path(__file__).parents[N]`` walks.
    """
    return Path(AIRSTACK_ROOT).joinpath(*parts)


def load_colcon_unit_test_config(workspace="robot"):
    """Load colcon test package list and pytest args from tests/colcon_unit_test_packages.yaml."""
    if not COLCON_UNIT_TEST_PACKAGES_YAML.is_file():
        raise FileNotFoundError(
            f"Missing {COLCON_UNIT_TEST_PACKAGES_YAML} — add packages to gate in colcon test."
        )
    with COLCON_UNIT_TEST_PACKAGES_YAML.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if workspace not in data:
        raise KeyError(
            f"No '{workspace}' entry in {COLCON_UNIT_TEST_PACKAGES_YAML.name}"
        )
    cfg = data[workspace] or {}
    packages = cfg.get("packages") or []
    if not packages:
        raise ValueError(
            f"'{workspace}.packages' is empty in {COLCON_UNIT_TEST_PACKAGES_YAML.name}"
        )
    raw_args = cfg.get("pytest_args", [])
    if isinstance(raw_args, str):
        pytest_args = shlex.split(raw_args) if raw_args else []
    elif isinstance(raw_args, list):
        pytest_args = [str(a) for a in raw_args]
    else:
        raise TypeError(
            f"'{workspace}.pytest_args' must be a list or string in "
            f"{COLCON_UNIT_TEST_PACKAGES_YAML.name}, got {type(raw_args).__name__}"
        )
    return packages, pytest_args


def colcon_test_robot_command(workspace="robot"):
    """Shell command for colcon test over unit-test packages (robot workspace).

    Pytest flags from the YAML are *not* put on this command. colcon's
    ``--pytest-args`` is a single nargs='*' option (last occurrence wins),
    and nesting those tokens through ``bash -ic`` also breaks quoting.
    Pass them as ``PYTEST_ADDOPTS`` via ``docker_exec(..., env=...)``.
    """
    packages, _ = load_colcon_unit_test_config(workspace)
    pkg_list = " ".join(packages)
    return (
        f"colcon test --packages-select {pkg_list} "
        "--event-handlers console_direct+ --return-code-on-test-failure"
    )


def format_pytest_addopts(pytest_args):
    """Build a PYTEST_ADDOPTS value that pytest will shlex-split back to tokens.

    Do not name this pytest_*: conftest functions with that prefix are treated
    as pytest hooks and fail collection (exit code 3).
    """
    return " ".join(shlex.quote(a) for a in pytest_args)


# Each listed package resolves to its <pkg>/test dir via these per-workspace globs.
_WORKSPACE_PKG_TEST_GLOBS = {
    "robot": "robot/ros_ws/src/**/{pkg}/test",
    "sim": "simulation/**/{pkg}/test",
}

# ament lint tests ship in every ROS package's test/ dir and import ament_* at
# module load (unavailable outside the built workspace). Skip them here — they run
# under `colcon test` instead (package pytest config skips ament linters).
_LINTER_TEST_FILENAMES = {
    "test_copyright.py", "test_flake8.py", "test_pep257.py",
    "test_pep8.py", "test_xmllint.py", "test_lint_cmake.py",
}


def unit_test_dirs():
    """Every co-located unit-test dir resolved from colcon_unit_test_packages.yaml."""
    if not COLCON_UNIT_TEST_PACKAGES_YAML.is_file():
        return []
    with COLCON_UNIT_TEST_PACKAGES_YAML.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    dirs = []
    for workspace, glob_tmpl in _WORKSPACE_PKG_TEST_GLOBS.items():
        cfg = data.get(workspace) or {}
        for pkg in cfg.get("packages") or []:
            for match in repo_path().glob(glob_tmpl.format(pkg=pkg)):
                if match.is_dir():
                    dirs.append(match.resolve())
    return dirs


_UNIT_TEST_DIRS = None


def _unit_test_dirs_cached():
    global _UNIT_TEST_DIRS
    if _UNIT_TEST_DIRS is None:
        _UNIT_TEST_DIRS = unit_test_dirs()
    return _UNIT_TEST_DIRS


def _is_unit_item(item):
    """True when a collected item's file lives under a co-located unit-test dir."""
    try:
        p = Path(str(item.path)).resolve()
    except Exception:
        return False
    return any(p.is_relative_to(d) for d in _unit_test_dirs_cached())


def unit_test_files():
    """Co-located unit-test files to collect: every ``test_*.py`` under a package
    ``test/`` dir, minus the ament lint files.

    We collect explicit files (not the dirs) because pytest does not apply ignore
    rules to files it recurses into from an explicitly-passed directory — passing
    the exact files is the only deterministic way to keep ament lint tests out.
    """
    files = []
    for d in _unit_test_dirs_cached():
        for f in sorted(d.glob("test_*.py")):
            if f.name not in _LINTER_TEST_FILENAMES:
                files.append(f)
    return files


def _arg_path(arg, invocation_dir):
    """Absolute path addressed by one pytest positional.

    Positionals are raw CLI strings and may be node ids
    (``system/test_x.py::TestY::test_z``); only the part before ``::`` addresses the
    filesystem. The path need not exist — pytest reports bad paths itself.
    """
    return Path(invocation_dir, str(arg).split("::", 1)[0]).resolve()


def collection_is_broad(args, invocation_dir, tests_root=None) -> bool:
    """True when the positionals do not narrow the run below ``tests/``.

    Co-located unit tests live outside ``tests/``, so ``pytest_configure`` appends them
    to ``config.args`` by hand. It must do that only for a run that already means
    "everything", or ``pytest tests/system/test_x.py`` would drag in every unit test.

    Broad == a positional names ``tests/`` itself or an ancestor of it::

        pytest                        (testpaths ``.``, cwd tests/)  -> broad
        pytest tests/                 (CI, and the documented commands)  -> broad
        pytest .                      (cwd repo root or tests/)  -> broad
        pytest tests/system                                       -> narrow
        pytest tests/system/test_x.py::TestY::test_z              -> narrow
        pytest ../simulation/.../test/test_frames.py              -> narrow

    ``any`` rather than ``all`` is deliberate: ``pytest_configure`` appends the
    co-located files (narrow, absolute) to ``config.args``, so ``all`` would flip the
    answer for anything re-deriving it after that mutation.
    """
    root = Path(tests_root or TESTS_DIR).resolve()
    invocation_dir = Path(invocation_dir).resolve()
    return any(
        root.is_relative_to(_arg_path(a, invocation_dir))
        for a in args
        if not str(a).startswith("-")
    )
