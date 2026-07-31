"""Unit-test discovery: which packages have unit tests and where their files live.

Driven by ``tests/colcon_unit_test_packages.yaml``. ``conftest.pytest_configure`` adds
``unit_test_files()`` to the pytest run, and ``pytest_itemcollected`` marks each of those
items ``unit`` via ``_is_unit_item``. ament lint tests are excluded here — they run under
``colcon test`` (see ``colcon_test_robot_command``'s ``-m not linter``).
"""
import os
from pathlib import Path

import yaml

AIRSTACK_ROOT = os.environ.get("AIRSTACK_ROOT", str(Path(__file__).resolve().parents[2]))
COLCON_UNIT_TEST_PACKAGES_YAML = (
    Path(AIRSTACK_ROOT) / "tests" / "colcon_unit_test_packages.yaml"
)


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
    return packages, cfg.get("pytest_args", "")


def colcon_test_robot_command(workspace="robot"):
    """Shell command for colcon test over unit-test packages (robot workspace)."""
    packages, pytest_args = load_colcon_unit_test_config(workspace)
    pkg_list = " ".join(packages)
    cmd = (
        f"colcon test --packages-select {pkg_list} "
        "--event-handlers console_direct+ --return-code-on-test-failure"
    )
    if pytest_args:
        cmd += f' --pytest-args "{pytest_args}"'
    return cmd


# Each listed package resolves to its <pkg>/test dir via these per-workspace globs.
_WORKSPACE_PKG_TEST_GLOBS = {
    "robot": "robot/ros_ws/src/**/{pkg}/test",
    "sim": "simulation/**/{pkg}/test",
}

# ament lint tests ship in every ROS package's test/ dir and import ament_* at
# module load (unavailable outside the built workspace). Skip them here — they run
# under `colcon test` instead (see colcon_test_robot_command's `-m not linter`).
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
