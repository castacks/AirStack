# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Contract tests for `airstack module` + tools/module_overlay.py (RFC #379, Phase P2).

The module machinery has host-visible contracts scripts and users depend on:
`module add <local-path>` records an ``x-local-modules`` entry in
./modules.repos, sync links the checkout into ``modules/`` and overlays it
(colcon symlink under robot/ros_ws/src/modules/, generated compose override in
.airstack/generated/), `module remove` restores a clean tree, remote adds are
refused without a pin (and refused *with* a branch-looking pin), and sync is
idempotent.

These tests shell the real ./airstack.sh — but inside a hermetic sandbox built
in tmp_path from copies of only the files the module commands touch
(airstack.sh, .airstack/modules/module.sh, tools/, common/module_schema/, the
hello_module fixture, stub repo dirs). No Docker, no network, no git state in
the sandbox, and the developer's real checkout is never mutated.
"""
import os
import shutil
import stat
import subprocess

import pytest
import yaml

from harness.discovery import repo_path

pytestmark = pytest.mark.unit

REPO = repo_path()
FIXTURE_REL = os.path.join("tests", "fixtures", "modules", "hello_module")

GENERATED_REL = os.path.join(".airstack", "generated", "docker-compose.modules.yaml")
ROBOT_LINK_REL = os.path.join("robot", "ros_ws", "src", "modules", "hello_module")


@pytest.fixture()
def sandbox(tmp_path):
    """A minimal AirStack checkout containing only what `airstack module` needs."""
    sb = tmp_path / "airstack"
    (sb / ".airstack" / "modules").mkdir(parents=True)
    (sb / "tools").mkdir()
    (sb / "robot" / "ros_ws" / "src").mkdir(parents=True)
    (sb / "simulation" / "isaac-sim" / "launch_scripts").mkdir(parents=True)

    shutil.copy(REPO / "airstack.sh", sb / "airstack.sh")
    (sb / "airstack.sh").chmod((sb / "airstack.sh").stat().st_mode | stat.S_IXUSR)
    shutil.copy(REPO / ".airstack" / "modules" / "module.sh",
                sb / ".airstack" / "modules" / "module.sh")
    shutil.copy(REPO / "tools" / "module_overlay.py", sb / "tools" / "module_overlay.py")
    shutil.copy(REPO / "tools" / "validate_module.py", sb / "tools" / "validate_module.py")
    # (P4) sync also runs the Docker layer planner (conflict gate + plan + lock)
    shutil.copy(REPO / "tools" / "compose_module_layers.py",
                sb / "tools" / "compose_module_layers.py")
    shutil.copytree(REPO / "common" / "module_schema", sb / "common" / "module_schema")
    shutil.copytree(
        REPO / FIXTURE_REL,
        sb / FIXTURE_REL,
        ignore=shutil.ignore_patterns("__pycache__"),
    )

    (sb / ".env").write_text('VERSION="0.19.0"\nPROJECT_NAME="airstack"\n', encoding="utf-8")
    (sb / "docker-compose.yaml").write_text("services: {}\n", encoding="utf-8")
    return sb


def run_airstack(sb, *args, check=True):
    result = subprocess.run(
        [str(sb / "airstack.sh"), *args],
        capture_output=True, text=True, cwd=str(sb), timeout=180,
    )
    out = result.stdout + result.stderr
    if check:
        assert result.returncode == 0, f"airstack {' '.join(args)} failed:\n{out}"
    return result.returncode, out


def add_hello(sb):
    return run_airstack(sb, "module", "add", FIXTURE_REL)


def read_repos(sb):
    with (sb / "modules.repos").open(encoding="utf-8") as f:
        return yaml.safe_load(f)


# ── add: repos entry, symlinks, generated compose ───────────────────────────

def test_add_local_module_records_x_local_entry(sandbox):
    add_hello(sandbox)
    data = read_repos(sandbox)
    assert data["repositories"] == {}  # local modules never enter the vcs list
    assert data["x-local-modules"] == [
        {"name": "hello_module", "path": FIXTURE_REL}
    ]


def test_add_local_module_links_checkout_and_colcon_overlay(sandbox):
    add_hello(sandbox)
    checkout = sandbox / "modules" / "hello_module"
    assert checkout.is_symlink()
    assert (checkout / "module.yaml").is_file()  # resolves through the link

    robot_link = sandbox / ROBOT_LINK_REL
    assert robot_link.is_symlink()
    assert os.readlink(robot_link) == os.path.join("..", "..", "..", "..", "modules", "hello_module")
    # the chain resolves on the host: link -> modules/hello_module -> fixture
    assert (robot_link / "module.yaml").is_file()


def test_add_local_module_generates_compose_volume(sandbox):
    add_hello(sandbox)
    generated = sandbox / GENERATED_REL
    assert generated.is_file()
    compose = yaml.safe_load(generated.read_text(encoding="utf-8"))
    fixture_real = os.path.realpath(sandbox / FIXTURE_REL)
    expected = f"{fixture_real}:/root/AirStack/modules/hello_module:rw"
    services = compose["services"]
    assert services, "generated compose declares no services"
    for service, definition in services.items():
        assert expected in definition["volumes"], (
            f"service {service} missing module volume; got {definition['volumes']}"
        )


def test_doctor_passes_after_add(sandbox):
    add_hello(sandbox)
    code, out = run_airstack(sandbox, "module", "doctor")
    assert code == 0
    assert "overlay OK" in out


# ── list ─────────────────────────────────────────────────────────────────────

def test_list_shows_module_as_valid(sandbox):
    add_hello(sandbox)
    _, out = run_airstack(sandbox, "module", "list")
    row = next((l for l in out.splitlines() if l.startswith("hello_module")), None)
    assert row is not None, f"no hello_module row in:\n{out}"
    assert "ros_package" in row
    assert "local" in row
    assert "yes" in row  # VALID column


# ── remove: everything is restored ───────────────────────────────────────────

def test_remove_cleans_all_artifacts(sandbox):
    add_hello(sandbox)
    run_airstack(sandbox, "module", "remove", "hello_module")

    assert not (sandbox / "modules.repos").exists()
    assert not (sandbox / "modules").exists()
    assert not (sandbox / ROBOT_LINK_REL).exists()
    assert not os.path.lexists(sandbox / ROBOT_LINK_REL)  # not even a dangling link
    assert not (sandbox / "robot" / "ros_ws" / "src" / "modules").exists()
    assert not (sandbox / GENERATED_REL).exists()
    assert not (sandbox / ".airstack" / "generated").exists()
    # the fixture source itself is untouched
    assert (sandbox / FIXTURE_REL / "module.yaml").is_file()


def test_remove_unknown_module_fails(sandbox):
    code, out = run_airstack(sandbox, "module", "remove", "no_such_module", check=False)
    assert code != 0
    assert "no_such_module" in out


# ── pinning rules for remote adds ────────────────────────────────────────────

def test_url_add_without_version_fails(sandbox):
    code, out = run_airstack(
        sandbox, "module", "add", "https://github.com/example/asm_thing.git",
        check=False,
    )
    assert code != 0
    assert "--version" in out
    assert not (sandbox / "modules.repos").exists()  # nothing was recorded


@pytest.mark.parametrize("branch", ["main", "develop"])
def test_url_add_with_branch_ref_fails(sandbox, branch):
    code, out = run_airstack(
        sandbox, "module", "add", "https://github.com/example/asm_thing.git",
        "--version", branch, check=False,
    )
    assert code != 0
    assert "branch" in out.lower()
    assert not (sandbox / "modules.repos").exists()


# ── idempotence ──────────────────────────────────────────────────────────────

def test_sync_twice_is_idempotent(sandbox):
    add_hello(sandbox)
    generated = sandbox / GENERATED_REL
    compose_before = generated.read_text(encoding="utf-8")
    repos_before = (sandbox / "modules.repos").read_text(encoding="utf-8")

    run_airstack(sandbox, "module", "sync")

    assert generated.read_text(encoding="utf-8") == compose_before
    assert (sandbox / "modules.repos").read_text(encoding="utf-8") == repos_before
    compose = yaml.safe_load(compose_before)
    for definition in compose["services"].values():
        volumes = definition["volumes"]
        assert len(volumes) == len(set(volumes)), f"duplicate volume entries: {volumes}"


def test_add_twice_keeps_single_entry(sandbox):
    add_hello(sandbox)
    add_hello(sandbox)
    data = read_repos(sandbox)
    names = [e["name"] for e in data["x-local-modules"]]
    assert names == ["hello_module"]
