# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Contract tests for tools/compose_module_layers.py (RFC #379 §6, Phase P4).

The layer planner turns synced modules' docker declarations (deps.apt/deps.pip,
dockerfile, overlay_image) into a per-host build plan
(.airstack/generated/layer_plan.json + layers/<host>/Dockerfile.composed) and a
deterministic modules.lock at the repo root. Contracts pinned here:

- **Zero-module identity rule** — no docker-relevant declarations means every
  host's plan is exactly {base_image: trunk tag, steps: [], final_tag: trunk
  tag}: no Dockerfile.composed, and the overlay-generated compose carries no
  ``image:`` overrides. A dep-free checkout keeps today's images byte-for-byte.
- **Determinism** — identical inputs produce byte-identical lock and plan, and
  the lock never leaks machine-local absolute paths.
- **Tier chaining** — a module with apt/pip deps AND a Dockerfile.module
  contributes a tier-1 step (one RUN per package manager, chained via ARG
  BASE_IMAGE) followed by a tier-2 step, in that order.
- **Conflict gate** — two modules pinning the same pip package differently
  fail --check-conflicts naming both; same-spec duplicates pass. `module sync`
  fails on a conflict (doctor hard gate #1).
- **Tier-3 sole-overlay rule** — a dockerfile-less overlay_image is accepted
  only as the sole docker-relevant module for its host; any other composition
  errors citing RFC #379 §6.
- **dep_hash** — changes when deps change, stable under unrelated manifest edits.

Tests run the real planner (imported once) against hermetic checkouts built in
tmp_path, mirroring test_module_overlay_contract.py; the two sync-integration
tests shell the real ./airstack.sh in the same sandbox pattern. No Docker, no
network, and the developer's real checkout is never mutated.
"""
import importlib.util
import json
import os
import shutil
import stat
import subprocess

import pytest
import yaml

from harness.discovery import repo_path

pytestmark = pytest.mark.unit

REPO = repo_path()
HELLO_REL = os.path.join("tests", "fixtures", "modules", "hello_module")
HEAVY_REL = os.path.join("tests", "fixtures", "modules", "heavy_module")

PLAN_REL = os.path.join(".airstack", "generated", "layer_plan.json")
LOCK_REL = "modules.lock"
LAYERS_REL = os.path.join(".airstack", "generated", "layers")
COMPOSE_REL = os.path.join(".airstack", "generated", "docker-compose.modules.yaml")

ENV_TEXT = (
    'VERSION="0.19.0"\n'
    'PROJECT_NAME="airstack"\n'
    'PROJECT_DOCKER_REGISTRY="registry.example.com/airstack"\n'
    'DOCKER_IMAGE_BUILD_MODE="dev"\n'
)
ROBOT_BASE = "registry.example.com/airstack/airstack:v0.19.0_robot-x86-64_dev"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def planner():
    return _load(REPO / "tools" / "compose_module_layers.py", "airstack_layer_planner")


@pytest.fixture(scope="module")
def overlay():
    return _load(REPO / "tools" / "module_overlay.py", "airstack_module_overlay")


def make_checkout(tmp_path, *fixtures, synthetic=()):
    """A minimal checkout: .env + modules/<name>/ copies + x-local modules.repos."""
    root = tmp_path / "airstack"
    (root / "modules").mkdir(parents=True)
    (root / ".env").write_text(ENV_TEXT, encoding="utf-8")
    names = []
    for rel in fixtures:
        name = os.path.basename(rel)
        shutil.copytree(REPO / rel, root / "modules" / name,
                        ignore=shutil.ignore_patterns("__pycache__"))
        names.append(name)
    for name, manifest in synthetic:
        mdir = root / "modules" / name
        mdir.mkdir()
        (mdir / "module.yaml").write_text(yaml.safe_dump(manifest), encoding="utf-8")
        names.append(name)
    (root / "modules.repos").write_text(
        yaml.safe_dump({
            "repositories": {},
            "x-local-modules": [{"name": n, "path": f"modules/{n}"} for n in names],
        }),
        encoding="utf-8",
    )
    return root


def manifest_for(name, **overrides):
    data = {
        "name": name,
        "description": f"Synthetic layer-plan fixture {name}.",
        "maintainer": "test@example.com",
        "license": "MIT",
        "type": "ros_package",
        "airstack_compat": ">=0.19.0 <0.21.0",
        "targets": ["robot"],
        "tests": {"packages": [], "marks": []},
    }
    data.update(overrides)
    return data


def read_plan(root):
    return json.loads((root / PLAN_REL).read_text(encoding="utf-8"))


def read_lock(root):
    return json.loads((root / LOCK_REL).read_text(encoding="utf-8"))


# ── zero-module identity rule ────────────────────────────────────────────────

def test_identity_rule_dep_free_module(planner, overlay, tmp_path, capsys):
    root = make_checkout(tmp_path, HELLO_REL)
    # run the P2 overlay too, then the planner — the sync order
    (root / "robot" / "ros_ws" / "src").mkdir(parents=True)
    (root / "simulation" / "isaac-sim" / "launch_scripts").mkdir(parents=True)
    assert overlay.run(root) == 0
    assert planner.main(["--project-root", str(root)]) == 0

    plan = read_plan(root)
    assert set(plan) == {"robot", "gcs", "isaac-sim", "ms-airsim"}
    for host, entry in plan.items():
        assert entry["steps"] == [], f"{host} has steps for a dep-free module"
        assert entry["final_tag"] == entry["base_image"]
    assert plan["robot"]["base_image"] == ROBOT_BASE
    assert plan["gcs"]["base_image"].endswith("_gcs")

    # no composed dockerfile anywhere
    assert not (root / LAYERS_REL).exists()
    # lock still records the module (identity affects images, not bookkeeping)
    lock = read_lock(root)
    assert [m["name"] for m in lock["modules"]] == ["hello_module"]
    assert lock["modules"][0]["pin"] == "local"
    assert lock["plan_hash"]

    # the overlay-generated compose must NOT gain image: overrides
    compose = yaml.safe_load((root / COMPOSE_REL).read_text(encoding="utf-8"))
    for service, definition in compose["services"].items():
        assert "image" not in definition, f"identity rule violated: {service} sets image"

    out = capsys.readouterr().out
    assert "0 docker-relevant modules — base images unchanged" in out


def test_zero_modules_cleans_generated_artifacts(planner, tmp_path):
    root = make_checkout(tmp_path, HELLO_REL)
    assert planner.main(["--project-root", str(root)]) == 0
    assert (root / PLAN_REL).is_file() and (root / LOCK_REL).is_file()

    shutil.rmtree(root / "modules")
    (root / "modules.repos").unlink()
    assert planner.main(["--project-root", str(root)]) == 0
    assert not (root / PLAN_REL).exists()
    assert not (root / LOCK_REL).exists()
    assert not (root / ".airstack" / "generated").exists()


# ── determinism ──────────────────────────────────────────────────────────────

def test_two_runs_are_byte_identical(planner, tmp_path):
    root = make_checkout(tmp_path, HELLO_REL, HEAVY_REL)
    assert planner.main(["--project-root", str(root)]) == 0
    lock1 = (root / LOCK_REL).read_bytes()
    plan1 = (root / PLAN_REL).read_bytes()

    assert planner.main(["--project-root", str(root)]) == 0
    assert (root / LOCK_REL).read_bytes() == lock1
    assert (root / PLAN_REL).read_bytes() == plan1

    # deterministic serialization must not embed machine-local paths
    assert str(tmp_path).encode() not in lock1


def test_lock_identical_across_checkout_locations(planner, tmp_path):
    root_a = make_checkout(tmp_path / "a", HEAVY_REL)
    root_b = make_checkout(tmp_path / "b", HEAVY_REL)
    assert planner.main(["--project-root", str(root_a)]) == 0
    assert planner.main(["--project-root", str(root_b)]) == 0
    assert (root_a / LOCK_REL).read_bytes() == (root_b / LOCK_REL).read_bytes()


# ── tier-1 + tier-2 chaining (heavy_module) ──────────────────────────────────

def test_heavy_module_tier1_then_tier2(planner, tmp_path):
    root = make_checkout(tmp_path, HEAVY_REL)
    assert planner.main(["--project-root", str(root)]) == 0

    plan = read_plan(root)
    robot = plan["robot"]
    assert [(s["module"], s["tier"]) for s in robot["steps"]] == [
        ("heavy_module", 1),
        ("heavy_module", 2),
    ]
    tier1, tier2 = robot["steps"]
    assert tier1["dockerfile"] is None
    assert tier2["dockerfile"] == os.path.join("modules", "heavy_module", "Dockerfile.module")
    assert tier1["dep_hash"] == tier2["dep_hash"]  # same module, same declaration hash

    # final tag: trunk scheme + -m<8-char plan_hash prefix>
    lock = read_lock(root)
    assert robot["base_image"] == ROBOT_BASE
    assert robot["final_tag"] == f"{ROBOT_BASE}-m{lock['plan_hash'][:8]}"

    # hosts the module does not target stay identity
    for host in ("gcs", "isaac-sim", "ms-airsim"):
        assert plan[host]["steps"] == []
        assert plan[host]["final_tag"] == plan[host]["base_image"]

    composed = (root / LAYERS_REL / "robot" / "Dockerfile.composed").read_text(encoding="utf-8")
    assert "ARG BASE_IMAGE" in composed
    assert "FROM ${BASE_IMAGE}" in composed
    assert "apt-get install -y --no-install-recommends cowsay" in composed
    assert "RUN pip3 install --no-cache-dir --break-system-packages tabulate" in composed
    # only the robot host composes
    assert not (root / LAYERS_REL / "gcs").exists()


# ── conflict gate ────────────────────────────────────────────────────────────

def test_conflicting_pip_pins_fail_naming_both_modules(planner, tmp_path, capsys):
    root = make_checkout(
        tmp_path,
        synthetic=[
            ("mod_new", manifest_for("mod_new", deps={"apt": [], "pip": ["tabulate==0.9.0"]})),
            ("mod_old", manifest_for("mod_old", deps={"apt": [], "pip": ["tabulate==0.8.0"]})),
        ],
    )
    assert planner.main(["--project-root", str(root), "--check-conflicts"]) == 1
    out = capsys.readouterr().out
    assert "CONFLICT" in out
    assert "tabulate" in out
    assert "mod_new" in out and "mod_old" in out


def test_same_spec_and_unpinned_duplicates_pass(planner, tmp_path):
    root = make_checkout(
        tmp_path,
        synthetic=[
            ("mod_a", manifest_for("mod_a", deps={"apt": ["cowsay"], "pip": ["tabulate==0.9.0"]})),
            ("mod_b", manifest_for("mod_b", deps={"apt": ["cowsay"], "pip": ["tabulate==0.9.0"]})),
            ("mod_c", manifest_for("mod_c", deps={"apt": [], "pip": ["tabulate"]})),
        ],
    )
    assert planner.main(["--project-root", str(root), "--check-conflicts"]) == 0


def test_conflicts_on_different_hosts_do_not_fight(planner, tmp_path):
    root = make_checkout(
        tmp_path,
        synthetic=[
            ("mod_robot", manifest_for("mod_robot", targets=["robot"],
                                       deps={"apt": [], "pip": ["numpy<2"]})),
            ("mod_gcs", manifest_for("mod_gcs", targets=["gcs"],
                                     deps={"apt": [], "pip": ["numpy>=2"]})),
        ],
    )
    assert planner.main(["--project-root", str(root), "--check-conflicts"]) == 0


# ── tier-3 sole-overlay rule ─────────────────────────────────────────────────

def test_sole_overlay_used_as_is(planner, tmp_path):
    ref = "ghcr.io/example/vla-overlay:0.19.0"
    root = make_checkout(
        tmp_path,
        synthetic=[("vla_planner", manifest_for("vla_planner", overlay_image=ref))],
    )
    assert planner.main(["--project-root", str(root)]) == 0
    robot = read_plan(root)["robot"]
    assert robot["final_tag"] == ref  # pulled, never rebuilt
    assert [(s["tier"], s["dockerfile"]) for s in robot["steps"]] == [(3, None)]
    assert not (root / LAYERS_REL).exists()  # nothing to build


def test_overlay_without_dockerfile_alongside_other_module_errors(planner, tmp_path, capsys):
    ref = "ghcr.io/example/vla-overlay:0.19.0"
    root = make_checkout(
        tmp_path,
        HEAVY_REL,
        synthetic=[("vla_planner", manifest_for("vla_planner", overlay_image=ref))],
    )
    assert planner.main(["--project-root", str(root)]) == 1
    out = capsys.readouterr().out
    assert "RFC #379 §6" in out
    assert "vla_planner" in out
    assert "heavy_module" in out  # names what else is composing


def test_two_overlays_error_unless_they_carry_dockerfiles(planner, tmp_path, capsys):
    root = make_checkout(
        tmp_path,
        synthetic=[
            ("ovl_a", manifest_for("ovl_a", overlay_image="ghcr.io/example/a:1")),
            ("ovl_b", manifest_for("ovl_b", overlay_image="ghcr.io/example/b:1")),
        ],
    )
    assert planner.main(["--project-root", str(root)]) == 1
    assert "RFC #379 §6" in capsys.readouterr().out


def test_overlay_with_dockerfile_composes_as_build(planner, tmp_path):
    root = make_checkout(
        tmp_path,
        HEAVY_REL,
        synthetic=[("ovl_a", manifest_for("ovl_a", overlay_image="ghcr.io/example/a:1"))],
    )
    # give the overlay module its fragment (fragment = source of truth, overlay = cache)
    dockerfile = root / "modules" / "ovl_a" / "Dockerfile.module"
    dockerfile.write_text("ARG BASE_IMAGE\nFROM ${BASE_IMAGE}\nRUN true\n", encoding="utf-8")
    manifest_path = root / "modules" / "ovl_a" / "module.yaml"
    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    data["dockerfile"] = "Dockerfile.module"
    manifest_path.write_text(yaml.safe_dump(data), encoding="utf-8")

    assert planner.main(["--project-root", str(root)]) == 0
    robot = read_plan(root)["robot"]
    assert [(s["module"], s["tier"]) for s in robot["steps"]] == [
        ("heavy_module", 1),   # tier 1 first
        ("heavy_module", 2),   # then tier-2 fragments
        ("ovl_a", 3),          # tier-3-as-build last
    ]
    assert robot["steps"][2]["dockerfile"] == os.path.join("modules", "ovl_a", "Dockerfile.module")
    assert robot["final_tag"] != robot["base_image"]


# ── dep_hash sensitivity ─────────────────────────────────────────────────────

def _lock_entry(root, name):
    return next(m for m in read_lock(root)["modules"] if m["name"] == name)


def test_dep_hash_changes_with_deps_and_not_with_metadata(planner, tmp_path):
    root = make_checkout(tmp_path, HEAVY_REL)
    assert planner.main(["--project-root", str(root)]) == 0
    baseline = _lock_entry(root, "heavy_module")["dep_hash"]

    manifest_path = root / "modules" / "heavy_module" / "module.yaml"
    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))

    # unrelated metadata change → hash stable
    data["description"] = "A different description, same dependencies."
    data["maintainer"] = "other@example.com"
    manifest_path.write_text(yaml.safe_dump(data), encoding="utf-8")
    assert planner.main(["--project-root", str(root)]) == 0
    assert _lock_entry(root, "heavy_module")["dep_hash"] == baseline

    # dep change → hash moves (and so does the plan hash / final tag)
    old_plan_hash = read_lock(root)["plan_hash"]
    data["deps"] = {"apt": ["cowsay"], "pip": ["tabulate", "rich"]}
    manifest_path.write_text(yaml.safe_dump(data), encoding="utf-8")
    assert planner.main(["--project-root", str(root)]) == 0
    assert _lock_entry(root, "heavy_module")["dep_hash"] != baseline
    assert read_lock(root)["plan_hash"] != old_plan_hash


def test_dep_hash_changes_when_dockerfile_bytes_change(planner, tmp_path):
    root = make_checkout(tmp_path, HEAVY_REL)
    assert planner.main(["--project-root", str(root)]) == 0
    baseline = _lock_entry(root, "heavy_module")["dep_hash"]

    dockerfile = root / "modules" / "heavy_module" / "Dockerfile.module"
    dockerfile.write_text(dockerfile.read_text(encoding="utf-8") + "RUN true\n",
                          encoding="utf-8")
    assert planner.main(["--project-root", str(root)]) == 0
    assert _lock_entry(root, "heavy_module")["dep_hash"] != baseline


# ── sync integration (shells the real airstack.sh, like the overlay tests) ───

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
    for tool in ("module_overlay.py", "validate_module.py", "compose_module_layers.py"):
        shutil.copy(REPO / "tools" / tool, sb / "tools" / tool)
    shutil.copytree(REPO / "common" / "module_schema", sb / "common" / "module_schema")
    for rel in (HELLO_REL, HEAVY_REL):
        shutil.copytree(REPO / rel, sb / rel, ignore=shutil.ignore_patterns("__pycache__"))

    (sb / ".env").write_text(ENV_TEXT, encoding="utf-8")
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


def test_sync_runs_planner_and_logs_identity_summary(sandbox):
    _, out = run_airstack(sandbox, "module", "add", HELLO_REL)
    assert "0 docker-relevant modules — base images unchanged" in out
    assert (sandbox / PLAN_REL).is_file()
    assert (sandbox / LOCK_REL).is_file()
    # identity: no image: overrides in the generated compose
    compose = yaml.safe_load((sandbox / COMPOSE_REL).read_text(encoding="utf-8"))
    for definition in compose["services"].values():
        assert "image" not in definition


def test_sync_fails_on_dependency_conflict(sandbox):
    conflicted = sandbox / "conflicted_module"
    conflicted.mkdir()
    (conflicted / "module.yaml").write_text(
        yaml.safe_dump(manifest_for(
            "conflicted_module", deps={"apt": [], "pip": ["tabulate==0.8.0"]})),
        encoding="utf-8",
    )
    heavy = sandbox / HEAVY_REL
    heavy_manifest = yaml.safe_load((heavy / "module.yaml").read_text(encoding="utf-8"))
    heavy_manifest["deps"] = {"apt": ["cowsay"], "pip": ["tabulate==0.9.0"]}
    (heavy / "module.yaml").write_text(yaml.safe_dump(heavy_manifest), encoding="utf-8")

    run_airstack(sandbox, "module", "add", HEAVY_REL)  # alone: fine
    code, out = run_airstack(sandbox, "module", "add", "conflicted_module", check=False)
    assert code != 0
    assert "CONFLICT" in out
    assert "heavy_module" in out and "conflicted_module" in out


def test_module_lock_subcommand(sandbox):
    run_airstack(sandbox, "module", "add", HEAVY_REL)
    lock_before = (sandbox / LOCK_REL).read_bytes()
    (sandbox / LOCK_REL).unlink()
    _, out = run_airstack(sandbox, "module", "lock")
    assert (sandbox / LOCK_REL).read_bytes() == lock_before


def test_remove_cleans_layer_artifacts(sandbox):
    run_airstack(sandbox, "module", "add", HEAVY_REL)
    assert (sandbox / PLAN_REL).is_file() and (sandbox / LOCK_REL).is_file()
    run_airstack(sandbox, "module", "remove", "heavy_module")
    assert not (sandbox / PLAN_REL).exists()
    assert not (sandbox / LOCK_REL).exists()
    assert not (sandbox / LAYERS_REL).exists()
    assert not (sandbox / ".airstack" / "generated").exists()
