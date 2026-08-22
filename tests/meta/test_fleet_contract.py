# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Contract tests for fleets (RFC #380 §2, Phase P6).

Pins the promises the fleet machinery makes:

- **Resolver parity** — ``sim_one_default`` resolves ``robot_1`` to exactly
  what the legacy ``robot_name_map`` resolver produces for
  ``airstack-robot-desktop-1`` (same ROBOT_NAME / ROS_DOMAIN_ID): opting into
  the fleet changes nothing for today's default checkout.
- **Generation** — ``fleet generate`` is deterministic (byte-identical
  re-runs), detects homogeneity (writes nothing for replica-able fleets), and
  emits the SPLIT placement: robot_3's ground host gets a service running the
  same split stack with ``AIRSTACK_STACK_ENTRY=offboard``.
- **The trajectory hard-gate holds through fleet placement** — every split
  stack the generated compose places passes ``gen_dds_router.py --check``
  (doctor hard gate #2: command authority stays onboard).
- **Launch intent** — ``airstack up --dry-run --fleet`` exports
  FLEET_CONFIG_FILE + derived NUM_ROBOTS + the fleet spawner; explicit env
  NUM_ROBOTS beats the fleet (banner); no fleet ⇒ no new effective-config
  keys (byte-identical legacy contract).
- **Schema errors are named** — bad hosts/stack/robots produce errors naming
  the offender.
- **Spawner mapping** — ``fleet_spawn.py``'s fleet→drone-config mapping is a
  pure stdlib function (no Isaac import at module scope) and matches the
  fleet file.
"""
import importlib.util
import os
import subprocess
import sys

import pytest
import yaml

from harness.discovery import repo_path

pytestmark = pytest.mark.unit

REPO = repo_path()
AIRSTACK = str(REPO / "airstack.sh")
RESOLVER = REPO / "tools" / "fleet" / "resolve_fleet.py"
GENERATOR = REPO / "tools" / "fleet" / "generate_fleet_compose.py"
GEN_DDS_ROUTER = REPO / "tools" / "gen_dds_router.py"
LEGACY_RESOLVER = REPO / "robot" / "docker" / "robot_name_map" / "resolve_robot_name.py"
LEGACY_MAP = REPO / "robot" / "docker" / "robot_name_map" / "default_robot_name_map.yaml"
FLEETS = REPO / "config" / "fleets"

CONFIG_BEGIN = "--- effective launch config ---"
CONFIG_END = "--- end effective launch config ---"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def rf():
    return _load(RESOLVER, "airstack_resolve_fleet")


@pytest.fixture(scope="module")
def legacy():
    return _load(LEGACY_RESOLVER, "airstack_resolve_robot_name")


def run_tool(script, *args):
    result = subprocess.run(
        [sys.executable, str(script), *args],
        capture_output=True, text=True, cwd=str(REPO), timeout=60,
    )
    return result.returncode, result.stdout, result.stderr


def run_up_dry(*flags, env=None, check=True):
    full_env = {**os.environ, **(env or {})}
    result = subprocess.run(
        [AIRSTACK, "up", "--dry-run", *flags],
        capture_output=True, text=True, cwd=str(REPO), env=full_env, timeout=120,
    )
    out = result.stdout + result.stderr
    cfg = {}
    in_cfg = False
    for line in out.splitlines():
        line = line.strip()
        if line == CONFIG_BEGIN:
            in_cfg = True
            continue
        if line == CONFIG_END:
            in_cfg = False
            continue
        if in_cfg and "=" in line:
            key, _, value = line.partition("=")
            cfg[key] = value
    if check:
        assert result.returncode == 0, f"dry-run failed unexpectedly:\n{out}"
        assert cfg, f"no effective-config block in output:\n{out}"
    return result.returncode, out, cfg


# ── resolver parity with the legacy robot_name_map ──────────────────────────

def test_sim_one_default_matches_legacy_resolver(rf, legacy):
    """Fleet resolution of today's default fleet == legacy resolver output for
    the default single-robot container name."""
    legacy_name, legacy_domain = legacy.resolve_robot_name(
        "airstack-robot-desktop-1", str(LEGACY_MAP)
    )
    fleet = rf.load_fleet(FLEETS / "sim_one_default.yaml")
    key = rf.resolve_identity(fleet, "airstack-robot-desktop-1")
    resolved = rf.resolve_robot(fleet, REPO, key)
    assert resolved["robot_name"] == legacy_name == "robot_1"
    assert str(resolved["domain_id"]) == legacy_domain == "1"


def test_replica_indices_match_legacy_for_three_robots(rf, legacy):
    fleet = rf.load_fleet(FLEETS / "sim_three_mixed.yaml")
    for i in (1, 2, 3):
        container = f"airstack-robot-desktop-{i}"
        legacy_name, legacy_domain = legacy.resolve_robot_name(container, str(LEGACY_MAP))
        key = rf.resolve_identity(fleet, container)
        resolved = rf.resolve_robot(fleet, REPO, key)
        assert resolved["robot_name"] == legacy_name
        assert str(resolved["domain_id"]) == legacy_domain


def test_resolver_exports_full_entry(rf):
    """The exports carry the whole fleet entry, not just name+domain."""
    fleet = rf.load_fleet(FLEETS / "sim_three_mixed.yaml")
    r3 = rf.resolve_robot(fleet, REPO, "robot_3")
    assert r3["stack"] == "stacks/lite_offload_global"
    assert r3["entry"] == "onboard"          # hosts: ⇒ the robot runs the onboard half
    assert r3["vehicle"] == "quad_default"
    assert r3["urdf_file"].endswith("iris_with_sensors.pegasus.robot.urdf")
    assert r3["hosts"] == {"offboard": "gcs"}
    r2 = rf.resolve_robot(fleet, REPO, "robot_2")
    assert r2["stack"] == "stacks/lite_default"
    assert r2["entry"] == "stack"


# ── fleet generate: determinism, homogeneity, split placement ───────────────

@pytest.fixture(scope="module")
def gen():
    return _load(GENERATOR, "airstack_generate_fleet_compose")


def test_generate_is_deterministic(gen, rf):
    fleet = rf.load_fleet(FLEETS / "sim_three_mixed.yaml")
    first = gen.render(gen.build_compose(fleet, REPO, "config/fleets/sim_three_mixed.yaml"))
    second = gen.render(gen.build_compose(fleet, REPO, "config/fleets/sim_three_mixed.yaml"))
    assert first == second
    assert "generated" in first.lower()
    yaml.safe_load(first)  # parses


def test_homogeneous_fleet_needs_no_generation(gen, rf):
    fleet = rf.load_fleet(FLEETS / "sim_one_default.yaml")
    assert rf.fleet_is_homogeneous(fleet, REPO)
    code, out, err = run_tool(GENERATOR, str(FLEETS / "sim_one_default.yaml"),
                              "--project-root", str(REPO), "--check-homogeneous")
    assert code == 0 and out.strip() == "homogeneous", err
    code, out, _ = run_tool(GENERATOR, str(FLEETS / "sim_one_default.yaml"),
                            "--project-root", str(REPO))
    assert code == 0
    assert "NUM_ROBOTS=1" in out and "No generation needed" in out


def test_mixed_fleet_is_heterogeneous(gen, rf):
    fleet = rf.load_fleet(FLEETS / "sim_three_mixed.yaml")
    assert not rf.fleet_is_homogeneous(fleet, REPO)


def test_generated_split_placement(gen, rf):
    """robot_3's onboard half + its ground host's offboard half, from one file."""
    fleet = rf.load_fleet(FLEETS / "sim_three_mixed.yaml")
    compose = gen.build_compose(fleet, REPO, "config/fleets/sim_three_mixed.yaml")
    services = compose["services"]
    assert set(services) == {"robot_1", "robot_2", "robot_3", "gcs-robot_3"}

    def env_of(svc):
        return dict(e.split("=", 1) for e in services[svc]["environment"] if "=" in e)

    onboard = env_of("robot_3")
    assert onboard["AIRSTACK_STACK_DIR"] == "/root/AirStack/stacks/lite_offload_global"
    assert onboard["AIRSTACK_STACK_ENTRY"] == "onboard"
    assert onboard["ROBOT_NAME"] == "robot_3"
    assert onboard["ROS_DOMAIN_ID"] == "3"

    offboard = env_of("gcs-robot_3")
    assert offboard["AIRSTACK_STACK_DIR"] == "/root/AirStack/stacks/lite_offload_global"
    assert offboard["AIRSTACK_STACK_ENTRY"] == "offboard"
    assert offboard["ROBOT_NAME"] == "robot_3"    # serves this tenant
    assert offboard["ROS_DOMAIN_ID"] == "0"       # the fleet's gcs_domain
    assert offboard["LAUNCH_PACKAGE"] == "autonomy_bringup"
    assert "ports" not in services["gcs-robot_3"]  # mirrors legacy robot-offboard

    # every service is self-contained: explicit identity + fleet env
    for name in services:
        env = env_of(name)
        assert env["FLEET_CONFIG_FILE"] == "/root/AirStack/config/fleets/sim_three_mixed.yaml"
        assert "ROBOT_NAME" in env and "AIRSTACK_STACK_DIR" in env
        assert "extends" not in services[name]


def test_split_stacks_placed_by_fleet_pass_bridge_hard_gate(gen, rf):
    """Doctor hard gate #2 must hold for every split stack the generated
    compose places: gen_dds_router.py --check exits 0 on its bridge.yaml."""
    fleet = rf.load_fleet(FLEETS / "sim_three_mixed.yaml")
    compose = gen.build_compose(fleet, REPO, "config/fleets/sim_three_mixed.yaml")
    split_stacks = set()
    for svc in compose["services"].values():
        env = dict(e.split("=", 1) for e in svc["environment"] if "=" in e)
        if env.get("AIRSTACK_STACK_ENTRY", "stack") != "stack":
            split_stacks.add(env["AIRSTACK_STACK_DIR"].replace("/root/AirStack/", ""))
    assert split_stacks == {"stacks/lite_offload_global"}
    for stack_rel in split_stacks:
        bridge = REPO / stack_rel / "bridge.yaml"
        assert bridge.is_file(), f"split stack {stack_rel} has no bridge.yaml"
        code, out, err = run_tool(GEN_DDS_ROUTER, str(bridge), "--check",
                                  "--project-root", str(REPO))
        assert code == 0, f"bridge gate failed for {stack_rel}:\n{out}\n{err}"


# ── launch intent: --fleet dry-run exports + precedence ─────────────────────

def test_dry_run_fleet_exports():
    code, out, cfg = run_up_dry("--fleet", "sim_one_default", "--sim", "isaac")
    assert code == 0, out
    assert cfg["FLEET_CONFIG_FILE"] == "/root/AirStack/config/fleets/sim_one_default.yaml"
    assert cfg["NUM_ROBOTS"] == "1"
    assert cfg["ISAAC_SIM_SCRIPT_NAME"] == "fleet_spawn.py"
    assert "robot_1" in out  # the resolved robot table prints


def test_dry_run_heterogeneous_fleet_swaps_profile_and_would_generate():
    """--dry-run derives the fleet config but WRITES NOTHING: the generator
    prints what it would generate (compose services + split-stack routers)."""
    code, out, cfg = run_up_dry("--fleet", "sim_three_mixed", "--sim", "isaac")
    assert code == 0, out
    assert cfg["NUM_ROBOTS"] == "3"
    profiles = cfg["COMPOSE_PROFILES"].split(",")
    assert "fleet" in profiles and "desktop" not in profiles
    assert "isaac-sim" in profiles
    assert "docker-compose.fleet.yaml" in out
    assert "Would write" in out
    # split-stack routers are part of the same one-pipeline messaging
    assert "Would generate DDS-router config" in out
    assert "lite_offload_global" in out


def test_fleet_generate_cli_writes_compose_and_split_stack_routers():
    """The `airstack fleet generate` path (the real-run pipeline) emits BOTH
    the per-robot compose services and the DDS-router configs for every split
    stack the fleet places."""
    result = subprocess.run(
        [AIRSTACK, "fleet", "generate", "sim_three_mixed"],
        capture_output=True, text=True, cwd=str(REPO), timeout=120,
    )
    out = result.stdout + result.stderr
    assert result.returncode == 0, out
    assert "docker-compose.fleet.yaml" in out
    assert "dds_router.lite_offload_global.yaml" in out

    generated = REPO / ".airstack" / "generated" / "docker-compose.fleet.yaml"
    assert generated.is_file()
    assert yaml.safe_load(generated.read_text())["x-airstack-fleet"] == (
        "config/fleets/sim_three_mixed.yaml"
    )
    router = REPO / ".airstack" / "generated" / "dds_router.lite_offload_global.yaml"
    assert router.is_file()
    assert "allowlist:" in router.read_text()


def _write_external_split_checkout(tmp_path):
    """Synthetic checkout: an <alias>/<stack> EXTERNAL split stack (fetched
    into stacks/.external/ by `airstack sync`) placed by a fleet."""
    root = tmp_path / "checkout"
    veh = root / "config" / "vehicles" / "quadx"
    veh.mkdir(parents=True)
    (veh / "vehicle.yaml").write_text(
        "airframe: {base_urdf: robot_descriptions/x/x.urdf}\n", encoding="utf-8"
    )
    stack = root / "stacks" / ".external" / "ext" / "split_x"
    (stack / "launch").mkdir(parents=True)
    (stack / "launch" / "onboard.launch.xml").write_text("<launch/>\n")
    (stack / "launch" / "offboard.launch.xml").write_text("<launch/>\n")
    (stack / "bridge.yaml").write_text(
        "stack: split_x\n"
        "bridge:\n"
        "  - topic: odometry\n"
        "    type: nav_msgs/msg/Odometry\n"
        "    direction: onboard_to_offboard\n"
        "    qos: reliable\n",
        encoding="utf-8",
    )
    fleets = root / "config" / "fleets"
    fleets.mkdir(parents=True)
    fleet_path = fleets / "ext_split.yaml"
    fleet_path.write_text(
        "defaults: {vehicle: quadx}\n"
        "robots:\n"
        "  r1: {stack: ext/split_x, hosts: {offboard: gcs}}\n"
        "ground:\n"
        "  gcs: {}\n",
        encoding="utf-8",
    )
    return root, fleet_path


def test_generator_emits_router_for_external_alias_split_stack(tmp_path):
    """Resolve-aware router generation: a split stack referenced as
    <alias>/<stack> (stacks/.external/) still gets its DDS-router config."""
    root, fleet_path = _write_external_split_checkout(tmp_path)
    code, out, err = run_tool(GENERATOR, str(fleet_path), "--project-root", str(root))
    assert code == 0, err
    assert (root / ".airstack" / "generated" / "docker-compose.fleet.yaml").is_file()
    router = root / ".airstack" / "generated" / "dds_router.split_x.yaml"
    assert router.is_file(), out
    assert "dds_router.split_x.yaml" in out
    assert "rt/$(env ROBOT_NAME)/odometry" in router.read_text()


def test_generator_dry_run_writes_nothing_for_external_alias_split_stack(tmp_path):
    root, fleet_path = _write_external_split_checkout(tmp_path)
    code, out, err = run_tool(GENERATOR, str(fleet_path),
                              "--project-root", str(root), "--dry-run")
    assert code == 0, err
    assert "Would write" in out and "Would generate DDS-router config" in out
    assert not (root / ".airstack" / "generated" / "docker-compose.fleet.yaml").exists()
    assert not (root / ".airstack" / "generated" / "dds_router.split_x.yaml").exists()


def test_explicit_num_robots_beats_fleet_with_banner():
    code, out, cfg = run_up_dry("--fleet", "sim_one_default", "--sim", "isaac",
                                env={"NUM_ROBOTS": "5"})
    assert code == 0, out
    assert cfg["NUM_ROBOTS"] == "5"
    assert "OVERRIDE" in out and "NUM_ROBOTS=5" in out


def test_explicit_isaac_script_beats_fleet_spawner():
    _, out, cfg = run_up_dry(
        "--fleet", "sim_one_default", "--sim", "isaac",
        env={"ISAAC_SIM_SCRIPT_NAME": "my_custom_scene.py"},
    )
    assert cfg["ISAAC_SIM_SCRIPT_NAME"] == "my_custom_scene.py"
    assert "OVERRIDE" in out


def test_fleet_and_robots_flags_are_mutually_exclusive():
    code, out, _ = run_up_dry("--fleet", "sim_one_default", "--robots", "2",
                              check=False)
    assert code != 0
    assert "mutually exclusive" in out


def test_unknown_fleet_is_fatal_and_lists_available():
    code, out, _ = run_up_dry("--fleet", "no_such_fleet", check=False)
    assert code != 0
    assert "no_such_fleet" in out
    assert "sim_one_default" in out


def test_no_fleet_keeps_effective_config_key_free():
    """Byte-identical legacy contract: no fleet anywhere ⇒ no FLEET_CONFIG_FILE
    key in the effective config at all (not even empty)."""
    _, _, cfg = run_up_dry("--sim", "isaac", env={"FLEET_CONFIG_FILE": ""})
    assert "FLEET_CONFIG_FILE" not in cfg


def test_env_fleet_config_file_opts_in_like_the_flag():
    """The harness path: FLEET_CONFIG_FILE via env (container path) triggers
    the same validation + derivation as --fleet."""
    code, out, cfg = run_up_dry(
        "--sim", "isaac",
        env={"FLEET_CONFIG_FILE": "/root/AirStack/config/fleets/sim_one_default.yaml"},
    )
    assert code == 0, out
    assert cfg["FLEET_CONFIG_FILE"] == "/root/AirStack/config/fleets/sim_one_default.yaml"
    assert cfg["NUM_ROBOTS"] == "1"


# ── schema errors are named ──────────────────────────────────────────────────

def _write_fleet(tmp_path, body):
    root = tmp_path / "checkout"
    fleets = root / "config" / "fleets"
    fleets.mkdir(parents=True)
    path = fleets / "bad.yaml"
    path.write_text(body, encoding="utf-8")
    return path


def _validate(path):
    return run_tool(RESOLVER, str(path), "--project-root", str(REPO), "--validate")


def test_error_hosts_naming_missing_ground(tmp_path):
    path = _write_fleet(tmp_path, """
defaults: {vehicle: quad_default, stack: stacks/full_default}
robots:
  r1: {stack: stacks/lite_offload_global, hosts: {offboard: edge_box}}
""")
    code, _, err = _validate(path)
    assert code == 1
    assert "edge_box" in err and "ground" in err


def test_error_split_stack_without_hosts(tmp_path):
    path = _write_fleet(tmp_path, """
defaults: {vehicle: quad_default, stack: stacks/lite_offload_global}
robots:
  r1: {}
""")
    code, _, err = _validate(path)
    assert code == 1
    assert "hosts" in err and "split" in err


def test_error_unknown_stack_named(tmp_path):
    path = _write_fleet(tmp_path, """
defaults: {vehicle: quad_default, stack: stacks/no_such_stack}
robots:
  r1: {}
""")
    code, _, err = _validate(path)
    assert code == 1
    assert "no_such_stack" in err


def test_error_unknown_vehicle_named(tmp_path):
    path = _write_fleet(tmp_path, """
defaults: {vehicle: no_such_vehicle, stack: stacks/full_default}
robots:
  r1: {}
""")
    code, _, err = _validate(path)
    assert code == 1
    assert "no_such_vehicle" in err


def test_error_empty_robots(tmp_path):
    path = _write_fleet(tmp_path, "robots: {}\n")
    code, _, err = _validate(path)
    assert code == 1
    assert "robots" in err


def test_error_hosts_role_without_entry_point(tmp_path):
    """A hosts role must match a launch entry file of the robot's stack."""
    path = _write_fleet(tmp_path, """
defaults: {vehicle: quad_default, stack: stacks/lite_offload_global}
robots:
  r1: {hosts: {edge_compute: gcs}}
ground:
  gcs: {}
""")
    code, _, err = _validate(path)
    assert code == 1
    assert "edge_compute" in err and "launch/edge_compute.launch.xml" in err


def test_error_unknown_robot_key_named(tmp_path):
    path = _write_fleet(tmp_path, """
defaults: {vehicle: quad_default, stack: stacks/full_default}
robots:
  r1: {vehicel: quad_default}
""")
    code, _, err = _validate(path)
    assert code == 1
    assert "vehicel" in err


def test_error_bad_domain_policy(tmp_path):
    path = _write_fleet(tmp_path, """
defaults: {vehicle: quad_default, stack: stacks/full_default}
robots:
  r1: {}
network: {domain_policy: static}
""")
    code, _, err = _validate(path)
    assert code == 1
    assert "domain_policy" in err and "static" in err


# ── fleet spawner mapping (no Isaac needed) ──────────────────────────────────

@pytest.fixture(scope="module")
def spawner():
    return _load(
        REPO / "simulation" / "isaac-sim" / "launch_scripts" / "fleet_spawn.py",
        "airstack_fleet_spawn",
    )


def test_fleet_spawn_imports_without_isaac(spawner):
    """Module scope must stay stdlib+PyYAML (deferred-import contract)."""
    assert callable(spawner.fleet_to_drone_configs)


def test_fleet_spawn_mapping_matches_fleet(spawner):
    fleet = yaml.safe_load((FLEETS / "sim_three_mixed.yaml").read_text())
    cfgs = spawner.fleet_to_drone_configs(fleet, str(REPO))
    assert [c["domain_id"] for c in cfgs] == [1, 2, 3]
    assert [c["robot_name"] for c in cfgs] == ["robot_1", "robot_2", "robot_3"]
    assert [c["x_m"] for c in cfgs] == [-2.0, 0.0, 2.0]
    assert all(c["z_m"] == 0.07 for c in cfgs)
    assert all(c["lidar"] for c in cfgs)  # quad_default carries a lidar_3d


def test_fleet_spawn_remaps_robot_container_path(spawner):
    assert spawner.remap_fleet_path("/root/AirStack/config/fleets/f.yaml") == (
        "/isaac-sim/AirStack/config/fleets/f.yaml"
    )
    assert spawner.remap_fleet_path("/elsewhere/f.yaml") == "/elsewhere/f.yaml"


def test_fleet_spawn_scene_resolution(spawner):
    envs = {"Default Environment": "omniverse://default", "Curved Gridroom": "omniverse://curved"}
    fleet = {"sim": {"scene": "default"}}
    assert spawner.fleet_env_url(fleet, envs) == "omniverse://default"
    assert spawner.fleet_env_url({}, envs) == "omniverse://default"
    assert spawner.fleet_env_url({"sim": {"scene": "Curved Gridroom"}}, envs) == "omniverse://curved"
    assert spawner.fleet_env_url({"sim": {"scene": "/scenes/x.usd"}}, envs) == "/scenes/x.usd"
    with pytest.raises(ValueError):
        spawner.fleet_env_url({"sim": {"scene": "nope"}}, envs)
