# Copyright (c) 2026 Carnegie Mellon University
# MIT License - see LICENSE in the repository root for full text.
"""Contract tests for `airstack up` launch-intent flags (--sim/--robots/...).

`airstack up --dry-run` derives the launch configuration (compose profiles,
URDF, Isaac script selection, robot count), runs the preflight checks, prints
the effective config between marker lines, and exits without starting
services. These tests pin that contract: the derivations the flags promise,
the preflight guards, and the exit codes.

They shell the real ./airstack.sh (no mocking) but never start containers —
--dry-run stops before compose up. Docker itself is required (the preflight
image check runs `docker compose config`), which CI's ubuntu-latest provides.
"""
import os
import subprocess
from pathlib import Path

import pytest

from harness.discovery import TESTS_DIR

pytestmark = pytest.mark.unit

REPO = TESTS_DIR.parent
AIRSTACK = str(REPO / "airstack.sh")

CONFIG_BEGIN = "--- effective launch config ---"
CONFIG_END = "--- end effective launch config ---"


def run_up_dry(*flags, env=None, check=True):
    """Run `airstack up --dry-run <flags>`; return (exit_code, stdout+stderr, config_dict)."""
    # Scrub AUTONOMY_ROLE from the invoking shell by default: it was removed
    # (preflight hard-errors on it) and must only be set by tests that pin
    # exactly that error.
    full_env = {**os.environ, "AUTONOMY_ROLE": "", **(env or {})}
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


def test_sim_airsim_derives_profile_and_urdf():
    _, _, cfg = run_up_dry("--sim", "airsim")
    assert "ms-airsim" in cfg["COMPOSE_PROFILES"].split(",")
    assert "isaac-sim" not in cfg["COMPOSE_PROFILES"]
    assert cfg["URDF_FILE"].endswith("iris_stereo.ms-airsim.urdf")


def test_sim_isaac_derives_profile_and_urdf():
    _, _, cfg = run_up_dry("--sim", "isaac")
    assert "isaac-sim" in cfg["COMPOSE_PROFILES"].split(",")
    assert "ms-airsim" not in cfg["COMPOSE_PROFILES"]
    assert cfg["URDF_FILE"].endswith("iris_with_sensors.pegasus.robot.urdf")


def test_sim_preserves_non_sim_profiles():
    """--sim swaps only the simulator profile; 'desktop' etc. survive."""
    _, _, cfg = run_up_dry("--sim", "airsim")
    assert "desktop" in cfg["COMPOSE_PROFILES"].split(",")


def test_robots_selects_multi_script_on_isaac():
    _, _, cfg = run_up_dry("--sim", "isaac", "--robots", "3")
    assert cfg["NUM_ROBOTS"] == "3"
    assert cfg["ISAAC_SIM_SCRIPT_NAME"] == "example_multi_px4_pegasus_launch_script.py"


def test_robots_1_selects_single_script_even_if_env_says_multi():
    _, _, cfg = run_up_dry(
        "--sim", "isaac", "--robots", "1",
        env={"ISAAC_SIM_SCRIPT_NAME": "example_multi_px4_pegasus_launch_script.py"},
    )
    assert cfg["ISAAC_SIM_SCRIPT_NAME"] == "example_one_px4_pegasus_launch_script.py"


def test_robots_never_overrides_custom_script():
    code, out, cfg = run_up_dry(
        "--sim", "isaac", "--robots", "2",
        env={"ISAAC_SIM_SCRIPT_NAME": "my_custom_scene.py"},
    )
    assert cfg["ISAAC_SIM_SCRIPT_NAME"] == "my_custom_scene.py"
    assert "custom" in out.lower() or "my_custom_scene.py" in out  # warns, keeps


def test_num_robots_single_script_mismatch_is_fatal():
    """NUM_ROBOTS>1 with the single-drone script spawns 1 drone silently — hard error."""
    code, out, _ = run_up_dry(
        env={
            "NUM_ROBOTS": "3",
            "COMPOSE_PROFILES": "desktop,isaac-sim",
            "ISAAC_SIM_SCRIPT_NAME": "example_one_px4_pegasus_launch_script.py",
        },
        check=False,
    )
    assert code != 0
    assert "ISAAC_SIM_SCRIPT_NAME" in out or "--robots" in out


def test_one_sim_guard_sees_env_file_overrides():
    """--env-file overrides/ms-airsim.env swaps the profile; must NOT trip the
    two-sims guard against .env's isaac-sim (the historical bypass bug)."""
    code, out, cfg = run_up_dry("--env-file", "overrides/ms-airsim.env")
    assert code == 0, out
    assert "ms-airsim" in cfg["COMPOSE_PROFILES"]
    assert "isaac-sim" not in cfg["COMPOSE_PROFILES"]


def test_two_sim_profiles_is_fatal():
    code, out, _ = run_up_dry(
        env={"COMPOSE_PROFILES": "desktop,isaac-sim,ms-airsim"}, check=False,
    )
    assert code != 0
    assert "one simulator" in out.lower()


def test_headless_and_play_flags():
    _, _, cfg = run_up_dry("--sim", "isaac", "--headless", "--no-play")
    assert cfg["ISAAC_SIM_HEADLESS"] == "true"
    assert cfg["MS_AIRSIM_HEADLESS"] == "true"
    assert cfg["PLAY_SIM_ON_START"] == "false"


def test_no_autolaunch_flag():
    _, _, cfg = run_up_dry("--sim", "isaac", "--no-autolaunch")
    assert cfg["AUTOLAUNCH"] == "false"


def test_flagless_dry_run_reflects_env_defaults():
    """No flags → today's behavior: .env values pass through untouched."""
    _, _, cfg = run_up_dry()
    env_profiles = None
    for line in (REPO / ".env").read_text().splitlines():
        if line.startswith("COMPOSE_PROFILES="):
            env_profiles = line.split("=", 1)[1].split("#")[0].strip().strip('"')
    assert env_profiles is not None
    assert cfg["COMPOSE_PROFILES"] == env_profiles


def test_effective_config_dump_written():
    if not os.access(REPO, os.W_OK):
        pytest.skip("checkout mounted read-only (tests container) — dump is best-effort")
    runs_dir = REPO / ".airstack" / "runs"
    before = set(runs_dir.glob("*/effective_config.env")) if runs_dir.exists() else set()
    run_up_dry("--sim", "isaac")
    after = set(runs_dir.glob("*/effective_config.env"))
    new = after - before
    assert new, "dry-run did not write an effective_config.env under .airstack/runs/"
    content = max(new, key=lambda p: p.stat().st_mtime).read_text()
    assert "COMPOSE_PROFILES=" in content


def test_invalid_sim_is_fatal():
    code, out, _ = run_up_dry("--sim", "gazebo", check=False)
    assert code != 0
    assert "gazebo" in out


# ── --stack dispatch (RFC #379 §3, P5-E1) ──────────────────────────────────

def test_stack_flag_exports_container_paths():
    """--stack validates host-side but exports the CONTAINER path (stacks/ is
    bind-mounted at /root/AirStack/stacks)."""
    _, _, cfg = run_up_dry("--sim", "isaac", "--stack", "full_default")
    assert cfg["AIRSTACK_STACK_DIR"] == "/root/AirStack/stacks/full_default"
    assert cfg["AIRSTACK_STACK_ENTRY"] == "stack"


def test_stack_split_entry_form():
    """--stack <name>:<entry> selects launch/<entry>.launch.xml (reserved for
    split stacks; the default entry file also resolves through it)."""
    _, _, cfg = run_up_dry("--sim", "isaac", "--stack", "full_default:stack")
    assert cfg["AIRSTACK_STACK_DIR"] == "/root/AirStack/stacks/full_default"
    assert cfg["AIRSTACK_STACK_ENTRY"] == "stack"


def test_unknown_stack_is_fatal():
    code, out, _ = run_up_dry("--stack", "no_such_stack", check=False)
    assert code != 0
    assert "no_such_stack" in out
    assert "full_default" in out  # error lists the available stacks


def test_stack_missing_entry_is_fatal():
    code, out, _ = run_up_dry("--stack", "full_default:onboard", check=False)
    assert code != 0
    assert "onboard.launch.xml" in out


def test_no_stack_defaults_to_full_default():
    """Stacks are the only dispatch: no --stack (and no stack env) → the
    effective config names the trunk reference stack full_default."""
    _, _, cfg = run_up_dry(
        "--sim", "isaac",
        # Scrub any stack vars inherited from the invoking shell.
        env={"AIRSTACK_STACK_DIR": "", "AIRSTACK_STACK_ENTRY": ""},
    )
    assert cfg["AIRSTACK_STACK_DIR"] == "/root/AirStack/stacks/full_default"
    assert cfg["AIRSTACK_STACK_ENTRY"] == "stack"


def test_autonomy_role_set_is_fatal():
    """AUTONOMY_ROLE was removed — an explicitly set value (env / --env-file /
    .env) hard-fails preflight with the removal message."""
    code, out, _ = run_up_dry(
        "--sim", "isaac",
        env={"AUTONOMY_ROLE": "full", "AIRSTACK_STACK_DIR": ""},
        check=False,
    )
    assert code != 0
    assert "AUTONOMY_ROLE was removed" in out
    assert "--stack" in out


def test_empty_autonomy_role_does_not_trip_removal_error():
    """Only an explicitly SET value trips the removal error — an empty/unset
    AUTONOMY_ROLE must pass."""
    code, out, _ = run_up_dry(
        "--sim", "isaac",
        env={"AUTONOMY_ROLE": "", "AIRSTACK_STACK_DIR": ""},
    )
    assert code == 0
    assert "AUTONOMY_ROLE was removed" not in out


def test_autonomy_role_fatal_even_with_stack_selected():
    """The removal error is unconditional: a stale AUTONOMY_ROLE next to a
    valid --stack still fails (no silent 'stack wins' anymore)."""
    code, out, _ = run_up_dry(
        "--sim", "isaac", "--stack", "full_default",
        env={"AUTONOMY_ROLE": "full"},
        check=False,
    )
    assert code != 0
    assert "AUTONOMY_ROLE was removed" in out


# ── override-file golden equivalence (RFC #380 P6, deliverable 8) ───────────
# overrides/*.env select sims/hardware, not topology — they must keep passing
# `up --dry-run` unchanged as the fleet/stack machinery lands on top of them.

def test_override_ms_airsim_env_still_derives_expected_config():
    code, out, cfg = run_up_dry(
        "--env-file", "overrides/ms-airsim.env",
        # Scrub stack/fleet vars a developer shell might carry.
        env={"AIRSTACK_STACK_DIR": "", "FLEET_CONFIG_FILE": ""},
    )
    assert code == 0, out
    profiles = cfg["COMPOSE_PROFILES"].split(",")
    assert "ms-airsim" in profiles and "desktop" in profiles
    assert "isaac-sim" not in profiles
    assert cfg["URDF_FILE"].endswith("iris_stereo.ms-airsim.urdf")
    # a sim override selects no stack of its own → the full_default default
    assert cfg["AIRSTACK_STACK_DIR"] == "/root/AirStack/stacks/full_default"
    # sim/hardware override files never opt into fleets on their own
    assert "FLEET_CONFIG_FILE" not in cfg


def test_override_l4t_px4_realrobot_env_still_derives_expected_config():
    code, out, cfg = run_up_dry(
        "--env-file", "overrides/l4t-px4-realrobot.env",
        # Scrub stack/fleet vars a developer shell might carry.
        env={"AIRSTACK_STACK_DIR": "", "FLEET_CONFIG_FILE": ""},
    )
    assert code == 0, out
    assert cfg["COMPOSE_PROFILES"] == "l4t"
    assert cfg["NUM_ROBOTS"] == "1"
    assert cfg["URDF_FILE"].endswith("iris_with_sensors.pegasus.robot.urdf")
    assert "FLEET_CONFIG_FILE" not in cfg
    # the override file is stack-form now (AUTONOMY_ROLE was removed): it
    # pins the full stack explicitly and must not draw the removal error
    assert cfg["AIRSTACK_STACK_DIR"] == "/root/AirStack/stacks/full_default"
    assert "AUTONOMY_ROLE was removed" not in out
