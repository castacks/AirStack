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


def test_robots_respects_natnet_script_pair():
    _, _, cfg = run_up_dry(
        "--sim", "isaac", "--robots", "2",
        env={"ISAAC_SIM_SCRIPT_NAME": "example_one_px4_pegasus_natnet_launch_script.py"},
    )
    assert cfg["ISAAC_SIM_SCRIPT_NAME"] == "example_multi_px4_pegasus_natnet_launch_script.py"


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


def test_no_stack_leaves_stack_vars_empty():
    """Legacy path: no --stack → both stack vars empty in effective config."""
    _, _, cfg = run_up_dry(
        "--sim", "isaac",
        # Scrub any stack vars inherited from the invoking shell.
        env={"AIRSTACK_STACK_DIR": "", "AIRSTACK_STACK_ENTRY": ""},
    )
    assert cfg.get("AIRSTACK_STACK_DIR", "") == ""


def test_autonomy_role_without_stack_warns_deprecation():
    """AUTONOMY_ROLE explicitly set (env/--env-file) + no stack → one
    deprecation warning pointing at --stack."""
    _, out, _ = run_up_dry(
        "--sim", "isaac",
        env={"AUTONOMY_ROLE": "full", "AIRSTACK_STACK_DIR": ""},
    )
    assert "legacy dispatch" in out
    assert "--stack full_default" in out


def test_no_deprecation_warning_without_explicit_role():
    """The compose files' own ${AUTONOMY_ROLE:-full} default must NOT trip the
    warning — only an explicit env / --env-file / .env value does."""
    _, out, _ = run_up_dry(
        "--sim", "isaac",
        env={"AUTONOMY_ROLE": "", "AIRSTACK_STACK_DIR": ""},
    )
    assert "legacy dispatch" not in out


def test_stack_and_role_both_set_warns_stack_wins():
    _, out, _ = run_up_dry(
        "--sim", "isaac", "--stack", "full_default",
        env={"AUTONOMY_ROLE": "full"},
    )
    assert "stack wins" in out
    assert "legacy dispatch" not in out
