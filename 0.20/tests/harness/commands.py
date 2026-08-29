"""Subprocess / docker-exec / ros2 command helpers with per-test output capture.

Every subprocess runs through ``_run_teed``, which records combined stdout+stderr in the
session so ``read_log_tail`` can surface it in failure messages. ``current_log`` ties
output to the running test's id so callers don't have to plumb a log name through every
layer.
"""
import os
import re
import shlex
import subprocess
from pathlib import Path

from harness.discovery import AIRSTACK_ROOT
from harness.session import current_item, last_cmd_output, logger, record_cmd_output

ROS_DISTRO_SETUP = "/opt/ros/jazzy/setup.bash"


def _nodeid_dotted(nodeid, with_path_sep=False):
    """pytest nodeid → `module.Class.test_name[params]` form. When
    `with_path_sep=True`, also flattens `/` in path prefixes (for log filenames)."""
    out = nodeid.replace(".py::", ".").replace("::", ".")
    return out.replace("/", ".") if with_path_sep else out


def current_log():
    """Log name for the currently-running pytest item, or None outside a test.

    Subprocess helpers default to this so every call fired from a test auto-logs
    to the right file without plumbing log_name through every layer."""
    item = current_item()
    if item is None:
        return None
    return _nodeid_dotted(item.nodeid, with_path_sep=True)


def read_log_tail(log_name=None, lines=50):
    """Return the tail of the most recent subprocess output for this context."""
    text = last_cmd_output(log_name)
    if not text:
        return ""
    return "\n".join(text.splitlines()[-lines:])


def _run_teed(cmd_list, timeout, log_name=None, env=None, cwd=None):
    """Run a subprocess and capture stdout+stderr for parsing and failure messages."""
    quoted = " ".join(shlex.quote(a) for a in cmd_list)
    logger.info("$ %s", quoted)
    result = subprocess.run(
        cmd_list, capture_output=True, text=True, timeout=timeout, env=env, cwd=cwd,
    )
    combined = (result.stdout or "") + (result.stderr or "")
    record_cmd_output(combined, log_name, quoted)
    return result


def docker_exec(container, cmd, timeout=60, log_name=None, env=None):
    full_cmd = ["docker", "exec"]
    if env:
        for key, value in env.items():
            full_cmd.extend(["-e", f"{key}={value}"])
    full_cmd.extend([container, "bash", "-c", cmd])
    return _run_teed(full_cmd, timeout=timeout, log_name=log_name)


def airstack_cmd(*args, env_overrides=None, timeout=1800, log_name=None):
    env = os.environ.copy()
    if env_overrides:
        env.update(env_overrides)
    cmd = [str(Path(AIRSTACK_ROOT) / "airstack.sh")] + list(args)
    return _run_teed(cmd, timeout=timeout, log_name=log_name,
                     env=env, cwd=AIRSTACK_ROOT)


def ros2_env(setup_bash, domain_id):
    """Shell prefix that makes `ros2` available on the requested domain."""
    return (
        f"source {ROS_DISTRO_SETUP} && source {setup_bash} "
        f"&& export ROS_DOMAIN_ID={domain_id}"
    )


def ros2_exec(container, ros2_cmd, domain_id=0, setup_bash=None, timeout=15, log_name=None):
    """Run `ros2 ...` inside a container with the right workspace sourced."""
    setup = setup_bash or "/root/AirStack/robot/ros_ws/install/setup.bash"
    inner = f"{ros2_env(setup, domain_id)} && {ros2_cmd}"
    return docker_exec(container, inner, timeout=timeout, log_name=log_name)


_HZ_RE = re.compile(r"average rate:\s+([\d.]+)")


def _parse_hz(text):
    m = _HZ_RE.search(text or "")
    return float(m.group(1)) if m else None
