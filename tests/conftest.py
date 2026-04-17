import inspect
import os
import subprocess
import time
from datetime import datetime
from pathlib import Path

AIRSTACK_ROOT = os.environ.get("AIRSTACK_ROOT", str(Path(__file__).parent.parent))
RUN_DIR = None
LOGS_DIR = None


def pytest_configure(config):
    global RUN_DIR, LOGS_DIR
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    results_root = Path(AIRSTACK_ROOT) / "tests" / "results"
    RUN_DIR = results_root / timestamp
    LOGS_DIR = RUN_DIR / "logs"
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    config.option.xmlpath = str(RUN_DIR / "results.xml")


# ── helpers ─────────────────────────────────────────────────────────────────

def log_name():
    frame = inspect.stack()[1]
    file = Path(frame.filename).stem
    func = frame.function
    return f"{file}.{func}"


def read_log_tail(log_name, lines=50):
    log_path = LOGS_DIR / f"{log_name}.log"
    if log_path.exists():
        all_lines = log_path.read_text().splitlines()
        return "\n".join(all_lines[-lines:])
    return ""


def docker_exec(container, cmd, timeout=60, domain_id=None, log_name=None):
    if domain_id is not None:
        cmd = f"export ROS_DOMAIN_ID={domain_id} && {cmd}"
    full_cmd = ["docker", "exec", container, "bash", "-c", cmd]
    if log_name:
        with open(LOGS_DIR / f"{log_name}.log", "a") as log:
            return subprocess.run(full_cmd, stdout=log, stderr=log, text=True, timeout=timeout)
    return subprocess.run(full_cmd, capture_output=True, text=True, timeout=timeout)


def airstack_cmd(*args, env_overrides=None, timeout=1800, log_name=None):
    env = os.environ.copy()
    if env_overrides:
        env.update(env_overrides)
    cmd = [str(Path(AIRSTACK_ROOT) / "airstack.sh")] + list(args)
    if log_name:
        with open(LOGS_DIR / f"{log_name}.log", "a") as log:
            return subprocess.run(cmd, stdout=log, stderr=log, text=True,
                                  timeout=timeout, cwd=AIRSTACK_ROOT, env=env)
    return subprocess.run(cmd, capture_output=True, text=True,
                          timeout=timeout, cwd=AIRSTACK_ROOT, env=env)


def find_container(name_pattern):
    result = subprocess.run(
        ["docker", "ps", "--filter", f"name={name_pattern}", "--format", "{{.Names}}"],
        capture_output=True, text=True,
    )
    names = result.stdout.strip().splitlines()
    return names[0] if names else None


def wait_for_container(name_pattern, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        name = find_container(name_pattern)
        if name:
            result = subprocess.run(
                ["docker", "inspect", "-f", "{{.State.Running}}", name],
                capture_output=True, text=True,
            )
            if "true" in result.stdout:
                return name
        time.sleep(5)
    raise TimeoutError(f"Container matching '{name_pattern}' not running after {timeout}s")
