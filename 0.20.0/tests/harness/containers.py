"""Docker container discovery, compute-usage sampling, and image helpers."""
import json
import os
import re
import time
from pathlib import Path

from harness.commands import _run_teed
from harness.discovery import AIRSTACK_ROOT


def find_all_containers(name_pattern):
    result = _run_teed(
        ["docker", "ps", "--filter", f"name={name_pattern}", "--format", "{{.Names}}"],
        timeout=10,
    )
    return [n for n in result.stdout.strip().splitlines() if n]


def find_container(name_pattern):
    names = find_all_containers(name_pattern)
    return names[0] if names else None


def get_robot_containers(pattern="robot.*desktop"):
    """Return running robot containers sorted by their replica index"""
    def _index(name):
        tail = name.rsplit("-", 1)[-1]
        return int(tail) if tail.isdigit() else 0
    return sorted(find_all_containers(pattern), key=_index)


def container_running(name):
    """True if the named container is currently Running."""
    result = _run_teed(
        ["docker", "inspect", "-f", "{{.State.Running}}", name],
        timeout=10,
    )
    return "true" in result.stdout


def wait_for_container(name_pattern, timeout=120):
    deadline = time.time() + timeout
    while time.time() < deadline:
        name = find_container(name_pattern)
        if name and container_running(name):
            return name
        time.sleep(5)
    raise TimeoutError(f"Container matching '{name_pattern}' not running after {timeout}s")


# ── compute-usage sampling ─────────────────────────────────────────────────

_BYTES_RE = re.compile(r"([\d.]+)\s*([kKMGT]?i?B)$")
_BYTES_TO_MB = {
    "B": 1 / (1024 * 1024),
    "KiB": 1 / 1024, "KB": 1 / 1000, "kB": 1 / 1000,
    "MiB": 1, "MB": 1,
    "GiB": 1024, "GB": 1000,
    "TiB": 1024 * 1024, "TB": 1_000_000,
}


def _parse_docker_bytes(s):
    """Parse a docker-stats byte string (e.g. '123.4MiB', '0B') to MB."""
    m = _BYTES_RE.match((s or "").strip())
    if not m:
        return 0.0
    return float(m.group(1)) * _BYTES_TO_MB.get(m.group(2), 1)


def sample_compute_usage(sim_container):
    """Snapshot of compute resources: per-container CPU/mem/disk-IO/net-IO plus
    global host CPU/mem and GPU util/VRAM/temp/power. Returns {key: value},
    keys shaped `{entity}.{metric}` where entity is the full container name or
    'host'. Per-robot replicas (e.g. airstack-robot-desktop-1/2/3) are kept
    distinct so raw metrics.json preserves per-robot data; parse_metrics
    pools them at report time. Silently omits metrics that fail to sample."""
    import psutil

    out = {}

    stats = _run_teed(
        ["docker", "stats", "--no-stream", "--format", "{{json .}}"],
        timeout=20,
    )
    for line in stats.stdout.strip().splitlines():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        name = d.get("Name", "")
        if not name or name.startswith("docker-test-run"):
            continue
        out[f"{name}.cpu_pct"] = float(d.get("CPUPerc", "0%").rstrip("%") or 0)
        mem_raw = d.get("MemUsage", "").split("/")[0].strip()
        out[f"{name}.mem_mb"] = _parse_docker_bytes(mem_raw)
        for io_field, metric in (("BlockIO", "disk_io_mb"), ("NetIO", "net_io_mb")):
            parts = (d.get(io_field, "") or "").split("/")
            total = sum(_parse_docker_bytes(p.strip()) for p in parts)
            out[f"{name}.{metric}"] = total

    out["host.cpu_pct"] = psutil.cpu_percent(interval=0.5)
    out["host.mem_mb"] = psutil.virtual_memory().used / (1024 * 1024)

    gpu = _run_teed(
        ["docker", "exec", sim_container, "nvidia-smi",
         "--query-gpu=utilization.gpu,memory.used,temperature.gpu,power.draw",
         "--format=csv,noheader,nounits"],
        timeout=10,
    )
    if gpu.returncode == 0 and gpu.stdout.strip():
        fields = [f.strip() for f in gpu.stdout.strip().splitlines()[0].split(",")]
        if len(fields) >= 4:
            try:
                out["host.gpu_pct"] = float(fields[0])
                out["host.vram_mb"] = float(fields[1])
                out["host.gpu_temp_c"] = float(fields[2])
                out["host.gpu_power_w"] = float(fields[3])
            except ValueError:
                pass

    return out


def _compose_images(env=None):
    """Resolved image refs that `docker compose up` would use under `env`."""
    compose_env = os.environ.copy()
    if env:
        compose_env.update(env)
    result = _run_teed(
        ["docker", "compose", "-f", str(Path(AIRSTACK_ROOT) / "docker-compose.yaml"),
         "config", "--images"],
        timeout=30, env=compose_env, cwd=AIRSTACK_ROOT,
    )
    return [l.strip() for l in result.stdout.strip().splitlines() if l.strip()]


def missing_images(env=None):
    """Images required by the current compose config but not present locally.
    Used by airstack_env to fail fast instead of letting `airstack up` hang
    pulling/building when images haven't been prebuilt."""
    missing = []
    for image in _compose_images(env=env):
        result = _run_teed(
            ["docker", "image", "inspect", image, "--format", "{{.Id}}"],
            timeout=10,
        )
        if result.returncode != 0:
            missing.append(image)
    return missing


def docker_image_size_mb(service, env=None):
    image = next((i for i in _compose_images(env=env) if service in i), None)
    if not image:
        return None
    result = _run_teed(
        ["docker", "image", "inspect", image, "--format", "{{.Size}}"],
        timeout=10,
    )
    if result.returncode == 0 and result.stdout.strip():
        return round(int(result.stdout.strip()) / 1_000_000, 1)
    return None
