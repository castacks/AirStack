"""Read-only compatibility evidence; not sensor readiness or flight verification."""
from __future__ import annotations

import json
import subprocess

SUPPORTED_NUMPY = "1.26.4"


def probe_isaac_runtime() -> dict:
    """Probe the running simulator interpreter, never host/robot Python.

    Only the image's explicitly pinned dependency profile is qualified here.
    All unavailable/ambiguous evidence fails closed. No repair or restart occurs.
    """
    result = {"schema_version": "rrm-isaac-runtime/v1", "status": "UNAVAILABLE",
              "compatible": False, "container": None, "numpy_version": None,
              "expected_numpy_version": SUPPORTED_NUMPY, "execution_dispatch": False,
              "reason": "Isaac runtime unavailable; inspect simulator container readiness."}
    running = []
    try:
        for name in ("isaac-sim-livestream", "isaac-sim"):
            inspected = subprocess.run(
                ["docker", "inspect", "--format", "{{.State.Running}}", name],
                capture_output=True, text=True, timeout=5,
            )
            if inspected.returncode == 0 and inspected.stdout.strip() not in ("true", "false"):
                raise ValueError("invalid container state")
            if inspected.returncode == 0 and inspected.stdout.strip() == "true":
                running.append(name)
        if len(running) != 1:
            if len(running) > 1:
                result["reason"] = "Multiple Isaac containers running; runtime identity is ambiguous."
            return result
        result["container"] = running[0]
        observed = subprocess.run(
            ["docker", "exec", running[0], "/isaac-sim/python.sh", "-c",
             "import json, numpy; print(json.dumps({'numpy_version': numpy.__version__}))"],
            check=True, capture_output=True, text=True, timeout=15,
        )
        # Isaac's interpreter wrapper may emit startup lines; require exactly one
        # JSON record and never expose arbitrary stderr/stdout to the browser.
        records = [json.loads(line) for line in observed.stdout.splitlines()
                   if line.lstrip().startswith("{")]
        if len(records) != 1 or not isinstance(records[0], dict):
            raise ValueError("invalid version record")
        version = records[0].get("numpy_version")
        if not isinstance(version, str) or not version or len(version) > 64:
            raise ValueError("invalid version")
        result["numpy_version"] = version
        result["compatible"] = version == SUPPORTED_NUMPY
        result["status"] = "COMPATIBLE" if result["compatible"] else "INCOMPATIBLE"
        result["reason"] = (
            "Isaac NumPy dependency matches the pinned profile; sensors and physics are not verified."
            if result["compatible"] else
            "Isaac NumPy does not match pinned 1.26.4. Follow README simulator recovery "
            "while grounded/disarmed with no active mission, then retry. Landing and STOP remain available."
        )
    except (OSError, subprocess.SubprocessError, ValueError, TypeError):
        result["reason"] = "Isaac dependency probe failed; inspect simulator runtime before a new mission."
    return result
