"""Preserve the R3 fail-closed host gate for a four-observation receipt."""
from functools import partial
import json
import os
from pathlib import Path
import subprocess
import sys

import recorder
from review_r4 import OBSERVATIONS, RAW_FILES, build_receipt, load, raw_hashes
from supervise_r3 import SHUTDOWN_LINE, validate_acquisition, verify_container_exit

FIXTURE_VERIFIER = Path(os.environ.get("ORE_R4_VERIFIER", "/invalid/verify.py"))
PHASES = ("isaac_startup", "stage_camera_writers", "control-5", "control-5-analytic",
          "control-15", "control-15-analytic", "ros_clock_setup", "clock_epoch", "timed_geometry")


def validate(capture, mode):
    raw = (capture / "phases.jsonl").read_bytes()
    if not raw.endswith(b"\n"):
        raise ValueError("incomplete geometry lifecycle journal")
    events = [json.loads(line) for line in raw.splitlines()]
    # Issue and target are nested in the timing phase.
    expected = [(name, state) for name in PHASES[:-1] for state in ("started", "completed")] + [
        ("timed_geometry", "started"), ("issue", "started"), ("issue", "completed"),
        ("target", "started"), ("target", "completed"), ("timed_geometry", "completed"),
        ("sequence", "captured"), ("shutdown", "started")]
    observed = [(event["phase"], event["status"]) for event in events]
    returned = observed == [*expected, ("shutdown", "completed")]
    if observed != expected and not returned:
        raise ValueError("incomplete geometry sequence or missing shutdown request")
    process = verify_container_exit(capture.parent)
    if not returned:
        log = (capture.parent / "child.log").read_bytes()
        if b"--/app/fastShutdown=True" not in log or not SHUTDOWN_LINE.search(log):
            raise ValueError("missing native Kit shutdown evidence")
    for name in OBSERVATIONS:
        validate_acquisition(capture / name, mode)
    if events[len(expected) - 2]["sha256"] != raw_hashes(capture):
        raise ValueError("persisted R4 sequence differs from its pre-shutdown seal")
    root = capture.parent.parent
    lock_sha = build_receipt(root, load(root / "input/fixture.json"),
                             FIXTURE_VERIFIER.with_name("fixture.json"), recorder.write_json)
    result = subprocess.run([sys.executable, str(FIXTURE_VERIFIER), str(root / "receipt.json"),
                             str(root / "lock.json"), "--expected-lock-sha256", lock_sha],
                            check=False, capture_output=True, timeout=15)
    recorder.write_artifact(root / "receipt-verifier.log", result.stdout + result.stderr)
    if result.returncode:
        raise ValueError("frozen receipt verifier rejected the geometry receipt")
    return {"process": process, "native_shutdown_return": "OBSERVED" if returned else "NOT_OBSERVED",
            "receipt": json.loads(result.stdout), "lock_sha256": lock_sha}


if __name__ == "__main__":
    recorder.main(partial(recorder.supervise, validator=validate, artifacts=("phases.jsonl", *RAW_FILES)))
