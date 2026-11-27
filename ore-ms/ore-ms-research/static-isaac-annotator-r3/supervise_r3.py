"""Apply the R3 readiness contract before the recorder may certify completion."""

from datetime import datetime
from functools import partial
import hashlib
import json
import os
import re

import recorder
from readiness import ASSOCIATION_SOURCE, MAX_ATTEMPTS, UNVERIFIED

PHASES = ("isaac_startup", "stage_open", "camera_initialize", "annotators_attach",
          "frame_readiness", "write_artifacts")
RETRY_STATUS = {"PAYLOAD_NOT_READY", UNVERIFIED, "FRAME_MISMATCH", "INVALID_BUFFER_SIZE",
                "NO_VALID_TARGET_DEPTH"}
JOURNAL = "readiness.jsonl"
ACQUISITION_FILES = ("depth.f32le", "instances.u32le", "frame.json", JOURNAL)
EXIT_RECORD = "container-exit.json"
SHUTDOWN_LINE = re.compile(rb"^\[[0-9.]+s\] Simulation App Shutting Down\r?$", re.MULTILINE)


def validate_acquisition(capture, mode):
    recorder.validate_frame(capture, mode, schema=2)
    metadata = json.loads((capture / "frame.json").read_text())
    association = metadata["frame_association"]
    attempt = association["attempt"]
    depth_ref = association["depth_reference"]
    instance_ref = association["instance_reference"]
    if (association["source"] != ASSOCIATION_SOURCE
            or not isinstance(depth_ref, list) or len(depth_ref) != 2
            or any(type(value) is not int for value in depth_ref)
            or depth_ref[0] < 0 or depth_ref[1] <= 0
            or instance_ref != depth_ref
            or any(type(value) is not int for value in instance_ref)
            or not recorder.finite_number(association["rendering_time_s"])
            or association["rendering_time_s"] != metadata["rendering_time_s"]
            or type(attempt) is not int or not 1 <= attempt <= MAX_ATTEMPTS
            or not isinstance(metadata["render_product"], str)
            or not metadata["render_product"].startswith("/")):
        raise ValueError(UNVERIFIED)

    journal = (capture / JOURNAL).read_bytes()
    if not journal.endswith(b"\n"):
        raise ValueError("incomplete readiness journal record")
    rows = [json.loads(line) for line in journal.splitlines()]
    if len(rows) != 2 * attempt:
        raise ValueError("incomplete readiness attempts")
    for index, (start, result) in enumerate(zip(rows[::2], rows[1::2]), start=1):
        allowed = {"READY"} if index == attempt else RETRY_STATUS
        if (start["attempt"] != index or start["status"] != "STARTED"
                or result["attempt"] != index or result["status"] not in allowed):
            raise ValueError("invalid readiness attempt sequence")
    ready = rows[-1]
    if (ready["association"] != association
            or any(ready[key]["status"] != "valid" or ready[key]["reference"] != depth_ref
                   for key in ("depth", "instances"))):
        raise ValueError(UNVERIFIED)


def acquisition_hashes(capture):
    return {name: hashlib.sha256((capture / name).read_bytes()).hexdigest()
            for name in ACQUISITION_FILES}


def complete_acquisition(capture, mode):
    with recorder.phase(capture, "acquisition_validation"):
        validate_acquisition(capture, mode)
        hashes = acquisition_hashes(capture)
    recorder.checkpoint(capture, "acquisition", "completed", sha256=hashes)


def verify_container_exit(output):
    receipt = output / EXIT_RECORD
    with receipt.open("rb") as stream:
        payload = stream.read()
        os.fsync(stream.fileno())
    data = json.loads(payload)
    cid = (output.parent / "container.cid").read_text().strip()
    state = data["state"]
    started, finished = (datetime.fromisoformat(state[key]) for key in ("StartedAt", "FinishedAt"))
    if (not re.fullmatch(r"[0-9a-f]{64}", cid) or data["id"] != cid
            or type(data["restart_count"]) is not int or data["restart_count"] != 0
            or state["Status"] != "exited" or state["Running"] is not False
            or state["Dead"] is not False or state["OOMKilled"] is not False or state["Error"] != ""
            or type(state["ExitCode"]) is not int or state["ExitCode"] != 0
            or started.tzinfo is None or finished.tzinfo is None or started.year == 1
            or finished < started):
        raise ValueError("container exit receipt does not show clean native process termination")
    return {"id": cid, "path": EXIT_RECORD, "sha256": hashlib.sha256(payload).hexdigest(),
            "started_at": state["StartedAt"], "finished_at": state["FinishedAt"]}


def validate_capture(capture, mode):
    journal = (capture / "phases.jsonl").read_bytes()
    if not journal.endswith(b"\n"):
        raise ValueError("incomplete lifecycle journal record")
    events = [json.loads(line) for line in journal.splitlines()]
    steps = [(event["phase"], event["status"]) for event in events]
    expected = [(name, status) for name in (*PHASES, "acquisition_validation")
                for status in ("started", "completed")]
    expected += [("acquisition", "completed"), ("shutdown", "started")]
    returned = steps == [*expected, ("shutdown", "completed")]
    if steps != expected and not returned:
        raise ValueError("capture did not complete every phase or record a shutdown request")

    output = capture.parent
    process = verify_container_exit(output) if mode == "isaac" else {"observed_by": "subprocess"}
    if not returned:
        log = (output / "child.log").read_bytes()
        if b"--/app/fastShutdown=True" not in log or not SHUTDOWN_LINE.search(log):
            raise ValueError("shutdown request lacks native shutdown evidence")

    validate_acquisition(capture, mode)
    if events[len(expected) - 2]["sha256"] != acquisition_hashes(capture):
        raise ValueError("acquisition artifacts changed after the worker checkpoint")
    return {
        "states": ["ACQUISITION_COMPLETE", "SHUTDOWN_REQUESTED", "PROCESS_TERMINATED",
                   "EVIDENCE_VERIFIED"],
        "native_shutdown_return": "OBSERVED" if returned else "NOT_OBSERVED",
        "shutdown_evidence": "python_return" if returned else "kit_fast_shutdown_log",
        "process": process,
    }


supervise = partial(recorder.supervise, validator=validate_capture,
                    artifacts=(*recorder.ARTIFACTS, JOURNAL))


if __name__ == "__main__":
    recorder.main(supervise)
