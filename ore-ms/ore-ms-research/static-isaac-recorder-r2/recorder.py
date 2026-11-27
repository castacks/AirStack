"""Journal one-frame capture and supervise its completion without Isaac dependencies."""

import argparse
from contextlib import contextmanager
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import traceback

WIDTH, HEIGHT = 33, 25
CAMERA = "/World/ReceiptCamera"
TARGET = "/World/ReceiptTarget"
PHASES = ("isaac_startup", "stage_open", "camera_initialize", "simulation_start",
          "render_step_1", "render_step_2", "render_step_3", "frame_readout",
          "write_artifacts", "shutdown")
ARTIFACTS = ("depth.f32le", "instances.u32le", "frame.json", "phases.jsonl")


def sync_directory(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def write_artifact(path, payload):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    sync_directory(path.parent)


def write_json(path, value):
    write_artifact(path, (json.dumps(value, indent=2, allow_nan=False) + "\n").encode())


def checkpoint(output, name, status, **details):
    record = {"phase": name, "status": status, "wall_time_ns": time.time_ns(), **details}
    append_record(output / "phases.jsonl", record)


def append_record(path, record):
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(record, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    sync_directory(path.parent)


@contextmanager
def phase(output, name):
    checkpoint(output, name, "started")
    try:
        yield
    except BaseException:
        checkpoint(output, name, "failed", traceback=traceback.format_exc())
        raise
    else:
        checkpoint(output, name, "completed")


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def finite_matrix(value, size):
    return (isinstance(value, list) and len(value) == size
            and all(isinstance(row, list) and len(row) == size
                    and all(finite_number(entry) for entry in row) for row in value))


def validate_capture(capture, mode, *, phases=PHASES, schema=1):
    events = [json.loads(line) for line in (capture / "phases.jsonl").read_text().splitlines()]
    expected = [(name, status) for name in phases for status in ("started", "completed")]
    if [(event["phase"], event["status"]) for event in events] != expected:
        raise ValueError("capture did not complete every phase through shutdown")
    validate_frame(capture, mode, schema=schema)


def validate_frame(capture, mode, *, schema):
    metadata = json.loads((capture / "frame.json").read_text())
    if metadata["schema"] != schema or metadata["source"] != mode:
        raise ValueError("unexpected frame schema or source")
    if (metadata["width"], metadata["height"]) != (WIDTH, HEIGHT):
        raise ValueError("unexpected sensor dimensions")
    if not finite_number(metadata["rendering_time_s"]) or metadata["rendering_time_s"] < 0:
        raise ValueError("missing or invalid native render timestamp or frame index")
    if schema == 1 and (type(metadata["rendering_frame"]) is not int or metadata["rendering_frame"] < 1):
        raise ValueError("missing or invalid native render timestamp or frame index")
    if (metadata["camera_prim"] != CAMERA or metadata["target_prim"] != TARGET
            or metadata["stage_meters_per_unit"] != 1 or metadata["stage_up_axis"] != "Z"
            or metadata["depth_annotator"] != "distance_to_image_plane"
            or metadata["instance_annotator"] != "instance_id_segmentation"
            or metadata["depth_native_dtype"] != "float32"
            or metadata["instance_native_dtype"] != "uint32"):
        raise ValueError("unexpected stage, sensor or annotator configuration")
    intrinsics = metadata["intrinsics_native"]
    if (not finite_matrix(intrinsics, 3) or intrinsics[0][0] <= 0 or intrinsics[1][1] <= 0
            or not finite_matrix(metadata["camera_local_to_world_usd"], 4)):
        raise ValueError("invalid camera metadata")

    depth = (capture / "depth.f32le").read_bytes()
    ids = (capture / "instances.u32le").read_bytes()
    if len(depth) != WIDTH * HEIGHT * 4 or len(ids) != len(depth):
        raise ValueError("depth or instance buffer does not contain one complete frame")
    labels = metadata["id_to_labels"]
    if not isinstance(labels, dict):
        raise ValueError("invalid instance label map")
    if not has_target_depth(depth, ids, labels):
        raise ValueError("no finite positive depth pixel corresponding to the target instance")


def has_target_depth(depth, ids, labels):
    pairs = zip(struct.iter_unpack("<f", depth), struct.iter_unpack("<I", ids))
    return any(math.isfinite(z) and z > 0 and labels.get(str(instance)) == TARGET
               for (z,), (instance,) in pairs)


def supervise(output, timeout, mode, command, *, validator=validate_capture, artifacts=ARTIFACTS):
    output.mkdir()
    sync_directory(output.parent)
    capture = output / "capture"
    capture.mkdir()
    log = output / "child.log"
    result = {"mode": mode, "command": command, "returncode": None}
    try:
        with log.open("xb", buffering=0) as stream:
            try:
                child = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                                       timeout=timeout, check=False)
            finally:
                os.fsync(stream.fileno())
        result["returncode"] = child.returncode
        if child.returncode != 0:
            raise RuntimeError(f"worker command exited with status {child.returncode}")
        evidence = validator(capture, mode)
        if evidence is not None:
            result["verification"] = evidence
        files = [log, *(capture / name for name in artifacts)]
        result["sha256"] = {str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
                            for path in files}
    except (Exception, KeyboardInterrupt):
        write_json(output / "FAILED.json", {**result, "traceback": traceback.format_exc()})
        return 1
    write_json(output / "COMPLETED.json", result)
    return 0


def interrupt(_signum, _frame):
    raise KeyboardInterrupt("supervisor interrupted by SIGTERM")


def main(supervisor=supervise):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout", required=True, type=float)
    parser.add_argument("--mode", choices=("isaac", "mock"), default="isaac")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command or not finite_number(args.timeout) or args.timeout <= 0:
        parser.error("specify a command and a finite positive timeout")
    signal.signal(signal.SIGTERM, interrupt)
    raise SystemExit(supervisor(args.output, args.timeout, args.mode, command))


if __name__ == "__main__":
    main()
