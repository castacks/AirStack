#!/usr/bin/env python3
"""Validate a static geometry receipt without Isaac, ROS, or forecast execution."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys

HERE = Path(__file__).resolve().parent
ADAPTER = HERE.parent / "offline-geometry-m08"
sys.dont_write_bytecode = True
sys.path.insert(0, str(ADAPTER))
import adapter

FIXTURE = HERE / "fixture.json"
HEX = re.compile(r"[a-f0-9]{64}")
REV = re.compile(r"[a-f0-9]{40}")
# Integral fixture geometry is exact in binary64. Permit arithmetic roundoff.
ROUND = 64 * math.ulp(10.0)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def keys(value, *names):
    require(isinstance(value, dict) and set(value) == set(names),
            f"expected fields {', '.join(names)}")


def number(value):
    require(type(value) in (int, float) and math.isfinite(value), "expected finite number")
    return value


def integer(value):
    require(type(value) is int and value >= 0, "expected nonnegative integer")
    return value


def vector(value):
    require(isinstance(value, list) and len(value) == 3, "expected 3-vector")
    return [number(item) for item in value]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def canonical(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def read_json(path):
    def pairs(items):
        result = dict(items)
        require(len(result) == len(items), "duplicate JSON key")
        return result

    def invalid(token):
        raise ValueError(f"nonfinite JSON token {token}")

    return json.loads(path.read_bytes(), object_pairs_hook=pairs, parse_constant=invalid)


def match_hash(value):
    require(isinstance(value, str) and HEX.fullmatch(value), "invalid SHA-256")


def verified_files(entries, folder, roles, *, extra=False):
    require(isinstance(entries, dict) and roles <= entries.keys() and
            (extra or entries.keys() == roles), "invalid evidence file roles")
    paths = []
    for entry in entries.values():
        keys(entry, "path", "sha256")
        raw = entry["path"]
        require(isinstance(raw, str) and raw and not Path(raw).is_absolute(), "invalid evidence path")
        path = (folder / raw).resolve()
        require(path.is_relative_to(folder) and path.is_file(), "evidence file must exist inside bundle")
        match_hash(entry["sha256"])
        with path.open("rb") as stream:
            require(hashlib.file_digest(stream, "sha256").hexdigest() == entry["sha256"],
                    "evidence file digest mismatch")
        paths.append(path)
    require(len(paths) == len(set(paths)), "duplicate evidence file path")


def dot(a, b):
    return math.fsum(x*y for x, y in zip(a, b, strict=True))


def normalise(values):
    return [x / math.hypot(*values) for x in values]


def same(a, b, message):
    require(max(abs(x-y) for x, y in zip(a, b, strict=True)) <= ROUND, message)


def pose(value, fixture):
    keys(value, "origin_m", "optical_to_stage")
    origin = vector(value["origin_m"])
    same(origin, fixture["origin_m"], "unqualified camera origin")
    matrix = value["optical_to_stage"]
    require(isinstance(matrix, list) and len(matrix) == 3, "expected 3x3 rotation")
    for row, expected in zip(matrix, fixture["optical_to_stage"], strict=True):
        same(vector(row), expected, "unsupported optical-to-stage transform")
    return origin, matrix


def f32(value):
    value = number(value)
    require(0 < value <= 16, "unsupported depth value")
    raw = struct.pack("<f", value)
    require(struct.unpack("<f", raw)[0] == value, "depth must be an exact binary32 sample")
    next_bits = struct.unpack("<I", raw)[0] + 1
    return struct.unpack("<f", struct.pack("<I", next_bits))[0] - value


def depth_budget(control, fixture):
    require(isinstance(control, list) and len(control) == 2, "expected two axial controls")
    residuals = []
    for actual, expected in zip(control, fixture["reference_control_axial_m"], strict=True):
        residuals.append(abs(actual - expected) + f32(actual))
    budget = max(residuals)
    require(budget < fixture["maximum_range_error_m"], "preflight depth exceeds M11 range ceiling")
    return budget


def read_lock(path, expected_sha, fixture):
    match_hash(expected_sha)
    require(digest(path.read_bytes()) == expected_sha, "capture lock hash mismatch")
    lock = read_json(path)
    keys(lock, "schema_version", "kind", "fixture_sha256", "isaac_version", "image_sha256",
         "airstack_commit", "pegasus_commit", "renderer", "stage", "control_axial_m", "files")
    require(integer(lock["schema_version"]) == 1 and lock["kind"] in ("synthetic_test", "isaac_capture"),
            "unsupported lock schema or kind")
    require(lock["fixture_sha256"] == digest(FIXTURE.read_bytes()), "wrong fixture pin")
    require(lock["isaac_version"] == fixture["isaac_version"], "wrong Isaac version")
    match_hash(lock["image_sha256"])
    for name in ("airstack_commit", "pegasus_commit"):
        require(isinstance(lock[name], str) and REV.fullmatch(lock[name]), "invalid source revision")
    require(isinstance(lock["renderer"], str) and lock["renderer"].strip(), "missing renderer")
    stage = lock["stage"]
    keys(stage, "meters_per_unit", "up_axis", "camera_prim", "only_renderable_prim",
         "target_center_m", "target_normal", "target_half_extents_xz_m")
    require(number(stage["meters_per_unit"]) == fixture["meters_per_unit"] and
            stage["up_axis"] == fixture["up_axis"] and stage["camera_prim"] == fixture["camera_prim"] and
            stage["only_renderable_prim"] == fixture["target_prim"], "unsupported stage geometry or units")
    same(vector(stage["target_center_m"]), fixture["target_center_m"], "surveyed target center disagrees")
    same(vector(stage["target_normal"]), fixture["target_normal"], "surveyed target normal disagrees")
    extents = stage["target_half_extents_xz_m"]
    require(isinstance(extents, list) and len(extents) == 2 and
            list(map(number, extents)) == fixture["target_half_extents_xz_m"], "target size disagrees")
    verified_files(lock["files"], path.parent.resolve(), {"stage", "exporter", "preflight"}, extra=True)
    return lock["kind"], depth_budget(lock["control_axial_m"], fixture)


def intersect(origin, ray, fixture):
    center, normal = fixture["target_center_m"], fixture["target_normal"]
    offset = [x-y for x, y in zip(center, origin, strict=True)]
    cosine = dot(ray, normal)
    require(cosine < -ROUND, "no forward front-face intersection")
    distance = dot(offset, normal) / cosine
    hit = [x + distance*d for x, d in zip(origin, ray, strict=True)]
    require(distance > 0 and all(abs(hit[i] - center[i]) < width - ROUND
            for i, width in zip((0, 2), fixture["target_half_extents_xz_m"], strict=True)),
            "ambiguous edge or missing first hit")
    return distance


def observations(value, fixture, stamp, tick, budget):
    keys(value, "tick", "pose_stamp_ns", "depth_stamp_ns", "depth_frame", "tf", "pose",
         "camera_info", "ray_origin_m", "ray_direction", "pixels")
    require(integer(value["tick"]) == tick and
            integer(value["pose_stamp_ns"]) == integer(value["depth_stamp_ns"]) == stamp,
            "pose/depth not tied to observation tick")
    camera_info = value["camera_info"]
    keys(camera_info, "stamp_ns", "frame", "intrinsics")
    require(integer(camera_info["stamp_ns"]) == stamp and
            camera_info["frame"] == value["depth_frame"] == fixture["camera_frame"] and
            camera_info["intrinsics"] == fixture["intrinsics"], "camera info/depth frame or intrinsics disagree")
    origin, matrix = pose(value["pose"], fixture)
    tf = value["tf"]
    keys(tf, "stamp_ns", "parent", "child", "pose")
    require(integer(tf["stamp_ns"]) == stamp and tf["parent"] == fixture["stage_frame"] and
            tf["child"] == fixture["camera_frame"], "missing exact-time camera TF")
    tf_origin, tf_matrix = pose(tf["pose"], fixture)
    same(tf_origin, origin, "TF and authored pose disagree")
    for a, b in zip(tf_matrix, matrix, strict=True):
        same(a, b, "TF and authored rotation disagree")
    same(vector(value["ray_origin_m"]), origin, "ray not at camera center")
    ray = vector(value["ray_direction"])
    require(abs(math.hypot(*ray) - 1) <= ROUND, "ray is not unit length")
    intrinsics = fixture["intrinsics"]
    require(isinstance(value["pixels"], list) and len(value["pixels"]) == 2,
            "center and off-axis sample required")
    ranges = []
    for sample, uv in zip(value["pixels"], fixture["pixels"], strict=True):
        keys(sample, "uv", "axial_m", "first_hit")
        require(isinstance(sample["uv"], list) and list(map(integer, sample["uv"])) == uv and
                sample["first_hit"] == fixture["target_prim"], "wrong pixel or first-hit ID")
        pixel = [(uv[0] - intrinsics["cx"]) / intrinsics["fx"],
                 (uv[1] - intrinsics["cy"]) / intrinsics["fy"], 1.0]
        scale = math.hypot(*pixel)
        direction = normalise([dot(row, pixel) for row in matrix])
        analytic = intersect(origin, direction, fixture)
        depth = sample["axial_m"]
        precision = f32(depth)
        allowance = (budget + precision) * scale + ROUND
        require(allowance <= fixture["maximum_range_error_m"], "depth budget exceeds M11 range ceiling")
        observed = depth * scale
        require(abs(observed - analytic) <= allowance, "rendered first hit disagrees with geometry")
        if uv == fixture["pixels"][0]:
            same(ray, direction, "center ray disagrees with camera")
        ranges.append({"analytic_range_m": analytic, "observed_range_m": observed,
                       "allowed_error_m": allowance})
    return ranges


def verify(receipt_path, lock_path, lock_sha, *, synthetic=False):
    fixture, receipt = read_json(FIXTURE), read_json(receipt_path)
    kind, budget = read_lock(lock_path, lock_sha, fixture)
    keys(receipt, "schema_version", "kind", "lock_sha256", "clock", "depth_annotator",
         "issue", "target", "files")
    require(integer(receipt["schema_version"]) == 1 and receipt["lock_sha256"] == lock_sha and
            kind == receipt["kind"], "receipt and capture lock disagree")
    require(kind == ("synthetic_test" if synthetic else "isaac_capture"), "receipt kind not allowed")
    require(receipt["depth_annotator"] == "distance_to_image_plane", "unsupported depth convention")
    verified_files(receipt["files"], receipt_path.parent.resolve(), {"clock", "issue", "target"})
    clock = receipt["clock"]
    keys(clock, "source", "epoch_ns", "period_ns", "samples_ns")
    period = fixture["tick_period_ns"]
    epoch = integer(clock["epoch_ns"])
    first, last = fixture["issue_tick"], fixture["target_tick"]
    require(clock["source"] == "IsaacReadSimulationTime:/clock" and
            integer(clock["period_ns"]) == period and (last-first)*period == adapter.HORIZON_NS,
            "unsupported simulation clock or horizon")
    require(isinstance(clock["samples_ns"], list) and
            list(map(integer, clock["samples_ns"])) == [epoch + tick*period for tick in range(first, last+1)],
            "noncontiguous or inconsistent simulation stamps")
    issue_ns, target_ns = clock["samples_ns"][0], clock["samples_ns"][-1]
    issue = observations(receipt["issue"], fixture, issue_ns, first, budget)
    # Build the issue binding before reading target observations. Depth verifies the
    # nominal surveyed path. It is not rounded or substituted into M08.
    qualified, expected, _ = adapter.load_fixture(ADAPTER / "fixture.json")
    geometry = qualified["geometry"]
    require(abs(issue[0]["analytic_range_m"] - expected) <= ROUND and
            adapter.path_length(geometry) == expected and period == qualified["tick_period_ns"],
            "incompatible with the qualified offline adapter")
    binding = {"geometry": geometry, "tick_period_ns": period, "issue_tick": first, "issue_ns": issue_ns}
    target = observations(receipt["target"], fixture, target_ns, last, budget)
    return {"status": "PASS", "kind": kind, "canonical_receipt_sha256": digest(canonical(receipt)),
            "issue_binding_sha256": digest(canonical(binding)), "issue_binding": binding,
            "target_tick": last, "target_ns": target_ns, "issue_ranges": issue, "target_ranges": target}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("receipt", type=Path)
    parser.add_argument("lock", type=Path)
    parser.add_argument("--expected-lock-sha256", required=True)
    parser.add_argument("--synthetic-test", action="store_true")
    args = parser.parse_args()
    try:
        result = verify(args.receipt, args.lock, args.expected_lock_sha256, synthetic=args.synthetic_test)
    except (ValueError, TypeError, OSError, OverflowError) as error:
        parser.exit(1, f"FAIL: {error}\n")
    sys.stdout.buffer.write(canonical(result))


if __name__ == "__main__":
    main()
