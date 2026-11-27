"""Decode raw stationary captures and build the frozen geometry receipt."""
import hashlib
import json
import math
from pathlib import Path
import struct

OBSERVATIONS = ("control-5", "control-15", "issue", "target")
FRAME_FILES = ("depth.f32le", "instances.u32le", "frame.json", "readiness.jsonl", "phases.jsonl", "survey.json")
RAW_FILES = tuple(f"{name}/{file}" for name in OBSERVATIONS for file in FRAME_FILES)
RAW_FILES += ("issue/tf.json", "target/tf.json", "clock.json", "clock-raw.jsonl", "clock-checks.jsonl")


def expect(condition, reason):
    if not condition:
        raise ValueError(reason)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text())


def raw_hashes(capture):
    return {name: digest(capture / name) for name in RAW_FILES}


def decode(folder, fixture):
    frame = load(folder / "frame.json")
    survey = load(folder / "survey.json")
    k = frame["intrinsics_native"]
    w, h = frame["width"], frame["height"]
    intrinsics = {"width": w, "height": h, "fx": k[0][0], "fy": k[1][1],
                  "cx": k[0][2] - .5, "cy": k[1][2] - .5}
    expect(intrinsics == fixture["intrinsics"], "wrong camera intrinsics")
    matrix = frame["camera_local_to_world_usd"]
    pose = {"origin_m": matrix[3][:3],
            "optical_to_stage": [[matrix[0][i], -matrix[1][i], -matrix[2][i]] for i in range(3)]}
    expect(survey["renderable_prims"] == [fixture["target_prim"]], "extra renderable surface")
    vertices = survey["vertices_m"]
    y = vertices[0][1]
    expect(vertices == [[-2, y, -1], [2, y, -1], [2, y, 3], [-2, y, 3]], "unsupported surveyed plane")
    depth = (folder / "depth.f32le").read_bytes()
    instances = (folder / "instances.u32le").read_bytes()
    expect(len(depth) == len(instances) == w * h * 4, "invalid raw buffer length")
    samples = []
    for u, v in fixture["pixels"]:
        offset = (v * w + u) * 4
        z, = struct.unpack_from("<f", depth, offset)
        identifier, = struct.unpack_from("<I", instances, offset)
        label = frame["id_to_labels"].get(str(identifier))
        expect(math.isfinite(z) and 0 < z <= 16 and label == fixture["target_prim"],
               f"invalid first hit at pixel {(u, v)}")
        samples.append({"uv": [u, v], "axial_m": z, "first_hit": label})
    return frame, survey, pose, intrinsics, samples


def validate_controls(capture, fixture, controls=(5, 15)):
    results = []
    for distance in controls:
        folder = capture / f"control-{distance}"
        frame, survey, pose, k, samples = decode(folder, fixture)
        expect(survey["vertices_m"][0][1] == distance, "control at wrong surveyed distance")
        comparisons = []
        for sample in samples:
            u, v = sample["uv"]
            optical = [(u - k["cx"]) / k["fx"], (v - k["cy"]) / k["fy"], 1]
            ray = [math.fsum(a * b for a, b in zip(row, optical))
                   for row in pose["optical_to_stage"]]
            expect(ray[1] > 0, "control ray is not forward")
            expected = (survey["vertices_m"][0][1] - pose["origin_m"][1]) * math.hypot(*ray) / ray[1]
            observed = sample["axial_m"] * math.hypot(*optical)
            bits, = struct.unpack("<I", struct.pack("<f", sample["axial_m"]))
            ulp = struct.unpack("<f", struct.pack("<I", bits + 1))[0] - sample["axial_m"]
            expect(abs(observed - expected) + ulp * math.hypot(*optical) < fixture["maximum_range_error_m"],
                   f"{distance} m control disagrees at pixel {sample['uv']}")
            comparisons.append({**sample, "independent_ray_m": expected, "observed_ray_m": observed})
        results.append({"control_m": distance, "reference": frame["frame_association"],
                        "survey": survey, "comparisons": comparisons})
    return results


def build_receipt(root, fixture, frozen_fixture, write_json):
    capture = root / "run" / "capture"
    controls = validate_controls(capture, fixture)
    write_json(root / "preflight.json", {"controls": controls, "renderer": "RaytracedLighting"})
    for index, name in enumerate(("issue", "target")):
        folder = capture / name
        frame, survey, pose, k, samples = decode(folder, fixture)
        clock = load(capture / "clock.json")
        stamp = clock["samples_ns"][0 if index == 0 else -1]
        native = frame["frame_association"]["rendering_time_s"]
        expect(survey["simulation_clock_ns"] == stamp and native == struct.unpack("<f", struct.pack("<f", stamp / 1e9))[0],
               "raw render reference/timestamp does not bind to observed clock")
        raw_clock = [json.loads(line) for line in (capture / "clock-raw.jsonl").read_text().splitlines()]
        raw_ns = [item["sec"] * 1_000_000_000 + item["nanosec"] for item in raw_clock]
        expect(raw_ns[-len(clock["samples_ns"]):] == clock["samples_ns"], "clock normalization differs from ROS samples")
        tf = load(folder / "tf.json")
        expect(tf["stamp_ns"] == stamp, "TF did not resolve at the exact observation stamp")
        direction = [row[2] for row in pose["optical_to_stage"]]
        norms = math.hypot(*direction)
        obs = {"tick": fixture["issue_tick" if index == 0 else "target_tick"], "pose_stamp_ns": stamp,
               "depth_stamp_ns": stamp, "depth_frame": fixture["camera_frame"], "pose": pose,
               "tf": {"stamp_ns": tf["stamp_ns"], "parent": tf["parent"], "child": tf["child"],
                      "pose": {key: tf[key] for key in ("origin_m", "optical_to_stage")}},
               "camera_info": {"stamp_ns": stamp, "frame": fixture["camera_frame"], "intrinsics": k},
               "ray_origin_m": pose["origin_m"], "ray_direction": [v / norms for v in direction],
               "pixels": samples}
        write_json(root / f"{name}.json", obs)
    target_frame, target_survey, _, _, _ = decode(capture / "target", fixture)
    vertices = target_survey["vertices_m"]
    expect(vertices[0][1] == 10, "target plane not at 10 m")

    def pin(path):
        return {"path": path, "sha256": digest(root / path)}

    lock = {"schema_version": 1, "kind": "isaac_capture", "fixture_sha256": digest(frozen_fixture),
            "isaac_version": fixture["isaac_version"],
            "image_sha256": (root / "image.txt").read_text().splitlines()[0].split("@sha256:")[1],
            "airstack_commit": (root / "source-commit.txt").read_text().strip(),
            "pegasus_commit": (root / "pegasus-commit.txt").read_text().strip(),
            "renderer": "RaytracedLighting",
            "stage": {"meters_per_unit": target_frame["stage_meters_per_unit"],
                      "up_axis": target_frame["stage_up_axis"], "camera_prim": target_frame["camera_prim"],
                      "only_renderable_prim": target_survey["renderable_prims"][0],
                      "target_center_m": [sum(v[i] for v in vertices) / 4 for i in range(3)],
                      "target_normal": [0, -1, 0],
                      "target_half_extents_xz_m": [(vertices[1][0] - vertices[0][0]) / 2,
                                                    (vertices[2][2] - vertices[1][2]) / 2]},
            "control_axial_m": [r["comparisons"][0]["axial_m"] for r in controls],
            "files": {"stage": pin("input/stage.usda"), "exporter": pin("input/capture_r4.py"),
                      "preflight": pin("preflight.json"), "input_manifest": pin("input.sha256")}}
    write_json(root / "lock.json", lock)
    lock_sha = digest(root / "lock.json")
    receipt = {"schema_version": 1, "kind": "isaac_capture", "lock_sha256": lock_sha, "clock": clock,
               "depth_annotator": "distance_to_image_plane", "issue": load(root / "issue.json"),
               "target": load(root / "target.json"),
               "files": {"clock": pin("run/capture/clock-raw.jsonl"), "issue": pin("issue.json"),
                         "target": pin("target.json")}}
    write_json(root / "receipt.json", receipt)
    return lock_sha
