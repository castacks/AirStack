"""Recorded SLAM proxy-reference diagnostics; no independent ground-truth claim."""

import argparse
from decimal import Decimal
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import yaml

try:
    from .audit_td3_geometry import pose_matrix, static_query
except ImportError:
    from audit_td3_geometry import pose_matrix, static_query


OPTICAL_FROM_NED = np.array([[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]])


def read_ns(path):
    return np.array([int((Decimal(s) * 1_000_000_000).to_integral_value())
                     for s in Path(path).read_text().splitlines()], dtype=np.int64)


def matrices(poses):
    poses = np.asarray(poses)
    result = np.repeat(np.eye(4)[None], len(poses), axis=0)
    result[:, :3, :3] = Rotation.from_quat(poses[:, 3:7]).as_matrix()
    result[:, :3, 3] = poses[:, :3]
    return result


class PoseReference:
    def __init__(self, time_ns, poses):
        self.time_ns = np.asarray(time_ns, dtype=np.int64)
        self.poses = np.asarray(poses, dtype=np.float64)[:, :7]
        if len(self.time_ns) != len(self.poses) or len(self.time_ns) < 2:
            raise ValueError("Reference requires at least two aligned pose/time rows")
        if not np.isfinite(self.poses).all() or np.any(np.diff(self.time_ns) <= 0):
            raise ValueError("Reference must be finite with strictly increasing timestamps")
        if not np.allclose(np.linalg.norm(self.poses[:, 3:], axis=1), 1., atol=1e-3):
            raise ValueError("Reference quaternions must be unit length")
        self.seconds = (self.time_ns - self.time_ns[0]) / 1e9
        self.slerp = Slerp(self.seconds, Rotation.from_quat(self.poses[:, 3:]))

    def at(self, time_ns):
        time_ns = np.asarray(time_ns, dtype=np.int64)
        if np.any(time_ns < self.time_ns[0]) or np.any(time_ns > self.time_ns[-1]):
            raise ValueError("Reference extrapolation is prohibited")
        seconds = (time_ns - self.time_ns[0]) / 1e9
        positions = np.stack([np.interp(seconds, self.seconds, self.poses[:, k]) for k in range(3)], axis=-1)
        return matrices(np.concatenate([positions, self.slerp(seconds).as_quat()], axis=-1))


def load_reference(dataset):
    directory = Path(dataset) / "super_odometry/odometry"
    info = yaml.safe_load((directory / "info.yaml").read_text())
    if info != {"frame_id": "sensor_init", "child_frame_id": "vehicle"}:
        raise ValueError(f"Unexpected recorded reference frame metadata: {info}")
    return PoseReference(read_ns(directory / "interp_timestamps.txt"),
                         np.loadtxt(directory / "interp_data.txt")), info


def camera_reference(reference, time_ns, dataset, calibration, modality):
    config = yaml.safe_load(Path(calibration).read_text())
    edges = [(e["from_frame"], e["to_frame"], pose_matrix(e["translation"] + e["quaternion"]))
             for e in config["transform_params"]]
    optical = "multisense/left_camera_optical_frame" if modality == "rgb" else "thermal_left/optical_frame"
    vehicle_from_optical = static_query(edges, "vehicle", optical)
    name = "multisense_left_rect_color_camera_info" if modality == "rgb" else "thermal_left_processed_camera_info"
    info = yaml.safe_load((Path(dataset) / "sensors" / name / "data.yaml").read_text())
    optical_from_rectified_ned = np.eye(4)
    optical_from_rectified_ned[:3, :3] = np.asarray(info["r"]).T @ OPTICAL_FROM_NED
    return reference.at(time_ns) @ vehicle_from_optical @ optical_from_rectified_ned


def trajectory_metrics(estimated, reference):
    """Anchor first pose only; no fitted rotation, translation, or scale."""
    estimated, reference = np.asarray(estimated), np.asarray(reference)
    if estimated.shape != reference.shape or len(estimated) < 2:
        raise ValueError("Aligned trajectories with at least two poses required")
    e = np.linalg.inv(estimated[0]) @ estimated
    r = np.linalg.inv(reference[0]) @ reference
    error = np.linalg.inv(r) @ e
    translation = np.linalg.norm(error[:, :3, 3], axis=1)
    rotation = np.degrees(Rotation.from_matrix(error[:, :3, :3]).magnitude())
    e_delta = np.linalg.inv(e[:-1]) @ e[1:]
    r_delta = np.linalg.inv(r[:-1]) @ r[1:]
    relative_error = np.linalg.inv(r_delta) @ e_delta
    rt = np.linalg.norm(relative_error[:, :3, 3], axis=1)
    rr = np.degrees(Rotation.from_matrix(relative_error[:, :3, :3]).magnitude())
    return {"anchoring": "first pose only; no scale or best-fit alignment",
            "anchored_translation_rmse_m": float(np.sqrt(np.mean(translation ** 2))),
            "anchored_rotation_rmse_deg": float(np.sqrt(np.mean(rotation ** 2))),
            "consecutive_translation_rmse_m": float(np.sqrt(np.mean(rt ** 2))),
            "consecutive_rotation_rmse_deg": float(np.sqrt(np.mean(rr ** 2))),
            "reference_net_displacement_m": float(np.linalg.norm(r[-1, :3, 3])),
            "estimated_net_displacement_m": float(np.linalg.norm(e[-1, :3, 3]))}


def select_window(reference, rows, count, max_gap_ns):
    times = np.array([r["rgb_left"]["time_ns"] for r in rows], dtype=np.int64)
    positions = reference.at(times)[:, :3, 3]
    candidates = []
    for start in range(len(rows) - count + 1):
        window = times[start:start + count]
        if np.max(np.diff(window)) > max_gap_ns:
            continue
        distance = float(np.linalg.norm(positions[start + count - 1] - positions[start]))
        candidates.append((distance, start))
    if not candidates:
        raise ValueError("No continuous window satisfies acquisition gap bound")
    distance, start = max(candidates, key=lambda x: (x[0], -x[1]))
    return {"manifest_start": start, "frames": count, "selection": "maximum body-reference endpoint displacement only",
            "max_gap_ns": max_gap_ns, "body_reference_displacement_m": distance,
            "duration_seconds": float((times[start + count - 1] - times[start]) / 1e9)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--trajectory-root", type=Path)
    args = parser.parse_args()
    if args.frames < 2:
        parser.error("At least two frames required")
    reference, info = load_reference(args.dataset)
    manifest = json.loads(args.manifest.read_text())
    times = read_ns(args.dataset / "tf/vehicle_to_sensor_init/timestamps.txt")
    poses = np.loadtxt(args.dataset / "tf/vehicle_to_sensor_init/transforms.txt")
    valid = (times >= reference.time_ns[0]) & (times <= reference.time_ns[-1])
    tf = matrices(poses[valid]); odom = reference.at(times[valid])
    residuals = np.linalg.inv(odom) @ tf
    inverse_residuals = np.linalg.inv(odom) @ np.linalg.inv(tf)
    audit = {"scope": "recorded SLAM proxy reference, not independent ground truth",
             "reference_frames": info, "reference_samples": len(reference.time_ns),
             "calibration_sha256": hashlib.sha256(args.calibration.read_bytes()).hexdigest(),
             "tf_same_direction_translation_median_m": float(np.median(np.linalg.norm(residuals[:, :3, 3], axis=1))),
             "tf_same_direction_rotation_median_deg": float(np.median(np.degrees(Rotation.from_matrix(residuals[:, :3, :3]).magnitude()))),
             "tf_inverse_direction_translation_median_m": float(np.median(np.linalg.norm(inverse_residuals[:, :3, 3], axis=1))),
             "selected_window": select_window(reference, manifest["pairs"], args.frames, 150_000_000),
             "evaluations": {}}
    if args.trajectory_root:
        for modality in ["rgb", "thermal"]:
            source = np.load(args.trajectory_root / modality / "trajectory.npz", allow_pickle=False)
            camera = camera_reference(reference, source["time_ns"].reshape(-1), args.dataset, args.calibration, modality)
            audit["evaluations"][modality] = trajectory_metrics(matrices(source["poses_camera_ned"]), camera)
            np.savez_compressed(args.trajectory_root / modality / "proxy-reference.npz",
                                T_world_camera_ned=camera, time_ns=source["time_ns"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps(audit, indent=2))


if __name__ == "__main__":
    main()
