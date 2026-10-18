"""Bounded camera-local VO execution, with provisional geometry and no accuracy claim."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
import torch
from DataLoader.Dataset.TD3 import TD3ManifestSequence
from Odometry.MACVO import MACVO
from Utility.Config import load_config, asNamespace


def run(args, modality):
    torch.manual_seed(0); np.random.seed(0)
    output = args.output / modality
    output.mkdir(parents=True, exist_ok=False)
    sequence = TD3ManifestSequence(args.dataset, args.manifest, modality,
                                   acknowledge_provisional_geometry=args.acknowledge_provisional_geometry)
    if args.start < 0 or len(sequence) < args.start + args.frames:
        raise ValueError("Manifest does not contain requested frame count")
    sequence.clip(start_idx=args.start, end_idx=args.start + args.frames)
    selected_rows = [sequence.rows[int(i)] for i in sequence.indices]
    _, config = load_config(ROOT / "Config/Experiment/MACVO/MACVO_Performant.yaml")
    config = {"Odometry": config["Odometry"]}
    odom = config["Odometry"]
    odom["args"].update(mapping=False, profile=False)
    odom["frontend"]["type"] = "FlowFormerCovFrontend"
    checkpoint = ROOT.parent / "models/macslam" / ("MACVO_FrontendCov.pth" if modality == "rgb" else "MACT_Offroad.pth")
    odom["frontend"]["args"].update(weight=str(checkpoint), enc_dtype="fp32", dec_dtype="fp32")
    odom["optimizer"]["args"]["parallel"] = False
    (output / "effective-config.json").write_text(json.dumps(config, indent=2) + "\n")
    report = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "modality": modality,
              "status": "RUNNING", "scope": "camera-local execution only; not accuracy or calibration validation",
              "coordinate_frame": "rectified left camera NED; identity body/sensor transform",
              "geometry": "absolute projection baseline and absolute disparity; signed thermal geometry unresolved",
              "projection_signed_baseline_m": sequence.signed_baseline,
              "execution_baseline_m": sequence.baseline, "K": sequence.K[0].tolist(),
              "input_rows": selected_rows, "manifest_start": args.start, "seed": 0,
              "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
              "torch": torch.__version__, "gpu": torch.cuda.get_device_name(0)}
    system = None
    try:
        system = MACVO.from_config(asNamespace(config))
        recorder = None
        if getattr(args, "record_uncertainty", False):
            from Scripts.Setup.td3_uncertainty import UncertaintyRecorder
            recorder = UncertaintyRecorder(system, output / "uncertainty")
        first_depth = system.Frontend.estimate_depth(sequence[0].stereo)
        depth = first_depth.depth.detach().cpu().numpy()
        covariance = first_depth.cov.detach().cpu().numpy()
        valid = np.isfinite(depth) & (depth > 0) & np.isfinite(covariance) & (covariance > 0)
        if not valid.any():
            raise RuntimeError("No finite positive depth/covariance samples")
        report["first_depth"] = {"shape": list(depth.shape), "finite_positive_fraction": float(valid.mean()),
                                 "depth_percentiles_m": np.percentile(depth[valid], [5, 50, 95]).tolist()}
        np.savez_compressed(output / "first-depth.npz", depth=depth, covariance=covariance, valid=valid)
        completed = 0
        for frame in sequence:
            system.run(frame)
            completed += 1
            print(f"{modality}: processed {completed}/{args.frames}", flush=True)
        system.terminate()
        if recorder is not None:
            recorder.finish()
        poses = system.graph.frames.data["pose"].tensor.detach().cpu().numpy()
        times = system.graph.frames.data["time_ns"].tensor.detach().cpu().numpy().astype(np.int64)
        if poses.shape != (args.frames, 7) or not np.isfinite(poses).all():
            raise RuntimeError(f"Incomplete/nonfinite trajectory: {poses.shape}")
        if not np.allclose(np.linalg.norm(poses[:, 3:], axis=1), 1., atol=1e-5):
            raise RuntimeError("Trajectory quaternion normalization failed")
        expected_times = np.array([r[modality + '_left']['time_ns'] for r in selected_rows], dtype=np.int64)
        if not np.array_equal(times.reshape(-1), expected_times):
            raise RuntimeError("Trajectory timestamps do not match acquisition timestamps")
        match_count = int(system.graph.match.index.tensor.numel())
        if match_count == 0:
            raise RuntimeError("No visual matches were retained")
        obs_cov = system.graph.match.data["obs2_covTc"].tensor.detach().cpu().numpy()
        finite = np.isfinite(obs_cov).all(axis=(1, 2))
        eigenvalues = np.linalg.eigvalsh(obs_cov[finite])
        report["tracking"] = {
            "retained_matches": match_count,
            "interpolated_frames": int(system.graph.frames.data["need_interp"].tensor.sum().item()),
            "observation_covariance_finite_fraction": float(finite.mean()),
            "observation_covariance_min_eigenvalue": float(eigenvalues.min()) if eigenvalues.size else None}
        np.savez_compressed(output / "tensor-map.npz", **system.graph.serialize())
        np.savez_compressed(output / "trajectory.npz", poses_camera_ned=poses, time_ns=times)
        report.update(status="PASS_EXECUTION_ONLY", completed_frames=completed,
                      pose_count=len(poses), final_translation_camera_ned_m=poses[-1, :3].tolist())
    except Exception as error:
        report.update(status="FAILED", error_type=type(error).__name__, error=str(error))
        raise
    finally:
        (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        if system is not None and not system.terminated:
            system.terminate()
    print(json.dumps({k: report[k] for k in ["modality", "status", "completed_frames", "first_depth"]}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--modality", choices=["rgb", "thermal"], required=True)
    parser.add_argument("--record-uncertainty", action="store_true", help="Save raw variance and floor diagnostics")
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--start", type=int, default=0, help="Start row in the paired manifest")
    parser.add_argument("--acknowledge-provisional-geometry", action="store_true")
    args = parser.parse_args()
    if not 2 <= args.frames <= 100:
        parser.error("Execution checks require 2–100 frames")
    if not args.acknowledge_provisional_geometry:
        parser.error("Pass --acknowledge-provisional-geometry for this camera-local execution experiment")
    run(args, args.modality)
