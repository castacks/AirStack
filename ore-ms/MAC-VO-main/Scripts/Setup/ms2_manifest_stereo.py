"""Opt-in native stereo execution on20 original manifest IDs; no source renumbering.

No VO, depth/reference scoring, timestamp correction or default changes. Run
from MAC-VO-main in macvo:blackwell. The output directory must not exist.
"""
import argparse
from datetime import datetime
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import torch

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE
from Module.Network.FlowFormer.configs.submission import get_cfg
from Module.Network.FlowFormerCov import build_flowformer
from Module.Network.FlowFormerCov.checkpoint import load_cov_checkpoint


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def tensor_sha(tensor):
    return hashlib.sha256(tensor.detach().cpu().contiguous().numpy().tobytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=REPO.parent)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--rgb-checkpoint", type=Path, required=True)
    parser.add_argument("--thermal-checkpoint", type=Path, required=True)
    parser.add_argument("--modalities", nargs="+", choices=["rgb", "thr"], default=["rgb", "thr"])
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-stereo-pairs", type=int, default=20, choices=range(1,21), help="Use1 only for baseline parity checks; primary windows use20")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report_path = args.output / "report.json"
    report = {"status": "RUNNING", "started_at": datetime.now(ZoneInfo("America/New_York")).isoformat(),
              "scope": "runtime compatibility only; no physical accuracy or exposure simultaneity claim",
              "thermal_recipe": THERMAL_RECIPE, "checkpoint_training_recipe_verified": False,
              "arguments": {k: str(v.resolve()) if isinstance(v, Path) else v for k, v in vars(args).items()},
              "modalities": {}, "pairs": []}

    def persist():
        report_path.write_text(json.dumps(report, indent=2) + "\n")

    persist()
    try:
        assert torch.cuda.is_available(), "CUDA required for the original frontend"
        assert len(set(args.modalities)) == len(args.modalities), "Duplicate modalities"
        torch.set_num_threads(1)
        cv2.setNumThreads(1)
        torch.manual_seed(0)
        np.random.seed(0)
        device = "cuda:0"
        report["runtime"] = {"python": platform.python_version(), "torch": torch.__version__,
                             "cuda": torch.version.cuda, "cudnn": torch.backends.cudnn.version(),
                             "gpu": torch.cuda.get_device_name(), "compute_capability": torch.cuda.get_device_capability(),
                             "seed": 0, "threads": 1, "dtype": "float32", "autocast": False,
                             "tf32_matmul": torch.backends.cuda.matmul.allow_tf32,
                             "tf32_cudnn": torch.backends.cudnn.allow_tf32,
                             "cudnn_benchmark": torch.backends.cudnn.benchmark,
                             "packages": {p: importlib.metadata.version(p) for p in
                                          ["torchvision", "pypose", "numpy", "opencv-python-headless", "timm", "einops"]}}
        report["source_commit"] = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip()
        report["runner_sha256"] = sha(__file__)
        source = json.loads(args.manifest.read_text())
        stems = [r["stem"] for r in source["records"]]
        assert len(stems)==20 and stems==sorted(set(stems)) and all(s.isdigit() and len(s)==6 for s in stems), "Expected20 distinct original source IDs"
        frozen = {str(args.manifest.resolve()): sha(args.manifest)}
        for row in source["records"]:
            for camera in row["cameras"].values():
                for side in ("left", "right"):
                    path = (args.root / camera[side]["path"]).resolve()
                    assert path.is_relative_to(args.root.resolve())
                    frozen[str(path)] = sha(path)
                    assert frozen[str(path)] == camera[side]["sha256"]
        report["frozen_inputs"] = frozen
        cfg = get_cfg()
        cfg.latentcostformer.pretrain = False
        (args.output / "effective-config.yaml").write_text(cfg.dump())
        for modality, checkpoint in [("rgb", args.rgb_checkpoint), ("thr", args.thermal_checkpoint)]:
            if modality not in args.modalities:
                continue
            sequence = MS2ManifestSequence(args.root, args.manifest, modality,
                                          thermal_transform=THERMAL_RECIPE if modality == "thr" else None)
            model = build_flowformer(cfg, torch.float32, torch.float32).eval()
            load = load_cov_checkpoint(model, torch.load(checkpoint, map_location="cpu", weights_only=True),
                                       allow_legacy=modality == "rgb")
            model = model.to(device)
            report["modalities"][modality] = {"checkpoint": str(checkpoint.resolve()), "checkpoint_sha256": sha(checkpoint),
                                             "checkpoint_load": load, "height": sequence.height, "width": sequence.width}
            persist()
            frames = [sequence[i] for i in range(20)]
            for i, frame in enumerate(frames):
                camera = source["records"][i]["cameras"][modality]
                assert frame.stereo.source_stem == [source["records"][i]["stem"]]
                assert frame.time_ns == frame.stereo.time_ns == [camera["left"]["recorded_time_ns"]]
                assert frame.stereo.right_time_ns == [camera["right"]["recorded_time_ns"]]
                assert frame.gt_pose is None and frame.stereo.gt_depth is None
            jobs = [("stereo", i, i) for i in range(args.max_stereo_pairs)]
            for kind, i, j in jobs:
                first, second = frames[i].stereo, frames[j].stereo
                image1, image2 = first.imageL, second.imageR if kind == "stereo" else second.imageL
                second_time = second.right_time_ns[0] if kind == "stereo" else second.time_ns[0]
                x, y = image1.to(device), image2.to(device)
                torch.cuda.reset_peak_memory_stats()
                torch.cuda.synchronize()
                start = time.perf_counter()
                with torch.inference_mode():
                    flow, covariance = model.inference(x, y)
                torch.cuda.synchronize()
                elapsed = time.perf_counter() - start
                expected = (1, 2, sequence.height, sequence.width)
                assert tuple(flow.shape) == tuple(covariance.shape) == expected
                assert flow.dtype == covariance.dtype == torch.float32
                assert torch.isfinite(flow).all().item() and torch.isfinite(covariance).all().item()
                assert (covariance > 0).all().item()
                name = f"{modality}-{kind}-{first.source_stem[0]}-{second.source_stem[0]}.npz"
                np.savez_compressed(args.output / name, flow=flow.cpu().numpy(), covariance=covariance.cpu().numpy(),
                                    source_ids=np.array([first.source_stem[0], second.source_stem[0]]),
                                    input_time_ns=np.array([first.time_ns[0], second_time], dtype=np.int64),
                                    camera_times_ns=np.array([first.time_ns[0], first.right_time_ns[0],
                                                              second.time_ns[0], second.right_time_ns[0]], dtype=np.int64))
                record = {"modality": modality, "kind": kind, "indices": [i, j], "file": name,
                          "sha256": sha(args.output / name), "input_sha256": [tensor_sha(image1), tensor_sha(image2)],
                          "input_time_ns": [first.time_ns[0], second_time], "shape": list(expected),
                          "flow_min": flow.min().item(), "flow_max": flow.max().item(),
                          "covariance_min": covariance.min().item(), "covariance_max": covariance.max().item(),
                          "elapsed_seconds": elapsed, "peak_cuda_bytes": torch.cuda.max_memory_allocated()}
                if i == 0:
                    with torch.inference_mode():
                        repeat_flow, repeat_cov = model.inference(x, y)
                    record["repeat"] = {"flow_bitwise": torch.equal(flow, repeat_flow),
                                        "covariance_bitwise": torch.equal(covariance, repeat_cov),
                                        "flow_max_abs_delta": (flow - repeat_flow).abs().max().item(),
                                        "covariance_max_abs_delta": (covariance - repeat_cov).abs().max().item()}
                    del repeat_flow, repeat_cov
                report["pairs"].append(record)
                persist()
                print(f"PASS {modality} {kind} {i:06d}->{j:06d} {elapsed:.3f}s", flush=True)
                del x, y, flow, covariance
            del model, frames
            torch.cuda.empty_cache()
        assert len(report["pairs"]) == args.max_stereo_pairs * len(args.modalities)
        assert all(sha(path) == digest for path, digest in frozen.items()), "Source changed during execution"
        report["frozen_inputs_unchanged"] = True
        report["status"] = "PASS"
    except Exception as error:
        report["status"] = "FAIL"
        report["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        report["finished_at"] = datetime.now(ZoneInfo("America/New_York")).isoformat()
        persist()


if __name__ == "__main__":
    main()
