"""Verify MACT weights against recovered FlowFormerCov; optionally run synthetic inference.

Run from the repository root with the project Python environment:
    python3 Scripts/Setup/verify_mact_checkpoint.py --forward
Synthetic outputs test execution only, not thermal accuracy or preprocessing.
"""

import argparse
import sys
import time
import json
from pathlib import Path

import torch

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from Module.Network.FlowFormer.configs.submission import get_cfg
from Module.Network.FlowFormerCov import build_flowformer
from Module.Network.FlowFormerCov.checkpoint import load_cov_checkpoint


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path,
                        default=REPO_ROOT.parent / "models/macslam/MACT_Offroad.pth")
    parser.add_argument("--forward", action="store_true")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--height", type=int, default=128)
    parser.add_argument("--width", type=int, default=128)
    parser.add_argument("--allow-legacy-rgb", action="store_true",
                        help="Allow only the audited released-RGB checkpoint migration")
    args = parser.parse_args()

    cfg = get_cfg()
    cfg.latentcostformer.pretrain = False
    model = build_flowformer(cfg, torch.float32, torch.float32).eval()
    weights = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    report = load_cov_checkpoint(model, weights, allow_legacy=args.allow_legacy_rgb)
    print(f"PASS checkpoint load with strict active-state validation: {len(weights)} entries")
    print(json.dumps(report))
    print(f"Model parameters: {sum(p.numel() for p in model.parameters())}")

    if args.forward:
        if args.height < 128 or args.width < 128:
            parser.error("Use dimensions of at least 128 for the encoder smoke test")
        if args.device.startswith("cuda"):
            print(f"GPU: {torch.cuda.get_device_name(args.device)}")
            print(f"Torch: {torch.__version__}; CUDA runtime: {torch.version.cuda}")
        torch.manual_seed(0)
        model = model.to(args.device)
        images = torch.rand(2, 3, args.height, args.width, device=args.device)
        start = time.perf_counter()
        with torch.inference_mode():
            flow, covariance = model.inference(images[:1], images[1:])
        if args.device.startswith("cuda"):
            torch.cuda.synchronize(args.device)
        elapsed = time.perf_counter() - start
        expected = (1, 2, args.height, args.width)
        assert tuple(flow.shape) == expected, flow.shape
        assert tuple(covariance.shape) == expected, covariance.shape
        assert torch.isfinite(flow).all(), "Nonfinite flow"
        assert torch.isfinite(covariance).all(), "Nonfinite covariance"
        assert (covariance > 0).all(), "Nonpositive covariance"
        print(f"PASS synthetic inference: flow/covariance {expected}, finite, positive covariance")
        print(f"Single forward elapsed: {elapsed:.3f}s (includes first-use overhead)")


if __name__ == "__main__":
    main()
