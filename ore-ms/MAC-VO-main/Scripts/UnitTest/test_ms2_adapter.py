"""MS2 contract tests with real Torch/PyPose; no checkpoint or GPU required."""
from copy import deepcopy
import hashlib
import json

import cv2
import numpy as np
import pytest
import torch

from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE, thermal_producer_replay


@pytest.fixture
def pilot(tmp_path):
    records = []
    for i in range(3):
        stem = f"{i:06d}"
        row = {"stem": stem, "cameras": {}, "references": {}}
        for sensor in ("rgb", "thr"):
            camera = {"K": [[20., 0., 16.], [0., 20., 8.], [0., 0., 1.]], "baseline_m": .3}
            for side in ("left", "right"):
                if sensor == "rgb":
                    image = np.tile(np.array([0, 128, 255], np.uint8), (16, 32, 1))
                else:
                    image = np.arange(512, dtype=np.uint16).reshape(16, 32) + 3000 + i
                path = tmp_path / sensor / side / (stem + ".png")
                path.parent.mkdir(parents=True, exist_ok=True)
                assert cv2.imwrite(str(path), image)
                camera[side] = {"path": str(path.relative_to(tmp_path)),
                                "shape": list(image.shape), "dtype": str(image.dtype),
                                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                                "recorded_time_ns": 1_600_000_000_000_000_000 + i * 100_000_000
                                + (34_000_000 if side == "right" else 0)}
            row["cameras"][sensor] = camera
            depth = np.full((16, 32), 512, dtype=np.uint16)
            depth[0, 0] = 0
            path = tmp_path / sensor / "depth" / (stem + ".png")
            path.parent.mkdir(parents=True, exist_ok=True)
            assert cv2.imwrite(str(path), depth)
            row["references"][sensor] = {"depth": {
                "path": str(path.relative_to(tmp_path)),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "meters_per_code": 1 / 256, "zero_invalid": True}}
        records.append(row)
    manifest = tmp_path / "manifest.json"
    manifest.write_text(json.dumps({"records": records}))
    return tmp_path, manifest, records


def sequence(pilot, modality="rgb", **kwargs):
    root, manifest, _ = pilot
    return MS2ManifestSequence(root, manifest, modality, **kwargs)


def rewrite(pilot, mutate):
    root, manifest, records = pilot
    data = deepcopy(records)
    mutate(data)
    manifest.write_text(json.dumps({"records": data}))


def test_rgb_preserves_exact_times_color_and_calibration(pilot):
    frame = sequence(pilot)[0]
    expected = torch.tensor([255, 128, 0], dtype=torch.float32) / 255
    assert torch.equal(frame.stereo.imageL[0, :, 0, 0], expected)
    assert frame.stereo.imageL.shape == (1, 3, 16, 32)
    assert frame.time_ns == [1_600_000_000_000_000_000]
    assert frame.stereo.right_time_ns == [1_600_000_000_034_000_000]
    assert frame.stereo.baseline.item() == .3
    assert frame.gt_pose is None and frame.stereo.gt_depth is None
    assert frame.stereo.recorded_time_semantics == ["unverified-source-recorded-time"]


def test_raw_thermal_access_and_explicit_transform(pilot):
    raw_sequence = sequence(pilot, "thr")
    raw = raw_sequence.raw_pair(0)[0]
    assert raw.dtype == np.uint16
    with pytest.raises(ValueError, match="explicit named"):
        raw_sequence[0]
    thermal = sequence(pilot, "thr", thermal_transform=THERMAL_RECIPE)
    original = thermal.raw_pair(0)[0]
    frame = thermal[0]
    assert np.array_equal(original, thermal.raw_pair(0)[0])
    assert frame.stereo.imageL.dtype == torch.float32
    assert 0 <= frame.stereo.imageL.min() <= frame.stereo.imageL.max() <= 1
    assert torch.equal(frame.stereo.imageL[:, 0], frame.stereo.imageL[:, 1])
    assert frame.stereo.input_transform == [THERMAL_RECIPE]


def test_clip_collate_and_independent_metadata(pilot):
    seq = sequence(pilot).clip(1, 3)
    first, second = seq[0], seq[1]
    assert first.idx == [0] and first.stereo.source_stem == ["000001"]
    assert first.time_ns == [1_600_000_000_100_000_000]
    batch = seq.collate_fn([first, second])
    assert batch.stereo.source_stem == ["000001", "000002"]
    assert batch.stereo.right_time_ns == [1_600_000_000_134_000_000, 1_600_000_000_234_000_000]
    assert batch.stereo.imageL.shape == (2, 3, 16, 32)
    record = seq.source_record(0)
    record["stem"] = "changed"
    assert seq.source_record(0)["stem"] == "000001"


def test_depth_is_explicit_and_zeros_are_masked(pilot):
    seq = sequence(pilot)
    depth, valid = seq.depth(0, "depth")
    assert depth.dtype == np.float32 and depth[1, 1] == 2.
    assert depth[0, 0] == 0 and not valid[0, 0] and valid[1, 1]
    assert seq[0].stereo.gt_depth is None
    with pytest.raises(ValueError, match="explicit"):
        seq.depth(0, "automatic")


@pytest.mark.parametrize("field", ["path", "sha256", "recorded_time_ns", "dtype", "shape", "baseline_m", "stem"])
def test_corrupt_manifest_rejected(pilot, field):
    def mutate(rows):
        cam = rows[1]["cameras"]["rgb"]
        if field == "path": cam["left"][field] = "../000001.png"
        elif field == "sha256": cam["left"][field] = "0" * 64
        elif field == "recorded_time_ns": cam["left"][field] = rows[0]["cameras"]["rgb"]["left"][field]
        elif field == "dtype": cam["left"][field] = "uint16"
        elif field == "shape": cam["left"][field] = [15, 32, 3]
        elif field == "baseline_m": cam[field] = 0
        elif field == "stem": rows[1][field] = "000000"
    rewrite(pilot, mutate)
    with pytest.raises(ValueError):
        sequence(pilot)[1]


def test_decoded_dtype_is_checked_even_when_manifest_hash_matches(pilot):
    root, _, _ = pilot
    path = root / "rgb/left/000000.png"
    cv2.imwrite(str(path), np.ones((16, 32), np.uint16))
    rewrite(pilot, lambda rows: rows[0]["cameras"]["rgb"]["left"].update(
        sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    with pytest.raises(ValueError, match="Decoded source"):
        sequence(pilot)[0]


def test_thermal_contract_rejects_unknown_or_rgb_use(pilot):
    with pytest.raises(ValueError): sequence(pilot, "thr", thermal_transform="display-percentile")
    with pytest.raises(ValueError): sequence(pilot, "rgb", thermal_transform=THERMAL_RECIPE)
    with pytest.raises(ValueError): thermal_producer_replay(np.ones((16, 32), np.uint16))
    with pytest.raises(ValueError): thermal_producer_replay(np.ones((16, 32), np.uint8))


def test_upstream_resize_preserves_source_metadata(pilot):
    from DataLoader.Transform import ScaleFrame
    frame = sequence(pilot)[0]
    original_times = (frame.stereo.time_ns.copy(), frame.stereo.right_time_ns.copy())
    scaled = ScaleFrame.scale_stereo(frame.stereo, 2., 2., "bilinear")
    assert scaled.imageL.shape == (1, 3, 8, 16)
    assert scaled.K[0, 0, 0] == 10 and scaled.K[0, 0, 2] == 8
    assert (scaled.time_ns, scaled.right_time_ns) == original_times
    assert scaled.source_stem == ["000000"]
