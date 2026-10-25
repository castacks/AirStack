"""Independent source/producer and saved-array audit for the MS² smoke test."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE, THERMAL_PRODUCER_SHA256


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", type=Path, required=True)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--run", type=Path)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    assert not args.output.exists(), "Do not overwrite an audit"
    source = json.loads(args.manifest.read_text())
    producer = args.root / "results/ms2-mac-adapter/inputs/get_thermal_data.py"
    assert sha(producer) == THERMAL_PRODUCER_SHA256
    functions = [n for n in ast.parse(producer.read_text()).body
                 if isinstance(n, ast.FunctionDef) and n.name in ("process_image", "enhance_image")]
    assert len(functions) == 2
    scope = {"np": np, "cv2": cv2}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(producer), "exec"), scope)
    cv2.setNumThreads(1)
    inputs = {}
    raw_files = {}
    for modality in ("rgb", "thr"):
        sequence = MS2ManifestSequence(args.root, args.manifest, modality,
                                      thermal_transform=THERMAL_RECIPE if modality == "thr" else None)
        assert len(sequence) == 20
        for i, row in enumerate(source["records"]):
            frame = sequence[i]
            camera = row["cameras"][modality]
            assert frame.stereo.source_stem == [row["stem"]]
            assert frame.time_ns == frame.stereo.time_ns == [camera["left"]["recorded_time_ns"]]
            assert frame.stereo.right_time_ns == [camera["right"]["recorded_time_ns"]]
            for side, tensor in [("left", frame.stereo.imageL), ("right", frame.stereo.imageR)]:
                record = camera[side]
                path = args.root / record["path"]
                assert sha(path) == record["sha256"]
                raw_files[str(path)] = record["sha256"]
                raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
                assert str(raw.dtype) == record["dtype"] and list(raw.shape) == record["shape"]
                if modality == "thr":
                    image = scope["enhance_image"](scope["process_image"](raw.copy(), "firestereo_v2"))
                    image = np.repeat(image[..., None], 3, axis=2)
                else:
                    image = raw[..., ::-1].copy()
                expected = (image.astype(np.float32) / np.float32(255)).transpose(2, 0, 1)[None]
                assert np.array_equal(tensor.numpy(), expected), (modality, i, side)
                inputs[modality, i, side] = hashlib.sha256(expected.tobytes()).hexdigest()
            if i == 0:
                with np.load(args.root / f"results/ms2-mac-adapter/{modality}-first-input.npz") as prior:
                    assert np.array_equal(frame.stereo.imageL.numpy(), prior["left"])
                    assert np.array_equal(frame.stereo.imageR.numpy(), prior["right"])
    checked_pairs = 0
    repeats = []
    if args.run:
        report = json.loads((args.run / "report.json").read_text())
        assert report["status"] == "PASS" and report["frozen_inputs_unchanged"]
        modalities = report["arguments"]["modalities"]
        assert modalities and set(modalities) <= {"rgb", "thr"}
        assert set(report["modalities"]) == set(modalities)
        expected_jobs = {(m, k, i, j) for m in modalities for k, i, j in
                         ([("stereo", i, i) for i in range(20)] + [("temporal", i, i + 1) for i in range(19)])}
        seen = set()
        for record in report["pairs"]:
            m, kind = record["modality"], record["kind"]
            i, j = record["indices"]
            key = (m, kind, i, j)
            assert key not in seen
            seen.add(key)
            side = "right" if kind == "stereo" else "left"
            assert record["input_sha256"] == [inputs[m, i, "left"], inputs[m, j, side]]
            path = args.run / record["file"]
            assert sha(path) == record["sha256"]
            first = source["records"][i]["cameras"][m]
            second = source["records"][j]["cameras"][m]
            shape = (1, 2, *first["left"]["shape"][:2])
            with np.load(path, allow_pickle=False) as data:
                for field in ("flow", "covariance"):
                    a = data[field]
                    assert a.shape == shape and a.dtype == np.float32 and np.isfinite(a).all()
                    assert float(a.min()) == record[field + "_min"] and float(a.max()) == record[field + "_max"]
                assert (data["covariance"] > 0).all()
                assert data["input_time_ns"].dtype == np.int64
                assert data["input_time_ns"].tolist() == record["input_time_ns"] == [first["left"]["recorded_time_ns"], second[side]["recorded_time_ns"]]
                assert data["camera_times_ns"].tolist() == [first[s]["recorded_time_ns"] for s in ("left", "right")] + [second[s]["recorded_time_ns"] for s in ("left", "right")]
                assert data["source_ids"].tolist() == [source["records"][index]["stem"] for index in (i, j)]
            if "repeat" in record:
                repeats.append({"modality": m, "kind": kind, **record["repeat"]})
            checked_pairs += 1
        assert seen == expected_jobs and checked_pairs == 39 * len(modalities)
        for path, digest in report["frozen_inputs"].items():
            assert sha(path) == digest
        for modality in report["modalities"].values():
            assert sha(modality["checkpoint"]) == modality["checkpoint_sha256"]
    assert all(sha(path) == digest for path, digest in raw_files.items())
    summary = {"status": "PASS", "source_images_checked": len(raw_files), "producer_model_tensors_exact": len(inputs),
               "historical_first_input_tensors_exact": 4, "saved_pairs_checked": checked_pairs,
               "opencv": cv2.__version__, "numpy": np.__version__, "repeats": repeats}
    args.output.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
