"""MS² source-protocol inputs with explicit thermal preprocessing and source times.

No exposure-time correction, rectification, pose interpretation or reference-depth
selection is implicit. The named thermal recipe is an audited producer replay;
its equivalence to the released MACT checkpoint's training recipe is unverified.
"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pypose as pp
import torch

from ..Interface import StereoData, StereoFrame
from ..SequenceBase import SequenceBase


THERMAL_RECIPE = "td3-firestereo-v2-clahe-bilateral-v1"
THERMAL_PRODUCER_SHA256 = "9e50052737bcddc97db6d5150b565352f05186ba6e1fa556b3dea1eaba58bd5c"


def thermal_producer_replay(raw):
    """Pure replay of archived process_image(firestereo_v2) then enhance_image.

    The percentile order-statistic indices, truncation and OpenCV parameters are
    deliberate source conventions. Never mutate raw pixels or rectify them again.
    """
    if raw.dtype != np.uint16 or raw.ndim != 2 or raw.size < 100:
        raise ValueError("Thermal input must be a nonempty uint16 grayscale image")
    ordered = np.sort(raw.reshape(-1))
    lower = ordered[round(ordered.size * .01)]
    upper = ordered[round(ordered.size * .99) - 1]
    if upper <= lower:
        raise ValueError("Degenerate thermal percentile range")
    clipped = np.clip(raw, lower, upper)
    image = (((clipped - lower) / (upper - lower)) * 255.0).astype(np.uint8)
    image = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(image)
    return cv2.bilateralFilter(image, 5, 20, 15)


@dataclass(kw_only=True)
class MS2StereoData(StereoData):
    right_time_ns: list[int]
    source_stem: list[str]
    input_transform: list[str]
    recorded_time_semantics: list[str]


class MS2ManifestSequence(SequenceBase[StereoFrame]):
    """Explicit source-protocol sequence; body/sensor identity is camera-local.

    root is the manifest's ore-ms path root. Thermal raw_pair() works without a
    transform, but model frames require the caller to name THERMAL_RECIPE. This
    is a scientific input choice, not confirmation of checkpoint provenance.
    """
    def __init__(self, root, manifest, modality, *, thermal_transform=None):
        if modality not in ("rgb", "thr"):
            raise ValueError("modality must be rgb or thr")
        if thermal_transform not in (None, THERMAL_RECIPE):
            raise ValueError("Unknown thermal preprocessing contract")
        if modality == "rgb" and thermal_transform is not None:
            raise ValueError("Thermal preprocessing cannot be applied to RGB")
        self.root = Path(root).resolve()
        self.manifest = json.loads(Path(manifest).read_text())
        self.rows = self.manifest["records"]
        self.modality = modality
        self.thermal_transform = thermal_transform
        if not self.rows:
            raise ValueError("Empty MS2 manifest")
        self._validate_rows()
        first = self.rows[0]["cameras"][modality]
        self.K = torch.tensor(first["K"], dtype=torch.float32).unsqueeze(0)
        self.baseline = first["baseline_m"]
        self.height, self.width = first["left"]["shape"][:2]
        super().__init__(len(self.rows))

    def _safe_path(self, record):
        path = (self.root / record["path"]).resolve()
        if not path.is_relative_to(self.root):
            raise ValueError("Manifest path leaves MS2 root")
        return path

    def _validate_rows(self):
        stems = [row["stem"] for row in self.rows]
        if stems != sorted(set(stems)):
            raise ValueError("Source stems must be unique and sorted")
        previous = {key: None for key in ("rgb_left", "rgb_right", "thr_left", "thr_right")}
        geometry = {}
        for row in self.rows:
            for sensor in ("rgb", "thr"):
                camera = row["cameras"][sensor]
                K = np.asarray(camera["K"], dtype=np.float64)
                baseline = camera["baseline_m"]
                if (K.shape != (3, 3) or not np.isfinite(K).all()
                        or K[0, 0] <= 0 or K[1, 1] <= 0
                        or not np.array_equal(K[2], [0, 0, 1])
                        or type(baseline) not in (int, float)
                        or not np.isfinite(baseline) or baseline <= 0):
                    raise ValueError("Invalid MS2 stereo calibration")
                signature = (K.tolist(), baseline, camera["left"]["shape"])
                if sensor in geometry and signature != geometry[sensor]:
                    raise ValueError("Variable calibration/shape is not supported")
                geometry[sensor] = signature
                for side in ("left", "right"):
                    rec = camera[side]
                    image_path = self._safe_path(rec)
                    if image_path.stem != row["stem"] or image_path.suffix != ".png":
                        raise ValueError("Image path does not match its source ID")
                    expected_dtype = "uint8" if sensor == "rgb" else "uint16"
                    shape = rec["shape"]
                    if (rec["dtype"] != expected_dtype or len(shape) != (3 if sensor == "rgb" else 2)
                            or (sensor == "rgb" and shape[2] != 3)
                            or shape[:2] != camera["left"]["shape"][:2]
                            or any(type(v) is not int or v <= 0 for v in shape)):
                        raise ValueError("Unexpected MS2 image schema")
                    t = rec["recorded_time_ns"]
                    key = sensor + "_" + side
                    if type(t) is not int or t < 0 or (previous[key] is not None and t <= previous[key]):
                        raise ValueError("Source camera times must be increasing integer ns")
                    previous[key] = t

    def source_record(self, local_index):
        return deepcopy(self.rows[self.get_index(local_index)])

    def _read(self, record):
        path = self._safe_path(record)
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != record["sha256"]:
            raise ValueError("Source file hash mismatch")
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError("Cannot decode MS2 PNG")
        return image

    def raw_pair(self, local_index):
        camera = self.rows[self.get_index(local_index)]["cameras"][self.modality]
        result = []
        for side in ("left", "right"):
            record = camera[side]
            image = self._read(record)
            if list(image.shape) != record["shape"] or str(image.dtype) != record["dtype"]:
                raise ValueError("Decoded source differs from declared image schema")
            result.append(image)
        return tuple(result)

    def __getitem__(self, local_index):
        if self.modality == "thr" and self.thermal_transform is None:
            raise ValueError("Thermal model input requires an explicit named preprocessing contract")
        row = self.rows[self.get_index(local_index)]
        camera = row["cameras"][self.modality]
        images = []
        for raw in self.raw_pair(local_index):
            if self.modality == "thr":
                gray = thermal_producer_replay(raw)
                image = np.repeat(gray[..., None], 3, axis=2)
            else:
                image = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
            tensor = torch.from_numpy(image.copy()).permute(2, 0, 1).unsqueeze(0).float() / 255.
            images.append(tensor)
        left_time = [camera["left"]["recorded_time_ns"]]
        return StereoFrame(idx=[local_index], time_ns=left_time.copy(), stereo=MS2StereoData(
            T_BS=pp.identity_SE3(1, dtype=torch.float64), K=self.K.clone(),
            baseline=torch.tensor([self.baseline], dtype=torch.float64),
            time_ns=left_time.copy(), right_time_ns=[camera["right"]["recorded_time_ns"]],
            source_stem=[row["stem"]], recorded_time_semantics=["unverified-source-recorded-time"],
            input_transform=[self.thermal_transform if self.modality == "thr" else "rgb-u8-rgb01-v1"],
            height=self.height, width=self.width, imageL=images[0], imageR=images[1]))

    def depth(self, local_index, variant):
        if variant not in ("depth", "depth_multi", "depth_filtered"):
            raise ValueError("Choose an explicit MS2 depth reference variant")
        record = self.rows[self.get_index(local_index)]["references"][self.modality][variant]
        image = self._read(record)
        if image.dtype != np.uint16 or image.shape != (self.height, self.width):
            raise ValueError("Expected native uint16 camera-sized reference depth")
        if record["meters_per_code"] != 1 / 256 or record["zero_invalid"] is not True:
            raise ValueError("Unexpected source depth encoding")
        return image.astype(np.float32) / 256., image != 0

    @classmethod
    def is_valid_config(cls, config):
        raise ValueError("Use the explicit source-protocol MS2 manifest and named thermal contract")
