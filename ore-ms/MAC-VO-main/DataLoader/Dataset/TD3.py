"""Manifest-backed TD3 camera-local sequence for provisional execution checks."""

import json
from pathlib import Path
import cv2
import numpy as np
import pypose as pp
import torch
import yaml

from ..Interface import StereoData, StereoFrame
from ..SequenceBase import SequenceBase


def read_uint8_rgb(path):
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(path)
    if image.dtype != np.uint8:
        raise ValueError("Execution adapter requires provided uint8 images; raw thermal is not normalized implicitly")
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    elif image.ndim == 3 and image.shape[-1] == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    else:
        raise ValueError(f"Unsupported image shape: {image.shape}")
    return torch.from_numpy(image.copy()).permute(2, 0, 1).unsqueeze(0).float() / 255.


class TD3ManifestSequence(SequenceBase[StereoFrame]):
    def __init__(self, dataset, manifest, modality, *, acknowledge_provisional_geometry=False):
        if not acknowledge_provisional_geometry:
            raise ValueError("Explicit acknowledgement of provisional camera-local geometry is required")
        if modality not in ("rgb", "thermal"):
            raise ValueError("modality must be rgb or thermal")
        self.root = Path(dataset).resolve()
        self.manifest = json.loads(Path(manifest).read_text())
        self.modality = modality
        self.rows = self.manifest["pairs"]
        names = ("multisense_left_rect_color_camera_info", "multisense_right_camera_info") if modality == "rgb" else (
            "thermal_left_processed_camera_info", "thermal_right_processed_camera_info")
        info = [yaml.safe_load((self.root / "sensors" / n / "data.yaml").read_text()) for n in names]
        left, right = [np.asarray(i["p"], dtype=np.float64) for i in info]
        if left.shape != (3, 4) or right.shape != (3, 4) or not np.isfinite([left, right]).all():
            raise ValueError("Expected finite stereo projection matrices")
        if left[0, 0] <= 0 or right[0, 0] <= 0 or left[1, 1] <= 0:
            raise ValueError("Projection focal lengths must be positive")
        self.signed_baseline = float(-right[0, 3] / right[0, 0] + left[0, 3] / left[0, 0])
        self.baseline = abs(self.signed_baseline)
        if not self.baseline > 0:
            raise ValueError("Stereo baseline must be nonzero")
        self.K = torch.tensor(left[:, :3], dtype=torch.float32).unsqueeze(0)
        self.height, self.width = info[0]["height"], info[0]["width"]
        if (self.height, self.width) != (info[1]["height"], info[1]["width"]):
            raise ValueError("Stereo image dimensions disagree")
        super().__init__(len(self.rows))

    def __getitem__(self, local_index):
        row = self.rows[self.get_index(local_index)]
        records = [row[self.modality + suffix] for suffix in ("_left", "_right")]
        images = []
        for record in records:
            path = (self.root / record["path"]).resolve()
            if not path.is_relative_to(self.root):
                raise ValueError("Manifest image path leaves dataset directory")
            image = read_uint8_rgb(path)
            if tuple(image.shape[-2:]) != (self.height, self.width):
                raise ValueError("Image dimensions disagree with camera-info")
            images.append(image)
        time_ns = [int(records[0]["time_ns"])]
        return StereoFrame(idx=[local_index], time_ns=time_ns, stereo=StereoData(
            T_BS=pp.identity_SE3(1, dtype=torch.float64), K=self.K.clone(),
            baseline=torch.tensor([self.baseline], dtype=torch.float64),
            time_ns=time_ns, height=self.height, width=self.width,
            imageL=images[0], imageR=images[1]))

    @classmethod
    def is_valid_config(cls, config):
        raise ValueError("Use the explicit bounded TD3 runner; this adapter is provisional")
