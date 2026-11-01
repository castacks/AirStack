"""Exact native-resolution TartanAir V1 RGB/forward-flow/source-mask packing."""
from pathlib import Path

import cv2
import numpy as np


def prepare_pair(image1, image2, flow_path, mask_path, output):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    images = []
    for path in (image1, image2):
        bgr = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if bgr is None or bgr.dtype != np.uint8 or bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError('source image must be uint8 BGR PNG with three channels')
        images.append(np.ascontiguousarray((bgr[:, :, ::-1].astype(np.float32) / np.float32(255)).transpose(2, 0, 1)))
    if images[0].shape != images[1].shape:
        raise ValueError('source image shapes differ')
    height, width = images[0].shape[1:]
    flow = np.load(flow_path, allow_pickle=False)
    mask = np.load(mask_path, allow_pickle=False)
    if flow.dtype != np.float32 or flow.shape != (height, width, 2):
        raise ValueError('provided forward flow must be float32[H,W,2]')
    if mask.dtype != np.uint8 or mask.shape != (height, width):
        raise ValueError('provided invalidity mask must be uint8[H,W]')
    arrays = {'image1':images[0], 'image2':images[1],
              'flow':np.ascontiguousarray(flow.transpose(2, 0, 1)),
              'valid_mask':np.ascontiguousarray((mask == 0)[None])}
    # Exclusive file creation; never silently overwrite a prepared sample.
    with output.open('xb') as stream:
        np.savez_compressed(stream, **arrays)
    return {'shape':[height,width], 'source_valid_pixels':int((mask == 0).sum()),
            'source_mask_codes':np.unique(mask).tolist()}
