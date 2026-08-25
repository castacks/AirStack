"""Score one RGB view against a target — VLFM's image-text matching step.

Talks to `search_baselines.itm_server`, a shared BLIP-2 ITM service. Shared and
not per-robot because VLFM scores SINGLE FRAMES WITH NO HISTORY: there is no
per-robot state on the scorer, so interleaving requests from different robots
cannot produce cross-talk. Measured 17.4 ms per call (57 Hz), 8.0 ms/image
batched, on 2.5 GiB — a fleet's worth of headroom from one model.

A failed call returns None, and None is NOT zero. Zero means "this view is
actively unpromising", which is a claim; a failed call is an absence of one, and
the value map must not be told otherwise.
"""

import base64
import io
import time

import numpy as np
import requests
from PIL import Image

# Per-call telemetry, same shape and purpose as chat_utils.LAST_CALL.
LAST_SCORE = {}

DEFAULT_URL = "http://localhost:8100"


def _encode(rgb, quality=85, max_side=512):
    """RGB ndarray -> base64 JPEG.

    Downscaled because BLIP-2's vision tower resizes to 224 internally anyway,
    so anything above a few hundred pixels is bytes on a socket and nothing else.
    """
    img = Image.fromarray(np.asarray(rgb, dtype=np.uint8))
    if max(img.size) > max_side:
        s = max_side / float(max(img.size))
        img = img.resize((max(1, int(img.size[0] * s)),
                          max(1, int(img.size[1] * s))), Image.BILINEAR)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    raw = buf.getvalue()
    return base64.b64encode(raw).decode("utf-8"), len(raw)


def health(base_url=DEFAULT_URL, timeout=5.0):
    r = requests.get(f"{base_url.rstrip('/')}/health", timeout=timeout)
    r.raise_for_status()
    return r.json()


def score_view_tiled(rgb, target, rows=2, cols=3, base_url=DEFAULT_URL,
                     timeout=10.0, max_side=512, text=None):
    """Per-COLUMN scores for this view, left to right, in [0, 1].

    Why tile at all: BLIP-2 resizes whatever it is given to 224x224, so a small
    distant target is destroyed before the model sees it. Measured on a fire
    scene containing ~8 people, each ~13 px wide in a 946 px frame (~3 px after
    the resize):

        whole frame          -> 0.090     <- BELOW a flat grey image (0.129)
        2x3 tiles, max       -> 0.423
        3x4 tiles, max       -> 0.435
        flat grey, 3x4 max   -> 0.129     <- tiling does NOT inflate a blank

    So the whole-frame score is not merely weak for aerial targets, it is
    ANTI-correlated: an empty view outscores a populated one. Tiling restores
    the signal because each tile is upscaled rather than downscaled.

    Rows are collapsed by MAX, not mean: a person in the lower half of the frame
    is evidence about that bearing regardless of what the sky above them looks
    like, and averaging would dilute it with the empty tile above.

    `rows=1, cols=1` gives canonical whole-frame VLFM, for a fidelity comparison.
    """
    LAST_SCORE.clear()
    arr = np.asarray(rgb)
    H, W = arr.shape[:2]
    rows = max(1, int(rows)); cols = max(1, int(cols))
    th, tw = H // rows, W // cols
    if th < 8 or tw < 8:
        rows = cols = 1; th, tw = H, W

    tiles, nbytes = [], 0
    for r in range(rows):
        for c in range(cols):
            sub = arr[r * th:(r + 1) * th, c * tw:(c + 1) * tw]
            b64, n = _encode(sub, max_side=max_side)
            tiles.append(b64); nbytes += n

    payload = {"images": tiles}
    if text:
        payload["text"] = text
    else:
        payload["target"] = target
    t0 = time.time()
    try:
        r = requests.post(f"{base_url.rstrip('/')}/score", json=payload,
                          timeout=timeout)
        r.raise_for_status()
        body = r.json()
    except Exception as exc:
        LAST_SCORE.update({"ok": False, "error": str(exc),
                           "round_trip_s": time.time() - t0, "bytes": nbytes})
        return None
    dt = time.time() - t0
    scores = body.get("scores") or []
    if len(scores) != rows * cols:
        LAST_SCORE.update({"ok": False, "error": f"got {len(scores)} scores",
                           "round_trip_s": dt, "bytes": nbytes})
        return None
    grid = np.asarray(scores, dtype=float).reshape(rows, cols)
    per_col = grid.max(axis=0)
    LAST_SCORE.update({
        "ok": True,
        "score": float(per_col.max()),
        "per_column": [float(v) for v in per_col],
        "tiles": [rows, cols],
        "round_trip_s": dt,
        "server_s": body.get("server_s"),
        "bytes": nbytes,
        "text": body.get("text"),
    })
    return [float(v) for v in per_col]


def score_view(rgb, target, base_url=DEFAULT_URL, timeout=10.0, max_side=512,
               text=None):
    """P(match) in [0, 1] for this view, or None if the call failed."""
    LAST_SCORE.clear()
    b64, nbytes = _encode(rgb, max_side=max_side)
    payload = {"images": [b64]}
    if text:
        payload["text"] = text
    else:
        payload["target"] = target
    t0 = time.time()
    try:
        r = requests.post(f"{base_url.rstrip('/')}/score", json=payload,
                          timeout=timeout)
        r.raise_for_status()
        body = r.json()
    except Exception as exc:
        LAST_SCORE.update({"ok": False, "error": str(exc),
                           "round_trip_s": time.time() - t0, "bytes": nbytes})
        return None
    dt = time.time() - t0
    scores = body.get("scores") or []
    if not scores:
        LAST_SCORE.update({"ok": False, "error": "empty scores",
                           "round_trip_s": dt, "bytes": nbytes})
        return None
    LAST_SCORE.update({
        "ok": True,
        "score": float(scores[0]),
        "round_trip_s": dt,
        "server_s": body.get("server_s"),
        "bytes": nbytes,
        "text": body.get("text"),
    })
    return float(scores[0])
