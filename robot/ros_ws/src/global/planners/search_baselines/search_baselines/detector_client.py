"""Detect + segment one frame on the SHARED detector service.

Talks to `search_baselines.detector_server` (one instance for the whole fleet,
on the offboard-compute container) and returns exactly what the in-process
detector returned: an `sv.Detections` with `xyxy`, `confidence`, `class_id` and
`mask`. That is not a stylistic choice — the vendored mapping code indexes all
four (`ros2_agents.py`: `len(detections.xyxy)`, `detections.class_id[i]`,
`detections.confidence[i]`, `detections.mask[i]` into `create_object_pcd`), the
vendored overlay ITERATES the object as a 6-tuple (`visualization.py`), and
`AirStackAgent` wraps `.detect` for its `last_detection` telemetry. Anything
duck-typed would have to reproduce all of that; the real class is one import.

`RemoteDetector` is the drop-in for the vendored
`Object_Detection_and_Segmentation`: same `.detect(image)`, and the same
`.yolo_model_w_classes` attribute `AirStackAgent` probes to decide open-vocab
vs closed-set. It loads NOTHING locally — no YOLO, no SAM, no CUDA context.

A failed call RAISES, and that is deliberate. The ITM client returns None
because "no score" is meaningfully different from a score of zero and the value
map can be left untouched; here the only two options are "these are the
detections" and "we do not know", and quietly returning an empty `Detections`
would tell the mapper that this frame contains no people. `_plan_loop` already
catches per-tick exceptions and logs them, so a raise costs one tick and shows
up in the log, where a silent empty frame would not.
"""

import base64
import io
import os
import time

import numpy as np
import requests
from PIL import Image

import supervision as sv

# Per-call telemetry, same shape and purpose as itm_client.LAST_SCORE.
LAST_DETECT = {}

# The fleet's detector lives on the offboard-compute container; the planner
# passes `detector_url` explicitly, so this default only matters for a one-off
# script or a server started next to the caller.
DEFAULT_URL = os.environ.get('SEARCH_BASELINES_DETECTOR_URL',
                             'http://offboard-compute:8200')
DEFAULT_TIMEOUT = 30.0
DEFAULT_CONF = 0.1                     # the vendored detector's own threshold


class DetectorError(RuntimeError):
    """The shared detector could not be reached, or refused the request."""


# ── image / mask codecs (the exact inverse of detector_server's) ─────────────

def encode_image(image, fmt='png', quality=90):
    """HxWx3 uint8 -> (base64 str, n_bytes), CHANNEL ORDER UNTOUCHED.

    PNG by default because the array must arrive bit-exact: it is what YOLO
    sees, and the masks it produces index the depth image pixel for pixel. The
    array handed to a detector here is BGR (upstream's `transform_rgb_bgr`);
    neither side interprets it, both just carry it, which is why this uses PIL
    rather than `cv2.imencode`/`imdecode` (cv2 swaps channels on decode).
    """
    arr = np.asarray(image)
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    if arr.ndim != 3 or arr.shape[2] != 3:
        raise ValueError(f'expected HxWx3 uint8, got {arr.shape} {arr.dtype}')
    buf = io.BytesIO()
    img = Image.fromarray(arr)
    if fmt.lower() in ('jpg', 'jpeg'):
        img.save(buf, format='JPEG', quality=int(quality))
    else:
        img.save(buf, format='PNG')
    raw = buf.getvalue()
    return base64.b64encode(raw).decode('utf-8'), len(raw)


def decode_mask(blob, dtype=None):
    """One encoded mask -> the ndarray the server encoded, bit for bit."""
    enc = blob.get('encoding', 'png')
    shape = tuple(int(v) for v in blob.get('shape', ()))
    if enc == 'rle':
        counts = list(blob.get('counts', []))
        flat = np.zeros(int(np.prod(shape)) if shape else 0, dtype=np.uint8)
        pos, val = 0, 0
        for c in counts:
            c = int(c)
            if val and c:
                flat[pos:pos + c] = 1
            pos += c
            val ^= 1
        if pos != flat.size:
            raise DetectorError(
                f'RLE covers {pos} px, mask is {flat.size} px — truncated response')
        out = flat.reshape(shape) if shape else flat
    elif enc == 'png':
        img = Image.open(io.BytesIO(base64.b64decode(blob['data'])))
        out = (np.asarray(img) > 127).astype(np.uint8)
        if shape and out.shape != shape:
            raise DetectorError(f'mask is {out.shape}, header says {shape}')
    else:
        raise DetectorError(f'unknown mask encoding {enc!r}')
    # The dtype SAM produced (bool), so downstream sees what it saw in-process.
    return out.astype(np.dtype(dtype)) if dtype else out.astype(bool)


def to_detections(body):
    """A /detect response -> `sv.Detections`, the object the vendor consumes."""
    dets = body.get('detections') or []
    n = len(dets)
    xyxy = np.asarray([d['xyxy'] for d in dets], dtype=np.float32).reshape(n, 4)
    conf = np.asarray([d['confidence'] for d in dets], dtype=np.float32)
    cid = np.asarray([d['class_id'] for d in dets], dtype=int)
    masks = None
    if n and 'mask' in dets[0]:
        dtype = body.get('mask_dtype')
        masks = np.stack([decode_mask(d['mask'], dtype) for d in dets])
    # Upstream builds `mask=None` when nothing was detected; mirror that rather
    # than an empty stack, so `detections.mask` is None in exactly the same
    # cases it used to be.
    return sv.Detections(xyxy=xyxy, confidence=conf, class_id=cid, mask=masks)


# ── calls ────────────────────────────────────────────────────────────────────

def health(base_url=DEFAULT_URL, timeout=10.0):
    """The server's model, vocabulary and device. Raises if it is not there."""
    try:
        r = requests.get(f'{base_url.rstrip("/")}/health', timeout=timeout)
        r.raise_for_status()
        return r.json()
    except Exception as exc:
        raise DetectorError(
            f'no detector service at {base_url} ({exc})') from exc


def metrics(base_url=DEFAULT_URL, timeout=10.0):
    r = requests.get(f'{base_url.rstrip("/")}/metrics', timeout=timeout)
    r.raise_for_status()
    return r.json()


def detect(image, classes=None, conf=DEFAULT_CONF, base_url=DEFAULT_URL,
           timeout=DEFAULT_TIMEOUT, mask_encoding='png', image_format='png',
           max_det=None, session=None, retries=1):
    """Detections for one frame, from the shared service.

    `image` is the array the vendored detector would have been handed (BGR
    uint8), `classes` the open-vocabulary prompt (ignored by a closed-set
    checkpoint, exactly as `set_classes` is). Raises `DetectorError` rather
    than returning an empty result — see the module docstring.
    """
    LAST_DETECT.clear()
    b64, nbytes = encode_image(image, fmt=image_format)
    payload = {'image': b64, 'conf': float(conf), 'mask_encoding': mask_encoding}
    if classes:
        payload['classes'] = [str(c) for c in classes]
    if max_det:
        payload['max_det'] = int(max_det)

    post = (session or requests).post
    url = f'{base_url.rstrip("/")}/detect'
    t0 = time.time()
    last_exc = None
    for attempt in range(max(1, int(retries) + 1)):
        try:
            r = post(url, json=payload, timeout=timeout)
            if r.status_code >= 400:
                # A 4xx is OUR bug (bad image, unknown encoding) and retrying it
                # just burns ticks; only transport faults are worth a second go.
                detail = r.text[:300]
                raise DetectorError(
                    f'detector returned HTTP {r.status_code}: {detail}')
            body = r.json()
            break
        except DetectorError:
            raise
        except Exception as exc:
            last_exc = exc
            if attempt >= retries:
                LAST_DETECT.update({'ok': False, 'error': str(exc),
                                    'round_trip_s': time.time() - t0,
                                    'bytes': nbytes})
                raise DetectorError(
                    f'detector call to {url} failed after {attempt + 1} '
                    f'attempt(s): {exc}. Is the shared detector running? '
                    f'`docker exec offboard-compute curl -s localhost:8200/health`'
                ) from exc
            time.sleep(0.2 * (attempt + 1))
    else:                                        # pragma: no cover
        raise DetectorError(str(last_exc))

    dt = time.time() - t0
    det = to_detections(body)
    LAST_DETECT.update({
        'ok': True,
        'n': int(body.get('n', 0)),
        'round_trip_s': dt,
        'server_s': body.get('server_s'),
        'yolo_s': body.get('yolo_s'),
        'sam_s': body.get('sam_s'),
        'bytes': nbytes,
        'vocab_switched': body.get('vocab_switched'),
    })
    return det


# ── the drop-in for the vendored detector ────────────────────────────────────

class _RemoteModel:
    """Stands in for `obj_det_seg.yolo_model_w_classes`.

    `AirStackAgent` asks the model two questions before it can resolve
    `goal_id`: does it have a REAL `set_classes` (open-vocabulary), and what
    are its `names`. Both are answered from the server's /health, so the client
    behaves the same whether the checkpoint is loaded here or over there.

    A closed-set instance genuinely has NO `set_classes` attribute — the
    open-vocab probe is `getattr(model, 'set_classes', None)`, and answering it
    with a shim would send a COCO checkpoint down the open-vocab path (the
    `IndexError` documented in airstack_agent.py).
    """

    def __init__(self, owner, names, open_vocab):
        self._owner = owner
        self.names = dict(names)
        if open_vocab:
            self.set_classes = self._set_classes

    def _set_classes(self, classes):
        """Record the prompt vocabulary; the server switches on next request.

        Deliberately NOT an eager RPC: the server takes `classes` with every
        /detect and switches only when the list changes, so the vocabulary
        cannot drift out of sync with the ids in a response even if several
        planners point at one server.
        """
        self._owner.classes = [str(c) for c in classes]


class RemoteDetector:
    """`Object_Detection_and_Segmentation` with the model on another machine.

    Constructed with the same three positional arguments as the vendored class
    (`args`, `classes`, `device`) so it can be substituted for it wherever that
    is constructed, and ignoring `device` because nothing is loaded here.
    """

    def __init__(self, args=None, classes=None, device=None,
                 base_url=DEFAULT_URL, timeout=DEFAULT_TIMEOUT,
                 conf=DEFAULT_CONF, mask_encoding='png', image_format='png',
                 health_info=None, retries=1):
        self.args = args
        self.device = device
        self.base_url = base_url
        self.timeout = float(timeout)
        self.conf = float(conf)
        self.mask_encoding = mask_encoding
        self.image_format = image_format
        self.retries = int(retries)
        self.classes = [str(c) for c in (classes or [])]
        # One session per detector: HTTP keep-alive matters here, where the
        # ITM's tiny payloads did not — a full-resolution PNG per frame pays
        # for a new TCP connection every time otherwise.
        self.session = requests.Session()
        info = health_info if health_info is not None else health(base_url, timeout)
        self.health_info = info
        self.open_vocab = bool(info.get('open_vocab'))
        names = {int(k): str(v) for k, v in (info.get('names') or {}).items()}
        self.yolo_model_w_classes = _RemoteModel(self, names, self.open_vocab)
        # The vendored class exposes this; nothing here needs it, but code that
        # pokes at it should find None rather than an AttributeError.
        self.sam_predictor = None

    def detect(self, image, *args, **kwargs):
        return detect(image, classes=(self.classes if self.open_vocab else None),
                      conf=self.conf, base_url=self.base_url,
                      timeout=self.timeout, mask_encoding=self.mask_encoding,
                      image_format=self.image_format, session=self.session,
                      retries=self.retries)

    def __repr__(self):
        return (f'<RemoteDetector {self.base_url} '
                f'{"open-vocab" if self.open_vocab else "closed-set"} '
                f'{len(self.classes)} classes>')
