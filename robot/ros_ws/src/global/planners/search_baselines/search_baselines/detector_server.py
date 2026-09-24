"""YOLO(+MobileSAM) as a shared detection service — one model load for a fleet.

The same argument that made the BLIP-2 scorer a service (`itm_server`) applies
here, and harder. Every `search_planner` process used to construct the vendored
`Object_Detection_and_Segmentation` in-process, which loads a YOLO checkpoint
AND MobileSAM onto the GPU. Three robots = three containers = three copies of
both, on one card, for a model that is **stateless**: detection looks at one
frame and keeps nothing, so requests from different robots interleave with no
cross-talk. Exactly the property that makes sharing safe for the ITM scorer.

    per planner process, in-process   yolov8l-world + mobile_sam  ~1.6 GiB
    three robots                       three copies               ~4.8 GiB
    three robots, this server          one copy                   ~1.6 GiB

Stateless with ONE asterisk: an open-vocabulary checkpoint (yolov8*-world.pt)
carries a prompt vocabulary set by `set_classes()`, and the class ids in a
response index THAT list. So the vocabulary is part of the request: a client
sends `classes` with every call, the server switches only when the list differs
from the one currently loaded (the CLIP text-embedding pass is the expensive
part, ~1 s, and re-running it per frame would be worse than loading a second
model). `/metrics` reports `vocab_switches` — a number that climbs during a run
means two clients disagree about the vocabulary and are thrashing the text
tower. A closed-set COCO checkpoint has no `set_classes`; `classes` is then
IGNORED and the ids are the checkpoint's own, which is what `AirStackAgent`
already assumes for that case.

What is served is a faithful re-implementation of the vendored
`utils/detection_segmentation.py::Object_Detection_and_Segmentation.detect` —
same `predict(...)` call, same SAM box prompt, same four output fields — and
NOT the vendored class itself, for two reasons: its constructor hardcodes
`set_classes(classes)` (which raises on a COCO checkpoint) and its `detect()`
hardcodes `conf=0.1`. The response carries everything an
`sv.Detections` needs, so `detector_client` reconstructs the same object the
vendored mapping code consumes: `xyxy`, `confidence`, `class_id`, `mask`.

Masks travel as a per-detection BITMASK — base64 PNG by default, run-length
otherwise — with the original numpy dtype recorded, so the client rebuilds an
array that is bit-identical to what SAM produced (ultralytics binarises SAM
output to bool: `pred_masks > mask_threshold`). Masks are FULL FRAME RESOLUTION
and must stay that way: `create_object_pcd` indexes the depth image with them.

Run it ONCE per box, in robot_1's container, before any planner:

    ros2 run search_baselines detector_server            # re-execs into the venv
    /opt/lvlm-venv/bin/python -m search_baselines.detector_server --port 8200

Weights come from the same three parameters the planner uses today, as flags or
as the CONAVGPT2_* env vars the planner exports (`--yolo-weights` /
`CONAVGPT2_YOLO_WORLD_WEIGHTS`, `--sam-weights` / `CONAVGPT2_SAM_WEIGHTS`,
`--weights-dir` / `CONAVGPT2_ULTRALYTICS_WEIGHTS_DIR`).
"""

import argparse
import base64
import io
import json
import logging
import os
import threading
import time
from typing import List, Optional

import numpy as np
import torch
import uvicorn
from fastapi import FastAPI, HTTPException
from PIL import Image
from pydantic import BaseModel

DEFAULT_YOLO = os.environ.get('CONAVGPT2_YOLO_WORLD_WEIGHTS', 'yolov8l-world.pt')
DEFAULT_SAM = os.environ.get('CONAVGPT2_SAM_WEIGHTS', 'mobile_sam.pt')
DEFAULT_WEIGHTS_DIR = os.environ.get('CONAVGPT2_ULTRALYTICS_WEIGHTS_DIR', '')
# The vendored detector's own threshold. Kept as the default so a request that
# does not say otherwise reproduces in-process behaviour exactly. The planner's
# `sem_threshold` (0.65) is a SEPARATE, later gate applied in mapping().
DEFAULT_CONF = 0.1
DEFAULT_PORT = 8200

log = logging.getLogger('detector_server')


# ── wire format ──────────────────────────────────────────────────────────────

class DetectRequest(BaseModel):
    image: str                                  # base64 PNG/JPEG, no data: prefix
    classes: Optional[List[str]] = None         # open-vocab prompt; ignored closed-set
    conf: Optional[float] = None                # None -> the server default (0.1)
    mask_encoding: str = 'png'                  # 'png' | 'rle' | 'none'
    max_det: Optional[int] = None               # cap on returned detections


def encode_mask(mask, encoding='png'):
    """One boolean mask -> a compact, LOSSLESS, JSON-safe blob.

    'png'  base64 PNG, mode L, 0/255. Small (a mask is mostly one value) and
           decodable by PIL, which both ends already depend on.
    'rle'  row-major run lengths starting with the count of ZEROS, COCO-style
           but uncompressed. Human-readable in a log and cheap to diff.

    Bit-exactness is the requirement, not compactness: `create_object_pcd`
    indexes depth and colour with this mask, so a resized or thresholded mask
    silently moves the object's point cloud.
    """
    m = np.asarray(mask)
    flat = (m > 0).astype(np.uint8)
    if encoding == 'rle':
        f = flat.reshape(-1)
        if f.size == 0:
            counts = []
        else:
            # Boundaries between runs, then the run lengths themselves.
            idx = np.flatnonzero(np.diff(f)) + 1
            bounds = np.concatenate(([0], idx, [f.size]))
            lens = np.diff(bounds).astype(int).tolist()
            # COCO convention: the first run is a run of ZEROS, so a mask that
            # starts with a 1 gets a leading 0-length run.
            counts = ([0] + lens) if f[0] == 1 else lens
        return {'encoding': 'rle', 'shape': list(m.shape), 'counts': counts}
    buf = io.BytesIO()
    Image.fromarray((flat * 255).astype(np.uint8), mode='L').save(buf, format='PNG')
    return {'encoding': 'png', 'shape': list(m.shape),
            'data': base64.b64encode(buf.getvalue()).decode('utf-8')}


def decode_image(b64):
    """base64 image bytes -> HxWx3 uint8, CHANNEL ORDER UNTOUCHED.

    The array the planner hands the detector is BGR (upstream's
    `transform_rgb_bgr`), which is what ultralytics expects from a numpy input.
    Both ends therefore treat the payload as an opaque 3-channel array and use
    PIL, never `cv2.imdecode` — cv2 would helpfully swap the channels on decode
    and the model would see a colour-inverted frame.
    """
    if b64.lstrip().startswith('data:') and ',' in b64[:64]:
        b64 = b64.split(',', 1)[1]
    img = Image.open(io.BytesIO(base64.b64decode(b64)))
    if img.mode != 'RGB':
        img = img.convert('RGB')
    return np.asarray(img, dtype=np.uint8)


# ── engine ───────────────────────────────────────────────────────────────────

class Engine:
    def __init__(self, yolo_weights, sam_weights, device, classes, conf,
                 weights_dir=''):
        self.yolo_weights = yolo_weights
        self.sam_weights = sam_weights
        self.device = device
        self.weights_dir = weights_dir
        self.default_conf = float(conf)
        self.requested_classes = list(classes or [])
        self.yolo = None
        self.sam = None
        self.open_vocab = False
        self.names = {}
        self.classes = []
        self.load_s = 0.0
        # One GPU, one forward at a time — the same discipline as itm_server,
        # held longer here (YOLO + SAM, tens to hundreds of ms).
        self._lock = threading.Lock()
        self.n_calls = 0
        self.n_dets = 0
        self.total_s = 0.0
        self.yolo_s = 0.0
        self.sam_s = 0.0
        self.vocab_switches = 0
        self.n_errors = 0

    # ultralytics' WEIGHTS_DIR defaults to the RELATIVE string 'weights', which
    # resolves against the CWD; YOLO-World's set_classes() then downloads CLIP
    # ViT-B/32 (338 MB) into wherever the server happened to be started. Pin it,
    # both the setting and the already-imported module constant, exactly as
    # planner_node does.
    def _pin_weights_dir(self):
        if not self.weights_dir:
            return
        try:
            from pathlib import Path
            import ultralytics.utils as uu
            os.makedirs(self.weights_dir, exist_ok=True)
            uu.SETTINGS.update({'weights_dir': self.weights_dir})
            uu.WEIGHTS_DIR = Path(self.weights_dir)
        except Exception as exc:
            log.warning('could not pin ultralytics weights_dir to %s: %s',
                        self.weights_dir, exc)

    def load(self):
        from ultralytics import SAM, YOLO
        t0 = time.time()
        self._pin_weights_dir()
        self.sam = SAM(self.sam_weights).to(self.device)
        self.yolo = YOLO(self.yolo_weights).to(self.device)
        # OPEN-VOCAB vs CLOSED-SET, decided the way AirStackAgent decides it:
        # a COCO checkpoint has no set_classes and its ids are its own.
        # (No no-op shim is installed here — this process never runs the
        # vendored constructor that needed one.)
        sc = getattr(self.yolo, 'set_classes', None)
        self.open_vocab = sc is not None and not getattr(sc, '_noop', False)
        if self.open_vocab and self.requested_classes:
            self.yolo.set_classes(list(self.requested_classes))
            self.classes = list(self.requested_classes)
        self._refresh_names()
        self.load_s = time.time() - t0
        log.info('loaded %s + %s in %.1f s | %s | %d classes | %.2f GiB',
                 self.yolo_weights, self.sam_weights, self.load_s,
                 'open-vocab' if self.open_vocab else 'closed-set',
                 len(self.names), self._gpu_gib() or 0.0)

    def _refresh_names(self):
        names = dict(getattr(self.yolo, 'names', {}) or {})
        self.names = {int(k): str(v) for k, v in names.items()}
        if not self.open_vocab or not self.classes:
            self.classes = [self.names[k] for k in sorted(self.names)]

    def _gpu_gib(self):
        if not str(self.device).startswith('cuda') or not torch.cuda.is_available():
            return None
        return round(torch.cuda.memory_allocated() / 2 ** 30, 3)

    def _ensure_vocab(self, classes):
        """Point the model at `classes`, if it is the kind that can be pointed.

        Called under the lock. A no-op when the list already matches — the CLIP
        text pass behind set_classes costs ~1 s and running it per frame would
        cost more than the detection.
        """
        if not self.open_vocab or not classes:
            return False
        want = [str(c) for c in classes]
        if want == list(self.classes):
            return False
        self.yolo.set_classes(want)
        self.classes = want
        # The model's `names` ARE the prompt list for an open-vocab checkpoint,
        # so they have to be re-read or every `class_name` in the response is a
        # label from the previous vocabulary.
        self._refresh_names()
        self.vocab_switches += 1
        if self.vocab_switches in (4, 32, 256):
            log.warning(
                'vocabulary switched %d times — clients disagree about `classes` '
                'and are thrashing the text encoder (~1 s each). Give every '
                'planner the same detection_classes.', self.vocab_switches)
        else:
            log.info('vocabulary -> %d classes: %s', len(want), want[:8])
        return True

    def detect(self, image, classes=None, conf=None, max_det=None):
        """Mirror of the vendored `Object_Detection_and_Segmentation.detect`.

        Same two stages: YOLO for boxes, then SAM prompted WITH THOSE BOXES for
        masks — and, like upstream, SAM is skipped entirely when YOLO found
        nothing (an empty box prompt is not a cheap SAM call, it is a
        differently-shaped one).
        """
        with self._lock:
            t0 = time.time()
            switched = self._ensure_vocab(classes)
            c = self.default_conf if conf is None else float(conf)
            ty = time.time()
            with torch.no_grad():
                res = self.yolo.predict(image, conf=c, verbose=False)
            yolo_s = time.time() - ty
            boxes = res[0].boxes
            confidences = boxes.conf.cpu().numpy()
            class_ids = boxes.cls.cpu().numpy().astype(int)
            xyxy_t = boxes.xyxy
            if max_det is not None and len(confidences) > int(max_det):
                keep = np.argsort(-confidences)[:int(max_det)]
                confidences = confidences[keep]
                class_ids = class_ids[keep]
                xyxy_t = xyxy_t[torch.as_tensor(keep.copy(), device=xyxy_t.device)]
            xyxy = xyxy_t.cpu().numpy()

            masks = None
            sam_s = 0.0
            if len(confidences) > 0:
                ts = time.time()
                with torch.no_grad():
                    sam_out = self.sam.predict(image, bboxes=xyxy_t, verbose=False)
                masks = sam_out[0].masks.data.cpu().numpy()
                sam_s = time.time() - ts

            dt = time.time() - t0
            self.n_calls += 1
            self.n_dets += int(len(confidences))
            self.total_s += dt
            self.yolo_s += yolo_s
            self.sam_s += sam_s
            return {'xyxy': xyxy, 'confidence': confidences, 'class_id': class_ids,
                    'mask': masks, 'yolo_s': yolo_s, 'sam_s': sam_s,
                    'server_s': dt, 'switched': switched}


# ── app ──────────────────────────────────────────────────────────────────────

def build_app(engine, args):
    app = FastAPI(title='yolo-sam-detector')

    @app.get('/health')
    def health():
        return {
            'status': 'ok',
            'yolo_weights': engine.yolo_weights,
            'sam_weights': engine.sam_weights,
            'device': engine.device,
            'open_vocab': engine.open_vocab,
            # The vocabulary ids in a response index. Closed-set: the
            # checkpoint's own names, which is what the client resolves
            # `goal_name` against.
            'names': {str(k): v for k, v in engine.names.items()},
            'classes': list(engine.classes),
            'n_classes': len(engine.classes),
            'default_conf': engine.default_conf,
            'load_s': round(engine.load_s, 2),
            'gpu_gib': engine._gpu_gib(),
        }

    @app.get('/metrics')
    def metrics():
        n = engine.n_calls
        return {
            'calls': n,
            'detections': engine.n_dets,
            'errors': engine.n_errors,
            'total_s': round(engine.total_s, 3),
            'mean_ms': round(1000.0 * engine.total_s / n, 2) if n else None,
            'yolo_mean_ms': round(1000.0 * engine.yolo_s / n, 2) if n else None,
            'sam_mean_ms': round(1000.0 * engine.sam_s / n, 2) if n else None,
            'throughput_hz': round(n / engine.total_s, 2) if engine.total_s else None,
            # Climbing during a run = two clients with different class lists.
            'vocab_switches': engine.vocab_switches,
            'gpu_gib': engine._gpu_gib(),
        }

    @app.post('/detect')
    def detect(req: DetectRequest):
        if req.mask_encoding not in ('png', 'rle', 'none'):
            raise HTTPException(400, f'unknown mask_encoding {req.mask_encoding!r}')
        try:
            image = decode_image(req.image)
        except Exception as exc:
            engine.n_errors += 1
            raise HTTPException(400, f'image decode failed: {exc}')
        if image.ndim != 3 or image.shape[2] != 3:
            engine.n_errors += 1
            raise HTTPException(400, f'expected HxWx3, got {image.shape}')
        try:
            out = engine.detect(image, classes=req.classes, conf=req.conf,
                                max_det=req.max_det)
        except Exception as exc:
            engine.n_errors += 1
            log.exception('detect failed')
            raise HTTPException(500, f'detect failed: {exc}')

        masks = out['mask']
        # dtype is carried so the client rebuilds the array SAM produced rather
        # than something merely equivalent — ultralytics binarises to bool.
        mask_dtype = str(masks.dtype) if masks is not None else None
        dets = []
        for i in range(len(out['confidence'])):
            cid = int(out['class_id'][i])
            d = {
                'xyxy': [float(v) for v in out['xyxy'][i]],
                'confidence': float(out['confidence'][i]),
                'class_id': cid,
                'class_name': engine.names.get(cid, str(cid)),
            }
            if masks is not None and req.mask_encoding != 'none':
                d['mask'] = encode_mask(masks[i], req.mask_encoding)
            dets.append(d)

        body = {
            'n': len(dets),
            'detections': dets,
            'image_shape': list(image.shape),
            'mask_shape': (list(masks.shape[1:]) if masks is not None else None),
            'mask_dtype': mask_dtype,
            'mask_encoding': req.mask_encoding,
            'classes': list(engine.classes),
            'open_vocab': engine.open_vocab,
            'conf': engine.default_conf if req.conf is None else float(req.conf),
            'yolo_s': round(out['yolo_s'], 4),
            'sam_s': round(out['sam_s'], 4),
            'server_s': round(out['server_s'], 4),
            'vocab_switched': bool(out['switched']),
        }
        if args.metrics_jsonl:
            try:
                with open(args.metrics_jsonl, 'a') as fh:
                    fh.write(json.dumps({
                        't': time.time(), 'n': body['n'],
                        'server_s': body['server_s'], 'yolo_s': body['yolo_s'],
                        'sam_s': body['sam_s'], 'shape': body['image_shape'],
                        'top': [d['class_name'] for d in dets[:5]],
                    }) + '\n')
            except OSError:
                pass
        return body

    return app


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--yolo-weights', default=DEFAULT_YOLO,
                   help='open-vocab (yolov8*-world.pt) or closed-set (yolov8x.pt); '
                        'env CONAVGPT2_YOLO_WORLD_WEIGHTS')
    p.add_argument('--sam-weights', default=DEFAULT_SAM,
                   help='env CONAVGPT2_SAM_WEIGHTS')
    p.add_argument('--weights-dir', default=DEFAULT_WEIGHTS_DIR,
                   help="ultralytics WEIGHTS_DIR; unset it defaults to the "
                        "RELATIVE 'weights' next to the CWD. "
                        'env CONAVGPT2_ULTRALYTICS_WEIGHTS_DIR')
    p.add_argument('--classes', default='',
                   help='comma-separated initial open-vocab vocabulary. Clients '
                        'send their own with each request; this only avoids the '
                        'first switch.')
    p.add_argument('--conf', type=float, default=DEFAULT_CONF,
                   help='default YOLO confidence when a request omits one '
                        '(the vendored detector hardcodes 0.1)')
    p.add_argument('--host', default='0.0.0.0')
    p.add_argument('--port', type=int, default=DEFAULT_PORT)
    p.add_argument('--device', default='cuda:0')
    p.add_argument('--metrics-jsonl', default='')
    p.add_argument('--log-level', default='info')
    args = p.parse_args(argv)

    logging.basicConfig(level=args.log_level.upper(),
                        format='%(asctime)s %(levelname)s [detector] %(message)s')
    classes = [c.strip() for c in args.classes.split(',') if c.strip()]
    engine = Engine(args.yolo_weights, args.sam_weights, args.device, classes,
                    args.conf, args.weights_dir)
    # Load BEFORE binding the port, so a port that answers means READY and a
    # planner's preflight cannot race the weight load (itm_server does the same).
    engine.load()
    warm = np.zeros((64, 64, 3), dtype=np.uint8)
    try:
        engine.detect(warm)
    except Exception as exc:                      # pragma: no cover
        log.warning('warmup detect failed: %s', exc)
    log.info('serving %s on http://%s:%d', args.yolo_weights, args.host, args.port)
    uvicorn.run(build_app(engine, args), host=args.host, port=args.port,
                log_level=args.log_level)


if __name__ == '__main__':
    main()
