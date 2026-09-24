#!/usr/bin/env python3
"""The shared detector's HTTP contract, driven end to end with a FAKE model.

A real detector server needs a GPU, two checkpoints and ~30 s of load time, so
none of that is tested here. What IS tested is everything BETWEEN the planner
and the checkpoint, which is where a shared service can go wrong quietly:

  * the frame arrives BIT-EXACT (PNG, channel order untouched) — the array YOLO
    sees must be the array the planner had, or the boxes are drawn on a
    colour-swapped image and nobody notices;
  * the masks come back bit-exact, in both encodings, at FULL FRAME
    RESOLUTION — `create_object_pcd` indexes the depth image with them, so a
    mask off by a pixel moves the survivor's point cloud;
  * `mask=None` when nothing is detected, exactly as the vendored detector
    returns it;
  * the object handed back is an `sv.Detections` supporting the four accesses
    the vendored mapping loop makes and the 6-tuple iteration the vendored
    overlay makes;
  * conf and the open-vocabulary vocabulary reach the model, and the ids in a
    response index the vocabulary that produced them;
  * a server that is not there RAISES with something an operator can act on,
    instead of returning "no detections" — which would read as "no survivors".

The server runs IN THIS PROCESS on a real socket (uvicorn in a thread) and the
REAL `detector_client` talks to it with `requests`, so the transport is the
transport. Only the two checkpoints are stubbed.

Run:  /opt/lvlm-venv/bin/python -m pytest tests/test_detector_api.py -v
      (host python usually lacks fastapi/torch/supervision -> skipped cleanly)
"""

import os
import socket
import sys
import threading
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
if PKG not in sys.path:
    sys.path.insert(0, PKG)

# The server module needs fastapi+uvicorn (the /opt/lvlm-venv side) and torch;
# the client needs supervision and requests. Where any is missing the whole
# file skips rather than failing — the same deps split _venv_exec exists for.
fastapi = pytest.importorskip('fastapi', reason='serving stack (/opt/lvlm-venv)')
uvicorn = pytest.importorskip('uvicorn', reason='serving stack (/opt/lvlm-venv)')
pytest.importorskip('torch', reason='detector_server imports torch')
pytest.importorskip('supervision', reason='detector_client returns sv.Detections')
pytest.importorskip('requests')

from search_baselines import detector_client as dc            # noqa: E402
from search_baselines import detector_server as ds            # noqa: E402


# ── the fake checkpoints ─────────────────────────────────────────────────────

CLASSES = ['person', 'car']

# Two detections with deliberately awkward masks: a solid block (few runs) and
# a checkerboard (a run per pixel). The checkerboard is the one that catches a
# lazy RLE or a resized PNG.
def _masks(h, w):
    a = np.zeros((h, w), dtype=bool)
    a[3:11, 5:20] = True
    b = np.zeros((h, w), dtype=bool)
    yy, xx = np.mgrid[0:h, 0:w]
    b[(yy + xx) % 2 == 0] = True
    b[:, w // 2:] = False
    return np.stack([a, b])


class FakeYOLO:
    """Only what `Engine` touches: `names` and an open-vocab `set_classes`."""

    def __init__(self, classes):
        self.names = {i: c for i, c in enumerate(classes)}
        self.set_calls = []

    def set_classes(self, classes):
        self.set_calls.append(list(classes))
        self.names = {i: c for i, c in enumerate(classes)}


class FakeEngine(ds.Engine):
    """Real vocabulary handling, real counters, fabricated model output.

    `detect` mirrors what the true one returns (the dict, the fields, the
    dtypes) without torch or a GPU: two boxes, confidences 0.95 and 0.42, one
    mask each at full frame resolution.
    """

    def __init__(self, classes=CLASSES, open_vocab=True):
        super().__init__('fake-yolo.pt', 'fake-sam.pt', 'cpu', classes, 0.1)
        self._want_open_vocab = open_vocab
        self.seen_images = []
        self.seen_conf = []

    def load(self):
        self.yolo = FakeYOLO(self.requested_classes)
        self.sam = object()
        self.open_vocab = self._want_open_vocab
        if self.open_vocab:
            self.classes = list(self.requested_classes)
        self._refresh_names()
        self.load_s = 0.0

    def detect(self, image, classes=None, conf=None, max_det=None):
        with self._lock:
            t0 = time.time()
            switched = self._ensure_vocab(classes)
            c = self.default_conf if conf is None else float(conf)
            self.seen_images.append(np.array(image, copy=True))
            self.seen_conf.append(c)
            h, w = image.shape[:2]
            xyxy = np.array([[5., 3., 20., 11.], [30., 4., 44., 18.]],
                            dtype=np.float32)
            confidence = np.array([0.95, 0.42], dtype=np.float32)
            class_id = np.array([0, 1], dtype=int)
            mask = _masks(h, w)
            keep = confidence >= c
            xyxy, confidence, class_id = xyxy[keep], confidence[keep], class_id[keep]
            mask = mask[keep]
            if max_det is not None and len(confidence) > max_det:
                order = np.argsort(-confidence)[:max_det]
                xyxy, confidence = xyxy[order], confidence[order]
                class_id, mask = class_id[order], mask[order]
            if len(confidence) == 0:
                mask = None                     # upstream's own empty shape
            dt = time.time() - t0
            self.n_calls += 1
            self.n_dets += int(len(confidence))
            self.total_s += dt
            return {'xyxy': xyxy, 'confidence': confidence, 'class_id': class_id,
                    'mask': mask, 'yolo_s': 0.001, 'sam_s': 0.002,
                    'server_s': dt, 'switched': switched}


class _Args:
    metrics_jsonl = ''


def _free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


class ServerHandle:
    def __init__(self, engine, url, server, thread):
        self.engine = engine
        self.url = url
        self._server = server
        self._thread = thread

    def stop(self):
        self._server.should_exit = True
        self._thread.join(timeout=10)


def _serve(engine):
    engine.load()
    app = ds.build_app(engine, _Args())
    port = _free_port()
    config = uvicorn.Config(app, host='127.0.0.1', port=port, log_level='warning')
    server = uvicorn.Server(config)
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    url = f'http://127.0.0.1:{port}'
    deadline = time.time() + 30
    while time.time() < deadline:
        try:
            dc.health(url, timeout=1.0)
            break
        except Exception:
            time.sleep(0.1)
    else:                                                    # pragma: no cover
        raise RuntimeError('test server never became healthy')
    return ServerHandle(engine, url, server, thread)


@pytest.fixture(scope='module')
def server():
    h = _serve(FakeEngine())
    yield h
    h.stop()


@pytest.fixture(scope='module')
def closed_set_server():
    h = _serve(FakeEngine(classes=['person', 'car'], open_vocab=False))
    yield h
    h.stop()


@pytest.fixture
def frame():
    """A frame with structure in every channel, so a channel swap is visible."""
    rng = np.random.default_rng(0)
    img = rng.integers(0, 256, size=(24, 48, 3), dtype=np.uint8)
    img[:, :, 0] = 10          # a constant plane per channel makes a swap
    img[:, :, 2] = 240         # unmistakable in the assertion below
    return img


# ── the contract ─────────────────────────────────────────────────────────────

def test_health(server):
    h = dc.health(server.url)
    assert h['status'] == 'ok'
    assert h['open_vocab'] is True
    assert h['classes'] == CLASSES
    assert h['names'] == {'0': 'person', '1': 'car'}
    assert h['default_conf'] == pytest.approx(0.1)


def test_metrics_counts_calls(server, frame):
    before = dc.metrics(server.url)['calls']
    dc.detect(frame, classes=CLASSES, base_url=server.url)
    after = dc.metrics(server.url)
    assert after['calls'] == before + 1
    assert after['mean_ms'] is not None
    assert after['detections'] >= 2


def test_frame_arrives_bit_exact(server, frame):
    """PNG in, the same array out — the encode/decode pair must not touch it."""
    n = len(server.engine.seen_images)
    dc.detect(frame, classes=CLASSES, base_url=server.url)
    received = server.engine.seen_images[n]
    assert received.shape == frame.shape
    assert received.dtype == frame.dtype
    assert np.array_equal(received, frame), 'frame was altered in transit'
    # And specifically not channel-swapped, which array_equal would also catch
    # but this names.
    assert received[0, 0, 0] == 10 and received[0, 0, 2] == 240


def test_detections_shape_and_fields(server, frame):
    det = dc.detect(frame, classes=CLASSES, base_url=server.url)
    # The four accesses the vendored mapping loop makes.
    assert len(det.xyxy) == 2
    assert det.xyxy.shape == (2, 4)
    assert det.confidence[0] == pytest.approx(0.95, abs=1e-6)
    assert int(det.class_id[0]) == 0
    assert det.mask.shape == (2, frame.shape[0], frame.shape[1])
    # The 6-tuple iteration the vendored overlay makes (visualization.py).
    rows = [row for row in det]
    assert len(rows) == 2 and len(rows[0]) == 6


@pytest.mark.parametrize('encoding', ['png', 'rle'])
def test_masks_round_trip_bit_exact(server, frame, encoding):
    truth = _masks(frame.shape[0], frame.shape[1])
    det = dc.detect(frame, classes=CLASSES, base_url=server.url,
                    mask_encoding=encoding)
    assert det.mask.dtype == np.bool_
    assert np.array_equal(det.mask[0], truth[0]), f'{encoding}: solid mask differs'
    assert np.array_equal(det.mask[1], truth[1]), f'{encoding}: checkerboard differs'


def test_mask_encoding_none(server, frame):
    det = dc.detect(frame, classes=CLASSES, base_url=server.url,
                    mask_encoding='none')
    assert det.mask is None
    assert len(det.xyxy) == 2


def test_conf_threshold_reaches_the_model(server, frame):
    det = dc.detect(frame, classes=CLASSES, conf=0.9, base_url=server.url)
    assert len(det.xyxy) == 1
    assert det.confidence[0] == pytest.approx(0.95, abs=1e-6)
    assert server.engine.seen_conf[-1] == pytest.approx(0.9)


def test_no_detections_gives_mask_none(server, frame):
    """`mask is None` on an empty frame — what upstream returns, and what the
    mapping loop's `len(detections.xyxy)` guard is written against."""
    det = dc.detect(frame, classes=CLASSES, conf=0.99, base_url=server.url)
    assert len(det.xyxy) == 0
    assert det.mask is None
    assert det.xyxy.shape == (0, 4)


def test_max_det_caps_and_keeps_the_best(server, frame):
    det = dc.detect(frame, classes=CLASSES, base_url=server.url, max_det=1)
    assert len(det.xyxy) == 1
    assert det.confidence[0] == pytest.approx(0.95, abs=1e-6)


def test_open_vocab_switch_and_class_names(server, frame):
    """The vocabulary is per REQUEST; ids index the list that produced them."""
    before = dc.metrics(server.url)['vocab_switches']
    dc.detect(frame, classes=['smoke', 'fire'], base_url=server.url)
    assert server.engine.yolo.set_calls[-1] == ['smoke', 'fire']
    mid = dc.metrics(server.url)
    assert mid['vocab_switches'] == before + 1
    # /health reports the vocabulary now loaded, and class_name follows it.
    assert dc.health(server.url)['classes'] == ['smoke', 'fire']
    # The SAME list again must not re-run the text encoder (~1 s a call).
    dc.detect(frame, classes=['smoke', 'fire'], base_url=server.url)
    assert dc.metrics(server.url)['vocab_switches'] == before + 1
    dc.detect(frame, classes=CLASSES, base_url=server.url)     # restore


def test_class_names_in_the_raw_response(server, frame):
    import requests
    b64, _ = dc.encode_image(frame)
    r = requests.post(f'{server.url}/detect',
                      json={'image': b64, 'classes': CLASSES, 'conf': 0.1},
                      timeout=10)
    body = r.json()
    assert [d['class_name'] for d in body['detections']] == ['person', 'car']
    assert body['mask_shape'] == [frame.shape[0], frame.shape[1]]
    assert body['mask_dtype'] == 'bool'
    assert body['image_shape'] == list(frame.shape)


def test_server_down_raises_actionable(frame):
    port = _free_port()                       # nothing is listening there
    with pytest.raises(dc.DetectorError) as e:
        dc.detect(frame, classes=CLASSES, base_url=f'http://127.0.0.1:{port}',
                  timeout=1.0, retries=0)
    msg = str(e.value)
    assert 'detector' in msg.lower() and '/health' in msg
    assert dc.LAST_DETECT['ok'] is False


def test_health_down_raises(frame):
    port = _free_port()
    with pytest.raises(dc.DetectorError):
        dc.health(f'http://127.0.0.1:{port}', timeout=1.0)


def test_bad_request_raises_not_retried(server, frame):
    with pytest.raises(dc.DetectorError) as e:
        dc.detect(frame, classes=CLASSES, base_url=server.url,
                  mask_encoding='bogus')
    assert 'HTTP 400' in str(e.value)


def test_non_rgb_image_rejected(server):
    with pytest.raises(ValueError):
        dc.encode_image(np.zeros((8, 8), dtype=np.uint8))


def test_jpeg_transport_still_decodes(server, frame):
    """JPEG is offered for bandwidth; it is LOSSY, so only shape is promised."""
    det = dc.detect(frame, classes=CLASSES, base_url=server.url,
                    image_format='jpeg')
    assert len(det.xyxy) == 2
    assert not np.array_equal(server.engine.seen_images[-1], frame)


# ── the drop-in the planner actually constructs ──────────────────────────────

def test_remote_detector_open_vocab(server, frame):
    rd = dc.RemoteDetector(classes=CLASSES, base_url=server.url)
    assert rd.open_vocab is True
    model = rd.yolo_model_w_classes
    # AirStackAgent's probe: a REAL set_classes means open-vocabulary.
    sc = getattr(model, 'set_classes', None)
    assert sc is not None and not getattr(sc, '_noop', False)
    model.set_classes(['person', 'car', 'tree'])
    det = rd.detect(frame)
    assert server.engine.yolo.set_calls[-1] == ['person', 'car', 'tree']
    assert len(det.xyxy) == 2
    assert dc.LAST_DETECT['ok'] is True and dc.LAST_DETECT['n'] == 2
    dc.detect(frame, classes=CLASSES, base_url=server.url)     # restore


def test_remote_detector_closed_set_has_no_set_classes(closed_set_server, frame):
    """A COCO checkpoint must NOT look open-vocabulary to `AirStackAgent`.

    The probe is `getattr(model, 'set_classes', None)`; answering it for a
    closed-set model is what sent yolov8x down the open-vocab path and crashed
    the vendored overlay with an IndexError (see test_closed_set_detector.py).
    """
    rd = dc.RemoteDetector(classes=['person'], base_url=closed_set_server.url)
    assert rd.open_vocab is False
    assert getattr(rd.yolo_model_w_classes, 'set_classes', None) is None
    assert rd.yolo_model_w_classes.names == {0: 'person', 1: 'car'}
    det = rd.detect(frame)
    assert len(det.xyxy) == 2
    # No vocabulary is sent for a closed-set model, so nothing can switch.
    assert closed_set_server.engine.vocab_switches == 0


def test_remote_detector_loads_nothing_locally(server):
    rd = dc.RemoteDetector(classes=CLASSES, base_url=server.url)
    assert rd.sam_predictor is None
    assert not hasattr(rd, 'model')
    assert 'ultralytics.models' not in repr(type(rd.yolo_model_w_classes))


def test_last_detection_telemetry_shape(server, frame):
    """`AirStackAgent._detect` reads exactly these fields off the result."""
    det = dc.detect(frame, classes=CLASSES, base_url=server.url)
    conf = getattr(det, 'confidence', None)
    cid = getattr(det, 'class_id', None)
    n = 0 if conf is None else len(conf)
    order = sorted(range(n), key=lambda i: -float(conf[i]))[:5]
    top = [(CLASSES[int(cid[i])], round(float(conf[i]), 3)) for i in order]
    assert top == [('person', 0.95), ('car', 0.42)]


def test_mask_indexes_a_depth_image(server, frame):
    """The one thing the mask is FOR: selecting pixels of the depth image."""
    det = dc.detect(frame, classes=CLASSES, base_url=server.url)
    depth = np.full(frame.shape[:2], 5.0, dtype=np.float32)
    m = np.logical_and(det.mask[0], depth > 0)
    assert m.shape == depth.shape
    assert depth[m].size == int(det.mask[0].sum()) > 0


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
