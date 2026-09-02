"""LvlmBehavior + lvlm_client — the OG LVLM-guided behaviour and its HTTP path.

The VLM is exercised against a real `http.server` in a thread, so the whole
client path (URL resolution, data-URL image part, response parsing, the worker
thread) is covered without a model.
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import pytest

from helpers import ctx, rays, scores_for
from raven_nav import lvlm_client
from raven_nav.behaviors import lvlm_behavior as lb
from raven_nav.behaviors.lvlm_behavior import (
    LvlmBehavior, build_prompt, clean_guiding_objects,
)


def test_og_constants_unchanged():
    assert lb.REQUEST_INTERVAL_S == 30.0
    assert lb.RAY_THRESHOLD == 0.9
    assert lb.MAGNITUDE_M == 5.0


# ── the OG prompt ───────────────────────────────────────────────────────────
def test_prompt_is_the_og_text():
    p = build_prompt(['person'])
    assert p == ('Find person. List three unique objects or areas that are '
                 'most helpful as clues or context to locate the person. '
                 'Write ONLY the object or area names as a plain '
                 'comma-separated list.')


def test_prompt_joins_multiple_targets():
    assert build_prompt(['person', 'car']).startswith('Find person, car.')


# ── OG set_guiding_objects cleaning ─────────────────────────────────────────
@pytest.mark.parametrize('raw,want', [
    ('a car, the roof, an ambulance', ['car', 'roof', 'ambulance']),
    ('Car, CAR, car', ['car']),
    ('debris., rubble,', ['debris', 'rubble']),
    ('  spaced  ,  out  ', ['spaced', 'out']),
    ('', []),
    (',,,', []),
    ('theatre', ['theatre']),          # 'the' prefix needs the space
    ('anvil', ['anvil']),
])
def test_cleaning(raw, want):
    assert clean_guiding_objects(raw) == want


def test_set_guiding_objects_flags_a_change():
    b = LvlmBehavior()
    b.set_guiding_objects('car, roof')
    assert b.guiding_changed is True
    b.set_guiding_objects('car, roof')
    assert b.guiding_changed is False


def test_stale_queries_lists_only_dropped_guiding_labels():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    stale = b.stale_queries(['person', 'sky', 'car', 'roof'],
                            target_objects=['person'],
                            background_objects=['sky'])
    assert stale == ['car']           # never a target or background label


# ── throttle + condition ────────────────────────────────────────────────────
def _ray_ctx(score, col, now=0.0, labels=('person', 'sky', 'roof'), **kw):
    q = len(labels)
    kw.setdefault('cur_pose', np.array([0.0, 0.0, 6.0]))
    return ctx(query_labels=list(labels), target_objects=['person'], now=now,
               **rays([(10.0, 0.0, 6.0)], [(1.0, 0.0, 0.0)],
                      scores_for(1, q, col, score)), **kw)


def test_trigger_every_tick_but_request_is_throttled():
    b = LvlmBehavior(request_interval_s=30.0)
    b.condition_check(_ray_ctx(0.99, 2, now=0.0))
    assert b.want_trigger is True and b.want_request is True
    b.condition_check(_ray_ctx(0.99, 2, now=10.0))
    assert b.want_trigger is True and b.want_request is False
    b.condition_check(_ray_ctx(0.99, 2, now=31.0))
    assert b.want_request is True


def test_no_targets_means_no_trigger():
    b = LvlmBehavior()
    c = _ray_ctx(0.99, 2)
    c.target_objects = []
    assert b.condition_check(c) is False
    assert b.want_trigger is False


def test_disabled_behaviour_never_fires():
    b = LvlmBehavior(enabled=False)
    b.set_guiding_objects('roof')
    assert b.condition_check(_ray_ctx(0.99, 2)) is False
    assert b.want_trigger is False


def test_condition_false_without_guiding_objects():
    b = LvlmBehavior()
    assert b.condition_check(_ray_ctx(0.99, 2)) is False


def test_condition_true_on_a_hot_guiding_column():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')                 # column 2
    assert b.condition_check(_ray_ctx(0.99, 2)) is True


def test_condition_false_below_the_guiding_threshold():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    assert b.condition_check(_ray_ctx(0.85, 2)) is False


def test_guiding_column_not_yet_registered_by_rayfronts():
    b = LvlmBehavior()
    b.set_guiding_objects('ambulance')            # no such column
    assert b.condition_check(_ray_ctx(0.99, 2)) is False


def test_column_mapping_tolerates_topic_name_sanitisation():
    """rayfronts turns 'fire truck' into the topic q3_fire_truck; the node maps
    it back with underscores, so the guiding label must still match."""
    b = LvlmBehavior()
    b.set_guiding_objects('fire truck')
    c = _ray_ctx(0.99, 2, labels=('person', 'sky', 'fire_truck'))
    assert b.condition_check(c) is True


def test_a_hot_target_column_does_not_fire_the_lvlm():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    assert b.condition_check(_ray_ctx(0.99, 0)) is False


# ── execution ───────────────────────────────────────────────────────────────
def test_hop_is_five_metres_along_the_mean_bearing():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    c = _ray_ctx(0.99, 2)
    assert b.condition_check(c) is True
    out = b.execute(c)
    assert len(out.path) == 2
    assert out.path[0][0] == pytest.approx(10.0)      # mean origin
    assert out.path[1][0] == pytest.approx(15.0)      # + 5 m
    # OG:124 — never locks, clears both waypoints.
    assert out.target_waypoint is None and out.target_waypoint2 is None


def test_mean_of_several_rays():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    c = ctx(query_labels=['person', 'sky', 'roof'], target_objects=['person'],
            cur_pose=np.array([0.0, 0.0, 6.0]),
            **rays([(10.0, 0.0, 6.0), (20.0, 0.0, 6.0)], [(1.0, 0.0, 0.0)] * 2,
                   scores_for(2, 3, 2, 0.99)))
    assert b.condition_check(c) is True
    assert b.execute(c).path[0][0] == pytest.approx(15.0)


def test_deviation_1_z_clamped():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    c = _ray_ctx(0.99, 2, min_altitude=3.0, max_altitude=15.0,
                 cur_pose=np.array([0.0, 0.0, 6.0]))
    c.ray_origins = np.array([[10.0, 0.0, 60.0]])
    assert b.condition_check(c) is True
    for wp in b.execute(c).path:
        assert 3.0 <= wp[2] <= 15.0


def test_deviation_2_hop_outside_the_polygon_is_dropped():
    b = LvlmBehavior()
    b.set_guiding_objects('roof')
    poly = np.array([[-5.0, -5.0], [5.0, -5.0], [5.0, 5.0], [-5.0, 5.0]])
    c = _ray_ctx(0.99, 2, search_area_xy=poly)
    assert b.condition_check(c) is True
    out = b.execute(c)
    assert out.path == []
    assert 'search_area' in out.note


# ── the HTTP client ─────────────────────────────────────────────────────────
class _Handler(BaseHTTPRequestHandler):
    answer = 'a car, the roof, debris.'
    seen = []

    def log_message(self, *a):        # keep the test output clean
        pass

    def _json(self, payload, code=200):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.endswith('/models'):
            self._json({'data': [{'id': 'test-vlm'}, {'id': 'other'}]})
        else:
            self._json({'error': 'nope'}, 404)

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        req = json.loads(self.rfile.read(n))
        _Handler.seen.append(req)
        self._json({'choices': [{'message': {'content': _Handler.answer}}]})


@pytest.fixture()
def vlm_server():
    _Handler.seen = []
    srv = HTTPServer(('127.0.0.1', 0), _Handler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    yield f'http://127.0.0.1:{srv.server_port}/v1'
    srv.shutdown()
    srv.server_close()


def test_url_resolution_precedence(monkeypatch):
    monkeypatch.delenv('VLM_URL', raising=False)
    monkeypatch.delenv('OPENAI_BASE_URL', raising=False)
    assert lvlm_client.resolve_base_url() == 'http://offboard-compute:8000/v1'
    monkeypatch.setenv('OPENAI_BASE_URL', 'http://b:1/v1')
    assert lvlm_client.resolve_base_url() == 'http://b:1/v1'
    monkeypatch.setenv('VLM_URL', 'http://a:1/v1')
    assert lvlm_client.resolve_base_url() == 'http://a:1/v1'
    assert lvlm_client.resolve_base_url('http://x:9/v1') == 'http://x:9/v1'


def test_model_resolution(monkeypatch):
    monkeypatch.delenv('CONAVGPT2_VLM_MODEL', raising=False)
    assert lvlm_client.resolve_model() == ''
    monkeypatch.setenv('CONAVGPT2_VLM_MODEL', 'm1')
    assert lvlm_client.resolve_model() == 'm1'
    assert lvlm_client.resolve_model('m2') == 'm2'


def test_preflight_picks_the_first_served_model(vlm_server):
    c = lvlm_client.VlmClient(base_url=vlm_server, model='')
    assert c.preflight() == ['test-vlm', 'other']
    assert c.model == 'test-vlm'


def test_preflight_raises_when_unreachable():
    c = lvlm_client.VlmClient(base_url='http://127.0.0.1:1/v1', model='m')
    with pytest.raises(Exception):
        c.preflight(timeout=0.5)


def test_chat_roundtrip_sends_a_data_url_image(vlm_server):
    c = lvlm_client.VlmClient(base_url=vlm_server, model='test-vlm')
    res = c.chat('hello?', b'\xff\xd8\xff-not-really-a-jpeg')
    assert res.ok and res.answer == 'a car, the roof, debris.'
    sent = _Handler.seen[-1]
    assert sent['model'] == 'test-vlm'
    parts = sent['messages'][0]['content']
    assert parts[0]['image_url']['url'].startswith('data:image/jpeg;base64,')
    assert parts[1]['text'] == 'hello?'


def test_chat_reports_a_dead_endpoint_instead_of_raising():
    c = lvlm_client.VlmClient(base_url='http://127.0.0.1:1/v1', model='m',
                              timeout_s=0.5)
    res = c.chat('hi', None)
    assert res.ok is False and res.error


def test_async_client_never_blocks_and_yields_once(vlm_server):
    c = lvlm_client.AsyncVlmClient(
        lvlm_client.VlmClient(base_url=vlm_server, model='test-vlm'))
    assert c.submit('hi', None) is True
    assert c.submit('again', None) is False, 'one request in flight at a time'
    res = None
    for _ in range(500):
        res = c.poll()
        if res is not None:
            break
        time.sleep(0.01)
    assert res is not None and res.ok
    assert c.poll() is None            # consumed exactly once


def test_answer_flows_into_guiding_objects(vlm_server):
    c = lvlm_client.VlmClient(base_url=vlm_server, model='test-vlm')
    b = LvlmBehavior()
    b.set_guiding_objects(c.chat(build_prompt(['person']), None).answer)
    assert b.guiding_objects == ['car', 'roof', 'debris']


def test_encode_jpeg_roundtrips():
    pytest.importorskip('cv2', reason='JPEG encoding needs OpenCV')
    img = np.zeros((8, 8, 3), dtype=np.uint8)
    data = lvlm_client.encode_jpeg(img)
    assert data[:2] == b'\xff\xd8'
