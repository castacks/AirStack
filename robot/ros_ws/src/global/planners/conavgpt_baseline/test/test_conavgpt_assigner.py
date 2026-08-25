"""Pure-function tests for the Co-NavGPT assigner — no ROS, no GPU.

Everything exercised here is importable without rclpy, torch or transformers,
which is the point: the parser and the renderer are the two places a bad model
response or a bad map can take the node down, and neither should need a robot
to test.

Run:  python3 -m pytest test/ -q     (from the package root)
"""

import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from conavgpt_baseline.conavgpt_assigner_node import (  # noqa: E402
    MODEL_JSON_EXAMPLE, RAW_MAX_CHARS, _fit_size, bev_extent, bev_projector,
    build_assignment, build_prompt, build_round_table, fallback_assignment,
    parse_assignments, parse_points, parse_regions, parse_robots, render_bev)


def _req():
    return {
        'round': 7, 'ts': 1699999999.5, 'leader': 1,
        'query': 'person, survivor', 'frame': 'global_enu',
        'search_area': [[-100.0, -100.0], [200.0, -100.0],
                        [200.0, 200.0], [-100.0, 200.0]],
        'robots': [
            {'id': 1, 'x': 12.0, 'y': -40.0, 'z': 15.0,
             'fresh': True, 'current_region': 3},
            {'id': 2, 'x': -88.0, 'y': 15.0, 'z': 15.0, 'fresh': False},
        ],
        'regions': [
            {'id': 0, 'x': 55.0, 'y': 120.0, 'z': 10.0, 'info_gain': 42.0,
             'dist_by_robot': {'1': 55.2, '2': 91.0}},
            {'id': 3, 'x': -20.0, 'y': 60.0, 'z': 10.0, 'info_gain': 17.5,
             'dist_by_robot': {'1': 103.4, '2': 82.1}},
            {'id': 11, 'x': 150.0, 'y': -60.0, 'z': 10.0, 'info_gain': 9.0,
             'dist_by_robot': {'1': 140.0, '2': 250.0}},
        ],
        'found': [{'label': 'person', 'x': 30.0, 'y': 12.0}],
    }


# ── request normalization ───────────────────────────────────────────────────

def test_parse_request_sections():
    req = _req()
    robots = parse_robots(req)
    assert [r['id'] for r in robots] == ['1', '2']
    assert robots[0]['current_region'] == 3
    assert robots[1]['fresh'] is False
    assert robots[1]['current_region'] is None
    regions = parse_regions(req)
    assert [g['id'] for g in regions] == [0, 3, 11]
    assert regions[0]['dist_by_robot']['2'] == pytest.approx(91.0)
    assert len(parse_points(req['search_area'])) == 4
    assert parse_points(req['found'])[0]['label'] == 'person'


def test_parse_request_skips_junk():
    bad = {'robots': ['nope', {'id': 'x', 'x': 1, 'y': 2},
                      {'id': 4, 'x': None, 'y': 2}, {'id': 4, 'x': 1, 'y': 2}],
           'regions': [{'id': 1}, {'id': 2, 'x': 'a', 'y': 0},
                       {'id': 2, 'x': 0, 'y': 0, 'dist_by_robot': {'1': 'far'}}],
           'search_area': [[1], {'x': 1}, [1, 2]]}
    assert [r['id'] for r in parse_robots(bad)] == ['4']
    regions = parse_regions(bad)
    assert [g['id'] for g in regions] == [2]
    assert regions[0]['dist_by_robot'] == {}
    assert len(parse_points(bad['search_area'])) == 1


def test_parse_handles_empty_request():
    assert parse_robots({}) == []
    assert parse_regions({}) == []
    assert parse_points(None) == []


# ── prompt ──────────────────────────────────────────────────────────────────

def test_prompt_carries_everything_the_model_needs():
    p = build_prompt(_req(), with_image=True)
    assert p.startswith('<image>\n')
    assert 'person, survivor' in p
    assert 'disaster site' in p
    assert '2 search robots' in p
    for frag in ('robot 1 at (12, -40)', 'robot 2 at (-88, 15)',
                 'region 0 at (55, 120)', 'region 3 at (-20, 60)',
                 'region 11 at (150, -60)'):
        assert frag in p, frag
    assert 'd(robot 1) = 55 m' in p
    assert 'new area 42' in p
    assert 'currently working on region 3' in p
    assert 'out of radio contact' in p
    assert 'person at (30, 12)' in p
    assert '4-sided polygon' in p
    assert 'STRICT JSON' in p
    assert MODEL_JSON_EXAMPLE in p


def test_prompt_text_mode_drops_the_image():
    p = build_prompt(_req(), with_image=False)
    assert '<image>' not in p
    assert 'no picture of the map' in p
    # The coordinates are what makes text-only viable at all.
    assert 'region 0 at (55, 120)' in p


def test_prompt_survives_an_empty_request():
    p = build_prompt({}, with_image=False)
    assert 'the search target' in p
    assert '(none)' in p and '(none reported)' in p


def test_prompt_relaxes_uniqueness_when_regions_are_scarce():
    req = _req()
    req['regions'] = req['regions'][:1]
    assert 'fewer regions than robots' in build_prompt(req)
    assert 'Never give the same region' in build_prompt(_req())


# ── response parsing ────────────────────────────────────────────────────────

VALID = [0, 3, 11]
ROBOTS = ['1', '2']


def test_parse_clean_json():
    a, info = parse_assignments('{"assignments": {"1": 3, "2": 0}}', VALID, ROBOTS)
    assert a == {'1': 3, '2': 0}
    assert info['method'] == 'json'
    assert info['dropped'] == []


def test_parse_markdown_fenced():
    txt = 'Sure!\n```json\n{"assignments": {"1": 11, "2": 3}}\n```\nHope that helps.'
    a, info = parse_assignments(txt, VALID, ROBOTS)
    assert a == {'1': 11, '2': 3}
    assert info['method'] == 'json'


def test_parse_prose_around_json():
    txt = ('Robot 1 is nearest region 0 so I will send it there. '
           'Final answer: {"assignments": {"1": 0, "2": 3}} '
           'because that spreads the team out.')
    a, _ = parse_assignments(txt, VALID, ROBOTS)
    assert a == {'1': 0, '2': 3}


def test_parse_string_and_float_values():
    a, _ = parse_assignments('{"assignments": {"1": "3", "2": 0.0}}', VALID, ROBOTS)
    assert a == {'1': 3, '2': 0}


def test_parse_unquoted_integer_keys():
    # Not valid JSON, but a very common 2B output; salvage has to catch it.
    a, info = parse_assignments('{"assignments": {1: 3, 2: 0}}', VALID, ROBOTS)
    assert a == {'1': 3, '2': 0}
    assert info['method'] == 'salvage'


def test_parse_bare_mapping_without_the_wrapper():
    a, info = parse_assignments('{"1": 3, "2": 11}', VALID, ROBOTS)
    assert a == {'1': 3, '2': 11}
    assert info['method'] == 'json'


def test_parse_list_of_records():
    txt = '{"assignments": [{"robot": 1, "region": 11}, {"robot": 2, "region": 0}]}'
    a, _ = parse_assignments(txt, VALID, ROBOTS)
    assert a == {'1': 11, '2': 0}


def test_parse_drops_region_ids_out_of_range():
    a, info = parse_assignments('{"assignments": {"1": 3, "2": 99}}', VALID, ROBOTS)
    assert a == {'1': 3}
    assert any('99' in d for d in info['dropped'])


def test_parse_drops_unknown_robot_ids():
    a, info = parse_assignments('{"assignments": {"1": 3, "9": 0}}', VALID, ROBOTS)
    assert a == {'1': 3}
    assert any('robot 9' in d for d in info['dropped'])


def test_parse_all_entries_invalid_is_a_fallback():
    a, info = parse_assignments('{"assignments": {"1": 99, "2": 98}}', VALID, ROBOTS)
    assert a == {}
    assert info['method'] == 'none'
    assert info['error']


def test_parse_non_json_answer():
    a, info = parse_assignments(
        'I am sorry, I cannot see the map well enough to decide.', VALID, ROBOTS)
    assert a == {}
    assert info['method'] == 'none'
    assert 'no JSON object' in info['error']


def test_parse_empty_and_none():
    for txt in ('', '   ', None):
        a, info = parse_assignments(txt, VALID, ROBOTS)
        assert a == {} and info['method'] == 'none'


def test_parse_salvages_prose_pairs():
    a, info = parse_assignments(
        'robot 1 -> region 11 and robot 2 -> region 3', VALID, ROBOTS)
    assert a == {'1': 11, '2': 3}
    assert info['method'] == 'salvage'


def test_parse_salvage_ignores_unknown_robots():
    a, _ = parse_assignments('robot 7 -> region 3', VALID, ROBOTS)
    assert a == {}


def test_parse_truncated_json_falls_through_to_salvage():
    a, info = parse_assignments('{"assignments": {"1": 3, "2": 0', VALID, ROBOTS)
    assert a == {'1': 3, '2': 0}
    assert info['method'] == 'salvage'


def test_parse_never_raises_on_hostile_input():
    for txt in ('{{{{', '}{', '[]', 'null', '{"assignments": null}',
                '{"assignments": []}', '{"assignments": "3"}',
                '{"assignments": {"a": "b"}}', '\x00\x01', '{' * 500):
        a, info = parse_assignments(txt, VALID, ROBOTS)
        assert isinstance(a, dict)
        assert isinstance(info['dropped'], list)


def test_parse_with_no_declared_robots_accepts_any_id():
    a, _ = parse_assignments('{"assignments": {"5": 3}}', VALID, [])
    assert a == {'5': 3}


# ── outgoing messages ───────────────────────────────────────────────────────

def test_build_assignment_matches_the_contract():
    m = build_assignment(_req(), {'1': 3, '2': 0}, 'OpenGVLab/InternVL3-2B',
                         4.2019, raw='x' * (RAW_MAX_CHARS + 500), ts=1.0)
    assert set(m) == {'round', 'ts', 'model', 'assignments', 'latency_s',
                      'fallback', 'raw', 'reason'}
    assert m['round'] == 7 and m['ts'] == 1.0
    assert m['assignments'] == {'1': 3, '2': 0}
    assert m['latency_s'] == 4.202
    assert m['fallback'] is False and m['reason'] == ''
    assert len(m['raw']) == RAW_MAX_CHARS
    json.dumps(m)                      # must survive the wire


def test_fallback_assignment_is_empty_with_a_reason():
    m = fallback_assignment(_req(), 'model still loading', ts=2.0)
    assert m['assignments'] == {}
    assert m['fallback'] is True
    assert m['reason'] == 'model still loading'


def test_round_table_names_the_unassigned_robot():
    req = _req()
    m = build_assignment(req, {'1': 3}, 'm', 1.0, ts=0.0)
    t = build_round_table(req, m, {'dropped': ['region 99 not offered to robot 2']},
                          superseded=[5, 6])
    assert 'robot 1 -> region 3 (-20, 60) d=103.4m' in t
    assert 'robot 2 -> nearest-frontier fallback' in t
    assert 'region 99' in t
    assert 'superseded rounds: 5, 6' in t


def test_round_table_on_a_fallback_round():
    req = _req()
    m = fallback_assignment(req, 'unusable model response', ts=0.0)
    t = build_round_table(req, m)
    assert 'FALLBACK' in t
    assert t.count('nearest-frontier fallback') == 2


# ── BEV projection ──────────────────────────────────────────────────────────

def test_extent_is_square_and_contains_every_point():
    x0, x1, y0, y1 = bev_extent(_req())
    assert (x1 - x0) == pytest.approx(y1 - y0)
    for gx, gy in [(g['x'], g['y']) for g in parse_regions(_req())] + \
                  [(r['x'], r['y']) for r in parse_robots(_req())] + \
                  [(-100.0, -100.0), (200.0, 200.0), (30.0, 12.0)]:
        assert x0 < gx < x1 and y0 < gy < y1


def test_extent_of_an_empty_request_is_finite():
    x0, x1, y0, y1 = bev_extent({})
    assert x1 > x0 and y1 > y0
    assert all(np.isfinite(v) for v in (x0, x1, y0, y1))


def test_extent_pads_a_single_point():
    x0, x1, _, _ = bev_extent({'robots': [{'id': 1, 'x': 5.0, 'y': 5.0}]})
    assert (x1 - x0) >= 30.0


def test_projector_orients_north_up_east_right():
    px = 512
    extent = (0.0, 100.0, 0.0, 100.0)
    p = bev_projector(extent, px)
    assert p(0.0, 100.0) == pytest.approx((0.0, 0.0))            # NW -> top-left
    assert p(100.0, 0.0) == pytest.approx((px - 1, px - 1))      # SE -> bottom-right
    east = p(75.0, 50.0)
    west = p(25.0, 50.0)
    assert east[0] > west[0]
    north = p(50.0, 75.0)
    south = p(50.0, 25.0)
    assert north[1] < south[1]


def test_projector_is_isotropic():
    p = bev_projector((0.0, 200.0, -50.0, 150.0), 400)
    a, b = p(0.0, 0.0), p(10.0, 0.0)
    c, e = p(0.0, 0.0), p(0.0, 10.0)
    assert abs(b[0] - a[0]) == pytest.approx(abs(e[1] - c[1]))


# ── BEV rendering ───────────────────────────────────────────────────────────

def test_render_returns_a_usable_rgb_image():
    img = render_bev(_req(), px=512)
    assert img.shape == (512, 512, 3)
    assert img.dtype == np.uint8
    # Not a flat background: markers, grid and legend all had to land.
    assert len(np.unique(img.reshape(-1, 3), axis=0)) > 20


def test_render_clamps_absurd_sizes():
    assert render_bev(_req(), px=1).shape[0] == 256
    assert render_bev(_req(), px=99999).shape[0] == 2048


def test_render_puts_markers_where_the_projector_says():
    px = 768
    img = render_bev(_req(), px=px)
    p = bev_projector(bev_extent(_req()), px)
    for g in parse_regions(_req()):
        cx, cy = p(g['x'], g['y'])
        patch = img[int(cy) - 8:int(cy) + 8, int(cx) - 8:int(cx) + 8]
        # Orange region disc or the dark glyph drawn on top of it.
        assert patch[..., 0].max() > 150


def test_render_survives_degenerate_requests():
    for req in ({}, {'regions': [], 'robots': []},
                {'robots': [{'id': 1, 'x': 0.0, 'y': 0.0}]},
                {'search_area': [[0, 0], [1, 1]]},
                {'query': 'x' * 300,
                 'regions': [{'id': i, 'x': 0.0, 'y': 0.0} for i in range(40)]}):
        img = render_bev(req, px=256)
        assert img.shape == (256, 256, 3)


def test_render_ignores_an_absent_observed_layer():
    a = render_bev(_req(), px=256)
    req = _req()
    req['observed'] = [[0.0, 0.0], [10.0, 10.0]]
    b = render_bev(req, px=256)
    assert a.shape == b.shape
    assert not np.array_equal(a, b)


def test_marker_font_shrinks_for_multi_digit_ids():
    one = _fit_size(20, 3)
    two = _fit_size(20, 11)
    assert one > two >= 8
    # Two digits must still fit across the marker's diameter.
    assert 2 * 0.6 * two <= 2 * 20


def test_negative_current_region_reads_as_unassigned():
    # raven_nav's "no region yet" sentinel is -1, not a missing key.
    req = {'robots': [{'id': 1, 'x': 0.0, 'y': 0.0, 'current_region': -1}],
           'regions': [{'id': 0, 'x': 5.0, 'y': 5.0}]}
    assert parse_robots(req)[0]['current_region'] is None
    p = build_prompt(req, with_image=False)
    assert 'region -1' not in p
    assert 'not assigned yet' in p
