"""Co-NavGPT VLM assigner — the "VLM-Assign" arm.

Co-NavGPT (Yu et al., arXiv:2310.07937) shows a vision-language model the team's
shared map with numbered frontier regions and numbered robots and asks it, in a
single call for the WHOLE TEAM, which robot should go to which region. This node
is that call. ONE instance runs, on the leader robot only; every other robot
receives the answer over the gossip protocol.

Adaptation (COA-docs/multi_robot_baselines.md sec. 9): the published method is
single-target, terminates on the first find, and starts the robots co-located.
This arm is multi-target, never terminates early, starts the robots apart, and
is scored time-integrated. That difference is the "VLM-Assign" arm and must be
stated wherever this baseline is reported.

The node owns only the VLM. Frontier extraction, clustering, coverage and the
results dump stay in raven_nav — only frontier *selection* is replaced here — so
this process never has to agree with raven_nav about anything but JSON.

Topics (robot = ROBOT_NAME, the leader):
  sub  /{robot}/conavgpt/assign_request   std_msgs/String   JSON, from raven_nav
  pub  /{robot}/conavgpt/assignment       std_msgs/String   JSON, gossiped to peers
  pub  /{robot}/conavgpt/map_image        sensor_msgs/Image the BEV actually fed to the model
  pub  /{robot}/conavgpt/round_table      std_msgs/String   human-readable summary

All coordinates on the wire are global ENU metres (x = east, y = north);
raven_nav converts to and from its local 'map' frame on its side.

Import layout: everything heavy (rclpy, torch, transformers, cv_bridge) is
imported behind a guard so the pure helpers below — prompt building, response
parsing, BEV projection and rendering — import and unit-test on a machine with
neither ROS nor a GPU.
"""

import json
import math
import os
import re
import signal
import threading
import time

import numpy as np

try:
    from PIL import Image as PIL_Image
    from PIL import ImageDraw, ImageFont
    _PIL_OK = True
except Exception:                                   # noqa: BLE001
    _PIL_OK = False

try:
    import rclpy
    import rclpy.executors
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    _ROS_OK = True
except Exception:                                   # noqa: BLE001
    _ROS_OK = False
    Node = object

try:
    import torch
    import torchvision.transforms as T
    from torchvision.transforms.functional import InterpolationMode
    from transformers import (AutoConfig, AutoModel, AutoTokenizer,
                              BitsAndBytesConfig)
    _TORCH_OK = True
except Exception:                                   # noqa: BLE001
    _TORCH_OK = False

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)

# Contract: the raw model text carried on the assignment message is truncated.
RAW_MAX_CHARS = 2000
# The one output shape the model is asked for; also the parser's target.
MODEL_JSON_EXAMPLE = '{"assignments": {"1": 3, "2": 0}}'
# A square BEV retiles to 2x2 448px tiles plus a thumbnail at this cap, which is
# what makes two-digit markers legible to the vision encoder.
BEV_TILES = 6

# ── BEV palette ─────────────────────────────────────────────────────────────
C_BG = (18, 20, 26)
C_GRID = (46, 51, 62)
C_PANEL = (30, 34, 43)
C_AREA = (96, 176, 255)
C_OBSERVED = (58, 66, 80)
C_REGION = (255, 150, 40)
C_ROBOT = (245, 248, 255)
C_FOUND = (255, 72, 72)
C_TEXT = (232, 236, 244)
C_DIM = (150, 158, 172)


# ══ pure helpers — no ROS, no CUDA ═══════════════════════════════════════════

def _as_int(v):
    """int(v) for ints, numeric strings and whole floats; None otherwise."""
    if isinstance(v, bool):
        return None
    if isinstance(v, int):
        return v
    if isinstance(v, float):
        return int(v) if float(v).is_integer() else None
    if isinstance(v, str):
        s = v.strip()
        m = re.fullmatch(r'[+-]?\d+', s)
        if m:
            return int(s)
        try:
            f = float(s)
        except ValueError:
            return None
        return int(f) if f.is_integer() else None
    return None


def _as_float(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def _f(v, nd=0):
    """Short fixed-point for prompt text; '?' for a missing number."""
    fv = _as_float(v)
    return '?' if fv is None else f'{fv:.{nd}f}'


def parse_robots(req):
    """Normalize request['robots'] -> [{'id': str, 'x','y','z': float,
    'fresh': bool, 'current_region': int|None}]. Unusable entries drop out."""
    out = []
    for r in (req.get('robots') or []):
        if not isinstance(r, dict):
            continue
        rid = _as_int(r.get('id'))
        x, y = _as_float(r.get('x')), _as_float(r.get('y'))
        if rid is None or x is None or y is None:
            continue
        # raven_nav sends -1 for "no region yet"; region ids are non-negative,
        # so anything negative collapses to None rather than reaching the
        # prompt as "currently working on region -1".
        cur = _as_int(r.get('current_region'))
        out.append({
            'id': str(rid), 'x': x, 'y': y,
            'z': _as_float(r.get('z')) if _as_float(r.get('z')) is not None else 0.0,
            'fresh': bool(r.get('fresh', True)),
            'current_region': cur if (cur is not None and cur >= 0) else None,
        })
    return out


def parse_regions(req):
    """Normalize request['regions'] -> [{'id': int, 'x','y','z': float,
    'info_gain': float|None, 'dist_by_robot': {robot_id_str: float}}]."""
    out = []
    for g in (req.get('regions') or []):
        if not isinstance(g, dict):
            continue
        gid = _as_int(g.get('id'))
        x, y = _as_float(g.get('x')), _as_float(g.get('y'))
        if gid is None or x is None or y is None:
            continue
        dist = {}
        raw_dist = g.get('dist_by_robot')
        if isinstance(raw_dist, dict):
            for k, v in raw_dist.items():
                rid, fv = _as_int(k), _as_float(v)
                if rid is not None and fv is not None:
                    dist[str(rid)] = fv
        out.append({
            'id': gid, 'x': x, 'y': y,
            'z': _as_float(g.get('z')) if _as_float(g.get('z')) is not None else 0.0,
            'info_gain': _as_float(g.get('info_gain')),
            'dist_by_robot': dist,
        })
    return out


def parse_points(seq):
    """Normalize a [[x, y], ...] or [{'x':..,'y':..,'label':..}, ...] list."""
    out = []
    for p in (seq or []):
        if isinstance(p, dict):
            x, y = _as_float(p.get('x')), _as_float(p.get('y'))
            label = str(p.get('label') or '').strip()
        elif isinstance(p, (list, tuple)) and len(p) >= 2:
            x, y, label = _as_float(p[0]), _as_float(p[1]), ''
        else:
            continue
        if x is not None and y is not None:
            out.append({'x': x, 'y': y, 'label': label})
    return out


# ── prompt ──────────────────────────────────────────────────────────────────

def build_prompt(req, with_image=True):
    """The single team-assignment question, verbatim.

    with_image=False drops the <image> token and the sentence describing the
    picture, so the same text is valid when rendering failed or prompt_mode is
    'text'. Coordinates are always in the text: a 2B model reads numbers far
    more reliably than it reads a map."""
    robots = parse_robots(req)
    regions = parse_regions(req)
    found = parse_points(req.get('found'))
    area = parse_points(req.get('search_area'))
    query = str(req.get('query') or '').strip() or 'the search target'

    L = []
    if with_image:
        L.append('<image>')
    L.append(
        f'You are the coordinator of a team of {len(robots)} search robots '
        '(drones) flying over a disaster site. The robots share one map and '
        'must find every target as fast as they can.')
    if with_image:
        L.append(
            'The picture is a top-down map of that site, north up and east '
            'right. Numbered white squares are the robots. Numbered orange '
            'circles are the candidate regions listed below. Red crosses are '
            'targets that have already been found. The blue outline is the '
            'search area.')
    else:
        L.append('You have no picture of the map; use the coordinates below.')
    L.append('')
    L.append(f'Search target: {query}')
    L.append('')

    L.append('Robots (global ENU metres, x = east, y = north):')
    if robots:
        for r in robots:
            bits = [f'  robot {r["id"]} at ({_f(r["x"])}, {_f(r["y"])})']
            if r['current_region'] is not None:
                bits.append(f'currently working on region {r["current_region"]}')
            else:
                bits.append('not assigned yet')
            if not r['fresh']:
                bits.append('last known position, currently out of radio contact')
            L.append(', '.join(bits))
    else:
        L.append('  (none reported)')
    L.append('')

    L.append('Candidate regions (global ENU metres; "new area" is how much '
             'unexplored ground the region opens up, "d(robot N)" is how far '
             'robot N must fly to reach it):')
    if regions:
        for g in regions:
            bits = [f'  region {g["id"]} at ({_f(g["x"])}, {_f(g["y"])})']
            if g['info_gain'] is not None:
                bits.append(f'new area {_f(g["info_gain"])}')
            for rid in sorted(g['dist_by_robot'], key=lambda s: int(s)):
                bits.append(f'd(robot {rid}) = {_f(g["dist_by_robot"][rid])} m')
            L.append(', '.join(bits))
    else:
        L.append('  (none)')
    L.append('')

    if found:
        L.append('Targets already found (do not send anyone back to these):')
        for p in found:
            name = p['label'] or 'target'
            L.append(f'  {name} at ({_f(p["x"])}, {_f(p["y"])})')
        L.append('')
    if len(area) >= 3:
        L.append(f'The search area is a {len(area)}-sided polygon; every region '
                 'listed above is already inside it.')
        L.append('')

    L.append('Assign every robot exactly one region. Rules:')
    if regions and robots and len(regions) < len(robots):
        L.append('  1. There are fewer regions than robots, so some robots have '
                 'to share; spread them as evenly as you can.')
    else:
        L.append('  1. Never give the same region to two robots — the team has '
                 'to cover the site in parallel.')
    L.append('  2. Prefer a region that is close to that robot, so little time '
             'is spent flying.')
    L.append('  3. Prefer a region that opens up more new area, and a region '
             f'where {query} is likely to be found.')
    L.append('  4. Keep robots away from places where a target was already found.')
    L.append('')
    L.append('Answer with STRICT JSON and nothing else, in exactly this form:')
    L.append(MODEL_JSON_EXAMPLE)
    L.append('The keys are robot numbers and the values are region numbers. Use '
             'only the numbers listed above. No explanation, no markdown.')
    return '\n'.join(L)


# ── response parsing ────────────────────────────────────────────────────────

_FENCE_RE = re.compile(r'```(?:json|JSON)?\s*(.*?)```', re.S)
# Last-resort salvage: "robot 2 -> region 5", '"2": 5', "2 = 5".
_PAIR_RE = re.compile(
    r'"?(?:robot[\s_-]*)?(\d+)"?\s*(?:->|=>|:|=)\s*"?(?:region[\s_-]*)?(\d+)"?',
    re.I)
_MAP_KEYS = ('assignments', 'assignment', 'robots', 'result', 'answer')
_ROBOT_KEYS = ('robot', 'robot_id', 'id', 'agent')
_REGION_KEYS = ('region', 'region_id', 'frontier', 'target', 'goal')


def _json_candidates(text):
    """Yield JSON-ish substrings, most-likely first: fenced blocks, the whole
    reply, then every balanced {...} block in order of appearance."""
    t = (text or '').strip()
    if not t:
        return
    for m in _FENCE_RE.finditer(t):
        inner = m.group(1).strip()
        if inner:
            yield inner
    yield t
    depth, start = 0, -1
    for i, ch in enumerate(t):
        if ch == '{':
            if depth == 0:
                start = i
            depth += 1
        elif ch == '}' and depth > 0:
            depth -= 1
            if depth == 0 and start >= 0:
                yield t[start:i + 1]


def _first_int(d, keys):
    for k in keys:
        if k in d:
            v = _as_int(d[k])
            if v is not None:
                return v
    return None


def _pairs_from_obj(obj):
    """Pull a robot->region mapping out of one parsed JSON object."""
    if not isinstance(obj, dict):
        return None
    for key in _MAP_KEYS:
        v = obj.get(key)
        if isinstance(v, dict) and v:
            return v
        if isinstance(v, list) and v:
            rec = {}
            for e in v:
                if not isinstance(e, dict):
                    continue
                r = _first_int(e, _ROBOT_KEYS)
                g = _first_int(e, _REGION_KEYS)
                if r is not None and g is not None:
                    rec[str(r)] = g
            if rec:
                return rec
    # A 2B model very often skips the wrapper and emits {"1": 3} directly.
    if obj and all(_as_int(k) is not None and _as_int(v) is not None
                   for k, v in obj.items()):
        return obj
    return None


def parse_assignments(text, valid_region_ids, robot_ids):
    """Defensively read a robot -> region mapping out of a model reply.

    Returns (assignments, info) where assignments is {robot_id_str: region_id
    int} containing only ids the request actually offered, and info carries
    {'method', 'dropped', 'error'} for the round record. An empty assignments
    dict is the caller's cue to emit fallback:true — raven_nav then uses
    nearest-frontier per robot. This function never raises."""
    valid_regions = {int(g) for g in (valid_region_ids or [])}
    known_robots = {str(r) for r in (robot_ids or [])}
    info = {'method': 'none', 'dropped': [], 'error': ''}

    pairs, method = None, 'none'
    for blob in _json_candidates(text):
        try:
            obj = json.loads(blob)
        except Exception:                           # noqa: BLE001
            continue
        got = _pairs_from_obj(obj)
        if got:
            pairs, method = got, 'json'
            break
    if pairs is None:
        # No JSON survived; scrape numeric pairs but accept only robots the
        # request named, which keeps stray numbers in prose out.
        rec = {}
        for m in _PAIR_RE.finditer(text or ''):
            rid = m.group(1)
            if known_robots and rid not in known_robots:
                continue
            rec[rid] = _as_int(m.group(2))
        if rec:
            pairs, method = rec, 'salvage'
    if pairs is None:
        info['error'] = 'no JSON object and no robot:region pair in the reply'
        return {}, info

    out = {}
    for k, v in pairs.items():
        rid, reg = _as_int(k), _as_int(v)
        if rid is None or reg is None:
            info['dropped'].append(f'{k!r}:{v!r} not a number pair')
            continue
        rkey = str(rid)
        if known_robots and rkey not in known_robots:
            info['dropped'].append(f'robot {rkey} not in this request')
            continue
        if reg not in valid_regions:
            info['dropped'].append(f'region {reg} not offered to robot {rkey}')
            continue
        out[rkey] = reg
    info['method'] = method if out else 'none'
    if not out and not info['error']:
        info['error'] = 'every entry the model returned was out of range'
    return out, info


# ── messages ────────────────────────────────────────────────────────────────

def build_assignment(req, assignments, model, latency_s, raw='',
                     fallback=False, reason='', ts=None):
    """The /conavgpt/assignment payload, exactly as the contract specifies."""
    return {
        'round': req.get('round'),
        'ts': float(ts) if ts is not None else time.time(),
        'model': str(model),
        'assignments': {str(k): int(v) for k, v in (assignments or {}).items()},
        'latency_s': round(float(latency_s), 3),
        'fallback': bool(fallback),
        'raw': (raw or '')[:RAW_MAX_CHARS],
        'reason': str(reason or ''),
    }


def fallback_assignment(req, reason, model='', latency_s=0.0, raw='', ts=None):
    """Empty assignment + a reason. Every robot then takes nearest-frontier,
    which is the measured degradation, not an error path to hide."""
    return build_assignment(req, {}, model, latency_s, raw,
                            fallback=True, reason=reason, ts=ts)


def build_round_table(req, assignment, info=None, superseded=()):
    """One human-readable block per round for /conavgpt/round_table."""
    info = info or {}
    robots = parse_robots(req)
    regions = {g['id']: g for g in parse_regions(req)}
    a = assignment.get('assignments') or {}
    head = (f'CoNavGPT round {assignment.get("round")} | '
            f'leader robot_{req.get("leader", "?")} | '
            f'{assignment.get("model", "?")}')
    sub = (f'query="{req.get("query", "")}" | {len(robots)} robots | '
           f'{len(regions)} regions | {assignment.get("latency_s", 0.0):.2f}s'
           + (' | FALLBACK' if assignment.get('fallback') else ''))
    lines = [head, sub, 'assignments:']
    if not robots:
        lines.append('  (no robots in request)')
    for r in robots:
        gid = a.get(r['id'])
        if gid is None:
            lines.append(f'  robot {r["id"]} -> nearest-frontier fallback')
            continue
        g = regions.get(gid)
        where = f'({_f(g["x"])}, {_f(g["y"])})' if g else '(unknown)'
        dist = g['dist_by_robot'].get(r['id']) if g else None
        dtxt = f' d={_f(dist, 1)}m' if dist is not None else ''
        lines.append(f'  robot {r["id"]} -> region {gid} {where}{dtxt}')
    if assignment.get('reason'):
        lines.append(f'reason: {assignment["reason"]}')
    if info.get('dropped'):
        lines.append('dropped: ' + '; '.join(str(d) for d in info['dropped']))
    if superseded:
        lines.append('superseded rounds: '
                     + ', '.join(str(s) for s in superseded))
    return '\n'.join(lines)


# ── BEV projection + rendering (numpy + PIL only) ───────────────────────────

def bev_points(req):
    """Every world XY the map has to contain."""
    pts = [(g['x'], g['y']) for g in parse_regions(req)]
    pts += [(r['x'], r['y']) for r in parse_robots(req)]
    pts += [(p['x'], p['y']) for p in parse_points(req.get('search_area'))]
    pts += [(p['x'], p['y']) for p in parse_points(req.get('found'))]
    pts += [(p['x'], p['y']) for p in parse_points(req.get('observed'))]
    return pts


def bev_extent(req, margin_frac=0.10, min_span_m=30.0):
    """Square world window (x0, x1, y0, y1) covering everything in the request.

    Square and equal-scale on both axes: the model is asked to reason about
    which region is nearer, so the picture must not stretch one axis."""
    pts = bev_points(req)
    if not pts:
        cx = cy = 0.0
        span = float(min_span_m)
    else:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        cx, cy = 0.5 * (min(xs) + max(xs)), 0.5 * (min(ys) + max(ys))
        span = max(max(xs) - min(xs), max(ys) - min(ys), float(min_span_m))
    span *= (1.0 + 2.0 * float(margin_frac))
    h = 0.5 * span
    return (cx - h, cx + h, cy - h, cy + h)


def bev_projector(extent, px):
    """world (x, y) -> pixel (col, row). North is up, so row inverts y."""
    x0, x1, y0, y1 = (float(v) for v in extent)
    span = max(x1 - x0, 1e-6)
    s = (int(px) - 1) / span

    def project(x, y):
        return ((float(x) - x0) * s, (y1 - float(y)) * s)
    return project


def _nice_step(span, divs=6):
    """Round 'span/divs' down to a 1/2/5 x 10^k step, for grid + scale bar."""
    raw = max(float(span), 1e-6) / max(int(divs), 1)
    mag = 10.0 ** math.floor(math.log10(raw))
    for m in (1.0, 2.0, 5.0):
        if raw <= m * mag:
            return m * mag
    return 10.0 * mag


_FONT_PATHS = (
    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf',
    '/usr/share/fonts/truetype/freefont/FreeSansBold.ttf',
    'DejaVuSans-Bold.ttf',
)
_FONT_CACHE = {}


def _font(size):
    """(font, scalable). The bitmap default is ~11 px, which disappears once the
    vision encoder retiles the map, so callers upscale it by hand."""
    size = int(max(6, size))
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    got = None
    for path in _FONT_PATHS:
        try:
            got = (ImageFont.truetype(path, size), True)
            break
        except Exception:                           # noqa: BLE001
            continue
    if got is None:
        try:
            got = (ImageFont.load_default(size=size), True)
        except TypeError:                           # Pillow < 10.1
            got = (ImageFont.load_default(), False)
    _FONT_CACHE[size] = got
    return got


def _fit_size(radius, s):
    """Largest font size whose digits still sit inside a marker of `radius`.

    Region ids reach two digits, and a glyph that spills past the disc is read
    back as a different number."""
    return int(max(8, min(radius * 1.5, radius * 2.6 / max(1, len(str(s))))))


def _text(img, draw, xy, s, size, fill, anchor='mm'):
    """Draw text at `size` px whether or not a scalable font exists."""
    s = str(s)
    font, scalable = _font(size)
    if scalable:
        draw.text(xy, s, font=font, fill=fill, anchor=anchor)
        return
    k = max(1, int(round(size / 11.0)))
    tmp = PIL_Image.new('RGBA', (max(8, 7 * len(s) + 4), 14), (0, 0, 0, 0))
    ImageDraw.Draw(tmp).text((1, 0), s, font=font, fill=tuple(fill) + (255,))
    tmp = tmp.resize((tmp.width * k, tmp.height * k), PIL_Image.NEAREST)
    x, y = xy
    if anchor == 'mm':
        x -= tmp.width / 2.0
        y -= tmp.height / 2.0
    img.paste(tmp, (int(x), int(y)), tmp)


def render_bev(req, px=768):
    """Top-down team map as an (px, px, 3) uint8 RGB array.

    Raises on any PIL/font problem; the caller falls back to text-only
    prompting rather than skipping the round."""
    if not _PIL_OK:
        raise RuntimeError('Pillow is not importable — cannot render the BEV')
    px = int(max(256, min(2048, int(px))))
    extent = bev_extent(req)
    to_px = bev_projector(extent, px)
    x0, x1, y0, y1 = extent
    span = x1 - x0

    img = PIL_Image.new('RGB', (px, px), C_BG)
    d = ImageDraw.Draw(img)

    # Grid: gives the model a metric ruler instead of bare marker positions.
    step = _nice_step(span, 6)
    lab = max(9, px // 64)
    k = math.ceil(x0 / step)
    while k * step <= x1:
        cx, _ = to_px(k * step, y0)
        d.line([(cx, 0), (cx, px)], fill=C_GRID, width=1)
        _text(img, d, (cx + 3, px - lab - 6), f'{k * step:.0f}', lab, C_DIM, 'la')
        k += 1
    k = math.ceil(y0 / step)
    while k * step <= y1:
        _, cy = to_px(x0, k * step)
        d.line([(0, cy), (px, cy)], fill=C_GRID, width=1)
        _text(img, d, (4, cy + 2), f'{k * step:.0f}', lab, C_DIM, 'la')
        k += 1

    # Already-observed ground, when raven_nav sends it (optional key).
    obs = parse_points(req.get('observed'))
    if obs:
        r = max(2, px // 220)
        for p in obs:
            cx, cy = to_px(p['x'], p['y'])
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=C_OBSERVED)

    area = parse_points(req.get('search_area'))
    if len(area) >= 3:
        poly = [to_px(p['x'], p['y']) for p in area]
        d.line(poly + [poly[0]], fill=C_AREA, width=max(2, px // 300))

    fr = max(6, px // 70)
    for p in parse_points(req.get('found')):
        cx, cy = to_px(p['x'], p['y'])
        w = max(3, px // 200)
        d.line([(cx - fr, cy - fr), (cx + fr, cy + fr)], fill=C_FOUND, width=w)
        d.line([(cx - fr, cy + fr), (cx + fr, cy - fr)], fill=C_FOUND, width=w)

    # Regions and robots last, and oversized: these carry the numbers the model
    # has to read back, and a 448 px tile is unforgiving.
    gr = max(14, px // 34)
    for g in parse_regions(req):
        cx, cy = to_px(g['x'], g['y'])
        d.ellipse([cx - gr, cy - gr, cx + gr, cy + gr],
                  fill=C_REGION, outline=C_BG, width=max(2, px // 380))
        _text(img, d, (cx, cy), g['id'], _fit_size(gr, g['id']), C_BG, 'mm')

    rr = max(14, px // 36)
    for r in parse_robots(req):
        cx, cy = to_px(r['x'], r['y'])
        d.rectangle([cx - rr, cy - rr, cx + rr, cy + rr],
                    fill=C_ROBOT, outline=C_BG, width=max(2, px // 380))
        _text(img, d, (cx, cy), r['id'], _fit_size(rr, r['id']), C_BG, 'mm')

    _draw_legend(img, d, px, req)
    _draw_scale(img, d, px, span)
    return np.asarray(img, dtype=np.uint8)


def _draw_legend(img, d, px, req):
    fs = max(11, px // 46)
    pad = max(6, px // 90)
    rows = [(C_ROBOT, 'square = robot'),
            (C_REGION, 'circle = candidate region'),
            (C_FOUND, 'X = target already found'),
            (C_AREA, 'outline = search area')]
    w = int(px * 0.42)
    h = pad * 2 + len(rows) * (fs + pad // 2) + fs + pad
    d.rectangle([pad, pad, pad + w, pad + h], fill=C_PANEL, outline=C_GRID)
    y = pad * 2
    _text(img, d, (pad * 2, y), f'target: {req.get("query") or ""}'[:44],
          fs, C_TEXT, 'la')
    y += fs + pad
    for color, label in rows:
        d.rectangle([pad * 2, y + 2, pad * 2 + fs - 4, y + fs - 2], fill=color)
        _text(img, d, (pad * 3 + fs, y), label, fs, C_TEXT, 'la')
        y += fs + pad // 2
    # North arrow, top-right: the model is told north is up, so show it.
    ax = px - pad * 3
    ay = pad * 3
    d.polygon([(ax, ay - fs), (ax - fs * 0.6, ay + fs * 0.6),
               (ax + fs * 0.6, ay + fs * 0.6)], fill=C_TEXT)
    _text(img, d, (ax, ay + fs * 1.6), 'N', fs, C_TEXT, 'mm')


def _draw_scale(img, d, px, span):
    fs = max(11, px // 46)
    bar_m = _nice_step(span, 5)
    bar_px = bar_m / max(span, 1e-6) * (px - 1)
    x1 = px - max(6, px // 90) * 2
    x0 = x1 - bar_px
    y = px - max(6, px // 90) * 2 - int(fs * 2.4)
    d.line([(x0, y), (x1, y)], fill=C_TEXT, width=max(2, px // 300))
    for x in (x0, x1):
        d.line([(x, y - fs // 2), (x, y + fs // 2)], fill=C_TEXT,
               width=max(2, px // 300))
    _text(img, d, ((x0 + x1) / 2.0, y + fs * 0.9), f'{bar_m:.0f} m', fs,
          C_TEXT, 'mm')


# ══ InternVL3 preprocessing ══════════════════════════════════════════════════
#
# Duplicated from lvlm_baseline rather than imported: there they are methods on
# an rclpy Node subclass, so importing them would drag rclpy, task_msgs and
# coordination_msgs into this module at import time and cost this node its
# ability to be unit-tested (and its independence from the robot's action
# stack). The bodies are identical, so the two baselines feed the model the same
# pixels.

def build_transform(input_size):
    return T.Compose([
        T.Lambda(lambda img: img.convert('RGB') if img.mode != 'RGB' else img),
        T.Resize((input_size, input_size), interpolation=InterpolationMode.BICUBIC),
        T.ToTensor(),
        T.Normalize(mean=IMAGENET_MEAN, std=IMAGENET_STD),
    ])


def find_closest_aspect_ratio(aspect_ratio, target_ratios, width, height, image_size):
    best_ratio_diff = float('inf')
    best_ratio = (1, 1)
    area = width * height
    for ratio in target_ratios:
        target_aspect_ratio = ratio[0] / ratio[1]
        ratio_diff = abs(aspect_ratio - target_aspect_ratio)
        if ratio_diff < best_ratio_diff:
            best_ratio_diff = ratio_diff
            best_ratio = ratio
        elif ratio_diff == best_ratio_diff:
            if area > 0.5 * image_size * image_size * ratio[0] * ratio[1]:
                best_ratio = ratio
    return best_ratio


def dynamic_preprocess(image, min_num=1, max_num=12, image_size=448,
                       use_thumbnail=False):
    orig_width, orig_height = image.size
    aspect_ratio = orig_width / orig_height
    target_ratios = set(
        (i, j) for n in range(min_num, max_num + 1) for i in range(1, n + 1)
        for j in range(1, n + 1) if i * j <= max_num and i * j >= min_num)
    target_ratios = sorted(target_ratios, key=lambda x: x[0] * x[1])
    target_aspect_ratio = find_closest_aspect_ratio(
        aspect_ratio, target_ratios, orig_width, orig_height, image_size)
    target_width = image_size * target_aspect_ratio[0]
    target_height = image_size * target_aspect_ratio[1]
    blocks = target_aspect_ratio[0] * target_aspect_ratio[1]
    resized_img = image.resize((target_width, target_height))
    processed_images = []
    for i in range(blocks):
        box = (
            (i % (target_width // image_size)) * image_size,
            (i // (target_width // image_size)) * image_size,
            ((i % (target_width // image_size)) + 1) * image_size,
            ((i // (target_width // image_size)) + 1) * image_size,
        )
        processed_images.append(resized_img.crop(box))
    assert len(processed_images) == blocks
    if use_thumbnail and len(processed_images) != 1:
        processed_images.append(image.resize((image_size, image_size)))
    return processed_images


def split_model(model_name):
    device_map = {}
    world_size = max(torch.cuda.device_count(), 1)
    config = AutoConfig.from_pretrained(model_name, trust_remote_code=True)
    num_layers = config.llm_config.num_hidden_layers
    num_layers_per_gpu = math.ceil(num_layers / (world_size - 0.5))
    num_layers_per_gpu = [num_layers_per_gpu] * world_size
    num_layers_per_gpu[0] = math.ceil(num_layers_per_gpu[0] * 0.5)
    layer_cnt = 0
    for i, num_layer in enumerate(num_layers_per_gpu):
        for _ in range(num_layer):
            device_map[f'language_model.model.layers.{layer_cnt}'] = i
            layer_cnt += 1
    device_map['vision_model'] = 0
    device_map['mlp1'] = 0
    device_map['language_model.model.tok_embeddings'] = 0
    device_map['language_model.model.embed_tokens'] = 0
    device_map['language_model.output'] = 0
    device_map['language_model.model.norm'] = 0
    device_map['language_model.model.rotary_emb'] = 0
    device_map['language_model.lm_head'] = 0
    device_map[f'language_model.model.layers.{num_layers - 1}'] = 0
    return device_map


def load_image(image, input_size=448, max_num=1):
    transform = build_transform(input_size=input_size)
    images = dynamic_preprocess(
        image, image_size=input_size, use_thumbnail=True, max_num=max_num)
    pixel_values = [transform(img) for img in images]
    return torch.stack(pixel_values)


# ══ node ════════════════════════════════════════════════════════════════════

class CoNavGPTAssigner(Node):
    """Leader-local VLM assigner. One inference per round, for the whole team."""

    def __init__(self):
        super().__init__('conavgpt_assigner')

        self._robot_name = os.getenv('ROBOT_NAME', 'robot_1')
        self._prefix = f'/{self._robot_name}'

        # ── parameters ────────────────────────────────────────────────────────
        self._model_path = str(self.declare_parameter(
            'model_path', 'OpenGVLab/InternVL3-2B').value)
        mode = str(self.declare_parameter('prompt_mode', 'image+text').value)
        self._prompt_mode = mode if mode in ('image+text', 'text') else 'image+text'
        if self._prompt_mode != mode:
            self.get_logger().warn(
                f"prompt_mode '{mode}' unknown — using 'image+text'")
        self._results_dir = str(self.declare_parameter('results_dir', '').value)
        self._max_new_tokens = int(self.declare_parameter('max_new_tokens', 256).value)
        self._map_px = int(self.declare_parameter('map_px', 768).value)
        # A request older than this when the worker reaches it is answered with
        # a fallback: raven_nav has already moved on, and a stale assignment is
        # worse than none.
        self._request_timeout_s = float(self.declare_parameter(
            'request_timeout_s', 60.0).value)

        self._bridge = CvBridge() if _ROS_OK else None
        # Latest-wins single slot. The worker holds no lock while the model
        # runs, so a request arriving mid-inference just replaces the pending
        # one instead of queueing behind seconds of GPU time.
        self._cv = threading.Condition()
        self._pending = None            # (request dict, arrival stamp)
        self._superseded = []           # rounds dropped while one was in flight

        self._stop = False
        self._model_ready = False
        self.model = None
        self.tokenizer = None
        # Greedy: the answer is a fixed JSON form, and sampling only invents
        # ways to break it.
        self.generation_config = dict(max_new_tokens=self._max_new_tokens,
                                      do_sample=False)
        self._n_rounds = 0
        self._n_fallbacks = 0

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self._cbg = ReentrantCallbackGroup()
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            String, f'{self._prefix}/conavgpt/assign_request',
            self._request_cb, 10, callback_group=self._cbg)
        # Latched: raven_nav and the gossip payload subscriber both start after
        # this node, and a late joiner should still see the standing assignment.
        self._assign_pub = self.create_publisher(
            String, f'{self._prefix}/conavgpt/assignment', latched)
        self._image_pub = self.create_publisher(
            Image, f'{self._prefix}/conavgpt/map_image', 1)
        self._table_pub = self.create_publisher(
            String, f'{self._prefix}/conavgpt/round_table', latched)

        self.get_logger().info(
            f'conavgpt_assigner starting (VLM-Assign arm) | leader={self._robot_name} '
            f'| model={self._model_path} | prompt_mode={self._prompt_mode} '
            f'| map_px={self._map_px} | results_dir={self._results_dir or "(off)"}')

        # Load off the ROS thread so requests are answered (with a fallback)
        # while the weights are still coming up.
        threading.Thread(target=self._load_model, daemon=True).start()
        # Inference off the executor thread: model.chat() blocks for seconds and
        # would otherwise stall the subscription that feeds it.
        threading.Thread(target=self._worker, daemon=True).start()

    # ── model ────────────────────────────────────────────────────────────────

    def _load_model(self):
        try:
            if not _TORCH_OK:
                raise RuntimeError(
                    'torch/transformers are not importable — run this node with '
                    '/opt/lvlm-venv/bin/python')
            try:
                import bitsandbytes as _bnb  # noqa: F401
                import accelerate as _acc    # noqa: F401
                deps_ok = f'bitsandbytes={_bnb.__version__} accelerate={_acc.__version__}'
            except Exception as _de:          # noqa: BLE001
                deps_ok = f'MISSING ({_de}) — rebuild/push the robot-desktop image'
            self.get_logger().info(
                f'Loading InternVL3 ({self._model_path}) | deps: {deps_ok} | '
                f'cuda_available={torch.cuda.is_available()} '
                f'n_gpus={torch.cuda.device_count()}...')
            device_map = split_model(self._model_path)
            quant_config = BitsAndBytesConfig(load_in_8bit=True)
            self.model = AutoModel.from_pretrained(
                self._model_path,
                torch_dtype=torch.bfloat16,
                quantization_config=quant_config,
                low_cpu_mem_usage=True,
                trust_remote_code=True,
                device_map=device_map).eval()
            self.tokenizer = AutoTokenizer.from_pretrained(
                self._model_path, trust_remote_code=True, use_fast=False)
            self._model_ready = True
            self.get_logger().info('InternVL3 loaded — assigner live')
        except Exception:                        # noqa: BLE001
            # Never fatal: every round then answers fallback:true and the team
            # runs as nearest-frontier, which is a result rather than a hang.
            import traceback
            self.get_logger().error(
                'InternVL3 load FAILED — every round will fall back to '
                'nearest-frontier:\n' + traceback.format_exc())

    # ── request intake ───────────────────────────────────────────────────────

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _request_cb(self, msg: 'String'):
        try:
            req = json.loads(msg.data)
            if not isinstance(req, dict):
                raise ValueError('payload is not a JSON object')
        except Exception as e:                   # noqa: BLE001
            self.get_logger().warn(f'assign_request unreadable: {e}')
            return
        with self._cv:
            if self._pending is not None:
                dropped = self._pending[0].get('round')
                self._superseded.append(dropped)
                self.get_logger().warn(
                    f'round {dropped} superseded by round {req.get("round")} '
                    'before it ran (latest-wins)')
            self._pending = (req, self._now())
            self._cv.notify()

    def _worker(self):
        while not self._stop:
            with self._cv:
                while self._pending is None and not self._stop:
                    self._cv.wait(0.5)
                if self._stop:
                    return
                req, arrival = self._pending
                superseded = list(self._superseded)
                self._superseded.clear()
                self._pending = None
            try:
                self._handle_request(req, arrival, superseded)
            except Exception:                    # noqa: BLE001
                # The worker is the only thread that answers requests; letting
                # it die would silently strand the whole team.
                import traceback
                self.get_logger().error(
                    'round handling raised — answering nothing this round:\n'
                    + traceback.format_exc())

    # ── one round ────────────────────────────────────────────────────────────

    def _handle_request(self, req, arrival, superseded):
        t_wall = time.monotonic()
        regions = parse_regions(req)
        robots = parse_robots(req)
        region_ids = [g['id'] for g in regions]
        robot_ids = [r['id'] for r in robots]

        # Staleness is measured against our own clock on both sides: raven_nav's
        # ts may be on a different clock (wall vs sim), and a cross-clock
        # subtraction would make every round look stale and fall back forever.
        # The skew is recorded instead, so a mismatch is visible in the results.
        age = self._now() - arrival
        req_ts = _as_float(req.get('ts'))
        ts_skew = round(self._now() - req_ts, 3) if req_ts is not None else None

        raw, image, reason = '', None, ''
        render_err = ''
        assignments, info = {}, {'method': 'none', 'dropped': [], 'error': ''}

        if self._request_timeout_s > 0.0 and age > self._request_timeout_s:
            reason = (f'request is {age:.1f}s old (> request_timeout_s='
                      f'{self._request_timeout_s:.0f}s)')
        elif not region_ids:
            reason = 'no candidate regions in the request'
        elif not robot_ids:
            reason = 'no robots in the request'
        elif not self._model_ready:
            reason = 'InternVL3 not loaded yet'
        else:
            if self._prompt_mode == 'image+text':
                try:
                    image = render_bev(req, self._map_px)
                except Exception as e:           # noqa: BLE001
                    render_err = f'{type(e).__name__}: {e}'
                    self.get_logger().warn(
                        f'BEV render failed ({render_err}) — text-only prompt')
            prompt = build_prompt(req, with_image=image is not None)
            try:
                raw = self._chat(prompt, image)
            except Exception as e:               # noqa: BLE001
                # CUDA OOM lands here; drop the cache so the next round has a
                # chance instead of failing forever.
                reason = f'inference failed: {type(e).__name__}: {e}'
                self.get_logger().error(reason)
                self._free_cuda()
            else:
                assignments, info = parse_assignments(raw, region_ids, robot_ids)
                if info.get('dropped'):
                    self.get_logger().warn(
                        'dropped model entries: ' + '; '.join(info['dropped']))
                if not assignments:
                    reason = ('unusable model response: '
                              + (info.get('error') or 'no assignments'))

        latency = time.monotonic() - t_wall
        fallback = not assignments
        assignment = build_assignment(
            req, assignments, self._model_path, latency, raw,
            fallback=fallback, reason=reason, ts=self._now())

        self._publish(assignment, image, req, info, superseded)
        self._n_rounds += 1
        self._n_fallbacks += int(fallback)
        self.get_logger().info(
            f'round {req.get("round")} | {len(assignments)}/{len(robot_ids)} '
            f'robots assigned | {latency:.2f}s'
            + (f' | FALLBACK: {reason}' if fallback else '')
            + f' | fallbacks {self._n_fallbacks}/{self._n_rounds}')
        self._maybe_dump_round(req, assignment, info, superseded, render_err,
                               image is not None, age, ts_skew)

    def _chat(self, prompt, image):
        pixel_values = None
        if image is not None:
            pil = PIL_Image.fromarray(np.asarray(image, dtype=np.uint8))
            pixel_values = load_image(
                pil, max_num=BEV_TILES).to(torch.bfloat16).cuda()
        return str(self.model.chat(
            self.tokenizer, pixel_values, prompt, self.generation_config))

    def _free_cuda(self):
        try:
            if _TORCH_OK and torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:                        # noqa: BLE001
            pass

    def _publish(self, assignment, image, req, info, superseded):
        try:
            self._assign_pub.publish(String(data=json.dumps(assignment)))
        except Exception as e:                   # noqa: BLE001
            self.get_logger().error(f'assignment publish failed: {e}')
        if image is not None:
            try:
                msg = self._bridge.cv2_to_imgmsg(
                    np.ascontiguousarray(image), encoding='rgb8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'global_enu'
                self._image_pub.publish(msg)
            except Exception as e:               # noqa: BLE001
                self.get_logger().warn(f'map_image publish failed: {e}')
        try:
            self._table_pub.publish(String(
                data=build_round_table(req, assignment, info, superseded)))
        except Exception as e:                   # noqa: BLE001
            self.get_logger().warn(f'round_table publish failed: {e}')

    # ── results dump ─────────────────────────────────────────────────────────

    def _maybe_dump_round(self, req, assignment, info, superseded, render_err,
                          image_used, age, ts_skew=None):
        if not self._results_dir:
            return
        record = {
            'ts': assignment['ts'],
            'round': req.get('round'),
            'leader': self._robot_name,
            'model': self._model_path,
            'prompt_mode': self._prompt_mode,
            'image_used': bool(image_used),
            'map_px': self._map_px,
            'render_error': render_err,
            'request_age_s': round(float(age), 3),
            'request_ts_skew_s': ts_skew,
            'latency_s': assignment['latency_s'],
            'fallback': assignment['fallback'],
            'reason': assignment['reason'],
            'parse': info,
            'superseded_rounds': list(superseded),
            'request': req,
            'assignment': assignment,
            'raw': assignment['raw'],
        }
        self._write_round(record)

    def _write_round(self, record):
        """Append one JSON line to <results_dir>/conavgpt_rounds.jsonl.

        Append-only and flushed per round: the mission runner ends a run by
        killing this process, so anything buffered is anything lost."""
        path = os.path.join(self._results_dir, 'conavgpt_rounds.jsonl')
        try:
            os.makedirs(self._results_dir, exist_ok=True)
            with open(path, 'a') as f:
                f.write(json.dumps(record, default=str) + '\n')
                f.flush()
        except OSError as e:
            self.get_logger().error(f'[results] round append failed: {e}')


def main(args=None):
    if not _ROS_OK:
        raise SystemExit(
            'conavgpt_assigner_node needs rclpy/cv_bridge on the path — run it '
            'as /opt/lvlm-venv/bin/python -m '
            'conavgpt_baseline.conavgpt_assigner_node')
    rclpy.init(args=args)
    node = CoNavGPTAssigner()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # semantic_search_task ends a run by SIGTERMing this node's process group
    # (5 s grace, then SIGKILL). The default disposition would skip the finally
    # block below.
    def _on_term(_sig, _frm):
        node._stop = True
        with node._cv:
            node._cv.notify_all()
        raise KeyboardInterrupt

    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(sig, _on_term)
        except ValueError:
            pass        # not on the main thread — rounds are already flushed

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        try:
            with node._cv:
                node._cv.notify_all()
        except Exception:                        # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
