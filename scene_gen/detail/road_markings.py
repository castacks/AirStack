"""
road_markings.py — crosswalks, stop bars, turn-lane flares, parking bays,
no-parking hatching and bike lanes.

Additive counterpart to the lane-marking pass inside
`scene_generator.apply_ground_planes`, which draws yellow centre lines and white
lane dashes but stops there, leaving intersections as bare asphalt. Called by
`generate_scene.py` after `apply_ground_planes`; nothing in the generator is
edited.

Every marking here is a flat quad, so this reuses `_make_dash_mesh` rather than
introducing new geometry code — same primitive the lane dashes already use.

WHERE A MARKING IS ALLOWED TO GO
--------------------------------
The corridor rect is the road's MAXIMUM extent, not its kerb line. `city_layout`
reserves a parking strip either side of every carriageway and then gives back
the ones its per-corridor policy doesn't park on, so a street reads as
``none`` / ``lo`` / ``hi`` / ``both`` and the pavement covers the rest. Painting
against the rect would put crosswalk bars and bay ticks on that pavement.

So markings are positioned against the kerb the corridor's policy implies
(`_kerb`), and a strip that exists is painted along its whole length:

* white bay ticks where parking is legal,
* yellow diagonal hatching where it is not — the daylighting keep-out either
  side of a junction, and the clear zone around a fire hydrant,
* a white pocket line and taper where the approach borrows the strip for a turn
  lane, since there the strip is a running lane rather than a prohibition.

A corridor carrying no policy (the built-in subdivider, or v1 scenes) falls back
to the rect on both sides, which is the behaviour it had before.

BIKE LANES
----------
A bike lane cannot be added to a corridor by widening it — the rect is fixed and
the abutting blocks are already built against it. It has to come out of the
carriageway, which is what a real road diet does. `plan_bike_lanes` therefore
gives the outermost *travel* lane on each side over to bikes, so a 4-lane
arterial becomes 2 travel lanes plus 2 bike lanes.

Taking exactly one `lane_width_m` per side is what makes this fit the geometry
`apply_ground_planes` has already drawn. Its outermost white lane divider sits
at `carriage_edge ± lane_w` for any `n_lanes >= 4`, which is precisely the new
lane's inner edge, and because the same width is taken off both sides the yellow
centre line stays on the centre of what is left. Neither has to be masked or
moved. Corridors under 4 lanes are skipped: there is no lane to give up.

Cross-section, kerb outward to centreline, is then the textbook one — the
parking lane (where the policy parks) stays outside the carriage edge, so the
bike lane lands between parked cars and moving ones and never on the pavement:

    kerb │ parking │ bike lane │ buffer │ travel lanes │ centre line

Longitudinally the lane is cut by every junction it meets. NOTHING is painted
from the junction box out to the far edge of the stop bar — a real crossing is
bare asphalt and a crosswalk, so the lane stops at the stop line and resumes on
the far side. Beyond that furniture it is dotted across the mixing zone where
right-turning traffic crosses it, and only the stretch past that is solid and
eligible for delineator posts. Green marks a MINORITY of those mixing zones,
which is what makes it read as a conflict point rather than as lane colour. Posts are planned here rather than
in `city_detail` so that the paint and the props cannot disagree about which
stretches are protected; `city_detail` consumes `plan_bike_lanes()["posts"]`.
"""

import math
import random

from scene_generator import _make_dash_mesh, _make_plane_mesh, _stage

# MARKING HEIGHTS. `apply_ground_planes` is tracked and draws its yellow centre
# lines and white lane dashes at a HARDCODED 0.020, so the two heights here are
# not a style choice — they are which side of that number a marking has to be
# on.
#
# HIGH is for markings that genuinely cross a lane dash and must read on top of
# it: the crossing band, its asphalt mask (whose whole job is to hide the centre
# line inside the band, since MUTCD carries no centre line through a junction),
# the stop bar, and the one bike lane line that lands on a lane divider by
# design. Anything at or below 0.020 there loses to the generator's paint.
#
# LOW is for everything drawn on bare asphalt — kerb-strip work and the inside
# of a bike lane. Real thermoplastic is 2-3 mm proud, and at 0.021 the paint
# visibly floats at close range. 5 mm reads flush while still clearing the road
# mesh at z=0 by enough not to stipple.
_Z_GEN_DASH = 0.020             # the generator's hardcoded height, not ours
_Z = 0.021                      # HIGH; overridden by markings.z_high_m
_Z_LOW = 0.005                  # overridden by markings.z_low_m
_WHITE = (0.92, 0.92, 0.92)
# Diagonal hatching in a no-parking zone is WHITE, not yellow — MUTCD 11th ed.
# §3B.25 ¶09 is a Standard, and ¶07 is what permits the device in a no-parking
# zone at all (both new in the 11th; the 2009 text had neither). Yellow is
# reserved by ¶08 for a neutral area separating OPPOSING directions and for the
# left shoulder of a divided road, so yellow at a right-hand kerb miscoded the
# marking as a median divider. Slightly off pure white so it reads as pavement
# paint rather than as the crosswalk bars.
_HATCH = (0.88, 0.88, 0.86)


def _run_span(c):
    """``(lo, hi)`` along the corridor's length."""
    if c.get("dir") == "ns":
        return float(c["y0"]), float(c["y1"])
    return float(c["x0"]), float(c["x1"])


def _cross_span(c):
    """``(lo, hi)`` across the corridor's width — the rect, i.e. the maximum."""
    if c.get("dir") == "ns":
        return float(c["x0"]), float(c["x1"])
    return float(c["y0"]), float(c["y1"])


def _carriage(c):
    """``(lo, hi)`` of the travelled way, excluding both kerb strips."""
    car = c.get("carriage")
    if not car or len(car) != 2:
        return _cross_span(c)
    return float(car[0]), float(car[1])


def _has_strip(c, side) -> bool:
    """True when *side* keeps its kerb strip as asphalt for parking."""
    policy = c.get("parking")
    if policy is None:                  # no profile: the rect edge is the kerb
        return True
    return policy == "both" or policy == side


def _kerb(c, side) -> float:
    """Cross coordinate of the kerb on *side*.

    The rect edge where the strip is asphalt, the carriage edge where the
    abutting blocks were built out over it.
    """
    lo, hi = _cross_span(c)
    if _has_strip(c, side):
        return lo if side == "lo" else hi
    c_lo, c_hi = _carriage(c)
    return c_lo if side == "lo" else c_hi


def _pt(c, s, cross):
    """Map (run, cross) back to world (x, y) for this corridor's direction."""
    return (cross, s) if c.get("dir") == "ns" else (s, cross)


def _merge(spans):
    """Coalesce a list of intervals into disjoint, sorted ones."""
    out = []
    for a0, a1 in sorted(spans):
        if out and a0 <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], a1))
        else:
            out.append((a0, a1))
    return out


def _subtract(span, cuts):
    """``span`` minus a list of (possibly overlapping, unsorted) intervals."""
    out = [span]
    for c0, c1 in cuts:
        nxt = []
        for a0, a1 in out:
            if c1 <= a0 or c0 >= a1:
                nxt.append((a0, a1))
                continue
            if c0 > a0:
                nxt.append((a0, c0))
            if c1 < a1:
                nxt.append((c1, a1))
        out = nxt
    return out


def _clip(span, keep):
    """The parts of *span* covered by *keep* (assumed disjoint and sorted)."""
    a0, a1 = span
    out = []
    for b0, b1 in keep:
        lo, hi = max(a0, b0), min(a1, b1)
        if hi - lo > 1e-6:
            out.append((lo, hi))
    return out


# Corridors that meet are trimmed flush against each other by the BSP rather
# than crossing, so an NS road running y=-193..193 meets the y=-200..-193 border
# road at a shared edge and their rects never overlap. Adjacency therefore has
# to be tested with a tolerance, not as intersection.
_TOUCH_M = 0.5


def _touch(a0, a1, b0, b1) -> bool:
    """True when two 1-D spans overlap or merely abut within _TOUCH_M."""
    return min(a1, b1) - max(a0, b0) >= -_TOUCH_M


def _merge_junctions(raw):
    """Fold overlapping junction boxes into one.

    The BSP can split twice at nearly the same coordinate in two subtrees, so
    two corridor rects overlap and ONE physical intersection is reported as
    several junctions a few metres apart. Every marking here is positioned
    against a box, so unmerged twins paint a second crosswalk and a second stop
    bar overlapping the first — which is exactly the doubled, intersecting lines
    seen at some corners. `city_detail` already merges for the same reason;
    this is the same rule applied to the paint so the two agree.

    The widest pair of legs wins, so the crossing spans the whole junction
    rather than the narrower of two coincident roads.
    """
    merged = []
    for ns_c, ew_c, box in raw:
        for i, (m_ns, m_ew, m_box) in enumerate(merged):
            if (box[2] <= m_box[0] or box[0] >= m_box[2]
                    or box[3] <= m_box[1] or box[1] >= m_box[3]):
                continue
            wider_ns = ns_c if (int(ns_c.get("n_lanes", 2))
                                > int(m_ns.get("n_lanes", 2))) else m_ns
            wider_ew = ew_c if (int(ew_c.get("n_lanes", 2))
                                > int(m_ew.get("n_lanes", 2))) else m_ew
            merged[i] = (wider_ns, wider_ew,
                         (min(box[0], m_box[0]), min(box[1], m_box[1]),
                          max(box[2], m_box[2]), max(box[3], m_box[3])))
            break
        else:
            merged.append((ns_c, ew_c, box))
    return merged


def _intersections(corridors):
    """Yield ``(ns, ew, box)`` for every NS x EW junction.

    *box* is the junction rect: the NS road's width in x by the EW road's width
    in y. That is the square of asphalt the two roads share, and every marking
    is positioned relative to it — even when the corridor rects only touch.
    """
    ns = [c for c in corridors if c.get("dir") == "ns"]
    ew = [c for c in corridors if c.get("dir") != "ns"]
    for a in ns:
        ax0, ax1 = sorted((float(a["x0"]), float(a["x1"])))
        ay0, ay1 = sorted((float(a["y0"]), float(a["y1"])))
        for b in ew:
            bx0, bx1 = sorted((float(b["x0"]), float(b["x1"])))
            by0, by1 = sorted((float(b["y0"]), float(b["y1"])))
            # The NS road has to reach across the EW road's line and vice versa.
            if _touch(ax0, ax1, bx0, bx1) and _touch(ay0, ay1, by0, by1):
                yield a, b, (ax0, by0, ax1, by1)


def _fits(c, s, length) -> bool:
    """True when a band of *length*, centred at run position *s*, is wholly on
    this corridor.

    A T-junction has three approaches, not four, and at a region edge the fourth
    would sit on a block. Testing the whole band rather than probing its centre
    against the road rects is what keeps the missing approach from hanging its
    bars a fraction of a metre off the end of the road.
    """
    lo, hi = _run_span(c)
    return lo <= s - length / 2.0 and s + length / 2.0 <= hi


def _kerb_zones(corridors, points, half_m, reach_m):
    """``{(id(corridor), side): [(s0, s1), ...]}`` of kerb reserved for *points*.

    Serves hydrants and bus stops alike: both stand in the furnishing zone, a
    metre or so back from the kerb on the pavement, so each is matched to the
    nearest kerb of any corridor it is within *reach_m* of, and reserves
    *half_m* of that kerb either side of itself. What differs is only what the
    caller then does with the span — a hydrant clear zone gets hatched, a bus
    zone deliberately does not.

    The hydrant figure is UVC (Millennium Ed. 2000) 11-1003(a)(2)(B), "within
    15 feet of a fire hydrant" = 4.6 m each side; the bus figure is TCRP
    Report 19 Figure 3. See the preset. NOTE: the hatching of the hydrant zone
    is almost certainly the wrong DEVICE — MUTCD has no carriageway marking for
    hydrant clearance at all — but that is a live question, not settled, so it
    is left as it is here deliberately.
    """
    out = {}
    for c in corridors:
        if float(c.get("park_w", 0.0)) <= 0.0:
            continue
        ns = c.get("dir") == "ns"
        run_lo, run_hi = _run_span(c)
        lo, hi = _cross_span(c)
        for px, py in points:
            s, cross = (py, px) if ns else (px, py)
            if not (run_lo - half_m <= s <= run_hi + half_m):
                continue
            if cross < lo:
                side, d = "lo", lo - cross
            elif cross > hi:
                side, d = "hi", cross - hi
            else:
                side = "lo" if cross - lo <= hi - cross else "hi"
                d = 0.0
            if d > reach_m or not _has_strip(c, side):
                continue
            out.setdefault((id(c), side), []).append((s - half_m, s + half_m))
    return out


def _ladder(stage, prefix, counter, band_c, band_half_depth, span_lo, span_hi,
            along_x, bar_w, bar_gap, ssf, z):
    """Draw one continental ("ladder") crosswalk band.

    *along_x* True means vehicles travel along X, so the bars run along X too
    and repeat across Y — bars always lie parallel to the direction of travel,
    which is what makes a ladder crossing read as one.
    """
    pitch = bar_w + bar_gap
    span = span_hi - span_lo
    n = max(1, int(span / pitch))
    step = span / n
    for k in range(n):
        t = span_lo + (k + 0.5) * step
        cx, cy = (band_c, t) if along_x else (t, band_c)
        _make_dash_mesh(stage, f"{prefix}/xwalk_{counter[0]}",
                        cx, cy, z,
                        band_half_depth * 2.0, bar_w,
                        0.0 if along_x else 90.0, ssf, _WHITE)
        counter[0] += 1


def _line(stage, path, p0, p1, width, ssf, color=_WHITE, z=_Z):
    """One flat quad spanning p0->p1, so tapers and hatching can run diagonally."""
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return
    _make_dash_mesh(stage, path,
                    (p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0, z,
                    length, width, math.degrees(math.atan2(dy, dx)),
                    ssf, color)


# ----------------------------------------------------------- bike lanes ------
# MUTCD 11th ed. (2023) governs the paint — Chapter 9E, which is where the
# 2009 edition's 9C.04 ended up, plus 3A.04 for line patterns and 3H.06 for
# green. MUTCD specifies no bike lane WIDTH at all (1A.02 para 02: geometry is
# not a traffic control device), so the cross-section comes from NACTO's Urban
# Bikeway Design Guide and AASHTO. Clause numbers are on the knobs in the
# preset.

# MUTCD IA-14 green conflict-area pavement. Spans the rideable width only, so
# it never reaches the lane divider and can sit on the low plane — just under
# the lane's own lines, which have to read on top of it.
_GREEN = (0.10, 0.52, 0.28)


def _travel_sign(c, side) -> float:
    """Direction of travel along the run axis for the *side* kerb lane.

    Right-hand traffic: facing +x the right-hand kerb is -y, facing +y it is
    +x — the same rule the stop bars use. A bike lane is a one-way facility, so
    this is what the lane arrows point along.
    """
    if c.get("dir") == "ns":
        return 1.0 if side == "hi" else -1.0
    return 1.0 if side == "lo" else -1.0


def _bike_cross(c, side, lane_w, ride_w):
    """``(ride_lo, ride_hi, buf_lo, buf_hi)`` cross spans for one bike lane.

    Rideable width hugs the kerb, buffer inboard of it against traffic.
    """
    c0, c1 = _carriage(c)
    if side == "lo":
        return c0, c0 + ride_w, c0 + ride_w, c0 + lane_w
    return c1 - ride_w, c1, c1 - lane_w, c1 - ride_w


def _post_stations(s0, s1, near, far, taper, setback):
    """Delineator run positions over one protected stretch.

    Spacing eases from *near* at either end to *far* in the middle. NACTO
    (Separating Protected Bike Lanes) puts delineators every 10-40 ft and
    "closer together ... on the approach and departure of intersections"; a
    protected stretch is bounded by exactly those, so distance-to-end is the
    signal and no separate intersection lookup is needed. NACTO gives no number
    for the tightening, only the direction, so *near* is engineering judgment
    pinned to the bottom of its own 10-40 ft band.
    """
    a, b = s0 + setback, s1 - setback
    if b - a < near:
        return []
    out, s = [], a
    while s <= b + 1e-6:
        out.append(s)
        d = min(s - a, b - s)
        step = near if taper <= 0 else near + (far - near) * min(1.0, d / taper)
        s += max(0.5, step)
    return out


def plan_bike_lanes(config: dict, layout: dict) -> list:
    """Where bike lanes go, as pure geometry — no stage, no USD.

    Called twice per bake, once by `apply` for the paint and once by
    `city_detail` for the posts, so it must stay deterministic in
    ``config["seed"]`` alone. Returns one dict per (corridor, side):

        dir, side          the corridor's axis and which kerb
        ride, buf          cross spans of the rideable lane and its buffer
        solid, mix, clear  run spans: plain lane, weave zone, and the junction
                           itself — *clear* is painted with NOTHING
        green              the subset of *mix* that gets conflict-area colour
        protected          the subset of *solid* that gets delineators
        posts              world (x, y) for every delineator on this side
    """
    cfg = ((_stage(config, "layout").get("roads") or {}).get("markings") or {}).get("bike_lanes")
    if not cfg or not cfg.get("enabled", True):
        return []

    roads = _stage(config, "layout").get("roads") or {}
    lane_w = float(roads.get("lane_width_m", 3.5))
    # A bike lane is never wider than the travel lane it replaces, and always
    # leaves something for the buffer.
    ride_w = min(float(cfg.get("ride_width_m", 2.0)), lane_w - 0.3)
    min_lanes = max(4, int(cfg.get("min_lanes", 4)))
    chance = float(cfg.get("corridor_chance", 0.55))
    prot_chance = float(cfg.get("protected_chance", 0.35))
    prot_min = float(cfg.get("protected_min_m", 30.0))
    prot_seg = float(cfg.get("protected_seg_m", 55.0))
    mix_m = float(cfg.get("mix_zone_m", 22.0))
    green_share = (float(cfg.get("green_chance", 0.25))
                   if cfg.get("green_conflict", True) else 0.0)
    # Nothing is painted from the junction box out to the far edge of the stop
    # bar. Derived from the crossing geometry rather than given its own knob,
    # because it IS that geometry: setback, then the crossing band, then the
    # gap, then the bar. A real junction is bare asphalt plus a crossing, not a
    # lattice of dotted lines, and the lane has to stop at the stop line.
    mkc = (_stage(config, "layout").get("roads") or {}).get("markings") or {}
    clear_m = (float(mkc.get("setback_m", 1.2))
               + float(mkc.get("crosswalk_depth_m", 2.4))
               + float(mkc.get("stop_bar_advance_m", 1.2))
               + float(mkc.get("stop_bar_width_m", 0.5)))
    sp_near = float(cfg.get("post_spacing_near_m", 3.0))
    sp_far = float(cfg.get("post_spacing_far_m", 12.0))
    taper = float(cfg.get("post_taper_m", 20.0))
    setback = float(cfg.get("post_setback_m", 2.0))

    corridors = layout.get("road_corridors") or []
    boxes = {}
    for ns_c, ew_c, (bx0, by0, bx1, by1) in _intersections(corridors):
        boxes.setdefault(id(ns_c), []).append((by0, by1))
        boxes.setdefault(id(ew_c), []).append((bx0, bx1))

    # Two streams, so that retuning protection does not reshuffle which
    # corridors carry a lane at all — one shared stream makes every knob
    # change the whole network.
    seed = int(config.get("seed", 0))
    rng_lane = random.Random(seed + 5273)
    rng_prot = random.Random(seed + 5279)
    rng_green = random.Random(seed + 5281)

    # Arterials only. Below 4 lanes there is no outer lane to give up without
    # either stranding a direction or moving the centre line.
    elig = [c for c in corridors
            if int(c.get("n_lanes", 2)) >= min_lanes
            and (_carriage(c)[1] - _carriage(c)[0]) - 2.0 * lane_w
            >= 2.0 * lane_w - 1e-6]
    # Both shares are taken as a COUNT of a shuffle rather than as a coin flip
    # per candidate. A scene has a couple of dozen arterials, and at that sample
    # size a per-corridor flip misses its own mean badly — measured, 0.60 landed
    # on 4 of 15. A count makes the knob mean what it says at any seed.
    order = list(range(len(elig)))
    rng_lane.shuffle(order)
    chosen = sorted(order[:int(round(chance * len(elig)))])

    spans, segs = {}, []
    for i in chosen:
        c = elig[i]
        run = _run_span(c)
        box = _merge(boxes.get(id(c), []))
        # The weave zone reaches mix_m back from every junction box; what is
        # left over in the middle of the block is the plain lane.
        wide = _merge([(b0 - mix_m, b1 + mix_m) for b0, b1 in box])
        # The weave zone starts where the junction's own markings stop, not at
        # the box edge, so the lane resumes cleanly past the stop bar.
        near = _merge([(b0 - clear_m, b1 + clear_m) for b0, b1 in box])
        mix = _merge([m for w in wide for m in _subtract(w, near)
                      if m[1] - m[0] > 1e-6])
        solid = _subtract(run, wide)
        spans[i] = (run, near, mix, solid)
        # Protection is decided per stretch, not per side: a real street
        # protects both kerbs of a block or neither. The stretch is chopped
        # first because the BSP leaves exactly one solid run per corridor, so
        # deciding per run would make protection an all-or-nothing property of
        # the whole block and leave the lane uniform along its length.
        for s0, s1 in solid:
            if s1 - s0 < prot_min:
                continue
            n = max(1, int(round((s1 - s0) / prot_seg)))
            step = (s1 - s0) / n
            segs += [(i, s0 + k * step, s0 + (k + 1) * step)
                     for k in range(n) if step >= prot_min]

    take = list(range(len(segs)))
    rng_prot.shuffle(take)
    prot_of = {}
    for j in take[:int(round(prot_chance * len(segs)))]:
        i, a, b = segs[j]
        prot_of.setdefault(i, []).append((a, b))

    out = []
    for i in chosen:
        c = elig[i]
        run, near, mix, solid = spans[i]
        prot = _merge(prot_of.get(i, []))
        for side in ("lo", "hi"):
            r0, r1, b0, b1 = _bike_cross(c, side, lane_w, ride_w)
            post_c = (b0 + b1) / 2.0
            posts = [_pt(c, s, post_c)
                     for p0, p1 in prot
                     for s in _post_stations(p0, p1, sp_near, sp_far,
                                             taper, setback)]
            out.append({
                "corridor": c, "dir": c.get("dir"), "side": side,
                "ride": (r0, r1), "buf": (b0, b1),
                "parked": _has_strip(c, side),
                "sign": _travel_sign(c, side),
                "solid": solid, "mix": _clip(run, mix),
                "clear": _clip(run, near), "green": [],
                "protected": prot, "posts": posts,
            })

    # Green marks a CONFLICT POINT, not the run of the lane — MUTCD 3H.06 para
    # 02(C) is about weave areas specifically. Taken as an exact count off a
    # shuffled list of every weave zone in the scene, the same way the arterial
    # and protected shares are, so the knob means what it says at any seed.
    sites = [(k, sp) for k, ln in enumerate(out) for sp in ln["mix"]]
    pick = list(range(len(sites)))
    rng_green.shuffle(pick)
    for j in pick[:int(round(green_share * len(sites)))]:
        k, sp = sites[j]
        out[k]["green"].append(sp)
    return out


def _dashed(stage, gnd, counter, c, cross, span, width, seg, gap, ssf, z):
    """A broken line down the run axis at a fixed cross offset."""
    s0, s1 = span
    n = 0
    s = s0
    while s + seg <= s1 + 1e-6:
        _line(stage, f"{gnd}/bike_{counter[0]}",
              _pt(c, s, cross), _pt(c, s + seg, cross), width, ssf, z=z)
        counter[0] += 1
        n += 1
        s += seg + gap
    return n


def _bike_arrow(stage, gnd, counter, c, cross, s, sign, length, width, ssf, z):
    """Directional arrow — shaft plus two barbs, three quads.

    MUTCD 9E.01 para 02 (Standard) requires longitudinal lines AND a bicycle
    symbol or BIKE LANE word marking; para 04 puts an arrow downstream of it.
    Only the arrow is drawn here. The bicycle symbol is a pictogram and cannot
    be built from the flat quads this module is limited to, so the lane is
    knowingly short of that Standard — see the preset.
    """
    tip = s + sign * length / 2.0
    tail = s - sign * length / 2.0
    _line(stage, f"{gnd}/bike_{counter[0]}",
          _pt(c, tail, cross), _pt(c, tip, cross), width * 1.6, ssf, z=z)
    counter[0] += 1
    barb = length * 0.38
    # The three strokes meet at the tip and therefore overlap there. Same
    # colour, so the overlap is invisible either way, but coplanar duplicates
    # are still coplanar duplicates — a tenth of a millimetre apart each and
    # the arrowhead cannot z-fight.
    for k, lat in enumerate((-1.0, 1.0)):
        _line(stage, f"{gnd}/bike_{counter[0]}",
              _pt(c, tip, cross),
              _pt(c, tip - sign * barb * 0.8, cross + lat * barb * 0.55),
              width * 1.6, ssf, z=z + 0.0001 * (k + 1))
        counter[0] += 1


def _draw_bike_lanes(stage, gnd, plan, cfg, ssf, counter, z_low, z_high,
                     transverse=None):
    """Paint one planned lane set. Returns ``(lanes, metres, dashes, arrows)``.

    *transverse* maps a corridor to the run spans of the markings that cross
    it — the crossing bands and the stop bars. The lane's paint is broken over
    those: a crossing is its own marking and MUTCD does not carry a
    longitudinal line through one, and two coplanar quads that overlap stipple.
    """
    line_w = float(cfg.get("line_width_m", 0.15))
    seg = float(cfg.get("dash_len_m", 0.6))
    gap = float(cfg.get("dash_gap_m", 0.6))
    # 0 switches the buffer chevrons off. MUTCD 9E.06 p14 makes them a Standard
    # above a 3 ft buffer, so this is a knowing deviation, not a correction —
    # every diagonal marking in the scene reads wrong here and they are all off
    # together. Narrowing the buffer under 3 ft would drop the requirement
    # legitimately, but that costs rideable width.
    hatch_pitch = float(cfg.get("buffer_hatch_pitch_m", 4.0))
    hatch_w = float(cfg.get("buffer_hatch_width_m", 0.12))
    if hatch_pitch <= 0.0 or hatch_w <= 0.0:
        hatch_pitch = 0.0
    sym_gap = float(cfg.get("symbol_spacing_m", 60.0))
    sym_len = float(cfg.get("symbol_len_m", 1.8))
    green = bool(cfg.get("green_conflict", True))

    transverse = transverse or {}
    metres = n_dash = n_sym = 0
    for lane in plan:
        c = lane["corridor"]
        # Break every painted span over the crossings and stop bars this
        # corridor carries. The lane still EXISTS there — the metre counts
        # below stay on the planned spans — it is only the stripe that stops.
        cut = _merge(list(transverse.get(id(c), ())) + list(lane["clear"]))
        solid_p = [q for sp in lane["solid"] for q in _subtract(sp, cut)]
        mix_p = [q for sp in lane["mix"] for q in _subtract(sp, cut)]
        r0, r1 = lane["ride"]
        b0, b1 = lane["buf"]
        sign = lane["sign"]
        # Buffer edges. MUTCD 9E.06 para 03 (Standard) requires a solid white
        # line along BOTH edges of the buffer. The traffic-side one lands on
        # the divider apply_ground_planes drew, so the dashes underneath are
        # covered rather than masked — and
        # it is the one marking in this pass that has to take the HIGH plane to
        # win that overlap. Everything else here is on bare asphalt.
        traffic_edge = b1 if lane["side"] == "lo" else b0
        bike_edge = b0 if lane["side"] == "lo" else b1
        kerb_edge = r0 if lane["side"] == "lo" else r1

        metres += sum(s1 - s0 for s0, s1 in lane["solid"])
        for s0, s1 in solid_p:
            for cross, z in ((traffic_edge, z_high), (bike_edge, z_low)):
                _line(stage, f"{gnd}/bike_{counter[0]}",
                      _pt(c, s0, cross), _pt(c, s1, cross), line_w, ssf, z=z)
                counter[0] += 1
            # Against parked cars the bike lane needs its own kerb-side line;
            # against a kerb the kerb is the edge and MUTCD marks nothing.
            # NOTE: NACTO requires a 3 ft door-zone buffer on the parking side
            # as well, which this cross-section has no width left for.
            if lane["parked"]:
                _line(stage, f"{gnd}/bike_{counter[0]}",
                      _pt(c, s0, kerb_edge), _pt(c, s1, kerb_edge),
                      line_w, ssf, z=z_low)
                counter[0] += 1
            # Buffer hatching. MUTCD 9E.06 para 11 (Standard) has the
            # diagonals slant AWAY from traffic in the adjacent travel lane, so
            # each stroke starts at the traffic edge and moves toward the bike
            # lane as it runs downstream. Not coplanar with either line, so it
            # stays on the low plane.
            if hatch_pitch > 0.0:
                # The stroke occupies [s, s+lean] whichever way travel points;
                # only which END is on the traffic edge flips with `sign`.
                # Leaning it off `s` directly would hang it past the start of
                # the run on a corridor whose travel runs negative.
                lean = b1 - b0
                # Butt the stroke against the two boundary lines rather than
                # into them: ending on the line centre buries a corner of the
                # stroke inside a coplanar quad of the same colour. Costs ~5
                # degrees of lean, which stays inside NACTO's 30-45.
                sgn = 1.0 if bike_edge > traffic_edge else -1.0
                t_in = traffic_edge + sgn * line_w / 2.0
                b_in = bike_edge - sgn * line_w / 2.0
                s = s0
                while s + lean <= s1 + 1e-6:
                    a, b = (s, s + lean) if sign > 0 else (s + lean, s)
                    _line(stage, f"{gnd}/bike_{counter[0]}",
                          _pt(c, a, t_in), _pt(c, b, b_in),
                          hatch_w, ssf, z=z_low)
                    counter[0] += 1
                    s += hatch_pitch
            # A symbol immediately after the junction, then at interval.
            if sym_gap > 0.0:
                ride_c = (r0 + r1) / 2.0
                first = s0 + sym_len if sign > 0 else s1 - sym_len
                n = max(1, int((s1 - s0) / sym_gap))
                for k in range(n):
                    s = first + sign * k * sym_gap
                    if not (s0 + sym_len <= s <= s1 - sym_len):
                        continue
                    _bike_arrow(stage, gnd, counter, c, ride_c, s, sign,
                                sym_len, line_w, ssf, z_low)
                    n_sym += 1

        # Weave zone. MUTCD 9E.02 para 11 (Guidance): the line defining a bike
        # lane "should be dotted on approaches to intersections where turning
        # vehicles are permitted to cross the path of through-moving
        # bicyclists". Dotted, not dashed — 9E.02's broken lane line is a
        # different device. The buffer stops: there is nothing left to buffer.
        metres += sum(s1 - s0 for s0, s1 in lane["mix"])
        for span in mix_p:
            n_dash += _dashed(stage, gnd, counter, c, traffic_edge, span,
                              line_w, seg, gap, ssf, z_high)
        # NOTHING is drawn across the junction itself. MUTCD 9E.03 para 07
        # would allow a dotted extension here, but a real signalised crossing
        # is bare asphalt and a crosswalk, not a lattice of dotted lines, so
        # the lane simply stops at the stop bar and resumes on the far side.
        metres += sum(s1 - s0 for s0, s1 in lane["clear"])

        # MUTCD 3H.06 para 04 (Standard): green supplementing a DOTTED line
        # "shall match the pattern of the dotted lines, thus filling in only
        # the areas that are directly between a pair of dotted line segments".
        # So the conflict paint is itself dotted, on the pitch of the line it
        # accompanies — not the continuous slab it looks like from a distance.
        # Only the weave zones the planner picked, and only outside the
        # junction furniture.
        green_p = [q for sp in lane["green"] for q in _subtract(sp, cut)]
        if green:
            for spans, gseg, ggap in ((green_p, seg, gap),):
                for s0, s1 in spans:
                    s = s0
                    while s + gseg <= s1 + 1e-6:
                        p0 = _pt(c, s, min(r0, r1))
                        p1 = _pt(c, s + gseg, max(r0, r1))
                        _make_plane_mesh(
                            stage, f"{gnd}/bike_green_{counter[0]}",
                            min(p0[0], p1[0]), min(p0[1], p1[1]),
                            max(p0[0], p1[0]), max(p0[1], p1[1]),
                            z_low - 0.0005, 4.0, ssf, display_color=_GREEN)
                        counter[0] += 1
                        s += gseg + ggap

    return len(plan), metres, n_dash, n_sym


def apply(stage, config: dict, layout: dict, parent_path: str,
          scene_scale_factor: float = 1.0, hydrants=None,
          bus_stops=None) -> int:
    """Write markings under ``<parent_path>/ground``. Returns the quad count.

    *hydrants* and *bus_stops* are optional iterables of ``(x_m, y_m)``.
    They are placed by `city_detail`, which runs after this module, so the
    caller has to hand them over; without them the hydrant clear zones are the
    only markings that go missing.
    """
    cfg = (_stage(config, "layout").get("roads") or {}).get("markings") or {}
    if not cfg:
        return 0

    do_xwalk = bool(cfg.get("crosswalks", True))
    do_stop = bool(cfg.get("stop_bars", True))
    do_bays = bool(cfg.get("parking_bays", True))
    do_flare = bool(cfg.get("turn_lane_flares", True))
    do_hatch = bool(cfg.get("no_parking_hatch", True))
    bar_w = float(cfg.get("bar_width_m", 0.45))
    bar_gap = float(cfg.get("bar_gap_m", 0.45))
    depth = float(cfg.get("crosswalk_depth_m", 2.4))
    stop_w = float(cfg.get("stop_bar_width_m", 0.5))
    setback = float(cfg.get("setback_m", 1.2))
    # Clear gap between the crosswalk's near edge and the stop bar. MUTCD 3B.16
    # sets a 4 ft minimum; `setback_m` is the kerb-to-crosswalk gap and is a
    # different measurement, so it cannot serve here.
    stop_adv = float(cfg.get("stop_bar_advance_m", 1.2))
    bay_len = float(cfg.get("parking_bay_len_m", 6.5))
    bay_depth = float(cfg.get("parking_bay_depth_m", 2.2))
    tick_w = float(cfg.get("bay_tick_width_m", 0.12))
    # Daylighting: no parking within this of a junction, which is also the space
    # the approach borrows for a turn lane.
    no_park = float(cfg.get("no_parking_m", 7.5))
    flare_len = float(cfg.get("flare_m", 8.0))
    taper_len = float(cfg.get("flare_taper_m", 5.0))
    flare_chance = float(cfg.get("flare_chance", 0.55))
    line_w = float(cfg.get("lane_line_width_m", 0.15))
    hatch_pitch = float(cfg.get("hatch_pitch_m", 1.6))
    hatch_w = float(cfg.get("hatch_width_m", 0.15))
    hyd_clear = float(cfg.get("hydrant_clear_m", 4.6))
    hyd_reach = float(cfg.get("hydrant_reach_m", 4.0))
    # Half a bus zone: the placed stop is treated as its middle.
    bus_half = float(cfg.get("bus_zone_m", 27.0)) / 2.0
    bus_reach = float(cfg.get("bus_reach_m", 5.0))
    # Two marking planes — see the module header. z_high is clamped above the
    # generator's hardcoded 0.020 because a value below it silently loses the
    # crossing band and the centre-line mask to paint we cannot edit.
    z_low = float(cfg.get("z_low_m", _Z_LOW))
    z_high = max(float(cfg.get("z_high_m", _Z)), _Z_GEN_DASH + 0.0005)

    ssf = float(scene_scale_factor)
    gnd = f"{parent_path}/ground"
    counter = [0]
    n_x = n_s = n_b = n_f = n_h = n_skip = 0
    rng = random.Random(int(config.get("seed", 0)) + 9311)

    corridors = layout.get("road_corridors") or []
    hydrants = [(float(p[0]), float(p[1])) for p in (hydrants or ())]
    bus_stops = [(float(p[0]), float(p[1])) for p in (bus_stops or ())]

    # Junctions drive the crossings, the parking keep-outs and the flares, so
    # they are resolved once and indexed by corridor for the strip pass below.
    keepout, signed, xwalk, boxspan = {}, {}, {}, {}
    # Run spans of every marking that lies ACROSS a corridor. The bike lane
    # runs along one, so its paint is broken wherever it meets one of these.
    transverse = {}
    # The crossing masks have to be the SAME asphalt as the road or they read as
    # grey patches, so the material is taken off the base plane rather than
    # re-resolved: apply_ground_planes has already run and bound it.
    uv_asphalt = float((_stage(config, "layout").get("roads") or {}).get("asphalt_uv_scale_m", 4.0))
    asphalt_mat = ""
    _base = stage.GetPrimAtPath(f"{gnd}/asphalt_base")
    if _base and _base.IsValid():
        from pxr import UsdShade
        _m = UsdShade.MaterialBindingAPI(_base).ComputeBoundMaterial()[0]
        if _m and _m.GetPrim().IsValid():
            asphalt_mat = str(_m.GetPrim().GetPath())
    junctions = _merge_junctions(list(_intersections(corridors)))
    # Bays are held back far enough to leave the whole turn pocket clear, but
    # only the legally signed length gets painted — hatching the flare reserve
    # as well would carpet every approach in yellow.
    clear = max(no_park, flare_len + taper_len)
    for ns_c, ew_c, (bx0, by0, bx1, by1) in junctions:
        keepout.setdefault(id(ns_c), []).append((by0 - clear, by1 + clear))
        keepout.setdefault(id(ew_c), []).append((bx0 - clear, bx1 + clear))
        signed.setdefault(id(ns_c), []).append((by0 - no_park, by1 + no_park))
        signed.setdefault(id(ew_c), []).append((bx0 - no_park, bx1 + no_park))
        # The junction box itself. The daylighting span reaches no_parking_m
        # PAST the box on both sides, so it spans the box too — and there is no
        # kerb strip in there to mark, it is the crossing road's carriageway.
        boxspan.setdefault(id(ns_c), []).append((by0, by1))
        boxspan.setdefault(id(ew_c), []).append((bx0, bx1))
    flared = {}

    for ns_c, ew_c, (ix0, iy0, ix1, iy1) in junctions:
        # (crossed corridor, box edge, outward sign, bars run along X)
        approaches = ((ew_c, ix0, -1.0, True), (ew_c, ix1, +1.0, True),
                      (ns_c, iy0, -1.0, False), (ns_c, iy1, +1.0, False))

        for road, edge, sign, along_x in approaches:
            band_c = edge + sign * (setback + depth / 2.0)
            if not _fits(road, band_c, depth):
                n_skip += 1
                continue

            # Kerb to kerb as the policy leaves it: a street that parks on one
            # side only is narrower on the other, and the crossing is shorter.
            lo, hi = _kerb(road, "lo"), _kerb(road, "hi")
            if hi - lo <= 1e-3:
                n_skip += 1
                continue

            # Right-hand traffic: facing +x the right-hand kerb is -y, facing +y
            # it is +x. Which cross-half a vehicle approaching this junction
            # occupies therefore flips with the road's direction as well as with
            # the side of the box, and both the stop bar and the turn pocket
            # belong on that half.
            right_hi = (sign > 0) if along_x else (sign < 0)

            if do_xwalk:
                # MUTCD treats a run through a junction as its own marking — a
                # dotted extension line — so the ordinary centre and lane lines
                # do not carry on across a crossing. Those come from
                # apply_ground_planes, which has already run, so the band is
                # masked in asphalt just under the bars: above the lane lines at
                # z=0.020, below the ladder at z_high.
                if along_x:
                    m0, m1 = (band_c - depth / 2.0, lo), (band_c + depth / 2.0, hi)
                else:
                    m0, m1 = (lo, band_c - depth / 2.0), (hi, band_c + depth / 2.0)
                _make_plane_mesh(stage, f"{gnd}/xwalk_mask_{counter[0]}",
                                 m0[0], m0[1], m1[0], m1[1], z_high - 0.0005,
                                 uv_asphalt, ssf,
                                 display_color=(0.15, 0.15, 0.15),
                                 mat_prim_path=asphalt_mat)
                counter[0] += 1
                _ladder(stage, gnd, counter, band_c, depth / 2.0, lo, hi,
                        along_x, bar_w, bar_gap, ssf, z_high)
                n_x += 1
                # Recorded so the kerb-strip pass can keep hatching out of the
                # crossing: the ladder spans kerb to kerb, parking strips
                # included, so the two overlap wherever a crossing meets a
                # daylighting zone.
                xwalk.setdefault(id(road), []).append(
                    (band_c - depth / 2.0, band_c + depth / 2.0))
                transverse.setdefault(id(road), []).append(
                    (band_c - depth / 2.0, band_c + depth / 2.0))
            if do_stop:
                # A stop bar governs only the approaching half of the roadway.
                mid = (lo + hi) / 2.0
                s_lo, s_hi = (mid, hi) if right_hi else (lo, mid)
                s_c = (s_lo + s_hi) / 2.0
                bar_c = band_c + sign * (depth / 2.0 + stop_w / 2.0 + stop_adv)
                cx, cy = (bar_c, s_c) if along_x else (s_c, bar_c)
                _make_dash_mesh(stage, f"{gnd}/stopbar_{counter[0]}",
                                cx, cy, z_high, stop_w, s_hi - s_lo,
                                0.0 if along_x else 90.0, ssf, _WHITE)
                counter[0] += 1
                n_s += 1
                # A stop bar reaches from the centreline to the kerb, so it
                # crosses a bike lane. Recorded for the same reason the
                # crossing band is: a longitudinal line painted through a
                # transverse one at the same height stipples.
                transverse.setdefault(id(road), []).append(
                    (bar_c - stop_w / 2.0, bar_c + stop_w / 2.0))

            # Turn-lane flare: on a street that parks on the near side, the
            # approach takes that lane back for a turn pocket. The line along
            # the carriage edge plus the taper out to the kerb is what makes
            # that read as a lane rather than as a gap in the parking.
            side = "hi" if right_hi else "lo"
            if not (do_flare and float(road.get("park_w", 0.0)) > 0.0
                    and _has_strip(road, side)):
                continue
            # The pocket line starts clear of the stop bar's FAR edge. This has
            # to include stop_adv: the bar sits at
            # setback + depth + stop_adv + stop_w/2 from the box, so leaving it
            # out started the flare inside the bar and the taper crossed the
            # line a car stops at.
            s_stop = edge + sign * (setback + depth + stop_adv + stop_w + 0.4)
            s_in = edge + sign * flare_len
            s_out = edge + sign * (flare_len + taper_len)
            if abs(s_in - s_stop) < 1.0:
                continue
            if not _fits(road, (edge + s_out) / 2.0, abs(s_out - edge)):
                continue
            if rng.random() >= flare_chance:
                continue
            c_lo, c_hi = _carriage(road)
            k_lo, k_hi = _cross_span(road)
            car_edge = c_lo if side == "lo" else c_hi
            kerb_edge = k_lo if side == "lo" else k_hi
            _line(stage, f"{gnd}/flare_{counter[0]}",
                  _pt(road, s_stop, car_edge), _pt(road, s_in, car_edge),
                  line_w, ssf, z=z_low)
            counter[0] += 1
            _line(stage, f"{gnd}/flare_{counter[0]}",
                  _pt(road, s_in, car_edge), _pt(road, s_out, kerb_edge),
                  line_w, ssf, z=z_low)
            counter[0] += 1
            n_f += 1
            flared.setdefault((id(road), side), []).append(
                (min(edge, s_out), max(edge, s_out)))

    # ---- the kerb strips: bays where parking is legal, hatch where it isn't --
    hyd_zones = _kerb_zones(corridors, hydrants, hyd_clear, hyd_reach)
    # A bus zone is no-parking kerb like a hydrant clear zone, so it keeps bays
    # out — but it is NOT hatched. Hatching there would be painting a parking
    # prohibition over a stopping place that is signed and kerb-marked instead.
    bus_zones = _kerb_zones(corridors, bus_stops, bus_half, bus_reach)
    for c in corridors:
        park_w = c.get("park_w")
        # An explicit 0 means the road reserves no kerb strip at all (the border
        # ring) — anything painted there would be over a running lane. A
        # corridor with no such key predates the profile and keeps the
        # configured bay depth.
        if park_w is not None and float(park_w) <= 0.0:
            continue
        strip_w = float(park_w) if park_w else bay_depth
        run = _run_span(c)
        banned = _merge(list(keepout.get(id(c), ())))
        for side in ("lo", "hi"):
            if not _has_strip(c, side):
                continue
            inward = 1.0 if side == "lo" else -1.0
            kerb = _kerb(c, side)
            hyd = hyd_zones.get((id(c), side), [])
            bus = _merge(bus_zones.get((id(c), side), []))
            # Hydrant zones still keep BAYS out — parking there is prohibited —
            # but they are not painted. "Hydrant" appears twice in the whole
            # MUTCD: 3B.18 p4 permits a bare KERB marking, and 1A.02 p3(B)
            # declares hydrant markers not to be traffic control devices at all.
            # There is no pavement marking for hydrant clearance, so hatching
            # one was inventing a device. The kerb itself is implicit geometry
            # here, so the correct marking cannot be drawn either; unpainted is
            # what most jurisdictions actually do.
            here = _merge(banned + hyd + bus)
            paint = [q for sp in _merge(list(signed.get(id(c), ())))
                     for q in _subtract(sp, bus)]

            if do_bays:
                off = kerb + inward * strip_w / 2.0
                for s0, s1 in _subtract(run, here):
                    if s1 - s0 < bay_len * 2.0:
                        continue
                    n = int((s1 - s0) / bay_len)
                    step = (s1 - s0) / n
                    for k in range(n + 1):
                        cx, cy = _pt(c, s0 + k * step, off)
                        _make_dash_mesh(stage, f"{gnd}/bay_{counter[0]}",
                                        cx, cy, z_low, strip_w, tick_w,
                                        0.0 if c.get("dir") == "ns" else 90.0,
                                        ssf, _WHITE)
                        counter[0] += 1
                        n_b += 1

            # Hatching goes where parking is banned but the asphalt is still
            # there. Not under a flare (that stretch is a live turn lane), not
            # inside the junction box (there is no kerb strip in there at all),
            # and not in a bus zone — a bus stop is no-parking kerb, but it is
            # signed and kerb-marked rather than hatched.
            if not (do_hatch and park_w):
                continue
            # Half a line width short of the carriage edge. A bike lane puts
            # its kerb-side line exactly there, and a stroke ending on that
            # line's centre buries a corner inside it — coplanar, same height.
            far = kerb + inward * (strip_w - line_w / 2.0)
            # Crossings are subtracted alongside the flares: a ladder spans
            # kerb to kerb, so it covers the parking strip the hatching is in.
            skip = (list(flared.get((id(c), side), ()))
                    + list(xwalk.get(id(c), ()))
                    + list(boxspan.get(id(c), ())))
            for s0, s1 in _clip(run, paint):
                for h0, h1 in _subtract((s0, s1), skip):
                    s = h0
                    while s + strip_w <= h1 + 1e-6:
                        _line(stage, f"{gnd}/hatch_{counter[0]}",
                              _pt(c, s, kerb), _pt(c, s + strip_w, far),
                              hatch_w, ssf, _HATCH, z=z_low)
                        counter[0] += 1
                        n_h += 1
                        s += hatch_pitch

    # Bike lanes last: they sit inside the carriageway, so nothing above
    # depends on them, and the plan is recomputed rather than passed because
    # `city_detail` has to derive the identical one for the delineators.
    n_bl = n_bm = n_bd = n_by = 0
    bike_cfg = cfg.get("bike_lanes") or {}
    if bike_cfg.get("enabled", True):
        before = counter[0]
        plan = plan_bike_lanes(config, layout)
        n_bl, n_bm, n_bd, n_by = _draw_bike_lanes(
            stage, gnd, plan, bike_cfg, ssf, counter, z_low, z_high,
            transverse)
        n_posts = sum(len(p["posts"]) for p in plan)
        n_prot = sum(sum(s1 - s0 for s0, s1 in p["protected"]) for p in plan)
        n_grn = sum(sum(s1 - s0 for s0, s1 in p["green"]) for p in plan)
        n_site = sum(len(p["mix"]) for p in plan)
        n_gsite = sum(len(p["green"]) for p in plan)
        print(f"[road_markings] bike lanes: {n_bl} lane-sides, {n_bm:,.0f} m "
              f"({n_prot:,.0f} m protected = {n_prot / max(1.0, n_bm):.0%}), "
              f"{n_posts} delineator stations, {n_bd} dashes, {n_by} symbols, "
              f"{counter[0] - before} quads")
        print(f"[road_markings] bike green: {n_gsite}/{n_site} conflict points "
              f"= {n_grn:,.0f} m ({n_grn / max(1.0, n_bm):.0%} of lane); "
              f"junction boxes carry no bike paint")

    print(f"[road_markings] {counter[0]} quads — {n_x} crosswalk bands, "
          f"{n_s} stop bars, {n_f} turn-lane flares, {n_b} bay ticks, "
          f"{n_h} no-parking hatches ({len(hydrants)} hydrants, "
          f"{n_skip} approaches skipped, no road there)")
    return counter[0]
