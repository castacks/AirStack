#!/usr/bin/env python3
"""
test_scour_relief.py — the 3D ground scour, pinned without Isaac.

`disaster/scour_relief.py` adds authored EARTH to a tornado track: the spoil of
the grooves a suction vortex cut, windrows, heaps of subsoil, rolled mats of
turf and thrown clods, standing on top of the flat mud overlay that
`ground.build_overlay` was already drawing. Six claims are worth pinning,
because every one of them is invisible in a render until it is wrong and then
it is a re-bake to find out:

  1. THE FIELD GATES IT. Nothing is authored outside the corridor, and the
     relief holds tighter to the core than the mud overlay does.
  2. THE PLACEMENT RULES ARE THE PHYSICS. Cycloidal marks land on the RIGHT
     flank (rotation and translation add there); deposition — mounds and
     windrows — is biased LEFT by `left_bias`; sod rolls sit on the coverage
     EDGE, not in the core, because in the core there is no turf left to peel.
  3. THE ARCS ARE ACTUALLY ARCS. They turn, and their two levees are never the
     same size, with the bigger one consistently on the outside.
  4. THE GEOMETRY IS SOUND. Faces index real points, windings and normals
     agree, and nothing points into the ground.
  5. EVERYTHING IS SEATED. No feature floats above its base plane or sinks
     into it, on soil or on asphalt — including a clod, whose `z` is its box
     CENTRE while every other kind's is a grade.
  6. IT IS CHEAP AND IT IS REPEATABLE. A 500 m plate fits in three meshes and
     a bounded point budget, and one seed gives one answer.

RUNS WITHOUT ISAAC. `scour_relief` imports `pxr` only inside `materials` and
`build`; `scatter`, `geometry` and `summarise` are pure Python, which is the
whole reason they are separate functions. Nothing here imports `pxr`, opens a
stage or needs a GPU.

USAGE
    python3 scene_gen/tests/test_scour_relief.py
    pytest -s scene_gen/tests/test_scour_relief.py
"""

import math
import os
import random
import statistics as stats
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

import numpy as np                                    # noqa: E402
from disaster import scour_relief as sr               # noqa: E402
from disaster import tornado as tn                    # noqa: E402

REGION = (-250.0, -250.0, 250.0, 250.0)
SEED = 11
GROUND_Z = 0.06          # `suburb_scene`'s mud band, between grass and asphalt
PAVE_Z = 0.11            # ...and just over the asphalt

FAILS = []


def check(cond, msg):
    print(("    PASS  " if cond else "    FAIL  ") + msg)
    if not cond:
        FAILS.append(msg)


# ---------------------------------------------------------------------------
# the scene under test, built once
# ---------------------------------------------------------------------------

def _fields(cfg=None, seed=SEED):
    cfg = dict(tn.DEFAULTS) if cfg is None else cfg
    inten = tn.intensity_field(cfg, REGION, np.random.default_rng(seed + 23))
    cov = tn.scour_coverage(cfg, REGION, np.random.default_rng(seed + 31),
                            intensity=inten)
    return cfg, inten, cov


def _streets(spacing=90.0, half_w=5.8):
    """A plain grid standing in for `suburb_net`'s edges: `(points, half_w)`,
    which is the shape `pavement_mask` takes and the shape an `Edge` already
    has (`e.pts`, `e.half_w`)."""
    out = []
    k0 = int(REGION[0] / spacing)
    for k in range(k0, -k0 + 1):
        out.append(([(REGION[0], k * spacing), (REGION[2], k * spacing)], half_w))
        out.append(([(k * spacing, REGION[1]), (k * spacing, REGION[3])], half_w))
    return out


def _scatter(pavement_at=None, skip=None, knobs=None, seed=SEED, cfg=None):
    cfg, _inten, cov = _fields(cfg, seed)
    return cfg, cov, sr.scatter(cfg, REGION, cov, random.Random(seed),
                                ground_z=GROUND_Z, pave_z=PAVE_Z,
                                pavement_at=pavement_at, skip=skip,
                                knobs=knobs)


def _anchor(spec):
    """Where a feature was PLACED — the point the density field was consulted
    at, which is `x`/`y` for everything the field pass authors.

    An `arc` has no seed point: it is walked station by station and each
    station is gated on its own, so its mid station is the right anchor. And a
    windrow's mid station is NOT — it can sit a metre off its seed point, and
    `scour_coverage`'s islands live on a ~2 m lattice, so measuring there asks
    a question about the polyline rather than about the placement.
    """
    if "x" in spec:
        return spec["x"], spec["y"]
    st = spec["stations"]
    return st[len(st) // 2][0], st[len(st) // 2][1]


def _by_kind(specs):
    out = {}
    for s in specs:
        out.setdefault(s["kind"], []).append(s)
    return out


def _table(title, headers, rows):
    cols = [len(h) for h in headers]
    srows = [[str(c) for c in r] for r in rows]
    for r in srows:
        for i, c in enumerate(r):
            cols[i] = max(cols[i], len(c))
    print("    " + title)
    print("      " + "  ".join(h.ljust(cols[i]) for i, h in enumerate(headers)))
    for r in srows:
        print("      " + "  ".join(c.ljust(cols[i]) for i, c in enumerate(r)))


# ---------------------------------------------------------------------------
# 1. the track frame round-trips
# ---------------------------------------------------------------------------

def test_track_frame_inverse():
    """`tornado.from_track` is the EXACT inverse of `frame`'s `to_track`.

    The arcs are generated in track coordinates and placed through the inverse,
    so any disagreement between the two puts the ground relief off the corridor
    the intensity field actually cut — and a render cannot say which of the two
    moved. Pinned to floating-point rather than to a tolerance, because they
    now share one `_wobble`.
    """
    print("\n[1] the track frame round-trips exactly")
    cfg = dict(tn.DEFAULTS)
    to_track, _u, _v = tn.frame(cfg)
    to_world = tn.from_track(cfg)
    rng = random.Random(3)
    worst_xy = worst_ac = 0.0
    for _k in range(4000):
        x = rng.uniform(-400.0, 400.0)
        y = rng.uniform(-400.0, 400.0)
        a, c = to_track(x, y)
        bx, by = to_world(a, c)
        worst_xy = max(worst_xy, math.hypot(bx - x, by - y))
        a2, c2 = to_track(bx, by)
        worst_ac = max(worst_ac, abs(a2 - a), abs(c2 - c))
    print("      worst world round-trip %.3g m, worst track round-trip %.3g m"
          % (worst_xy, worst_ac))
    check(worst_xy < 1e-8, "world -> track -> world is exact (%.3g m)" % worst_xy)
    check(worst_ac < 1e-8, "track -> world -> track is exact (%.3g m)" % worst_ac)

    # ...and a wobbling track really is not a straight one, or the test above
    # would pass on an identity map.
    off = [abs(to_world(a, 0.0)[1] - to_world(0.0, 0.0)[1]) for a in
           range(0, 400, 17)]
    check(max(off) > 5.0, "the centreline actually meanders (%.1f m of it)"
          % max(off))


# ---------------------------------------------------------------------------
# 2. the field gates the pass
# ---------------------------------------------------------------------------

def test_field_gates_everything():
    """Nothing outside the corridor, and the relief is TIGHTER than the mud."""
    print("\n[2] the coverage field gates every feature")
    _cfg, cov, specs = _scatter()
    kn = sr.DEFAULT_KNOBS
    covs = [float(cov(*_anchor(s))) for s in specs]
    check(len(specs) > 300, "the pass authors something at all (%d features)"
          % len(specs))
    check(min(covs) >= kn["min_coverage"] - 1e-9,
          "nothing below min_coverage=%.2f (lowest %.3f)"
          % (kn["min_coverage"], min(covs)))

    # The overlay reaches further than the relief does, and that is the point:
    # peeling turf takes less wind than gouging the subsoil out from under it.
    ov_area = rel_area = 0
    n = 200
    for i in range(n):
        for j in range(n):
            x = REGION[0] + (i + 0.5) * (REGION[2] - REGION[0]) / n
            y = REGION[1] + (j + 0.5) * (REGION[3] - REGION[1]) / n
            c = float(cov(x, y))
            ov_area += c > 0.06                     # `build_overlay`'s own cut
            rel_area += c >= kn["min_coverage"]
    print("      overlay covers %.1f%% of the plate, relief %.1f%%"
          % (100.0 * ov_area / (n * n), 100.0 * rel_area / (n * n)))
    check(0 < rel_area < ov_area,
          "the relief band is inside the mud band, not equal to it")

    # A track with no damage anywhere authors nothing rather than crashing.
    dead = dict(tn.DEFAULTS, peak=0.0)
    _c2, _cov2, none_specs = _scatter(cfg=dead)
    check(not none_specs, "a zero-intensity track authors nothing (%d)"
          % len(none_specs))


# ---------------------------------------------------------------------------
# 3. the placement rules are the physics
# ---------------------------------------------------------------------------

def test_placement_rules():
    """Arcs right, deposition left, sod on the edge."""
    print("\n[3] where each kind lands is the mechanism, not a taste")
    cfg, cov, specs = _scatter()
    to_track, _u, _v = tn.frame(cfg)
    by = _by_kind(specs)
    rows = []
    for k in sorted(by):
        v = by[k]
        cr = [to_track(*_anchor(s))[1] for s in v]
        cv = [float(cov(*_anchor(s))) for s in v]
        rows.append((k, len(v), "%.0f%%" % (100.0 * sum(1 for q in cr if q > 0)
                                            / len(cr)),
                     "%.2f" % stats.median(cv)))
    _table("by kind", ("kind", "n", "left of track", "median coverage"), rows)

    def frac_left(kind):
        v = by.get(kind) or []
        cr = [to_track(*_anchor(s))[1] for s in v]
        return (sum(1 for q in cr if q > 0) / float(len(cr))) if cr else 0.0

    check(by.get("arc"), "cycloidal marks were authored (%d)"
          % len(by.get("arc") or ()))
    check(frac_left("arc") < 0.25,
          "the marks are cut on the RIGHT flank (%.0f%% left)"
          % (100 * frac_left("arc")))
    # `left_bias` 0.55 keeps 55% of the right-flank cells, so the expectation
    # is 1 / (1 + 0.55) = 65% left on a field that is symmetric in `cross`.
    check(frac_left("mound") > 0.56,
          "mounds are biased LEFT, the deposition side (%.0f%%)"
          % (100 * frac_left("mound")))
    # POOLED OVER SEEDS for the windrows. There are a few dozen of them on a
    # plate against a few hundred mounds, and one plate's left-fraction is a
    # coin flip at that count — the assertion would pass or fail on the seed
    # rather than on the rule. They ride the same `keep_dep` gate the mounds
    # do, so what is being pinned is that the gate reaches them at all.
    n_l = n_r = 0
    for sd in (SEED, SEED + 1, SEED + 2, SEED + 3):
        c2, _v2, sp2 = _scatter(seed=sd)
        tt2 = tn.frame(c2)[0]
        for s in _by_kind(sp2).get("ridge") or ():
            if tt2(*_anchor(s))[1] > 0:
                n_l += 1
            else:
                n_r += 1
    check(n_l > n_r,
          "windrows are biased LEFT too, over 4 seeds (%d left / %d right)"
          % (n_l, n_r))

    med = {k: stats.median([float(cov(*_anchor(s))) for s in by[k]])
           for k in by if by[k]}
    check(med["sod"] < med["mound"] - 0.08,
          "sod rolls sit on the coverage EDGE, mounds in the core "
          "(%.2f vs %.2f)" % (med["sod"], med["mound"]))
    check(med["arc"] >= sr.DEFAULT_KNOBS["arc_min_coverage"] - 0.02,
          "the marks are held to the core (%.2f)" % med["arc"])


# ---------------------------------------------------------------------------
# 4. the arcs are arcs
# ---------------------------------------------------------------------------

def test_arcs_are_cycloidal():
    """A mark TURNS, and its spoil banks to one side of it."""
    print("\n[4] a cycloidal mark curves, and its levees are asymmetric")
    _cfg, _cov, specs = _scatter()
    arcs = _by_kind(specs).get("arc") or []
    check(len(arcs) >= 8, "several marks, not one (%d)" % len(arcs))

    turns, ratios, lens = [], [], []
    for s in arcs:
        st = s["stations"]
        total = 0.0
        for a, b, c in zip(st, st[1:], st[2:]):
            v1 = math.atan2(b[1] - a[1], b[0] - a[0])
            v2 = math.atan2(c[1] - b[1], c[0] - b[0])
            total += (v2 - v1 + math.pi) % (2 * math.pi) - math.pi
        turns.append(abs(math.degrees(total)))
        lens.append(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                        for a, b in zip(st, st[1:])))
        ratios.append(stats.mean(q[3] / max(1e-6, q[2]) for q in st))
    print("      %d mark(s): median length %.0f m, median turn %.0f deg, "
          "median right/left crest %.2f"
          % (len(arcs), stats.median(lens), stats.median(turns),
             stats.median(ratios)))
    check(stats.median(turns) > 20.0,
          "the marks curve rather than run straight (%.0f deg median)"
          % stats.median(turns))
    check(min(lens) >= sr.DEFAULT_KNOBS["arc_min_len_m"] - 1e-6,
          "no stub runs kept (shortest %.1f m)" % min(lens))
    # The bias is `(sin(theta) - k) / speed`, negative for every looping mark,
    # so the RIGHT levee is the big one on every one of them — not on average.
    check(all(r > 1.15 for r in ratios),
          "every mark banks its spoil to the same side (min ratio %.2f)"
          % min(ratios))


# ---------------------------------------------------------------------------
# 5. the geometry is sound
# ---------------------------------------------------------------------------

def test_geometry_is_sound():
    """Faces index real points; windings and normals agree; nothing is
    authored pointing into the ground."""
    print("\n[5] every mesh is well formed and lit from the right side")
    _cfg, _cov, specs = _scatter()
    bad_index = degenerate = down = 0
    n_faces = 0
    for s in specs:
        pts, faces = sr.geometry(s)
        for f in faces:
            n_faces += 1
            if any(v < 0 or v >= len(pts) for v in f):
                bad_index += 1
                continue
            if len(set(f)) != len(f):
                degenerate += 1
            if s["kind"] == "clod":
                continue            # a box has an underside, by construction
            if sr._normal(pts, f)[2] <= 0.0:
                down += 1
    print("      %d feature(s), %d face(s)" % (len(specs), n_faces))
    check(bad_index == 0, "every face indexes a real point (%d bad)" % bad_index)
    check(degenerate == 0, "no face repeats a vertex (%d bad)" % degenerate)
    check(down == 0,
          "every open-surface face is wound to face UP (%d inverted)" % down)

    # And the one closed solid in the set really is closed both ways: a box
    # has exactly one face pointing up and one pointing down.
    clod = _by_kind(specs)["clod"][0]
    pts, faces = sr.geometry(clod)
    nz = sorted(sr._normal(pts, f)[2] for f in faces)
    check(nz[0] < -0.5 and nz[-1] > 0.5,
          "a clod box has an underside and a top (nz %.2f .. %.2f)"
          % (nz[0], nz[-1]))

    # A `fan` is a squashed `mound`, not a different shape — the pass walks
    # continuously between them and a knob has to be able to.
    _c2, _cov2, paved = _scatter(pavement_at=sr.pavement_mask(_streets(), REGION))
    fans = _by_kind(paved).get("fan") or []
    check(fans, "road wash was authored (%d fan(s))" % len(fans))
    if fans:
        a = sr.geometry(fans[0])
        b = sr.geometry(dict(fans[0], kind="mound"))
        check(len(a[0]) == len(b[0]) and a[1] == b[1],
              "a fan and a mound are the same mesh at different heights")


# ---------------------------------------------------------------------------
# 6. everything is seated
# ---------------------------------------------------------------------------

def test_everything_is_seated():
    """Nothing floats and nothing sinks — on soil or on asphalt."""
    print("\n[6] every feature stands on its own base plane")
    pave = sr.pavement_mask(_streets(), REGION)
    _cfg, _cov, specs = _scatter(pavement_at=pave)
    lo = hi = 0.0
    worst_kind = None
    for s in specs:
        pts, _f = sr.geometry(s)
        d = min(q[2] for q in pts) - float(s["base"])
        if d < lo:
            lo, worst_kind = d, s["kind"]
        hi = max(hi, d)
    print("      lowest point relative to base: %.4f m .. %.4f m (worst %s)"
          % (lo, hi, worst_kind))
    # `_SINK_M` is deliberate: every rim is set a couple of centimetres below
    # grade so the silhouette has no hairline of background under it.
    check(lo >= -sr._SINK_M - 1e-6,
          "nothing sinks further than _SINK_M (%.4f m)" % lo)
    check(hi <= 0.001, "nothing hovers above its base (%.4f m)" % hi)

    # The two grades really are different, and a clod on the road takes the
    # road's. This is the bug that hid: a lawn-grade clod inside asphalt that
    # sits 4 cm higher is invisible from every angle except the one that
    # matters.
    bases = sorted({round(float(s["base"]), 4) for s in specs})
    check(bases == [round(GROUND_Z, 4), round(PAVE_Z, 4)],
          "features are based on soil grade OR pavement grade %s" % (bases,))


# ---------------------------------------------------------------------------
# 7. pavement, pools and the plate edge
# ---------------------------------------------------------------------------

def test_pavement_pools_and_edges():
    """The road gets wash and nothing else; a skip is respected; nothing hangs
    off the plate."""
    print("\n[7] the road, the pools and the edge of the world")
    streets = _streets()
    pave = sr.pavement_mask(streets, REGION)
    check(pave(0.0, 0.0) and pave(120.0, 90.0),
          "a point on a carriageway is masked")
    check(not pave(40.0, 40.0), "a point in the middle of a block is not")

    # A pool: everything inside this square must be empty.
    box = (-40.0, 60.0, 0.0, 100.0)

    def skip(x, y):
        return box[0] <= x <= box[2] and box[1] <= y <= box[3]

    _cfg, _cov, specs = _scatter(pavement_at=pave, skip=skip)

    on_road = {}
    in_box = 0
    outside = 0
    for s in specs:
        pts, _f = sr.geometry(s)
        cx = sum(q[0] for q in pts) / len(pts)
        cy = sum(q[1] for q in pts) / len(pts)
        if pave(cx, cy):
            on_road[s["kind"]] = on_road.get(s["kind"], 0) + 1
        if skip(*_anchor(s)):
            in_box += 1
        if (min(q[0] for q in pts) < REGION[0]
                or max(q[0] for q in pts) > REGION[2]
                or min(q[1] for q in pts) < REGION[1]
                or max(q[1] for q in pts) > REGION[3]):
            outside += 1
    print("      kinds sitting on a carriageway: %s" % (on_road or "{}"))
    check(in_box == 0, "the skipped rectangle is empty (%d features in it)"
          % in_box)
    check(outside == 0, "no geometry leaves the plate (%d features do)"
          % outside)
    check(set(on_road) <= {"fan", "clod"},
          "only wash and thrown clods reach the asphalt (%s)"
          % sorted(set(on_road)))
    check(on_road.get("fan", 0) > 0, "the roads DO get mud (%d fan(s))"
          % on_road.get("fan", 0))
    for s in specs:
        if s["kind"] == "fan":
            check_z = abs(float(s["base"]) - PAVE_Z) < 1e-9
            if not check_z:
                check(False, "a fan is based off the pavement grade")
                break
    else:
        check(True, "every fan sits on the pavement grade")


# ---------------------------------------------------------------------------
# 8. the knobs do what they say
# ---------------------------------------------------------------------------

def test_knobs():
    """`SCOUR_*` reaches the scatter, and `height` is the one knob that scales
    everything vertical without moving anything."""
    print("\n[8] the knobs")
    old = {k: os.environ.get(k) for k in ("SCOUR_HEIGHT", "SCOUR_RELIEF",
                                          "SCOUR_MOUNDS_PER_100M2")}
    try:
        os.environ["SCOUR_HEIGHT"] = "2.0"
        os.environ["SCOUR_MOUNDS_PER_100M2"] = "1.7"
        kn = sr.knobs_from_env()
        check(kn["height"] == 2.0 and kn["mounds_per_100m2"] == 1.7,
              "SCOUR_* overrides land in the knob dict")
        check(kn["arc_vortices"] == sr.DEFAULT_KNOBS["arc_vortices"],
              "an unset knob keeps its default")
        os.environ["SCOUR_RELIEF"] = "0"
        check(not sr.enabled(), "SCOUR_RELIEF=0 turns the pass off")
        os.environ["SCOUR_RELIEF"] = "1"
        check(sr.enabled(), "...and 1 turns it back on")
    finally:
        for k, v in old.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v

    # `height` scales heights and leaves counts and plan positions alone.
    _c1, _v1, base = _scatter(knobs={"height": 1.0})
    _c2, _v2, tall = _scatter(knobs={"height": 2.0})
    # NEARLY, not exactly. A clod and a sod roll scale bodily (their height
    # is their size), so a taller pass has slightly bigger plan extents and
    # `clip_to_region` drops one or two more at the plate edge. Anything more
    # than that means the knob is moving placements, which it must not.
    drift = abs(len(base) - len(tall)) / float(len(base))
    check(drift < 0.01,
          "height barely changes what is authored (%d vs %d, %.2f%%)"
          % (len(base), len(tall), 100 * drift))

    def _h(specs, kind):
        out = []
        for s in specs:
            if s["kind"] != kind:
                continue
            pts, _f = sr.geometry(s)
            out.append(max(q[2] for q in pts) - min(q[2] for q in pts))
        return stats.median(out) if out else 0.0

    for kind in ("mound", "sod", "arc"):
        r = _h(tall, kind) / max(1e-9, _h(base, kind))
        print("      %-6s median height x%.2f" % (kind, r))
        check(1.5 < r < 2.1, "%s scales with height (x%.2f)" % (kind, r))


# ---------------------------------------------------------------------------
# 9. it is cheap, and it is repeatable
# ---------------------------------------------------------------------------

def test_budget_and_determinism():
    """A 500 m plate is three meshes, a bounded point budget and one answer
    per seed."""
    print("\n[9] the budget and the seed")
    t0 = time.time()
    _cfg, _cov, specs = _scatter()
    dt = time.time() - t0
    summ = sr.summarise(specs)
    _table("summary", ("field", "value"),
           sorted((k, v) for k, v in summ.items()))
    check(dt < 5.0, "the scatter is host-side cheap (%.2f s)" % dt)
    check(summ["points"] < 150000,
          "the point budget is a plank field's, not a fracture's (%d)"
          % summ["points"])
    check(len(summ["by_class"]) <= len(sr.CLASSES),
          "no more classes than materials — %d mesh(es) for the plate"
          % len(summ["by_class"]))
    check(summ["max_height_m"] < 0.9,
          "nothing is a boulder (tallest feature %.2f m)"
          % summ["max_height_m"])
    med = stats.median([max(q[2] for q in sr.geometry(s)[0])
                        - min(q[2] for q in sr.geometry(s)[0])
                        for s in specs if s["kind"] == "mound"])
    check(0.10 < med < 0.60,
          "a typical mound is ankle to knee high (%.2f m)" % med)

    _c1, _v1, a = _scatter(seed=SEED)
    _c2, _v2, b = _scatter(seed=SEED)
    check(a == b, "one seed gives one scatter")
    _c3, _v3, c = _scatter(seed=SEED + 1)
    check(a != c, "a different seed gives a different one")


# ---------------------------------------------------------------------------
# 10. the launcher wires it up correctly
# ---------------------------------------------------------------------------

_LAUNCHER = os.path.join(
    os.path.dirname(_SCENE_GEN), "simulation", "isaac-sim", "launch_scripts",
    "suburb_tornado_launch_script.py")


def test_launcher_wiring():
    """The scene's own `7a` block, SLICED OUT OF THE LAUNCHER and run.

    The module can be perfect and the scene still wrong, because everything
    that can go wrong at the seam is a number: which grade the relief is based
    on, which grade the road wash is based on, whether the pool rings and the
    standing houses actually reach the skip. None of that is reachable by
    importing the launcher — its module scope starts Isaac — so the block is
    read as SOURCE and exec'd against stubs, the same trick
    `test_scene_modularity` uses on the REGION_M parser. What runs here is the
    repo's real wiring rather than a copy of it.
    """
    print("\n[10] the launcher's own scour-relief block")
    import textwrap
    import types

    src = open(_LAUNCHER, encoding="utf-8").read()
    i = src.index("    # 7a) THE 3D SCOUR RELIEF")
    j = src.index("    # 7b) THE PEOPLE", i)
    block = textwrap.dedent(src[i:j])
    check("srl.scatter(" in block and "srl.build(" in block,
          "the block is the one that scatters and builds")

    seen = {}

    class _Srl(object):
        DEFAULT_KNOBS = sr.DEFAULT_KNOBS
        enabled = staticmethod(lambda: True)
        knobs_from_env = staticmethod(sr.knobs_from_env)
        pavement_mask = staticmethod(sr.pavement_mask)
        summarise = staticmethod(lambda specs: {"features": len(specs),
                                                "by_kind": {}, "points": 0,
                                                "max_height_m": 0.0})
        materials = staticmethod(lambda *a, **k: {})
        build = staticmethod(lambda *a, **k: ["/World/scourRelief/soil"])

        @staticmethod
        def scatter(cfg, region, coverage_at, rng, **kw):
            seen.update(kw)
            seen["region"] = region
            seen["coverage_at"] = coverage_at
            return []

    class _Edge(object):
        def __init__(self, pts, hw):
            self.pts, self.half_w = pts, hw

    # A pool ring and a house, both of which must end up in the skip.
    pool = [(-30.0, 60.0), (-10.0, 60.0), (-10.0, 80.0), (-30.0, 80.0)]
    edges = {0: _Edge([(-250.0, 0.0), (250.0, 0.0)], 5.8)}
    zs = 0.3125                       # `ground_z_scale` for a 500 m plate
    mud_z = (0.02 + 0.5 * (0.10 - 0.02)) * zs

    ns = {
        "srl": _Srl,
        "ground": types.SimpleNamespace(skip_rects=__import__(
            "disaster.ground", fromlist=["ground"]).skip_rects),
        "ss": types.SimpleNamespace(_Z_ASPHALT=0.10),
        "binfo": {"pool_rects": [pool],
                  "net": types.SimpleNamespace(edges=edges)},
        "standing": [(100.0, 100.0, 12.0)],
        "region": REGION, "tcfg": dict(tn.DEFAULTS), "cov": (lambda x, y: 0.5),
        "SEED": SEED, "throw_deg": 55.0, "mud_z": mud_z, "zs": zs,
        "stage": None, "PARENT": "/World/stage/generated", "ssf": 1.0,
        "DO_GROUND": True, "random": random, "time": time,
        "made_r": [], "_sm": None,
    }
    exec(compile(block, "<7a>", "exec"), ns)

    check(seen, "the block reached srl.scatter")
    # THE Z LADDER. The relief stands ON the overlay and the road wash stands
    # on the road; getting either backwards buries the geometry in the surface
    # it is supposed to be lying on.
    print("      mud band %.4f m, relief base %.4f m, wash base %.4f m"
          % (mud_z, seen["ground_z"], seen["pave_z"]))
    check(seen["ground_z"] > mud_z,
          "the relief is based ABOVE the mud band it stands on")
    check(seen["pave_z"] > 0.10 * zs,
          "the road wash is based ABOVE the asphalt")
    check(seen["pave_z"] > seen["ground_z"],
          "...and the road is above the lawn, as on the z ladder")
    check(seen["flow_deg"] == 55.0, "the throw bearing is passed through")
    check(seen["coverage_at"] is ns["cov"],
          "the relief reads the SAME coverage field the overlay drew")

    skip = seen["skip"]
    check(skip(-20.0, 70.0), "a point in the pool is skipped")
    check(skip(100.0, 100.0) and skip(104.0, 96.0),
          "a point inside a standing house is skipped")
    check(not skip(0.0, -120.0), "open ground is not")

    pave = seen["pavement_at"]
    check(pave is not None and pave(30.0, 0.0),
          "the carriageway mask came from the street graph")
    check(not pave(30.0, 40.0), "...and it is not the whole plate")

    check(seen["knobs"]["height"] == sr.DEFAULT_KNOBS["height"],
          "the SCOUR_* knobs are read at the seam")
    check(ns["made_r"] and ns["_sm"],
          "the block publishes its meshes and its summary to the banner")


# ---------------------------------------------------------------------------
# runner
# ---------------------------------------------------------------------------

def main():
    tests = [(n, o) for n, o in sorted(globals().items())
             if n.startswith("test_") and callable(o)]
    print("scour relief: %d tests, offline (no pxr, no GPU)" % len(tests))
    broken = []
    for name, fn in tests:
        try:
            fn()
        except Exception as exc:                       # noqa: BLE001
            import traceback
            broken.append(name)
            print("    ERROR %s: %s" % (name, exc))
            print(traceback.format_exc())
    print("\n" + "=" * 72)
    if FAILS or broken:
        for m in FAILS:
            print("  FAILED: " + m)
        for m in broken:
            print("  ERRORED: " + m)
        return 1
    print("  all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
