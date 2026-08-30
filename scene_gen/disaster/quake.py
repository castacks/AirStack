"""quake — assemble an earthquake-damaged district from baked archetypes.

The scene-level half of the earthquake pipeline (`quake_flow` is the
per-building half). The downtown layout is generated with the PRISTINE kit
archetypes in its building pools, so the packer lays out real footprints;
this pass then reads the disaster field at every placed building, picks an
EMS-98 grade for its construction type, and re-points the reference at the
baked damaged archetype of that grade. Same origin, same pose — the bake
re-centred every archetype on the building's own centre — so nothing moves.

Two things are done here rather than in the bake:

* **tilt_sink** is a rigid transform on the pristine archetype (rotate about
  a base edge, sink). Baking it would multiply the library for no gain.
* **which building gets what** is a scene decision: the field, the
  construction type's vulnerability, a per-building jitter so the grades are
  not contour rings, and a slenderness gate on tilting (Adapazarı: only
  H/B > 2 blocks free on a side tilted significantly).

Module level is stdlib + numpy + pxr, the same bar the rest of `scene_gen`
holds.
"""

import json
import math
import os
import random

from . import quake_flow as qf

# `pxr` is imported PER FUNCTION below, not at module scope (agent F, round
# 4): this file used to `from pxr import Gf, Sdf, Usd, UsdGeom` here, which
# made it unimportable on the host where `_clear_under_heaps`'s pure helpers
# (`heap_reach_m`, `in_reach`, `_heap_action`, `_dedupe_actions`) need to run
# for `tests/test_quake_heap_clearance.py`. Same convention `quake_flow.py`
# already holds, for the same reason.

ARCH_PREFIX = "bld_"


# ---------------------------------------------------------------------------
# HEAP CLEARANCE, pure half (agent F): street trees, lamps/signs/hydrants/
# bins and parked cars caught under a collapse pile. PURE — no pxr, no
# `scene_generator` — so `tests/test_quake_heap_clearance.py` can import this
# module and exercise the reach/decision math on the host. The pxr-touching
# half (`_clear_under_heaps`, `_lean_matrix`) lives further down, next to
# `_mono_pass`, which it runs after.
# ---------------------------------------------------------------------------
HEAP_REACH_FLOOR_M = 1.5      # a heap always reaches at least this far
HEAP_REACH_CAP_M = 10.0      # fall side: matches quake_rubble.RUNOUT_CAP_M
HEAP_REACH_CAP_BLIND_M = 3.0  # blind / party-wall sides: matches the planner's blind cap

# run-out fraction of H on the STREET / FALL side, DG5 rc baseline: 0.65 at
# H <= 4 m, 0.40 at H == 12 m, 0.35 at H >= 20 m, linear between the knots
# (agent A's memo, `_plans/eq_round4_rubble_research.md` SS1b/SS6 — a tall
# building's debris pile is proportionally SHORTER-reaching than a short
# one's, not longer). URM debris does not travel as far (x0.85); an rc_glass
# tower at DG5 sheds only its cladding, not the frame (x0.5). A BLIND /
# party-wall side gets a flat 0.10 H regardless of type — the far side of a
# heap does not know what construction it is.
_FALL_KNOTS = ((4.0, 0.65), (12.0, 0.40), (20.0, 0.35))
_BLIND_FRAC = 0.10


def _fall_frac(H):
    """The run-out curve's value at height `H`, flat past the end knots."""
    (h0, f0), (h1, f1), (h2, f2) = _FALL_KNOTS
    if H <= h0:
        return f0
    if H <= h1:
        return f0 + (f1 - f0) * (H - h0) / (h1 - h0)
    if H <= h2:
        return f1 + (f2 - f1) * (H - h1) / (h2 - h1)
    return f2


# Downtown street furniture comes from two generators with two vocabularies
# (`scene_generator.build_city`'s legacy packer vs. `detail/city_detail.py`'s
# per-category placer — see `_CATEGORY` there), both feeding the same flat
# `placements` list `assemble` already receives. Every entry is its own
# referenced prim (`scene_generator.apply_placements`: "NOT INSTANCED BY
# DEFAULT... every placement composes as its own reference"), never a
# PointInstancer — `_clear_under_heaps` still handles that case defensively
# because a future/other pipeline could scatter whole props that way.
_TREE_CATS = frozenset({"tree", "street_tree"})
_CAR_CATS = frozenset({"car"})
_POLE_CATS = frozenset({
    "streetlight", "traffic_light", "sign", "fire_hydrant", "bollard",
    "utility_pole", "billboard", "mailbox", "parking_meter", "bike_rack",
    "newspaper_box", "phone_booth", "trash_can", "dumpster",
    "planter_fence", "bike_lane_delineator", "traffic_cone",
})

_ACTION_PRIORITY = {"remove": 4, "tip": 3, "lean": 2, "bury": 1}


def heap_reach_m(btype, grade, H, fall_side=True):
    """Metres a heap's debris reaches beyond the wall line, on the STREET /
    FALL side (`fall_side=True`) or a BLIND / party-wall side (False).

    `btype` only matters on the fall side (see `_fall_frac`'s note); the
    blind side is a flat fraction of H whatever the construction is. DG4
    (partial) always halves whichever of those two values applies — half the
    building came down, roughly half as much debris travels. Floored so a
    short building still buries its own kerb, capped so a 60 m tower does
    not reach across the street."""
    H = float(H)
    if fall_side:
        frac = _fall_frac(H)
        if btype == "urm":
            frac *= 0.85
        elif btype == "rc_glass":
            frac *= 0.5
    else:
        frac = _BLIND_FRAC
    if str(grade) == "DG4":
        frac *= 0.5
    cap = HEAP_REACH_CAP_M if fall_side else HEAP_REACH_CAP_BLIND_M
    return min(cap, max(HEAP_REACH_FLOOR_M, frac * H))


def heap_reach_sides(btype, grade, H, prim_path=""):
    """{"S", "E", "N", "W"} -> reach_m for one building's heap. "S" is the
    FRONT — -Y in the building's own frame, the side facing `yaw_deg`'s
    street, the same S/E/N/W convention `quake_flow._outward` already uses —
    and always gets the fall-side reach. A baked archetype does not record
    which way it actually fell, so a SECOND side, drawn deterministically
    from `prim_path`, also gets the fall-side reach (a corner building or an
    L-plan throwing debris across two streets is not rare). The remaining
    two sides get the blind/party-wall reach.

    A STABLE hash (`zlib.crc32`) picks the second side, not Python's
    built-in `hash()` — `hash()` on a string is salted per PROCESS
    (`PYTHONHASHSEED`) since Python 3.3, and this whole pipeline is seeded
    for reproducibility end to end (`build_city`: "Deterministic for a given
    config['seed']"); the same scene run twice has to throw debris the same
    way both times.
    """
    import zlib

    fall = heap_reach_m(btype, grade, H, fall_side=True)
    blind = heap_reach_m(btype, grade, H, fall_side=False)
    others = ("E", "N", "W")
    extra = others[zlib.crc32(str(prim_path).encode("utf-8")) % len(others)]
    return {"S": fall, "E": fall if extra == "E" else blind,
            "N": fall if extra == "N" else blind,
            "W": fall if extra == "W" else blind}


def _heap_reach_for(r, btype, grade, H, prim_path=""):
    """The per-side reach dict `_clear_under_heaps` uses for one heap
    record `r` (one entry of `assemble`'s `records` list).

    MEASURED, WHEN THE BAKE RECORDED IT: `r["reach_m"]`, a {side: float}
    dict copied onto the record from the archetype's own manifest row at
    swap time (a future bake pass — `quake_flow`'s wave-2 rubble routing —
    can put real per-side run-out there once it plans an actual pile
    instead of a discrete-block heap). `r["fall_sides"]` names which of
    the FOUR sides count as a fall/street side for a fill-in when the
    dict does not cover all of them (defaulting to `{"S"}`, the packer's
    own front-door convention, if the record has no opinion either).

    DRAWN, OTHERWISE (`heap_reach_sides`) — today's behaviour, unchanged,
    and still what every kit archetype gets until something populates
    `reach_m` on its manifest row.

    Pure — no pxr, no stage — so `_clear_under_heaps`'s use of measured
    stats is testable on the host without a USD stage at all.
    """
    r_reach = r.get("reach_m")
    if isinstance(r_reach, dict) and r_reach:
        fall_sides = set(r.get("fall_sides") or ("S",))
        return {s: float(r_reach[s]) if s in r_reach else
                heap_reach_m(btype, grade, H, fall_side=(s in fall_sides))
                for s in ("S", "E", "N", "W")}
    return heap_reach_sides(btype, grade, H, prim_path)


def in_reach(cx, cy, W, D, yaw_deg, x, y, reach):
    """(inside, dist_m, side) for a point (x, y) against a building footprint
    centred at (cx, cy), W x D, yawed `yaw_deg` degrees. `reach` is either a
    single number (isotropic — kept as a convenience wrapper for a caller
    that does not care which side) or a {"S", "E", "N", "W"} dict, one reach
    per side (`heap_reach_sides`) — the run-out is not the same on the
    street side as it is against the neighbour's party wall.

    Computed in the building's own yaw frame (local x along W, y along D,
    found by rotating the offset by -yaw). `dist_m` is the ACTUAL metre
    distance from the wall line on whichever side the point falls nearest —
    0 exactly on the wall, positive outside, negative INSIDE the footprint
    (the distance to the NEAREST wall, negated, so a prop dead centre reads
    more "inside" than one a metre past the wall line). `side` names that
    nearest wall, so the caller can look up ITS OWN reach and its own
    H-scaled buried band (see `_heap_action`). `inside` is
    `dist_m <= reach[side]`.

    Symmetric under mirroring through the centre whenever `reach` is a
    single number (or every side shares one value): only `abs(local x/y)` is
    used, so `dist_m` for `(2*cx - x, 2*cy - y)` is identical. With a
    genuinely per-side `reach` dict the mirrored point can name the OPPOSITE
    side, which by construction may carry a different reach — that asymmetry
    is the entire reason this signature exists.
    """
    if not isinstance(reach, dict):
        reach = {"S": reach, "E": reach, "N": reach, "W": reach}
    a = math.radians(-float(yaw_deg))
    dx, dy = x - cx, y - cy
    lx = dx * math.cos(a) - dy * math.sin(a)
    ly = dx * math.sin(a) + dy * math.cos(a)
    hw, hd = W / 2.0, D / 2.0
    ox = abs(lx) - hw          # <= 0 while inside the footprint on this axis
    oy = abs(ly) - hd
    side_x = "E" if lx >= 0.0 else "W"
    side_y = "N" if ly >= 0.0 else "S"
    if ox <= 0.0 and oy <= 0.0:
        side, d = (side_y, oy) if oy >= ox else (side_x, ox)
    elif oy <= 0.0:
        side, d = side_x, ox
    elif ox <= 0.0:
        side, d = side_y, oy
    else:
        side = side_x if ox >= oy else side_y
        d = math.hypot(ox, oy)
    r = float(reach.get(side, 0.0))
    return (d <= r), d, side


def _prop_kind(category):
    """"tree" | "pole" | "car" | None for a placement's `category` string —
    the vocabulary `_heap_action` keys on."""
    if category in _TREE_CATS:
        return "tree"
    if category in _CAR_CATS:
        return "car"
    if category in _POLE_CATS:
        return "pole"
    return None


def _heap_action(kind, dist_m, reach_m, H):
    """"remove" | "lean" | "tip" | "bury" | None for one prop `kind` at
    `dist_m` metres from the wall line (see `in_reach`) on a side whose heap
    reaches `reach_m`, for a building `H` tall.

    Bands are in METRES OF H, not fractions of the reach (agent A's memo):
    0 to 0.3 H is buried/crushed — a tree does not lean there, it is gone; a
    lamppost is not merely knocked over, it is under the pile; a car is sunk
    to its roofline. 0.3 H out to the reach is lighter: a tree still leans, a
    car is untouched (a heap's outer rim does not reach a car's roofline the
    way it reaches a standing post), a lamppost still goes over — a slender
    post has no partial-buried state worth drawing, so POLE tips across the
    WHOLE reach regardless of band.

    Where the side's own reach is SHORTER than 0.3 H — a blind/party-wall
    side, most of the time — there is no room for the outer band at all: the
    whole reach counts as buried, which `min(0.3 * H, reach_m)` gives for
    free.
    """
    if dist_m > reach_m:
        return None
    buried = min(0.3 * float(H), float(reach_m))
    if kind == "tree":
        return "remove" if dist_m < buried else "lean"
    if kind == "pole":
        return "tip"
    if kind == "car":
        return "bury" if dist_m < buried else None
    return None


def _dedupe_actions(entries):
    """`entries` is an iterable of (prim_path, action, payload). Returns
    {prim_path: (action, payload)} — one entry per prim_path, keeping
    whichever action ranks highest on `_ACTION_PRIORITY` (remove > tip > lean
    > bury; first-seen wins a tie) — so a prop two overlapping heaps both
    reach is only ever touched by the stronger one, once."""
    best = {}
    for path, action, payload in entries:
        if not path or action not in _ACTION_PRIORITY:
            continue
        cur = best.get(path)
        if cur is None or _ACTION_PRIORITY[action] > _ACTION_PRIORITY[cur[0]]:
            best[path] = (action, payload)
    return best


def _arch_join(arch_dir, name):
    """`<arch_dir>/<name>` for a local directory OR an `omniverse://` folder
    (os.path.join would keep the URL intact too, but be explicit)."""
    a = str(arch_dir)
    if a.startswith("omniverse://"):
        return a.rstrip("/") + "/" + name
    return os.path.join(a, name)


def _read_text(path):
    """The text of a local file or of an `omniverse://` file (omni.client,
    imported lazily so the host-side tests never need it)."""
    if str(path).startswith("omniverse://"):
        import omni.client as oc
        r, _ver, content = oc.read_file(path)
        if r != oc.Result.OK:
            raise IOError("cannot read {0}: {1}".format(path, r))
        return bytes(memoryview(content)).decode("utf-8")
    with open(path) as fh:
        return fh.read()


def load_manifest(arch_dir):
    """{(style, level): record} from the bake's archetypes.json.

    Every record's `usd` is REBASED onto `arch_dir` by basename. The bake
    writes `os.path.abspath(out)` — wherever it exported at the time, as a
    CONTAINER path — and `assemble` references that string verbatim. The
    library has since been renamed (`assets/archetypes_quake/` ->
    `assets/archetype/`) and mirrored to Nucleus, so the recorded paths
    named a directory that no longer existed and every damaged swap would
    have referenced a missing file (found 2026-08-30, first Isaac run after
    round 4). The manifest is the catalogue; `arch_dir` is where the files
    are — the caller chose it, so it wins. Works for a local directory and
    for an `omniverse://` folder (the manifest is read through omni.client).
    """
    recs = json.loads(_read_text(_arch_join(arch_dir, "archetypes.json")))
    out = {}
    for r in recs:
        r = dict(r)
        u = r.get("usd")
        if u:
            r["usd"] = _arch_join(arch_dir, os.path.basename(str(u)))
        out[(r["style"], r["level"])] = r
    return out


# Foundation-family levels have no numeral (`SETTLE`/`TILT`/`OV`), so they
# cannot ride the `rpartition("_DG")` split below — that only ever fires for
# `DG<n>`. Tried in this order once the DG split comes back empty.
_FOUNDATION_LEVELS = ("SETTLE", "TILT", "OV")


def style_of(usd):
    """`.../bld_office_wide_DG0.usd` -> ("office_wide", "DG0").

    Also parses the foundation family baked by the same launcher for the
    same styles — `bld_office_wide_TILT.usd` -> ("office_wide", "TILT") —
    and a `_vN` variant suffix on either family (`..._DG3_v2.usd` -> level
    "DG3", `..._TILT_v2.usd` -> level "TILT").

    BEFORE THIS FIX: only `_DG<n>` parsed at all. `_mono_pass` skips
    anything `style_of` resolves (`if style or category != "house": continue`
    — a resolved style means "a kit archetype `assemble` already handled",
    not a monolith); `assemble`'s FOUNDATION pass swaps a standing
    building's reference to `bld_<style>_TILT.usd` / `_SETTLE.usd` /
    `_OV.usd` and updates `p["usd"]` to match, but `_mono_pass` runs
    AFTER that pass, over the SAME `placements` list, and re-reads
    `p["usd"]`. With the old parser, `style_of("bld_office_wide_TILT.usd")`
    == `(None, None)` — a TILT-swapped kit building read as an
    unrecognised monolith and `_mono_pass` could tilt/lean it A SECOND
    TIME (`_tilt_prim` composes its matrix onto whatever local transform
    is already there, so this is a real compounding double-transform, not
    a relabelling). Recognising the foundation family here is what makes
    `_mono_pass`'s guard actually skip it.
    """
    base = os.path.basename(str(usd))
    if not base.startswith(ARCH_PREFIX) or not base.endswith(".usd"):
        return None, None
    stem = base[len(ARCH_PREFIX):-4]
    style, _, level = stem.rpartition("_DG")
    if style:
        level = "DG" + level
        if "_v" in level:
            level = level.split("_v")[0]
        return style, level
    for tok in _FOUNDATION_LEVELS:
        style, sep, rest = stem.rpartition("_" + tok)
        if style and sep:
            level = tok + rest              # rest is "" or "_vN"
            if "_v" in level:
                level = level.split("_v")[0]
            return style, level
    return None, None


def _variants(manifest, style, grade):
    """Every baked level for (style, grade): `DG3`, `DG3_v1`, `DG3_v2`..."""
    out = []
    for (st, lv), r in manifest.items():
        if st == style and (lv == grade or lv.startswith(grade + "_v")):
            out.append(r)
    return out


def _region(config):
    w, h = config["layout"]["region_m"]
    return (-float(w) / 2.0, -float(h) / 2.0, float(w) / 2.0, float(h) / 2.0)


def _soft_soil(config, rng):
    """The liquefaction patch: one ellipse on the plate where foundation
    failures concentrate (Adapazari, Christchurch, Niigata: whole districts
    at once, not a scatter). Returns `inside(x, y) -> 0..1` (1 at the centre,
    0 outside), or None when `disaster.soft_soil: false`."""
    dis = config.get("disaster") or {}
    soil = dis.get("soft_soil")
    if soil is False:
        return None
    soil = soil if isinstance(soil, dict) else {}
    x0, y0, x1, y1 = _region(config)
    w, h = x1 - x0, y1 - y0
    c = soil.get("center") or [rng.uniform(x0 + 0.25 * w, x1 - 0.25 * w),
                               rng.uniform(y0 + 0.25 * h, y1 - 0.25 * h)]
    cx, cy = float(c[0]), float(c[1])
    rx = float(soil.get("rx_m", 0.28 * w))
    ry = float(soil.get("ry_m", 0.2 * h))
    ang = math.radians(float(soil.get("angle_deg", rng.uniform(0, 180))))

    def inside(x, y):
        dx, dy = x - cx, y - cy
        u = (dx * math.cos(ang) + dy * math.sin(ang)) / rx
        v = (-dx * math.sin(ang) + dy * math.cos(ang)) / ry
        r = math.hypot(u, v)
        return max(0.0, 1.0 - r * r)
    inside.centre = (cx, cy)
    inside.radii = (rx, ry)
    return inside


def _blocked(p, rec, fall_yaw_deg, H, placements):
    """WHAT a building falling toward `fall_yaw_deg` would land on, or None.
    Sweeps a rectangle H long past the wall line and tests other buildings'
    centres against it; returns the NEAREST one it finds.

    Returns the blocker rather than a bare flag (still falsy when the fall is
    clear, so the OV gate reads the same): a blocked overturn is exactly the
    pair an interaction wants — the block goes over, meets the next one along
    and comes to rest against it instead of flat on the street. `assemble`
    hands those pairs to `_d_interactions`."""
    x, y = float(p["x_m"]), float(p["y_m"])
    W, D = float(rec.get("W", 20.0)), float(rec.get("D", 20.0))
    a = math.radians(fall_yaw_deg)
    ux, uy = math.cos(a), math.sin(a)
    half = max(W, D) / 2.0
    best, bd = None, 1e9
    for q in placements:
        if q is p or q.get("category") != "house":
            continue
        dx, dy = float(q["x_m"]) - x, float(q["y_m"]) - y
        along = dx * ux + dy * uy
        across = abs(-dx * uy + dy * ux)
        if half < along < half + H + 4.0 and across < half + 6.0:
            if along < bd:
                best, bd = q, along
    return best


def assemble(stage, config, placements, arch_dir, seed=11, ssf=1.0,
             tilt_chance=None, verbose=True, foundation_rate=None):
    """Swap placed pristine archetypes for damaged ones by the field.

    Then the FOUNDATION pass: on the soft-soil patch a share of the
    buildings that did not collapse (DG0-DG3) get a baked foundation
    archetype instead — `SETTLE` (level sink), `TILT` (10-30 deg on a raft)
    or `OV` (overturned, one per scene, only where the fall is clear) —
    and a few elsewhere get the mild lean (`_tilt_prim`).

    Returns a stats dict: `buildings`, `tally` (grade -> count),
    `tilted`, `missing`, `foundation`, `soft_soil`.
    """
    import scene_generator as sg

    dis = config.get("disaster") or {}
    field = sg.make_damage_field(dis.get("field") or {"kind": "uniform", "inside": 0.0},
                                 _region(config))
    debris = dis.get("debris") or {}
    if tilt_chance is None:
        tilt_chance = float(debris.get("tilt_chance", 0.25))
    tilt_deg = debris.get("tilt_deg", [3.0, 9.0])
    sink_m = debris.get("sink_m", [0.4, 1.4])
    manifest = load_manifest(arch_dir)
    rng = random.Random(seed + 4242)
    soft = _soft_soil(config, random.Random(seed + 77))
    grade_scale = float(dis.get("grade_scale", 1.0))
    dur_boost = float(dis.get("duration_boost", 1.0))     # research §13; rc types only
    if foundation_rate is None:
        soil_cfg = dis.get("soft_soil") if isinstance(dis.get("soft_soil"), dict) else {}
        foundation_rate = float(soil_cfg.get("rate", 0.55))
    tally, n, tilted, missing = {}, 0, 0, 0
    n_found = {"SETTLE": 0, "TILT": 0, "OV": 0}
    ov_used = False
    records = []
    # Buildings whose OVERTURN the fall sweep refused because a NEIGHBOUR was
    # in the way: `_d_interactions` wants exactly those — the block goes over
    # and comes to rest against the next one along.
    lean_hints = []
    for p in placements:
        style, level = style_of(p.get("usd"))
        if not style or p.get("category") != "house":
            continue
        path = p.get("prim_path")
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            continue
        rec = manifest.get((style, "DG0")) or {}
        btype = rec.get("type") or qf.FAMILY_TYPE.get(rec.get("family", ""), "urm")
        x, y = float(p["x_m"]), float(p["y_m"])
        inten = float(field(x, y))
        grade = qf.level_for_intensity(inten * grade_scale, btype, rng,
                                       duration_boost=dur_boost)
        n += 1
        chosen = None
        if grade != "DG0":
            vs = _variants(manifest, style, grade)
            if not vs:
                # step down until something is baked, so a missing DG4
                # becomes a DG3 rather than a pristine building in the core
                g = int(grade[2])
                while g > 0 and not vs:
                    g -= 1
                    vs = _variants(manifest, style, "DG{0}".format(g))
                missing += 1
            if vs:
                chosen = rng.choice(vs)
                grade = chosen["level"].split("_v")[0]
                refs = prim.GetReferences()
                refs.ClearReferences()
                refs.AddReference(chosen["usd"])
                prim.Load()
                p["usd"] = chosen["usd"]
        # FOUNDATION FAILURE: intact (or lightly damaged) buildings on the
        # strong-shaking side lean and sink. Gated on slenderness the way
        # Adapazarı was, and on the grade — a pancaked building cannot also
        # tilt, and a DG4 archetype was settled flat by PhysX so must not be
        # rolled (the wildfire skill's rule).
        H = float(rec.get("H", 12.0))
        B = max(1.0, min(float(rec.get("W", 20.0)), float(rec.get("D", 20.0))))
        slender = H / B
        # FOUNDATION FAILURE ON THE SOFT-SOIL PATCH. Only buildings still
        # standing (a pancake cannot also tilt, and a baked collapse was
        # settled flat — never roll it). Slender ones tilt or go over, squat
        # ones settle. One overturn per scene, only where the fall lands on
        # nothing (the OV archetype falls toward its own front, -Y).
        soil_w = soft(x, y) if soft else 0.0
        # inside the patch the weight is 0.35-1.0 (not (1-r^2), which fell to
        # nothing by mid-radius and left one settled building in a 250 m
        # scene); a small chance anywhere on the plate as well
        # off the patch: a small chance that scales with the patch's own rate,
        # so an M5.5 (rate 0, no liquefaction) settles nothing at all
        p_found = ((foundation_rate * (0.35 + 0.65 * soil_w)) if soil_w > 0.0
                   else 0.05 * min(1.0, foundation_rate / 0.5))
        if grade in ("DG0", "DG1", "DG2", "DG3") and rng.random() < p_found:
            r = rng.random()
            if slender > 1.5 and not ov_used and r < 0.3 and inten > 0.4 \
                    and btype != "rc_glass" and H <= 36.0:
                lvl = "OV"
            elif slender > 1.0 and r < 0.65:
                lvl = "TILT"
            else:
                lvl = "SETTLE"
            if lvl == "OV" and _blocked(p, rec, float(p.get("yaw_deg", 0.0)) - 90.0,
                                        H, placements):
                lvl = "TILT"
                lean_hints.append(len(records))
            vs = _variants(manifest, style, lvl)
            if not vs and lvl != "SETTLE":
                lvl = "SETTLE"
                vs = _variants(manifest, style, lvl)
            if vs:
                chosen = rng.choice(vs)
                refs = prim.GetReferences()
                refs.ClearReferences()
                refs.AddReference(chosen["usd"])
                prim.Load()
                p["usd"] = chosen["usd"]
                if lvl == "OV":
                    ov_used = True
                n_found[lvl] += 1
                tally[lvl] = tally.get(lvl, 0) + 1
                records.append(dict(style=style, x=x, y=y, intensity=round(inten, 3),
                                    grade=lvl, prim=path))
                continue
        p_tilt = tilt_chance * inten * (1.3 if slender > 1.6 else 0.6)
        if grade in ("DG0", "DG1", "DG2", "DG3") and rng.random() < p_tilt:
            deg = rng.uniform(float(tilt_deg[0]), float(tilt_deg[1]))
            snk = rng.uniform(float(sink_m[0]), float(sink_m[1]))
            _tilt_prim(stage, prim, p, rec, deg, snk, rng, ssf,
                       bounds=_c_plate_bounds(config, ssf))
            tilted += 1
            grade = grade + "+tilt"
        tally[grade] = tally.get(grade, 0) + 1
        records.append(dict(style=style, x=x, y=y, intensity=round(inten, 3),
                            grade=grade, prim=path))
    if verbose:
        print("[quake] {0} buildings: {1}; {2} mild-tilted, {3} grade(s) stepped down "
              "for a missing archetype".format(
                  n, ", ".join("{0}={1}".format(k, v) for k, v in sorted(tally.items())),
                  tilted, missing))
        if soft:
            print("[quake] soft-soil patch at ({0:.0f}, {1:.0f}) radii {2:.0f} x {3:.0f} m: "
                  "SETTLE {4}, TILT {5}, OV {6}".format(
                      soft.centre[0], soft.centre[1], soft.radii[0], soft.radii[1],
                      n_found["SETTLE"], n_found["TILT"], n_found["OV"]))
    # MONOLITHS (asset set urban_quake_v2): standalone buildings that are not
    # kit archetypes. Rigid bodies only — see `_mono_pass`.
    try:
        n_mono = _mono_pass(stage, config, placements, field, grade_scale, rng, ssf,
                            records, tally, bounds=_c_plate_bounds(config, ssf),
                            verbose=verbose)
        n += n_mono
    except Exception as exc:
        import traceback
        traceback.print_exc()
        print("[quake] monolith pass FAILED: {0}".format(exc))
    stats = {"buildings": n, "tally": tally, "tilted": tilted, "missing": missing,
             "records": records, "foundation": n_found,
             "soft_soil": (soft.centre + soft.radii) if soft else None}
    # HEAP CLEARANCE: trees, lamps/signs/hydrants/bins and cars caught under a
    # DG4/DG5 pile (or a monolith ruin swap) — see `_clear_under_heaps`. Runs
    # BEFORE `_d_interactions` so a lamppost this pass already tipped is not
    # read as a live neighbour by a lean-on/collapse-onto pair. Guarded like
    # the monolith and interaction passes: a failure here must not cost the
    # scene its grades.
    try:
        stats["cleared"] = _clear_under_heaps(
            stage, placements, records, manifest, ssf, rng,
            bounds=_c_plate_bounds(config, ssf), verbose=verbose)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        print("[quake] heap clearance FAILED: {0}".format(exc))
        stats["cleared"] = {"error": str(exc)}
    # BATCH D: make a few pairs of buildings touch (see `_d_interactions`).
    # Guarded, because it is the one pass here that runs PhysX at assembly:
    # a failure must not cost the scene its grades.
    try:
        stats["interactions"] = _d_interactions(
            stage, config, stats, placements, arch_dir, ssf=ssf, seed=seed,
            hints=lean_hints, verbose=verbose)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        print("[quake] interactions FAILED: {0}".format(exc))
        stats["interactions"] = {"pairs": 0, "error": str(exc)}
    return stats


# ---------------------------------------------------------------------------
# THE MILD LEAN, AND ITS GROUND (agent C)
# ---------------------------------------------------------------------------
# Round-1 review: some leaning buildings had disturbed soil round them and some
# had none. The ones that had it were the baked SETTLE / TILT / OV archetypes,
# which carry their ground INSIDE the archetype (`quake_flow._c_ground_
# response` runs in the recipe). The ones that had none were these: a rigid
# transform on a referenced PRISTINE archetype, with nothing authored round it
# at all. So the same builder is called here, from the placement's own frame.
GROUND_MARGIN_M = 2.0     # every ground effect stays this far inside the plate
_C_MATS = {}
_C_TAG = [0]


def _c_plate_bounds(config, ssf=1.0, margin=None):
    """(x0, y0, x1, y1) of the plate, inset by `margin` metres, in the frame
    the ground helpers author in (city-local metres x `ssf`; the launcher
    offsets the parent afterwards).

    Everything scattered on the ground is clamped to this. The soft-soil
    ellipse is drawn from the plate size and can cross the plate edge, and a
    200 m two-city run put four sand boils on bare ground outside the city."""
    mg = GROUND_MARGIN_M if margin is None else float(margin)
    x0, y0, x1, y1 = _region(config)
    s = float(ssf)
    return ((x0 + mg) * s, (y0 + mg) * s, (x1 - mg) * s, (y1 - mg) * s)


def _c_mats(stage, parent):
    """`quake_flow.materials` once per (stage, parent). It is idempotent on
    the stage, so this only saves the reference-resolve cost."""
    key = (id(stage), str(parent))
    out = _C_MATS.get(key)
    if out is None:
        out = qf.materials(stage, str(parent))
        _C_MATS[key] = out
    return out


def _c_tag(prefix="t"):
    """A unique authoring tag per city building (prim names are
    `<tag>_<uid>` and every building gets a fresh uid counter)."""
    _C_TAG[0] += 1
    return "{0}{1}".format(prefix, _C_TAG[0])


def _c_mass(p, rec, ssf=1.0):
    """A `quake_flow` mass frame for a PLACED archetype, out of the manifest
    record and the placement. The city has no `describe()` — the building is
    one referenced prim — and the ground helpers only need the footprint."""
    W = float(rec.get("W", 20.0)) * ssf
    D = float(rec.get("D", 20.0)) * ssf
    H = float(rec.get("H", 12.0)) * ssf
    z0 = float(p.get("z_m", 0.0)) * ssf
    return {"cx": float(p["x_m"]) * ssf, "cy": float(p["y_m"]) * ssf,
            "yaw": float(p.get("yaw_deg", 0.0)), "W": W, "D": D, "z0": z0,
            "top": z0 + H, "levels": [z0], "module": 4.0}


def _c_tilt_ground(stage, base, m, M, rng, geom=None, mats=None, tag=None,
                   bounds=None, raft=True, scope=None):
    """The ground response round a city building that leaned or sank, authored
    as SIBLINGS of the placed archetype (under `<base>/quake_tilt`) so it does
    not ride the archetype's transform. `base` is the placement scope, which
    is at the identity — the same assumption `_tilt_prim` already makes."""
    from pxr import Sdf, UsdGeom

    scope = scope or (str(base) + "/quake_tilt")
    UsdGeom.Scope.Define(stage, Sdf.Path(scope))
    kw = {}
    if geom:
        kw = dict(low_side=geom["low"], drop_m=geom["drop"], rise_m=geom["rise"])
    return qf._c_ground_response(
        stage, m, M=M, raft=raft, parent=scope,
        mats=mats if mats is not None else _c_mats(stage, base),
        rng=rng, tag=tag or _c_tag(), bounds=bounds, **kw)


def _tilt_prim(stage, prim, p, rec, deg, sink, rng, ssf, bounds=None,
               ground=True, side=None):
    """The city's MILD LEAN: a placed archetype leans TOWARD one of its sides,
    sinks, and gets the ground response round it.

    Round 1 rotated by -theta about the base edge on the chosen side, which
    (measured — `quake_flow._c_read_M`) leans the building AWAY from that side
    and never lifts anything out of the ground. `_c_tilt_matrix` owns the
    matrix now, shared with `quake_flow.r_tilt_sink`, and pivots INSIDE the
    footprint so the far edge comes clear. Returns its geometry dict."""
    from pxr import Gf, UsdGeom

    m = _c_mass(p, rec, ssf)
    side = side or rng.choice(["S", "E", "N", "W"])
    M, g = qf._c_tilt_matrix(m, side, deg, abs(sink) * ssf,
                             max_drop_m=qf.C_MAX_DROP_M * ssf)
    xf = UsdGeom.Xformable(prim)
    local = UsdGeom.XformCache().GetLocalTransformation(prim)[0]
    tr = Gf.Transform(local * M)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(tr.GetTranslation()))
    q = tr.GetRotation().GetQuat()
    xf.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
    xf.AddScaleOp().Set(Gf.Vec3f(tr.GetScale()))
    if ground:
        _c_tilt_ground(stage, str(prim.GetPath().GetParentPath()), m, M, rng,
                       geom=g, bounds=bounds)
    return g


# ---------------------------------------------------------------------------
# BATCH D — the ground and the gaps: dust halo, fissures, boils, pounding
# ---------------------------------------------------------------------------
# A FLAT pale grey, not a photographed surface: the halo is a film of
# concrete dust over whatever is there, and a pavement map (with its moss)
# projected through the overlay read as a second pavement.
DUST_TEXTURE = "airstack://scene_gen/assets/materials/quake/Dust_Grey.png"


def _bld_masses(records, manifest, placements):
    """[(x, y, yaw, W, D, H, grade)] for every assembled building."""
    out = []
    by_prim = {p.get("prim_path"): p for p in placements}
    for r in records:
        p = by_prim.get(r["prim"]) or {}
        rec = manifest.get((r["style"], "DG0")) or {}
        out.append((float(r["x"]), float(r["y"]), float(p.get("yaw_deg", 0.0)),
                    float(r.get("W", rec.get("W", 20.0))), float(r.get("D", rec.get("D", 20.0))),
                    float(r.get("H", rec.get("H", 12.0))), r["grade"]))
    return out


def _mono_dims(stage, prim, p):
    """(W, D, H) of a placed monolith in ITS OWN yaw frame, from the world
    bound: the layout places at 0/90/180/270, where the world box is exact."""
    from pxr import Usd, UsdGeom

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng_ = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if rng_.IsEmpty():
        return None
    lo, hi = rng_.GetMin(), rng_.GetMax()
    sx, sy, sz = hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]
    yaw = float(p.get("yaw_deg", 0.0)) % 180.0
    if 45.0 < yaw < 135.0:
        sx, sy = sy, sx
    return float(sx), float(sy), float(sz)


def _mono_pass(stage, config, placements, field, grade_scale, rng, ssf, records,
               tally, bounds=None, verbose=True):
    """Damage for buildings that are NOT kit archetypes (standalone monoliths
    from `urban_quake_v2`). They cannot be fractured, so the vocabulary is
    what a rigid body can show — drawn as RC on the same ladder cuts:
      * DG5 on a tower (H >= MONO_RUIN_MIN_H) with a `destroyed` pool:
        the model is swapped for a ruin tower (its own footprint, same spot);
      * DG4-DG5 otherwise: a HEAVY foundation failure — 5-10 deg lean, 0.8-
        1.6 m sink, ground response (Adapazari's intact-but-leaning blocks);
      * DG3: half of them a mild lean (2-4 deg, 0.3-0.5 m);
      * DG0-DG2: untouched (the dust halo of the neighbours still reaches it).
    Records carry W/D/H so the ground pass and the pounding pass see them;
    `mono=True` keeps them out of the live interaction rebuild."""
    ruins = []
    for e in ((config.get("usds") or {}).get("buildings") or {}).get("destroyed") or []:
        u = e.get("usd") if isinstance(e, dict) else e
        if u:
            ruins.append(str(u))
    n = 0
    n_ruin = n_heavy = n_mild = 0
    for p in placements:
        style, _ = style_of(p.get("usd"))
        if style or p.get("category") != "house":
            continue
        path = p.get("prim_path")
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            continue
        dims = _mono_dims(stage, prim, p)
        if not dims:
            continue
        W, D, H = (v / ssf for v in dims)
        x, y = float(p["x_m"]), float(p["y_m"])
        inten = float(field(x, y))
        grade = qf.level_for_intensity(inten * grade_scale, "rc", rng,
                                       duration_boost=float((config.get("disaster") or {}).get("duration_boost", 1.0)))
        rec = {"W": W, "D": D, "H": H}
        n += 1
        label = grade
        if grade == "DG5" and ruins and H >= MONO_RUIN_MIN_H:
            u = rng.choice(ruins)
            refs = prim.GetReferences()
            refs.ClearReferences()
            refs.AddReference(u)
            prim.Load()
            p["usd"] = u
            n_ruin += 1
        elif grade in ("DG4", "DG5"):
            deg = rng.uniform(*MONO_HEAVY_DEG)
            snk = rng.uniform(*MONO_HEAVY_SINK)
            _tilt_prim(stage, prim, p, rec, deg, snk, rng, ssf, bounds=bounds)
            label = "TILT" if grade == "DG5" else "DG4+tilt"
            n_heavy += 1
        elif grade == "DG3" and rng.random() < MONO_MILD_P:
            deg = rng.uniform(*MONO_MILD_DEG)
            snk = rng.uniform(*MONO_MILD_SINK)
            _tilt_prim(stage, prim, p, rec, deg, snk, rng, ssf, bounds=bounds)
            label = "DG3+tilt"
            n_mild += 1
        tally[label] = tally.get(label, 0) + 1
        records.append(dict(style="mono", x=x, y=y, intensity=round(inten, 3),
                            grade=label, prim=path, W=round(W, 1), D=round(D, 1),
                            H=round(H, 1), mono=True))
    if verbose and n:
        print("[quake] {0} monolith(s): {1} ruin swap(s), {2} heavy lean(s), {3} mild "
              "lean(s)".format(n, n_ruin, n_heavy, n_mild))
    return n


# monolith knobs (see `_mono_pass`)
MONO_RUIN_MIN_H = 35.0          # m: only a tower becomes a ruin tower
MONO_HEAVY_DEG = (5.0, 10.0)    # Adapazari 1999: 4-12 deg on intact blocks
MONO_HEAVY_SINK = (0.8, 1.6)
MONO_MILD_P = 0.5
MONO_MILD_DEG = (2.0, 4.0)
MONO_MILD_SINK = (0.3, 0.5)


# ---------------------------------------------------------------------------
# HEAP CLEARANCE, pxr half (agent F)
# ---------------------------------------------------------------------------
# Round-3 review (`b0_apartment_tall_DG5_obl.png`, `b3_highrise_04_DG5_obl.
# png`): street trees stood untouched THROUGH the rubble pile of a totally
# collapsed building, a bus sat clean at the toe, lampposts stood inside the
# pile. A real pile buries, tips and crushes what is under it. This runs
# AFTER the grade loop and the monolith pass (it needs both — a mono ruin
# swap throws a heap exactly like a kit DG5 does) and BEFORE
# `_d_interactions`, so a lamppost this pass has already tipped is not read
# as a live neighbour by a lean-on/collapse-onto pair.
def _lean_matrix(pivot, out_x, out_y, deg, sink_m, ssf):
    """Rotate `deg` about a horizontal axis through `pivot` (world/stage
    units) so the object's TOP tips toward the unit direction (out_x, out_y),
    then sink `sink_m` metres. Same construction as `_d_lean_matrix` — a
    small positive rotation about axis (-out_y, out_x, 0) moves a point above
    the pivot toward (out_x, out_y), by the right-hand rule."""
    from pxr import Gf

    P = Gf.Vec3d(float(pivot[0]), float(pivot[1]), float(pivot[2]))
    axis = Gf.Vec3d(-out_y, out_x, 0.0)
    R = Gf.Matrix4d().SetRotate(Gf.Rotation(axis, float(deg)))
    return (Gf.Matrix4d().SetTranslate(-P) * R * Gf.Matrix4d().SetTranslate(P)
            * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, -float(sink_m) * ssf)))


def _clear_under_heaps(stage, placements, records, manifest, ssf, rng,
                       bounds=None, verbose=True):
    """Trees, lamps/signs/hydrants/bins and cars caught under a DG4/DG5 heap
    (or a monolith swapped for a ruin) get removed, leaned, tipped or buried
    — see `heap_reach_m` / `_heap_action` for the ladder. Everything else on
    the sidewalk (benches, cafe sets, manholes, ...) is left alone; it is not
    in the brief and a flush-with-ground prop like a manhole is not something
    a heap tips over anyway.

    Returns {"trees_removed", "trees_leaned", "lamps_tipped", "cars_buried"}.
    """
    from pxr import Gf, Usd, UsdGeom

    zero = {"trees_removed": 0, "trees_leaned": 0, "lamps_tipped": 0,
            "cars_buried": 0}
    by_prim = {p.get("prim_path"): p for p in placements if p.get("prim_path")}

    # One heap per record with a pile: (cx, cy, W, D, yaw, reach_dict, H).
    # `reach_dict` is per-side (`_heap_reach_for`: measured off the record
    # when it has `reach_m`, drawn via `heap_reach_sides` otherwise) — the
    # fall side reaches much further than a blind/party-wall side does.
    heaps = []
    for r in records:
        grade = str(r.get("grade", ""))
        if r.get("mono"):
            if grade != "DG5":
                continue                   # a mono heavy-lean has no pile
            btype = "rc"                    # `_mono_pass` always scores as rc
            W = float(r.get("W", 20.0))
            D = float(r.get("D", 20.0))
            H = float(r.get("H", 12.0))
        else:
            if grade not in ("DG4", "DG5"):
                continue
            rec0 = manifest.get((r.get("style"), "DG0")) or {}
            btype = rec0.get("type") or qf.FAMILY_TYPE.get(rec0.get("family", ""), "urm")
            W = float(rec0.get("W", 20.0))
            D = float(rec0.get("D", 20.0))
            H = float(rec0.get("H", 12.0))
        prim_path = r.get("prim") or ""
        p = by_prim.get(prim_path) or {}
        yaw = float(p.get("yaw_deg", 0.0))
        # THE MEASURED REACH LIVES ON THE SWAPPED LEVEL'S MANIFEST ROW, not on
        # the `records` entry `assemble` wrote (style / x / y / grade / prim
        # only). Find the row for the level this building actually got — the
        # placement's `usd` names the variant — and hand `_heap_reach_for` a
        # copy of the record carrying that row's `reach_m` / `fall_sides`
        # (written by the bake's `_rubble_fields`); an old manifest without
        # them falls through to the draw exactly as before.
        rr = r
        if not r.get("mono") and manifest:
            rows = _variants(manifest, r.get("style"), grade)
            row = next((v for v in rows if v.get("usd") == p.get("usd")),
                       rows[0] if rows else None)
            if row and (row.get("reach_m") or row.get("fall_sides")):
                rr = dict(r)
                for k in ("reach_m", "fall_sides", "crown_m"):
                    if row.get(k) is not None and rr.get(k) is None:
                        rr[k] = row[k]
        heaps.append((float(r["x"]), float(r["y"]), W, D, yaw,
                      _heap_reach_for(rr, btype, grade, H, prim_path), H))

    if not heaps:
        if verbose:
            print("[quake] cleared under heaps: 0 trees removed, 0 leaned, "
                  "0 lamps tipped, 0 cars buried")
        return zero

    # Every (prop, heap) pair the prop is in reach of, so `_dedupe_actions`
    # can pick the strongest outcome when two piles overlap the same prop.
    entries = []
    for p in placements:
        kind = _prop_kind(p.get("category"))
        path = p.get("prim_path")
        if kind is None or not path:
            continue
        x, y = float(p.get("x_m", 0.0)), float(p.get("y_m", 0.0))
        for (cx, cy, W, D, yaw, reach_sides, H) in heaps:
            inside, dist_m, side = in_reach(cx, cy, W, D, yaw, x, y, reach_sides)
            if not inside:
                continue
            action = _heap_action(kind, dist_m, reach_sides[side], H)
            if action is None:
                continue
            entries.append((path, action, (kind, cx, cy)))

    winners = _dedupe_actions(entries)
    counts = dict(zero)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    for path, (action, (kind, cx, cy)) in winners.items():
        try:
            prim = stage.GetPrimAtPath(path) if path else None
            if not prim or not prim.IsValid():
                continue
            is_instancer = prim.GetTypeName() == "PointInstancer"
            if action == "remove":
                if is_instancer:
                    pi = UsdGeom.PointInstancer(prim)
                    proto = pi.GetProtoIndicesAttr().Get()
                    n = len(proto) if proto else 0
                    ids_attr = pi.GetIdsAttr()
                    ids = list(ids_attr.Get()) if ids_attr and ids_attr.Get() \
                        else list(range(n))
                    pi.CreateInvisibleIdsAttr().Set(ids)
                else:
                    prim.SetActive(False)
                counts["trees_removed"] += 1
                continue
            if is_instancer:
                if verbose:
                    print("[quake] heap clearance: {0} is a PointInstancer — "
                          "cannot {1} an instance, only remove ({2}) is "
                          "supported".format(path, action, kind))
                continue
            rng_ = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng_.IsEmpty():
                continue
            lo, hi = rng_.GetMin(), rng_.GetMax()
            px, py, pz = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), lo[2]
            if bounds is not None:
                bx0, by0, bx1, by1 = bounds
                if not (bx0 <= px <= bx1 and by0 <= py <= by1):
                    continue                # pivot off the plate: skip
            dx, dy = px - cx * ssf, py - cy * ssf
            norm = math.hypot(dx, dy) or 1.0
            ox, oy = dx / norm, dy / norm
            if action == "lean":            # tree: away from the building
                deg, sink = rng.uniform(25.0, 45.0), 0.2
                out_x, out_y = ox, oy
            elif action == "tip":           # pole/sign/hydrant/bin: away
                deg, sink = rng.uniform(70.0, 85.0), 0.0
                out_x, out_y = ox, oy
            else:                           # bury: car, toward the building
                deg, sink = rng.uniform(8.0, 15.0), 0.35
                out_x, out_y = -ox, -oy
            M = _lean_matrix((px, py, pz), out_x, out_y, deg, sink, ssf)
            xf = UsdGeom.Xformable(prim)
            local = UsdGeom.XformCache().GetLocalTransformation(prim)[0]
            tr = Gf.Transform(local * M)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(tr.GetTranslation()))
            q = tr.GetRotation().GetQuat()
            xf.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
            xf.AddScaleOp().Set(Gf.Vec3f(tr.GetScale()))
            if action == "lean":
                counts["trees_leaned"] += 1
            elif action == "tip":
                counts["lamps_tipped"] += 1
            else:
                counts["cars_buried"] += 1
        except Exception as exc:
            if verbose:
                print("[quake] heap clearance: {0} on {1} failed ({2}), "
                      "skipped".format(action, path, exc))
            continue
    if verbose:
        print("[quake] cleared under heaps: {trees_removed} trees removed, "
              "{trees_leaned} leaned, {lamps_tipped} lamps tipped, "
              "{cars_buried} cars buried".format(**counts))
    return counts


def ground_effects(stage, config, stats, placements, arch_dir, parent, ssf,
                   seed=11, dust=True, fissures=True, boils=True, pounding=True,
                   verbose=True):
    """Author what the quake did to the GROUND and the GAPS, after `assemble`.

    * DUST HALO: every DG4/DG5 building throws a grey concrete-dust film over
      the ground within 1-2 H (the research's halo), as a translucent overlay
      through `ground.build_overlay` — so it needs BOTH fractional-cutout
      flags on the app command line AND re-asserted after the stage loads,
      exactly as the burn scar does (wildfire skill). Roads, walks and roofs
      under it stay legible; the whiteness is the read.
    * FISSURES: on the soft-soil patch, meandering tension cracks 0.3-0.9 m
      wide as dark strips, roughly parallel to the patch's long axis.
    * BOILS: sand-boil fans across the patch on open ground.
    * POUNDING: where two buildings stand within 1.5 m, a vertical scar on the
      taller one at the shorter one's roof line and a wedge of rubble in the
      gap (Mexico City 1985: 40 % of buildings collided).
    """
    from pxr import Sdf, UsdGeom
    import scene_generator as sg
    from . import ground, quake_flow as qf

    rng = random.Random(seed + 313)
    manifest = load_manifest(arch_dir)
    blds = _bld_masses(stats.get("records", []), manifest, placements)
    x0, y0, x1, y1 = _region(config)
    mats = qf.materials(stage, parent)
    scope = parent + "/quake_ground"
    UsdGeom.Scope.Define(stage, Sdf.Path(scope))
    n_dust = n_fis = n_boil = n_pound = 0

    # a minimal ctx so the per-building helpers can author here
    def _ctx(tag):
        return {"stage": stage, "parent": scope, "rng": rng, "mats": mats, "tag": tag,
                "authored": [], "static_extra": [], "loose": [], "velocity": {},
                "notes": [], "n_uid": 0, "info": {"type": "rc"}}

    dcfg = (config.get("disaster") or {}).get("dust") or {}
    reach5 = float(dcfg.get("reach_h5", 1.0))
    reach4 = float(dcfg.get("reach_h4", 0.5))
    op_max = float(dcfg.get("opacity_max", 0.42))
    if dust:
        heavy = [b for b in blds if b[6] in ("DG4", "DG5", "OV")]

        def coverage_at(x, y):
            c = 0.0
            for bx, by, _yaw, W, D, H, g in heavy:
                r = math.hypot(x - bx, y - by) - max(W, D) * 0.5
                # ~1.0 H / 0.5 H, not the research's outer 2 H: at 2 H eleven
                # collapses whitened the whole 250 m plate and buried the
                # road markings. The halo has to leave clean ground between
                # piles for the piles to read as piles.
                reach = (reach5 if g in ("DG5", "OV") else reach4) * H
                if r < reach:
                    c += (0.8 if g in ("DG5", "OV") else 0.3) * max(0.0, 1.0 - max(0.0, r) / reach) ** 1.8
            return min(1.0, c)
        if heavy:
            made = ground.build_overlay(
                stage, coverage_at, (x0, y0, x1, y1), ssf, 0.035,
                material_parent=scope, root=scope + "/dust", cell_m=3.0, bands=10,
                tile_m=18.0, op_range=(0.06, op_max),
                texture=sg._join_asset_root(DUST_TEXTURE, ""), verbose=verbose)
            n_dust = len(made)

    soft = stats.get("soft_soil")
    if soft and (fissures or boils):
        cx, cy, rx, ry = soft
        ctx = _ctx("soil")
        # ON THE PLATE. The soft-soil ellipse is 0.36 w x 0.24 h round a centre
        # drawn anywhere in the middle half, so it crosses the plate edge on a
        # small region: a 200 m two-city run put four boils on bare ground
        # outside the city. A boil fan is up to ~3.8 m across, so its CENTRE
        # needs that much more margin than a fissure segment.
        fx0, fy0, fx1, fy1 = _c_plate_bounds(config)
        ctx["bounds"] = _c_plate_bounds(config, margin=GROUND_MARGIN_M + 4.0)
        if fissures:
            # 2-4 cracks, each a chain of thin dark boxes following a noisy
            # line roughly along the patch's long axis
            n_f = rng.randrange(2, 5)
            ang0 = 0.0 if rx >= ry else 90.0
            for k in range(n_f):
                off = rng.uniform(-0.6, 0.6) * min(rx, ry)
                length = rng.uniform(0.8, 1.6) * max(rx, ry)
                a = math.radians(ang0 + rng.uniform(-25, 25))
                px = cx - math.cos(a) * length / 2.0 - math.sin(a) * off
                py = cy - math.sin(a) * length / 2.0 + math.cos(a) * off
                step = 3.0
                w = rng.uniform(0.3, 0.9)
                heading = a
                for i in range(int(length / step)):
                    heading += math.radians(rng.uniform(-14, 14))
                    nx, ny = px + math.cos(heading) * step, py + math.sin(heading) * step
                    mx, my = (px + nx) / 2.0, (py + ny) / 2.0
                    if not (fx0 < mx < fx1 and fy0 < my < fy1):
                        break
                    path = "{0}/fissure_{1}".format(scope, qf._uid(ctx))
                    qf._box(stage, path, mx, my, 0.02, step * 1.15, w * rng.uniform(0.7, 1.3),
                            0.06, math.degrees(heading), mats["rebar"])
                    # an upthrown lip on one side
                    if rng.random() < 0.5:
                        lp = "{0}/lip_{1}".format(scope, qf._uid(ctx))
                        sgn = rng.choice((-1, 1))
                        qf._box(stage, lp, mx - math.sin(heading) * sgn * w, my + math.cos(heading) * sgn * w,
                                0.06, step * 1.1, 0.5, 0.12, math.degrees(heading), mats["concrete"])
                    px, py = nx, ny
                    n_fis += 1
        if boils:
            n_b = int(rx * ry / 1400.0) + 4
            m = {"cx": cx, "cy": cy, "yaw": 0.0, "W": 1.0, "D": 1.0, "z0": 0.0, "top": 6.0}
            # scatter over the ellipse, avoiding building footprints
            for k in range(n_b):
                for _try in range(8):
                    u, v = rng.uniform(-1, 1), rng.uniform(-1, 1)
                    if u * u + v * v > 1.0:
                        continue
                    bx, by = cx + u * rx, cy + v * ry
                    if not qf._c_ok(ctx, bx, by):
                        continue
                    if any(abs(bx - b[0]) < b[3] / 2.0 + 2.0 and abs(by - b[1]) < b[4] / 2.0 + 2.0
                           for b in blds):
                        continue
                    mm = dict(m, cx=bx, cy=by)
                    qf._ejecta(ctx, mm, 1, reach_frac=0.3)
                    n_boil += 1
                    break

    if pounding:
        # POUNDING NEEDS NEIGHBOURS THAT NEARLY TOUCH, and the first version of
        # this test could not find them for two reasons. It compared |dx|
        # against W and |dy| against D — ignoring YAW, so on the half of a
        # downtown block laid at yaw 90 it was measuring the wrong side — and
        # it wanted a gap under 1.5 m while `packing.building_gap_m` in the
        # preset is 6.0. City 9 printed "0 pounding scar(s)" for both reasons
        # at once. `qf.d_rect_gap` is the yaw-aware separating-axis distance;
        # the threshold is a knob; and when nothing fires the CLOSEST pair in
        # the scene is printed, so the next person can see whether the layout
        # can produce pounding at all.
        ctx = _ctx("pound")
        pcfg = (config.get("disaster") or {}).get("pounding") or {}
        max_gap = float(os.environ.get("QUAKE_POUND_GAP", "").strip()
                        or pcfg.get("max_gap_m", 1.5))
        min_face = float(pcfg.get("min_face_m", 3.0))
        p_fire = float(pcfg.get("chance", 0.85))
        cap = int(pcfg.get("max_pairs", 24))
        closest, closest_pair = 1e9, None
        cands = []
        for i in range(len(blds)):
            for j in range(i + 1, len(blds)):
                a, b = blds[i], blds[j]
                ra = (a[0], a[1], a[2], a[3], a[4])
                rb = (b[0], b[1], b[2], b[3], b[4])
                gap, nrm = qf.d_rect_gap(ra, rb)
                face = qf.d_rect_overlap(ra, rb, nrm)
                if face > min_face and -0.6 < gap < closest:
                    closest, closest_pair = gap, (i, j)
                if a[6] in ("DG5", "OV") or b[6] in ("DG5", "OV"):
                    continue          # a heap does not hammer anything
                if gap > max_gap or gap < -0.6 or face < min_face:
                    continue
                cands.append((gap, i, j, nrm))
        cands.sort(key=lambda q: q[0])
        for gap, i, j, nrm in cands[:cap]:
            if rng.random() > p_fire:
                continue
            a, b = blds[i], blds[j]
            ma = qf.d_box_mass(a[0], a[1], a[2], a[3], a[4], a[5])
            mb = qf.d_box_mass(b[0], b[1], b[2], b[3], b[4], b[5])
            qf.d_pound_marks(ctx, ma, mb, gap=max(0.0, gap), normal=nrm)
            n_pound += 1
        if verbose and not n_pound:
            print("[quake] pounding: no pair within {0:.2f} m (closest facing "
                  "pair in this scene: {1}). Pounding needs party-wall "
                  "neighbours — lower `packing.building_gap_m` (the downtown "
                  "earthquake preset holds 6.0 m) or raise QUAKE_POUND_GAP."
                  .format(max_gap,
                          "{0:.2f} m".format(closest) if closest_pair else "none"))
    if verbose:
        print("[quake] ground: {0} dust band(s), {1} fissure segment(s), {2} boil(s), "
              "{3} pounding scar(s)".format(n_dust, n_fis, n_boil, n_pound))
    return {"dust_bands": n_dust, "fissures": n_fis, "boils": n_boil, "pounding": n_pound}


# ---------------------------------------------------------------------------
# BATCH D — BUILDINGS THAT TOUCH EACH OTHER, at city scale
# ---------------------------------------------------------------------------
# The pair recipes (`quake_flow.r_lean_on`, `r_collapse_onto`, `r_pounding`)
# cannot be baked: their geometry belongs to the PAIR — the gap, the two
# heights, the contact angle — and an archetype is one building. So this pass
# runs after the grades are drawn, finds the pairs where a foundation failure
# would actually reach a neighbour, and makes them interact. Two paths:
#
#   LIVE (`QUAKE_INTERACT=N`, default 3): the leaner is REBUILT from the kit
#     at its own place, the archetype reference switched off, `r_lean_on` run
#     on it with the neighbour as a box, and the whole lot settled with PhysX
#     once at the end. This is the bench pipeline, at assembly, for N
#     buildings — the honest cost is a bench column each (~10-30 s of fracture
#     plus its share of one settle; a 3-pair scene measured on the bench at
#     ~25 s fracture + ~60 s settle for 1.5k bodies).
#   GEOMETRIC (`QUAKE_INTERACT=0`, or for pairs past the live cap): the placed
#     archetype is rigid-rotated onto its neighbour and the contact is
#     AUTHORED — spall bands on both faces, the crushed cornice, the wedge in
#     the gap, the berm on the side that sank. No fracture, no physics,
#     milliseconds per pair. It reads from the air and from the street; what
#     it cannot do is show broken material at the contact.
#
# Baking "pair archetypes" is the third option and is NOT built: it would need
# one bake per (style, style, gap, side) and the gap is continuous.
def _d_lean_matrix(m, side, deg, lift, z0, ssf):
    """The rigid transform of a lean-on: rotate `deg` about the base edge
    OPPOSITE `side` (so the near edge digs in and the top goes over toward the
    neighbour), then lift so the raft levers out rather than burying a storey.

    THE PIVOT IS BUILT FROM THE LOCAL SIDE NORMAL, not from `_outward`.
    `_outward` is already yaw-rotated and `_to_world` rotates again, so the
    pivot of a yawed building lands at twice its yaw — invisible in the bench
    and the bake (both build at yaw 0) and wrong in the city, where the packer
    lays half a block at yaw 90. (`r_tilt_sink`, `r_tilt_severe` and
    `r_overturn` all still have that form; they only ever run at yaw 0.)"""
    from pxr import Gf

    far = qf._opposite(side)
    B = m["D"] if side in ("S", "N") else m["W"]
    lnx, lny = qf._SIDE_NORMAL[far]
    px, py = qf._to_world(m, lnx * B / 2.0 if far in ("E", "W") else 0.0,
                          lny * B / 2.0 if far in ("S", "N") else 0.0)
    ox, oy = qf._outward(m, side)
    P = Gf.Vec3d(px * ssf, py * ssf, z0 * ssf)
    R = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(-oy, ox, 0.0), abs(deg)))
    return (Gf.Matrix4d().SetTranslate(-P) * R * Gf.Matrix4d().SetTranslate(P)
            * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, lift * ssf)))


def _d_apply_matrix(stage, prim, M):
    """Post-multiply one placed prim's LOCAL transform by M and re-author it
    as translate / orient / scale, exactly as `_tilt_prim` does."""
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(prim)
    local = UsdGeom.XformCache().GetLocalTransformation(prim)[0]
    tr = Gf.Transform(local * M)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(tr.GetTranslation()))
    q = tr.GetRotation().GetQuat()
    xf.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
    xf.AddScaleOp().Set(Gf.Vec3f(tr.GetScale()))


def _d_pairs(blds, hints=None, max_deg=26.0, min_deg=4.0, min_face=4.0,
             crush_m=0.35):
    """Every (leaner, neighbour) pair whose lean would actually reach, best
    first. `blds` is [(x, y, yaw, W, D, H, grade, idx)]; a pair scores on how
    far the block turns (a 2 deg lean is not a picture), on how much façade
    the two share, and on being one of `hints` — the buildings whose OVERTURN
    the `_blocked` sweep refused because a neighbour was in the fall path,
    which is this failure exactly."""
    hints = set(hints or ())
    out = []
    for i, a in enumerate(blds):
        if str(a[6]).split("+")[0] not in ("DG0", "DG1", "DG2", "DG3"):
            continue                       # collapsed / already on a raft
        if "+tilt" in str(a[6]):
            continue                       # `_tilt_prim` already rolled it
        ra = (a[0], a[1], a[2], a[3], a[4])
        for j, b in enumerate(blds):
            if j == i:
                continue
            rb = (b[0], b[1], b[2], b[3], b[4])
            gap, nrm = qf.d_rect_gap(ra, rb)
            if gap < -0.6 or gap > 0.75 * a[5]:
                continue
            face = qf.d_rect_overlap(ra, rb, nrm)
            if face < min_face:
                continue
            ma = qf.d_box_mass(a[0], a[1], a[2], a[3], a[4], a[5])
            side = qf._d_facing_side(ma, nrm[0], nrm[1])
            B = ma["D"] if side in ("S", "N") else ma["W"]
            deg, zc = qf._d_contact_angle(a[5], B, max(0.02, gap) + crush_m,
                                          b[5], max_deg=max_deg)
            kind = "lean"
            if deg is None:
                # NO CONTACT does not mean nothing happens: a block that much
                # taller than its neighbour goes OVER its parapet, and what
                # comes off the top lands on its roof instead (Kobe, Amatrice).
                # It only reaches if a fragment thrown at the settle's speed
                # ceiling covers the gap while it falls the height difference.
                drop = a[5] - b[5]
                if drop < 4.0:
                    continue
                reach = 5.4 * math.sqrt(2.0 * drop / 9.81)
                if gap + 2.0 > reach:
                    continue
                kind, deg, zc = "collapse", 0.0, b[5]
            elif deg < min_deg and gap > 1.0:
                continue
            score = (3.0 if a[7] in hints else 0.0) + min(deg, 22.0) / 22.0 \
                + min(face, 20.0) / 40.0 + (0.5 if kind == "collapse" else 0.0)
            out.append((-score, i, j, side, deg, zc, gap, face, kind))
    out.sort()
    used = set()
    picked = []
    for _s, i, j, side, deg, zc, gap, face, kind in out:
        if i in used or j in used:
            continue                       # one interaction per building
        used.add(i)
        used.add(j)
        picked.append(dict(i=i, j=j, side=side, deg=deg, zc=zc, gap=gap,
                           face=face, kind=kind))
    return picked


D_LEAN_MIN_INTENSITY = 0.6     # field x grade_scale under a lean-on candidate


def _d_interactions(stage, config, stats, placements, arch_dir, parent=None,
                    ssf=1.0, seed=11, live=None, pairs=None, hints=None,
                    settle_steps=1800, verbose=True):
    """Make a few pairs of buildings touch. See the batch note above.

    Returns {"pairs", "live", "geometric", "seconds", "notes"}.
    """
    import time

    from pxr import Sdf, UsdGeom

    t0 = time.time()
    manifest = load_manifest(arch_dir)
    recs = stats.get("records", [])
    by_prim = {p.get("prim_path"): p for p in placements}
    blds = []
    # a bearing failure needs strong shaking under the leaner: gate on the
    # field (times the compiled grade_scale) so an M5.5 city, where nothing
    # collapses, does not get three 22-degree lean-ons (two-city run 5)
    gs = float((config.get("disaster") or {}).get("grade_scale", 1.0))
    for k, r in enumerate(recs):
        if r.get("mono"):
            continue                       # a monolith has no kit to rebuild
        if float(r.get("intensity", 1.0)) * gs < D_LEAN_MIN_INTENSITY:
            continue
        p = by_prim.get(r["prim"]) or {}
        rec = manifest.get((r["style"], "DG0")) or {}
        blds.append((float(r["x"]), float(r["y"]), float(p.get("yaw_deg", 0.0)),
                     float(rec.get("W", 20.0)), float(rec.get("D", 20.0)),
                     float(rec.get("H", 12.0)), r["grade"], k))
    if parent is None:
        for p in placements:
            if p.get("prim_path"):
                parent = str(p["prim_path"]).rsplit("/", 1)[0]
                break
    if not blds or not parent:
        return {"pairs": 0, "live": 0, "geometric": 0, "seconds": 0.0, "notes": []}
    n_live = int(os.environ.get("QUAKE_INTERACT", "").strip() or
                 (3 if live is None else live))
    n_pairs = int(os.environ.get("QUAKE_INTERACT_PAIRS", "").strip() or
                  (pairs if pairs is not None else max(3, n_live)))
    if n_pairs <= 0:
        return {"pairs": 0, "live": 0, "geometric": 0, "seconds": 0.0, "notes": []}
    rng = random.Random(seed + 909)
    picked = _d_pairs(blds, hints=hints)[:n_pairs]
    scope = parent + "/quake_interact"
    UsdGeom.Scope.Define(stage, Sdf.Path(scope))
    mats = qf.materials(stage, parent)
    cache = {}
    notes = []
    loose_all, static_all, vel_all = [], [], {}
    n_l = n_g = 0
    for k, pr in enumerate(picked):
        a, b = blds[pr["i"]], blds[pr["j"]]
        ra, rb = recs[a[7]], recs[b[7]]
        pa = by_prim.get(ra["prim"])
        prim = stage.GetPrimAtPath(ra["prim"]) if ra.get("prim") else None
        if not pa or not prim or not prim.IsValid():
            continue
        ma = qf.d_box_mass(a[0], a[1], a[2], a[3], a[4], a[5])
        mb = qf.d_box_mass(b[0], b[1], b[2], b[3], b[4], b[5])
        nb = qf.d_box_neighbour(mb["cx"], mb["cy"], mb["yaw"], mb["W"], mb["D"],
                                b[5], pr["side"], max(0.02, pr["gap"]))   # H: d_box_mass has no "H" key
        tag = "ix{0}".format(k)
        done = False
        if n_l < n_live:
            try:
                res = _d_live_lean(stage, scope, tag, ra, pa, nb, mats, cache,
                                   ssf, seed + 31 * k,
                                   recipe=("collapse_onto" if pr["kind"] == "collapse"
                                           else "lean_on"), verbose=verbose)
                if res is not None:
                    prim.SetActive(False)          # the archetype steps aside
                    loose_all += res["loose"]
                    static_all += res["static_extra"]
                    vel_all.update(res["velocity"])
                    notes += ["[live] " + q for q in res["notes"]]
                    n_l += 1
                    done = True
            except Exception as exc:
                import traceback
                traceback.print_exc()
                # A HALF-BUILT KIT BUILDING MUST NOT BE LEFT STANDING. The
                # archetype is still active at this point (it is switched off
                # only on success), so the rebuilt shell would be a second
                # building inside the first.
                hp = stage.GetPrimAtPath("{0}/{1}".format(scope, tag))
                if hp and hp.IsValid():
                    hp.SetActive(False)
                print("[quake] interaction {0}: live path failed ({1}); "
                      "falling back to the geometric one".format(tag, exc))
        if not done:
            if pr["kind"] == "collapse":
                notes.append("[geom] " + _d_geom_collapse(
                    stage, scope, tag, ma, nb, ra, prim, mats, rng, pr,
                    manifest, arch_dir))
            else:
                notes.append("[geom] " + _d_geom_lean(stage, scope, tag, ma, nb,
                                                      pa, prim, mats, rng, pr, ssf))
            n_g += 1
        static_all.append(rb.get("prim"))
        kind = "collapse_onto" if pr["kind"] == "collapse" else "lean_on"
        ra["grade"] = str(ra["grade"]) + "+" + pr["kind"]
        ra["interaction"] = kind
        rb["hit_by"] = ra.get("prim")
        stats.setdefault("tally", {})
        key = "LEAN" if pr["kind"] == "lean" else "ONTO"
        stats["tally"][key] = stats["tally"].get(key, 0) + 1
    if loose_all:
        from . import settle
        settle.run(stage, loose_all, [q for q in static_all if q],
                   steps=settle_steps, kick=0.12, rng=random.Random(seed + 5),
                   bake_result=True, velocity_map=vel_all, density=1900.0,
                   max_speed=6.0)
    dt = time.time() - t0
    if verbose:
        print("[quake] interactions: {0} pair(s) — {1} live, {2} geometric, "
              "{3} loose bodies, {4:.1f} s".format(n_l + n_g, n_l, n_g,
                                                   len(loose_all), dt))
        for q in notes:
            print("[quake]     " + q)
    return {"pairs": n_l + n_g, "live": n_l, "geometric": n_g,
            "seconds": round(dt, 1), "notes": notes}


def _d_live_lean(stage, scope, tag, rec, p, nb, mats, cache, ssf, seed,
                 recipe="lean_on", verbose=True):
    """Rebuild one building from the kit where its archetype stands and run
    `lean_on` on it for real — fracture, crushed contact, debris. Returns the
    `wreck_building` ctx (the caller settles every pair together), or None if
    the style is not a kit style (a harvested block shell has no modules)."""
    import scene_generator as sg
    from detail import urban_building as ub
    from pxr import Sdf, UsdGeom

    style = rec["style"]
    if style not in ub.STYLES:
        return None
    x, y = float(rec["x"]), float(rec["y"])
    yaw = float(p.get("yaw_deg", 0.0))
    holder = "{0}/{1}".format(scope, tag)
    UsdGeom.Scope.Define(stage, Sdf.Path(holder))
    # SEED 4 IS THE BAKE'S FAÇADE SEED (`ARCH_SEED`), so the rebuilt building
    # is the same building the archetype library holds for this style.
    pls = ub.build_building(style, x, y, yaw, random.Random(4))
    sg.apply_placements(stage, pls, holder, ssf)
    try:
        ub.apply_glass_tint(stage, pls)
    except Exception:
        pass
    qf.d_set_neighbours(tag, [nb])
    rng = random.Random(seed)
    import numpy as _np
    res = qf.wreck_building(stage, holder, style, pls, x, y, yaw,
                            [(recipe, {})], rng, _np.random.default_rng(seed),
                            mats, tag, fit_storeys=None, mat_cache=cache)
    return res


def _d_geom_lean(stage, scope, tag, ma, nb, p, prim, mats, rng, pr, ssf):
    """The no-physics path: rotate the placed archetype onto its neighbour and
    AUTHOR the contact. Everything on the leaner is authored first and carried
    through the same matrix; everything on the neighbour, in the gap and on
    the ground is authored after, in world."""
    side, deg, zc, gap = pr["side"], pr["deg"], pr["zc"], pr["gap"]
    mb = nb["m"]
    B = ma["D"] if side in ("S", "N") else ma["W"]
    dig = B * math.sin(math.radians(deg))
    lift = 0.45 * dig
    z_contact = zc + lift
    if z_contact > mb["top"] - 0.3:
        lift = max(0.0, mb["top"] - 0.3 - zc)
        z_contact = zc + lift
    ctx = {"stage": stage, "parent": scope, "rng": rng, "mats": mats,
           "tag": tag, "authored": [], "static_extra": [], "loose": [],
           "velocity": {}, "notes": [], "n_uid": 0,
           "info": {"type": "urm"}, "neighbours": [nb]}
    span = qf._d_span(ma, side, mb) or (-ma["W"] / 4.0, ma["W"] / 4.0)
    st_h = 3.4
    # 1) the leaner's own crushed corner, authored in its PRE-lean pose and
    #    carried round by the same matrix as the archetype
    on_leaner = qf._d_face_band(ctx, ma, side, zc, span, height=(0.8, 1.6),
                                n_seg=(3, 6), thick=0.16, tag="leanhit")
    on_leaner += qf._d_face_band(ctx, ma, side, zc - st_h, span,
                                 height=(0.4, 0.9), n_seg=(2, 5), tag="leanhit")
    M = _d_lean_matrix(ma, side, deg, lift, 0.0, ssf)
    qf._transform_prims(stage, on_leaner, M)
    _d_apply_matrix(stage, prim, M)
    # 2) the neighbour, unmoved: the contact band, the cornice, the ghost of a
    #    party wall if they were touching
    nside = qf._d_facing_side(mb, -(qf._outward(ma, side)[0]),
                              -(qf._outward(ma, side)[1]))
    nspan = qf._d_span(mb, nside, ma) or (-4.0, 4.0)
    qf._d_face_band(ctx, mb, nside, z_contact, nspan, height=(0.7, 1.4),
                    n_seg=(3, 6), thick=0.15, tag="hit")
    if z_contact > mb["top"] - 1.8 * st_h:
        qf._d_face_band(ctx, mb, nside, mb["top"] - 0.5, nspan,
                        height=(0.5, 1.0), n_seg=(2, 5), tag="cornice")
    if gap < 0.6:
        qf._d_party_ghost(ctx, mb, nside, nspan, list(mb["levels"][1:]))
    # 3) the gap, the street and the ground
    qf._d_gap_debris(ctx, ma, side, span, max(0.35, gap) + 0.6,
                     depth_m=rng.uniform(0.5, 1.0), spill_m=rng.uniform(2.0, 3.6),
                     glass=0.16)
    qf._d_ground_response(ctx, ma, side, dig * 0.55, lift_m=lift)
    return ("lean_on {0:.1f} deg toward {1} onto its neighbour "
            "(gap {2:.2f} m, contact z {3:.1f} m, {4:.0f} m of shared façade)"
            .format(deg, side, gap, z_contact, pr["face"]))


def _d_geom_collapse(stage, scope, tag, ma, nb, rec, prim, mats, rng, pr,
                     manifest, arch_dir):
    """The no-physics COLLAPSE-ONTO: the taller building's archetype is bumped
    to a heavier grade if the library has one (its own top has to look failed
    from somewhere), and the material it shed is authored ON the neighbour's
    roof, in the gap and over the neighbour's far parapet.

    WHAT THIS CANNOT DO is punch the neighbour's roof: that roof is inside a
    referenced archetype, and cutting a hole in it needs the kit modules —
    i.e. the live path. From the air the heap still reads; the hole does not."""
    side, gap = pr["side"], pr["gap"]
    mb = nb["m"]
    ctx = {"stage": stage, "parent": scope, "rng": rng, "mats": mats,
           "tag": tag, "authored": [], "static_extra": [], "loose": [],
           "velocity": {}, "notes": [], "n_uid": 0,
           "info": {"type": "urm"}, "neighbours": [nb]}
    bumped = ""
    g = str(rec.get("grade", "DG0")).split("+")[0]
    if g in ("DG0", "DG1", "DG2", "DG3"):
        vs = _variants(manifest, rec["style"], "DG4")
        if vs:
            chosen = rng.choice(vs)
            refs = prim.GetReferences()
            refs.ClearReferences()
            refs.AddReference(chosen["usd"])
            prim.Load()
            rec["grade"] = "DG4"
            bumped = ", archetype bumped {0} -> DG4".format(g)
    nside = qf._d_facing_side(mb, -(qf._outward(ma, side)[0]),
                              -(qf._outward(ma, side)[1]))
    nspan = qf._d_span(mb, nside, ma) or (-4.0, 4.0)
    span = qf._d_span(ma, side, mb) or (-ma["W"] / 4.0, ma["W"] / 4.0)
    Dn = mb["D"] if nside in ("S", "N") else mb["W"]
    # the heap sits a third of the way in from the wall it was thrown over
    depth = Dn * rng.uniform(0.22, 0.4)
    ilx, ily = ((0.5 * (nspan[0] + nspan[1]), -mb["D"] / 2.0 + depth)
                if nside == "S" else
                (0.5 * (nspan[0] + nspan[1]), mb["D"] / 2.0 - depth)
                if nside == "N" else
                (-mb["W"] / 2.0 + depth, 0.5 * (nspan[0] + nspan[1]))
                if nside == "W" else
                (mb["W"] / 2.0 - depth, 0.5 * (nspan[0] + nspan[1])))
    R = max(2.4, min(0.3 * min(mb["W"], mb["D"]), 0.45 * (nspan[1] - nspan[0])))
    qf._d_roof_heap(ctx, mb, mb["top"] - 0.15, ilx, ily, R,
                    rng.uniform(0.9, 1.8), tag="onroof")
    qf._d_face_band(ctx, mb, nside, mb["top"] - 0.4, nspan, height=(0.6, 1.1),
                    n_seg=(2, 5), tag="cornice")
    qf._d_gap_debris(ctx, ma, side, span, max(0.35, gap) + 0.6,
                     depth_m=rng.uniform(0.6, 1.1), spill_m=rng.uniform(2.5, 4.0),
                     glass=0.14)
    qf._heap(ctx, mb, mb["z0"], 0.0, 0.16, fill=False,
             sides=(qf._opposite(nside),), depth_m=rng.uniform(0.4, 0.9),
             tag="spill")
    return ("collapse_onto (authored): rubble on the {0} roof at ({1:.1f}, "
            "{2:.1f}) r {3:.1f} m, gap {4:.2f} m{5}".format(
                nside, ilx, ily, R, gap, bumped))
