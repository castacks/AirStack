"""tornado_collapse — the INDUSTRIAL tilt-up / light-roof collapse class
(`_plans/urban_tornado_plan.md` §8c, R11's SECOND real urban collapse
class; the first is `disaster.tornado_urban.t_facade_collapse`, the
brownstone/URM half). User: "we have industrial buildings and brownstones.
Those could collapse right ... So show those."

SELF-CONTAINED — NO PIECE GRID. Every other disaster module in this
pipeline (`tornado_urban`, `quake_sliced`, `tornado_kit`) damages a SLICED
building's element table (`quake_flow.describe`'s per-piece placements).
This class has no such table to damage: a Dmytro FactoryDistrict shed is a
single merged mesh with no cut lines at all, and the record's own failure
mode for this construction type (§0.3, §6, §8.2 below) is not "pieces come
off a grid" — it is a hinge-and-topple of whole PANELS the size of the
wall itself, plus a roof that comes down as one collapsed field, neither
of which the piece-grid vocabulary (`_apply_region`, toothing, boundary
piers) has any use for. So this module is PROCEDURAL instead: given a
building's own measured footprint (W, D, H) and placement (x, y, yaw), it
authors the collapse state directly — `plan_industrial` decides WHAT comes
down and where it lands (pure, JSON-serialisable, no `pxr`), and `apply_
industrial` authors it as merged meshes on a live stage.

WHY THIS IS A SEPARATE CLASS FROM `tornado_urban.t_facade_collapse`, AND
WHY ITS GRADE LADDER STARTS SO MUCH EARLIER
------------------------------------------------------------------------
`_plans/urban_tornado_research.md` §0.3 identifies TWO, and only two,
low-rise total-collapse populations in the tornado record, and they fail by
COMPLETELY DIFFERENT MECHANISMS:

  1. True unreinforced-masonry BEARING-WALL construction (Waco's Dennis
     Building) needed NEAR-F5 conditions to fully collapse, while a steel-
     framed neighbour on the SAME BLOCK, same event, was essentially
     untouched — the wall MATERIAL is what fails, and it takes an extreme
     event. `tornado_urban.t_facade_collapse`'s own `i >= 0.82` floor is
     calibrated to this population.
  2. Tilt-up CONCRETE PANEL and CMU-with-light-steel-roof construction
     (warehouses, factories, big-box retail, metal building systems) fails
     at the ROOF-TO-WALL CONNECTION — welds and anchor bolts, nothing to do
     with wall material strength — and the record shows this triggering
     across essentially the WHOLE useful range of the ladder: Fort Worth's
     Sweet Shop Factory and Johns Manville Warehouse totally collapsed at
     only F1-F2 (80-100 mph, §2.3/§0.3); Joplin's Home Depot (2-storey
     steel frame, 12 in precast tilt-up walls) and Cummins Equipment (steel
     frame, unanchored CMU curtain walls) and Mayfield's candle factory
     (tapered steel columns, "fell like dominoes") did so at EF3-EF4 (§6).
     §8.2's own words: "construction quality, not wind speed, dominates" —
     the SAME lesson the hurricane doc reaches independently for its own
     wall systems. Using `t_facade_collapse`'s i >= 0.82 threshold for this
     population would be WRONG — far too conservative for a class that the
     record shows failing at F1-F2 in more than one documented case.

Grades (this module's own, NOT `tornado_urban.LEVELS`' T0-T4 — see
`grade_for_intensity`): **partial** (0.5 <= i < 0.7) — one wall down plus
its two corner-adjacent segments (Mayfield's Pilgrims Pride North plant:
"2 wall panels on the east side fell outward, taking the supporting roof
beams down locally" is this shape). **total** (i >= 0.7) — 60-80% of the
perimeter down (Joplin's Home Depot: "walls fell — S/W inward, E outward,
N upright" is the same order of magnitude, simplified here to OUTWARD-only
per this module's own spec — see `_build_panel`'s docstring for the one
named deviation).

THE PANEL — HINGED AT THE WALL LINE, VERIFIED NUMERICALLY
-------------------------------------------------------------
A tilt-up panel is modelled as one box: length = its own wall segment
(`l`), "width" = its own standing HEIGHT (`w`, roughly the building's own
H), thickness = `t` (0.18-0.30 m). `tornado_urban_usd._frag_box`'s
rotation is `Rz(yaw) . Rx(tilt)` about the box's own CENTRE — there is no
"pivot about an edge" in that function (it is built for small debris
fragments, where a `_seat_z`-style "lowest point touches the ground"
heuristic is good enough; a fragment that size does not care exactly where
its own hinge was). A wall-height panel DOES care — "hinge at the wall
line" is this class's own explicit spec — so `_build_panel` solves for the
box's own CENTRE position at every tilt such that its base-inner corner
stays fixed at the wall line and z = 0, in closed form:

    y_off = hw*cos(tilt) - ht*cos(tilt)... [see the function body: the
            outward offset of the box centre from the wall line]
    z_c   = hw*sin(tilt) + ht*cos(tilt)    [the box centre's height]

(`hw` = half the panel's standing height, `ht` = half its thickness, both
about `_frag_box`'s own local axes — `tilt_deg` 0 = fully fallen flat, 90 =
still standing, the SAME convention `_seat_z`'s own docstring states for a
`t/w` box). VERIFIED, not just derived on paper (this module's own "the
bench is the only oracle" discipline, `.agents/skills/author-human-poses`):
a direct numerical probe of `_frag_box` at all four wall orientations and a
non-trivial building yaw confirmed the formula is orientation-INDEPENDENT
(same `(y_off, z_c)` behaviour on S/N/W/E alike) — flat gives `z in [0, t]`
(base flush with grade) and a centre offset of `h_panel/2` outward (the
fallen panel's own outward reach, matching "flat lying on the ground,
extending outward by its own height"); standing (`tilt=90`) gives `z in
[0, h_panel]`; every tilt in between keeps the base at `z ~ 0` (within the
panel's own half-thickness — the base is a FACE, not a point, so a few cm
of give is the box's own geometry, not an error).

ONE NAMED DEVIATION FROM THE FULL RECORD: real events show panels falling
BOTH outward AND inward on different walls of the same building (Joplin's
Home Depot: "S/W inward, E outward, N upright"; Cummins Equipment: "S/W
walls fell in, N/E walls fell out"). This module's own task spec (§8c) asks
only for "fallen OUTWARD flat (70%) / leaning (30%)" — no inward mode — so
that is what is built; an inward-falling panel (which would need to be
authored INSIDE the footprint, colliding with roof/contents debris that
also lands there) is a documented gap for a future round, not silently
dropped.

THE ROOF, CONTENTS, RUBBLE — NO HINGE, SIMPLE FIELDS
-----------------------------------------------------
The light roof comes down INSIDE the footprint as a HEAPED field of
corrugated/membrane sheet fragments (0.4-0.9 of the footprint's own area,
z 0.2-0.8 m — sitting on top of whatever the collapse piled beneath it, not
flush with grade), clustered into a handful of drop piles rather than
scattered uniformly (`_build_roof`'s own docstring — round4/D4: a plain
independent-per-sheet scatter is exactly what read as "paper confetti" on
the bench, regardless of what colour the sheets were), plus 4-10 joist
lines (long thin boxes, same z band, their own `"steel_joist"` material —
see `_build_joists`).
Sparse contents (shelving/equipment, 5-15 boxes) sit on the ground inside
the footprint; rubble (spall off the panel impacts) rings each fallen
panel just beyond its own outward reach. Contents/rubble use their OWN tiny
`_ground_seat_z` (a copy of `tornado_urban_usd._seat_z`'s formula, not an
import of it — see that function's own docstring for why: keeping this
half of the module free of any `pxr`-touching import means `plan_
industrial` stays importable and testable on a bare host with no `pxr`
installed at all, the same "no pxr anywhere in the plan half" discipline
`tornado_urban.py`'s own module docstring states for its planner).

WHAT `apply_industrial` DOES AND DOES NOT DO
------------------------------------------------
Authors one merged `UsdGeom.Mesh` per (kind, material) class through
`tornado_urban_usd.build_debris`-STYLE topology (`_frag_box` for the box
geometry, `debris_material` for the look-bucket materials — imported,
READ-ONLY reuse, never edited by this stream) — but NOT a literal call to
`build_debris` itself: that function recomputes every fragment's own Z from
`_seat_z`'s ground-hugging heuristic regardless of what a fragment dict
carries, which is WRONG for this module's two Z-bearing geometries (a
panel's hinge-preserving centre height, a roof sheet's explicit 0.2-0.8 m
band) — see `apply_industrial`'s own docstring for the full reasoning.
Deactivates an intact shed reference at `ctx["parent"] + "/src"` if the
caller placed one there (this module's own probe does, for visual
comparison — the SAME `<cell>/src` convention every other sliced-building
probe in this pipeline uses). Does NOT hide the shed's ORIGINAL placement
in the city, and does NOT wrap this cell in a world-space holder Xform —
both are the LAUNCHER's job (§8c: "the launcher hook is the LEAD's,
applied after stream SP's launcher edits land"), exactly like `tornado_
urban_usd.wreck_urban`'s own "x/y/yaw are the CELL's and are always zero
here" convention: this module operates in a cell at the origin, like every
other wreck_* pass in this pipeline.
"""

import math

# ---------------------------------------------------------------------------
# GRADES — this module's OWN ladder (NOT `tornado_urban.LEVELS`' T0-T4).
# §8.2's own cuts: partial 0.5-0.7, total >= 0.7. Below 0.5, not a record —
# the same "not a record" convention `tornado_urban.level_for_intensity`'s
# T0 band uses.
# ---------------------------------------------------------------------------
GRADES = ("partial", "total")
PARTIAL_MIN_I = 0.5
TOTAL_MIN_I = 0.7


def grade_for_intensity(i):
    """`"partial"` (0.5 <= i < 0.7), `"total"` (i >= 0.7), or `None` (i <
    0.5, not a record) — the gate/dry-run's own draw, kept here so the two
    numbers live in exactly one place."""
    i = float(i)
    if i >= TOTAL_MIN_I:
        return "total"
    if i >= PARTIAL_MIN_I:
        return "partial"
    return None


# ---------------------------------------------------------------------------
# WALL GEOMETRY — four sides, a LOCAL (pre-building-yaw) frame each.
# ---------------------------------------------------------------------------
_WALLS = ("S", "N", "W", "E")


def _wall_geom(side, W, D):
    """`(length, base_fn(t) -> (lx, ly), along_dir, outward_dir)` for one
    side, all in the building's own LOCAL frame (yaw not yet applied).
    `base_fn(t)` is the point `t` metres along the wall from its own t=0
    end; `along_dir`/`outward_dir` are unit vectors. VERIFIED (this
    module's docstring) to compose correctly with `_build_panel`'s hinge
    formula for all four sides at a non-trivial building yaw."""
    if side == "S":
        return W, (lambda t: (-W / 2.0 + t, -D / 2.0)), (1.0, 0.0), (0.0, -1.0)
    if side == "N":
        return W, (lambda t: (-W / 2.0 + t, D / 2.0)), (1.0, 0.0), (0.0, 1.0)
    if side == "W":
        return D, (lambda t: (-W / 2.0, -D / 2.0 + t)), (0.0, 1.0), (-1.0, 0.0)
    return D, (lambda t: (W / 2.0, -D / 2.0 + t)), (0.0, 1.0), (1.0, 0.0)


# which END (first or last tiled segment) of an ADJACENT wall touches a
# given PRIMARY wall's own corner — derived from `_wall_geom`'s own
# coordinate convention (see the module's git history / the C3 notes for
# the by-hand derivation): S/N meet W/E at t=0/t=length of W/E respectively
# in a way that is NOT symmetric, because `_wall_geom` tiles every wall
# from its own local origin, not from a shared corner.
_CORNER_END = {
    ("S", "W"): "first", ("S", "E"): "first",
    ("N", "W"): "last", ("N", "E"): "last",
    ("W", "S"): "first", ("W", "N"): "first",
    ("E", "S"): "last", ("E", "N"): "last",
}
_ADJACENT_WALLS = {"S": ("W", "E"), "N": ("W", "E"),
                   "W": ("S", "N"), "E": ("S", "N")}


# ---------------------------------------------------------------------------
# PANELS
# ---------------------------------------------------------------------------
_PANEL_LEN_M = (2.4, 7.0)
_PANEL_THICK_M = (0.18, 0.30)
_PANEL_HEIGHT_FRAC = (0.85, 1.0)
_FLAT_FRAC = 0.70
_LEAN_DEG_FROM_VERTICAL = (15.0, 40.0)
_TOTAL_FALLEN_FRAC = (0.60, 0.80)


def _wall_segments(rng, length):
    """Tile `length` into 2.4-7 m panel segments, `[(start, seglen), ...]`.
    The final segment absorbs whatever is left (never a sub-1 m sliver
    panel dangling off the end of a run)."""
    segs = []
    pos = 0.0
    lo, hi = _PANEL_LEN_M
    guard = 0
    while pos < length - 1e-6 and guard < 10000:
        guard += 1
        remaining = length - pos
        if remaining <= hi:
            seg = remaining
        else:
            seg = rng.uniform(lo, hi)
            if 0.0 < remaining - seg < lo:
                seg = remaining
        seg = max(1e-3, min(seg, remaining))
        segs.append((pos, seg))
        pos += seg
    return segs


def _all_segments(rng, W, D):
    """Every wall's own tiled segments, in a fixed S/N/W/E order (so this
    is deterministic given the seeded `rng`)."""
    out = []
    for side in _WALLS:
        length, _base, _along, _outward = _wall_geom(side, W, D)
        for i, (start, seglen) in enumerate(_wall_segments(rng, length)):
            out.append({"side": side, "i": i, "start": start,
                       "seglen": seglen, "mid_t": start + seglen / 2.0})
    return out


def _windward_wall(wind):
    """The wall whose OUTWARD normal most opposes the wind bearing —
    `tornado_urban.side_weights`' own "windward = max(0, -dot)" rule,
    collapsed to a single best-side pick (this module has no piece grid to
    weight bay-by-bay, only four flat walls)."""
    brg = math.radians(float((wind or {}).get("bearing_deg", 0.0)))
    dx, dy = math.cos(brg), math.sin(brg)
    best_side, best_dot = _WALLS[0], -2.0
    for side in _WALLS:
        _length, _base, _along, (ox, oy) = _wall_geom(side, 1.0, 1.0)
        dot = -(ox * dx + oy * dy)
        if dot > best_dot:
            best_dot, best_side = dot, side
    return best_side


def _pick_fallen_total(rng, segments, perimeter):
    """60-80% of the perimeter (by LENGTH), a random subset of segments
    shuffled across all four walls — leaves a scattered ~20-40% standing,
    never a single clean wall spared.

    A plain greedy-until-target walk can OVERSHOOT the 80% ceiling by up to
    one segment's own length (a large final segment pushing `acc` past
    `target` in one step) — measured, not assumed: an early draft of this
    function landed 0.807 on a real shed footprint against an 0.80 upper
    bound. Once `acc` has already cleared the 60% FLOOR, a candidate
    segment that would push `acc` past the 80% CEILING is skipped in
    favour of a smaller one later in the shuffle, rather than accepted
    regardless; below the floor every segment is still accepted regardless
    (reaching 60% at all matters more than a clean ceiling on the way
    there, and no real shed's perimeter is tiled coarsely enough for a
    single segment to jump from 0% to over 80% in one step)."""
    target_frac = rng.uniform(*_TOTAL_FALLEN_FRAC)
    lo_frac, hi_frac = _TOTAL_FALLEN_FRAC
    order = list(range(len(segments)))
    rng.shuffle(order)
    fallen, acc = set(), 0.0
    for idx in order:
        frac_now = (acc / perimeter) if perimeter > 1e-9 else 0.0
        if frac_now >= target_frac:
            break
        seglen = segments[idx]["seglen"]
        would_be = ((acc + seglen) / perimeter) if perimeter > 1e-9 else 0.0
        if frac_now >= lo_frac and would_be > hi_frac:
            continue
        fallen.add(idx)
        acc += seglen
    return fallen, acc, target_frac


def _pick_fallen_partial(segments, primary):
    """ONE wall (`primary`, the windward pick) entirely down, plus the ONE
    segment on each of its two adjacent walls nearest their shared corner
    with it (`_CORNER_END`) — Mayfield's Pilgrims Pride North plant shape
    ("2 wall panels on the east side... taking the supporting roof beams
    down locally")."""
    fallen = set()
    by_side = {}
    for idx, s in enumerate(segments):
        by_side.setdefault(s["side"], []).append(idx)
    for idx in by_side.get(primary, ()):
        fallen.add(idx)
    for adj in _ADJACENT_WALLS.get(primary, ()):
        idxs = by_side.get(adj, ())
        if not idxs:
            continue
        end = _CORNER_END.get((primary, adj), "first")
        fallen.add(idxs[0] if end == "first" else idxs[-1])
    return fallen


def _build_panel(rng, side, seg, W, D, H, yaw, x, y):
    """One fallen panel dict, hinged at the wall line — see the module
    docstring's "THE PANEL" section for the closed-form derivation and its
    numerical verification. `mode` is drawn 70% `"flat"` (`tilt_deg = 0`,
    fully down) / 30% `"leaning"` (`tilt_deg = 90 - lean`, `lean` drawn
    15-40 deg from vertical per Sec8c)."""
    length, base_fn, along, outward = _wall_geom(side, W, D)
    lx0, ly0 = base_fn(seg["mid_t"])
    thickness = rng.uniform(*_PANEL_THICK_M)
    h_panel = H * rng.uniform(*_PANEL_HEIGHT_FRAC)
    mode = "flat" if rng.random() < _FLAT_FRAC else "leaning"
    lean_deg = 0.0
    if mode == "flat":
        tilt_deg = 0.0
    else:
        lean_deg = rng.uniform(*_LEAN_DEG_FROM_VERTICAL)
        tilt_deg = 90.0 - lean_deg
    hw, ht = h_panel / 2.0, thickness / 2.0
    tilt = math.radians(tilt_deg)
    y_off = hw * math.cos(tilt) - ht * math.sin(tilt)
    z_c = hw * math.sin(tilt) + ht * math.cos(tilt)
    lx = lx0 + outward[0] * y_off
    ly = ly0 + outward[1] * y_off
    a = math.radians(yaw)
    wx = x + lx * math.cos(a) - ly * math.sin(a)
    wy = y + lx * math.sin(a) + ly * math.cos(a)
    hinge_x = x + lx0 * math.cos(a) - ly0 * math.sin(a)
    hinge_y = y + lx0 * math.sin(a) + ly0 * math.cos(a)
    along_angle = math.degrees(math.atan2(along[1], along[0]))
    panel_yaw = (yaw + along_angle) % 360.0
    return {
        "side": side, "seg_i": int(seg["i"]), "length": float(seg["seglen"]),
        "thickness": float(thickness), "height": float(h_panel),
        "mode": mode, "tilt_deg": float(tilt_deg), "lean_deg": float(lean_deg),
        "x": float(wx), "y": float(wy), "z": float(z_c),
        "yaw_deg": float(panel_yaw),
        "hinge_x": float(hinge_x), "hinge_y": float(hinge_y),
        "material": "concrete_panel",
    }


# ---------------------------------------------------------------------------
# ROOF, JOISTS, CONTENTS, RUBBLE — no hinge, simple fields.
# ---------------------------------------------------------------------------
_ROOF_COVERAGE = (0.4, 0.9)
_ROOF_Z = (0.2, 0.8)
_ROOF_SHEET_DIMS = ((1.0, 3.0), (0.5, 2.0))
_ROOF_SHEET_THICK = (0.03, 0.08)
_ROOF_N_CLUSTERS = (2, 6)
_ROOF_CLUSTER_REACH_FRAC = (0.12, 0.22)
_ROOF_PILE_TOP_M = (0.35, _ROOF_Z[1])
_N_JOISTS = (4, 10)
_JOIST_THICK = (0.15, 0.30)
_JOIST_LEN_FRAC = (0.6, 0.9)
_N_CONTENTS = (5, 15)
_CONTENTS_DIM = (0.4, 1.5)
_RUBBLE_PER_PANEL = (2, 5)
_RUBBLE_DIM = (0.15, 0.5)
_RUBBLE_RING_OUT_M = (0.3, 2.5)


def _ground_seat_z(t, w, tilt_deg, ground_z=0.0, bed=0.02):
    """The same box-centre-height-that-seats-on-its-face rule
    `tornado_urban_usd._seat_z` uses, duplicated (not imported) so this
    half of the module stays free of any `pxr`-touching import — see the
    module docstring's "THE ROOF, CONTENTS, RUBBLE" section."""
    tilt = math.radians(float(tilt_deg or 0.0))
    half_h = 0.5 * (abs(float(t) * math.cos(tilt)) + abs(float(w) * math.sin(tilt)))
    bed = min(bed, float(t) / 2.0)
    return float(ground_z) + half_h - bed


def _to_world(lx, ly, yaw, x, y):
    a = math.radians(yaw)
    return (x + lx * math.cos(a) - ly * math.sin(a),
            y + lx * math.sin(a) + ly * math.cos(a))


def _scatter_footprint(rng, n, W, D, margin=0.5):
    w2 = max(0.0, W / 2.0 - margin)
    d2 = max(0.0, D / 2.0 - margin)
    return [(rng.uniform(-w2, w2), rng.uniform(-d2, d2)) for _ in range(n)]


def _build_roof(rng, W, D, yaw, x, y, material):
    """The collapsed roof deck -- a PILE, not a uniform lawn-sprinkler
    scatter (D4/round4: the bench read as "random single coloured
    rectangles... paper confetti", the SAME failure a plain
    `_scatter_footprint` call produces regardless of material colour,
    since every sheet lands independently with no relationship to its
    neighbours). A handful of drop CLUSTERS -- roughly where each fallen
    bay's own roof structure let go together -- each sheet jittered around
    its own cluster centre with a SMALL reach (a fraction of the shed's own
    short side), so sheet footprints actually overlap their neighbours
    inside a cluster the way a real heaped deck does, with gaps between
    clusters instead of one even carpet.

    Z still comes from the SAME `_ROOF_Z` = 0.2-0.8 m band this module's
    docstring and `test_roof_field_stays_inside_the_footprint` (unchanged)
    already commit to -- but each CLUSTER draws its own pile-top height
    within that band first, and every sheet in it samples up to THAT top,
    so different piles read at different heights instead of every sheet
    independently sampling the full band (the OLD behaviour, which put a
    tall sheet and a squashed one side by side with no correlation and
    read as flat and undifferentiated from above)."""
    coverage = rng.uniform(*_ROOF_COVERAGE)
    (l_lo, l_hi), (w_lo, w_hi) = _ROOF_SHEET_DIMS
    mean_area = ((l_lo + l_hi) / 2.0) * ((w_lo + w_hi) / 2.0)
    n_sheets = max(1, int(round(coverage * W * D / max(1e-6, mean_area))))

    n_clusters = max(_ROOF_N_CLUSTERS[0],
                     min(_ROOF_N_CLUSTERS[1],
                        int(round(math.sqrt(float(n_sheets)) / 3.0))))
    reach = rng.uniform(*_ROOF_CLUSTER_REACH_FRAC) * min(W, D)
    centres = _scatter_footprint(rng, n_clusters, W, D, margin=max(0.3, reach))
    pile_top = [rng.uniform(*_ROOF_PILE_TOP_M) for _ in centres]
    w2 = max(0.0, W / 2.0 - 0.3)
    d2 = max(0.0, D / 2.0 - 0.3)

    sheets = []
    for _i in range(n_sheets):
        ci = rng.randrange(n_clusters)
        cx, cy = centres[ci]
        lx = min(w2, max(-w2, cx + rng.uniform(-reach, reach)))
        ly = min(d2, max(-d2, cy + rng.uniform(-reach, reach)))
        length = rng.uniform(l_lo, l_hi)
        width = rng.uniform(w_lo, w_hi)
        thick = rng.uniform(*_ROOF_SHEET_THICK)
        wx, wy = _to_world(lx, ly, yaw, x, y)
        z = rng.uniform(_ROOF_Z[0], pile_top[ci])
        sheets.append({
            "x": float(wx), "y": float(wy), "z": float(z),
            "size": [float(length), float(width), float(thick)],
            "yaw_deg": float(rng.uniform(0.0, 360.0)),
            "tilt_deg": float(rng.uniform(0.0, 10.0)),
            "material": material})
    return sheets, coverage


def _build_joists(rng, W, D, yaw, x, y):
    """D4 (round4): the plan's own §8c spec calls joists out as "dark
    steel", distinct from a generic sheet-metal roof. `material` is
    `"steel_joist"` rather than the bare `"metal"` every roof/contents
    fragment already uses -- `tornado_urban_usd._classify`'s substring
    match on `"steel"` still routes this into the SAME `metal` look bucket
    TODAY (so nothing here depends on an unlanded change), but it gives the
    lead a distinct material STRING to key a genuinely darker/less-glossy
    bucket off of without this module needing to change again -- see this
    round's report for the exact `_classify`/`_CLASS_LOOK` addition
    proposed."""
    n = rng.randint(*_N_JOISTS)
    joists = []
    for _i in range(n):
        length = rng.uniform(*_JOIST_LEN_FRAC) * min(W, D)
        thick = rng.uniform(*_JOIST_THICK)
        (lx, ly), = _scatter_footprint(rng, 1, W, D,
                                       margin=max(0.5, length / 2.0))
        wx, wy = _to_world(lx, ly, yaw, x, y)
        z = rng.uniform(*_ROOF_Z)
        joists.append({
            "x": float(wx), "y": float(wy), "z": float(z),
            "size": [float(length), float(thick), float(thick)],
            "yaw_deg": float(rng.uniform(0.0, 360.0)),
            "tilt_deg": float(rng.uniform(0.0, 6.0)),
            "material": "steel_joist"})
    return joists


def _build_contents(rng, W, D, yaw, x, y):
    n = rng.randint(*_N_CONTENTS)
    out = []
    for (lx, ly) in _scatter_footprint(rng, n, W, D, margin=1.0):
        l = rng.uniform(*_CONTENTS_DIM)
        w = rng.uniform(*_CONTENTS_DIM)
        t = rng.uniform(*_CONTENTS_DIM)
        wx, wy = _to_world(lx, ly, yaw, x, y)
        tilt = rng.uniform(0.0, 15.0) if rng.random() < 0.3 else 0.0
        z = _ground_seat_z(t, w, tilt)
        out.append({
            "x": float(wx), "y": float(wy), "z": float(z),
            "size": [float(l), float(w), float(t)],
            "yaw_deg": float(rng.uniform(0.0, 360.0)),
            "tilt_deg": float(tilt), "material": "metal"})
    return out


def _build_rubble(rng, panels):
    out = []
    lo_n, hi_n = _RUBBLE_PER_PANEL
    for panel in panels:
        dx = panel["x"] - panel["hinge_x"]
        dy = panel["y"] - panel["hinge_y"]
        norm = math.hypot(dx, dy)
        ox, oy = (dx / norm, dy / norm) if norm > 1e-6 else (0.0, -1.0)
        lax, lay = -oy, ox
        for _i in range(rng.randint(lo_n, hi_n)):
            extra = rng.uniform(*_RUBBLE_RING_OUT_M)
            lateral = rng.uniform(-panel["length"] / 2.0, panel["length"] / 2.0)
            rx = panel["x"] + ox * extra + lax * lateral
            ry = panel["y"] + oy * extra + lay * lateral
            d1 = rng.uniform(*_RUBBLE_DIM)
            d2 = rng.uniform(*_RUBBLE_DIM)
            thick = min(d1, d2)
            tilt = rng.uniform(0.0, 20.0)
            z = _ground_seat_z(thick, max(d1, d2), tilt)
            out.append({
                "x": float(rx), "y": float(ry), "z": float(z),
                "size": [float(d1), float(d2), float(thick)],
                "yaw_deg": float(rng.uniform(0.0, 360.0)),
                "tilt_deg": float(tilt), "material": "concrete_panel",
                "from_panel_side": panel["side"],
                "from_panel_seg_i": panel["seg_i"]})
    return out


# ---------------------------------------------------------------------------
# THE PLAN
# ---------------------------------------------------------------------------
def plan_industrial(W, D, H, yaw, x, y, grade, wind, rng):
    """Everything the industrial collapse does to one shed, decided with NO
    stage access at all (pure — the same "no pxr anywhere in the plan half"
    contract `tornado_urban.plan_damage` keeps). `grade` is the CALLER's own
    choice (`grade_for_intensity(i)`, above) — this function does not read
    intensity itself, the same separation `tornado_urban.plan_damage` keeps
    from `level_for_intensity`.

    Returns a `tornado_industrial_plan.v1` dict: `panels`/`roof_sheets`/
    `joists`/`contents`/`rubble` (lists of plain fragment dicts —
    `x`/`y`/`z`/`size`/`yaw_deg`/`tilt_deg`/`material`, `panels` carry a few
    extra fields, see `_build_panel`), `notes`, `stats`.
    """
    if grade not in GRADES:
        raise KeyError("unknown industrial grade {0!r} (have {1})".format(
            grade, ", ".join(GRADES)))
    W, D, H = float(W), float(D), float(H)
    yaw, x, y = float(yaw), float(x), float(y)
    wind = dict(wind or {})

    segments = _all_segments(rng, W, D)
    perimeter = sum(s["seglen"] for s in segments)

    primary_wall = None
    if grade == "total":
        fallen_idx, fallen_len, target_frac = _pick_fallen_total(
            rng, segments, perimeter)
    else:
        primary_wall = _windward_wall(wind)
        fallen_idx = _pick_fallen_partial(segments, primary_wall)
        fallen_len = sum(segments[i]["seglen"] for i in fallen_idx)
        target_frac = None

    panels = [_build_panel(rng, segments[i]["side"], segments[i], W, D, H,
                           yaw, x, y) for i in sorted(fallen_idx)]
    n_flat = sum(1 for p in panels if p["mode"] == "flat")
    n_leaning = len(panels) - n_flat

    by_side_all, by_side_fallen = {}, {}
    for idx, s in enumerate(segments):
        by_side_all[s["side"]] = by_side_all.get(s["side"], 0) + 1
        if idx in fallen_idx:
            by_side_fallen[s["side"]] = by_side_fallen.get(s["side"], 0) + 1
    walls_fully_down = sorted(
        sd for sd in _WALLS
        if by_side_all.get(sd, 0) > 0
        and by_side_fallen.get(sd, 0) == by_side_all.get(sd, 0))

    roof_material = "metal" if rng.random() < 0.5 else "membrane"
    roof_sheets, roof_coverage = _build_roof(rng, W, D, yaw, x, y, roof_material)
    joists = _build_joists(rng, W, D, yaw, x, y)
    contents = _build_contents(rng, W, D, yaw, x, y)
    rubble = _build_rubble(rng, panels)

    frac = (fallen_len / perimeter) if perimeter > 1e-9 else 0.0
    notes = ["industrial {0}: {1} of {2} perimeter panel(s) down "
            "({3:.1f} of {4:.1f} m, {5:.2f} of the perimeter), roof "
            "material {6}".format(grade, len(panels), len(segments),
                                  fallen_len, perimeter, frac, roof_material)]
    if primary_wall:
        notes.append("industrial partial: primary windward wall {0} down, "
                    "plus its two corner-adjacent segments".format(primary_wall))

    return {
        "schema": "tornado_industrial_plan.v1",
        "grade": grade, "W": W, "D": D, "H": H, "yaw": yaw, "x": x, "y": y,
        "wind": {"bearing_deg": float(wind.get("bearing_deg", 0.0)),
                "speed_frac": float(wind.get("speed_frac", 0.0))},
        "panels": panels, "roof_sheets": roof_sheets, "joists": joists,
        "contents": contents, "rubble": rubble, "notes": notes,
        "stats": {
            "perimeter_m": float(perimeter), "fallen_perimeter_m": float(fallen_len),
            "fallen_frac": float(frac), "target_frac": (float(target_frac)
                                                         if target_frac is not None
                                                         else None),
            "n_segments_total": len(segments), "n_panels_fallen": len(panels),
            "n_flat": n_flat, "n_leaning": n_leaning,
            "primary_wall": primary_wall, "walls_fully_down": walls_fully_down,
            "roof_material": roof_material, "roof_coverage": float(roof_coverage),
            "n_roof_sheets": len(roof_sheets), "n_joists": len(joists),
            "n_contents": len(contents), "n_rubble": len(rubble),
        },
    }


# ---------------------------------------------------------------------------
# THE APPLY HALF — pxr, imported lazily so `plan_industrial` above stays
# importable (and this module stays importABLE at all) with no `pxr`
# installed.
# ---------------------------------------------------------------------------
def apply_industrial(stage, ctx, plan, holder=None, verbose=True):
    """Author `plan` (from `plan_industrial`) onto `stage` under
    `ctx["parent"]` — see the module docstring's "WHAT `apply_industrial`
    DOES AND DOES NOT DO" section for the full contract, in particular why
    this does NOT call `tornado_urban_usd.build_debris` directly (it would
    silently discard every fragment's own `z`).

    `holder`: accepted for forward-compatibility with a launcher's own
    per-building world-space Xform wrapper; unused this round — every
    fragment here is already authored in the SAME frame `plan`'s own
    x/y/yaw were given in (a cell at the origin, like every other wreck_*
    pass in this pipeline; see the module docstring).

    Returns a small counts dict, the same shape `tornado_urban_usd.
    apply_plan`'s own return follows.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    from . import damage
    from . import quake_flow as qf
    from . import tornado_urban_usd as tuw

    mats = ctx.setdefault("mats", {})
    parent = ctx.get("parent") or "/World"

    n_src_deactivated = 0
    src_path = ctx.get("src_path") or (parent + "/src")
    prim = stage.GetPrimAtPath(src_path)
    if prim and prim.IsValid():
        if qf._deactivate(stage, src_path):
            n_src_deactivated = 1

    groups = {}

    def _add(kind, frag):
        material = str(frag.get("material") or "unknown")
        groups.setdefault((kind, material), []).append(frag)

    for p in plan.get("panels") or ():
        _add("panel", {"x": p["x"], "y": p["y"], "z": p["z"],
                       "size": [p["length"], p["height"], p["thickness"]],
                       "yaw_deg": p["yaw_deg"], "tilt_deg": p["tilt_deg"],
                       "material": p.get("material", "concrete_panel")})
    for s in plan.get("roof_sheets") or ():
        _add("roof_sheet", s)
    for j in plan.get("joists") or ():
        _add("joist", j)
    for c in plan.get("contents") or ():
        _add("contents", c)
    for r in plan.get("rubble") or ():
        _add("rubble", r)

    root = "{0}/industrial_debris".format(parent)
    UsdGeom.Scope.Define(stage, Sdf.Path(root))
    made = []
    for (kind, material), frags in sorted(groups.items()):
        pts, counts, idx_, nrm = [], [], [], []
        for frag in frags:
            l, w, t = (float(q) for q in frag["size"])
            gx, gy, gz = float(frag["x"]), float(frag["y"]), float(frag.get("z", 0.0))
            yaw_deg = float(frag.get("yaw_deg", 0.0))
            tilt_deg = float(frag.get("tilt_deg", 0.0))
            p8, n6 = tuw._frag_box(l, w, t, gx, gy, gz, yaw_deg, tilt_deg)
            base = len(pts)
            pts.extend(Gf.Vec3f(*q) for q in p8)
            for fi, face in enumerate(tuw._FACES):
                counts.append(4)
                idx_.extend(base + v for v in face)
                nrm.extend([Gf.Vec3f(*n6[fi])] * 4)
        path = "{0}/{1}_{2}".format(root, tuw._safe_name(kind),
                                    tuw._safe_name(material))
        m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
        m.CreatePointsAttr(Vt.Vec3fArray(pts))
        m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray(idx_))
        m.CreateNormalsAttr(Vt.Vec3fArray(nrm))
        m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        xs = [q[0] for q in pts]
        ys = [q[1] for q in pts]
        zs = [q[2] for q in pts]
        m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                            Gf.Vec3f(max(xs), max(ys), max(zs))])
        mat = tuw.debris_material(stage, ctx, kind, material)
        if mat is None:
            # BELT AND SUSPENDERS (D4/round4): a container probe across
            # every (kind, material) pair this module ever emits (panel/
            # concrete_panel, roof_sheet/metal, roof_sheet/membrane,
            # joist/steel_joist, contents/metal, rubble/concrete_panel)
            # never found `debris_material` returning None -- every branch
            # in that function ends by assigning a real material. But an
            # UNBOUND mesh renders as Kit's own flat default (often reads
            # as white/blank, the D4 bench symptom this round exists to
            # rule out), so this stays a hard guarantee rather than a
            # trusted invariant: a magenta flag material, unmistakably NOT
            # any class colour, so a future regression upstream is visible
            # in a render instead of quietly reading as ordinary debris.
            fallback_path = "{0}/_UNCLASSIFIED_{1}_{2}".format(
                root, tuw._safe_name(kind), tuw._safe_name(material))
            mat = damage._pbr(stage, fallback_path, (0.85, 0.05, 0.85), 0.6)
            print("[tornado_collapse] WARNING: tuw.debris_material(kind={0!r}, "
                  "material={1!r}) returned None -- bound a magenta flag "
                  "material at {2} instead of leaving {3} unbound".format(
                      kind, material, fallback_path, path))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
        made.append(path)

    out = {"n_panels": len(plan.get("panels") or ()),
          "n_roof_sheets": len(plan.get("roof_sheets") or ()),
          "n_joists": len(plan.get("joists") or ()),
          "n_contents": len(plan.get("contents") or ()),
          "n_rubble": len(plan.get("rubble") or ()),
          "n_meshes": len(made), "n_src_deactivated": n_src_deactivated,
          "meshes": made}
    if verbose:
        print("[tornado_collapse] {0} ({1}): {2} panel(s), {3} roof "
              "sheet(s), {4} joist(s), {5} content(s), {6} rubble, {7} "
              "mesh(es), src deactivated={8}".format(
                  plan.get("grade"), root, out["n_panels"], out["n_roof_sheets"],
                  out["n_joists"], out["n_contents"], out["n_rubble"],
                  out["n_meshes"], bool(n_src_deactivated)))
    return out
