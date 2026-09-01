"""urban_tornado_people.py — BENCH-GRADE urban-tornado victim placement.

Source: `_plans/urban_tornado_people_research.md` (stream V, round 3) — a
primary-source study (NIST NCSTAR 3's Joplin location table, direct-fetched
and `pdftotext`'d, plus Waco/Fort Worth/Atlanta/Lubbock/Mayfield/Nashville/
Salt Lake City retrospectives) answering the user's own question, verbatim:
"where victims are found here? must be different than suburban." That
document is the settled research; this module is the FIRST placement code
built against it, not a guess pending the research — but it is still
BENCH-GRADE: a small set of hand-anchored classes for the R12 bench's D-row
(one archetype per anchor: a berm, a pile edge, a refuge doorway, a struck
car, a standing entry), not the full corridor-budget urban placer that
research doc's own `_pools`/`_quota`-shaped city solve is the eventual home
for. That placer is a later round's job; this module exists so the bench
(`urban_tornado_bench_launch_script.py`) has something to show NOW.

WHY NOT REIMPLEMENT THE POSE MACHINERY. Every function here returns a plain
PURE dict (class/pose/x/y/yaw_deg/prone/anchor/rule) — no `pxr`, no stage.
Turning one into an authored figure is `disaster.people._human_placement`'s
job (the SAME function `tornado_people`/`fire_people` delegate to for
exactly this reason — see `tornado_people._human_placement`'s own
docstring): per-character stature scaling, the RenderPeople yaw/roll
corrections, the lay-down roll and the long-axis spin that puts a figure on
its side, all derived by measurement against the rigs. `to_placement`
below is the one bridge function that touches `people`, kept separate so
every anchor/burial-fraction/pose-selection function in this file stays
importable and testable with NO `pxr` on the path at all.

THE RULES THIS MODULE ENCODES, numbered exactly as the research doc's own
"RULES FOR THE PLACER" section (cited per function below):

  1. Casualty classes are GATED to T3/T4 buildings (`casualty_gate`) — NCSTAR
     3 Finding 41: virtually every indoor Joplin death was in a building
     rated "demolished" or EF-3+ exposed. A1's T2 cell gets no casualties.
  2. A `urm`/big-box T3/T4 building sheds its casualties at the WINDWARD
     wall-base rubble berm (0-4 m out, `BERM_OFFSET_RANGE_M`), at the FRONT/
     exposed side — never a back-refuge interior (Home Depot: deaths were
     "toward the front of the store in the lumber area", not the back
     training room 28 people survived in).
  3. Vehicles get roughly DOUBLE the suburb `street` share — `VEHICLE_SHARE`
     0.20 (NCSTAR 3 Table 4-3 re-normalised to a residential-free corridor,
     the research doc's own §3 figure), and a struck-car occupant is a
     first-order class (`car_occupant`), not incidental set-dressing.
  4. A NEW class the suburb has no equivalent for: a pedestrian or parked
     car crushed OUTDOORS by a URM wall that failed outward onto a
     sidewalk/street (Waco, Fort Worth, Atlanta — three independent events,
     three EF/F ratings). `crush_victim`, gated to fire ONLY where the
     cell's own collapse recipe actually sheds a macroblock — never at
     every damaged building.
  5. Institutional buildings (not in this bench's cell table — no hospital/
     courthouse archetype in the §8d table) would get a dense, damage-
     -level-decoupled count along interior circulation. Not implemented
     here; noted so the gap is a decision, not an omission.
  6. St John's Regional Medical Center: NCSTAR 3 Table 4-3 codes **12**
     deaths (not 6 — a press-account undercount the plan brief's own first
     draft repeated), 10 of them impact-related blunt-force trauma through
     blown-in windward glazing, 2 heart attacks — NOT ventilator/life-
     -support failure, which NIST's own cause-of-death coding contradicts.
     Carried here only as a corrected fact (no institutional class in this
     bench); do not requote the "6 deaths"/ventilator version.
  7. A big-box/industrial designated refuge (a back room, a walk-in cooler,
     a "strongest" hallway) stays MOSTLY casualty-free, with 1-2 figures at
     its own DOORWAY/THRESHOLD (`doorway_threshold`) — the Pizza Hut cooler
     pattern: the person holding the door shut is disproportionately the
     casualty, not the 18 people inside it.
  8. Cap the outdoor-open-space-pedestrian class at <= 1 per scene
     (`MAX_OPEN_PEDESTRIAN`). This module places ZERO of that class on the
     bench (every D-row anchor is at a named architectural feature, per
     rule 2's own headline finding), so the cap is trivially honoured; it
     is still exported so a caller that adds one somewhere enforces it.
  9. A thrown-far-from-building casualty is rarer than the suburb's already
     -starved `trail` class (0-1 per scene) — no PERSON-throw record exists
     anywhere in the research; every long-throw urban record is a VEHICLE.
     Not implemented as a class here (nothing in the §8d table calls for
     one); noted for completeness.
 10. No floor-based gradient for apartment/mixed-use buildings — NCSTAR 3
     could not determine floor level for its own three apartment sites.
     N/A to this bench (no apartment archetype in the §8d table).
 11. Cluster commercial/industrial casualties TIGHTLY (fewer, denser bunches
     per building) — Home Depot's 7-8 in one department, Mayfield's 9 in one
     hallway, Waco's 22 in one building. `digger_pair`/`pile_edge_digger`
     draw their figures within a few metres of ONE anchor point, never
     spread across a whole wall run.
 12. If a future stream adds urban diggers/survivors (this module places
     CASUALTIES AND diggers both, per the §8d table — the suburb module's
     own diggers were cut entirely in an earlier round, but the urban bench
     was explicitly asked to show "diff types of damage and human
     placements", so `digger_pair`/`pile_edge_digger`/`doorway_threshold`
     ARE upright, non-casualty figures), their correct location is a
     standing building's own door/window, not a rubble-pile edge — honoured
     by `doorway_threshold` anchoring at the actual door, separately from
     the berm/pile-edge digger classes.

DETERMINISM. Every function below takes an explicit `random.Random` (or
draws no randomness at all) and touches no global state — the SAME seed
into the SAME call sequence reproduces the SAME output, which is what
`tests/test_urban_tornado_people.py` pins.
"""

import math

# ---------------------------------------------------------------------------
# Constants — every one cited to a rule above.
# ---------------------------------------------------------------------------

#: Rule 2 / R8 (the plan's own rubble-berm mechanic): a windward wall-base
#: berm runs 0-4 m out from the wall.
BERM_OFFSET_RANGE_M = (0.0, 4.0)
#: The berm's own height PROFILE, for `berm_profile`/`burial_fraction` — a
#: placeholder triangular ramp (rises from the wall to a peak near 1 m out,
#: falls back to grade by 4 m), BENCH-GRADE: not stream DB's own measured
#: berm geometry (`tornado_urban_ground`'s real profile, not yet landed as
#: of this round — reconcile once it exists, see `_plans/
#: urban_tornado_B_notes.md`).
BERM_PEAK_M = 1.0
BERM_SPAN_M = 4.0
BERM_PEAK_FRAC = 0.65
#: Above this covered-fraction a trapped figure reads as buried/unconscious
#: (`buried_reach`-weighted); below it, a visible lying attitude.
BURIAL_THRESHOLD = 0.45

#: Rule 4: a pedestrian/car crushed by an outward-failed URM wall sits
#: 1-2 car-widths (3-6 m) out — clearly past the berm's own 0-4 m band, on
#: open sidewalk/street, not in the rubble itself.
CRUSH_OFFSET_RANGE_M = (3.0, 6.0)

#: Rule 3, informational: target share of a FUTURE corridor-budget placer's
#: casualty population that should be vehicle-anchored — roughly double the
#: suburb skill's incidental 0.10 `street` share. Not consumed by anything
#: in this bench (a fixed exemplar grid has no "budget" to weight), kept
#: here as the number the next round's placer should target.
VEHICLE_SHARE = 0.20

#: Rule 8: cap on the outdoor-open-space-pedestrian class, per scene.
MAX_OPEN_PEDESTRIAN = 1

#: Rule 1: casualty classes (everything except `digger_pair`,
#: `pile_edge_digger`, `doorway_threshold`, `evacuee_pair` — the upright,
#: non-casualty classes) are gated to these damage levels.
CASUALTY_GATE_LEVELS = ("T3", "T4")

#: Poses, drawn from the SAME vocabulary `tornado_people`/`fire_people`
#: already ship (every name is a key of `scene_generator._HUMAN_POSES`, or a
#: lying attitude in `disaster.people.LYING_POSES` — see `to_placement`'s
#: docstring for why this module does not repeat that table). Grouped by
#: what the figure is DOING, not by class, so multiple classes can share a
#: table without duplicating it.
#: `dig_bent`/`dig_kneel` are the shipped rescuer-at-work poses
#: (`scene_generator._HUMAN_POSES`: "78 deg of forward pitch... eyes on the
#: pile" / a kneeling variant with the shin flat on the ground) — the
#: `author-human-poses` skill's own FK-verified pair, weighted ahead of the
#: more generic `crouch`/`stand_calm`/`wave_help`.
_DIGGER_POSES = (("dig_bent", 0.40), ("dig_kneel", 0.25), ("crouch", 0.15),
                 ("stand_calm", 0.10), ("wave_help", 0.10))
_DOORWAY_POSES = (("stand_calm", 0.60), ("crouch", 0.40))
_EVACUEE_POSES = (("idle", 0.40), ("stand_slump", 0.35), ("walk", 0.25))
_SEATED_POSES = (("seated_car", 0.50), ("seated_car_arms_down", 0.50))
#: A visibly-buried figure — dominated by `buried_reach` (fire_people's own
#: "supine, one arm extended — for partial burial" pose), the read a heavy
#: burial fraction should carry.
_BURIED_POSES = (("buried_reach", 0.50), ("lying_prone", 0.25),
                 ("lying_supine", 0.25))
#: A lightly-covered / merely-lying figure — spread across the full lying
#: vocabulary so a bench of several does not repeat one silhouette.
_VISIBLE_LYING_POSES = (("lying_supine", 0.20), ("lying_supine_open", 0.15),
                        ("lying_prone", 0.20), ("lying_prone_reach", 0.15),
                        ("lying_side_l", 0.15), ("lying_side_r", 0.15))


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def casualty_gate(level):
    """Rule 1: True only for T3/T4 — the level string as it appears in a
    `tornado_urban_plan.v1`/`tornado_city.record` (e.g. `"T3"`)."""
    return str(level) in CASUALTY_GATE_LEVELS


def anchor(x, y, bearing_deg, along_m=0.0, **extra):
    """A named architectural feature a placement class is anchored to: a
    point, an OUTWARD bearing (math convention, degrees — the direction
    FROM the feature INTO open ground, e.g. 270 for a wall whose windward
    face is -Y), and `along_m`, the feature's own RUN length (a wall's
    length for a berm anchor, a door's width for a threshold, a spacing
    hint for a car/entry). Extra keys pass through untouched (a caller's
    own bookkeeping, e.g. `{"cell": "B1"}`)."""
    d = {"x": float(x), "y": float(y), "bearing_deg": float(bearing_deg) % 360.0,
         "along_m": float(along_m)}
    d.update(extra)
    return d


def _weighted_choice(rng, table):
    """One name from `((name, weight), ...)`, deterministic given `rng`."""
    total = sum(w for _, w in table)
    if total <= 0.0:
        return table[0][0]
    r = rng.random() * total
    acc = 0.0
    for name, w in table:
        acc += w
        if r <= acc:
            return name
    return table[-1][0]


def _along_wall(a, along_frac, offset_m):
    """A point `offset_m` OUT from anchor `a` (along its outward bearing)
    and `along_frac` in [0, 1] slid along the anchor's own `along_m` run
    (0 = one end, 1 = the other, 0.5 = centre)."""
    r = math.radians(a["bearing_deg"])
    ax, ay = -math.sin(r), math.cos(r)   # unit vector ALONG the run
    ox, oy = math.cos(r), math.sin(r)    # unit vector OUTWARD
    s = (float(along_frac) - 0.5) * float(a.get("along_m", 0.0))
    return a["x"] + ax * s + ox * float(offset_m), \
        a["y"] + ay * s + oy * float(offset_m)


def berm_offset(rng, lo=BERM_OFFSET_RANGE_M[0], hi=BERM_OFFSET_RANGE_M[1]):
    """One distance out from a wall along its own berm — rule 2 / R8's
    0-4 m band by default."""
    return rng.uniform(float(lo), float(hi))


def berm_profile(offset_m, peak_m=BERM_PEAK_M, span_m=BERM_SPAN_M,
                 peak_frac=BERM_PEAK_FRAC):
    """A triangular berm-height profile: 0 at the wall and past `span_m`,
    rising to `peak_frac` at `peak_m` out. Returns a fraction in [0, 1] —
    BENCH-GRADE placeholder geometry (see `BERM_PEAK_M`'s own docstring),
    not a measurement of any authored berm mesh."""
    d = float(offset_m)
    if d <= 0.0 or d >= span_m:
        return 0.0
    if d <= peak_m:
        return peak_frac * (d / peak_m)
    return peak_frac * (1.0 - (d - peak_m) / (span_m - peak_m))


def burial_fraction(offset_m, rng=None, jitter=0.08, **kw):
    """Fraction of a lying figure the berm covers at `offset_m` out from
    the wall — `berm_profile` plus a small honest jitter (rubble is not a
    smooth ramp), clamped to [0, 1]. Pass `rng=None` for the noiseless
    profile (what the tests pin); a caller placing a real figure should
    pass its own `rng` so repeated calls do not all land on the same curve.
    """
    frac = berm_profile(offset_m, **kw)
    if rng is not None:
        frac += rng.uniform(-jitter, jitter)
    return max(0.0, min(1.0, frac))


def burial_fraction_from_debris(x, y, debris, radius_m=1.2):
    """Burial fraction estimated from REAL nearby debris — "via debris
    overlap", the way the suburb tornado-people bench measures cover: total
    plan-view AREA of every fragment in `debris` (the building's OWN
    `plan["debris"]` ledger, `tornado_urban_plan.v1`'s schema — plain dicts
    with `x`/`y`/`l`/`w`, JSON-safe, no `pxr` needed) that falls within
    `radius_m` of `(x, y)`, against the circle's own area. A cheap, honest
    proxy — not `tornado_people._Deck`'s measured-surface/`_cover` solve,
    which is the fuller model a later round should move this class onto.
    Returns 0.0 for no/empty debris (never guesses)."""
    if not debris:
        return 0.0
    r2 = float(radius_m) * float(radius_m)
    circle_area = math.pi * r2
    covered = 0.0
    for f in debris:
        dx = float(f.get("x", 0.0)) - float(x)
        dy = float(f.get("y", 0.0)) - float(y)
        if dx * dx + dy * dy > r2:
            continue
        covered += float(f.get("l", 0.3)) * float(f.get("w", 0.2))
    return max(0.0, min(1.0, covered / circle_area))


def _lying_pose_for_burial(frac, rng):
    table = _BURIED_POSES if frac >= BURIAL_THRESHOLD else _VISIBLE_LYING_POSES
    return _weighted_choice(rng, table)


# ---------------------------------------------------------------------------
# Placement classes — each returns plain dict(s): class/pose/x/y/yaw_deg/
# prone/anchor/rule. No pxr, no stage.
# ---------------------------------------------------------------------------

def digger_pair(berm_line, rng, n=2, cluster_span_m=3.0):
    """Rule 2 + 11: `n` (default 2) upright rescuer figures clustered
    TIGHTLY (within `cluster_span_m` of one another) at a windward wall-
    base berm — the FRONT/exposed side the caller's `berm_line` anchor
    already names (never a back-refuge interior; see `windward_wall_anchor`
    in the bench launcher for how that anchor is measured). Diggers stand
    close to the wall, on the berm's near slope, not out at its full 4 m
    reach."""
    along_len = max(float(berm_line.get("along_m", 0.0)), 1e-6)
    centre = rng.uniform(0.35, 0.65)
    span_frac = min(0.45, (cluster_span_m / along_len) * 0.5)
    out = []
    for _ in range(int(n)):
        along = max(0.0, min(1.0, centre + rng.uniform(-span_frac, span_frac)))
        offset = berm_offset(rng, 0.0, 1.6)
        x, y = _along_wall(berm_line, along, offset)
        pose = _weighted_choice(rng, _DIGGER_POSES)
        out.append({
            "class": "digger", "pose": pose, "x": x, "y": y,
            # face the work (inward, opposite the wall's own outward bearing)
            "yaw_deg": (berm_line["bearing_deg"] + 180.0) % 360.0,
            "prone": False, "anchor": "wall_berm", "rule": "R2,R11",
        })
    return out


def trapped_in_berm(berm_line, rng, debris=None, along_frac=None):
    """Rule 2: one figure partially buried in the SAME windward berm the
    diggers work at (rule 11's clustering — this is not a second, separate
    location). Burial is measured from real `debris` when the caller has
    it (`burial_fraction_from_debris` — "via debris overlap"), else from
    the placeholder `berm_profile` shape."""
    along = along_frac if along_frac is not None else rng.uniform(0.2, 0.8)
    offset = berm_offset(rng)
    x, y = _along_wall(berm_line, along, offset)
    frac = (burial_fraction_from_debris(x, y, debris) if debris
            else burial_fraction(offset, rng=rng))
    pose = _lying_pose_for_burial(frac, rng)
    return {
        "class": "trapped", "pose": pose, "x": x, "y": y,
        "yaw_deg": rng.uniform(0.0, 360.0), "prone": True,
        "offset_m": offset, "burial_frac": frac,
        "anchor": "wall_berm", "rule": "R2",
    }


def crush_victim(berm_line, rng, macroblock_present, along_frac=None):
    """Rule 4: a pedestrian or parked car crushed OUTDOORS, 3-6 m from a
    URM wall that failed outward — GATED to fire ONLY when the cell's own
    collapse recipe actually shed a macroblock (`macroblock_present`,
    e.g. `bool(ctx["plan"].get("macroblocks"))` after `wreck_kit`/
    `wreck_urban` runs). Returns `None` when the gate is closed — never a
    guessed placement at an ordinary damaged building. Half the time the
    victim is a pedestrian (lying), half a parked-car occupant (seated) —
    both mechanisms are independently documented (Waco: car; Fort Worth/
    Atlanta: pedestrian)."""
    if not macroblock_present:
        return None
    along = along_frac if along_frac is not None else rng.uniform(0.3, 0.7)
    offset = rng.uniform(*CRUSH_OFFSET_RANGE_M)
    x, y = _along_wall(berm_line, along, offset)
    is_vehicle = rng.random() < 0.5
    if is_vehicle:
        pose = _weighted_choice(rng, _SEATED_POSES)
        yaw = berm_line["bearing_deg"]   # parked broadside/along the street
    else:
        pose = _weighted_choice(rng, _VISIBLE_LYING_POSES)
        yaw = rng.uniform(0.0, 360.0)
    return {
        "class": "crush_victim", "pose": pose, "x": x, "y": y,
        "yaw_deg": yaw, "prone": not is_vehicle, "vehicle": is_vehicle,
        "offset_m": offset, "anchor": "wall_outdoor", "rule": "R4",
    }


def pile_edge_digger(pile_edge, rng, n=2, spread_m=2.0):
    """Rule 11: `n` (default 2) rescuer figures at the edge of an
    industrial-collapse pile, clustered within `spread_m` — the same
    tight-bunch pattern `digger_pair` uses, at a point anchor rather than a
    wall run (`pile_edge["along_m"]` is ignored here; use `along_m` on the
    anchor only if the pile itself has a long straight edge worth sliding
    along — most collapse piles read better as a point)."""
    out = []
    for _ in range(int(n)):
        offset = rng.uniform(0.5, 2.5)
        along = rng.uniform(-spread_m, spread_m)
        r = math.radians(pile_edge["bearing_deg"])
        ax, ay = -math.sin(r), math.cos(r)
        ox, oy = math.cos(r), math.sin(r)
        x = pile_edge["x"] + ax * along + ox * offset
        y = pile_edge["y"] + ay * along + oy * offset
        pose = _weighted_choice(rng, _DIGGER_POSES)
        out.append({
            "class": "digger", "pose": pose, "x": x, "y": y,
            "yaw_deg": (pile_edge["bearing_deg"] + 180.0) % 360.0,
            "prone": False, "anchor": "pile_edge", "rule": "R11",
        })
    return out


def covered_casualty(pile_center, rng, debris=None):
    """Rule 2 (Walmart's pattern — even inside a partial collapse, deaths
    cluster away from the exterior wall): one figure near the geometric
    centre of a collapse pile, burial measured the same way
    `trapped_in_berm` does."""
    offset = rng.uniform(0.5, 3.0)
    r = math.radians(pile_center["bearing_deg"])
    ox, oy = math.cos(r), math.sin(r)
    x = pile_center["x"] + ox * offset
    y = pile_center["y"] + oy * offset
    frac = (burial_fraction_from_debris(x, y, debris) if debris
            else burial_fraction(offset, rng=rng, span_m=3.0))
    pose = _lying_pose_for_burial(frac, rng)
    return {
        "class": "covered_casualty", "pose": pose, "x": x, "y": y,
        "yaw_deg": rng.uniform(0.0, 360.0), "prone": True,
        "offset_m": offset, "burial_frac": frac,
        "anchor": "pile_center", "rule": "R2",
    }


def doorway_threshold(door, rng, n=None):
    """Rule 7: 1-2 figures AT a designated refuge's own doorway threshold —
    the Pizza Hut cooler pattern (the person at the door is
    disproportionately the casualty; the interior stays MOSTLY clear).
    Places ONLY at the threshold — this function authors nothing interior,
    by construction. `n` defaults to a coin-flip between 1 and 2."""
    if n is None:
        n = 1 if rng.random() < 0.5 else 2
    out = []
    for _ in range(int(n)):
        jitter = rng.uniform(-0.6, 0.6)
        r = math.radians(door["bearing_deg"])
        ax, ay = -math.sin(r), math.cos(r)
        x = door["x"] + ax * jitter
        y = door["y"] + ay * jitter
        pose = _weighted_choice(rng, _DOORWAY_POSES)
        out.append({
            "class": "doorway", "pose": pose, "x": x, "y": y,
            "yaw_deg": (door["bearing_deg"] + 180.0) % 360.0,
            "prone": False, "anchor": "doorway", "rule": "R7",
        })
    return out


def car_occupant(car, rng, seat="driver"):
    """Rule 3: an occupant in a struck car — vehicles are a first-order
    urban casualty class (~20 %, `VEHICLE_SHARE`), roughly double the
    suburb's incidental share. `car` names the car's OWN position/heading
    (`bearing_deg` here means the car's facing, not an outward wall
    normal); the caller places the actual car prim separately and should
    pass its FINAL resting transform if `tornado.car_pose` moved it."""
    pose = _weighted_choice(rng, _SEATED_POSES)
    return {
        "class": "car_occupant", "pose": pose, "x": car["x"], "y": car["y"],
        "yaw_deg": car["bearing_deg"], "prone": False, "seat": seat,
        "anchor": "car", "rule": "R3",
    }


def evacuee_pair(entry, rng, spacing_m=1.2, stand_off_m=(1.5, 3.5)):
    """Standing entry class (unchanged from the plan brief — NOT a
    casualty; not gated by `casualty_gate`): two upright evacuees near a
    standing building's own door, spaced `spacing_m` apart, `stand_off_m`
    out from the threshold."""
    out = []
    r = math.radians(entry["bearing_deg"])
    ax, ay = -math.sin(r), math.cos(r)
    ox, oy = math.cos(r), math.sin(r)
    for s in (-0.5, 0.5):
        offset = rng.uniform(*stand_off_m)
        x = entry["x"] + ax * s * spacing_m + ox * offset
        y = entry["y"] + ay * s * spacing_m + oy * offset
        pose = _weighted_choice(rng, _EVACUEE_POSES)
        out.append({
            "class": "evacuee", "pose": pose, "x": x, "y": y,
            "yaw_deg": entry["bearing_deg"], "prone": False,
            "anchor": "entry", "rule": "evacuee (unchanged)",
        })
    return out


# ---------------------------------------------------------------------------
# The one bridge to `pxr`/`disaster.people` — everything above stays pure.
# ---------------------------------------------------------------------------

def to_placement(ctx, spec, usd, z_ground=0.0):
    """One spec from the classes above -> a `people._human_placement`-shaped
    placement dict (`usd`/`x_m`/`y_m`/`z_m`/`yaw_deg`/`roll_deg`/
    `pitch_deg`/`scale`/`category`/`axis_up`/`pose`), via the SAME shipped
    machinery `tornado_people`/`fire_people` delegate to — see this
    module's own docstring, "WHY NOT REIMPLEMENT". `ctx` needs only the two
    keys `_human_placement` itself reads (`"asset_pools"`, `"resolver"`) —
    a caller does not need the suburb's full `people.build_ctx`. Imports
    `disaster.people` LAZILY so every OTHER function in this module stays
    importable with no `pxr` anywhere on the path."""
    from . import people
    return people._human_placement(
        ctx, usd, spec["x"], spec["y"], z_ground, spec["yaw_deg"],
        spec["pose"], prone=bool(spec.get("prone", False)))
