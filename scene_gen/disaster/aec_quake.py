#!/usr/bin/env python
"""aec_quake.py -- earthquake damage for an AEC brownstone ROW, by name.

    import aec_quake
    aec_quake.quake_row(stage, "/World/b0", grade="DG4", seed=7)

USER DIRECTIVE (2026-09-02): "brownstone aec has a new skill file on how to
damage it, we did it for fire but you need to include it for earthquake
too." The "skill file" is `disaster/aec_burn.py`'s own method -- read the
asset's own named-part hierarchy, de-instance the burning/damaged units so a
bind or a deactivate does not silently resolve to the shared prototype, then
address parts by CATEGORY NAME (`Windows`/`Doors`/`Stairs`/`Railings`/
`Roofs`/`Walls_Exterior`/`Floors`/`Structural_Framing`/...) and by geometric
position (interior vs exterior, storey, side) rather than cutting the mesh
the way `monolith_damage.cut_shell` does. That cutter is the WRONG tool
here: it assumes a hardcoded -Y front and a `uv0` primvar this asset never
carries (`tools/openings_probe.py`'s own Tf error, recorded in `quake.py`'s
PRISTINE_PACKS header and the `build-earthquake-scenes` skill's round-6
"open" item, `scratchpad/design/mono_wiring.md`) -- an AEC row is already a
kit internally (307-3,684 named part meshes measured across the pack, one
`Reference_Brownstone<N>Row` per terrace unit under a `Brownstone02_
{Instanced,Referenced}` prototype), so what it needs is de-instance + a
named-part ladder, exactly `aec_burn`'s own shape, ported from fire levels to
EMS-98 grades.

THIS MODULE IMPORTS `aec_burn`'S ADDRESSING RATHER THAN COPYING IT.
`aec_burn.py` is fire-owned (another session's live file, `scene_gen/tests/
test_aec_burn.py` + the render pipeline both depend on its exact behaviour)
-- this module treats it as a read-only library and reuses, by name, the
parts of it that are disaster-agnostic geometry/addressing rather than fire
model: `measure_row` (the asset reader: units, bboxes, window islands, wall
planes, storey levels, the instanceable prim to de-instance), `_win_id`
(window-island grouping), `_is_interior` (interior/exterior classification),
`_flat_material`/`_kill`/`_author_body`/`_box_tris`/`_debris_scope`/
`_author_tris` (the no-physics authoring primitives: a flat PreviewSurface,
deactivate-and-mark-dead, a static box-chunk of debris, a conformal standoff
decal). None of `aec_burn`'s FIRE-specific code (soot skins, char materials,
flame events, `LADDER`'s fire levels) is used or duplicated here.

GRADE LADDER (construction type `urm`, per aec_burn's own docstring note
that FAMILY_TYPE["03"] -- this pack's brownstone family -- scores as urm for
the DAMAGE ladder even where a texture lookup elsewhere treats it
differently). Applied PER TERRACE UNIT so one row shows variation rather
than every unit taking identical damage -- cumulative, each grade implying
everything the grade below it does:

  DG1  cosmetic: a roof-plant part (the closest thing this asset's own
       category vocabulary has to "a chimney") toppled on a FEW units, a
       handful of roof-edge pieces dropped (the parapet/cornice run, this
       asset's own unlabelled-by-position category -- see
       `_parapet_edge_parts`), a couple of small facade SCAR decals (the
       same conformal-standoff-quad idiom `aec_burn`'s soot layer and
       `wall_overlay` both use, reused here with a flat dark material
       instead of a soot skin).
  DG2  + the parapet/cornice run removed along the WHOLE row (not just a
       few pieces) with kerb debris at every unit's foot, some window UNITS
       voided (glass+sash+frame all killed as one group, `aec_burn._win_id`'s
       own grouping).
  DG3  + 1-2 units' roof (deck + roof plant) removed/collapsed inward, more
       and bigger scar decals ("wall cracks"), stoop/railing scatter on a
       couple of units, a SMALL rubble pile at the frontage.
  DG4  + one unit's Walls_Exterior (this asset's own single mesh for BOTH
       the front and rear wall -- `aec_burn`'s docstring) is largely
       removed along with its windows/doors/exterior fixtures, exposing its
       Floors (never touched -- exposed, not removed); a bigger share of
       roof loss across units; a BIG rubble pile at the frontage.
  DG5  majority collapse: most units (never ALL -- one is always left
       standing, the "gable/party-wall remnant" this asset has no separate
       gable mesh to model any other way) lose their walls and roofs; a
       HUGE pile burying the frontage.

NO PHYSICS. Every removal is a `aec_burn._kill` (`SetActive(False)`, marked
dead) or a hand-placed static box-chunk (`aec_burn._box_tris` +
`aec_burn._author_body` -- these author PLAIN static geometry; a "rigid body
CANDIDATE" only becomes one when a caller later runs PhysX APIs over it via
`disaster.settle.run`, which this module never calls). This is the user's
"lazy... place some by hand" directive (`SETTLE_BODY_BUDGET`'s own header in
`quake_collapse.py`) applied to AEC from the start rather than retrofitted:
no `settle.run`, no convergence risk, no body-count budget to manage --
just a debris PRIM-count budget (`debris_budget`, default
`AEC_QUAKE_DEBRIS_BUDGET` env or 80 per row) since nothing here is ever
simulated. Debris lands at a KNOWN reference height (the unit's own ground
level or roof deck, both already measured by `measure_row`) rather than a
`quake_collapse._deck_support_z` stage-wide query -- that helper earns its
keep when a piece might land on an arbitrary neighbour's roof; every piece
authored here lands on one of this row's own two known planes.

THE FRONTAGE PILE reuses the verified rubble-v2 planner/emitter directly --
`quake_rubble.plan_pile` (kind `"windrow"`/`"fan"`, `sides` restricted to the
row's own street-facing side, `along` narrowed to the damaged units' own
span) + `quake_rubble_usd.author` -- rather than `aec_burn`'s own ad hoc
brick/mound authoring (which exists there because the fire ladder's
progressive burn-through wall loss needed a bespoke shape; an earthquake
pile is architecturally the SAME kind of debris every other `urm` collapse
in this dataset already produces `plan_pile` for).

TILT is NOT this module's job. `disaster.quake._mono_pass` still draws the
"modest lean on top" this ladder's own docstring in `quake.py` describes
(reduced-magnitude versions of its existing rigid-lean constants) -- this
module only authors the named-part damage and the pile.
"""
import os
import random
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.normpath(os.path.join(_HERE, "..")), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:                                      # package import (disaster.aec_quake)
    from . import aec_burn as ab
except ImportError:                       # bare-script / sys.path import
    import aec_burn as ab                 # noqa: F401


# ---------------------------------------------------------------------------
# tunables
# ---------------------------------------------------------------------------
DEBRIS_BUDGET_DEFAULT = int(os.environ.get("AEC_QUAKE_DEBRIS_BUDGET", "80"))
#: dusty masonry/plaster rubble -- a fallen cornice, kerb chip or stoop chunk
DEBRIS_RGB = (0.145, 0.125, 0.105)
DEBRIS_ROUGH = 0.95
#: a facade scar/crack decal -- dark, matte, no soot-skin gradient (that is
#: fire's own look; an earthquake crack is a flat shadow line)
CRACK_RGB = (0.050, 0.045, 0.040)
CRACK_ROUGH = 0.90
CHUNK_SIZE_M = (0.22, 0.14, 0.10)
CORNICE_SIZE_M = (0.55, 0.30, 0.18)

#: frontage-pile sizing per grade tier -- `quake_rubble.plan_pile` kind,
#: outward depth range, the "element height that fell" scale (drives the
#: pile's reach -- see that module's own `elem_h_m` docstring), and how far
#: past the damaged unit(s)' own span (metres) the pile's along-axis window
#: is padded. `elem_h_m=None` (the "huge" tier) uses the row's own full
#: height instead of a fixed figure -- "burying the frontage" scales with
#: the building, not a constant.
PILE_TIER = {
    "small": dict(kind="windrow", depth_m=(0.6, 1.0), elem_h_m=3.2, span_pad=1.5),
    "big":   dict(kind="fan",     depth_m=(1.6, 2.6), elem_h_m=6.5, span_pad=2.5),
    "huge":  dict(kind="fan",     depth_m=(2.6, 3.8), elem_h_m=None, span_pad=4.0),
}


# ---------------------------------------------------------------------------
# THE DAMAGE LADDER -- see the module docstring for the prose version.
# Cumulative, `dict(LADDER["DGn-1"], ...)`, same construction `aec_burn.
# LADDER` uses. `roof_units`/`wall_units`: a `(lo, hi)` tuple is an explicit
# UNIT COUNT (`random.randint`), a bare float is a FRACTION of the row's own
# unit count (`_select_units`) -- DG3's "1-2 units" is a count, DG4/DG5's
# "a share of the row" is a fraction.
# ---------------------------------------------------------------------------
LADDER = {}
LADDER["DG0"] = {}
LADDER["DG1"] = {"chimney_p": 0.5, "cornice_n": (1, 3), "scar_n": (1, 2)}
LADDER["DG2"] = dict(LADDER["DG1"], chimney_p=0.7, cornice_n=(2, 4),
                     scar_n=(2, 4), parapet_run=True, kerb_debris=True,
                     win_void_p=0.15)
LADDER["DG3"] = dict(LADDER["DG2"], chimney_p=0.9, cornice_n=(3, 5),
                     scar_n=(3, 6), win_void_p=0.30, roof_units=(1, 2),
                     stoop_scatter=True, pile="small")
LADDER["DG4"] = dict(LADDER["DG3"], chimney_p=1.0, cornice_n=(4, 6),
                     scar_n=(4, 8), win_void_p=0.45, roof_units=0.6,
                     wall_units=(1, 1), pile="big")
LADDER["DG5"] = dict(LADDER["DG4"], chimney_p=1.0, cornice_n=(5, 8),
                     scar_n=(6, 10), win_void_p=0.65, roof_units=0.9,
                     wall_units=0.7, pile="huge")


# ---------------------------------------------------------------------------
# unit selection + part-finding (by category name / position, `aec_burn`'s
# own idiom -- see the module docstring)
# ---------------------------------------------------------------------------
def _select_units(units, spec_val, rng, reserve=0):
    """`spec_val` a `(lo, hi)` count tuple or a fraction-of-n float. Never
    takes more than `len(units) - reserve` -- DG5's "majority collapse"
    still leaves a "gable/party-wall remnant" unit standing (`reserve=1`
    from the wall-loss caller); roof loss has no such reserve."""
    n = len(units)
    if isinstance(spec_val, tuple):
        hi = max(spec_val[0], min(spec_val[1], n))
        k = rng.randint(spec_val[0], hi) if hi >= spec_val[0] else 0
    else:
        k = max(1, int(round(float(spec_val) * n)))
    k = max(0, min(k, n - max(0, reserve)))
    if k <= 0:
        return []
    return rng.sample(list(units), k)


def _roof_plant_parts(unit):
    """Roof-mounted mechanical/railing parts -- the closest this asset's own
    category vocabulary comes to "a chimney" (aec_burn's `damage_row` treats
    the same categories, at the same height test, as "roof plant")."""
    deck_z = unit["deck_z"]
    out = []
    for mrec in unit["meshes"]:
        if mrec.get("dead"):
            continue
        b = mrec["bbox"]
        cat = mrec["cat"]
        if 0.5 * (b[2] + b[5]) >= deck_z - 0.05 and (
                cat in ("Mechanical_Equipment", "Specialty_Equipment")
                or cat.startswith(("Railings", "Top_Rails"))):
            out.append(mrec)
    return out


def _roof_deck_parts(unit):
    return [m for m in unit["meshes"] if not m.get("dead")
            and m["cat"].startswith("Roofs")]


def _parapet_edge_parts(unit, meas, frac=0.35):
    """The `Roofs`-category parts nearest the FRONT wall plane -- a
    parapet/cornice run sits over the street, and this asset does not label
    that sub-category separately (`aec_burn`'s own docstring: an unlabelled
    category is "told apart from the labelled ones by LOCATION only"). At
    least one part if the unit has any roof parts at all."""
    perp = meas["perp"]
    parts = _roof_deck_parts(unit)
    if not parts:
        return []
    scored = sorted(parts, key=lambda m: abs(
        0.5 * (m["bbox"][perp] + m["bbox"][3 + perp]) - unit["plane_lo"]))
    k = max(1, int(round(len(scored) * frac)))
    return scored[:k]


def _void_windows(stage, unit, rng, p, stats):
    """Kill whole window-UNIT groups (`aec_burn._win_id`'s grouping: glass +
    sash + frame, whatever meshes share one window's id) with probability
    `p` each -- "some window units voided", not individual panes."""
    groups = {}
    for mrec in unit["meshes"]:
        if mrec["cat"] == "Windows" and not mrec.get("dead"):
            groups.setdefault(ab._win_id(mrec["name"]), []).append(mrec)
    n = 0
    for _wid, ms in groups.items():
        if rng.random() < p:
            for r in ms:
                ab._kill(stage, r, stats["killed"], "window_void")
            n += 1
    return n


def _drop_chunk(stage, scope, mat, pos, size, rng, tag, stats, key, budget):
    """One static box-chunk of debris at `pos` (world metres), budget-capped
    -- see the module docstring on why this never becomes a physics body."""
    if stats["debris_n"] >= budget:
        return None
    V, N = ab._box_tris(pos, size, rng.uniform(0.0, 360.0), rng.uniform(-25.0, 25.0))
    path, _c = ab._author_body(
        stage, "{0}/{1}_{2}".format(scope, tag, stats["debris_n"]), V, mat)
    stats["debris"].append(path)
    stats["debris_n"] += 1
    stats["debris_kinds"][key] = stats["debris_kinds"].get(key, 0) + 1
    return path


def _author_scar(stage, meas, unit, root, mpu, xf, geo, mat, rng, tag):
    """A small conformal standoff decal on the unit's FRONT wall -- the same
    idiom `aec_burn`'s soot layer and `wall_overlay.author_quad` both use (a
    quad pushed `aec_burn.STANDOFF_M` off the real surface, revealed by an
    opacity-textured/flat material), authored directly in this asset's own
    wall-plane coordinates rather than through `quake_flow._piece_frame` (an
    AEC row has none -- `wall_overlay.author_quad` cannot be reused as-is).
    Reuses `aec_burn._author_tris` for the actual world-to-root-local
    authoring (points, normal-pushed offset, `st`, material bind, extent)."""
    perp = meas["perp"]
    along = 1 - perp
    front = "W" if perp == 0 else "S"
    plane = unit["plane_lo"] if front in ("W", "S") else unit["plane_hi"]
    b = unit["bbox"]
    a0, a1 = b[along], b[3 + along]
    span = min(a1 - a0 - 0.4, rng.uniform(0.35, 0.9))
    if span <= 0.05:
        return None
    ac = (rng.uniform(a0 + span / 2.0 + 0.2, a1 - span / 2.0 - 0.2)
          if a1 - a0 > span + 0.4 else 0.5 * (a0 + a1))
    z0 = b[2] + rng.uniform(0.2, 0.6)
    z1 = min(b[5] - 0.2, z0 + rng.uniform(0.5, 1.6))
    if z1 <= z0:
        return None

    def _pt(a, z):
        p = [0.0, 0.0, z]
        p[along] = a
        p[perp] = plane
        return p

    corners = [_pt(ac - span / 2.0, z0), _pt(ac + span / 2.0, z0),
               _pt(ac + span / 2.0, z1), _pt(ac - span / 2.0, z1)]
    uvs = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    C = np.asarray(corners, dtype=np.float64)
    U = np.asarray(uvs, dtype=np.float64)
    tri_idx = ((0, 1, 2), (0, 2, 3))
    V = np.asarray([[C[i] for i in t] for t in tri_idx])
    UV = np.asarray([[U[i] for i in t] for t in tri_idx])
    e1, e2 = V[:, 1] - V[:, 0], V[:, 2] - V[:, 0]
    N = np.cross(e1, e2)
    L = np.linalg.norm(N, axis=1, keepdims=True)
    N = N / np.maximum(L, 1e-9)
    out_sign = -1.0 if front in ("W", "S") else 1.0
    want = np.zeros(3)
    want[perp] = out_sign
    if float(np.dot(N[0], want)) < 0.0:
        V = V[:, ::-1, :]
        UV = UV[:, ::-1, :]
        N = -N
    path = "{0}/{1}_{2}".format(geo, tag, unit["name"].rsplit("_", 1)[-1])
    ab._author_tris(stage, path, root, mpu, xf, V, N, UV, ab.STANDOFF_M, mat)
    return path


def _remove_roof(stage, unit, stats):
    """Roof deck + roof plant, all of it -- "collapsed inward", never a
    partial cut (no VTK clip; a whole named part is either there or it is
    not, `aec_burn`'s own by-name idiom)."""
    parts = _roof_deck_parts(unit) + _roof_plant_parts(unit)
    for m in parts:
        ab._kill(stage, m, stats["killed"], "roof_gone")
    return len(parts)


def _remove_wall(stage, unit, meas, stats):
    """This unit's `Walls_Exterior` (front AND rear, one mesh -- `aec_burn`'s
    docstring), its Windows and Doors, and any exterior fixture actually set
    into that wall. Everything set in a lost wall goes with it (`aec_burn.
    _lose_wall`'s own note: left standing they read as floating window
    frames) -- Floors are deliberately NOT in this list; they are what "the
    wall exposes", not what falls with it."""
    n = 0
    for m in unit["meshes"]:
        if m.get("dead"):
            continue
        cat = m["cat"]
        if cat in ("Walls_Exterior", "Windows", "Doors"):
            ab._kill(stage, m, stats["killed"], "wall_gone")
            n += 1
        elif cat in ("Generic_Models", "Lighting_Fixtures") and not ab._is_interior(
                m, unit, meas["perp"], unit["deck_z"]):
            ab._kill(stage, m, stats["killed"], "wall_gone")
            n += 1
    return n


def _do_unit(stage, meas, unit, spec, rng, root, mpu, xf, decal_scope,
            debris_scope, debris_mat, scar_mat, stats, budget):
    """The per-unit cosmetic/moderate actions -- rolled independently per
    unit (even at a fixed row grade) so a row shows variation rather than
    every unit taking identical damage."""
    deck_z = unit["deck_z"]
    ground_z = unit["bbox"][2]
    perp = meas["perp"]
    along = 1 - perp
    front = "W" if perp == 0 else "S"
    out_sign = -1.0 if front in ("W", "S") else 1.0
    plane = unit["plane_lo"] if front in ("W", "S") else unit["plane_hi"]
    utag = unit["name"].rsplit("_", 1)[-1]

    # 1. chimney topple: kill one roof-plant part, leave a chunk on the deck
    # near where it stood (a toppled unit does not fall off the building).
    plants = _roof_plant_parts(unit)
    if plants and rng.random() < spec.get("chimney_p", 0.0):
        mrec = rng.choice(plants)
        b = mrec["bbox"]
        pos = [0.5 * (b[0] + b[3]), 0.5 * (b[1] + b[4]), deck_z + 0.12]
        sz = (max(0.3, b[3] - b[0]), max(0.3, b[4] - b[1]), max(0.25, b[5] - b[2]))
        ab._kill(stage, mrec, stats["killed"], "chimney")
        _drop_chunk(stage, debris_scope, debris_mat, pos, sz, rng,
                   "chimney_" + utag, stats, "chimney", budget)

    # 2. cornice pieces dropped (DG1, a few) / a parapet RUN (DG2+, most of
    # the front edge) -- see `_parapet_edge_parts`'s own docstring for why
    # this is the addressable stand-in for "cornice".
    if spec.get("parapet_run"):
        targets = _parapet_edge_parts(unit, meas)
    else:
        cn = spec.get("cornice_n")
        pool = _roof_deck_parts(unit)
        targets = (rng.sample(pool, min(len(pool), rng.randint(*cn)))
                  if pool and cn else [])
    for mrec in targets:
        b = mrec["bbox"]
        ab._kill(stage, mrec, stats["killed"], "cornice")
        pos = [0.0, 0.0, ground_z + rng.uniform(0.1, 0.3)]
        pos[along] = 0.5 * (b[along] + b[3 + along]) + rng.uniform(-0.3, 0.3)
        pos[perp] = plane + out_sign * rng.uniform(0.3, 1.3)
        _drop_chunk(stage, debris_scope, debris_mat, pos, CORNICE_SIZE_M, rng,
                   "cornice_" + utag, stats, "cornice", budget)

    # 3. kerb debris: a small scatter at the foot (DG2+).
    if spec.get("kerb_debris"):
        b = unit["bbox"]
        for _i in range(rng.randint(2, 5)):
            pos = [0.0, 0.0, ground_z + rng.uniform(0.03, 0.12)]
            pos[along] = rng.uniform(b[along] + 0.3, b[3 + along] - 0.3)
            pos[perp] = plane + out_sign * rng.uniform(0.15, 0.7)
            _drop_chunk(stage, debris_scope, debris_mat, pos, CHUNK_SIZE_M, rng,
                       "kerb_" + utag, stats, "kerb", budget)

    # 4. window units voided.
    p = spec.get("win_void_p", 0.0)
    if p > 0.0:
        stats["window_units_voided"] += _void_windows(stage, unit, rng, p, stats)

    # 5. facade scars (DG1+) / wall cracks (DG3+, same idiom, `scar_n`
    # already grows with grade).
    n_scar = rng.randint(*spec["scar_n"]) if spec.get("scar_n") else 0
    for i in range(n_scar):
        p2 = _author_scar(stage, meas, unit, root, mpu, xf, decal_scope,
                          scar_mat, rng, "scar{0}".format(i))
        if p2:
            stats["scars"] += 1

    # 6. stoop / railing scatter (DG3+): the low (< 3 m) stair/railing parts.
    if spec.get("stoop_scatter"):
        stoop = [m for m in unit["meshes"] if not m.get("dead")
                and m["cat"].startswith(("Stairs", "Runs", "Railings", "Top_Rails"))
                and 0.5 * (m["bbox"][2] + m["bbox"][5]) < ground_z + 3.0]
        for mrec in rng.sample(stoop, min(len(stoop), rng.randint(1, 3))):
            b = mrec["bbox"]
            ab._kill(stage, mrec, stats["killed"], "stoop")
            pos = [0.5 * (b[0] + b[3]), 0.5 * (b[1] + b[4]), ground_z + 0.08]
            _drop_chunk(stage, debris_scope, debris_mat, pos, CHUNK_SIZE_M, rng,
                       "stoop_" + utag, stats, "stoop", budget)


# ---------------------------------------------------------------------------
# the frontage pile (`quake_rubble.plan_pile` + `quake_rubble_usd.author`)
# ---------------------------------------------------------------------------
def _row_mass(meas):
    """A `quake_flow`/`quake_rubble` mass dict for the WHOLE row -- same
    shape `aec_burn.plan_row` builds for its fire mass box, yaw always 0.0
    (this asset is placed axis-aligned in its own local frame; the row
    prim's own placement yaw/translate is a separate xform the caller
    applies on top, exactly as `aec_burn`'s own `m` assumes)."""
    units = meas["units"]
    bb = [min(r["bbox"][i] for r in units) for i in range(3)] + \
         [max(r["bbox"][i] for r in units) for i in range(3, 6)]
    x0, y0, z0, x1, y1, z1 = bb
    deck_z = float(np.median([r["deck_z"] for r in units]))
    lv_all = sorted(set(round(z, 1) for r in units for z in r["levels"][1:]))
    levels = [float(z0)]
    for z in lv_all:
        if z - levels[-1] > 1.5:
            levels.append(float(z))
    return {"cx": 0.5 * (x0 + x1), "cy": 0.5 * (y0 + y1), "yaw": 0.0,
            "W": float(x1 - x0), "D": float(y1 - y0), "z0": float(z0),
            "top": float(z1), "deck_z": deck_z, "levels": levels}


def _author_pile(stage, meas, m, tier, span_units, rng, root_path, stats,
                 mats=None, tag="aec_eq", asset_root=None,
                 flatten_instances=False):
    """One `quake_rubble.plan_pile` (`kind="windrow"`/`"fan"`, restricted to
    the row's own street-facing side) authored under `aec_burn._debris_scope`
    (a SIBLING of the row prim, so a later building-level lean does not tilt
    debris already resting on the ground). `span_units` (the roof- or
    wall-loss units, whichever this grade drew) narrows the along-axis
    window to where the damage actually is; the whole row's span otherwise
    ("huge", DG5 -- "burying the frontage")."""
    from . import quake_rubble, quake_rubble_usd
    cfg = PILE_TIER[tier]
    perp = meas["perp"]
    along = 1 - perp
    front = "W" if perp == 0 else "S"
    L = m["D"] if perp == 0 else m["W"]
    c = m["cy"] if perp == 0 else m["cx"]
    if span_units:
        lo = min(u["bbox"][along] for u in span_units) - cfg["span_pad"]
        hi = max(u["bbox"][3 + along] for u in span_units) + cfg["span_pad"]
    else:
        lo, hi = c - L / 2.0, c + L / 2.0
    t_lo = max(-0.5, (lo - c) / L)
    t_hi = min(0.5, (hi - c) / L)
    if t_hi - t_lo < 0.1:
        t_lo, t_hi = -0.15, 0.15
    elem_h = cfg["elem_h_m"] if cfg["elem_h_m"] is not None \
        else max(3.0, m["top"] - m["z0"])
    scope = ab._debris_scope(stage, root_path)
    plan = quake_rubble.plan_pile(
        m, "urm", rng, kind=cfg["kind"], sides=(front,), along=(t_lo, t_hi),
        depth_m=rng.uniform(*cfg["depth_m"]), elem_h_m=elem_h)
    ret = quake_rubble_usd.author(stage, scope, plan, mats=mats, tag=tag,
                                  asset_root=asset_root,
                                  flatten_instances=flatten_instances)
    stats["pile"] = {"tier": tier, "scope": scope, "n": len(ret.get("all") or []),
                     "stats": plan.get("stats")}
    return ret


# ---------------------------------------------------------------------------
# the entry point
# ---------------------------------------------------------------------------
def quake_row(stage, root_path, grade="DG3", seed=7, verbose=True,
             mats=None, debris_budget=None, asset_root=None,
             flatten_instances=False):
    """measure -> per-unit ladder damage -> row-level roof/wall removal ->
    frontage pile. Returns a stats dict. `grade="DG0"` (or any grade not in
    `LADDER`) is a no-op BEFORE any stage mutation -- `measure_row` only
    reads the stage, so a DG0 call is byte-identical to never having called
    this at all."""
    from pxr import Sdf, Usd, UsdGeom
    meas = ab.measure_row(stage, root_path, verbose=verbose)
    spec = LADDER.get(grade) or {}
    stats = {"grade": grade, "root": str(root_path), "n_units": len(meas["units"]),
             "deinstanced": 0, "killed": {}, "debris": [], "debris_n": 0,
             "debris_kinds": {}, "scars": 0, "window_units_voided": 0,
             "roof_unit_names": [], "wall_unit_names": [], "pile": None}
    if not spec:
        if verbose:
            print("[aec_quake] {0}: {1} -- no-op".format(root_path, grade))
        return stats

    rng = random.Random(seed)
    budget = DEBRIS_BUDGET_DEFAULT if debris_budget is None else int(debris_budget)
    root = stage.GetPrimAtPath(root_path)
    mpu = meas["mpu"]
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())

    # de-instance every unit up front -- named-part removal/authoring needs
    # real prims (a bind or a deactivate onto an instance proxy silently
    # resolves to the shared prototype, `aec_burn.author_row`'s own trap).
    for unit in meas["units"]:
        ip = unit.get("inst")
        if ip is not None and ip.IsInstanceable():
            ip.SetInstanceable(False)
            stats["deinstanced"] += 1
    if stats["deinstanced"]:
        for unit in meas["units"]:
            for mrec in unit["meshes"]:
                mrec["prim"] = stage.GetPrimAtPath(mrec["path"])
        xf.Clear()

    looks = stage.DefinePrim(Sdf.Path(root_path).AppendChild("QuakeLooks"), "Scope")
    debris_mat = ab._flat_material(stage, str(looks.GetPath().AppendChild("eq_debris")),
                                   DEBRIS_RGB, DEBRIS_ROUGH)
    scar_mat = ab._flat_material(stage, str(looks.GetPath().AppendChild("eq_scar")),
                                 CRACK_RGB, CRACK_ROUGH)
    decal_scope = str(stage.DefinePrim(
        Sdf.Path(root_path).AppendChild("QuakeDecals"), "Scope").GetPath())
    debris_scope = ab._debris_scope(stage, root_path)

    for unit in meas["units"]:
        _do_unit(stage, meas, unit, spec, rng, root, mpu, xf, decal_scope,
                debris_scope, debris_mat, scar_mat, stats, budget)

    units = meas["units"]
    roof_units, wall_units = [], []
    if spec.get("roof_units"):
        roof_units = _select_units(units, spec["roof_units"], rng, reserve=0)
        for u in roof_units:
            _remove_roof(stage, u, stats)
    if spec.get("wall_units"):
        # reserve=1: never remove every unit's wall -- the "gable/party-wall
        # remnant" the DG5 prose asks for.
        wall_units = _select_units(units, spec["wall_units"], rng, reserve=1)
        for u in wall_units:
            _remove_wall(stage, u, meas, stats)
    stats["roof_unit_names"] = [u["name"] for u in roof_units]
    stats["wall_unit_names"] = [u["name"] for u in wall_units]

    tier = spec.get("pile")
    if tier:
        m = _row_mass(meas)
        span_units = wall_units or roof_units
        _author_pile(stage, meas, m, tier, span_units, rng, root_path, stats,
                    mats=mats, asset_root=asset_root,
                    flatten_instances=flatten_instances)

    if verbose:
        print("[aec_quake] {0} {1}: {2} unit(s), {3} de-instanced, killed {4}, "
              "{5} debris chunk(s), {6} scar(s), {7} window unit(s) voided, "
              "pile={8}".format(
                  root_path, grade, stats["n_units"], stats["deinstanced"],
                  stats["killed"], stats["debris_n"], stats["scars"],
                  stats["window_units_voided"],
                  stats["pile"]["tier"] if stats["pile"] else None))
    return stats
