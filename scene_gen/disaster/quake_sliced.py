"""quake_sliced — the earthquake ladder for SLICED whole-asset buildings.

`quake_flow` breaks a building that `detail/urban_building.py` ASSEMBLED from
façade modules. A GreatAmericanCity / downtowncity asset is the opposite: one
merged mesh. `detail/gac_storey_slice.slice_to_kit` (or `kit_bake.load_kit`)
now cuts one into kit-shaped placements — roles wall / pier / corner / core /
roof / parapet on a storey x bay grid — and `quake_flow.describe` reads those
placements as an element table with no change at all. This module is what
takes that table apart.

WHY IT IS A SEPARATE MODULE AND NOT A FLAG ON `quake_flow`
----------------------------------------------------------
One constraint decides the whole design: **a sliced piece must never be handed
to `fracture.fracture_prim`.** VTK segfaults on a clipped shell (memory
`gac-fire-pipeline`; three crashes on `SM_Building_09`), and work package H is
diagnosing that in parallel. Every collapse recipe in `quake_flow` fractures —
`r_masonry_collapse`, `r_pancake`, `r_soft_storey`, `r_out_of_plane`,
`r_corner_fail` and `r_overturn` all call `_break` / `_break_split` /
`_p_ragged_courses` on kit pieces. So none of them can be pointed at a sliced
building as they stand, and this module expresses the same damage states with
a vocabulary that needs no fracture at all:

  * **REMOVAL** on the piece grid. The slicer already cut each bay into a wide
    opening panel between two narrow piers (`BAY_SPLITS = 0.26/0.48/0.26`), so
    removal alone can produce a TOOTHED edge — which is the whole reason a lost
    region does not read as a rectangular module cut-out.
  * **RIGID DISPLACEMENT** of whole pieces: a wall rotated about its own bottom
    outer edge until it lies on the street (a macroblock), the block above a
    crushed storey dropped and leaned, a piece laid on a rubble pile as a panel.
  * **the fit-out** (`quake_flow.fit_interior`, which works off the mass box and
    authors boxes — no fracture) so an opened storey shows floors and contents.
  * **rubble v2** (`disaster.quake_rubble` + `quake_rubble_usd`) for the pile:
    one heightfield mound, a handful of large elements, instanced scatter.

Everything else — the ctx shape, the fit-out, the roof dressing, the settle
and the bake — is `quake_flow`'s, unchanged, so the bake launcher does not
have to know which kind of building it was handed.

WHAT THIS MODULE DELIBERATELY DOES NOT DO
-----------------------------------------
No fracture, therefore: no ragged course lines cut into a surviving wall
(`_p_ragged_courses`), no torn slab rims (`_a_slab_rim`), no interior litter
(`_disturb_interior` fractures a kit piece and is NOT called — the plan only
RECORDS which storeys and sides were opened, in `plan["interior"]`, for a
later pass once package H lands), and no shell fragments. Those are the four
things to revisit when a clipped shell can be fractured again.

THE SHAPE OF THE CODE
---------------------
    plan_damage(info, elements, grade_or_recipes, btype, rng) -> plan
        PURE. Arithmetic on the element table; no stage, no pxr, no USD. This
        is what the tests check, the way `fire_collapse.plan_partial_collapse`
        is checked by `tests/test_fire_collapse.py`.
    apply_plan(stage, ctx, plan)
        The USD half: deactivate, transform, pile, ground response.
    wreck_sliced(...) -> ctx
        The entry point, with `quake_flow.wreck_building`'s ctx shape exactly.
"""

import json as _json
import math
import os as _os
import random

import numpy as np

# `quake_flow` imports pxr only INSIDE its stage-touching functions, so this is
# a host-safe import and the frame maths below is the SAME function the kit
# ladder uses rather than a copy of it that can drift.
from . import quake_flow as qf

_to_world = qf._to_world
_to_local = qf._to_local
_outward = qf._outward

# ---------------------------------------------------------------------------
# CONSTRUCTION TYPE — from measured data, never from a family default
# ---------------------------------------------------------------------------
# A kit building declares its family and `quake_flow.FAMILY_TYPE` maps that to
# urm / rc / rc_glass. A merged asset declares nothing, and the height guess
# `gac_fire.prepare` makes ("urm below 25 m, rc above") is wrong for most of
# the stock: `SM_Building_02` is 38.6 m of brick and `SM_Building_07` is 71 m
# of concrete, and the guess calls both `rc`.
#
# The evidence is `_plans/gac_building_material.json` — per GAC asset, the
# façade area in m2 of each material family, measured off the asset's own
# GeomSubsets — plus `_plans/gac_buildings.json` for W/D/H and the asset set's
# own `material:` key for the standalone packs.
MATERIAL_TYPE = {
    "brick": "urm",
    "stone": "urm",
    "concrete": "rc",
    "glass": "rc_glass",
    "steel": "rc_glass",       # a steel-framed asset in this library is a tower
}
# With no material evidence at all, height is the only signal left, and it is
# the same cut `gac_fire.prepare` uses: unreinforced masonry above eight
# storeys is rare enough to be the wrong default.
H_URM_MAX = 25.0

# Above this, a "brick" building is a FRAME WITH A MASONRY FAÇADE, not
# unreinforced masonry: the tallest load-bearing-masonry building ever built is
# the 16-storey Monadnock (66 m, with 1.8 m walls at its base), and nothing in
# this stock has walls like that. It still gets the URM ladder for its façade
# damage — the cladding is what an aerial camera sees — but at DG5 it comes
# down as a frame (`pancake`), not as a masonry heap. See `_guard`.
TALL_URM_M = 40.0
# No curtain-wall tower has ever collapsed in an earthquake (research §12 /
# the rc_glass cut table in `quake_flow.level_for_intensity`: DG5 < 1 %), so
# above this height the total-collapse recipes are refused outright.
TOWER_NO_COLLAPSE_M = 60.0

# ONE TABLE, ONE DECISION PER ASSET, EACH WITH ITS REASON.
# Keyed by asset BASENAME (no directory, no extension). `tests/
# test_quake_sliced.py` checks every row against the two JSON files, so this
# table cannot silently drift away from the measurement it came from.
CONSTRUCTION = {
    # ---- GreatAmericanCity (31 assets; material from gac_building_material.json,
    #      H from gac_buildings.json) ------------------------------------------
    "SM_Building_01": "urm",        # brick 1179 m2, 29x28x55 m
    "SM_Building_02": "urm",        # brick 387 m2, 28x14x39 m
    "SM_Building_03": "urm",        # brick 1332 m2, 29x29x63 m
    "SM_Building_04": "rc",         # NO material evidence at all; 32x29x53 m -> by height
    "SM_Building_05": "urm",        # brick 1243 m2, 30x28x72 m (tall: see TALL_URM_M)
    "SM_Building_06_Small": "urm",  # brick 518 m2, 29x15x59 m
    "SM_Building_07": "rc",         # concrete 730 m2, no brick at all, 29x15x71 m
    "SM_Building_08": "urm",        # brick 2272 m2 vs concrete 659, 86x58x59 m
    "SM_Building_09": "urm",        # brick 2273 m2 vs concrete 659, 44x58x59 m
    "SM_Building_10": "rc",         # concrete 4579 m2, no brick, 86x60x135 m
    "SM_Building_11": "rc",         # concrete 4273 m2, 45x109x64 m
    "SM_Building_12": "rc",         # concrete 1783 m2, 46x44x104 m
    "SM_Building_13": "rc",         # concrete 2866 m2, 30x45x140 m
    "SM_Building_14": "rc_glass",   # glass 19701 m2 against 297 concrete, 85x58x194 m
    "SM_Building_15": "rc_glass",   # glass 11922 m2, 85x57x231 m
    "SM_Building_16": "rc_glass",   # glass 17463 m2, 85x57x312 m
    "SM_Building_17": "rc",         # concrete 3203 m2 vs glass 2378, 85x57x135 m
    "SM_Building_18": "rc_glass",   # glass dominant, 42x42x150 m
    "SM_Building_19": "rc_glass",   # glass dominant, 29x42x70 m
    "SM_Building_20": "rc",         # concrete dominant, 62x85x151 m
    "SM_Building_21": "urm",        # brick, 58x73x78 m (tall: see TALL_URM_M)
    "SM_Building_22": "urm",        # brick, 58x71x61 m (tall)
    "SM_Building_23": "rc",         # concrete, 45x57x134 m
    "SM_Building_24": "rc_glass",   # glass, 29x58x43 m — a low glass block, not a tower
    "SM_Building_25": "urm",        # brick, 28x28x69 m (tall)
    "SM_Building_26": "rc_glass",   # glass, 14x28x69 m
    "SM_Building_27": "rc_glass",   # glass, 43x29x88 m
    "SM_Building_28": "rc_glass",   # glass, 45x71x77 m
    "SM_Building_29": "urm",        # brick, 14x28x48 m (tall)
    "SM_Building_30": "rc",         # concrete, 28x42x48 m
    "SM_Building_31": "rc",         # concrete, 60x142x302 m
    # ---- downtowncity (no `material:` key in urban_gac.yaml) -----------------
    # 42.3 x 48.8 x 231.4 m. A 231 m all-glass tower by inspection of the
    # asset (and of its name); the height rule alone would call it plain `rc`
    # and it would then be a candidate for `pancake`, which is exactly the
    # failure mode the tower guard exists to prevent.
    "Amar_Tower": "rc_glass",
    "Building_11": "rc",            # 30.9 x 35.4 x 32.6 m, no material key -> by height
    "Building_12": "rc",            # 44.3 x 18.7 x 38.9 m, no material key -> by height
    # ---- downtowncity Carved_* (12 blocks, `_plans/dtc_buildings.json`) ------
    # THE VALUES BELOW ARE EXACTLY WHAT THE HEIGHT RULE ALREADY RETURNED for
    # these assets (`H_URM_MAX` = 25 m against the measured H in the comment);
    # writing them down changes no behaviour and is not meant to. They are
    # here because `gac_fire.prepare` now asks this table for a downtowncity
    # block's construction type (2026-08-30, the `dtc:` fire path), and a
    # per-asset judgement that lives only in a fallback is not reviewable.
    # The evidence is thin on purpose: these are merged blocks with no
    # `material:` key and mixed brick/plaster/concrete façades
    # (`Brick_3`, `WallMaterial_03`, `Concrete_wall_with_cracks`,
    # `Plaster_011`), which is why height is still the deciding signal —
    # masonry infill at four to seven storeys, a frame above that. Revise a
    # row here, with its reason, if a façade-area measurement like
    # `gac_building_material.json` is ever made for this pack.
    "Carved_04": "urm",             # 34.1 x 39.1 x 16.0 m
    "Carved_17": "urm",             # 44.0 x 24.1 x 17.0 m
    "Carved_05": "urm",             # 44.4 x 44.3 x 18.6 m
    "Carved_06": "urm",             # 39.9 x 44.4 x 19.0 m
    "Carved_15": "urm",             # 83.9 x 44.4 x 19.0 m
    "Carved_21": "urm",             # 37.4 x 43.5 x 21.0 m
    "Carved_02": "urm",             # 43.2 x 44.3 x 22.0 m
    "Carved_14": "rc",              # 84.0 x 39.5 x 25.4 m — just over the cut
    "Carved_03": "rc",              # 42.6 x 44.4 x 27.6 m
    "Carved_01": "rc",              # 42.8 x 44.0 x 28.4 m
    "Carved_18": "rc",              # 44.4 x 44.4 x 29.2 m
    "Carved_13": "rc",              # 83.9 x 44.2 x 30.6 m
}


def construction_type(usd, H=None, material=None):
    """The construction type of one whole-asset building.

    `usd` is the asset path (or bare name); `H` its measured height in metres;
    `material` the asset set's own `material:` value if it carries one.

    Precedence: the per-asset table (measured evidence, and the only place an
    asset-specific judgement is allowed to live) -> the asset set's declared
    material -> height. Anything unrecognised falls back the way
    `gac_fire.prepare` does, so a new pack routes somewhere sane on day one.
    """
    base = str(usd or "").rstrip("/").rsplit("/", 1)[-1]
    for ext in (".usdc", ".usda", ".usdz", ".usd"):
        if base.lower().endswith(ext):
            base = base[: -len(ext)]
            break
    if base in CONSTRUCTION:
        return CONSTRUCTION[base]
    if material:
        t = MATERIAL_TYPE.get(str(material).strip().lower())
        if t:
            return t
    if H is not None:
        return "urm" if float(H) <= H_URM_MAX else "rc"
    return "urm"


# ---------------------------------------------------------------------------
# The rubble contract (mirrors `disaster.quake_rubble`, which is written in
# parallel — these are the numbers the PLANNER needs and it must not import
# that module to get them, or a test of this file would depend on that file.)
# ---------------------------------------------------------------------------
# Round 4, agent A's measured memo (`_plans/eq_round4_rubble_research.md`
# §1b / §5a / §5b): the crown of a total-collapse mound, as a fraction of the
# building height, CAPPED — a 300 m tower's pile is not 90 m high.
CROWN_FRAC = {"urm": 0.28, "rc": 0.30, "rc_glass": 0.12}
CROWN_MAX_M = 12.0
# These two are ESTIMATES ONLY, recorded in the pile spec as `crown_est_m` /
# `spread_est` so the plan says what it expects and a test can check it. They
# are NOT passed to `plan_pile`: the run-out is now drawn there, per side and
# asymmetrically (street side 0.35-0.65 H, blind sides 0.05-0.15 H), off the
# `sides` the plan names. Overriding it here would fossilise a duplicate of
# agent A's numbers in the wrong file.
SPREAD_EST = {"urm": 0.27, "rc": 0.38, "rc_glass": 0.30}

# A WINDROW'S REACH COMES FROM WHAT FELL, NOT FROM THE BUILDING.
# A parapet shed off a 120 m tower makes the same 1-2 m windrow it makes off a
# 15 m shop — the reach is ~1.0 x the height of the element that came down, so
# the pile spec carries `elem_h_m` (summed `_size[2]` of the removed pieces)
# and `plan_pile` sets the reach from it. Deriving reach from H put a 17 m
# apron of brick round a tower that had lost a coping.
WINDROW_DEPTH_M = {"parapet": 0.45, "wall": 0.80}

# TOTAL COLLAPSE, GROUND-FLOOR STUB (agent A §5b). A standing stub survives in
# 40-60 % of total collapses; when it does, 0.6-2.7 m of it is left. This
# module cannot CUT a stub down (no fracture on a clipped shell), so the stub
# is expressed on the piece grid instead: the storey-0 pieces on the fall side
# go, the other two or three sides stay, and `stub_h_m` (absolute metres, the
# storey-0 piece height) tells the pile planner how much shell the mound has
# to fill. `residual_m` records the height a later fracture pass should cut
# the stub to.
STUB_KEEP_P = (0.4, 0.6)
STUB_RESIDUAL_M = (0.6, 2.7)

# The slicer's own bay grammar. `BAY_SPLITS` is duplicated from
# `detail/gac_storey_slice.py` rather than imported: the planner must stay
# importable with no `detail` package on the path (the bake driver runs it
# from a bare process), and the only thing needed here is the ARITY.
BAY_SPLITS = (0.26, 0.48, 0.26)

SIDES = ("S", "E", "N", "W")
_CORNER_SIDES = {"SW": ("S", "W"), "SE": ("S", "E"),
                 "NW": ("N", "W"), "NE": ("N", "E")}
# Which END of a side's bay run a corner sits at. `ring()` lays the S and N
# runs west->east and the W and E runs south->north, so this is not symmetric.
_CORNER_END = {("S", "SW"): "lo", ("S", "SE"): "hi",
               ("N", "NW"): "lo", ("N", "NE"): "hi",
               ("W", "SW"): "lo", ("W", "NW"): "hi",
               ("E", "SE"): "lo", ("E", "NE"): "hi"}
_SIDE_NORMAL = {"S": (0.0, -1.0), "N": (0.0, 1.0),
                "E": (1.0, 0.0), "W": (-1.0, 0.0)}

# TOOTHING. When a region of façade is lost, its edge must WANDER on the piece
# grid or it reads as a rectangular module cut-out — the single most artificial
# thing a removal-only vocabulary can produce (the reviewer's standing note:
# "no rectangular module cut-outs or pristine neighbouring walls").
# Two draws do it, both on the boundary of the region:
KEEP_PIER_FRAC = (0.20, 0.40)    # piers on the boundary row that SURVIVE
STRAY_OPEN_FRAC = (0.15, 0.30)   # openings in the adjacent INTACT bays that go


# ---------------------------------------------------------------------------
# THE LADDERS — `quake_flow.LADDER`, grade by grade, in this vocabulary
# ---------------------------------------------------------------------------
# Same names, same grade semantics (EMS-98 DG1..DG5), so a bake launcher can
# hand a sliced style the same grade string it hands a kit style and get the
# same severity. What differs is only HOW the state is reached: removal and
# rigid displacement instead of fracture.
#
# `glass_loss`'s `frac` is a (lo, hi) pair drawn per building, from the field
# record: DG1 loses 5-15 % of ONE elevation's windows, DG2 20-40 %, and from
# DG3 up the racked storeys lose 40-70 %.
LADDER_S = {
    "urm": {
        "DG0": [],
        "DG1": [("glass_loss", {"frac": (0.05, 0.15), "sides": 1})],
        "DG2": [("parapet_fall", {"sides": 1, "frac": 0.5}),
                ("glass_loss", {"frac": (0.20, 0.40), "sides": 1})],
        "DG3": [("parapet_fall", {"sides": 2, "frac": 0.8}),
                ("corner_fail", {"storeys": 2}),
                ("glass_loss", {"frac": (0.40, 0.70), "sides": 2})],
        "DG4": [("out_of_plane", {"sides": 1, "from_storey": 1}),
                ("parapet_fall", {"sides": 3, "frac": 0.9}),
                ("corner_fail", {"storeys": 2}),
                ("glass_loss", {"frac": (0.40, 0.70), "sides": 2})],
        "DG5": [("masonry_collapse", {})],
    },
    "rc": {
        "DG0": [],
        "DG1": [("glass_loss", {"frac": (0.05, 0.15), "sides": 1})],
        # A frame's DG2 is dropped infill panels, which on the piece grid IS a
        # scatter of removed opening panels — `infill_fail` in `quake_flow`.
        "DG2": [("infill_fail", {"storeys": 1, "frac": 0.35}),
                ("parapet_fall", {"sides": 1, "frac": 0.4}),
                ("glass_loss", {"frac": (0.20, 0.40), "sides": 1})],
        "DG3": [("infill_fail", {"storeys": 2, "frac": 0.55}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("corner_fail", {"storeys": 1}),
                ("glass_loss", {"frac": (0.40, 0.70), "sides": 2})],
        "DG4": [("glass_loss", {"frac": (0.40, 0.70), "sides": 2}),
                ("storey_collapse", {})],
        "DG5": [("pancake", {})],
    },
    # The tower ladder: glass, then a podium soft storey, then a tilt. Never a
    # collapse — see TOWER_NO_COLLAPSE_M.
    "rc_glass": {
        "DG0": [],
        "DG1": [("glass_loss", {"frac": (0.01, 0.05), "sides": 1})],
        "DG2": [("glass_loss", {"frac": (0.10, 0.20), "sides": 1}),
                ("parapet_fall", {"sides": 1, "frac": 0.3})],
        "DG3": [("glass_loss", {"frac": (0.25, 0.40), "sides": 2}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("infill_fail", {"storeys": 1, "frac": 0.4})],
        "DG4": [("glass_loss", {"frac": (0.40, 0.55), "sides": 2}),
                ("soft_storey", {"storey": 0, "lean_deg": 2.5})],
        "DG5": [("glass_loss", {"frac": (0.40, 0.55), "sides": 3}),
                ("tilt_sink", {"tilt_deg": 9.0, "sink_m": 1.4})],
    },
}

# The foundation family, chosen by the ASSEMBLY (the soft-soil patch) and not
# by the shaking field, exactly as in `quake_flow.FOUNDATION`. These three are
# rigid transforms of the whole building plus authored ground art — no
# fracture anywhere in `r_settlement` / `r_tilt_severe` / `_c_ground_response`
# (verified by reading them), so they are routed straight to `quake_flow`.
# `overturn` is the exception: `quake_flow.r_overturn` DOES fracture the
# landing side and splits the roof strip, so this module has its own rigid
# version (`s_overturn` / `_ov_apply`).
FOUNDATION_S = {
    "SETTLE": [("settlement", {})],
    "TILT": [("tilt_severe", {})],
    "OV": [("overturn", {})],
}
for _t in LADDER_S:
    LADDER_S[_t].update({k: list(v) for k, v in FOUNDATION_S.items()})
# A podium-and-tower going over as one rigid body is not in the record.
LADDER_S["rc_glass"]["OV"] = []

GRADES = ("DG0", "DG1", "DG2", "DG3", "DG4", "DG5")
LEVELS = GRADES + ("SETTLE", "TILT", "OV")


def _guard(recs, btype, info):
    """Refuse the physically impossible, and say so in the plan's notes.

    Two rules, both from the reconnaissance record rather than from taste:
    a curtain-wall tower does not come down, and a 40 m+ "brick" building is
    a frame with a masonry façade and comes down as a frame.
    """
    H = float(info.get("H") or 0.0)
    out, notes = [], []
    total = ("pancake", "masonry_collapse")
    for name, kw in recs:
        kw = dict(kw or {})
        if name in total and btype == "rc_glass" and H > TOWER_NO_COLLAPSE_M:
            notes.append("guard: {0} refused on a {1:.0f} m curtain-wall tower "
                         "(no such collapse is on the record); glass + podium "
                         "soft storey instead".format(name, H))
            out.append(("glass_loss", {"frac": (0.40, 0.55), "sides": 2}))
            out.append(("soft_storey", {"storey": 0, "lean_deg": 3.0}))
            continue
        if name == "masonry_collapse" and btype == "urm" and H > TALL_URM_M:
            notes.append("guard: {0:.0f} m > {1:.0f} m of load-bearing masonry "
                         "is not a thing — brick façade on a frame, so DG5 is "
                         "a pancake".format(H, TALL_URM_M))
            out.append(("pancake", kw))
            continue
        out.append((name, kw))
    return out, notes


def _resolve(recipes, btype, info):
    """A grade string or an explicit [(name, kwargs)] -> the guarded list."""
    if isinstance(recipes, str):
        lad = LADDER_S.get(btype) or LADDER_S["urm"]
        if recipes not in lad:
            raise KeyError("no grade {0!r} in LADDER_S[{1!r}] (have {2})".format(
                recipes, btype, ", ".join(sorted(lad))))
        recs = [(n, dict(kw or {})) for n, kw in lad[recipes]]
    else:
        recs = [(n, dict(kw or {})) for n, kw in (recipes or ())]
    for name, _kw in recs:
        if name not in RECIPES_S:
            raise KeyError("unknown sliced recipe {0!r}".format(name))
    return _guard(recs, btype, info)


# ---------------------------------------------------------------------------
# RIGID TRANSFORMS — ONE implementation, in numpy, used by both halves
# ---------------------------------------------------------------------------
# The plan stores a displacement as a plain dict, not a matrix, so the whole
# plan is JSON-serialisable and a test can transform a point without pxr:
#
#     {"pivot": (x, y, z), "axis": (ax, ay, az), "deg": float,
#      "translate": (dx, dy, dz), "after": (dx, dy, dz)}
#
# and the matrix is  T(translate) . R(pivot, axis, deg) . T(after)  in USD's
# ROW-VECTOR convention (p' = p * M), which is what `quake_flow._transform_prims`
# post-multiplies with. Translate-then-rotate is the same order
# `quake_flow.r_soft_storey` composes its own drop-and-lean in.
#
# `apply_plan` feeds `rigid_matrix()`'s 16 numbers straight into a Gf.Matrix4d,
# so there is exactly one implementation of this arithmetic and the test that
# checks a macroblock lands in the street is checking the matrix that will be
# authored.


def _rot3(axis, deg):
    """Row-vector 3x3 rotation of `deg` about `axis` (right-handed)."""
    a = np.asarray(axis, dtype=float)
    n = float(np.linalg.norm(a))
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = a
    K = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    R = np.eye(3) * c + K * s + np.outer(a, a) * (1.0 - c)
    # R is the COLUMN-vector rotation (v' = R @ v); USD multiplies row vectors,
    # so the matrix that goes into a Gf.Matrix4d is its transpose.
    return R.T


def _t4(v):
    M = np.eye(4)
    M[3, :3] = np.asarray(v, dtype=float)
    return M


def rigid_matrix(spec):
    """The 4x4 row-vector world matrix of one displacement spec."""
    M = _t4(spec.get("translate") or (0.0, 0.0, 0.0))
    deg = float(spec.get("deg") or 0.0)
    if abs(deg) > 1e-9:
        P = np.asarray(spec.get("pivot") or (0.0, 0.0, 0.0), dtype=float)
        R = np.eye(4)
        R[:3, :3] = _rot3(spec.get("axis") or (0.0, 0.0, 1.0), deg)
        M = M @ (_t4(-P) @ R @ _t4(P))
    tw = float(spec.get("twist_deg") or 0.0)
    if abs(tw) > 1e-9:
        # THE MID-STOREY SIGNATURE: the block above a crushed band sits on it
        # rotated a few degrees IN PLAN, so the façade lines break at the
        # seam. Composed after the lean, exactly as `quake_flow.r_soft_storey`
        # composes `M * _rot_about(z) * _translate(offset)`.
        Pz = np.asarray(spec.get("twist_pivot")
                        or spec.get("pivot") or (0.0, 0.0, 0.0), dtype=float)
        Rz = np.eye(4)
        Rz[:3, :3] = _rot3((0.0, 0.0, 1.0), tw)
        M = M @ (_t4(-Pz) @ Rz @ _t4(Pz))
    after = spec.get("after")
    if after and any(abs(float(q)) > 1e-12 for q in after):
        M = M @ _t4(after)
    return M


def apply_rigid(spec, pt):
    """Transform one world point through a displacement spec."""
    p = np.array([float(pt[0]), float(pt[1]), float(pt[2]), 1.0])
    return tuple(float(q) for q in (p @ rigid_matrix(spec))[:3])


def _disp(pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), deg=0.0,
          translate=(0.0, 0.0, 0.0), after=(0.0, 0.0, 0.0), twist_deg=0.0,
          twist_pivot=None, why=""):
    return {"pivot": [float(q) for q in pivot],
            "axis": [float(q) for q in axis], "deg": float(deg),
            "translate": [float(q) for q in translate],
            "after": [float(q) for q in after],
            "twist_deg": float(twist_deg),
            "twist_pivot": (None if twist_pivot is None
                            else [float(q) for q in twist_pivot]),
            "why": str(why)}


# ---------------------------------------------------------------------------
# THE PIECE GRID — reading the slicer's own grammar back off the placements
# ---------------------------------------------------------------------------
# `gac_storey_slice.ring()` writes `_bay = k * len(BAY_SPLITS) + j`: k is the
# bay, j the sub-panel (0 and 2 the narrow piers, 1 the wide opening). With
# `splits=None` — the budget's second lever, spent on wide buildings — there is
# one panel per bay and `_bay = k`. The arity is therefore NOT a constant, and
# it cannot be read off `_role` either: `roof_and_parapet` relabels a top-band
# `pier` AND a top-band `wall` to `parapet`, so on the parapet band the role
# no longer distinguishes them. The sub-index does, always.


def n_sub_of(els):
    """3 when the slicer kept `BAY_SPLITS`, 1 when the budget collapsed it."""
    for e in els:
        if (e.get("p") or {}).get("_role") == "pier":
            return len(BAY_SPLITS)
    return 1


def bay_no(p, n_sub):
    return int(p.get("_bay", 0)) // max(1, int(n_sub))


def sub_ix(p, n_sub):
    return int(p.get("_bay", 0)) % max(1, int(n_sub))


def is_opening(p, n_sub):
    """The wide middle sub-panel — the one with the window in it."""
    return n_sub <= 1 or sub_ix(p, n_sub) == n_sub // 2


def is_pier(p, n_sub):
    return not is_opening(p, n_sub)


def _out_dist(m, side, lx, ly):
    """Signed distance OUTSIDE the mass's wall line on `side` (local frame).

    Positive is out in the street, negative is inside the building — the same
    sign convention `fire_collapse.outward_of` uses, and the one every heap
    and macroblock assertion rests on.
    """
    if side == "S":
        return -(m["D"] / 2.0) - ly
    if side == "N":
        return ly - m["D"] / 2.0
    if side == "W":
        return -(m["W"] / 2.0) - lx
    return lx - m["W"] / 2.0


def _along_frac(m, side, lx, ly):
    """Where a point sits along a side, 0..1 from the side's LOW bay end."""
    if side in ("S", "N"):
        return min(1.0, max(0.0, (lx + m["W"] / 2.0) / max(1e-6, m["W"])))
    return min(1.0, max(0.0, (ly + m["D"] / 2.0) / max(1e-6, m["D"])))


class _Grid(object):
    """Every index the planner needs over one building's element table."""

    def __init__(self, info, els):
        self.info = info
        self.els = [e for e in els if not e.get("dead")]
        self.n_sub = n_sub_of(self.els)
        self.runs = {}        # (side, storey, bay) -> [element]
        self.corners = {}     # (corner, storey)    -> [element]
        self.other = []       # roof / core, no side
        self.storeys = set()
        self.sides = {}       # side -> set of bays present
        for e in self.els:
            p = e.get("p") or {}
            sd, role = p.get("_side"), p.get("_role")
            st = int(p.get("_storey", e.get("storey", 0)))
            self.storeys.add(st)
            if sd in SIDES:
                b = bay_no(p, self.n_sub)
                self.runs.setdefault((sd, st, b), []).append(e)
                self.sides.setdefault(sd, set()).add(b)
            elif sd in _CORNER_SIDES:
                self.corners.setdefault((sd, st), []).append(e)
            else:
                self.other.append(e)
        self.top = max(self.storeys) if self.storeys else 0
        self.n_bays = {sd: (max(bs) + 1 if bs else 0)
                       for sd, bs in self.sides.items()}

    # -- lookups ---------------------------------------------------------
    def cells(self, side=None, storey=None):
        for key in sorted(self.runs):
            if side is not None and key[0] != side:
                continue
            if storey is not None and key[1] != storey:
                continue
            yield key

    def at(self, key):
        return self.runs.get(key, [])

    def role_pieces(self, roles, storey=None):
        out = []
        for e in self.els:
            p = e.get("p") or {}
            if p.get("_role") not in roles:
                continue
            if storey is not None and int(p.get("_storey", 0)) != storey:
                continue
            out.append(e)
        return out

    def mass_of(self, e):
        return self.info["masses"].get(e.get("mass") or "main") \
            or self.info["masses"]["main"]

    def height_of_cells(self, keys, roles=None):
        """Summed piece height over one bay column — what a windrow's reach
        is drawn from (agent A: reach ~ 1.0 x the height of what fell)."""
        by_st = {}
        for sd, st, b in keys:
            for e in self.at((sd, st, b)):
                if roles is not None and (e.get("p") or {}).get("_role") not in roles:
                    continue
                by_st[st] = max(by_st.get(st, 0.0),
                                float((e.get("p") or {}).get("_size",
                                                             (0, 0, e["h"]))[2]))
        return float(sum(by_st.values()))


def _path(e):
    return (e.get("p") or {}).get("prim_path")


def _size(e):
    s = (e.get("p") or {}).get("_size") or (1.0, 1.0, float(e.get("h") or 3.0))
    return (float(s[0]), float(s[1]), float(s[2]))


# ---------------------------------------------------------------------------
# THE PLANNER'S OWN CONTEXT (pure; no stage anywhere below this line until
# `apply_plan`)
# ---------------------------------------------------------------------------
def _pctx(info, elements, btype, rng, plan):
    g = _Grid(info, elements)
    return {"info": info, "g": g, "btype": btype, "rng": rng, "plan": plan,
            "H": float(info.get("H") or 0.0), "n_sub": g.n_sub}


def _note(pctx, text):
    pctx["plan"]["notes"].append(text)


def _pick_sides(rng, n, prefer_front=True, avoid=()):
    """Sides to damage, front (`S`) first — `quake_flow._pick_sides`'s rule."""
    sides = [s for s in ("S", "E", "W", "N") if s not in avoid]
    if not sides:
        return []
    if prefer_front and sides[0] == "S":
        rest = sides[1:]
        rng.shuffle(rest)
        sides = ["S"] + rest
    else:
        rng.shuffle(sides)
    return sides[: max(0, min(len(sides), int(n)))]


def _remove(pctx, els, why=""):
    plan = pctx["plan"]
    n = 0
    for e in els:
        p = _path(e)
        if not p or p in plan["_removed_set"]:
            continue
        plan["_removed_set"].add(p)
        plan["removed"].append(p)
        n += 1
    return n


# ---------------------------------------------------------------------------
# TOOTHING — the one thing that makes removal read as damage
# ---------------------------------------------------------------------------
def _boundary(region, present):
    """Cells of `region` with a neighbour that EXISTS but is not lost.

    "Exists" matters: the top row of a corner failure has no cell above it, and
    a wall that simply stops at the top of the building is not a torn edge.
    """
    out = []
    for (sd, st, b) in sorted(region):
        for nb in ((sd, st, b - 1), (sd, st, b + 1),
                   (sd, st - 1, b), (sd, st + 1, b)):
            if nb in present and nb not in region:
                out.append((sd, st, b))
                break
    return out


def _adjacent_bays(region, present):
    """The intact cells immediately beside the lost region, same storey."""
    out = []
    for (sd, st, b) in sorted(region):
        for nb in ((sd, st, b - 1), (sd, st, b + 1)):
            if nb in present and nb not in region and nb not in out:
                out.append(nb)
    return out


def _apply_region(pctx, region, corners=(), tooth=True, tag="", roles=None,
                  keep_pier=None, stray=None):
    """Lose `region` (a set of (side, storey, bay)) and `corners`, TOOTHED.

    NEVER A CLEAN RECTANGLE. Two draws, both on the boundary of the lost area:

      * a random 20-40 % of the PIERS on the boundary row survive — a pier is
        the narrow panel between two openings and it is what actually stands
        proud on a broken masonry edge;
      * a random 15-30 % of the OPENINGS in the bays immediately beside the
        region go anyway, so the outline wanders into intact wall.

    At least one of each is forced when there is anything to force, so the
    signature is present on a two-bay failure as well as a twenty-bay one and
    the test for it is not a coin flip.

    `roles` restricts which pieces of a cell take part. A cell is NOT always
    one piece: on a top band that `roof_and_parapet` split, the same
    (side, storey, bay) holds both the last storey's wall panel and the
    parapet upstand above it, and a parapet fall must take only the second.

    Returns a dict of what happened, for the plan and for the tests.
    """
    rng = pctx["rng"]
    g = pctx["g"]

    def _in(e):
        return roles is None or (e.get("p") or {}).get("_role") in roles

    present = set(k for k in g.runs if any(_in(e) for e in g.at(k)))
    region = set(region)
    bnd = set(_boundary(region, present)) if tooth else set()
    kp = rng.uniform(*(keep_pier or KEEP_PIER_FRAC)) if tooth else 0.0
    sp = rng.uniform(*(stray or STRAY_OPEN_FRAC)) if tooth else 0.0

    kept_piers, lost, cand_piers = [], [], []
    for key in sorted(region):
        for e in g.at(key):
            p = e.get("p") or {}
            if not _in(e):
                continue
            if key in bnd and is_pier(p, g.n_sub):
                cand_piers.append((key, e))
                continue
            lost.append(e)
    for key, e in cand_piers:
        if rng.random() < kp:
            kept_piers.append(e)
        else:
            lost.append(e)
    if cand_piers and not kept_piers:
        # FORCED. A boundary row with every pier gone is a ruler line.
        e = cand_piers[rng.randrange(len(cand_piers))][1]
        kept_piers.append(e)
        lost = [q for q in lost if q is not e]

    strays, cand_open = [], []
    for key in _adjacent_bays(region, present):
        for e in g.at(key):
            if _in(e) and is_opening(e.get("p") or {}, g.n_sub):
                cand_open.append(e)
    for e in cand_open:
        if rng.random() < sp:
            strays.append(e)
    if cand_open and not strays:
        strays.append(cand_open[rng.randrange(len(cand_open))])

    # CORNERS GO ONLY WITH THEIR TWO ADJOINING BAYS. A corner piece removed
    # on its own leaves the two runs it joined hanging in the air off nothing.
    corner_els = []
    for (cn, st) in sorted(set(corners)):
        sa, sb = _CORNER_SIDES[cn]
        ok = True
        for sd in (sa, sb):
            end = _CORNER_END[(sd, cn)]
            nb = g.n_bays.get(sd, 0)
            b = 0 if end == "lo" else max(0, nb - 1)
            if (sd, st, b) in present and (sd, st, b) not in region:
                ok = False
        if ok:
            corner_els += [e for e in g.corners.get((cn, st), []) if _in(e)]
    _remove(pctx, lost + strays + corner_els, why=tag)
    return {"region": sorted(region), "boundary": sorted(bnd),
            "kept_piers": [_path(e) for e in kept_piers if _path(e)],
            "strays": [_path(e) for e in strays if _path(e)],
            "lost": [_path(e) for e in lost if _path(e)],
            "corners": [_path(e) for e in corner_els if _path(e)],
            "keep_pier_frac": float(kp), "stray_frac": float(sp)}


# ---------------------------------------------------------------------------
# PILE SPECS — what `disaster.quake_rubble.plan_pile` will be handed
# ---------------------------------------------------------------------------
# Only the keys in `_PILE_KW` are forwarded; everything else in a pile spec is
# for the plan's own record (and for the tests). `crown_m` and `spread_frac`
# are deliberately left None so the run-out is drawn where agent A's numbers
# live, per side and asymmetrically, off the `sides` this plan names.
_PILE_KW = ("kind", "crown_m", "spread_frac", "sides", "along", "depth_m",
            "offset_m", "stub_h_m", "elem_h_m", "plate_ok")


def _pile(pctx, kind, mass="main", sides=None, along=None, depth_m=None,
          offset_m=0.0, stub_h_m=0.0, elem_h_m=None, base_z=None,
          crown_m=None, spread_frac=None, tag=""):
    btype = pctx["btype"]
    H = max(1.0, pctx["H"])
    spec = {"kind": kind, "mass": mass, "tag": tag,
            "sides": list(sides) if sides else None,
            "along": [float(along[0]), float(along[1])] if along else None,
            "depth_m": None if depth_m is None else float(depth_m),
            "offset_m": float(offset_m),
            "stub_h_m": float(stub_h_m),
            "elem_h_m": None if elem_h_m is None else float(elem_h_m),
            "base_z": None if base_z is None else float(base_z),
            "crown_m": None if crown_m is None else float(crown_m),
            "spread_frac": None if spread_frac is None else float(spread_frac),
            # ESTIMATES, not passed on: what this plan expects the planner to
            # draw, so the plan is self-describing and testable.
            "crown_est_m": float(min(CROWN_MAX_M, CROWN_FRAC.get(btype, 0.28) * H)),
            "spread_est": float(SPREAD_EST.get(btype, 0.30)),
            "panels": [], "fit": []}
    pctx["plan"]["piles"].append(spec)
    return spec


def _unremove(pctx, path):
    """Take a piece back out of the lost set — it is going to be MOVED, not
    deleted (a macroblock, or a panel laid on the pile)."""
    plan = pctx["plan"]
    if path in plan["_removed_set"]:
        plan["_removed_set"].discard(path)
        plan["removed"] = [q for q in plan["removed"] if q != path]


def _expose(pctx, storeys, sides):
    """Record which storeys/sides an opened wall now shows the inside of.

    NOT authored here. `quake_flow._disturb_interior` — the function that
    tips furniture and litters a floor — fractures a kit piece, so it cannot
    be called on a sliced building until work package H lands. This is the
    hand-off list for that pass.
    """
    it = pctx["plan"]["interior"]
    it["storeys"] = sorted(set(it["storeys"]) | set(int(s) for s in storeys))
    it["sides"] = sorted(set(it["sides"]) | set(sides))


def _bay_run(rng, n_bays, want, end=None):
    """A contiguous run of `want` bays out of `n_bays`, optionally anchored."""
    want = max(1, min(int(n_bays), int(want)))
    if end == "lo":
        b0 = 0
    elif end == "hi":
        b0 = n_bays - want
    else:
        b0 = rng.randrange(0, max(1, n_bays - want + 1))
    return list(range(b0, b0 + want))


# ---------------------------------------------------------------------------
# THE RECIPES. Each is PURE: it reads the element table and writes the plan.
# ---------------------------------------------------------------------------
def s_glass_loss(pctx, frac=(0.05, 0.15), sides=1, storeys=None, band=None):
    """Glass out of a CONTIGUOUS band of storeys on one or two elevations.

    The slicer cannot cut a window out of a piece, so the pane's material
    subset is rebound to a dark void instead (`apply_plan._void_glass`, the
    same read `gac_fire.darken_glass` does for a fire). That is the whole of
    what DG1-DG2 looks like from the air, and it is the glass slot of every
    higher grade's ladder too.
    """
    rng, g = pctx["rng"], pctx["g"]
    lo, hi = (frac, frac) if isinstance(frac, (int, float)) else frac
    f = rng.uniform(float(lo), float(hi))
    sts = sorted(g.storeys)
    if not sts:
        return
    if storeys is not None:
        band_st = [s for s in sts if s in set(storeys)]
    else:
        n_band = band or max(1, int(round(len(sts) * (0.30 + 0.45 * f))))
        n_band = min(len(sts), n_band)
        i0 = rng.randrange(0, max(1, len(sts) - n_band + 1))
        band_st = sts[i0:i0 + n_band]
    picked, on = [], _pick_sides(rng, sides)
    for sd in on:
        cand = [e for st in band_st for b in sorted(g.sides.get(sd, ()))
                for e in g.at((sd, st, b))
                if is_opening(e.get("p") or {}, g.n_sub)]
        hit = [_path(e) for e in cand if _path(e) and rng.random() < f]
        if cand and not hit and f > 0.0:
            # DG1 IS ONE OR TWO PANES, NOT ZERO. At 1-5 % of a tower's band the
            # draw comes up empty about half the time, and a DG1 tower with
            # nothing at all on it is a DG0 tower.
            hit = [_path(cand[rng.randrange(len(cand))])]
        picked += [q for q in hit if q]
    plan = pctx["plan"]
    known = set(plan["glass"])
    plan["glass"] += [q for q in picked if q not in known]
    plan["glass_bands"].append({"sides": list(on),
                                "storeys": [int(s) for s in band_st],
                                "frac": float(f), "n": len(picked)})
    _note(pctx, "glass_loss: {0} pane(s) voided, {1:.0f} % of storeys {2}-{3} "
                "on {4}".format(len(picked), 100.0 * f, band_st[0], band_st[-1],
                                "/".join(on) or "-"))


def s_parapet_fall(pctx, sides=1, frac=0.5):
    """Parapet / cornice off `frac` of a side, and the windrow it makes.

    The parapet is the first thing to go in every reconnaissance record
    (EMS-98 DG2) and, at 0.45 m deep and one parapet-height out, the windrow
    is small — the reach comes from what FELL, never from the building
    (agent A §5a: a coping off a 120 m tower makes the same windrow as one
    off a shop)."""
    rng, g = pctx["rng"], pctx["g"]
    par = g.role_pieces(("parapet", "parapet_corner"))
    st_par = g.top
    if par:
        st_par = max(int((e.get("p") or {}).get("_storey", g.top)) for e in par)
    for sd in _pick_sides(rng, sides):
        nb = g.n_bays.get(sd, 0)
        if nb <= 0:
            continue
        run = _bay_run(rng, nb, max(1, int(round(nb * float(frac)))))
        region = {(sd, st_par, b) for b in run
                  if any((e.get("p") or {}).get("_role") in ("parapet",) or not par
                         for e in g.at((sd, st_par, b)))}
        if not region:
            continue
        res = _apply_region(pctx, region,
                            corners=[(c, st_par) for c in _CORNER_SIDES],
                            tag="parapet_fall",
                            roles=("parapet", "parapet_corner") if par else None)
        eh = g.height_of_cells(
            region, roles=("parapet",) if par else None) or 1.0
        _pile(pctx, "windrow", sides=[sd],
              along=(run[0] / float(nb), (run[-1] + 1) / float(nb)),
              depth_m=WINDROW_DEPTH_M["parapet"], elem_h_m=eh,
              base_z=None, tag="parapet_{0}".format(sd))
        _note(pctx, "parapet_fall: {0} piece(s) off {1} over bays {2}-{3} of "
                    "{4}, windrow {5:.2f} m deep, reach ~{6:.1f} m".format(
                        len(res["lost"]) + len(res["corners"]), sd, run[0],
                        run[-1], nb, WINDROW_DEPTH_M["parapet"], eh))


def s_infill_fail(pctx, storeys=1, frac=0.4, sides=None):
    """A frame's masonry INFILL panels drop out — a scatter of openings, not
    a region. This is the RC signature at DG2-DG3 (Northridge, Antakya): the
    frame is legible and the panels between it are gone."""
    rng, g = pctx["rng"], pctx["g"]
    sts = sorted(g.storeys)
    if not sts:
        return
    n = max(1, min(len(sts), int(storeys)))
    lowest = sts[: max(1, int(len(sts) * 0.6)) or 1]
    st_pick = sorted(rng.sample(lowest, min(n, len(lowest))))
    got = []
    for sd in (sides or _pick_sides(rng, 1 + (1 if frac > 0.45 else 0))):
        for st in st_pick:
            for b in sorted(g.sides.get(sd, ())):
                for e in g.at((sd, st, b)):
                    if is_opening(e.get("p") or {}, g.n_sub) and rng.random() < frac:
                        got.append(e)
        if got:
            eh = max([_size(e)[2] for e in got] or [3.0])
            _pile(pctx, "windrow", sides=[sd], along=(0.0, 1.0),
                  depth_m=0.35, elem_h_m=eh, tag="infill_{0}".format(sd))
    _remove(pctx, got, why="infill_fail")
    _expose(pctx, st_pick, sorted(set((e.get("p") or {}).get("_side")
                                      for e in got) - {None}))
    _note(pctx, "infill_fail: {0} infill panel(s) out on storey(s) {1}".format(
        len(got), ",".join(str(s) for s in st_pick)))


def s_corner_fail(pctx, storeys=2, corner=None, mass="main"):
    """The corner of the building goes, over the top `storeys` storeys.

    THE STAIRCASE. A corner failure is never a rectangular bite: it widens
    upward, because the corner is where two out-of-plane walls meet and each
    one peels a little further at every storey. So the LOWEST lost storey
    loses one bay on each side and every storey above it loses two, and the
    whole thing is toothed on top of that. Never the ground storey — a corner
    that failed at grade is a collapse, not a corner failure.
    """
    rng, g = pctx["rng"], pctx["g"]
    cn = corner or sorted(_CORNER_SIDES)[rng.randrange(4)]
    sa, sb = _CORNER_SIDES[cn]
    k = max(1, int(storeys))
    st_lo = max(1, g.top - k + 1)
    lost_st = [st for st in range(st_lo, g.top + 1) if st in g.storeys]
    if not lost_st:
        return
    region, shape = set(), []
    for i, st in enumerate(lost_st):
        want = 1 if i == 0 else 2
        row = []
        for sd in (sa, sb):
            nb = g.n_bays.get(sd, 0)
            if nb <= 0:
                continue
            for b in _bay_run(rng, nb, want, end=_CORNER_END[(sd, cn)]):
                if (sd, st, b) in g.runs:
                    region.add((sd, st, b))
                    row.append((sd, b))
        shape.append({"storey": int(st), "bays": int(want), "cells": len(row)})
    if not region:
        return
    res = _apply_region(pctx, region, corners=[(cn, st) for st in lost_st],
                        tag="corner_fail")
    pctx["plan"]["regions"].append({"recipe": "corner_fail", "corner": cn,
                                   "sides": [sa, sb], "storeys": lost_st,
                                   "shape": shape,
                                   "cells": sorted(list(c) for c in region)})
    _expose(pctx, lost_st, [sa, sb])
    eh = g.height_of_cells(region) or 3.0
    _pile(pctx, "fan", sides=[sa, sb], depth_m=WINDROW_DEPTH_M["wall"],
          elem_h_m=eh, tag="corner_{0}".format(cn))
    _note(pctx, "corner_fail: {0} corner, storeys {1}-{2} (1 bay at the "
                "bottom, 2 above), {3} piece(s) down, {4} boundary pier(s) "
                "left standing, fan pile on {5}+{6}".format(
                    cn, lost_st[0], lost_st[-1],
                    len(res["lost"]) + len(res["corners"]),
                    len(res["kept_piers"]), sa, sb))


# A macroblock rotates about its own bottom outer edge until it is on the
# street: 65-88 degrees. Below 65 it reads as a lean; at 90 it is flat and the
# façade texture is invisible from the air, so the band stops short of it.
MACRO_DEG = (65.0, 88.0)
MACRO_N = (1, 2)


def s_out_of_plane(pctx, sides=1, from_storey=1, side=None, bays=None,
                   storeys=None, mass="main"):
    """A run of wall peels off one elevation and lands in the street.

    The URM DG4 signature and the one directional failure in an earthquake:
    the wall rotates about its foot, so the top leads and it lands as a FAN
    rather than a windrow. One or two of the pieces stay WHOLE — masonry
    fails in macroblocks, and a 4 x 3 m panel of façade lying face-up in the
    road is the single most legible thing in an aerial frame — while the rest
    of the run is in the pile.
    """
    rng, g = pctx["rng"], pctx["g"]
    sd = side or (_pick_sides(rng, 1)[0])
    nb = g.n_bays.get(sd, 0)
    if nb <= 0:
        return
    sts = sorted(s for s in g.storeys if s >= max(1, int(from_storey)))
    if not sts:
        return
    n_st = storeys if storeys is not None else rng.randint(1, 3)
    n_st = max(1, min(len(sts), int(n_st)))
    lost_st = sts[-n_st:]                       # the UPPER storeys
    want = bays if bays is not None else rng.randint(3, 6)
    # A RUN, NEVER THE WHOLE ELEVATION. An out-of-plane failure that takes
    # every bay of a side is not `out_of_plane`, it is a collapse — and with
    # no intact bay left beside it the toothing has nothing to wander into,
    # so the edge comes out as a ruler line at the corner.
    if nb >= 2:
        want = max(1, min(int(want), nb - 1))
    run = _bay_run(rng, nb, want)
    region = {(sd, st, b) for st in lost_st for b in run if (sd, st, b) in g.runs}
    if not region:
        return
    # PICK THE MACROBLOCKS BEFORE THE REGION IS LOST, from the upper storeys —
    # a block that came off the top is the one that clears the sidewalk.
    cand = [e for st in lost_st[-2:] for b in run
            for e in g.at((sd, st, b))
            if is_opening(e.get("p") or {}, g.n_sub) and _path(e)]
    rng.shuffle(cand)
    n_mb = rng.randint(*MACRO_N)
    macros = cand[:n_mb]
    panel = cand[n_mb:n_mb + 1] if (len(cand) > n_mb and rng.random() < 0.6) else []

    res = _apply_region(pctx, region, corners=[], tag="out_of_plane")
    m = g.mass_of(macros[0]) if macros else pctx["info"]["masses"][mass]
    plan = pctx["plan"]
    ox, oy = _outward(m, sd)
    ax, ay = -oy, ox                 # the horizontal axis ALONG the wall
    for e in macros:
        _unremove(pctx, _path(e))
        sx, sy, sz = _size(e)
        depth = sy if sd in ("S", "N") else sx
        # The pivot is the piece's OWN bottom outer edge: local centre pushed
        # out by half its own depth, at its own base z.
        lnx, lny = _SIDE_NORMAL[sd]
        plx = e["lx"] + lnx * depth / 2.0
        ply = e["ly"] + lny * depth / 2.0
        px, py = _to_world(m, plx, ply)
        deg = rng.uniform(*MACRO_DEG)
        plan["displaced"][_path(e)] = _disp(
            pivot=(px, py, e["z"]), axis=(ax, ay, 0.0), deg=deg,
            why="macroblock off {0} at storey {1}".format(sd, e["storey"]))
        plan["macroblocks"].append({"path": _path(e), "side": sd,
                                    "storey": int(e["storey"]),
                                    "deg": float(deg),
                                    "pivot": [float(px), float(py), float(e["z"])],
                                    "size": [sx, sy, sz]})
    for e in panel:
        _unremove(pctx, _path(e))
        plan["panels"].append([_path(e), list(_size(e))])

    eh = g.height_of_cells(region) or 3.0
    pile = _pile(pctx, "fan", sides=[sd],
                 along=(run[0] / float(nb), (run[-1] + 1) / float(nb)),
                 depth_m=WINDROW_DEPTH_M["wall"], elem_h_m=eh,
                 tag="oop_{0}".format(sd))
    for e in panel:
        pile["panels"].append([_path(e), list(_size(e))])
    plan["regions"].append({"recipe": "out_of_plane", "side": sd,
                            "storeys": [int(s) for s in lost_st],
                            "bays": [int(b) for b in run],
                            "cells": sorted(list(c) for c in region)})
    _expose(pctx, lost_st, [sd])
    _note(pctx, "out_of_plane: {0} bay(s) x {1} storey(s) off {2}, {3} piece(s) "
                "down, {4} macroblock(s) at {5}, {6} boundary pier(s) left, "
                "fan reach ~{7:.1f} m".format(
                    len(run), len(lost_st), sd, len(res["lost"]), len(macros),
                    "/".join("{0:.0f}".format(q["deg"]) for q in plan["macroblocks"][-len(macros):]) or "-",
                    len(res["kept_piers"]), eh))


# ---------------------------------------------------------------------------
# SOFT / MID STOREY — the MECHANISM, not a lean bolted onto a drop
# ---------------------------------------------------------------------------
# The first version of this recipe put the pivot on the base edge of the side
# it leaned toward and rotated `+lean` there, which LIFTS the far side of the
# block (a 30 m frame at 5.5 deg raises its far corner 2.9 m and cancels the
# whole drop). `quake_flow.r_soft_storey` hides the same geometry behind a
# `sign = -1.0` that quietly leans the block away from the side it names.
# Moving the pivot to the far edge fixes the lift but then the lean has to be
# capped at `asin(crush / span)` or the low corner goes through the pavement —
# and on a 30 m frame that cap is ~1.6 deg, which is not what Kobe or Antakya
# look like.
#
# The cap is the wrong answer because a soft storey is not one mechanism. Two
# things happen in the record, and the geometry follows from which:
#
# 1. DIFFERENTIAL CRUSH (60 %). The columns crush, and they crush MORE on one
#    side, so the block above sits on a WEDGE. The angle is not free: it is
#    (r_far - r_lean) / span. Draw the two residual heights and the angle comes
#    out of them, which is why a 30 m frame can lean 5 deg without its low
#    corner being underground — the far side is 3 m of standing column stub,
#    not the same crush as the low side.
# 2. SIDESWAY (40 %). The columns hinge top and bottom and the storey RACKS:
#    the block above stays PLUMB and slides sideways. This is the Northridge
#    Meadows / Antakya first-storey photograph — a building offset a metre or
#    two over its own ground floor, its façade lines still vertical — and it is
#    invisible to any model that only knows how to tilt.
#
# `sway_deg` is much larger than `lean_deg` because they measure different
# things: 8-25 deg is the column's rack angle over ONE storey (0.7-2.1 m of
# offset on a 5 m storey), while 2.5-6.5 deg is the tilt of the WHOLE block.
P_SWAY = 0.40                    # share of soft storeys that rack rather than tilt
R_LEAN_FRAC = (0.15, 0.40)       # residual height on the LEAN side, x storey h
R_FAR_CEIL_FRAC = 0.95           # the far side cannot be taller than it started
LEAN_DEG = (2.5, 6.5)            # the tilt a differential crush is drawn for
SWAY_DEG = (8.0, 25.0)           # the column rack angle of a sidesway
SWAY_CRUSH_FRAC = (0.15, 0.35)   # squash ON TOP of the rack's own shortening


def s_soft_storey(pctx, storey=0, mass="main", lean_deg=None, crush_frac=None,
                  twist_deg=0.0, offset_m=0.0, lean_side=None, mechanism=None):
    """The columns of `storey` fail and everything above comes down on them.

    Two mechanisms (above). Both are expressed as ONE rigid displacement spec
    shared by every piece above the failed storey, so the arithmetic is
    checkable with no stage:

    * CRUSH — translate the block down until its far base edge is at
      `z_lo + r_far`, then rotate `+lean` about that edge. The lean side's base
      then lands exactly on `z_lo + r_lean` (that is what
      `span sin(lean) = r_far - r_lean` means), the top swings toward the lean
      side, and nothing rises.
    * SWAY — translate only: `h_st sin(phi)` toward the lean side and
      `h_st (1 - cos phi) + crush` down. No rotation at all.

    Explicit `lean_deg` or `crush_frac` forces CRUSH (the tower ladder's
    podium asks for a named angle).
    """
    rng, g = pctx["rng"], pctx["g"]
    info = pctx["info"]
    m = info["masses"].get(mass) or info["masses"]["main"]
    lv = m["levels"]
    k = int(storey)
    if k < 0 or k >= len(lv):
        return
    z_lo = lv[k]
    z_hi = lv[k + 1] if k + 1 < len(lv) else m["top"]
    h_st = max(0.5, z_hi - z_lo)
    sd = lean_side or ("S", "E", "N", "W")[rng.randrange(4)]
    ox, oy = _outward(m, sd)
    ax, ay = -oy, ox                       # the horizontal axis along that wall
    lnx, lny = _SIDE_NORMAL[sd]
    # SPAN is the mass extent PERPENDICULAR to the failing side's wall — the
    # lever the wedge is measured over.
    span = max(1.0, m["D"] if sd in ("S", "N") else m["W"])
    # ...and the pivot edge is the base edge on the FAR side.
    fx, fy = _to_world(m, -lnx * m["W"] / 2.0, -lny * m["D"] / 2.0)
    forced = lean_deg is not None or crush_frac is not None
    if mechanism is None:
        mechanism = "crush" if (forced or rng.random() >= P_SWAY) else "sway"

    rec = {"storey": k, "h_st": float(h_st), "lean_side": sd,
           "span_m": float(span), "mechanism": mechanism,
           "twist_deg": float(twist_deg)}
    if mechanism == "crush":
        lean_drawn = float(lean_deg) if lean_deg is not None else rng.uniform(*LEAN_DEG)
        r_lean = (float(crush_frac) if crush_frac is not None
                  else rng.uniform(*R_LEAN_FRAC)) * h_st
        r_far = min(R_FAR_CEIL_FRAC * h_st,
                    r_lean + span * math.sin(math.radians(lean_drawn)))
        lean = math.degrees(math.asin(
            min(1.0, max(0.0, (r_far - r_lean) / span))))
        spec = _disp(pivot=(fx, fy, z_lo + r_far), axis=(ax, ay, 0.0), deg=lean,
                     translate=(0.0, 0.0, -(h_st - r_far)),
                     twist_deg=float(twist_deg),
                     twist_pivot=(m["cx"], m["cy"], z_lo + r_far),
                     after=(ox * float(offset_m), oy * float(offset_m), 0.0),
                     why="soft storey {0} (differential crush)".format(k))
        crush = 0.5 * (r_lean + r_far)     # the collar's crown
        rec.update({"r_lean_m": float(r_lean), "r_far_m": float(r_far),
                    "lean_deg": float(lean), "lean_drawn_deg": float(lean_drawn),
                    "capped": bool(r_far >= R_FAR_CEIL_FRAC * h_st - 1e-9),
                    "pivot_side": qf._opposite(sd),
                    "pivot": [float(fx), float(fy), float(z_lo + r_far)],
                    "drop_far_m": float(h_st - r_far),
                    "drop_lean_m": float(h_st - r_lean),
                    "sway_deg": 0.0, "offset_m": float(offset_m)})
        head = ("storey {0} crushed to {1:.2f} m on {2} and {3:.2f} m on {4} — "
                "the block above tilts {5:.1f} deg over a {6:.0f} m span"
                .format(k, r_lean, sd, r_far, qf._opposite(sd), lean, span))
    else:
        phi = rng.uniform(*SWAY_DEG)
        d = h_st * math.sin(math.radians(phi)) + float(offset_m)
        squash = rng.uniform(*SWAY_CRUSH_FRAC) * h_st
        drop = h_st * (1.0 - math.cos(math.radians(phi))) + squash
        spec = _disp(deg=0.0, translate=(ox * d, oy * d, -drop),
                     twist_deg=float(twist_deg),
                     twist_pivot=(m["cx"], m["cy"], z_lo),
                     why="soft storey {0} (sidesway)".format(k))
        crush = max(0.05 * h_st, h_st - drop)
        rec.update({"sway_deg": float(phi), "offset_m": float(d),
                    "squash_m": float(squash), "drop_m": float(drop),
                    "lean_deg": 0.0, "r_lean_m": float(crush),
                    "r_far_m": float(crush), "pivot_side": None,
                    "column_incline_deg": float(phi)})
        head = ("storey {0} racked {1:.0f} deg — the block above stays PLUMB, "
                "offset {2:.2f} m toward {3} and down {4:.2f} m"
                .format(k, phi, d, sd, drop))
    rec["crush_m"] = float(crush)

    plan = pctx["plan"]
    n_above = 0
    for e in g.els:
        st = int((e.get("p") or {}).get("_storey", e.get("storey", 0)))
        pth = _path(e)
        if st > k and pth and pth not in plan["_removed_set"]:
            plan["displaced"][pth] = dict(spec)
            n_above += 1
    rec["n_above"] = n_above
    # the failed storey itself: a full-band region, toothed, so what is left
    # of it are stumps of pier rather than a clean gap.
    # A CRUSHED STOREY LEAVES STUMPS, NOT A WALL. The default toothing keeps
    # 20-40 % of the boundary piers, which is right for an edge a failure
    # stopped at; here the whole band IS the boundary and 5-20 % is what
    # survives being sat on by the building above.
    region = {key for key in g.cells(storey=k)}
    res = _apply_region(pctx, region,
                        corners=[(c, k) for c in _CORNER_SIDES],
                        tag="soft_storey", keep_pier=(0.05, 0.20)
                        ) if region else {"lost": [], "kept_piers": []}
    for e in g.role_pieces(("core",), storey=k):
        _remove(pctx, [e], why="soft_storey core")

    plan["fit_ops"].append({"op": "displace_above", "mass": mass, "storey": k,
                            "transform": dict(spec)})
    plan["fit_ops"].append({"op": "columns_to_pile", "mass": mass, "storey": k,
                            "pile": len(plan["piles"]),
                            # A RACKED STOREY'S COLUMNS ARE STILL THERE, LEANING.
                            # In a sidesway the columns hinge rather than
                            # shatter, so they are authored inclined by the rack
                            # angle before they join the pile's large elements;
                            # a differential crush shatters them and they go in
                            # upright.
                            "incline_deg": float(rec.get("column_incline_deg") or 0.0),
                            "incline_side": sd})
    plan["fit_ops"].append({"op": "deactivate_partitions", "mass": mass,
                            "storeys": [k]})
    plan["fit_ops"].append({"op": "bury_props", "mass": mass, "storeys": [k],
                            "base_z": float(z_lo), "heap_h": float(crush)})
    # THE COLLAR: the crushed storey squeezed out round the perimeter. An
    # explicit override of the run-out, and the one place it is right to
    # override — this is 1-3 m of material pushed out of a seam, not a mound
    # of a whole building, so agent A's per-side run-out does not apply.
    _pile(pctx, "dome", mass=mass, sides=list(SIDES), base_z=z_lo,
          crown_m=crush, spread_frac=rng.uniform(0.10, 0.16),
          elem_h_m=h_st, stub_h_m=0.0, tag="soft{0}".format(k))
    plan["storey_collapse"] = rec
    _expose(pctx, [k, min(g.top, k + 1)], [sd])
    _note(pctx, "soft_storey: {0}; {1} piece(s) of it gone, {2} pier stump(s) "
                "left, {3} piece(s) above moved{4}".format(
                    head, len(res["lost"]), len(res["kept_piers"]), n_above,
                    ", twisted {0:.1f} deg".format(twist_deg) if twist_deg else ""))


def s_mid_storey(pctx, storey=None, mass="main"):
    """The same failure at an INTERMEDIATE storey, with the block above
    twisted in plan — Kobe / Mexico City. The twist is what tells a
    mid-storey collapse from a building that is merely shorter."""
    rng = pctx["rng"]
    info = pctx["info"]
    m = info["masses"].get(mass) or info["masses"]["main"]
    n = len(m["levels"])
    if n < 3:
        return s_soft_storey(pctx, storey=0, mass=mass)
    k = int(storey) if storey is not None else rng.randrange(1, n - 1)
    # THE TWIST IS THE WHOLE SIGNATURE and it is kept for both mechanisms: the
    # block above sits on the crushed band rotated a few degrees IN PLAN, so
    # the façade lines break at the seam. Without it a mid-storey collapse is
    # a building that is merely shorter. The mechanism itself is drawn, not
    # forced — a mid storey racks as readily as it wedges.
    return s_soft_storey(pctx, storey=k, mass=mass,
                         twist_deg=rng.uniform(2.0, 8.0) * (1 if rng.random() < 0.5 else -1),
                         offset_m=rng.uniform(0.3, 2.0))


def s_storey_collapse(pctx, mass="main"):
    """The RC DG4 slot: a soft ground storey two times in three, a mid-storey
    otherwise — `quake_flow.r_storey_collapse`'s own draw."""
    if pctx["rng"].random() < 0.66:
        return s_soft_storey(pctx, storey=0, mass=mass)
    return s_mid_storey(pctx, mass=mass)


PANELS_N = (2, 4)


def _total_collapse(pctx, kind, mass="main", stub=None, fall_sides=None):
    """DG5: the building is a pile with two or three walls of a stub round it.

    THE SHELL DOES NOT VANISH AND IT DOES NOT ALL STAND. Two things carry the
    read of a total collapse from the air and both are on the piece grid:

      * 2-4 whole WALL PANELS lying on the mound — preferentially the wide
        opening panels, because a panel with a window in it is instantly
        legible as a piece of building and a blank pier is not (agent A §1b:
        "the pile's silhouette is set by 3-8 large elements, not by thousands
        of cells");
      * a ground-floor STUB, in 40-60 % of cases (agent A §5b), 0.6-2.7 m
        high. Nothing here can CUT a stub down to 0.6 m — that is a fracture —
        so it is expressed as: the storey-0 pieces on the fall side go, the
        other two or three sides stay, and `stub_h_m` tells the pile planner
        how much shell the mound has to fill. `residual_m` records the height
        a later fracture pass should trim it to.

    The fall side(s) are named in the dome spec, because the run-out is
    asymmetric — 0.35-0.65 H into the street, 0.05-0.15 H against a blind
    party wall.
    """
    rng, g = pctx["rng"], pctx["g"]
    info = pctx["info"]
    m = info["masses"].get(mass) or info["masses"]["main"]
    plan = pctx["plan"]
    H = max(1.0, pctx["H"])
    fall = list(fall_sides) if fall_sides else _pick_sides(
        rng, 1 if rng.random() < 0.6 else 2)

    # --- the panels, chosen before anything is lost ----------------------
    up = [e for e in g.els
          if int((e.get("p") or {}).get("_storey", 0)) >= 1 and _path(e)
          and (e.get("p") or {}).get("_role") in ("wall", "pier", "parapet")]
    openings = [e for e in up if is_opening(e.get("p") or {}, g.n_sub)]
    pool = openings or up
    rng.shuffle(pool)
    n_panels = rng.randint(*PANELS_N)
    panels = pool[:n_panels]
    keep = set(_path(e) for e in panels)

    # --- everything above the ground storey goes -------------------------
    gone = [e for e in g.els
            if int((e.get("p") or {}).get("_storey", 0)) >= 1
            and _path(e) not in keep]
    _remove(pctx, gone, why=kind)

    # --- the stub --------------------------------------------------------
    keep_stub = (rng.random() < rng.uniform(*STUB_KEEP_P)) if stub is None else bool(stub)
    st0 = [e for e in g.els if int((e.get("p") or {}).get("_storey", 0)) == 0]
    stub_h = max([_size(e)[2] for e in st0] or [0.0])
    residual = rng.uniform(*STUB_RESIDUAL_M) if keep_stub else 0.0
    lost_sides = []
    if keep_stub:
        # the fall side's ground storey is what the mass came down through
        lost_sides = [sd for sd in fall if g.n_bays.get(sd, 0) > 0][:2]
        region = {key for sd in lost_sides for key in g.cells(side=sd, storey=0)}
        if region:
            _apply_region(pctx, region,
                          corners=[(c, 0) for c in _CORNER_SIDES], tag=kind + "_stub")
    else:
        _remove(pctx, st0, why=kind + " (no stub)")
        stub_h = 0.0
    kept = [_path(e) for e in st0
            if _path(e) and _path(e) not in plan["_removed_set"]]
    plan["stub"] = {"keep": bool(keep_stub), "storey": 0,
                    "top_z": float(m["z0"] + stub_h) if keep_stub else float(m["z0"]),
                    "height_m": float(stub_h), "residual_m": float(residual),
                    "sides_lost": lost_sides,
                    "sides_kept": [sd for sd in SIDES if sd not in lost_sides] if keep_stub else [],
                    "kept": sorted(kept),
                    "note": ("cut the stub down to residual_m once a clipped "
                             "shell can be fractured (work package H)")}

    # --- the pile --------------------------------------------------------
    pile = _pile(pctx, "dome", mass=mass, sides=fall, base_z=m["z0"],
                 stub_h_m=float(stub_h), tag=kind)
    for e in panels:
        item = [_path(e), list(_size(e))]
        plan["panels"].append(item)
        pile["panels"].append(item)

    # --- the fit-out: the slabs ARE the pile's mass ----------------------
    plan["fit_ops"] += [
        {"op": "deactivate_slabs", "mass": mass},
        {"op": "deactivate_columns", "mass": mass},
        {"op": "deactivate_partitions", "mass": mass, "storeys": None},
        {"op": "bury_props", "mass": mass, "storeys": None,
         "base_z": float(m["z0"]),
         "heap_h": float(min(CROWN_MAX_M, CROWN_FRAC.get(pctx["btype"], 0.28) * H))},
    ]
    plan["collapse"] = {"kind": kind, "fall_sides": fall,
                        "n_removed": len(gone),
                        "n_panels": len(panels),
                        "stub": bool(keep_stub)}
    _note(pctx, "{0}: {1} piece(s) down, {2} panel(s) laid on the mound, "
                "{3}, dome over the footprint falling toward {4} "
                "(crown ~{5:.1f} m)".format(
                    kind, len(gone), len(panels),
                    ("stub kept on {0} ({1:.1f} m standing, trim to {2:.1f} m)"
                     .format("/".join(plan["stub"]["sides_kept"]), stub_h, residual)
                     if keep_stub else "no stub — cleared to the slab"),
                    "/".join(fall), pile["crown_est_m"]))


def s_pancake(pctx, mass="main", **kw):
    """RC total collapse. The pile planner supplies the slab RAFTS that set a
    concrete pile's silhouette; the fit-out's own slabs are deactivated
    because their mass is in the mound, not stacked as boxes."""
    return _total_collapse(pctx, "pancake", mass=mass, **kw)


def s_masonry_collapse(pctx, mass="main", **kw):
    """URM total collapse — the same grid arithmetic; what differs is the
    pile's LOOK (brick rubble, lintel/quoin monoliths, no concrete rafts),
    which `quake_rubble` picks from `btype`."""
    return _total_collapse(pctx, "masonry_collapse", mass=mass, **kw)


# ---------------------------------------------------------------------------
# THE FOUNDATION FAMILY — routed to `quake_flow`, because it is already rigid
# ---------------------------------------------------------------------------
# `r_settlement`, `r_tilt_severe` and `r_tilt_sink` transform `_everything(ctx)`
# — which is `[e["p"]["prim_path"] for e in _els(ctx)] + ctx["fit"]["all"]`, so
# it picks up SLICED pieces exactly as it picks up kit modules — plus a raft,
# and then author ground art (`_c_ground_response`, `_berm`, `_heap`). None of
# those fracture anything, so they run unchanged on a sliced building. VERIFIED
# by reading them, not assumed.
#
# `r_overturn` is the exception: it `_break`s the landing side's parapet and
# top storey and `_split_strip`s the roof, both of which would hand a clipped
# shell to VTK. `s_overturn` plans the same event with removal instead, and
# `_ov_apply` does the rigid half itself.
def s_settlement(pctx, sink_m=None):
    pctx["plan"]["ground"] = {"recipe": "settlement",
                              "kwargs": {} if sink_m is None else {"sink_m": float(sink_m)}}
    _note(pctx, "settlement: routed to quake_flow.r_settlement (rigid sink + "
                "subsidence ring)")


def s_tilt_severe(pctx, **kw):
    pctx["plan"]["ground"] = {"recipe": "tilt_severe", "kwargs": dict(kw)}
    _note(pctx, "tilt_severe: routed to quake_flow.r_tilt_severe (10-30 deg on "
                "a raft that levers out)")


def s_tilt_sink(pctx, **kw):
    pctx["plan"]["ground"] = {"recipe": "tilt_sink", "kwargs": dict(kw)}
    _note(pctx, "tilt_sink: routed to quake_flow.r_tilt_sink ({0})".format(
        ", ".join("{0}={1}".format(k, v) for k, v in sorted(kw.items())) or "defaults"))


def s_overturn(pctx, angle_deg=None, side=None, mass="main"):
    """Whole-body overturning, 60-90 deg about one base edge, RIGID.

    The shell stays whole (Antakya: columns uprooted unbroken, the façade
    legible on its side). What `quake_flow.r_overturn` fractures — the parapet
    and top storey on the landing edge, which hit first — is REMOVED here and
    its mass put into the landing windrow instead.
    """
    rng, g = pctx["rng"], pctx["g"]
    m = pctx["info"]["masses"].get(mass) or pctx["info"]["masses"]["main"]
    sd = side or ("S", "E", "N", "W")[rng.randrange(4)]
    angle = float(angle_deg) if angle_deg is not None else rng.uniform(62.0, 90.0)
    H = max(1.0, pctx["H"])
    crush = [e for e in g.els
             if (e.get("p") or {}).get("_side") in (sd,) + tuple(
                 c for c in _CORNER_SIDES if sd in _CORNER_SIDES[c])
             and ((e.get("p") or {}).get("_role") in ("parapet", "parapet_corner")
                  or int((e.get("p") or {}).get("_storey", 0)) >= g.top)]
    _remove(pctx, crush, why="overturn landing crush")
    land = H * math.sin(math.radians(angle)) if angle < 89.0 else H
    _pile(pctx, "windrow", mass=mass, sides=[sd], along=(0.0, 1.0),
          depth_m=WINDROW_DEPTH_M["wall"],
          elem_h_m=max([_size(e)[2] for e in crush] or [3.0]),
          offset_m=max(0.0, land - 2.5), tag="landing")
    pctx["plan"]["ground"] = {"recipe": "overturn",
                              "kwargs": {"angle_deg": angle, "side": sd}}
    _note(pctx, "overturn: {0:.0f} deg about the {1} edge (rigid — no fracture "
                "on a sliced shell), {2} landing-edge piece(s) removed, windrow "
                "at {3:.0f} m".format(angle, sd, len(crush), land))


RECIPES_S = {
    "glass_loss": s_glass_loss,
    "parapet_fall": s_parapet_fall,
    "infill_fail": s_infill_fail,
    "corner_fail": s_corner_fail,
    "out_of_plane": s_out_of_plane,
    "soft_storey": s_soft_storey,
    "mid_storey": s_mid_storey,
    "storey_collapse": s_storey_collapse,
    "pancake": s_pancake,
    "masonry_collapse": s_masonry_collapse,
    "settlement": s_settlement,
    "tilt_severe": s_tilt_severe,
    "tilt_sink": s_tilt_sink,
    "overturn": s_overturn,
}


# ---------------------------------------------------------------------------
# THE PERIMETER OF EVERY HOLE (round 6b) — a ragged tear on every surviving
# piece that touches a removed region, plus a ragged rim on the slab a
# removed storey exposes.
#
# `fire_collapse.plan_edges` / `_tear_perimeter` already do exactly this for
# the FIRE ladder on sliced GAC / downtowncity pieces (`fire_collapse.
# is_sliced` / `edge_gap_tol` exist BECAUSE that ran on real slices,
# 2026-08-30). This hooks the same machinery into the QUAKE ladder, from
# THIS module only — `fire_collapse.py`, `quake_flow.py` and `fracture.py`
# are not touched (the fire freeze). `quake_flow._a_slab_rim` /
# `_ragged_slabs` do the slab half; both operate exclusively on
# `ctx["fit"]["slabs"]` (`fit_interior`'s own authored `_box`es), never on a
# clipped shell, so they are safe here in a way nothing that touches a piece
# is.
# ---------------------------------------------------------------------------
QS_RAGGED = _os.environ.get("QS_RAGGED", "1").strip().lower() not in (
    "0", "false", "no", "off")
QS_MAX_TEARS = int(_os.environ.get("QS_MAX_TEARS", "40") or 40)
QS_TEAR_TOL_M = float(_os.environ.get("QS_TEAR_TOL_M", "0.6") or 0.6)
TEAR_PAD_M = 0.6          # fire_collapse.plan_partial_collapse's own `pad` order


def _tear_rng(info, plan):
    """A PRIVATE generator, seeded from the building's own identity.

    Zero shared draws: `plan_damage` runs every recipe off `pctx["rng"]`,
    and a tear pass that consumed from it would move every later outcome of
    every OTHER recipe — and, in a multi-building bake, every later
    building in the same process. Same discipline as `fire_collapse.
    _own_rng` / `fracture.stable_seed`.
    """
    from . import fracture
    return random.Random(fracture.stable_seed(
        info.get("style"), plan.get("grade") or "",
        len(info["elements"]), len(plan["_removed_set"])))


def _plan_tears(pctx, plan):
    """`fire_collapse.plan_edges` jobs for every hole this plan opened.

    DRIVEN OFF `plan["_removed_set"]` (still live at this point in
    `plan_damage`, before `_finalise` pops it) — NOT `plan["regions"]`: only
    `s_corner_fail` and `s_out_of_plane` ever append a region record, and
    `s_parapet_fall` / `s_infill_fail` — the commonest boundaries in the
    measured GAC bakes — register no region at all, so a regions-driven pass
    would leave the commonest hole untorn.

    Returns [] when nothing was removed, when the building came down
    entirely (`plan["collapse"]` — nothing left to tear), or when QS_RAGGED
    is off. In every one of those cases the private rng below is never even
    constructed, so a plan with tears disabled draws nothing extra from
    anywhere and is byte-identical to one built before this pass existed.
    """
    if not QS_RAGGED or plan.get("collapse") or not plan["_removed_set"]:
        return []
    from . import fire_collapse as fc
    from detail import urban_building as ub
    info, g = pctx["info"], pctx["g"]
    # `el_footprint`'s silent fallback: a piece with no row in `ub.PIECES`
    # gets a guessed `module x 0.4 m` strip footprint and every adjacency
    # answer built on it is garbage, with no error raised anywhere. Refuse
    # rather than plan tears on geometry nobody measured.
    missing = sum(1 for e in g.els if ub.PIECES.get(e.get("name")) is None)
    if missing:
        _note(pctx, "tears: skipped — {0} piece(s) with no urban_building."
                    "PIECES row (footprint would be guessed)".format(missing))
        return []
    by_path = {_path(e): e for e in g.els if _path(e)}
    prng = _tear_rng(info, plan)
    moved = set(plan["displaced"]) | set(p for p, _s in plan["panels"])
    by_mass = {}
    for p in plan["_removed_set"]:
        e = by_path.get(p)
        if e is None or p in moved:
            continue
        by_mass.setdefault(e.get("mass") or "main", []).append(e)
    out, scope = [], {}
    for mass in sorted(by_mass):
        kill = by_mass[mass]
        m = info["masses"].get(mass) or info["masses"]["main"]
        sides = sorted({e["side"] for e in kill if e["side"] in SIDES})
        storeys = sorted({int(e["storey"]) for e in kill})
        if not sides:
            continue
        edge_plan = {"mass": mass, "sides": tuple(sides), "storeys": storeys,
                     "pad_m": TEAR_PAD_M, "kill": kill}
        # `plan_edges` needs only `ctx["info"]["elements"]`.
        jobs = fc.plan_edges({"info": info}, edge_plan, m, prng,
                             tol=QS_TEAR_TOL_M,
                             budget=max(0, QS_MAX_TEARS - len(out)))
        for j in jobs:
            e = j["el"]
            dropped = bool(j.get("dropped"))
            p = _path(e)
            # A core piece (`classify` still calls it "wall" mid-plan) is
            # several metres inboard — tearing it puts a ragged hole in the
            # middle of the plan where nothing is visible. A piece that is
            # itself removed or about to be MOVED must never also be torn:
            # a torn piece is never a removed piece (the rule the round-6b
            # `_sweep_roof_props_sliced` rect union depends on), and
            # `_break_split`ing a prim that is about to be relocated tears a
            # dead reference out from under the move.
            if (e.get("p") or {}).get("_role") == "core":
                dropped = True
            if p in moved or p in plan["_removed_set"]:
                dropped = True
            j["dropped"] = dropped
            j["mass"] = mass
        out += jobs
        scope[mass] = {"sides": sides, "storeys": storeys}
    plan["tear_scope"] = scope
    return out


def _tears_to_json(jobs):
    """`plan["tears"]`, stripped to plain JSON-safe records.

    `j["el"]` is a live reference to an element dict — fine within one
    process, but `plan_to_json` must not be asked to serialise it (a bug in
    a recipe is meant to raise there, not this). `_author_tears` re-resolves
    the element by prim path off `ctx["info"]["elements"]`, which already
    carries it, so nothing is lost by dropping the reference here — and a
    plan round-tripped through JSON authors identically to one fresh off
    `plan_damage`.
    """
    out = []
    for j in jobs:
        e = j.get("el") or {}
        out.append({
            "path": _path(e), "side": j.get("side"),
            "storey": int(j.get("storey", 0) or 0),
            "mass": j.get("mass", "main"),
            "classes": list(j.get("classes") or ()),
            "cuts": [dict(c) for c in (j.get("cuts") or ())],
            "dropped": bool(j.get("dropped")),
        })
    return out


# ---------------------------------------------------------------------------
# THE PLANNER
# ---------------------------------------------------------------------------
def plan_damage(info, elements, grade_or_recipes, btype, rng):
    """Everything an earthquake does to one sliced building, decided with NO
    USD access at all.

    THE PLAN (JSON-serialisable throughout — matrices are specs, not
    matrices, and every number is a plain float):

        btype, style, grade, H       what was asked for
        recipes   [[name, kwargs]]   the GUARDED list actually run
        removed   [prim_path]        deactivate these
        displaced {prim_path: spec}  rigid move; `rigid_matrix(spec)` is the
                                     4x4, `apply_rigid(spec, p)` the point
        panels    [[prim_path, [sx, sy, sz]]]   pieces laid on a pile
        piles     [pile spec]        see `_pile`; `_PILE_KW` is forwarded to
                                     `quake_rubble.plan_pile`, the rest is
                                     this plan's own record
        stub      {keep, height_m, residual_m, sides_kept, sides_lost, kept}
        glass     [prim_path]        pieces whose panes go to the void tone
        glass_bands [{sides, storeys, frac, n}]
        macroblocks [{path, side, storey, deg, pivot, size}]
        regions   [{recipe, ..., cells}]        what was planned to be lost
        interior  {storeys, sides}   opened up; litter is NOT authored yet
        fit_ops   [{op, ...}]        what to do with `ctx["fit"]`
        ground    {recipe, kwargs}   the foundation family, or None
        collapse / storey_collapse   the DG4-DG5 summaries
        tears     [{path, side, storey, mass, classes, cuts, dropped}]
                                     round-6b: the ragged tear job on every
                                     surviving piece touching a hole (§
                                     `_plan_tears`). JSON-safe as stored;
                                     `_author_tears` re-resolves each `path`
                                     against `ctx["info"]["elements"]`.
        tear_scope {mass: {sides, storeys}}   which side/storey bands lost
                                     pieces, for `_author_floor_edges`
        notes     [str]              one human-readable line per recipe
        stats     {...}
    """
    recs, guard_notes = _resolve(grade_or_recipes, btype, info)
    plan = {
        "btype": btype, "style": info.get("style"),
        "grade": grade_or_recipes if isinstance(grade_or_recipes, str) else None,
        "H": float(info.get("H") or 0.0),
        "recipes": [[n, dict(kw or {})] for n, kw in recs],
        "removed": [], "displaced": {}, "panels": [], "piles": [],
        "stub": None, "glass": [], "glass_bands": [], "macroblocks": [],
        "regions": [], "interior": {"storeys": [], "sides": []},
        "fit_ops": [], "ground": None, "collapse": None,
        "storey_collapse": None, "tears": [], "tear_scope": {},
        "notes": list(guard_notes), "stats": {},
        "_removed_set": set(),
    }
    pctx = _pctx(info, elements, btype, rng, plan)
    for name, kw in recs:
        RECIPES_S[name](pctx, **(kw or {}))
    # ---- the whole perimeter of every hole (round 6b) ---------------------
    # PURE. `fire_collapse.plan_edges` is geometry only — no pxr, no stage —
    # so this stays in the planner and is host-testable, exactly as the kit
    # ladder's own edge pass is checked in `tests/test_fire_collapse.py`.
    plan["tears"] = _plan_tears(pctx, plan)
    return _finalise(pctx, plan)


def _finalise(pctx, plan):
    """A piece can be lost, moved or laid on the pile — never two of them.
    A TORN piece is a fourth state, and a distinct one: it is always a
    SURVIVING piece (never lost, moved or piled) that picked up a ragged cut
    on the edge nearest a hole — see `_plan_tears`'s own note."""
    moved = set(plan["displaced"]) | set(p for p, _s in plan["panels"])
    plan["removed"] = [p for p in plan["removed"] if p not in moved]
    plan.pop("_removed_set", None)
    g = pctx["g"]
    raw_tears = plan.get("tears") or []
    n_tears = sum(1 for j in raw_tears if not j.get("dropped"))
    n_tears_dropped = sum(1 for j in raw_tears if j.get("dropped"))
    plan["tears"] = _tears_to_json(raw_tears)
    plan["stats"] = {
        "n_pieces": len(g.els), "n_sub": g.n_sub, "n_storeys": len(g.storeys),
        "n_removed": len(plan["removed"]), "n_displaced": len(plan["displaced"]),
        "n_panels": len(plan["panels"]), "n_piles": len(plan["piles"]),
        "n_glass": len(plan["glass"]), "n_macroblocks": len(plan["macroblocks"]),
        "n_tears": n_tears, "n_tears_dropped": n_tears_dropped,
        "removed_frac": (len(plan["removed"]) / float(len(g.els))
                         if g.els else 0.0),
    }
    # ORDER, so a re-plan of the same building with the same seed is the same
    # plan byte for byte and a diff of two plans is readable.
    plan["removed"] = sorted(plan["removed"])
    plan["glass"] = sorted(plan["glass"])
    plan["panels"] = sorted(plan["panels"], key=lambda q: q[0])
    return plan


# ---------------------------------------------------------------------------
# THE PLAN, JSON-SAFE — for a bake sidecar to carry
# ---------------------------------------------------------------------------
# `plan_damage`'s own docstring already commits to "JSON-serialisable
# throughout — matrices are specs, not matrices, and every number is a plain
# float", so there is no real TYPE conversion to do here (unlike, say,
# `fire_bake.mass_to_json`, which deliberately drops fields). What these two
# functions are for: a bake launcher writing a `"quake": {"plan": ...}`
# sidecar field (the `slice-buildings-into-kits` skill's "What the sidecar
# must carry for a non-fire disaster" — "write a `plan_to_json`/`plan_from_
# json` pair the way `fire_bake.py` writes `mass_to_json`/`events_to_json`,
# not a reuse of the fire ones, since the field names differ") needs ONE
# named, tested function to call rather than reaching for a bare
# `json.dumps`/`json.loads` at the call site, and a plan round-tripped
# through JSON turns every tuple (`frac`, `pivot`, `axis`, `sides`, ...) into
# a list anyway — canonicalising that HERE, at bake time, means a caller who
# forgot and wrote a tuple somewhere new gets a loud `TypeError` from
# `plan_to_json` instead of a silent list-vs-tuple mismatch three call sites
# downstream.
def plan_to_json(plan):
    """`plan_damage`'s return, canonicalised to plain JSON-native data (every
    tuple becomes a list, `sort_keys=True` so two plans from the same seed
    serialise identically). Raises `TypeError` if `plan` carries anything
    that is not already JSON-safe — a bug in a recipe, not a legitimate use
    of this function, so it is not caught here."""
    return _json.loads(_json.dumps(plan, sort_keys=True))


def plan_from_json(data):
    """The inverse of `plan_to_json`. A plain `dict` copy is the whole of it:
    `apply_plan` never reads `plan["_removed_set"]` (the one key `_finalise`
    pops before `plan_damage` ever returns a plan), so there is nothing here
    to reconstruct — the loaded dict is already exactly what `apply_plan`
    would be handed fresh off `plan_damage`."""
    return dict(data)


# ---------------------------------------------------------------------------
# THE USD HALF
# ---------------------------------------------------------------------------
# `disaster.quake_rubble` / `quake_rubble_usd` are written in parallel (work
# packages B and C). They are resolved LAZILY and through these two hooks, so
# (a) this module imports and plans with neither of them on disk, and (b) a
# test can stub them without touching `sys.modules`.
PLAN_PILE = None
AUTHOR = None


def _rubble():
    pp, au = PLAN_PILE, AUTHOR
    if pp is None:
        from . import quake_rubble as _qr
        pp = _qr.plan_pile
    if au is None:
        from . import quake_rubble_usd as _qru
        au = _qru.author
    return pp, au


def _glass_void(mats):
    """What a broken pane becomes. NOT `mats["glass"]` — that is the pale
    intact pane tint and it renders as a bright rectangle where a window
    should now be a dark hole."""
    for key in ("void", "scar_shadow", "crack", "dark_concrete"):
        if mats and mats.get(key) is not None:
            return mats[key]
    return None


def _tex_of(mat_prim):
    """The diffuse texture URL of a material prim, or None. A cut-down
    `gac_fire._diffuse_of` — that module belongs to the live fire session and
    is not imported here."""
    try:
        from pxr import UsdShade
        mat = UsdShade.Material(mat_prim)
        src = mat.ComputeSurfaceSource()[0] if mat else None
        if not src:
            return None
        for inp in src.GetInputs():
            nm = inp.GetBaseName().lower()
            if "diffuse" in nm or "basecolor" in nm or "albedo" in nm:
                v = inp.Get()
                if v is None:
                    continue
                return getattr(v, "resolvedPath", None) or getattr(v, "path", None) or str(v)
    except Exception:
        return None
    return None


def _void_glass(stage, paths, mat, glass_tex=None):
    """Rebind the GLASS subsets of `paths` to the void tone.

    A sliced piece keeps the source asset's GeomSubsets, one per material, so
    the panes are addressable even though the slicer cannot cut a window out —
    the same read `gac_fire.darken_glass` does for a fire, by material name
    and by diffuse-texture filename.
    """
    if mat is None or not paths:
        return 0
    from pxr import Usd, UsdGeom, UsdShade
    if glass_tex is None:
        try:
            from detail.gac_slice import GLASS_TEX as glass_tex
        except Exception:
            glass_tex = ("glass", "window", "curtain", "glazing", "win")
    n = 0
    for path in paths:
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        for mesh in Usd.PrimRange(prim):
            if not mesh.IsA(UsdGeom.Mesh):
                continue
            for s in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(mesh)):
                cur = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
                if not cur or not cur.GetPrim().IsValid():
                    continue
                name = cur.GetPrim().GetName().lower()
                tex = (_tex_of(cur.GetPrim()) or "").rsplit("/", 1)[-1].lower()
                if any(g in name for g in glass_tex) or (
                        tex and any(g in tex for g in glass_tex)):
                    UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(mat)
                    n += 1
    return n


def _fit_storey_of(path):
    """The storey index encoded in a `part_<mass>_<i>_<k>` / `prop_...` name."""
    try:
        return int(str(path).rsplit("_", 2)[-2])
    except (ValueError, IndexError):
        return 0


def _chip_dropped_fitout(ctx, slab_paths, col_paths, part_paths):
    """Chip `fit_interior`'s own `_box`-authored slabs/columns/partitions
    once a recipe has dropped them as part of a crushed block — round-5
    follow-up, the "rectangular and cuboid debris/broken parts still exist"
    finding (probe: `slab_main_1..10` under `.../fit_g0/`, each a clean
    6-face box after `displace_above` moved the whole storey rigidly and
    nothing else touched it).

    These are NEVER a sliced/clipped shell — `fit_interior` authors them
    itself, plain boxes, no UVs — so handing them to
    `quake_collapse._chip_prim` (the SAME `fracture.chip_box` round trip the
    KIT ladder's own dropped floor slabs get in `_fall_fitout`) is safe in a
    way it is NOT for a sliced piece; `_chip_prim` also refuses anything with
    a real `st` primvar as a second guard. Slabs get the slab treatment
    (`_CHIP_SLAB`, tessellated — an 8-corner plate has nothing for the
    roughening pass to displace otherwise — and rebound to the
    Damaged_Concrete_Floor beam look); columns are RC-only
    (`fit_interior`: `if columns and btype != "urm"`) so they get the same
    beam look; partitions keep their plaster binding (`beam=False`).
    `quake_collapse._chip_pieces` is itself gated on `fracture.chips_enabled()`
    (`QC_CHIP=0`) and `quake_flow._RUBBLE_MODE` ("v2", the default), so this
    is a no-op under either escape hatch — the boxes come out byte-identical
    to what `fit_interior` authored.

    ONE proof line per call (a handful per building, never per piece) — the
    exact gap the round-5 diagnosis called out: "no positive log evidence
    that chips fire during a real bake."
    """
    from . import fracture
    from . import quake_collapse as qc
    n = 0
    if slab_paths:
        n += qc._chip_pieces(ctx, slab_paths, qc._CHIP_SLAB,
                             tessellate=True, beam=True)
    if col_paths:
        n += qc._chip_pieces(ctx, col_paths, qc._CHIP_PRISM,
                             tessellate=True, beam=True)
    if part_paths:
        n += qc._chip_pieces(ctx, part_paths, qc._CHIP_PRISM, tessellate=True)
    total = len(slab_paths) + len(col_paths) + len(part_paths)
    if total:
        print("[chip] quake_sliced fit-out ({0}): {1} chipped, {2} "
              "passed-through (vtk={3})".format(
                  ctx.get("tag"), n, total - n, fracture.chips_enabled()))
    return n


def _apply_fit_ops(stage, ctx, plan):
    fit = ctx.get("fit") or {}
    dead = set()
    for op in plan.get("fit_ops") or []:
        kind = op.get("op")
        mt = op.get("mass", "main")
        if kind == "displace_above":
            k = int(op.get("storey", 0))
            M = _gf(op["transform"])
            slab_paths = [pth for (m_, i), pth in (fit.get("slabs") or {}).items()
                          if m_ == mt and i > k and pth and pth not in dead]
            col_paths = [p for (m_, i), cols in (fit.get("columns") or {}).items()
                        if m_ == mt and i > k for p in cols if p not in dead]
            part_paths = [q for q in (fit.get("partitions") or [])
                          if _fit_storey_of(q) > k and q not in dead]
            prop_paths = [p for (m_, i), props in (fit.get("props") or {}).items()
                         if m_ == mt and i > k for p in props if p not in dead]
            paths = [q for q in dict.fromkeys(
                slab_paths + col_paths + part_paths + prop_paths) if q]
            if paths:
                qf._transform_prims(stage, paths, M)
                ctx["static_extra"] += paths
                # ROUND-5 FOLLOW-UP: `paths` here is a whole crushed block
                # riding down as ONE rigid body — right for the transform,
                # wrong for the READ (see `_chip_dropped_fitout`'s
                # docstring). Props are skipped: `_prop` REFERENCES a
                # Nucleus furniture asset, never authors a box, and must
                # never be handed to VTK.
                _chip_dropped_fitout(ctx, slab_paths, col_paths, part_paths)
        elif kind == "columns_to_pile":
            cols = list((fit.get("columns") or {}).get((mt, int(op.get("storey", 0))), []))
            i = int(op.get("pile", -1))
            inc = float(op.get("incline_deg") or 0.0)
            side = op.get("incline_side") or "S"
            mm = ctx["info"]["masses"].get(mt) or ctx["info"]["masses"]["main"]
            ox, oy = _outward(mm, side)
            for cpath in cols:
                try:
                    _c = qf._box_dims(stage, cpath)
                    cx_, cy_, cz_ = float(_c[0]), float(_c[1]), float(_c[2])
                    size = [float(_c[3]), float(_c[4]), float(_c[5])]
                except Exception:
                    cx_ = cy_ = cz_ = 0.0
                    size = [qf.COLUMN_W, qf.COLUMN_W, 2.5]
                if inc > 0.0 and cz_:
                    # A RACKED COLUMN IS NOT A BROKEN ONE. In a sidesway the
                    # columns hinge at both ends and finish standing at the
                    # rack angle, which is the whole read of the mechanism at
                    # street level; they are tilted about their own BASE
                    # toward the side the block went, then handed to the pile.
                    qf._transform_prims(stage, [cpath], _gf(_disp(
                        pivot=(cx_, cy_, cz_ - size[2] / 2.0),
                        axis=(-oy, ox, 0.0), deg=inc,
                        why="column racked with the storey")))
                if 0 <= i < len(plan["piles"]):
                    plan["piles"][i]["panels"].append([cpath, size])
        elif kind in ("deactivate_slabs", "deactivate_columns",
                      "deactivate_partitions"):
            sts = op.get("storeys")
            if kind == "deactivate_slabs":
                targets = [p for (m_, i), p in (fit.get("slabs") or {}).items()
                           if m_ == mt and (sts is None or i in sts)]
            elif kind == "deactivate_columns":
                targets = [p for (m_, i), ps in (fit.get("columns") or {}).items()
                           if m_ == mt and (sts is None or i in sts) for p in ps]
            else:
                targets = [p for p in (fit.get("partitions") or [])
                           if sts is None or _fit_storey_of(p) in sts]
            for p in targets:
                if qf._deactivate(stage, p):
                    dead.add(p)
        elif kind == "bury_props":
            sts = op.get("storeys")
            props = [p for (m_, i), ps in (fit.get("props") or {}).items()
                     if m_ == mt and (sts is None or i in sts) for p in ps]
            if props:
                qf._a_bury_props(ctx, props, float(op.get("base_z", 0.0)),
                                 float(op.get("heap_h", 1.0)))
                dead |= set(props)
    if dead:
        fit["all"] = [q for q in (fit.get("all") or []) if q not in dead]
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q not in dead]
    return dead


def _gf(spec):
    """The displacement spec as a Gf.Matrix4d — the SAME 16 numbers
    `rigid_matrix` gives a test."""
    from pxr import Gf
    return Gf.Matrix4d(*[float(v) for v in rigid_matrix(spec).ravel()])


def apply_plan(stage, ctx, plan, verbose=True):
    """Do to the stage what `plan_damage` decided. Returns a small summary."""
    info = ctx["info"]
    by_path = {}
    for e in info["elements"]:
        p = (e.get("p") or {}).get("prim_path")
        if p:
            by_path[p] = e
    mats = ctx.get("mats") or {}

    # 1) glass first: it binds materials on pieces that may be about to move,
    #    and a moved piece carries its bindings with it.
    n_glass = 0
    if plan.get("glass"):
        n_glass = _void_glass(stage, plan["glass"], _glass_void(mats))

    # 2) removal
    n_rm = 0
    for path in plan.get("removed") or ():
        if qf._deactivate(stage, path):
            n_rm += 1
        e = by_path.get(path)
        if e is not None:
            e["dead"] = True

    # 2.5) THE PERIMETER OF EVERY HOLE (round 6b). After removal — the
    #      pieces a tear is "against" must already be gone from the live
    #      element table — and BEFORE the rigid displacement, so a
    #      macroblock that is about to be rotated into the street is still
    #      whole when this runs: `_break_split` deactivates its source, and
    #      a moved-then-torn piece would be a dead prim sitting in the
    #      street. `_plan_tears` already dropped any job whose target is in
    #      `plan["displaced"]` / `plan["panels"]`, so this ordering is a
    #      second, cheap guarantee of the same thing, not the only one.
    n_tear = _author_tears(stage, ctx, plan)

    # 3) rigid displacement, grouped so one matrix is built per distinct move
    groups = {}
    for path, spec in sorted((plan.get("displaced") or {}).items()):
        groups.setdefault(_json.dumps(spec, sort_keys=True), []).append(path)
    n_mv = 0
    for key, paths in sorted(groups.items()):
        M = _gf(_json.loads(key))
        n_mv += qf._transform_prims(stage, paths, M)
        ctx["static_extra"] += paths

    # 4) the fit-out (columns become pile elements, so BEFORE the piles)
    _apply_fit_ops(stage, ctx, plan)

    # 5) the piles
    n_pile = 0
    if plan.get("piles"):
        try:
            plan_pile, author = _rubble()
        except ImportError as exc:
            print("[quake_sliced] RUBBLE MISSING ({0}) — {1} pile(s) NOT "
                  "authored; the collapse has no mound".format(exc, len(plan["piles"])))
            plan_pile = author = None
        if plan_pile is not None:
            for i, spec in enumerate(plan["piles"]):
                n_pile += _author_pile(stage, ctx, plan, spec, i,
                                       plan_pile, author)

    # 6) the foundation family
    if plan.get("ground"):
        _apply_ground(ctx, plan)

    for line in plan.get("notes") or ():
        if line not in ctx["notes"]:
            ctx["notes"].append(line)
    out = {"removed": n_rm, "displaced": n_mv, "glass": n_glass,
           "piles": n_pile, "tears": n_tear}
    if verbose:
        print("[quake_sliced] {0}: {1} piece(s) removed, {2} displaced, "
              "{3} glass subset(s) voided, {4} pile(s), {5} tear(s)".format(
                  plan.get("grade") or "recipes", n_rm, n_mv, n_glass, n_pile,
                  n_tear))
    return out


def _author_tears(stage, ctx, plan):
    """Tear every surviving piece that touches a hole, then ragged the slab
    a removed storey exposes. Returns the total piece + slab count touched.

    `fire_collapse._tear_perimeter` unchanged: one `quake_flow._break_split`
    per piece, that piece's own cut judges unioned, the façade measured
    BEFORE the split and re-skinned onto every static fragment afterwards
    (`facade_skin` / `skin_fragment`). Runs entirely under a PRIVATE
    `fire_collapse._own_rng` pair so ZERO draws come off the shared
    `ctx["rng"]` / `ctx["nrng"]` — a tear pass that touched the shared
    stream would move every later outcome in the same bake process,
    including other buildings.

    `plan["tears"]` is the JSON-safe record `_tears_to_json` wrote (a plan
    round-tripped through `plan_from_json` carries the same shape), so the
    element each job targets is re-resolved here by prim path rather than
    carried as a live reference.
    """
    tears = [t for t in (plan.get("tears") or ()) if not t.get("dropped")]
    n = 0
    n_failed_resolve = 0
    if tears:
        from . import fire_collapse as fc
        from . import fracture
        # `fire_collapse._tear_perimeter` writes `ctx["velocity"][path]` for
        # every loose shard — a key `wreck_sliced`'s own ctx always carries
        # but that a hand-built ctx (a test, a bench) may not, since nothing
        # before this pass ever read it in `quake_sliced`.
        ctx.setdefault("velocity", {})
        info = ctx["info"]
        by_path = {_path(e): e for e in info["elements"] if _path(e)}
        by_mass = {}
        for t in tears:
            by_mass.setdefault(t.get("mass") or "main", []).append(t)
        for mass in sorted(by_mass):
            m = info["masses"].get(mass) or info["masses"]["main"]
            jobs = []
            for t in by_mass[mass]:
                e = by_path.get(t.get("path"))
                if e is None or e.get("dead"):
                    n_failed_resolve += 1
                    continue
                jobs.append({"el": e, "side": t.get("side"),
                            "cuts": t.get("cuts") or []})
            if not jobs:
                continue
            seed = fracture.stable_seed(ctx.get("tag"), mass, "tear")
            prng = random.Random(seed)
            pnrng = np.random.default_rng(seed & 0xFFFFFFFF)
            with fc._own_rng(ctx, prng, pnrng):
                n += fc._tear_perimeter(ctx, plan, m, prng, jobs)
    # THE SLAB HALF. `_a_slab_rim` / `_ragged_slabs` only ever touch
    # `ctx["fit"]["slabs"]` and `_a_roofify`'s roof boxes — authored
    # `_box`es, never a clipped shell — so they are safe here in a way
    # nothing that touches a piece is. Mirrors `quake_collapse._author_one`'s
    # own ordering (crush -> slab rims; otherwise -> ragged slabs).
    n += _author_floor_edges(stage, ctx, plan)
    if tears or n_failed_resolve:
        print("[tear] quake_sliced ({0}): {1} piece(s)/slab(s) touched, "
              "{2} job(s) dropped, {3} could not be re-resolved".format(
                  ctx.get("tag"), n,
                  sum(1 for t in (plan.get("tears") or ()) if t.get("dropped")),
                  n_failed_resolve))
    return n


def _author_floor_edges(stage, ctx, plan):
    """The slab half of the ragged boundary. Only ever touches
    `ctx["fit"]["slabs"]` and `_a_roofify`'s roof boxes — authored `_box`es,
    never a clipped shell.

    Gated on `QS_RAGGED` exactly like `_author_tears`'s piece half, so the
    whole pass is a no-op when it is off and the bake is byte-identical to
    round 6b (`plan["storey_collapse"]` / `plan["tear_scope"]` are set by
    planning regardless — only the AUTHORING is gated here).
    """
    if not QS_RAGGED:
        return 0
    from . import fire_collapse as fc
    from . import fracture
    n = 0
    sc = plan.get("storey_collapse") or {}
    seed = fracture.stable_seed(ctx.get("tag"), "floor_edge")
    prng = random.Random(seed)
    pnrng = np.random.default_rng(seed & 0xFFFFFFFF)
    with fc._own_rng(ctx, prng, pnrng):
        if sc.get("storey") is not None:
            k = int(sc["storey"])
            mt = sc.get("mass", "main")
            n += len(qf._a_slab_rim(ctx, mt, k, n_sides=2))
            n += len(qf._a_slab_rim(ctx, mt, k + 1, n_sides=2, bars=False))
        else:
            for mt, sc2 in sorted((plan.get("tear_scope") or {}).items()):
                sts = set(int(s) for s in sc2.get("storeys") or ())
                for sd in sc2.get("sides") or ():
                    qf._ragged_slabs(ctx, mt, sd, sts)
                    n += 1
    return n


def _author_pile(stage, ctx, plan, spec, i, plan_pile, author):
    m = dict(ctx["info"]["masses"].get(spec.get("mass") or "main")
             or ctx["info"]["masses"]["main"])
    if spec.get("base_z") is not None:
        m["z0"] = float(spec["base_z"])
    kw = {k: spec[k] for k in _PILE_KW if spec.get(k) is not None}
    kw.pop("plate_ok", None)
    if ctx.get("plate_ok") is not None:
        kw["plate_ok"] = ctx["plate_ok"]
    panels = [(p, tuple(s)) for p, s in (spec.get("panels") or []) if p]
    try:
        p = plan_pile(m, plan["btype"], ctx["rng"], panels=panels,
                      seed_tag="{0}_{1}_{2}".format(ctx["tag"], spec.get("tag") or "", i),
                      **kw)
    except TypeError as exc:
        print("[quake_sliced] plan_pile rejected {0}: {1}".format(kw, exc))
        return 0
    out = author(stage, ctx["parent"], p, mats=ctx.get("mats"),
                 tag="{0}_{1}".format(ctx["tag"], spec.get("tag") or i),
                 # a CALLABLE, like quake_flow._rubble's `uid=lambda: _uid(ctx)`
                 # — `author` builds `next_id` from it and CALLS it per large
                 # element; an int here was the pilot bake's
                 # "'int' object is not callable" (2026-08-30)
                 uid=lambda: qf._uid(ctx))
    ctx["static_extra"] += [q for q in (out.get("static") or []) if q]
    ctx["authored"] += [q for q in (out.get("all") or []) if q]
    spec["authored"] = {"n_static": len(out.get("static") or []),
                        "n_all": len(out.get("all") or [])}
    # RECORD THE PLANNER'S OWN NUMBERS. `quake_flow._rubble` (the kit path)
    # appends `dict(plan["stats"])` to `ctx.setdefault("rubble", []).append`
    # for exactly this reason — a bake launcher summarising `fall_sides` /
    # `reach_m` / `extent_m` / `crown_m` into a manifest
    # (`bake_quake_archetypes_launch_script._rubble_fields`) reads that key.
    # The sliced path never populated it, so a sliced-building bake sidecar
    # had nowhere to read the same numbers from. Mirrors `quake_flow._rubble`
    # byte for byte, off `p["stats"]` (`plan_pile`'s own return, the local
    # `p` above — the SAME dict `quake_flow._rubble` calls `plan` and reads
    # `plan["stats"]` off, just named differently here to avoid shadowing
    # this function's own `plan` argument).
    stats = dict(p.get("stats") or {})
    stats.update({"kind": spec.get("kind"), "sides": spec.get("sides"),
                  "tag": spec.get("tag") or i})
    ctx.setdefault("rubble", []).append(stats)
    return 1


def _apply_ground(ctx, plan):
    """SETTLE / TILT / OV. The first three are `quake_flow`'s own recipes,
    unchanged; `overturn` is this module's rigid version."""
    g = plan["ground"]
    name, kw = g.get("recipe"), dict(g.get("kwargs") or {})
    if name == "settlement":
        return qf.r_settlement(ctx, **kw)
    if name == "tilt_severe":
        return qf.r_tilt_severe(ctx, **kw)
    if name == "tilt_sink":
        return qf.r_tilt_sink(ctx, **kw)
    if name == "overturn":
        return _ov_apply(ctx, **kw)
    raise KeyError("unknown ground recipe {0!r}".format(name))


def _ov_apply(ctx, angle_deg=90.0, side="S", mass="main"):
    """The rigid half of an overturn: the whole shell, the fit-out and the
    raft rotate about one base edge; the ground gets `_c_overturn_ground`.

    No `_break`, no `_split_strip` — the two things `quake_flow.r_overturn`
    does that a clipped shell cannot survive.
    """
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    angle = abs(float(angle_deg))
    ox, oy = _outward(m, side)
    lnx, lny = _SIDE_NORMAL[side]
    px, py = _to_world(m, lnx * m["W"] / 2.0, lny * m["D"] / 2.0)
    spec = _disp(pivot=(px, py, m["z0"]), axis=(-oy, ox, 0.0), deg=angle,
                 why="overturn about the {0} edge".format(side))
    raft = qf._raft(ctx, m)
    paths = qf._everything(ctx) + [raft] + list(ctx.pop("c_carry", []))
    qf._transform_prims(ctx["stage"], paths, _gf(spec))
    ctx["static_extra"] += paths
    og = qf._c_overturn_ground(ctx, m, _gf(spec), angle_deg=angle)
    ctx["notes"].append("overturn(rigid): {0:.0f} deg about the {1} edge, onto "
                        "{2}".format(angle, side, (og or {}).get("fall", side)))
    return og


# ---------------------------------------------------------------------------
# ROUND-5 FOLLOW-UP: the floating roof-plant fix
# ---------------------------------------------------------------------------
# User: floating water tanks/silos on GAC buildings. Audit: every
# `gac_SM_Building_02_*` bake seats its roof tank at bottom-z ~ 38.13-38.18 m
# with 2.1-2.6 m of AIR under it (measured support under the footprint
# ~ 35.6 m); `gac_SM_Building_20_DG2` shows the same at 1.25 m.
#
# `quake_flow.dress_roof` seats every tank/AC unit at
# `z = info["masses"][mass]["top"] + 0.02` — and `m["top"]` (`describe` ->
# `_mass_specs`) is the ADVERTISED height: the sum of `ub.STYLES[style]`'s
# own band heights, right for a kit building whose bands ARE its measured
# storey grid. A GAC style's TOP band is measured across a coping/parapet
# strip (`gac_storey_slice`'s own grid), and the real, walkable roof DECK the
# slicer actually placed a "roof"-role piece at sits below that grid line —
# by exactly the gap the user found. `dress_roof` cannot know this: it only
# ever sees `info["masses"]`, never the roof piece's own authored geometry.
#
# The fix stays in THIS module (not `quake_flow.dress_roof`, shared with the
# kit ladder, where `m["top"]` is already correct) and runs AFTER
# `dress_roof`, as a cheap RIGID correction: measure the roof piece(s)
# `describe` already classified `role == "roof"` for, with a bbox cache on
# the geometry the slicer actually authored, and slide every prop
# `dress_roof` just placed down by the one delta between the advertised and
# measured tops (every prop in `ctx["roof_plant"]` was seated at the SAME z,
# so one delta corrects all of them with one `_transform_prims` call).
def _roof_plant_target_z(advertised_top, measured_tops, epsilon=0.02):
    """The z `quake_flow.dress_roof` SHOULD have used for the roof plant.

    `measured_tops` is any iterable of world-space top-z values read off the
    roof piece(s) actually placed under the footprint (`_measure_roof_tops`);
    the MAX of those, when there are any, wins over `advertised_top`
    (`info["masses"][mass]["top"]`, the style's own band-height sum) — never
    the other way round, so a style whose grid is already right (a kit
    building routed here by mistake, or a GAC style whose top band IS the
    deck) measures ~0 correction and this is a no-op.

    PURE: no pxr, no stage, so a test hands it two floats with nothing else.
    """
    tops = list(measured_tops)
    top = float(max(tops)) if tops else float(advertised_top)
    return top + float(epsilon)


def _measure_roof_tops(stage, elements, mass):
    """World-space top-z of every `role == "roof"` piece of `mass`, keyed by
    prim_path — a `UsdGeom.BBoxCache` read of the ACTUAL authored geometry,
    never the style's declared H. A missing/invalid/empty-bound prim is
    skipped, not raised on: a bad measurement should fall back to the
    advertised top (`_roof_plant_target_z`'s own rule), not crash a recipe.
    """
    from pxr import Usd, UsdGeom
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    out = {}
    for e in elements or ():
        if e.get("mass") != mass or e.get("role") != "roof":
            continue
        p = (e.get("p") or {}).get("prim_path")
        if not p or p in out:
            continue
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsValid():
            continue
        try:
            rng_ = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        except Exception:
            continue
        if rng_.IsEmpty():
            continue
        out[p] = float(rng_.GetMax()[2])
    return out


def _mesh_up_faces(prim, xcache, up_threshold):
    """Every UP-FACING triangle of `prim`, in world space, as
    `(tri_z, (ax,ay), (bx,by), (cx,cy))` — a fan triangulation of each face,
    the same `tri_soup` idiom `quake_collapse._deck_support_z` /
    `tools/roof_plant_seat_probe.py` already trust in production, duplicated
    here (not imported) because this pass deliberately has NO per-mesh AABB
    prune (see `_reseat_roof_plant`'s docstring for why one would just
    reintroduce the bug at a different margin).

    Returns `None` when `prim` is not a `UsdGeom.Mesh` at all (a `UsdGeom.
    Cube` test fixture, say — nothing to fan-triangulate, so the caller
    falls back to that prim's own BBoxCache top, which is exact for a
    single un-merged primitive), or `[]` when it is a Mesh with no usable
    geometry."""
    from pxr import UsdGeom
    if not prim.IsA(UsdGeom.Mesh):
        return None
    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not counts or not idx:
        return []
    M = np.array(xcache.GetLocalToWorldTransform(prim), dtype=np.float64)
    P = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float64)
    P = (np.hstack([P, np.ones((len(P), 1))]) @ M)[:, :3]
    counts = np.asarray(counts)
    idx = np.asarray(idx)
    off = np.concatenate([[0], np.cumsum(counts)[:-1]])
    out = []
    for n, o in zip(counts, off):
        if n < 3:
            continue
        fan = idx[o:o + int(n)]
        a = P[fan[0]]
        for k in range(1, int(n) - 1):
            b, c = P[fan[k]], P[fan[k + 1]]
            nrm = np.cross(b - a, c - a)
            mag = float(np.linalg.norm(nrm))
            if mag <= 1e-9:
                continue
            nz = float(nrm[2]) / mag
            if nz <= up_threshold:
                continue
            out.append((float((a[2] + b[2] + c[2]) / 3.0),
                       (float(a[0]), float(a[1])),
                       (float(b[0]), float(b[1])),
                       (float(c[0]), float(c[1]))))
    return out


def _pt_in_tri(px, py, a, b, c):
    """Sign-based point-in-triangle, exactly `_deck_support_z`'s own test —
    the QUERY point against the CANDIDATE's triangle, never the other way
    round (a candidate face many times a prop's footprint's size, the usual
    shape for a kit/GAC slab, would never contain ITS OWN centroid inside a
    small footprint if the test ran backwards)."""
    d1 = (px - b[0]) * (a[1] - b[1]) - (a[0] - b[0]) * (py - b[1])
    d2 = (px - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (py - c[1])
    d3 = (px - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (py - a[1])
    neg = d1 < 0 or d2 < 0 or d3 < 0
    pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (neg and pos)


def _reseat_roof_plant(stage, ctx, mass="main"):
    """Correct `dress_roof`'s tank/AC seat for a SLICED (GAC) building —
    round-5 follow-up, the floating-roof-plant finding. See the section note
    above. Call AFTER `qf.dress_roof(ctx)`, before anything else moves
    `ctx["roof_plant"]`. Returns how many props were moved (0 = no
    correction needed, or nothing to correct).

    ROUND-7 FIX — POINTS-BASED, UNDER THE PROP'S OWN FOOTPRINT. The round-6
    version (per-prop footprint, but scored by a candidate's WHOLE-PIECE
    `UsdGeom.BBoxCache` top) still left `SM_Building_19`/`_12`/`_24` floating
    2.5-3.8 m after a full re-bake with new seeds — the SAME gaps before and
    after, so a deterministic geometric miss, not settle noise. Ground truth
    (bare `pxr`, points, no Kit): a real up-facing deck triangle exists
    EXACTLY under every one of those tanks' own footprints (measured:
    `SM_Building_19` 69.276 m and `SM_Building_24` ~40.0-40.3 m, matching
    each tank's own seat to within centimetres) — inside a merged `roof_x_*`
    piece whose OVERALL bbox reaches 1-2.4 m higher because the SAME prim
    also carries a coping run, or a raised section, ELSEWHERE on the roof. A
    bbox-of-the-whole-piece score cannot tell "the deck under THIS footprint"
    from "the tallest thing anywhere in this prim" — the exact
    `UsdGeom.BBoxCache` blind spot the `fix-floating-debris` skill already
    named for fracture debris, here for a stepped/multi-level roof instead.
    `SM_Building_12` shows the other-signed failure of the SAME root cause:
    the round-6 code's `ctop <= pz0 + 0.30` ceiling (sized for "a few cm of
    interpenetration" — the ORIGINAL bug's shape, advertised too HIGH)
    excludes the true roof band entirely when the advertised seat is instead
    metres too LOW, and the search falls through to a bare, storey-height
    WALL SHELL (zero real deck, e.g. `core_x_0_20` — a piece with no
    up-facing triangle at all) that merely happens to still qualify.

    The fix: score each candidate by the highest UP-FACING triangle actually
    reachable under the PROP'S OWN footprint (`_mesh_up_faces` + `_pt_in_
    tri`, the query-RECTANGLE-vs-triangle idiom `quake_collapse._deck_
    support_z` already trusts in production) instead of its whole-mesh
    envelope, and drop the `ctop <= pz0 + slack` ceiling altogether: a
    containment test that only ever fires on a genuine up-facing surface
    cannot be fooled by an unrelated tall neighbour the way a bbox-overlap
    test can (a bare wall contributes NO candidate triangles at all, so it
    can never win regardless of its own height), so the search is free to
    find the real deck whether the advertised seat guessed too high or, as
    these three buildings show, far too low. `info["elements"]` is one
    building's own small piece roster (tens to a few hundred cells), not a
    whole-stage scan, so — unlike `_deck_support_z` — this can afford to
    skip the coarse per-mesh AABB prune entirely instead of tuning a margin
    wide enough to survive a coping run without reintroducing the same
    blind spot at a different scale.

    A candidate that is not a `UsdGeom.Mesh` (a `UsdGeom.Cube` test fixture)
    has no points/faces to fan-triangulate, so it falls back to its own
    BBoxCache top — exact for a single un-merged primitive, which is all a
    non-Mesh candidate here ever is."""
    info = ctx["info"]
    m = (info.get("masses") or {}).get(mass)
    plant = list(ctx.get("roof_plant") or ())
    if not plant or m is None:
        return 0
    from pxr import Usd, UsdGeom
    from . import quake_collapse as qc          # read-only: a shared constant
    up_threshold = qc.ROOF_PROP_UP_THRESHOLD
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    cand = []
    for e in info.get("elements") or ():
        if e.get("mass") != mass:
            continue
        # NEVER a parapet: a coping run's own up-facing cap sits at the
        # ADVERTISED height (or close to it) and its thin-wall footprint
        # overlaps any edge-seated prop's footprint too, so with it in the
        # pool the rim wins the support max and the prop stays ~3 m above
        # the real deck (SM_02 DG4, round-6 audit). A tank stands on the
        # DECK; rims only ever stand beside it — true whether the rim is
        # scored by bbox or, as here, by its own up-facing triangles.
        if "parapet" in str(e.get("role") or ""):
            continue
        pth = (e.get("p") or {}).get("prim_path")
        if not pth:
            continue
        prim = stage.GetPrimAtPath(pth)
        if not prim or not prim.IsValid():
            continue
        try:
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        except Exception:
            continue
        if r.IsEmpty():
            continue
        lo, hi = r.GetMin(), r.GetMax()
        faces = _mesh_up_faces(prim, xcache, up_threshold)
        cand.append((float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]),
                    faces, float(hi[2])))
    moved, deltas = 0, []
    for pth in plant:
        prim = stage.GetPrimAtPath(pth)
        if not prim or not prim.IsValid():
            continue
        try:
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        except Exception:
            continue
        if r.IsEmpty():
            continue
        lo, hi = r.GetMin(), r.GetMax()
        px0, py0, pz0 = float(lo[0]), float(lo[1]), float(lo[2])
        px1, py1 = float(hi[0]), float(hi[1])
        qxy = ((px0, py0), (px1, py0), (px1, py1), (px0, py1),
              ((px0 + px1) / 2.0, (py0 + py1) / 2.0))
        # support = highest UP-FACING surface actually reachable under this
        # prop's own footprint, across every non-parapet candidate — no
        # ceiling relative to `pz0` (see the docstring above for why one
        # is no longer needed).
        sup = None
        for cx0, cy0, cx1, cy1, faces, cbtop in cand:
            if not (cx0 < px1 and cx1 > px0 and cy0 < py1 and cy1 > py0):
                continue
            if faces is None:
                sup = cbtop if sup is None else max(sup, cbtop)
                continue
            for tz, a, b, c in faces:
                if sup is not None and tz <= sup:
                    continue
                if any(_pt_in_tri(qx, qy, a, b, c) for qx, qy in qxy):
                    sup = tz
        if sup is None:
            continue
        delta = (sup + 0.02) - pz0
        if abs(delta) < 0.01:
            continue
        if qf._transform_prims(stage, [pth], qf._translate(0.0, 0.0, delta)):
            moved += 1
            deltas.append(delta)
    if moved:
        ctx["notes"].append(
            "[qgac] roof_plant reseated (per-footprint): {0} piece(s) moved "
            "{1:+.2f}..{2:+.2f} m".format(moved, min(deltas), max(deltas)))
    return moved


# ---------------------------------------------------------------------------
# ROUND-6 FOLLOW-UP: the roof-plant fix that reaches DAMAGE, not just dress
# ---------------------------------------------------------------------------
# `_reseat_roof_plant` (above) fixes `dress_roof`'s seat BEFORE any damage
# recipe runs — right for "the advertised top was never the real deck," wrong
# for "a recipe below just took the real deck away." Audit (2026-08-31,
# `quake_collapse.py`'s round-6 fix for the kit ladder): `quake_sliced.py`
# imports `quake_collapse` for nothing but `_chip_prim`, and every `s_*`
# recipe here is a PURE PLANNER — `plan_damage(info, elements, ..., rng)`
# takes no `ctx`, so a recipe cannot see `ctx["roof_plant"]` even though
# `dress_roof` already ran and the props already exist on the stage by the
# time planning happens. So neither of `quake_collapse`'s two mechanisms
# (`_author_band`'s in-line carry, `_sweep_roof_props`'s in-line re-seat) can
# be inlined into an `s_*` function here — both have to be a POST-`apply_
# plan` pass instead, reading back what the plan already did.
#
# TWO MECHANISMS, reused by IMPORT, none reinvented:
#
#   1. BAND / STOREY CARRY. `s_soft_storey` (and `s_mid_storey` /
#      `s_storey_collapse`, which call it) already record the ONE rigid
#      transform "the block above storey k" got, as a `fit_ops` entry
#      (`{"op": "displace_above", "mass", "storey", "transform": <spec>}`) —
#      the SAME spec `apply_plan` fed `_gf` + `qf._transform_prims` for every
#      piece above the crush. Any surviving `ctx["roof_plant"]` / `ctx["roof_
#      fixed"]` path at or above that storey's ceiling gets the identical
#      transform, so it rides down and tilts with the block it was standing
#      on — the missing half `quake_collapse._author_band`'s docstring found
#      false for the kit ladder, true here from the start (nothing here ever
#      claimed otherwise; there was simply no call site for it before now).
#
#   2. REGION REMOVAL. `s_corner_fail` / `s_out_of_plane` (`_apply_region` /
#      `_remove`) delete pieces outright, so there is no spec to carry — only
#      `plan["removed"]`, a flat list of PATHS. `quake_collapse.
#      roof_prop_footprint_lost`'s test (a prop's 5 points, majority inside
#      `plan["region"]`) needs a world-space region polygon quake_collapse's
#      OWN planner can build (`fc.in_region` + `_to_local`) because it runs
#      against a fixed module grid; this module's plan has no such polygon,
#      only paths. The equivalent built here: the world XY rectangles
#      (widened `_REGION_MARGIN_M` so two adjoining removed pieces bridge
#      the gap between them) of every REMOVED piece whose OWN bbox reaches
#      within one storey height of the roof — i.e. it was part of the top
#      band, not a ground-floor infill panel — unioned, then the SAME
#      5-point-majority test against that union. A majority match is placed
#      on real support via `quake_collapse._deck_support_z` (the `tri_soup`
#      probe, unmodified) and tipped/buried exactly as the kit ladder now
#      does: `quake_collapse.ROOF_PROP_BIG_TIP_STOREY_FRAC` decides the
#      threshold, `quake_flow._a_bury_props` supplies the DG5-style tip/roll
#      for a real fall, `quake_flow.B_ROOF_PLANT_TIP_DEG` the idle tip for a
#      shallow one. KNOWN APPROXIMATION: `out_of_plane`'s macroblocks/panels
#      are pulled back OUT of `plan["removed"]` once picked (they topple
#      into the street rather than vanish), so the union can have a small
#      gap exactly at a macroblock's ORIGINAL cell — `_REGION_MARGIN_M`
#      bridges a single missing cell between two removed neighbours; it does
#      not chase the macroblock's new position.
#
# TOTAL COLLAPSE IS OUT OF SCOPE HERE, ON PURPOSE: `plan.get("collapse")`
# (set by `_total_collapse`, i.e. `masonry_collapse` / `pancake`) skips
# mechanism 2 entirely, same as `quake_collapse._sweep_roof_props` keeps its
# own total/pancake branch separate from elevation/corner — `wreck_sliced`'s
# very next line, `qf._b_settle_roof_plant(ctx, ...)`, already owns burying
# roof plant on a total collapse BY RECIPE NAME, and re-testing every prop
# against the fresh pile here would double-process exactly what that call is
# about to do.
_REGION_MARGIN_M = 0.6


def _sweep_roof_props_sliced(stage, ctx, plan):
    """Post-`apply_plan` roof-plant fix for a sliced building. See the
    section note above for the two mechanisms and why neither can run
    in-line inside an `s_*` recipe. Call AFTER `apply_plan`, BEFORE
    `quake_flow._b_settle_roof_plant`. Returns `(n_carried, n_reseated)`."""
    plant = list(dict.fromkeys(
        list(ctx.get("roof_plant") or ()) + list(ctx.get("roof_fixed") or ())))
    if not plant:
        return 0, 0

    from pxr import Usd, UsdGeom
    info = ctx["info"]
    roof_mass = ctx.get("roof_plant_mass", "main")
    m = info["masses"].get(roof_mass) or info["masses"].get("main")
    if m is None:
        return 0, 0

    xf = UsdGeom.XformCache()
    resolved = set()

    # 1) BAND / STOREY CARRY -------------------------------------------------
    n_carried = 0
    for op in (plan.get("fit_ops") or ()):
        if op.get("op") != "displace_above" or (op.get("mass") or "main") != roof_mass:
            continue
        lv = m["levels"]
        k = int(op.get("storey", 0))
        z_hi = lv[k + 1] if k + 1 < len(lv) else m["top"]
        M = _gf(op["transform"])
        carry = []
        for pth in plant:
            if pth in resolved:
                continue
            pr = stage.GetPrimAtPath(pth)
            if not pr or not pr.IsValid() or not pr.IsActive():
                continue
            try:
                t = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
            except Exception:
                continue
            if float(t[2]) >= z_hi - 0.05:
                carry.append(pth)
        if carry:
            qf._transform_prims(stage, carry, M)
            ctx["static_extra"] += carry
            resolved.update(carry)
            n_carried += len(carry)

    # 2) REGION REMOVAL: majority-footprint-lost, then a geometric re-seat --
    n_reseated = 0
    if not plan.get("collapse"):        # total/pancake: _b_settle_roof_plant's job
        storey_h = max(2.5, float(m["top"]) - float(m["levels"][-1]))
        z_floor = float(m["top"]) - storey_h
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        removed_rects = []
        for pth in (plan.get("removed") or ()):
            pr = stage.GetPrimAtPath(pth)
            if not pr or not pr.IsValid():
                continue
            try:
                r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
            except Exception:
                continue
            if r.IsEmpty():
                continue
            lo, hi = r.GetMin(), r.GetMax()
            if float(hi[2]) < z_floor:
                continue
            removed_rects.append((float(lo[0]) - _REGION_MARGIN_M,
                                  float(lo[1]) - _REGION_MARGIN_M,
                                  float(hi[0]) + _REGION_MARGIN_M,
                                  float(hi[1]) + _REGION_MARGIN_M))

        if removed_rects:
            from . import quake_collapse as qc

            fall = []
            for pth in plant:
                if pth in resolved:
                    continue
                pr = stage.GetPrimAtPath(pth)
                if not pr or not pr.IsValid() or not pr.IsActive():
                    continue
                r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
                if r.IsEmpty():
                    continue
                lo, hi = r.GetMin(), r.GetMax()
                pts = ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]),
                      (lo[0], hi[1]), ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0))
                n_in = sum(1 for px, py in pts
                          if any(rx0 <= px <= rx1 and ry0 <= py <= ry1
                                 for rx0, ry0, rx1, ry1 in removed_rects))
                if n_in * 2 > len(pts):
                    fall.append((pth, lo, hi))

            big_drop_m = storey_h * qc.ROOF_PROP_BIG_TIP_STOREY_FRAC
            rng = ctx["rng"]
            for pth, lo, hi in fall:
                cx = (lo[0] + hi[0]) / 2.0
                cy = (lo[1] + hi[1]) / 2.0
                half_w = max(0.3, (hi[0] - lo[0]) / 2.0 * 1.15)
                half_d = max(0.3, (hi[1] - lo[1]) / 2.0 * 1.15)
                base_z = float(lo[2])
                support = qc._deck_support_z(stage, ctx["parent"], cx, cy,
                                             half_w, half_d, base_z,
                                             exclude=(pth,))
                if support is None:
                    support = float(m["z0"])
                drop = base_z - support
                if drop > big_drop_m:
                    # a real fall: the DG5-style bury dressing, reused
                    # verbatim (see `quake_collapse._sweep_roof_props`'s own
                    # comment on why this is reuse and not invention).
                    qf._a_bury_props(ctx, [pth], support,
                                     max(0.4, drop * 0.15), keep=1.0)
                else:
                    if drop > 0.02:
                        qf._transform_prims(
                            stage, [pth],
                            qf._translate(0.0, 0.0, support - base_z))
                    c = qf._pivot_of(ctx, pth)
                    Mtip = qf._rot_about(
                        c, (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0),
                        rng.uniform(-qf.B_ROOF_PLANT_TIP_DEG,
                                    qf.B_ROOF_PLANT_TIP_DEG))
                    qf._transform_prims(stage, [pth], Mtip)
                    ctx["static_extra"].append(pth)
                resolved.add(pth)
                n_reseated += 1

    if ctx.get("roof_plant"):
        ctx["roof_plant"] = [p for p in ctx["roof_plant"] if p not in resolved]
    if ctx.get("roof_fixed"):
        ctx["roof_fixed"] = [p for p in ctx["roof_fixed"] if p not in resolved]
    if n_carried or n_reseated:
        ctx["notes"].append(
            "roof_plant (sliced): {0} carried with a storey crush, {1} "
            "re-seated on real support after a region loss".format(
                n_carried, n_reseated))
    return n_carried, n_reseated


# ---------------------------------------------------------------------------
# STRANDED SHELL BANDS — the sky-grid finding (round 8, `stranded-bands`)
# ---------------------------------------------------------------------------
# Confirmed by photo + USD forensics on `eq500_v4` (`se_obl.png` ~850-1550,
# 0-420; `nw_obl.png` ~690-815, 75-410): regular grids/columns of tan wall/
# parapet ring pieces hanging in the sky. `_sweep_roof_props_sliced` above
# fixes the ROOF-PLANT population (tanks, AC units — furniture `dress_roof`
# placed on top of the deck); it says nothing about the SHELL ITSELF —
# `gac_storey_slice.ring()` / `roof_and_parapet()`'s own `wall_*` / `corner_*`
# / `parapet_*` placements — which is a different population with no carrier
# of its own until now (`fire_bake.deactivate_airborne` is the kit ladder's
# analog; this module never had one).
#
# TWO CASES, TRACED SEPARATELY:
#
# 1. BAND / STOREY CRUSH (`displace_above`). `s_soft_storey` already loops
#    `g.els` for `_storey > k` with NO role filter at all, so every wall /
#    pier / corner / parapet / parapet_corner / roof piece above the crushed
#    band is already in `plan["displaced"]` and rides the identical spec
#    `_apply_fit_ops`'s own "displace_above" branch uses for the fit-out.
#    VERIFIED empirically (a fixture probe against `plan_damage` output: 59/59
#    roof+parapet placements above a crushed storey land in `plan["displaced"]`,
#    zero orphans) and pinned by
#    `test_soft_storey_carries_every_shell_role_above_the_crush` below — no
#    code change was needed here, only the regression test.
#
# 2. REGION REMOVAL (`corner_fail` / `out_of_plane`). `_apply_region`'s own
#    `_boundary()` test counts a VERTICAL neighbour (`st-1`, `st+1`) as a
#    boundary condition exactly like a horizontal one, so a wide, MULTI-
#    STOREY region's two edge-bay columns are boundary cells at EVERY storey
#    of the region, not just its top and bottom row. `kept_piers` then rolls
#    keep/lose PER CELL, independently — so it is entirely possible (measured:
#    3-12 instances per seed on a 10-storey fixture, `out_of_plane`) for the
#    pier at (side, st, bay) to survive the draw while the pier at
#    (side, st-1, bay) — directly beneath it, in the SAME column, part of the
#    SAME region — loses its own, independent draw. The survivor is left
#    exactly where it was authored: nothing rigid-transforms it, nothing gives
#    it physics, nothing sweeps it. That is the sky-grid: a periodic (bay-
#    pitch) pattern of kept boundary pieces whose own support just vanished
#    two rows below.
#
# THE REPAIR IS DELIBERATELY NOT IN `_apply_region`/`plan_damage` ITSELF: the
# pure planner's own RNG draws and `plan["removed"]` must stay bit-identical
# (another agent's materials work reads the same plan), so this runs as a
# POST-`apply_plan` stage pass, same shape as `_sweep_roof_props_sliced`
# above: a CHEAP, PURE pre-filter off the grid (no stage) finds candidates,
# then the real geometry (`quake_collapse._deck_support_z`, the SAME
# `tri_soup` idiom, with its `candidates=` cache built ONCE per building so
# the cost is bounded regardless of how many candidates there are) decides
# what is actually there to land on.
_SHELL_ORPHAN_DROP_STOREYS_MAX = 2.0   # beyond this many storeys of real
                                       # fall, delete instead of dropping —
                                       # the task's own ">2 storeys" ceiling
_SHELL_ROLES_RUN = ("wall", "pier", "parapet")        # side + bay, in g.runs
_SHELL_ROLES_CORNER = ("corner", "parapet_corner")    # corner + storey only


def _shell_column_index(info, mass):
    """`(side, storey, bay) -> [(path, alive)]` and `(corner, storey) ->
    [(path, alive)]` for every wall/pier/corner/parapet/parapet_corner piece
    of `mass` EVER authored — dead ones included, so "removed" (authored,
    now `dead`) can be told apart from "never authored here at all" (an
    entrance gap, the ground storey's own street-level opening). Pure: reads
    `info["elements"]`'s own `_storey`/`_side`/`_bay` fields and each
    element's `dead` flag (set by `apply_plan`'s removal step, so this must
    be called AFTER `apply_plan` has run), no stage."""
    els = [e for e in (info.get("elements") or ()) if e.get("mass") == mass]
    n_sub = n_sub_of(els) or 1
    idx_runs, idx_corners = {}, {}
    for e in els:
        p = e.get("p") or {}
        role = p.get("_role")
        path = p.get("prim_path")
        if not path or role not in (_SHELL_ROLES_RUN + _SHELL_ROLES_CORNER):
            continue
        alive = not e.get("dead")
        sd = p.get("_side")
        st = int(p.get("_storey", e.get("storey", 0)))
        if role in _SHELL_ROLES_RUN and sd in SIDES:
            b = bay_no(p, n_sub)
            idx_runs.setdefault((sd, st, b), []).append((path, alive))
        elif role in _SHELL_ROLES_CORNER and sd in _CORNER_SIDES:
            idx_corners.setdefault((sd, st), []).append((path, alive))
    return idx_runs, idx_corners


def _orphaned_shell_candidates(info, plan, mass, loose=False):
    """Live shell paths whose directly-below, SAME-column cell was authored
    but is now entirely `dead` — `_apply_region`'s per-cell toothing draw,
    read back off the element table with no stage.

    `loose=False` (item 2's own gate): the below cell must have EXISTED and
    have EVERY one of its own pieces `dead`. `loose=True` (the safety net's
    broader gate, item 3): the below cell may also be PARTIALLY lost — some
    but not all of its pieces `dead` — since a surviving pier stub does not
    always reach out to cover this piece's own footprint; the stage-time
    `_deck_support_z` check that follows is what actually decides, this only
    widens who gets asked. Either way the candidate itself must be alive and
    not already claimed by a rigid `plan["displaced"]` transform (that piece
    already has its own carrier)."""
    displaced = set(plan.get("displaced") or {})
    idx_runs, idx_corners = _shell_column_index(info, mass)

    def _gone_enough(below):
        if below is None:
            return False
        dead = [not alive for _p, alive in below]
        return all(dead) if not loose else any(dead)

    out = []
    for (sd, st, b), items in idx_runs.items():
        alive_here = [p for p, alive in items if alive and p not in displaced]
        if alive_here and _gone_enough(idx_runs.get((sd, st - 1, b))):
            out.extend(alive_here)
    for (sd, st), items in idx_corners.items():
        alive_here = [p for p, alive in items if alive and p not in displaced]
        if alive_here and _gone_enough(idx_corners.get((sd, st - 1))):
            out.extend(alive_here)
    return out


def _typical_storey_h(m):
    lv = m.get("levels") or []
    if len(lv) > 1:
        return max(1.5, (float(lv[-1]) - float(lv[0])) / (len(lv) - 1))
    return max(1.5, float(m.get("top", 3.0)) - float(m.get("z0", 0.0)))


def _repair_stranded_shell_sliced(stage, ctx, plan):
    """Item 2: a surviving shell piece whose own support was toothed away
    below it (case 2 above) is dropped onto whatever REAL support
    `quake_collapse._deck_support_z` finds under its own footprint — plus a
    small settle tip, the same idle-tip idiom `_sweep_roof_props_sliced`
    already uses for a shallow reseat — when that drop is a storey or two;
    beyond `_SHELL_ORPHAN_DROP_STOREYS_MAX` storeys (or nothing real found
    below it at all) it is deactivated instead and left to the pile already
    authored for this building, the same call `_total_collapse`'s own
    wholesale upper-storey removal makes for a piece the mound already
    accounts for in aggregate — repositioning ONE big wall panel onto an
    uneven rubble mound reads worse than the mound alone. Call AFTER
    `apply_plan` (needs `e["dead"]`) and after `_sweep_roof_props_sliced`.
    Returns `(n_dropped, n_deleted)`."""
    from pxr import Usd, UsdGeom

    from . import quake_collapse as qc

    info = ctx["info"]
    by_path = {(e.get("p") or {}).get("prim_path"): e
              for e in (info.get("elements") or ())}
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    cache = None
    n_dropped = n_deleted = 0
    drops = []
    for mass, m in (info.get("masses") or {}).items():
        cand = _orphaned_shell_candidates(info, plan, mass, loose=False)
        if not cand:
            continue
        if cache is None:
            cache = qc._deck_support_candidates(stage, ctx["parent"])
        h_st = _typical_storey_h(m)
        for path in cand:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid() or not prim.IsActive():
                continue
            try:
                r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            except Exception:
                continue
            if r.IsEmpty():
                continue
            lo, hi = r.GetMin(), r.GetMax()
            cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
            half_w = max(0.3, (hi[0] - lo[0]) / 2.0)
            half_d = max(0.3, (hi[1] - lo[1]) / 2.0)
            base_z = float(lo[2])
            support = qc._deck_support_z(stage, ctx["parent"], cx, cy,
                                        half_w, half_d, base_z,
                                        exclude=(path,), candidates=cache)
            if support is None:
                support = float(m.get("z0", base_z))
            drop = base_z - support
            if drop <= 0.02:
                continue
            if drop > _SHELL_ORPHAN_DROP_STOREYS_MAX * h_st:
                if qf._deactivate(stage, path):
                    n_deleted += 1
                    e = by_path.get(path)
                    if e is not None:
                        e["dead"] = True
                continue
            qf._transform_prims(stage, [path],
                               qf._translate(0.0, 0.0, support - base_z))
            rng = ctx["rng"]
            ax = (rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), 0.0)
            if abs(ax[0]) + abs(ax[1]) < 1e-3:
                ax = (1.0, 0.0, 0.0)
            qf._transform_prims(stage, [path], qf._rot_about(
                (cx, cy, support), ax,
                rng.uniform(-qf.B_ROOF_PLANT_TIP_DEG, qf.B_ROOF_PLANT_TIP_DEG)))
            ctx["static_extra"].append(path)
            n_dropped += 1
            drops.append(drop)
    if n_dropped or n_deleted:
        ctx["notes"].append(
            "[qgac] stranded shell: {0} dropped to real support ({1:.2f}"
            "..{2:.2f} m), {3} deleted (>{4:.0f} storeys of fall, left to "
            "the pile)".format(n_dropped, min(drops) if drops else 0.0,
                              max(drops) if drops else 0.0, n_deleted,
                              _SHELL_ORPHAN_DROP_STOREYS_MAX))
    return n_dropped, n_deleted


def _sweep_airborne_shell_sliced(stage, ctx, plan, gap_m=1.0, verbose=True):
    """Item 3, THE SAFETY NET: this module's analog of `fire_bake.
    deactivate_airborne`, scoped to the shell (never the fit-out / roof
    plant, both already swept above). Call last, where `wreck_sliced`
    finishes.

    NOT a scan of every shell piece in the building — walls are not decks,
    so a plain wall-on-wall stack routinely has no real up-facing triangle
    for `_deck_support_z` to find under an ORDINARY, undamaged piece (that
    is exactly why `roof_and_parapet._ensure_roof` exists: a ring()'d piece
    can come back with none at all), and testing every piece against that
    idiom would flag intact buildings wholesale. The candidate pool stays
    the SAME grid heuristic `_repair_stranded_shell_sliced` uses (`loose=
    True` here: the below cell only has to have lost SOME of its own pieces,
    not all of them, since a partial toothed loss can still leave this
    piece's own footprint uncovered) — bounded cost, and this only ever
    fires on a piece a real removal actually put in question. Whatever
    `_repair_stranded_shell_sliced` already resolved is skipped (it is
    `dead` or has moved); anything still found unsupported beyond `gap_m` is
    deactivated, never repositioned — it already had its one chance."""
    from pxr import Usd, UsdGeom

    from . import quake_collapse as qc

    info = ctx["info"]
    by_path = {(e.get("p") or {}).get("prim_path"): e
              for e in (info.get("elements") or ())}
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    cache = None
    n = 0
    for mass, m in (info.get("masses") or {}).items():
        cand = _orphaned_shell_candidates(info, plan, mass, loose=True)
        if not cand:
            continue
        if cache is None:
            cache = qc._deck_support_candidates(stage, ctx["parent"])
        z0 = float(m.get("z0", 0.0))
        for path in cand:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid() or not prim.IsActive():
                continue
            try:
                r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            except Exception:
                continue
            if r.IsEmpty():
                continue
            lo, hi = r.GetMin(), r.GetMax()
            cx, cy = (lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0
            half_w = max(0.3, (hi[0] - lo[0]) / 2.0)
            half_d = max(0.3, (hi[1] - lo[1]) / 2.0)
            base_z = float(lo[2])
            if abs(base_z - z0) < 0.05:
                continue    # sitting at grade already — not airborne
            support = qc._deck_support_z(stage, ctx["parent"], cx, cy,
                                        half_w, half_d, base_z,
                                        exclude=(path,), candidates=cache)
            if support is not None and base_z - support <= gap_m:
                continue
            if qf._deactivate(stage, path):
                n += 1
                e = by_path.get(path)
                if e is not None:
                    e["dead"] = True
    if n:
        ctx["notes"].append("[qgac] airborne shell sweep: {0} deactivated"
                            .format(n))
    if verbose:
        print("[qgac] airborne shell sweep: {0} deactivated".format(n))
    return n


# ---------------------------------------------------------------------------
# ENTRY POINT
# ---------------------------------------------------------------------------
def wreck_sliced(stage, cell, placements, style, recipes, rng, nrng, mats, tag,
                 btype=None, usd=None, mat_cache=None, fit_storeys=None,
                 x=0.0, y=0.0, yaw=0.0, material=None, plate_ok=None,
                 verbose=True):
    """Apply `recipes` (a grade string or [(name, kwargs)]) to one SLICED
    building, and return `quake_flow.wreck_building`'s ctx, unchanged in shape.

        pls, grid, meas = gss.slice_to_kit(stage, src, cell, style)   # or kit_bake.load_kit
        ctx = quake_sliced.wreck_sliced(stage, cell, pls, style, "DG4",
                                        rng, nrng, mats, tag, usd=asset_usd)

    `x` / `y` / `yaw` are the CELL's, and default to zero because the slicer
    writes its pieces in the cell's own frame — exactly as `gac_fire.burn_gac`
    calls `burn_building(..., 0.0, 0.0, 0.0, ...)`.

    The construction type is ROUTED FROM DATA (`construction_type`) and
    overrides `describe`'s `info["type"]`, which for a sliced style is always
    `urm`: `gac_slice.register_style` has to declare a family and it declares
    "01". That single override is why a concrete GAC tower stops being given
    the masonry ladder.
    """
    info = qf.describe(style, placements, x, y, yaw)
    btype = btype or construction_type(usd or style, H=info.get("H"),
                                       material=material)
    info["type"] = btype
    ctx = {"stage": stage, "parent": cell, "info": info, "rng": rng,
           "nrng": nrng, "mats": mats,
           "cache": mat_cache if mat_cache is not None else {},
           "tag": tag, "loose": [], "static_extra": [], "velocity": {},
           "authored": [], "notes": [],
           "fit": {"slabs": {}, "columns": {}, "partitions": [], "props": {},
                   "all": []}}
    if plate_ok is not None:
        ctx["plate_ok"] = plate_ok
    ctx["sliced"] = {"style": style, "usd": usd, "btype": btype,
                     "n_pieces": len(placements)}

    recs, guard_notes = _resolve(recipes, btype, info)
    if not recs:
        qf.dress_roof(ctx)
        _reseat_roof_plant(stage, ctx)
        ctx["static_extra"] += list(ctx.get("roof_plant", []))
        ctx["notes"] += guard_notes
        ctx["plan"] = None
        return ctx

    # Foundation-only motion has the same fit-out requirement here as it
    # does for a native kit.  In particular, TILT needs only the locally
    # distressed first storey; fabricating an otherwise invisible frame on
    # every floor turns any later opening into the same pristine slab/column
    # grid the review found on block_residential.  Keep all-storey fit-out for
    # actual collapse recipes, where the newly exposed interior is needed.
    fit_scope = qf._fit_scope_for_recipes(recs, fit_storeys)
    ctx["fit"] = qf.fit_interior(
        stage, cell, info, mats, rng, storeys=fit_scope, tag=tag,
        col_roof_shorten=qf.COL_ROOF_SHORTEN_M)
    if fit_scope is not None:
        ctx["notes"].append(
            "foundation fit-out limited to storeys {0}; intact upper shell "
            "kept without synthetic exposed frame".format(
                ",".join(str(q) for q in fit_scope) or "none"))
    qf.dress_roof(ctx)
    _reseat_roof_plant(stage, ctx)
    ctx["fit"]["all"] += list(ctx.get("roof_plant", []))

    # The ORIGINAL `recipes` goes in, so the plan records the grade it was
    # asked for; `_resolve` is idempotent and consumes no rng, so running it
    # twice (once above for the emptiness test) changes nothing.
    plan = plan_damage(info, info["elements"], recipes, btype, rng)
    ctx["plan"] = plan
    apply_plan(stage, ctx, plan, verbose=verbose)

    # ROUND 6: roof plant carried with a storey crush, or re-seated on real
    # support if a region loss reached the roof — BEFORE the generic settle
    # pass below, which must never re-process what this already resolved
    # (see the section note above `_sweep_roof_props_sliced`).
    _sweep_roof_props_sliced(stage, ctx, plan)

    # STRANDED SHELL BANDS (round 8, `stranded-bands`): a region removal's own
    # toothing can keep a boundary pier two rows up from another one it lost
    # in the same draw — see the section note above `_repair_stranded_shell_
    # sliced`. Runs off `e["dead"]`, so AFTER `apply_plan`, same slot as the
    # roof-plant sweep above (before settle touches anything further).
    _repair_stranded_shell_sliced(stage, ctx, plan)

    # tanks / AC units follow their roof — and are BURIED on a total collapse,
    # which `_b_settle_roof_plant` decides from the recipe NAMES, so the two
    # modules have to spell `pancake` / `masonry_collapse` the same way.
    qf._b_settle_roof_plant(ctx, [(n, kw) for n, kw in plan["recipes"]])

    # Same pre-PhysX invariant as the native kit path.  This only recognizes
    # qf-authored fit-out boxes by name/topology; sliced UV shell pieces are
    # explicitly outside its positive list.
    from . import quake_collapse as qc
    qc.normalize_detached_rectangles(ctx)

    # everything still standing is static for the settle — `wreck_building`'s
    # own bookkeeping, byte for byte, so the settle and the bake see the same
    # shape they see for a kit building.
    ctx["static_extra"] += [e["p"].get("prim_path") for e in qf._els(ctx)
                            if e["p"].get("prim_path")]
    ctx["static_extra"] += [p for p in ctx["fit"]["all"]
                            if p not in ctx["loose"]]
    loose = set(ctx["loose"])
    seen, st = set(), []
    for p in ctx["static_extra"]:
        if p in loose or p in seen:
            continue
        seen.add(p)
        st.append(p)
    ctx["static_extra"] = st
    ctx["sliced"].update(plan["stats"])

    # THE SAFETY NET, WHERE THIS FUNCTION FINISHES: `_repair_stranded_shell_
    # sliced` targets the one traced mechanism; this catches anything else —
    # a different recipe combination, a future recipe — that leaves a shell
    # piece hanging with no real support (`fire_bake.deactivate_airborne`'s
    # analog for this ladder).
    _sweep_airborne_shell_sliced(stage, ctx, plan, verbose=verbose)

    if verbose:
        print("[quake_sliced] {0} ({1}, {2:.0f} m, {3} piece(s)): {4}".format(
            style, btype, info.get("H") or 0.0, len(placements),
            "; ".join(plan["notes"]) or "nothing"))
    return ctx


def level_for_intensity(i, btype, rng, jitter=0.05, duration_boost=1.0):
    """The grade draw is `quake_flow`'s — the vulnerability classes do not
    change because the building was sliced rather than assembled."""
    return qf.level_for_intensity(i, btype, rng, jitter=jitter,
                                  duration_boost=duration_boost)


def check(verbose=True):
    """Host-side: every ladder name is a real recipe, every type has every
    grade, every construction row is a known type."""
    bad = []
    for t, lad in LADDER_S.items():
        for lvl in LEVELS:
            if lvl not in lad:
                bad.append("{0}: no level {1}".format(t, lvl))
        for lvl, recs in lad.items():
            for name, _kw in recs:
                if name not in RECIPES_S:
                    bad.append("{0}/{1}: unknown recipe {2}".format(t, lvl, name))
                elif not callable(RECIPES_S[name]):
                    bad.append("{0}: not callable".format(name))
    for k, v in CONSTRUCTION.items():
        if v not in LADDER_S:
            bad.append("CONSTRUCTION[{0}] = {1}: no such type".format(k, v))
    if "pancake" in [n for n, _ in LADDER_S["rc_glass"]["DG5"]]:
        bad.append("rc_glass DG5 pancakes a tower")
    if verbose:
        print("[quake_sliced] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
