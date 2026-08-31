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
        "storey_collapse": None, "notes": list(guard_notes), "stats": {},
        "_removed_set": set(),
    }
    pctx = _pctx(info, elements, btype, rng, plan)
    for name, kw in recs:
        RECIPES_S[name](pctx, **(kw or {}))
    return _finalise(pctx, plan)


def _finalise(pctx, plan):
    """A piece can be lost, moved or laid on the pile — never two of them."""
    moved = set(plan["displaced"]) | set(p for p, _s in plan["panels"])
    plan["removed"] = [p for p in plan["removed"] if p not in moved]
    plan.pop("_removed_set", None)
    g = pctx["g"]
    plan["stats"] = {
        "n_pieces": len(g.els), "n_sub": g.n_sub, "n_storeys": len(g.storeys),
        "n_removed": len(plan["removed"]), "n_displaced": len(plan["displaced"]),
        "n_panels": len(plan["panels"]), "n_piles": len(plan["piles"]),
        "n_glass": len(plan["glass"]), "n_macroblocks": len(plan["macroblocks"]),
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


def _apply_fit_ops(stage, ctx, plan):
    fit = ctx.get("fit") or {}
    dead = set()
    for op in plan.get("fit_ops") or []:
        kind = op.get("op")
        mt = op.get("mass", "main")
        if kind == "displace_above":
            k = int(op.get("storey", 0))
            M = _gf(op["transform"])
            paths = []
            for (m_, i), pth in (fit.get("slabs") or {}).items():
                if m_ == mt and i > k and pth:
                    paths.append(pth)
            for (m_, i), cols in (fit.get("columns") or {}).items():
                if m_ == mt and i > k:
                    paths += list(cols)
            for (m_, i), props in (fit.get("props") or {}).items():
                if m_ == mt and i > k:
                    paths += list(props)
            paths += [q for q in (fit.get("partitions") or [])
                      if _fit_storey_of(q) > k]
            paths = [q for q in dict.fromkeys(paths) if q and q not in dead]
            if paths:
                qf._transform_prims(stage, paths, M)
                ctx["static_extra"] += paths
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
           "piles": n_pile}
    if verbose:
        print("[quake_sliced] {0}: {1} piece(s) removed, {2} displaced, "
              "{3} glass subset(s) voided, {4} pile(s)".format(
                  plan.get("grade") or "recipes", n_rm, n_mv, n_glass, n_pile))
    return out


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
        ctx["static_extra"] += list(ctx.get("roof_plant", []))
        ctx["notes"] += guard_notes
        ctx["plan"] = None
        return ctx

    ctx["fit"] = qf.fit_interior(stage, cell, info, mats, rng,
                                 storeys=fit_storeys, tag=tag)
    qf.dress_roof(ctx)
    ctx["fit"]["all"] += list(ctx.get("roof_plant", []))

    # The ORIGINAL `recipes` goes in, so the plan records the grade it was
    # asked for; `_resolve` is idempotent and consumes no rng, so running it
    # twice (once above for the emptiness test) changes nothing.
    plan = plan_damage(info, info["elements"], recipes, btype, rng)
    ctx["plan"] = plan
    apply_plan(stage, ctx, plan, verbose=verbose)

    # tanks / AC units follow their roof — and are BURIED on a total collapse,
    # which `_b_settle_roof_plant` decides from the recipe NAMES, so the two
    # modules have to spell `pancake` / `masonry_collapse` the same way.
    qf._b_settle_roof_plant(ctx, [(n, kw) for n, kw in plan["recipes"]])

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
