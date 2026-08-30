"""quake_rubble.py — the rubble PLANNER: a mound heightfield plus the large
elements and instance scatters that sit in it.

Round-3's `quake_flow._heap` authored a pile as 1,500-5,600 identical `_a_lump`
boxes — every object in the heap had the same character, so from 40 m it read
as a heap of toy blocks (`_plans/earthquake_round4_plan.md` "Diagnosis").
Round 4's design is a SURFACE with things in it: one heightfield mesh (the
mound), a handful of large recognisable elements sitting on it (slab rafts,
column stubs, rebar tangles, lintels/quoins, joists, wall panels), a few
textured FAB debris-pile clusters sunk into the flanks, and a scatter of
small chunks/flakes with density falling from crown to toe.

This module is PURE numpy + stdlib — no `pxr` import anywhere, so it can be
unit-tested and iterated on without Kit/Isaac. `disaster/quake_rubble_usd.py`
(another agent's file) is the only module that turns a plan into USD prims.

CONVENTIONS (mirrored from `quake_flow`, kept in sync by hand since this
module must stay pxr-free and therefore cannot import quake_flow — see
`quake_flow.describe`, `_to_local`, `_to_world`, `_side_of`, `_SIDE_NORMAL`,
`_outward`, lines ~279-345 and the mass dict built by `_mass_specs`):

    m = {"cx", "cy", "W", "D", "yaw", "z0", "top", "levels", ...}

Local frame: x runs along W, y runs along D, front = -y = side "S" (matches
`detail/urban_building`). World = rotate the local frame by `m["yaw"]`
degrees about +Z, then translate by (`m["cx"]`, `m["cy"]`); z is unaffected
by yaw (z-up throughout, and every catalogue asset is Z-up with its footprint
centred on the origin in x/y and its base at z=0).

RESEARCH NUMBERS: `_plans/earthquake_research.md` (rubble composition,
fragment-size rules) and `_plans/eq_round4_rubble_research.md` (WP A —
mound-shape morphology: crown/height, per-side run-out, repose, the standing
stub, the windrow/fan taper). Every constant below cites which memo section
it came from; where the two disagree the round-4 memo (dated later, agent A's
dedicated measurement pass) wins.
"""

import math

import numpy as np

# ---------------------------------------------------------------------------
# Mass-frame helpers (mirrors of `quake_flow`'s — see module docstring)
# ---------------------------------------------------------------------------

_SIDE_NORMAL = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}
_SIDES = ("S", "N", "E", "W")


def _to_local(m, wx, wy):
    a = math.radians(-m["yaw"])
    dx, dy = wx - m["cx"], wy - m["cy"]
    return (dx * math.cos(a) - dy * math.sin(a),
            dx * math.sin(a) + dy * math.cos(a))


def _to_world(m, lx, ly):
    a = math.radians(m["yaw"])
    return (m["cx"] + lx * math.cos(a) - ly * math.sin(a),
            m["cy"] + lx * math.sin(a) + ly * math.cos(a))


def _outward(m, side):
    nx, ny = _SIDE_NORMAL[side]
    a = math.radians(m["yaw"])
    return (nx * math.cos(a) - ny * math.sin(a),
            nx * math.sin(a) + ny * math.cos(a))


def _side_length(m, side):
    return m["W"] if side in ("S", "N") else m["D"]


def in_reach(m, x, y, reach_m):
    """Is (x, y) within `reach_m` of the mass's own footprint (world coords)?"""
    lx, ly = _to_local(m, x, y)
    halfW, halfD = m["W"] / 2.0, m["D"] / 2.0
    dx = max(0.0, abs(lx) - halfW)
    dy = max(0.0, abs(ly) - halfD)
    return math.hypot(dx, dy) <= reach_m


# ---------------------------------------------------------------------------
# CATALOGUE — every measured debris asset. `url` is relative to the assets
# root (`RUBBLE_ASSET_ROOT` in quake_rubble_usd.py). `size` = (sx, sy, sz) m,
# native (unscaled) bounding box, matching the measured .usdc directly: every
# asset is metres, Z-up, footprint centred on the origin in x/y, base at
# z = 0. `_hp` twins are flagged so nobody instances the 2-3x tri-count
# high-poly duplicate by mistake.
# ---------------------------------------------------------------------------

CATALOGUE = {
    # --- standalone/debris/pieces — untextured, flat-shaded, cheap -----
    "slab_01": {"url": "standalone/debris/pieces/slab_01/slab_01.usdc", "size": (4.189, 3.030, 0.373), "tris": 5643, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_02": {"url": "standalone/debris/pieces/slab_02/slab_02.usdc", "size": (4.518, 2.735, 0.353), "tris": 5047, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_03": {"url": "standalone/debris/pieces/slab_03/slab_03.usdc", "size": (2.511, 4.483, 0.307), "tris": 7106, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_04": {"url": "standalone/debris/pieces/slab_04/slab_04.usdc", "size": (2.504, 4.436, 0.418), "tris": 5253, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_05": {"url": "standalone/debris/pieces/slab_05/slab_05.usdc", "size": (3.636, 3.835, 0.521), "tris": 9374, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_06": {"url": "standalone/debris/pieces/slab_06/slab_06.usdc", "size": (3.423, 4.512, 0.500), "tris": 9586, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_07": {"url": "standalone/debris/pieces/slab_07/slab_07.usdc", "size": (2.301, 1.603, 0.190), "tris": 568, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_08": {"url": "standalone/debris/pieces/slab_08/slab_08.usdc", "size": (2.379, 3.531, 0.454), "tris": 1084, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_09": {"url": "standalone/debris/pieces/slab_09/slab_09.usdc", "size": (2.480, 2.497, 0.989), "tris": 1088, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_10": {"url": "standalone/debris/pieces/slab_10/slab_10.usdc", "size": (3.748, 3.549, 0.587), "tris": 1628, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_11": {"url": "standalone/debris/pieces/slab_11/slab_11.usdc", "size": (2.420, 4.104, 1.205), "tris": 1258, "kind": "raft", "textured": False, "material": "concrete"},
    "slab_12": {"url": "standalone/debris/pieces/slab_12/slab_12.usdc", "size": (2.161, 1.313, 0.675), "tris": 1088, "kind": "raft", "textured": False, "material": "concrete"},

    "chunk_01": {"url": "standalone/debris/pieces/chunk_01/chunk_01.usdc", "size": (0.971, 0.985, 0.786), "tris": 74, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_02": {"url": "standalone/debris/pieces/chunk_02/chunk_02.usdc", "size": (0.881, 1.004, 0.813), "tris": 76, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_03": {"url": "standalone/debris/pieces/chunk_03/chunk_03.usdc", "size": (0.971, 0.984, 0.786), "tris": 74, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_04": {"url": "standalone/debris/pieces/chunk_04/chunk_04.usdc", "size": (0.905, 1.004, 0.813), "tris": 76, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_05": {"url": "standalone/debris/pieces/chunk_05/chunk_05.usdc", "size": (0.925, 0.735, 0.892), "tris": 74, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_06": {"url": "standalone/debris/pieces/chunk_06/chunk_06.usdc", "size": (0.982, 0.751, 0.849), "tris": 76, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_07": {"url": "standalone/debris/pieces/chunk_07/chunk_07.usdc", "size": (0.656, 0.965, 0.830), "tris": 76, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_08": {"url": "standalone/debris/pieces/chunk_08/chunk_08.usdc", "size": (0.830, 0.685, 0.892), "tris": 74, "kind": "chunk", "textured": False, "material": "concrete"},
    "chunk_09": {"url": "standalone/debris/pieces/chunk_09/chunk_09.usdc", "size": (0.945, 0.779, 0.862), "tris": 76, "kind": "chunk", "textured": False, "material": "concrete"},

    "lump_01": {"url": "standalone/debris/pieces/lump_01/lump_01.usdc", "size": (0.361, 0.370, 0.062), "tris": 3952, "kind": "flake", "textured": False, "material": "concrete"},
    "lump_02": {"url": "standalone/debris/pieces/lump_02/lump_02.usdc", "size": (0.223, 0.240, 0.055), "tris": 3768, "kind": "flake", "textured": False, "material": "concrete"},
    "lump_03": {"url": "standalone/debris/pieces/lump_03/lump_03.usdc", "size": (0.195, 0.192, 0.043), "tris": 4046, "kind": "flake", "textured": False, "material": "concrete"},
    "lump_04": {"url": "standalone/debris/pieces/lump_04/lump_04.usdc", "size": (0.199, 0.196, 0.066), "tris": 4088, "kind": "flake", "textured": False, "material": "concrete"},
    "lump_05": {"url": "standalone/debris/pieces/lump_05/lump_05.usdc", "size": (0.240, 0.232, 0.063), "tris": 3898, "kind": "flake", "textured": False, "material": "concrete"},
    "lump_06": {"url": "standalone/debris/pieces/lump_06/lump_06.usdc", "size": (0.156, 0.146, 0.045), "tris": 522, "kind": "flake", "textured": False, "material": "concrete"},

    "rebar_01": {"url": "standalone/debris/pieces/rebar_01/rebar_01.usdc", "size": (4.370, 1.215, 0.227), "tris": 3405, "kind": "rebar", "textured": False, "material": "steel"},
    "rebar_02": {"url": "standalone/debris/pieces/rebar_02/rebar_02.usdc", "size": (4.412, 1.449, 0.212), "tris": 3438, "kind": "rebar", "textured": False, "material": "steel"},
    "rebar_03": {"url": "standalone/debris/pieces/rebar_03/rebar_03.usdc", "size": (4.131, 2.541, 0.188), "tris": 3754, "kind": "rebar", "textured": False, "material": "steel"},
    "rebar_04": {"url": "standalone/debris/pieces/rebar_04/rebar_04.usdc", "size": (3.953, 2.753, 0.162), "tris": 2494, "kind": "rebar", "textured": False, "material": "steel"},

    "sheet_01": {"url": "standalone/debris/pieces/sheet_01/sheet_01.usdc", "size": (4.364, 2.933, 0.461), "tris": 8351, "kind": "sheet", "textured": False, "material": "steel"},
    "sheet_02": {"url": "standalone/debris/pieces/sheet_02/sheet_02.usdc", "size": (4.522, 2.851, 0.559), "tris": 9209, "kind": "sheet", "textured": False, "material": "steel"},
    "sheet_03": {"url": "standalone/debris/pieces/sheet_03/sheet_03.usdc", "size": (4.009, 1.844, 0.293), "tris": 3181, "kind": "sheet", "textured": False, "material": "steel"},

    # --- concrete_rubble_debris/split — textured FAB spreads -----------
    "brick_debris_pile": {"url": "concrete_rubble_debris/split/brick_debris_pile/brick_debris_pile.usdc", "size": (6.071, 4.897, 1.196), "tris": 58000, "kind": "spread", "textured": True, "material": "brick"},
    "brick_debris_pile_hp": {"url": "concrete_rubble_debris/split/brick_debris_pile_hp/brick_debris_pile_hp.usdc", "size": (6.071, 4.930, 1.200), "tris": 189076, "kind": "spread", "textured": True, "material": "brick", "hp": True},
    "concrete_debris_elements": {"url": "concrete_rubble_debris/split/concrete_debris_elements/concrete_debris_elements.usdc", "size": (3.530, 2.579, 0.379), "tris": 96826, "kind": "spread", "textured": True, "material": "concrete"},
    "concrete_debris_elements_hp": {"url": "concrete_rubble_debris/split/concrete_debris_elements_hp/concrete_debris_elements_hp.usdc", "size": (3.537, 2.580, 0.381), "tris": 124705, "kind": "spread", "textured": True, "material": "concrete", "hp": True},
    "concrete_sidewalk_elements": {"url": "concrete_rubble_debris/split/concrete_sidewalk_elements/concrete_sidewalk_elements.usdc", "size": (1.723, 1.767, 0.338), "tris": 29410, "kind": "street", "textured": True, "material": "concrete"},
    "concrete_slabs": {"url": "concrete_rubble_debris/split/concrete_slabs/concrete_slabs.usdc", "size": (3.639, 2.459, 0.669), "tris": 58176, "kind": "spread", "textured": True, "material": "concrete"},
    "cracked_paving_slabs": {"url": "concrete_rubble_debris/split/cracked_paving_slabs/cracked_paving_slabs.usdc", "size": (1.081, 1.601, 0.314), "tris": 27613, "kind": "street", "textured": True, "material": "concrete"},
    "huge_concrete_rubble_pile": {"url": "concrete_rubble_debris/split/huge_concrete_rubble_pile/huge_concrete_rubble_pile.usdc", "size": (8.046, 7.680, 1.476), "tris": 71293, "kind": "spread", "textured": True, "material": "concrete"},
    "lamppost_block": {"url": "concrete_rubble_debris/split/lamppost_block/lamppost_block.usdc", "size": (0.162, 0.349, 0.739), "tris": 20538, "kind": "street", "textured": True, "material": "concrete"},
    "lamppost_block_v2": {"url": "concrete_rubble_debris/split/lamppost_block_v2/lamppost_block_v2.usdc", "size": (0.162, 0.349, 0.739), "tris": 20538, "kind": "street", "textured": True, "material": "concrete"},

    # --- standalone/debris/piles — Quixel, textured ---------------------
    "concrete_rubble_pile": {"url": "standalone/debris/piles/concrete_rubble_pile/concrete_rubble_pile.usdc", "size": (2.670, 2.227, 0.234), "tris": 725, "kind": "spread", "textured": True, "material": "concrete"},
    "rocky_ground": {"url": "standalone/debris/piles/rocky_ground/rocky_ground.usdc", "size": (3.671, 3.609, 0.514), "tris": 7451, "kind": "spread", "textured": True, "material": "earth"},
}

_RAFTS = ["slab_{0:02d}".format(i) for i in range(1, 13)]
_CHUNKS = ["chunk_{0:02d}".format(i) for i in range(1, 10)]
_FLAKES = ["lump_{0:02d}".format(i) for i in range(1, 7)]
_REBARS = ["rebar_{0:02d}".format(i) for i in range(1, 5)]
_SHEETS = ["sheet_{0:02d}".format(i) for i in range(1, 4)]
_TOE = ["concrete_sidewalk_elements", "cracked_paving_slabs"]

# PROTO_SETS: asset POOLS per construction type. "chunk"/"flake"/"cluster"/
# "toe" are drawn from repeatedly (they feed PointInstancers); "raft"/
# "rebar"/"sheet" are drawn once per large-element slot (they feed
# individually-authored `large` references, never instanced — a 2-6 m raft
# on the crown has to be counted and sized by `plan_pile`, not left to a
# per-instance draw).
PROTO_SETS = {
    "urm": {
        "chunk": list(_CHUNKS),
        "flake": list(_FLAKES),
        "cluster": ["brick_debris_pile"],
        "toe": list(_TOE),
        "raft": [],            # URM has timber floors: no concrete rafts
        "rebar": [],
        "sheet": [],
    },
    "rc": {
        "chunk": list(_CHUNKS),
        "flake": list(_FLAKES),
        "cluster": ["concrete_debris_elements", "concrete_slabs",
                    "huge_concrete_rubble_pile", "concrete_rubble_pile"],
        "toe": list(_TOE),
        "raft": list(_RAFTS),
        "rebar": list(_REBARS),
        "sheet": list(_SHEETS),          # rare — see SHEET_PROB
    },
}
PROTO_SETS["rc_glass"] = {k: list(v) for k, v in PROTO_SETS["rc"].items()}


# ---------------------------------------------------------------------------
# Constants. Every one cites the memo section it comes from; `eq_round4_
# rubble_research.md` (WP A) numbers override the original round-4-plan
# brief where the two disagree (dated later, dedicated measurement pass).
# ---------------------------------------------------------------------------

REPOSE_DEG = 35.0                 # rubble_research.md sec1c — flanks, mixed demolition rubble w/ fines
# The crown-height design target AND the post-noise relaxation must use the
# SAME baseline slope, a few degrees under REPOSE_DEG, not REPOSE_DEG itself:
# if the crown is sized to put the whole base ramp exactly AT repose, the
# relaxation (which only redistributes LOCAL excess, and cannot "shorten" a
# uniformly-at-cap ramp within a fixed run) has no headroom left to absorb
# even a small amount of noise (measured: every draw came out 40-42 deg).
REPOSE_DESIGN_DEG = REPOSE_DEG - 3.0

# Anti-z-fight lip (round-4 review): a mound/apron vertex at EXACTLY z0 is
# coplanar with the ground plate and flickers on render. Same fix as
# `quake_flow._c_dish`'s -0.01/-0.02 ring offsets, just upward: every mound
# outer-ring vertex sits at z0 + MOUND_LIP_M, every apron vertex (its rim
# included) at >= z0 + APRON_LIP_M — never exactly z0.
MOUND_LIP_M = 0.008
APRON_LIP_M = 0.012
APRON_REPOSE_DEG = 31.0           # rubble_research.md sec1c — pure-fines toe apron sits lower, 30-32 deg

CROWN_FRAC = {"urm": 0.28, "rc": 0.30, "rc_glass": 0.12}   # rubble_research.md "Constants" table sec1a
CROWN_CAP_M = 12.0                # rubble_research.md sec1a — low-confidence engineering ceiling, no data above ~11 m

# --- run-out (per side, height-dependent) --- rubble_research.md sec1b -----
# Moya et al. 2020, 851 LiDAR-measured collapsed buildings: street/fall-side
# run-out fraction of H is a piecewise-linear curve in H; the blind/party-
# wall side is flat and much smaller (P[D=0] falls fast with H). URM gets an
# 0.85x correction because the Moya fit is a wood-frame (more-complete-
# toppling) upper bound.
_RUNOUT_FALL_ANCHORS = ((4.0, 0.65), (12.0, 0.40), (20.0, 0.35))
RUNOUT_BLIND_FRAC = 0.10          # midpoint of the measured 0.05-0.15 blind/party-wall range
RUNOUT_FLOOR_M = 1.5              # rubble_research.md sec1b — floor on any run-out
# Moya's fit is wood-frame, 1-4 storeys; extrapolating 0.35xH straight up to a
# 55 m tower over-spills (Kahramanmaras pancakes spilled 5-10 m into the
# street) — coordinator round-3 review. Absolute ceiling, applied on top of
# the fraction-of-H formula.
RUNOUT_CAP_FALL_M = 10.0
RUNOUT_CAP_BLIND_M = 3.0
URM_RUNOUT_MULT = 0.85            # rubble_research.md sec1b — Moya fit is a wood-frame upper bound


def runout_frac(H, btype, fall=True):
    """Run-out beyond the wall line as a fraction of building height `H`.

    `fall=True` -> the street/fall side (a piecewise-linear fit to 851
    LiDAR-measured collapsed buildings, Moya et al. 2020: 0.65 at H<=4m,
    0.40 at H=12m, 0.35 at H>=20m, linear between anchors).
    `fall=False` -> a blind/party-wall side (flat 0.10, i.e. the midpoint of
    the measured 0.05-0.15 range — P[D=0] on that side falls with height but
    no per-height curve was fit for its magnitude when D>0).
    URM gets the 0.85x upper-bound correction either way.
    See `_plans/eq_round4_rubble_research.md` sec1b. Exposed publicly so
    `disaster/quake.py` (engulfment / clearance) uses the same law.
    """
    if not fall:
        f = RUNOUT_BLIND_FRAC
    else:
        Hc = max(0.0, float(H))
        pts = _RUNOUT_FALL_ANCHORS
        if Hc <= pts[0][0]:
            f = pts[0][1]
        elif Hc >= pts[-1][0]:
            f = pts[-1][1]
        else:
            f = pts[-1][1]
            for (h0, f0), (h1, f1) in zip(pts, pts[1:]):
                if h0 <= Hc <= h1:
                    f = f0 + (f1 - f0) * (Hc - h0) / (h1 - h0)
                    break
    if btype == "urm":
        f *= URM_RUNOUT_MULT
    return f


RAFT_SCALE = {"rc": 1.0, "rc_glass": 0.6}          # added: rc_glass -> "fewer rafts" (PROTO_SETS note)
COLUMN_STUBS = (2, 5)              # round4 plan design table sec3a — rc/rc_glass only
REBAR_N = (2, 4)                   # round4 plan design table sec3a — rc/rc_glass only
LINTEL_N = (3, 6)                  # round4 plan design table sec3a — urm only
JOIST_N = (6, 14)                  # round4 plan design table sec3a — urm only
SHEET_PROB = {"urm": 0.0, "rc": 0.15, "rc_glass": 0.35}   # added: "sheets rare" / "more likely"

CHUNK_DENSITY = {"crown": 2.5, "mid": 1.2, "toe": 0.35, "cap": 4000}  # round-4 review: was 1.2/0.3/600 -- read as a dune
FLAKE_N = (300, 600)               # round-4 review: was 100-300
CLUSTER_N = (3, 10)                # round4 plan design table (layer 3)
RUNOUT_CHUNK_FRAC = (0.08, 0.15)   # round4 brief — share of chunks landing beyond the toe
RUNOUT_CHUNK_REACH_MULT = 1.4      # round4 brief — up to 1.4x reach

BURY = {
    "raft": (0.10, 0.30),          # rubble_research.md sec3b / "Constants" table (was 0.15-0.45)
    "chunk": (0.30, 0.60),         # rubble_research.md sec3b
    "cluster": (0.25, 0.50),       # round4 brief (not separately measured — kept)
    "flake": (0.0, 0.30),          # round4 brief
    "panel": (0.20, 0.50),         # round4 brief
    # extra categories needed for authored/asset "large" elements the brief's
    # BURY dict didn't cover (added, not part of the original 5-key table):
    "column": (0.35, 0.55),
    "lintel": (0.10, 0.40),
    "joist": (0.20, 0.50),
    "rebar": (0.50, 0.80),         # rubble_research.md sec3b — "rebar tangles / small debris"
}

PANEL_LEAN_DEG = (30.0, 70.0)      # rubble_research.md sec3b (was 55-80)
PANEL_TILT_DEG = (10.0, 35.0)      # round4 brief — general (non-leaning) panel tilt
RAFT_TILT_DEG = (0.0, 25.0)        # round4 brief
RAFT_CROWN_TILT_DEG = (5.0, 25.0)  # round4 brief — crown-group subset
CHUNK_SCALE = (0.35, 1.0)          # round4 brief
CLUSTER_SCALE = (0.8, 1.3)         # round4 brief
RAFT_SCALE_RANGE = (0.9, 1.1)      # round4 brief
FLAKE_SCALE = (0.7, 1.3)           # added — not given explicitly

# Authored-box size ranges (mirror `quake_flow._p_lintels`'s two big
# categories; the third ("arch head / coping stone") is dropped rather than
# inventing an un-cited size range).
LINTEL_LONG_SIZE = ((1.1, 2.2), (0.20, 0.32), (0.18, 0.30))
LINTEL_QUOIN_SIZE = ((0.36, 0.58), (0.28, 0.42), (0.24, 0.38))
LINTEL_LONG_SHARE = 0.6
JOIST_SIZE = ((2.5, 4.0), (0.08, 0.12), (0.16, 0.24))
COLUMN_SIZE_XY = 0.45
COLUMN_SIZE_Z = (1.0, 3.0)
COLUMN_TILT_FROM_VERTICAL = (60.0, 90.0)   # round4 plan design table sec3a

# Windrow/fan (out-of-plane wall / parapet / gable) — rubble_research.md sec5b
WINDROW_DEPTH_PARAPET = (0.30, 0.60)
WINDROW_DEPTH_WALL = (0.50, 1.00)
WINDROW_PARAPET_ELEM_H_MAX = 1.6      # heuristic threshold distinguishing the two above
WINDROW_REACH_FRAC = 1.0              # rubble_research.md sec5b — reach ~= failed element's own height
WINDROW_REACH_JITTER = (0.85, 1.15)
FAN_WIDEN = (1.3, 1.6)                 # rubble_research.md sec5b / round4 brief — toe span x wall run

_GRID_TARGET_CELLS_ACROSS = 40.0
_GRID_CELL_RANGE = (0.6, 0.9)

# --- round-2 review (Blender contact-sheet: "smooth dune with sprinkled
# pebbles") -----------------------------------------------------------------
GRID_CELL_M = 0.5          # was ~0.6-0.9 target-cells-across; 1-2 m lumps need this
GRID_TRI_CAP = 12000       # per mesh (mound, each apron piece); coarsens cell if exceeded

# fbm octaves 1-3 (the original crown-shaped waves) keep their wavelengths but
# HALF their amplitude fraction — they were reading as long gentle waves.
NOISE_AMP_SCALE_123 = 0.5
# octaves 4-5: real 1-2 m surface lumps, in ABSOLUTE metres (not a fraction of
# footprint), masked to 0 at the toe / where the pile is under ~0.6 m tall so
# it still feathers into the ground.
RELIEF_WAVELENGTH_4 = (1.2, 2.0)
RELIEF_AMP_4 = (0.25, 0.40)
RELIEF_WAVELENGTH_5 = (0.5, 0.7)
RELIEF_AMP_5 = (0.08, 0.12)
RELIEF_MASK_HEIGHT_M = 0.6

# "shoulders": every raft/panel/column/cluster raises the mound locally so it
# looks embedded rather than resting on top of a dune.
SHOULDER_BUMP_HEIGHT_FRAC = 0.25     # x the element's own rotated thickness
SHOULDER_BUMP_RADIUS_FRAC = 0.6      # x the element's footprint diagonal
SHOULDER_BUMP_MAX_M = 0.15           # absolute ceiling: a "shoulder", not a second hill

# scatter: patchy density (drifts, not an even sprinkle) and a mix of chunks
# lying flat vs sticking out.
CHUNK_PATCH_WAVELENGTH_FRAC = 0.25   # x max(W, D)
CHUNK_PATCH_CONTRAST = (0.3, 1.7)
CHUNK_STICKOUT_PROB = (0.20, 0.30)
CHUNK_FLAT_TILT_DEG = (0.0, 20.0)
CHUNK_STICKOUT_TILT_DEG = (30.0, 70.0)
TOE_RING_R = (1.8, 2.3)              # dome r-space, ~= 0.8-1.3x reach beyond the wall
TOE_RING_N = (20, 50)
TOE_RING_SCALE = (0.22, 0.45)        # -> ~0.2-0.4 m off the ~0.8-1.0 m native chunks

RAFT_FLANK_TILT_DEG = (20.0, 40.0)   # 1-2 rafts on the flank, not the crown

# round-3 review: the toe was a hard rectangle in every wide view — modulate
# the footprint's angular radius so the outline is lobed/irregular, not a
# rounded rectangle. Same lobe params drive the apron, scaled wider.
ANGLE_LOBES = (3, 5)
ANGLE_AMP = (0.15, 0.25)
APRON_WIDER_FRAC = (0.15, 0.25)       # round-4 review: was 0.30-0.40 (1.4x), rendered as a big flat rectangle
SHEET_MAX_PER_RC = 1                  # was up to 2, drawn independently — coordinator round-3
REBAR_BESIDE_RAFT_P = 0.7            # rebar tangles beside a raft, not scattered independently
CLUSTER_N_URM = (5, 10)              # was the generic 3-10 for everyone
LINTEL_CROWN_P = 0.8                 # "lintels near the crown", was 0.3


# ---------------------------------------------------------------------------
# Small math helpers
# ---------------------------------------------------------------------------

def _clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _grid_dims(span_x, span_y, cell_m=GRID_CELL_M, tri_cap=GRID_TRI_CAP):
    """(nx, ny) for a `cell_m`-ish grid over (span_x, span_y), coarsened just
    enough to keep 2*(nx-1)*(ny-1) triangles under `tri_cap`."""
    nx = max(5, int(round(span_x / cell_m)) + 1)
    ny = max(5, int(round(span_y / cell_m)) + 1)
    tris = 2 * (nx - 1) * (ny - 1)
    if tris > tri_cap:
        factor = math.sqrt(tris / float(tri_cap))
        cell2 = cell_m * factor
        nx = max(5, int(round(span_x / cell2)) + 1)
        ny = max(5, int(round(span_y / cell2)) + 1)
    return nx, ny


def _grid_faces(ny, nx):
    """CCW (seen from +z) triangle indices for a regular ny x nx point grid,
    row-major flattened (index = iy*nx+ix)."""
    iy, ix = np.meshgrid(np.arange(ny - 1), np.arange(nx - 1), indexing="ij")
    i00 = (iy * nx + ix).ravel()
    i10 = (iy * nx + ix + 1).ravel()
    i01 = ((iy + 1) * nx + ix).ravel()
    i11 = ((iy + 1) * nx + ix + 1).ravel()
    t1 = np.stack([i00, i10, i11], axis=1)
    t2 = np.stack([i00, i11, i01], axis=1)
    return np.concatenate([t1, t2], axis=0).astype(np.int64)


def _lattice_noise(nrng, X, Y, wavelength, xmin, ymin):
    """Bilinearly-interpolated value noise on a lattice spaced `wavelength`
    apart, evaluated at every (X, Y). Deterministic given `nrng`'s state."""
    wavelength = max(wavelength, 1e-3)
    nxl = max(2, int(np.ceil((X.max() - xmin) / wavelength)) + 2)
    nyl = max(2, int(np.ceil((Y.max() - ymin) / wavelength)) + 2)
    lattice = nrng.uniform(-1.0, 1.0, size=(nyl, nxl))
    fx = np.clip((X - xmin) / wavelength, 0.0, nxl - 1.0 - 1e-6)
    fy = np.clip((Y - ymin) / wavelength, 0.0, nyl - 1.0 - 1e-6)
    ix0 = np.floor(fx).astype(int)
    iy0 = np.floor(fy).astype(int)
    ix1 = np.minimum(ix0 + 1, nxl - 1)
    iy1 = np.minimum(iy0 + 1, nyl - 1)
    tx, ty = fx - ix0, fy - iy0
    v00, v10 = lattice[iy0, ix0], lattice[iy0, ix1]
    v01, v11 = lattice[iy1, ix0], lattice[iy1, ix1]
    v0 = v00 * (1 - tx) + v10 * tx
    v1 = v01 * (1 - tx) + v11 * tx
    return v0 * (1 - ty) + v1 * ty


def _fbm2d(nrng, X, Y, xmin, ymin, wavelengths, amplitudes):
    total = np.zeros_like(X, dtype=np.float64)
    for wl, amp in zip(wavelengths, amplitudes):
        if amp <= 0.0:
            continue
        total = total + amp * _lattice_noise(nrng, X, Y, wl, xmin, ymin)
    return total


def _lattice_noise1d(nrng, T, wavelength, tmin):
    wavelength = max(wavelength, 1e-3)
    ntl = max(2, int(np.ceil((T.max() - tmin) / wavelength)) + 2)
    lattice = nrng.uniform(-1.0, 1.0, size=(ntl,))
    ft = np.clip((T - tmin) / wavelength, 0.0, ntl - 1.0 - 1e-6)
    it0 = np.floor(ft).astype(int)
    it1 = np.minimum(it0 + 1, ntl - 1)
    tt = ft - it0
    return lattice[it0] * (1 - tt) + lattice[it1] * tt


def _fbm1d(nrng, T, tmin, wavelengths, amplitudes):
    total = np.zeros_like(T, dtype=np.float64)
    for wl, amp in zip(wavelengths, amplitudes):
        if amp <= 0.0:
            continue
        total = total + amp * _lattice_noise1d(nrng, T, wl, tmin)
    return total


def _bilinear_lookup(x0, y0, dx, dy, nx, ny, arr, x, y):
    """Bilinear lookup on a regular grid; returns None if (x, y) is outside
    the grid's bounds."""
    fx = (x - x0) / dx if dx else 0.0
    fy = (y - y0) / dy if dy else 0.0
    if fx < 0.0 or fx > nx - 1 or fy < 0.0 or fy > ny - 1:
        return None
    ix0 = int(math.floor(fx)); iy0 = int(math.floor(fy))
    ix1 = min(ix0 + 1, nx - 1); iy1 = min(iy0 + 1, ny - 1)
    tx, ty = fx - ix0, fy - iy0
    v00, v10 = arr[iy0, ix0], arr[iy0, ix1]
    v01, v11 = arr[iy1, ix0], arr[iy1, ix1]
    v0 = v00 * (1 - tx) + v10 * tx
    v1 = v01 * (1 - tx) + v11 * tx
    return v0 * (1 - ty) + v1 * ty


# ---------------------------------------------------------------------------
# Orientation: quaternions (w, x, y, z), and the Euler XYZ convention used
# for authored-box `rot_deg` in `large` entries. CONVENTION (document this
# for the emitter, agent C): R = Rz(rz) @ Ry(ry) @ Rx(rx) applied to a column
# vector, i.e. rotate about local X, then Y, then Z, each about the fixed
# (pre-rotation) axes — the same convention as USD's rotateXYZ xformOp and
# Blender's default 'XYZ' Euler order.
# ---------------------------------------------------------------------------

def _quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def _quat_from_axis_angle(axis, angle_deg):
    ax, ay, az = axis
    n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    ax, ay, az = ax / n, ay / n, az / n
    half = math.radians(angle_deg) * 0.5
    s = math.sin(half)
    return _quat_normalize((math.cos(half), ax * s, ay * s, az * s))


def _quat_to_matrix(q):
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([[1 - (yy + zz), xy - wz, xz + wy],
                      [xy + wz, 1 - (xx + zz), yz - wx],
                      [xz - wy, yz + wx, 1 - (xx + yy)]])


def _euler_xyz_to_matrix(rx, ry, rz):
    cx, sx = math.cos(math.radians(rx)), math.sin(math.radians(rx))
    cy, sy = math.cos(math.radians(ry)), math.sin(math.radians(ry))
    cz, sz = math.cos(math.radians(rz)), math.sin(math.radians(rz))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _matrix_to_euler_xyz_deg(Rm):
    """Inverse of `_euler_xyz_to_matrix` (R = Rz @ Ry @ Rx convention)."""
    r20 = _clip(float(Rm[2, 0]), -1.0, 1.0)
    ry = math.asin(-r20)
    cy = math.cos(ry)
    if abs(cy) > 1e-6:
        rx = math.atan2(float(Rm[2, 1]), float(Rm[2, 2]))
        rz = math.atan2(float(Rm[1, 0]), float(Rm[0, 0]))
    else:                                   # gimbal lock, arbitrary split
        rx = math.atan2(-float(Rm[1, 2]), float(Rm[1, 1]))
        rz = 0.0
    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))


def _quat_to_euler_xyz_deg(q):
    return _matrix_to_euler_xyz_deg(_quat_to_matrix(q))


def _quat_align_vec_to(src, dst):
    """Shortest-arc rotation taking unit vector `src` to unit vector `dst`."""
    s = np.asarray(src, dtype=np.float64)
    s = s / (float(np.linalg.norm(s)) or 1.0)
    d = np.asarray(dst, dtype=np.float64)
    nn = float(np.linalg.norm(d))
    if nn < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    d = d / nn
    dp = float(np.dot(s, d))
    if dp > 0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    if dp < -0.999999:
        perp = np.cross(s, [1.0, 0.0, 0.0])
        if float(np.linalg.norm(perp)) < 1e-6:
            perp = np.cross(s, [0.0, 1.0, 0.0])
        perp = perp / float(np.linalg.norm(perp))
        return _quat_from_axis_angle(tuple(perp), 180.0)
    axis = np.cross(s, d)
    axis = axis / (float(np.linalg.norm(axis)) or 1.0)
    angle = math.degrees(math.acos(_clip(dp, -1.0, 1.0)))
    return _quat_from_axis_angle(tuple(axis), angle)


def _quat_align_z_to(normal):
    """Shortest-arc rotation taking +Z to unit vector `normal`."""
    return _quat_align_vec_to((0.0, 0.0, 1.0), normal)


def _random_unit_quat(rng):
    """Uniform-on-SO(3) unit quaternion (Shoemake 1992) drawn from `rng`."""
    u1, u2, u3 = rng.random(), rng.random(), rng.random()
    a = math.sqrt(1.0 - u1)
    b = math.sqrt(u1)
    return _quat_normalize((b * math.cos(2 * math.pi * u3),
                             a * math.sin(2 * math.pi * u2),
                             a * math.cos(2 * math.pi * u2),
                             b * math.sin(2 * math.pi * u3)))


def _orient_on_surface(rng, normal_world, tilt_range):
    """Align local +Z to `normal_world`, tilt a further `tilt_range` degrees
    about a random world-horizontal axis, and spin the object about its own
    (pre-tilt) Z by a random yaw. Used for rafts/panels/clusters."""
    q_align = _quat_align_z_to(normal_world)
    spin = rng.uniform(0.0, 360.0)
    q_spin = _quat_from_axis_angle((0.0, 0.0, 1.0), spin)
    tilt = rng.uniform(*tilt_range)
    tang = rng.uniform(0.0, 360.0)
    tx, ty = math.cos(math.radians(tang)), math.sin(math.radians(tang))
    q_tilt = _quat_from_axis_angle((tx, ty, 0.0), tilt)
    return _quat_normalize(_quat_mul(_quat_mul(q_tilt, q_align), q_spin))


def _orient_tilted_from_vertical(rng, tilt_from_vertical_range):
    """A near-horizontal orientation: local +Z tipped `tilt_from_vertical`
    degrees away from world +Z toward a random azimuth, then spun about its
    own axis. Used for column stubs lying at 60-90 deg from vertical."""
    tilt = rng.uniform(*tilt_from_vertical_range)
    az = rng.uniform(0.0, 360.0)
    target = (math.sin(math.radians(tilt)) * math.cos(math.radians(az)),
              math.sin(math.radians(tilt)) * math.sin(math.radians(az)),
              math.cos(math.radians(tilt)))
    return _orient_on_surface(rng, target, (0.0, 0.0))


def rotated_extent(size, scale, rot):
    """(zmin_rel, zmax_rel): the world-Z range of a `size` x `scale` box,
    rotated by `rot` about its OWN origin, given that origin is the box's
    BOTTOM-CENTRE — x, y in [-s/2, s/2], z in [0, s] before rotation — which
    is the convention for every asset in `CATALOGUE` and every authored box
    in `large` (column/lintel/quoin/joist: same convention, the emitter
    authors those boxes bottom-centred too). `rot` is either a quaternion
    (w, x, y, z) or an (rx, ry, rz) XYZ-Euler-degrees tuple, i.e. either
    representation `plan_pile` itself emits (`instances[...]["orientations"]`
    or `large[i]["rot_deg"]`).

    Public: the emitter / its tests use this to verify a piece's world
    footing without re-deriving the rotation math (round-2 review: the
    previous placement code assumed a CENTRED origin, which is wrong for a
    bottom-centred asset whenever the rotation has a non-zero Z-component —
    it put rafts up to half their thickness above the surface at low bury
    fractions)."""
    if len(rot) == 4:
        R = _quat_to_matrix(rot)
    else:
        R = _euler_xyz_to_matrix(rot[0], rot[1], rot[2])
    sx, sy, sz = size[0] * scale, size[1] * scale, size[2] * scale
    hx, hy = sx / 2.0, sy / 2.0
    zs = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (0.0, sz):
                zs.append(R[2, 0] * dx + R[2, 1] * dy + R[2, 2] * dz)
    return float(min(zs)), float(max(zs))


def _chunk_orientation(rng, size, stickout_prob):
    """A chunk's thinnest local axis points roughly up (lying on its
    broadest face) most of the time; `stickout_prob` of the time it is
    tilted hard so a corner/edge reads as sticking out of the pile instead
    (round-2 review: pure random rotation read as "pebbles sprinkled
    evenly", not settled rubble)."""
    thin_i = min(range(3), key=lambda i: size[i])
    thin_axis = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)][thin_i]
    if rng.random() < stickout_prob:
        tilt = rng.uniform(*CHUNK_STICKOUT_TILT_DEG)
    else:
        tilt = rng.uniform(*CHUNK_FLAT_TILT_DEG)
    az = rng.uniform(0.0, 360.0)
    target = (math.sin(math.radians(tilt)) * math.cos(math.radians(az)),
              math.sin(math.radians(tilt)) * math.sin(math.radians(az)),
              math.cos(math.radians(tilt)))
    q_align = _quat_align_vec_to(thin_axis, target)
    q_spin = _quat_from_axis_angle(thin_axis, rng.uniform(0.0, 360.0))
    return _quat_normalize(_quat_mul(q_align, q_spin))


# ---------------------------------------------------------------------------
# DOME heightfield — a total collapse over the whole footprint.
#
# Shape: superellipse-normalised radius `r` (0 at the footprint centre, 1 at
# the wall line, 2 at the toe/run-out), a FLAT crown for r<=0.35 and a purely
# LINEAR taper from there to 0 at r=2.0 (eq_round4_rubble_research.md sec1d:
# "concave-to-straight, not convex" flanks; sec1b/5b: height tapers linearly
# to zero at the run-out) with 3-octave fbm layered on top and clamped so
# the crown, not the noise, sets the peak.
# ---------------------------------------------------------------------------

def _dome_gxy(X, Y, halfW, halfD, reach):
    gx = np.empty_like(X)
    inside = np.abs(X) <= halfW
    gx[inside] = X[inside] / halfW
    outp = X > halfW
    gx[outp] = 1.0 + (X[outp] - halfW) / reach["E"]
    outn = X < -halfW
    gx[outn] = -1.0 + (X[outn] + halfW) / reach["W"]
    gy = np.empty_like(Y)
    insidey = np.abs(Y) <= halfD
    gy[insidey] = Y[insidey] / halfD
    outpy = Y > halfD
    gy[outpy] = 1.0 + (Y[outpy] - halfD) / reach["N"]
    outny = Y < -halfD
    gy[outny] = -1.0 + (Y[outny] + halfD) / reach["S"]
    return gx, gy


def _dome_shape_r(gx, gy, p=4.0, r_flat=0.35, r_toe=2.0):
    r = (np.abs(gx) ** p + np.abs(gy) ** p) ** (1.0 / p)
    shape = np.clip(1.0 - (r - r_flat) / (r_toe - r_flat), 0.0, 1.0)
    return r, shape


def _draw_lobe(rng):
    """3-5 dominant lobes plus a weaker higher harmonic so the outline is
    irregular, not a clean flower — round-3 review ("the toe is a hard
    rectangle in every wide view")."""
    return {"n": rng.randint(*ANGLE_LOBES), "amp": rng.uniform(*ANGLE_AMP),
            "phase": rng.uniform(0.0, 2.0 * math.pi), "phase2": rng.uniform(0.0, 2.0 * math.pi)}


def _lobe_r_toe(gx, gy, lobe, base=2.0):
    """Per-point toe radius (in r-space): `base`, modulated +-`amp` with
    `n` dominant lobes and a weaker (2n+1) harmonic, as a function of angle
    in the (gx, gy) superellipse space."""
    theta = np.arctan2(gy, gx)
    n = lobe["n"]
    mod = (lobe["amp"] * np.sin(n * theta + lobe["phase"]) +
           0.4 * lobe["amp"] * np.sin((2 * n + 1) * theta + lobe["phase2"]))
    return base * (1.0 + mod)


def dome_volume_estimate(W, D, reach, crown_m, r_flat=0.35, r_toe=2.0, p=4.0, n=400):
    """Independent numeric double-integral of the SAME closed-form dome shape
    `plan_pile` builds (no noise/stub/plate_ok — those wash out in volume to
    well under the acceptance tolerance), at a resolution decoupled from the
    mesh's own 0.6-0.9 m cells. Used by the test suite as the "analytic dome
    estimate" cross-check, and by anyone else who wants a volume without
    building the mesh."""
    halfW, halfD = W / 2.0, D / 2.0
    xmin, xmax = -halfW - reach["W"], halfW + reach["E"]
    ymin, ymax = -halfD - reach["S"], halfD + reach["N"]
    xs = np.linspace(xmin, xmax, n)
    ys = np.linspace(ymin, ymax, n)
    X, Y = np.meshgrid(xs, ys)
    gx, gy = _dome_gxy(X, Y, halfW, halfD, reach)
    _, shape = _dome_shape_r(gx, gy, p=p, r_flat=r_flat, r_toe=r_toe)
    height = crown_m * shape
    dx, dy = xs[1] - xs[0], ys[1] - ys[0]
    cell_avg = 0.25 * (height[:-1, :-1] + height[1:, :-1] + height[:-1, 1:] + height[1:, 1:])
    return float(np.sum(cell_avg) * dx * dy)


def _prism_volume(height, dx, dy):
    cell_avg = 0.25 * (height[:-1, :-1] + height[1:, :-1] + height[:-1, 1:] + height[1:, 1:])
    return float(np.sum(cell_avg) * dx * dy)


def _max_slope_deg(height, dx, dy, mask=None):
    gy_, gx_ = np.gradient(height, dy, dx)
    slope = np.degrees(np.arctan(np.hypot(gx_, gy_)))
    vals = slope[mask] if mask is not None else slope
    return float(np.max(vals)) if vals.size else 0.0


def _limit_slope(height, dx, dy, max_slope_deg, iters=30, clamp_edges=False):
    """Thermal-erosion-style relaxation: repeatedly halve any edge-to-edge
    height difference that implies a slope over `max_slope_deg` (axis cap at
    max_slope/sqrt(2) so the COMBINED gradient magnitude, which is what
    `_max_slope_deg` measures, stays under the limit even on a worst-case
    diagonal). This is the actual repose-angle enforcement — the analytic
    crown cap only sets a good starting point; the piecewise gx/gy scale
    changes at the flatten radius and the wall line, plus the fbm noise, can
    both create local slopes well past that starting point (measured while
    tuning this module), and no closed-form fix for both at once was found.
    Cells already within the limit are untouched (excess is 0 there), so
    this does not flatten compliant regions.

    `clamp_edges`: re-zero the outer ring after every pass, not just once at
    the end. Without this, a persistently over-cap monotonic run all the way
    to the boundary (a blind-side corner bulge, or just an unlucky noise
    draw) has "nowhere to go" — this is a mass-conserving Jacobi diffusion,
    so with a free edge it keeps shovelling height toward the dead-end
    boundary every pass, and after enough iterations the boundary-adjacent
    cells end up HIGHER, not lower (measured: a single fix-the-cap-at-the-
    end pass left a bigger cliff than not relaxing at all). Anchoring the
    edge at its known target (0 here) every pass fixes that."""
    max_tan_axis = math.tan(math.radians(max_slope_deg)) / math.sqrt(2.0)
    allowed_x, allowed_y = max_tan_axis * dx, max_tan_axis * dy
    h = height.copy()
    for _ in range(iters):
        dxarr = h[:, 1:] - h[:, :-1]
        exx = np.where(np.abs(dxarr) > allowed_x, (np.abs(dxarr) - allowed_x) * np.sign(dxarr) * 0.5, 0.0)
        h[:, 1:] -= exx
        h[:, :-1] += exx
        dyarr = h[1:, :] - h[:-1, :]
        eyy = np.where(np.abs(dyarr) > allowed_y, (np.abs(dyarr) - allowed_y) * np.sign(dyarr) * 0.5, 0.0)
        h[1:, :] -= eyy
        h[:-1, :] += eyy
        if clamp_edges:
            h[0, :] = 0.0; h[-1, :] = 0.0
            h[:, 0] = 0.0; h[:, -1] = 0.0
    return h


def _build_dome_grid(m, btype, rng, nrng, crown_m_arg, fall_sides, stub_h_m, plate_ok):
    W, D = float(m["W"]), float(m["D"])
    halfW, halfD = W / 2.0, D / 2.0
    Hbldg = max(0.1, m["top"] - m["z0"])

    reach = {}
    for side in _SIDES:
        is_fall = side in fall_sides
        frac = runout_frac(Hbldg, btype, fall=is_fall) * rng.uniform(0.85, 1.15)
        cap = RUNOUT_CAP_FALL_M if is_fall else RUNOUT_CAP_BLIND_M
        reach[side] = min(cap, max(RUNOUT_FLOOR_M, frac * Hbldg))

    crown_target = (crown_m_arg if crown_m_arg is not None
                    else min(CROWN_FRAC.get(btype, CROWN_FRAC["rc"]) * Hbldg, CROWN_CAP_M))
    # Repose-angle cap on crown height so the r=[0.35,2.0] linear ramp never
    # exceeds REPOSE_DEG. Only the FALL side(s)' reach bounds this: a blind /
    # party-wall side is confined by the standing neighbour, not by open
    # repose (eq_round4_rubble_research.md sec1b/5a) — it is allowed to bank
    # up steeper than repose against that wall, so a small blind-side reach
    # (often the 1.5 m floor) must not suppress the crown everywhere.
    fall_scales = [reach[s] for s in fall_sides] or [reach["S"]]
    min_scale = min([halfW, halfD] + fall_scales)
    crown_cap_phys = 1.65 * min_scale * math.tan(math.radians(REPOSE_DESIGN_DEG))
    crown_target = max(0.3, min(crown_target, crown_cap_phys))

    lobe = _draw_lobe(rng)
    infl = 1.0 + 1.4 * lobe["amp"]          # worst-case combined lobe bulge; grow the domain to fit it
    xmin, xmax = -halfW - reach["W"] * infl, halfW + reach["E"] * infl
    ymin, ymax = -halfD - reach["S"] * infl, halfD + reach["N"] * infl
    nx, ny = _grid_dims(xmax - xmin, ymax - ymin)
    xs = np.linspace(xmin, xmax, nx)
    ys = np.linspace(ymin, ymax, ny)
    X, Y = np.meshgrid(xs, ys)          # shape (ny, nx)

    gx, gy = _dome_gxy(X, Y, halfW, halfD, reach)
    r = (np.abs(gx) ** 4 + np.abs(gy) ** 4) ** 0.25
    r_toe = _lobe_r_toe(gx, gy, lobe, base=2.0)
    shape = np.clip(1.0 - (r - 0.35) / (r_toe - 0.35), 0.0, 1.0)
    height = crown_target * shape

    mx = max(W, D)
    wavelengths = [0.35 * mx, 0.12 * mx, 0.04 * mx]
    amplitudes = [NOISE_AMP_SCALE_123 * f * crown_target for f in (0.10, 0.06, 0.03)]
    envelope = np.clip(1.0 - np.maximum(r - 1.0, 0.0) / (r_toe - 1.0), 0.0, 1.0)  # 1 at r<=1, 0 at the (lobed) toe
    height = height + _fbm2d(nrng, X, Y, xmin, ymin, wavelengths, amplitudes) * envelope

    # Octaves 4-5: real 1-2 m surface lumps (round-2 review — the mound read
    # as a smooth dune). Absolute-metre wavelength/amplitude, masked to 0 at
    # the toe / anywhere the pile (pre-fine-relief) is under ~0.6 m tall.
    wl4, amp4 = rng.uniform(*RELIEF_WAVELENGTH_4), rng.uniform(*RELIEF_AMP_4)
    wl5, amp5 = rng.uniform(*RELIEF_WAVELENGTH_5), rng.uniform(*RELIEF_AMP_5)
    # Must ALSO fade by r toward the true toe, same as `envelope` above — a
    # column off the fall-side axis can still show a moderate base height
    # (>0) right up against the domain's hard r>=2 edge (a blind-side corner
    # bulge), and without this the fine relief kept adding a bump right
    # against the boundary that then got clipped to 0, producing a one-cell
    # cliff (measured while chasing a spurious repose-slope failure).
    relief_mask = np.clip(height / RELIEF_MASK_HEIGHT_M, 0.0, 1.0) * envelope
    height = height + _fbm2d(nrng, X, Y, xmin, ymin, [wl4, wl5], [amp4, amp5]) * relief_mask

    stub_h_m = float(stub_h_m or 0.0)
    if stub_h_m > 0.0:
        band = np.clip(1.0 - np.abs(r - 1.0) / 0.25, 0.0, 1.0)
        height = np.maximum(height, 0.5 * stub_h_m * band)

    dxg, dyg = xs[1] - xs[0], ys[1] - ys[0]
    # Target a few degrees under REPOSE_DEG, not AT it: the relaxation's
    # fixed point (Jacobi-style pairwise diffusion, not a global solve) can
    # still leave isolated cells a few degrees over its own nominal cap —
    # measured while chasing this, see the relief_mask/envelope fix above
    # for the other half of the story. More iterations stopped helping past
    # ~80 (it had already reached its fixed point).
    height = _limit_slope(height, dxg, dyg, REPOSE_DESIGN_DEG, iters=80, clamp_edges=True)
    if stub_h_m > 0.0:
        height = np.maximum(height, 0.5 * stub_h_m * band)   # relaxation must not erase the fill rule

    ceiling = max(crown_target, 0.5 * stub_h_m)
    # Floor at MOUND_LIP_M, not 0: NOTHING in this mesh may be exactly
    # coplanar with the ground plate (round-4 review — a black z-fight band
    # hugging the toe on several sides). The lip is 8 mm, invisible next to
    # a multi-metre crown.
    height = np.clip(height, MOUND_LIP_M, ceiling)
    height[0, :] = MOUND_LIP_M; height[-1, :] = MOUND_LIP_M
    height[:, 0] = MOUND_LIP_M; height[:, -1] = MOUND_LIP_M

    if plate_ok is not None:
        a = math.radians(m["yaw"])
        WX = m["cx"] + X * math.cos(a) - Y * math.sin(a)
        WY = m["cy"] + X * math.sin(a) + Y * math.cos(a)
        mask = np.vectorize(lambda wx, wy: bool(plate_ok(wx, wy)))(WX, WY)
        height = np.where(mask, height, MOUND_LIP_M)

    # "the flanks" for the repose-slope check = the FALL-side flanks only
    # (see the crown-cap comment above: a blind/party-wall side is allowed to
    # be steeper, confined by the standing neighbour). A point's slope is
    # dominated by whichever of gx/gy is larger in magnitude (that is the
    # axis `r`'s gradient is steepest along, since r=(gx^4+gy^4)^0.25) — a
    # small blind-side reach makes gx or gy shoot up fast right past that
    # wall, and that steep patch can sit CLOSER (in plain xy distance) to a
    # fall-side wall line than to its own blind wall, so classifying by
    # nearest-wall distance mis-attributes it. Classify by dominant axis
    # instead.
    x_dom = np.abs(gx) >= np.abs(gy)
    is_S = (~x_dom) & (gy < 0.0)
    is_N = (~x_dom) & (gy >= 0.0)
    is_E = x_dom & (gx >= 0.0)
    is_W = x_dom & (gx < 0.0)
    fall_mask = np.zeros_like(x_dom)
    for side, side_mask in (("S", is_S), ("N", is_N), ("E", is_E), ("W", is_W)):
        if side in fall_sides:
            fall_mask = fall_mask | side_mask
    slope_mask = (r > 0.40) & (r < 0.97 * r_toe) & fall_mask
    return {"X": X, "Y": Y, "height": height, "xs": xs, "ys": ys,
            "x0": xmin, "y0": ymin, "dx": xs[1] - xs[0], "dy": ys[1] - ys[0],
            "nx": nx, "ny": ny, "reach": reach, "crown_target": crown_target,
            "halfW": halfW, "halfD": halfD, "r": r, "r_toe": r_toe,
            "slope_mask": slope_mask, "fall_mask": fall_mask, "lobe": lobe}


def _build_dome_apron(m, reach, apron_thick, rng, nrng, lobe, mult=1.35):
    """A thin skirt from the mound's (lobed) toe out to `mult`x that toe
    radius — same lobe params as the mound, so its outline follows the
    mound's, just wider (round-3 review: "no straight edges anywhere on the
    outline"). It feathers down to APRON_LIP_M at its own rim, NOT to z0
    (round-4 review: an apron rim exactly at z0 is coplanar with the ground
    plate and z-fights)."""
    W, D = float(m["W"]), float(m["D"])
    halfW, halfD = W / 2.0, D / 2.0
    infl = mult * (1.0 + 1.4 * lobe["amp"])
    ereach = {s: reach[s] * infl for s in _SIDES}
    xmin, xmax = -halfW - ereach["W"], halfW + ereach["E"]
    ymin, ymax = -halfD - ereach["S"], halfD + ereach["N"]
    nx, ny = _grid_dims(xmax - xmin, ymax - ymin, cell_m=0.75)
    xs = np.linspace(xmin, xmax, nx)
    ys = np.linspace(ymin, ymax, ny)
    X, Y = np.meshgrid(xs, ys)
    gx, gy = _dome_gxy(X, Y, halfW, halfD, reach)      # SAME normalisation as the mound
    r = (np.abs(gx) ** 4 + np.abs(gy) ** 4) ** 0.25
    r_toe_mound = _lobe_r_toe(gx, gy, lobe, base=2.0)
    r_toe_apron = r_toe_mound * mult
    span = np.maximum(r_toe_apron - r_toe_mound, 1e-3)
    apron_shape = np.where(r >= r_toe_mound, np.clip(1.0 - (r - r_toe_mound) / span, 0.0, 1.0), 0.0)
    height = APRON_LIP_M + apron_thick * apron_shape
    height[0, :] = APRON_LIP_M; height[-1, :] = APRON_LIP_M
    height[:, 0] = APRON_LIP_M; height[:, -1] = APRON_LIP_M
    return {"X": X, "Y": Y, "height": height, "xs": xs, "ys": ys,
            "x0": xmin, "y0": ymin, "dx": xs[1] - xs[0], "dy": ys[1] - ys[0],
            "nx": nx, "ny": ny}


# ---------------------------------------------------------------------------
# WINDROW / FAN — a shed wall, parapet or gable, along one or more `sides`.
# Cross-section perpendicular to the wall tapers from `depth_m` at the wall
# line to 0 at `reach` (eq_round4_rubble_research.md sec5b: linear taper,
# reach ~= the failed element's own height); along-length variation via fbm;
# tapering ends. A FAN is a windrow whose effective width (via a d-dependent
# end-taper fraction, not a sheared mesh) widens toward the toe.
# ---------------------------------------------------------------------------


def _trim_flat(pts, faces, eps=1e-6):
    """Drop every triangle whose three vertices all sit on the mesh's flat
    floor (its lip level), and compact the point array.

    Each heightfield cell is built on a RECTANGULAR grid whose domain is
    inflated to fit the worst-case lobe bulge, and everything outside the
    lobed toe is floored at the lip (`MOUND_LIP_M` / `APRON_LIP_M`). Left in
    the mesh, that floor is a flat, textured rectangle 1 cm above the ground
    round every pile — the first Isaac render of round 4 (2026-08-30,
    r4_commercial DG5 top view) showed exactly that: a pale dust RECTANGLE
    with sharp straight edges under a lobed mound, the "plaza" the v3 review
    had rejected, back again because the Blender preview ground sat 2 cm
    below it and hid it. Trimming to the raised faces leaves the lobed
    outline as the mesh outline; the surviving rim vertices still sit at the
    lip, so nothing is coplanar with the ground.
    """
    pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
    faces = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    if pts.shape[0] == 0 or faces.shape[0] == 0:
        return pts, faces
    zflat = float(pts[:, 2].min()) + eps
    flat_v = pts[:, 2] <= zflat
    keep = ~(flat_v[faces[:, 0]] & flat_v[faces[:, 1]] & flat_v[faces[:, 2]])
    faces = faces[keep]
    if faces.shape[0] == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
    used = np.unique(faces)
    remap = -np.ones(pts.shape[0], dtype=np.int64)
    remap[used] = np.arange(used.shape[0])
    return pts[used], remap[faces]

def _side_axes(side, m, offset_m, reach, t_lo, t_hi):
    W, D = float(m["W"]), float(m["D"])
    halfW, halfD = W / 2.0, D / 2.0
    if side == "S":
        xr = (t_lo, t_hi); yr = (-halfD - offset_m - reach, -halfD - offset_m)
        d_of = lambda X, Y: (-halfD - offset_m) - Y
        t_of = lambda X, Y: X
        near = "row-1"
    elif side == "N":
        xr = (t_lo, t_hi); yr = (halfD + offset_m, halfD + offset_m + reach)
        d_of = lambda X, Y: Y - (halfD + offset_m)
        t_of = lambda X, Y: X
        near = "row0"
    elif side == "E":
        yr = (t_lo, t_hi); xr = (halfW + offset_m, halfW + offset_m + reach)
        d_of = lambda X, Y: X - (halfW + offset_m)
        t_of = lambda X, Y: Y
        near = "col0"
    elif side == "W":
        yr = (t_lo, t_hi); xr = (-halfW - offset_m - reach, -halfW - offset_m)
        d_of = lambda X, Y: (-halfW - offset_m) - X
        t_of = lambda X, Y: Y
        near = "col-1"
    else:
        raise ValueError(side)
    return xr, yr, d_of, t_of, near


def _build_strip(m, side, t_lo, t_hi, reach, depth_m, offset_m, rng, nrng,
                  taper_lo=0.12, taper_hi=0.12):
    if t_hi < t_lo:
        t_lo, t_hi = t_hi, t_lo
    span = max(t_hi - t_lo, 1e-3)
    xr, yr, d_of, t_of, near = _side_axes(side, m, offset_m, reach, t_lo, t_hi)
    span_x, span_y = xr[1] - xr[0], yr[1] - yr[0]
    cellx = _clip(span_x / _GRID_TARGET_CELLS_ACROSS, *_GRID_CELL_RANGE) if span_x > 0 else 0.75
    celly = _clip(span_y / _GRID_TARGET_CELLS_ACROSS, *_GRID_CELL_RANGE) if span_y > 0 else 0.75
    nx = max(5, int(round(span_x / cellx)) + 1)
    ny = max(5, int(round(span_y / celly)) + 1)
    xs = np.linspace(xr[0], xr[1], nx)
    ys = np.linspace(yr[0], yr[1], ny)
    X, Y = np.meshgrid(xs, ys)          # (ny, nx)

    D_ = d_of(X, Y)
    T = t_of(X, Y)
    u = (T - t_lo) / span
    taper_frac = np.clip(taper_lo + (taper_hi - taper_lo) * np.clip(D_ / reach, 0.0, 1.0), 1e-3, 0.49)
    edge = np.minimum(u, 1.0 - u)
    end_taper = np.clip(edge / taper_frac, 0.0, 1.0)
    cross = depth_m * np.clip(1.0 - (D_ / reach) ** 1.6, 0.0, 1.0)
    wl = [0.30 * span, 0.10 * span]
    amp = [0.20 * depth_m, 0.10 * depth_m]
    noise_mult = np.clip(1.0 + _fbm1d(nrng, T, t_lo, wl, amp), 0.4, 1.6)
    height = cross * end_taper * noise_mult
    height = np.clip(height, 0.0, depth_m)

    if near != "row0": height[0, :] = MOUND_LIP_M
    if near != "row-1": height[-1, :] = MOUND_LIP_M
    if near != "col0": height[:, 0] = MOUND_LIP_M
    if near != "col-1": height[:, -1] = MOUND_LIP_M

    return {"X": X, "Y": Y, "height": height, "xs": xs, "ys": ys,
            "x0": xr[0], "y0": yr[0], "dx": xs[1] - xs[0] if nx > 1 else 1.0,
            "dy": ys[1] - ys[0] if ny > 1 else 1.0, "nx": nx, "ny": ny,
            "d": D_, "t": T, "near": near}


# ---------------------------------------------------------------------------
# Mesh assembly, surface lookups, sampling, element placement.
# ---------------------------------------------------------------------------

def _mesh_to_world(m, cell, z0):
    X, Y, H = cell["X"], cell["Y"], cell["height"]
    a = math.radians(m["yaw"])
    ca, sa = math.cos(a), math.sin(a)
    WX = m["cx"] + X * ca - Y * sa
    WY = m["cy"] + X * sa + Y * ca
    WZ = z0 + H
    pts = np.stack([WX.ravel(), WY.ravel(), WZ.ravel()], axis=1).astype(np.float64)
    faces = _grid_faces(cell["ny"], cell["nx"])
    return pts, faces


def _cell_height_at(cell, x, y):
    return _bilinear_lookup(cell["x0"], cell["y0"], cell["dx"], cell["dy"],
                             cell["nx"], cell["ny"], cell["height"], x, y)


def _cell_normal_local(cell, x, y):
    eps = 0.5 * min(cell["dx"], cell["dy"])
    hx1 = _cell_height_at(cell, x + eps, y); hx0 = _cell_height_at(cell, x - eps, y)
    hy1 = _cell_height_at(cell, x, y + eps); hy0 = _cell_height_at(cell, x, y - eps)
    if None in (hx1, hx0, hy1, hy0):
        return (0.0, 0.0, 1.0)
    dhdx = (hx1 - hx0) / (2 * eps)
    dhdy = (hy1 - hy0) / (2 * eps)
    n = np.array([-dhdx, -dhdy, 1.0])
    nn = float(np.linalg.norm(n)) or 1.0
    n = n / nn
    return (float(n[0]), float(n[1]), float(n[2]))


def _normal_to_world(m, n_local):
    a = math.radians(m["yaw"]); ca, sa = math.cos(a), math.sin(a)
    nx, ny, nz = n_local
    return (nx * ca - ny * sa, nx * sa + ny * ca, nz)


def _apply_bump(cells, lx, ly, amp, radius):
    """Add a Gaussian "shoulder" of peak height `amp` and characteristic
    `radius` to whichever cell contains (lx, ly) — round-2 review: large
    elements need to look embedded in the fines, not resting on top of a
    smooth dune. Cheap no-op if amp/radius are ~0."""
    if amp <= 1e-6 or radius <= 1e-6:
        return
    for c in cells:
        x0, x1 = c["x0"], c["x0"] + c["dx"] * (c["nx"] - 1)
        y0, y1 = c["y0"], c["y0"] + c["dy"] * (c["ny"] - 1)
        if not (x0 <= lx <= x1 and y0 <= ly <= y1):
            continue
        d2 = (c["X"] - lx) ** 2 + (c["Y"] - ly) ** 2
        c["height"] = c["height"] + amp * np.exp(-d2 / (2.0 * radius * radius))
        return


def _finalize_cell_boundary(cell):
    """Re-clamp a cell's outer ring to z0 + MOUND_LIP_M after any post-build
    mutation (shoulder bumps) — same near-edge exemption `_build_strip` uses
    for a windrow/fan (the wall-facing edge is not an "outer ring"). Never
    plain z0 (round-4 review: z-fights the ground plate)."""
    near = cell.get("near")
    if near != "row0": cell["height"][0, :] = MOUND_LIP_M
    if near != "row-1": cell["height"][-1, :] = MOUND_LIP_M
    if near != "col0": cell["height"][:, 0] = MOUND_LIP_M
    if near != "col-1": cell["height"][:, -1] = MOUND_LIP_M


def _summarize_mound(cells):
    """(crown_actual, volume_m3, max_slope_deg) from the CURRENT state of
    `cells` — called once before placement (a rough estimate to normalise
    density weighting) and once after (the authoritative numbers reported in
    `stats`, once shoulder bumps have been baked in)."""
    crown_actual = max((float(np.max(c["height"])) for c in cells), default=0.0)
    volume_m3 = sum(_prism_volume(c["height"], c["dx"], c["dy"]) for c in cells)
    slopes = []
    for c in cells:
        gy_, gx_ = np.gradient(c["height"], c["dy"], c["dx"])
        slope = np.degrees(np.arctan(np.hypot(gx_, gy_)))
        mask = c.get("slope_mask")
        if mask is None:
            mask = c["d"] > 0.0
        vals = slope[mask]
        slopes.append(float(np.max(vals)) if vals.size else 0.0)
    max_slope = max(slopes) if slopes else 0.0
    return crown_actual, float(volume_m3), max_slope


def _cells_bounds(cells):
    x0 = min(c["x0"] for c in cells)
    x1 = max(c["x0"] + c["dx"] * (c["nx"] - 1) for c in cells)
    y0 = min(c["y0"] for c in cells)
    y1 = max(c["y0"] + c["dy"] * (c["ny"] - 1) for c in cells)
    return x0, x1, y0, y1


def _reject_xy(rng, xmin, xmax, ymin, ymax, weight_fn, wmax, m, plate_ok, max_tries):
    for _ in range(max_tries):
        x = rng.uniform(xmin, xmax)
        y = rng.uniform(ymin, ymax)
        w = weight_fn(x, y)
        if w is None or w <= 0.0:
            continue
        if rng.random() > w / wmax:
            continue
        if plate_ok is not None:
            wx, wy = _to_world(m, x, y)
            if not plate_ok(wx, wy):
                continue
        return x, y
    return None


def _sample_flank_point(rng, cell, m, plate_ok, crown=False, max_tries=200):
    hmax = float(np.max(cell["height"])) if cell["height"].size else 0.0
    x0, x1 = cell["x0"], cell["x0"] + cell["dx"] * (cell["nx"] - 1)
    y0, y1 = cell["y0"], cell["y0"] + cell["dy"] * (cell["ny"] - 1)
    for _ in range(max_tries):
        x = rng.uniform(x0, x1); y = rng.uniform(y0, y1)
        h = _cell_height_at(cell, x, y)
        if h is None:
            continue
        rel = h / hmax if hmax > 1e-9 else 0.0
        if crown and rel < 0.6:
            continue
        if not crown and rel > 0.9 and rng.random() < 0.7:
            continue
        if plate_ok is not None:
            wx, wy = _to_world(m, x, y)
            if not plate_ok(wx, wy):
                continue
        return x, y
    return (0.0, 0.0)


def _sample_near_wall_point(rng, cell, m, plate_ok, max_tries=200):
    x0, x1 = cell["x0"], cell["x0"] + cell["dx"] * (cell["nx"] - 1)
    y0, y1 = cell["y0"], cell["y0"] + cell["dy"] * (cell["ny"] - 1)
    has_r = "r" in cell
    for _ in range(max_tries):
        x = rng.uniform(x0, x1); y = rng.uniform(y0, y1)
        if has_r:
            rr = _bilinear_lookup(cell["x0"], cell["y0"], cell["dx"], cell["dy"],
                                   cell["nx"], cell["ny"], cell["r"], x, y)
            if rr is None or not (0.75 <= rr <= 1.25):
                continue
        if plate_ok is not None:
            wx, wy = _to_world(m, x, y)
            if not plate_ok(wx, wy):
                continue
        return x, y
    return (0.0, 0.0)


def _sample_toe_point(rng, cell, m, plate_ok, side="S", max_tries=200):
    """A point out near the run-out ring. `side=None` skips the directional
    bias (any toe point, for a general "clusters at the toe" placement)."""
    x0, x1 = cell["x0"], cell["x0"] + cell["dx"] * (cell["nx"] - 1)
    y0, y1 = cell["y0"], cell["y0"] + cell["dy"] * (cell["ny"] - 1)
    has_r = "r" in cell
    nx_, ny_ = _SIDE_NORMAL.get(side, (0.0, -1.0)) if side else (0.0, 0.0)
    for _ in range(max_tries):
        x = rng.uniform(x0, x1); y = rng.uniform(y0, y1)
        if has_r:
            rr = _bilinear_lookup(cell["x0"], cell["y0"], cell["dx"], cell["dy"],
                                   cell["nx"], cell["ny"], cell["r"], x, y)
            if rr is None or not (1.4 <= rr <= 2.0):
                continue
        if side and (x * nx_ + y * ny_) < 0.0 and rng.random() < 0.8:
            continue                        # bias toward the requested (street) side
        if plate_ok is not None:
            wx, wy = _to_world(m, x, y)
            if not plate_ok(wx, wy):
                continue
        return x, y
    return (0.0, -m["D"] / 2.0 - 1.0)


def _append_instance(inst, name, pos, quat, scale):
    if name not in inst["protos"]:
        inst["protos"].append(name)
    inst["proto_index"].append(inst["protos"].index(name))
    inst["positions"].append(pos)
    inst["orientations"].append(quat)
    inst["scales"].append(scale)


def _empty_instance_set(look=None):
    return {"protos": [], "proto_index": [], "positions": [], "orientations": [], "scales": [],
            "look": look}


# ---------------------------------------------------------------------------
# plan_pile — the API contract entry point.
# ---------------------------------------------------------------------------

def plan_pile(m, btype, rng, kind="dome", crown_m=None, spread_frac=None,
              sides=None, along=None, depth_m=None, offset_m=0.0,
              plate_ok=None, stub_h_m=0.0, panels=(), budget=None, seed_tag="",
              elem_h_m=None):
    """Plan a rubble pile for one collapsed mass (or shed wall) `m`.

    `elem_h_m` (added, not in the original contract text): the height of the
    specific wall/parapet element that fell, for `kind in ("windrow","fan")`
    — eq_round4_rubble_research.md sec5b: reach ~= 1.0x that element's own
    height, not `spread_frac x H` of the whole building. When given it
    overrides `spread_frac` for windrow/fan; `spread_frac` still works as a
    fallback (or for callers that don't know the failed element's height).
    """
    nrng = np.random.default_rng(rng.getrandbits(32))
    z0 = float(m["z0"])
    Hbldg = max(0.1, float(m["top"]) - z0)
    storeys = max(1, len(m.get("levels", [z0])))
    btype = btype if btype in PROTO_SETS else "rc"
    look = "urm" if btype == "urm" else "rc"
    proto_sets = PROTO_SETS[btype]
    stub_h_m = float(stub_h_m or 0.0)

    cells = []
    apron_cells = []
    fall_sides = set()

    if kind == "dome":
        if sides is None:
            other = [s for s in ("N", "E", "W")]
            fall_sides = {"S", rng.choice(other)}
        else:
            fall_sides = set(sides) or {"S"}
        cell = _build_dome_grid(m, btype, rng, nrng, crown_m, fall_sides, stub_h_m, plate_ok)
        cells.append(cell)
        reach_report = dict(cell["reach"])
        reach_scalar = max(reach_report.values())
        footprint = {"kind": "dome", "W": m["W"], "D": m["D"], "reach": reach_report,
                     "fall_sides": sorted(fall_sides)}
        apron_thick = rng.uniform(0.03, 0.08)      # round-4 review: was 0.05-0.12
        apron_mult = 1.0 + rng.uniform(*APRON_WIDER_FRAC)
        apron_cells.append(_build_dome_apron(m, cell["reach"], apron_thick, rng, nrng,
                                              cell["lobe"], mult=apron_mult))
        density_cell = cell

    elif kind in ("windrow", "fan"):
        use_sides = tuple(sides) if sides else ("S",)
        t0, t1 = along if along else (-0.5, 0.5)
        if depth_m is not None:
            depth_use = float(depth_m)
        elif elem_h_m is not None and elem_h_m <= WINDROW_PARAPET_ELEM_H_MAX:
            depth_use = rng.uniform(*WINDROW_DEPTH_PARAPET)
        else:
            depth_use = rng.uniform(*WINDROW_DEPTH_WALL)

        strip_reach = {}
        for side in use_sides:
            if elem_h_m is not None:
                reach_s = max(RUNOUT_FLOOR_M,
                               elem_h_m * WINDROW_REACH_FRAC * rng.uniform(*WINDROW_REACH_JITTER))
            else:
                sf = (spread_frac if spread_frac is not None else
                      0.5 * (runout_frac(Hbldg, btype, True) + runout_frac(Hbldg, btype, False)))
                reach_s = min(RUNOUT_CAP_FALL_M, max(RUNOUT_FLOOR_M, sf * Hbldg))
            reach_s = max(reach_s, 2.3 * depth_use)      # repose floor (sec1c/5b)
            strip_reach[side] = reach_s

        widen = rng.uniform(*FAN_WIDEN) if kind == "fan" else 1.0
        for side in use_sides:
            L = _side_length(m, side)
            t_lo0, t_hi0 = t0 * L, t1 * L
            reach_s = strip_reach[side]
            if kind == "fan":
                centre = 0.5 * (t_lo0 + t_hi0); half0 = 0.5 * (t_hi0 - t_lo0)
                t_lo, t_hi = centre - half0 * widen, centre + half0 * widen
                taper_lo = _clip(0.5 * (1.0 - 1.0 / widen), 0.05, 0.45)
                taper_hi = 0.05
            else:
                t_lo, t_hi = t_lo0, t_hi0
                taper_lo = taper_hi = 0.12
            cells.append(_build_strip(m, side, t_lo, t_hi, reach_s, depth_use, offset_m,
                                       rng, nrng, taper_lo, taper_hi))
            fall_sides.add(side)

        reach_report = strip_reach
        reach_scalar = max(strip_reach.values())
        footprint = {"kind": kind, "sides": list(use_sides), "reach": reach_report,
                     "along": (t0, t1), "depth_m": depth_use,
                     "widen": widen if kind == "fan" else None, "fall_sides": list(use_sides)}

        # round-4 review: no apron for a windrow/fan at all (dome only) — a
        # thin strip apron next to an already-narrow ridge read as its own
        # separate flat rectangle, not a dust skirt.
        density_cell = cells[0]
    else:
        raise ValueError("plan_pile: unknown kind {0!r}".format(kind))

    # Rough (pre-shoulder-bump) crown/mound read, used only to normalise the
    # density weighting below; the authoritative numbers (reported in
    # `stats`) are recomputed at the very end, after every large element and
    # cluster has raised the surface locally.
    crown_actual, _v0, _s0 = _summarize_mound(cells)

    # --- surface lookups used while placing elements ---
    def surf_h(x, y):
        for c in cells:
            v = _cell_height_at(c, x, y)
            if v is not None:
                return v
        return 0.0

    def surf_normal(x, y):
        for c in cells:
            v = _cell_height_at(c, x, y)
            if v is not None:
                return _normal_to_world(m, _cell_normal_local(c, x, y))
        return (0.0, 0.0, 1.0)

    def place(lx, ly, quat, size, scale, bury_range):
        """Bottom-centred placement (round-2 burial fix): the piece's own
        LOWEST rotated point lands exactly `bury x thickness` below the
        surface, not its (undefined, since the origin is bottom-centred, not
        the centroid) "centre"."""
        h = surf_h(lx, ly)
        bury = rng.uniform(*bury_range)
        zmin_rel, zmax_rel = rotated_extent(size, scale, quat)
        thickness = max(1e-3, zmax_rel - zmin_rel)
        origin_z = z0 + h - zmin_rel - bury * thickness
        wx, wy = _to_world(m, lx, ly)
        return (wx, wy, origin_z), bury

    bump_history = []

    def place_bumped(lx, ly, quat, size, scale, bury_range):
        """Like `place`, but first raises the surface under (lx, ly) with a
        Gaussian "shoulder" sized from the piece's own rotated thickness and
        footprint diagonal, so the piece reads as embedded in the fines
        rather than resting on top of a smooth mound (round-2 review).

        `radius` is floored at ~1.3x `amp`: an element with a small footprint
        but a tall rotated thickness (a column stub lying at 60-90 deg, e.g.
        — tiny sx/sy, but its long axis still contributes a lot of rotated Z)
        would otherwise get a needle-narrow bump whose OWN slope is 60+ deg —
        measured while chasing a blown repose-slope stat that had nothing to
        do with the mound shape itself."""
        zmin_rel, zmax_rel = rotated_extent(size, scale, quat)
        thickness = max(1e-3, zmax_rel - zmin_rel)
        diag = math.hypot(size[0] * scale, size[1] * scale)
        # A big, already-flat FAB spread (a cluster like huge_concrete_
        # rubble_pile, ~8x7.7x1.5 m) can show several metres of rotated Z
        # extent from even a shallow tilt, purely because its OWN footprint
        # is huge — that is not a "shoulder", it is the asset's own bulk, so
        # cap the bump at a real single-lump height regardless.
        # A windrow/fan is already a thin, narrow ridge sized to `depth_m`;
        # a shoulder bump there overshoots that target height rather than
        # reading as "embedded" (round-4 review: a fan rendered thicker
        # than its own depth_m because of exactly this). Bumps are a dome-
        # only embellishment.
        if kind != "dome":
            return place(lx, ly, quat, size, scale, bury_range)
        amp = min(SHOULDER_BUMP_HEIGHT_FRAC * thickness, SHOULDER_BUMP_MAX_M)
        radius = max(SHOULDER_BUMP_RADIUS_FRAC * diag, 2.5 * amp)
        # Two bumps landing close together (5-10 clusters isn't much room)
        # stack, and the combined slope at their shared edge can still clear
        # the cap even though each bump alone doesn't — damp `amp` by how
        # crowded this spot already is rather than relaxing the FINAL field
        # (which would float every already-seated element near it).
        crowd = 0.0
        for (bx, by, br) in bump_history:
            d = math.hypot(lx - bx, ly - by)
            if d < br + radius:
                crowd += max(0.0, 1.0 - d / (br + radius))
        amp *= 1.0 / (1.0 + 1.5 * crowd)
        bump_history.append((lx, ly, radius))
        _apply_bump(cells, lx, ly, amp, radius)
        return place(lx, ly, quat, size, scale, bury_range)

    # --- large elements ---
    large = []

    n_raft = 0
    if btype != "urm" and proto_sets["raft"]:
        n_raft = int(_clip(round(0.6 * storeys * RAFT_SCALE.get(btype, 1.0)), 3, 8))
    # round-2 review: "the rafts must READ" — at least 3 (or all, if fewer
    # than 3 total) clustered tight on the crown, tilted 5-25 deg, bury
    # 0.10-0.30 so most of each stands proud; 1-2 on the flank, tilted
    # steeper (20-40 deg).
    n_crown_raft = n_raft if n_raft <= 3 else max(3, n_raft - rng.randint(1, 2))
    raft_local_pts = []
    for i in range(n_raft):
        name = rng.choice(proto_sets["raft"])
        size = CATALOGUE[name]["size"]
        on_crown = i < n_crown_raft
        lx, ly = _sample_flank_point(rng, density_cell, m, plate_ok, crown=on_crown)
        tilt = RAFT_CROWN_TILT_DEG if on_crown else RAFT_FLANK_TILT_DEG
        quat = _orient_on_surface(rng, surf_normal(lx, ly), tilt)
        scale = rng.uniform(*RAFT_SCALE_RANGE)
        pos, bury = place_bumped(lx, ly, quat, size, scale, BURY["raft"])
        large.append({"asset": name, "prim_path": None, "kind": "raft", "pos": pos,
                      "rot_deg": _quat_to_euler_xyz_deg(quat), "scale": scale,
                      "size": size, "bury": bury, "look": None})
        raft_local_pts.append((lx, ly))

    n_col = rng.randint(*COLUMN_STUBS) if btype in ("rc", "rc_glass") else 0
    for _ in range(n_col):
        size = (COLUMN_SIZE_XY, COLUMN_SIZE_XY, rng.uniform(*COLUMN_SIZE_Z))
        lx, ly = _sample_flank_point(rng, cells[0], m, plate_ok, crown=False)
        quat = _orient_tilted_from_vertical(rng, COLUMN_TILT_FROM_VERTICAL)
        pos, bury = place_bumped(lx, ly, quat, size, 1.0, BURY["column"])
        large.append({"asset": None, "prim_path": None, "kind": "column", "pos": pos,
                      "rot_deg": _quat_to_euler_xyz_deg(quat), "scale": 1.0,
                      "size": size, "bury": bury, "look": "concrete"})

    n_rebar = rng.randint(*REBAR_N) if (btype in ("rc", "rc_glass") and proto_sets["rebar"]) else 0
    for _ in range(n_rebar):
        name = rng.choice(proto_sets["rebar"])
        size = CATALOGUE[name]["size"]
        # "rebar tangles beside rafts" — sit next to a raft rather than
        # scattered independently, most of the time.
        if raft_local_pts and rng.random() < REBAR_BESIDE_RAFT_P:
            rlx, rly = rng.choice(raft_local_pts)
            lx = rlx + rng.uniform(-1.5, 1.5)
            ly = rly + rng.uniform(-1.5, 1.5)
        else:
            lx, ly = _sample_flank_point(rng, cells[0], m, plate_ok, crown=False)
        quat = _orient_on_surface(rng, surf_normal(lx, ly), (0.0, 30.0))
        scale = rng.uniform(0.9, 1.1)
        pos, bury = place(lx, ly, quat, size, scale, BURY["rebar"])
        large.append({"asset": name, "prim_path": None, "kind": "rebar", "pos": pos,
                      "rot_deg": _quat_to_euler_xyz_deg(quat), "scale": scale,
                      "size": size, "bury": bury, "look": "rust"})

    # Corrugated sheets read as a flag when tilted — at most SHEET_MAX_PER_RC
    # (1), always flat, never on a windrow/fan (round-3 review).
    if proto_sets["sheet"] and kind == "dome" and rng.random() < SHEET_PROB.get(btype, 0.0):
        name = rng.choice(proto_sets["sheet"])
        size = CATALOGUE[name]["size"]
        lx, ly = _sample_flank_point(rng, cells[0], m, plate_ok, crown=False)
        thin_i = min(range(3), key=lambda i: size[i])
        thin_axis = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)][thin_i]
        tilt = rng.uniform(0.0, 8.0)
        az = rng.uniform(0.0, 360.0)
        target = (math.sin(math.radians(tilt)) * math.cos(math.radians(az)),
                  math.sin(math.radians(tilt)) * math.sin(math.radians(az)),
                  math.cos(math.radians(tilt)))
        quat = _quat_normalize(_quat_mul(_quat_align_vec_to(thin_axis, target),
                                          _quat_from_axis_angle(thin_axis, rng.uniform(0.0, 360.0))))
        scale = rng.uniform(0.9, 1.1)
        pos, bury = place(lx, ly, quat, size, scale, (0.3, 0.5))
        large.append({"asset": name, "prim_path": None, "kind": "sheet", "pos": pos,
                      "rot_deg": _quat_to_euler_xyz_deg(quat), "scale": scale,
                      "size": size, "bury": bury, "look": "rust"})

    n_lintel = rng.randint(*LINTEL_N) if btype == "urm" else 0
    for _ in range(n_lintel):
        if rng.random() < LINTEL_LONG_SHARE:
            sx, sy, sz = (rng.uniform(*a) for a in LINTEL_LONG_SIZE)
            subkind = "lintel"
        else:
            sx, sy, sz = (rng.uniform(*a) for a in LINTEL_QUOIN_SIZE)
            subkind = "quoin"
        size = (sx, sy, sz)
        lx, ly = _sample_flank_point(rng, cells[0], m, plate_ok, crown=(rng.random() < LINTEL_CROWN_P))
        yaw = rng.uniform(0.0, 360.0)
        tip = rng.uniform(0.0, 15.0); tang = rng.uniform(0.0, 360.0)
        tx, ty = math.cos(math.radians(tang)), math.sin(math.radians(tang))
        quat = _quat_normalize(_quat_mul(_quat_from_axis_angle((tx, ty, 0.0), tip),
                                          _quat_from_axis_angle((0.0, 0.0, 1.0), yaw)))
        pos, bury = place(lx, ly, quat, size, 1.0, BURY["lintel"])
        large.append({"asset": None, "prim_path": None, "kind": subkind, "pos": pos,
                      "rot_deg": _quat_to_euler_xyz_deg(quat), "scale": 1.0,
                      "size": size, "bury": bury, "look": "stone"})

    n_joist = rng.randint(*JOIST_N) if btype == "urm" else 0
    for _ in range(n_joist):
        size = tuple(rng.uniform(*a) for a in JOIST_SIZE)
        lx, ly = _sample_flank_point(rng, cells[0], m, plate_ok, crown=False)
        quat = _orient_on_surface(rng, surf_normal(lx, ly), (5.0, 40.0))
        pos, bury = place(lx, ly, quat, size, 1.0, BURY["joist"])
        large.append({"asset": None, "prim_path": None, "kind": "joist", "pos": pos,
                      "rot_deg": _quat_to_euler_xyz_deg(quat), "scale": 1.0,
                      "size": size, "bury": bury, "look": "timber"})

    for prim_path, size in panels:
        if stub_h_m > 0.0:
            lx, ly = _sample_near_wall_point(rng, cells[0], m, plate_ok)
            tilt = PANEL_LEAN_DEG
        else:
            lx, ly = _sample_flank_point(rng, cells[0], m, plate_ok, crown=False)
            tilt = PANEL_TILT_DEG
        quat = _orient_on_surface(rng, surf_normal(lx, ly), tilt)
        pos, bury = place_bumped(lx, ly, quat, size, 1.0, BURY["panel"])
        large.append({"asset": None, "prim_path": prim_path, "kind": "panel", "pos": pos,
                      "rot_deg": _quat_to_euler_xyz_deg(quat), "scale": 1.0,
                      "size": size, "bury": bury, "look": None})

    # --- instance sets --- "look" is a per-set material tag for the
    # emitter (round-4 review): chunk/flake are untextured flat-shaded
    # meshes and need a tint; cluster/toe are already-textured FAB spreads
    # that keep their own referenced material (look=None -> no override).
    chunk_flake_look = "brick" if btype == "urm" else "concrete"
    instances = {"chunk": _empty_instance_set(chunk_flake_look),
                 "flake": _empty_instance_set(chunk_flake_look),
                 "cluster": _empty_instance_set(None), "toe": _empty_instance_set(None)}

    xmin_all, xmax_all, ymin_all, ymax_all = _cells_bounds(cells)

    # Patchy density: chunks cluster in drifts, not an even sprinkle
    # (round-2 review). One low-frequency noise lattice per plan.
    patch_wl = max(CHUNK_PATCH_WAVELENGTH_FRAC * max(m["W"], m["D"]), 0.5)
    _pnx = max(2, int(math.ceil((xmax_all - xmin_all) / patch_wl)) + 2)
    _pny = max(2, int(math.ceil((ymax_all - ymin_all) / patch_wl)) + 2)
    patch_lattice = nrng.uniform(0.0, 1.0, size=(_pny, _pnx))

    def patch_factor(x, y):
        v = _bilinear_lookup(xmin_all, ymin_all, patch_wl, patch_wl, _pnx, _pny, patch_lattice, x, y)
        if v is None:
            v = 0.5
        lo, hi = CHUNK_PATCH_CONTRAST
        return lo + (hi - lo) * v

    def _chunk_density(rel):
        """Piecewise crown/mid-flank/toe density (round-4 review: a plain
        crown-to-toe LINEAR interpolation put too few chunks on the crown
        plateau itself — a real crown is dense with small chunks, and only
        the flank thins out toward the toe)."""
        crown, mid, toe = CHUNK_DENSITY["crown"], CHUNK_DENSITY["mid"], CHUNK_DENSITY["toe"]
        return np.where(rel >= 0.5, mid + (crown - mid) * (rel - 0.5) / 0.5,
                         toe + (mid - toe) * (rel / 0.5))

    def chunk_weight(x, y):
        h = None
        for c in cells:
            h = _cell_height_at(c, x, y)
            if h is not None:
                break
        if h is None:
            return None
        rel = _clip(h / max(crown_actual, 1e-6), 0.0, 1.0)
        base = float(_chunk_density(rel))
        return base * patch_factor(x, y)

    stickout_prob = rng.uniform(*CHUNK_STICKOUT_PROB)

    def chunk_scale(_rng=rng):
        return 0.28 + 0.72 * (_rng.random() ** 2.2)     # power law: median ~0.4, tail to ~1.0

    dens_hi = CHUNK_DENSITY["crown"]
    n_est = 0.0
    if crown_actual > 0:
        for c in cells:
            rel = np.clip(c["height"] / crown_actual, 0.0, 1.0)
            dens = _chunk_density(rel)
            n_est += float(np.sum(dens[:-1, :-1]) * c["dx"] * c["dy"])
    n_chunks = int(_clip(round(n_est) if n_est > 0 else 40, 20, CHUNK_DENSITY["cap"]))

    n_runout = int(round(rng.uniform(*RUNOUT_CHUNK_FRAC) * n_chunks)) if kind == "dome" else 0
    n_primary = max(0, n_chunks - n_runout)
    wmax_chunk = dens_hi * CHUNK_PATCH_CONTRAST[1]
    for _ in range(n_primary):
        xy = _reject_xy(rng, xmin_all, xmax_all, ymin_all, ymax_all, chunk_weight,
                         wmax_chunk, m, plate_ok, 400)
        if xy is None:
            continue
        lx, ly = xy
        name = rng.choice(proto_sets["chunk"])
        size = CATALOGUE[name]["size"]
        quat = _chunk_orientation(rng, size, stickout_prob)
        scale = chunk_scale()
        pos, _b = place(lx, ly, quat, size, scale, BURY["chunk"])
        _append_instance(instances["chunk"], name, pos, quat, scale)

    for _ in range(n_runout):
        xy = None
        for _try in range(30):
            side = rng.choice(_SIDES)
            reach_s = reach_report.get(side, reach_scalar)
            ext = max(reach_s * (RUNOUT_CHUNK_REACH_MULT - 1.0), 0.05)
            d_extra = rng.uniform(0.05 * reach_s, ext)
            t = rng.uniform(-0.5, 0.5)
            if side == "S":
                lx, ly = t * m["W"], -m["D"] / 2.0 - reach_s - d_extra
            elif side == "N":
                lx, ly = t * m["W"], m["D"] / 2.0 + reach_s + d_extra
            elif side == "E":
                lx, ly = m["W"] / 2.0 + reach_s + d_extra, t * m["D"]
            else:
                lx, ly = -m["W"] / 2.0 - reach_s - d_extra, t * m["D"]
            wx, wy = _to_world(m, lx, ly)
            if plate_ok is None or plate_ok(wx, wy):
                xy = (lx, ly)
                break
        if xy is None:
            continue
        lx, ly = xy
        name = rng.choice(proto_sets["chunk"])
        size = CATALOGUE[name]["size"]
        quat = _chunk_orientation(rng, size, stickout_prob)
        scale = chunk_scale()
        pos, _b = place(lx, ly, quat, size, scale, BURY["chunk"])
        _append_instance(instances["chunk"], name, pos, quat, scale)

    # A denser band of small chunks right at the run-out ring (round-2
    # review): dome only — for a windrow/fan the reach is already small.
    if kind == "dome" and "r" in density_cell:
        for _ in range(rng.randint(*TOE_RING_N)):
            xy = None
            for _try in range(60):
                x = rng.uniform(xmin_all, xmax_all)
                y = rng.uniform(ymin_all, ymax_all)
                rr = _bilinear_lookup(density_cell["x0"], density_cell["y0"], density_cell["dx"],
                                       density_cell["dy"], density_cell["nx"], density_cell["ny"],
                                       density_cell["r"], x, y)
                if rr is None or not (TOE_RING_R[0] <= rr <= TOE_RING_R[1]):
                    continue
                if plate_ok is not None:
                    wx, wy = _to_world(m, x, y)
                    if not plate_ok(wx, wy):
                        continue
                xy = (x, y)
                break
            if xy is None:
                continue
            lx, ly = xy
            name = rng.choice(proto_sets["chunk"])
            size = CATALOGUE[name]["size"]
            quat = _chunk_orientation(rng, size, stickout_prob)
            scale = rng.uniform(*TOE_RING_SCALE)
            pos, _b = place(lx, ly, quat, size, scale, BURY["chunk"])
            _append_instance(instances["chunk"], name, pos, quat, scale)

    def flake_weight(x, y):
        h = None
        for c in cells:
            h = _cell_height_at(c, x, y)
            if h is not None:
                break
        if h is None:
            return None
        rel = _clip(h / max(crown_actual, 1e-6), 0.0, 1.0)
        return 1.2 - 0.9 * rel                    # fines concentrate at the toe (sec4)

    n_flake = rng.randint(*FLAKE_N)
    for _ in range(n_flake):
        xy = _reject_xy(rng, xmin_all, xmax_all, ymin_all, ymax_all, flake_weight, 1.2,
                         m, plate_ok, 300)
        if xy is None:
            continue
        lx, ly = xy
        name = rng.choice(proto_sets["flake"])
        quat = _random_unit_quat(rng)
        scale = rng.uniform(*FLAKE_SCALE)
        size = CATALOGUE[name]["size"]
        pos, _b = place(lx, ly, quat, size, scale, BURY["flake"])
        _append_instance(instances["flake"], name, pos, quat, scale)

    # round-2 review: URM was reading with only 2 brick piles — "5-10 on the
    # flanks AND at the toe", half-and-half.
    n_cluster = rng.randint(*CLUSTER_N_URM) if btype == "urm" else rng.randint(*CLUSTER_N)
    n_cluster_toe = n_cluster // 2
    for i in range(n_cluster):
        if not proto_sets["cluster"]:
            break
        if i < n_cluster_toe:
            lx, ly = _sample_toe_point(rng, density_cell, m, plate_ok, side=None)
        else:
            lx, ly = _sample_flank_point(rng, density_cell, m, plate_ok, crown=False)
        name = rng.choice(proto_sets["cluster"])
        quat = _orient_on_surface(rng, surf_normal(lx, ly), (5.0, 20.0))
        scale = rng.uniform(*CLUSTER_SCALE)
        size = CATALOGUE[name]["size"]
        pos, _b = place_bumped(lx, ly, quat, size, scale, BURY["cluster"])
        _append_instance(instances["cluster"], name, pos, quat, scale)

    if proto_sets["toe"]:
        street_side = "S" if "S" in fall_sides or not fall_sides else sorted(fall_sides)[0]
        for _ in range(rng.randint(2, 5)):
            lx, ly = _sample_toe_point(rng, density_cell, m, plate_ok, side=street_side)
            name = rng.choice(proto_sets["toe"])
            quat = _quat_from_axis_angle((0.0, 0.0, 1.0), rng.uniform(0.0, 360.0))
            scale = rng.uniform(*CLUSTER_SCALE)
            size = CATALOGUE[name]["size"]
            pos, _b = place(lx, ly, quat, size, scale, BURY["cluster"])
            _append_instance(instances["toe"], name, pos, quat, scale)

    # --- budget ---
    if budget:
        if "n_large" in budget and len(large) > budget["n_large"]:
            order = {"panel": 0, "raft": 1, "rebar": 2, "sheet": 3, "column": 4,
                     "lintel": 5, "quoin": 5, "joist": 6}
            large = sorted(large, key=lambda e: order.get(e["kind"], 9))[:budget["n_large"]]
        if "n_instances" in budget:
            total = sum(len(v["positions"]) for v in instances.values())
            cap = int(budget["n_instances"])
            if total > cap > 0:
                ratio = cap / float(total)
                keeps = {k: int(round(len(v["positions"]) * ratio)) for k, v in instances.items()}
                over = sum(keeps.values()) - cap
                # independent per-set rounding can overshoot the cap by a
                # few — trim the excess from whichever sets are largest.
                for k in sorted(keeps, key=lambda kk: -keeps[kk]):
                    if over <= 0:
                        break
                    take = min(over, keeps[k])
                    keeps[k] -= take
                    over -= take
                for k, v in instances.items():
                    keep = keeps[k]
                    v["proto_index"] = v["proto_index"][:keep]
                    v["positions"] = v["positions"][:keep]
                    v["orientations"] = v["orientations"][:keep]
                    v["scales"] = v["scales"][:keep]

    # --- assemble the mound + apron meshes NOW, after every shoulder bump
    # from the placement passes above has been baked into `cells` ---
    # (A global relaxation pass here was tried and reverted: smoothing the
    # FINAL field after elements are already seated moves the surface out
    # from under them — every element near a smoothed bump ends up floating,
    # which is worse than the slope excess it was fixing. `place_bumped`'s
    # own per-bump amp/radius cap is what has to hold the line instead.)
    for c in cells:
        # A shoulder bump's Gaussian footprint can reach past its own
        # element (which is itself plate-checked) onto a neighbouring
        # off-plate vertex and un-zero it — re-clamp after every bump, not
        # just at the original build.
        if plate_ok is not None:
            a = math.radians(m["yaw"])
            WX = m["cx"] + c["X"] * math.cos(a) - c["Y"] * math.sin(a)
            WY = m["cy"] + c["X"] * math.sin(a) + c["Y"] * math.cos(a)
            mask = np.vectorize(lambda wx, wy: bool(plate_ok(wx, wy)))(WX, WY)
            c["height"] = np.where(mask, c["height"], MOUND_LIP_M)
        _finalize_cell_boundary(c)
    crown_actual, volume_m3, max_slope = _summarize_mound(cells)

    def _concat_cells(cell_list):
        pts_l, faces_l, off = [], [], 0
        for c in cell_list:
            pts, faces = _mesh_to_world(m, c, z0)
            pts, faces = _trim_flat(pts, faces)
            pts_l.append(pts); faces_l.append(faces + off); off += pts.shape[0]
        if not pts_l:
            return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
        return np.concatenate(pts_l, axis=0), np.concatenate(faces_l, axis=0)

    mound_points, mound_faces = _concat_cells(cells)
    tri_count = int(mound_faces.shape[0])
    apron = None
    if apron_cells:
        apron_points, apron_faces = _concat_cells(apron_cells)
        tri_count += int(apron_faces.shape[0])
        apron = {"points": apron_points, "faces": apron_faces, "look": "dust"}

    def _grid_dict(c):
        return {"x0": c["x0"], "y0": c["y0"], "dx": c["dx"], "dy": c["dy"],
                "nx": c["nx"], "ny": c["ny"], "z": z0 + c["height"]}

    grid = _grid_dict(cells[0])
    grid.update({"yaw": m["yaw"], "cx": m["cx"], "cy": m["cy"], "z0": z0})
    if len(cells) > 1:
        grid["extra_grids"] = [_grid_dict(c) for c in cells[1:]]

    mound = {"points": mound_points, "faces": mound_faces, "look": look,
             "crown_z": z0 + crown_actual, "volume_m3": float(volume_m3),
             "reach_m": float(reach_scalar), "footprint": footprint,
             "grid": grid, "z0": z0}

    # --- floating check (round-2 review item 5): the lowest rotated point of
    # every large element / instance must not sit more than 5 cm above the
    # (bumped, final) surface. Should be 0 by construction of `place`.
    def _surf_final(x, y):
        for c in cells:
            v = _cell_height_at(c, x, y)
            if v is not None:
                return v
        return 0.0

    floating = 0
    for e in large:
        lx, ly = _to_local(m, e["pos"][0], e["pos"][1])
        zmin_rel, _zmax = rotated_extent(e["size"], e["scale"], e["rot_deg"])
        min_z_world = e["pos"][2] + zmin_rel
        surf = z0 + _surf_final(lx, ly)
        if min_z_world - surf > 0.05:
            floating += 1
    for v in instances.values():
        for i in range(len(v["positions"])):
            px, py, pz = v["positions"][i]
            quat = v["orientations"][i]
            scale = v["scales"][i]
            name = v["protos"][v["proto_index"][i]]
            size = CATALOGUE[name]["size"]
            lx, ly = _to_local(m, px, py)
            zmin_rel, _zmax = rotated_extent(size, scale, quat)
            min_z_world = pz + zmin_rel
            surf = z0 + _surf_final(lx, ly)
            if min_z_world - surf > 0.05:
                floating += 1

    n_inst_per_set = {k: len(v["positions"]) for k, v in instances.items()}
    stats = {"n_large": len(large), "n_instances": n_inst_per_set,
             "n_instances_total": sum(n_inst_per_set.values()),
             "crown_m": crown_actual, "volume_m3": float(volume_m3),
             "reach_m": reach_report, "fall_sides": sorted(fall_sides),
             "tri_count": tri_count, "max_slope_deg": float(max_slope),
             "kind": kind, "btype": btype, "look": look, "storeys": storeys,
             "floating": floating, "seed_tag": seed_tag}

    return {"mound": mound, "apron": apron, "large": large,
            "instances": instances, "stats": stats}


def surface_z(mound, x, y):
    """Bilinear height on the mound's heightfield; `mound["z0"]` outside."""
    g = mound["grid"]
    a = math.radians(-g["yaw"])
    dx0, dy0 = x - g["cx"], y - g["cy"]
    lx = dx0 * math.cos(a) - dy0 * math.sin(a)
    ly = dx0 * math.sin(a) + dy0 * math.cos(a)
    v = _bilinear_lookup(g["x0"], g["y0"], g["dx"], g["dy"], g["nx"], g["ny"], g["z"], lx, ly)
    if v is not None:
        return float(v)
    for eg in g.get("extra_grids", []):
        v = _bilinear_lookup(eg["x0"], eg["y0"], eg["dx"], eg["dy"], eg["nx"], eg["ny"],
                              eg["z"], lx, ly)
        if v is not None:
            return float(v)
    return float(g["z0"])
