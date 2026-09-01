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

This module is PURE numpy + stdlib at module scope — no top-level `pxr`
import — so it can be unit-tested and iterated on without Kit/Isaac.
`disaster/quake_rubble_usd.py` (another agent's file) is the only module
that turns a plan into USD prims. ONE narrow, deliberate exception:
`plan_street_scatter`'s seating optionally opens a catalogue asset's own
`.usdc` file with a LAZY, try/except-guarded `pxr` import to read its real
mesh points (`_load_local_mesh_points`/`_points_min_z`) — the fix for a
`UsdGeom.BBoxCache`-style blind spot (see the `fix-floating-debris` skill)
where a curled/warped scan piece's AABB-corner extrapolation sits well
below the real mesh, seating it with daylight underneath. Never required —
`pxr` unavailable, no local asset mirror, or any read failure all fall back
silently to the AABB-corner approximation (`rotated_extent`) every other
placement in this module already uses.

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

import json
import math
import os

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

# round-5 addendum ("use the scanned concrete debris more"): a urm pile's
# minority concrete share (see `_select_proto_sets`'s cluster block) repeats
# these three names — the big FAB spread and the two cheap Quixel piles —
# more than `concrete_debris_elements`/`concrete_slabs`, which already carry
# rc's cluster pool on their own. Repetition count in the weighted pool, not
# a probability.
_URM_CONC_MINORITY_BOOST = {"huge_concrete_rubble_pile": 2, "concrete_rubble_pile": 2,
                            "rocky_ground": 2}


# ---------------------------------------------------------------------------
# round-5: per-pile prototype selection from the HD debris piece library
# (`assets/rubble_hd/`, `HD_CATALOGUE` at the bottom of this file — 840
# textured pieces: chunk 340, flake 357, raft 22, street 121, material brick
# or concrete). `_select_proto_sets` resolves a fresh, per-pile-CAPPED
# `PROTO_SETS`-shaped dict from it, drawn with the PILE'S OWN `rng` so two
# piles differ but one pile is reproducible for a given seed. Falls back to
# the original flat-colour `PROTO_SETS[btype]` lists entirely when
# `HD_CATALOGUE` is empty (the library was never built on this checkout) —
# printed once, not once per pile, so a city-scale bake with no HD library
# still runs without spamming the log.
#
# `_asset_entry(name)` is the resolver every per-instance/per-large SIZE
# lookup in `plan_pile` goes through (never a bare `CATALOGUE[name]`, which
# KeyErrors on an HD name) — checks the original flat-colour `CATALOGUE`
# first, then `HD_CATALOGUE`. `HD_CATALOGUE` is defined at the bottom of this
# module (after `load_hd_catalogue()` runs at import time); referencing it
# here is fine because this function is only ever CALLED after the whole
# module has finished loading, never at import/definition time.
# ---------------------------------------------------------------------------
HD_PROTO_CAP = {"chunk": 24, "flake": 16, "raft": 8, "toe": 8}
HD_URM_MINORITY_MATERIAL = "concrete"     # lintels / floor fragments mixed into a brick pile
HD_URM_MINORITY_FRAC = 0.20

_HD_FALLBACK_WARNED = False


def _asset_entry(name):
    """`CATALOGUE[name]`, falling back to `HD_CATALOGUE[name]` — every
    prototype name a per-pile `PROTO_SETS` draw can produce (HD pieces mixed
    in alongside the original flat-colour catalogue, round-5) resolves
    through this. Returns `None` if `name` is in neither (defensive; should
    not happen for a name this module itself drew)."""
    return CATALOGUE.get(name) or HD_CATALOGUE.get(name)


def _hd_names_by(kind, materials):
    """Sorted (deterministic) list of `HD_CATALOGUE` names of the given
    `kind` whose `material` is in `materials`."""
    mats = set(materials)
    out = []
    for n, e in HD_CATALOGUE.items():
        if e.get("kind") != kind or e.get("material") not in mats:
            continue
        sz = tuple(float(v) for v in e.get("size", (0, 0, 0)))
        if kind == "chunk" and (not _chunky(sz) or _hd_open_frac(n) > HD_CHUNK_MAX_OPEN):
            continue        # a thin/open scan shell is not a chunk (v8b/v8c renders: standing foil)
        if kind == "flake" and _hd_open_frac(n) > HD_FLAKE_MAX_OPEN:
            continue
        out.append(n)
    return sorted(out)


# round-5 "more debris" addendum (user: "there's also more debris here...
# use this too along with the other debris assets i already gave you" —
# `assets/standalone/debris/`). `slab`/`chunk`/`lump`/`rebar`/`sheet` were
# already in `CATALOGUE` (round-4's ORIGINAL flat-colour set — see
# `_debris_material_for` in `quake_rubble_usd.py`, "the 34 flat-grey pieces
# this round's catalogue lists"), but only `raft` (the slabs) was ever
# UNIONED into the HD-era pools (line ~448 below); `chunk`/`flake` fell back
# to the flat set ONLY when `HD_CATALOGUE` is empty, which it never is on a
# built checkout — so chunk_01-09/lump_01-06 were effectively invisible.
#
# `_standalone_names_by` is `_hd_names_by`'s twin over the FLAT `CATALOGUE`
# instead of `HD_CATALOGUE`, gated by the SAME `_chunky`/open-fraction
# thresholds, reading `_hd_open_frac` off the SAME `open_frac.json` (now
# extended with a one-off measured entry per standalone piece — see
# `tools/`-style census note in this module's own docstring pattern).
# Measured (welded-vertex boundary-edge count, not a units artefact):
#   chunk_01-09: open_frac 0.21-0.22 -- ABOVE `HD_CHUNK_MAX_OPEN` (0.15).
#     These are genuinely part-open (a real topological gap spanning
#     top-to-bottom, not just a hidden buried underside), so they correctly
#     FAIL this gate and never enter `plan_pile`'s own "chunk" pool -- they
#     DO pass the street pool's looser 0.35 cap (`_street_chunk_pool` below)
#     with a healthy PCA volume_ratio (0.44-0.53), which is exactly the
#     "genuinely 3-D fragment, more boundary edges than a flat slab" profile
#     that gate was loosened for.
#   lump_01-06: open_frac 0.0-0.05 -- comfortably under `HD_FLAKE_MAX_OPEN`
#     (0.30). These qualify for the pile's own "flake" pool.
def _standalone_names_by(kind, materials):
    """Like `_hd_names_by` but over the flat `CATALOGUE` (`standalone/
    debris/pieces/*` only, via the `url` prefix check — so a future non-HD
    `CATALOGUE` addition of the same `kind` doesn't silently join this pool
    unmeasured)."""
    mats = set(materials)
    out = []
    for n, e in CATALOGUE.items():
        if e.get("kind") != kind or e.get("material") not in mats:
            continue
        if not str(e.get("url", "")).startswith("standalone/debris/pieces/"):
            continue
        sz = tuple(float(v) for v in e.get("size", (0, 0, 0)))
        if kind == "chunk" and (not _chunky(sz) or _hd_open_frac(n) > HD_CHUNK_MAX_OPEN):
            continue
        if kind == "flake" and _hd_open_frac(n) > HD_FLAKE_MAX_OPEN:
            continue
        out.append(n)
    return sorted(out)


# Target minority share (of the RETURNED, already-capped pool) for the
# standalone flake pieces folded into the pile's own flake instancing —
# ~15-30% asked for; 0.20 sits at the middle of that band.
STANDALONE_FLAKE_MINORITY_FRAC = 0.20


def _sample_pool_with_minority(rng, major_pool, minor_pool, minor_frac, cap):
    """`cap` names, `minor_frac` (rounded, capped by availability) drawn
    from `minor_pool` and the rest from `major_pool` -- `_sample_mixed_
    names`'s body, generalised for a caller that already has two NAME POOLS
    in hand (rather than a (kind, material) pair `_hd_names_by` would
    resolve for it). Tops up from whichever pool has names left if either
    alone can't fill its share."""
    n_minor = min(cap, len(minor_pool), int(round(cap * minor_frac)))
    n_major = min(cap - n_minor, len(major_pool))
    picked = _sample_names(rng, minor_pool, n_minor) + _sample_names(rng, major_pool, n_major)
    if len(picked) < cap:
        rest = [n for n in (major_pool + minor_pool) if n not in picked]
        picked = picked + _sample_names(rng, rest, cap - len(picked))
    return picked


# A connected component cut from a photogrammetry spread is often an OPEN
# SHELL a few centimetres thick — the scanned top of a piece, not the piece.
# Scattered with a stick-out tilt those render as sheets of foil standing on
# edge (round-5 v8b proof render). A "chunk" prototype must be chunky.
HD_CHUNK_MIN_ASPECT = float(os.environ.get("RUBBLE_HD_CHUNK_MIN_ASPECT", "0.22"))
HD_CHUNK_MIN_THICK_M = float(os.environ.get("RUBBLE_HD_CHUNK_MIN_THICK_M", "0.07"))


def _chunky(size):
    lo, hi = min(size), max(size)
    return hi > 0 and lo >= HD_CHUNK_MIN_THICK_M and (lo / hi) >= HD_CHUNK_MIN_ASPECT


# Open-edge fraction per HD piece (`assets/rubble_hd/open_frac.json`, written
# by a census over the 840 files: boundary edges / all edges). A connected
# component of a PILE SCAN is a surface patch: median 0.23 open, only 38 of
# 840 under 0.10. Rendered as loose pieces they are foil with holes (v8c
# proof render), so the chunk/flake classes are restricted to the
# near-closed pieces and the WHOLE spreads (which do read as rubble) carry
# the coverage as `cluster` instances (`CLUSTER_COVERAGE`).
HD_CHUNK_MAX_OPEN = float(os.environ.get("RUBBLE_HD_CHUNK_MAX_OPEN", "0.15"))
HD_FLAKE_MAX_OPEN = float(os.environ.get("RUBBLE_HD_FLAKE_MAX_OPEN", "0.30"))


def _hd_open_frac(name):
    return float(HD_OPEN_FRAC.get(name, 1.0))


def _sample_names(rng, pool, cap):
    """Up to `cap` names from `pool`, drawn without replacement via the
    caller's own `rng` (a `random.Random`) so the draw is reproducible for a
    given seed; the whole pool if it's already <= `cap`."""
    pool = list(pool)
    if not pool:
        return []
    if len(pool) <= cap:
        return pool
    return rng.sample(pool, cap)


def _sample_mixed_names(rng, kind, major_material, minor_material, minor_frac, cap):
    """`cap` names of `kind`, drawing `minor_frac` (rounded, capped by
    availability) from `minor_material`'s pool and the rest from
    `major_material`'s pool — e.g. a URM chunk/flake set: mostly brick with
    a ~20% minority of concrete (lintels, floor fragments). Tops up from
    whichever pool has names left if either pool alone can't fill its share
    (a tiny minority pool should not silently shrink the total draw)."""
    major_pool = _hd_names_by(kind, (major_material,))
    minor_pool = _hd_names_by(kind, (minor_material,))
    n_minor = min(cap, len(minor_pool), int(round(cap * minor_frac)))
    n_major = min(cap - n_minor, len(major_pool))
    picked = _sample_names(rng, minor_pool, n_minor) + _sample_names(rng, major_pool, n_major)
    if len(picked) < cap:
        rest = [n for n in (major_pool + minor_pool) if n not in picked]
        picked = picked + _sample_names(rng, rest, cap - len(picked))
    return picked


def _warn_no_hd_catalogue():
    global _HD_FALLBACK_WARNED
    if not _HD_FALLBACK_WARNED:
        print("[quake_rubble] HD_CATALOGUE is empty (assets/rubble_hd/catalogue.json "
              "missing or unreadable) -> falling back to the flat-colour PROTO_SETS "
              "catalogue for every pile this process plans.")
        _HD_FALLBACK_WARNED = True


# ---------------------------------------------------------------------------
# round-5 addendum: `_hp` opt-in. User review: "I don't see a lot of the
# concrete debris being used ... use that more (the ones on nucleus)" named
# the `_hp` high-poly twins specifically (`brick_debris_pile_hp`,
# `concrete_debris_elements_hp` — see CATALOGUE above, "flagged, never
# instanced"). `RUBBLE_HP=1` (env, default OFF) makes `_select_proto_sets`
# PREFER a name's `_hp` twin, in every instanced category, whenever one
# exists AND actually resolves. Off by default: the round-4 catalogue note
# still holds unless a caller opts in.
#
# THE MISSING-ASSET TRAP THIS GUARDS AGAINST: `quake_rubble_usd.
# _resolve_asset_url` does not check whether a name's file exists — it just
# joins `asset_root` + the catalogue `url` and hands the result to
# `AddReference`. A reference to a file that is not there resolves to
# NOTHING (an empty, invisible prototype) with no error and no fallback —
# `tools/nucleus_fetch.py`'s local mirror explicitly SKIPS every `_hp`
# sibling (see the module docstring's catalogue table: "skipping `*_hp`
# siblings"), so turning this on against a local `RUBBLE_ASSET_ROOT` mirror
# would otherwise draw invisible prototypes. The SELECTION side is what has
# to be robust instead: `_hp_resolvable` probes the filesystem directly for
# a LOCAL asset_root (not a URL scheme) and only offers the `_hp` name when
# the file is actually there; for an `omniverse://` root (the real default —
# see `quake_rubble_usd.ASSET_ROOT`) or no root at all, this module cannot
# reach Nucleus to check, so it trusts the catalogue.
# ---------------------------------------------------------------------------
RUBBLE_HP = os.environ.get("RUBBLE_HP", "0").strip().lower() not in (
    "0", "", "false", "no")


def _hp_resolvable(rel_url, asset_root):
    """Will `rel_url` (a catalogue entry's `url`, relative to `asset_root`)
    actually resolve? For a LOCAL `asset_root` (no `omniverse://` scheme,
    and not empty/`None`), probe the filesystem directly. For an
    `omniverse://` root, or no root given at all (the un-set-env default,
    which `quake_rubble_usd.ASSET_ROOT` itself resolves to the Nucleus
    `omniverse://` URL), trust the catalogue — there is no cheap way for
    this pxr-free module to check a Nucleus path."""
    if not asset_root or str(asset_root).startswith("omniverse://"):
        return True
    local = os.path.join(str(asset_root), str(rel_url or "").replace("/", os.sep))
    return os.path.exists(local)


def _prefer_hp(names, asset_root):
    """`names`, each swapped for its `_hp` twin when `CATALOGUE` has one
    (`name + "_hp"`, flagged `hp: True`) that `_hp_resolvable` confirms —
    never a bare guess. Returns `(new_names, any_hp_used)`; `new_names` is
    always a NEW list (the caller's own list is never mutated in place)."""
    out = []
    any_hp = False
    for n in names:
        hp_name = str(n) + "_hp"
        entry = CATALOGUE.get(hp_name)
        if entry and entry.get("hp") and _hp_resolvable(entry.get("url"), asset_root):
            out.append(hp_name)
            any_hp = True
        else:
            out.append(n)
    return out, any_hp


def _apply_hp_preference(proto_sets, asset_root):
    """A NEW `proto_sets`-shaped dict with `_prefer_hp` applied to every
    instanced category (chunk/flake/raft/toe/cluster) — called ONLY when
    `RUBBLE_HP` is truthy, so the default-off path never copies or touches
    `proto_sets` at all (preserving, e.g., the empty-`HD_CATALOGUE`
    fallback's `is PROTO_SETS[btype]` identity when the knob is off)."""
    ar = asset_root if asset_root is not None else os.environ.get("RUBBLE_ASSET_ROOT")
    out = dict(proto_sets)
    for k in ("chunk", "flake", "raft", "toe", "cluster"):
        names = out.get(k)
        if names:
            out[k], _used = _prefer_hp(names, ar)
    return out


def _select_proto_sets(btype, rng, asset_root=None):
    """Per-pile chunk/flake/raft/toe prototype pools. `cluster` (whole FAB
    spreads) is untouched either way — it always comes straight from
    `PROTO_SETS[btype]["cluster"]`.

    HD_CATALOGUE non-empty (the normal case once the library is built):
      * urm chunk/flake: brick majority + `HD_URM_MINORITY_FRAC` concrete
        minority (lintels/floor fragments), capped per `HD_PROTO_CAP`;
      * rc/rc_glass chunk/flake: concrete only, capped;
      * raft (rc/rc_glass only — urm keeps raft=[], "no concrete rafts"):
        the HD `raft` pieces (22, both materials) UNION the original
        authored `_RAFTS` (`slab_01..12`) pool, capped;
      * toe: HD `street` pieces (121, concrete), capped.
    Any one of those draws coming back empty (a material filter with
    nothing to draw from) falls back to that ONE category's original
    flat-colour list rather than leaving `plan_pile` with an empty
    `rng.choice()` pool.

    HD_CATALOGUE empty: returns `PROTO_SETS[btype]` unchanged (every
    category — chunk/flake/raft/toe/cluster — the pre-round-5 flat-colour
    behaviour), after printing one warning line (see `_warn_no_hd_
    catalogue`).

    Returns `(proto_sets, used_hd)`. `asset_root`: passed to `_hp_
    resolvable` when `RUBBLE_HP` is on (see the module note above); `None`
    reads `RUBBLE_ASSET_ROOT` from the environment (the same knob
    `quake_rubble_usd.ASSET_ROOT` reads).
    """
    base = PROTO_SETS[btype]
    if not HD_CATALOGUE:
        _warn_no_hd_catalogue()
        out, used_hd = base, False
    else:
        out = dict(base)
        if btype == "urm":
            out["chunk"] = _sample_mixed_names(rng, "chunk", "brick", HD_URM_MINORITY_MATERIAL,
                                               HD_URM_MINORITY_FRAC, HD_PROTO_CAP["chunk"])
            out["flake"] = _sample_mixed_names(rng, "flake", "brick", HD_URM_MINORITY_MATERIAL,
                                               HD_URM_MINORITY_FRAC, HD_PROTO_CAP["flake"])
            # raft stays [] -- "URM has timber floors: no concrete rafts" (unchanged design note)
        else:
            out["chunk"] = _sample_names(rng, _hd_names_by("chunk", ("concrete",)), HD_PROTO_CAP["chunk"])
            # round-5 "more debris" addendum: lump_01-06 (measured open_frac
            # 0.0-0.05, comfortably under HD_FLAKE_MAX_OPEN) join the HD
            # flake pool as a bounded ~20% minority -- see
            # `_standalone_names_by`'s comment above (chunk_01-09 do NOT get
            # the same treatment here: they fail this pool's own stricter
            # 0.15 open-fraction gate, and only qualify for the LOOSER
            # street gate below).
            out["flake"] = _sample_pool_with_minority(
                rng, _hd_names_by("flake", ("concrete",)),
                _standalone_names_by("flake", ("concrete",)),
                STANDALONE_FLAKE_MINORITY_FRAC, HD_PROTO_CAP["flake"])
            if base["raft"]:
                raft_pool = list(base["raft"]) + _hd_names_by("raft", ("brick", "concrete"))
                out["raft"] = _sample_names(rng, raft_pool, HD_PROTO_CAP["raft"])
        if base["toe"]:
            out["toe"] = _sample_names(rng, _hd_names_by("street", ("concrete",)), HD_PROTO_CAP["toe"])
        # cluster = the whole scanned spreads (round-5 v8c: the coverage class).
        # A brick pile draws its own brick spread most of the time with the
        # concrete spreads as a minority (floors, lintels, the odd slab); a
        # concrete pile draws every concrete spread. Weighted by repetition so
        # `rng.choice` keeps its uniform draw.
        spreads = [n for n, e in CATALOGUE.items() if e.get("kind") == "spread" and not e.get("hp")]
        brick_sp = [n for n in spreads if CATALOGUE[n].get("material") == "brick"]
        conc_sp = [n for n in spreads if CATALOGUE[n].get("material") != "brick"]
        if btype == "urm" and brick_sp:
            # round-5 addendum ("use the scanned concrete debris more"): within
            # a urm pile's minority concrete share, the big FAB spread
            # (`huge_concrete_rubble_pile`) and the two cheap Quixel piles
            # (`concrete_rubble_pile`, `rocky_ground`) repeat more than the
            # other two concrete spreads — repetition is the same weighting
            # `rng.choice` already uses for brick vs. concrete here.
            conc_sp_urm = []
            for n in conc_sp:
                conc_sp_urm += [n] * _URM_CONC_MINORITY_BOOST.get(n, 1)
            out["cluster"] = brick_sp * max(1, int(round(len(conc_sp_urm) * 1.2))) + conc_sp_urm
        elif conc_sp:
            out["cluster"] = list(conc_sp)

        for k in ("chunk", "flake", "raft", "toe"):
            if base.get(k) and not out.get(k):
                out[k] = base[k]
        used_hd = True

    if RUBBLE_HP:
        out = _apply_hp_preference(out, asset_root)
    return out, used_hd


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

# round-5 review: HD debris pieces are already natural-sized (median ~0.33 m
# longest side), so a fixed per-m^2 COUNT (the old `CHUNK_DENSITY`) doesn't
# adapt when the mean footprint area of whatever prototypes got drawn this
# pile changes — a pile of small HD chunks needs many more instances per m^2
# than a pile of the old ~0.9 m flat boxes to cover the same fraction of
# surface. `COVERAGE` is a TARGET FRACTION of local surface area the class's
# own footprints should cover (same crown/mid/toe piecewise shape the old
# density table used — see `_coverage_frac`); density-per-m^2 is derived at
# plan time from `COVERAGE[zone] / mean_footprint_area` (`_mean_footprint_
# area`, `_zone_count_estimate`). >1 is intentional: the crown is meant to
# be fully hidden under OVERLAPPING pieces, not just sparsely dotted (the
# tornado skill's "loose material on a crest, at more than one size,
# overlapping, is what makes earth read as earth" lesson).
COVERAGE = {"crown": 0.45, "mid": 0.30, "toe": 0.15}   # round-5 v8c: chunks are accents over the cluster layer (was 1.1/0.7/0.35)
# Hard ceiling on TOTAL instances (chunk+flake+cluster+toe) per pile, env-
# overridable — a real building-scale dome's coverage-derived chunk/flake
# estimate can run into the tens of thousands once mean footprint area drops
# to HD-piece scale; this is the safety valve, applied by scaling every
# zone's count down by the SAME ratio (so crown stays denser than toe, just
# smaller overall) rather than truncating one class or one zone outright.
RUBBLE_MAX_INSTANCES = int(os.environ.get("RUBBLE_MAX_INSTANCES", "6000"))
# The cap SCALES WITH THE PILE (round-5 v7 proof renders): a flat 6000 on a
# 22 x 18 m building's dome (~900 m^2 of pile) delivered 0.16-0.45 x the
# coverage target and the fines tile showed through everywhere, while the
# same table on a 30 m windrow was fine. The effective cap is
# max(RUBBLE_MAX_INSTANCES, RUBBLE_INSTANCES_PER_M2 x pile area) capped at
# RUBBLE_MAX_INSTANCES_CEIL. Flakes are a FILL class: they target
# FLAKE_COVERAGE_FRAC of the chunk coverage table (they are ~4 x smaller in
# footprint, so at equal coverage they would eat 4 x the instance budget
# for a quarter of the visual mass) and are trimmed FIRST when the cap
# bites (never below half the chunk count).
RUBBLE_INSTANCES_PER_M2 = float(os.environ.get("RUBBLE_INSTANCES_PER_M2", "14"))
RUBBLE_MAX_INSTANCES_CEIL = int(os.environ.get("RUBBLE_MAX_INSTANCES_CEIL", "24000"))
FLAKE_COVERAGE_FRAC = float(os.environ.get("RUBBLE_FLAKE_COVERAGE_FRAC", "0.30"))
CLUSTER_N = (3, 10)                # round4 plan design table (layer 3) — rc_glass default
# round-5 addendum ("I don't see a lot of the concrete debris being used"):
# a windrow/fan (out-of-plane fan, soft-storey collar) draws exactly
# `n_cluster_min` clusters, no coverage boost — CLUSTER_N was everyone's
# floor. rc reads bare there next to a real rc dome's much denser crown, so
# rc gets its own, higher floor; rc_glass (cladding-only sheds, a much
# smaller pile per CROWN_FRAC) keeps the original CLUSTER_N.
CLUSTER_N_RC = (6, 14)
RUNOUT_CHUNK_FRAC = (0.08, 0.15)   # round4 brief — share of chunks landing beyond the toe
RUNOUT_CHUNK_REACH_MULT = 1.4      # round4 brief — up to 1.4x reach

BURY = {
    "raft": (0.10, 0.30),          # rubble_research.md sec3b / "Constants" table (was 0.15-0.45)
    # round-5 v8 proof render: at 0.30-0.60 a median HD chunk (0.26 m) sat
    # with its top 9 cm proud and 37 % of all instances were fully under the
    # fines — the mound read BARE at 11k instances. Shallower now; the fines
    # layer is a proxy, the visible pieces are the pile.
    "chunk": (0.12, 0.35),         # was 0.30-0.60 (rubble_research.md sec3b)
    "cluster": (0.25, 0.50),       # round4 brief (not separately measured — kept)
    "flake": (0.0, 0.12),          # was 0.0-0.30 (round4 brief)
    "panel": (0.20, 0.50),         # round4 brief
    # extra categories needed for authored/asset "large" elements the brief's
    # BURY dict didn't cover (added, not part of the original 5-key table):
    "column": (0.35, 0.55),
    "lintel": (0.10, 0.40),
    "joist": (0.20, 0.50),
    "rebar": (0.50, 0.80),         # rubble_research.md sec3b — "rebar tangles / small debris"
}

PANEL_LEAN_DEG = (50.0, 80.0)      # rubble_research.md sec3b (was 55-80, then
#                                    30-70) — round-6 diagnosis, rect-cutouts
#                                    review 2026-08-31: my first attempt at
#                                    this fix LOWERED the range to (10, 35)
#                                    (matching `PANEL_TILT_DEG`'s numbers) on
#                                    the assumption that a smaller number
#                                    reads as "less lean". MEASURED that
#                                    assumption is backwards before touching
#                                    the render: `_orient_on_surface` aligns
#                                    local +Z (the panel's HEIGHT axis, since
#                                    it is placed the way it stood in the
#                                    wall) to the mound's surface normal
#                                    FIRST, then tips it `tilt` degrees
#                                    further FROM THAT ALIGNMENT — so 0 deg
#                                    is standing bolt upright and 90 deg is
#                                    flat on its face; the number counts
#                                    degrees tipped AWAY FROM STANDING, not
#                                    degrees propped up from lying. 150-seed
#                                    measurement of the REAL `plan_pile` (a
#                                    22x18x20 m rc dome, stub_h_m=1.4, a
#                                    throwaway harness reusing `tools/
#                                    rubble_preview.py`'s idiom),
#                                    angle from vertical of the panel's own
#                                    up axis: old (30,70) -> mean 53 deg;
#                                    (10,35) -> mean 32 deg — MORE upright,
#                                    the wrong direction, would have made
#                                    "stands with a perfect rectangle
#                                    outline" worse. (50,80) -> mean 67 deg
#                                    (21 deg off horizontal), matching a
#                                    physical cross-check (a 3 m panel
#                                    resting its far edge on a 1.4 m stub
#                                    leans arccos(1.4/3.0) ~ 62 deg from
#                                    vertical) and reading as clearly LYING,
#                                    propped rather than standing — the
#                                    round-4 "pile lays panels whole" intent,
#                                    just no longer upright. This is closer
#                                    to the original (55,80) than to the
#                                    (30,70) it replaces; whatever motivated
#                                    that earlier reduction was not this
#                                    artefact.
PANEL_TILT_DEG = (10.0, 35.0)      # round4 brief — general (non-leaning) panel tilt
RAFT_TILT_DEG = (0.0, 25.0)        # round4 brief
RAFT_CROWN_TILT_DEG = (5.0, 25.0)  # round4 brief — crown-group subset
# round-5: HD chunks/flakes are already natural-sized real pieces (unlike
# the old flat catalogue's oversized boxes, which needed the old power-law
# chunk_scale() to shrink them down) — a modest scale draw around 1.0 is
# enough size variety; also used (see `_mean_footprint_area`) to fold the
# scale draw's own E[scale^2] into the coverage->density conversion.
CHUNK_SCALE_RANGE = (1.0, 2.0)     # round-5 v8: HD chunks are 0.26 m median at natural size — x1-2 puts the class at 0.3-0.9 m, the wall/floor-fragment scale a pile is made of (was 0.75-1.35)
CLUSTER_SCALE = (0.6, 1.4)         # round-5: was 0.8-1.3
CLUSTER_COVERAGE = float(os.environ.get("RUBBLE_CLUSTER_COVERAGE", "1.6"))   # dome: fraction of pile area under whole-spread clusters
# round-5 addendum: rc/rc_glass piles get a further boost over the shared
# 1.6 base — the user's read on the first OSMO city scene was specifically
# that not enough of the scanned concrete debris was visible, and an rc pile
# is where every cluster prototype (`concrete_debris_elements`,
# `huge_concrete_rubble_pile`, the two Quixel piles) is already concrete, so
# raising the target here (rather than urm's mostly-brick pool) puts the
# extra coverage on the piles that read the most like poured concrete.
CLUSTER_COVERAGE_RC = float(os.environ.get("RUBBLE_CLUSTER_COVERAGE_RC", "2.1"))
CLUSTER_MAX = int(os.environ.get("RUBBLE_CLUSTER_MAX", "90"))
CLUSTER_MAX_RC = int(os.environ.get("RUBBLE_CLUSTER_MAX_RC", "115"))
RAFT_SCALE_RANGE = (0.9, 1.1)      # round4 brief
FLAKE_SCALE = (0.8, 1.5)           # round-5 v8: was (0.6, 1.2)
FLAKE_STICKOUT_FACTOR = float(os.environ.get("RUBBLE_FLAKE_STICKOUT", "0.3"))   # flakes lie flat; a standing flake is foil

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
    # every rng draw happens ONCE, up here, so the adaptive rebuild below
    # reproduces the same pile on a bigger grid instead of re-rolling it
    wl4, amp4 = rng.uniform(*RELIEF_WAVELENGTH_4), rng.uniform(*RELIEF_AMP_4)
    wl5, amp5 = rng.uniform(*RELIEF_WAVELENGTH_5), rng.uniform(*RELIEF_AMP_5)

    # THE DOMAIN. In g-space the lobed toe is at r = 2 (1 + mod), mod up to
    # 1.4 amp, and `_dome_gxy` maps r = 1 to the wall and r = 2 to wall +
    # reach, so the widest lobe sits reach x (2 (1 + 1.4 amp) - 1) beyond
    # the wall. But the repose relaxation below (`_limit_slope`, a Jacobi
    # diffusion) then SPREADS the foot past that nominal toe — on a blind
    # side with a 1.5-3 m reach the design ramp is far steeper than repose,
    # and the relaxed foot runs 4-6 m out — so the grid is grown until the
    # row next to its boundary is back at the lip. Before this the boundary
    # clamp cut the foot off at a straight line: a 0.2-0.6 m CLIFF on a
    # rectangle round every pile (the straight-edged piles of the first
    # Isaac captures, r4_commercial 2026-08-30). With the foot fully inside
    # the domain, `_trim_flat` leaves the real outline as the mesh outline.
    r_toe_max = 2.0 * (1.0 + 1.4 * lobe["amp"])
    margin = 2.0 * GRID_CELL_M
    ext = {s: reach[s] * (r_toe_max - 1.0) + margin for s in _SIDES}
    for _attempt in range(6):
        cell = _dome_grid_once(m, reach, crown_target, lobe, ext, nrng,
                               (wl4, amp4, wl5, amp5), stub_h_m, plate_ok, fall_sides)
        h = cell["height"]
        tol = MOUND_LIP_M + 0.02
        grow = {"W": bool((h[:, 1] > tol).any()), "E": bool((h[:, -2] > tol).any()),
                "S": bool((h[1, :] > tol).any()), "N": bool((h[-2, :] > tol).any())}
        if not any(grow.values()):
            break
        for s_, g_ in grow.items():
            if g_:
                ext[s_] = ext[s_] * 1.5 + margin
    cell["reach"] = reach
    cell["crown_target"] = crown_target
    cell["lobe"] = lobe
    cell["ext"] = ext
    return cell


def _dome_grid_once(m, reach, crown_target, lobe, ext, nrng, relief, stub_h_m, plate_ok, fall_sides):
    """One build of the dome heightfield on the domain `ext` (metres beyond
    each wall) — see `_build_dome_grid` for the adaptive loop round it."""
    W, D = float(m["W"]), float(m["D"])
    halfW, halfD = W / 2.0, D / 2.0
    wl4, amp4, wl5, amp5 = relief
    xmin, xmax = -halfW - ext["W"], halfW + ext["E"]
    ymin, ymax = -halfD - ext["S"], halfD + ext["N"]
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
    # Octaves 4-5: real 1-2 m surface lumps (round-2 review — the mound read
    # as a smooth dune). Absolute-metre wavelength/amplitude, masked to 0 at
    # the toe / anywhere the pile (pre-fine-relief) is under ~0.6 m tall.
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
    # Same mapping as the mound (see `_build_dome_grid`): the apron's outer
    # toe is at r = mult x 2 (1 + mod), i.e. reach x (2 mult (1 + 1.4 amp) - 1)
    # beyond the wall at the widest lobe; the old `mult (1 + 1.4 amp)` domain
    # ended inside it and the skirt was clipped to a rectangle (the pale
    # "plaza" under every DG5 in the first Isaac captures).
    ext = 2.0 * mult * (1.0 + 1.4 * lobe["amp"]) - 1.0
    margin = 2.0 * 0.75
    ereach = {s: reach[s] * ext + margin for s in _SIDES}
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



def extent_by_side(m, points):
    """How far the (trimmed) mound mesh ACTUALLY runs past each wall line,
    {S, E, N, W} in metres, measured from world-space `points` in the mass's
    own yaw frame. `reach_m` is the NOMINAL run-out the planner asked for;
    after the repose relaxation a blind side's foot runs 4-6 m out (see
    `_build_dome_grid`), and `quake._clear_under_heaps` — which removes
    trees / tips lamps / buries cars under the pile — must know where the
    pile really ended (2026-08-30 eq500_gui: street trees standing inside
    the blind-side foot of a brownstone DG5 because clearance used the 3 m
    nominal blind reach). 0 on a side the mesh does not pass."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    out = {"S": 0.0, "E": 0.0, "N": 0.0, "W": 0.0}
    if pts.shape[0] == 0:
        return out
    a = math.radians(float(m.get("yaw", 0.0)))
    ca, sa = math.cos(a), math.sin(a)
    dx = pts[:, 0] - float(m["cx"])
    dy = pts[:, 1] - float(m["cy"])
    lx = dx * ca + dy * sa
    ly = -dx * sa + dy * ca
    halfW, halfD = float(m["W"]) / 2.0, float(m["D"]) / 2.0
    out["E"] = round(max(0.0, float(lx.max()) - halfW), 2)
    out["W"] = round(max(0.0, -float(lx.min()) - halfW), 2)
    out["N"] = round(max(0.0, float(ly.max()) - halfD), 2)
    out["S"] = round(max(0.0, -float(ly.min()) - halfD), 2)
    return out

def _trim_flat(pts, faces, eps=0.02):
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

    # floor at the lip, never 0 (a windrow floor exactly at z0 z-fights the
    # ground plate the same way a mound rim did — round-4 review)
    height = np.maximum(height, MOUND_LIP_M)
    cell = {"X": X, "Y": Y, "height": height, "xs": xs, "ys": ys,
            "x0": xr[0], "y0": yr[0], "dx": xs[1] - xs[0] if nx > 1 else 1.0,
            "dy": ys[1] - ys[0] if ny > 1 else 1.0, "nx": nx, "ny": ny,
            "d": D_, "t": T, "near": near}
    # the far edge fades to the lip over the last metre (the
    # design taper reaches 0 only AT the boundary row, so the row inside it
    # could still carry 0.2 m x `noise_mult` — a straight 20 cm step along
    # the fan's far edge in the r4_commercial out_of_plane capture)
    far = {"row0": "row-1", "row-1": "row0", "col0": "col-1", "col-1": "col0"}[near]
    _taper_to_boundary(cell, MOUND_LIP_M, width_m=1.0, edges=(far,))   # ends keep their own end_taper
    height = cell["height"]
    if near != "row0": height[0, :] = MOUND_LIP_M
    if near != "row-1": height[-1, :] = MOUND_LIP_M
    if near != "col0": height[:, 0] = MOUND_LIP_M
    if near != "col-1": height[:, -1] = MOUND_LIP_M
    return cell


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



def _taper_to_boundary(cell, lip, width_m=1.5, edges=None):
    """Fade a cell's height to `lip` over the outer `width_m` of its grid on
    every edge that is not the wall-facing `near` edge of a strip. Whatever
    the repose relaxation, a lobe bulge or a shoulder bump left against the
    domain boundary becomes a ramp instead of a cliff (a 0.2 m residual over
    1.5 m is 7.6 deg — invisible; a 0.2 m vertical step on a straight line
    is the "rectangle" of the first Isaac captures). Runs after everything
    that can raise the surface, so `_trim_flat` finds a flat ring to cut."""
    h = cell["height"]
    ny, nx = h.shape
    dx, dy = float(cell["dx"]), float(cell["dy"])
    near = cell.get("near")
    big = 1e9
    i = np.arange(nx, dtype=np.float64)[None, :]
    j = np.arange(ny, dtype=np.float64)[:, None]
    # distance measured from ONE CELL inside the boundary, so the outer two
    # rows both sit at the lip and the faces between them are exactly flat
    # (what `_trim_flat` cuts); the ramp starts on the third row
    d = np.full(h.shape, big)
    use = (lambda e: e != near) if edges is None else (lambda e: e in edges)
    if use("col0"):
        d = np.minimum(d, (i - 1.0) * dx)
    if use("col-1"):
        d = np.minimum(d, (nx - 2 - i) * dx)
    if use("row0"):
        d = np.minimum(d, (j - 1.0) * dy)
    if use("row-1"):
        d = np.minimum(d, (ny - 2 - j) * dy)
    w = np.clip(d / max(width_m, 1e-6), 0.0, 1.0)
    cell["height"] = lip + (h - lip) * w
    return cell

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
# round-5: coverage -> density math, and the total-instance hard cap.
# ---------------------------------------------------------------------------

def _coverage_frac(rel):
    """Piecewise crown/mid/toe TARGET COVERAGE (fraction of local surface
    area a size class's own footprints should cover), continuous in `rel` =
    height/crown_actual in [0, 1] — the same crown-plateau / flank-taper
    shape the pre-round-5 density table used, now interpreted as an area
    fraction instead of a raw per-m^2 count (see `COVERAGE`'s docstring
    comment)."""
    crown, mid, toe = COVERAGE["crown"], COVERAGE["mid"], COVERAGE["toe"]
    return np.where(rel >= 0.5, mid + (crown - mid) * (rel - 0.5) / 0.5,
                     toe + (mid - toe) * (rel / 0.5))


def _mean_footprint_area(names, scale_range):
    """Mean footprint area (m^2) of `names` (catalogue/HD entries), each
    scaled by an INDEPENDENT draw from `scale_range` — folds in
    `E[scale^2]` (the closed-form second moment of `Uniform(a, b)`) rather
    than just using the range's midpoint, since footprint area scales with
    scale SQUARED. Returns a small positive floor (never 0) so a caller can
    always safely divide a coverage target by this."""
    areas = []
    for name in names:
        e = _asset_entry(name)
        if not e:
            continue
        sx, sy = e["size"][0], e["size"][1]
        areas.append(max(sx * sy, 1e-6))
    if not areas:
        return 1.0
    a, b = scale_range
    e_scale2 = (a * a + a * b + b * b) / 3.0
    return max((sum(areas) / len(areas)) * e_scale2, 1e-6)


def _zone_count_estimate(cells, crown_actual, mean_area):
    """Numeric double-integral (same grid the mound mesh itself uses, not a
    separate resolution) of `_coverage_frac(rel) / mean_area` over every
    cell's footprint area — the TARGET instance count for one size class,
    before any hard-cap scaling. 0 if `crown_actual` or `mean_area` is
    non-positive (nothing to place, or no resolvable prototype)."""
    if crown_actual <= 0 or mean_area <= 0:
        return 0.0
    n_est = 0.0
    for c in cells:
        rel = np.clip(c["height"] / crown_actual, 0.0, 1.0)
        dens = _coverage_frac(rel) / mean_area
        n_est += float(np.sum(dens[:-1, :-1]) * c["dx"] * c["dy"])
    return n_est


def _trim_instances_to_cap(instances, cap):
    """Scale every instance set's arrays down by the SAME ratio so the
    combined total across all sets is <= `cap` (round-4's per-budget
    trimmer, promoted to a reusable top-level function so the round-5 hard
    default cap and an explicit caller `budget["n_instances"]` share one
    implementation). Independent per-set rounding can overshoot `cap` by a
    few; the overshoot is trimmed from whichever sets are currently largest.

    Returns `(total_before, total_after, capped)`; a no-op (returns the
    original total twice, `capped=False`) when `cap` is falsy or the total
    is already within it.
    """
    total = sum(len(v["positions"]) for v in instances.values())
    cap = int(cap) if cap else 0
    if cap <= 0 or total <= cap:
        return total, total, False
    ratio = cap / float(total)
    keeps = {k: int(round(len(v["positions"]) * ratio)) for k, v in instances.items()}
    over = sum(keeps.values()) - cap
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
    total_after = sum(len(v["positions"]) for v in instances.values())
    return total, total_after, True


# ---------------------------------------------------------------------------
# plan_pile — the API contract entry point.
# ---------------------------------------------------------------------------

def plan_pile(m, btype, rng, kind="dome", crown_m=None, spread_frac=None,
              sides=None, along=None, depth_m=None, offset_m=0.0,
              plate_ok=None, stub_h_m=0.0, panels=(), budget=None, seed_tag="",
              elem_h_m=None, asset_root=None):
    """Plan a rubble pile for one collapsed mass (or shed wall) `m`.

    `elem_h_m` (added, not in the original contract text): the height of the
    specific wall/parapet element that fell, for `kind in ("windrow","fan")`
    — eq_round4_rubble_research.md sec5b: reach ~= 1.0x that element's own
    height, not `spread_frac x H` of the whole building. When given it
    overrides `spread_frac` for windrow/fan; `spread_frac` still works as a
    fallback (or for callers that don't know the failed element's height).

    `asset_root` (round-5 addendum): forwarded to `_select_proto_sets` for
    the `RUBBLE_HP` opt-in's local-file probe; `None` (every existing
    caller) reads `RUBBLE_ASSET_ROOT` from the environment instead.
    """
    nrng = np.random.default_rng(rng.getrandbits(32))
    z0 = float(m["z0"])
    Hbldg = max(0.1, float(m["top"]) - z0)
    storeys = max(1, len(m.get("levels", [z0])))
    btype = btype if btype in PROTO_SETS else "rc"
    look = "urm" if btype == "urm" else "rc"
    proto_sets, using_hd = _select_proto_sets(btype, rng, asset_root=asset_root)
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
        # BURY IN PIECE-HEIGHT METRES, NOT ROTATED-EXTENT METRES. A big
        # planar FAB spread tilted with a steep windrow flank shows metres
        # of ROTATED thickness (6 m x sin 60 deg ~ 5 m), and `bury x
        # thickness` sank clusters 5 m below grade in the first GAC pilot
        # bake (gac_SM_Building_02_DG4_s7: instancer zmin -5.18). The piece
        # sits ON the pile sunk by a fraction of its own unrotated height —
        # the tilt changes its silhouette, not how deep it lies.
        bury_m = bury * min(thickness, size[2] * scale * 1.5 + 0.30)
        origin_z = z0 + h - zmin_rel - bury_m
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
        size = _asset_entry(name)["size"]
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
        size = _asset_entry(name)["size"]
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
        size = _asset_entry(name)["size"]
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
    # round-5: an HD prototype already carries its OWN real per-piece
    # material/texture (mixed brick/concrete for urm) — a uniform per-set
    # override would erase that variety and undo the whole point of drawing
    # from the HD library, so only the OLD flat-colour catalogue (no real
    # per-piece texture, every flat chunk/lump is `material: "concrete"`
    # regardless of building type) still needs the generic construction-
    # type override.
    chunk_flake_look = None if using_hd else ("brick" if btype == "urm" else "concrete")
    instances = {"chunk": _empty_instance_set(chunk_flake_look),
                 "flake": _empty_instance_set(chunk_flake_look),
                 "cluster": _empty_instance_set(None), "toe": _empty_instance_set(None)}

    xmin_all, xmax_all, ymin_all, ymax_all = _cells_bounds(cells)

    # round-5: the final assembly pass below (`_taper_to_boundary`, called
    # AFTER every instance here is placed) fades a DOME cell's height to the
    # ground lip over the outer `width_m=1.5` m of the grid — a few cm of
    # legitimate pre-taper height there (the adaptive domain's own tolerance
    # is only `MOUND_LIP_M + 0.02`, plus whatever relief noise survives) is
    # real height at PLACEMENT time but gets shaved to ~0 afterward, which
    # floats anything already seated in that band. Under the old sparse
    # density this band's tiny area × low instance count meant it almost
    # never got hit; the round-5 flake count (thousands, and `flake_weight`
    # itself favours LOW rel — i.e. exactly this near-zero-height band) hits
    # it often enough that a few pinned test seeds now float a handful of
    # flakes (measured: 6/~6000 on one seed, all in the outer band, all
    # <10 cm). Insetting the SCATTER sampling bbox (not `patch_factor`'s
    # lattice, which only needs to cover the same area either way) by the
    # taper width plus a cell-size safety margin keeps every reject-sampled
    # point out of the band `_taper_to_boundary` can still move. Windrow/fan
    # strips skip `_taper_to_boundary` entirely (only their exact outer
    # ring is re-clamped, not a ramp) so they keep the full bbox.
    if kind == "dome":
        _tm = 1.5 + 2.0 * GRID_CELL_M
        pxmin, pxmax = xmin_all + _tm, xmax_all - _tm
        pymin, pymax = ymin_all + _tm, ymax_all - _tm
        if pxmin >= pxmax or pymin >= pymax:
            pxmin, pxmax, pymin, pymax = xmin_all, xmax_all, ymin_all, ymax_all
    else:
        pxmin, pxmax, pymin, pymax = xmin_all, xmax_all, ymin_all, ymax_all

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

    def chunk_weight(x, y):
        h = None
        for c in cells:
            h = _cell_height_at(c, x, y)
            if h is not None:
                break
        if h is None:
            return None
        rel = _clip(h / max(crown_actual, 1e-6), 0.0, 1.0)
        base = float(_coverage_frac(rel))
        return base * patch_factor(x, y)

    stickout_prob = rng.uniform(*CHUNK_STICKOUT_PROB)

    def chunk_scale(_rng=rng):
        return _rng.uniform(*CHUNK_SCALE_RANGE)

    # round-5: TARGET COVERAGE -> density (`_zone_count_estimate`) replaces
    # the old fixed per-m^2 `CHUNK_DENSITY` table for BOTH chunk and flake —
    # each size class's mean footprint area (its OWN chosen prototypes at
    # their OWN scale draw, `_mean_footprint_area`) sets how many pieces it
    # takes to reach `COVERAGE`'s target fraction of surface area. The two
    # classes' raw (pre-cap) estimates are then scaled by the SAME ratio if
    # their combined total would exceed `RUBBLE_MAX_INSTANCES` (env-
    # overridable) or a caller-supplied `budget["n_instances"]`, whichever is
    # smaller — "scales all zones down proportionally", not a per-class or
    # per-zone truncation, since both classes' densities share the identical
    # crown/mid/toe SHAPE (`_coverage_frac`) and only their absolute scale
    # differs.
    mean_area_chunk = _mean_footprint_area(proto_sets["chunk"], CHUNK_SCALE_RANGE)
    mean_area_flake = _mean_footprint_area(proto_sets["flake"], FLAKE_SCALE)
    n_est_chunk = _zone_count_estimate(cells, crown_actual, mean_area_chunk)
    n_est_flake = _zone_count_estimate(cells, crown_actual, mean_area_flake) * FLAKE_COVERAGE_FRAC

    # pile area above the lip, on the mound's own grid -> area-scaled cap
    pile_area_m2 = 0.0
    for c in cells:
        raised = (c["height"] > MOUND_LIP_M + 0.02)
        pile_area_m2 += float(np.sum(raised[:-1, :-1]) * c["dx"] * c["dy"])
    area_cap = int(min(RUBBLE_MAX_INSTANCES_CEIL,
                       max(RUBBLE_MAX_INSTANCES, RUBBLE_INSTANCES_PER_M2 * pile_area_m2)))
    user_cap = int(budget["n_instances"]) if (budget and budget.get("n_instances")) else None
    effective_cap = min(area_cap, user_cap) if user_cap is not None else area_cap
    total_est = n_est_chunk + n_est_flake
    if total_est > effective_cap > 0:
        # flakes give first, down to half the chunk count; then both scale
        n_est_flake = max(min(n_est_flake, effective_cap - n_est_chunk), 0.5 * n_est_chunk)
        total_est = n_est_chunk + n_est_flake
    pre_ratio = (effective_cap / total_est) if (total_est > effective_cap > 0) else 1.0

    n_chunks = int(_clip(round(n_est_chunk * pre_ratio) if n_est_chunk > 0 else 20, 20, effective_cap))

    n_runout = int(round(rng.uniform(*RUNOUT_CHUNK_FRAC) * n_chunks)) if kind == "dome" else 0
    n_primary = max(0, n_chunks - n_runout)
    wmax_chunk = COVERAGE["crown"] * CHUNK_PATCH_CONTRAST[1]
    for _ in range(n_primary):
        xy = _reject_xy(rng, pxmin, pxmax, pymin, pymax, chunk_weight,
                         wmax_chunk, m, plate_ok, 400)
        if xy is None:
            continue
        lx, ly = xy
        name = rng.choice(proto_sets["chunk"])
        size = _asset_entry(name)["size"]
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
        size = _asset_entry(name)["size"]
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
                x = rng.uniform(pxmin, pxmax)
                y = rng.uniform(pymin, pymax)
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
            size = _asset_entry(name)["size"]
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

    # round-5: coverage-derived count (see the chunk block above) instead of
    # the old fixed FLAKE_N=(300,600) range — `flake_weight`'s own toe-biased
    # spatial law (WHERE a flake lands) is unchanged.
    n_flake = int(_clip(round(n_est_flake * pre_ratio) if n_est_flake > 0 else 20, 20, effective_cap))
    for _ in range(n_flake):
        xy = _reject_xy(rng, pxmin, pxmax, pymin, pymax, flake_weight, 1.2,
                         m, plate_ok, 300)
        if xy is None:
            continue
        lx, ly = xy
        name = rng.choice(proto_sets["flake"])
        size = _asset_entry(name)["size"]
        # thin axis up (mostly): a flake LIES on the pile. `_random_unit_quat`
        # stood a third of them on edge — sheets of foil in the v8b render.
        quat = _chunk_orientation(rng, size, stickout_prob * FLAKE_STICKOUT_FACTOR)
        scale = rng.uniform(*FLAKE_SCALE)
        pos, _b = place(lx, ly, quat, size, scale, BURY["flake"])
        _append_instance(instances["flake"], name, pos, quat, scale)

    # round-2 review: URM was reading with only 2 brick piles — "5-10 on the
    # flanks AND at the toe", half-and-half. round-5 addendum: rc gets its
    # own, higher strip floor (`CLUSTER_N_RC`) — a windrow/fan draws exactly
    # this many clusters with no coverage boost below, and an rc pile read
    # bare there ("use the scanned concrete debris more"); rc_glass keeps
    # the original `CLUSTER_N` (a much smaller pile per CROWN_FRAC).
    if btype == "urm":
        n_cluster_min = rng.randint(*CLUSTER_N_URM)
    elif btype == "rc":
        n_cluster_min = rng.randint(*CLUSTER_N_RC)
    else:
        n_cluster_min = rng.randint(*CLUSTER_N)
    # round-5 v8c: the whole scanned spreads are the one debris class that
    # reads as rubble at every distance, so on a DOME they carry the
    # coverage — count from `CLUSTER_COVERAGE` of the pile area over the
    # chosen spreads' mean footprint; the first `n_cluster_min` get the
    # shoulder bump (sunk into the flank), the rest sit on the surface
    # (bump after bump overshoots repose — round-4 known gap). rc/rc_glass
    # use the higher `CLUSTER_COVERAGE_RC` target and `CLUSTER_MAX_RC`
    # ceiling (round-5 addendum) — every cluster prototype on an rc pile is
    # already concrete, so raising coverage there puts the extra scanned
    # debris on the piles that read the most like poured concrete.
    n_cluster = n_cluster_min
    if kind == "dome" and proto_sets["cluster"]:
        is_rc = btype in ("rc", "rc_glass")
        coverage = CLUSTER_COVERAGE_RC if is_rc else CLUSTER_COVERAGE
        cluster_max = CLUSTER_MAX_RC if is_rc else CLUSTER_MAX
        mean_area_cluster = _mean_footprint_area(proto_sets["cluster"], CLUSTER_SCALE)
        n_cluster = int(_clip(round(coverage * pile_area_m2 / max(mean_area_cluster, 1e-6)),
                              n_cluster_min, cluster_max))
    n_cluster_toe = n_cluster_min // 2

    def cluster_weight(x, y):
        # uniform over the RAISED pile (v8e render: crown-weighted draws left
        # the lower slopes and the toe as bare tile) — the coverage class has
        # to tile the whole mound, toe included
        h = None
        for c in cells:
            h = _cell_height_at(c, x, y)
            if h is not None:
                break
        if h is None or h <= MOUND_LIP_M + 0.05:
            return None
        return 1.0

    for i in range(n_cluster):
        if not proto_sets["cluster"]:
            break
        if i < n_cluster_toe:
            lx, ly = _sample_toe_point(rng, density_cell, m, plate_ok, side=None)
        elif i < n_cluster_min:
            lx, ly = _sample_flank_point(rng, density_cell, m, plate_ok, crown=False)
        else:
            xy = _reject_xy(rng, pxmin, pxmax, pymin, pymax, cluster_weight, 1.0, m, plate_ok, 200)
            if xy is None:
                continue
            lx, ly = xy
        name = rng.choice(proto_sets["cluster"])
        quat = _orient_on_surface(rng, surf_normal(lx, ly), (5.0, 20.0))
        scale = rng.uniform(*CLUSTER_SCALE)
        size = _asset_entry(name)["size"]
        if i < n_cluster_min:
            pos, _b = place_bumped(lx, ly, quat, size, scale, BURY["cluster"])
        else:
            pos, _b = place(lx, ly, quat, size, scale, BURY["cluster"])
        _append_instance(instances["cluster"], name, pos, quat, scale)

    if proto_sets["toe"]:
        street_side = "S" if "S" in fall_sides or not fall_sides else sorted(fall_sides)[0]
        for _ in range(rng.randint(2, 5)):
            lx, ly = _sample_toe_point(rng, density_cell, m, plate_ok, side=street_side)
            name = rng.choice(proto_sets["toe"])
            quat = _quat_from_axis_angle((0.0, 0.0, 1.0), rng.uniform(0.0, 360.0))
            scale = rng.uniform(*CLUSTER_SCALE)
            size = _asset_entry(name)["size"]
            pos, _b = place(lx, ly, quat, size, scale, BURY["cluster"])
            _append_instance(instances["toe"], name, pos, quat, scale)

    # --- budget ---
    if budget and "n_large" in budget and len(large) > budget["n_large"]:
        order = {"panel": 0, "raft": 1, "rebar": 2, "sheet": 3, "column": 4,
                 "lintel": 5, "quoin": 5, "joist": 6}
        large = sorted(large, key=lambda e: order.get(e["kind"], 9))[:budget["n_large"]]

    # round-5: the FINAL exact trim to `effective_cap` (min of the round-5
    # default `RUBBLE_MAX_INSTANCES` and any caller `budget["n_instances"]`,
    # already computed above for the chunk/flake pre-scale). `n_chunks`/
    # `n_flake` were already sized close to this cap before generation (so
    # the placement loops above never do tens of thousands of wasted
    # rejection-sample draws for a target that would just get sliced away),
    # but cluster/toe (small, fixed counts) are added AFTER that pre-scale
    # and can push the combined total slightly over — this pass guarantees
    # the total across every set never exceeds `effective_cap`, exactly, via
    # the shared `_trim_instances_to_cap` helper (same proportional-across-
    # sets trim the old caller-only `budget["n_instances"]` path used).
    n_instances_before_cap, n_instances_after_cap, instances_capped = \
        _trim_instances_to_cap(instances, effective_cap)

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
        if not c.get("near"):
            # dome cells only: their taper zone lies beyond the relaxed foot,
            # where nothing was placed. A strip is tapered at BUILD time in
            # `_build_strip` — tapering it here, after placement, dropped the
            # surface under chunks already seated on it (251 floaters on a
            # fan when this ran for every cell).
            _taper_to_boundary(c, MOUND_LIP_M)
        _finalize_cell_boundary(c)
    for c in apron_cells:
        _taper_to_boundary(c, APRON_LIP_M)
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
            size = _asset_entry(name)["size"]
            lx, ly = _to_local(m, px, py)
            zmin_rel, _zmax = rotated_extent(size, scale, quat)
            min_z_world = pz + zmin_rel
            surf = z0 + _surf_final(lx, ly)
            if min_z_world - surf > 0.05:
                floating += 1

    n_inst_per_set = {k: len(v["positions"]) for k, v in instances.items()}
    n_protos_per_set = {k: len(v["protos"]) for k, v in instances.items()}

    # round-5: "achieved" coverage per zone — the density this pile actually
    # ended up drawing (after the pre-scale AND the final exact trim, both
    # folded into the final/estimate ratio) as a fraction of `COVERAGE`'s
    # own target, combined across chunk + flake (both classes' densities
    # share the SAME crown/mid/toe shape, `_coverage_frac`, so their ratio
    # to the target is uniform across zones within one class — see
    # `_zone_count_estimate`'s docstring).
    ratio_chunk = n_inst_per_set.get("chunk", 0) / max(n_est_chunk, 1e-9)
    ratio_flake = n_inst_per_set.get("flake", 0) / max(n_est_flake, 1e-9)
    coverage_stats = {zone: {"target": COVERAGE[zone],
                              "achieved": float(COVERAGE[zone] * (ratio_chunk + ratio_flake))}
                       for zone in ("crown", "mid", "toe")}

    # round-5 addendum: honest per-plan `hp` bit — did an `_hp` twin actually
    # get AUTHORED (a `large` element's `asset`, or an instance set's
    # `protos`), not just "was one available in the pool" (a rare name can
    # sit in `proto_sets` and never once get drawn by `rng.choice`).
    using_hp = any(str(e.get("asset")).endswith("_hp") for e in large if e.get("asset")) or \
        any(str(p).endswith("_hp") for v in instances.values() for p in v["protos"])
    stats = {"n_large": len(large), "n_instances": n_inst_per_set,
             "n_instances_total": sum(n_inst_per_set.values()),
             "n_protos": n_protos_per_set, "hd": using_hd, "hp": using_hp,
             "coverage": coverage_stats,
             "instances_before_cap": n_instances_before_cap,
             "instances_after_cap": n_instances_after_cap,
             "instances_capped": instances_capped, "instance_cap": effective_cap,
             "crown_m": crown_actual, "volume_m3": float(volume_m3),
             "reach_m": reach_report, "fall_sides": sorted(fall_sides),
             "extent_m": extent_by_side(m, mound_points),
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


# ---------------------------------------------------------------------------
# STREET DEBRIS (round-5 addendum — user, on the first OSMO city scene:
# "I don't see a lot of the concrete debris being used ... use that more").
# `plan_pile`'s own "toe" draw (`proto_sets["toe"]`) already puts 2-5 street
# pieces at the very foot of an ACTUAL pile — but that only exists at all on
# a DG4/DG5 dome, and 2-5 pieces at ONE toe ring reads thin from the street.
# `plan_street_scatter` is a SECOND, INDEPENDENT, much smaller planner: no
# heightfield, no burial fraction, no crown/mid/toe coverage math — just a
# handful of `street`-kind pieces (sidewalk slabs, cracked paving, a
# lamppost stub) laid FLAT on the EXISTING ground, on a damaged building's
# fall side(s), starting just beyond wherever its own pile (real or nominal)
# already reaches. Called once per DG3+ building by `quake._street_debris_
# pass` (from `quake.ground_effects`, city time, after every grade is
# drawn) — so a DG3 building (standing, no pile at all) gets a few pieces
# too, not just DG4/DG5.
# ---------------------------------------------------------------------------
STREET_DEBRIS_N_BY_GRADE = {"DG3": (3, 8), "DG4": (10, 22), "DG5": (20, 40)}
STREET_DEBRIS_BAND_M = (0.3, 2.6)      # beyond the (pile) reach, on the fall side
STREET_DEBRIS_MAX_PER_BUILDING = 40
STREET_DEBRIS_END_MARGIN = 0.42        # keep clear of the wall's own corners; density, not precision


def _side_local_point(m, side, t, d):
    """Local (lx, ly) for a point `t` metres along `side`'s own wall, from
    the wall's centre, and `d` metres beyond the wall line — the same
    S/N/E/W convention (`_SIDE_NORMAL`, `_side_length`, `_side_axes`) every
    other placement in this module uses, without needing a heightfield cell
    at all (street debris lies on the EXISTING ground, not a mound)."""
    halfW, halfD = m["W"] / 2.0, m["D"] / 2.0
    if side == "S":
        return (t, -halfD - d)
    if side == "N":
        return (t, halfD + d)
    if side == "E":
        return (halfW + d, t)
    if side == "W":
        return (-halfW - d, t)
    raise ValueError(side)


# A modest absolute floor above the pile's own HD_CHUNK_MIN_THICK_M (0.07):
# `HD_VOLUME_RATIO` (below) is now the PRIMARY shape gate, so this is a
# defensive minimum-size guard, not the main lever — a dust-sized fleck
# could pass a pure ratio test and still look wrong scattered alone.
STREET_CHUNK_MIN_THICK_M = float(os.environ.get("RUBBLE_STREET_CHUNK_MIN_THICK_M", "0.12"))
STREET_CHUNK_MAX_OPEN = float(os.environ.get("RUBBLE_STREET_CHUNK_MAX_OPEN", "0.35"))
STREET_MIN_VOLUME_RATIO = float(os.environ.get("RUBBLE_STREET_MIN_VOLUME_RATIO", "0.15"))

# ---------------------------------------------------------------------------
# ROUND 4 — the user's review of `street_debris_dg3_s5_close.png` (round 3's
# render): seating was flush, but ORIENTATION failed — several scan pieces
# stood up-ended on a point or leaned near-vertical (a flat star-shaped patch
# standing upright, a propped shard, a curled bowl on edge). Root cause:
# `_chunk_orientation`'s thin axis is the AABB's own thinnest COORDINATE
# axis, which is only the piece's real thin direction for an axis-aligned
# scan — for a curled/warped fragment the two can point in unrelated
# directions (see `HD_THIN_AXIS`'s docstring, `concrete_slabs_p028`'s
# measured (-0.09, 0.77, 0.63)). `_thin_axis_for` + `_orient_flat_on_axis`
# fix this by aligning the piece's own measured PCA-thinnest axis instead.
# ---------------------------------------------------------------------------
STREET_FLAT_TILT_DEG = (0.0, 12.0)     # "plus up to ~12 deg random tilt"
STREET_MAX_HEIGHT_M = 0.45             # realized (post-seating) height cap
STREET_HEIGHT_RESCALE_FLOOR = 0.35     # below this, redraw rather than shrink
STREET_HEIGHT_MAX_REDRAWS = 6

# ROUND 4 — "mix in authored chipped chunks": roughly half of a building's
# street debris is not a scanned piece at all but a small, irregular,
# CHIPPED concrete box authored through the exact same `large` channel
# `plan_pile`'s rafts/lintels/columns already use (`quake_rubble_usd.
# _author_large`), reusing an EXISTING `_BEAM_KINDS`/`_CHIP_KIND` kind
# ("quoin": a cut-stone-like block that mostly SPALLED rather than snapped
# at two ends, the closer analogy for a broken slab hunk than a bar-shaped
# lintel/column/sill) so it gets the Damaged_Concrete_Floor beam look and a
# real irregular/chipped silhouette for free, with NO change to
# `quake_rubble_usd.py`. Sized as a FLAT prism (thickness is always a
# fraction of the shorter footprint edge, so it can never stand taller than
# it is wide) — never a standing cube.
STREET_CHIP_SHARE = 0.5                # ~half chip boxes, ~half scan pieces
STREET_CHIP_XY_M = (0.3, 1.2)          # footprint edge length (m)
STREET_CHIP_Z_FRAC = (0.15, 0.45)      # thickness as a fraction of min(sx, sy)
STREET_CHIP_TILT_DEG = (0.0, 10.0)
STREET_CHIP_KIND = "quoin"             # an existing quake_rubble_usd kind


_STREET_FAMILY_BLACKLIST = ("huge_concrete_rubble_pile",)


def _street_chunk_pool(btype):
    """The near-closed, genuinely VOLUMETRIC chunk-kind HD prototype names
    safe to scatter ALONE on clean pavement.

    ROUND-3 FINDING (`street_debris_dg3_s5_close.png`, third pass): round
    2's pool — `_hd_names_by("chunk", materials)`, i.e. `_chunky` (an AABB
    aspect-ratio test) + `HD_OPEN_FRAC <= HD_CHUNK_MAX_OPEN` (a topological
    boundary-edge test) — still drew pieces that read as thin curled
    patches with upturned wing tips, one appearing to hover above its own
    detached shadow. Measured directly on the actual catalogue:
    `concrete_slabs_p028` has bbox_aspect 0.412 (comfortably "chunky") and
    `HD_OPEN_FRAC` 0.107 (comfortably under 0.15) — it passes BOTH bbox-
    based tests — yet its REAL mesh points' smallest/largest PCA-axis
    std-dev ratio is 0.013: essentially a flat 2-D scan whose CURVATURE
    (not thickness) spans all three AABB axes. Neither existing filter can
    see that, because neither one looks at the actual points.

    `HD_VOLUME_RATIO` (`assets/rubble_hd/volume_ratio.json`, the SAME
    "computed once from real geometry, shipped as static data" pattern
    `HD_OPEN_FRAC`/`open_frac.json` already established, generated by a
    one-off PCA census over every piece's real mesh points against the
    local asset mirror) is the actual discriminator here — it is what
    `_load_local_mesh_points`/`_points_min_z` (this module's SEATING fix)
    could compute live if this ran against a local `asset_root`, but a
    real production run's `asset_root` is `omniverse://...` (see
    `quake_rubble_usd.ASSET_ROOT`), which this pure module cannot open
    live — so, exactly like `HD_OPEN_FRAC`, the geometry is measured ONCE
    offline and shipped as data instead of re-derived per call.

    MEASURED, AND WHY `STREET_CHUNK_MAX_OPEN` (0.35) IS LOOSER THAN THE
    PILE'S `HD_CHUNK_MAX_OPEN` (0.15): in this dataset, volume ratio and
    open fraction are ANTI-correlated — a genuinely 3-D, multi-facet
    rubble fragment (cut from `huge_concrete_rubble_pile`, a messy real
    pile scan) naturally shows MORE boundary edges than a single flat slab
    cutout, so the pile's own tight open cap systematically FAVOURS flat
    pieces. Requiring `HD_OPEN_FRAC <= 0.15` AND `HD_VOLUME_RATIO >= 0.15`
    together leaves only 4-5 candidates per material; loosening the open
    cap to 0.35 (still a real ceiling against genuinely hole-punched
    fragments) while keeping the 0.15 volume floor leaves ~90-100 per
    material family — real diversity, not 2 repeated prototypes.

    Never the "toe"/"street" kind, and never the flat, pre-HD-split
    `CATALOGUE` street spreads (`concrete_sidewalk_elements`, `cracked_
    paving_slabs`) either — unmeasured, scanned surfaces, never proven
    safe, never a fallback.

    Returns `[]` (never a fallback to an unsafe pool) when nothing
    qualifies for this `btype` — including when `HD_VOLUME_RATIO` itself
    is empty (the census file is missing on this checkout): no debris
    beats fake-looking debris.

    ROUND-5 "MORE DEBRIS" ADDENDUM: when the HD-sourced pool above is
    non-empty, `chunk_01..09` (`standalone/debris/pieces/chunk_*` — the
    SAME 34-piece flat-colour catalogue `_debris_material_for` in
    `quake_rubble_usd.py` already knows how to tint) join it as a bounded
    minority sized to land at `STANDALONE_STREET_CHUNK_FRAC` (~20%) of the
    RETURNED pool. They fail the PILE's own stricter 0.15 open cap (a real,
    welded-vertex topological hole spanning top-to-bottom — see
    `_standalone_names_by`'s comment) so they never enter `plan_pile`'s own
    chunk instancing, but measure a healthy PCA volume_ratio (0.44-0.53,
    comfortably over `STREET_MIN_VOLUME_RATIO`) with open_frac 0.21-0.22 —
    under this LOOSER street cap — which is exactly the "genuinely 3-D
    fragment, more boundary edges than a flat slab" profile the loosened
    cap exists for (see above). Skipped entirely when the HD-sourced pool
    is empty: a minority needs something to be a minority OF, and "nothing
    qualifies" must still mean `[]`, never a silent substitute census.

    DELIBERATELY NO `pca_thin_axis.json` ENTRY for `chunk_01..09` (unlike
    the HD population, and unlike `lump_01..06`'s own entries added
    alongside): a near-cubic chunk's PCA-thinnest axis is a DIAGONAL
    direction through a box whose three dimensions are all similar (that's
    WHY it has a healthy `volume_ratio` in the first place — it is not a
    flat scan patch). Aligning a diagonal axis to vertical makes
    `rotated_extent`'s AABB-corner height LARGER than any single box
    dimension (measured: 1.2-1.5 m for pieces 0.66-0.99 m on a side),
    blowing the street `STREET_MAX_HEIGHT_M` cap past even the rescale
    floor and silently dropping nearly every draw after `STREET_HEIGHT_
    MAX_REDRAWS` (measured actual scan-channel share: <1%, not the ~20%
    the pool composition promises). Leaving no census entry here makes
    `_thin_axis_for` fall back to its pre-round-4 AABB-thinnest-COORDINATE-
    axis rule instead, which for a near-cubic box IS its smallest
    dimension (measured: 100% height-cap acceptance across all 9 pieces).
    """
    materials = ("brick", "concrete") if btype == "urm" else ("concrete",)
    candidates = set()
    for n, e in HD_CATALOGUE.items():
        if e.get("kind") != "chunk" or e.get("material") not in materials:
            continue
        # ROUND-4 RENDER FINDING (reviewer): `huge_concrete_rubble_pile_*`
        # crops pass every geometric gate yet render as dark rebar/wire
        # tangles when ALONE on clean pavement (fine INSIDE a pile's
        # carpet, where the same family reads as photographic concrete —
        # see rc_dome_s3). A messy multi-object pile scan's crops carry
        # rebar and shadowed cavities no per-piece geometry test can see,
        # so the family is excluded from the STREET pool by name; the
        # chip-box channel fills the share.
        if any(n.startswith(f) for f in _STREET_FAMILY_BLACKLIST):
            continue
        sz = tuple(float(v) for v in e.get("size", (0, 0, 0)))
        if not _chunky(sz) or _hd_open_frac(n) > STREET_CHUNK_MAX_OPEN:
            continue
        if min(sz) < STREET_CHUNK_MIN_THICK_M:
            continue
        candidates.add(n)
    hd_out = []
    for n in candidates:
        ratio = HD_VOLUME_RATIO.get(n)
        if ratio is None or ratio < STREET_MIN_VOLUME_RATIO:
            continue
        hd_out.append(n)
    if not hd_out:
        return []
    standalone = _standalone_street_chunk_candidates(materials)
    n_standalone = min(len(standalone), max(0, int(round(
        STANDALONE_STREET_CHUNK_FRAC * len(hd_out) / (1.0 - STANDALONE_STREET_CHUNK_FRAC)))))
    return sorted(hd_out + standalone[:n_standalone])


# Target minority share (of the RETURNED pool) for the standalone chunk
# pieces folded into the street pool — same ~15-30% band, mid-point 0.20,
# as `STANDALONE_FLAKE_MINORITY_FRAC` above.
STANDALONE_STREET_CHUNK_FRAC = 0.20


def _standalone_street_chunk_candidates(materials):
    """`chunk_01..09` names passing the SAME street gates `_street_chunk_
    pool` applies to an HD name (`_chunky`, `HD_OPEN_FRAC <=
    STREET_CHUNK_MAX_OPEN`, `STREET_CHUNK_MIN_THICK_M`, `HD_VOLUME_RATIO >=
    STREET_MIN_VOLUME_RATIO`) — reading the SAME `HD_OPEN_FRAC`/`HD_VOLUME_
    RATIO` census dicts `_street_chunk_pool` reads, now extended with a
    one-off measured entry per standalone piece (round-5 "more debris"
    addendum; see `assets/rubble_hd/open_frac.json`/`volume_ratio.json`).
    Sorted (deterministic) so the SAME leading slice is chosen every call —
    there is no `rng` here (this is a per-btype pool, not a per-instance
    draw; variety across instances still comes from `rng.choice(pool)` in
    `plan_street_scatter`)."""
    out = []
    for n in _CHUNKS:
        e = CATALOGUE.get(n)
        if not e or e.get("kind") != "chunk" or e.get("material") not in materials:
            continue
        sz = tuple(float(v) for v in e.get("size", (0, 0, 0)))
        if not _chunky(sz) or _hd_open_frac(n) > STREET_CHUNK_MAX_OPEN:
            continue
        if min(sz) < STREET_CHUNK_MIN_THICK_M:
            continue
        ratio = HD_VOLUME_RATIO.get(n)
        if ratio is None or ratio < STREET_MIN_VOLUME_RATIO:
            continue
        out.append(n)
    return sorted(out)


_STREET_MESH_POINTS_CACHE = {}


def _load_local_mesh_points(name, asset_root):
    """Local-space (x, y, z) vertex points of every `Mesh` in the catalogue
    asset `name` references, read directly from its OWN `.usdc` file under
    a LOCAL `asset_root` — the ACTUAL scanned geometry, not the AABB `size`
    a corner extrapolation (`rotated_extent`) is built from. See
    `_points_min_z`'s docstring for why the distinction matters.

    Lazily imports `pxr` (usd-core) — see the module docstring's note on
    this one narrow exception. Returns `None` (never raises) when `pxr` is
    not importable, `asset_root` is not a local path (empty, `None`, or an
    `omniverse://` URL — no cheap way for this pure module to fetch a
    remote file), the resolved file is not on disk, or it opens with no
    mesh at all; the caller falls back to `rotated_extent` in every one of
    those cases, exactly this module's pre-existing behaviour.

    Cached per `(name, asset_root)` — a city's worth of street-scatter
    instances redraws the same handful of prototype names over and over,
    and re-parsing the same file per instance would be real, avoidable I/O.
    """
    key = (name, str(asset_root))
    if key in _STREET_MESH_POINTS_CACHE:
        return _STREET_MESH_POINTS_CACHE[key]
    pts = _load_local_mesh_points_uncached(name, asset_root)
    _STREET_MESH_POINTS_CACHE[key] = pts
    return pts


def _load_local_mesh_points_uncached(name, asset_root):
    if not asset_root or str(asset_root).startswith("omniverse://"):
        return None
    entry = _asset_entry(name)
    rel = entry.get("url") if entry else None
    if not rel:
        return None
    path = os.path.join(str(asset_root), str(rel).replace("/", os.sep))
    if not os.path.exists(path):
        return None
    try:
        from pxr import Usd, UsdGeom
    except Exception:
        return None
    try:
        stage = Usd.Stage.Open(path)
        if not stage:
            return None
        xf_cache = UsdGeom.XformCache()
        pts = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            arr = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not arr:
                continue
            local_to_root = xf_cache.GetLocalToWorldTransform(prim)
            for p in arr:
                wp = local_to_root.Transform(p)
                pts.append((float(wp[0]), float(wp[1]), float(wp[2])))
        if not pts:
            return None
        return np.asarray(pts, dtype=np.float64)
    except Exception:
        return None


def _points_z_extent(points_local, scale, rot):
    """(zmin, zmax): world-Z range of `points_local` (Nx3 REAL mesh
    vertices, in the asset's own local frame) after uniform `scale` and
    rotation `rot` (quaternion or XYZ-Euler-degrees — `rotated_extent`'s
    own two accepted forms, identical convention) — the analogue, for a
    standalone catalogue file rather than a live authored prim, of `bake.
    world_point_bounds` (the `fix-floating-debris` skill's fix for the
    `UsdGeom.BBoxCache` blind spot).

    WHY THIS DIFFERS FROM `rotated_extent`: a `size`-box's rotated CORNER
    extrapolation is a mathematically exact bound for a uniform BOX (a
    linear functional over a box attains its extremum at a vertex), but
    the real mesh is a SUBSET of that box, not the box itself — for a
    curled/warped/off-axis scan, the true mesh's rotated lowest point can
    sit well ABOVE the box's rotated lowest corner. Seating on the corner
    then leaves the real geometry floating with daylight underneath: the
    "hovering above its own detached shadow" defect round-5's street-
    scatter review reported.

    `zmax` (round 4, added alongside the pre-existing `zmin`-only
    `_points_min_z`) is what `plan_street_scatter`'s realized-height cap
    needs: `zmax - zmin` is the piece's true on-screen height above grade
    once seated, which for a curled/warped scan is not simply `size[2] *
    scale` (the AABB height) either.

    Returns `None` if `points_local` is `None`/empty (caller falls back to
    `rotated_extent`)."""
    if points_local is None or points_local.size == 0:
        return None
    if len(rot) == 4:
        R = _quat_to_matrix(rot)
    else:
        R = _euler_xyz_to_matrix(rot[0], rot[1], rot[2])
    world_z = (points_local * float(scale)) @ np.asarray(R[2, :], dtype=np.float64)
    return float(np.min(world_z)), float(np.max(world_z))


def _points_min_z(points_local, scale, rot):
    """Min world-Z of `points_local` — see `_points_z_extent` (this is a
    thin `[0]`-projection of it, kept as its own name since every existing
    caller/test only ever needed the minimum)."""
    ext = _points_z_extent(points_local, scale, rot)
    return None if ext is None else ext[0]


def plan_street_scatter(m, grade, rng, btype="rc", fall_sides=None,
                        reach_sides=None, max_instances=STREET_DEBRIS_MAX_PER_BUILDING,
                        asset_root=None):
    """A small, FLAT scatter of near-closed, chunky HD debris (mixed with
    authored chipped-concrete chunks, round 4) on the sidewalk/road band
    just beyond one damaged building's own pile reach, on its fall side(s)
    — see the module note above for why this is separate from `plan_pile`'s
    own toe draw.

    `fall_sides` / `reach_sides`: the SAME per-side vocabulary `quake.
    heap_reach_sides` / `_heap_reach_for` already compute for heap
    clearance — the caller (`quake._street_debris_pass`) passes them
    straight through so the two passes agree on which side is "the street"
    and where the pile (if any, real or nominal) already reaches. Every
    instance lands at `d = reach_sides[side] + U(*STREET_DEBRIS_BAND_M)`,
    STRICTLY beyond that reach — so this can never double-place inside a
    pile `quake._clear_under_heaps` already cleared street furniture under.
    Defaults to a single "S" fall side at the research floor
    (`RUNOUT_FLOOR_M`) when the caller has no manifest row to measure from.

    Counts scale with `grade` (`STREET_DEBRIS_N_BY_GRADE`, base grade only —
    a "DG3+tilt" label is treated as "DG3"): a DG3 building (standing, no
    pile) gets a handful; DG5 gets up to `max_instances` (<=
    `STREET_DEBRIS_MAX_PER_BUILDING`, the per-building budget the city-level
    cap in `quake._street_debris_pass` also enforces). `t` (the position
    along the wall) is drawn over the inner `2 x STREET_DEBRIS_END_MARGIN`
    share of the wall's length, not its corners — the fire-service rule of
    thumb that debris fans out from mid-wall, and cheap insurance against
    piling pieces at a corner two adjacent sides both claim; this keeps
    density LOW near a crosswalk without knowing where one is.

    TWO CHANNELS PER PIECE (round 4, `STREET_CHIP_SHARE` of the total each
    building draws land in each, ALWAYS the chip channel when the scan pool
    is empty — see below):

      * SCAN pieces (`instances["street"]`) — drawn from `_street_chunk_
        pool(btype)` (see its docstring; NOT the "toe"/"street" kind, and
        NOT the flat pre-HD-split `CATALOGUE` street spreads). Orientation:
        `_orient_flat_on_axis` aligns the piece's own MEASURED PCA-thinnest
        axis (`_thin_axis_for`/`HD_THIN_AXIS`) to world-up, not the AABB's
        thinnest COORDINATE axis (`_chunk_orientation`'s convention, wrong
        for a curled/warped scan whose true thin direction is diagonal —
        the round-4 review's up-ended/near-vertical pieces) — plus up to
        `STREET_FLAT_TILT_DEG` off vertical and a free yaw about up.
        Seating: `_points_min_z`/`_points_z_extent` (the piece's own real
        mesh points, read via `asset_root` when possible) rather than
        `rotated_extent`'s AABB-corner approximation — falls back to
        `rotated_extent` when the points cannot be read (see `_load_local_
        mesh_points`'s docstring for every case that triggers the
        fallback). HEIGHT CAP (round 4): the realized height above grade
        (`zmax - zmin` at the piece's actual scale, computed the same way
        as seating) is capped at `STREET_MAX_HEIGHT_M` — a piece that would
        exceed it is RESCALED down (height is linear in scale, so this is
        exact) unless that would shrink it below `STREET_HEIGHT_RESCALE_
        FLOOR` (reads as a shrunken pebble, not a to-scale chunk), in which
        case a different piece/orientation is redrawn (up to `STREET_
        HEIGHT_MAX_REDRAWS` times); a piece that still can't fit after every
        redraw is DROPPED, never authored over-height.

      * CHIP pieces (`large`) — small, irregular AUTHORED concrete boxes
        (`STREET_CHIP_KIND`, an existing `quake_rubble_usd._BEAM_KINDS`/
        `_CHIP_KIND` kind, so they get the Damaged_Concrete_Floor beam look
        and a real chipped/irregular silhouette with NO emitter changes),
        footprint edges `STREET_CHIP_XY_M`, thickness always a FRACTION of
        the shorter footprint edge (`STREET_CHIP_Z_FRAC`) so a piece can
        never stand taller than it is wide, laid flat (a free yaw plus up to
        `STREET_CHIP_TILT_DEG` off vertical) with its bottom exactly at
        grade (`rotated_extent`, exact for an authored box). Also subject
        to the `STREET_MAX_HEIGHT_M` height cap (the same one the scan
        pieces respect) as a safety net for the rare large-footprint +
        near-max-tilt combination — enforced by uniformly shrinking the
        box's own (sx, sy, sz), exact because `rotated_extent` is linear
        under a uniform size multiplier, so no redraw is needed here (an
        authored box's dimensions are ours to pick, unlike a fixed scanned
        prototype). Needs no scan pool at all — the fallback population
        when `_street_chunk_pool` is empty (no HD census on this
        checkout), so a building still gets SOME concrete debris rather
        than none.

    No burial fraction either way: street debris rests ON the existing
    ground.

    Returns a `plan_pile`-shaped dict (`mound=None`, `apron=None`, `large`
    the authored chip-chunk list, one `instances["street"]` set for the
    scan pieces) — the exact contract `quake_rubble_usd.author` already
    knows how to write, so no new emitter code is needed for this pass.
    `stats`: `n_instances` (BOTH channels combined), `n_scan`, `n_chip`,
    `n_by_side` (both channels), `grade`, `capped`, `fall_sides`, `hd`
    (whether the scan pool was non-empty).
    """
    grade = str(grade).split("+")[0]
    fall_sides = sorted(set(fall_sides)) if fall_sides else ["S"]
    reach_sides = dict(reach_sides) if reach_sides else {}
    btype = btype if btype in PROTO_SETS else "rc"
    resolved_asset_root = asset_root if asset_root is not None else os.environ.get("RUBBLE_ASSET_ROOT")
    pool = _street_chunk_pool(btype)
    instances = {"street": _empty_instance_set(None)}
    large = []
    z0 = float(m.get("z0", 0.0))
    lo_hi = STREET_DEBRIS_N_BY_GRADE.get(grade)
    # NOT gated on `pool` any more (round 4): the chip channel below needs
    # no scan pool at all, so an empty pool now means "100% chip" rather
    # than "nothing at all" (see the docstring above).
    n_target = min(int(max_instances), rng.randint(*lo_hi)) if lo_hi else 0

    n_by_side = {}
    n_scan = 0
    n_chip = 0
    for i in range(n_target):
        side = fall_sides[i % len(fall_sides)]
        L = _side_length(m, side)
        t = rng.uniform(-STREET_DEBRIS_END_MARGIN, STREET_DEBRIS_END_MARGIN) * L
        reach = float(reach_sides.get(side, RUNOUT_FLOOR_M))
        d = reach + rng.uniform(*STREET_DEBRIS_BAND_M)
        lx, ly = _side_local_point(m, side, t, d)
        wx, wy = _to_world(m, lx, ly)
        n_by_side[side] = n_by_side.get(side, 0) + 1

        if (not pool) or (rng.random() < STREET_CHIP_SHARE):
            sx = rng.uniform(*STREET_CHIP_XY_M)
            sy = rng.uniform(*STREET_CHIP_XY_M)
            sz = min(sx, sy) * rng.uniform(*STREET_CHIP_Z_FRAC)   # always flatter than wide
            quat = _orient_on_surface(rng, (0.0, 0.0, 1.0), STREET_CHIP_TILT_DEG)
            zmin_rel, zmax_rel = rotated_extent((sx, sy, sz), 1.0, quat)
            height = zmax_rel - zmin_rel
            if height > STREET_MAX_HEIGHT_M:
                # `rotated_extent` is EXACTLY linear under a uniform size
                # multiplier (scaling every corner by the same factor scales
                # the whole corner-extremum search by it too), so shrinking
                # (sx, sy, sz) together by this ratio lands the realized
                # height EXACTLY at the cap -- no redraw needed, unlike the
                # scan-piece pool (an authored box's own dimensions are ours
                # to pick, not a fixed scanned prototype's).
                k = STREET_MAX_HEIGHT_M / max(height, 1e-9)
                sx, sy, sz = sx * k, sy * k, sz * k
                zmin_rel, zmax_rel = rotated_extent((sx, sy, sz), 1.0, quat)
            large.append({
                "asset": None, "prim_path": None, "kind": STREET_CHIP_KIND,
                "pos": (wx, wy, z0 - zmin_rel), "rot_deg": _quat_to_euler_xyz_deg(quat),
                "scale": 1.0, "size": (sx, sy, sz), "bury": 0.0, "look": "concrete",
            })
            n_chip += 1
            continue

        for _attempt in range(STREET_HEIGHT_MAX_REDRAWS):
            name = rng.choice(pool)
            size = _asset_entry(name)["size"]
            axis_local = _thin_axis_for(name, size)
            quat = _orient_flat_on_axis(rng, axis_local, STREET_FLAT_TILT_DEG)
            scale = rng.uniform(*CLUSTER_SCALE)
            pts_local = _load_local_mesh_points(name, resolved_asset_root)
            if pts_local is not None:
                zmin_u, zmax_u = _points_z_extent(pts_local, 1.0, quat)
            else:
                zmin_u, zmax_u = rotated_extent(size, 1.0, quat)
            height_u = max(zmax_u - zmin_u, 1e-9)   # height is linear in scale
            height = height_u * scale
            if height > STREET_MAX_HEIGHT_M:
                needed_scale = STREET_MAX_HEIGHT_M / height_u
                if needed_scale < STREET_HEIGHT_RESCALE_FLOOR:
                    continue                          # would read as a shrunken pebble -- redraw
                scale = needed_scale
            zmin_rel = zmin_u * scale
            _append_instance(instances["street"], name, (wx, wy, z0 - zmin_rel), quat, scale)
            n_scan += 1
            break
        # else: every redraw exceeded the cap even at the rescale floor --
        # this instance is DROPPED (never author an over-height piece).

    _n_before, n_after_street, capped = _trim_instances_to_cap(instances, max_instances)
    n_after = n_after_street + len(large)
    stats = {"n_instances": n_after, "n_scan": n_scan, "n_chip": n_chip,
             "n_by_side": n_by_side, "grade": grade, "capped": capped,
             "fall_sides": fall_sides, "hd": bool(pool)}
    return {"mound": None, "apron": None, "large": large, "instances": instances, "stats": stats}


# ---------------------------------------------------------------------------
# HD debris piece library loader — `tools/split_debris_spreads.py` splits
# the textured FAB "spreads" (`assets/concrete_rubble_debris/split/*`, each
# one merged whole-pile Mesh) into per-piece HD chunks
# (`assets/rubble_hd/<spread>/<spread>_pNNN.usdc` + `assets/rubble_hd/
# catalogue.json`) — the fix for "the debris is very low poly / cartoonesque"
# (the only chunk-scale entries in `CATALOGUE` above are the 34 flat-colour
# `standalone/debris/pieces/*`). This loader just reads that JSON back into
# exactly `CATALOGUE`'s per-entry shape so a planner can treat an HD piece
# like any other catalogue asset; it does NOT modify `CATALOGUE`,
# `PROTO_SETS`, or `plan_pile` — wiring HD pieces into the actual scatter is
# a separate change. Must NEVER raise: this module is imported by every
# running Isaac process, and the HD library may not have been built yet.
_HD_CATALOGUE_DEFAULT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "assets", "rubble_hd", "catalogue.json")


def load_hd_catalogue(path=None):
    """Read `assets/rubble_hd/catalogue.json` (written by
    `tools/split_debris_spreads.py`) and return a dict keyed by piece name,
    each value shaped exactly like a `CATALOGUE` entry: `{"url", "size"
    (x, y, z tuple), "tris", "kind", "textured": True, "material"}`.

    Returns `{}` (never raises) if `path` doesn't exist, isn't readable, or
    is malformed — the HD library is optional build output, not something
    every checkout is guaranteed to have."""
    if path is None:
        path = _HD_CATALOGUE_DEFAULT_PATH
    try:
        with open(path, "r") as f:
            data = json.load(f)
    except Exception:
        return {}

    pieces = data.get("pieces") if isinstance(data, dict) else data
    out = {}
    try:
        for entry in pieces or []:
            name = entry.get("name")
            size = entry.get("size")
            if not name or size is None or len(size) != 3:
                continue
            out[name] = {
                "url": entry.get("url"),
                "size": (float(size[0]), float(size[1]), float(size[2])),
                "tris": int(entry.get("tris", 0)),
                "kind": entry.get("kind"),
                "textured": True,
                "material": entry.get("material"),
            }
    except Exception:
        return {}
    return out


HD_CATALOGUE = load_hd_catalogue()


def _load_open_frac():
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "..", "assets", "rubble_hd", "open_frac.json")) as f:
            return {str(k): float(v) for k, v in json.load(f).items()}
    except Exception:
        return {}


HD_OPEN_FRAC = _load_open_frac()


def _load_volume_ratio():
    """{name: float} from `assets/rubble_hd/volume_ratio.json` — a PCA
    census (smallest / largest principal-axis std-dev of a piece's REAL
    mesh points, computed once offline against the local asset mirror) —
    the SAME "computed once, shipped as data" pattern `_load_open_frac`
    already established, added for `_street_chunk_pool`'s volumetric gate
    (see its docstring for why the existing bbox-based `_chunky`/
    `HD_OPEN_FRAC` filters cannot see this on their own). Never raises;
    `{}` when the file is missing (this checkout never ran the census, or
    predates it) — `_street_chunk_pool` degrades to "nothing qualifies",
    not to an unmeasured/unsafe pool."""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "..", "assets", "rubble_hd", "volume_ratio.json")) as f:
            return {str(k): float(v) for k, v in json.load(f).items()}
    except Exception:
        return {}


def _load_pca_thin_axis():
    """{name: (x, y, z)} from `assets/rubble_hd/pca_thin_axis.json` — the
    LOCAL-frame unit vector of a piece's own thinnest principal axis
    (smallest-eigenvalue eigenvector of the covariance of its REAL mesh
    points), generated by the SAME PCA census that produced `HD_VOLUME_
    RATIO` (that census's own eigenvector, not a second measurement) —
    the same "computed once from real geometry against the local mirror,
    shipped as static data" pattern `HD_OPEN_FRAC`/`HD_VOLUME_RATIO`
    already use.

    ROUND-4 FIX this exists for: `_chunk_orientation`'s pre-round-4
    convention picked the AABB's own thinnest COORDINATE axis (x, y, or z)
    as "the thin axis" and rotated THAT to vertical. For an axis-aligned
    scan that is exactly right, but for a curled/warped/diagonal fragment
    the true thin direction can point nowhere near any single coordinate
    axis — `concrete_slabs_p028` (the round-3 flat-shell finding) measures
    at (-0.09, 0.77, 0.63) here, a genuinely diagonal direction, not (0, 0,
    1) or any permutation of it. Aligning the AABB-thin axis for a piece
    like that leaves the REAL thin direction (and therefore the piece's
    broadest real face) tilted away from horizontal — exactly the
    up-ended/near-vertical silhouettes the round-4 render review reported
    (a flat star-shaped patch standing upright, a propped shard, a curled
    bowl on edge). `_orient_flat_on_axis` aligns THIS axis instead.

    Never raises; `{}` when the file is missing — `_thin_axis_for` then
    falls back to the pre-round-4 AABB-thinnest-coordinate-axis rule
    (never an outright failure, just the old, less-precise behaviour)."""
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, "..", "assets", "rubble_hd", "pca_thin_axis.json")) as f:
            data = json.load(f)
        return {str(k): tuple(float(x) for x in v) for k, v in data.items()}
    except Exception:
        return {}


HD_THIN_AXIS = _load_pca_thin_axis()


def _thin_axis_for(name, size):
    """The LOCAL unit vector to treat as `name`'s "thin axis" for flat
    orientation: `HD_THIN_AXIS`'s measured PCA eigenvector when the census
    has an entry for `name`, else the pre-round-4 AABB-thinnest-coordinate-
    axis fallback (`_chunk_orientation`'s own convention) — the same
    graceful degradation `_load_local_mesh_points` already has for a
    non-local `asset_root` (a real production run's `asset_root` is
    `omniverse://...`, but the census is static data, so it works there
    too; this fallback only matters for a name the census never measured,
    e.g. a non-HD `CATALOGUE` prototype, or a checkout predating the
    census file)."""
    axis = HD_THIN_AXIS.get(name)
    if axis is not None:
        v = np.asarray(axis, dtype=np.float64)
        n = float(np.linalg.norm(v))
        if n > 1e-9:
            return tuple((v / n).tolist())
    thin_i = min(range(3), key=lambda i: size[i])
    return [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)][thin_i]


def _orient_flat_on_axis(rng, axis_local, tilt_range):
    """Round 4: orient a piece so its own `axis_local` (a LOCAL-frame unit
    vector — the piece's real PCA-thinnest axis, see `_thin_axis_for`)
    points world-up, plus a further `tilt_range` degrees off vertical and a
    free random yaw about world-up — the `_orient_on_surface` recipe
    (align, spin, tilt), generalised from a FIXED local axis (that
    function always aligns local +Z) to an arbitrary one, since a scan's
    real thin axis is not necessarily local Z.

    Composition order matches `_orient_on_surface` exactly: spin about
    `axis_local` FIRST (in the piece's own local frame, before it moves),
    then align `axis_local` to world +Z, then tilt off vertical about a
    random world-horizontal axis. Spinning about the very axis that is
    about to become vertical is equivalent (by the standard axis-
    conjugation identity) to spinning about world-up AFTER alignment — so
    this is genuinely "align, then free yaw about up, then a small tilt",
    just computed in the order that only needs one alignment quaternion.
    """
    q_align = _quat_align_vec_to(axis_local, (0.0, 0.0, 1.0))
    spin = rng.uniform(0.0, 360.0)
    q_spin = _quat_from_axis_angle(axis_local, spin)
    tilt = rng.uniform(*tilt_range)
    tang = rng.uniform(0.0, 360.0)
    tx, ty = math.cos(math.radians(tang)), math.sin(math.radians(tang))
    q_tilt = _quat_from_axis_angle((tx, ty, 0.0), tilt)
    return _quat_normalize(_quat_mul(_quat_mul(q_tilt, q_align), q_spin))


HD_VOLUME_RATIO = _load_volume_ratio()
