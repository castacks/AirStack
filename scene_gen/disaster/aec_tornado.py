#!/usr/bin/env python3
"""TORNADO damage for the AEC `Reference_Brownstone*Row` assets, BY NAME.

The urban-fire route (`disaster/aec_burn.py`, 2026-09-02) proved these row
assets can be damaged without slicing or baking: reference the asset RAW
(`gac_fire.place_source`), de-instance the units being touched, and
deactivate named parts (window glass, sashes, doors, roof plant, rear-deck
timber). This module reuses exactly that addressing — `aec_burn.
measure_row` and its part classifiers — for a TORNADO instead of a fire:

- NO soot, NO char, NO flames. The rows are URM masonry: per the tornado
  brief they must never read near-collapse ("They are sturdy, they aren't
  meant to have full collapses"). The ladder tops out at near-total windward
  glazing + all windward railings + windward cornice/coping gone + the rear
  deck's timber gone + a MODEST, GUARDED top-corner brick bite on the single
  most-windward unit of the row — the shell and roof deck always stand.
  2026-09-02 escalation (user: "a tornado can do more damage than what
  you've shown ... to the actual buildings and also to props ... which can
  be 'flown' away and settled"): heavier glazing/sash ladder, ALL windward
  railings (stoop AND roof), the windward cornice/coping stripped, the
  guarded corner bite (`_windward_corner_bite`, reusing `aec_burn.
  _mesh_dict_world` / `_clip_box` / `_write_world_piece` exactly the way
  `_lose_wall` clips the facade — but ONE storey tall, ONE row corner, and
  wrapped in try/except so a clip failure never fails the cell), and real
  killed timber FLOWN as its own mesh (tumbled, carried downwind, dropped
  onto the ground) instead of small proxy board boxes.
- Side shares use the SAME windward formula `tornado_urban.side_weights`
  uses (bearing in MATH convention — `dx, dy = cos, sin` — windward =
  most negative dot of the outward normal with the wind direction,
  leeward x0.35, flank x0.55), so a holder that will be SEATED at yaw
  simply passes the CELL-frame bearing (world bearing minus seat yaw) and
  the damage rotates with the building. The wreck itself must run at an
  IDENTITY holder (the bench's `_seat_holder` transform-trap rule):
  `measure_row` measures in world frame, the wall-bite piece is re-authored
  under the BUILDING's own root (so any asset scale is respected, exactly
  `_lose_wall`'s own trick), and all ground/flown debris is authored under
  the CELL from those world numbers.
- The interiors are the asset's own modelled fit-out (floors, stairs,
  casework), left INTACT and visible through the voided windows. No
  `fit_interior` here — this asset does not need the kit interior kit.
- Ground evidence is authored as merged static meshes seated on the ground
  (bottom 5 mm into it — exact seating, nothing floats), in the bench's own
  debris vocabulary: `tornado_urban_usd.debris_material` "brick"
  (Brick_Wall_Worn — these rows ARE red brick, and now also carry the
  corner-bite's and the stripped cornice's chunks) and dusted "metal" for
  thrown roof plant AND now every windward railing, `planks.wood_material`
  for sash splinters, and — for the rear deck's real killed timber and every
  removed railing alike — the ACTUAL member mesh, tumbled and carried
  downwind rather than a proxy board box (see `_fly_piece`).

  2026-09-02 REVIEW ROUND 2 (live render, three defects). (17) "can't
  really tell the windows are blown out ... we'd rather have cracks there":
  a hit windward pane no longer defaults to `_kill` -- it stays ACTIVE,
  rebound to `_cracked_glass_material` (a dark translucent pane, still
  glass) plus a jagged standoff crack pattern (`_pane_crack_tris`, bright
  thin quads a few mm proud of the pane, world-metre geometry authored
  under the BUILDING's own root exactly like the corner bite -- a decal has
  to move with the window it is stuck to, which is why it is NOT under the
  cell like the ground debris is). Only the most extreme case -- windward,
  the very TOP storey, `LADDER[level]["blowout_top"]` -- still fully blows
  out (0 at T2, rising through T3/T4); everything else cracks. Sash removal
  is gated the same way: a frame comes off only on a pane that also blew
  out, never on one that merely cracked. (18) "railings are gone but have
  them scattered": a removed windward railing is now flown exactly like the
  deck lumber (`_fly_piece`, its own `RAIL_TUMBLE_DEG`/`RAIL_THROW_M` --
  metal does not carry as far as a board), merged into one
  `aec_railing_metal` mesh under the cell's own debris scope, dusted-metal
  via `tornado_urban_usd.debris_material` "metal". (19) "no missing chunks
  or brick bite, and the rear lumber is floating": (a) the guarded
  top-corner bite now reaches `BITE_HEIGHT_STOREYS` (~1.5 storeys) down
  from the deck and a wider `BITE_WIDTH_FRAC`, so it reads as a real
  missing chunk rather than a sliver, and flings more brick with it; (b)
  `LUMBER_TUMBLE_DEG`'s upper bound is CAPPED (was 90, a tumble that stood
  a 2-3 m board on end and read as impaled/floating at bbox z 2.69 m) --
  every flown piece (lumber or railing) is still sunk to its OWN lowest
  point individually, per piece, before the population is merged into one
  authored mesh, exactly as `_fly_piece` always did. Capping the tumble
  fixed the SINGLE-board case, but an offline test written against this fix
  (welding a merged debris mesh's coincident vertices back into its real
  connected pieces, since none of these triangles share a vertex INDEX)
  caught a SECOND, worse case on railings: one `Railings`/`Top_Rails` prim
  is often several free-standing balusters sharing one mesh, and flying
  that whole assembly as ONE rigid body only grounds whichever baluster
  happens to be lowest -- every OTHER baluster stayed exactly where the
  tumble/throw transform put it, metres in the air, no matter how small the
  tumble cap was. `_split_islands` (a per-vertex-position union-find) now
  runs BEFORE every `_fly_piece` call, so a multi-part member becomes
  several independently tumbled/thrown/grounded pieces instead of one
  invisible rigid frame -- which reads as MORE scattered anyway, not less.

Levels mirror the bench's kit ladder in spirit:
  T2  brushed — a heavier share of windward glass, front-door glass, a
      light brick sprinkle. No sash, no plant, no railings, no cornice, no
      corner bite.
  T3  scraped — most windward glass + some flanks, over half the hit sashes
      out, ALL windward railings gone (stoop and roof), the windward
      cornice/coping stripped, most roof plant thrown downwind, half the
      rear deck lumber flown, a guarded top-corner brick bite on the row's
      most-exposed unit.
  T4  raked  — near-total windward glazing, most sashes, all plant and
      windward railings, cornice gone, the deck stripped and flown, the
      same guarded corner bite; the masonry still stands.

Verify offline with bare usd-core (+ vtk for the corner-bite clip) against
the real (local) asset — see `tests/test_aec_tornado.py`. No Kit, no
Nucleus.
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
for _p in (os.path.normpath(os.path.join(_HERE, "..")), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

try:                                      # package import (disaster.aec_tornado)
    from . import aec_burn as ab
    from . import damage as dmg
    from . import planks
    from . import tornado_urban_usd as tuu
except ImportError:                       # bare-script / sys.path import
    import aec_burn as ab                 # noqa: F401
    import damage as dmg                  # noqa: F401
    import planks
    import tornado_urban_usd as tuu


# ---------------------------------------------------------------------------
# THE LADDER — shares of the windward elevation, scaled per side by the
# windward weight. `bricks` is PER UNIT (coping/pointing shed by the wind,
# not a wall failure) plus whatever the cornice bite adds; everything else
# derives from actual kills. `rail_all_windward`: every Railings/Top_Rails
# part (stoop AND roof) on the windward elevation is gone, not a coin-flip.
# `cornice`/`corner_bite` gate the two masonry-facing escalations, both
# guarded to T3+ and both wrapped in try/except at the call site.
# ---------------------------------------------------------------------------
#: `blowout_top`: of a HIT windward pane on the row's very TOP storey, the
#: share that still fully blows out (`_kill`) rather than merely cracking
#: (`_crack_glass_part`) -- 2026-09-02 round 2: "I can't really tell the
#: windows are blown out ... we'd rather have cracks there". Guarded to the
#: top storey AND the windward elevation so it stays the rare, extreme case;
#: everything else that is hit cracks instead. 0 at T2 (brushed).
LADDER = {
    "T2": dict(glass=0.55, sash=0.00, door_glass=True, deck_lumber=0.00,
               plant=0.00, railing=0.00, rail_all_windward=False,
               cornice=False, corner_bite=False, bricks=(4, 8),
               blowout_top=0.00),
    "T3": dict(glass=0.90, sash=0.55, door_glass=True, deck_lumber=0.50,
               plant=0.65, railing=0.45, rail_all_windward=True,
               cornice=True, corner_bite=True, bricks=(10, 18),
               blowout_top=0.15),
    "T4": dict(glass=0.98, sash=0.85, door_glass=True, deck_lumber=1.00,
               plant=1.00, railing=0.80, rail_all_windward=True,
               cornice=True, corner_bite=True, bricks=(18, 30),
               blowout_top=0.35),
}

#: storey gradient: upper storeys catch more wind. Applied to the glass
#: share as `GRAD_LO + (GRAD_HI - GRAD_LO) * st / (n_st - 1)`.
GRAD_LO, GRAD_HI = 0.72, 1.25

#: debris box sizes (m) — brick and mortared chunk from `aec_burn`'s own
#: measured vocabulary, sash sticks sized like the real parts they came off.
BRICK_M = ab.BRICK_SIZE_M                 # (0.20, 0.095, 0.06)
CHUNK_M = ab.CHUNK_SIZE_M                 # (0.34, 0.20, 0.14)
SASH_M = (0.85, 0.055, 0.045)
#: bottom of every laid box (or dropped flown mesh) sinks this far into
#: what it sits on
SINK_M = 0.005
#: how far outside the unit's own bbox face the foot band reaches
FOOT_BAND_M = (0.4, 5.0)
#: thrown roof plant: downwind reach past the part's own centre, scaled by
#: `0.6 + 0.6 * speed_frac`
PLANT_THROW_M = (4.0, 14.0)
#: a Roofs-category piece within this many metres of the windward wall
#: plane (in the PERP coordinate) is cornice/coping, not the deck membrane
CORNICE_PLANE_TOL_M = 1.5
#: a Railings/Top_Rails part is "on the windward elevation" when its own
#: PERP-coordinate centre sits at or outboard of the windward wall plane —
#: a stoop rail's SHAPE (long across the wall, thin along the row) makes
#: `_part_side` misread it as an S/N party-line part, so association with
#: an elevation has to come from PLANE position, not shape, exactly like
#: the cornice test above
RAIL_PLANE_TOL_M = 0.6
#: the corner bite: at most this share of the elevation's own width, and how
#: tall -- 2026-09-02 round 2 ("no missing chunks or brick bite"): both
#: raised so it reads as a real hole rather than a sliver, one unit per row
BITE_WIDTH_FRAC = (0.30, 0.48)
#: ~1.5 storeys down from the deck (was exactly one storey, `levels[-1]` to
#: `deck_z`) -- half the storey below the top floor is included too, capped
#: at `levels[-2]` so it never eats a second full storey
BITE_HEIGHT_STOREYS = 1.5
#: flown real timber (and, with their own smaller `RAIL_*` pair below, every
#: removed railing): tumble about a random horizontal axis through its own
#: centroid, then carried downwind `LUMBER_THROW_M * (0.6 + 0.6*speed_frac)`
#: plus lateral scatter, then dropped so ITS OWN lowest point sinks `SINK_M`
#: -- each piece is grounded individually, before the population is merged
#: into one authored mesh, so a merged mesh's global min being ~0 is not the
#: only thing keeping a piece off the air. THE TUMBLE CAP (2026-09-02 round
#: 2): the old (20, 90) let a 2-3 m board rotate up to 90 degrees about an
#: axis near-perpendicular to its own length, which stands its long axis
#: NEAR VERTICAL -- grounded at the foot, but reading as impaled/floating at
#: the top (bbox z 2.69 m, live-render review). Capped low enough that a
#: board settles mostly FLAT, at most a modest lean.
LUMBER_TUMBLE_DEG = (5.0, 32.0)
LUMBER_THROW_M = (3.0, 9.0)
LUMBER_JITTER_M = 2.0
#: a removed railing flies the SAME way (`_fly_piece`), but metal balustrade
#: does not carry like a board -- shorter throw, a shallower tumble cap (the
#: run is longer and stiffer than a single joist, so a big tumble reads even
#: less plausibly as "settled" than it does for lumber)
RAIL_TUMBLE_DEG = (5.0, 28.0)
RAIL_THROW_M = (1.2, 4.5)
#: the cracked-glass pane: a dark, still-translucent base bound onto the
#: pane that is HIT but does not blow out (2026-09-02 round 2) -- diffuse
#: tint, roughness, and how far it fogs the pane (`opacity_constant`; 0 is
#: invisible, 1 opaque -- see `_cracked_glass_material`, the same
#: `enable_opacity`/`opacity_threshold=0`/`opacity_mode=0` recipe
#: `urban_fire.materials` uses for the fire-glass ladder)
CRACK_PANE_RGB = (0.050, 0.056, 0.060)
CRACK_PANE_ROUGH = 0.30
CRACK_PANE_OPACITY = 0.60
#: the crack lines themselves: bright (a fracture SCATTERS light, so it
#: reads LIGHTER than the pane around it, never darker -- `urban_fire`'s own
#: `glass_crack` note), glossy, standing `CRACK_STANDOFF_M` proud of the
#: pane so it never z-fights it. `CRACK_N_STROKES` jagged, two-segment
#: strokes radiate from one point near an edge (thermal fracture roots at
#: the cool frame) per hit pane, `CRACK_WIDTH_M` wide.
CRACK_LINE_RGB = (0.58, 0.62, 0.64)
CRACK_LINE_ROUGH = 0.12
CRACK_STANDOFF_M = 0.012
CRACK_WIDTH_M = (0.025, 0.045)
CRACK_N_STROKES = (3, 6)
#: which world axis a pane's own THIN dimension runs along, by its side --
#: matches `_NORMALS` / `aec_burn._side_by_shape`'s own convention
_PANE_AXIS = {"W": 0, "E": 0, "S": 1, "N": 1}

_WOOD_MATS = ab._WOOD_MATS

#: outward normals, `quake_sliced.SIDES` order
_NORMALS = {"S": (0.0, -1.0), "E": (1.0, 0.0), "N": (0.0, 1.0),
            "W": (-1.0, 0.0)}


def side_weights(bearing_deg):
    """`tornado_urban.side_weights`'s non-`over` branch, on the four fixed
    axis normals: windward + leeward*0.35 + flank*0.55, clamped to [0, 1]."""
    brg = math.radians(float(bearing_deg))
    dx, dy = math.cos(brg), math.sin(brg)
    out = {}
    for sd, (nx, ny) in _NORMALS.items():
        dot = nx * dx + ny * dy
        w = max(0.0, -dot) + max(0.0, dot) * 0.35 + (1.0 - abs(dot)) * 0.55
        out[sd] = float(min(1.0, max(0.0, w)))
    return out


def _region_from_env():
    """Per-call `TU_PLATE_REGION` read — the same late-bound discipline
    `tornado_urban_usd.build_debris` uses (the bench arms it per cell)."""
    raw = os.environ.get("TU_PLATE_REGION", "")
    if not raw:
        return None
    try:
        v = tuple(float(t) for t in raw.split(","))
        return v if len(v) == 4 else None
    except ValueError:
        return None


def _clamp_xy(x, y, reg):
    if reg is None:
        return x, y
    return (min(max(x, reg[0] + 0.6), reg[2] - 0.6),
            min(max(y, reg[1] + 0.6), reg[3] - 0.6))


# ---------------------------------------------------------------------------
# merged laid-flat box meshes
# ---------------------------------------------------------------------------
def _author_tris_mesh(stage, path, V, N, mat):
    """One merged static Mesh of disconnected world-metre triangles `V`
    (T, 3, 3) with per-corner normals `N` (T, 3) -- the shared body of
    `_author_boxes` (ground debris) and `_pane_crack_tris`'s standoff
    quads."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    if V is None or not len(V):
        return None
    pts = V.reshape(-1, 3)
    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in pts])
    mesh.CreateFaceVertexCountsAttr([3] * int(V.shape[0]))
    mesh.CreateFaceVertexIndicesAttr(list(range(int(pts.shape[0]))))
    nrm = np.repeat(N, 3, axis=0)
    mesh.CreateNormalsAttr([Gf.Vec3f(*map(float, n)) for n in nrm])
    mesh.SetNormalsInterpolation("faceVarying")
    mesh.CreateExtentAttr([Gf.Vec3f(*map(float, pts.min(axis=0))),
                           Gf.Vec3f(*map(float, pts.max(axis=0)))])
    mesh.CreateDoubleSidedAttr(True)
    if mat is not None:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


def _author_boxes(stage, path, boxes, mat):
    """One merged static Mesh from `[(centre, size, yaw_deg), ...]` —
    `aec_burn._box_tris` boxes, tilt 0 (laid flat, exactly seated)."""
    if not boxes:
        return None
    V_all, N_all = [], []
    for centre, size, yaw in boxes:
        V, N = ab._box_tris(centre, size, yaw, 0.0)
        V_all.append(V)
        N_all.append(N)
    return _author_tris_mesh(stage, path, np.concatenate(V_all),
                             np.concatenate(N_all), mat)


def _laid(x, y, z_base, size, yaw):
    """A box laid FLAT with its bottom `SINK_M` below `z_base`."""
    return ((float(x), float(y), float(z_base) + 0.5 * size[2] - SINK_M),
            size, float(yaw))


def _merge_mds(mds):
    """Concatenate `aec_burn._mesh_dict_world`-shaped dicts (`P`, `tris`,
    `UV`, each already index-consistent) into ONE indexed mesh dict, so a
    whole population of flown parts becomes a single authored prim."""
    mds = [m for m in mds if m is not None and len(m.get("tris", ()))]
    if not mds:
        return None
    Ps, Ts, UVs, off = [], [], [], 0
    for m in mds:
        P = np.asarray(m["P"], dtype=np.float64)
        T = np.asarray(m["tris"], dtype=np.int64)
        UV = np.asarray(m["UV"], dtype=np.float64)
        Ps.append(P)
        UVs.append(UV)
        Ts.append(T + off)
        off += P.shape[0]
    return {"P": np.concatenate(Ps, axis=0), "tris": np.concatenate(Ts, axis=0),
            "UV": np.concatenate(UVs, axis=0)}


def _split_islands(md, weld_m=1e-4):
    """Break one killed part's mesh dict into its own physically CONNECTED
    islands, by welding coincident vertex POSITIONS (`aec_burn.
    _mesh_dict_world`'s triangles are fully DE-INDEXED — no two triangles
    share a vertex INDEX even where they share a vertex POSITION, so
    connectivity can only be found by position). Returns a list of mesh
    dicts, `[md]` unchanged when it is already one island.

    THE BUG THIS FIXES (2026-09-02 round 2, caught by the offline test that
    welds and re-splits an AUTHORED flown-debris mesh the same way): a
    single named part is not always one rigid lump — a `Railings`/
    `Top_Rails` prim is often several free-standing balusters that do not
    touch each other, sharing one mesh. `_fly_piece` tumbles and drops
    whatever it is handed as ONE rigid body, sinking its lowest point to
    the ground — which is correct for a real single board, but for a
    multi-island part it grounds only the ONE island that happens to be
    lowest and leaves every other, physically disconnected island exactly
    where the rigid transform put it, metres in the air. Splitting BEFORE
    `_fly_piece` (each island then gets its OWN tumble/throw/ground) turns
    that into what it should have been anyway — several balusters
    scattered independently, not one huge invisible rigid frame."""
    P = np.asarray(md["P"], dtype=np.float64)
    T = np.asarray(md["tris"], dtype=np.int64)
    UV = np.asarray(md["UV"], dtype=np.float64)
    n_tri = T.shape[0]
    if n_tri <= 1:
        return [md]
    key = np.round(P / weld_m).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = np.asarray(inv).reshape(-1)
    parent = list(range(n_tri))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    vert_to_tri = {}
    for ti in range(n_tri):
        for vi in T[ti]:
            vert_to_tri.setdefault(int(inv[vi]), []).append(ti)
    for tris_here in vert_to_tri.values():
        first = tris_here[0]
        rf = find(first)
        for other in tris_here[1:]:
            ro = find(other)
            if ro != rf:
                parent[ro] = rf

    groups = {}
    for ti in range(n_tri):
        groups.setdefault(find(ti), []).append(ti)
    if len(groups) <= 1:
        return [md]
    out = []
    for tri_idxs in groups.values():
        sub_T = T[tri_idxs]
        idx = np.unique(sub_T.reshape(-1))
        remap = {int(v): i for i, v in enumerate(idx)}
        new_T = np.asarray([[remap[int(v)] for v in row] for row in sub_T],
                           dtype=np.int64)
        out.append({"P": P[idx], "tris": new_T, "UV": UV[idx]})
    return out


def _fly_piece(md, rng, wdir, speed, reg, tumble_deg=None, throw_m=None,
               jitter_m=None):
    """A killed member's REAL mesh (`aec_burn._mesh_dict_world`, world
    metres) put through a rigid transform: (a) a random tumble rotation
    about a HORIZONTAL axis through its own centroid (`tumble_deg`, default
    `LUMBER_TUMBLE_DEG`), (b) a downwind translation scaled by `speed_frac`
    plus lateral scatter (`throw_m`, default `LUMBER_THROW_M`), (c) a drop so
    THIS PIECE'S OWN lowest point sinks `SINK_M` into the ground. A STATIC
    'flown and settled' result — no physics sim. The piece's post-transform
    centroid is clamped into the plate region exactly like every other
    ground scatter. `tumble_deg`/`throw_m` let a different population (a
    flown railing, `RAIL_TUMBLE_DEG`/`RAIL_THROW_M`) reuse this exact
    mechanism at its own scale instead of lumber's.

    GROUNDING IS PER PIECE, BY CONSTRUCTION: step (c) sinks THIS `md`'s own
    `P[:, 2].min()`, and the caller invokes this once per killed member,
    before `_merge_mds` concatenates the population into one authored mesh
    — so every piece touches down on its own, not just the merged mesh's
    global minimum (2026-09-02 round 2 review)."""
    P = np.asarray(md["P"], dtype=np.float64)
    c = P.mean(axis=0)
    theta = rng.uniform(0.0, 2.0 * math.pi)
    phi = math.radians(rng.uniform(*(tumble_deg or LUMBER_TUMBLE_DEG)))
    Rz = ab._rot_about_axis(2, theta)
    Rx = ab._rot_about_axis(0, phi)
    R = Rz @ Rx @ Rz.T                    # rotation about the horizontal
    Pr = (P - c) @ R.T + c                # axis `theta` picks out of Rz
    t = rng.uniform(*(throw_m or LUMBER_THROW_M)) * (0.6 + 0.6 * speed)
    jit = LUMBER_JITTER_M if jitter_m is None else jitter_m
    Pr[:, 0] += wdir[0] * t + rng.uniform(-jit, jit)
    Pr[:, 1] += wdir[1] * t + rng.uniform(-jit, jit)
    Pr[:, 2] += -float(Pr[:, 2].min()) - SINK_M
    cx, cy = float(Pr[:, 0].mean()), float(Pr[:, 1].mean())
    cxn, cyn = _clamp_xy(cx, cy, reg)
    Pr[:, 0] += (cxn - cx)
    Pr[:, 1] += (cyn - cy)
    out = dict(md)
    out["P"] = Pr
    return out


# ---------------------------------------------------------------------------
# cracked glass (2026-09-02 round 2): a hit windward pane stays IN, rebound
# to a dark translucent look, with a bright standoff crack pattern -- only
# the most extreme (windward, top storey, `LADDER[level]["blowout_top"]`)
# case still fully blows out.
# ---------------------------------------------------------------------------
def _cracked_glass_material(stage, cell, mats):
    """The cracked pane's own base material: dark, translucent (the
    `enable_opacity`/`opacity_threshold=0`/`opacity_mode=0` recipe
    `urban_fire.materials` uses for its fire-glass ladder), cached in
    `mats` — the SAME dict `tornado_urban_usd.debris_material` shares — so
    it is authored once per cell under `TornadoDebrisLooks` alongside every
    other tornado look."""
    from pxr import Sdf, UsdShade
    key = "aec_cracked_glass"
    got = mats.get(key)
    if got is not None:
        return got
    path = cell + "/TornadoDebrisLooks/" + key
    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing:
        mats[key] = existing
        return existing
    mat = dmg._pbr(stage, path, CRACK_PANE_RGB, CRACK_PANE_ROUGH)
    sh = UsdShade.Shader.Get(stage, Sdf.Path(path).AppendChild("Shader"))
    if sh:
        sh.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("opacity_constant",
                       Sdf.ValueTypeNames.Float).Set(float(CRACK_PANE_OPACITY))
        sh.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
        sh.CreateInput("opacity_mode", Sdf.ValueTypeNames.Int).Set(0)
    mats[key] = mat
    return mat


def _crack_line_material(stage, cell, mats):
    """The crack lines: bright, glossy — a fracture SCATTERS light, so it
    reads LIGHTER than the pane around it (`urban_fire`'s own `glass_crack`
    note), never as a dark scribble. Cached the same way as the pane."""
    from pxr import Sdf, UsdShade
    key = "aec_crack_line"
    got = mats.get(key)
    if got is not None:
        return got
    path = cell + "/TornadoDebrisLooks/" + key
    existing = UsdShade.Material.Get(stage, Sdf.Path(path))
    if existing:
        mats[key] = existing
        return existing
    mat = dmg._pbr(stage, path, CRACK_LINE_RGB, CRACK_LINE_ROUGH)
    mats[key] = mat
    return mat


def _pane_crack_tris(bb, side, rng):
    """A jagged spidered-crack pattern lying flush on one glass pane's own
    outward face, world metres — `(V, N)` triangle arrays shaped like
    `aec_burn._box_tris`'s own `(T, 3, 3)`/`(T, 3)`, meant to be pushed
    `CRACK_STANDOFF_M` proud by the caller (`aec_burn._author_tris`'s own
    `offset` argument — the same push-along-normal-then-convert-to-the-
    root's-local-frame idiom the soot layer uses, so the crack decal rides
    with the building through the eventual seat transform instead of being
    authored in a frame that will move out from under it).

    Rooted near one edge (thermal fracture starts at the cool frame, same
    physical story as `urban_fire._glass_pane`'s own crack step) with
    `CRACK_N_STROKES` two-segment jagged branches running toward the
    interior. None if the pane is too small to carry a crack at all."""
    axis = _PANE_AXIS.get(side)
    if axis is None:
        return None
    along = 1 - axis
    nx, ny = _NORMALS[side]
    ncomp = nx if axis == 0 else ny
    face = bb[axis] if ncomp < 0 else bb[3 + axis]
    u0, u1 = bb[along], bb[3 + along]
    z0, z1 = bb[2], bb[5]
    w, h = u1 - u0, z1 - z0
    if w <= 0.12 or h <= 0.12:
        return None
    edge = rng.random()
    if edge < 0.5:                          # from the head
        cu, cz = rng.uniform(u0 + 0.1 * w, u1 - 0.1 * w), z1 - 0.06 * h
    elif edge < 0.8:                        # from a jamb
        cu = u0 + 0.04 * w if rng.random() < 0.5 else u1 - 0.04 * w
        cz = rng.uniform(z0 + 0.15 * h, z1 - 0.15 * h)
    else:                                   # from the cill
        cu, cz = rng.uniform(u0 + 0.1 * w, u1 - 0.1 * w), z0 + 0.06 * h
    n = rng.randint(*CRACK_N_STROKES)
    n_world = np.zeros(3)
    n_world[axis] = ncomp
    V_all, N_all = [], []
    for _ in range(n):
        ang = rng.uniform(0.0, 2.0 * math.pi)
        length = rng.uniform(0.35, 0.85) * min(w, h)
        tu = min(max(cu + math.cos(ang) * length, u0 + 0.03 * w), u1 - 0.03 * w)
        tz = min(max(cz + math.sin(ang) * length, z0 + 0.03 * h), z1 - 0.03 * h)
        mu = cu + (tu - cu) * rng.uniform(0.4, 0.6) + rng.uniform(-0.06, 0.06) * w
        mz = cz + (tz - cz) * rng.uniform(0.4, 0.6) + rng.uniform(-0.06, 0.06) * h
        width = rng.uniform(*CRACK_WIDTH_M)
        for (au, az), (bu, bz) in (((cu, cz), (mu, mz)), ((mu, mz), (tu, tz))):
            du, dz = bu - au, bz - az
            seg = math.hypot(du, dz)
            if seg < 1e-4:
                continue
            ddu, ddz = du / seg, dz / seg
            wu, wz = -ddz * 0.5 * width, ddu * 0.5 * width

            def _pt(u, z, su):
                p = np.zeros(3)
                p[along] = u + su * wu
                p[2] = z + su * wz
                p[axis] = face
                return p

            p00, p10 = _pt(au, az, -1.0), _pt(bu, bz, -1.0)
            p11, p01 = _pt(bu, bz, 1.0), _pt(au, az, 1.0)
            V_all.append([p00, p10, p11])
            V_all.append([p00, p11, p01])
            N_all.append(n_world.copy())
            N_all.append(n_world.copy())
    if not V_all:
        return None
    return (np.asarray(V_all, dtype=np.float64),
            np.asarray(N_all, dtype=np.float64))


def _crack_glass_part(stage, mrec, side, rng, pane_mat, crack_tris, stats):
    """Rebind one hit glass sub-mesh to the cracked-glass pane (kept ACTIVE,
    never `_kill`ed), and queue its own standoff crack pattern for the
    caller's one merged `TornadoCracks` mesh."""
    from pxr import UsdShade
    prim = mrec["prim"]
    if prim and prim.IsValid():
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            pane_mat, UsdShade.Tokens.strongerThanDescendants)
    mrec["cracked"] = True
    stats["glass_cracked"] = stats.get("glass_cracked", 0) + 1
    got = _pane_crack_tris(mrec["bbox"], side, rng)
    if got is not None:
        crack_tris.append(got)


# ---------------------------------------------------------------------------
# the guarded top-corner brick bite (masonry loss — the ONLY one)
# ---------------------------------------------------------------------------
def _windward_corner_bite(stage, root_path, unit, windward_long, perp, along,
                          near_lo, rng, bricks, reg):
    """A top-corner brick bite out of the windward facade of ONE unit --
    reuses `aec_burn._mesh_dict_world` / `_clip_box` / `_write_world_piece`
    exactly the way `_lose_wall` clips the facade, but only
    `BITE_HEIGHT_STOREYS` down from the deck and at most `BITE_WIDTH_FRAC`
    of the elevation's own width, at the row's most-exposed corner. 'The
    tornado bit the corner' — never a collapse, the shell stands. Returns
    the tri count removed from the facade (0 if there was nothing to cut).
    The caller wraps this call in try/except so a clip failure never fails
    the cell."""
    from pxr import Sdf, Usd, UsdGeom, UsdShade
    wall = next((r for r in unit["meshes"] if r["cat"] == "Walls_Exterior"
                 and not r.get("dead")), None)
    if wall is None:
        return 0
    root = stage.GetPrimAtPath(root_path)
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage)) or 1.0
    bb, levels, deck_z = unit["bbox"], unit["levels"], unit["deck_z"]
    z1 = float(deck_z)
    # ~1.5 storeys down from the deck (2026-09-02 round 2: "no missing
    # chunks or brick bite" -- the old exactly-one-storey bite did not
    # read). Half the storey below the top floor is included too, capped at
    # `levels[-2]` so it can never eat a second full storey.
    if len(levels) >= 2:
        story_h = float(levels[-1]) - float(levels[-2])
        z0 = (max(float(levels[-2]), z1 - BITE_HEIGHT_STOREYS * story_h)
              if story_h > 0.5 else float(levels[-1]))
    else:
        z0 = float(levels[-1])
    if z1 - z0 < 0.5:
        return 0
    plane = unit["plane_lo"] if windward_long in ("W", "S") else unit["plane_hi"]
    # the through-wall reach: bay to street (outer clear) or through the
    # inner face (`WALL_THROUGH_M`), exactly `_lose_wall`'s own a_lo/a_hi
    if windward_long in ("W", "S"):
        a_lo, a_hi = bb[perp] - 0.5, plane + ab.WALL_THROUGH_M
    else:
        a_lo, a_hi = plane - ab.WALL_THROUGH_M, bb[3 + perp] + 0.5
    L0, L1 = bb[along], bb[3 + along]
    span = min(0.50, rng.uniform(*BITE_WIDTH_FRAC)) * (L1 - L0)
    c_lo, c_hi = (L0, L0 + span) if near_lo else (L1 - span, L1)
    lo = [0.0, 0.0, z0]
    hi = [0.0, 0.0, z1]
    lo[perp], hi[perp] = a_lo, a_hi
    lo[along], hi[along] = c_lo, c_hi
    md = ab._mesh_dict_world(wall["prim"], mpu, xf)
    if md is None:
        return 0
    removed = ab._clip_box(md, lo, hi, keep_outside=False)
    if removed is None or not len(removed.get("tris", ())):
        return 0
    kept = ab._clip_box(md, lo, hi, keep_outside=True)
    n_tris = int(len(removed["tris"]))
    if kept is not None and len(kept.get("tris", ())):
        src_mat, _ = UsdShade.MaterialBindingAPI(wall["prim"]).ComputeBoundMaterial()
        tag = unit["name"].rsplit("_", 1)[-1]
        scope = stage.DefinePrim(
            Sdf.Path(root_path).AppendChild("TornadoBite"), "Scope")
        new = ab._write_world_piece(
            stage, str(scope.GetPath()) + "/wall_{0}".format(tag),
            root, mpu, xf, kept, src_mat if src_mat else None)
        wall["prim"].SetActive(False)
        wall["prim"] = new.GetPrim()
        wall["path"] = str(new.GetPath())
    # the bitten brick, flung to the ground at that corner's foot
    nx, ny = _NORMALS[windward_long]
    corner_u = c_lo if near_lo else c_hi
    if perp == 0:
        fx = bb[0] if windward_long == "W" else bb[3]
        fy = corner_u
    else:
        fy = bb[1] if windward_long == "S" else bb[4]
        fx = corner_u
    n_b = max(8, min(40, round(span * (z1 - z0) * 3.2 * rng.uniform(0.85, 1.15))))
    for _ in range(n_b):
        size = CHUNK_M if rng.random() < 0.3 else BRICK_M
        d = rng.uniform(*FOOT_BAND_M)
        x = fx + nx * d + rng.uniform(-1.6, 1.6)
        y = fy + ny * d + rng.uniform(-1.6, 1.6)
        x, y = _clamp_xy(x, y, reg)
        bricks.append(_laid(x, y, 0.0, size, rng.uniform(0.0, 360.0)))
    return n_tris


# ---------------------------------------------------------------------------
# THE WRECK
# ---------------------------------------------------------------------------
def wreck_row(stage, cell, root_path, level, wind, rng, verbose=True):
    """Tornado-damage the AEC row under `root_path` (the `place_source`
    holder's `/asset` child) and author its ground evidence under
    `cell + "/tornado_debris"`. `wind` is the CELL-frame wind dict
    (`bearing_deg`, `speed_frac`); `rng` a seeded `random.Random`.
    Returns a stats dict (`glass`, `sash`, `roof_plant`, ... counts plus
    `n_units`, `windward`, `n_debris`)."""
    from pxr import Usd, UsdGeom
    spec = LADDER.get(level) or LADDER["T3"]
    meas = ab.measure_row(stage, root_path, verbose=verbose)
    perp = meas["perp"]
    along = 1 - perp
    mpu = meas["mpu"]
    xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    weights = side_weights(float(wind.get("bearing_deg", 0.0)))
    speed = float(wind.get("speed_frac", 0.8) or 0.8)
    brg = math.radians(float(wind.get("bearing_deg", 0.0)))
    wdir = (math.cos(brg), math.sin(brg))
    # the two LONG elevations (the ones with windows) face along `perp`
    long_sides = ("W", "E") if perp == 0 else ("S", "N")
    windward_long = max(long_sides, key=lambda s: weights[s])
    reg = _region_from_env()

    stats = {"n_units": len(meas["units"]), "windward": windward_long}

    # the corner bite touches AT MOST ONE unit per row: the row's own most
    # exposed end, picked from the flank (non-long) side the wind favours,
    # then whichever end unit actually sits on that side of the row
    bite_unit, near_lo = None, True
    if spec.get("corner_bite"):
        flank_sides = tuple(s for s in _NORMALS if s not in long_sides)
        flank = max(flank_sides, key=lambda s: weights.get(s, 0.0))
        near_lo = _NORMALS[flank][along] < 0.0
        centers = [(u, 0.5 * (u["bbox"][along] + u["bbox"][3 + along]))
                   for u in meas["units"]]
        pick = min if near_lo else max
        bite_unit = pick(centers, key=lambda t: t[1])[0]

    # de-instance EVERY unit (the tornado touches the whole row), then
    # refresh the measured prim handles — `aec_burn.damage_row`'s own order.
    for unit in meas["units"]:
        ip = unit.get("inst")
        if ip is not None and ip.IsInstanceable():
            ip.SetInstanceable(False)
            stats["deinstanced"] = stats.get("deinstanced", 0) + 1
    for unit in meas["units"]:
        for mrec in unit["meshes"]:
            mrec["prim"] = stage.GetPrimAtPath(mrec["path"])

    # debris accumulators: [(centre, size, yaw)] for the box populations,
    # [mesh dict, ...] for the REAL flown timber/railings; cell==world frame
    # here. `crack_tris`: [(V, N), ...] world-metre standoff quads, one
    # entry per cracked pane, authored under the BUILDING's own root (a
    # decal has to move with the window it is on). `dctx` is created here
    # (not down by the ground-evidence section) because the window loop
    # below needs it for the cracked-glass/crack-line looks.
    bricks, sash_sticks, plant_metal = [], [], []
    flown_lumber, flown_rail, crack_tris = [], [], []
    dctx = {"mats": {}, "parent": cell}
    pane_mat = _cracked_glass_material(stage, cell, dctx["mats"])

    for unit in meas["units"]:
        bb = unit["bbox"]
        deck_z = unit["deck_z"]
        levels = unit["levels"]
        n_st = max(1, len(levels) - 1)
        # the deck TOP (the membrane's own upper face) and which Roofs mesh
        # IS that membrane — `deck_z` is the area-max roof's bbox BOTTOM
        # and a brick placed there embeds; everything else `Roofs`-typed is
        # a smaller feature (cornice, coping, a bay roof) that never gets
        # confused with the deck because it loses the area vote
        roofs = [(m, abs((m["bbox"][3] - m["bbox"][0]) *
                         (m["bbox"][4] - m["bbox"][1])), m["bbox"][5])
                 for m in unit["meshes"] if m["cat"].startswith("Roofs")]
        if roofs:
            deck_mesh, _area, deck_top = max(roofs, key=lambda t: t[1])
            deck_top = float(deck_top)
        else:
            deck_mesh, deck_top = None, deck_z

        # --- windows: glass, then a share of sashes ----------------------
        groups = {}
        for mrec in unit["meshes"]:
            if mrec["cat"] == "Windows":
                groups.setdefault(ab._win_id(mrec["name"]), []).append(mrec)
        for wid, ms in groups.items():
            gb = [min(r["bbox"][i] for r in ms) for i in range(3)] + \
                 [max(r["bbox"][i] for r in ms) for i in range(3, 6)]
            side = ab._part_side(gb, unit, perp)
            st = ab._storey_of(levels, 0.5 * (gb[2] + gb[5]))
            grad = GRAD_LO + (GRAD_HI - GRAD_LO) * (st / max(1, n_st - 1))
            p = min(0.97, spec["glass"] * weights.get(side, 0.0) * grad)
            if rng.random() >= p:
                continue
            # a HIT pane either CRACKS (default -- kept, rebound, spidered)
            # or, on the single most extreme case (windward, the row's very
            # top storey), fully BLOWS OUT (`_kill`) -- 2026-09-02 round 2:
            # "can't really tell the windows are blown out ... we'd rather
            # have cracks there". A sash only comes off a pane that ALSO
            # blew out; a merely-cracked pane keeps its frame.
            top_storey = st >= len(levels) - 1
            blow = (side == windward_long and top_storey and
                    rng.random() < spec.get("blowout_top", 0.0))
            alu = [r for r in ms if not (r["mat"].startswith("Clear_Glass")
                                         or r["mat"] == "")]
            trim = max(alu, key=lambda r:
                       (r["bbox"][3] - r["bbox"][0]) * (r["bbox"][5] - r["bbox"][2])
                       + (r["bbox"][4] - r["bbox"][1]) * (r["bbox"][5] - r["bbox"][2])
                       ) if alu else None
            sash_hit = blow and spec["sash"] > 0.0 and \
                rng.random() < spec["sash"] * weights.get(side, 0.0)
            for r in ms:
                if r["mat"].startswith("Clear_Glass") or r["mat"] == "":
                    if blow:
                        ab._kill(stage, r, stats, "glass")
                    else:
                        _crack_glass_part(stage, r, side, rng, pane_mat,
                                          crack_tris, stats)
                elif r is not trim and sash_hit:
                    ab._kill(stage, r, stats, "sash")
            if sash_hit and side == windward_long:
                # two painted sticks at the foot below the window. The
                # OUTWARD coordinate starts at the UNIT bbox face, not the
                # window's wall plane — the stoop stands ~3.5 m proud of
                # the wall and a stick pushed from the plane embeds in it.
                nx, ny = _NORMALS[windward_long]
                fx = bb[0] if windward_long == "W" else (
                    bb[3] if windward_long == "E" else 0.5 * (gb[0] + gb[3]))
                fy = bb[1] if windward_long == "S" else (
                    bb[4] if windward_long == "N" else 0.5 * (gb[1] + gb[4]))
                for _ in range(2):
                    d = rng.uniform(*FOOT_BAND_M)
                    x = fx + nx * d + wdir[0] * rng.uniform(0.0, 2.0)
                    y = fy + ny * d + wdir[1] * rng.uniform(0.0, 2.0)
                    x, y = _clamp_xy(x, y, reg)
                    sash_sticks.append(_laid(x, y, 0.0, SASH_M,
                                             rng.uniform(0.0, 360.0)))

        # --- everything else, by category --------------------------------
        for mrec in unit["meshes"]:
            if mrec.get("dead") or mrec["cat"] == "Windows":
                continue
            b = mrec["bbox"]
            cat, mat = mrec["cat"], mrec["mat"] or ""
            wood = any(mat.startswith(w) for w in _WOOD_MATS)
            glassy = mat.startswith("Clear_Glass") or mat == "Glass"

            # front/rear door glass on a hard-hit side
            if cat == "Doors":
                if ab._is_interior(mrec, unit, perp, deck_z):
                    continue
                side = ab._part_side(b, unit, perp)
                if (glassy and spec["door_glass"]
                        and weights.get(side, 0.0) > 0.55
                        and rng.random() < 0.65):
                    ab._kill(stage, mrec, stats, "door_glass")
                continue

            # the windward cornice/coping: a small-footprint Roofs piece
            # (never the deck membrane itself) sitting near the windward
            # wall plane. Its brick joins the ground evidence at that
            # corner's foot; the big deck membrane is NEVER touched.
            if cat.startswith("Roofs"):
                if mrec is deck_mesh or not spec.get("cornice"):
                    continue
                c_perp = 0.5 * (b[perp] + b[3 + perp])
                plane = (unit["plane_lo"] if windward_long in ("W", "S")
                         else unit["plane_hi"])
                if abs(c_perp - plane) > CORNICE_PLANE_TOL_M:
                    continue
                ab._kill(stage, mrec, stats, "cornice")
                c_along = 0.5 * (b[along] + b[3 + along])
                nx, ny = _NORMALS[windward_long]
                if perp == 0:
                    fx = bb[0] if windward_long == "W" else bb[3]
                    fy = c_along
                else:
                    fy = bb[1] if windward_long == "S" else bb[4]
                    fx = c_along
                for _ in range(rng.randint(2, 4)):
                    size = CHUNK_M if rng.random() < 0.3 else BRICK_M
                    d = rng.uniform(*FOOT_BAND_M)
                    x = fx + nx * d + rng.uniform(-1.2, 1.2)
                    y = fy + ny * d + rng.uniform(-1.2, 1.2)
                    x, y = _clamp_xy(x, y, reg)
                    bricks.append(_laid(x, y, 0.0, size,
                                        rng.uniform(0.0, 360.0)))
                continue

            # roof plant + ALL railings (stoop AND roof) — `aec_burn.
            # damage_row`'s own plant category test minus the fire gating;
            # railings are no longer gated to roof height (2026-09-02: the
            # user wants the stoop rails gone too)
            is_plant = (0.5 * (b[2] + b[5]) >= deck_z - 0.05 and cat in
                        ("Mechanical_Equipment", "Specialty_Equipment"))
            is_rail = cat.startswith("Railings") or cat.startswith("Top_Rails")
            if is_plant:
                if spec["plant"] > 0.0 and rng.random() < spec["plant"]:
                    ab._kill(stage, mrec, stats, "roof_plant")
                    # thrown downwind as 1-2 dented chunks sized off the part
                    cx, cy = 0.5 * (b[0] + b[3]), 0.5 * (b[1] + b[4])
                    sx = min(1.0, max(0.25, 0.4 * (b[3] - b[0])))
                    sy = min(0.8, max(0.2, 0.4 * (b[4] - b[1])))
                    sz = min(0.5, max(0.12, 0.35 * (b[5] - b[2])))
                    for _ in range(rng.randint(1, 2)):
                        t = rng.uniform(*PLANT_THROW_M) * (0.6 + 0.6 * speed)
                        x = cx + wdir[0] * t + rng.uniform(-2.0, 2.0)
                        y = cy + wdir[1] * t + rng.uniform(-2.0, 2.0)
                        x, y = _clamp_xy(x, y, reg)
                        plant_metal.append(_laid(
                            x, y, 0.0,
                            (sx * rng.uniform(0.7, 1.2),
                             sy * rng.uniform(0.7, 1.2), sz),
                            rng.uniform(0.0, 360.0)))
                continue
            if is_rail:
                if spec["railing"] > 0.0:
                    plane = (unit["plane_lo"] if windward_long in ("W", "S")
                             else unit["plane_hi"])
                    c_perp = 0.5 * (b[perp] + b[3 + perp])
                    on_windward = ((c_perp <= plane + RAIL_PLANE_TOL_M)
                                  if windward_long in ("W", "S") else
                                  (c_perp >= plane - RAIL_PLANE_TOL_M))
                    if on_windward and spec.get("rail_all_windward"):
                        p = 1.0
                    else:
                        side = ab._part_side(b, unit, perp)
                        p = spec["railing"] * (0.6 + 0.4 * weights.get(side, 0.5))
                    if rng.random() < p:
                        # a removed railing is FLOWN, not vanished
                        # (2026-09-02 round 2: "have them scattered or
                        # something not just disappeared") -- the real
                        # mesh, tumbled and dropped exactly like the deck
                        # lumber, just at metal's own shorter/shallower
                        # `RAIL_THROW_M`/`RAIL_TUMBLE_DEG`. `_split_islands`
                        # FIRST: a Railings/Top_Rails part is often several
                        # free-standing balusters sharing one mesh, and
                        # flying that whole assembly as ONE rigid body grounds
                        # only whichever baluster is lowest, leaving the rest
                        # floating -- each island gets its own independent
                        # tumble/throw/ground instead.
                        piece = ab._mesh_dict_world(mrec["prim"], mpu, xf_cache)
                        ab._kill(stage, mrec, stats, "railing")
                        if piece is not None and len(piece.get("tris", ())):
                            for isl in _split_islands(piece):
                                flown_rail.append(_fly_piece(
                                    isl, rng, wdir, speed, reg,
                                    tumble_deg=RAIL_TUMBLE_DEG,
                                    throw_m=RAIL_THROW_M))
                                stats["rail_flown"] = stats.get("rail_flown", 0) + 1
                continue

            # the rear deck's timber — killed AND flown as its own mesh
            # (tumbled downwind, dropped on the ground): the "small proxy
            # board boxes" the 2026-09-02 review rejected are gone
            if cat == "Structural_Framing" and wood:
                side = ("E" if 0.5 * (b[0] + b[3]) > unit["cx"] else "W") \
                    if perp == 0 else \
                    ("N" if 0.5 * (b[1] + b[4]) > unit["cy"] else "S")
                p = spec["deck_lumber"] * (0.4 + 0.6 * weights.get(side, 0.5))
                if spec["deck_lumber"] > 0.0 and rng.random() < p:
                    piece = ab._mesh_dict_world(mrec["prim"], mpu, xf_cache)
                    ab._kill(stage, mrec, stats, "deck_lumber")
                    if piece is not None and len(piece.get("tris", ())):
                        # `_split_islands` first, same reason as the
                        # railings just above -- most deck members are one
                        # solid board already (one island, unchanged
                        # behaviour), but this still catches the rare
                        # multi-part member instead of assuming it away.
                        for isl in _split_islands(piece):
                            flown_lumber.append(_fly_piece(isl, rng, wdir,
                                                           speed, reg))
                            stats["lumber_flown"] = stats.get("lumber_flown", 0) + 1

        # --- brick sprinkle: windward foot + a few on the deck -----------
        n_b = rng.randint(*spec["bricks"])
        nx, ny = _NORMALS[windward_long]
        fx = bb[0] if windward_long == "W" else (
            bb[3] if windward_long == "E" else 0.5 * (bb[0] + bb[3]))
        fy = bb[1] if windward_long == "S" else (
            bb[4] if windward_long == "N" else 0.5 * (bb[1] + bb[4]))
        for i in range(n_b):
            size = CHUNK_M if rng.random() < 0.25 else BRICK_M
            if i < max(1, n_b // 3):
                # on the deck, just in from the windward parapet
                pl = unit["plane_lo"] if windward_long in ("W", "S") \
                    else unit["plane_hi"]
                u = pl - nx * rng.uniform(0.4, 2.0) if perp == 0 \
                    else pl - ny * rng.uniform(0.4, 2.0)
                if perp == 0:
                    x = u
                    y = rng.uniform(bb[1] + 0.5, bb[4] - 0.5)
                else:
                    x = rng.uniform(bb[0] + 0.5, bb[3] - 0.5)
                    y = u
                bricks.append(_laid(x, y, deck_top, size,
                                    rng.uniform(0.0, 360.0)))
            else:
                d = rng.uniform(*FOOT_BAND_M)
                x = fx + nx * d + rng.uniform(-2.5, 2.5) * abs(ny)
                y = fy + ny * d + rng.uniform(-2.5, 2.5) * abs(nx)
                x, y = _clamp_xy(x, y, reg)
                bricks.append(_laid(x, y, 0.0, size,
                                    rng.uniform(0.0, 360.0)))

    # --- the guarded top-corner brick bite (at most one unit) --------------
    if bite_unit is not None:
        try:
            n_bite = _windward_corner_bite(
                stage, root_path, bite_unit, windward_long, perp, along,
                near_lo, rng, bricks, reg)
            if n_bite:
                stats["corner_bite_tris"] = n_bite
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[aec_tornado] corner bite FAILED on {0}: {1}".format(
                bite_unit["name"], exc))

    # --- author the evidence, one merged mesh per look ---------------------
    from pxr import Sdf, UsdGeom
    scope = cell + "/tornado_debris"
    UsdGeom.Xform.Define(stage, Sdf.Path(scope))
    n_debris = 0
    if bricks:
        _author_boxes(stage, scope + "/aec_bricks", bricks,
                      tuu.debris_material(stage, dctx, "wall", "brick"))
        n_debris += len(bricks)
    if plant_metal:
        _author_boxes(stage, scope + "/aec_plant_metal", plant_metal,
                      tuu.debris_material(stage, dctx, "roof", "metal"))
        n_debris += len(plant_metal)
    if sash_sticks:
        _author_boxes(stage, scope + "/aec_sash_wood", sash_sticks,
                      planks.wood_material(
                          stage, cell + "/TornadoDebrisLooks/aec_sash_wood",
                          tile_m=0.5, tint=(0.62, 0.58, 0.52)))
        n_debris += len(sash_sticks)
    cell_prim = stage.GetPrimAtPath(cell)
    if flown_lumber:
        merged = _merge_mds(flown_lumber)
        if merged is not None and len(merged.get("tris", ())):
            mat = planks.wood_material(
                stage, cell + "/TornadoDebrisLooks/aec_flown_lumber",
                tile_m=1.1, tint=(0.80, 0.74, 0.66))
            ab._write_world_piece(stage, scope + "/aec_flown_lumber",
                                  cell_prim, mpu, xf_cache, merged, mat)
            n_debris += len(flown_lumber)
    if flown_rail:
        # every removed railing, scattered — the SAME dusted-metal look the
        # thrown roof plant already uses (`debris_material`'s bucket comes
        # from the MATERIAL string alone, so "railing"/"metal" resolves to
        # the identical cached material as "roof"/"metal")
        merged_r = _merge_mds(flown_rail)
        if merged_r is not None and len(merged_r.get("tris", ())):
            rail_mat = tuu.debris_material(stage, dctx, "railing", "metal")
            ab._write_world_piece(stage, scope + "/aec_railing_metal",
                                  cell_prim, mpu, xf_cache, merged_r, rail_mat)
            n_debris += len(flown_rail)
    stats["n_debris"] = n_debris

    # --- the cracked-glass standoff decals, ONE merged mesh under the
    # BUILDING's own root (it has to move with the window it is stuck to,
    # unlike the ground debris above) --------------------------------------
    if crack_tris:
        root_prim = stage.GetPrimAtPath(root_path)
        Vc = np.concatenate([t[0] for t in crack_tris])
        Nc = np.concatenate([t[1] for t in crack_tris])
        crack_scope = stage.DefinePrim(
            Sdf.Path(root_path).AppendChild("TornadoCracks"), "Scope")
        crack_mat = _crack_line_material(stage, cell, dctx["mats"])
        ab._author_tris(stage, str(crack_scope.GetPath()) + "/glass_cracks",
                        root_prim, mpu, xf_cache, Vc, Nc, None,
                        CRACK_STANDOFF_M, crack_mat)

    if verbose:
        summary = {k: v for k, v in sorted(stats.items())
                   if isinstance(v, int)}
        print("[aec_tornado] {0} windward {1}: {2}".format(
            level, windward_long,
            ", ".join("{0} {1}".format(k, v) for k, v in summary.items())))
    return stats
