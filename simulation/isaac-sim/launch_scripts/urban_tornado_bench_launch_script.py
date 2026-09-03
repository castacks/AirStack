#!/usr/bin/env python
"""
urban_tornado_bench — THE BENCH: `_plans/urban_tornado_plan.md` §8d, R12,
the user's own review vehicle, verbatim: *"run a bench instead of the full
scene. Show me the diff types of damage and human placements. That way it
comes up faster and we can iterate."* This REPLACES the full-scene launch
(`urban_tornado_city_launch_script.py`) as the review loop; that launcher
stays for the final cell.

A labelled grid of exemplar CELLS on a small flat asphalt plate — NO city
generator, no district/road-network layout. Each cell is built by the SAME
entry points the city launcher uses (`gac_fire.place_source` +
`gac_storey_slice.slice_to_kit` + `tornado_urban_usd.wreck_urban` for a
sliced GAC building; `tornado_kit.wreck_kit` for a kit-style building) — a
bench cell that used different code would prove nothing about the city.

    ISAAC_SIM_HEADLESS=true \\
    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/tornado_bench \\
    PYTHONUNBUFFERED=1 PYTHONPATH="$ISAAC_SIM_PYTHONPATH" \\
    /isaac-sim/python.sh \\
    /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/urban_tornado_bench_launch_script.py \\
    --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts --no-window

    # dry-structure check, NO Kit at all -- bare pxr + VTK + Nucleus:
    docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \\
      UTB_PLAN_ONLY=1 bash scene_gen/tools/usd_python.sh \\
      simulation/isaac-sim/launch_scripts/urban_tornado_bench_launch_script.py"

THE CELL TABLE (§8d)

    A1-A3  sliced GAC urm midrise (SM_Building_02) at T2 / T3 / T4
    A4     sliced rc_glass (SM_Building_24) at T3 -- envelope/glass case
    B1     kit brownstone_row T4 (+ t_facade_collapse if landed) + berm
    B2     kit dw_terrace T3 (cladding band, window knock-outs)
    B3     kit walkup T4 (chunk + top-storey loss)
    B4/B5  industrial shed, PARTIAL / TOTAL collapse (tornado_collapse, if
           landed) -- an intact Dmytro FactoryDistrict shed otherwise
    C1     street strip: streetlights/traffic light/bins/trees/cars on the
           shared asphalt plate, tornado_street.plan_street at i=0.85 if
           landed, ALWAYS a forced flipped+thrown car (tornado.car_pose)
    C2     ground-evidence swatch: berm/drift/stain gradient, bare asphalt
    D-row  people at the B1 berm, the B5 pile edge + refuge doorway, C1's
           struck car, and A2's standing entry -- `disaster.
           urban_tornado_people`, gated to T3/T4 cells (rule 1)

IMPORT-IF-PRESENT, EVERY ONE OF THESE. Streams RF/SP/C3 land asynchronously
and this bench must run at every intermediate state: `disaster.tornado_roof`
(`plan_roof`/`apply_roof`), `disaster.tornado_street` (`plan_street`),
`disaster.tornado_collapse` (`plan_industrial`/`apply_industrial`), and the
`t_facade_collapse` recipe inside `disaster.tornado_urban` (checked with
`hasattr`, not a separate import — that module itself is core stream L and
is not optional). Absence prints `[bench] SKIP <cell>: <module> not
available (<reason>)` and the cell still builds with whatever IS wired
(the ordinary T3/T4 ladder in `tornado_urban.LADDER_T`, an intact shed, a
standing street). `tornado_roof`/`t_facade_collapse` are wired INSIDE
`wreck_urban`/`wreck_kit` themselves once they land (nothing in THIS file
calls them directly) -- probed here only for the manifest's own module-
status line.

WIND: ONE fixed `tornado.wind_at` reading, off a synthetic core track
(the SAME right-flank rig `tools/tornado_urban_probe.py` uses: origin
`[0, 60]`, heading 35 -> measured bearing ~57.6), shared by every cell —
every holder's own yaw is 0, so the cell-local frame IS the world frame and
south (-Y, world bearing 270) is windward everywhere on this bench, with no
per-building wind-frame rotation to get wrong (contrast the city
launcher's own "THE WIND-FRAME ROTATION", which this bench does not need).

DUAL-MODE, for the plan-only structure check. This file imports `isaacsim`/
`omni.*`/`pegasus.*` ONLY when `UTB_PLAN_ONLY` is not set — every cell
builder function below takes a bare `Usd.Stage` (in-memory under
`UTB_PLAN_ONLY=1`, the live Kit-composed stage otherwise) and touches
nothing Kit-specific (no `omni.kit.app.get_app().update()`, no viewport);
`disaster.*`/`detail.*`/`scene_generator`/`suburb_scene`/`compile_disaster`
are all Kit-independent at import time (verified: none of them imports
`omni`/`isaacsim`/`carb` at module scope — the same property
`tools/tornado_urban_probe.py` already depends on). PLAN_ONLY skips
snapshots and the Kit environment/sky, prints the SAME manifest, and exits
immediately.

Env:
    UTB_CELLS       comma list of cell ids to build (default: all eleven
                    A/B/C cells; the D-row people always follow whichever
                    of B1/B5/C1/A2 were actually built).
    UTB_SEED        int, per-cell rng base (default 7). Each cell's own
                    seed is `UTB_SEED * 1000003 + <cell index>`, the same
                    convention `urban_tornado_city_launch_script.
                    select_damage`/`apply_damage` use.
    UTB_ASSET_CONFIG  a `SCENE_CONFIG`-shaped preset compiled ONLY for its
                    `asset_root`/material/street-furniture/human/shed pools
                    (`AssetPools`, `SizeResolver`) -- no layout is ever
                    built from it (default `downtown_tornado_bench_500`,
                    round 2's own urban preset).
    UTB_PLAN_ONLY   1 runs the dry-structure check: bare `Usd.Stage.
                    CreateInMemory()`, no Kit, no snapshots -- see "DUAL-
                    MODE" above. Quote its printed manifest table; do not
                    launch Isaac to get it.
    UTB_FAST_SLICE  1 (default) uses the artifact-safe complementary-sweep
                    slicer for canonical-cache misses.
    UTB_KIT_CACHE   1 opts into the experimental canonical GAC cache.
                    Default 0: cache material rehoming currently changes
                    tornado glazing classification, so correctness wins.
    SNAP_DIR        viewport captures (ignored under UTB_PLAN_ONLY), MUST
                    be under /isaac-sim/.nvidia-omniverse/logs/.
    KEEP_OPEN       1 keeps the Kit app up after captures (ignored under
                    UTB_PLAN_ONLY).
    ISAAC_SIM_HEADLESS  true for a headless Kit run.

Banner: `URBAN TORNADO BENCH DONE` / `URBAN TORNADO BENCH FAILED`, with a
per-cell BUILT/SKIPPED(reason)/FAILED status table. Target < 5 min to DONE
— the A-row slices are the only slow part (~6-70 s each, printed per cell).
"""

import importlib
import math
import os
import random
import sys
import time

import numpy as np


def _env(name, default=""):
    """Empty env var == absent (the container exports every launcher knob
    as an empty string) -- `urban_tornado_city_launch_script._env`, copied
    (this file cannot import that one: a second `SimulationApp` in one
    process is a segfault, see that file's own docstring)."""
    v = os.environ.get(name)
    return default if not (v or "").strip() else v.strip()


def _flag(name, default="0"):
    return _env(name, default).lower() in ("1", "true", "yes")


PLAN_ONLY = _flag("UTB_PLAN_ONLY", "0")
_HEADLESS = _env("ISAAC_SIM_HEADLESS", "false").lower() in ("1", "true", "yes")

KIT_ARGS = ["--/rtx/raytracing/fractionalCutoutOpacity=true",
            "--/rtx/pathtracing/fractionalCutoutOpacity=true"]

simulation_app = None
if not PLAN_ONLY:
    from isaacsim import SimulationApp
    if __name__ == "__main__":
        simulation_app = SimulationApp(
            launch_config={"headless": _HEADLESS, "extra_args": KIT_ARGS})

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

if not PLAN_ONLY:
    from isaacsim.core.utils.extensions import enable_extension   # noqa: E402
    enable_extension("omni.kit.window.script_editor")
    import carb                                                    # noqa: E402
    import omni.kit.app                                            # noqa: E402
    import omni.timeline                                           # noqa: E402
    import omni.usd                                                # noqa: E402
    from omni.isaac.core.world import World                        # noqa: E402
    from pegasus.simulator.params import SIMULATION_ENVIRONMENTS   # noqa: E402
    from pegasus.simulator.logic.interface.pegasus_interface import (  # noqa: E402
        PegasusInterface)

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade                    # noqa: E402

import scene_generator as sg                                       # noqa: E402
from suburb_scene import AssetPools, _raw_pool                      # noqa: E402
from compile_disaster import load_scene_config                     # noqa: E402
from detail import gac_storey_slice as gss                         # noqa: E402
from detail import kit_bake as kb                                  # noqa: E402
from disaster import fracture                                       # noqa: E402
from disaster import gac_fire as gcf                                # noqa: E402
from disaster import quake_flow as qf                                # noqa: E402
from disaster import quake_sliced as qs                             # noqa: E402
from disaster import tornado as tn                                  # noqa: E402
from disaster import tornado_kit as tk                              # noqa: E402
from disaster import tornado_urban as tu                            # noqa: E402
from disaster import tornado_urban_ground as tug                    # noqa: E402
from disaster import tornado_urban_usd as tuu                       # noqa: E402
from disaster import urban_tornado_people as utp                    # noqa: E402


def _try_import(dotted, attrs=()):
    """IMPORT-IF-PRESENT. Returns `(module_or_None, status_str)`."""
    try:
        mod = importlib.import_module(dotted)
    except ImportError as exc:
        return None, "not available ({0})".format(exc)
    missing = [a for a in attrs if not hasattr(mod, a)]
    if missing:
        return mod, "present but missing {0}".format(missing)
    return mod, "available"


tornado_roof, ROOF_STATUS = _try_import(
    "disaster.tornado_roof", ("plan_roof", "apply_roof"))
tornado_street, STREET_STATUS = _try_import(
    "disaster.tornado_street", ("plan_street", "apply_street"))
tornado_collapse, COLLAPSE_STATUS = _try_import(
    "disaster.tornado_collapse", ("plan_industrial", "apply_industrial"))
FACADE_COLLAPSE_STATUS = (
    "available" if hasattr(tu, "t_facade_collapse")
    else "not available (disaster.tornado_urban has no t_facade_collapse "
         "recipe yet -- the ordinary T3/T4 ladder runs instead)")

MODULE_STATUS = [
    ("disaster.tornado_roof (RF)", ROOF_STATUS),
    ("disaster.tornado_street (SP)", STREET_STATUS),
    ("disaster.tornado_collapse (C3)", COLLAPSE_STATUS),
    ("tornado_urban.t_facade_collapse (C3)", FACADE_COLLAPSE_STATUS),
]
for _name, _status in MODULE_STATUS:
    tag = "OK" if _status in ("available",) else "SKIP"
    print("[bench] {0} {1}: {2}".format(tag, _name, _status))


PARENT = "/World/tornado_bench"
GROUND_MAT_SCOPE = PARENT + "/materials"
FAMILY_OF_BTYPE = {"urm": "01", "rc": "02", "rc_glass": "05"}

UTB_SEED = int(_env("UTB_SEED", "7") or "7")
UTB_CELLS_RAW = _env("UTB_CELLS", "")
ASSET_CONFIG = _env("UTB_ASSET_CONFIG", "downtown_tornado_bench_500")
SNAP_DIR = _env("SNAP_DIR", "")
KEEP_OPEN = _flag("KEEP_OPEN", "0")
FAST_SLICE = _flag("UTB_FAST_SLICE", "1")
KIT_CACHE = _flag("UTB_KIT_CACHE", "0")

#: `Dmytro/Assets/Game/FactoryDistrict/Meshes/Building_TypeC_A.usd` --
#: 25.1 x 25.1 x 11.1 m, `config/asset_sets/urban_gac.yaml`'s own measured
#: comment against this exact entry (`usds.buildings.lowrise`) -- the
#: smallest of the eleven Dmytro sheds in that pool, picked so B4/B5 fit
#: the bench's 60 m column pitch with clearance either side.
SHED_USD_REL = "Dmytro/Assets/Game/FactoryDistrict/Meshes/Building_TypeC_A.usd"


# ---------------------------------------------------------------------------
# The grid. GRID_PITCH is the plan's own "60 m pitch"; row depth is a touch
# wider (70 m) so a 20-25 m building half-depth plus its own berm/street
# clearance does not brush the next row. Every holder's yaw is 0 (the plan's
# own "origin-frame Xform holder... yaw 0") -- see the module docstring's
# WIND section for what that buys.
# ---------------------------------------------------------------------------
GRID_PITCH = 60.0
A_Y, B_Y, C_Y = 70.0, 0.0, -70.0


def _centered(n, pitch):
    return [(i - (n - 1) / 2.0) * pitch for i in range(n)]


_A_X = _centered(4, GRID_PITCH)
_B_X = _centered(5, GRID_PITCH)

DEFAULT_CELL_ORDER = ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4", "B5"]
# Stable IDs are part of the visual fixture.  Filtering the bench must not
# redraw every later cell merely because an earlier optional cell is absent.
CELL_SEED_INDEX = {c: i for i, c in enumerate(
    ["A1", "A2", "A3", "A4", "B1", "B2", "B3", "B4", "B5", "C1", "C2"])}
CELL_ORDER = list(DEFAULT_CELL_ORDER)
CELLS = {
    "A1": {"row": "A", "kind": "gac", "asset": "SM_Building_02", "level": "T2",
          "intensity": 0.45, "x": _A_X[0], "y": A_Y,
          "seat": 180.0, "cell_bearing": 180.0},
    "A2": {"row": "A", "kind": "gac", "asset": "SM_Building_02", "level": "T3",
          "intensity": 0.65, "x": _A_X[1], "y": A_Y,
          "seat": 180.0, "cell_bearing": 180.0},
    "A3": {"row": "A", "kind": "gac", "asset": "SM_Building_02", "level": "T4",
          "intensity": 0.85, "x": _A_X[2], "y": A_Y,
          "seat": 180.0, "cell_bearing": 180.0},
    "A4": {"row": "A", "kind": "gac", "asset": "SM_Building_24", "level": "T3",
          "intensity": 0.65, "x": _A_X[3], "y": A_Y,
          "seat": 90.0, "cell_bearing": 270.0},
    "B1": {"row": "B", "kind": "kit", "style": "brownstone_row", "level": "T4",
          "intensity": 0.85, "x": _B_X[0], "y": B_Y},
    "B2": {"row": "B", "kind": "kit", "style": "dw_terrace", "level": "T3",
          "intensity": 0.65, "x": _B_X[1], "y": B_Y},
    "B3": {"row": "B", "kind": "kit", "style": "walkup", "level": "T4",
          "intensity": 0.85, "x": _B_X[2], "y": B_Y},
    "B4": {"row": "B", "kind": "industrial", "grade": "partial",
          "intensity": 0.6, "x": _B_X[3], "y": B_Y},
    "B5": {"row": "B", "kind": "industrial", "grade": "total",
          "intensity": 0.8, "x": _B_X[4], "y": B_Y},
    "C1": {"row": "C", "kind": "street", "intensity": 0.85, "length_m": 100.0,
          "x": -30.0, "y": C_Y},
    "C2": {"row": "C", "kind": "ground", "span_m": 50.0, "x": 130.0, "y": C_Y},
}
if UTB_CELLS_RAW:
    _wanted = {c.strip().upper() for c in UTB_CELLS_RAW.split(",") if c.strip()}
    CELL_ORDER = [c for c in CELL_SEED_INDEX if c in _wanted]

#: The plate -- big enough to hold every cell above plus a margin. (x0, y0,
#: x1, y1), metres.
PLATE = (-200.0, -120.0, 200.0, 120.0)


def _safe_token(s):
    s = str(s or "unknown")
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in s)
    return out or "unknown"


def _seed_for(cell_id):
    """`UTB_SEED * 1000003 + <index in CELL_ORDER>` -- the SAME per-record
    seed convention `urban_tornado_city_launch_script.select_damage`/
    `apply_damage` use, so a bench cell and its city counterpart draw from
    the same kind of stream (not the SAME draw -- the index space differs
    -- but the same recipe)."""
    i = CELL_SEED_INDEX.get(cell_id, 997)
    return UTB_SEED * 1000003 + i


def _synth_wind_cfg():
    """The right-flank rig `tools/tornado_urban_probe.py` uses (origin
    [0, 60], heading 35), reused verbatim so this bench's wind is the SAME
    well-measured case the bug catalogue already documents (bearing ~57.6,
    windward S) rather than a new, unverified scenario."""
    cfg = dict(tn.DEFAULTS)
    cfg.update({"origin_m": [0.0, 60.0], "heading_deg": 35.0, "width_m": 300.0,
               "wobble_m": 0.0, "edge_noise_m": 0.0, "along_min": 1.0,
               "width_min": 1.0})
    return cfg


WIND_CFG = _synth_wind_cfg()
WIND = tn.wind_at(WIND_CFG, 0.0, 0.0)
#: South (-Y) is windward for this bearing on every yaw-0 cell -- measured
#: the same way the probe's own `_windward_side` does: of the four cell-
#: local outward normals, S=(0,-1) has the most negative dot with the wind
#: direction. Pinned as a constant (not re-derived per cell) because every
#: holder shares one yaw.
WINDWARD_BEARING_DEG = 270.0


def place_holder(stage, stem, x, y, z, yaw_deg, ssf):
    """A fresh origin-frame holder + child `cell` Xform -- `urban_tornado_
    city_launch_script.place_holder`'s own "THE TRANSFORM TRAP" seat, copied
    (this file cannot import that launcher). `cell` is an Xform, never a
    Scope (a Scope carries no transform op order)."""
    holder = "{0}/{1}".format(PARENT, _safe_token(stem))
    xf = UsdGeom.Xform.Define(stage, Sdf.Path(holder))
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x) * ssf, float(y) * ssf,
                                     float(z) * ssf))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    xf.AddScaleOp().Set(Gf.Vec3f(float(ssf), float(ssf), float(ssf)))
    cell = holder + "/cell"
    UsdGeom.Xform.Define(stage, Sdf.Path(cell))
    return holder, cell


def _seat_holder(stage, holder, x, y, z, yaw_deg, ssf):
    """Round 4 -- THE TRANSFORM TRAP, closed for real. `describe`'s masses,
    the tear pass's replacement meshes and `fit_interior` all compute in
    WORLD frame and author under the cell, so a holder that already carries
    its bench offset/yaw gets that transform applied TWICE -- the headless
    audit measured brk_*/fit subtrees 10-118 m off their buildings, at
    storey heights (the "floating random things"). Wreck at an IDENTITY
    holder, then seat it here, after the wreck, so everything under it
    (pieces, tears, fit-out, backing, debris) rides the move together.
    NOTE: `urban_tornado_city_launch_script` has the same latent bug --
    its place_holder is also set BEFORE the wreck. Fix it there before the
    next city build."""
    xf = UsdGeom.Xform(stage.GetPrimAtPath(Sdf.Path(holder)))
    for op in xf.GetOrderedXformOps():
        t = op.GetOpType()
        if t == UsdGeom.XformOp.TypeTranslate:
            op.Set(Gf.Vec3d(float(x) * ssf, float(y) * ssf, float(z) * ssf))
        elif t == UsdGeom.XformOp.TypeRotateXYZ:
            op.Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))


def _bbox(stage, path):
    """World-space aligned bbox of *path*, in STAGE units -- `tools/
    tornado_urban_probe.py._bbox`, copied verbatim."""
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim or not prim.IsValid():
        return None
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    lo, hi = r.GetMin(), r.GetMax()
    return (float(lo[0]), float(lo[1]), float(lo[2]),
            float(hi[0]), float(hi[1]), float(hi[2]))


def windward_wall_anchor(stage, cell_path, ssf, offset_out_m=0.0):
    """The measured south face of whatever is authored under *cell_path*,
    as an `urban_tornado_people.anchor` in METRES (the bbox comes back in
    stage units, divided by `ssf`). South is windward for every cell on
    this bench -- see `WINDWARD_BEARING_DEG`'s own comment. `offset_out_m`
    pushes the anchor POINT itself out from the measured wall face (0 =
    exactly on it), for callers that want a doorway/entry point rather than
    the wall line itself."""
    bb = _bbox(stage, cell_path)
    if bb is None:
        return None
    x0, y0, _z0, x1, _y1, _z1 = bb
    cx = 0.5 * (x0 + x1) / ssf
    wall_y = y0 / ssf
    along = (x1 - x0) / ssf
    a = utp.anchor(cx, wall_y, WINDWARD_BEARING_DEG, along_m=along)
    if offset_out_m:
        x, y = utp._along_wall(a, 0.5, offset_out_m)
        a = utp.anchor(x, y, WINDWARD_BEARING_DEG, along_m=along)
    return a


def footprint_wall_anchor(cx, cy, W, D, offset_out_m=0.0):
    """The SAME windward-wall-anchor geometry as `windward_wall_anchor`,
    from a KNOWN world centre + measured pre-damage footprint (`W` along
    local X, `D` along local Y, both metres) instead of a live stage bbox.

    B4/B5's `cell` bbox, read back AFTER `tornado_collapse.
    apply_industrial` runs, includes the FALLEN debris field -- panels lie
    flat outward from the walls (`plan_industrial`'s own panels/rubble),
    so `windward_wall_anchor`'s bbox-of-the-whole-cell approach would
    measure a misleadingly large "wall" run and an outward-shifted wall
    line. `build_industrial_cell` measures W/D from the INTACT shed,
    before the collapse is authored, and this function places the anchor
    against THAT footprint instead."""
    wall_y = cy - float(D) / 2.0
    a = utp.anchor(cx, wall_y, WINDWARD_BEARING_DEG, along_m=float(W))
    if offset_out_m:
        x, y = utp._along_wall(a, 0.5, offset_out_m)
        a = utp.anchor(x, y, WINDWARD_BEARING_DEG, along_m=float(W))
    return a


# ---------------------------------------------------------------------------
# Ground plate -- one asphalt quad under the whole bench. `_load_mat`
# mirrors `scene_generator.apply_ground_planes`'s own inner closure (that
# one is nested and not importable); the SAME material-USD-reference idiom.
# ---------------------------------------------------------------------------

def _load_mat(stage, config, key, asset_root):
    mat_cfg = (config.get("usds", {}) or {}).get("materials", {}) or {}
    url = mat_cfg.get(key, "")
    if not url:
        return ""
    url = sg._join_asset_root(url, asset_root)
    prim_path = GROUND_MAT_SCOPE + "/" + key
    prim = stage.DefinePrim(Sdf.Path(prim_path))
    prim.GetReferences().AddReference(url)
    prim.Load()
    return prim_path


def build_ground(stage, ssf, config):
    UsdGeom.Scope.Define(stage, Sdf.Path(GROUND_MAT_SCOPE))
    asset_root = str(config.get("asset_root", "") or "")
    asphalt = _load_mat(stage, config, "asphalt", asset_root)
    x0, y0, x1, y1 = PLATE
    sg._make_plane_mesh(stage, PARENT + "/ground_plate", x0, y0, x1, y1, 0.0,
                        4.0, ssf, display_color=(0.15, 0.15, 0.15),
                        mat_prim_path=asphalt)
    print("[bench] ground plate {0:.0f} x {1:.0f} m, material={2}".format(
        x1 - x0, y1 - y0, asphalt or "(none -- flat colour only)"))


# ---------------------------------------------------------------------------
# A-row: sliced GAC buildings. Same chain `tools/tornado_urban_probe.py`
# and `urban_tornado_city_launch_script.apply_damage` both run.
# ---------------------------------------------------------------------------

def build_a_cell(stage, cell_id, spec, ssf):
    # Round 4 (stream S's measured verdict): both GAC slices model their N
    # (+ E) elevations and fill S/W with the blind M_Building_01_WallBack
    # quad -- and this bench's wind AND camera both hit the S/W faces, so
    # every A-row capture showed filler mottling ("mismatched patches") and
    # A4 a bare wall. The holder ends up yawed 180 deg so cell-N faces
    # world-S -- but it is placed at IDENTITY here and seated only AFTER
    # the wreck (see `_seat_holder`: the world-frame machinery must run at
    # the origin or everything it authors is transformed twice).
    holder, cell = place_holder(stage, cell_id, 0.0, 0.0, 0.0, 0.0, ssf)
    kind, asset = gcf.split_kind(spec["asset"], spec.get("kind"))
    pack = gcf.PACKS[kind]
    url = gcf.asset_url(asset, kind)
    scale = gcf.asset_scale(url, pack["scale"], verbose=False)
    btype_guess = qs.construction_type(asset)
    family = FAMILY_OF_BTYPE.get(btype_guess, "01")
    # The canonical loader registers this asset name as the synthetic kit
    # style. Use it on misses too, making cached and uncached plans identical.
    style = asset
    force_regular = asset in (pack.get("force_regular_grid") or ())
    cache_hit = KIT_CACHE and kb.have_kit(asset, None)
    if cache_hit:
        pls, _grid, _measured = kb.load_kit(
            stage, cell, asset, ssf=1.0, signature=None)
        print("[bench] canonical slice HIT {0}: {1} pieces".format(
            asset, len(pls)))
    else:
        src = gcf.place_source(stage, cell, url, scale)
        if not src:
            raise RuntimeError(
                "place_source composed nothing for {0}".format(asset))
        pls, _grid, _measured = gss.slice_to_kit(
            stage, src, cell, style, region=None, family=family,
            verbose=False, force_regular=force_regular)
        # A one-off asset would pay an extra full slice without helping this
        # iteration. Cache only assets selected more than once.
        repeated = sum(1 for cid in CELL_ORDER
                       if CELLS[cid].get("asset") == asset) > 1
        if KIT_CACHE and repeated:
            rec = kb.save_kit(
                asset, None, url, scale, asset, region=None, family=family,
                force_regular=force_regular, verbose=False)
            print("[bench] canonical slice {0} {1}".format(
                "STORED" if rec else "STORE FAILED", asset))

    seed = _seed_for(cell_id)
    seat = float(spec.get("seat", 0.0))
    wind_cell = dict(WIND)
    wind_cell["bearing_deg"] = float(spec.get(
        "cell_bearing", float(WIND["bearing_deg"]) - seat))
    ctx = tuu.wreck_urban(
        stage, cell, pls, style, spec["level"], random.Random(seed),
        np.random.default_rng(seed), {}, "utb", wind_cell, btype=None,
        height_class=None, intensity=spec["intensity"], usd=asset,
        verbose=False)
    counts = ctx.get("counts") or {}
    _seat_holder(stage, holder, spec["x"], spec["y"], 0.0, seat, ssf)
    return {"holder": holder, "cell": cell, "n_pieces": len(pls),
            "ctx": ctx, "counts": counts,
            "summary": "{0} ({1}) T={2} pieces={3} removed={4} glass={5} "
                       "debris={6}".format(
                           asset, btype_guess, spec["level"], len(pls),
                           counts.get("n_removed", 0),
                           counts.get("n_glass", 0),
                           counts.get("n_fragments", 0))}


# ---------------------------------------------------------------------------
# B-row (kit styles): brownstone_row / dw_terrace / walkup. Same
# `tornado_kit.wreck_kit` the city launcher's kit pass calls.
# ---------------------------------------------------------------------------

def build_b_kit_cell(stage, cell_id, spec, ssf):
    # Round 4: identity holder during the wreck, seated after -- see
    # `_seat_holder` (the transform trap; B2's displaced cornices and B1's
    # 158 m-wide `parts` bbox were this).
    holder, cell = place_holder(stage, cell_id, 0.0, 0.0, 0.0, 0.0, ssf)
    seed = _seed_for(cell_id)
    ctx = tk.wreck_kit(
        stage, cell, spec["style"], spec["level"], random.Random(seed),
        np.random.default_rng(seed), {}, "utb", WIND, seed=seed % 1000,
        height_class=None, intensity=spec["intensity"], ssf=1.0,
        verbose=False)
    counts = ctx.get("counts") or {}
    n_pieces = int((ctx.get("kit") or {}).get("n_pieces", 0))
    _seat_holder(stage, holder, spec["x"], spec["y"], 0.0, 0.0, ssf)
    return {"holder": holder, "cell": cell, "n_pieces": n_pieces,
            "ctx": ctx, "counts": counts,
            "summary": "{0} T={1} pieces={2} removed={3} glass={4} "
                       "debris={5}".format(
                           spec["style"], spec["level"], n_pieces,
                           counts.get("n_removed", 0),
                           counts.get("n_glass", 0),
                           counts.get("n_fragments", 0))}


# ---------------------------------------------------------------------------
# B4/B5: industrial shed. `disaster.tornado_collapse` (`plan_industrial`/
# `apply_industrial`) is IMPORT-IF-PRESENT and, per this round's actual
# state, absent -- the shed is referenced INTACT either way (a whole-mesh
# Dmytro placement, never a sliceable gac/dtc pack, so there is no
# `wreck_urban`-shaped path for it regardless of `tornado_collapse`'s own
# state). Resolved through the SAME `AssetPools`/`SizeResolver` machinery
# every human/car/street-furniture placement in this codebase uses.
# ---------------------------------------------------------------------------

def build_industrial_cell(stage, cell_id, spec, ssf, config, resolver, pools):
    holder, cell = place_holder(stage, cell_id, spec["x"], spec["y"], 0.0,
                               0.0, ssf)
    raw_lowrise = _raw_pool(config, "buildings", "lowrise")
    shed_paths = pools.load(raw_lowrise)
    asset_root = str(config.get("asset_root", "") or "")
    shed_url = sg._join_asset_root(SHED_USD_REL, asset_root)
    if shed_url not in shed_paths:
        cands = [p for p in shed_paths if "FactoryDistrict" in p]
        if cands:
            shed_url = cands[0]
    scale = pools.scale_of(shed_url)
    # NO axis_up/yaw-offset/roll correction on this reference (`place_
    # source` applies none) -- checked against `urban_gac.yaml`'s own
    # entry for this shed: a bare relative-path string, no `axis_up`/
    # `yaw-offset`/`roll` override, so `AssetPools` defaults every one of
    # `pools.axis_of/yaw_of/roll_of(shed_url)` to Z-up/0/0 anyway (Dmytro's
    # own FactoryDistrict export convention). `place_source` (`gac_fire`'s,
    # generic over ANY usd + scale, not gac/dtc-specific) is used here
    # rather than the generic `apply_placements` path THIS FILE'S FIRST
    # CUT used, because `tornado_collapse.apply_industrial` expects the
    # intact source at `ctx["parent"] + "/src"` (its own default
    # `src_path`, deactivated as part of authoring the collapse) -- the
    # SAME `<cell>/src` seat `wreck_urban`'s own gac/dtc pass uses.
    src = gcf.place_source(stage, cell, shed_url, scale)
    if not src:
        raise RuntimeError(
            "place_source composed nothing for shed {0}".format(shed_url))
    bb = _bbox(stage, cell)
    if bb is None:
        raise RuntimeError(
            "could not measure shed bbox for {0}".format(shed_url))
    W = (bb[3] - bb[0]) / ssf
    D = (bb[4] - bb[1]) / ssf
    H = (bb[5] - bb[2]) / ssf

    counts = {}
    if tornado_collapse is not None and hasattr(
            tornado_collapse, "plan_industrial") and hasattr(
            tornado_collapse, "apply_industrial"):
        grade = spec["grade"]
        rng = random.Random(_seed_for(cell_id))
        # `plan_industrial`'s own x/y/yaw are the CELL-LOCAL placement --
        # always (0, 0, 0) here, the same "a cell at the origin, like
        # every other wreck_* pass" convention `apply_industrial`'s own
        # docstring names. `WIND` is passed unrotated -- every holder on
        # this bench carries yaw 0, so cell-local IS world (see the module
        # docstring's WIND section).
        plan = tornado_collapse.plan_industrial(
            W, D, H, 0.0, 0.0, 0.0, grade, WIND, rng)
        ctx = {"parent": cell, "mats": {}, "verbose": False}
        counts = tornado_collapse.apply_industrial(
            stage, ctx, plan, verbose=False)
        note = ("tornado_collapse.plan_industrial+apply_industrial: "
                "grade={0} panels={1} roof_sheets={2} rubble={3} "
                "src_deactivated={4}".format(
                    grade, counts.get("n_panels", 0),
                    counts.get("n_roof_sheets", 0),
                    counts.get("n_rubble", 0),
                    bool(counts.get("n_src_deactivated"))))
        print("[bench] {0}: {1}".format(cell_id, note))
    else:
        note = "intact shed (tornado_collapse {0})".format(COLLAPSE_STATUS)
        print("[bench] SKIP {0}: disaster.tornado_collapse {1}".format(
            cell_id, COLLAPSE_STATUS))

    return {"holder": holder, "cell": cell,
            "n_pieces": int(counts.get("n_meshes", 1)), "ctx": {},
            "counts": counts, "shed_usd": shed_url,
            "W": W, "D": D, "H": H,
            "summary": "{0} ({1:.1f}x{2:.1f}x{3:.1f}m, {4}) [{5}]".format(
                os.path.basename(shed_url), W, D, H, spec["grade"], note)}


# ---------------------------------------------------------------------------
# C1: street strip. `disaster.tornado_street` (`plan_street`) is IMPORT-IF-
# PRESENT and, per this round's actual state, absent -- felled/carried/
# snapped furniture is SKIPPED, but the flipped+thrown car is ALWAYS
# authored directly via `tornado.car_pose`/`tornado.toss_prim` (those exist
# now), per the plan's own "ALWAYS include" instruction.
# ---------------------------------------------------------------------------

def _prop_placement(pools, resolver, usd, x, y, yaw, category="prop"):
    sc = pools.scale_of(usd)
    au = pools.axis_of(usd)
    fp = resolver.get(usd, category, scale=sc, axis_up=au)
    return {"usd": usd, "x_m": x, "y_m": y, "z_m": float(fp.get("base", 0.0)),
            "yaw_deg": float(yaw) + pools.yaw_of(usd),
            "roll_deg": pools.roll_of(usd), "pitch_deg": 0.0, "scale": sc,
            "category": category, "axis_up": au}


#: The category vocabulary `disaster.tornado_street.plan_street` actually
#: reads (`FELLED`/`CARRIED`/its own `street_tree`/`car` branches) -- a
#: bare `"prop"`/`"tree"` (this file's FIRST cut, before `apply_street`
#: landed) is invisible to it: every one of those items falls through to
#: "UNTOUCHED or an unmapped category" and is silently left standing.
_STREETLIGHT_CAT = "streetlight"
_SIGNAL_CAT = "traffic_light"
_BIN_CAT = "trash_can"
_TREE_CAT = "street_tree"
_CAR_CAT = "car"


def build_c1_cell(stage, cell_id, spec, ssf, config, resolver, pools):
    holder, cell = place_holder(stage, cell_id, spec["x"], spec["y"], 0.0,
                               0.0, ssf)
    rng = random.Random(_seed_for(cell_id))
    length = float(spec.get("length_m", 100.0))
    intensity = float(spec["intensity"])

    lights = pools.load(_raw_pool(config, "streetlights"))
    bins_ = pools.load(_raw_pool(config, "trash_cans"))
    signals = pools.load(_raw_pool(config, "traffic_lights"))
    # Round 4 (stream T): the STREET tree pool, not the park pool --
    # urban.yaml's own comment: Black_Oak (19.7 m tall, 25.4 m crown) "is a
    # park specimen, not a kerb tree", and measured its crown-centre offset
    # never exceeds 0.79x its own crown radius at any lean, so it can never
    # read fallen from above (bench-v3's giant middle "standing" tree).
    trees = pools.load(_raw_pool(config, "street_trees"))
    cars = pools.load(_raw_pool(config, "cars"))

    placements = []
    for i, sx in enumerate((-0.35, -0.12, 0.12, 0.35)):
        if not lights:
            break
        x = sx * length
        side = 5.0 if i % 2 == 0 else -5.0
        placements.append(_prop_placement(
            pools, resolver, lights[i % len(lights)], x, side,
            rng.uniform(0.0, 360.0), category=_STREETLIGHT_CAT))
    if signals:
        placements.append(_prop_placement(
            pools, resolver, signals[0], -0.42 * length, 6.0,
            rng.uniform(0.0, 360.0), category=_SIGNAL_CAT))
    for i, sx in enumerate((-0.25, 0.05, 0.30)):
        if not bins_:
            break
        placements.append(_prop_placement(
            pools, resolver, bins_[i % len(bins_)], sx * length,
            -5.5 if i % 2 else 5.5, rng.uniform(0.0, 360.0),
            category=_BIN_CAT))
    for i, sx in enumerate((-0.30, 0.0, 0.30)):
        if not trees:
            break
        placements.append(_prop_placement(
            pools, resolver, trees[i % len(trees)], sx * length,
            7.5 if i % 2 == 0 else -7.5, rng.uniform(0.0, 360.0),
            category=_TREE_CAT))

    # THREE cars, not two -- `min_tipped=2` (the coordinator's own floor)
    # forces the two HIGHEST-intensity car candidates to tip if fewer than
    # two already have; at a uniform i=0.85 every candidate ties on
    # intensity, so `plan_street`'s own ordinary pass decides which ones
    # move first and the floor only tops up the shortfall. A THIRD car
    # gives the `car_occupant` D-row anchor (rule 3: most struck-car
    # occupants SURVIVE) a real chance at a shoved-not-toppled or even
    # untouched car to sit in, rather than guaranteeing every car on the
    # strip ends up dramatically flipped.
    car_placements = []
    if cars:
        for i, (sx, sy) in enumerate(((-0.30, 3.0), (-0.05, -3.5),
                                      (0.20, 3.5))):
            usd_c = cars[i % len(cars)]
            yaw0 = rng.uniform(0.0, 360.0)
            p = _prop_placement(pools, resolver, usd_c, sx * length, sy,
                               yaw0, category=_CAR_CAT)
            placements.append(p)
            car_placements.append(p)

    if placements:
        # ONE `apply_placements` call, ALL props (furniture + cars) --
        # `plan_street` needs every placement's REAL `prim_path`
        # (`apply_placements` mutates each dict in place with it), so this
        # must run BEFORE `plan_street` is ever called.
        sg.apply_placements(stage, placements, cell + "/furniture", ssf,
                            resolver=resolver, instance_categories=set())

    car_recs = {}
    street_counts = {}
    if tornado_street is not None and hasattr(
            tornado_street, "plan_street") and hasattr(
            tornado_street, "apply_street"):
        # Confirmed signatures (coordinator, 2026-09-01): `plan_street(
        # placements, intensity_fn, wind_at_fn, rng, *, min_tipped=...) ->
        # actions`; `apply_street(stage, actions, *, min_tipped=..., ...) ->
        # counts`. C1 has no spatial field of its own (a synthetic strip,
        # not a real corridor) -- CONSTANT core intensity (spec's own
        # 0.85) and the ONE shared bench wind, per the plan's own "applied
        # at core intensity ... same wind_at bearing" instruction.
        def _intensity_fn(_x, _y, _i=intensity):
            return _i

        def _wind_at_fn(_x, _y, _w=WIND):
            return _w

        # Round 4 (stream T): the SECOND review floor -- `min_moved` (3 on
        # this bench: side+thrown, roof, shoved-askew; 0 pristine) on top of
        # `min_tipped` (2). A `min_moved` promotion is never a tip and is
        # stamped forced=True, floor="moved", so the Paulikas shares hold.
        _min_moved = int(_env("UT_MIN_MOVED", "3") or "3")
        actions = tornado_street.plan_street(
            placements, _intensity_fn, _wind_at_fn, rng, min_tipped=2,
            min_moved=_min_moved)
        street_counts = tornado_street.apply_street(
            stage, actions, min_tipped=2, min_moved=_min_moved,
            verbose=False)

        # DEDUPE (coordinator): `plan_street` reads the SAME `category:
        # "car"` placements this cell already authors, so it covers both
        # (now three) cars -- the ad-hoc `tornado.car_pose`/`toss_prim`
        # block this file's first cut used is dropped entirely rather than
        # double-tossing the same cars.
        car_actions = {a.get("path"): a for a in actions
                      if a.get("kind") == "car"}
        best = None
        for p in car_placements:
            a = car_actions.get(p.get("prim_path"))
            drama = 0 if a is None else (
                2 if a.get("thrown") else 1 if a.get("toppled") else 0)
            if best is None or drama < best[0]:
                dx = float(a["dx"]) if a else 0.0
                dy = float(a["dy"]) if a else 0.0
                yd = float(a.get("yaw_delta_deg", 0.0)) if a else 0.0
                best = (drama, p, dx, dy, yd)
        if best is not None:
            _drama, p, dx, dy, yd = best
            car_recs["struck"] = {
                "prim_path": p["prim_path"], "x": p["x_m"] + dx,
                "y": p["y_m"] + dy, "yaw_deg": p["yaw_deg"] + yd,
                "drama": _drama}
        print("[bench] {0}: disaster.tornado_street.plan_street+"
              "apply_street -- {1} action(s), {2} applied, {3} failed, "
              "occupant car drama={4}".format(
                  cell_id, street_counts.get("n_actions", 0),
                  street_counts.get("n_applied", 0),
                  street_counts.get("n_failed", 0),
                  car_recs.get("struck", {}).get("drama", "n/a")))
    else:
        print("[bench] SKIP {0}: disaster.tornado_street {1} -- felled "
              "runs/carried furniture/snapped trees not authored; the "
              "forced car flip/throw falls back to `tornado.car_pose`/"
              "`toss_prim` directly so a struck car is ALWAYS present"
              .format(cell_id, STREET_STATUS))
        if car_placements:
            pa = car_placements[0]
            pose_a = tn.car_pose(
                intensity, rng, WIND["bearing_deg"], 20.0, x=pa["x_m"],
                y=pa["y_m"],
                long_axis_deg=pa["yaw_deg"] - pools.yaw_of(pa["usd"]),
                force="tip")
            tn.toss_prim(stage, pa["prim_path"], pose_a["dx"], pose_a["dy"],
                         pose_a["roll_deg"], pose_a["yaw_delta_deg"],
                         pitch_deg=pose_a["pitch_deg"], seat=True)
            car_recs["flip"] = {
                "prim_path": pa["prim_path"], "x": pa["x_m"] + pose_a["dx"],
                "y": pa["y_m"] + pose_a["dy"],
                "yaw_deg": pa["yaw_deg"] + pose_a["yaw_delta_deg"]}
            if len(car_placements) > 1:
                pb = car_placements[1]
                car_recs["struck"] = {
                    "prim_path": pb["prim_path"], "x": pb["x_m"],
                    "y": pb["y_m"], "yaw_deg": pb["yaw_deg"], "drama": 0}

    return {"holder": holder, "cell": cell, "n_pieces": len(placements),
            "ctx": {}, "counts": street_counts, "cars": car_recs,
            "summary": "{0} prop(s) incl. {1} car(s), street={2} "
                       "applied/{3} failed".format(
                           len(placements), len(car_placements),
                           street_counts.get("n_applied", "n/a"),
                           street_counts.get("n_failed", "n/a"))}


# ---------------------------------------------------------------------------
# C2: ground-evidence swatch. `disaster.tornado_urban_ground` directly, over
# a small synthetic left-to-right intensity gradient (no real corridor here
# -- the bench IS the corridor stand-in).
# ---------------------------------------------------------------------------

def build_c2_cell(stage, cell_id, spec, ssf):
    holder, cell = place_holder(stage, cell_id, spec["x"], spec["y"], 0.0,
                               0.0, ssf)
    span = float(spec.get("span_m", 50.0))
    # Round 4 -- the transform trap, C2 edition: this region/cfg used WORLD
    # coordinates while everything is authored under the (x, y) holder, so
    # the whole swatch (debris + stain) landed doubled -- 105 m SE of its
    # cell, off the plate, which is why C2 renders as an empty frame.
    # LOCAL frame throughout: region centred on the cell origin, corridor
    # origin at the cell origin; the holder transform carries it to place.
    region = (-span / 2.0, -span / 2.0, span / 2.0, span / 2.0)

    def intensity(x, y):
        t = (x - region[0]) / max(1e-6, (region[2] - region[0]))
        return max(0.0, min(1.0, 0.20 + 0.70 * t))

    rng = random.Random(_seed_for(cell_id))
    # Round 4 (stream T): centre the corridor cfg ON the swatch -- C2 sits
    # 180 m off the shared track's centreline, so the corridor envelope
    # capped its stain at 0.375 opacity (and the debris bearing pointed at
    # a corridor the cell is not in). A local origin makes both meaningful.
    _c2cfg = dict(WIND_CFG)
    _c2cfg["origin_m"] = [0.0, 0.0]
    frags = tug.scatter_corridor(region, intensity, _c2cfg, rng,
                                 placements=(), per_100m2=1.4)
    ground_ctx = {"parent": PARENT, "mats": {}, "verbose": False}
    meshes = tug.build(stage, cell, frags, ground_ctx, ground_z=0.0)
    # STAIN: world frame, deliberately -- `ground.build_overlay` authors its
    # band meshes under the GLOBAL /World/tornadoGroundStain scope (measured
    # on the offline stage), so they never inherit this holder's transform.
    # The scatter above is cell-local (authored under the cell); the stain
    # must be given world coordinates or it lands at the plate origin.
    region_w = (spec["x"] + region[0], spec["y"] + region[1],
                spec["x"] + region[2], spec["y"] + region[3])
    _c2cfg_w = dict(WIND_CFG)
    _c2cfg_w["origin_m"] = [float(spec["x"]), float(spec["y"])]

    def intensity_w(x, y):
        return intensity(x - spec["x"], y - spec["y"])

    stain_paths = tug.stain_overlay(
        stage, cell, region_w, _c2cfg_w, np.random.default_rng(
            _seed_for(cell_id) + 1), intensity_w, ssf=ssf, verbose=False)
    return {"holder": holder, "cell": cell, "n_pieces": len(frags),
            "ctx": {}, "counts": {},
            "summary": "{0} fragment(s) -> {1} mesh(es), stain {2} band(s)"
                       .format(len(frags), len(meshes), len(stain_paths))}


# ---------------------------------------------------------------------------
# D-row: people. `disaster.urban_tornado_people`, rule 1's T3/T4 gate.
# ---------------------------------------------------------------------------

def build_people(stage, ssf, results, resolver, pools, config):
    """Author every D-row figure whose anchor cell actually built. Returns
    `(specs, table_rows)` -- `specs` is what got authored (or would be,
    under PLAN_ONLY where nothing is authored), `table_rows` the printable
    anchor table."""
    raw_h = _raw_pool(config, "humans")
    rigged = pools.load_tagged(raw_h, "rigged")
    if not rigged:
        posed = pools.load_tagged(raw_h, "posed_standing")
        rigged = [u for u in pools.load(raw_h) if u not in posed]
    ppl_ctx = {"asset_pools": pools, "resolver": resolver}

    specs = []
    rows = []

    def _anchor_of(cell_id, offset_out_m=0.0):
        r = results.get(cell_id)
        if not r or r.get("status") != "BUILT":
            return None
        return windward_wall_anchor(stage, r["cell"], ssf,
                                    offset_out_m=offset_out_m)

    # -- B1: berm digger pair + trapped + (gated) crush victim -------------
    b1 = results.get("B1")
    berm = _anchor_of("B1")
    if b1 and b1.get("status") == "BUILT" and berm is not None and \
            utp.casualty_gate(CELLS["B1"]["level"]):
        rng = random.Random(_seed_for("B1") + 501)
        debris = ((b1["ctx"].get("plan") or {}).get("debris")) or None
        specs += [(s, "B1") for s in utp.digger_pair(berm, rng)]
        specs.append((utp.trapped_in_berm(berm, rng, debris=debris), "B1"))
        macro = bool((b1["ctx"].get("plan") or {}).get("macroblocks"))
        cv = utp.crush_victim(berm, rng, macroblock_present=macro)
        if cv is not None:
            specs.append((cv, "B1"))
        rows.append(("B1", "digger_pair(2) + trapped_in_berm" +
                    (" + crush_victim" if cv else ""),
                    "macroblocks={0}".format(macro)))
    elif b1 is not None:
        rows.append(("B1", "(no casualties)",
                    "gate closed or B1 not built"))

    # -- B5: pile-edge diggers + covered casualty + refuge doorway ---------
    # `footprint_wall_anchor`, NOT `_anchor_of`/`windward_wall_anchor` --
    # B5's `cell` bbox after `tornado_collapse.apply_industrial` includes
    # the fallen debris field (panels lie flat outward from the walls),
    # so a live stage-bbox read would place the anchor at the debris
    # field's own outer edge rather than the shed's original wall line.
    # `build_industrial_cell` measures W/D from the INTACT shed and
    # returns them on its own result dict for exactly this reason.
    b5 = results.get("B5")
    pile = None
    if b5 and b5.get("status") == "BUILT" and "W" in b5:
        pile = footprint_wall_anchor(CELLS["B5"]["x"], CELLS["B5"]["y"],
                                     b5["W"], b5["D"])
    if b5 and b5.get("status") == "BUILT" and pile is not None and \
            utp.casualty_gate("T4"):
        rng = random.Random(_seed_for("B5") + 502)
        specs += [(s, "B5") for s in utp.pile_edge_digger(pile, rng)]
        specs.append((utp.covered_casualty(pile, rng), "B5"))
        door = utp.anchor(*utp._along_wall(pile, 0.5, 0.5),
                          bearing_deg=pile["bearing_deg"])
        specs += [(s, "B5") for s in utp.doorway_threshold(door, rng)]
        rows.append(("B5", "pile_edge_digger(2) + covered_casualty + "
                          "doorway_threshold", "refuge interior kept clear"))
    elif b5 is not None:
        rows.append(("B5", "(no casualties)", "B5 not built"))

    # -- C1: the struck-car occupant ----------------------------------------
    c1 = results.get("C1")
    if c1 and c1.get("status") == "BUILT" and c1.get("cars", {}).get("struck"):
        car = c1["cars"]["struck"]
        rng = random.Random(_seed_for("C1") + 503)
        car_anchor = utp.anchor(car["x"], car["y"], car["yaw_deg"])
        specs.append((utp.car_occupant(car_anchor, rng), "C1"))
        rows.append(("C1", "car_occupant (struck car)", "rule 3, ~20% share"))
    elif c1 is not None:
        rows.append(("C1", "(no occupant)", "C1 not built / no struck car"))

    # -- A2: evacuee pair at a standing entry --------------------------------
    a2 = results.get("A2")
    entry = _anchor_of("A2", offset_out_m=2.5)
    if a2 and a2.get("status") == "BUILT" and entry is not None:
        rng = random.Random(_seed_for("A2") + 504)
        specs += [(s, "A2") for s in utp.evacuee_pair(entry, rng)]
        rows.append(("A2", "evacuee_pair", "not gated (not a casualty)"))
    elif a2 is not None:
        rows.append(("A2", "(no evacuees)", "A2 not built"))

    # -- author --------------------------------------------------------------
    authored = 0
    if not PLAN_ONLY and rigged:
        placements = []
        for i, (spec, _cell) in enumerate(specs):
            usd = rigged[i % len(rigged)]
            z_ground = 0.0
            placements.append(utp.to_placement(ppl_ctx, spec, usd, z_ground))
        if placements:
            sg.apply_placements(stage, placements, PARENT + "/people", ssf,
                                resolver=resolver, instance_categories=set())
            authored = len(placements)
    elif not rigged:
        print("[bench] !! NO RIGGED HUMANS in the pool -- {0} D-row spec(s) "
              "planned but nothing authored".format(len(specs)))

    print("[bench] D-row people: {0} spec(s), {1} authored ({2})".format(
        len(specs), authored, "PLAN_ONLY -- authoring skipped" if PLAN_ONLY
        else "live"))
    return specs, rows


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------

def _dispatch(stage, cell_id, ssf, config, resolver, pools):
    spec = CELLS[cell_id]
    kind = spec["kind"]
    # Round 4 (stream D): arm the debris landing clamp for THIS cell, in
    # the CELL-LOCAL frame (plans are authored at x=y=yaw=0 and the holder
    # is seated AFTER the wreck). tornado_urban re-reads this env per
    # plan_damage call, but its import-time module value would WIN over the
    # per-cell update -- so the relaunch line must NOT set TU_PLATE_REGION
    # globally any more. The plate must be transformed INTO the local frame
    # with the SEAT yaw accounted for: the A row is seated at 180 deg, so
    # its local region is the plate negated about the cell origin (the
    # first offline audit of this fix measured A-row debris at y=136 on a
    # 120 plate with the yaw-blind version).
    yaw_seat = float(spec.get("seat", 0.0))
    angle = math.radians(-yaw_seat)
    ca, sa = math.cos(angle), math.sin(angle)
    corners = []
    for wx in (PLATE[0], PLATE[2]):
        for wy in (PLATE[1], PLATE[3]):
            dx, dy = wx - spec["x"], wy - spec["y"]
            corners.append((ca * dx - sa * dy, sa * dx + ca * dy))
    region = (min(p[0] for p in corners), min(p[1] for p in corners),
              max(p[0] for p in corners), max(p[1] for p in corners))
    os.environ["TU_PLATE_REGION"] = "{0},{1},{2},{3}".format(*region)
    if kind == "gac":
        return build_a_cell(stage, cell_id, spec, ssf)
    if kind == "kit":
        return build_b_kit_cell(stage, cell_id, spec, ssf)
    if kind == "industrial":
        return build_industrial_cell(stage, cell_id, spec, ssf, config,
                                     resolver, pools)
    if kind == "street":
        return build_c1_cell(stage, cell_id, spec, ssf, config, resolver,
                             pools)
    if kind == "ground":
        return build_c2_cell(stage, cell_id, spec, ssf)
    raise ValueError("unknown cell kind {0!r}".format(kind))


def run(stage, ssf):
    t0 = time.time()
    print("[bench] compiling {0} for its asset_root/material/street/human "
          "pools ONLY -- no layout is built from it".format(ASSET_CONFIG))
    config = load_scene_config(ASSET_CONFIG)
    resolver = sg._make_resolver(config)
    pools = AssetPools(config)

    build_ground(stage, ssf, config)
    fracture.ensure_vtk(verbose=False)
    if FAST_SLICE:
        from detail import gac_storey_slice_fast
        gac_storey_slice_fast.install()
        print("[bench] fast complementary-sweep slicer enabled")

    results = {}
    for cell_id in CELL_ORDER:
        t_c0 = time.time()
        try:
            r = _dispatch(stage, cell_id, ssf, config, resolver, pools)
            r["status"] = "BUILT"
            r["time_s"] = round(time.time() - t_c0, 2)
            results[cell_id] = r
            print("[bench] BUILT {0:<3} {1:6.2f}s  {2}".format(
                cell_id, r["time_s"], r.get("summary", "")))
        except Exception as exc:                                  # noqa: BLE001
            import traceback
            traceback.print_exc()
            results[cell_id] = {"status": "FAILED", "error": str(exc),
                                "time_s": round(time.time() - t_c0, 2)}
            print("[bench] FAILED {0:<3} {1:6.2f}s  {2}".format(
                cell_id, results[cell_id]["time_s"], exc))

    people_specs, people_rows = build_people(stage, ssf, results, resolver,
                                             pools, config)

    t_total = time.time() - t0
    return {"config": config, "resolver": resolver, "pools": pools,
            "results": results, "people_specs": people_specs,
            "people_rows": people_rows, "t_total": t_total, "ssf": ssf}


def report(state):
    lines = []
    lines.append("=" * 78)
    lines.append("URBAN TORNADO BENCH -- cell manifest (seed {0})".format(
        UTB_SEED))
    lines.append("=" * 78)
    lines.append("{0:<4} {1:<8} {2:<6} {3:<45}".format(
        "cell", "status", "time", "contents"))
    for cell_id in CELL_ORDER:
        r = state["results"].get(cell_id, {"status": "SKIPPED",
                                           "time_s": 0.0})
        status = r.get("status", "SKIPPED")
        detail = (r.get("summary") if status == "BUILT"
                  else r.get("error", "not requested (UTB_CELLS)"))
        lines.append("{0:<4} {1:<8} {2:5.2f}s {3:<45}".format(
            cell_id, status, r.get("time_s", 0.0), str(detail)[:70]))
    lines.append("-" * 78)
    lines.append("D-ROW PEOPLE ANCHORS")
    for cell_id, contents, note in state["people_rows"]:
        lines.append("  {0:<4} {1:<48} {2}".format(cell_id, contents, note))
    n_casualty = sum(1 for s, _c in state["people_specs"]
                     if s["class"] in ("trapped", "crush_victim",
                                       "covered_casualty", "car_occupant"))
    n_upright = len(state["people_specs"]) - n_casualty
    lines.append("  totals: {0} figure(s) planned -- {1} casualty, {2} "
                "upright (digger/doorway/evacuee)".format(
                    len(state["people_specs"]), n_casualty, n_upright))
    lines.append("-" * 78)
    for name, status in MODULE_STATUS:
        lines.append("  module {0}: {1}".format(name, status))
    lines.append("=" * 78)
    text = "\n".join(lines)
    print(text)
    out_path = os.path.join(_SCENE_GEN_DIR, "_plans",
                            "urban_tornado_bench_{0}_report.md".format(
                                UTB_SEED))
    try:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w") as fh:
            fh.write(text + "\n")
        print("[bench] report -> {0}".format(out_path))
    except OSError as exc:
        print("[bench] could not write report file: {0}".format(exc))
    return text


def banner(state):
    failed = [c for c, r in state["results"].items()
             if r.get("status") == "FAILED"]
    built = [c for c, r in state["results"].items()
            if r.get("status") == "BUILT"]
    ok = not failed
    print("\n" + "=" * 78)
    print("URBAN TORNADO BENCH   {0}/{1} cell(s) built, {2} failed, {3} "
          "people spec(s)   {4:.0f}s total".format(
              len(built), len(CELL_ORDER), len(failed),
              len(state["people_specs"]), state["t_total"]))
    print("=" * 78 + "\n")
    print("URBAN TORNADO BENCH " + ("DONE" if ok else "FAILED"))
    return ok


# ---------------------------------------------------------------------------
# Snapshots (Kit mode only)
# ---------------------------------------------------------------------------

def capture(stage, ssf, state):
    if PLAN_ONLY or not SNAP_DIR:
        return
    try:
        import importlib.util as _ilu
        sp = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
        spec = _ilu.spec_from_file_location("snapshots", sp)
        snaps = _ilu.module_from_spec(spec)
        spec.loader.exec_module(snaps)
        os.makedirs(SNAP_DIR, exist_ok=True)
        app = omni.kit.app.get_app()
        for _ in range(40):
            app.update()

        points = {c: (CELLS[c]["x"], CELLS[c]["y"]) for c in CELL_ORDER
                 if state["results"].get(c, {}).get("status") == "BUILT"}
        # SIZE-AWARE rig (round 4). The fixed 45/30/aim-6 close oblique
        # framed the A row into the facade BASE (A4: inside the wall of the
        # taller SM_Building_24) while the damage the bench exists to review
        # sits in the upper storeys. Measure each built cell's world bbox
        # and scale the camera to it; small cells land near the old numbers.
        bcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                   ["default", "render"])
        first = True
        review_pose = None
        for c in CELL_ORDER:
            if c not in points:
                continue
            rig = dict(top_h=55.0, obl_dist=45.0, obl_h=30.0, aim_h=6.0)
            cpath = state["results"].get(c, {}).get("cell")
            prim = stage.GetPrimAtPath(cpath) if cpath else None
            if prim and prim.IsValid():
                box = bcache.ComputeWorldBound(prim).ComputeAlignedRange()
                if not box.IsEmpty():
                    mn, mx = box.GetMin(), box.GetMax()
                    w = (mx[0] - mn[0]) / ssf
                    d = (mx[1] - mn[1]) / ssf
                    h = max(0.0, (mx[2] - mn[2]) / ssf)
                    r = max(w, d) / 2.0
                    rig = dict(
                        top_h=max(55.0, 1.25 * h + 1.6 * r + 15.0),
                        obl_dist=max(40.0, 1.5 * r + 1.1 * h + 12.0),
                        obl_h=max(24.0, 0.75 * h + 10.0),
                        aim_h=max(4.0, 0.45 * h))
            # 60 settle frames on the FIRST capture only -- the round-2
            # stale-frame lesson (the first shot after a big camera jump
            # rendered the previous frame).
            snaps.views_around(stage, {c: points[c]}, SNAP_DIR, ssf,
                               frames=60 if first else 40, **rig)
            # Damage-review contact sheet: the whole-building oblique is an
            # orientation shot, not a geometry/material gate. Frame the
            # authored break groups themselves and inspect all four sides.
            damage_ranges = []
            if prim and prim.IsValid():
                for q in Usd.PrimRange(prim):
                    if q.IsActive() and q.GetName().startswith("brk_"):
                        qr = bcache.ComputeWorldBound(q).ComputeAlignedRange()
                        if not qr.IsEmpty():
                            damage_ranges.append(qr)
            if damage_ranges:
                lo = [min(float(r.GetMin()[i]) for r in damage_ranges)
                      for i in range(3)]
                hi = [max(float(r.GetMax()[i]) for r in damage_ranges)
                      for i in range(3)]
                aim = tuple((lo[i] + hi[i]) * 0.5 for i in range(3))
                span = max(4.0 * ssf, hi[0] - lo[0], hi[1] - lo[1],
                           hi[2] - lo[2])
                dist = 1.15 * span
                for label, dx, dy in (("E", 1, 0), ("N", 0, 1),
                                      ("W", -1, 0), ("S", 0, -1)):
                    eye = (aim[0] + dx * dist, aim[1] + dy * dist,
                           aim[2] + 0.12 * span)
                    snaps.place_camera(stage, eye, aim, focal_mm=28.0)
                    snaps.snapshot(os.path.join(
                        SNAP_DIR, "{0}_damage_{1}.png".format(c, label)),
                        30)
                # Leave the interactive viewport on the outward damage face,
                # not on the overview shot authored below. This is the view
                # the reviewer sees when KEEP_OPEN enters Play mode.
                vx, vy = aim[0] - points[c][0] * ssf, aim[1] - points[c][1] * ssf
                vl = max(1e-6, (vx * vx + vy * vy) ** 0.5)
                vx, vy = vx / vl, vy / vl
                review_pose = ((aim[0] + vx * dist, aim[1] + vy * dist,
                                aim[2] + 0.12 * span), aim)
            first = False
        x0, y0, x1, y1 = PLATE
        span = max(x1 - x0, y1 - y0)
        snaps.overview(stage, (0.0, 0.0), span * 1.05,
                       os.path.join(SNAP_DIR, "bench_overview.png"), ssf)
        if review_pose is not None:
            snaps.place_camera(stage, review_pose[0], review_pose[1],
                               focal_mm=28.0)
            for _ in range(20):
                app.update()
        print("[bench] snapshots ({0} cell(s) + 1 overview) -> {1}".format(
            len(points), SNAP_DIR))
    except Exception as exc:                                       # noqa: BLE001
        import traceback
        traceback.print_exc()
        print("[bench] snapshots FAILED: {0}".format(exc))


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def main_plan_only():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))
    ssf = 1.0
    state = run(stage, ssf)
    report(state)
    ok = banner(state)
    return 0 if ok else 1


def main_kit():
    t0 = time.time()
    pg = PegasusInterface()
    pg._world = World(**pg._world_settings)
    pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("Stage failed to load")
    for _ in range(30):
        omni.kit.app.get_app().update()
    from scene_prep import get_stage_meters_per_unit
    _, ssf = get_stage_meters_per_unit(stage)
    # STRIP THE DEFAULT ENVIRONMENT'S OWN FURNITURE (lead fix, second GUI
    # run): Pegasus's "Default Environment" ships a ~250 m checkered
    # GroundPlane and an Environment scope that sat mid-plate through the
    # first lit bench and read as a giant missing-texture square. The city
    # launcher strips exactly these two roots (`_remove_env_clutter`,
    # `_ENV_CLUTTER = {"GroundPlane", "Environment"}`) — a launch script
    # cannot be imported for its helpers, so the loop is copied with
    # attribution.
    _n_clutter = 0
    for _root_path in ("/", "/World", "/World/stage"):
        _root = (stage.GetPseudoRoot() if _root_path == "/"
                 else stage.GetPrimAtPath(_root_path))
        if not _root or not _root.IsValid():
            continue
        for _child in _root.GetChildren():
            if _child.GetName() not in ("GroundPlane", "Environment") \
                    or not _child.IsActive():
                continue
            if _child.SetActive(False):
                _n_clutter += 1
            else:
                UsdGeom.Imageable(_child).MakeInvisible()
    print("[bench] env clutter: {0} prim(s) deactivated".format(_n_clutter))
    # LIGHT THE BENCH (lead fix, first GUI run): this launcher authored NO
    # light at all — Pegasus's "Default Environment" carries nothing the
    # snapshot renderer can see, and every capture came back BLACK (the
    # same "cells ship unlit" defect the frozen-dataset runs once hit).
    # `sky_presets.apply_sky_preset` is the city launcher's own named-preset
    # path: one dome + one sun, "mid_day" default, and an unrecognised
    # `UTB_SKY` value falls back to mid_day rather than to darkness.
    from sky_presets import apply_sky_preset
    _sky = apply_sky_preset(stage, _env("UTB_SKY", "mid_day"), prefix="utb")
    print("[bench] sky: UTB_SKY={0} -> preset '{1}'".format(
        _env("UTB_SKY", "mid_day"), _sky))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT))

    state = run(stage, ssf)
    _stage_out = _env("UTB_STAGE_OUT", "").strip()
    if _stage_out:
        stage.GetRootLayer().Export(_stage_out)
        print("[bench] diagnostic stage -> {0}".format(_stage_out))
    capture(stage, ssf, state)
    report(state)
    ok = banner(state)
    print("[bench] built in {0:.0f}s (target < 300s)".format(
        time.time() - t0))
    return 0 if ok else 1


if __name__ == "__main__":
    if PLAN_ONLY:
        sys.exit(main_plan_only())
    try:
        rc = main_kit()
    except Exception as _exc:                                      # noqa: BLE001
        import traceback
        traceback.print_exc()
        print("URBAN TORNADO BENCH FAILED: {0}".format(_exc))
        rc = 1
    if KEEP_OPEN or not _HEADLESS:
        _app = omni.kit.app.get_app()
        omni.timeline.get_timeline_interface().play()
        while simulation_app.is_running():
            _app.update()
        omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
    sys.exit(rc)
