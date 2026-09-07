#!/usr/bin/env python
"""One asset, every earthquake rung, photographed in Isaac.

    SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/rungs \\
    PREVIEW_ASSET="DownTown/Assets/BG_Building_E.usd" \\
    ISAAC_SIM_SCRIPT_NAME=rung_preview_launch_script.py airstack up isaac-sim

WHY IN ISAAC AND NOT IN BLENDER
-------------------------------
`tools/render_archetypes.py` renders the baked library through Cycles, which is
fast and runs on the host — but Blender cannot evaluate MDL, so anything bound
only through MDL comes out a black silhouette (the AEC vegetation does exactly
that). Kit evaluates the materials the assets actually ship, so this is the
view that says whether a rung LOOKS right, as opposed to whether its geometry
is the right shape.

IT BUILDS THROUGH STAGE A, NOT BESIDE IT
----------------------------------------
Every cell here is built by `archetypes.bake.Baker._build_one` and settled by
`Baker._settle_batch` — the same calls a real bake makes, at the same grid
spacing, off the same compiled config. So a preview that looks wrong is
evidence about the bake rather than about this script. The one thing it does
NOT do is export: the archetypes for one tower run to several GB, and looking
at something should not cost a bake's worth of disk.
"""
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

_HERE = os.path.dirname(os.path.abspath(__file__))          # .../launch_scripts
_ISAAC_SIM_DIR = os.path.dirname(_HERE)                     # .../isaac-sim
_AIRSTACK = os.path.dirname(os.path.dirname(_ISAAC_SIM_DIR))  # repo root
for _p in (os.path.join(_AIRSTACK, "scene_gen"), _ISAAC_SIM_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import omni.usd                                                  # noqa: E402
from compile_disaster import load_scene_config                   # noqa: E402
from archetypes import bake as A                                 # noqa: E402
from archetypes import plan as P                                 # noqa: E402
from disaster import levels as L                                 # noqa: E402

ASSET = os.environ.get("PREVIEW_ASSET", "DownTown/Assets/BG_Building_E.usd")
CONFIG = os.environ.get("PREVIEW_CONFIG", "urban_v2")
LEVELS = [s.strip() for s in os.environ.get(
    "PREVIEW_LEVELS",
    "pristine,cracked,soft_storey,partial_collapse,pancaked").split(",")
    if s.strip()]
SNAP_DIR = os.environ.get(
    "SNAP_DIR", "/isaac-sim/.nvidia-omniverse/logs/rung_preview")
SEED = int(os.environ.get("PREVIEW_SEED", "7"))


def _snaps():
    """`utils/snapshots.py`, BY PATH — `from utils import snapshots` does not
    work here. Kit's own extensions import a package called `utils` long before
    this script runs (cv2 ships one), so the name is already in `sys.modules`
    and a plain import silently resolves to
    `omni.pip.compute/pip_prebundle/cv2/utils`. `targets_showcase` loads it the
    same way for the same reason."""
    import importlib.util as ilu

    path = os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py")
    spec = ilu.spec_from_file_location("airstack_snapshots", path)
    mod = ilu.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _obl_only(snaps):
    """`views_around` with the top-down shot dropped."""
    def only(stage, points, out_dir, ssf=1.0, top_h=60.0, obl_dist=45.0,
             obl_h=22.0, frames=40, target_z=1.0):
        import math
        written, d = [], float(obl_dist) / math.sqrt(2.0)
        for name, (x, y) in sorted(points.items()):
            x, y = float(x), float(y)
            snaps.place_camera(stage, ((x - d) * ssf, (y - d) * ssf,
                                       float(obl_h) * ssf),
                               (x * ssf, y * ssf, float(target_z) * ssf))
            written.append(snaps.snapshot(
                os.path.join(out_dir, f"{name}_obl.png"), frames))
        return [p for p in written if p]
    return only


def _item_for(config, source):
    """The plan's own `Item` for *source* — so scale and axis-up are the
    pack's, not a guess. Referencing a centimetre-authored asset at 1.0 is how
    Stage A once built archetypes eight kilometres across."""
    want = str(source).lower().strip()
    items = P.build_plan(config, "earthquake")
    # A BARE TYPE SLUG IS ENOUGH — `old_brick_shop`, not the pack path. The
    # loop is meant to be typed by hand a hundred times a day.
    for it in items:
        if it.type.lower() == want:
            return it
    for it in items:
        src = str(it.source).lower()
        if src.endswith(want) or want.endswith(src) or want in src:
            return it
    near = sorted(i.type for i in items if want[:6] in i.type.lower())[:6]
    raise SystemExit(f"[rungs] {source!r} is not in the {CONFIG} plan"
                     + (f"; did you mean {near}?" if near else ""))


def main() -> int:
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    A.prepare_stage(stage)

    config = load_scene_config(CONFIG)
    item = _item_for(config, ASSET)
    print(f"[rungs] {item.type}  scale={item.scale}  up={item.axis_up}")

    baker = A.Baker(stage, config, "earthquake", "", seed=SEED)

    # SPACING FROM THE BUILDING, NOT FROM `GRID_M`. Stage A's 40 m grid must
    # "exceed the largest asset's footprint plus its debris throw" — and
    # BG_Building_E is 60 m across, so laying its rungs out at 40 m would have
    # them settle into each other and each pick up a piece of its neighbour.
    # The envelope is only knowable after the asset is referenced, so a probe
    # is placed far out of the way, measured, and removed.
    from pxr import Sdf, UsdGeom
    probe = f"{baker.parent}/probe"
    UsdGeom.Scope.Define(stage, Sdf.Path(probe))
    baker._reference(f"{probe}/asset", item.source, 0.0, 5000.0,
                     scale=item.scale, axis_up=item.axis_up)
    (px, py), pz = baker._envelope(f"{probe}/asset")
    baker._unload(probe)
    step = max(float(A.GRID_M), max(px, py) * 2.0 + 20.0)
    print(f"[rungs] asset {px:.0f}x{py:.0f}x{pz:.0f} m -> cell spacing "
          f"{step:.0f} m (GRID_M is {A.GRID_M:.0f})", flush=True)

    pending, spots = [], {}
    for i, level in enumerate(LEVELS):
        if level not in L.bake_levels("earthquake", L.STRUCTURE):
            print(f"[rungs] skipping unknown rung {level!r}")
            continue
        x, y = float(i) * step, 0.0
        cell = f"{baker.parent}/rung_{level}"
        UsdGeom.Scope.Define(stage, Sdf.Path(cell))
        n_loose, n_static = len(baker.loose), len(baker.static)
        baker.cell_record = {}
        print(f"[rungs] building {item.type}_{level} at x={x:.0f}", flush=True)
        paths, extra = baker._build_one(item, level, x, y, cell, 1.0)
        pending.append({"item": item, "level": level, "x": x, "y": y,
                        "cell": cell, "paths": paths, "extra": extra,
                        "loose": baker.loose[n_loose:],
                        "static": baker.static[n_static:],
                        "t_cell": 0.0, "record": {}})
        spots[level] = (x, y)

    # SETTLED IN BATCHES, THE SAME WAY A BAKE WOULD. Calling `_settle_batch`
    # on the whole row bypasses `SETTLE_BATCH_BODIES`, and on a big asset that
    # is not a preview of anything: BG_Building_E's four rungs are 8,247 loose
    # fragments, against a budget of 1,200. The first attempt at this row did
    # exactly that and never came out of the solver. Chunking here keeps the
    # preview honest — what you look at is what the baker would produce.
    stats = {}
    chunk = []
    for pd in pending:
        if chunk and baker._batch_full(chunk):
            stats.update(baker._settle_batch(chunk))
            chunk = []
        chunk.append(pd)
    if chunk:
        stats.update(baker._settle_batch(chunk))
    for level, st in sorted((p["level"], stats.get(p["cell"])) for p in pending):
        if st:
            print(f"[rungs] {level:17} {st['bodies']:5d} bodies  "
                  f"{st['steps_used']:5d} steps  drop {st['drop_median_m']:+.2f} m"
                  f"  spread {st['spread_max_m']:.1f} m  "
                  f"converged={st['converged']}", flush=True)

    # A CAMERA SIZED ON THE BUILDING, not on the default 60 m. BG_Building_E is
    # most of a hundred metres tall and the stock framing would photograph its
    # ground floor.
    (sx, sy), sz = baker._envelope(f"{pending[0]['cell']}/asset") \
        if pending else ((20.0, 20.0), 20.0)
    span = max(sx, sy, sz, 10.0)
    os.makedirs(SNAP_DIR, exist_ok=True)
    print(f"[rungs] envelope {sx:.0f}x{sy:.0f}x{sz:.0f} m -> framing {span:.0f} m",
          flush=True)
    snaps = _snaps()
    if os.environ.get("PREVIEW_VIEWS", "both").lower() == "obl":
        # HALF THE CAPTURES. Each is 60 ray-traced frames; in a tight loop the
        # top-down adds little that the oblique does not already show.
        snaps.views_around = _obl_only(snaps)
    snaps.views_around(stage, spots, SNAP_DIR,
                          top_h=span * 1.9, obl_dist=span * 1.7,
                          obl_h=span * 0.85, target_z=sz * 0.45, frames=60)
    print(f"[rungs] DONE -> {SNAP_DIR}", flush=True)

    while simulation_app.is_running():
        simulation_app.update()
    return 0


if __name__ == "__main__":
    sys.exit(main())
