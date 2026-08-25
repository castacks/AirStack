#!/usr/bin/env python
"""
Catalogue of the ModernCityEnvironment01 kit: ONE instance of every asset in
`/Projects/SEI-COA/ModernCityEnvironment01/Meshes/`, laid out in rows on an
empty stage so the whole pack can be read off in one view.

    ISAAC_SIM_SCRIPT_NAME=modern_city_catalogue_launch_script.py airstack up isaac-sim

The folder is listed at RUNTIME (`omni.client.list`), every `SM_X.usd` that is
not a `_payload` is measured (`UsdGeom.BBoxCache`) and placed so its bbox
corner sits at its cell origin and its base on the ground — pivots in this kit
are all over the place (centred roofs, left-end façades, box corners), so
laying out by pivot would overlap. Rows are packed by measured width and wrap
at ROW_W metres; names sort alphabetically, which clusters the families
(SM_MBuilding01_*, SM_MBuilding02_*, ... then the street props).

Each prim is named after its asset (`/World/stage/generated/SM_X_<n>_<i>`),
so selecting a thing in the viewport tells you what it is. The same table is
printed to the pane and written to CATALOGUE_CSV (host: ~/docker/isaac-sim/logs/).

Environment:
    CAT_ROW_W      row width in metres before wrapping (default 160)
    CAT_MARGIN     gap between assets in metres (default 4)
    CAT_FILTER     substring; only assets whose name contains it (default all)
    CAT_SNAP_DIR   write an overview + one capture per row to this directory
"""

import csv
import os
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.client
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

from omni.isaac.core.world import World
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

from scene_prep import scale_stage_prim, add_sky, get_stage_meters_per_unit
import scene_generator as sg

# ----- CONFIGURATION -----
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
KIT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443"
       "/Projects/SEI-COA/ModernCityEnvironment01/Meshes/")
ROW_W = float(os.environ.get("CAT_ROW_W", "160"))
MARGIN = float(os.environ.get("CAT_MARGIN", "4"))
FILTER = os.environ.get("CAT_FILTER", "").strip()
SNAP_DIR = os.environ.get("CAT_SNAP_DIR", "").strip()
CATALOGUE_CSV = "/isaac-sim/.nvidia-omniverse/logs/mce_catalogue.csv"
PARENT = "/World/stage/generated"
CAM = "/World/catCam"
# -------------------------


def list_assets():
    res, entries = omni.client.list(KIT)
    if res != omni.client.Result.OK:
        raise RuntimeError(f"omni.client.list({KIT}) -> {res}")
    names = sorted(e.relative_path for e in entries
                   if e.relative_path.endswith(".usd")
                   and not e.relative_path.endswith("_payload.usd"))
    if FILTER:
        names = [n for n in names if FILTER.lower() in n.lower()]
    return names


def measure(url):
    """(min, max) in metres of the asset's default prim, or None."""
    stage = Usd.Stage.Open(url)
    if not stage:
        return None
    dp = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bc.ComputeWorldBound(dp).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    mn, mx = r.GetMin(), r.GetMax()
    return ([mn[i] * mpu for i in range(3)], [mx[i] * mpu for i in range(3)])


def layout(names):
    """Row-packed placements; returns (placements, rows, table, missing).

    Anything wider than half a row (SM_GroundModule04 is a 500 x 500 m ground
    slab) goes LAST, on its own, so it does not sit in the middle of the grid.
    """
    placements, table, missing, rows = [], [], [], []
    measured, giants = [], []
    for k, n in enumerate(names):
        bb = measure(KIT + n)
        if bb is None:
            missing.append(n)
            continue
        mn, mx = bb
        (giants if mx[0] - mn[0] > ROW_W / 2 or mx[1] - mn[1] > ROW_W / 2
         else measured).append((k, n, mn, mx))
    x = y = 0.0
    row_d = 0.0
    row_i = 0
    row_x0 = 0.0
    for k, n, mn, mx in measured + giants:
        url = KIT + n
        sx, sy, sz = (mx[i] - mn[i] for i in range(3))
        if x > 0.0 and (x + sx > ROW_W or sx > ROW_W / 2):
            rows.append((row_i, row_x0, x, y, row_d))
            row_i += 1
            y += row_d + MARGIN * 2
            x, row_d = 0.0, 0.0
        placements.append({
            "usd": url, "x_m": x - mn[0], "y_m": y - mn[1], "z_m": -mn[2],
            "yaw_deg": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "scale": 1.0, "axis_up": "Z", "raw_pivot": True,
            "category": n[:-4]})
        table.append((k, n[:-4], row_i, x + sx / 2, y + sy / 2, sx, sy, sz))
        x += sx + MARGIN
        row_d = max(row_d, sy)
    rows.append((row_i, row_x0, x, y, row_d))
    return placements, rows, table, missing


def wait_for_stage(stage, timeout_s=10.0):
    for _ in range(int(timeout_s / 0.1)):
        omni.kit.app.get_app().update()
        world_prim = stage.GetPrimAtPath("/World")
        if world_prim.IsValid():
            if [c for c in world_prim.GetChildren() if c.GetName() != "PhysicsScene"]:
                return True
        time.sleep(0.1)
    return False


def _look_at(eye, target):
    import math
    dx, dy, dz = (target[i] - eye[i] for i in range(3))
    yaw = math.degrees(math.atan2(dy, dx)) - 90.0
    pitch = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
    return Gf.Vec3f(90.0 + pitch, 0.0, yaw)


def aim_camera(stage, eye, target):
    cam = UsdGeom.Camera.Get(stage, Sdf.Path(CAM))
    if not cam:
        cam = UsdGeom.Camera.Define(stage, Sdf.Path(CAM))
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.5, 10000.0))
        cam.GetFocalLengthAttr().Set(18.0)          # ~60 deg, not Kit's 50 mm
        cam.GetHorizontalApertureAttr().Set(20.955)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*eye))
    xf.AddRotateXYZOp().Set(_look_at(eye, target))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = CAM
    except Exception as exc:
        carb.log_warn(f"could not retarget the viewport: {exc}")


def snapshot(path, frames=30):
    import omni.kit.viewport.utility as vp
    app = omni.kit.app.get_app()
    for _ in range(frames):
        app.update()
    vp.capture_viewport_to_file(vp.get_active_viewport(), path)
    for _ in range(frames):
        app.update()
    print(f"[catalogue] snapshot -> {path}")


class CatalogueApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline.stop()

        pg = PegasusInterface()
        pg._world = World(**pg._world_settings)
        pg.load_environment(ENV_URL)

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage failed to load")
        if not wait_for_stage(stage):
            carb.log_warn("Stage load timed out — continuing anyway.")
        self.stage = stage

        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            for _ in range(10):
                omni.kit.app.get_app().update()

        names = list_assets()
        print(f"[catalogue] {len(names)} assets in {KIT}")
        t0 = time.time()
        placements, rows, table, missing = layout(names)
        print(f"[catalogue] measured in {time.time() - t0:.0f} s; "
              f"{len(placements)} placed, {len(missing)} with no geometry")
        self.rows, self.table = rows, table

        _, ssf = get_stage_meters_per_unit(stage)
        sg.apply_placements(stage, placements, PARENT, ssf)
        for _ in range(10):
            omni.kit.app.get_app().update()
        add_sky(stage, "")

        with open(CATALOGUE_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["index", "asset", "row", "x_m", "y_m", "size_x", "size_y", "size_z"])
            for rec in table:
                w.writerow([rec[0], rec[1], rec[2]] + [f"{v:.2f}" for v in rec[3:]])
        print(f"[catalogue] table -> {CATALOGUE_CSV}")

        print("\n" + "=" * 78)
        print("MODERN CITY ENVIRONMENT 01 — CATALOGUE READY")
        print(f"  {len(placements)} assets in {len(rows)} rows, row width {ROW_W:.0f} m")
        print(f"  {'#':>4} {'asset':<44} {'row':>3} {'x':>7} {'y':>7}  size (m)")
        for k, n, r, cx, cy, sx, sy, sz in table:
            print(f"  {k:4d} {n:<44} {r:3d} {cx:7.1f} {cy:7.1f}  "
                  f"{sx:5.1f} x {sy:5.1f} x {sz:5.1f}")
        if missing:
            print(f"  no geometry ({len(missing)}): " + ", ".join(missing))
        print("=" * 78 + "\n")

        # Frame the rows of ordinary assets; the giants' row is behind them.
        ordinary = [r for r in rows if r[4] <= ROW_W / 2] or rows
        total_d = ordinary[-1][3] + ordinary[-1][4]
        self.poses = [("overview", (ROW_W / 2, -0.9 * max(ROW_W, total_d),
                                    0.7 * max(ROW_W, total_d)),
                       (ROW_W / 2, total_d / 2, 0.0))]
        for r, x0, x1, y, d in rows:
            self.poses.append((f"row{r:02d}", (ROW_W / 2, y - 0.55 * ROW_W, 0.25 * ROW_W),
                               (ROW_W / 2, y + d / 2, 3.0)))
        aim_camera(stage, *self.poses[0][1:])
        self.timeline.play()

    def run(self):
        app = omni.kit.app.get_app()
        if SNAP_DIR:
            os.makedirs(SNAP_DIR, exist_ok=True)
            for _ in range(60):
                app.update()
            for name, eye, target in self.poses:
                aim_camera(self.stage, eye, target)
                snapshot(os.path.join(SNAP_DIR, f"{name}.png"))
            aim_camera(self.stage, *self.poses[0][1:])
            print(f"[catalogue] {len(self.poses)} snapshots in {SNAP_DIR}")
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    CatalogueApp().run()


if __name__ == "__main__":
    main()
