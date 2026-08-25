#!/usr/bin/env python
"""
Urban building showcase: the pack author's four MERGED buildings in the front
row, then every style `scene_gen/detail/urban_building.py` assembles — 30
styles across the ModernCityEnvironment01, Downtown_West and CivilianArea
kits, from an 8 m church to a 103 m tower and four full city-block massings —
plus the five city blocks harvested from Dmytro's downtown_edited_v2.usd,
shortest row in front.

    ISAAC_SIM_SCRIPT_NAME=urban_showcase_launch_script.py airstack up isaac-sim

Front row: `/Projects/SEI-COA/ModernCityEnvironment/Collected_Building0N/
SM_MERGED_BP_MBuilding0N.usd` (01, 02, 03, 05) — whole flattened buildings,
the reference for what each kit family is meant to look like. Three of them
are authored in centimetres while claiming metersPerUnit 1, so each is
measured at launch and scaled 0.01 when its extent says so.

Every prim is named after its style (`/World/stage/generated/bld_<style>_...`,
`harvested_dw_block_N_...`), so selecting a piece in the viewport says which
building it belongs to. The table of (style, position, size, height, note) is
printed at the end. Family-05 curtain walls carry a per-building glass tint.

Environment:
    URBAN_SEED        RNG seed for façade variety (default 7)
    URBAN_COLLIDERS   1 to apply PhysX colliders (default off: ~14,000 gprims)
    URBAN_SNAP_DIR    write viewport captures (overview + per row) here
"""

import os
import random
import sys
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

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
from detail import urban_building

# ----- CONFIGURATION -----
ENV_URL = SIMULATION_ENVIRONMENTS["Default Environment"]
STAGE_SCALE = 1.00
SEED = int(os.environ.get("URBAN_SEED", "7"))
COLLIDERS = os.environ.get("URBAN_COLLIDERS", "0") in ("1", "true", "yes")
SNAP_DIR = os.environ.get("URBAN_SNAP_DIR", "").strip()
MERGED_ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443"
               "/Projects/SEI-COA/ModernCityEnvironment/")
MERGED = [MERGED_ROOT + f"Collected_Building{n}/SM_MERGED_BP_MBuilding{n}.usd"
          for n in ("01", "02", "03", "05")]
ROW_W = 240.0        # assembled rows wrap at this width
GAP = 14.0           # between buildings in a row
ROW_GAP = 45.0       # street between rows
PARENT = "/World/stage/generated"
CAM = "/World/showcaseCam"
# -------------------------


def measure(url):
    """(min, max, scale): bbox in the asset's own units and the uniform scale
    that makes it metric (0.01 for the cm-authored merged buildings)."""
    stage = Usd.Stage.Open(url)
    if not stage:
        return None
    dp = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    r = bc.ComputeWorldBound(dp).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    ext = max(mx[i] - mn[i] for i in range(3))
    scale = 0.01 if ext > 500.0 else 1.0
    return ([mn[i] for i in range(3)], [mx[i] for i in range(3)], scale)


def merged_row(y_front):
    """The merged buildings packed along +X with their north face on *y_front*."""
    placements, where = [], []
    x = 0.0
    for url in MERGED:
        m = measure(url)
        name = url.rsplit("/", 1)[-1].replace(".usd", "")
        if m is None:
            print(f"[showcase] WARN could not measure {name}")
            continue
        mn, mx, sc = m
        sx, sy, sz = ((mx[i] - mn[i]) * sc for i in range(3))
        y0 = y_front - sy
        placements.append({
            "usd": url, "x_m": x - mn[0] * sc, "y_m": y0 - mn[1] * sc,
            "z_m": -mn[2] * sc, "yaw_deg": 0.0, "roll_deg": 0.0, "pitch_deg": 0.0,
            "scale": sc, "axis_up": "Z", "raw_pivot": True,
            "category": "merged_" + name.replace("SM_MERGED_BP_", "")})
        where.append((name, x + sx / 2, y0 + sy / 2, sx, sy, sz, sc))
        x += sx + GAP
    return placements, where


def add_colliders_skip_empty(prim):
    applied = skipped = 0
    for p in Usd.PrimRange(prim):
        if not p.IsA(UsdGeom.Gprim):
            continue
        if p.IsA(UsdGeom.Mesh):
            pts = p.GetAttribute("points")
            if not pts or not pts.HasAuthoredValue() or not (pts.Get() or []):
                skipped += 1
                continue
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(p)
            applied += 1
    print(f"[showcase] colliders: {applied} applied, {skipped} empty skipped")


def report_missing(stage, placements):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    empty = {}
    for p in placements:
        path = p.get("prim_path")
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            continue
        if bc.ComputeWorldBound(prim).ComputeAlignedRange().IsEmpty():
            empty[p["usd"]] = empty.get(p["usd"], 0) + 1
    if empty:
        print(f"[showcase] WARN {sum(empty.values())} placements drew nothing:")
        for usd, n in sorted(empty.items(), key=lambda kv: -kv[1]):
            print(f"[showcase]   {n:4d} x {usd.rsplit('/', 1)[-1]}")
    else:
        print("[showcase] every placement drew geometry")


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
    print(f"[showcase] snapshot -> {path}")


class ShowcaseApp:
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

        problems = urban_building.check()
        if problems:
            raise RuntimeError("urban_building.check() failed: " + "; ".join(problems))

        rng = random.Random(SEED)
        assembled, where, n_rows = urban_building.build_showcase(
            rng, row_w=ROW_W, gap_m=GAP, row_gap_m=ROW_GAP, y0=0.0)
        merged, merged_where = merged_row(y_front=-ROW_GAP)
        placements = merged + assembled
        print("[showcase] " + urban_building.summarise(assembled)
              .replace("\n", "\n[showcase] "))
        self.where, self.merged_where, self.n_rows = where, merged_where, n_rows

        _, ssf = get_stage_meters_per_unit(stage)
        sg.apply_placements(stage, placements, PARENT, ssf)
        for _ in range(10):
            omni.kit.app.get_app().update()
        if COLLIDERS:
            gen = stage.GetPrimAtPath(PARENT)
            if gen.IsValid():
                add_colliders_skip_empty(gen)
            for _ in range(10):
                omni.kit.app.get_app().update()
        report_missing(stage, placements)
        n_tint = urban_building.apply_glass_tint(stage, placements)
        print(f"[showcase] glass tint: {n_tint} window textures scaled")
        add_sky(stage, "")

        # Camera poses: one overview from the south, then one per row.
        ys = sorted({round(w[2] - w[4] / 2) for w in where})     # row fronts
        y_back = max(w[2] + w[4] / 2 for w in where)
        y_front = min(w[2] - w[4] / 2 for w in merged_where) if merged_where else 0.0
        depth = y_back - y_front
        self.poses = [("overview", (ROW_W / 2, y_front - 0.9 * max(ROW_W, depth),
                                    0.55 * max(ROW_W, depth)),
                       (ROW_W / 2, y_front + depth * 0.45, 10.0))]
        if merged_where:
            ym = min(w[2] - w[4] / 2 for w in merged_where)
            hm = max(w[5] for w in merged_where)
            self.poses.append(("merged", (ROW_W / 2, ym - 0.7 * ROW_W, 0.6 * hm + 25.0),
                               (ROW_W / 2, ym + 15.0, hm * 0.4)))
        # High oblique per row: the 69 m merged block in front would otherwise
        # sit between an eye-level camera and the first rows.
        for r, yf in enumerate(ys):
            hs = [w[5] for w in where if round(w[2] - w[4] / 2) == yf]
            self.poses.append((f"row{r}", (ROW_W / 2, yf - 0.5 * ROW_W, 0.7 * ROW_W),
                               (ROW_W / 2, yf + 15.0, max(hs) * 0.3)))
        aim_camera(stage, *self.poses[0][1:])

        print("\n" + "=" * 96)
        print("URBAN SHOWCASE READY")
        print(f"  seed {SEED}   {len(where)} assembled buildings in {n_rows} rows"
              f" + {len(merged_where)} merged   colliders {'on' if COLLIDERS else 'off'}")
        print("  front row — the pack's merged buildings:")
        for n, x, y, sx, sy, sz, sc in merged_where:
            print(f"    {n:<30} ({x:6.1f},{y:7.1f})  {sx:3.0f} x {sy:3.0f} m  "
                  f"{sz:3.0f} m tall  scale {sc}")
        print("  assembled, shortest row first:")
        for s, x, y, w, d, h in where:
            note = (urban_building.STYLES[s]["note"] if s in urban_building.STYLES
                    else "harvested Downtown_West block (artist-assembled)")
            print(f"    {s:<20} ({x:6.1f},{y:6.1f})  {w:3.0f} x {d:3.0f} m  {h:3.0f} m  {note}")
        print("=" * 96 + "\n")
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
            print(f"[showcase] {len(self.poses)} snapshots in {SNAP_DIR}")
        while simulation_app.is_running():
            app.update()
        self.timeline.stop()
        simulation_app.close()


def main():
    ShowcaseApp().run()


if __name__ == "__main__":
    main()
