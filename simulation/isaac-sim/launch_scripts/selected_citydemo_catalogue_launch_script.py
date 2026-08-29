#!/usr/bin/env python
"""Blank-stage catalogue for every building in selected_citydemo.

The three source folders (houses, midrise, tower) are listed from Nucleus at
runtime.  Assets are measured before placement, seated by their actual bounds,
and packed into category-preserving rows.  Select a prim in the viewport to
read its source asset name.

Environment:
    SELECTED_ROW_W   maximum row width in metres (default 260)
    SELECTED_MARGIN  clear gap around each building (default 8)
    SNAP_DIR         optional mounted output directory for review captures
"""

import csv
import os
import time

from isaacsim import SimulationApp


def _env(name, default=""):
    value = os.environ.get(name)
    return default if not (value or "").strip() else value.strip()


simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
enable_extension("omni.kit.window.script_editor")

import omni.client  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux  # noqa: E402

ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "selected_citydemo/")
CATEGORIES = ("houses", "midrise", "tower")
ROW_W = float(_env("SELECTED_ROW_W", "260"))
MARGIN = float(_env("SELECTED_MARGIN", "8"))
SNAP_DIR = _env("SNAP_DIR", "")
CSV_PATH = "/isaac-sim/.nvidia-omniverse/logs/selected_citydemo_catalogue.csv"
_ISAAC = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), ".."))


def list_assets():
    assets = []
    for category in CATEGORIES:
        result, entries = omni.client.list(ROOT + category + "/")
        if result != omni.client.Result.OK:
            raise RuntimeError("could not list {0}: {1}".format(category, result))
        for entry in sorted(entries, key=lambda e: e.relative_path):
            if entry.relative_path.endswith(".usd"):
                assets.append((category, entry.relative_path,
                               ROOT + category + "/" + entry.relative_path))
    return assets


def measure(url):
    asset_stage = Usd.Stage.Open(url)
    if not asset_stage:
        return None
    prim = asset_stage.GetDefaultPrim() or asset_stage.GetPseudoRoot()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bound = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if bound.IsEmpty():
        return None
    mpu = UsdGeom.GetStageMetersPerUnit(asset_stage) or 1.0
    mn, mx = bound.GetMin(), bound.GetMax()
    return (tuple(float(mn[i]) * mpu for i in range(3)),
            tuple(float(mx[i]) * mpu for i in range(3)))


def make_layout(assets):
    measured, missing = [], []
    for category, name, url in assets:
        bounds = measure(url)
        if bounds is None:
            missing.append(category + "/" + name)
        else:
            measured.append((category, name, url, bounds[0], bounds[1]))

    placements, rows, table = [], [], []
    y = 0.0
    row_number = 0
    for category in CATEGORIES:
        members = [item for item in measured if item[0] == category]
        x = 0.0
        row_depth = 0.0
        row_start = y
        for _, name, url, mn, mx in members:
            sx, sy, sz = (mx[i] - mn[i] for i in range(3))
            if x and x + sx > ROW_W:
                rows.append((row_number, category, row_start, y, row_depth, x))
                row_number += 1
                y += row_depth + MARGIN * 2.0
                row_start, x, row_depth = y, 0.0, 0.0
            stem = name[:-4]
            placements.append((category, stem, url,
                               x - mn[0], y - mn[1], -mn[2]))
            table.append((category, stem, row_number,
                          x + sx / 2.0, y + sy / 2.0, sx, sy, sz))
            x += sx + MARGIN
            row_depth = max(row_depth, sy)
        if members:
            rows.append((row_number, category, row_start, y, row_depth, x))
            row_number += 1
            y += row_depth + MARGIN * 3.0
    return placements, rows, table, missing


def add_reference(stage, category, name, url, x, y, z):
    path = "/World/Buildings/{0}/{1}".format(category, name)
    holder = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    asset = stage.DefinePrim(Sdf.Path(path + "/asset"))
    asset.GetReferences().AddReference(url)
    xf = UsdGeom.Xformable(holder)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    return path


def light_and_ground(stage, width, depth):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/Dome"))
    dome.CreateIntensityAttr(850.0)
    dome.CreateColorAttr(Gf.Vec3f(0.78, 0.82, 0.90))
    sun = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/Sun"))
    sun.CreateIntensityAttr(3200.0)
    sun.CreateAngleAttr(0.7)
    sun.AddRotateXYZOp().Set(Gf.Vec3f(-38.0, 0.0, 28.0))
    ground = UsdGeom.Mesh.Define(stage, Sdf.Path("/World/Ground"))
    pad = 30.0
    pts = [Gf.Vec3f(-pad, -pad, -0.02),
           Gf.Vec3f(width + pad, -pad, -0.02),
           Gf.Vec3f(width + pad, depth + pad, -0.02),
           Gf.Vec3f(-pad, depth + pad, -0.02)]
    ground.CreatePointsAttr(pts)
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    ground.CreateDisplayColorAttr([Gf.Vec3f(0.24, 0.25, 0.27)])
    ground.CreateExtentAttr([pts[0], pts[2]])


def main():
    started = time.time()
    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/Buildings"))

    assets = list_assets()
    print("[selected-catalogue] listed {0} assets".format(len(assets)))
    placements, rows, table, missing = make_layout(assets)
    for category in CATEGORIES:
        UsdGeom.Scope.Define(stage, Sdf.Path("/World/Buildings/" + category))
    for rec in placements:
        add_reference(stage, *rec)
    for _ in range(30):
        omni.kit.app.get_app().update()

    depth = max((r[3] + r[4] for r in rows), default=ROW_W)
    light_and_ground(stage, ROW_W, depth)
    with open(CSV_PATH, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("category", "asset", "row", "x_m", "y_m",
                         "size_x", "size_y", "size_z"))
        writer.writerows(table)

    print("\n" + "=" * 78)
    print("SELECTED CITYDEMO CATALOGUE READY")
    print("  {0} buildings in {1} organized rows; {2:.0f} x {3:.0f} m plate"
          .format(len(placements), len(rows), ROW_W, depth))
    for category in CATEGORIES:
        count = sum(1 for row in table if row[0] == category)
        row_ids = [r[0] for r in rows if r[1] == category]
        print("  {0:<8}: {1:2d} assets, rows {2}".format(
            category, count, ",".join(str(i) for i in row_ids)))
    if missing:
        print("  missing geometry: " + ", ".join(missing))
    print("  table: " + CSV_PATH)
    print("  built in {0:.1f} s".format(time.time() - started))
    print("=" * 78 + "\n")

    if SNAP_DIR:
        try:
            import importlib.util
            path = os.path.join(_ISAAC, "utils", "snapshots.py")
            spec = importlib.util.spec_from_file_location("snapshots", path)
            snaps = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(snaps)
            snaps.overview(stage, (ROW_W / 2.0, depth / 2.0),
                           max(ROW_W, depth) * 1.08,
                           os.path.join(SNAP_DIR, "all_top.png"))
            for row_id, category, _, y, row_depth, used_w in rows:
                centre = (min(used_w, ROW_W) / 2.0, y + row_depth / 2.0)
                span = max(min(used_w, ROW_W), row_depth * 2.0, 80.0)
                snaps.place_camera(stage,
                    (centre[0] - span * 0.55, centre[1] - span * 0.55,
                     span * 0.42),
                    (centre[0], centre[1], min(20.0, row_depth)))
                snaps.snapshot(os.path.join(
                    SNAP_DIR, "{0}_row_{1:02d}.png".format(category, row_id)))
            print("[selected-catalogue] snapshots -> " + SNAP_DIR)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("[selected-catalogue] snapshots FAILED: {0}".format(exc))

    while simulation_app.is_running():
        omni.kit.app.get_app().update()
    simulation_app.close()


if __name__ == "__main__":
    main()
