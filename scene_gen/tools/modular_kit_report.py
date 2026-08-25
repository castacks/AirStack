#!/usr/bin/env python
"""
modular_kit_report.py — survey a Nucleus asset kit: tree, sizes, pivots.

The numbers in `detail/modular_house.py` came from this. Re-run it when the kit
changes, or point it at another one; assembling a house from a kit is entirely a
question of module span and pivot, and both have to be MEASURED. Guessing that
`Floor_8th__01` is centred on its 2.5 m module costs 25 cm on every tile.

Must run inside the isaac-sim container: `pxr` / `omni.client` only import once
SimulationApp has started, so this boots a headless app (~15 s) first. Output
goes to a file because Kit swallows stdout.

    docker cp scene_gen/tools/modular_kit_report.py isaac-sim:/tmp/r.py
    docker exec isaac-sim bash -c 'cd /isaac-sim && \
        PYTHONPATH="$ISAAC_SIM_PYTHONPATH" ./python.sh /tmp/r.py >/dev/null 2>&1; \
        cat /tmp/kit_report.txt'

Environment:
    KIT_ROOT    omniverse:// URL to walk (default: ModularNeighborhood)
    KIT_OUT     output path (default /tmp/kit_report.txt)
    KIT_DEPTH   recursion depth (default 6)

WHAT TO READ IN THE OUTPUT
--------------------------
* `size_m` is the MESH extent, which is not the module span — a wall piece is
  20 cm wider than its module so butted pieces overlap at the corner.
* `bbox_units` is origin-relative, and is the important column: a piece whose
  min/max are not symmetric has an off-centre pivot, and every placement of it
  needs that offset applied. `Garage_Door_01offset` runs -199.9..84.4, i.e. it
  hangs 2 m BELOW its origin.
* `meshes=0 points=0` means the USD is a MATERIAL, not geometry. Referencing it
  places a prim that loads clean and draws nothing. Those assets normally have a
  `<name>_0.usd` sibling that carries the mesh.
"""

import os
import traceback

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})

import carb                                    # noqa: E402
import omni.client                             # noqa: E402
from pxr import Usd, UsdGeom                    # noqa: E402

OUT = os.environ.get("KIT_OUT", "/tmp/kit_report.txt")
ROOT = os.environ.get(
    "KIT_ROOT",
    "omniverse://airlab-nucleus.andrew.cmu.edu:443"
    "/Library/Stages/Muyang/ModularNeighborhood/")
MAX_DEPTH = int(os.environ.get("KIT_DEPTH", "6"))

# Texture and material trees are enormous and hold no placeable geometry.
SKIP = ("textures", "texture", "materials", "material", "maps", ".thumbs",
        "sourceimages", "images", "thumbs")
USD_EXT = (".usd", ".usda", ".usdc", ".usdz")


def walk(url, depth, out, files):
    if depth > MAX_DEPTH:
        return
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        out.write(f"!! {url} -> {res}\n")
        return
    dirs = []
    for e in sorted(entries, key=lambda x: x.relative_path):
        name = e.relative_path
        if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
            dirs.append(name)
            out.write(f"{'  ' * depth}D {name}/\n")
        else:
            out.write(f"{'  ' * depth}F {name}  ({getattr(e, 'size', 0)} B)\n")
            if name.lower().endswith(USD_EXT):
                files.append(url + name)
    for d in dirs:
        if d.lower() in SKIP:
            out.write(f"{'  ' * (depth + 1)}... skipped (texture/material tree)\n")
            continue
        walk(f"{url}{d}/", depth + 1, out, files)


def measure(path, out):
    try:
        stage = Usd.Stage.Open(path)
        if not stage:
            out.write("  !! could not open\n")
            return
        dp = stage.GetDefaultPrim()
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        up = UsdGeom.GetStageUpAxis(stage)
        out.write(f"  defaultPrim={dp.GetPath() if dp else None}  "
                  f"metersPerUnit={mpu}  upAxis={up}\n")
        root = dp if dp else stage.GetPseudoRoot()
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        rng = bc.ComputeWorldBound(root).ComputeAlignedRange()
        if rng.IsEmpty():
            out.write("  bbox EMPTY\n")
        else:
            mn, mx = rng.GetMin(), rng.GetMax()
            sz = mx - mn
            out.write(f"  bbox_units  min=({mn[0]:.2f},{mn[1]:.2f},{mn[2]:.2f}) "
                      f"max=({mx[0]:.2f},{mx[1]:.2f},{mx[2]:.2f})\n")
            out.write(f"  size_m      {sz[0] * mpu:.3f} x {sz[1] * mpu:.3f} x "
                      f"{sz[2] * mpu:.3f}\n")
        kids = list(root.GetChildren()) if root else []
        out.write(f"  children({len(kids)}): "
                  + ", ".join(f"{c.GetName()}[{c.GetTypeName()}]"
                              for c in kids[:25]) + "\n")
        nmesh = npts = 0
        for p in Usd.PrimRange(root):
            if p.IsA(UsdGeom.Mesh):
                nmesh += 1
                a = p.GetAttribute("points")
                if a and a.HasAuthoredValue():
                    v = a.Get()
                    npts += len(v) if v else 0
        out.write(f"  meshes={nmesh}  points={npts}"
                  + ("   <- MATERIAL ONLY, references nothing visible\n"
                     if nmesh == 0 else "\n"))
    except Exception:
        out.write("  !! " + traceback.format_exc().replace("\n", "\n     ") + "\n")


def main():
    srv = os.environ.get("OMNI_SERVER", "").strip().strip('"').rstrip("/")
    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", srv)
    files = []
    with open(OUT, "w") as out:
        out.write(f"===== TREE {ROOT} (depth {MAX_DEPTH}) =====\n")
        walk(ROOT, 0, out, files)
        out.write(f"\n===== {len(files)} USD FILE(S) =====\n")
        for f in files:
            out.write(f"\n--- {f[len(ROOT):]} ---\n")
            measure(f, out)
    print(f"wrote {OUT}: {len(files)} assets")


main()
app.close()
