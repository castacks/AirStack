"""asset_catalogue_launch_script.py — every building of an asset pack, alone,
under the sim's own RTX renderer.

    CAT_PACK=urban_v2 SNAP_DIR=/isaac-sim/.nvidia-omniverse/logs/cat_urban_v2 \
    PYTHONPATH="$ISAAC_SIM_PYTHONPATH" /isaac-sim/python.sh \
        /isaac-sim/AirStack/simulation/isaac-sim/launch_scripts/asset_catalogue_launch_script.py \
        --ext-folder ~/.local/share/ov/data/documents/Kit/shared/exts

WHY THIS EXISTS. `scene_gen/tools/building_gallery.py` renders the same pack in
Cycles, which cannot read MDL and does not share RTX's opinion of a material:
`DownTown/Assets/BG_Building_*` towers that Cycles shows as dark glazing come
out as SOLID BLACK slabs in the sim. Only the sim can say what the sim shows,
so this lays every building on a line — metres, Z-up, the pack's own
`scale`/`axis-up` — and photographs each from two azimuths with
`utils/snapshots.py` (1280x720; the high-resolution set is the Cycles gallery).

    <SNAP_DIR>/00_overview.png
    <SNAP_DIR>/<pool>__<name>__az035.png   front-right (+X, +Y faces)
    <SNAP_DIR>/<pool>__<name>__az215.png   back-left (-X, -Y faces)

`CAT_POOLS` (comma list, default `intact,damaged,destroyed`) picks the
sub-trees; `CAT_MATCH` is a regex on the file stem. `CAT_CATEGORY` picks the
`usds.<category>` tree and defaults to `buildings` — set it to `debris` to lay
out the rubble instead.
"""
import math
import os
import re
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import omni.kit.app                                                 # noqa: E402
import omni.usd                                                     # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux                       # noqa: E402

_ISAAC_SIM_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import yaml                                                         # noqa: E402
import scene_generator as sg                                        # noqa: E402

PACK = os.environ.get("CAT_PACK", "urban_v2")
SNAP_DIR = os.environ.get("SNAP_DIR", "").strip()
POOLS = [p.strip() for p in os.environ.get("CAT_POOLS", "intact,damaged,destroyed").split(",") if p.strip()]
#: Which `usds.<category>` tree to photograph. `buildings` is the original and
#: the default; `debris` catalogues the rubble the same way, which is how the
#: piles / pieces / scatters split was judged.
CATEGORY = os.environ.get("CAT_CATEGORY", "buildings").strip() or "buildings"
#: Camera elevation, degrees. 20 shows a building's facade, which is the point
#: for `buildings`. FLAT ASSETS NEED A HIGH ANGLE: the DebrisGroundScatter pack
#: is decal CARDS with zero height, and at 20 deg they are edge-on and render
#: as nothing at all — every one of its 60 assets came back a blank frame.
#: Shoot those from 85.
ELEV = float(os.environ.get("CAT_ELEV", "20"))
MATCH = re.compile(os.environ.get("CAT_MATCH", "") or ".")
FRAMES = int(os.environ.get("CAT_FRAMES", "40"))


def entries():
    """[(pool, name, usd, scale, axis)] from the pack yaml."""
    pack_path = os.path.join(_SCENE_GEN_DIR, "config", "asset_packs", f"{PACK}.yaml")
    cfg = sg.resolve_asset_pack({"asset_pack": PACK}, pack_path)
    root = str(cfg.get("asset_root", "") or "")
    dscale = float(cfg.get("asset_scale", 1.0) or 1.0)
    out = []

    def walk(node, path):
        if isinstance(node, list):
            for e in node:
                usd, sc, ax, _yaw, _tags = sg._parse_usd_entry(e, dscale, root)
                name = os.path.splitext(os.path.basename(usd))[0]
                if MATCH.search(name):
                    out.append(("/".join(path), name, usd, sc, ax))
        elif isinstance(node, dict):
            for k, v in node.items():
                walk(v, path + [k])

    bld = (cfg.get("usds") or {}).get(CATEGORY) or {}
    for pool in POOLS:
        if pool in bld:
            walk(bld[pool], [pool])
    return out


def main():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.GetSizeAttr().Set(1.0)
    UsdGeom.Xformable(ground.GetPrim()).AddScaleOp().Set(Gf.Vec3f(20000.0, 20000.0, 0.2))
    UsdGeom.Xformable(ground.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.1))
    ground.GetDisplayColorAttr().Set([Gf.Vec3f(0.36, 0.36, 0.37)])

    dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
    dome.CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, "/World/Key")
    key.CreateIntensityAttr(2500.0)
    key.CreateAngleAttr(1.0)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 35.0))

    items = entries()
    print(f"[catalogue] {len(items)} building(s) from {PACK}: {POOLS}", flush=True)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render], useExtentsHint=False)

    # Reference everything FIRST, then wait for Nucleus to deliver: a
    # reference to an omniverse:// layer composes asynchronously, and a bbox
    # taken on the same frame is empty (measured: every DownTown and
    # FactoryDistrict asset came back EMPTY while the locally cached ones did not).
    app = omni.kit.app.get_app()
    prims = []
    for i, (pool, name, usd, sc, ax) in enumerate(items):
        path = Sdf.Path(f"/World/Cat/b{i:03d}_{re.sub('[^A-Za-z0-9_]', '_', name)}")
        # TYPELESS, then Load(): exactly what `scene_generator.apply_placements`
        # does and why — the DownTown assets have a Mesh as their ROOT prim, so
        # a local "Xform" opinion turns the gprim into a container that renders
        # nothing, and FactoryDistrict wraps its geometry in a payload that a
        # prim composed into a running stage does not load by itself.
        prim = stage.DefinePrim(path)
        prim.GetReferences().AddReference(usd)
        if prim.GetTypeName() == "Scope":
            prim.SetTypeName("Xform")
        prim.Load()
        xf = UsdGeom.Xformable(prim)
        t_op = xf.AddTranslateOp()
        if str(ax).upper() == "Y":
            xf.AddRotateXOp().Set(90.0)
        xf.AddScaleOp().Set(Gf.Vec3f(float(sc)))
        prims.append((pool, name, usd, prim, t_op))
    # Fixed pump, not "until loading status says done": the status counter
    # reports done before FactoryDistrict's payloads and DownTown's textures
    # are even requested. CAT_WAIT frames (default 900, ~1 min) is what it takes.
    for _ in range(int(os.environ.get("CAT_WAIT", "900"))):
        app.update()

    placed = []
    x = 0.0
    for pool, name, usd, prim, t_op in prims:
        bbox.Clear()
        rng = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
        if rng.IsEmpty():
            print(f"[catalogue] EMPTY: {pool}/{name} <- {usd}", flush=True)
            placed.append((pool, name, None, None))
            continue
        lo, hi = rng.GetMin(), rng.GetMax()
        size = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
        cx = x + size[0] / 2.0
        t_op.Set(Gf.Vec3d(cx - (lo[0] + hi[0]) / 2.0, -(lo[1] + hi[1]) / 2.0, -lo[2]))
        placed.append((pool, name, (cx, 0.0, size[2] / 2.0), size))
        print(f"[catalogue] {pool}/{name}: {size[0]:.1f} x {size[1]:.1f} x {size[2]:.1f} m", flush=True)
        x += size[0] + 40.0

    app = omni.kit.app.get_app()
    for _ in range(60):
        app.update()

    if SNAP_DIR:
        import importlib.util as _ilu
        spec = _ilu.spec_from_file_location("snapshots", os.path.join(_ISAAC_SIM_DIR, "utils", "snapshots.py"))
        snaps = _ilu.module_from_spec(spec)
        spec.loader.exec_module(snaps)
        os.makedirs(SNAP_DIR, exist_ok=True)
        tallest = max((s[2] for _, _, c, s in placed if s), default=50.0)
        snaps.overview(stage, (x / 2.0, 0.0), max(x * 1.05, tallest + 40.0),
                       os.path.join(SNAP_DIR, "00_overview.png"), 1.0, frames=FRAMES)
        for pool, name, c, size in placed:
            if c is None:
                continue
            r = 0.5 * math.sqrt(size[0] ** 2 + size[1] ** 2 + size[2] ** 2)
            dist = r / math.sin(math.radians(16.0)) * 1.05     # ~32 deg horizontal fov
            for az in (35, 215):
                a = math.radians(az)
                el = math.radians(ELEV)
                eye = (c[0] + dist * math.cos(el) * math.cos(a), c[1] + dist * math.cos(el) * math.sin(a),
                       c[2] + dist * math.sin(el))
                snaps.place_camera(stage, eye, (c[0], c[1], c[2]))
                snaps.snapshot(os.path.join(SNAP_DIR, f"{pool.replace('/', '_')}__{name}__az{az:03d}.png"), FRAMES)
        print(f"[catalogue] captures -> {SNAP_DIR}", flush=True)

    print("=" * 60)
    print(f"ASSET CATALOGUE READY: {len(placed)} building(s)")
    print("=" * 60, flush=True)
    while simulation_app.is_running():
        app.update()
    simulation_app.close()


if __name__ == "__main__":
    main()
