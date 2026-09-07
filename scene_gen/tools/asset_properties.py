#!/usr/bin/env python3
"""What each asset in a pack IS, before any disaster touches it.

    AirStack/.venv/bin/python scene_gen/tools/asset_properties.py \\
        --config urban_v2 --disaster earthquake -o scene_gen/_bakelab

The other half of a bake report. `archetypes/bake.py` records what a bake COST
and PRODUCED; this records what it was HANDED, for every type the plan names —
including the ones a bake never reached, which is what makes "we baked 40 of 89"
readable as a fact about the assets rather than only about the clock.

Everything here is a candidate explanation for a slow cell: point count first
(fracture is superlinear in it), then whether the art is a shell that
`solidify` has to give walls to, then how many separate meshes the operators
have to walk, then the material — which sets the fragment size, and so the
fragment COUNT, and so the settle.

READS LOCAL MIRRORS, NOT NUCLEUS. Plain `pxr` cannot open an `omniverse://`
URL (that needs `omni.usd_resolver`, which lives inside Kit), so every entry is
resolved to its on-disk twin through `tools/local_mirror.py`. An asset with no
mirror is reported with `local: false` and no geometry columns rather than
skipped — the pack still names it, and its absence here is itself worth seeing.

Related: `tools/measure_assets.py` does the same measuring for a bare directory
of USDs with no pack behind it, which is the right tool when vetting art that
is not wired up yet.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_SCENE_GEN, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import local_mirror as M                                        # noqa: E402


def measure(path: str, scale: float) -> dict:
    """Geometry and material counts for one USD, at the scene's scale."""
    from pxr import Usd, UsdGeom, UsdShade

    stage = Usd.Stage.Open(path)
    if stage is None:
        return {"opened": False}
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage) or 1.0)
    up = str(UsdGeom.GetStageUpAxis(stage) or "Z")

    meshes = points = faces = 0
    mats, textures = set(), set()
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            meshes += 1
            m = UsdGeom.Mesh(prim)
            pts = m.GetPointsAttr().Get()
            points += len(pts) if pts else 0
            fvc = m.GetFaceVertexCountsAttr().Get()
            faces += len(fvc) if fvc else 0
        if prim.IsA(UsdShade.Material):
            mats.add(prim.GetName())
        if prim.IsA(UsdShade.Shader):
            for inp in UsdShade.Shader(prim).GetInputs():
                val = inp.Get()
                if hasattr(val, "path") and str(val.path):
                    textures.add(str(val.path))

    root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    bb = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
    ).ComputeWorldBound(root).ComputeAlignedRange()
    # THE SCENE'S SCALE, not the file's. A pack entry carries `scale:` exactly
    # because these packs are not all authored in metres, and a footprint
    # reported in authored units is not comparable across two rows of the
    # table. `bake.Item.scale` is the same number Stage A references at.
    size = [0.0, 0.0, 0.0] if bb.IsEmpty() else [
        round(float(bb.GetSize()[i]) * scale, 2) for i in range(3)]

    # Do the maps this asset names actually exist on disk? A texture that
    # resolves only inside Kit is why a preview render comes out grey.
    tex_local = sum(1 for t in textures if M.local_for(M.to_url(t)))
    return {
        "opened": True, "mpu": mpu, "up_axis": up,
        "meshes": meshes, "points": points, "faces": faces,
        "materials": len(mats), "textures": len(textures),
        "textures_on_disk": tex_local,
        "x_m": size[0], "y_m": size[1], "z_m": size[2],
        "file_mb": round(os.path.getsize(path) / 1e6, 2),
    }


def collect(config_name: str, disaster: str) -> list:
    """One row per planned type: what the pack says, plus what the file says."""
    from compile_disaster import load_scene_config
    from archetypes import plan as P
    from scene_generator import solid_assets
    from disaster import mesh_damage as md
    from disaster import kinds

    config = load_scene_config(config_name)
    items = P.build_plan(config, disaster)
    solid = set(solid_assets(config))
    mcfg = (config.get("disaster") or {}).get("mesh_damage") or {}
    table = kinds._pack_materials(config)

    rows = []
    for it in items:
        url = M.to_url(it.source)
        local = M.local_for(url) if it.build == "library" else None
        row = {
            "type": it.type, "kind": it.kind, "build": it.build,
            "source": str(it.source), "levels": len(it.levels),
            "scale": it.scale, "axis_up_cfg": it.axis_up,
            # DECLARED, not detected — `solid_assets` explains why. This is
            # the flag that decides whether `solidify` runs, which is the
            # single biggest lever on both bake time and output size.
            "hollow": it.source not in solid,
            "material": md.material_for_asset(
                it.source, mcfg.get("material"), table),
            "local": bool(local),
            "local_path": os.path.relpath(local, _SCENE_GEN) if local else "",
        }
        if local:
            try:
                row.update(measure(local, it.scale))
            except Exception as exc:                            # noqa: BLE001
                row["opened"] = False
                row["error"] = f"{type(exc).__name__}: {exc}"
        rows.append(row)
        print(f"  {row['type']:28s} {'ok ' if row.get('opened') else '-- '}"
              f"{row.get('points', 0):>9,} pts  {row.get('meshes', 0):>4} mesh"
              f"  {row.get('x_m', 0):6.1f}x{row.get('y_m', 0):6.1f}"
              f"x{row.get('z_m', 0):6.1f} m  {row['material']}", flush=True)
    return rows


COLUMNS = ("type", "kind", "build", "material", "hollow", "levels",
           "x_m", "y_m", "z_m", "points", "faces", "meshes", "materials",
           "textures", "textures_on_disk", "file_mb", "mpu", "up_axis",
           "scale", "local", "opened", "source", "local_path")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", default="urban_v2")
    ap.add_argument("--disaster", default="earthquake")
    ap.add_argument("-o", "--out", default=os.path.join(_SCENE_GEN, "_bakelab"))
    args = ap.parse_args()

    rows = collect(args.config, args.disaster)
    os.makedirs(args.out, exist_ok=True)
    jpath = os.path.join(args.out, "asset_properties.json")
    cpath = os.path.join(args.out, "asset_properties.csv")
    json.dump(rows, open(jpath, "w"), indent=1, sort_keys=True)
    with open(cpath, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    n_local = sum(1 for r in rows if r.get("opened"))
    print(f"\n{len(rows)} type(s), {n_local} measured locally\n  {jpath}\n"
          f"  {cpath}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
