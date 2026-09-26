"""Compose everything built so far into one stage: data/recon/disaster_city.usda.

    ~/.venvs/recon/bin/python assemble_scene.py

References (world coordinates throughout, Z-up, metres):
  /World/lod1        data/recon/buildings_lod1.usd, minus every building that has a hero model
  /World/heroes/<ID> data/recon/*/<ID>.usd for each built hero (buildings, rubble)
  /World/tiles_R02   data/recon/rubble_east/R02_tiles.usd (tile cut-out, no drone footage)
  /World/site        data/recon/site_ground.usd (ground.py: terrain + classified tile surface)
  /World/trees, /World/vehicles  place_assets.py (tile canopy switched off when trees exist)
  /World/people      place_people.py: the survivors seen in the video (specs/survivors.yaml)
Add new pieces to PIECES; a hero's ID deactivates its LOD1 box.
"""
from pathlib import Path
from pxr import Usd, UsdGeom, UsdLux, Sdf, Gf
from _paths import R

PIECES = {"R01": "rubble_west/R01_assets.usd", "B01": "b01/B01.usd", "R02": "rubble_east/R02_assets.usd", "S01": "s01/S01.usd", "B03": "b03/B03.usd", "PAD": "pad/PAD.usd", "B06": "b06/B06.usd", "S08": "s08/S08.usd", "B35": "b35/B35.usd", "D01": "d01/D01.usd", "S09": "s09/S09.usd"}

stage = Usd.Stage.CreateNew(str(R / "disaster_city.usda"))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World"); stage.SetDefaultPrim(world.GetPrim())
if (R / "site_ground.usd").exists(): stage.DefinePrim("/World/site").GetReferences().AddReference("./site_ground.usd")
for name in ("trees", "vehicles", "debris", "people"):                # place_assets.py; place_people.py
    if (R / f"{name}.usd").exists(): stage.DefinePrim(f"/World/{name}").GetReferences().AddReference(f"./{name}.usd")
if (R / "trees.usd").exists():                              # the placed trees replace the tile canopy
    stage.OverridePrim("/World/site/tiles_vegetation").SetActive(False)
# the scene carries its own sun + sky: a frozen scene with no sky light renders black on another
# machine (see .agents/skills/freeze-portable-scenes). The HDR is an Isaac-shipped outdoor probe,
# copied into data/recon/sky/ so the relative path travels with package.py.
env = UsdGeom.Xform.Define(stage, "/World/Environment")
sun = UsdLux.DistantLight.Define(stage, "/World/Environment/sun"); sun.CreateIntensityAttr(3000.0); sun.CreateAngleAttr(0.5)
UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(35, 0, 150))            # ~55 deg sun elevation from the SSE
sky = UsdLux.DomeLight.Define(stage, "/World/Environment/sky"); sky.CreateIntensityAttr(1000.0)
sky.CreateTextureFileAttr(Sdf.AssetPath("./sky/sunflowers.hdr")); sky.CreateTextureFormatAttr(UsdLux.Tokens.latlong)
lod1 = stage.DefinePrim("/World/lod1"); lod1.GetReferences().AddReference("./buildings_lod1.usd")
for p in lod1.GetChildren():
    if p.GetName().split("_")[0] in PIECES: p.SetActive(False)
for pid, rel in PIECES.items():
    if not (R / rel).exists(): print(f"missing {rel}"); continue
    stage.DefinePrim(f"/World/{pid}").GetReferences().AddReference(f"./{rel}")
stage.Save()

# the BEFORE scene for comparisons: the raw Google tiles (tiles_extract.py disc, r 400 m round the
# site), no grafts, same sun + sky -- so a render of each from the same camera shows what the
# pipeline changed. Built only if the raw export exists.
if (R / "raw_tiles/tiles_site.usd").exists():
    raw = Usd.Stage.CreateNew(str(R / "disaster_city_raw_tiles.usda"))
    UsdGeom.SetStageUpAxis(raw, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(raw, 1.0)
    w = UsdGeom.Xform.Define(raw, "/World"); raw.SetDefaultPrim(w.GetPrim())
    raw.DefinePrim("/World/tiles").GetReferences().AddReference("./raw_tiles/tiles_site.usd")
    raw.DefinePrim("/World/Environment").GetReferences().AddReference("./disaster_city.usda", "/World/Environment")
    raw.Save()
    print(f"wrote {R / 'disaster_city_raw_tiles.usda'} (before scene)")
print(f"wrote {R / 'disaster_city.usda'}: {len(PIECES)} pieces + {sum(p.IsActive() for p in lod1.GetChildren())} LOD1 boxes")
