"""Compose everything built so far into one stage: data/recon/disaster_city.usda.

    ~/.venvs/recon/bin/python assemble_scene.py

References (world coordinates throughout, Z-up, metres):
  /World/lod1        data/recon/buildings_lod1.usd, minus every building that has a hero model
  /World/heroes/<ID> data/recon/*/<ID>.usd for each built hero (buildings, rubble)
  /World/tiles_R02   data/recon/rubble_east/R02_tiles.usd (tile cut-out, no drone footage)
  /World/site        data/recon/site_ground.usd (ground.py: terrain + classified tile surface)
  /World/trees, /World/vehicles  place_assets.py (tile canopy switched off when trees exist)
Add new pieces to PIECES; a hero's ID deactivates its LOD1 box.
"""
from pathlib import Path
from pxr import Usd, UsdGeom, Sdf
from _paths import R

PIECES = {"R01": "rubble_west/R01.usd", "B01": "b01/B01.usd", "R02": "rubble_east/R02_tiles.usd", "S01": "s01/S01.usd", "B03": "b03/B03.usd", "PAD": "pad/PAD.usd", "B06": "b06/B06.usd"}

stage = Usd.Stage.CreateNew(str(R / "disaster_city.usda"))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World"); stage.SetDefaultPrim(world.GetPrim())
if (R / "site_ground.usd").exists(): stage.DefinePrim("/World/site").GetReferences().AddReference("./site_ground.usd")
for name in ("trees", "vehicles", "debris"):                          # place_assets.py
    if (R / f"{name}.usd").exists(): stage.DefinePrim(f"/World/{name}").GetReferences().AddReference(f"./{name}.usd")
if (R / "trees.usd").exists():                              # the placed trees replace the tile canopy
    stage.OverridePrim("/World/site/tiles_vegetation").SetActive(False)
lod1 = stage.DefinePrim("/World/lod1"); lod1.GetReferences().AddReference("./buildings_lod1.usd")
for p in lod1.GetChildren():
    if p.GetName().split("_")[0] in PIECES: p.SetActive(False)
for pid, rel in PIECES.items():
    if not (R / rel).exists(): print(f"missing {rel}"); continue
    stage.DefinePrim(f"/World/{pid}").GetReferences().AddReference(f"./{rel}")
stage.Save()
print(f"wrote {R / 'disaster_city.usda'}: {len(PIECES)} pieces + {sum(p.IsActive() for p in lod1.GetChildren())} LOD1 boxes")
