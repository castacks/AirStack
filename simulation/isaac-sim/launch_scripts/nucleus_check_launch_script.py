#!/usr/bin/env python
"""Stat the Nucleus paths the assembly needs, and exit. Fast resolution check."""
import os
from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": True})
import omni.client

ROOT = os.environ.get(
    "NUCLEUS_ROOT",
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets")
PATHS = [
    # green tree species (pristine trees + layout measure)
    "aec/brownstone/Assets/Vegetation/Trees/American_Beech.usd",
    "aec/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd",
    "aec/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd",
    "aec/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd",
    "aec/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd",
    "aec/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd",
    # a tree material + texture dep
    "aec/brownstone/Assets/Vegetation/Trees/materials",
    "aec/tower/Assets/Vegetation/Black_Oak/materials/textures",
    # the metal-prop mdl that also failed
    "aec/brownstone/Materials/Base/Metals/Steel_Cast.mdl",
    # road / driveway / grass / scar materials
    "materials/megascans/Road_Asphalt.usda",
    "materials/megascans/Damaged_Asphalt.usda",
    "materials/megascans/Brick_Wall_Worn.usda",
    "materials/megascans/Burnt_Forest_Floor.usda",
    "materials/Grass_Cut.usda",
    # our archetypes (should already be there)
    "archetypes/archetypes.json",
]
print("\n===== NUCLEUS RESOLUTION CHECK =====")
print("root:", ROOT)
ok = miss = 0
for rel in PATHS:
    url = ROOT.rstrip("/") + "/" + rel
    try:
        res, entry = omni.client.stat(url)
        good = str(res).endswith("OK")
    except Exception as exc:
        good = False
    print("  {0}  {1}".format("OK  " if good else "MISS", rel))
    ok += 1 if good else 0
    miss += 0 if good else 1
print("----- {0} OK, {1} MISSING -----\n".format(ok, miss))
simulation_app.close()
