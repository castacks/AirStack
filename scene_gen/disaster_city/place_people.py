"""Survivors from specs/survivors.yaml -> data/recon/people.usd (Kit: the rigs are kit-written crates).

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh place_people.py

Each survivor is a RenderPeople rig (mirrored from Nucleus Library/Stages/Muyang/People by
nucleus_mirror.py) placed through scene_gen's own `detail.site_features._place_person`: it binds
the procedural pose, rolls and lifts the lying poses and drops the seated ones onto their support
by the rig's measured hip -- none of which is guessed here. Class `person`, with the source frame
and note as custom data.
"""
import os, sys
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import yaml
from pxr import Usd, UsdGeom, Sdf
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
from _paths import R, SPECS
from detail.site_features import _place_person

os.chdir(R)                                          # the rig paths below are relative to data/recon
ROOT = "./assets/nucleus/Library/Stages"
stage = Usd.Stage.CreateNew(str(R / "people.usd"))
UsdGeom.SetStageUpAxis(stage, "Z"); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/people"); stage.SetDefaultPrim(root.GetPrim())
cache = {}
for s in yaml.safe_load(open(SPECS / "survivors.yaml")):
    xf = UsdGeom.Xform.Define(stage, f"/people/{s['id']}")
    rig = f"Muyang/People/Assets/rp_{s['rig']}_rigged_00{ {'carla': 1, 'claudia': 2, 'eric': 1, 'manuel': 1, 'nathan': 3, 'sophia': 3}[s['rig']] }_ue4.usd"
    f = {"asset": rig, "pose": s["pose"], "at": s["at"][:2], "z_m": s["at"][2], "yaw_deg": s["yaw"] + 90.0}    # art faces -Y
    _place_person(stage, xf, f, ROOT, 1.0, cache)
    p = xf.GetPrim(); p.AddAppliedSchema("SemanticsLabelsAPI:class")
    p.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set(["person"])
    p.SetCustomDataByKey("frame", s["frame"]); p.SetCustomDataByKey("note", s["note"])
    print(s["id"], s["pose"], p.GetCustomDataByKey("poseDropM"), flush=True)
stage.Save()
print("->", R / "people.usd")
app.close()
