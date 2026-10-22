"""Survivors from a spec -> a people layer in data/recon/ (Kit: the rigs are kit-written crates).

    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh place_people.py                       # specs/survivors.yaml -> people.usd
    OMNI_KIT_ACCEPT_EULA=YES ~/isaacsim/python.sh place_people.py survivors/v1/survivors.yaml people_v1.usd

Spec paths are relative to data/recon/; the layer is written next to the scene, so its rig paths
stay relative. Also writes each posed person's skinned surface points to <layer>.points.npz for
verify_people.py. The rigs have no SkelRoot, which RTX does not need but UsdSkel's baking does,
so the pose is skinned here directly (linear blend, as the renderer does).

Each survivor is a RenderPeople rig (mirrored from Nucleus Library/Stages/Muyang/People by
nucleus_mirror.py) placed through scene_gen's own `detail.site_features._place_person`: it binds
the procedural pose, rolls and lifts the lying poses and drops the seated ones onto their support
by the rig's measured hip -- none of which is guessed here. Class `person`, with the source frame
and note as custom data.
"""
import json, os, sys
from pathlib import Path
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np
import yaml
from pxr import Usd, UsdGeom, UsdSkel, Sdf, Vt, Gf
HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent)]
from _paths import R, SPECS
from detail.site_features import _place_person

os.chdir(R)                                          # the rig paths below are relative to data/recon
ROOT = "./assets/nucleus/Library/Stages"
spec = Path(sys.argv[1]) if len(sys.argv) > 1 else SPECS / "survivors.yaml"
name = sys.argv[2] if len(sys.argv) > 2 else "people.usd"
stage = Usd.Stage.CreateNew(str(R / name))
UsdGeom.SetStageUpAxis(stage, "Z"); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/people"); stage.SetDefaultPrim(root.GetPrim())
cache = {}
for s in yaml.safe_load(open(spec)):
    xf = UsdGeom.Xform.Define(stage, f"/people/{s['id']}")
    rig = f"Muyang/People/Assets/rp_{s['rig']}_rigged_00{ {'carla': 1, 'claudia': 2, 'eric': 1, 'manuel': 1, 'nathan': 3, 'sophia': 3}[s['rig']] }_ue4.usd"
    f = {"asset": rig, "pose": s["pose"], "at": s["at"][:2], "z_m": s["at"][2], "yaw_deg": s["yaw"] + 90.0}    # art faces -Y
    _place_person(stage, xf, f, ROOT, 1.0, cache)
    p = xf.GetPrim(); p.AddAppliedSchema("SemanticsLabelsAPI:class")
    p.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set(["person"])
    p.SetCustomDataByKey("frame", s.get("frame") or ""); p.SetCustomDataByKey("note", s.get("note") or "")
    print(s["id"], s["pose"], p.GetCustomDataByKey("poseDropM"), flush=True)
stage.Save()
xc = UsdGeom.XformCache(Usd.TimeCode.Default())
skel_cache = UsdSkel.Cache()
points = {}
for person in root.GetPrim().GetChildren():
    kids = list(Usd.PrimRange(person))
    skel = next(p for p in kids if p.IsA(UsdSkel.Skeleton))
    mesh = next(p for p in kids if p.IsA(UsdGeom.Mesh))
    sq = skel_cache.GetSkelQuery(UsdSkel.Skeleton(skel))
    xf = sq.ComputeSkinningTransforms(Usd.TimeCode.Default())
    bind = UsdSkel.BindingAPI(mesh)
    joints = bind.GetJointsAttr().Get()
    if joints:
        xf = UsdSkel.AnimMapper(sq.GetJointOrder(), joints).RemapTransforms(xf, Vt.Matrix4dArray(len(joints), Gf.Matrix4d(1)))
    idx, wts = bind.GetJointIndicesPrimvar(), bind.GetJointWeightsPrimvar()
    pts = Vt.Vec3fArray(UsdGeom.Mesh(mesh).GetPointsAttr().Get())
    UsdSkel.SkinPointsLBS(bind.GetGeomBindTransformAttr().Get() or Gf.Matrix4d(1), xf, idx.Get(), wts.Get(),
                          idx.GetElementSize(), pts)
    W = np.asarray(xc.GetLocalToWorldTransform(skel))
    p = np.asarray(pts, np.float64) @ W[:3, :3] + W[3, :3]
    points[person.GetName()] = p[np.random.default_rng(0).permutation(len(p))[:4000]].astype(np.float32)
np.savez_compressed(R / f"{name}.points.npz", **points)
print("->", R / name)
app.close()
