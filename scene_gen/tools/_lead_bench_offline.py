"""Drive the FULL bench authoring headless (plan-only import gate, real
authoring) onto a saved stage so the lead can audit the assembled result."""
import os
import sys

os.environ["UTB_PLAN_ONLY"] = "1"
os.environ.setdefault("UTB_SEED", "7")
base = "/isaac-sim/AirStack/simulation/isaac-sim"
sys.path.insert(0, base + "/launch_scripts")
sys.path.insert(0, base + "/utils")
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")

from pxr import Sdf, Usd, UsdGeom

out = "/isaac-sim/.cache/tornado_probe/bench_offline.usd"
if os.path.exists(out):
    os.remove(out)
stage = Usd.Stage.CreateNew(out)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

import urban_tornado_bench_launch_script as B

state = B.run(stage, 1.0)
ok = B.report(state)
stage.Save()
print("[driver] saved %s ok=%s" % (out, ok))
