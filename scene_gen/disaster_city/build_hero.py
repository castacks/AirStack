"""Enterable building from a measured spec -> USD of boxes with colliders.

    ~/.venvs/recon/bin/python build_hero.py specs/B01.yaml     # -> data/recon/b01/B01.usd

Everything is an axis-aligned box in the building's LOCAL frame (x along the
long side, y across, z up, origin at a ground-level corner); the spec's
`origin` + `yaw_deg` place that frame in the world. Boxes keep geometry and
collision identical and cheap -- a doorway in the model is a doorway for the
drone. Spec parts:

  box:    {name, min: [x,y,z], max: [x,y,z], mat}
  wall:   {name, from: [x,y], to: [x,y], z: [z0,z1], t: thickness, mat,
           openings: [{at: dist along wall to opening's left edge, w, sill, h}]}
          (walls must run along x or y)
  stair:  {name, from: [x,y,z], to: [x,y,z], width, mat}   straight run, one box per step
  rail:   {name, from: [x,y], to: [x,y], z, h: 1.1, mat}   posts every 1.5 m + top and mid rails
          (a solid box would read as a wall to a drone's depth sensor and collider)
  beam:   {name, from: [x,y,z], to: [x,y,z], t: 0.2, mat}   oriented square bar (bracing, booms, tilted slabs with t2)
  cyl:    {name, c: [x,y,z] (centre), r, h, axis: X|Y|Z, mat}
  sphere: {name, c: [x,y,z], r, mat}
"""
import sys
from pathlib import Path
import numpy as np, yaml
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf, Tf
from _paths import R

MATS = {"concrete": (0.62, 0.60, 0.56), "steel": (0.30, 0.31, 0.33), "grating": (0.22, 0.22, 0.24),
        "wood": (0.55, 0.42, 0.28), "rust": (0.45, 0.25, 0.15), "tank_black": (0.08, 0.08, 0.09), "tank_white": (0.85, 0.85, 0.83)}

def wall_boxes(w):
    (x0, y0), (x1, y1), (z0, z1), t = w["from"], w["to"], w["z"], w.get("t", 0.25)
    along_x = abs(x1 - x0) >= abs(y1 - y0)
    L = abs(x1 - x0) if along_x else abs(y1 - y0)
    sgn = np.sign((x1 - x0) if along_x else (y1 - y0)) or 1
    def seg(a, b, za, zb):                                  # a..b along the wall, za..zb height
        if b - a < 1e-3 or zb - za < 1e-3: return None
        if along_x:
            xa, xb = sorted((x0 + sgn * a, x0 + sgn * b)); return [xa, y0 - t / 2, za], [xb, y0 + t / 2, zb]
        ya, yb = sorted((y0 + sgn * a, y0 + sgn * b)); return [x0 - t / 2, ya, za], [x0 + t / 2, yb, zb]
    out, pos = [], 0.0
    for o in sorted(w.get("openings", []), key=lambda o: o["at"]):
        a, b = o["at"], o["at"] + o["w"]; s, top = z0 + o.get("sill", 0), z0 + o.get("sill", 0) + o["h"]
        out += [seg(pos, a, z0, z1), seg(a, b, z0, s), seg(a, b, top, z1)]
        pos = b
    out.append(seg(pos, L, z0, z1))
    return [b for b in out if b]

def stair_boxes(s):
    p0, p1 = np.array(s["from"], float), np.array(s["to"], float)
    n = max(1, int(round((p1[2] - p0[2]) / 0.18)))          # ~18 cm risers
    d = (p1[:2] - p0[:2]); run = np.linalg.norm(d); u = d / run; v = np.array([-u[1], u[0]]) * s["width"] / 2
    out = []
    for i in range(n):
        a, b = p0[:2] + u * run * i / n, p0[:2] + u * run * (i + 1) / n
        c = np.array([a + v, a - v, b + v, b - v]); z = p0[2] + (p1[2] - p0[2]) * (i + 1) / n
        out.append(([*c.min(0), z - 0.05], [*c.max(0), z]))
    return out

def rail_boxes(r):
    p0, p1 = np.array(r["from"], float), np.array(r["to"], float); z, h, t = r["z"], r.get("h", 1.1), 0.05
    L = np.linalg.norm(p1 - p0); u = (p1 - p0) / L
    out = []
    for i in range(int(np.ceil(L / 1.5)) + 1):
        c = p0 + u * min(i * 1.5, L); out.append(([c[0] - t, c[1] - t, z], [c[0] + t, c[1] + t, z + h]))
    lo, hi = np.minimum(p0, p1) - t / 2, np.maximum(p0, p1) + t / 2
    for zz in (z + h - t, z + h / 2 - t / 2):
        out.append(([lo[0], lo[1], zz], [hi[0], hi[1], zz + t]))
    return out

spec_path = Path(sys.argv[1]); spec = yaml.safe_load(open(spec_path))
out = R / spec["id"].lower() / f"{spec['id']}.usd"; out.parent.mkdir(parents=True, exist_ok=True)
stage = Usd.Stage.CreateNew(str(out))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, f"/{spec['id']}_{spec['name']}"); stage.SetDefaultPrim(root.GetPrim())
root.AddTranslateOp().Set(Gf.Vec3d(*spec["origin"])); root.AddRotateZOp().Set(spec.get("yaw_deg", 0))
p = root.GetPrim(); p.AddAppliedSchema("SemanticsLabelsAPI:class")
p.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set([spec.get("semantic", "building")])

def solid(grp, name, gprim, mat):
    gprim.CreateDisplayColorAttr([MATS[mat]]); UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())

n = 0
for part in spec["parts"]:
    (kind, d), = part.items()
    d = {**d, "name": Tf.MakeValidIdentifier(d["name"])}          # prim names: no '-' or '.'
    mat = d.get("mat", "concrete")
    if kind in ("beam", "cyl", "sphere"):
        x = UsdGeom.Xform.Define(stage, root.GetPath().AppendChild(d["name"]))
        if kind == "beam":
            a, b = np.array(d["from"], float), np.array(d["to"], float); L = np.linalg.norm(b - a)
            rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*((b - a) / L)))
            x.AddTranslateOp().Set(Gf.Vec3d(*((a + b) / 2))); x.AddOrientOp().Set(Gf.Quatf(rot.GetQuat()))
            x.AddScaleOp().Set(Gf.Vec3f(L / 2, d.get("t2", d.get("t", 0.2)) / 2, d.get("t", 0.2) / 2))
            g = UsdGeom.Cube.Define(stage, x.GetPath().AppendChild("c"))
        elif kind == "cyl":
            x.AddTranslateOp().Set(Gf.Vec3d(*d["c"]))
            g = UsdGeom.Cylinder.Define(stage, x.GetPath().AppendChild("c")); g.CreateRadiusAttr(d["r"]); g.CreateHeightAttr(d["h"]); g.CreateAxisAttr(d.get("axis", "Z"))
        else:
            x.AddTranslateOp().Set(Gf.Vec3d(*d["c"]))
            g = UsdGeom.Sphere.Define(stage, x.GetPath().AppendChild("c")); g.CreateRadiusAttr(d["r"])
        solid(x, d["name"], g, mat); n += 1
        continue
    boxes = {"box": lambda d: [(d["min"], d["max"])], "wall": wall_boxes, "stair": stair_boxes, "rail": rail_boxes}[kind](d)
    grp = UsdGeom.Xform.Define(stage, root.GetPath().AppendChild(d["name"]))
    for i, (lo, hi) in enumerate(boxes):
        lo, hi = np.array(lo, float), np.array(hi, float)
        x = UsdGeom.Xform.Define(stage, grp.GetPath().AppendChild(f"b{i}"))
        x.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2))); x.AddScaleOp().Set(Gf.Vec3f(*((hi - lo) / 2)))
        c = UsdGeom.Cube.Define(stage, x.GetPath().AppendChild("c"))
        c.CreateDisplayColorAttr([MATS[d.get("mat", "concrete")]])
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        n += 1
stage.Save()
print(f"{spec['id']}: {len(spec['parts'])} parts, {n} boxes -> {out}")
