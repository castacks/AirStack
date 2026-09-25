"""LOD1 box buildings measured off the Google tile mesh.

    ~/.venvs/recon/bin/python lod1_buildings.py

For every `kind: building` in LABELS.yaml: ray-cast a 0.25 m heightmap of
data/recon/tiles_site.npz, take the blob of cells >= MIN_H above local ground that
contains the label point, fit a minimum-area rectangle -> footprint, height =
90th percentile of the blob. Writes
  data/recon/buildings_lod1.yaml   id, centre, size, yaw, ground z, height
  data/recon/buildings_lod1.usd    per building: ortho-textured roof + wall mesh, colliders, `building` label
  data/recon/buildings_lod1.jpg    footprints drawn on the ortho, for checking
Trees touching a building get merged into its blob -- check the jpg.
"""
import json
from pathlib import Path
import cv2, numpy as np, open3d as o3d, yaml
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

from _paths import R, LABELS
RES, WIN, MIN_H = 0.25, 30.0, 2.0          # m/cell, half window around a label, blob threshold

t = np.load(R / "tiles_site.npz")
scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(o3d.core.Tensor(t["verts"].astype(np.float32)), o3d.core.Tensor(t["faces"].astype(np.uint32)))

def heightmap(cx, cy):
    g = np.arange(-WIN, WIN, RES) + RES / 2
    X, Y = np.meshgrid(cx + g, cy - g)                         # row 0 = north
    rays = np.stack([X, Y, np.full_like(X, 500), 0 * X, 0 * X, -np.ones_like(X)], -1).astype(np.float32)
    return 500 - scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()

geo = json.load(open(R / "ortho_site.json"))
ortho = cv2.imread(str(R / "ortho_site.png")).astype(np.int16)
exg = 2 * ortho[..., 1] - ortho[..., 0] - ortho[..., 2]           # excess green: canopy > 0, roofs ~ 0

def grid(cx, cy):
    g = np.arange(-WIN, WIN, RES) + RES / 2
    return np.meshgrid(cx + g, cy - g)

labels = yaml.safe_load(open(LABELS))["labels"]
bldg = np.array([l["at"] for l in labels if l["kind"] == "building"])
out = []
for l in labels:
    if l["kind"] != "building": continue
    cx, cy = l["at"]
    H = heightmap(cx, cy)
    X, Y = grid(cx, cy)
    ground = np.nanpercentile(H, 10)
    u = np.clip(((X - geo["x0"]) / geo["m_per_px"]).astype(int), 0, exg.shape[1] - 1)
    v = np.clip(((geo["y1"] - Y) / geo["m_per_px"]).astype(int), 0, exg.shape[0] - 1)
    others = bldg[np.hypot(*(bldg - [cx, cy]).T) > 0.5]
    mine = np.hypot(X - cx, Y - cy) <= np.min(np.hypot(X[..., None] - others[:, 0], Y[..., None] - others[:, 1]), -1)
    c0 = int(WIN / RES)
    roof = H[c0 - 6:c0 + 7, c0 - 6:c0 + 7]; roof = roof[roof - ground >= MIN_H]
    if not len(roof): print(f"{l['id']}: nothing >= {MIN_H} m at the label"); continue
    level = np.abs(H - np.median(roof)) < 2.5                 # roof band: drops canopy above and yard below
    blob = (((H - ground) >= MIN_H) & level & (exg[v, u] < 12) & mine).astype(np.uint8)
    blob = cv2.morphologyEx(blob, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))   # cut thin tree/wire links
    n, cc = cv2.connectedComponents(blob)
    c0 = int(WIN / RES)
    ids = cc[c0 - 8:c0 + 8, c0 - 8:c0 + 8]; ids = ids[ids > 0]                 # blob under the label (+-2 m)
    if not len(ids): print(f"{l['id']}: nothing >= {MIN_H} m at the label"); continue
    m = (cc == np.bincount(ids).argmax()).astype(np.uint8)
    (u, v), (w, h), ang = cv2.minAreaRect(cv2.findNonZero(m))
    x, y = cx - WIN + (u + 0.5) * RES, cy + WIN - (v + 0.5) * RES
    rec = {"id": l["id"], "name": l["name"], "at": [round(x, 2), round(y, 2)],
           "size_m": [round(w * RES, 2), round(h * RES, 2)], "yaw_deg": round(-ang, 1),
           "ground_z": round(float(ground), 2), "height_m": round(float(np.percentile(H[m > 0], 90) - ground), 2)}
    out.append(rec)
    print(rec)

yaml.safe_dump({"buildings": out}, open(R / "buildings_lod1.yaml", "w"), sort_keys=False, default_flow_style=None)

# USD: per building a roof mesh (UV = world XY into data/recon/ortho_site.png, so the
# roof shows the real roof) + a wall mesh (plain concrete), both colliders.
# The ground_z bottom is sunk 0.5 m so a sloping site never shows a gap.
OW, OH = [x * geo["m_per_px"] for x in ortho.shape[1::-1]]
stage = Usd.Stage.CreateNew(str(R / "buildings_lod1.usd"))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/buildings_lod1"); stage.SetDefaultPrim(root.GetPrim())
mat = UsdShade.Material.Define(stage, "/buildings_lod1/roof_mat")
sh = UsdShade.Shader.Define(stage, "/buildings_lod1/roof_mat/pbr"); sh.CreateIdAttr("UsdPreviewSurface")
tex = UsdShade.Shader.Define(stage, "/buildings_lod1/roof_mat/tex"); tex.CreateIdAttr("UsdUVTexture")
tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./ortho_site.png")
rd = UsdShade.Shader.Define(stage, "/buildings_lod1/roof_mat/st"); rd.CreateIdAttr("UsdPrimvarReader_float2")
rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

def mesh(path, V, faces):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in V])
    m.CreateFaceVertexCountsAttr([len(f) for f in faces]); m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    return m

for b in out:
    x = UsdGeom.Xform.Define(stage, f"/buildings_lod1/{b['id']}_{b['name']}")
    th = np.radians(b["yaw_deg"]); u = np.array([np.cos(th), np.sin(th)]); v = np.array([-np.sin(th), np.cos(th)])
    c = np.array(b["at"]); hx, hy = b["size_m"][0] / 2, b["size_m"][1] / 2
    xy = [c - u * hx - v * hy, c + u * hx - v * hy, c + u * hx + v * hy, c - u * hx + v * hy]      # CCW from above
    z0, z1 = b["ground_z"] - 0.5, b["ground_z"] + b["height_m"]
    top = [(*p, z1) for p in xy]; bot = [(*p, z0) for p in xy]
    roof = mesh(x.GetPath().AppendChild("roof"), top, [[0, 1, 2, 3]])
    st = [((p[0] - geo["x0"]) / OW, 1 - (geo["y1"] - p[1]) / OH) for p in xy]
    UsdGeom.PrimvarsAPI(roof).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(st)
    UsdShade.MaterialBindingAPI.Apply(roof.GetPrim()).Bind(mat)
    walls = mesh(x.GetPath().AppendChild("walls"), bot + top, [[i, (i + 1) % 4, 4 + (i + 1) % 4, 4 + i] for i in range(4)])
    walls.CreateDisplayColorAttr([(0.72, 0.70, 0.66)])
    p = x.GetPrim(); p.AddAppliedSchema("SemanticsLabelsAPI:class")
    p.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set(["building"])
    p.SetCustomDataByKey("label:id", b["id"])
stage.Save()

geo = json.load(open(R / "ortho_site.json"))
img = cv2.imread(str(R / "ortho_site.png"))
to_px = lambda x, y: ((x - geo["x0"]) / geo["m_per_px"], (geo["y1"] - y) / geo["m_per_px"])
for b in out:
    u, v = to_px(*b["at"])
    box = cv2.boxPoints(((u, v), (b["size_m"][0] / geo["m_per_px"], b["size_m"][1] / geo["m_per_px"]), -b["yaw_deg"]))
    cv2.polylines(img, [box.astype(np.int32)], True, (0, 0, 255), 4)
    cv2.putText(img, f"{b['id']} {b['height_m']:.1f}m", (int(u) - 60, int(v)), 0, 1.2, (0, 0, 0), 6)
    cv2.putText(img, f"{b['id']} {b['height_m']:.1f}m", (int(u) - 60, int(v)), 0, 1.2, (0, 255, 255), 2)
cv2.imwrite(str(R / "buildings_lod1.jpg"), cv2.resize(img, (2240, 2240)), [cv2.IMWRITE_JPEG_QUALITY, 90])
print(f"{len(out)} buildings")
