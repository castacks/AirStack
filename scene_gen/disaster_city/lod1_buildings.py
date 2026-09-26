"""LOD1 box buildings measured off the Google tile mesh.

    ~/.venvs/recon/bin/python lod1_buildings.py

For every `kind: building` in LABELS.yaml: ray-cast a 0.25 m heightmap of
data/recon/tiles_site.npz, take the blob of cells >= MIN_H above local ground that
contains the label point, fit a minimum-area rectangle -> footprint, height =
90th percentile of the blob. Writes
  data/recon/buildings_lod1.yaml   id, centre, size, yaw, ground z, height
  data/recon/buildings_lod1.usd    per building: the tile outline extruded -- main roof level plus
                                   any attached lower annex -- ortho-textured roofs, walls, colliders
  data/recon/buildings_lod1.jpg    footprints drawn on the ortho, for checking
Trees touching a building get merged into its blob -- check the jpg.
"""
import json
from pathlib import Path
import cv2, numpy as np, open3d as o3d, yaml, mapbox_earcut
from shapely.geometry import Polygon
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

from _paths import CODE, DATA, R, LABELS
RES, WIN, MIN_H = 0.25, 30.0, 2.0          # m/cell, half window around a label, blob threshold
ANNEX_H, ANNEX_A = 2.0, 0.0                # an attached lower wing / lean-to (a lower threshold swallows parked cars and bus rows)

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
    notgreen = exg[v, u] < 12
    blob = (((H - ground) >= MIN_H) & level & notgreen & mine).astype(np.uint8)
    blob = cv2.morphologyEx(blob, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))   # cut thin tree/wire links
    n, cc = cv2.connectedComponents(blob)
    c0 = int(WIN / RES)
    ids = cc[c0 - 8:c0 + 8, c0 - 8:c0 + 8]; ids = ids[ids > 0]                 # blob under the label (+-2 m)
    if len(ids):
        m = (cc == np.bincount(ids).argmax()).astype(np.uint8)
    else:                       # a tile house with its walls but no roof (the ray falls to the floor): its walls' convex hull
        walls = (((H - ground) >= MIN_H) & notgreen & mine).astype(np.uint8)
        nw, cw = cv2.connectedComponents(cv2.dilate(walls, np.ones((9, 9), np.uint8)))
        d = np.where(cw > 0, np.hypot(X - cx, Y - cy), np.inf)
        if not np.isfinite(d.min()) or d.min() > 4: print(f"{l['id']}: nothing >= {MIN_H} m at the label"); continue
        k = (cw == cw.flat[d.argmin()]) & (walls > 0)
        m = cv2.fillConvexPoly(np.zeros_like(walls), cv2.convexHull(cv2.findNonZero(k.astype(np.uint8))), 1)
        print(f"{l['id']}: roofless in the tiles -- its walls' hull ({m.sum() * RES * RES:.0f} m2)")
        H = np.where(m > 0, np.percentile(H[k], 80), H)
    (u, v), (w, h), ang = cv2.minAreaRect(cv2.findNonZero(m))
    x, y = cx - WIN + (u + 0.5) * RES, cy + WIN - (v + 0.5) * RES
    rec = {"id": l["id"], "name": l["name"], "at": [round(x, 2), round(y, 2)],
           "size_m": [round(w * RES, 2), round(h * RES, 2)], "yaw_deg": round(-ang, 1),
           "ground_z": round(float(ground), 2), "height_m": round(float(np.percentile(H[m > 0], 90) - ground), 2)}
    # the building's real outline, as extrudable polygons: the main roof level, plus any lower
    # annex (porch, lean-to, wing) that is attached to it. The rectangle above stays for the tools
    # that want a frame (measure_sheets, recon_mesh); the USD is built from these.
    to_w = lambda cnt: [[round(float(cx - WIN + (px + 0.5) * RES), 2), round(float(cy + WIN - (py + 0.5) * RES), 2)] for px, py in cnt]
    def outline(mask):
        cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        polys_ = []
        for cn in cs:
            if cv2.contourArea(cn) * RES * RES < 3: continue
            pg = Polygon(to_w(cn[:, 0])).buffer(RES / 2).simplify(0.35)   # +half a cell: contours run on cell centres
            if pg.is_valid and pg.area > 3: polys_.append(pg)
        return polys_
    levels = [(outline(m), rec["height_m"])]
    annex = (((H - ground) >= l.get("annex_h", ANNEX_H)) & ~level & (H < np.median(roof)) & notgreen & mine).astype(np.uint8)
    annex = cv2.morphologyEx(annex, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))           # lighter: keeps low wings and thick walls
    na, ca = cv2.connectedComponents(annex)
    touching = [k for k in range(1, na) if (cv2.dilate(m, np.ones((5, 5), np.uint8)).astype(bool) & (ca == k)).any() and (ca == k).sum() * RES * RES >= ANNEX_A]
    if touching:
        am = np.isin(ca, touching)
        levels.append((outline(am), round(float(np.percentile(H[am], 80) - ground), 2)))
    rec["levels"] = [{"height_m": hgt, "rings": [[list(pt) for pt in pg.exterior.coords[:-1]] for pg in pgs]} for pgs, hgt in levels if pgs]
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

# walls: a tileable texture off the drone video (materials/, video_materials.py) tinted to each building's
# wall colour in the tiles -- its two longest walls rendered head-on from 12 m (render_views.py), median
# of the middle; grey walls take the concrete texture, coloured ones the stucco
def wall_tints():
    import subprocess, tempfile
    views = []
    for b in out:
        xy = np.array(max(b["levels"][0]["rings"], key=len), float)
        if not Polygon(xy).exterior.is_ccw: xy = xy[::-1]
        e = np.roll(xy, -1, 0) - xy; L = np.linalg.norm(e, axis=1)
        for j in np.argsort(-L)[:2]:
            t = e[j] / L[j]; n = np.array([t[1], -t[0], 0.0]); fwd = -n
            right = np.cross(fwd, [0, 0, 1]); down = np.cross(fwd, right)
            C = np.r_[(xy[j] + xy[(j + 1) % len(xy)]) / 2, b["ground_z"] + b["levels"][0]["height_m"] / 2] + 12 * n
            views.append({"name": f"{b['id']}_{j}", "R_wc": np.c_[right, down, fwd].tolist(), "C": C.tolist(),
                          "fx": 300, "fy": 300, "cx": 160, "cy": 120, "w": 320, "h": 240})
    with tempfile.TemporaryDirectory() as d:
        json.dump(views, open(f"{d}/views.json", "w"))
        subprocess.run(["blender", "-b", str(DATA / "blender_data/disaster_city.blend"), "--python", str(CODE / "render_views.py"), "--",
                        f"{d}/views.json", d], check=True, capture_output=True)
        tint = {}
        for v in views:
            im = cv2.imread(f"{d}/{v['name']}.png")[80:200, 100:220].reshape(-1, 3)[:, ::-1] / 255
            tint.setdefault(v["name"].split("_")[0], []).append(np.median(im, axis=0))
    return {k: np.median(v, axis=0).round(3).tolist() for k, v in tint.items()}
def lift(c):                        # the tiles bake shade into walls: brighten to a plausible albedo, keep the hue
    c = np.array(c); lum = c.mean(); return (c * np.clip(lum * 1.5, 0.45, 0.78) / max(lum, 1e-3)).round(3).tolist()
TINT = {k: lift(v) for k, v in wall_tints().items()}; LIB = json.load(open(R / "materials/materials.json"))
_wm = {}
def wall_mat(rgb):
    """stucco or concrete (by saturation), its texture scaled to the building's tint; one material per tint"""
    rgb = np.array(rgb); name = "stucco" if rgb.max() - rgb.min() > 0.06 else "concrete"
    key = f"wall_{name}_{'_'.join(str(int(c * 255)) for c in rgb)}"
    if key not in _wm:
        texmean = cv2.imread(str(R / "materials" / f"{name}.png")).reshape(-1, 3).mean(0)[::-1] / 255
        m = UsdShade.Material.Define(stage, f"/buildings_lod1/{key}")
        pbr = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("pbr")); pbr.CreateIdAttr("UsdPreviewSurface")
        pbr.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
        r = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("uv")); r.CreateIdAttr("UsdPrimvarReader_float2")
        r.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
        xf = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("tile")); xf.CreateIdAttr("UsdTransform2d")
        xf.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(r.ConnectableAPI(), "result")
        xf.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*[1 / LIB[name]["tile_m"]] * 2))
        t = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("tex")); t.CreateIdAttr("UsdUVTexture")
        t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./materials/{name}.png")
        t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(xf.ConnectableAPI(), "result")
        for w in ("wrapS", "wrapT"): t.CreateInput(w, Sdf.ValueTypeNames.Token).Set("repeat")
        t.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
        t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*(np.clip(rgb / texmean, 0, 2)), 1))
        pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(t.ConnectableAPI(), "rgb")
        m.CreateSurfaceOutput().ConnectToSource(pbr.ConnectableAPI(), "surface"); _wm[key] = m
    return _wm[key]

def mesh(path, V, faces):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float, v)) for v in V])
    m.CreateFaceVertexCountsAttr([len(f) for f in faces]); m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    UsdPhysics.CollisionAPI.Apply(m.GetPrim())
    return m

for b in out:
    x = UsdGeom.Xform.Define(stage, f"/buildings_lod1/{b['id']}_{b['name']}")
    k = 0
    for lv in b["levels"]:
        z0, z1 = b["ground_z"] - 0.5, b["ground_z"] + lv["height_m"]
        for ring in lv["rings"]:
            xy = np.array(ring, float)
            if not Polygon(xy).exterior.is_ccw: xy = xy[::-1]                 # CCW from above: outward walls, upward roof
            n_ = len(xy)
            tri = mapbox_earcut.triangulate_float64(xy, np.array([n_], np.uint32)).reshape(-1, 3)
            roof = mesh(x.GetPath().AppendChild(f"roof{k}"), [(*p, z1) for p in xy], tri.tolist())
            st = [((p[0] - geo["x0"]) / OW, 1 - (geo["y1"] - p[1]) / OH) for p in xy]
            UsdGeom.PrimvarsAPI(roof).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(st)
            UsdShade.MaterialBindingAPI.Apply(roof.GetPrim()).Bind(mat)
            walls = mesh(x.GetPath().AppendChild(f"walls{k}"), [(*p, z0) for p in xy] + [(*p, z1) for p in xy],
                         [[i, (i + 1) % n_, n_ + (i + 1) % n_, n_ + i] for i in range(n_)])
            per = np.r_[0, np.cumsum(np.linalg.norm(np.roll(xy, -1, 0) - xy, axis=1))]           # metres along the outline
            UsdGeom.PrimvarsAPI(walls).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(
                [c for i in range(n_) for c in ((per[i], z0), (per[i + 1], z0), (per[i + 1], z1), (per[i], z1))])
            tint = TINT.get(b["id"], [0.72, 0.70, 0.66])
            walls.CreateDisplayColorAttr([tuple(tint)]); UsdShade.MaterialBindingAPI.Apply(walls.GetPrim()).Bind(wall_mat(tint)); k += 1
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
