"""Site base layer from the Google tiles: bare earth + classified raised surface -> USD.

    ~/.venvs/recon/bin/python ground.py

Inputs: data/recon/tiles_site.npz (tile mesh), data/recon/ortho_site.{png,json} (12.5 cm
ortho of it), data/recon/osm.npz (osm_export.py), data/recon/buildings_lod1.yaml,
LABELS.yaml, and the hero list in assemble_scene.py.

1. DSM at RES from the tile mesh; bare earth (DTM) = cells within 0.7 m of a
   51 m morphological opening, the rest filled by normalised Gaussian smoothing.
2. Terrain cells -> three textured meshes (UV = world XY into the ortho):
     road   asphalt-coloured areas >= 150 m2, 3 m off the OSM roads (parking); OSM roads/paths are a
            separate mesh (their own polygons, subdivided to 2 m, draped 5 cm over
            the terrain) so kerbs are smooth instead of 1 m steps
     water  OSM water
     ground the rest
3. Tile triangles standing > RAISED m above the DTM, outside every LOD1 box and
   hero footprint, are kept as vertex-coloured meshes by class: `vegetation` if
   >= 20% of the raised cells within ~3 m are green (shadowed canopy isn't green
   itself) -- or the DSM is rough (> 0.8 m std over 3 m; bare winter trees are
   grey) with no label within 15 m -- and no labelled vehicle/structure/rubble is within 8 m, otherwise the kind of the nearest LABELS feature within 15 m (rubble,
   vehicle, structure->building), else `clutter`.
Writes data/recon/site_ground.usd -- one Mesh per class under /site, each with a class
label and a static collider. Everything is textured by top-down projection of the ortho.
"""
import json
from pathlib import Path
import cv2, numpy as np, open3d as o3d, yaml
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Vt, Gf

from _paths import CODE, R, LABELS
RES, RAISED = 1.0, 0.8
geo = json.load(open(R / "ortho_site.json")); ortho = cv2.imread(str(R / "ortho_site.png"))
OX0, OY1, OM = geo["x0"], geo["y1"], geo["m_per_px"]
X0, Y1 = OX0, OY1; N = int(ortho.shape[1] * OM / RES)                    # grid = the ortho window
xs = X0 + (np.arange(N) + 0.5) * RES; ys = Y1 - (np.arange(N) + 0.5) * RES
X, Y = np.meshgrid(xs, ys)

t = np.load(R / "tiles_site.npz"); TV, TF = t["verts"].astype(np.float32), t["faces"].astype(np.int64)
scene = o3d.t.geometry.RaycastingScene(); scene.add_triangles(o3d.core.Tensor(TV), o3d.core.Tensor(TF.astype(np.uint32)))
rays = np.stack([X, Y, np.full_like(X, 500), 0 * X, 0 * X, -np.ones_like(X)], -1).astype(np.float32)
DSM = 500 - scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
valid = np.isfinite(DSM); DSM[~valid] = np.nanmedian(DSM[valid])

# 1. bare earth
k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (51, 51))
opened = cv2.dilate(cv2.erode(DSM.astype(np.float32), k), k)
bare = (DSM - opened < 0.7) & valid
w = cv2.GaussianBlur(bare.astype(np.float32), (0, 0), 6); v = cv2.GaussianBlur(np.where(bare, DSM, 0).astype(np.float32), (0, 0), 6)
DTM = np.where(bare, DSM, v / np.maximum(w, 1e-6))
print(f"bare earth: {bare.mean():.0%} of cells measured, rest filled")
np.savez(R / "dtm.npz", dtm=DTM.astype(np.float32), x0=X0, y1=Y1, res=RES)   # recon_mesh.py tapers pile edges onto it

# ortho colour per cell (for classification)
ou = np.clip(((X - OX0) / OM).astype(int), 0, ortho.shape[1] - 1); ov = np.clip(((OY1 - Y) / OM).astype(int), 0, ortho.shape[0] - 1)
small = cv2.resize(ortho, (N, N), interpolation=cv2.INTER_AREA)
hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV).astype(int); b_, g_, r_ = [small[..., i].astype(int) for i in range(3)]
exg = 2 * g_ - r_ - b_

def raster(tris):
    m = np.zeros((N, N), np.uint8)
    for tri in tris:
        p = np.stack([(tri[:, 0] - X0) / RES, (Y1 - tri[:, 1]) / RES], -1)
        cv2.fillConvexPoly(m, np.round(p * 8).astype(np.int32), 1, shift=3)
    return m.astype(bool)

osm = np.load(R / "osm.npz")
osm_road = raster(np.concatenate([osm[k] for k in osm.files if k.startswith(("road_", "path_"))]))
water = raster(osm["water"])

# 2. terrain classes
# asphalt here is bluish grey (hue ~115, sat ~50, val 95-125), not neutral
asphalt = ((hsv[..., 1] < 45) | ((hsv[..., 0] >= 100) & (hsv[..., 0] <= 130) & (hsv[..., 1] < 85))) & (hsv[..., 2] < 140) & (hsv[..., 2] > 40) & bare
# raster road = parking lots only: big asphalt areas kept 3 m clear of the OSM roads, so the
# draped road mesh alone defines the kerb line (the raster would add back 1 m stair-steps)
off_road = asphalt & ~cv2.dilate(osm_road.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
n, cc, st, _ = cv2.connectedComponentsWithStats(cv2.morphologyEx(off_road.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)))
paved = np.isin(cc, np.flatnonzero(st[:, cv2.CC_STAT_AREA] * RES * RES >= 150)[1:])
water &= exg < 5                                            # OSM's pond polygon spills onto meadow
road = paved & ~water

labels = yaml.safe_load(open(LABELS))["labels"]
lab_at = {l["id"]: l for l in labels}
heroes = {}                                               # id -> footprint mask
import importlib.util
spec = importlib.util.spec_from_file_location("a", CODE / "assemble_scene.py"); src = open(spec.origin).read()
PIECES = eval(src.split("PIECES = ")[1].split("\n")[0])
hero_mask = np.zeros((N, N), bool)
for pid in PIECES:
    if pid not in lab_at: continue
    l = lab_at[pid]; r = l.get("size_m", 20) / 2 + (2 if l["kind"] == "rubble" else 0)
    if pid.startswith("R"): hero_mask |= np.hypot(X - l["at"][0], Y - l["at"][1]) < r - 0.5   # pile discs carry their own ground
# SWALLOW, don't clip: a model replaces the whole raised tile blob it stands on, not just the
# tile triangles inside its own box -- the leftovers from box-only masking were the "floating
# mesh pieces" around the pad objects and vehicles. The blob is clipped to `reach` m around
# the model, because tile blobs run on into the trees and piles a building touches.
raised_cells = cv2.morphologyEx(((DSM - DTM) > 0.6).astype(np.uint8), cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
_ncc, raised_cc = cv2.connectedComponents(raised_cells)
def swallow(m, reach):
    m = m.astype(bool)
    touch = np.unique(raised_cc[cv2.dilate(m.astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)])
    near = cv2.dilate(m.astype(np.uint8), cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * int(reach / RES) + 1,) * 2)).astype(bool)
    return cv2.dilate((m | (np.isin(raised_cc, touch[touch > 0]) & near)).astype(np.uint8), np.ones((3, 3), np.uint8)).astype(bool)

box_mask = np.zeros((N, N), bool)
boxes = yaml.safe_load(open(R / "buildings_lod1.yaml"))["buildings"]
def in_box(px, py, b, pad):
    th = np.radians(b["yaw_deg"]); dx, dy = px - b["at"][0], py - b["at"][1]
    lu, lv = dx * np.cos(th) + dy * np.sin(th), -dx * np.sin(th) + dy * np.cos(th)
    return (abs(lu) < b["size_m"][0] / 2 + pad) & (abs(lv) < b["size_m"][1] / 2 + pad)
for b in boxes:                                              # the extruded tile outline (lod1_buildings.py levels)
    if b["id"] in PIECES: continue                           # its hero model masks below
    m = np.zeros((N, N), np.uint8)
    for lv in b.get("levels", []):
        for ring in lv["rings"]:
            ring = np.array(ring); cv2.fillPoly(m, [np.round(np.c_[(ring[:, 0] - X0) / RES, (Y1 - ring[:, 1]) / RES]).astype(np.int32)], 1)
    box_mask |= swallow(m if m.any() else in_box(X, Y, b, 0.5), 2.0)
# hand-built models: mask their world bound (oriented box from the built USD) so the
# tile mesh of the same thing is not kept alongside it
for pid, rel in PIECES.items():
    if pid.startswith("R") or not (R / rel).exists(): continue
    hs = Usd.Stage.Open(str(R / rel))                      # keep a reference: the prim dies with its stage
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    m = np.zeros((N, N), np.uint8)
    for part in hs.GetDefaultPrim().GetChildren():         # per part, so a spread-out rig masks only its members
        bb = cache.ComputeWorldBound(part)
        if bb.GetRange().IsEmpty(): continue
        rng, M = bb.GetRange(), bb.GetMatrix(); lo_, hi_ = rng.GetMin(), rng.GetMax()
        poly = np.array([[*M.Transform(Gf.Vec3d(x, y, lo_[2]))][:2] for x, y in ((lo_[0], lo_[1]), (hi_[0], lo_[1]), (hi_[0], hi_[1]), (lo_[0], hi_[1]))])
        cv2.fillConvexPoly(m, np.round(np.c_[(poly[:, 0] - X0) / RES, (Y1 - poly[:, 1]) / RES]).astype(np.int32), 1)
    sw = swallow(m, 4.0); box_mask |= sw
    print(f"  {pid}: masked {sw.sum() * RES * RES:.0f} m2 of tile surface ({m.sum() * RES * RES:.0f} m2 model footprint)")

replaced = np.zeros((N, N), bool)                          # vehicle blobs place_assets.py swapped for assets --
if (R / "replaced.npz").exists():                           # cut from the USD only, NOT from the rasters it reads back
    for poly in np.load(R / "replaced.npz")["polys"]:
        m = np.zeros((N, N), np.uint8)
        cv2.fillConvexPoly(m, np.round(np.c_[(poly[:, 0] - X0) / RES, (Y1 - poly[:, 1]) / RES]).astype(np.int32), 1)
        replaced |= swallow(m, 1.5)

cls = np.full((N, N), "ground", object); cls[road] = "road"; cls[water] = "water"
print(f"terrain: road {road.mean():.1%}, water {water.mean():.1%}")

# 3. raised tile triangles
cen = TV[TF].mean(1)
ci = np.clip(((cen[:, 0] - X0) / RES).astype(int), 0, N - 1); cj = np.clip(((Y1 - cen[:, 1]) / RES).astype(int), 0, N - 1)
up = cen[:, 2] - DTM[cj, ci] > RAISED
inside = (cen[:, 0] > X0) & (cen[:, 0] < X0 + N * RES) & (cen[:, 1] < Y1) & (cen[:, 1] > Y1 - N * RES)
keep = up & inside & ~hero_mask[cj, ci] & ~box_mask[cj, ci]
# also drop tile triangles in hero ground discs even if low -- the hero covers them
# vegetation by neighbourhood: shadowed canopy is not green itself, but sits among green
raised_c = (DSM - DTM > RAISED).astype(np.float32)
gf = cv2.GaussianBlur(raised_c * (exg > 8), (0, 0), 3 / RES) / np.maximum(cv2.GaussianBlur(raised_c, (0, 0), 3 / RES), 1e-3)
pts = np.array([l["at"] for l in labels if l["kind"] in ("rubble", "vehicle", "structure", "building")])
kinds = [l["kind"] for l in labels if l["kind"] in ("rubble", "vehicle", "structure", "building")]
d = np.hypot(cen[:, None, 0] - pts[None, :, 0], cen[:, None, 1] - pts[None, :, 1])
near = d.argmin(1); dmin = d.min(1)
KIND = {"rubble": "rubble", "vehicle": "vehicle", "structure": "building", "building": "building"}
labelled = np.array([KIND[kinds[i]] for i in near])
# bare winter trees are grey, not green -- but canopy is ROUGH where roofs and vehicles are smooth
rough = np.sqrt(np.maximum(cv2.blur(DSM.astype(np.float32) ** 2, (3, 3)) - cv2.blur(DSM.astype(np.float32), (3, 3)) ** 2, 0))
veg = ((gf[cj, ci] > 0.2) | ((rough[cj, ci] > 0.8) & (dmin > 15))) & ~((dmin < 8) & (labelled != "building"))
tri_cls = np.where(veg, "vegetation", np.where(dmin < 15, labelled, "clutter"))
# unlabelled shards within 3 m of canopy are canopy fragments (dark trunks, shadowed undersides)
vr = np.zeros((N, N), np.uint8); vr[cj[keep & veg], ci[keep & veg]] = 1
near_veg = cv2.dilate(vr, np.ones((7, 7), np.uint8)).astype(bool)
tri_cls = np.where((tri_cls == "clutter") & near_veg[cj, ci], "vegetation", tri_cls)

# rasters for place_assets.py: surface, bare earth, and which cells carry each raised class
cls_r = np.zeros((N, N), np.uint8)
for code, name in enumerate(("vegetation", "rubble", "vehicle", "building", "clutter"), 1):
    sel = keep & (tri_cls == name); cls_r[cj[sel], ci[sel]] = code
np.savez(R / "site_rasters.npz", dsm=DSM.astype(np.float32), dtm=DTM.astype(np.float32), cls=cls_r, road=road, water=water,
         osm_road=osm_road, occupied=box_mask | hero_mask, veg=cv2.dilate(vr, np.ones((3, 3), np.uint8)).astype(bool),
         x0=X0, y1=Y1, res=RES, cls_names=np.array(["none", "vegetation", "rubble", "vehicle", "building", "clutter"]))

# ---------------- USD ----------------
stage = Usd.Stage.CreateNew(str(R / "site_ground.usd"))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/site"); stage.SetDefaultPrim(root.GetPrim())
mat = UsdShade.Material.Define(stage, "/site/ortho_mat")
sh = UsdShade.Shader.Define(stage, "/site/ortho_mat/pbr"); sh.CreateIdAttr("UsdPreviewSurface")
sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
tex = UsdShade.Shader.Define(stage, "/site/ortho_mat/tex"); tex.CreateIdAttr("UsdUVTexture")
tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("./ortho_site.png")
rd = UsdShade.Shader.Define(stage, "/site/ortho_mat/st"); rd.CreateIdAttr("UsdPrimvarReader_float2")
rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rd.ConnectableAPI(), "result")
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")

def emit(name, V, F, label, colors=None, textured=False):
    m = UsdGeom.Mesh.Define(stage, f"/site/{name}")
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V.astype(np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(F), 3, np.int32)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.astype(np.int32).ravel()))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.stack([V.min(0), V.max(0)]).astype(np.float32)))
    if textured:
        st = np.c_[(V[:, 0] - OX0) / (ortho.shape[1] * OM), 1 - (OY1 - V[:, 1]) / (ortho.shape[0] * OM)]
        UsdGeom.PrimvarsAPI(m).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(st.astype(np.float32)))
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    if colors is not None:
        m.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray.FromNumpy(colors.astype(np.float32)))
    UsdPhysics.CollisionAPI.Apply(m.GetPrim()); UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("none")
    p = m.GetPrim(); p.AddAppliedSchema("SemanticsLabelsAPI:class")
    p.CreateAttribute("semantics:labels:class", Sdf.ValueTypeNames.TokenArray).Set([label])
    print(f"  {name}: {len(F)} tris ({label})")

GV = np.stack([X, Y, DTM], -1).reshape(-1, 3); idx = np.arange(N * N).reshape(N, N)
for name in ("ground", "road", "water"):
    c = cls == name                                       # piles sit on top of the ground
    q = c[:-1, :-1]                                        # quad owned by its top-left cell
    a0, a1, a2, a3 = idx[:-1, :-1][q], idx[:-1, 1:][q], idx[1:, :-1][q], idx[1:, 1:][q]
    F = np.concatenate([np.stack([a0, a2, a1], 1), np.stack([a1, a2, a3], 1)])
    used = np.unique(F); rm = -np.ones(N * N, int); rm[used] = np.arange(len(used))
    emit(name, GV[used], rm[F], name, textured=True)

# OSM roads as their real polygons (smooth kerbs), subdivided to <= 2 m and draped 5 cm over the terrain
tri = np.concatenate([osm[k] for k in osm.files if k.startswith(("road_", "path_"))]).astype(np.float64)
# adaptive longest-edge bisection (uniform midpoint subdivision blew up to 13M tris on the slivers);
# the T-junctions it leaves are sub-centimetre gaps on near-flat roads
V2, T2 = tri.reshape(-1, 2), np.arange(len(tri) * 3).reshape(-1, 3)
while True:
    P3 = V2[T2]; e = np.linalg.norm(P3 - np.roll(P3, -1, 1), axis=2)          # edge k: vertex k -> k+1
    long_ = e.max(1) > 2.0
    if not long_.any(): break
    k = e[long_].argmax(1); t = T2[long_]
    a, b, c = t[np.arange(len(t)), k], t[np.arange(len(t)), (k + 1) % 3], t[np.arange(len(t)), (k + 2) % 3]
    mid = len(V2) + np.arange(len(t)); V2 = np.r_[V2, (V2[a] + V2[b]) / 2]
    T2 = np.r_[T2[~long_], np.c_[a, mid, c], np.c_[mid, b, c]]
rm_ = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.c_[V2, np.zeros(len(V2))]), o3d.utility.Vector3iVector(T2))
rm_.merge_close_vertices(0.01)
RV = np.asarray(rm_.vertices).copy()
inside_ = (RV[:, 0] > X0) & (RV[:, 0] < X0 + N * RES) & (RV[:, 1] < Y1) & (RV[:, 1] > Y1 - N * RES)
RV[:, 2] = DTM[np.clip(((Y1 - RV[:, 1]) / RES).astype(int), 0, N - 1), np.clip(((RV[:, 0] - X0) / RES).astype(int), 0, N - 1)] + 0.05
RF = np.asarray(rm_.triangles); RF = RF[inside_[RF].all(1)]
cr = lambda F_: (RV[F_[:, 1], 0] - RV[F_[:, 0], 0]) * (RV[F_[:, 2], 1] - RV[F_[:, 0], 1]) - (RV[F_[:, 1], 1] - RV[F_[:, 0], 1]) * (RV[F_[:, 2], 0] - RV[F_[:, 0], 0])
RF = RF[cr(RF) != 0]
flip = cr(RF) < 0                                            # face up
RF[flip] = RF[flip][:, ::-1]
emit("road_osm", RV, RF, "road", textured=True)

# CLEAN-UP of what is left: the tile mesh duplicates vertices along texture seams, so pieces are
# found on a WELDED copy (5 cm). Dropped: pieces hanging in the air (lowest point > 0.5 m above
# bare earth) with < 50 triangles -- the shards left where a model replaced a blob, or where a
# wall's lower triangles fell under the RAISED cut -- and tree-like pieces (mostly over canopy
# cells, > 5 m tall) that the class rules put in another class: the instanced trees stand there.
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
vegc = cv2.dilate(vr, np.ones((3, 3), np.uint8)).astype(bool)
def clean_pieces(F, name):
    _, weld = np.unique(np.round(TV / 0.05).astype(np.int64), axis=0, return_inverse=True); weld = weld.ravel()
    Fw = weld[F]; nv = int(Fw.max()) + 1; e = np.r_[Fw[:, [0, 1]], Fw[:, [1, 2]], Fw[:, [2, 0]]]
    _n, lab = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(nv, nv)), directed=False)
    flab = lab[Fw[:, 0]]; keep_f = np.ones(len(F), bool); shards = trees = 0
    for k in np.unique(flab):
        idx = flab == k; P = TV[np.unique(F[idx])]
        ii = np.clip(((Y1 - P[:, 1]) / RES).astype(int), 0, N - 1); jj = np.clip(((P[:, 0] - X0) / RES).astype(int), 0, N - 1)
        gap = (P[:, 2] - DTM[ii, jj]).min(); tall = np.ptp(P[:, 2])
        if gap > 0.5 and idx.sum() < 50: keep_f[idx] = False; shards += 1
        elif vegc[ii, jj].mean() > 0.45 and tall > 5: keep_f[idx] = False; trees += 1
    return F[keep_f], (shards, trees, int((~keep_f).sum()))
n_drop = {}

for name in ("vegetation", "rubble", "vehicle", "building", "clutter"):
    sel = keep & (tri_cls == name) & ~replaced[cj, ci]
    if not sel.any(): continue
    F = TF[sel]
    if name != "vegetation": F, dropped = clean_pieces(F, name); n_drop[name] = dropped
    used = np.unique(F); rm = -np.ones(len(TV), int); rm[used] = np.arange(len(used))
    V = TV[used]
    emit(f"tiles_{name}", V, rm[F], name if name != "clutter" else "other", textured=True)   # tops read right; walls smear
print("  removed:", ", ".join(f"{k} {v[0]} shards + {v[1]} tree pieces ({v[2]} tris)" for k, v in n_drop.items()))
stage.Save()
print(f"wrote {R / 'site_ground.usd'}")
