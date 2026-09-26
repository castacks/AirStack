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
  stair:  {name, from: [x,y,z], to: [x,y,z], width, mat, handrails}   straight run, treads on two stringers; banisters optional
  rail:   {name, from: [x,y], to: [x,y], z, h: 1.1, mat}   posts every 1.5 m + top and mid rails
          (a solid box would read as a wall to a drone's depth sensor and collider)
  beam:   {name, from: [x,y,z], to: [x,y,z], t: 0.2, mat}   oriented square bar (bracing, booms, tilted slabs with t2)
  cyl:    {name, c: [x,y,z] (centre), r, h, axis: X|Y|Z, mat}
  sphere: {name, c: [x,y,z], r, mat}
  hip:    {name, min: [x,y,z], max: [x,y,z], mat}   hip roof over the box's footprint, ridge along its long side

Optional top-level `default_mat` (else concrete) for parts without a `mat`.
Optional top-level `lights: [{rect: [x0,y0,x1,y1], ceiling: z, spacing: 5}]`
fills each rect with SphereLights 0.4 m under its ceiling. They go under
/<ID>/lights, a scope with no collider or label; deactivate it for a dark building.
The interior fill is there because real-time RTX has no bounce light into rooms.
"""
import json, sys
from pathlib import Path
import numpy as np, yaml
from pxr import Usd, UsdGeom, UsdLux, UsdShade, UsdPhysics, Sdf, Gf, Tf
from _paths import R

MATS = {"concrete": (0.62, 0.60, 0.56), "steel": (0.30, 0.31, 0.33), "grating": (0.22, 0.22, 0.24),
        "wood": (0.55, 0.42, 0.28), "rust": (0.45, 0.25, 0.15), "tank_black": (0.08, 0.08, 0.09), "tank_white": (0.85, 0.85, 0.83),
        "steel_dark": (0.20, 0.17, 0.15), "stucco": (0.72, 0.62, 0.48), "roof_brown": (0.40, 0.28, 0.20), "metal_white": (0.80, 0.80, 0.78),
        "metal_ribbed": (0.48, 0.48, 0.47), "metal_grey": (0.42, 0.42, 0.43), "panel_dark": (0.28, 0.29, 0.29), "yellow": (0.84, 0.77, 0.56), "sign": (0.92, 0.92, 0.90), "canvas": (0.80, 0.72, 0.58), "grating_open": (0.22, 0.22, 0.24)}

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

# Textures: data/recon/materials/<mat>.png, tileable, cut from the drone video by video_materials.py
# (materials.json: tile size in m, roughness, metallic). A mat without one stays a flat colour.
LIB = json.load(open(R / "materials/materials.json")) if (R / "materials/materials.json").exists() else {}
PHOTO = spec.get("photo")                 # {mats: [...]}: those parts also get an atlas baked from the video (photo_bake.py)
_mats = {}
def material(mat):
    """UsdPreviewSurface, rough (Isaac's default is glossy and interior lights glinted off every wall);
    textured through primvar `st` (metres) when the library has the mat, or the photo atlas (`st_photo`)."""
    if mat not in _mats:
        m = UsdShade.Material.Define(stage, root.GetPath().AppendChild("materials").AppendChild(mat))
        sh = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("pbr")); sh.CreateIdAttr("UsdPreviewSurface")
        lib = LIB.get(mat, {})
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(lib.get("rough", 0.6 if mat in ("steel", "grating") else 0.9))
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(lib.get("metal", 0.0))
        png = f"./{spec['id']}_photo.png" if mat == "photo" else (f"../materials/{mat}.png" if lib else None)
        if png:
            rd = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("uv")); rd.CreateIdAttr("UsdPrimvarReader_float2")
            rd.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st_photo" if mat == "photo" else "st")
            src = rd.ConnectableAPI()
            if mat != "photo":                                       # metres -> texture repeats
                xf = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("tile")); xf.CreateIdAttr("UsdTransform2d")
                xf.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(src, "result")
                xf.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(1 / lib["tile_m"], 1 / lib["tile_m"])); src = xf.ConnectableAPI()
            tx = UsdShade.Shader.Define(stage, m.GetPath().AppendChild("tex")); tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png)
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(src, "result")
            for w in ("wrapS", "wrapT"): tx.CreateInput(w, Sdf.ValueTypeNames.Token).Set("clamp" if mat == "photo" else "repeat")
            tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
            if lib.get("cutout"):                                      # alpha 0 in the holes: see-through grating
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(tx.ConnectableAPI(), "a")
                sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.5)
        else:
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*MATS.get(mat, MATS["concrete"])))
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface"); _mats[mat] = m
    return _mats[mat]

def solid(grp, name, gprim, mat):
    gprim.CreateDisplayColorAttr([MATS[mat]]); UsdPhysics.CollisionAPI.Apply(gprim.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(gprim.GetPrim()).Bind(material(mat))

# a box = 8 corners (local frame) -> 6 quads; st = the face's two in-plane axes in metres, taken
# in the building frame so the texture runs on across the boxes of one wall
FACES = [((0, 2, 6, 4), 0), ((1, 5, 7, 3), 0), ((0, 4, 5, 1), 1), ((2, 3, 7, 6), 1), ((0, 1, 3, 2), 2), ((4, 6, 7, 5), 2)]
def box_corners(lo, hi, M=None):
    c = np.array([[(lo, hi)[i >> 0 & 1][0], (lo, hi)[i >> 1 & 1][1], (lo, hi)[i >> 2 & 1][2]] for i in range(8)], float)
    return c if M is None else c @ M[:3, :3].T + M[:3, 3]

atlas, slots = [], {}                       # photo faces: world rectangles for photo_bake.py; per mesh, their atlas slots
def emit(path, corners_list, mat):
    """One Mesh (and collider) for all boxes of a part."""
    P, counts, idx, st, stp, photo = [], [], [], [], [], PHOTO and mat in PHOTO.get("mats", ["concrete"])
    for C in corners_list:
        for q, _ in FACES:
            Q = C[list(q)]; n = np.cross(Q[2] - Q[0], Q[3] - Q[1])                  # diagonals: also right for a triangle
            if np.linalg.norm(n) < 1e-9: continue                                  # collapsed face (a hip's ridge)
            n /= np.linalg.norm(n)
            if n @ (Q.mean(0) - C.mean(0)) < 0: Q, n = Q[::-1], -n          # outward, counter-clockwise
            # in-plane axes: vertical face -> (horizontal, z); flat face -> (x, y)
            ua = np.array([-n[1], n[0], 0.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0, 0])
            ua /= np.linalg.norm(ua); va = np.cross(n, ua)
            base = len(P); P += Q.tolist(); counts.append(4); idx += [base, base + 1, base + 2, base + 3]
            st += [(float(p @ ua), float(p @ va)) for p in Q]
            if photo:
                uv = np.array([(p @ ua, p @ va) for p in Q]); lo_ = uv.min(0); size = uv.max(0) - lo_
                atlas.append({"o": (ua * lo_[0] + va * lo_[1] + n * (Q[0] @ n)).tolist(), "u": ua.tolist(), "v": va.tolist(),
                              "n": n.tolist(), "size": size.tolist(), "path": str(path), "mat": mat, "st0": lo_.tolist()})
                stp += [(len(atlas) - 1, *((uv_ - lo_) / np.maximum(size, 1e-6))) for uv_ in uv]
    g = UsdGeom.Mesh.Define(stage, path)
    g.CreatePointsAttr(P); g.CreateFaceVertexCountsAttr(counts); g.CreateFaceVertexIndicesAttr(idx)
    g.CreateSubdivisionSchemeAttr("none")
    pv = UsdGeom.PrimvarsAPI(g)
    pv.CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(st)
    if photo: slots[path] = stp                                          # atlas slots, resolved after packing
    g.CreateDisplayColorAttr([MATS.get(mat, MATS["concrete"])])
    UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(material("photo" if photo else mat))
    UsdPhysics.CollisionAPI.Apply(g.GetPrim()); UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr("none")
    return g

n = 0
for part in spec["parts"]:
    (kind, d), = part.items()
    d = {**d, "name": Tf.MakeValidIdentifier(d["name"])}          # prim names: no '-' or '.'
    mat = d.get("mat", spec.get("default_mat", "concrete")); path = root.GetPath().AppendChild(d["name"])
    if kind in ("cyl", "sphere"):
        x = UsdGeom.Xform.Define(stage, path); x.AddTranslateOp().Set(Gf.Vec3d(*d["c"]))
        if kind == "cyl":
            g = UsdGeom.Cylinder.Define(stage, x.GetPath().AppendChild("c")); g.CreateRadiusAttr(d["r"]); g.CreateHeightAttr(d["h"]); g.CreateAxisAttr(d.get("axis", "Z"))
        else:
            g = UsdGeom.Sphere.Define(stage, x.GetPath().AppendChild("c")); g.CreateRadiusAttr(d["r"])
        solid(x, d["name"], g, mat); n += 1
        continue
    if kind == "beam":                                               # oriented box: x along from->to, t2 across, t through
        a, b = np.array(d["from"], float), np.array(d["to"], float); L = np.linalg.norm(b - a); ax = (b - a) / L
        side = np.cross([0, 0, 1], ax); side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-6 else np.array([0, 1.0, 0])
        M = np.eye(4); M[:3, :3] = np.c_[ax, side, np.cross(ax, side)]; M[:3, 3] = (a + b) / 2
        t, t2 = d.get("t", 0.2), d.get("t2", d.get("t", 0.2))
        corners = [box_corners([-L / 2, -t2 / 2, -t / 2], [L / 2, t2 / 2, t / 2], M)]
    elif kind == "hip":                                              # a box whose top face is pulled in to a ridge
        lo, hi = np.array(d["min"], float), np.array(d["max"], float); c = box_corners(lo, hi)
        ax = 0 if hi[0] - lo[0] >= hi[1] - lo[1] else 1; inset = (hi[1 - ax] - lo[1 - ax]) / 2
        for i in range(4, 8):
            c[i, 1 - ax] = (lo[1 - ax] + hi[1 - ax]) / 2
            c[i, ax] = lo[ax] + inset if c[i, ax] == lo[ax] else hi[ax] - inset
        corners = [c]
    else:
        boxes = {"box": lambda d: [(d["min"], d["max"])], "wall": wall_boxes, "stair": stair_boxes, "rail": rail_boxes}[kind](d)
        corners = [box_corners(np.array(lo, float), np.array(hi, float)) for lo, hi in boxes]
        if kind == "stair":                                          # the treads hang between two stringers, not in the air
            p0, p1 = np.array(d["from"], float), np.array(d["to"], float); u = p1 - p0; L = np.linalg.norm(u); ax = u / L
            side = np.cross([0, 0, 1], ax); side /= np.linalg.norm(side); up = np.cross(ax, side)
            for sgn in (-1, 1):
                M = np.eye(4); M[:3, :3] = np.c_[ax, side, up]
                M[:3, 3] = (p0 + p1) / 2 + side * sgn * (d["width"] / 2 + 0.03) - up * 0.12
                corners.append(box_corners([-L / 2 - 0.15, -0.03, -0.15], [L / 2 + 0.15, 0.03, 0.15], M))
            if d.get("handrails"):                                     # banisters both sides: posts every ~1.2 m, sloped top + mid rail
                hz = np.cross(side, [0, 0, 1.0]); hz = np.array([ax[0], ax[1], 0]); hz /= np.linalg.norm(hz)
                for sgn in (-1, 1):
                    off = side * sgn * (d["width"] / 2 + 0.06)
                    for k in range(int(np.ceil(L / 1.2)) + 1):
                        c0 = p0 + (p1 - p0) * min(k * 1.2 / L, 1.0) + off
                        corners.append(box_corners([c0[0] - 0.025, c0[1] - 0.025, c0[2]], [c0[0] + 0.025, c0[1] + 0.025, c0[2] + 0.95]))
                    for hgt in (0.95, 0.5):
                        M = np.eye(4); M[:3, :3] = np.c_[ax, side, up]; M[:3, 3] = (p0 + p1) / 2 + off + np.array([0, 0, hgt])
                        corners.append(box_corners([-L / 2, -0.025, -0.025], [L / 2, 0.025, 0.025], M))
    emit(path, corners, mat); n += len(corners)

# pack the photo faces into one atlas (shelf packing, texel size grown until it fits) and write the UVs
if atlas:
    SIDE, PAD = PHOTO.get("atlas_px", 4096), 3
    area = sum(a["size"][0] * a["size"][1] for a in atlas); ts = np.sqrt(area) / SIDE
    while True:
        x = y = row = 0; ok = True
        for a in sorted(range(len(atlas)), key=lambda i: -atlas[i]["size"][1]):
            w, h = (int(np.ceil(s / ts)) + 2 * PAD for s in atlas[a]["size"])
            if x + w > SIDE: x, y, row = 0, y + row, 0
            if y + h > SIDE or w > SIDE: ok = False; break
            atlas[a]["px"] = [x + PAD, y + PAD, w - 2 * PAD, h - 2 * PAD]; x += w; row = max(row, h)
        if ok: break
        ts *= 1.05
    for path, sl in slots.items():
        uvs = [((atlas[i]["px"][0] + u * atlas[i]["px"][2]) / SIDE, 1 - (atlas[i]["px"][1] + (1 - v) * atlas[i]["px"][3]) / SIDE)
               for i, u, v in sl]
        UsdGeom.PrimvarsAPI(stage.GetPrimAtPath(path)).CreatePrimvar("st_photo", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(uvs)
    M = np.array(UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T      # local -> world
    json.dump({"side": SIDE, "texel_m": ts, "local_to_world": M.tolist(), "faces": atlas},
              open(out.parent / f"{spec['id']}_atlas.json", "w"))
    print(f"  photo atlas: {len(atlas)} faces, {area:.0f} m2, {ts * 100:.1f} cm/texel")
LIGHT_INTENSITY = 15000.0     # nits-ish; tuned so a lit room reads like an overcast-day interior
if spec.get("lights"):
    UsdGeom.Scope.Define(stage, root.GetPath().AppendChild("lights"))
    k = 0
    for L in spec["lights"]:
        x0, y0, x1, y1 = L["rect"]; sp = L.get("spacing", 5.0)
        nx, ny = max(1, round((x1 - x0) / sp)), max(1, round((y1 - y0) / sp))
        for i in range(nx):
            for j in range(ny):
                lt = UsdLux.SphereLight.Define(stage, root.GetPath().AppendChild("lights").AppendChild(f"l{k:03d}")); k += 1
                lt.CreateRadiusAttr(0.15); lt.CreateIntensityAttr(L.get("intensity", LIGHT_INTENSITY))
                lt.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.88)); lt.CreateTreatAsPointAttr(False)
                UsdGeom.Xformable(lt).AddTranslateOp().Set(Gf.Vec3d(x0 + (i + 0.5) * (x1 - x0) / nx, y0 + (j + 0.5) * (y1 - y0) / ny, L["ceiling"] - 0.4))
    print(f"  {k} interior lights")
stage.Save()
print(f"{spec['id']}: {len(spec['parts'])} parts, {n} boxes -> {out}")
