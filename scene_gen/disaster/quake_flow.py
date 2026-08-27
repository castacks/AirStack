"""quake_flow — take one URBAN kit building apart the way an earthquake does.

The earthquake counterpart of `damage_flow` (fire) and `wind_flow` (wind),
for the FAÇADE-KIT buildings `detail/urban_building.py` assembles rather than
the timber houses those two break. Everything that differs is structural
rather than cosmetic, so this is a separate module and not a flag on either:

1. **The building is hollow, and an earthquake exposes interiors.** A kit
   building is a stack of façade BANDS plus a flat roof — no floors, no
   columns, nothing behind the walls. A fire or a wind can ignore that
   because a timber house that fails goes to a heap. An earthquake peels a
   masonry wall off and leaves the floors behind it standing ("dollhouse"),
   drops a concrete frame onto its own slabs ("pancake"), and sheds a
   curtain wall's glass so the slab edges show through. So every damaged
   building is FITTED OUT before it is broken: `fit_interior` authors floor
   slabs, columns (for the concrete families) and a few partitions, in the
   building's own frame, and those become part of what fails.

2. **Gravity only, straight down, nothing consumed.** No `bias` (wind), no
   `consume` (fire). Material stays on the lot: the mound is the building.
   The one directional thing is OUT-OF-PLANE wall failure, which is a
   rotation about the wall's base — so fragments high on a peeling wall are
   given an outward initial velocity that grows with height
   (`settle.prepare(velocity_map=...)`), and the wall lands as a fan on the
   street. Everything else is a drop.

3. **Construction type decides the recipe, not the level alone.** Unreinforced
   masonry (stone 01, brownstone 03, brick 04, the storefront terrace, the
   church) fails by shedding walls and parapets; a reinforced-concrete frame
   (office 02, the 05 podium, civic) fails at a STOREY — soft ground floor,
   mid-storey, pancake; a glass curtain-wall tower loses its glass and almost
   never comes down. `FAMILY_TYPE` maps each kit family to one of those, and
   `LADDER[type][grade]` is the recipe list. Grades are EMS-98 (DG1..DG5).

4. **Materials are the building's own plus what is INSIDE it.** Façade
   fragments keep their cladding (`damage.bound_texture`, triplanar). But a
   broken wall shows its back — plaster — and a broken slab shows concrete
   and rebar, so a share of every fragment set takes an interior material,
   and the pile of a concrete building is GREY where a brick building's is
   RED. This is most of what tells the two apart from the air.

The module imports `pxr` only inside the stage-touching functions, like the
rest of `disaster/`, so the tables are host-checkable.
"""

import math
import random

import numpy as np

# ---------------------------------------------------------------------------
# Construction types, per kit family (urban_building.STYLES[...]["family"])
# ---------------------------------------------------------------------------
FAMILY_TYPE = {
    "01": "urm",        # stone apartment block — unreinforced masonry
    "02": "rc",         # office — reinforced-concrete frame
    "03": "urm",        # brownstone
    "04": "urm",        # brick commercial
    "05": "rc_glass",   # concrete podium + glass curtain-wall tower
    "dw_b": "urm",      # storefront terrace, brick above a shop front
    "civ": "rc",        # civic hall / offices
    "church": "urm",    # stone church
}

GRADES = ("DG0", "DG1", "DG2", "DG3", "DG4", "DG5")

# Per construction type, per grade: the recipes applied, in order. A recipe
# is a (name, kwargs) pair; `wreck_building` runs them in sequence on the
# same building so later ones see what earlier ones left standing.
#
# The numbers follow EMS-98's grade descriptions for vulnerability classes
# A-B (URM) and C-D (RC): DG2 = "moderate" (falling plaster, parapets,
# chimneys), DG3 = "substantial to heavy" (large cracks, roof tiles and
# gable parts fall, single elements collapse), DG4 = "very heavy" (serious
# failure of walls, partial structural failure of roofs and floors), DG5 =
# "destruction" (total or near-total collapse). Soft-storey is the RC DG4
# signature (Northridge 1994, Kahramanmaraş 2023); pancake is RC DG5 (Mexico
# City 1985/2017, Kobe 1995). Towers on steel/RC cores are a class of their
# own: glass, cladding and podium damage, and in the worst case a tilt from a
# failed foundation, but essentially never a collapse.
LADDER = {
    "urm": {
        "DG0": [],
        "DG1": [("facade_scars", {"frac": 0.10}), ("rooftop_fail", {"frac": 0.3}),
                ("glass_loss", {"frac": 0.08})],
        "DG2": [("parapet_fall", {"sides": 1, "frac": 0.5}),
                ("facade_scars", {"frac": 0.18}), ("rooftop_fail", {"frac": 0.5}),
                ("signage_fail", {}),
                ("glass_loss", {"frac": 0.25})],
        "DG3": [("parapet_fall", {"sides": 2, "frac": 0.8}),
                ("corner_fail", {"storeys": 2}),
                ("roof_hole", {"frac": 0.25}),
                ("facade_scars", {"frac": 0.22}), ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("glass_loss", {"frac": 0.45})],
        "DG4": [("out_of_plane", {"sides": 1, "from_storey": 1}),
                ("parapet_fall", {"sides": 3, "frac": 0.9}),
                ("roof_hole", {"frac": 0.35}), ("rooftop_fail", {"frac": 0.8}),
                ("facade_scars", {"frac": 0.18}),
                ("glass_loss", {"frac": 0.7})],
        "DG5": [("masonry_collapse", {})],
    },
    "rc": {
        "DG0": [],
        "DG1": [("facade_scars", {"frac": 0.08}), ("rooftop_fail", {"frac": 0.3}),
                ("glass_loss", {"frac": 0.1})],
        "DG2": [("infill_fail", {"storeys": 1, "frac": 0.35}),
                ("facade_scars", {"frac": 0.15}), ("rooftop_fail", {"frac": 0.5}),
                ("signage_fail", {}),
                ("balcony_fail", {"frac": 0.3}),
                ("parapet_fall", {"sides": 1, "frac": 0.4}),
                ("glass_loss", {"frac": 0.3})],
        "DG3": [("infill_fail", {"storeys": 2, "frac": 0.55}),
                ("balcony_fail", {"frac": 0.6}),
                ("facade_scars", {"frac": 0.18}), ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("glass_loss", {"frac": 0.5})],
        # soft storey OR mid-storey: the bake draws one per style
        "DG4": [("balcony_fail", {"frac": 0.7}),
                ("storey_collapse", {}), ("rooftop_fail", {"frac": 0.8}),
                ("glass_loss", {"frac": 0.6})],
        "DG5": [("pancake", {})],
    },
    "rc_glass": {
        "DG0": [],
        "DG1": [("glass_fallout", {"frac": 0.06})],
        "DG2": [("glass_fallout", {"frac": 0.2}),
                ("parapet_fall", {"sides": 1, "frac": 0.3})],
        "DG3": [("glass_fallout", {"frac": 0.45}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("infill_fail", {"storeys": 1, "frac": 0.4})],
        "DG4": [("glass_fallout", {"frac": 0.7}),
                ("soft_storey", {"storey": 0, "lean_deg": 2.5})],
        "DG5": [("glass_fallout", {"frac": 0.85}),
                ("tilt_sink", {"tilt_deg": 9.0, "sink_m": 1.4})],
    },
}

# FOUNDATION LEVELS, baked per style like the grades, chosen by the
# assembly on the soft-soil patch rather than by the shaking field.
FOUNDATION = {
    "SETTLE": [("settlement", {})],
    "TILT":   [("tilt_severe", {})],
    "OV":     [("overturn", {})],
}
for _t in LADDER:
    LADDER[_t].update(FOUNDATION)
# A 43 m podium-and-tower going over as one rigid body is not in the record
# (the Antakya overturns were 5-8 storey RC blocks) and its toppled podium
# frame read as a rack of shelves on the bench. Towers settle and lean only.
LADDER["rc_glass"]["OV"] = []

# What a fragment is bound to, per construction type: (cladding, inner
# material key, share of fragments that take the inner material). The inner
# share climbs with how much of the building is broken — set per recipe.
INNER = {"urm": "plaster", "rc": "concrete", "rc_glass": "concrete"}

# Kit sub-names that are a PARAPET band (they do not raise the roof and sit
# on top of it), per urban_building's band `sub` vocabulary.
_PARAPET_SUBS = ("parapet", "ledge", "cornice", "rooftop")
_SPECIAL_SUBS = ("roof", "portico", "pediment", "ornament")


# ---------------------------------------------------------------------------
# Reading a building back out of its placements
# ---------------------------------------------------------------------------
def _piece_name(p):
    return str(p.get("usd", "")).rsplit("/", 1)[-1].rsplit(".", 1)[0]


def _sub_and_mass(category, style):
    """`bld_<style>_wing0_storey_corner` -> ("storey_corner", "wing0")."""
    c = str(category or "")
    pre = "bld_{0}_".format(style)
    rest = c[len(pre):] if c.startswith(pre) else c
    mass = "main"
    while rest.startswith("wing"):
        k, _, rest = rest.partition("_")
        mass = k if mass == "main" else mass + "_" + k
    return rest, mass


def classify(p, style):
    """One placement -> element record (dict with role/sub/mass/height)."""
    from detail import urban_building as ub

    sub, mass = _sub_and_mass(p.get("category"), style)
    name = _piece_name(p)
    meas = ub.PIECES.get(name)
    h = float(meas[2]) if meas else 3.0
    if sub in _SPECIAL_SUBS:
        role = sub
    elif sub.endswith("_corner"):
        role = "corner"
        base = sub[:-len("_corner")]
        if base in _PARAPET_SUBS:
            role = "parapet_corner"
    elif sub.endswith("_extra"):
        role = "balcony"
    elif sub in _PARAPET_SUBS:
        role = "parapet"
    else:
        role = "wall"
    return {"p": p, "sub": sub, "mass": mass, "role": role, "name": name,
            "h": h, "z": float(p.get("z_m", 0.0)),
            "x": float(p.get("x_m", 0.0)), "y": float(p.get("y_m", 0.0)),
            "yaw": float(p.get("yaw_deg", 0.0))}


def _mass_specs(style, x, y, yaw, spec=None, z0=0.0, tag="main"):
    """[(tag, spec, cx, cy, W, D, z0, levels, top)] for the main body and
    every wing, mirroring `urban_building.build_building`'s own arithmetic
    so a slab lands exactly where the kit's band boundary is."""
    from detail import urban_building as ub

    spec = spec or ub.STYLES[style]
    W, D = ub.footprint(spec)
    levels = []
    z = z0
    for band in spec["bands"]:
        if band.get("parapet"):
            continue
        for _ in range(band.get("repeat", 1)):
            levels.append(z)
            z += band["h"]
    top = z
    out = [dict(tag=tag, spec=spec, cx=x, cy=y, yaw=yaw, W=W, D=D, z0=z0,
                levels=levels, top=top, module=spec["bands"][0]["module"])]
    ox, oy = -W / 2.0, -D / 2.0
    wings = list(spec.get("wings", []))
    if spec.get("tower"):
        wings.append((spec["tower"], None))
    for k, (wing, off) in enumerate(wings):
        tw, td = ub.footprint(wing)
        tx, ty = off if off is not None else ((W - tw) / 2.0, (D - td) / 2.0)
        cx, cy = ub._rot(ox + tx + tw / 2.0, oy + ty + td / 2.0, yaw)
        wtag = "wing{0}".format(k) if tag == "main" else tag + "_wing{0}".format(k)
        out += _mass_specs(style, x + cx, y + cy, yaw, spec=wing, z0=top,
                           tag=wtag)
    return out


def describe(style, placements, x, y, yaw):
    """Everything a recipe needs to know about one placed building."""
    from detail import urban_building as ub

    spec = ub.STYLES[style]
    els = [classify(p, style) for p in placements]
    masses = {m["tag"]: m for m in _mass_specs(style, x, y, yaw)}
    for e in els:
        m = masses.get(e["mass"]) or masses["main"]
        e["storey"] = _storey_of(m, e["z"])
        # Which side of its mass the piece is on, in the mass's own frame.
        lx, ly = _to_local(m, e["x"], e["y"])
        e["side"] = _side_of(m, lx, ly)
        e["lx"], e["ly"] = lx, ly
    return {"style": style, "family": spec.get("family", "01"),
            "type": FAMILY_TYPE.get(spec.get("family", "01"), "urm"),
            "x": x, "y": y, "yaw": yaw, "masses": masses, "elements": els,
            "H": max(m["top"] for m in masses.values())}


def _storey_of(m, z):
    lv = m["levels"]
    for i in range(len(lv) - 1, -1, -1):
        if z >= lv[i] - 0.05:
            return i
    return 0


def _to_local(m, wx, wy):
    a = math.radians(-m["yaw"])
    dx, dy = wx - m["cx"], wy - m["cy"]
    return (dx * math.cos(a) - dy * math.sin(a),
            dx * math.sin(a) + dy * math.cos(a))


def _to_world(m, lx, ly):
    a = math.radians(m["yaw"])
    return (m["cx"] + lx * math.cos(a) - ly * math.sin(a),
            m["cy"] + lx * math.sin(a) + ly * math.cos(a))


def _side_of(m, lx, ly):
    """S/E/N/W by which wall line the point is nearest (local frame, front
    = -Y = "S", matching urban_building)."""
    W, D = m["W"], m["D"]
    d = {"S": abs(ly + D / 2.0), "N": abs(ly - D / 2.0),
         "W": abs(lx + W / 2.0), "E": abs(lx - W / 2.0)}
    return min(d, key=d.get)


_SIDE_NORMAL = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0),
                "W": (-1.0, 0.0)}


def _opposite(side):
    return {"S": "N", "N": "S", "E": "W", "W": "E"}[side]


def _outward(m, side):
    nx, ny = _SIDE_NORMAL[side]
    a = math.radians(m["yaw"])
    return (nx * math.cos(a) - ny * math.sin(a),
            nx * math.sin(a) + ny * math.cos(a))


# ---------------------------------------------------------------------------
# Materials
# ---------------------------------------------------------------------------
_MEGA = "airstack://scene_gen/assets/materials/megascans/"
MATERIAL_URLS = {
    "concrete": _MEGA + "Worn_Pavement.usda",
    "brick": _MEGA + "Brick_Wall_Worn.usda",
    # WORLD-PROJECTED, because the berm mesh and every authored chunk carry
    # no UVs: the AEC `Dirt.usda` is UV-space and rendered the berm as one
    # flat brown mat. Soil_Mud is the megascans pack the tornado scour uses.
    "soil": _MEGA + "Soil_Mud.usda",
}


def materials(stage, parent):
    """The earthquake material set under `<parent>/QuakeLooks`. Cached on the
    stage: calling twice returns the same prims."""
    from pxr import Gf, Sdf, UsdShade
    import scene_generator as sg
    from . import damage

    scope = parent + "/QuakeLooks"
    out = {}
    for key, url in MATERIAL_URLS.items():
        path = scope + "/" + key
        if not UsdShade.Material.Get(stage, path):
            prim = stage.DefinePrim(Sdf.Path(path))
            prim.GetReferences().AddReference(sg._join_asset_root(url, ""))
            prim.Load()
        out[key] = UsdShade.Material.Get(stage, path)
    # Interior plaster: flat, slightly warm, matte. Rebar: near-black rust.
    # Glass shards: pale blue-green, glossy. Slab edge / crushed concrete:
    # the concrete map again but darker, for the inside of a pile.
    # DUSTY, NOT WHITE. The first bench bound a bright plaster to the back
    # of every fragment and the pile read as crumpled paper — a broken wall's
    # inside face is plaster under a coat of mortar dust, and everything in a
    # collapse is under that dust. Every flat colour here sits at 0.45-0.68
    # luma on purpose.
    flat = {
        "plaster": ((0.66, 0.62, 0.55), 1.0),
        "mortar": ((0.58, 0.55, 0.50), 1.0),
        "rebar": ((0.30, 0.19, 0.13), 0.55),
        "glass": ((0.62, 0.78, 0.80), 0.12),
        "timber": ((0.48, 0.36, 0.24), 0.9),
        "dark_concrete": ((0.40, 0.40, 0.38), 1.0),
        "crack": ((0.11, 0.10, 0.09), 1.0),
    }
    for key, (rgb, rough) in flat.items():
        path = scope + "/" + key
        m = UsdShade.Material.Get(stage, path)
        if not m:
            m = damage._pbr(stage, path, rgb, rough)
        out[key] = m
    # TIMBER IS A TEXTURE, NOT A COLOUR. A flat beige on a 4 m slab fragment
    # reads as cardboard; the sawn-plank map `planks.wood_material` carries
    # (triplanar, ~1.1 m a tile) reads as a floor deck. Darkened: an old
    # joisted floor is not fresh framing.
    try:
        from . import planks
        out["timber"] = planks.wood_material(stage, scope + "/timber_deck",
                                             tile_m=1.1, tint=(0.50, 0.41, 0.31),
                                             roughness=0.95)
    except Exception as exc:
        print("[quake_flow] plank texture unavailable ({0}); flat timber".format(exc))
    # A warm interior "room" material for partitions: paint over plaster.
    return out


def _bind(stage, path, mat):
    from pxr import UsdShade
    pr = stage.GetPrimAtPath(path)
    if pr and pr.IsValid() and mat is not None:
        UsdShade.MaterialBindingAPI(pr).Bind(mat)


def _clad_material(stage, parent, cache, texture):
    """A triplanar OmniPBR around a module's own cladding texture, cached."""
    from . import damage
    if not texture:
        return None
    m = cache.get(texture)
    if m is None:
        m = damage._pbr(stage, "{0}/QuakeLooks/clad_{1}".format(
            parent, len(cache)), (1.0, 1.0, 1.0), 0.85,
            texture=texture, scale_uv=(0.45, 0.45))
        cache[texture] = m
    return m


# ---------------------------------------------------------------------------
# Authoring helpers — boxes with a transform, so they can be moved & simulated
# ---------------------------------------------------------------------------
def _box(stage, path, cx, cy, cz, sx, sy, sz, yaw_deg=0.0, mat=None,
         uv_m=1.0):
    """A closed box mesh centred at (cx, cy, cz) with translate/rotateZ ops.

    Points are authored ABOUT THE CENTRE so the piece can be simulated (a
    rigid body rotates about its own origin) and transformed by a recipe
    (`_transform_prims`). Normals are faceVarying so the edges stay crisp —
    a slab with averaged normals renders as a pillow (planks.py).
    """
    from pxr import Gf, Sdf, UsdGeom, Vt

    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    P = Gf.Vec3f
    pts = [P(-hx, -hy, -hz), P(hx, -hy, -hz), P(hx, hy, -hz), P(-hx, hy, -hz),
           P(-hx, -hy, hz), P(hx, -hy, hz), P(hx, hy, hz), P(-hx, hy, hz)]
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    nrm = [(0, 0, -1), (0, 0, 1), (0, -1, 0), (1, 0, 0), (0, 1, 0), (-1, 0, 0)]
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray(pts))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    m.CreateNormalsAttr(Vt.Vec3fArray([P(*n) for n in nrm for _ in range(4)]))
    m.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr([P(-hx, -hy, -hz), P(hx, hy, hz)])
    xf = UsdGeom.Xformable(m)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xf.AddRotateZOp().Set(float(yaw_deg))
    if mat is not None:
        _bind(stage, path, mat)
    return path


def _cyl(stage, path, p0, p1, r, mat=None, sides=7):
    """A thin tube from p0 to p1 — a rebar. Authored in world space with no
    xform; it is never simulated."""
    from pxr import Gf, Sdf, UsdGeom, Vt

    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    ax = p1 - p0
    L = float(np.linalg.norm(ax))
    if L < 1e-4:
        return None
    ax /= L
    ref = np.array([0.0, 0.0, 1.0]) if abs(ax[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(ax, ref)
    u /= np.linalg.norm(u)
    v = np.cross(ax, u)
    pts, faces, counts = [], [], []
    for k in range(sides):
        a = 2.0 * math.pi * k / sides
        d = u * math.cos(a) + v * math.sin(a)
        pts.append(p0 + d * r)
        pts.append(p1 + d * r)
    for k in range(sides):
        a, b = 2 * k, 2 * ((k + 1) % sides)
        faces += [a, b, b + 1, a + 1]
        counts.append(4)
    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in pts]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    if mat is not None:
        _bind(stage, path, mat)
    return path


def _transform_prims(stage, paths, M):
    """Post-multiply every prim's world transform by the Gf.Matrix4d *M*
    (row-vector convention: p' = p * local * M) and re-author it as
    translate / orient / scale, the way `settle.bake` does.

    Only valid for prims whose parent is at the identity — every placement
    and fragment under the generated scope is — because the world delta is
    written back as a LOCAL transform.
    """
    from pxr import Gf, UsdGeom

    xf = UsdGeom.XformCache()
    n = 0
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        m = xf.GetLocalToWorldTransform(prim) * M
        tr = Gf.Transform(m)
        t = tr.GetTranslation()
        q = tr.GetRotation().GetQuat()
        sc = tr.GetScale()
        x = UsdGeom.Xformable(prim)
        x.ClearXformOpOrder()
        x.AddTranslateOp().Set(Gf.Vec3d(t))
        x.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
        x.AddScaleOp().Set(Gf.Vec3f(sc))
        n += 1
    return n


def _rot_about(pivot, axis, deg):
    """Gf.Matrix4d rotating `deg` about `axis` through `pivot` (row-vector)."""
    from pxr import Gf
    P = Gf.Vec3d(*pivot)
    R = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*axis), float(deg)))
    return (Gf.Matrix4d().SetTranslate(-P) * R
            * Gf.Matrix4d().SetTranslate(P))


def _translate(dx, dy, dz):
    from pxr import Gf
    return Gf.Matrix4d().SetTranslate(Gf.Vec3d(dx, dy, dz))


# ---------------------------------------------------------------------------
# Interior fit-out
# ---------------------------------------------------------------------------
SLAB_T = {"urm": 0.30, "rc": 0.22, "rc_glass": 0.22}
WALL_INSET = 0.55      # slab edge sits this far inside the wall line
COLUMN_W = 0.45

# WHAT IS IN THE ROOMS. Cheap props only (a few hundred points each — the
# ArchVis sofas are 260k and are not here), referenced from Nucleus, placed
# on every fitted-out slab so that whatever a recipe opens up has something
# behind it, and whatever it drops has contents in the pile. Office kit for
# the concrete families, the residential kit for masonry. Sizes are from
# the catalogue; both packs are Z-up, the residential one is cm-authored.
_NUC = "omniverse://airlab-nucleus.andrew.cmu.edu:443/"
_OFF = _NUC + "NVIDIA/Assets/Isaac/5.1/Isaac/Environments/Office/Props/"
_RES = _NUC + "Projects/SEI-COA/ModularNeighborhood/Assets/"
FURNITURE = {
    "rc": [(_OFF + "SM_SecretaryDeskA.usd", 1.0, 3),
           (_OFF + "SM_TableWorkingDouble.usd", 1.0, 2),
           (_OFF + "SM_ChairOffice.usd", 1.0, 4),
           (_OFF + "SM_FileCabinet_01.usd", 1.0, 2),
           (_OFF + "SM_BookcaseA.usd", 1.0, 1),
           (_OFF + "SM_Sofa.usd", 1.0, 1),
           (_OFF + "SM_Partition.usd", 1.0, 2),
           (_OFF + "SM_Plant01.usd", 1.0, 1)],
    "urm": [(_RES + "Bed_Double_01.usd", 0.01, 2),
            (_RES + "Kitchen_Unit_01.usd", 0.01, 1),
            (_RES + "Bookcase_01.usd", 0.01, 1),
            (_RES + "Office_Desk_01.usd", 0.01, 1),
            (_RES + "Chair_01.usd", 0.01, 3),
            (_RES + "table_01.usd", 0.01, 1),
            (_RES + "Fridge_01.usd", 0.01, 1),
            (_RES + "coffee_table_01.usd", 0.01, 1)],
}
FURNITURE["rc_glass"] = FURNITURE["rc"]
FURNITURE_PER_100M2 = 1.6      # props per 100 m2 of slab


def _prop(stage, path, usd, x, y, z_floor, yaw, scale, rng):
    """Reference a prop and seat it on z_floor by its measured bound."""
    from pxr import Gf, Sdf, Usd, UsdGeom
    prim = stage.DefinePrim(Sdf.Path(path))
    if not prim.GetReferences().AddReference(usd):
        return None
    prim.Load()
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return None
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z_floor))
    xf.AddRotateZOp().Set(float(yaw))
    xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    try:
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if not r.IsEmpty():
            c = r.GetMidpoint()
            xf.GetOrderedXformOps()[0].Set(Gf.Vec3d(
                x - (c[0] - x), y - (c[1] - y), z_floor - r.GetMin()[2] + z_floor))
    except Exception:
        pass
    return path


def fit_interior(stage, parent, info, mats, rng, storeys=None,
                 columns=True, partitions=True, tag="b"):
    """Author slabs (+ columns, partitions) for every mass of one building.

    Returns {"slabs": {(mass, storey): path}, "columns": {(mass, storey):
    [paths]}, "partitions": [...], "all": [...]}. Slab `storey` i is the
    floor OF storey i (its top at `levels[i]`); storey 0 has no slab (that
    is the ground). The roof is the kit's own roof piece.

    `storeys` limits which storeys get a fit-out (None = all): a recipe that
    only opens the top two floors of a twelve-storey block has no reason to
    pay for the other ten, and nothing can see them.
    """
    from pxr import Sdf, UsdGeom

    btype = info["type"]
    t_slab = SLAB_T[btype]
    slab_mat = mats["timber"] if btype == "urm" else mats["concrete"]
    out = {"slabs": {}, "columns": {}, "partitions": [], "props": {}, "all": []}
    UsdGeom.Scope.Define(stage, Sdf.Path("{0}/fit_{1}".format(parent, tag)))
    pool = []
    for usd, sc, w in FURNITURE.get(btype, ()):
        pool += [(usd, sc)] * int(w)
    for mtag, m in info["masses"].items():
        W, D = m["W"] - 2 * WALL_INSET, m["D"] - 2 * WALL_INSET
        n_lv = len(m["levels"])
        for i, z in enumerate(m["levels"]):
            if storeys is not None and i not in storeys:
                continue
            if i > 0:
                path = "{0}/fit_{1}/slab_{2}_{3}".format(parent, tag, mtag, i)
                _box(stage, path, m["cx"], m["cy"], z - t_slab / 2.0,
                     W, D, t_slab, m["yaw"], slab_mat)
                out["slabs"][(mtag, i)] = path
                out["all"].append(path)
            if columns and btype != "urm":
                cols = []
                pitch = max(4.0, float(m["module"]))
                h_st = (m["levels"][i + 1] if i + 1 < n_lv else m["top"]) - z
                nx = max(2, int(round(W / pitch)) + 1)
                ny = max(2, int(round(D / pitch)) + 1)
                for a in range(nx):
                    for b in range(ny):
                        # perimeter + an interior grid; skip the very centre
                        # rows on wide plans so it reads as a frame, not a
                        # forest
                        lx = -W / 2.0 + COLUMN_W / 2.0 + a * (W - COLUMN_W) / (nx - 1)
                        ly = -D / 2.0 + COLUMN_W / 2.0 + b * (D - COLUMN_W) / (ny - 1)
                        wx, wy = _to_world(m, lx, ly)
                        cpath = "{0}/fit_{1}/col_{2}_{3}_{4}_{5}".format(
                            parent, tag, mtag, i, a, b)
                        _box(stage, cpath, wx, wy, z + (h_st - t_slab) / 2.0,
                             COLUMN_W, COLUMN_W, h_st - t_slab, m["yaw"],
                             mats["concrete"])
                        cols.append(cpath)
                out["columns"][(mtag, i)] = cols
                out["all"].extend(cols)
            if pool and (i > 0 or btype != "urm"):
                # CONTENTS. Seated on the slab (or the ground floor), kept
                # 1.5 m inside the wall line so nothing pokes through a
                # façade, and yawed to the building.
                n_props = max(2, int(round(W * D / 100.0 * FURNITURE_PER_100M2)))
                props = []
                for k in range(n_props):
                    usd, sc = pool[rng.randrange(len(pool))]
                    lx = rng.uniform(-W / 2.0 + 1.5, W / 2.0 - 1.5)
                    ly = rng.uniform(-D / 2.0 + 1.5, D / 2.0 - 1.5)
                    wx, wy = _to_world(m, lx, ly)
                    ppath = "{0}/fit_{1}/prop_{2}_{3}_{4}".format(parent, tag, mtag, i, k)
                    if _prop(stage, ppath, usd, wx, wy, z + 0.01,
                             m["yaw"] + rng.choice((0.0, 90.0, 180.0, 270.0)),
                             sc, rng):
                        props.append(ppath)
                out["props"][(mtag, i)] = props
                out["all"].extend(props)
            if partitions and i > 0:
                # Two or three plaster partitions per storey, in the mass's
                # short direction, so an opened floor reads as rooms.
                h_st = (m["levels"][i + 1] if i + 1 < n_lv else m["top"]) - z
                n_part = 2 + (1 if W > 20 else 0)
                for k in range(n_part):
                    lx = -W / 2.0 + W * (k + 1) / (n_part + 1) + rng.uniform(-1.0, 1.0)
                    depth = D * rng.uniform(0.35, 0.7)
                    ly = rng.uniform(-D / 2.0 + depth / 2.0, D / 2.0 - depth / 2.0)
                    wx, wy = _to_world(m, lx, ly)
                    ppath = "{0}/fit_{1}/part_{2}_{3}_{4}".format(parent, tag, mtag, i, k)
                    _box(stage, ppath, wx, wy, z + (h_st - t_slab) / 2.0,
                         0.12, depth, h_st - t_slab - 0.05, m["yaw"],
                         mats["plaster"])
                    out["partitions"].append(ppath)
                    out["all"].append(ppath)
    return out


# ---------------------------------------------------------------------------
# Fracture wrappers
# ---------------------------------------------------------------------------
def _chunk_material(stage, parent, cache, texture, mats, btype, rng,
                    inner_p):
    if rng.random() < inner_p or not texture:
        if btype == "urm":
            r = rng.random()
            key = "brick" if r < 0.45 else ("mortar" if r < 0.75 else "plaster")
        else:
            r = rng.random()
            key = "concrete" if r < 0.5 else ("dark_concrete" if r < 0.8 else "plaster")
        return mats.get(key)
    return _clad_material(stage, parent, cache, texture)


def _break(stage, parent, el, tag, n, rng, nrng, mats, cache, btype,
           inner_p=0.35, partial=None, mode="uniform", rough=0.012,
           min_volume_frac=0.002, consume=0.0):
    """Fracture one element. Returns (static_paths, loose_paths).

    `consume` drops that share of the fragments, LARGEST first (the fracture
    module's bias). Nothing is destroyed in an earthquake, but a kit wall is
    an open shell and its fragments are foil: the pile's MASS is the authored
    heap, and thinning the shells out of it is what stops the heap reading as
    a pile of paper. The material is still there — it is in the heap."""
    from . import damage, fracture

    path = el["p"].get("prim_path")
    if not path:
        return [], []
    tex = damage.bound_texture(stage, path)
    out = "{0}/brk_{1}_{2}".format(parent, tag, path.rsplit("/", 1)[-1])
    if partial is not None:
        st, lo = fracture.fracture_partial(
            stage, path, out, n_pieces=n, rng=nrng, cut_frac=partial,
            mode=mode, rough=rough, consume=consume * 0.5,
            min_volume_frac=min_volume_frac)
    else:
        st, lo = [], fracture.fracture_prim(
            stage, path, out, n_pieces=n, rng=nrng, mode=mode, rough=rough,
            verbose=False, consume=consume, consume_pool=1.6,
            min_volume_frac=min_volume_frac)
    for pth in list(st) + list(lo):
        _bind(stage, pth, _chunk_material(stage, parent, cache, tex, mats,
                                          btype, rng, inner_p))
    return list(st), list(lo)


def _break_split(ctx, path, n, judge, mat_fn, rough=0.012,
                 min_volume_frac=0.0015, mode="uniform", aspect=None):
    """Fracture *path* and split the fragments by `judge(centroid) -> bool`
    (True = comes loose). The rest stay as STATIC stubs, so the surviving
    edge is made of real cell boundaries: this is `fracture_partial` with a
    caller-supplied judge instead of a height cut, and it is what makes a
    hole ragged along its sides rather than only along its bottom.
    Returns (static_paths, loose_paths)."""
    from . import fracture
    stage = ctx["stage"]
    mesh = fracture.prim_to_mesh(stage, path)
    if mesh is None:
        return [], []
    frags = fracture.fracture_mesh(mesh, n, ctx["nrng"], mode=mode, rough=rough,
                                   consume=0.0, min_volume_frac=min_volume_frac,
                                   aspect=aspect)
    if not frags:
        return [], []
    out = "{0}/brk_{1}_{2}".format(ctx["parent"], ctx["tag"], path.rsplit("/", 1)[-1])
    from pxr import Sdf, UsdGeom
    UsdGeom.Scope.Define(stage, Sdf.Path(out))
    st, lo = [], []
    for i, f in enumerate(frags):
        c = np.asarray(f.centroid, dtype=float)
        fp = "{0}/frag_{1:03d}".format(out, i)
        fracture._write_mesh(stage, fp, f)
        _bind(stage, fp, mat_fn())
        (lo if judge(c) else st).append(fp)
    src = stage.GetPrimAtPath(path)
    if src and src.IsValid():
        src.SetActive(False)
    return st, lo


def _wander(rng, amp, freq=(1.0, 2.6)):
    """A 1-D wobble f(t) in [-amp, amp] for ragged break lines."""
    ph = rng.uniform(0, 6.28)
    fr = rng.uniform(*freq)
    return lambda t: amp * (0.6 * math.sin(t * fr * 6.28 + ph)
                            + 0.4 * math.sin(t * fr * 2.7 + ph * 1.7))


def _edge_judge(m, side, depth_m, rng):
    """judge(world centroid) -> True when the point lies within a WANDERING
    depth of the mass's `side` wall line (measured inward)."""
    W, D = m["W"], m["D"]
    wob = _wander(rng, depth_m * 0.45)

    def judge(c):
        lx, ly = _to_local(m, c[0], c[1])
        if side == "S":
            d, t = ly + D / 2.0, (lx + W / 2.0) / W
        elif side == "N":
            d, t = D / 2.0 - ly, (lx + W / 2.0) / W
        elif side == "W":
            d, t = lx + W / 2.0, (ly + D / 2.0) / D
        else:
            d, t = W / 2.0 - lx, (ly + D / 2.0) / D
        return d < depth_m + wob(t)
    return judge


def _toward_judge(m, side, frac, rng):
    """For a wall piece on a side ADJACENT to a failed one: the part of the
    piece nearest the failed side comes away (`frac` of its length), the
    far part stays, with a wandering line between them."""
    W, D = m["W"], m["D"]
    wob = _wander(rng, 0.35)

    def judge(c):
        lx, ly = _to_local(m, c[0], c[1])
        if side == "S":
            u, v = (ly + D / 2.0) / D, c[2]
        elif side == "N":
            u, v = (D / 2.0 - ly) / D, c[2]
        elif side == "W":
            u, v = (lx + W / 2.0) / W, c[2]
        else:
            u, v = (W / 2.0 - lx) / W, c[2]
        return u < frac + wob(v / 6.0) * 0.15
    return judge


def _mat_fn(ctx, texture, inner_p=0.35):
    btype = ctx["info"]["type"]
    return lambda: _chunk_material(ctx["stage"], ctx["parent"], ctx["cache"],
                                   texture, ctx["mats"], btype, ctx["rng"], inner_p)


def _ragged_neighbours(ctx, mass, side, storeys, depth_bays=1.0, frac=0.45):
    """The bays adjoining a failed side, on the two neighbouring sides, lose
    their near ends along a wandering line — so the hole's vertical edges
    are Voronoi boundaries, not the kit's module seams."""
    from . import damage
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    W, D = m["W"], m["D"]
    reach = depth_bays * max(4.0, float(m["module"])) + 1.0
    ox, oy = _outward(m, side)
    for e in list(_els(ctx, mass=mass, role=("wall", "corner", "balcony"))):
        if e["side"] == side or e["side"] == _opposite(side):
            continue
        if e["storey"] not in storeys:
            continue
        # distance of the piece's near end from the failed wall line
        if side == "S":
            d = e["ly"] + D / 2.0
        elif side == "N":
            d = D / 2.0 - e["ly"]
        elif side == "W":
            d = e["lx"] + W / 2.0
        else:
            d = W / 2.0 - e["lx"]
        # pieces are pivoted at their LEFT end as seen from outside; on the E
        # side that end is toward S, on the W side toward N — so a piece
        # whose pivot is one module past the line can still reach it
        if d > reach:
            continue
        path = e["p"].get("prim_path")
        if not path:
            continue
        tex = damage.bound_texture(ctx["stage"], path)
        keep = rng.uniform(0.25, 0.6) if d < 0.5 else rng.uniform(0.1, 0.35)
        st, lo = _break_split(ctx, path, 9 + rng.randrange(5),
                              _toward_judge(m, side, keep, rng),
                              _mat_fn(ctx, tex, 0.35))
        for pth in lo:
            v = rng.uniform(0.5, 1.6)
            ctx["velocity"][pth] = (ox * v, oy * v, 0.0)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True


def _box_dims(stage, path):
    """(cx, cy, cz, sx, sy, sz, yaw) of a box authored by `_box`."""
    from pxr import Gf, UsdGeom
    pr = stage.GetPrimAtPath(path)
    m = UsdGeom.Mesh(pr)
    pts = m.GetPointsAttr().Get()
    xs = [q[0] for q in pts]; ys = [q[1] for q in pts]; zs = [q[2] for q in pts]
    t, yaw = (0.0, 0.0, 0.0), 0.0
    for op in UsdGeom.Xformable(pr).GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            t = tuple(op.Get())
        elif op.GetOpType() == UsdGeom.XformOp.TypeRotateZ:
            yaw = float(op.Get())
    return (t[0], t[1], t[2], max(xs) - min(xs), max(ys) - min(ys),
            max(zs) - min(zs), yaw)


def _split_strip(ctx, path, m, side, depth, mat):
    """Cut a `_box` into an untouched REMAINDER and a STRIP `depth` wide
    along the mass's `side`; deactivate the original. Returns
    (remainder_path, strip_path). Only the strip is then fractured, so an
    intact roof stays one clean slab instead of a crack mosaic."""
    stage = ctx["stage"]
    cx, cy, cz, sx, sy, sz, yaw = _box_dims(stage, path)
    # box is authored in the mass frame (yaw = mass yaw); local x along W
    lx, ly = _to_local(m, cx, cy)
    along_x = side in ("E", "W")
    full = sx if along_x else sy
    depth = min(depth, full * 0.6)
    sgn = 1.0 if side in ("E", "N") else -1.0
    if along_x:
        sc = lx + sgn * (sx / 2.0 - depth / 2.0)
        rc = lx - sgn * depth / 2.0
        strip_dims, rem_dims = (depth, sy), (sx - depth, sy)
        sxy, rxy = (sc, ly), (rc, ly)
    else:
        sc = ly + sgn * (sy / 2.0 - depth / 2.0)
        rc = ly - sgn * depth / 2.0
        strip_dims, rem_dims = (sx, depth), (sx, sy - depth)
        sxy, rxy = (lx, sc), (lx, rc)
    from pxr import UsdShade
    bm = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path)).ComputeBoundMaterial()[0]
    keep_mat = bm if bm else mat
    wx, wy = _to_world(m, *rxy)
    rem = "{0}_rem{1}".format(path, _uid(ctx))
    _box(stage, rem, wx, wy, cz, rem_dims[0], rem_dims[1], sz, yaw, keep_mat)
    wx, wy = _to_world(m, *sxy)
    strip = "{0}_strip{1}".format(path, _uid(ctx))
    _box(stage, strip, wx, wy, cz, strip_dims[0], strip_dims[1], sz, yaw, keep_mat)
    pr = stage.GetPrimAtPath(path)
    if pr and pr.IsValid():
        pr.SetActive(False)
    return rem, strip


def _ragged_slabs(ctx, mass, side, storeys, depth=(0.8, 2.6)):
    """Floor slabs (and the roof) lose a wandering strip along the failed
    side: the exposed floor edge is broken, not a machined line."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    fit = ctx["fit"]
    mats = ctx["mats"]
    btype = ctx["info"]["type"]
    slab_mat = (lambda: mats["timber"] if (btype == "urm" and rng.random() < 0.7)
                else (mats["concrete"] if rng.random() < 0.6 else mats["dark_concrete"]))
    for (mt, i), pth in list(fit["slabs"].items()):
        if mt != mass or i not in storeys or not pth:
            continue
        d = rng.uniform(*depth)
        rem, strip = _split_strip(ctx, pth, m, side, d + 1.2, mats["concrete"])
        st, lo = _break_split(ctx, strip, 10 + rng.randrange(5),
                              _edge_judge(m, side, d, rng),
                              slab_mat, min_volume_frac=0.0008)
        fit["slabs"][(mt, i)] = rem
        fit["all"] = [q for q in fit["all"] if q != pth] + [rem] + st
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
    # the roof over the failed side: the strip along it, no more
    for e in list(_els(ctx, mass=mass, role="roof")):
        box = _roof_box(ctx, e)
        if not box:
            continue
        d = rng.uniform(*depth)
        rem, strip = _split_strip(ctx, box, m, side, d + 0.8, mats["concrete"])
        from pxr import UsdShade
        bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(strip)).ComputeBoundMaterial()[0]
        st, lo = _break_split(ctx, strip, 10 + rng.randrange(5),
                              _edge_judge(m, side, d, rng),
                              lambda: (mats["timber"] if (btype == "urm" and rng.random() < 0.5)
                                       else mats["concrete"]),
                              min_volume_frac=0.0006)
        if bm:
            for pth in st:
                _bind(ctx["stage"], pth, bm)      # surviving roof keeps its look
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        ctx["authored"].append(rem)
        e["dead"] = True


def _disturb_interior(ctx, mass, storeys, side=None):
    """What an opened floor looks like: partitions snapped or toppled,
    rubble litter on the slab, the odd prop knocked over. Only on the
    storeys a recipe exposed."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    fit = ctx["fit"]
    m = ctx["info"]["masses"][mass]
    mats = ctx["mats"]
    W, D = m["W"], m["D"]
    for pth in list(fit["partitions"]):
        try:
            _, pm, i, _k = pth.rsplit("/", 1)[-1].split("_", 3)
            i = int(i)
        except ValueError:
            continue
        if pm != mass or i not in storeys:
            continue
        r = rng.random()
        if r < 0.35:
            # snapped: the upper part comes away
            from . import fracture
            st, lo = fracture.fracture_partial(
                ctx["stage"], pth, pth + "_brk", n_pieces=7, rng=nrng,
                cut_frac=rng.uniform(0.3, 0.7), mode="uniform", rough=0.01,
                consume=0.2, min_volume_frac=0.001)
            for q in st + lo:
                _bind(ctx["stage"], q, mats["plaster"])
            ctx["loose"] += lo
            ctx["static_extra"] += st
            fit["all"] = [q for q in fit["all"] if q != pth] + st
        elif r < 0.5:
            # toppled: rigid body, tipped past balance so the solver lays it
            ctx["loose"].append(pth)
            fit["all"] = [q for q in fit["all"] if q != pth]
            M = _rot_about(_pivot_of(ctx, pth), (1.0, 0.0, 0.0), rng.uniform(25, 40))
            _transform_prims(ctx["stage"], [pth], M)
    # litter on the slab, dense toward the opening
    for i in storeys:
        z = m["levels"][i] if i < len(m["levels"]) else m["top"]
        n = int(W * D / 100.0 * 9)
        for k in range(n):
            lx = rng.uniform(-W / 2.0 + 0.8, W / 2.0 - 0.8)
            ly = rng.uniform(-D / 2.0 + 0.8, D / 2.0 - 0.8)
            if side is not None:
                # pull toward the opened side
                t = rng.random() ** 0.5
                if side == "S":
                    ly = -D / 2.0 + 0.8 + (D - 1.6) * (1 - t) * rng.random()
                elif side == "N":
                    ly = D / 2.0 - 0.8 - (D - 1.6) * (1 - t) * rng.random()
                elif side == "W":
                    lx = -W / 2.0 + 0.8 + (W - 1.6) * (1 - t) * rng.random()
                else:
                    lx = W / 2.0 - 0.8 - (W - 1.6) * (1 - t) * rng.random()
            wx, wy = _to_world(m, lx, ly)
            sz = rng.uniform(0.12, 0.55)
            path = "{0}/litter_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
            mat = (mats["brick"] if (ctx["info"]["type"] == "urm" and rng.random() < 0.5)
                   else (mats["plaster"] if rng.random() < 0.5 else mats["dark_concrete"]))
            _box(ctx["stage"], path, wx, wy, z + sz * 0.2, sz, sz * rng.uniform(0.5, 1.0),
                 sz * rng.uniform(0.3, 0.6), rng.uniform(0, 180), mat)
            ctx["authored"].append(path)
            ctx["static_extra"].append(path)


def _pivot_of(ctx, path):
    from pxr import Usd, UsdGeom
    pr = ctx["stage"].GetPrimAtPath(path)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    if r.IsEmpty():
        return (0.0, 0.0, 0.0)
    c = r.GetMidpoint()
    return (c[0], c[1], r.GetMin()[2])


def _spall(ctx, mass, rate=0.15, storeys=None):
    """Small losses off otherwise intact walls — a top course here, a
    corner there — so the standing sides are not showroom-clean."""
    from . import fracture, damage
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    top = len(m["levels"]) - 1
    for e in list(_els(ctx, mass=mass, role=("wall", "corner"))):
        if storeys is not None and e["storey"] not in storeys:
            continue
        p = rate * (1.8 if e["storey"] == top else 0.7)
        if rng.random() >= p:
            continue
        path = e["p"].get("prim_path")
        if not path:
            continue
        tex = damage.bound_texture(ctx["stage"], path)
        st, lo = fracture.fracture_partial(
            ctx["stage"], path, "{0}/brk_{1}_{2}".format(ctx["parent"], ctx["tag"],
                                                         path.rsplit("/", 1)[-1]),
            n_pieces=8 + rng.randrange(4), rng=nrng,
            cut_frac=rng.uniform(0.72, 0.9), mode="uniform", rough=0.01,
            consume=0.0, min_volume_frac=0.0015)
        mf = _mat_fn(ctx, tex, 0.3)
        for q in st + lo:
            _bind(ctx["stage"], q, mf())
        ox, oy = _outward(m, e["side"])
        for q in lo:
            v = rng.uniform(0.4, 1.2)
            ctx["velocity"][q] = (ox * v, oy * v, 0.0)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True


def _break_box(stage, path, n, rng, nrng, mat, inner_mat=None, inner_p=0.5,
               mode="uniform", aspect=None, consume=0.0):
    """Fracture an authored box (slab / column) into chunks. Returns paths."""
    from . import fracture
    out = path + "_brk"
    made = fracture.fracture_prim(stage, path, out, n_pieces=n, rng=nrng,
                                  mode=mode, aspect=aspect, rough=0.012,
                                  verbose=False, consume=consume,
                                  consume_pool=1.6, min_volume_frac=0.0008)
    for pth in made:
        _bind(stage, pth, inner_mat if (inner_mat is not None
                                        and rng.random() < inner_p) else mat)
    return made


def _deactivate(stage, path):
    pr = stage.GetPrimAtPath(path) if path else None
    if pr and pr.IsValid() and pr.IsActive():
        pr.SetActive(False)
        return True
    return False


# ---------------------------------------------------------------------------
# The recipes. Each takes the shared `ctx` and returns nothing; they append
# to ctx["loose"], ctx["static_extra"], ctx["velocity"], ctx["removed"].
# ---------------------------------------------------------------------------
def _uid(ctx):
    """A per-building counter for authored prim names. Two recipes on one
    building (a corner fan and a windrow, two shard fields) otherwise name
    their chunks alike and the second `Define` trips on the first's ops."""
    ctx["n_uid"] = ctx.get("n_uid", 0) + 1
    return ctx["n_uid"]


def _els(ctx, mass=None, role=None, side=None, storey=None):
    for e in ctx["info"]["elements"]:
        if e.get("dead"):
            continue
        if mass is not None and e["mass"] != mass:
            continue
        if role is not None and (e["role"] not in role
                                 if isinstance(role, (tuple, list, set))
                                 else e["role"] != role):
            continue
        if side is not None and e["side"] != side:
            continue
        if storey is not None and e["storey"] != storey:
            continue
        yield e


def _pick_sides(ctx, n, prefer_front=True):
    sides = ["S", "E", "W", "N"]
    rng = ctx["rng"]
    if not prefer_front:
        rng.shuffle(sides)
    else:
        rest = sides[1:]
        rng.shuffle(rest)
        sides = ["S"] + rest
    return sides[:max(0, min(4, int(n)))]


def r_parapet_fall(ctx, sides=1, frac=0.5, mass="main"):
    """Parapet / cornice pieces on `sides` walls break off and drop."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    top = len(m["levels"]) - 1
    for side in _pick_sides(ctx, sides):
        run = list(_els(ctx, mass=mass, role=("parapet", "parapet_corner"),
                        side=side))
        # NO PARAPET BAND (brick commercial, the storefront terrace, the
        # church): the top course of the top storey goes instead — a
        # partial break with the cut line high, so a ragged edge is left.
        partial = None
        if not run:
            run = list(_els(ctx, mass=mass, role="wall", side=side, storey=top))
            partial = True
            frac = min(1.0, frac * 1.6)      # a course off most of the side
        if not run:
            continue
        # ALONG THE SIDE, spatially — the placement order is not a position
        # order once corners are appended, and the windrow below is placed
        # from the chosen pieces' own coordinates.
        along_key = (lambda e: e["lx"]) if side in ("S", "N") else (lambda e: e["ly"])
        run.sort(key=along_key)
        L = m["W"] if side in ("S", "N") else m["D"]
        k = max(2 if len(run) > 2 else 1, int(round(len(run) * frac)))
        start = rng.randrange(0, max(1, len(run) - k + 1))
        chosen = run[start:start + k]
        ts = [along_key(e) / L for e in chosen]
        ox, oy = _outward(m, side)
        for e in chosen:
            st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                            (7 if partial else 5) + rng.randrange(4), rng, nrng,
                            ctx["mats"], ctx["cache"], ctx["info"]["type"],
                            inner_p=0.3, rough=0.01,
                            partial=(rng.uniform(0.55, 0.78) if partial else None))
            for pth in lo:
                v = rng.uniform(0.8, 2.0)
                ctx["velocity"][pth] = (ox * v, oy * v, 0.0)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
        # The windrow on the sidewalk — 1-4 m into the street, 0.5-1.5 m
        # deep (Christchurch parapet reconnaissance), under the pieces that
        # went (a piece is pivoted at its near end, so extend by one module).
        t0, t1 = min(ts) - 0.02, max(ts) + max(4.0, float(m["module"])) / L
        _heap(ctx, m, m["z0"], 0.0, 0.14, fill=False, sides=(side,),
              depth_m=rng.uniform(0.5, 1.1), along=(t0, t1), tag="windrow")


def r_glass_loss(ctx, frac=0.3):
    """Windows in an opaque façade: the kit bakes glass into each module, so
    there is nothing to remove — this only scatters a glass shard field on
    the sidewalk under `frac` of the façade. Cheap and authored."""
    _shard_field(ctx, frac, height_bias=0.6)


def _shard_field(ctx, frac, height_bias=0.6, mass="main", sides=None):
    rng = ctx["rng"]
    info = ctx["info"]
    m = info["masses"][mass]
    H = m["top"] - m["z0"]
    n = int(28 * frac * (m["W"] + m["D"]) / 40.0 * max(1.0, H / 12.0))
    made = []
    for side in (sides or ("S", "E", "N", "W")):
        ox, oy = _outward(m, side)
        L = m["W"] if side in ("S", "N") else m["D"]
        for k in range(max(1, n // 4)):
            t = rng.uniform(-0.5, 0.5) * L
            d = rng.uniform(0.3, 0.18 * H + 1.5)
            if side in ("S", "N"):
                lx, ly = t, (-m["D"] / 2.0 - d if side == "S" else m["D"] / 2.0 + d)
            else:
                lx, ly = (-m["W"] / 2.0 - d if side == "W" else m["W"] / 2.0 + d), t
            wx, wy = _to_world(m, lx, ly)
            s = rng.uniform(0.15, 0.55)
            path = "{0}/shard_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
            _box(ctx["stage"], path, wx, wy, 0.01, s, s * rng.uniform(0.4, 1.0),
                 0.012, rng.uniform(0, 180), ctx["mats"]["glass"])
            made.append(path)
    ctx["authored"] += made


def r_glass_fallout(ctx, frac=0.4, mass=None):
    """Curtain-wall modules (family 05 `Skyscraper*`) removed at random,
    biased to the lower storeys and to one face; shard field below."""
    rng = ctx["rng"]
    info = ctx["info"]
    targets = [e for e in _els(ctx, role=("wall", "corner"))
               if "Skyscraper" in e["name"] and (mass is None or e["mass"] == mass)]
    if not targets:
        return
    masses = sorted({e["mass"] for e in targets})
    bad_side = rng.choice(["S", "E", "N", "W"])
    n_removed = 0
    for e in targets:
        m = info["masses"][e["mass"]]
        H = max(1.0, m["top"] - m["z0"])
        hfrac = (e["z"] - m["z0"]) / H
        p = frac * (1.35 - 0.7 * hfrac) * (1.45 if e["side"] == bad_side else 0.8)
        if rng.random() < p:
            if _deactivate(ctx["stage"], e["p"].get("prim_path")):
                e["dead"] = True
                n_removed += 1
    for mt in masses:
        _shard_field(ctx, frac * 1.6, mass=mt)
    ctx["notes"].append("glass_fallout: {0} module(s) of {1}".format(
        n_removed, len(targets)))


def r_infill_fail(ctx, storeys=1, frac=0.4, mass="main"):
    """RC frame: masonry infill panels between the columns blow out of the
    bottom `storeys` storeys. The façade module goes, the fragments drop at
    the foot of the wall, and the column/slab grid behind shows."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    n_lv = len(m["levels"])
    # STOREYS 1..N, NOT THE GROUND BAND. On the concrete families the ground
    # band is a 7 m glazed lobby (curved storefront pieces); the masonry
    # INFILL that drops out of an RC frame is the panel between the columns
    # on the storeys above it. Targeting storey 0 removed a couple of shop
    # windows and read as nothing.
    opened = set()
    for s in range(1, min(n_lv, 1 + storeys)):
        for e in list(_els(ctx, mass=mass, role="wall", storey=s)):
            if rng.random() >= frac:
                continue
            opened.add(s)
            st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                            8 + rng.randrange(5), rng, nrng, ctx["mats"],
                            ctx["cache"], ctx["info"]["type"], inner_p=0.5,
                            consume=0.4)
            m = ctx["info"]["masses"][mass]
            ox, oy = _outward(m, e["side"])
            for pth in lo:
                v = rng.uniform(0.3, 0.9)
                ctx["velocity"][pth] = (ox * v, oy * v, 0.0)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
    if opened:
        _disturb_interior(ctx, mass, opened)
        # the hollow-block rubble under the dropped panels
        _heap(ctx, m, m["z0"], 0.0, 0.1, fill=False, sides=("S", "E", "N", "W"),
              depth_m=rng.uniform(0.3, 0.6), tag="windrow")


def r_corner_fail(ctx, storeys=2, mass="main", corner=None):
    """The top `storeys` storeys of one corner drop: corner pieces plus the
    adjoining bay on each side. A V-notch in the roofline."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    corner = corner or rng.choice(["SE", "SW", "NE", "NW"])
    cx = m["W"] / 2.0 * (1 if "E" in corner else -1)
    cy = m["D"] / 2.0 * (1 if "N" in corner else -1)
    n_lv = len(m["levels"])
    top_storeys = set(range(max(0, n_lv - storeys), n_lv))
    reach = max(4.0, float(m["module"])) * 1.6
    for e in list(_els(ctx, mass=mass)):
        if e["role"] not in ("wall", "corner", "parapet", "parapet_corner", "balcony"):
            continue
        if e["storey"] not in top_storeys and e["role"] not in ("parapet", "parapet_corner"):
            continue
        # distance from the corner, in the local frame, measured at the
        # piece's NEAR end (pieces are pivoted at their left end)
        d = math.hypot(e["lx"] - cx, e["ly"] - cy)
        if d > reach + 2.5:
            continue
        st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                        8 + rng.randrange(5), rng, nrng, ctx["mats"],
                        ctx["cache"], ctx["info"]["type"], inner_p=0.4)
        ox, oy = _outward(m, e["side"])
        for pth in lo:
            v = rng.uniform(0.2, 0.8)
            ctx["velocity"][pth] = (ox * v, oy * v, 0.0)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True
    # Pieces just beyond the notch lose their near ends (ragged verticals),
    # the slabs and the roof lose a broken corner, the exposed floors are
    # disturbed, and the rest of the building gets a little spalling.
    from . import damage
    for e in list(_els(ctx, mass=mass, role=("wall", "corner"))):
        if e["storey"] not in top_storeys:
            continue
        d = math.hypot(e["lx"] - cx, e["ly"] - cy)
        if d > reach + 2.5 and d < reach + 2.5 + max(4.0, float(m["module"])) + 1.0:
            path = e["p"].get("prim_path")
            if not path:
                continue
            tex = damage.bound_texture(ctx["stage"], path)
            wob = _wander(rng, 0.9)
            keep_r = d - rng.uniform(1.0, 3.0)

            def judge(c, _k=keep_r, _w=wob):
                lx, ly = _to_local(m, c[0], c[1])
                return math.hypot(lx - cx, ly - cy) < _k + _w(c[2] / 6.0)
            st, lo = _break_split(ctx, path, 9 + rng.randrange(4), judge,
                                  _mat_fn(ctx, tex, 0.35))
            ox, oy = _outward(m, e["side"])
            for q in lo:
                v = rng.uniform(0.3, 1.0)
                ctx["velocity"][q] = (ox * v, oy * v, 0.0)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
    fit = ctx["fit"]
    mats = ctx["mats"]
    btype = ctx["info"]["type"]
    c_sides = (("S" if "S" in corner else "N"), ("E" if "E" in corner else "W"))

    def _corner_break(path, mat_fn):
        """Strip along each of the corner's two sides, fractured against a
        wandering radius about the corner; the rest of the slab stays one
        clean piece."""
        rem = path
        statics, loose = [], []
        other = {c_sides[0]: c_sides[1], c_sides[1]: c_sides[0]}
        for sd in c_sides:
            # narrow: `reach` is already a bay and a half; the strip only
            # has to hold the notch's ragged edge, not half the roof
            rem, strip = _split_strip(ctx, rem, m, sd, min(reach * 0.75 + 0.5,
                                                            0.35 * (m["W"] if sd in ("S", "N") else m["D"])),
                                      mats["concrete"])
            # ...AND ONLY THE CORNER'S LENGTH OF IT. A strip the full length
            # of the side leaves a crack line right across the roof; cut it
            # again along the OTHER corner side so the far part stays whole.
            far_rem, strip = _split_strip(ctx, strip, m, other[sd], reach + 2.0,
                                          mats["concrete"])
            statics.append(far_rem)
            wob = _wander(rng, 1.2)
            rr = reach + rng.uniform(0.0, 1.5)

            def judge(c, _r=rr, _w=wob):
                lx, ly = _to_local(m, c[0], c[1])
                return math.hypot(lx - cx, ly - cy) < _r + _w(math.atan2(ly - cy, lx - cx))
            from pxr import UsdShade
            bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(strip)).ComputeBoundMaterial()[0]
            st, lo = _break_split(ctx, strip, 9 + rng.randrange(4), judge, mat_fn,
                                  min_volume_frac=0.0008)
            if bm:
                for q in st:
                    _bind(ctx["stage"], q, bm)
            statics += st
            loose += lo
        return rem, statics, loose

    for (mt, i), pth in list(fit["slabs"].items()):
        if mt != mass or i not in top_storeys or not pth:
            continue
        rem, st, lo = _corner_break(
            pth, lambda: (mats["timber"] if btype == "urm" else mats["concrete"]))
        fit["slabs"][(mt, i)] = rem
        fit["all"] = [q for q in fit["all"] if q != pth] + [rem] + st
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
    for e in list(_els(ctx, mass=mass, role="roof")):
        # only tiles that reach the corner; the rest stay kit tiles (and
        # stay available to `roof_hole`, which runs after this on DG3-4)
        if math.hypot(e["lx"] - cx, e["ly"] - cy) > reach + 7.0 and \
                max(m["W"], m["D"]) > 12.0 and len(list(_els(ctx, mass=mass, role="roof"))) > 1:
            continue
        box = _roof_box(ctx, e)
        if not box:
            continue
        rem, st, lo = _corner_break(
            box, lambda: (mats["timber"] if (btype == "urm" and rng.random() < 0.5)
                          else mats["concrete"]))
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        ctx["authored"].append(rem)
        e["dead"] = True
    _disturb_interior(ctx, mass, top_storeys)
    _spall(ctx, mass, rate=0.06)
    # the notch's own rubble fan on the pavement below the corner — the
    # last `reach` metres of each of the two sides that meet there
    for sd in (("S" if "S" in corner else "N"), ("E" if "E" in corner else "W")):
        L = m["W"] if sd in ("S", "N") else m["D"]
        # side coordinate runs -0.5..0.5 from SW/SE toward the far end
        # (S/N: along +x; E/W: along +y), so the corner is at +-0.5
        at_hi = (("E" in corner) if sd in ("S", "N") else ("N" in corner))
        w = min(0.5, (reach + 2.0) / L)
        along = (0.5 - w, 0.5) if at_hi else (-0.5, -0.5 + w)
        _heap(ctx, m, m["z0"], 0.0, 0.18, fill=False, sides=(sd,),
              depth_m=rng.uniform(0.6, 1.2), along=along, tag="windrow")


ROOF_T = 0.25


def _roof_box(ctx, e, thick=None):
    """Swap a kit roof piece for a SOLID slab of the same footprint.

    Every kit roof tile is a zero-thickness quad (`SM_MBuilding01_Roof` is
    5 x 5 x 0.000 m, four points). Fractured, those come out as paper and
    PhysX cooks no hull for a flat polygon ("a flat quad cannot be a
    convex-hull collider"), so a roof that has to break or move is first
    replaced by a 0.25 m box at its own world bbox, bound to the tile's own
    texture (triplanar) so it still reads as that roof. Returns the box
    path, or None if the piece could not be measured."""
    from pxr import Gf, Usd, UsdGeom
    from . import damage
    stage = ctx["stage"]
    path = e["p"].get("prim_path")
    pr = stage.GetPrimAtPath(path) if path else None
    if not pr or not pr.IsValid():
        return None
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    mn, mx = r.GetMin(), r.GetMax()
    sx, sy = max(0.3, mx[0] - mn[0]), max(0.3, mx[1] - mn[1])
    top = mx[2]
    T = float(thick or ROOF_T)
    # CONCRETE, NOT THE TILE'S OWN TEXTURE. A kit roof tile's base colour is
    # the family's façade ATLAS, sampled through UVs that pick out the roof
    # patch; projected triplanar onto a UV-less slab it paints WINDOWS across
    # the roof (seen on the office family in the first assembled city). A
    # flat roof is grey concrete from the air anyway.
    mat = ctx["mats"]["concrete"]
    # an axis-aligned box: kit roofs are laid at the building yaw, and the
    # bbox is already in world, so a yawed building gets an oversize box —
    # acceptable at the yaw jitter the layout uses, and exact at 0/90.
    yaw = float(e.get("yaw", 0.0)) % 90.0
    if yaw > 1.0 and yaw < 89.0:
        # rotated roof: rebuild in the mass frame instead of the world bbox
        m = ctx["info"]["masses"][e["mass"]]
        c = r.GetMidpoint()
        lx, ly = _to_local(m, c[0], c[1])
        wx, wy = _to_world(m, lx, ly)
        box = "{0}/roofslab_{1}_{2}".format(ctx["parent"], ctx["tag"], path.rsplit("/", 1)[-1])
        _box(stage, box, wx, wy, top - T / 2.0, sx, sy, T, m["yaw"], mat)
    else:
        box = "{0}/roofslab_{1}_{2}".format(ctx["parent"], ctx["tag"], path.rsplit("/", 1)[-1])
        _box(stage, box, (mn[0] + mx[0]) / 2.0, (mn[1] + mx[1]) / 2.0,
             top - T / 2.0, sx, sy, T, 0.0, mat)
    pr.SetActive(False)
    ctx["authored"].append(box)
    return box


def _break_box_like(ctx, e, n, timber=False, consume=0.0):
    """Fracture a kit ROOF piece: swap it for a solid slab first."""
    from . import fracture
    box = _roof_box(ctx, e, thick=(0.14 if timber else ROOF_T))
    if not box:
        return []
    out = box + "_brk"
    made = fracture.fracture_prim(ctx["stage"], box, out, n_pieces=n,
                                  rng=ctx["nrng"],
                                  mode=("plank" if timber else "uniform"),
                                  aspect=((1.6, 3.5) if timber else None),
                                  rough=0.01, verbose=False, consume=consume,
                                  consume_pool=1.6, min_volume_frac=0.0008)
    mat = ctx["mats"]["concrete"]
    for pth in made:
        r = ctx["rng"].random()
        if timber and r < 0.55:
            _bind(ctx["stage"], pth, ctx["mats"]["timber"])
        elif r < 0.4:
            _bind(ctx["stage"], pth, ctx["mats"]["dark_concrete"])
    # the rest keep the roof texture the box carried (bound on the box,
    # not inherited by fragments) — bind it explicitly
    from pxr import UsdShade
    bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(box)).ComputeBoundMaterial()[0]
    for pth in made:
        pr = ctx["stage"].GetPrimAtPath(pth)
        if pr and not UsdShade.MaterialBindingAPI(pr).GetDirectBinding().GetMaterial():
            _bind(ctx["stage"], pth, bm if bm else mat)
    return made


def r_out_of_plane(ctx, sides=1, from_storey=1, mass="main", which=None):
    """A masonry wall peels off from `from_storey` up and falls OUTWARD,
    leaving floors, partitions and the other walls standing."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    H = m["top"] - m["z0"]
    chosen = which or _pick_sides(ctx, sides)
    for side in chosen:
        ox, oy = _outward(m, side)
        for e in list(_els(ctx, mass=mass, side=side)):
            if e["role"] not in ("wall", "corner", "parapet", "parapet_corner", "balcony"):
                continue
            if e["storey"] < from_storey and e["role"] not in ("parapet", "parapet_corner"):
                continue
            # The storey just above the break line breaks PARTIALLY so the
            # surviving edge is ragged, not a course line.
            partial = None
            if e["storey"] == from_storey and e["role"] == "wall" and rng.random() < 0.6:
                partial = rng.uniform(0.15, 0.45)
            st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                            7 + rng.randrange(5), rng, nrng, ctx["mats"],
                            ctx["cache"], ctx["info"]["type"], inner_p=0.35,
                            partial=partial, consume=0.22)
            # Outward speed grows with height: the wall rotates about its
            # foot, so the top leads.
            for pth in lo:
                zf = min(1.0, max(0.0, (e["z"] + e["h"] * 0.5 - m["z0"]) / max(H, 1.0)))
                v = 0.4 + 2.4 * zf + rng.uniform(-0.2, 0.3)
                ctx["velocity"][pth] = (ox * v, oy * v, 0.15 * v)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
        _heap(ctx, m, m["z0"], 0.0, rng.uniform(0.22, 0.4), fill=False,
              sides=(side,), depth_m=rng.uniform(1.0, 1.9), tag="windrow")
        opened = set(range(from_storey, len(m["levels"])))
        # THE EDGES OF THE HOLE ARE RAGGED, NOT MODULE SEAMS: the bays either
        # side lose their near ends along a wandering line, the floor slabs
        # and the roof lose a broken strip along the open edge, and what is
        # left standing is disturbed.
        _ragged_neighbours(ctx, mass, side, opened)
        _ragged_slabs(ctx, mass, side, opened)
        r_droop(ctx, mass=mass, side=side, storeys=opened, p=0.45)
        _disturb_interior(ctx, mass, opened, side=side)
    _spall(ctx, mass, rate=0.07)


def r_soft_storey(ctx, storey=0, mass="main", lean_deg=None, crush_m=None,
                  twist_deg=0.0, offset_m=0.0):
    """The columns of `storey` fail; everything above drops by (storey
    height - crush) and leans a few degrees; that storey's walls are
    crushed into a rubble skirt."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    info = ctx["info"]
    m = info["masses"][mass]
    lv = m["levels"]
    if storey >= len(lv):
        return
    z_lo = lv[storey]
    z_hi = lv[storey + 1] if storey + 1 < len(lv) else m["top"]
    h_st = z_hi - z_lo
    crush = crush_m if crush_m is not None else rng.uniform(0.28, 0.42) * h_st
    drop = h_st - crush
    lean = lean_deg if lean_deg is not None else rng.uniform(2.5, 6.5)
    lean_side = rng.choice(["S", "E", "N", "W"])
    ox, oy = _outward(m, lean_side)

    # 1) the failed storey's walls -> rubble, pushed outward and DOWN so they
    #    start in the skirt band rather than inside the dropped mass.
    fit = ctx["fit"]
    for e in list(_els(ctx, mass=mass, storey=storey)):
        if e["role"] not in ("wall", "corner", "balcony"):
            continue
        st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                        8 + rng.randrange(5), rng, nrng, ctx["mats"],
                        ctx["cache"], info["type"], inner_p=0.5, consume=0.45)
        sx, sy = _outward(m, e["side"])
        push = rng.uniform(0.8, 2.2)
        M = _translate(sx * push, sy * push, 0.0)
        _squash(ctx["stage"], lo, z_lo, crush / h_st)
        _transform_prims(ctx["stage"], lo, M)
        for pth in lo:
            ctx["velocity"][pth] = (sx * rng.uniform(0.3, 1.2),
                                    sy * rng.uniform(0.3, 1.2), 0.0)
        ctx["loose"] += lo + st
        e["dead"] = True
    # THE COLLAR: the crushed storey's material squeezed out round the
    # perimeter, 1-3 m wide (Northridge Meadows, Antakya) — solid chunks,
    # since the shells above are foil.
    collar = _heap(ctx, m, z_lo, 0.0, 0.12, fill=False, sides=("S", "E", "N", "W"),
                   depth_m=rng.uniform(0.5, 0.9) + crush * 0.25, tag="collar")
    if storey > 0:
        # at a MID storey the collar is authored in the air beside the
        # crushed band; let it fall to the base rather than hover
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q not in set(collar)]
        ctx["loose"] += collar
    for pth in list(fit["partitions"]):
        if "_{0}_{1}_".format(mass, storey) in pth:
            ctx["loose"] += _break_box(ctx["stage"], pth, 4, rng, nrng,
                                       ctx["mats"]["plaster"], consume=0.5)
            fit["all"] = [q for q in fit["all"] if q != pth]
    # its columns -> stubs + chunks
    for cpath in fit["columns"].get((mass, storey), []):
        cx_, cy_, cz_, sx_, sy_, sz_, yaw_ = _box_dims(ctx["stage"], cpath)
        made = _break_box(ctx["stage"], cpath, 4, rng, nrng,
                          ctx["mats"]["concrete"], ctx["mats"]["dark_concrete"])
        _squash(ctx["stage"], made, z_lo, crush / h_st)
        ctx["loose"] += made
        if rng.random() < 0.6:
            _lantern(ctx, cx_, cy_, z_lo + crush * 0.5, crush * 0.6)
    # 2) everything above: translate down, lean about the far bottom edge.
    above = []
    for e in _els(ctx, mass=None):
        em = info["masses"][e["mass"]]
        if e["mass"] == mass or e["mass"].startswith(mass + "_") or (
                mass == "main" and e["mass"] != "main"):
            if e["z"] >= z_hi - 0.05 or e["role"] in ("roof",) and em["top"] >= z_hi:
                above.append(e["p"].get("prim_path"))
    for (mt, i), pth in fit["slabs"].items():
        if i > storey or (mt != mass and mt.startswith(mass)) or (mass == "main" and mt != "main"):
            above.append(pth)
    for (mt, i), cols in fit["columns"].items():
        if i > storey or (mass == "main" and mt != "main"):
            above.extend(cols)
    for pth in fit["partitions"]:
        # partition names carry the storey index: part_<mass>_<i>_<k>
        try:
            i = int(pth.rsplit("_", 2)[-2])
        except ValueError:
            i = 0
        if i > storey:
            above.append(pth)
    for (mt, i), props in fit["props"].items():
        if i > storey or (mass == "main" and mt != "main"):
            above.extend(props)
        elif i == storey and mt == mass:
            ctx["loose"] += props          # crushed with the storey
    above = [a for a in above if a]
    # pivot: bottom edge of the mass on the lean side, at the crushed height
    edge = (m["D"] if lean_side in ("S", "N") else m["W"]) / 2.0
    px, py = _to_world(m, ox * edge if lean_side in ("E", "W") else 0.0,
                       oy * edge if lean_side in ("S", "N") else 0.0)
    # axis is along the edge (perpendicular to outward, horizontal)
    ax, ay = -oy, ox
    sign = -1.0
    M = _translate(0.0, 0.0, -drop) * _rot_about((px, py, z_lo + crush),
                                                  (ax, ay, 0.0), sign * lean)
    if twist_deg or offset_m:
        # MID-STOREY SIGNATURE (Kobe, Mexico City): the upper block sits on
        # the crushed band ROTATED IN PLAN a few degrees and shifted, so the
        # facade lines break at the seam — the one thing that tells it from
        # a building that is merely shorter.
        M = M * _rot_about((m["cx"], m["cy"], z_lo + crush), (0.0, 0.0, 1.0),
                           float(twist_deg)) * _translate(ox * float(offset_m),
                                                          oy * float(offset_m), 0.0)
    _transform_prims(ctx["stage"], above, M)
    ctx["static_extra"] += above
    ctx["notes"].append("soft_storey: storey {0} crushed {1:.1f} -> {2:.1f} m, "
                        "lean {3:.1f} deg toward {4}".format(
                            storey, h_st, crush, lean, lean_side))


def _squash(stage, paths, z_floor, k):
    """Compress a set of pieces' heights above `z_floor` by factor k (moves
    their centres; does not deform them)."""
    from pxr import Gf, UsdGeom
    for pth in paths:
        pr = stage.GetPrimAtPath(pth)
        if not pr or not pr.IsValid():
            continue
        x = UsdGeom.Xformable(pr)
        ops = x.GetOrderedXformOps()
        for op in ops:
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                t = Gf.Vec3d(op.Get())
                t[2] = z_floor + (t[2] - z_floor) * k
                op.Set(t)
                break


def _rebar_tuft(ctx, at_path, z, n=3, length=(0.5, 1.3)):
    from pxr import Gf, UsdGeom
    stage = ctx["stage"]
    pr = stage.GetPrimAtPath(at_path)
    if not pr or not pr.IsValid():
        return
    xf = UsdGeom.XformCache()
    c = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
    rng = ctx["rng"]
    for k in range(n):
        a = rng.uniform(0, 2 * math.pi)
        L = rng.uniform(*length)
        tilt = rng.uniform(0.2, 0.9)
        p0 = (c[0] + rng.uniform(-0.15, 0.15), c[1] + rng.uniform(-0.15, 0.15), z)
        p1 = (p0[0] + math.cos(a) * L * tilt, p0[1] + math.sin(a) * L * tilt,
              z + L * math.sqrt(max(0.0, 1 - tilt * tilt)))
        path = "{0}/rebar_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        _cyl(stage, path, p0, p1, 0.012, ctx["mats"]["rebar"])
        ctx["authored"].append(path)


def r_pancake(ctx, mass="main", pitch_m=None):
    """Total collapse of a concrete frame: slabs stack at the base, façade
    fractures and sheds outward, roof lands on top. Wings come down with
    it."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    info = ctx["info"]
    stage = ctx["stage"]
    fit = ctx["fit"]
    all_masses = [t for t in info["masses"] if t == mass or t.startswith(mass) or mass == "main"]
    for mt in all_masses:
        m = info["masses"][mt]
        n_lv = len(m["levels"])
        pitch = pitch_m if pitch_m is not None else rng.uniform(0.55, 0.95)
        # façade -> loose chunks, pushed a little outward
        for e in list(_els(ctx, mass=mt)):
            if e["role"] in ("roof",):
                continue
            partial = None
            if e["storey"] == 0 and e["role"] == "wall" and rng.random() < 0.35:
                partial = rng.uniform(0.1, 0.3)
            st, lo = _break(stage, ctx["parent"], e, ctx["tag"],
                            9 + rng.randrange(6), rng, nrng, ctx["mats"],
                            ctx["cache"], info["type"], inner_p=0.5,
                            partial=partial)
            ox, oy = _outward(m, e["side"])
            H = max(1.0, m["top"] - m["z0"])
            for pth in lo:
                zf = (e["z"] - m["z0"]) / H
                v = 0.3 + 1.6 * zf
                ctx["velocity"][pth] = (ox * v * rng.uniform(0.5, 1.2),
                                        oy * v * rng.uniform(0.5, 1.2), 0.0)
            ctx["loose"] += lo + st
            e["dead"] = True
        # columns -> short chunks (they are what the slabs crush)
        for (cm, i), cols in fit["columns"].items():
            if cm != mt:
                continue
            for cpath in cols:
                made = _break_box(stage, cpath, 3, rng, nrng,
                                  ctx["mats"]["concrete"], ctx["mats"]["dark_concrete"])
                ctx["loose"] += made
        for (pm, i), props in fit["props"].items():
            if pm == mt:
                ctx["loose"] += props
        # PARTITIONS COME DOWN WITH THE FLOORS. Left static they stayed at
        # their storey heights over the stack — a grid of white panels in
        # mid-air on the first concrete bench.
        for pth in list(fit["partitions"]):
            if "_{0}_".format(mt) in pth:
                ctx["loose"] += _break_box(stage, pth, 4, rng, nrng,
                                           ctx["mats"]["plaster"], consume=0.5)
                fit["all"] = [q for q in fit["all"] if q != pth]
        # slabs: RE-AUTHOR as a stack (not simulated — a stack of thin boxes
        # is exactly what PhysX does worst), each with its own small tilt and
        # jitter, then the roof on top. The stack base is the mass's own z0
        # (a wing pancakes onto what the main body left).
        base = m["z0"] if mt == "main" else info["masses"]["main"]["z0"] + (
            len(info["masses"]["main"]["levels"]) * pitch + 0.4)
        stack = []
        for i in range(1, n_lv + 1):
            key = (mt, i)
            pth = fit["slabs"].get(key)
            if pth is None:
                continue
            k = i - 1
            z = base + pitch * (k + 0.5)
            tilt = rng.uniform(-4.0, 4.0)
            taxis = rng.choice(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
            jx, jy = rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6)
            self_z = m["levels"][i] - SLAB_T[info["type"]] / 2.0 if i < n_lv else m["top"] - SLAB_T[info["type"]] / 2.0
            M = _translate(jx, jy, z - self_z) * _rot_about(
                (m["cx"], m["cy"], z), taxis, tilt)
            _transform_prims(stage, [pth], M)
            stack.append(pth)
            # rebar tufts at slab edges
            for q in range(2):
                _rebar_tuft(ctx, pth, z + 0.1, n=2, length=(0.4, 1.0))
        ctx["static_extra"] += stack
        # the roof piece(s) land on top, tilted
        top_z = base + pitch * n_lv + 0.15
        for e in list(_els(ctx, mass=mt, role="roof")):
            pth = _roof_box(ctx, e)
            if not pth:
                continue
            tilt = rng.uniform(3.0, 9.0)
            M = _translate(rng.uniform(-0.8, 0.8), rng.uniform(-0.8, 0.8),
                           top_z - e["z"]) * _rot_about(
                (m["cx"], m["cy"], top_z),
                rng.choice(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))), tilt)
            _transform_prims(stage, [pth], M)
            ctx["static_extra"].append(pth)
            e["dead"] = True
        if mt == "main" and rng.random() < 0.55:
            _shaft(ctx, m)
        # a mound of crushed concrete around the stack, authored
        # RC spreads further than masonry (0.5-1.0 H) but the slabs carry
        # most of the height; the heap is the crushed-column/infill fill
        # between and around them.
        _heap(ctx, m, base, pitch * n_lv * 0.85, rng.uniform(0.3, 0.45), fill=True)
        ctx["notes"].append("pancake {0}: {1} slabs at {2:.2f} m pitch, mound to "
                            "{3:.1f} m".format(mt, len(stack), pitch,
                                               base + pitch * n_lv))


def _heap(ctx, m, base, h, spread_frac, fill=True, sides=None,
          depth_m=None, along=None, tag="heap", mat_fn=None, offset_m=0.0):
    """A heap of solid chunks — the mass of a collapse, AUTHORED.

    Fractured kit walls are open shells, so a pile made only of them reads
    as crumpled paper. A real pile is mostly small solid rubble with the
    recognisable fragments ON it, so the volume is authored as boxes and the
    fragments settle onto it. Two shapes:

      * `fill=True`:  a dome over the whole footprint (pancake / total
                      collapse) — peak `h` at the centre, falling to the
                      skirt, which reaches `spread_frac` x the mass height.
      * `fill=False`: a WINDROW along `sides` (a shed wall, a parapet):
                      `depth_m` tall at the wall line, tapering over
                      `spread_frac` x the mass height outward. `along` is a
                      (t0, t1) fraction of the side's length to cover.

    Static for the settle (triangle colliders), so loose pieces land on it.
    """
    rng = ctx["rng"]
    btype = ctx["info"]["type"]
    mats = ctx["mats"]
    made = []
    W, D = m["W"], m["D"]
    Hm = max(3.0, m["top"] - m["z0"])

    def _mat():
        if mat_fn is not None:
            return mat_fn()
        r = rng.random()
        if btype == "urm":
            return (mats["brick"] if r < 0.6 else
                    mats["mortar"] if r < 0.9 else mats["plaster"])
        return (mats["concrete"] if r < 0.5 else
                mats["dark_concrete"] if r < 0.92 else mats["plaster"])

    def _chunk(lx, ly, z, s):
        wx, wy = _to_world(m, lx, ly)
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy, z, s, s * rng.uniform(0.5, 1.0),
             s * rng.uniform(0.35, 0.7), rng.uniform(0, 180), _mat())
        made.append(path)

    if fill:
        reach = spread_frac * Hm
        RX, RY = W / 2.0 + reach, D / 2.0 + reach
        vol = (math.pi * RX * RY * h) / 2.0
        n = int(min(2600, max(120, vol * 0.55 / 0.9)))
        for k in range(n):
            u, v = rng.uniform(-1, 1), rng.uniform(-1, 1)
            r = math.hypot(u, v)
            if r > 1.0:
                continue
            zmax = h * max(0.0, 1.0 - r ** 1.6)
            sz = rng.uniform(0.35, 1.7) * (1.0 if r < 0.75 else 0.7)
            _chunk(u * RX, v * RY, base + rng.uniform(0.0, zmax) + sz * 0.15, sz)
    else:
        depth = depth_m if depth_m is not None else 1.2
        reach = max(1.5, spread_frac * Hm)
        for side in (sides or ("S",)):
            L = W if side in ("S", "N") else D
            t0, t1 = along if along else (-0.5, 0.5)
            span = (t1 - t0) * L
            n = int(max(30, span * reach * depth * 0.9 / 0.5))
            for k in range(n):
                t = rng.uniform(t0, t1) * L
                d = abs(rng.gauss(0.0, reach * 0.45)) + 0.15
                if d > reach:
                    continue
                zmax = depth * max(0.0, 1.0 - (d / reach) ** 1.3)
                d += float(offset_m)
                if side in ("S", "N"):
                    lx, ly = t, (-D / 2.0 - d if side == "S" else D / 2.0 + d)
                else:
                    lx, ly = (-W / 2.0 - d if side == "W" else W / 2.0 + d), t
                # WINDROW CHUNKS ARE SMALL — a parapet comes down as bricks
                # and mortar, and 1 m plaster cubes on the pavement read as
                # packing crates. Bigger pieces only under a peeled wall.
                sz = rng.uniform(0.18, 0.55 if depth < 1.0 else 0.95)
                _chunk(lx, ly, base + rng.uniform(0.0, zmax) + sz * 0.15, sz)
    ctx["authored"] += made
    ctx["static_extra"] += made
    return made


def _mound(ctx, m, base, h, spread_frac):
    return _heap(ctx, m, base, h, spread_frac, fill=True)


def r_masonry_collapse(ctx, mass="main", keep_stub=True):
    """URM total collapse: every wall above the ground storey fractures and
    falls (outward, top leading), the roof and every floor slab become loose
    bodies and drop into the pile. The ground storey survives as stubs."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    info = ctx["info"]
    stage = ctx["stage"]
    fit = ctx["fit"]
    for mt, m in info["masses"].items():
        H = max(1.0, m["top"] - m["z0"])
        for e in list(_els(ctx, mass=mt)):
            if e["role"] == "roof":
                # A MASONRY building's roof is a timber deck, and it comes
                # apart into boards and rafters, not into concrete plates:
                # many small pieces, a third of them lost into the heap.
                ctx["loose"] += _break_box_like(ctx, e, 30, timber=True,
                                                consume=0.45)
                e["dead"] = True
                continue
            partial = None
            if e["storey"] == 0 and keep_stub and e["role"] == "wall":
                partial = rng.uniform(0.25, 0.6)
            st, lo = _break(stage, ctx["parent"], e, ctx["tag"],
                            8 + rng.randrange(5), rng, nrng, ctx["mats"],
                            ctx["cache"], info["type"], inner_p=0.45,
                            partial=partial, consume=0.3)
            ox, oy = _outward(m, e["side"])
            for pth in lo:
                zf = min(1.0, max(0.0, (e["z"] - m["z0"]) / H))
                v = 0.3 + 1.8 * zf
                ctx["velocity"][pth] = (ox * v * rng.uniform(0.4, 1.1),
                                        oy * v * rng.uniform(0.4, 1.1), 0.0)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
        for (sm, i), pth in fit["slabs"].items():
            if sm == mt and pth:
                # A JOISTED TIMBER FLOOR COMES APART INTO BOARDS AND JOISTS.
                # `plank` seeds give long rectangular cells; a third are lost
                # into the heap (they are under it, not gone).
                # HALF THE DECK IS LOST INTO THE HEAP. At 0.35 a ten-storey
                # brick block's pile was a pale mountain of floorboards over
                # the brick; a real masonry collapse is brick with timber IN
                # it, not the other way round.
                made = _break_box(stage, pth, 30 + rng.randrange(10), rng, nrng,
                                  ctx["mats"]["timber"], ctx["mats"]["plaster"], 0.15,
                                  mode="plank", aspect=(2.5, 6.0), consume=0.55)
                ctx["loose"] += made
        for pth in fit["partitions"]:
            if "_{0}_".format(mt) in pth:
                ctx["loose"] += _break_box(stage, pth, 8, rng, nrng,
                                           ctx["mats"]["plaster"], consume=0.45)
        for (pm, i), props in fit["props"].items():
            if pm == mt:
                ctx["loose"] += props
        # THE PILE IS H/3 (FEMA's 0.33 air-space factor; Amatrice LiDAR gives
        # ~5 m heaps for 2-4 storey stone). Masonry spreads 0.4-0.7 H.
        _heap(ctx, m, m["z0"], H * 0.28, rng.uniform(0.2, 0.34), fill=True)


def r_tilt_sink(ctx, tilt_deg=8.0, sink_m=1.0, azimuth=None):
    """Foundation failure: the whole building, fit-out included, rotates
    about a base edge and sinks. Stays intact. Soil heave ring authored."""
    rng = ctx["rng"]
    info = ctx["info"]
    m = info["masses"]["main"]
    side = azimuth or rng.choice(["S", "E", "N", "W"])
    ox, oy = _outward(m, side)
    edge = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
    px, py = _to_world(m, ox * edge if side in ("E", "W") else 0.0,
                       oy * edge if side in ("S", "N") else 0.0)
    ax, ay = -oy, ox
    paths = [e["p"].get("prim_path") for e in _els(ctx)] + list(ctx["fit"]["all"])
    paths = [p for p in paths if p]
    M = _rot_about((px, py, m["z0"]), (ax, ay, 0.0), -abs(tilt_deg)) * _translate(
        0.0, 0.0, -abs(sink_m))
    _transform_prims(ctx["stage"], paths, M)
    ctx["static_extra"] += paths
    # HEAVED SOIL: one continuous berm MESH round the footprint, not boxes.
    # The ground is squeezed up as the footing goes down, so the crest is
    # highest on the sunk side, dies away round the flanks, and is absent on
    # the lifted side, where the footing has pulled clear instead. Scattered
    # slabs of soil read as cardboard; a ridge reads as earth.
    _berm(ctx, m, side, crest_m=0.3 + 0.4 * min(1.0, abs(sink_m)),
          reach_m=1.9)
    ctx["notes"].append("tilt_sink: {0:.1f} deg toward {1}, sunk {2:.2f} m".format(
        tilt_deg, side, sink_m))


def _berm(ctx, m, hot_side, crest_m=0.5, reach_m=2.5, crest_d=0.7, seg_m=1.5):
    """A ring of heaved soil round a mass: three concentric loops (under the
    wall line, the crest, the toe) triangulated into two quad strips, with
    the crest height varying by side and band-limited noise along it."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    rng = ctx["rng"]
    W, D = m["W"], m["D"]
    # perimeter parameter u in [0, 1): S (from SW), E, N, W
    n = max(24, int((2 * W + 2 * D) / seg_m))
    hot = {"S": 0.125, "E": 0.375, "N": 0.625, "W": 0.875}[hot_side]
    ph = [rng.uniform(0, 6.28) for _ in range(3)]

    def _at(u):
        u = u % 1.0
        P = 2 * W + 2 * D
        d = u * P
        if d < W:
            return (-W / 2.0 + d, -D / 2.0), (0.0, -1.0)
        d -= W
        if d < D:
            return (W / 2.0, -D / 2.0 + d), (1.0, 0.0)
        d -= D
        if d < W:
            return (W / 2.0 - d, D / 2.0), (0.0, 1.0)
        d -= W
        return (-W / 2.0, D / 2.0 - d), (-1.0, 0.0)

    def _h(u):
        # angular distance to the hot side's midpoint, 0..0.5
        a = abs(((u - hot) + 0.5) % 1.0 - 0.5)
        base = max(0.0, 1.0 - a / 0.42) ** 1.4          # 1 at hot side, 0 opposite
        noise = (math.sin(u * 6.28 * 5 + ph[0]) * 0.5
                 + math.sin(u * 6.28 * 11 + ph[1]) * 0.3
                 + math.sin(u * 6.28 * 23 + ph[2]) * 0.2)
        return crest_m * base * (0.75 + 0.35 * noise)

    pts, rings = [], []
    for k in range(n):
        u = k / float(n)
        (px, py), (nx, ny) = _at(u)
        # corner rounding: normals at the corners are averaged by the strip
        h = _h(u)
        inner = (px - nx * 0.4, py - ny * 0.4, -0.02)
        crest = (px + nx * crest_d, py + ny * crest_d, h)
        toe = (px + nx * reach_m * (0.6 + 0.4 * (h / max(crest_m, 1e-3))),
               py + ny * reach_m * (0.6 + 0.4 * (h / max(crest_m, 1e-3))), -0.02)
        for lx, ly, z in (inner, crest, toe):
            wx, wy = _to_world(m, lx, ly)
            pts.append(Gf.Vec3f(wx, wy, m["z0"] + z))
        rings.append(len(pts) - 3)
    faces, counts = [], []
    for k in range(n):
        a, b = rings[k], rings[(k + 1) % n]
        for j in range(2):
            faces += [a + j, b + j, b + j + 1, a + j + 1]
            counts.append(4)
    path = "{0}/berm_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    _bind(ctx["stage"], path, ctx["mats"]["soil"])
    ctx["authored"].append(path)
    return path


# ---------------------------------------------------------------------------
# FOUNDATION FAILURE: settle, lean, overturn — the building stays whole
# ---------------------------------------------------------------------------
# Research (scene_gen/_plans/earthquake_research.md, s3.13 and ladder L8):
# Adapazari 1999 settlements 0.004-1.6 m (max 3.4 m), significant tilt only
# on slender H/B > 2 blocks free on one side, raft levered out on the high
# side with wet silt squeezed out on the low side; Christchurch 40-50 cm and
# 0.4-0.5 deg; Niigata 1964 up to 80 deg, one block fully over, structurally
# undamaged; Antakya 2023 whole-body overturning with columns uprooted
# unbroken. So: the shell is NOT broken here. What is authored is what the
# ground did — the raft, the berm, the silt, the sand boils, the buckled
# pavement — and, for an overturn, what the landing did to the landing side.

RAFT_T = 0.8


def _raft(ctx, m, tag="raft"):
    """The foundation raft: a concrete slab under the footprint, top just
    below grade so it is invisible until a tilt lifts one edge out."""
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
    _box(ctx["stage"], path, m["cx"], m["cy"], m["z0"] - 0.06 - RAFT_T / 2.0,
         m["W"] + 1.2, m["D"] + 1.2, RAFT_T, m["yaw"], ctx["mats"]["dark_concrete"])
    ctx["authored"].append(path)
    ctx["static_extra"].append(path)
    return path


def _ejecta(ctx, m, n, reach_frac=1.2, bias_side=None):
    """Sand boils: low flat fans of wet silt on the pavement round the
    building, thicker toward `bias_side`. Flat discs, 12-sided, wobbled."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    rng = ctx["rng"]
    H = max(3.0, m["top"] - m["z0"])
    made = []
    for k in range(n):
        side = bias_side if (bias_side and rng.random() < 0.6) else rng.choice(["S", "E", "N", "W"])
        ox, oy = _outward(m, side)
        L = m["W"] if side in ("S", "N") else m["D"]
        t = rng.uniform(-0.6, 0.6) * L
        d = rng.uniform(0.5, reach_frac * H * 0.5)
        if side in ("S", "N"):
            lx, ly = t, (-m["D"] / 2.0 - d if side == "S" else m["D"] / 2.0 + d)
        else:
            lx, ly = (-m["W"] / 2.0 - d if side == "W" else m["W"] / 2.0 + d), t
        cx, cy = _to_world(m, lx, ly)
        r = rng.uniform(1.2, 3.8)
        ph = rng.uniform(0, 6.28)
        # A LOW DOME, NOT A STARFISH: 24 sides, a gentle two-harmonic wobble,
        # and the centre raised so the fan catches the light as a mound.
        N = 24
        pts = [Gf.Vec3f(cx, cy, m["z0"] + 0.02 + 0.06 * r)]
        for i in range(N):
            a = 6.283 * i / N
            rr = r * (1.0 + 0.08 * math.sin(2 * a + ph) + 0.05 * math.sin(5 * a + ph * 2))
            pts.append(Gf.Vec3f(cx + rr * math.cos(a), cy + rr * math.sin(a), m["z0"] + 0.02))
        faces, counts = [], []
        for i in range(N):
            faces += [0, 1 + i, 1 + (i + 1) % N]
            counts.append(3)
        path = "{0}/boil_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
        mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        _bind(ctx["stage"], path, ctx["mats"]["soil"])
        made.append(path)
    ctx["authored"] += made
    return made


def _buckled_pavement(ctx, m, n, sides=None, lift=(0.1, 0.35), tilt=(4.0, 14.0)):
    """Pavement slabs round the footing pushed up and tipped — the kerb
    line no longer flat. Authored boxes with the concrete look."""
    rng = ctx["rng"]
    made = []
    for k in range(n):
        side = rng.choice(list(sides) if sides else ["S", "E", "N", "W"])
        ox, oy = _outward(m, side)
        L = m["W"] if side in ("S", "N") else m["D"]
        t = rng.uniform(-0.5, 0.5) * L
        d = rng.uniform(0.4, 2.2)
        if side in ("S", "N"):
            lx, ly = t, (-m["D"] / 2.0 - d if side == "S" else m["D"] / 2.0 + d)
        else:
            lx, ly = (-m["W"] / 2.0 - d if side == "W" else m["W"] / 2.0 + d), t
        wx, wy = _to_world(m, lx, ly)
        sx, sy = rng.uniform(1.4, 2.6), rng.uniform(1.2, 2.2)
        path = "{0}/kerb_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy, m["z0"] + rng.uniform(*lift), sx, sy, 0.14,
             m["yaw"] + rng.uniform(-15, 15), ctx["mats"]["concrete"])
        # tip it about a horizontal axis
        a = rng.uniform(0, 6.28)
        M = _rot_about((wx, wy, m["z0"]), (math.cos(a), math.sin(a), 0.0),
                       rng.uniform(*tilt))
        _transform_prims(ctx["stage"], [path], M)
        made.append(path)
    ctx["authored"] += made
    ctx["static_extra"] += made
    return made


def _everything(ctx):
    """Every prim that IS the building: kit pieces still alive + fit-out."""
    paths = [e["p"].get("prim_path") for e in _els(ctx)] + list(ctx["fit"]["all"])
    return [p for p in paths if p]


def r_settlement(ctx, sink_m=None):
    """Level settlement: the whole building sinks 0.3-1.6 m. Sills at grade,
    the pavement heaved and cracked round it, a few sand boils."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    sink = sink_m if sink_m is not None else rng.uniform(0.35, 1.6)
    raft = _raft(ctx, m)
    paths = _everything(ctx) + [raft]
    _transform_prims(ctx["stage"], paths, _translate(0.0, 0.0, -sink))
    ctx["static_extra"] += paths
    for sd in ("S", "E", "N", "W"):
        _berm(ctx, m, sd, crest_m=0.15 + 0.25 * min(1.0, sink / 1.2), reach_m=1.6)
    _buckled_pavement(ctx, m, int((m["W"] + m["D"]) * 0.35))
    _ejecta(ctx, m, 2 + rng.randrange(3))
    ctx["notes"].append("settlement: sunk {0:.2f} m".format(sink))


def r_tilt_severe(ctx, tilt_deg=None, sink_m=None, side=None):
    """Severe lean, 10-30 deg, on a raft that levers out of the ground on
    the high side while silt is squeezed out on the low side (Adapazari)."""
    rng = ctx["rng"]
    info = ctx["info"]
    m = info["masses"]["main"]
    tilt = tilt_deg if tilt_deg is not None else rng.uniform(10.0, 30.0)
    sink = sink_m if sink_m is not None else rng.uniform(0.5, 2.0)
    side = side or rng.choice(["S", "E", "N", "W"])
    ox, oy = _outward(m, side)
    edge = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
    px, py = _to_world(m, ox * edge if side in ("E", "W") else 0.0,
                       oy * edge if side in ("S", "N") else 0.0)
    ax, ay = -oy, ox
    raft = _raft(ctx, m)
    paths = _everything(ctx) + [raft]
    M = _rot_about((px, py, m["z0"]), (ax, ay, 0.0), -abs(tilt)) * _translate(0.0, 0.0, -abs(sink))
    _transform_prims(ctx["stage"], paths, M)
    ctx["static_extra"] += paths
    # the low side: silt squeezed out as a windrow of mud, a berm, boils
    _heap(ctx, m, m["z0"], 0.0, 0.12, fill=False, sides=(side,),
          depth_m=0.5 + 0.5 * min(1.0, sink / 1.5), tag="silt",
          mat_fn=lambda: ctx["mats"]["soil"])
    _berm(ctx, m, side, crest_m=0.4 + 0.5 * min(1.0, sink / 1.5), reach_m=2.4)
    _ejecta(ctx, m, 3 + rng.randrange(4), bias_side=side)
    _buckled_pavement(ctx, m, int((m["W"] + m["D"]) * 0.3), sides=(side,))
    # the high side: a pit where the raft came out — a dark soil slab at
    # grade so the void reads from above, plus torn pavement along it
    opp = _opposite(side)
    _buckled_pavement(ctx, m, int((m["W"] if opp in ("S", "N") else m["D"]) * 0.5),
                      sides=(opp,), lift=(0.05, 0.2), tilt=(8.0, 22.0))
    ctx["notes"].append("tilt_severe: {0:.0f} deg toward {1}, sunk {2:.2f} m".format(
        tilt, side, sink))


def r_overturn(ctx, angle_deg=None, side="S"):
    """Whole-body overturning, 60-90 deg about the base edge on `side`: the
    shell stays whole (Antakya: columns uprooted unbroken, facade legible on
    its side), the raft comes with it, the landing side's parapet and top
    course crush, contents tumble, and a windrow marks the landing line."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    info = ctx["info"]
    m = info["masses"]["main"]
    angle = angle_deg if angle_deg is not None else rng.uniform(62.0, 90.0)
    ox, oy = _outward(m, side)
    edge = (m["D"] if side in ("S", "N") else m["W"]) / 2.0
    px, py = _to_world(m, ox * edge if side in ("E", "W") else 0.0,
                       oy * edge if side in ("S", "N") else 0.0)
    ax, ay = -oy, ox
    H = info["H"]
    # 1) what the landing crushes: the parapet, corners and top storey on
    #    the landing side (they hit first), and the roof strip along it
    top = len(m["levels"]) - 1
    for e in list(_els(ctx, mass="main", side=side)):
        if e["role"] in ("parapet", "parapet_corner") or (
                e["role"] in ("wall", "corner", "balcony") and e["storey"] == top):
            st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                            7 + rng.randrange(4), rng, nrng, ctx["mats"],
                            ctx["cache"], info["type"], inner_p=0.4, consume=0.25)
            ctx["loose"] += lo + st
            e["dead"] = True
    carried = []          # roof remainders ride with the shell (they are
    #                       not kit pieces any more, so `_everything` would
    #                       miss them and leave a roof plate floating where
    #                       the building used to stand)
    for e in list(_els(ctx, role="roof")):
        em = ctx["info"]["masses"][e["mass"]]
        d = rng.uniform(1.0, 2.5)
        box = _roof_box(ctx, e)
        if not box:
            continue
        if e["mass"] == "main":
            rem, strip = _split_strip(ctx, box, em, side, d + 0.8, ctx["mats"]["concrete"])
            st, lo = _break_split(ctx, strip, 8 + rng.randrange(4),
                                  _edge_judge(em, side, d, rng),
                                  lambda: ctx["mats"]["concrete"], min_volume_frac=0.0006)
            ctx["loose"] += lo + st
        else:
            rem = box          # a wing's roof goes over whole
        carried.append(rem)
        ctx["authored"].append(rem)
        e["dead"] = True
    # contents tumble inside the tipped shell
    for (mt, i), props in ctx["fit"]["props"].items():
        ctx["loose"] += props
    ctx["fit"]["all"] = [q for q in ctx["fit"]["all"] if q not in set(ctx["loose"])]
    # 2) the rigid body: everything else, raft included, about the base edge
    raft = _raft(ctx, m)
    paths = _everything(ctx) + [raft] + carried
    M = _rot_about((px, py, m["z0"]), (ax, ay, 0.0), -abs(angle))
    _transform_prims(ctx["stage"], paths, M)
    _transform_prims(ctx["stage"], [q for q in ctx["loose"]], M)
    ctx["static_extra"] += paths
    # the loose bits get an outward-and-down shove so they clear the shell
    for q in ctx["loose"]:
        ctx["velocity"][q] = (ox * rng.uniform(0.5, 2.0), oy * rng.uniform(0.5, 2.0), -1.0)
    # 3) the ground: a torn footing pit on the pivot edge (heaved earth
    #    both sides of the raft line), the landing windrow at distance ~H
    _berm(ctx, m, side, crest_m=0.6, reach_m=2.0)
    _berm(ctx, m, _opposite(side), crest_m=0.5, reach_m=2.2)
    land = H * math.sin(math.radians(angle)) if angle < 89.0 else H
    _heap(ctx, m, m["z0"], 0.0, 0.10, fill=False, sides=(side,),
          depth_m=rng.uniform(0.6, 1.2), tag="landing", offset_m=max(0.0, land - 2.5))
    _ejecta(ctx, m, 2 + rng.randrange(3), bias_side=_opposite(side))
    _buckled_pavement(ctx, m, int((m["W"] + m["D"]) * 0.25))
    ctx["notes"].append("overturn: {0:.0f} deg toward {1} (H {2:.0f} m)".format(angle, side, H))


def r_mid_storey(ctx, mass="main", storey=None):
    """Intermediate-storey collapse: a storey 1..n-2 crushes, the block
    above drops onto it twisted 2-8 deg in plan and offset 0.3-2 m."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    n = len(m["levels"])
    if n < 3:
        return r_soft_storey(ctx, storey=0, mass=mass)
    k = storey if storey is not None else rng.randrange(1, n - 1)
    r_soft_storey(ctx, storey=k, mass=mass, lean_deg=rng.uniform(1.5, 4.0),
                  twist_deg=rng.uniform(2.0, 8.0) * rng.choice((-1, 1)),
                  offset_m=rng.uniform(0.3, 2.0))
    ctx["notes"][-1] = ctx["notes"][-1].replace("soft_storey", "mid_storey")


def _droop_strip(ctx, slab_path, m, side, depth_m, angle_deg):
    """Hinge the outer `depth_m` of a slab down by `angle_deg` about the line
    where it meets the remainder — the USAR "lean-to" / "V" floor. Returns
    (remainder, strip)."""
    rem, strip = _split_strip(ctx, slab_path, m, side, depth_m, ctx["mats"]["concrete"])
    W, D = m["W"], m["D"]
    # hinge line: the strip's inner edge, in local coords
    along_x = side in ("E", "W")
    full = (W if along_x else D) - 2 * WALL_INSET
    sgn = 1.0 if side in ("E", "N") else -1.0
    hinge = sgn * (full / 2.0 - min(depth_m, full * 0.6))
    hx, hy = (_to_world(m, hinge, 0.0) if along_x else _to_world(m, 0.0, hinge))
    ox, oy = _outward(m, side)
    ax, ay = -oy, ox
    cx, cy, cz, sx, sy, sz, yaw = _box_dims(ctx["stage"], strip)
    M = _rot_about((hx, hy, cz + sz / 2.0), (ax, ay, 0.0), abs(angle_deg))
    _transform_prims(ctx["stage"], [strip], M)
    ctx["static_extra"] += [rem, strip]
    ctx["authored"] += [rem, strip]
    return rem, strip


def r_droop(ctx, mass="main", side="S", storeys=None, p=0.6):
    """Floors at an opened side droop 12-35 deg (or fold into a V when the
    strip is deep). Run AFTER out_of_plane / corner_fail on the same side."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    fit = ctx["fit"]
    for (mt, i), pth in list(fit["slabs"].items()):
        if mt != mass or not pth or (storeys is not None and i not in storeys):
            continue
        if rng.random() >= p:
            continue
        depth = rng.uniform(0.25, 0.5) * (m["D"] if side in ("S", "N") else m["W"])
        rem, strip = _droop_strip(ctx, pth, m, side, depth, rng.uniform(12.0, 35.0))
        fit["slabs"][(mt, i)] = rem
        fit["all"] = [q for q in fit["all"] if q != pth] + [rem, strip]
        _rebar_tuft(ctx, strip, m["levels"][i] if i < len(m["levels"]) else m["top"], n=3)


def r_balcony_fail(ctx, mass="main", frac=0.5):
    """Cantilever balconies snap at the wall line: most hang at 20-60 deg,
    the rest drop (Turkiye 2023 'cikma' failures)."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    bals = list(_els(ctx, mass=mass, role="balcony"))
    if not bals:
        ctx["notes"].append("balcony_fail: no balconies on this style")
        return
    n_hang = n_drop = 0
    for e in bals:
        if rng.random() >= frac:
            continue
        path = e["p"].get("prim_path")
        if not path:
            continue
        ox, oy = _outward(m, e["side"])
        ax, ay = -oy, ox
        # hinge at the wall line, at the balcony's floor level
        hx, hy = e["x"], e["y"]
        if rng.random() < 0.65:
            # nearly vertical: the kit balcony is a storey-high module, and
            # at 30 deg it read as an open flap; hanging on its bars it lies
            # flat down the face (research: "hanging vertically")
            M = _rot_about((hx, hy, e["z"]), (ax, ay, 0.0), rng.uniform(65.0, 88.0))
            _transform_prims(ctx["stage"], [path], M)
            ctx["static_extra"].append(path)
            n_hang += 1
        else:
            ctx["loose"].append(path)
            ctx["velocity"][path] = (ox * 0.5, oy * 0.5, 0.0)
            n_drop += 1
        e["dead"] = True
    ctx["notes"].append("balcony_fail: {0} hanging, {1} dropped of {2}".format(
        n_hang, n_drop, len(bals)))


def r_roof_hole(ctx, mass="main", frac=None):
    """Roof diaphragm collapse: a 20-50 % patch of the roof drops through
    onto the top floor, which breaks under it — the nadir 'major damage'
    cue (Copernicus): a dark hole in an otherwise regular roof plane."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    fit = ctx["fit"]
    W, D = m["W"], m["D"]
    area = frac if frac is not None else rng.uniform(0.2, 0.5)
    hw = W * math.sqrt(area) * rng.uniform(0.7, 1.3) / 2.0
    hd = (W * D * area) / (4.0 * hw)
    hw, hd = min(hw, W * 0.45), min(hd, D * 0.45)
    hcx = rng.uniform(-W / 2.0 + hw + 1.0, W / 2.0 - hw - 1.0)
    hcy = rng.uniform(-D / 2.0 + hd + 1.0, D / 2.0 - hd - 1.0)
    wob = _wander(rng, 0.8)

    def judge(c):
        lx, ly = _to_local(m, c[0], c[1])
        return (abs(lx - hcx) < hw + wob(ly / 5.0)) and (abs(ly - hcy) < hd + wob(lx / 5.0))

    n_hit = 0
    for e in list(_els(ctx, mass=mass, role="roof")):
        # only tiles that actually reach INTO the hole are touched; a 6 m
        # margin fractured most of the roof and the crack mosaic came back
        if abs(e["lx"] - hcx) > hw + 2.6 or abs(e["ly"] - hcy) > hd + 2.6:
            continue
        box = _roof_box(ctx, e, thick=(0.14 if ctx["info"]["type"] == "urm" else ROOF_T))
        if not box:
            continue
        from pxr import UsdShade
        bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(box)).ComputeBoundMaterial()[0]
        st, lo = _break_split(ctx, box, 14 + rng.randrange(6), judge,
                              lambda: (ctx["mats"]["timber"] if ctx["info"]["type"] == "urm"
                                       else ctx["mats"]["concrete"]),
                              min_volume_frac=0.0005,
                              mode=("plank" if ctx["info"]["type"] == "urm" else "uniform"),
                              aspect=((1.5, 4.0) if ctx["info"]["type"] == "urm" else None))
        if bm:
            for q in st:
                _bind(ctx["stage"], q, bm)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        ctx["authored"] += st
        n_hit += len(lo)
        e["dead"] = True
    # the top floor under the hole gives way too (it took the roof's weight)
    top = len(m["levels"]) - 1
    pth = fit["slabs"].get((mass, top))
    if pth and rng.random() < 0.7:
        wob2 = _wander(rng, 0.6)

        def judge2(c):
            lx, ly = _to_local(m, c[0], c[1])
            return (abs(lx - hcx) < hw * 0.8 + wob2(ly / 5.0)) and (abs(ly - hcy) < hd * 0.8 + wob2(lx / 5.0))
        st, lo = _break_split(ctx, pth, 12 + rng.randrange(6), judge2,
                              lambda: ctx["mats"]["timber"] if ctx["info"]["type"] == "urm" else ctx["mats"]["concrete"],
                              min_volume_frac=0.0006)
        fit["slabs"][(mass, top)] = None
        fit["all"] = [q for q in fit["all"] if q != pth] + st
        ctx["loose"] += lo
        ctx["static_extra"] += st
    _disturb_interior(ctx, mass, {top})
    ctx["notes"].append("roof_hole: {0:.0f}% of the roof, {1} pieces down".format(area * 100, n_hit))


def r_storey_collapse(ctx, mass="main"):
    """DG4 concrete: a soft storey two times in three, a mid-storey
    otherwise (Northridge/Antakya vs Kobe/Mexico City)."""
    if ctx["rng"].random() < 0.66 or len(ctx["info"]["masses"][mass]["levels"]) < 4:
        return r_soft_storey(ctx, storey=0, mass=mass)
    return r_mid_storey(ctx, mass=mass)


# ---------------------------------------------------------------------------
# BATCH C: surface scars, column lanterns, shafts, rooftop plant, signage
# ---------------------------------------------------------------------------
def _piece_frame(e):
    """(origin_x, origin_y, yaw_rad, width, height, outward_depth) of a kit
    wall piece in world: pieces are pivoted at their left end on the wall
    line, run along local +X, and their art sits at local -Y (outward)."""
    from detail import urban_building as ub
    meas = ub.PIECES.get(e["name"])
    if not meas:
        return None
    sx, sy, sz, xmin, ymin, zmin = meas
    frame = ub._kit(e["name"])[1]
    if frame == "dw":
        # along +Y centred, outward +X: treat width = sy, outward = +x
        return (e["x"], e["y"], math.radians(e["yaw"] - 90.0), sy, sz, -abs(xmin) - 0.02, True)
    return (e["x"], e["y"], math.radians(e["yaw"]), sx, sz, ymin - 0.02, False)


def _face_xy(fr, u):
    """World (x, y) of a point `u` along a wall piece, on its OUTER face."""
    ox, oy, yaw, width, height, depth, dw = fr
    ca, sa = math.cos(yaw), math.sin(yaw)
    if dw:
        u = u - width / 2.0
    d = -depth
    return ox + ca * u + sa * d, oy + sa * u - ca * d


def _on_face(fr, u, v, w_along, w_up, ctx, mat, kind="patch", thick=0.03):
    """A thin slab flush on a wall piece's outer face at (u along, v up)."""
    ox, oy, yaw, width, height, depth, dw = fr
    cx, cy = _face_xy(fr, u)
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], kind, ctx["tag"], _uid(ctx))
    _box(ctx["stage"], path, cx, cy, v, w_along, thick, w_up, math.degrees(yaw), mat)
    ctx["authored"].append(path)
    return path


def _face_patch(fr, u, v, ra, rv, ctx, mat, kind="scar", proud=0.02):
    """An irregular polygon (10-14 sides, two-harmonic wobble) lying in the
    wall plane, a hair proud of the face — a plaster-loss patch. Rectangles
    read as signs; these read as damage."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    rng = ctx["rng"]
    ox, oy, yaw, width, height, depth, dw = fr
    ca, sa = math.cos(yaw), math.sin(yaw)
    nx, ny = sa, -ca
    cx, cy = _face_xy(fr, u)
    cx, cy = cx + nx * proud, cy + ny * proud
    N = rng.randrange(10, 15)
    ph = rng.uniform(0, 6.28)
    pts = []
    for i in range(N):
        a = 6.283 * i / N
        w = 1.0 + 0.22 * math.sin(2 * a + ph) + 0.14 * math.sin(3 * a + ph * 1.7)
        du, dv = ra * w * math.cos(a), rv * w * math.sin(a)
        pts.append(Gf.Vec3f(cx + ca * du, cy + sa * du, v + dv))
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], kind, ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([N]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(list(range(N))))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    _bind(ctx["stage"], path, mat)
    ctx["authored"].append(path)
    return path


def r_facade_scars(ctx, frac=0.3, mass=None):
    """DG1-2 on standing walls: plaster-loss patches showing the brick or
    concrete behind, and diagonal X-cracks across piers. Geometry, not
    decals, so it needs no cutout-opacity flag; sized to read at 20-60 m."""
    rng = ctx["rng"]
    btype = ctx["info"]["type"]
    mats = ctx["mats"]
    n = 0
    for e in list(_els(ctx, mass=mass, role="wall")):
        if rng.random() >= frac:
            continue
        fr = _piece_frame(e)
        if not fr:
            continue
        ox, oy, yaw, width, height, depth, dw = fr
        if width < 1.5 or height < 2.0:
            continue
        # 1-2 finish-loss patches: what is UNDER the finish shows. A rendered
        # concrete frame is plastered brick infill (Turkiye: red under the
        # render); a brick or stone wall loses face brick and shows the dark
        # mortar bed behind it.
        inner = mats["dark_concrete"] if btype == "urm" else mats["brick"]
        for k in range(1):
            u = rng.uniform(0.5, width - 0.5)
            v = e["z"] + rng.uniform(0.6, height - 0.6)
            _face_patch(fr, u, v, rng.uniform(0.3, 0.8), rng.uniform(0.25, 0.6), ctx, inner)
        # an X-crack across the pier: two thin dark bars at +-40 deg
        if rng.random() < 0.7:
            u = rng.uniform(0.6, width - 0.6)
            v = e["z"] + rng.uniform(0.8, height - 0.8)
            L = rng.uniform(1.0, min(2.2, height * 0.8))
            for sgn in (1, -1):
                pth = _on_face(fr, u, v, L, 0.045, ctx, mats["crack"], "crack", thick=0.02)
                # tilt the bar in the wall plane about its centre
                ca, sa = math.cos(yaw), math.sin(yaw)
                cxw = ox + ca * (u - (width / 2.0 if dw else 0.0)) + sa * (-depth)
                cyw = oy + sa * (u - (width / 2.0 if dw else 0.0)) - ca * (-depth)
                M = _rot_about((cxw, cyw, v), (sa, -ca, 0.0), sgn * rng.uniform(35.0, 50.0))
                _transform_prims(ctx["stage"], [pth], M)
        n += 1
    ctx["notes"].append("facade_scars: {0} wall piece(s) scarred".format(n))


def _lantern(ctx, x, y, z0, h, n_bars=None):
    """A spalled column head: the longitudinal bars buckled outward into a
    lantern above the crushed concrete (EMS-98 G3-G4 'buckling of rods')."""
    rng = ctx["rng"]
    nb = n_bars or rng.randrange(4, 7)
    r0 = 0.16
    for k in range(nb):
        a = 6.283 * k / nb + rng.uniform(-0.2, 0.2)
        bulge = rng.uniform(0.18, 0.32)
        p0 = (x + r0 * math.cos(a), y + r0 * math.sin(a), z0)
        p1 = (x + (r0 + bulge) * math.cos(a), y + (r0 + bulge) * math.sin(a), z0 + h * 0.5)
        p2 = (x + r0 * 0.6 * math.cos(a), y + r0 * 0.6 * math.sin(a), z0 + h)
        for q0, q1 in ((p0, p1), (p1, p2)):
            path = "{0}/bar_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
            _cyl(ctx["stage"], path, q0, q1, 0.014, ctx["mats"]["rebar"])
            ctx["authored"].append(path)


def _shaft(ctx, m, h_frac=None):
    """The lift / stair shaft that survives a pancake as a lone concrete
    tower (CTV building, Christchurch)."""
    rng = ctx["rng"]
    H = m["top"] - m["z0"]
    h = H * (h_frac if h_frac is not None else rng.uniform(0.45, 0.8))
    lx = rng.uniform(-m["W"] / 2.0 + 3.0, m["W"] / 2.0 - 3.0)
    ly = rng.uniform(-m["D"] / 2.0 + 3.0, m["D"] / 2.0 - 3.0)
    wx, wy = _to_world(m, lx, ly)
    sx, sy = rng.uniform(2.6, 3.4), rng.uniform(2.6, 5.0)
    t = 0.25
    paths = []
    # four walls, so it reads hollow from above
    for dx, dy, wsx, wsy in ((sx / 2.0, 0, t, sy), (-sx / 2.0, 0, t, sy),
                             (0, sy / 2.0, sx, t), (0, -sy / 2.0, sx, t)):
        a = math.radians(m["yaw"])
        px = wx + dx * math.cos(a) - dy * math.sin(a)
        py = wy + dx * math.sin(a) + dy * math.cos(a)
        path = "{0}/shaft_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, px, py, m["z0"] + h / 2.0, wsx, wsy, h, m["yaw"],
             ctx["mats"]["dark_concrete"])
        paths.append(path)
    # the ragged top: fracture the top metre of each wall
    from . import fracture
    for path in list(paths):
        st, lo = fracture.fracture_partial(
            ctx["stage"], path, path + "_brk", n_pieces=6, rng=ctx["nrng"],
            cut_frac=rng.uniform(0.82, 0.92), mode="uniform", rough=0.01,
            consume=0.3, min_volume_frac=0.001)
        for q in st + lo:
            _bind(ctx["stage"], q, ctx["mats"]["dark_concrete"])
        ctx["static_extra"] += st
        ctx["loose"] += lo
    for k in range(rng.randrange(2, 5)):
        _rebar_tuft(ctx, paths[0], m["z0"] + h * 0.9, n=2, length=(0.6, 1.4))
    ctx["authored"] += paths
    return paths


# Rooftop plant: authored water tanks (cylinder on legs) and the kit's AC
# unit. Tanks are the NYC roof signature and they tip in a quake.
_AC_UNIT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
            "ModernCityEnvironment01/Meshes/SM_airConditioner01.usd")


def _tank(ctx, x, y, z, r=1.2, h=2.6, yaw=0.0):
    """A cylindrical water tank on four legs; returns [tank, legs...]."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    rng = ctx["rng"]
    paths = []
    leg_h = 1.2
    for k in range(4):
        a = 0.785 + 1.571 * k + math.radians(yaw)
        path = "{0}/tankleg_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, x + (r * 0.75) * math.cos(a), y + (r * 0.75) * math.sin(a),
             z + leg_h / 2.0, 0.12, 0.12, leg_h, yaw, ctx["mats"]["rebar"])
        paths.append(path)
    N = 18
    pts, faces, counts = [], [], []
    z0, z1 = leg_h, leg_h + h
    for k in range(N):
        a = 6.283 * k / N
        pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), z0))
        pts.append(Gf.Vec3f(r * math.cos(a), r * math.sin(a), z1))
    for k in range(N):
        a, b = 2 * k, 2 * ((k + 1) % N)
        faces += [a, b, b + 1, a + 1]
        counts.append(4)
    top = len(pts)
    pts.append(Gf.Vec3f(0.0, 0.0, z1 + 0.35))
    for k in range(N):
        faces += [2 * k + 1, 2 * ((k + 1) % N) + 1, top]
        counts.append(3)
    bot = len(pts)
    pts.append(Gf.Vec3f(0.0, 0.0, z0))
    for k in range(N):
        faces += [2 * ((k + 1) % N), 2 * k, bot]
        counts.append(3)
    path = "{0}/tank_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(float(yaw))
    _bind(ctx["stage"], path, ctx["mats"]["timber"])     # a cedar tank
    paths.insert(0, path)
    ctx["authored"] += paths
    return paths


def dress_roof(ctx, mass="main", tanks=None, acs=None):
    """Rooftop plant on the top roof of `mass`: 0-2 water tanks and 2-6 AC
    units. Called for EVERY level including pristine, so the damaged
    grades have something to tip. Returns the prim paths."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    if len(m["levels"]) < 3:
        return []
    W, D = m["W"] - 4.0, m["D"] - 4.0
    z = m["top"] + 0.02
    made = []
    nt = tanks if tanks is not None else rng.choice((0, 1, 1, 2))
    for k in range(nt):
        lx, ly = rng.uniform(-W / 2.0, W / 2.0), rng.uniform(-D / 2.0, D / 2.0)
        wx, wy = _to_world(m, lx, ly)
        made += _tank(ctx, wx, wy, z, r=rng.uniform(1.0, 1.4), h=rng.uniform(2.2, 3.0),
                      yaw=m["yaw"] + rng.uniform(0, 90))
    na = acs if acs is not None else rng.randrange(2, 7)
    for k in range(na):
        lx, ly = rng.uniform(-W / 2.0, W / 2.0), rng.uniform(-D / 2.0, D / 2.0)
        wx, wy = _to_world(m, lx, ly)
        path = "{0}/ac_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        if _prop(ctx["stage"], path, _AC_UNIT, wx, wy, z, m["yaw"] + rng.choice((0, 90, 180, 270)),
                 1.0, rng):
            made.append(path)
            ctx["authored"].append(path)
    ctx["roof_plant"] = made
    return made


def r_rooftop_fail(ctx, frac=0.5):
    """Rooftop plant in a quake: tanks tip (loose, kicked over), AC units
    slide/tip; run after `dress_roof`."""
    rng = ctx["rng"]
    for pth in list(ctx.get("roof_plant", [])):
        if rng.random() >= frac:
            continue
        ctx["loose"].append(pth)
        ctx["fit"]["all"] = [q for q in ctx["fit"]["all"] if q != pth]
        # tip it past balance so the solver lays it over
        M = _rot_about(_pivot_of(ctx, pth), (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0),
                       rng.uniform(25.0, 45.0))
        _transform_prims(ctx["stage"], [pth], M)


def r_signage_fail(ctx, n=None):
    """Shop signs and awnings on the sidewalk: thin boxes on end, tipped."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    k = n if n is not None else rng.randrange(1, 4)
    for i in range(k):
        side = rng.choice(["S", "S", "E", "W"])
        ox, oy = _outward(m, side)
        L = m["W"] if side in ("S", "N") else m["D"]
        t = rng.uniform(-0.45, 0.45) * L
        d = rng.uniform(0.6, 2.5)
        if side in ("S", "N"):
            lx, ly = t, (-m["D"] / 2.0 - d if side == "S" else m["D"] / 2.0 + d)
        else:
            lx, ly = (-m["W"] / 2.0 - d if side == "W" else m["W"] / 2.0 + d), t
        wx, wy = _to_world(m, lx, ly)
        path = "{0}/sign_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy, m["z0"] + 0.12, rng.uniform(2.0, 4.0), 0.18,
             rng.uniform(0.8, 1.4), m["yaw"] + rng.uniform(-30, 30),
             ctx["mats"]["plaster"] if rng.random() < 0.5 else ctx["mats"]["dark_concrete"])
        M = _rot_about((wx, wy, m["z0"]), (math.cos(math.radians(m["yaw"])),
                                           math.sin(math.radians(m["yaw"])), 0.0),
                       rng.uniform(60.0, 88.0))
        _transform_prims(ctx["stage"], [path], M)
        ctx["authored"].append(path)
        ctx["static_extra"].append(path)


RECIPES = {
    "facade_scars": r_facade_scars,
    "rooftop_fail": r_rooftop_fail,
    "signage_fail": r_signage_fail,
    "storey_collapse": r_storey_collapse,
    "mid_storey": r_mid_storey,
    "droop": r_droop,
    "balcony_fail": r_balcony_fail,
    "roof_hole": r_roof_hole,
    "settlement": r_settlement,
    "tilt_severe": r_tilt_severe,
    "overturn": r_overturn,
    "parapet_fall": r_parapet_fall,
    "glass_loss": r_glass_loss,
    "glass_fallout": r_glass_fallout,
    "infill_fail": r_infill_fail,
    "corner_fail": r_corner_fail,
    "out_of_plane": r_out_of_plane,
    "soft_storey": r_soft_storey,
    "pancake": r_pancake,
    "masonry_collapse": r_masonry_collapse,
    "tilt_sink": r_tilt_sink,
}


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def wreck_building(stage, parent, style, placements, x, y, yaw, recipes,
                   rng, nrng, mats, tag, fit_storeys=None, mat_cache=None):
    """Apply `recipes` ([(name, kwargs)] or a grade string) to one placed kit
    building. Returns a ctx dict with `loose`, `static_extra`, `velocity`,
    `authored`, `notes`, `fit`.

    The building must already be on the stage (`apply_placements` has set
    each placement's `prim_path`). Fit-out happens here, before any recipe
    runs, and only when at least one recipe is going to run.
    """
    info = describe(style, placements, x, y, yaw)
    if isinstance(recipes, str):
        recipes = LADDER[info["type"]][recipes]
    ctx = {"stage": stage, "parent": parent, "info": info, "rng": rng,
           "nrng": nrng, "mats": mats, "cache": mat_cache if mat_cache is not None else {},
           "tag": tag, "loose": [], "static_extra": [], "velocity": {},
           "authored": [], "notes": [], "fit": {"slabs": {}, "columns": {},
                                                "partitions": [], "all": []}}
    if not recipes:
        dress_roof(ctx)
        ctx["static_extra"] += list(ctx.get("roof_plant", []))
        return ctx
    ctx["fit"] = fit_interior(stage, parent, info, mats, rng,
                              storeys=fit_storeys, tag=tag)
    dress_roof(ctx)
    ctx["fit"]["all"] += list(ctx.get("roof_plant", []))
    for name, kw in recipes:
        RECIPES[name](ctx, **(kw or {}))
    # everything still standing is static for the settle
    ctx["static_extra"] += [e["p"].get("prim_path") for e in _els(ctx)
                            if e["p"].get("prim_path")]
    ctx["static_extra"] += [p for p in ctx["fit"]["all"]
                            if p not in ctx["loose"]]
    # dedupe, keep order, drop anything that became loose
    loose = set(ctx["loose"])
    seen, st = set(), []
    for p in ctx["static_extra"]:
        if p in loose or p in seen:
            continue
        seen.add(p)
        st.append(p)
    ctx["static_extra"] = st
    return ctx


def level_for_intensity(i, btype, rng, jitter=0.05):
    """Field intensity (0..1) -> EMS-98 grade for a construction type.

    NOT A THRESHOLD ON THE FIELD. Thresholding makes every building inside
    the strong-shaking radius the same grade, and the reconnaissance record
    is the opposite: the worst-hit blocks of Antakya, Christchurch and
    Amatrice are a MIX, with total collapse a minority even there. So the
    grade is DRAWN: `v = i * u`, u uniform, then cut — at full intensity the
    cuts below reproduce the worst-block mixes in the research report
    (URM 5/20/30/25/20 % for none / DG1-2 / DG3 / DG4 / DG5; RC frames
    20/30/28/12/10; glass towers 55/30/13/2/<1), and as the field falls off
    the draw compresses toward the light grades. Vulnerability classes A/B
    (URM) fail before C (RC) before D/E (towers), which is what the three
    cut tables encode.
    """
    v = float(i) * rng.random() + rng.gauss(0.0, jitter)
    v = min(1.0, max(0.0, v))
    cuts = {
        "urm":      (0.05, 0.14, 0.25, 0.55, 0.80),
        "rc":       (0.20, 0.36, 0.50, 0.78, 0.90),
        "rc_glass": (0.55, 0.72, 0.85, 0.98, 0.995),
    }[btype]
    g = 0
    for c in cuts:
        if v >= c:
            g += 1
    return GRADES[g]


def check(verbose=True):
    """Host-side: every ladder recipe exists, every family has a type."""
    from detail import urban_building as ub
    bad = []
    for t, ladder in LADDER.items():
        for g, recs in ladder.items():
            for name, _kw in recs:
                if name not in RECIPES:
                    bad.append("{0}/{1}: unknown recipe {2}".format(t, g, name))
    for s, spec in ub.STYLES.items():
        if spec.get("family") not in FAMILY_TYPE:
            bad.append("style {0}: family {1} has no type".format(s, spec.get("family")))
    if verbose:
        print("[quake_flow] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad
