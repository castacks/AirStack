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

import bisect
import math
import os as _os
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
        # ---- round 2 (agent B): the user's "random white debris" ----
        # 0.66 luma plaster is the brightest thing in the set and it is what
        # reads as PAPER on a pavement at 20 m (the DG2 bench: white cubes in
        # a brick windrow, a white plank of a sign). Anything that leaves the
        # building and lands outside takes the dusty variant instead; the
        # interior `plaster` stays as it was for partitions and fragment
        # backs, which are seen in shadow behind a break.
        # THESE NUMBERS ARE LINEAR ALBEDO, NOT SCREEN GREY. `damage._pbr`
        # writes them straight into OmniPBR's `diffuse_color_constant`, and
        # the renderer encodes to sRGB afterwards — so 0.47 linear comes out
        # at 0.71 on screen, i.e. WHITE, and the whole existing palette
        # ("0.45-0.68 luma on purpose", above) is a full stop lighter than
        # its comment claims. That is most of why round 1 read as white
        # paper. Rule of thumb: screen grey ~= linear ** 0.42, so a mid-dark
        # material is 0.04-0.09 linear and a dusty grey is ~0.15.
        "plaster_dusty": ((0.155, 0.145, 0.130), 1.0),     # -> ~0.43 on screen
        # The dark inside of a finish-loss patch: what you actually see where
        # render/stucco has come off is a SHADOWED recess, not a bright
        # sticker.
        # Measured on the bench (B2_com/1_commercial_DG2_sw.png): at 0.040
        # linear the patch came out at 0.27 of the wall's screen value and
        # read as a BLACK HOLE punched through the brick. A spall is 1-3 cm
        # deep, not a void — ~0.45 of the wall is the shadowed-recess read,
        # which is 0.12 linear against this brick.
        "scar": ((0.125, 0.100, 0.085), 1.0),              # -> ~0.39
        "scar_shadow": ((0.050, 0.042, 0.036), 1.0),       # -> ~0.25
        # A cedar roof tank. FLAT, not the plank texture: OmniPBR's
        # `diffuse_color_constant` does NOT visibly multiply `diffuse_texture`
        # in this build — `planks.wood_material(tint=...)` at 0.50 and at
        # 0.085 render pixel-identical (B1_com vs B2_com storey_collapse_ne),
        # so a textured tank cannot be darkened at all and stays a pale cream
        # barrel. The hoops and the conical lid carry the read instead.
        "tank_dark": ((0.032, 0.023, 0.016), 0.95),        # -> ~0.20
        # Broken window glass on a pavement: tinted float glass seen edge-on,
        # grey-green and only weakly glossy. The `glass` key above (pale, 0.12
        # roughness) is the pane tint and blows out to white on a 0.2 m shard.
        # ROUGH, not glossy. At 0.38 roughness a shard lying face-up mirrors
        # the sky dome and comes out pale mint whatever its albedo
        # (B2_apt/0_apartment_tall_DG2_nw.png); a field of broken glass on
        # asphalt is a scatter of dark grey-green flecks that occasionally
        # glints, and 0.72 is what stops the whole field glinting at once.
        "glass_shard": ((0.045, 0.062, 0.056), 0.72),      # -> ~0.24/0.28
        # Shop signage: painted sheet metal under quake dust. Never plaster —
        # a white 3 m box on the sidewalk is the single most paper-like thing
        # in the scene.
        "sign": ((0.020, 0.045, 0.028), 0.75),             # dark green
        "sign_red": ((0.090, 0.014, 0.012), 0.75),
        "sign_blue": ((0.014, 0.030, 0.100), 0.75),
        "awning": ((0.070, 0.040, 0.028), 0.95),           # faded canvas
        # Rooftop condenser housings. The Nucleus `SM_airConditioner01` ships
        # a near-white shell and a row of them reads as a row of white boxes
        # at 40 m — which is exactly the "random white debris" note. Bound
        # OVER the asset (`_b_bind_over`) as dusty galvanised steel.
        "plant_metal": ((0.130, 0.132, 0.126), 0.62),      # -> ~0.40
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
    # A ROOFTOP CEDAR TANK IS DARK. Bound to the floor-deck `timber` above,
    # a 2.5 m tank came out pale cream with a fine horizontal band and read
    # as a wicker basket / a white drum from 40 m (quake_city9 sw_obl). A
    # weathered redwood tank under city grime is 0.20 on screen. Two tries
    # with `planks.wood_material(tint=...)` (0.50 then 0.085) rendered
    # PIXEL-IDENTICAL, so the tint does not reach OmniPBR's diffuse when a
    # texture is bound: `tank_dark`, a flat colour, is the only lever there
    # is.
    out["tank_wood"] = out.get("tank_dark") or out.get("timber")
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


def _b_bind_over(stage, path, mat):
    """Bind `mat` so it BEATS the bindings inside a referenced asset.

    The default `Bind` is weakerThanDescendants, which is right for a mesh
    we authored and useless for a Nucleus prop whose own materials are bound
    on its leaves."""
    from pxr import UsdShade
    pr = stage.GetPrimAtPath(path)
    if pr and pr.IsValid() and mat is not None:
        UsdShade.MaterialBindingAPI(pr).Bind(
            mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants)


def _b_dusty(ctx, mat):
    """Swap the interior `plaster` for its dusty variant on anything that
    ends up OUTSIDE the building.

    0.66 luma is right for a partition seen in shadow through a broken wall
    and wrong for a 0.4 m chunk lying in sunlight on asphalt, where it is
    the "random white debris" the user picked out. Cheap to apply at the
    bind, so the fragment material tables do not have to fork."""
    mats = ctx["mats"]
    try:
        if mat is not None and mats.get("plaster") is not None \
                and mat == mats["plaster"]:
            return mats.get("plaster_dusty", mat)
    except Exception:
        pass
    return mat


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


# TWO-SCALE BREAK KNOBS (round 2). The user, on the first library: "the shear
# is along a straight line ... unnatural rectangular/square parts broken off".
# The cause was one fracture at one scale: 10-15 seeds over a 20 m roof gives
# 4-5 m cells, so a loose/static split judged cell by cell has a surviving
# edge made of 4 m flat planes, and the whole slab reads as a crack mosaic
# because `shrink=0.97` opens an 8 cm gap between two 5 m cells.
# `fracture.fracture_split` fixes both; these are its defaults here.
EDGE_CELL_M = 0.32       # tooth size at a break line (0.2-0.4 m, per the brief)
EDGE_MAX = 16            # edge seeds per refined cell
# CELLS REFINED PER ELEMENT — the time ceiling, and the one knob worth having
# on the launch line. Refinement is what costs: a `commercial` column went
# 4 s -> 15 s (corner_fail) and 3 s -> 24 s (storey_collapse) turning it on, so
# a 96-archetype bake pays roughly 4x its fracture time for it. `EQ_REFINE_MAX`
# in the environment overrides; 0 turns the second scale off entirely and gives
# back the round-1 look and the round-1 speed.
REFINE_MAX = (int(_os.environ["EQ_REFINE_MAX"])
              if _os.environ.get("EQ_REFINE_MAX", "").strip().isdigit() else 12)
CHEW_OUT = 0.20          # share of edge statics bitten off the stub
CHEW_IN = 0.14           # share of edge loose pieces left HANGING (static)
CRACK_FRAC = 0.16        # share of rim statics that open a visible crack
GAP_LOOSE_M = 0.008      # separation PhysX needs between two loose cells
GAP_STATIC_M = 0.0008    # hairline: a surviving slab must read as ONE slab
GAP_CRACK_M = 0.011      # a crack radiating out of the break
ROUGH_M = 0.028          # scar amplitude in METRES, not a fraction of the cell


def _break_split(ctx, path, n, judge, mat_fn, rough=ROUGH_M,
                 min_volume_frac=0.0015, mode="uniform", aspect=None,
                 static_mat=None, refine=True, edge_cell_m=EDGE_CELL_M,
                 chew=(CHEW_OUT, CHEW_IN), crack_frac=CRACK_FRAC,
                 refine_max=REFINE_MAX, gap_static_m=GAP_STATIC_M,
                 max_loose_m=None, edge_consume=0.5):
    """Fracture *path* and split the fragments by `judge(centroid) -> bool`
    (True = comes loose). The rest stay as STATIC stubs, so the surviving
    edge is made of real cell boundaries: this is `fracture_partial` with a
    caller-supplied judge instead of a height cut, and it is what makes a
    hole ragged along its sides rather than only along its bottom.

    ROUND 2: the cut is made at TWO SCALES (`fracture.fracture_split`) —
    coarse cells away from the line, `edge_cell_m` teeth at it, a chewed edge,
    and gaps measured in METRES so what survives butts back together instead
    of crazing. `static_mat` binds every surviving cell to ONE material (the
    piece's own, normally): a random draw per cell is what turned an intact
    roof into a patchwork even before the gaps showed.

    Returns (static_paths, loose_paths)."""
    from . import fracture
    stage = ctx["stage"]
    mesh = fracture.prim_to_mesh(stage, path)
    if mesh is None:
        return [], []
    st_m, lo_m = fracture.fracture_split(
        mesh, n, judge, ctx["nrng"], mode=mode, aspect=aspect,
        rough_m=max(0.008, min(0.06, float(rough))),
        edge_cell_m=edge_cell_m, edge_max=EDGE_MAX,
        refine=bool(refine) and int(refine_max) > 0,
        refine_max=refine_max, min_volume_frac=min_volume_frac,
        chew_out=chew[0], chew_in=chew[1], crack_frac=crack_frac,
        gap_loose_m=GAP_LOOSE_M, gap_static_m=gap_static_m,
        gap_crack_m=GAP_CRACK_M, max_loose_m=max_loose_m,
        edge_consume=edge_consume)
    if not st_m and not lo_m:
        return [], []
    out = "{0}/brk_{1}_{2}".format(ctx["parent"], ctx["tag"], path.rsplit("/", 1)[-1])
    from pxr import Sdf, UsdGeom
    UsdGeom.Scope.Define(stage, Sdf.Path(out))
    st, lo, i = [], [], 0
    for group, sink, mf in ((st_m, st, static_mat), (lo_m, lo, None)):
        for f in group:
            fp = "{0}/frag_{1:03d}".format(out, i)
            i += 1
            fracture._write_mesh(stage, fp, f)
            _bind(stage, fp, mf if mf is not None else mat_fn())
            sink.append(fp)
    src = stage.GetPrimAtPath(path)
    if src and src.IsValid():
        src.SetActive(False)
    return st, lo


def _wander(rng, amp, freq=(1.0, 2.6)):
    """A 1-D wobble f(t) in [-amp, amp] for ragged break lines, t normalised
    0..1 along the line. Superseded for break lines by `_a_wobble` (which
    works in metres and knows about masonry courses); still used where a
    caller only wants a lazy curve."""
    ph = rng.uniform(0, 6.28)
    fr = rng.uniform(*freq)
    return lambda t: amp * (0.6 * math.sin(t * fr * 6.28 + ph)
                            + 0.4 * math.sin(t * fr * 2.7 + ph * 1.7))


# ---------------------------------------------------------------------------
# BREAK LINES — stepped for masonry, torn for concrete
# ---------------------------------------------------------------------------
#
# A brick or stone wall does not shear on a smooth curve. The bed joints are
# the weak planes and the perpends are staggered, so a wall that comes down in
# part leaves a STAIRCASE: horizontal runs of one to four brick lengths with
# risers of a few courses — the "toothing" every photograph of a URM
# out-of-plane or corner failure shows (Christchurch 2011, Amatrice 2016,
# Antakya 2023; see _plans/earthquake_research.md §URM). A brick is 0.215 m
# long and a course 0.075 m, so a run is 0.3-1.2 m and a riser 0.1-0.4 m.
#
# Reinforced concrete does the opposite: it tears. The crack follows the
# aggregate, the cover spalls off ahead of it, and pieces hang on the bars —
# so the line is smooth at the metre scale, rough at the decimetre scale, and
# the ragged part comes from the CHEW in `fracture_split` rather than from the
# line itself.
STEP_H = ((0.30, 1.20), (0.10, 0.40))   # break ALONG a wall: (run, riser)
STEP_V = ((0.20, 0.60), (0.10, 0.30))   # break UP a wall: (run in z, riser)


def _a_stepped(rng, amp, span_m, run_m, rise_m):
    """A staircase f(t), t in METRES along the break line, bounded to ±amp.

    The walk REFLECTS off the bound rather than clamping: a clamped walk
    spends most of its length riding the rail and comes out looking like a
    straight line with notches."""
    ts, ys = [0.0], [0.0]
    t, y = 0.0, 0.0
    lim = max(1e-3, float(amp))
    lo_rise = float(rise_m[0])
    while t < float(span_m) + run_m[1]:
        t += rng.uniform(*run_m)
        step = rng.uniform(*rise_m) * (1.0 if rng.random() < 0.5 else -1.0)
        ny = y + step
        if ny > lim or ny < -lim:
            # REFLECT, DON'T CLAMP — a clamped walk rides the rail and comes
            # out as a straight line with notches. But a reflection can land
            # back within a centimetre of where it started (y=0.4, +0.4,
            # lim=0.6 -> 0.4 exactly), and a 1 cm riser is not a step at all;
            # measured over a 22 m wall the minimum riser was 0.010 m. Turn
            # round instead whenever the reflection would not clear one course.
            ry = (2.0 * lim - ny) if ny > lim else (-2.0 * lim - ny)
            ny = (y - step) if abs(ry - y) < lo_rise else ry
            ny = max(-lim, min(lim, ny))
        y = ny
        ts.append(t)
        ys.append(y)

    def f(tm):
        i = bisect.bisect_right(ts, float(tm)) - 1
        return ys[max(0, min(len(ys) - 1, i))]
    return f


def _a_torn(rng, amp, span_m):
    """A smooth-but-not-lazy tear f(t), t in METRES, bounded to ~±amp.

    `_wander` puts one or two cycles across the WHOLE piece, which at 20 m is
    a curve so lazy it reads as straight. FOUR octaves down to ~span/36 (about
    0.6 m on a 22 m wall) give the metre-scale structure a torn concrete edge
    has; measured on a 22 m span this turns 5 zero crossings into 15-25
    turning points, which is the difference between an arc and a tear. The
    sub-decimetre scale is NOT this function's job — that comes from the cell
    refinement and the chew in `fracture_split`."""
    n = 4
    ph = [rng.uniform(0, 6.28) for _ in range(n)]
    L = max(1.0, float(span_m))
    w = [6.2832 / (L / f) for f in (2.2, 6.5, 16.0, 36.0)]
    a = [0.42, 0.28, 0.19, 0.11]

    def f(tm):
        t = float(tm)
        return float(amp) * sum(a[k] * math.sin(t * w[k] + ph[k]) for k in range(n))
    return f


def _a_wobble(rng, amp, span_m, btype="rc", vertical=False):
    """The break-line offset function for a construction type. `vertical` =
    the line runs UP the wall (so the runs are in z and the risers sideways)."""
    if btype == "urm":
        run, rise = (STEP_V if vertical else STEP_H)
        return _a_stepped(rng, amp, span_m, run, rise)
    return _a_torn(rng, amp, span_m)


def _a_side_t(m, side, c):
    """Distance in METRES along the mass's `side` wall, from its low end."""
    lx, ly = _to_local(m, c[0], c[1])
    return (lx + m["W"] / 2.0) if side in ("S", "N") else (ly + m["D"] / 2.0)


def _a_side_span(m, side):
    return m["W"] if side in ("S", "N") else m["D"]


def _a_zline_judge(m, side, z0, rng, btype="rc", amp=0.5, loose_above=True):
    """judge(world c) for a HORIZONTAL break at height `z0` that steps (urm)
    or tears (rc) along the wall. This is what kills the kit's module seam:
    a storey that is removed otherwise leaves the wall above it with a level
    bottom edge exactly on the slab line."""
    wob = _a_wobble(rng, amp, _a_side_span(m, side), btype, vertical=False)

    def judge(c):
        zb = float(z0) + wob(_a_side_t(m, side, c))
        return (c[2] > zb) if loose_above else (c[2] < zb)
    return judge


def _edge_judge(m, side, depth_m, rng, btype=None):
    """judge(world centroid) -> True when the point lies within a WANDERING
    depth of the mass's `side` wall line (measured inward).

    `btype` picks the line's character: `urm` steps along the courses, `rc`
    (or None, the old behaviour's replacement) tears. The offset is now a
    function of the distance ALONG the wall in METRES, not of a 0..1
    parameter, so the same wobble reads the same on a 9 m wing and a 30 m
    block instead of stretching to fit."""
    W, D = m["W"], m["D"]
    wob = _a_wobble(rng, depth_m * 0.55, _a_side_span(m, side), btype or "rc",
                    vertical=False)

    def judge(c):
        lx, ly = _to_local(m, c[0], c[1])
        if side == "S":
            d, t = ly + D / 2.0, lx + W / 2.0
        elif side == "N":
            d, t = D / 2.0 - ly, lx + W / 2.0
        elif side == "W":
            d, t = lx + W / 2.0, ly + D / 2.0
        else:
            d, t = W / 2.0 - lx, ly + D / 2.0
        return d < depth_m + wob(t)
    return judge


def _toward_judge(m, side, frac, rng, btype=None, z0=0.0, span_z=None):
    """For a wall piece on a side ADJACENT to a failed one: the part of the
    piece nearest the failed side comes away (`frac` of its length), the
    far part stays, with a wandering line between them.

    This line runs UP the wall, so for masonry it is the VERTICAL toothing
    (runs of a few courses, risers of half a brick): `_a_wobble(vertical=True)`.
    The old version keyed the wobble off `c[2] / 6.0` against a normalised
    fraction, which made the offset about 5 cm on a 4 m bay — invisible, and
    the ends came off straight."""
    W, D = m["W"], m["D"]
    reach = (D if side in ("S", "N") else W)
    if span_z is None:
        span_z = max(6.0, float(m.get("top", 12.0)) - float(m.get("z0", 0.0)))
    wob = _a_wobble(rng, 0.45, max(3.0, float(span_z)), btype or "rc",
                    vertical=True)

    def judge(c):
        lx, ly = _to_local(m, c[0], c[1])
        if side == "S":
            u = ly + D / 2.0
        elif side == "N":
            u = D / 2.0 - ly
        elif side == "W":
            u = lx + W / 2.0
        else:
            u = W / 2.0 - lx
        return u < frac * reach + wob(c[2] - float(z0))
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
        # THE SURVIVING HALF OF A BAY IS STILL THAT BAY. Drawing a material
        # per cell (`_mat_fn`) turned what is left standing into a patchwork
        # of brick / mortar / plaster panels; the piece's own cladding, bound
        # to every static cell, keeps it one wall with a torn end.
        st, lo = _break_split(
            ctx, path, 9 + rng.randrange(4),
            _toward_judge(m, side, keep, rng, btype=ctx["info"]["type"]),
            _mat_fn(ctx, tex, 0.35),
            static_mat=_clad_material(ctx["stage"], ctx["parent"], ctx["cache"], tex)
            if tex else None, refine_max=6)
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
    if path in ctx.get("roof_slabs", ()):
        ctx["roof_slabs"].append(rem)      # the remainder IS still that roof
    return rem, strip


def _ragged_slabs(ctx, mass, side, storeys, depth=(0.8, 2.6)):
    """Floor slabs (and the roof) lose a wandering strip along the failed
    side: the exposed floor edge is broken, not a machined line."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    fit = ctx["fit"]
    mats = ctx["mats"]
    btype = ctx["info"]["type"]
    # LOOSE slab pieces are debris, so they take the dark debris tints; the
    # SURVIVING cells are rebound to the slab's own material (`static_mat`).
    # ...and `mats["timber"]` is the PALE plank map (agent B measured that
    # `planks.wood_material`'s tint does not multiply the texture in this
    # build, so it cannot be darkened) — right for an intact floor deck seen
    # through a hole, wrong for a board lying in the rubble.
    slab_mat = (lambda: _a_mat(ctx, "timber_dusty")
                if (btype == "urm" and rng.random() < 0.7)
                else (_a_mat(ctx, "concrete_dusty") if rng.random() < 0.6
                      else _a_mat(ctx, "dust")))
    # STRIP WIDTH: the break line sits `d` in from the wall, and the strip has
    # to hold BOTH sides of it — the pieces that come away AND a real band of
    # surviving cells behind, or the ragged edge butts straight onto the
    # remainder box and the seam between them is the straight line again.
    # 2.2 m of static band is about four `EDGE_CELL_M` teeth deep.
    from pxr import UsdShade
    for (mt, i), pth in list(fit["slabs"].items()):
        if mt != mass or i not in storeys or not pth:
            continue
        d = rng.uniform(*depth)
        keep_m = UsdShade.MaterialBindingAPI(
            ctx["stage"].GetPrimAtPath(pth)).ComputeBoundMaterial()[0]
        rem, strip = _split_strip(ctx, pth, m, side, d + 2.2, mats["concrete"])
        st, lo = _break_split(ctx, strip, 12 + rng.randrange(5),
                              _edge_judge(m, side, d, rng, btype=btype),
                              slab_mat, min_volume_frac=0.0008,
                              static_mat=keep_m if keep_m else None)
        fit["slabs"][(mt, i)] = rem
        fit["all"] = [q for q in fit["all"] if q != pth] + [rem] + st
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        _a_edge_bars(ctx, st, btype, m, side)
    # the roof over the failed side: the strip along it, no more
    for e in list(_els(ctx, mass=mass, role="roof")):
        box = _roof_box(ctx, e)
        if not box:
            continue
        d = rng.uniform(*depth)
        rem, strip = _split_strip(ctx, box, m, side, d + 2.2, mats["concrete"])
        bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(strip)).ComputeBoundMaterial()[0]
        st, lo = _break_split(ctx, strip, 12 + rng.randrange(5),
                              _edge_judge(m, side, d, rng, btype=btype),
                              lambda: (_a_mat(ctx, "timber_dusty")
                                       if (btype == "urm" and rng.random() < 0.5)
                                       else _a_mat(ctx, "concrete_dusty")),
                              min_volume_frac=0.0006,
                              static_mat=bm if bm else None, max_loose_m=3.2)
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        ctx["authored"].append(rem)
        e["dead"] = True


def _a_edge_bars(ctx, statics, btype, m, side, n=None, p=0.6):
    """A broken REINFORCED-concrete slab does not end at the concrete.

    The top and bottom mats run on through the break and stand out of it,
    bent — it is the single most recognisable cue that a slab edge is torn
    rather than sawn, and USAR photographs of every pancake and every
    balcony failure show it. Masonry has no bars, so this is an `rc` /
    `rc_glass` detail only; a URM timber deck ends in splintered joists,
    which the `plank` seeding already gives.

    `statics` are the surviving cells of a break; a share of the ones nearest
    the open side grow a tuft."""
    if btype == "urm" or not statics:
        return
    rng = ctx["rng"]
    ox, oy = _outward(m, side)
    # the cells nearest the opening, by their own world position
    ranked = []
    for pth in statics:
        c = _pivot_of(ctx, pth)
        ranked.append((c[0] * ox + c[1] * oy, pth, c))
    ranked.sort(key=lambda q: -q[0])
    n = n if n is not None else max(1, int(round(len(ranked) * 0.22)))
    for _s, pth, c in ranked[:max(1, min(6, n))]:
        if rng.random() >= p:
            continue
        _rebar_tuft(ctx, pth, c[2] + 0.05, n=2 + rng.randrange(3),
                    length=(0.35, 1.0))


def _a_slab_rim(ctx, mass, storey, n_sides=2, depth=(0.4, 1.3), bars=True):
    """Tear the exposed perimeter of ONE floor slab.

    A crushed storey leaves the slab under it open to the sky all the way
    round, and the fit-out authors slabs as `_box`es — so what a camera sees
    across the gap of a soft/mid-storey collapse is a rectangular plate with
    four ruler edges. Two sides of it lose a wandering strip; the rest of the
    perimeter is hidden by the block above."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    btype = ctx["info"]["type"]
    fit = ctx["fit"]
    pth = fit["slabs"].get((mass, storey))
    if not pth:
        return []
    from pxr import UsdShade
    sides = ["S", "E", "N", "W"]
    rng.shuffle(sides)
    # ONLY THE FINAL REMAINDER IS LIVE. Splitting the remainder again
    # deactivates it, so an intermediate `rem` must not end up in the returned
    # list — a caller that transforms the result would be moving dead prims and
    # (worse) would MISS the statics of the first side if it only carried the
    # last group. `r_pancake` had exactly that bug.
    orig, sts = pth, []
    for sd in sides[:max(1, int(n_sides))]:
        km = UsdShade.MaterialBindingAPI(
            ctx["stage"].GetPrimAtPath(pth)).ComputeBoundMaterial()[0]
        d = rng.uniform(*depth)
        rem, strip = _split_strip(ctx, pth, m, sd, d + 1.6, ctx["mats"]["concrete"])
        st, lo = _break_split(
            ctx, strip, 10 + rng.randrange(4),
            _edge_judge(m, sd, d, rng, btype=btype),
            lambda: (_a_mat(ctx, "timber_dusty") if btype == "urm"
                     else _a_mat(ctx, "dust")),
            min_volume_frac=0.0008, static_mat=km if km else None,
            refine_max=6, max_loose_m=2.6)
        ctx["loose"] += lo
        sts += st
        ctx["authored"].append(rem)
        pth = rem
    live = [pth] + sts
    ctx["static_extra"] += live
    fit["all"] = [q for q in fit["all"] if q != orig] + live
    fit["slabs"][(mass, storey)] = pth
    if bars:
        # `bars=False` when the caller is about to TRANSFORM this slab: a
        # rebar tuft is authored in world space with no xform of its own, so
        # it would stay behind where the slab used to be.
        _a_edge_bars(ctx, sts, btype, m, sides[0], n=2)
    return live


def _a_ragged_courses(ctx, mass, storey, sides=None, band=(0.28, 0.85),
                      above=True, below=True, n_seeds=None, p=0.92,
                      near=None):
    """Destroy the kit's HORIZONTAL module seam at a removed storey.

    THE BUG THIS FIXES. Wall damage removes whole kit modules, so a storey
    that is taken out (soft_storey, mid_storey, storey_collapse, the bottom
    of an out_of_plane peel) leaves the modules ABOVE it with a dead-level
    bottom edge and the ones BELOW with a dead-level top edge, both exactly on
    the slab line. The user, on the first library: "there's some unnatural
    rectangular/square parts broken off" — those two lines are the top and
    bottom of the rectangle.

    Each neighbouring module therefore loses a band along that shared edge,
    judged with a line that STEPS along the courses for masonry and tears for
    concrete (`_a_zline_judge`), so no surviving edge coincides with a kit
    seam. Returns a dict of the paths made, keyed `above_static`,
    `above_loose`, `below_static`, `below_loose` — the caller has to know
    which statics belong to the block ABOVE, because in `soft_storey` that
    block is about to be translated and they must ride with it.
    """
    from . import damage
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    btype = ctx["info"]["type"]
    lv = m["levels"]
    out = {"above_static": [], "above_loose": [],
           "below_static": [], "below_loose": []}
    z_lo = lv[storey] if storey < len(lv) else m["top"]
    z_hi = lv[storey + 1] if storey + 1 < len(lv) else m["top"]
    jobs = []
    if above and storey + 1 < len(lv):
        jobs.append((storey + 1, z_hi, True, "above"))     # its BOTTOM edge
    if below and storey - 1 >= 0:
        jobs.append((storey - 1, z_lo, False, "below"))    # its TOP edge
    for st_i, z_ref, is_above, key in jobs:
        for e in list(_els(ctx, mass=mass, role=("wall", "corner", "balcony"),
                           storey=st_i)):
            if sides is not None and e["side"] not in sides:
                continue
            if near is not None and not near(e):
                continue
            if rng.random() >= p:
                continue
            path = e["p"].get("prim_path")
            if not path:
                continue
            b = rng.uniform(*band)
            tex = damage.bound_texture(ctx["stage"], path)
            # The band is measured INTO the module from the failed storey. For
            # the module ABOVE, its bottom `b` metres come away, so the loose
            # side is BELOW a line at z_hi + b; for the one BELOW, its top `b`
            # metres go and the loose side is ABOVE z_lo - b.
            z0 = z_ref + b if is_above else z_ref - b
            judge = _a_zline_judge(m, e["side"], z0, rng, btype=btype,
                                   amp=b * 0.8, loose_above=not is_above)
            keep_mat = (_clad_material(ctx["stage"], ctx["parent"],
                                       ctx["cache"], tex) if tex else None)
            # A BAND ON A WALL MODULE IS A SMALL FEATURE and there are a lot of
            # modules — the whole of two storeys on a soft-storey collapse.
            # `refine_max=4` and 8 coarse cells keep this to ~0.4 s a module
            # (at REFINE_MAX/EDGE_MAX it was 2 s, and `storey_collapse` on a
            # 4-storey commercial went from 3 s to 36 s).
            s, l = _break_split(ctx, path, n_seeds or (7 + rng.randrange(3)),
                                judge, _mat_fn(ctx, tex, 0.4),
                                static_mat=keep_mat, refine_max=4,
                                edge_cell_m=0.38)
            if not s:
                # the whole module came away — that is a hole, not a torn
                # edge; put it back as loose and let the next module carry it
                ctx["loose"] += l
                e["dead"] = True
                out[key + "_loose"] += l
                continue
            ox, oy = _outward(m, e["side"])
            for q in l:
                v = rng.uniform(0.2, 0.9)
                ctx["velocity"][q] = (ox * v, oy * v, 0.0)
            ctx["loose"] += l
            out[key + "_static"] += s
            out[key + "_loose"] += l
            e["dead"] = True
        if key == "below":
            ctx["static_extra"] += out["below_static"]
    return out


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
                # the STUB keeps interior plaster; what snapped off is debris
                _bind(ctx["stage"], q,
                      mats["plaster"] if q in st else _a_mat(ctx, "plaster_dusty"))
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
            # DUSTY, NOT BRIGHT PLASTER. This litter is seen from NADIR
            # through a roof hole as often as it is seen in shadow behind a
            # broken wall, and at 0.66 luma in sunlight it is white confetti.
            mat = (mats["brick"] if (ctx["info"]["type"] == "urm" and rng.random() < 0.5)
                   else (_a_mat(ctx, "plaster_dusty") if rng.random() < 0.45
                         else (_a_mat(ctx, "dust") if rng.random() < 0.4
                               else _a_mat(ctx, "concrete_dusty"))))
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
            _bind(ctx["stage"], q, _b_dusty(ctx, mf()))
        ox, oy = _outward(m, e["side"])
        for q in lo:
            v = rng.uniform(0.4, 1.2)
            ctx["velocity"][q] = (ox * v, oy * v, 0.0)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True


def _break_box(stage, path, n, rng, nrng, mat, inner_mat=None, inner_p=0.5,
               mode="uniform", aspect=None, consume=0.0, consume_pool=1.6):
    """Fracture an authored box (slab / column) into chunks. Returns paths.

    `consume_pool` is exposed (round 2): 1.6 lets middling pieces into the
    consumption draw, which is right for thinning a wall's shells, but a
    collapsed timber DECK has to lose its BIGGEST boards specifically or the
    heap comes out a lumber yard. Pass 1.0-1.1 for that."""
    from . import fracture
    out = path + "_brk"
    made = fracture.fracture_prim(stage, path, out, n_pieces=n, rng=nrng,
                                  mode=mode, aspect=aspect, rough=0.012,
                                  verbose=False, consume=consume,
                                  consume_pool=consume_pool,
                                  min_volume_frac=0.0008)
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
            # TINTED, NOT THE PANE COLOUR. `glass` is the pale, glossy tint a
            # 3 m curtain-wall panel has when it is still in the frame; on a
            # 0.3 m fragment lying on asphalt the same material is a white
            # chip, and a pavement of them was half of the user's "random
            # white debris". Broken float glass seen against the ground is
            # dark grey-green.
            _box(ctx["stage"], path, wx, wy, m["z0"] + 0.008,
                 s, s * rng.uniform(0.4, 1.0),
                 0.012, rng.uniform(0, 180), ctx["mats"]["glass_shard"])
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
        _a_dustify(ctx, lo)
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
            # the line runs UP the wall, so masonry tooths vertically
            wob = _a_wobble(rng, 0.85, max(6.0, m["top"] - m["z0"]),
                            ctx["info"]["type"], vertical=True)
            keep_r = d - rng.uniform(1.0, 3.0)

            def judge(c, _k=keep_r, _w=wob, _z=m["z0"]):
                lx, ly = _to_local(m, c[0], c[1])
                return math.hypot(lx - cx, ly - cy) < _k + _w(c[2] - _z)
            st, lo = _break_split(
                ctx, path, 9 + rng.randrange(4), judge, _mat_fn(ctx, tex, 0.35),
                static_mat=_clad_material(ctx["stage"], ctx["parent"],
                                          ctx["cache"], tex) if tex else None)
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
            far_rem, strip = _split_strip(ctx, strip, m, other[sd], reach + 3.4,
                                          mats["concrete"])
            statics.append(far_rem)
            rr = reach + rng.uniform(0.0, 1.5)
            # THE WOBBLE RUNS IN ARC LENGTH, NOT IN RADIANS. `_wander` keyed on
            # atan2 put one cycle over the whole quarter-turn, so the notch's
            # edge was a smooth arc — a compass line rather than a break.
            wob = _a_wobble(rng, 1.1, max(4.0, 1.6 * rr), btype)

            def judge(c, _r=rr, _w=wob):
                lx, ly = _to_local(m, c[0], c[1])
                th = math.atan2(ly - cy, lx - cx)
                return math.hypot(lx - cx, ly - cy) < _r + _w(th * _r)
            from pxr import UsdShade
            bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(strip)).ComputeBoundMaterial()[0]
            st, lo = _break_split(ctx, strip, 12 + rng.randrange(5), judge, mat_fn,
                                  min_volume_frac=0.0008,
                                  static_mat=bm if bm else None)
            statics += st
            loose += lo
            _a_edge_bars(ctx, st, btype, m, sd)
        return rem, statics, loose

    for (mt, i), pth in list(fit["slabs"].items()):
        if mt != mass or i not in top_storeys or not pth:
            continue
        rem, st, lo = _corner_break(
            pth, lambda: (_a_mat(ctx, "timber_dusty") if btype == "urm"
                          else _a_mat(ctx, "concrete_dusty")))
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
            box, lambda: (_a_mat(ctx, "timber_dusty")
                          if (btype == "urm" and rng.random() < 0.5)
                          else _a_mat(ctx, "concrete_dusty")))
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        ctx["authored"].append(rem)
        e["dead"] = True
    # THE FLOOR OF THE NOTCH IS A KIT SEAM. The lowest removed storey leaves
    # the modules under it with a level top edge on the slab line — the
    # horizontal half of the "rectangular part broken off". Only near the
    # corner: the rest of that storey is undamaged and should stay so.
    k0 = min(top_storeys) if top_storeys else 0
    if k0 > 0:
        _a_ragged_courses(
            ctx, mass, k0, sides=c_sides, above=False, below=True,
            band=(0.25, 0.75), p=0.9,
            near=lambda e: math.hypot(e["lx"] - cx, e["ly"] - cy) < reach + 5.0)
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

# A FLAT ROOF IS DARK FROM THE AIR. `_roof_box` bound `mats["concrete"]`
# (the Worn_Pavement map) to every authored roof slab, and on the families
# whose kit roof tiles are dark that left a roof half dark kit and half BRIGHT
# PLATE as soon as any recipe replaced part of it (agent D, round 2:
# D_rc2/1_office_wide_collapse_onto_sw.png). Built-up roofing is asphalt or
# felt over a deck; the damaged-asphalt map is dark on every family, is
# TEXTURED (a flat colour on a 22 m plate reads as a card) and is
# world-projected like everything else authored here.
_A_ROOF_URL = _MEGA + "Damaged_Asphalt.usda"


def _a_roof_mat(ctx):
    """The material an authored roof slab and its fragments take."""
    from pxr import Sdf, UsdShade
    path = ctx["parent"] + "/QuakeLooks/a_roof"
    mm = UsdShade.Material.Get(ctx["stage"], path)
    if not mm:
        try:
            import scene_generator as sg
            prim = ctx["stage"].DefinePrim(Sdf.Path(path))
            prim.GetReferences().AddReference(sg._join_asset_root(_A_ROOF_URL, ""))
            prim.Load()
            mm = UsdShade.Material.Get(ctx["stage"], path)
        except Exception as exc:
            print("[quake_flow] roof material unavailable ({0})".format(exc))
    return mm or ctx["mats"]["dark_concrete"]



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
    # NOT THE TILE'S OWN TEXTURE. A kit roof tile's base colour is the
    # family's façade ATLAS, sampled through UVs that pick out the roof patch;
    # projected triplanar onto a UV-less slab it paints WINDOWS across the
    # roof (seen on the office family in the first assembled city). And not
    # `mats["concrete"]` either — that is the PALE pavement map, which is what
    # made a half-replaced roof read as a bright plate. See `_a_roof_mat`.
    mat = _a_roof_mat(ctx)
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
    # REGISTER IT. Once a recipe has swapped a kit roof tile for a slab, the
    # tile is `dead` and no later recipe can find it through `_els`; on a
    # single-tile roof (`commercial` is one 22 x 18 m tile) that meant DG3's
    # `corner_fail` consumed the whole roof and `roof_hole`, which runs after
    # it, reported "25% of the roof, 0 pieces down" — an intact roof at the
    # grade whose whole point is a hole in it. `_a_roof_slabs` gives the later
    # recipe the boxes to work on instead.
    ctx.setdefault("roof_slabs", []).append(box)
    return box


def _break_box_like(ctx, e, n, timber=False, consume=0.0, dusty=False,
                    consume_pool=None):
    """Fracture a kit ROOF piece: swap it for a solid slab first.

    `dusty` is the collapse variant: the boards take the dark joist tint and
    a share of them go to mortar dust instead, and the consumption takes the
    BIGGEST pieces (`consume_pool` 1.05) — a roof deck that comes down into a
    heap should not put pale 3 m sheets on the crown of it."""
    from . import fracture
    box = _roof_box(ctx, e, thick=(0.14 if timber else ROOF_T))
    if not box:
        return []
    out = box + "_brk"
    made = fracture.fracture_prim(ctx["stage"], box, out, n_pieces=n,
                                  rng=ctx["nrng"],
                                  mode=("plank" if timber else "uniform"),
                                  aspect=((1.4, 3.0) if timber else None),
                                  rough=0.01, verbose=False, consume=consume,
                                  consume_pool=(consume_pool if consume_pool
                                                else (1.05 if dusty else 1.6)),
                                  min_volume_frac=0.0008)
    mat = _a_roof_mat(ctx)
    for pth in made:
        r = ctx["rng"].random()
        if dusty:
            _bind(ctx["stage"], pth,
                  _a_mat(ctx, "timber_dusty") if (timber and r < 0.5) else
                  _a_mat(ctx, "brick_dusty") if r < 0.8 else
                  _a_mat(ctx, "dust"))
        elif timber and r < 0.55:
            _bind(ctx["stage"], pth, ctx["mats"]["timber"])
        elif r < 0.4:
            _bind(ctx["stage"], pth, _a_mat(ctx, "concrete_dusty"))
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
    from . import damage
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    btype = ctx["info"]["type"]
    H = m["top"] - m["z0"]
    chosen = which or _pick_sides(ctx, sides)
    for side in chosen:
        ox, oy = _outward(m, side)
        for e in list(_els(ctx, mass=mass, side=side)):
            if e["role"] not in ("wall", "corner", "parapet", "parapet_corner", "balcony"):
                continue
            if e["storey"] < from_storey and e["role"] not in ("parapet", "parapet_corner"):
                continue
            # THE FOOT OF THE PEEL IS THE ONE EDGE THAT SURVIVES, so it is the
            # one that has to be right. `fracture_partial`'s smooth wobble over
            # 9-12 whole cells left a chain of 1.2 m planes on a course line;
            # `_break_split` steps the line along the courses (urm) or tears it
            # (rc) and refines every cell the line crosses.
            path = e["p"].get("prim_path")
            if (e["storey"] == from_storey and e["role"] == "wall"
                    and path and rng.random() < 0.8):
                z0 = e["z"] + e["h"] * rng.uniform(0.15, 0.5)
                tex = damage.bound_texture(ctx["stage"], path)
                st, lo = _break_split(
                    ctx, path, 10 + rng.randrange(4),
                    _a_zline_judge(m, side, z0, rng, btype=btype,
                                   amp=e["h"] * 0.30, loose_above=True),
                    _mat_fn(ctx, tex, 0.35),
                    static_mat=_clad_material(ctx["stage"], ctx["parent"],
                                              ctx["cache"], tex) if tex else None)
                for pth in lo:
                    v = 0.4 + rng.uniform(0.0, 0.8)
                    ctx["velocity"][pth] = (ox * v, oy * v, 0.05 * v)
                ctx["loose"] += lo
                ctx["static_extra"] += st
                e["dead"] = True
                continue
            # MORE, SMALLER PIECES AND MORE OF THEM THINNED. A kit module cut
            # into 7-11 cells sheds 1.5-3 m PLATES, and a flat plate with an
            # outward kick settles standing on edge — the fan on the pavement
            # came out as a row of boards on end with the authored windrow
            # invisible underneath. 10-15 cells and `consume` 0.34 (the skill's
            # 0.22 was tuned before the windrow carried the mass) puts the
            # rubble back on top and the recognisable panels among it.
            st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                            10 + rng.randrange(6), rng, nrng, ctx["mats"],
                            ctx["cache"], ctx["info"]["type"], inner_p=0.35,
                            partial=None, consume=0.34)
            # Outward speed grows with height: the wall rotates about its
            # foot, so the top leads.
            for pth in lo:
                zf = min(1.0, max(0.0, (e["z"] + e["h"] * 0.5 - m["z0"]) / max(H, 1.0)))
                v = 0.4 + 2.4 * zf + rng.uniform(-0.2, 0.3)
                ctx["velocity"][pth] = (ox * v, oy * v, 0.15 * v)
            # what leaves the building lands in sunlight on asphalt: the
            # 0.66-luma interior plaster and the pavement map's joint grid are
            # the "white paper" / "striped" debris of the round-1 review
            _a_dustify(ctx, lo)
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
        # ...and the storey UNDER the peel keeps a level top edge on the slab
        # line unless it is torn too (`from_storey` walls that drew the 20%
        # "no partial break" branch, and the whole of `from_storey - 1`).
        if from_storey >= 1:
            _a_ragged_courses(ctx, mass, from_storey, sides=(side,),
                              above=False, below=True, band=(0.3, 0.9), p=0.9)
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

    # 0) NO EDGE ON A KIT SEAM. The storey above the crushed one otherwise
    #    keeps a dead-level bottom edge and the one below a dead-level top
    #    edge, both exactly on a slab line — the two straight lines that made
    #    the first mid_storey bench read as a block lifted off with a pallet
    #    fork. Run this BEFORE the walls go, and carry the surviving
    #    fragments of the storey above with the block when it drops (they are
    #    not `_els` any more, so `above` below would miss them).
    # p HIGH: one module in four left with a clean bottom edge is one 5 m
    # ruler line across the seam, and that is the thing the eye finds.
    courses = _a_ragged_courses(ctx, mass, storey, band=(0.3, 0.95), p=0.92)
    # ...and BOTH slabs the gap exposes: the one under the crushed storey (it
    # stays put) and the one the block above sits on, whose edge is the pale
    # straight line across the seam in the first bench shot. The upper one is
    # about to be translated, so it takes no rebar tufts here and its statics
    # join `above`.
    _a_slab_rim(ctx, mass, storey, n_sides=2)
    rim_above = _a_slab_rim(ctx, mass, storey + 1, n_sides=2, bars=False)

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
        _a_dustify(ctx, lo + st)
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
                                       _a_mat(ctx, "plaster_dusty"), consume=0.5)
            fit["all"] = [q for q in fit["all"] if q != pth]
    # its columns -> stubs + chunks
    for cpath in fit["columns"].get((mass, storey), []):
        cx_, cy_, cz_, sx_, sy_, sz_, yaw_ = _box_dims(ctx["stage"], cpath)
        made = _break_box(ctx["stage"], cpath, 4, rng, nrng,
                          _a_mat(ctx, "concrete_dusty"), _a_mat(ctx, "dust"))
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
    # the torn bottom edges of the storey ABOVE ride down with the block; the
    # pieces that came away from it are already loose and fall onto the collar
    above += list(courses.get("above_static", [])) + list(rim_above)
    # DEDUPE, OR THE SLAB MOVES TWICE. `_transform_prims` post-multiplies per
    # occurrence, and `rim_above[0]` is also the value `fit["slabs"]` now holds
    # for storey+1, so it would be collected by both loops and drop 2 x `drop`.
    _seen, above = set(), [a for a in above if a]
    above = [a for a in above if not (a in _seen or _seen.add(a))]
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
            # `consume` WAS MISSING HERE. Every other collapse recipe thins the
            # kit shells (0.22-0.45) because they are open, single-sided foil
            # and the pile's mass is the authored heap; `pancake` kept all of
            # them, which is why its mound was 2.2k bodies of dark glazing and
            # white panel with no rubble visible between them.
            st, lo = _break(stage, ctx["parent"], e, ctx["tag"],
                            10 + rng.randrange(6), rng, nrng, ctx["mats"],
                            ctx["cache"], info["type"], inner_p=0.5,
                            partial=partial, consume=0.34)
            ox, oy = _outward(m, e["side"])
            H = max(1.0, m["top"] - m["z0"])
            for pth in lo:
                zf = (e["z"] - m["z0"]) / H
                v = 0.3 + 1.6 * zf
                ctx["velocity"][pth] = (ox * v * rng.uniform(0.5, 1.2),
                                        oy * v * rng.uniform(0.5, 1.2), 0.0)
            _a_dustify(ctx, lo + st)
            ctx["loose"] += lo + st
            e["dead"] = True
        # columns -> short chunks (they are what the slabs crush)
        for (cm, i), cols in fit["columns"].items():
            if cm != mt:
                continue
            for cpath in cols:
                made = _break_box(stage, cpath, 3, rng, nrng,
                                  _a_mat(ctx, "concrete_dusty"), _a_mat(ctx, "dust"))
                ctx["loose"] += made
        # PARTITIONS COME DOWN WITH THE FLOORS. Left static they stayed at
        # their storey heights over the stack — a grid of white panels in
        # mid-air on the first concrete bench.
        for pth in list(fit["partitions"]):
            if "_{0}_".format(mt) in pth:
                ctx["loose"] += _break_box(stage, pth, 4, rng, nrng,
                                           _a_mat(ctx, "plaster_dusty"),
                                           consume=0.5)
                fit["all"] = [q for q in fit["all"] if q != pth]
        # slabs: RE-AUTHOR as a stack (not simulated — a stack of thin boxes
        # is exactly what PhysX does worst), each with its own small tilt and
        # jitter, then the roof on top. The stack base is the mass's own z0
        # (a wing pancakes onto what the main body left).
        base = m["z0"] if mt == "main" else info["masses"]["main"]["z0"] + (
            len(info["masses"]["main"]["levels"]) * pitch + 0.4)
        stack, n_plates = [], 0
        from pxr import UsdShade
        for i in range(1, n_lv + 1):
            key = (mt, i)
            pth = fit["slabs"].get(key)
            if pth is None:
                continue
            # A PANCAKED SLAB DOES NOT KEEP ITS FORMWORK EDGE. The stack was
            # re-authored from the intact `_box` slabs, so every plate in the
            # pile had four ruler-straight sides — visible from the air as a
            # deck of cards. One or two sides of each are torn off along a
            # wandering line and the pieces join the pile.
            cur, sts = pth, []
            # THE TOP PLATE IS THE ONE THE NADIR CAMERA SEES, so it loses
            # three or four sides; the ones buried in the stack lose one or
            # two, which is all that shows and all the fracture budget is
            # worth. (The first pass tore 1-2 sides on every plate and the
            # DG5 top view was a pale tiled rectangle with two ruler edges.)
            allside = ["S", "E", "N", "W"]
            rng.shuffle(allside)
            top_plate = (i >= n_lv - 1)
            sides = allside[:(3 + (1 if rng.random() < 0.5 else 0))
                            if top_plate else (2 if rng.random() < 0.45 else 1)]
            for sd in sides:
                km = UsdShade.MaterialBindingAPI(
                    stage.GetPrimAtPath(cur)).ComputeBoundMaterial()[0]
                d = rng.uniform(0.5, 1.6)
                rem, strip = _split_strip(ctx, cur, m, sd, d + 1.5,
                                          ctx["mats"]["concrete"])
                st, lo = _break_split(
                    ctx, strip, 10 + rng.randrange(4),
                    _edge_judge(m, sd, d, rng, btype=info["type"]),
                    lambda: (_a_mat(ctx, "concrete_dusty") if rng.random() < 0.6
                             else _a_mat(ctx, "dust")),
                    min_volume_frac=0.0008, static_mat=km if km else None,
                    refine_max=6, max_loose_m=2.6)
                # `cur` is deactivated by the next split, so only the FINAL
                # remainder plus every side's statics ride the stack
                cur, sts = rem, sts + st
                ctx["loose"] += lo
                ctx["authored"].append(rem)
                # NO BARS HERE: this slab has not been moved to the stack yet
                # and `_rebar_tuft` authors in world space with no xform, so a
                # tuft placed now would hang where the storey used to be. The
                # tufts below run after `_transform_prims`.
            group = [cur] + sts
            fit["slabs"][key] = cur
            k = i - 1
            z = base + pitch * (k + 0.5)
            tilt = rng.uniform(-4.0, 4.0)
            taxis = rng.choice(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
            jx, jy = rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6)
            self_z = m["levels"][i] - SLAB_T[info["type"]] / 2.0 if i < n_lv else m["top"] - SLAB_T[info["type"]] / 2.0
            M = _translate(jx, jy, z - self_z) * _rot_about(
                (m["cx"], m["cy"], z), taxis, tilt)
            _transform_prims(stage, group, M)
            # A PANCAKED SLAB IS NOT A CLEAN FLOOR. The fit-out binds slabs
            # with the pale pavement map, which is right for a floor seen
            # through a hole and reads from the air as a sheet of bathroom
            # tile on the crown of the pile; every plate in the stack is under
            # dust and broken concrete, so it takes the same dark asphalt map
            # the authored roof slabs do.
            for q in group:
                _bind(stage, q, _a_roof_mat(ctx))
            stack += group
            n_plates += 1
            # rebar tufts at slab edges
            for q in range(2):
                _rebar_tuft(ctx, group[0], z + 0.1, n=2, length=(0.4, 1.0))
        ctx["static_extra"] += stack
        # the contents of the building are UNDER the stack, not on it
        for (pm, i), props in fit["props"].items():
            if pm == mt:
                _a_bury_props(ctx, props, base, pitch * n_lv * 0.6, keep=0.25)
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
                            "{3:.1f} m".format(mt, n_plates, pitch,
                                               base + pitch * n_lv))


# ---------------------------------------------------------------------------
# THE DEBRIS PALETTE — round 2, "the DG5 heaps read as a LUMBER YARD"
# ---------------------------------------------------------------------------
#
# The first city's collapse heaps at 40 m were pale timber sheets, clean
# plaster and whole striped mattresses sitting on the crown, with almost no
# brick anywhere. What the reconnaissance shows is the opposite: an Amatrice /
# Christchurch / Antakya URM heap is 70-85 % brick and stone rubble with
# plaster fines, ALL of it under a coat of mortar dust; timber joists are a
# minority of dark broken sticks; and the contents of the building are buried,
# not on top. Three tints `materials()` does not carry, added on demand so the
# shared palette is untouched.
A_DEBRIS = {
    # LINEAR ALBEDO, NOT SCREEN GREY — agent B's finding, and it is why the
    # first two passes at this table did nothing. `damage._pbr` writes these
    # straight into OmniPBR's `diffuse_color_constant` and the renderer encodes
    # to sRGB afterwards, so screen ~= linear ** 0.42: the 0.52 "mid grey dust"
    # of the first attempt came out at 0.75 on screen (white), and even 0.42
    # came out at 0.70. Inverse: linear = screen ** 2.38.
    "dust":         ((0.135, 0.127, 0.115), 1.0),   # -> ~0.42 screen
    "timber_dusty": ((0.062, 0.046, 0.032), 0.98),  # -> ~0.30 screen, brown
    "brick_dusty":  ((0.100, 0.057, 0.044), 1.0),   # -> ~0.38/0.30/0.27
    # crushed concrete in a heap: darker than the pavement it lies on
    "concrete_dusty": ((0.078, 0.077, 0.072), 1.0),  # -> ~0.34 screen
}

# NOTE ON THE KEYS: `brick` and `concrete` are the megascans TEXTURES (fine at
# any size); every other entry has to be one of the dark tints above or
# `plaster_dusty`, because the rest of the flat palette (`mortar` 0.58,
# `dark_concrete` 0.40, `plaster` 0.66) is LINEAR albedo and renders at 0.79 /
# 0.68 / 0.84 on screen — i.e. white. Do not put them in a heap.
HEAP_MIX = {
    "urm": (("brick", 0.28), ("brick_dusty", 0.30), ("dust", 0.16),
            ("plaster_dusty", 0.14), ("concrete_dusty", 0.06),
            ("timber_dusty", 0.06)),
    "rc": (("concrete", 0.24), ("concrete_dusty", 0.34), ("dust", 0.18),
           ("plaster_dusty", 0.12), ("brick_dusty", 0.10), ("timber_dusty", 0.02)),
}
HEAP_MIX["rc_glass"] = HEAP_MIX["rc"]


def _a_mat(ctx, key):
    """A material by key: the debris tints above, else the shared palette."""
    if key in A_DEBRIS:
        from . import damage
        from pxr import UsdShade
        path = ctx["parent"] + "/QuakeLooks/a_" + key
        mm = UsdShade.Material.Get(ctx["stage"], path)
        if not mm:
            rgb, rough = A_DEBRIS[key]
            mm = damage._pbr(ctx["stage"], path, rgb, rough)
        return mm
    return ctx["mats"].get(key) or ctx["mats"]["plaster"]


def _a_mix_mat(ctx, btype, dusty=0.0):
    """Draw one material from `HEAP_MIX`. `dusty` (0..1) shifts the draw
    toward the dust tints — the crown of a heap is what the fines settle on,
    so the top of a dome comes out greyer than its flanks."""
    rng = ctx["rng"]
    mix = HEAP_MIX.get(btype, HEAP_MIX["rc"])
    if dusty > 0.0 and rng.random() < dusty:
        return _a_mat(ctx, "dust" if rng.random() < 0.6 else "plaster_dusty")
    r, acc = rng.random(), 0.0
    for key, share in mix:
        acc += share
        if r < acc:
            return _a_mat(ctx, key)
    return _a_mat(ctx, mix[-1][0])


def _a_dustify(ctx, paths, p=0.7):
    """Re-tint the fragments of a COLLAPSE after the fact.

    `_chunk_material` draws the shared palette, which is right for a piece
    still on the building and wrong for one lying in a heap: `plaster` at 0.66
    luma is the "random white debris", and `concrete` is the Worn_Pavement map
    whose joint grid, on a 0.6 m chunk, is the green-and-white "striped
    mattress" the DG5 city review picked out. Both go dusty for a share `p` of
    the pieces — a share, not all, so the pile keeps some variety."""
    from pxr import UsdShade
    rng = ctx["rng"]
    mats = ctx["mats"]
    # `None` = "draw a dark debris tint" (see below). `mortar` at 0.58 luma is
    # the second-brightest thing in the palette after `plaster` and it is 30 %
    # of the URM inner draw, so a collapse heap came out pale even after the
    # plaster was fixed.
    swap = {}
    for src, dst in (("plaster", "plaster_dusty"), ("concrete", None),
                     ("mortar", None), ("dark_concrete", None)):
        s = mats.get(src)
        if s is None:
            continue
        swap[s.GetPath()] = (mats.get(dst) if dst else None)
    for q in paths:
        pr = ctx["stage"].GetPrimAtPath(q)
        if not pr or not pr.IsValid():
            continue
        if rng.random() >= p:
            continue
        try:
            b = UsdShade.MaterialBindingAPI(pr).GetDirectBinding().GetMaterial()
        except Exception:
            continue
        if not b:
            continue
        key = b.GetPath()
        if key not in swap:
            continue
        alt = swap[key]
        if alt is None:
            r = rng.random()
            alt = (_a_mat(ctx, "concrete_dusty") if r < 0.4
                   else _a_mat(ctx, "dust") if r < 0.75
                   else _a_mat(ctx, "brick_dusty"))
        if alt is not None:
            _bind(ctx["stage"], q, alt)


def _a_bury_props(ctx, props, base_z, heap_h, keep=0.3):
    """The contents of a collapsed building are UNDER the rubble.

    Left as loose bodies the props settle on TOP of the authored heap, and an
    intact desk or a striped mattress on the crown of a DG5 pile is the single
    most artificial thing in the shot (reviewer, on the first city). Most are
    deactivated — they are inside a solid heap volume and nothing would see
    them; a `keep` share are dropped to the bottom of the pile, tipped, and
    made STATIC so the heap chunks author over them and they read as a corner
    of something buried."""
    rng = ctx["rng"]
    kept = []
    for p in props:
        if not p:
            continue
        if rng.random() > keep:
            _deactivate(ctx["stage"], p)
            continue
        c = _pivot_of(ctx, p)
        z = base_z + rng.uniform(0.0, max(0.25, heap_h * 0.4))
        ax = (rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), 0.0)
        if abs(ax[0]) + abs(ax[1]) < 1e-3:
            ax = (1.0, 0.0, 0.0)
        M = _translate(rng.uniform(-1.2, 1.2), rng.uniform(-1.2, 1.2),
                       z - c[2]) * _rot_about((c[0], c[1], z), ax,
                                              rng.uniform(20.0, 75.0))
        _transform_prims(ctx["stage"], [p], M)
        ctx["static_extra"].append(p)
        kept.append(p)
    return kept


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

    def _mat(dusty=0.0):
        if mat_fn is not None:
            return mat_fn()
        return _a_mix_mat(ctx, btype, dusty=dusty)

    def _chunk(lx, ly, z, s, dusty=0.0):
        wx, wy = _to_world(m, lx, ly)
        if not _c_ok(ctx, wx, wy):
            return                         # off the plate (see `_c_ok`)
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy, z, s, s * rng.uniform(0.5, 1.0),
             s * rng.uniform(0.35, 0.7), rng.uniform(0, 180), _mat(dusty))
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
            # SIZE IS A POWER LAW, NOT A UNIFORM DRAW. `uniform(0.35, 1.7)`
            # makes every second chunk a 1 m block and the heap reads as a
            # crate of boxes; rubble is mostly 0.3-0.6 m with the occasional
            # lump. 0.28 + 1.5*u^2.2 has a median near 0.5 m and a tail.
            sz = (0.28 + 1.5 * rng.random() ** 2.2) * (1.0 if r < 0.75 else 0.7)
            zc = base + rng.uniform(0.0, zmax) + sz * 0.15
            # the crown carries the fines
            dusty = 0.35 * max(0.0, 1.0 - r * 1.3) if h > 0.5 else 0.0
            _chunk(u * RX, v * RY, zc, sz, dusty)
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
                sz = 0.16 + (0.42 if depth < 1.0 else 0.82) * rng.random() ** 1.8
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
    from . import damage
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
                # many small pieces, most of them lost into the heap.
                ctx["loose"] += _break_box_like(ctx, e, 52, timber=True,
                                                consume=0.68, dusty=True)
                e["dead"] = True
                continue
            # THE STUB'S TOP IS A STEPPED EDGE, NOT A LEVEL ONE. `_break`'s
            # partial path is `fracture_partial`, whose break line is a smooth
            # wobble over 8-12 whole cells — on a 4 m module that is a chain of
            # 1.2 m planes at one height, which is what the review saw as "the
            # shear is along a straight line". The heap sits against this stub,
            # so it is the most-looked-at edge in the whole recipe.
            path = e["p"].get("prim_path")
            if e["storey"] == 0 and keep_stub and e["role"] == "wall" and path:
                z0 = e["z"] + e["h"] * rng.uniform(0.25, 0.62)
                tex = damage.bound_texture(stage, path)
                st, lo = _break_split(
                    ctx, path, 10 + rng.randrange(4),
                    _a_zline_judge(m, e["side"], z0, rng, btype=info["type"],
                                   amp=e["h"] * 0.30, loose_above=True),
                    _mat_fn(ctx, tex, 0.5),
                    static_mat=_clad_material(ctx["stage"], ctx["parent"],
                                              ctx["cache"], tex) if tex else None)
            else:
                # SMALLER AND THINNER. 8-12 cells on a kit module sheds 1.2-2 m
                # plates, and they land last, on the CROWN of the heap, so the
                # pile reads as sheets over rubble however good the rubble is.
                st, lo = _break(stage, ctx["parent"], e, ctx["tag"],
                                12 + rng.randrange(6), rng, nrng, ctx["mats"],
                                ctx["cache"], info["type"], inner_p=0.45,
                                partial=None, consume=0.42)
            ox, oy = _outward(m, e["side"])
            for pth in lo:
                zf = min(1.0, max(0.0, (e["z"] - m["z0"]) / H))
                v = 0.3 + 1.8 * zf
                ctx["velocity"][pth] = (ox * v * rng.uniform(0.4, 1.1),
                                        oy * v * rng.uniform(0.4, 1.1), 0.0)
            _a_dustify(ctx, lo)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
        for (sm, i), pth in fit["slabs"].items():
            if sm == mt and pth:
                # A JOISTED TIMBER FLOOR COMES APART INTO BOARDS AND JOISTS,
                # and then it is BURIED. Round 2 (reviewer, on the first city:
                # "the DG5 heaps read as a LUMBER YARD"): 30-40 plank cells out
                # of a 22 x 18 m deck are 3.5 x 1.2 m boards, five storeys of
                # them cover the brick completely, and bound to the pale
                # `timber` map they are the brightest thing in the shot. So:
                # more, smaller cells; `consume` up to 0.72 with the pool at
                # 1.05 so it takes the BIGGEST ones; and the dusty joist tint,
                # with 45% of the pieces in mortar dust instead.
                made = _break_box(stage, pth, 54 + rng.randrange(16), rng, nrng,
                                  _a_mat(ctx, "timber_dusty"),
                                  _a_mat(ctx, "brick_dusty"), 0.45,
                                  mode="plank", aspect=(1.8, 3.6), consume=0.80,
                                  consume_pool=1.05)
                ctx["loose"] += made
        for pth in fit["partitions"]:
            if "_{0}_".format(mt) in pth:
                ctx["loose"] += _break_box(stage, pth, 8, rng, nrng,
                                           _a_mat(ctx, "plaster_dusty"),
                                           consume=0.45)
        for (pm, i), props in fit["props"].items():
            if pm == mt:
                _a_bury_props(ctx, props, m["z0"], H * 0.28)
        # THE PILE IS H/3 (FEMA's 0.33 air-space factor; Amatrice LiDAR gives
        # ~5 m heaps for 2-4 storey stone). Masonry spreads 0.4-0.7 H.
        _heap(ctx, m, m["z0"], H * 0.28, rng.uniform(0.2, 0.34), fill=True)


def r_tilt_sink(ctx, tilt_deg=8.0, sink_m=1.0, azimuth=None, max_drop_m=2.6):
    """Foundation failure: the whole building, fit-out included, LEANS TOWARD
    `azimuth` and sinks. The shell stays intact — the whole story is in the
    ground, and `_c_ground_response` tells both halves of it (heave and torn
    pavement on the low side, the opened gap and the raft edge on the high).

    The rotation sign changed in round 2: the old code rotated by -theta about
    the base edge on `azimuth`, which leans the building toward the OPPOSITE
    side (measured, `_c_read_M`) while berming `azimuth` — so the heaved soil
    was on the side that ROSE. See `_c_tilt_matrix` for the pivot."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    side = azimuth or rng.choice(["S", "E", "N", "W"])
    M, g = _c_tilt_matrix(m, side, tilt_deg, sink_m, max_drop_m=max_drop_m)
    raft = _raft(ctx, m)
    paths = _everything(ctx) + [raft] + list(ctx.pop("c_carry", []))
    _transform_prims(ctx["stage"], paths, M)
    ctx["static_extra"] += paths
    _c_ground_response(ctx, m, low_side=g["low"], drop_m=g["drop"],
                       rise_m=g["rise"], tag="tilt")
    ctx["notes"].append(
        "tilt_sink: {0:.1f} deg toward {1}, sunk {2:.2f} m (low edge -{3:.2f} m, "
        "high edge +{4:.2f} m)".format(g["tilt"], side, g["sink"], g["drop"],
                                       g["rise"]))


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


def _raft(ctx, m, tag="raft", dress=True):
    """The foundation raft: a concrete slab under the footprint, top just
    below grade so it is invisible until a tilt lifts one edge out.

    When a tilt DOES lift it, its edge and underside are the whole "exposed
    foundation" read, so it is a dirty world-projected concrete (the flat
    `dark_concrete` came out as a white plate on the round-2 bench) with earth
    still clinging along the edge. The clinging clods have to ride the same
    rigid transform as the slab, so they are parked on `ctx["c_carry"]` for
    the caller to add to its transform list."""
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
    _box(ctx["stage"], path, m["cx"], m["cy"], m["z0"] - 0.06 - RAFT_T / 2.0,
         m["W"] + 1.2, m["D"] + 1.2, RAFT_T, m["yaw"], _c_look(ctx, "raft"))
    ctx["authored"].append(path)
    ctx["static_extra"].append(path)
    if dress:
        ctx["c_carry"] = list(ctx.get("c_carry", [])) + _c_raft_dress(ctx, m)
    return path


def _c_raft_dress(ctx, m, tag="raftsoil"):
    """Earth still stuck to the raft: clods along its edge, some hanging under
    it. Rides the raft's transform (`ctx["c_carry"]`)."""
    rng = ctx["rng"]
    made = []
    W, D = m["W"] + 1.2, m["D"] + 1.2
    peri = 2.0 * (W + D)
    mm = dict(m, W=W, D=D)
    for _k in range(int(peri / 1.5)):
        (lx, ly), (nx, ny) = _c_perim(mm, rng.uniform(0.0, peri))
        d = rng.uniform(-0.25, 0.35)
        wx, wy = _to_world(m, lx + nx * d, ly + ny * d)
        if not _c_ok(ctx, wx, wy):
            continue
        sz = rng.uniform(0.22, 0.75)
        z = m["z0"] - 0.06 - RAFT_T * rng.uniform(0.05, 1.05)
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy, z, sz, sz * rng.uniform(0.6, 1.1),
             sz * rng.uniform(0.5, 1.0), rng.uniform(0, 180),
             _c_look(ctx, "soil" if rng.random() < 0.75 else "silt"))
        made.append(path)
    ctx["authored"] += made
    return made


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
        # ON THE PLATE ONLY. The soft-soil ellipse can cross the plate edge,
        # and a two-city run put four boils on bare ground outside the city.
        # `ctx["bounds"]` is set by `_c_ground_response(bounds=...)` and by the
        # city's ground pass; absent on the bench and the bake.
        if not _c_ok(ctx, cx, cy):
            continue
        ph = rng.uniform(0, 6.28)
        # A LOW DOME, NOT A STARFISH: 24 sides, a gentle two-harmonic wobble,
        # and the centre raised so the fan catches the light as a mound.
        N = 24
        pts = [Gf.Vec3f(cx, cy, m["z0"] + 0.02 + 0.14 * r)]
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
        _bind(ctx["stage"], path, _c_look(ctx, "silt"))
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


# ---------------------------------------------------------------------------
# THE GROUND RESPONSE — what the soil did under a building that leaned, sank
# or was levered out of its footprint.
# ---------------------------------------------------------------------------
# Round-1 review: "for tilted buildings it looks unnatural because there isn't
# soil/debris where it tilted into or where it got uprooted from". A leaning
# building has TWO ground signatures and the old `_berm` authored half of one:
#
#   LOW side   the footing ploughs down and squeezes a wedge of soil and
#              pavement up against the wall — a mound with a ragged crest,
#              slabs riding on it tipped up toward the wall, a kerb broken off
#              its line, soil spilt onto the road. (Adapazari 1999, UC Davis
#              Geotechnical Photo Album: "a mound of wet grey silt is squeezed
#              out on the low side".)
#   HIGH side  the raft levers OUT of the ground — an open trench along the
#              wall with torn edges, the foundation edge showing under the wall
#              base, pavement slabs pulled up and leaning, soil clods.
#              ("its shallow foundation raft is levered clear of the soil on
#              the high side", same source; Niigata 1964 Kawagishi-cho.)
#   both       fissures out of the corners, and sand boils when the case is a
#              liquefaction one (research s3.13: settlement 0.004-1.6 m).
#
# Pure settlement has a third signature: a subsidence ring, the pavement
# dished into the wall all round and cracked into plates, a shallow gap at the
# wall and a mud line on the facade (Christchurch 2010-11: 40-50 cm total
# settlement, building tilts of only 0.4-0.5 deg).
#
# Everything here works BOTH inside a recipe (a full `ctx`, bench and bake)
# and at city-assembly time (`_c_ctx(stage, parent, mats, rng)`), because the
# city's mild lean (`quake._tilt_prim`) is a transform on a referenced
# archetype and had no ground response at all — those were the buildings in
# the round-1 city with nothing around them.

C_TILT_PIVOT_FRAC = 0.35   # pivot this far from the centre toward the low side
C_DROP_REF = 1.8           # low-edge drop (m) at which crest/reach max out
C_RISE_REF = 0.9           # high-edge rise (m) at which the gap maxes out
C_CREST_M = (0.35, 1.45)   # heave crest height, min drop -> C_DROP_REF
C_REACH_M = (1.8, 4.8)     # how far the wedge spreads from the wall line
C_GAP_W = (0.18, 0.80)     # opened gap on the high side: width
C_GAP_D = (0.30, 1.00)     #                             : depth
C_BOIL_DROP = 0.30         # below this drop it is not a liquefaction case
C_FISSURE_M = (2.0, 6.0)   # corner fissure length
C_SEG_M = 0.8              # crest / trench sampling step: fine enough to be jagged
C_MAX_DROP_M = 2.4         # city mild tilt: keep the low corner out of the basement
C_MIN_RISE_M = 0.12        # a lean always lifts its far edge at least this far
C_MAX_RISE_M = 3.0         # ...and never lifts it further than this


def _c_lerp(pair, t):
    a, b = pair
    return a + (b - a) * min(1.0, max(0.0, float(t)))


def _c_ctx(stage, parent, mats, rng, tag="c"):
    """A minimal `ctx` for callers with no building context (the city
    assembly). Mirrors `quake.ground_effects._ctx`."""
    return {"stage": stage, "parent": parent, "rng": rng, "mats": mats,
            "tag": tag, "authored": [], "static_extra": [], "loose": [],
            "velocity": {}, "notes": [], "n_uid": 0, "info": {"type": "rc"}}


def _c_ok(ctx, wx, wy):
    """Is (wx, wy) inside the plate? `ctx["bounds"]` is (x0, y0, x1, y1) in the
    same frame the helpers author in, or absent (the bench and the bake, where
    there is no plate). Set by `_c_ground_response(bounds=...)`.

    The reviewer's two-city run put four sand boils on bare ground OUTSIDE the
    city, because the soft-soil ellipse crossed the plate edge: every scatter
    in this file is gated on this."""
    b = ctx.get("bounds")
    if not b:
        return True
    return b[0] <= wx <= b[2] and b[1] <= wy <= b[3]


def _c_clampxy(ctx, wx, wy):
    """(wx, wy) pulled back onto the plate — for MESH vertices, where dropping
    the point would tear the surface."""
    b = ctx.get("bounds")
    if not b:
        return wx, wy
    return (min(max(wx, b[0]), b[2]), min(max(wy, b[1]), b[3]))


def _c_flank(side, turn):
    """The side `turn` quarter-turns round from `side` (S -> E -> N -> W)."""
    order = ("S", "E", "N", "W")
    return order[(order.index(side) + turn) % 4]


def _c_noise(rng, freqs=(0.45, 1.3, 3.1), amps=(0.55, 0.30, 0.15)):
    """A band-limited wobble in roughly [-1, 1] along a distance in METRES:
    wavelengths ~14 / 4.8 / 2.0 m, so a crest sampled at C_SEG_M is jagged at
    the scale of a person, not a smooth extrusion."""
    ph = [rng.uniform(0.0, 6.2832) for _ in freqs]

    def f(s):
        return sum(a * math.sin(s * w + p) for a, w, p in zip(amps, freqs, ph))
    return f


# WORLD-PROJECTED LOOKS FOR THE AUTHORED GROUND.
# `materials()` gives the kit's megascans materials as REFERENCED `.usda`
# packs, and those sample in UV space. Every prim in this file is authored
# with no UVs at all, so on the first round-2 bench the soil fans came out as
# flat CREAM discs ("spilled paper" on the asphalt), the pavement slabs as
# pale green-grey quads and the levered-out raft as a white plate. Same trap
# the skill already records for the AEC `Dirt.usda`, one layer further in:
# it is not enough to pick a megascans pack, it has to be bound through
# `damage._pbr(texture=...)`, which sets `project_uvw` + `world_or_object` and
# scales in repeats per METRE.
# (texture, tint, roughness, repeats per metre, albedo_brightness, desaturation)
#
# THE TINT DOES NOTHING ON ITS OWN. `damage._pbr` sets `diffuse_color_constant`
# to the tint, and `planks.wood_material`'s comment says that multiplies the
# map — it does not, at least not in this OmniPBR: three benches at 0.40, 0.33
# and 0.22 rendered the soil the SAME bright orange, while changing the
# TEXTURE changed the look immediately. OmniPBR's albedo-map controls are
# `albedo_brightness` (multiplier) and `albedo_desaturation`, and those are
# set below through the shader directly. Desaturation matters as much as
# brightness: a neutral multiplier cannot take the orange out of a mud map,
# it only makes it a darker orange.
_C_TEX = {
    "soil":  ("megascans/Soil_Mud/T_pjuph20_1K_B.jpg", (0.22, 0.21, 0.20), 0.98, (0.70, 0.70), 0.42, 0.50),
    "silt":  ("megascans/Dirt_Rough/T_yd0lfcqcc_1k_B.png", (0.30, 0.30, 0.31), 0.96, (0.55, 0.55), 0.55, 0.60),
    # NOT Worn_Pavement: its map carries green moss in the joints, and a 1.2 m
    # kerb block at 0.38 repeats/m showed one big square of it — a row of them
    # along a wall read as green mosaic tiles. Damaged_Asphalt is a plain grey
    # cracked surface; brightened it is concrete, darkened it is the road.
    "pave":  ("megascans/Damaged_Asphalt/T_vizcebf_2K_B.png", (0.60, 0.59, 0.57), 0.90, (0.55, 0.55), 0.80, 0.25),
    "asph":  ("megascans/Damaged_Asphalt/T_vizcebf_2K_B.png", (0.38, 0.38, 0.37), 0.92, (0.30, 0.30), 0.70, 0.35),
    "raft":  ("megascans/Damaged_Asphalt/T_vizcebf_2K_B.png", (0.32, 0.32, 0.31), 0.95, (0.28, 0.28), 0.60, 0.35),
    "brick": ("megascans/Brick_Wall_Worn/T_sexkaitb_1K_B.jpg", (0.42, 0.36, 0.32), 0.92, (0.70, 0.70), 0.72, 0.20),
}
_C_FALLBACK = {"soil": "soil", "silt": "soil", "pave": "concrete",
               "asph": "dark_concrete", "raft": "dark_concrete", "brick": "brick"}


def _c_look(ctx, key):
    """One of `_C_TEX`, built once and cached IN the shared `mats` dict (so a
    bench row or a city pays for it once), falling back to the kit material of
    the same kind if the texture will not resolve."""
    mats = ctx["mats"]
    k = "c_" + key
    got = mats.get(k)
    if got is not None:
        return got
    rel, rgb, rough, scale, bright, desat = _C_TEX[key]
    try:
        import scene_generator as sg
        from pxr import Gf, Sdf, UsdShade
        from . import damage
        path = "{0}/QuakeLooks/c_{1}".format(ctx["parent"], key)
        got = damage._pbr(
            ctx["stage"], path, rgb, rough, tint=rgb, scale_uv=scale,
            texture=sg._join_asset_root(
                "airstack://scene_gen/assets/materials/" + rel, ""))
        sh = UsdShade.Shader.Get(ctx["stage"], path + "/Shader")
        if sh:
            sh.CreateInput("albedo_brightness",
                           Sdf.ValueTypeNames.Float).Set(float(bright))
            sh.CreateInput("albedo_desaturation",
                           Sdf.ValueTypeNames.Float).Set(float(desat))
            # belt and braces: whichever of the two names this OmniPBR build
            # honours as the map multiplier
            sh.CreateInput("diffuse_tint",
                           Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    except Exception as exc:
        print("[quake_flow] ground look {0} unavailable ({1})".format(key, exc))
        got = mats.get(_C_FALLBACK[key])
    mats[k] = got
    return got


def _c_perim(m, d):
    """(local point, outward normal) at arc length `d` round the footprint,
    from the SW corner, running S, E, N, W (the `_berm` order)."""
    W, D = m["W"], m["D"]
    P = 2.0 * (W + D)
    d = d % P
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


def _c_side_range(m, side):
    W, D = m["W"], m["D"]
    return {"S": (0.0, W), "E": (W, W + D), "N": (W + D, 2 * W + D),
            "W": (2 * W + D, 2 * W + 2 * D)}[side]


def _c_along(m, side):
    """(world along-side unit vector, its compass angle in degrees). This is
    the LEFT perpendicular of the outward vector, the axis every tip in this
    file rotates about."""
    ox, oy = _outward(m, side)
    ax, ay = -oy, ox
    return (ax, ay), math.degrees(math.atan2(ay, ax))


def _c_tilt_matrix(m, low_side, tilt_deg, sink_m, pivot_frac=C_TILT_PIVOT_FRAC,
                   max_drop_m=None, min_rise_m=C_MIN_RISE_M,
                   max_rise_m=C_MAX_RISE_M):
    """The rigid transform of a lean-and-sink, plus what it does to the ground.

    The building leans TOWARD `low_side` (+theta about the left perpendicular
    of that side's outward vector drops the mass on that side — the same rule
    the balconies and droops use).

    The pivot is NOT the base edge. A bearing-capacity failure rotates the
    footing about a point INSIDE the footprint, and that is the only way the
    raft on the far side comes clear of the ground: with an edge pivot the
    high side merely sinks less, never rises, so it has no story to tell —
    which is exactly what round 1 looked like. `pivot_frac` is how far the
    pivot sits from the centre toward the low side, as a fraction of the
    half-width. `max_drop_m` trims the sink so a wide building's low corner
    does not disappear into a basement, and `min_rise_m` trims it so the high
    edge always comes at least that far clear (a lean whose high side stays
    buried is a SETTLEMENT with a slope on it, and has nothing to show).

    Returns (M, {"low", "high", "drop", "rise", "pivot", "tilt", "sink"}),
    `drop`/`rise` in metres relative to grade at the two edge midpoints.
    """
    off = {"S": (0.0, -1.0), "N": (0.0, 1.0),
           "W": (-1.0, 0.0), "E": (1.0, 0.0)}[low_side]
    half = (m["D"] if low_side in ("S", "N") else m["W"]) / 2.0
    tilt = abs(float(tilt_deg))
    sink = abs(float(sink_m))
    s = math.sin(math.radians(tilt))
    if max_drop_m is not None:
        lim = float(max_drop_m)
        # spend the sink first...
        over = sink + half * (1.0 - pivot_frac) * s - lim
        if over > 0.0:
            sink = max(0.0, sink - over)
    # ...then the ANGLE, from both ends: a wide raft cannot rotate as far as a
    # narrow one before its low edge is in the basement or its high edge is a
    # storey in the air. This is the geometric half of Adapazari's H/B > 2
    # rule — a 32 m block at 18 deg is 10 m of corner-to-corner height
    # difference however the pivot is placed, and the first office bench had
    # one edge 6.65 m up, which no photograph of a wide block shows.
    caps = []
    if max_drop_m is not None:
        caps.append((float(max_drop_m) - sink) / max(half * (1.0 - pivot_frac), 1e-3))
    if max_rise_m is not None:
        caps.append((float(max_rise_m) + sink) / max(half * (1.0 + pivot_frac), 1e-3))
    if caps:
        cap = max(0.02, min(caps))
        if s > cap:
            s = min(1.0, cap)
            tilt = math.degrees(math.asin(s))
    if min_rise_m is not None and tilt > 1.0:
        short = float(min_rise_m) - (half * (1.0 + pivot_frac) * s - sink)
        if short > 0.0:
            sink = max(0.0, sink - short)
    lx = off[0] * (m["W"] / 2.0) * pivot_frac
    ly = off[1] * (m["D"] / 2.0) * pivot_frac
    px, py = _to_world(m, lx, ly)
    (ax, ay), _ang = _c_along(m, low_side)
    M = _rot_about((px, py, m["z0"]), (ax, ay, 0.0), tilt) * _translate(0.0, 0.0, -sink)
    return M, {"low": low_side, "high": _opposite(low_side),
               "drop": sink + half * (1.0 - pivot_frac) * s,
               "rise": half * (1.0 + pivot_frac) * s - sink,
               "pivot": (px, py), "tilt": tilt, "sink": sink}


def _c_read_M(m, M):
    """(low_side, drop, rise) MEASURED from a transform: where the four base
    edge midpoints ended up. Never assume a sign — another recipe's rotation
    convention is not this one's."""
    from pxr import Gf
    z0 = m["z0"]
    z = {}
    for sd in ("S", "E", "N", "W"):
        lx, ly = {"S": (0.0, -m["D"] / 2.0), "N": (0.0, m["D"] / 2.0),
                  "W": (-m["W"] / 2.0, 0.0), "E": (m["W"] / 2.0, 0.0)}[sd]
        wx, wy = _to_world(m, lx, ly)
        z[sd] = M.Transform(Gf.Vec3d(wx, wy, z0))[2]
    low = min(z, key=lambda k: z[k])
    return low, z0 - z[low], z[_opposite(low)] - z0


def _c_fall_side(m, M):
    """Which way the mass actually swung: where the top of the building went."""
    from pxr import Gf
    p = M.Transform(Gf.Vec3d(m["cx"], m["cy"], m["top"]))
    dx, dy = p[0] - m["cx"], p[1] - m["cy"]
    a = math.radians(-m["yaw"])
    lx = dx * math.cos(a) - dy * math.sin(a)
    ly = dx * math.sin(a) + dy * math.cos(a)
    if abs(lx) >= abs(ly):
        return "E" if lx > 0 else "W"
    return "N" if ly > 0 else "S"


# --- authoring primitives --------------------------------------------------
def _c_ribbon(ctx, kind, rows, mat, closed=False):
    """A quad mesh from equal-length rows of points (the cross-sections of a
    wedge, a trench or a crater). Double-sided: these are open surfaces."""
    from pxr import Sdf, UsdGeom, Vt
    from pxr import Gf
    pts, faces, counts = [], [], []
    R = len(rows[0])
    for row in rows:
        for q in row:
            wx, wy = _c_clampxy(ctx, float(q[0]), float(q[1]))
            pts.append(Gf.Vec3f(wx, wy, float(q[2])))
    n = len(rows)
    for i in range(n if closed else n - 1):
        a, b = i * R, ((i + 1) % n) * R
        for j in range(R - 1):
            faces += [a + j, b + j, b + j + 1, a + j + 1]
            counts.append(4)
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], kind, ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    _bind(ctx["stage"], path, mat)
    ctx["authored"].append(path)
    return path


def _c_plate(ctx, cx, cy, cz, r, thick, yaw_deg, tilt_axis_deg, tilt_deg, mat,
             elong=1.0, tag="plate"):
    """A BROKEN pavement slab: an irregular 5-8 sided prism, `elong` times
    longer along its own X, tipped `tilt_deg` about the horizontal axis at
    `tilt_axis_deg`. A rectangle reads as a brick; a wobbled polygon reads as
    concrete that cracked."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    rng = ctx["rng"]
    if not _c_ok(ctx, cx, cy):
        return None
    N = rng.randrange(5, 9)
    ph = rng.uniform(0.0, 6.2832)
    ring = []
    for i in range(N):
        a = 6.2832 * i / N + rng.uniform(-0.16, 0.16)
        w = 1.0 + 0.26 * math.sin(2 * a + ph) + 0.15 * math.sin(3 * a + ph * 1.7)
        ring.append((r * elong * w * math.cos(a), r * w * math.sin(a)))
    pts = [Gf.Vec3f(x, y, thick / 2.0) for x, y in ring]
    pts += [Gf.Vec3f(x, y, -thick / 2.0) for x, y in ring]
    faces = list(range(N)) + [N + i for i in range(N - 1, -1, -1)]
    counts = [N, N]
    nrm = [Gf.Vec3f(0.0, 0.0, 1.0)] * N + [Gf.Vec3f(0.0, 0.0, -1.0)] * N
    for i in range(N):
        j = (i + 1) % N
        faces += [i, N + i, N + j, j]      # outward winding (see `_box`)
        counts.append(4)
        ex, ey = ring[j][0] - ring[i][0], ring[j][1] - ring[i][1]
        L = math.hypot(ex, ey) or 1.0
        nrm += [Gf.Vec3f(ey / L, -ex / L, 0.0)] * 4
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateNormalsAttr(Vt.Vec3fArray(nrm))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    e = max(r * elong, r) * 1.4
    mesh.CreateExtentAttr([Gf.Vec3f(-e, -e, -thick), Gf.Vec3f(e, e, thick)])
    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    xf.AddRotateZOp().Set(float(yaw_deg))
    _bind(ctx["stage"], path, mat)
    if abs(tilt_deg) > 0.01:
        aa = math.radians(tilt_axis_deg)
        _transform_prims(ctx["stage"], [path],
                         _rot_about((cx, cy, cz), (math.cos(aa), math.sin(aa), 0.0),
                                    float(tilt_deg)))
    ctx["authored"].append(path)
    return path


def _c_soil_patch(ctx, cx, cy, z, r, elong=1.0, yaw_deg=0.0, mat=None,
                  dome=None, tag="spill"):
    """An irregular fan of soil/silt spilt on the pavement, its centre raised
    so it catches the light as a low MOUND. A perfectly flat polygon on
    asphalt reads as a paper cut-out however it is textured (round-2 bench)."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    rng = ctx["rng"]
    if not _c_ok(ctx, cx, cy):
        return None
    N = 20
    ph = rng.uniform(0.0, 6.2832)
    ca, sa = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    pts = [Gf.Vec3f(cx, cy, z + (r * 0.16 if dome is None else dome))]
    for i in range(N):
        a = 6.2832 * i / N
        w = 1.0 + 0.20 * math.sin(2 * a + ph) + 0.12 * math.sin(5 * a + ph * 1.7)
        px, py = r * elong * w * math.cos(a), r * w * math.sin(a)
        pts.append(Gf.Vec3f(cx + ca * px - sa * py, cy + sa * px + ca * py, z))
    faces, counts = [], []
    for i in range(N):
        faces += [0, 1 + i, 1 + (i + 1) % N]
        counts.append(3)
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    _bind(ctx["stage"], path, mat if mat is not None else _c_look(ctx, "soil"))
    ctx["authored"].append(path)
    return path


def _c_clods(ctx, m, samples, n, at=None, spread=0.55, size=(0.16, 0.55),
             z_frac=(0.45, 1.0), stone_p=0.18, tag="clod"):
    """Lumps of turned-up earth straddling a line. The round-1 berm was a
    smooth extrusion and read as a moulding; a crest needs loose material ON
    it, overlapping, at more than one size, to read as earth."""
    rng = ctx["rng"]
    mats = ctx["mats"]
    made = []
    if not samples:
        return made
    for _k in range(int(n)):
        lx, ly, nx, ny, dc, h, reach = samples[rng.randrange(len(samples))]
        d = (dc if at is None else at) + rng.gauss(0.0, spread)
        sz = rng.uniform(*size)
        px, py = lx + nx * d, ly + ny * d
        wx, wy = _to_world(m, px, py)
        if not _c_ok(ctx, wx, wy):
            continue
        mat = (_c_look(ctx, "soil" if rng.random() < 0.7 else "silt")
               if rng.random() > stone_p else
               _c_look(ctx, "pave" if rng.random() < 0.6 else "brick"))
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy,
             m["z0"] + h * rng.uniform(*z_frac) + sz * 0.2,
             sz, sz * rng.uniform(0.6, 1.1), sz * rng.uniform(0.45, 0.9),
             rng.uniform(0, 180), mat)
        made.append(path)
    ctx["authored"] += made
    ctx["static_extra"] += made
    return made


# --- the two sides ---------------------------------------------------------
def _c_heave(ctx, m, side, crest_m, reach_m, wrap_m=None, seg_m=C_SEG_M,
             tag="heave", span_frac=(0.0, 1.0)):
    """LOW SIDE: the wedge of soil and pavement squeezed up against the wall.

    A mesh of three rings (tucked under the wall, the crest, the toe) with a
    crest that both WANDERS in plan and jumps in height, wrapped a little
    round both corners and dying there. Returns a profile dict the pavement,
    kerb and clod passes ride on."""
    from pxr import Gf
    rng = ctx["rng"]
    d0, d1 = _c_side_range(m, side)
    L = d1 - d0
    d0, d1 = d0 + span_frac[0] * L, d0 + span_frac[1] * L
    wrap = wrap_m if wrap_m is not None else max(1.5, 0.20 * (d1 - d0))
    a, b = d0 - wrap, d1 + wrap
    span = b - a
    n = max(8, int(span / max(0.3, seg_m)))
    hn, dn, rn = _c_noise(rng), _c_noise(rng), _c_noise(rng)
    rows, samples = [], []
    for k in range(n + 1):
        d = a + span * k / float(n)
        (lx, ly), (nx, ny) = _c_perim(m, d)
        t = (d - a) / span
        # max(0.0, ...): at k == n, t rounds to 1 + 2e-16 and a NEGATIVE base
        # under a fractional power is a complex number, not an error
        taper = min(1.0, max(0.0, min(t, 1.0 - t)) / max(1e-3, wrap / span)) ** 0.7
        h = crest_m * taper * max(0.10, 0.72 + 0.45 * hn(d))
        reach = max(0.5, reach_m * (0.55 + 0.45 * taper) * (0.85 + 0.30 * rn(d)))
        dc = min(reach * 0.85, 0.30 + 0.40 * reach + 0.20 * reach * dn(d * 1.6))
        row = []
        for px, py, pz in ((lx - nx * 0.30, ly - ny * 0.30, h * 0.55),
                           (lx + nx * dc, ly + ny * dc, h),
                           (lx + nx * reach, ly + ny * reach, -0.02)):
            wx, wy = _to_world(m, px, py)
            row.append(Gf.Vec3f(wx, wy, m["z0"] + pz))
        rows.append(row)
        samples.append((lx, ly, nx, ny, dc, h, reach))
    # NOT a static collider: an open double-sided ribbon is a triangle-mesh
    # collider PhysX has to cook for nothing (the foundation recipes make
    # almost no loose bodies, and what they do make belongs on the road).
    path = _c_ribbon(ctx, tag, rows, _c_look(ctx, "soil"))
    return {"side": side, "mesh": path, "samples": samples, "crest": crest_m,
            "reach": reach_m, "a": a, "b": b, "span": span,
            "step": span / float(n)}


def _c_pave_break(ctx, m, prof, n, tilt=(8.0, 34.0), tag="slab"):
    """Pavement riding the heave: slabs lifted and tipped, the inner flank
    tipped up toward the wall and the outer flank down toward the road."""
    rng = ctx["rng"]
    mats = ctx["mats"]
    S = prof["samples"]
    made = []
    if not S:
        return made
    _u, base = _c_along(m, prof["side"])
    for _k in range(int(n)):
        lx, ly, nx, ny, dc, h, reach = S[rng.randrange(len(S))]
        # DRAW THE SIZE FIRST: a 1.2 m slab centred 0.4 m off the wall line
        # reaches through the facade, and on the low side the wall is right
        # there. Keep the centre at least most of a half-width out.
        r = rng.uniform(0.55, 1.35)
        d = rng.uniform(max(0.35 * dc, r * 0.85),
                        max(0.4 * dc + r * 0.9, 0.95 * reach))
        if d <= dc:
            z = h * (0.55 + 0.45 * (d / max(dc, 1e-3)))
        else:
            z = h * max(0.0, 1.0 - (d - dc) / max(1e-3, reach - dc)) ** 1.3
        px, py = lx + nx * d, ly + ny * d
        wx, wy = _to_world(m, px, py)
        # THICK ENOUGH TO SEE THE EDGE: at 0.10-0.18 m these read from the
        # air as flat cards, which is what "torn paper" looks like.
        thick = rng.uniform(0.15, 0.26)
        # +theta about the along-side axis drops the OUTBOARD edge (the rule
        # in the skill: `a = (-oy, ox)`, `a x o = -z`). Outer flank +, inner -.
        tdeg = rng.uniform(*tilt) * (1.0 if d > dc else -1.0)
        made.append(_c_plate(
            ctx, wx, wy, m["z0"] + z + thick * 0.5, r,
            thick, base + rng.uniform(-28, 28), base + rng.uniform(-15, 15),
            tdeg, _c_look(ctx, "pave" if rng.random() < 0.5 else "asph"),
            elong=rng.uniform(1.0, 1.9), tag=tag))
    made = [q for q in made if q]
    ctx["static_extra"] += made
    return made


def _c_kerb(ctx, m, prof, tag="kerb"):
    """The kerb at the toe of the heave, off its line: segments shoved out of
    true, a few tipped over, some missing under the soil."""
    rng = ctx["rng"]
    S = prof["samples"]
    made = []
    if not S:
        return made
    _u, base = _c_along(m, prof["side"])
    stride = max(1, int(1.25 / max(0.05, prof["step"])))
    for k in range(0, len(S), stride):
        if rng.random() < 0.18:
            continue                       # buried / carried away
        lx, ly, nx, ny, dc, h, reach = S[k]
        d = reach * rng.uniform(0.92, 1.15)
        wx, wy = _to_world(m, lx + nx * d, ly + ny * d)
        if not _c_ok(ctx, wx, wy):
            continue
        z = m["z0"] + 0.09 + h * rng.uniform(0.0, 0.35)
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy, z, rng.uniform(0.9, 1.5), 0.32, 0.18,
             base + rng.uniform(-9, 9), _c_look(ctx, "pave"))
        if rng.random() < 0.30:
            aa = math.radians(base + rng.uniform(-20, 20))
            _transform_prims(ctx["stage"], [path],
                             _rot_about((wx, wy, z), (math.cos(aa), math.sin(aa), 0.0),
                                        rng.uniform(-38, 38)))
        made.append(path)
    ctx["authored"] += made
    ctx["static_extra"] += made
    return made


def _c_gap(ctx, m, side, width, depth, wrap_m=None, seg_m=C_SEG_M, tag="gap",
           span_frac=(0.0, 1.0)):
    """HIGH SIDE: the ground has opened away from the wall — a trench with
    TORN edges (the outer lip wanders by +-45 %), dark inside so it reads as a
    void from the air as well as from the street."""
    from pxr import Gf
    rng = ctx["rng"]
    d0, d1 = _c_side_range(m, side)
    # `span_frac` takes a sub-range of the side: the flanks of a tilt only
    # open where the building CAME UP, and running the trench the whole way
    # would push it under the heave wrapping round the low corner.
    L = d1 - d0
    d0, d1 = d0 + span_frac[0] * L, d0 + span_frac[1] * L
    wrap = wrap_m if wrap_m is not None else max(1.0, 0.12 * (d1 - d0))
    a, b = d0 - wrap, d1 + wrap
    span = b - a
    n = max(8, int(span / max(0.3, seg_m)))
    wn, dn = _c_noise(rng), _c_noise(rng)
    rows, samples = [], []
    for k in range(n + 1):
        d = a + span * k / float(n)
        (lx, ly), (nx, ny) = _c_perim(m, d)
        t = (d - a) / span
        taper = min(1.0, max(0.0, min(t, 1.0 - t)) / max(1e-3, wrap / span)) ** 0.6
        w = max(0.08, width * taper * (0.62 + 0.75 * wn(d)))
        dp = max(0.06, depth * taper * (0.55 + 0.80 * dn(d)))
        row = []
        for px, py, pz in ((lx - nx * 0.06, ly - ny * 0.06, 0.05),
                           (lx + nx * w * 0.32, ly + ny * w * 0.32, -dp),
                           (lx + nx * w * 0.78, ly + ny * w * 0.78, -dp * 0.8),
                           (lx + nx * w, ly + ny * w, 0.06 * (0.4 + 0.6 * abs(wn(d * 2.3))))):
            wx, wy = _to_world(m, px, py)
            row.append(Gf.Vec3f(wx, wy, m["z0"] + pz))
        rows.append(row)
        samples.append((lx, ly, nx, ny, w, dp, w))
    path = _c_ribbon(ctx, tag, rows, ctx["mats"]["crack"])
    return {"side": side, "mesh": path, "samples": samples, "width": width,
            "depth": depth, "a": a, "b": b, "span": span,
            "step": span / float(n)}


def _c_dish(ctx, m, side, sink_m, reach_m, span_frac=(0.0, 1.0), wrap_m=1.0,
            seg_m=1.2, tag="dish"):
    """The ground DRAGGED DOWN with a settling building: a continuous dished
    apron along one side, deepest at the wall. Scattered plates alone left the
    top-down view of a settled building looking like a building that had
    simply been placed 0.5 m low — from 60 m up a benchmark drone sees the
    RING or it sees nothing."""
    from pxr import Gf
    rng = ctx["rng"]
    d0, d1 = _c_side_range(m, side)
    L = d1 - d0
    d0, d1 = d0 + span_frac[0] * L, d0 + span_frac[1] * L
    a, b = d0 - wrap_m, d1 + wrap_m
    span = b - a
    n = max(6, int(span / max(0.4, seg_m)))
    rn, zn = _c_noise(rng), _c_noise(rng)
    rows = []
    for k in range(n + 1):
        d = a + span * k / float(n)
        (lx, ly), (nx, ny) = _c_perim(m, d)
        reach = max(1.0, reach_m * (0.8 + 0.3 * rn(d)))
        deep = abs(sink_m) * (0.42 + 0.22 * zn(d))
        row = []
        for px, py, pz in ((lx + nx * 0.05, ly + ny * 0.05, -deep),
                           (lx + nx * reach * 0.45, ly + ny * reach * 0.45, -deep * 0.42),
                           (lx + nx * reach, ly + ny * reach, -0.01)):
            wx, wy = _to_world(m, px, py)
            row.append(Gf.Vec3f(wx, wy, m["z0"] + pz))
        rows.append(row)
    # SOIL, not pavement: from straight overhead a 0.2 m dish the same value
    # as the road is invisible, and the top-down view is the benchmark's
    # primary view. What a settling building actually pulls up round itself is
    # mud, and the cracked plates ride ON that.
    return _c_ribbon(ctx, tag, rows, _c_look(ctx, "soil"))


def _c_lip_slabs(ctx, m, prof, n, tag="lip"):
    """Pavement at the edge of an opened gap: most slabs dip INTO it, some
    were pulled up and lean away from the wall."""
    rng = ctx["rng"]
    mats = ctx["mats"]
    S = prof["samples"]
    made = []
    if not S:
        return made
    _u, base = _c_along(m, prof["side"])
    for _k in range(int(n)):
        lx, ly, nx, ny, w, dp, _r = S[rng.randrange(len(S))]
        d = w + rng.uniform(0.05, 1.7)
        wx, wy = _to_world(m, lx + nx * d, ly + ny * d)
        thick = rng.uniform(0.15, 0.25)
        into = rng.random() < 0.6
        # into the gap: the INBOARD edge drops (-theta); pulled up and leaning
        # away: the outboard edge drops (+theta).
        tdeg = (-1.0 if into else 1.0) * rng.uniform(9.0, 30.0)
        z = m["z0"] + (-dp * rng.uniform(0.15, 0.5) if into else rng.uniform(0.02, 0.22))
        made.append(_c_plate(
            ctx, wx, wy, z, rng.uniform(0.5, 1.2), thick,
            base + rng.uniform(-30, 30), base + rng.uniform(-15, 15), tdeg,
            _c_look(ctx, "pave" if rng.random() < 0.5 else "asph"),
            elong=rng.uniform(1.0, 1.8), tag=tag))
    made = [q for q in made if q]
    ctx["static_extra"] += made
    return made


def _c_fissures(ctx, m, corners=None, n_each=(1, 3), length=C_FISSURE_M,
                width=(0.06, 0.22), tag="fissure"):
    """Tension cracks out of the corners: the stress concentration of a
    footing that rotated. Chains of thin dark boxes on a wandering heading,
    2-6 m (research s3.13: "tension cracks ... offset kerbs")."""
    rng = ctx["rng"]
    made = []
    W, D = m["W"], m["D"]
    pts = {"SW": (-W / 2.0, -D / 2.0), "SE": (W / 2.0, -D / 2.0),
           "NE": (W / 2.0, D / 2.0), "NW": (-W / 2.0, D / 2.0)}
    for cname in (corners or list(pts)):
        lx, ly = pts[cname]
        for _q in range(rng.randrange(n_each[0], n_each[1] + 1)):
            L = rng.uniform(*length)
            heading = math.atan2(ly, lx) + rng.uniform(-0.7, 0.7) + math.radians(m["yaw"])
            wx, wy = _to_world(m, lx * 1.02, ly * 1.02)
            w = rng.uniform(*width)
            step = 1.2
            for _i in range(max(2, int(L / step))):
                heading += math.radians(rng.uniform(-16, 16))
                nx2, ny2 = wx + math.cos(heading) * step, wy + math.sin(heading) * step
                mx, my = (wx + nx2) / 2.0, (wy + ny2) / 2.0
                if not _c_ok(ctx, mx, my):
                    break
                path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
                _box(ctx["stage"], path, mx, my, m["z0"] + 0.02, step * 1.12,
                     w * rng.uniform(0.7, 1.3), 0.06, math.degrees(heading),
                     ctx["mats"]["crack"])
                made.append(path)
                wx, wy = nx2, ny2
                w *= rng.uniform(0.72, 0.98)
    ctx["authored"] += made
    return made


def _c_mudline(ctx, m, sink_m, tag="mudline"):
    """The dirt line a sunk building carries on its facade: a dark band on
    the ground-storey wall pieces from the new grade up. Only meaningful for
    a PURE settlement — the piece frames come from the placement record and a
    tilt has moved the walls away from them."""
    rng = ctx["rng"]
    els = (ctx.get("info") or {}).get("elements")
    if not els:
        return []
    h = 0.22 + 0.45 * min(1.0, abs(sink_m) / 1.2)
    made = []
    for e in _els(ctx, role=("wall", "corner"), storey=0):
        fr = _piece_frame(e)
        if not fr:
            continue
        width = fr[3]
        made.append(_on_face(fr, width / 2.0,
                             m["z0"] + h * rng.uniform(0.45, 0.62),
                             width * 1.01, h * rng.uniform(0.85, 1.2), ctx,
                             _c_look(ctx, "soil"), kind=tag, thick=0.05))
    return made


def _c_subsidence(ctx, m, sink_m, plates_per_m=0.55, tag="dish"):
    """PURE SETTLEMENT: the ring. A shallow gap at the wall all round, the
    pavement dished into it and cracked into plates, silt at the wall base.
    Christchurch 2011: 40-50 cm of settlement and almost no tilt, and this
    ring is the whole read."""
    rng = ctx["rng"]
    s = min(1.0, abs(sink_m) / 1.2)
    made = 0
    for side in ("S", "E", "N", "W"):
        reach = 1.6 + 2.6 * s
        _c_dish(ctx, m, side, abs(sink_m), reach, tag=tag + "_bowl")
        gp = _c_gap(ctx, m, side, 0.25 + 0.55 * s, 0.14 + 0.60 * s,
                    wrap_m=0.6, tag=tag + "_gap")
        L = m["W"] if side in ("S", "N") else m["D"]
        _u, base = _c_along(m, side)
        S = gp["samples"]
        for _k in range(int(L * plates_per_m)):
            lx, ly, nx, ny, w, dp, _r = S[rng.randrange(len(S))]
            rr = rng.uniform(0.6, 1.4)
            d = w + rng.uniform(0.1, reach) + rr * 0.75
            wx, wy = _to_world(m, lx + nx * d, ly + ny * d)
            thick = rng.uniform(0.14, 0.24)
            # dished TOWARD the wall: the inboard edge is the low one, so the
            # outboard edge rides up (-theta).
            z = m["z0"] - 0.42 * sink_m * max(0.0, 1.0 - d / max(0.8, reach))
            _c_plate(ctx, wx, wy, z, rr, thick,
                     base + rng.uniform(-30, 30), base + rng.uniform(-15, 15),
                     -rng.uniform(3.0, 13.0),
                     _c_look(ctx, "pave" if rng.random() < 0.5 else "asph"),
                     elong=rng.uniform(1.0, 1.8), tag=tag)
            made += 1
        _c_clods(ctx, m, [(a1, b1, c1, d1, w1 + 0.35, 0.10 + 0.25 * s, w1)
                          for (a1, b1, c1, d1, w1, _dp, _r) in S],
                 max(6, int(L * 0.5)), spread=0.5, size=(0.12, 0.36),
                 z_frac=(0.2, 0.9), tag=tag + "_clod")
        # the silt apron: what came UP as the building went down. Christchurch
        # 2011 photographs are of buildings standing in a field of grey ejecta,
        # not of a clean pavement with a step in it.
        for _k in range(3 + rng.randrange(3)):
            lx, ly, nx, ny, w, dp, _r = S[rng.randrange(len(S))]
            dd = w + rng.uniform(0.4, reach * 1.5)
            wx, wy = _to_world(m, lx + nx * dd, ly + ny * dd)
            _c_soil_patch(ctx, wx, wy, m["z0"] + 0.02, rng.uniform(1.1, 2.6),
                          elong=rng.uniform(1.4, 2.6),
                          yaw_deg=base + rng.uniform(-25, 25),
                          mat=_c_look(ctx, "silt" if rng.random() < 0.5 else "soil"),
                          tag=tag + "_silt")
    return made


def _c_overturn_ground(ctx, m, M, angle_deg=90.0, tag="ovg"):
    """THE GROUND HALF OF AN OVERTURN — the footprint the building was levered
    out of. The raft went over with the shell, so what stays is a crater with
    the torn-off footing stubs round its rim, heaved soil on the hinge edge,
    a lip of ripped pavement on the side the mass swung over, ejecta and
    fissures. Round 1 left clean pavement here, which is the single most
    unreal thing a toppled building can stand next to.

    `M` is the transform the shell was given; the hinge and fall directions
    are MEASURED from it, so this stays correct if the fall logic's sign
    changes."""
    from pxr import Gf
    rng = ctx["rng"]
    fall = _c_fall_side(m, M)
    hinge = _opposite(fall)
    W, D = m["W"], m["D"]
    depth = 0.45 + 0.55 * min(1.0, abs(angle_deg) / 90.0)
    # 1) the crater: a dished mesh over the footprint, deepest at the hinge
    nx_, ny_ = max(4, int(W / 3.0)), max(4, int(D / 3.0))
    hn = _c_noise(rng, freqs=(0.5, 1.6), amps=(0.6, 0.3))
    off = {"S": (0.0, -1.0), "N": (0.0, 1.0), "W": (-1.0, 0.0), "E": (1.0, 0.0)}[hinge]
    rows = []
    for i in range(nx_ + 1):
        u = -0.5 + i / float(nx_)
        row = []
        for j in range(ny_ + 1):
            v = -0.5 + j / float(ny_)
            lx, ly = u * (W + 1.4), v * (D + 1.4)
            r = max(abs(u), abs(v)) * 2.0
            bowl = max(0.0, 1.0 - r ** 2.2)
            # deepest where the footing was levered against: the hinge edge
            bias = 0.55 + 0.45 * (off[0] * u + off[1] * v) * 2.0
            z = -depth * bowl * max(0.0, bias) * (0.75 + 0.35 * hn(lx + ly))
            wx, wy = _to_world(m, lx, ly)
            row.append(Gf.Vec3f(wx, wy, m["z0"] + min(0.0, z)))
        rows.append(row)
    crater = _c_ribbon(ctx, tag, rows, _c_look(ctx, "soil"))
    # 2) the torn footing: stubs of the strip footing left in the ground
    peri = 2.0 * (W + D)
    for _k in range(int(peri / 2.6)):
        d = rng.uniform(0.0, peri)
        (lx, ly), (nxn, nyn) = _c_perim(m, d)
        if rng.random() < 0.3:
            continue
        wx, wy = _to_world(m, lx + nxn * rng.uniform(-0.3, 0.4),
                           ly + nyn * rng.uniform(-0.3, 0.4))
        sz = rng.uniform(0.45, 1.15)
        path = "{0}/{1}_stub_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        z = m["z0"] - depth * rng.uniform(0.1, 0.6) + sz * 0.3
        _box(ctx["stage"], path, wx, wy, z, sz, sz * rng.uniform(0.4, 0.8),
             sz * rng.uniform(0.5, 1.2), rng.uniform(0, 180),
             _c_look(ctx, "raft"))
        if rng.random() < 0.45:
            aa = rng.uniform(0.0, 6.2832)
            _transform_prims(ctx["stage"], [path],
                             _rot_about((wx, wy, z), (math.cos(aa), math.sin(aa), 0.0),
                                        rng.uniform(-40, 40)))
        ctx["authored"].append(path)
        ctx["static_extra"].append(path)
        if rng.random() < 0.5:
            _rebar_tuft(ctx, path, z + sz * 0.3, n=rng.randrange(2, 4))
    # 3) the rim: heaved earth on the hinge side, ripped pavement on the fall
    #    side (the footing dragged up through it), clods everywhere
    hp = _c_heave(ctx, m, hinge, 0.35 + 0.45 * depth, 2.4, tag=tag + "_rim")
    _c_clods(ctx, m, hp["samples"], max(10, int(hp["span"] * 1.4)), tag=tag + "_clod")
    _c_pave_break(ctx, m, hp, max(4, int(hp["span"] * 0.28)), tag=tag + "_slab")
    fp = _c_gap(ctx, m, fall, 0.55, 0.5, tag=tag + "_tear")
    _c_lip_slabs(ctx, m, fp, max(5, int(fp["span"] * 0.35)), tag=tag + "_lip")
    # THE FLANKS ARE WHERE THE MONEY IS. A block that goes over past ~60 deg
    # lies across its own footprint (D 17 m against H 33 m on apartment_tall),
    # so the crater and the fall-side tear are under the shell and only the
    # RIM shows: the hinge edge, and the two long sides either side of the
    # fallen slab. Both flanks get the torn lip.
    for fl in (_c_flank(fall, +1), _c_flank(fall, -1)):
        fh = _c_heave(ctx, m, fl, 0.28 + 0.35 * depth, 2.0, wrap_m=1.2,
                      tag=tag + "_frim")
        _c_clods(ctx, m, fh["samples"], max(10, int(fh["span"] * 1.7)),
                 size=(0.14, 0.55), tag=tag + "_fclod2")
        _c_pave_break(ctx, m, fh, max(3, int(fh["span"] * 0.3)),
                      tag=tag + "_fslab2")
    _c_clods(ctx, m, [(a1, b1, c1, d1, w1 + 0.7, 0.14, w1) for
                      (a1, b1, c1, d1, w1, _dp, _r) in fp["samples"]],
             max(8, int(fp["span"] * 0.9)), spread=0.8, size=(0.14, 0.45),
             z_frac=(0.1, 0.9), tag=tag + "_fclod")
    _c_fissures(ctx, m, n_each=(1, 3), tag=tag + "_fis")
    _ejecta(ctx, m, 2 + rng.randrange(3), bias_side=hinge)
    for _k in range(3):
        d = rng.uniform(0.0, peri)
        (lx, ly), (nxn, nyn) = _c_perim(m, d)
        wx, wy = _to_world(m, lx + nxn * rng.uniform(1.5, 4.0),
                           ly + nyn * rng.uniform(1.5, 4.0))
        _c_soil_patch(ctx, wx, wy, m["z0"] + 0.02, rng.uniform(1.0, 2.4),
                      elong=rng.uniform(1.2, 2.2), yaw_deg=rng.uniform(0, 180),
                      tag=tag + "_spill")
    return {"crater": crater, "fall": fall, "hinge": hinge}


def _c_ground_response(ctx_or_stage, m, low_side=None, drop_m=0.0, rise_m=0.0,
                       sink_m=0.0, M=None, raft=False, parent=None, mats=None,
                       rng=None, tag="ground", crest_scale=1.0, reach_scale=1.0,
                       boils=None, fissures=True, mudline=True, kerb=True,
                       spill=True, bounds=None):
    """THE ground response of a building that leaned or sank — the one entry
    point, used by the recipes (`tilt_sink`, `tilt_severe`, `settlement`) and
    by the city assembly's mild lean (`quake._tilt_prim`).

    Give it EITHER `low_side` + `drop_m` + `rise_m` (from `_c_tilt_matrix`),
    or `M` (the transform applied to the building — the sides are then
    measured from it), or `sink_m` alone with no `low_side` for a pure
    settlement. With no `ctx` it needs `parent`, `mats` and `rng`.

    Knobs: `crest_scale` / `reach_scale` on the low-side wedge, `boils`
    (None = automatic above C_BOIL_DROP), `fissures`, `mudline`, `kerb`,
    `spill`, plus the C_* module constants.
    """
    ctx = ctx_or_stage if isinstance(ctx_or_stage, dict) else _c_ctx(
        ctx_or_stage, parent, mats, rng, tag)
    if bounds is not None:
        ctx["bounds"] = tuple(float(q) for q in bounds)
    rngl = ctx["rng"]
    if M is not None and low_side is None and sink_m <= 0.0:
        low_side, drop_m, rise_m = _c_read_M(m, M)
    drop, rise = max(0.0, float(drop_m)), max(0.0, float(rise_m))
    if raft and M is not None:
        _transform_prims(ctx["stage"], [_raft(ctx, m)], M)
    out = {"low": low_side, "drop": drop, "rise": rise}

    if low_side is None:
        # ---- pure settlement: the ring
        _c_subsidence(ctx, m, abs(sink_m), tag=tag + "_dish")
        if mudline:
            _c_mudline(ctx, m, abs(sink_m), tag=tag + "_mud")
        if fissures:
            _c_fissures(ctx, m, n_each=(1, 2), length=(1.6, 4.5),
                        tag=tag + "_fis")
        if boils or (boils is None and abs(sink_m) > C_BOIL_DROP):
            _ejecta(ctx, m, 2 + rngl.randrange(3))
        return out

    # ---- the low side: heave
    t = min(1.0, drop / C_DROP_REF)
    crest = _c_lerp(C_CREST_M, t) * float(crest_scale)
    reach = _c_lerp(C_REACH_M, t) * float(reach_scale)
    hp = _c_heave(ctx, m, low_side, crest, reach, tag=tag + "_heave")
    _c_clods(ctx, m, hp["samples"], max(20, int(hp["span"] * 2.8)),
             size=(0.14, 0.22 + 0.55 * crest), tag=tag + "_clod")
    # A SECOND RING AT THE TOE. The wedge mesh ends in a crisp polygonal
    # silhouette against the asphalt and reads as a sticker; lumps spilt past
    # the toe are what makes the edge of a slumped mass.
    _c_clods(ctx, m, [(a1, b1, c1, d1, r1 * 1.06, 0.10, r1) for
                      (a1, b1, c1, d1, _dc, _h, r1) in hp["samples"]],
             max(10, int(hp["span"] * 1.2)), spread=0.9, size=(0.14, 0.55),
             z_frac=(0.1, 1.0), tag=tag + "_toe")
    _c_pave_break(ctx, m, hp, max(6, int(hp["span"] * 0.6)), tag=tag + "_slab")
    if kerb and crest > 0.30:
        _c_kerb(ctx, m, hp, tag=tag + "_kerb")
    if spill:
        S = hp["samples"]
        _u, base = _c_along(m, low_side)
        for _k in range(2 + rngl.randrange(3)):
            lx, ly, nx, ny, dc, h, rch = S[rngl.randrange(len(S))]
            d = rch * rngl.uniform(1.0, 1.7)
            wx, wy = _to_world(m, lx + nx * d, ly + ny * d)
            _c_soil_patch(ctx, wx, wy, m["z0"] + 0.02, rngl.uniform(0.8, 1.8),
                          elong=rngl.uniform(1.3, 2.2), yaw_deg=base,
                          dome=rngl.uniform(0.12, 0.30),
                          mat=_c_look(ctx, "silt" if rngl.random() < 0.6 else "soil"),
                          tag=tag + "_spill")

    # ---- the high side: the gap the raft came out of
    high = _opposite(low_side)
    u = min(1.0, (rise + 0.15) / C_RISE_REF)
    # When the raft is properly levered out the "gap" is the HOLE it came out
    # of, not a crack: widen it with the rise (Adapazari's photographs show a
    # metre or two of open ground under a lifted footing).
    gw = max(_c_lerp(C_GAP_W, u), min(2.2, rise * 0.5))
    gd = min(1.4, max(_c_lerp(C_GAP_D, u), rise * 0.7))
    gp = _c_gap(ctx, m, high, gw, gd, tag=tag + "_gap")
    _c_lip_slabs(ctx, m, gp, max(6, int(gp["span"] * 0.5)), tag=tag + "_lip")
    _c_clods(ctx, m, [(a1, b1, c1, d1, w1 + 0.5, 0.12 + 0.2 * u, w1) for
                      (a1, b1, c1, d1, w1, _dp, _r) in gp["samples"]],
             max(10, int(gp["span"] * 1.3)), spread=0.7, size=(0.12, 0.40),
             z_frac=(0.1, 0.9), tag=tag + "_gclod")
    if rise > 0.35:
        # the raft is properly out of the ground: the hole under it shows
        _u, base = _c_along(m, high)
        S = gp["samples"]
        for _k in range(2 + rngl.randrange(3)):
            lx, ly, nx, ny, w, dp, _r = S[rngl.randrange(len(S))]
            wx, wy = _to_world(m, lx + nx * (w * 0.5), ly + ny * (w * 0.5))
            _c_soil_patch(ctx, wx, wy, m["z0"] - gd * 0.7, rngl.uniform(1.0, 2.2),
                          elong=rngl.uniform(1.2, 2.0), yaw_deg=base,
                          tag=tag + "_pit")

    # ---- the flanks, the corners, the water
    # THE FLANKS ARE HALF AND HALF. The wall on a flank runs from buried at
    # the low corner to clear at the high one, so the soil does too: a mound
    # over the low half, the open trench over the high half. Round 2's first
    # bench left the flanks bare and the wall entered the ground along a clean
    # diagonal ruler line. The perimeter runs S -> E -> N -> W, so the +1 flank
    # STARTS at the corner it shares with the low side and the -1 flank ends
    # there.
    for fl, hf, sf in ((_c_flank(low_side, +1), (0.0, 0.58), (0.42, 1.0)),
                       (_c_flank(low_side, -1), (0.42, 1.0), (0.0, 0.58))):
        fh = _c_heave(ctx, m, fl, crest * 0.62, reach * 0.78, wrap_m=1.2,
                      tag=tag + "_fheave", span_frac=hf)
        _c_clods(ctx, m, fh["samples"], max(8, int(fh["span"] * 1.6)),
                 size=(0.13, 0.18 + 0.4 * crest), tag=tag + "_fclod")
        _c_pave_break(ctx, m, fh, max(3, int(fh["span"] * 0.35)),
                      tag=tag + "_fslab")
        fp = _c_gap(ctx, m, fl, 0.10 + 0.2 * u, 0.10 + 0.35 * gd,
                    wrap_m=0.8, tag=tag + "_fgap", span_frac=sf)
        _c_lip_slabs(ctx, m, fp, max(3, int(fp["span"] * 0.28)), tag=tag + "_flip")
    if fissures:
        _c_fissures(ctx, m, n_each=(1, 3), tag=tag + "_fis")
    if boils or (boils is None and drop > C_BOIL_DROP):
        _ejecta(ctx, m, 2 + rngl.randrange(4), bias_side=low_side)
    return out


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
    paths = _everything(ctx) + [raft] + list(ctx.pop("c_carry", []))
    _transform_prims(ctx["stage"], paths, _translate(0.0, 0.0, -sink))
    ctx["static_extra"] += paths
    # The subsidence RING: the pavement dished into the wall all round and
    # cracked into plates, a shallow gap at the wall, silt at the base and a
    # mud line on the facade. Round 1 authored a low berm on each side, which
    # from the street read as a moulding round a building that had simply been
    # placed too low. (Christchurch 2011: 40-50 cm of settlement, tilt 0.4-0.5
    # deg, and this ring is the whole read.)
    _c_ground_response(ctx, m, sink_m=sink, tag="settle")
    ctx["notes"].append("settlement: sunk {0:.2f} m".format(sink))


def r_tilt_severe(ctx, tilt_deg=None, sink_m=None, side=None, max_drop_m=3.2):
    """Severe lean, 10-30 deg, on a raft that levers out of the ground on the
    high side while silt is squeezed out on the low side (Adapazari 1999).

    At these angles the two sides of the ground could not be more different —
    metres of raft in the air on one, the ground floor swallowed on the other
    — so this is the recipe that most needs `_c_ground_response`. The lean is
    TOWARD `side` (round 1 leaned away from it; see `r_tilt_sink`)."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    tilt = tilt_deg if tilt_deg is not None else rng.uniform(10.0, 30.0)
    sink = sink_m if sink_m is not None else rng.uniform(0.5, 2.0)
    side = side or rng.choice(["S", "E", "N", "W"])
    M, g = _c_tilt_matrix(m, side, tilt, sink, max_drop_m=max_drop_m)
    raft = _raft(ctx, m)
    paths = _everything(ctx) + [raft] + list(ctx.pop("c_carry", []))
    _transform_prims(ctx["stage"], paths, M)
    ctx["static_extra"] += paths
    # the low side also gets the WINDROW of silt squeezed from under the raft,
    # on top of the heave — wet mud, not rubble, so it takes the soil material
    _heap(ctx, m, m["z0"], 0.0, 0.12, fill=False, sides=(g["low"],),
          depth_m=0.5 + 0.5 * min(1.0, g["sink"] / 1.5), tag="silt",
          mat_fn=lambda: ctx["mats"]["soil"])
    _c_ground_response(ctx, m, low_side=g["low"], drop_m=g["drop"],
                       rise_m=g["rise"], tag="tiltsev", crest_scale=1.15,
                       reach_scale=1.1)
    ctx["notes"].append(
        "tilt_severe: {0:.1f} deg toward {1}, sunk {2:.2f} m (low edge -{3:.2f} m, "
        "high edge +{4:.2f} m)".format(g["tilt"], side, g["sink"], g["drop"],
                                       g["rise"]))


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
    # THE PIVOT IS A LOCAL OFFSET, so it must come from the LOCAL normal.
    # `_outward` is already rotated into world, and feeding it back through
    # `_to_world` rotates it a second time: at yaw 90 the S pivot came out on
    # the centre line, at yaw 45 on a diagonal. Harmless on the bench and in
    # the bake (both build at yaw 0) and wrong for anything run on a placed
    # city building. `_c_tilt_matrix` does the same thing the same way.
    lnx, lny = _SIDE_NORMAL[side]
    px, py = _to_world(m, lnx * m["W"] / 2.0, lny * m["D"] / 2.0)
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
    paths = _everything(ctx) + [raft] + carried + list(ctx.pop("c_carry", []))
    M = _rot_about((px, py, m["z0"]), (ax, ay, 0.0), -abs(angle))
    _transform_prims(ctx["stage"], paths, M)
    _transform_prims(ctx["stage"], [q for q in ctx["loose"]], M)
    ctx["static_extra"] += paths
    # the loose bits get an outward-and-down shove so they clear the shell
    for q in ctx["loose"]:
        ctx["velocity"][q] = (ox * rng.uniform(0.5, 2.0), oy * rng.uniform(0.5, 2.0), -1.0)
    # 3) the ground: the footprint it was levered OUT of — a crater with the
    #    torn footing stubs round its rim, heaved earth on the hinge edge and
    #    ripped pavement on the side the mass swung over. Agent C owns this
    #    half; the fall side is measured from `M`, not assumed.
    og = _c_overturn_ground(ctx, m, M, angle_deg=angle)
    # the landing windrow, at distance ~H on the side the mass ACTUALLY swung
    # over (`side` names the hinge edge, and -theta about it throws the mass
    # the other way — measured, `_c_fall_side`; round 1 put this windrow on
    # the hinge side, under the standing edge). Ejecta and torn pavement are
    # inside `_c_overturn_ground` now.
    land = H * math.sin(math.radians(angle)) if angle < 89.0 else H
    _heap(ctx, m, m["z0"], 0.0, 0.10, fill=False, sides=(og["fall"],),
          depth_m=rng.uniform(0.6, 1.2), tag="landing", offset_m=max(0.0, land - 2.5))
    ctx["notes"].append("overturn: {0:.0f} deg about the {1} edge, onto {2} "
                        "(H {3:.0f} m)".format(angle, side, og["fall"], H))


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


def _droop_strip(ctx, slab_path, m, side, depth_m, angle_deg, tear=True):
    """Hinge the outer `depth_m` of a slab down by `angle_deg` about the line
    where it meets the remainder — the USAR "lean-to" / "V" floor. Returns
    (remainder, strip).

    `tear` breaks the strip's FREE edge first. The drooping slab is the thing
    a camera sees through the hole an out_of_plane peel leaves, and re-authored
    as a `_box` it hangs there with a dead-straight formwork edge — a ramp,
    not a broken floor. The outermost 1.4 m are fractured against a wandering
    (or stepped) line, the surviving cells hinge WITH the strip and the rest
    fall."""
    rng = ctx["rng"]
    rem, strip = _split_strip(ctx, slab_path, m, side, depth_m, ctx["mats"]["concrete"])
    group = [strip]
    if tear:
        from pxr import UsdShade
        km = UsdShade.MaterialBindingAPI(
            ctx["stage"].GetPrimAtPath(strip)).ComputeBoundMaterial()[0]
        d = rng.uniform(0.35, 0.9)
        s_rem, s_edge = _split_strip(ctx, strip, m, side, d + 1.4,
                                     ctx["mats"]["concrete"])
        st, lo = _break_split(
            ctx, s_edge, 10 + rng.randrange(4),
            _edge_judge(m, side, d, rng, btype=ctx["info"]["type"]),
            lambda: (_a_mat(ctx, "concrete_dusty") if rng.random() < 0.55
                     else _a_mat(ctx, "dust")),
            min_volume_frac=0.0008, static_mat=km if km else None)
        group = [s_rem] + st
        ctx["loose"] += lo
        ctx["authored"] += [s_rem]
    W, D = m["W"], m["D"]
    # hinge line: the strip's inner edge, in local coords
    along_x = side in ("E", "W")
    full = (W if along_x else D) - 2 * WALL_INSET
    sgn = 1.0 if side in ("E", "N") else -1.0
    hinge = sgn * (full / 2.0 - min(depth_m, full * 0.6))
    hx, hy = (_to_world(m, hinge, 0.0) if along_x else _to_world(m, 0.0, hinge))
    ox, oy = _outward(m, side)
    ax, ay = -oy, ox
    cx, cy, cz, sx, sy, sz, yaw = _box_dims(ctx["stage"], group[0])
    M = _rot_about((hx, hy, cz + sz / 2.0), (ax, ay, 0.0), abs(angle_deg))
    _transform_prims(ctx["stage"], group, M)
    ctx["static_extra"] += [rem] + group
    ctx["authored"] += [rem]
    _a_edge_bars(ctx, group[1:], ctx["info"]["type"], m, side, n=2)
    return rem, group[0]


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


def _a_roof_slabs(ctx):
    """Authored roof slabs still on the stage — what an earlier recipe left
    where the kit roof tiles used to be. See the note in `_roof_box`."""
    out = []
    for p in ctx.get("roof_slabs", []):
        pr = ctx["stage"].GetPrimAtPath(p)
        if pr and pr.IsValid() and pr.IsActive():
            out.append(p)
    return out


def _a_hole_outline(rng, lobes=(1, 4)):
    """An irregular closed outline: radius(theta) about 1.0, in a frame where
    the hole is a unit circle. Returns (rad, rmax).

    A ROOF HOLE IS NOT A RECTANGLE. The first version tested
    `abs(lx-hcx) < hw + wobble` on each axis independently, which is a
    rounded BOX — and the user saw exactly that ("unnatural rectangular /
    square parts broken off"). A diaphragm that fails drops the bay it
    failed in and tears round the framing, so the outline is a rough polygon
    with one or two runs where it followed a joist further than the rest:
    five harmonics for the general irregularity, plus `lobes` gaussian bumps
    for the runs."""
    TAU = 2.0 * math.pi
    k = [(1, rng.uniform(0.10, 0.22)), (2, rng.uniform(0.08, 0.18)),
         (3, rng.uniform(0.05, 0.14)), (5, rng.uniform(0.03, 0.09)),
         (7, rng.uniform(0.02, 0.06))]
    k = [(n, a, rng.uniform(0.0, TAU)) for n, a in k]
    lob = [(rng.uniform(0.0, TAU), rng.uniform(0.22, 0.55),
            rng.uniform(0.22, 0.55)) for _ in range(rng.randrange(*lobes))]

    def rad(th):
        r = 1.0 + sum(a * math.sin(n * th + p) for n, a, p in k)
        for th0, w, amp in lob:
            # TAU, NOT 6.283. The harmonics are exactly 2*pi-periodic; the
            # lobe's angular wrap was not, so the outline did not close and
            # the seam showed as a notch at theta = +-pi (which `atan2` puts
            # on the -x axis of every hole in the library).
            dth = (th - th0 + math.pi) % TAU - math.pi
            r += amp * math.exp(-(dth / w) ** 2)
        return r

    # A GENUINE UPPER BOUND: `_r_of`'s margin test scales the tile cut-off by
    # this, so a coarse sample that under-reports the peak clips a lobe.
    rmax = max(rad(i * (TAU / 720.0)) for i in range(721)) * 1.02
    return rad, rmax


def r_roof_hole(ctx, mass="main", frac=None):
    """Roof diaphragm collapse: a 20-50 % patch of the roof drops through
    onto the top floor, which breaks under it — the nadir 'major damage'
    cue (Copernicus): a dark hole in an otherwise regular roof plane.

    Round 2: the outline is an irregular polygon rather than a rounded box,
    the rim SAGS (pieces still attached, hinged 15-40 deg down into the hole),
    a few pieces end up on the floor below rather than on the roof, and the
    cells at the rim open cracks that radiate into the surviving slab."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    m = ctx["info"]["masses"][mass]
    btype = ctx["info"]["type"]
    fit = ctx["fit"]
    W, D = m["W"], m["D"]
    area = frac if frac is not None else rng.uniform(0.2, 0.5)
    hw = W * math.sqrt(area) * rng.uniform(0.7, 1.3) / 2.0
    hd = (W * D * area) / (4.0 * hw)
    hw, hd = min(hw, W * 0.42), min(hd, D * 0.42)
    rad, rmax = _a_hole_outline(rng)
    # THE MARGIN IS A SOFT CONSTRAINT. Pushing the centre in by a fixed 1 m
    # kept every hole an even rectangle's-worth from the parapet; the outline
    # wanders by `rmax` anyway, so the centre is drawn against the MEAN radius
    # and a lobe is allowed to run out to the edge of the roof.
    hcx = rng.uniform(-W / 2.0 + hw * 0.75, W / 2.0 - hw * 0.75)
    hcy = rng.uniform(-D / 2.0 + hd * 0.75, D / 2.0 - hd * 0.75)

    def _r_of(lx, ly):
        u, v = (lx - hcx) / max(hw, 1e-3), (ly - hcy) / max(hd, 1e-3)
        rr = math.hypot(u, v)
        return rr, (rr / rad(math.atan2(v, u)) if rr > 1e-6 else 0.0)

    def judge(c):
        lx, ly = _to_local(m, c[0], c[1])
        return _r_of(lx, ly)[1] < 1.0

    # TARGETS: the kit roof tiles this mass still has, plus any slab an
    # EARLIER recipe already authored in their place (see `_roof_box`).
    from pxr import UsdShade
    targets = []
    for e in list(_els(ctx, mass=mass, role="roof")):
        targets.append((e, e["lx"], e["ly"]))
    if not targets:
        for p in _a_roof_slabs(ctx):
            try:
                cx_, cy_ = _box_dims(ctx["stage"], p)[:2]
            except Exception:
                continue
            lx_, ly_ = _to_local(m, cx_, cy_)
            targets.append((p, lx_, ly_))

    n_hit, rim_st, own_loose = 0, [], []
    for tgt, tlx, tly in targets:
        # only pieces that actually reach INTO the hole are touched; a 6 m
        # margin fractured most of the roof and the crack mosaic came back
        if (abs(tlx - hcx) > hw * rmax + 2.6
                or abs(tly - hcy) > hd * rmax + 2.6):
            continue
        if isinstance(tgt, dict):
            box = _roof_box(ctx, tgt, thick=(0.14 if btype == "urm" else ROOF_T))
        else:
            box = tgt
        if not box:
            continue
        bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(box)).ComputeBoundMaterial()[0]
        # MORE, SMALLER, DARKER PIECES. At 14-20 `plank` cells over a 22 x 18 m
        # roof box the cells are ~4 m and the aspect stretches the worst of
        # them to 8 m: the first pass put a pale board the length of the
        # building across the hole. 26-34 cells at aspect 1.4-2.8, a 3.2 m
        # ceiling on anything loose, and the dusty tints for what fell (a
        # board that has been through a roof collapse is grey, not new pine).
        st, lo = _break_split(ctx, box, (26 + rng.randrange(8) if btype == "urm"
                                         else 16 + rng.randrange(6)), judge,
                              lambda: (_a_mat(ctx, "timber_dusty") if
                                       (btype == "urm" and rng.random() < 0.55)
                                       else _a_mat(ctx, "dust")
                                       if rng.random() < 0.6
                                       else _a_mat(ctx, "concrete_dusty")),
                              min_volume_frac=0.0005,
                              mode=("plank" if btype == "urm" else "uniform"),
                              aspect=((1.4, 2.8) if btype == "urm" else None),
                              static_mat=bm if bm else None,
                              crack_frac=0.30, max_loose_m=3.2)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        ctx["authored"] += st
        rim_st += st
        own_loose += lo
        n_hit += len(lo)
        if isinstance(tgt, dict):
            tgt["dead"] = True
    # THE RIM SAGS. A diaphragm that lets go does not shear off flush: the
    # boards and the slab strips at the edge stay attached and hinge down into
    # the hole (every USAR photograph of a collapsed roof shows the rim
    # drooping before it shows the hole). Static, hinged about their outer
    # edge, 15-40 deg.
    z_top = m["top"]
    n_sag = 0
    for pth in rim_st:
        c = _pivot_of(ctx, pth)
        lx, ly = _to_local(m, c[0], c[1])
        rr, rn = _r_of(lx, ly)
        if rn > 1.35 or rng.random() > 0.45:
            continue
        # inward direction, in world: toward the hole centre
        wx, wy = _to_world(m, hcx, hcy)
        dx, dy = wx - c[0], wy - c[1]
        L = math.hypot(dx, dy) or 1.0
        dx, dy = dx / L, dy / L
        # hinge on the OUTER edge of the piece, axis across the inward run
        # +theta about the LEFT perpendicular of the INWARD run tips the
        # inner end DOWN (a x inward = -z; see the skill's droop note, which
        # is the same construction about the outward vector).
        px, py = c[0] - dx * 0.45, c[1] - dy * 0.45
        M = _rot_about((px, py, c[2]), (-dy, dx, 0.0),
                       rng.uniform(15.0, 40.0))
        _transform_prims(ctx["stage"], [pth], M)
        n_sag += 1
    # the top floor under the hole gives way too (it took the roof's weight)
    top = len(m["levels"]) - 1
    pth = fit["slabs"].get((mass, top))
    if pth and rng.random() < 0.7:
        rad2, _rmax2 = _a_hole_outline(rng)

        def judge2(c):
            lx, ly = _to_local(m, c[0], c[1])
            u, v = (lx - hcx) / max(hw * 0.85, 1e-3), (ly - hcy) / max(hd * 0.85, 1e-3)
            rr = math.hypot(u, v)
            return rr < rad2(math.atan2(v, u)) if rr > 1e-6 else True
        from pxr import UsdShade
        km = UsdShade.MaterialBindingAPI(
            ctx["stage"].GetPrimAtPath(pth)).ComputeBoundMaterial()[0]
        st, lo = _break_split(ctx, pth, 14 + rng.randrange(6), judge2,
                              lambda: (_a_mat(ctx, "timber_dusty") if btype == "urm"
                                       else _a_mat(ctx, "dust")),
                              min_volume_frac=0.0006,
                              static_mat=km if km else None, crack_frac=0.28,
                              max_loose_m=3.2)
        fit["slabs"][(mass, top)] = None
        fit["all"] = [q for q in fit["all"] if q != pth] + st
        ctx["loose"] += lo
        ctx["static_extra"] += st
        _a_edge_bars(ctx, st, btype, m, "S", n=3)
    # A FEW PIECES END UP INSIDE. What falls through a roof lands on the floor
    # below, not on the roof beside the hole — but a loose fragment authored at
    # roof level with the slab under it already broken can just as easily catch
    # on a surviving cell. Dropping a third of them to floor level before the
    # settle guarantees the hole reads as a hole with rubble at the bottom.
    z_floor = m["levels"][top] if top < len(m["levels"]) else m["z0"]
    n_in = 0
    for q in own_loose:
        if rng.random() > 0.35:
            continue
        c = _pivot_of(ctx, q)
        if c[2] < z_top - 1.0 or c[2] - z_floor < 1.0:
            continue
        _transform_prims(ctx["stage"], [q],
                         _translate(rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6),
                                    -(c[2] - z_floor - 0.5)))
        n_in += 1
    _disturb_interior(ctx, mass, {top})
    ctx["notes"].append(
        "roof_hole: {0:.0f}% of the roof, {1} pieces down, {2} rim pieces "
        "sagging, {3} dropped inside".format(area * 100, n_hit, n_sag, n_in))


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


def _b_face_pt(fr, u, v, out=0.0):
    """World point at `u` along a wall piece, world height `v`, pushed `out`
    metres along the piece's OUTWARD normal (local -Y, world (sin, -cos))."""
    ox, oy, yaw, width, height, depth, dw = fr
    ca, sa = math.cos(yaw), math.sin(yaw)
    if dw:
        u = u - width / 2.0
    d = -depth
    return (ox + ca * u + sa * (d + out), oy + sa * u - ca * (d + out), v)


def _b_face_mesh(ctx, pts, mat, kind, double=True):
    """One flat mesh in a wall plane from a list of (x, y, z) triangles/quads
    already fanned by the caller. `pts` is [(points, counts, indices)]."""
    from pxr import Gf, Sdf, UsdGeom, Vt
    P, counts, idx = pts
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], kind, ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in P]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    if double:
        mesh.CreateDoubleSidedAttr(True)
    _bind(ctx["stage"], path, mat)
    ctx["authored"].append(path)
    return path


def _b_blob_outline(rng, ra, rv, n=None):
    """An irregular closed outline in (du, dv) — three harmonics, so it has
    lobes and notches rather than the smooth ellipse two gave."""
    N = n or rng.randrange(11, 17)
    p1, p2, p3 = (rng.uniform(0, 6.28), rng.uniform(0, 6.28), rng.uniform(0, 6.28))
    out = []
    for i in range(N):
        a = 6.283 * i / N
        w = (1.0 + 0.26 * math.sin(2 * a + p1) + 0.17 * math.sin(3 * a + p2)
             + 0.10 * math.sin(5 * a + p3))
        out.append((ra * w * math.cos(a), rv * w * math.sin(a)))
    return out


# Finish-loss patch geometry. A patch is TWO co-planar rings: a dark FILL a
# few mm off the wall and, around it, a RIM standing `PATCH_RIM_PROUD` proud
# in a dusty render tone. The rim is what makes the fill read as a RECESS —
# the render is 1-3 cm thick and its broken edge stands over the hole — and
# it is why the first version, one bright polygon 2 cm PROUD of the wall,
# read as a paper sticker stuck on the brick (user, round 1).
PATCH_FILL_PROUD = 0.004       # the fill itself, just off the wall plane
PATCH_RING_PROUD = 0.007       # the shadow line, in front of the fill
PATCH_RING_W = 0.05            # width of the shadow line, m


def _face_patch(fr, u, v, ra, rv, ctx, mat, kind="scar", proud=PATCH_FILL_PROUD,
                rim_mat=None):
    """A plaster / render loss patch: an irregular DARK area with a DARKER
    line round it — the shadow the broken edge of the surviving finish casts
    into the recess. Returns [paths].

    THE RING MUST BE DARKER THAN THE WALL, NEVER LIGHTER. The first attempt
    made it the surviving render (a 0.47 dusty grey) standing 22 mm proud,
    to sell the recess by geometry. On the bench that is a WHITE OUTLINE
    round a grey blob, which is precisely the sticker the user objected to
    (B1_com/1_commercial_DG2_sw.png). A shadow line does the same job and
    cannot go bright."""
    rng = ctx["rng"]
    line = _b_blob_outline(rng, ra, rv)
    N = len(line)
    made = []

    # 1) the fill — a triangle fan, so a concave lobe still tessellates
    P = [_b_face_pt(fr, u, v, proud)]
    for du, dv in line:
        P.append(_b_face_pt(fr, u + du, v + dv, proud))
    counts, idx = [], []
    for i in range(N):
        counts.append(3)
        idx += [0, 1 + i, 1 + (i + 1) % N]
    made.append(_b_face_mesh(ctx, (P, counts, idx), mat, kind))

    # 2) the shadow — a narrow band along the UPPER arc only, where the
    #    surviving finish overhangs the recess. A full closed loop is an INK
    #    OUTLINE: it turns the patch into a cartoon sticker at any distance
    #    (B2_apt/0_apartment_tall_DG2_nw.png). The outline is generated in
    #    angle order, so the first ~55 % of it is the top half.
    if rim_mat is not None:
        arc = max(3, int(N * 0.55))
        P2, counts2, idx2 = [], [], []
        for i in range(arc):
            du, dv = line[i]
            rr = max(0.12, math.hypot(du, dv))
            k = 1.0 + PATCH_RING_W * rng.uniform(0.7, 1.3) / rr
            k = max(1.02, min(1.16, k))
            P2.append(_b_face_pt(fr, u + du, v + dv, PATCH_RING_PROUD))
            P2.append(_b_face_pt(fr, u + du * k, v + dv * k, PATCH_RING_PROUD))
        for i in range(arc - 1):
            a, b = 2 * i, 2 * (i + 1)
            counts2.append(4)
            idx2 += [a, b, b + 1, a + 1]
        made.append(_b_face_mesh(ctx, (P2, counts2, idx2), rim_mat, kind + "ring"))
    return made


def _b_crack(ctx, fr, u0, v0, u1, v1, mat, width=0.016, proud=0.008,
             n_seg=None, jag=0.10):
    """A JAGGED shear crack between two points on a wall face, as one
    quad-strip mesh.

    Masonry cracks step from mortar bed to perpend and back, so the line is
    a zigzag with a diagonal mean — not the two straight 2 m bars 45 mm wide
    the first version drew, which read as lumber nailed to the wall (user,
    round 1). Real cracks run corner to corner between openings, which is
    what the caller aims them at."""
    rng = ctx["rng"]
    n = n_seg or rng.randrange(5, 10)
    du, dv = u1 - u0, v1 - v0
    L = math.hypot(du, dv) or 1.0
    pu, pv = -dv / L, du / L                  # in-plane perpendicular
    line, off = [], 0.0
    for i in range(n + 1):
        t = i / float(n)
        # a bounded random walk across the mean line: alternating sign so the
        # steps read as courses, not as a smooth wander
        off = (0.0 if i in (0, n)
               else max(-jag, min(jag, -0.55 * off + rng.uniform(-jag, jag))))
        line.append((u0 + du * t + pu * off, v0 + dv * t + pv * off))
    P, counts, idx = [], [], []
    for i, (uu, vv) in enumerate(line):
        j = min(len(line) - 1, i + 1)
        k = max(0, i - 1)
        tu, tv = line[j][0] - line[k][0], line[j][1] - line[k][1]
        tl = math.hypot(tu, tv) or 1.0
        qu, qv = -tv / tl, tu / tl
        w = width * (0.55 + 0.9 * math.sin(math.pi * i / float(len(line) - 1 or 1)))
        P.append(_b_face_pt(fr, uu + qu * w / 2.0, vv + qv * w / 2.0, proud))
        P.append(_b_face_pt(fr, uu - qu * w / 2.0, vv - qv * w / 2.0, proud))
    for i in range(len(line) - 1):
        a = 2 * i
        counts.append(4)
        idx += [a, a + 2, a + 3, a + 1]
    return _b_face_mesh(ctx, (P, counts, idx), mat, "crack")


def _b_crumbs(ctx, fr, m, u, n=None, spread=1.1, reach=(0.25, 1.5)):
    """The small pile of render crumbs on the pavement under a loss patch.

    Plaster that comes off a wall lands at its foot: a low, dusty, grey-brown
    scatter 0.2-1.5 m out. Authored, not simulated — it is 10 boxes."""
    rng = ctx["rng"]
    made = []
    keys = ("plaster_dusty", "mortar", "plaster_dusty", "brick")
    for k in range(n or rng.randrange(5, 12)):
        du = rng.uniform(-spread, spread)
        out = rng.uniform(*reach)
        x, y, _z = _b_face_pt(fr, u + du, 0.0, out)
        s = rng.uniform(0.06, 0.22)
        path = "{0}/crumb_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, x, y, m["z0"] + s * 0.35,
             s, s * rng.uniform(0.5, 1.0), s * rng.uniform(0.4, 0.8),
             rng.uniform(0, 180), ctx["mats"][keys[rng.randrange(len(keys))]])
        made.append(path)
    ctx["authored"] += made
    return made


def _b_is_glass(e):
    """A curtain-wall / shopfront glazing piece — never scar these."""
    if e["p"].get("glass"):
        return True
    nm = e.get("name") or ""
    return ("Skyscraper" in nm) or ("Glass" in nm) or ("Window_Wall" in nm)


SCARS_MAX_PROJ = 0.30      # m a scarrable module may stand proud of the wall
SCARS_MAX_PATCHES = 16     # per building. A 33 m apartment_tall has ~330 wall
SCARS_MAX_CRACKS = 11      # pieces, so a bare `frac` of 0.18 put 59 patches
#                            and 23 cracks on it — a rash, not damage. These
#                            caps keep the count of MARKS roughly constant
#                            with building size, which is what reads.


def r_facade_scars(ctx, frac=0.22, mass=None, patches=(1, 2), crack_p=0.45,
                   cracks=(1, 2), crumb_p=0.7,
                   max_patches=SCARS_MAX_PATCHES, max_cracks=SCARS_MAX_CRACKS):
    """DG1-4 on STANDING walls: render / face-brick loss patches showing the
    dark bed behind, crumbs of it on the pavement, and jagged diagonal shear
    cracks running corner to corner across the piers.

    Geometry, not decals, so it needs no cutout-opacity flag. Everything
    here is DARK: the round-1 version drew a bright polygon standing 2 cm
    proud and two 45 mm bars, and the user read them, correctly, as white
    paper blobs and wooden Xs nailed to the wall.

    Where it applies: unreinforced masonry (face brick / stone loses its
    outer wythe) and the RENDERED INFILL of a concrete frame (Kahramanmaras:
    red hollow block under a fallen cement render). Never on glazing."""
    rng = ctx["rng"]
    btype = ctx["info"]["type"]
    mats = ctx["mats"]
    # MASONRY ONLY. A curtain-wall tower has no render to lose, and the
    # concrete families' "infill" panel in THIS kit is a 4 m glazed module
    # with a 0.3-0.5 m deep reveal: `_piece_frame` measures its outermost
    # projection, so a patch drawn flush on that face hangs half a metre in
    # front of the glass behind it (B3_off/0_office_DG1_street.png), and
    # there is no pier wide enough to put it on. RC frames also crack at
    # beam-column joints rather than across a panel, which this kit cannot
    # express. Their DG1-3 vocabulary is `infill_fail` / `balcony_fail` /
    # `glass_loss`, which is already in the ladder.
    if btype != "urm":
        ctx["notes"].append("facade_scars: skipped ({0} — no rendered pier "
                            "in this kit)".format(btype))
        return
    n_p = n_c = 0
    # SHUFFLED, so the cap does not eat the whole north side: `_els` yields
    # in placement order, which is side by side.
    walls = list(_els(ctx, mass=mass, role="wall"))
    rng.shuffle(walls)
    for e in walls:
        if n_p >= max_patches and n_c >= max_cracks:
            break
        if _b_is_glass(e) or rng.random() >= frac:
            continue
        # NEVER THE GROUND BAND, on any family. It is the shopfront /
        # lobby: a `FirstFloor_*` module that is mostly glass, and its
        # measured face is the OUTERMOST projection of its arch surround, so
        # a patch drawn flush on that face floats a hand's width in FRONT of
        # the glass behind it (B2_com/0_commercial_DG1_street.png: two black
        # blobs hanging over a shop window). The kit gives no way to find the
        # glazing inside a module, so the rule is positional.
        if e["storey"] == 0:
            continue
        fr = _piece_frame(e)
        if not fr:
            continue
        ox, oy, yaw, width, height, depth, dw = fr
        if width < 1.5 or height < 2.0:
            continue
        # NOT ON A PROJECTING BAND. `depth` is the module's OUTERMOST point,
        # so on a piece with a deep cornice or an oriel (MBuilding04_TopFloor
        # is 0.71 m proud, MBuilding01_Corner_A 0.55) a patch drawn flush on
        # that plane stands off the wall behind it by that much. The plain
        # façade modules are 0.10-0.22 m proud, so 0.30 is the cut.
        if abs(depth) > SCARS_MAX_PROJ:
            continue
        m = ctx["info"]["masses"].get(e["mass"]) or ctx["info"]["masses"]["main"]
        # WHAT IS UNDER THE FINISH. A brick or stone wall that loses its face
        # wythe shows the dark mortar/rubble bed; a rendered frame shows the
        # red hollow-block infill. Both are DARKER than the wall — that is
        # the whole read.
        # TEXTURED, MOSTLY. A flat colour patch is a paper cut-out however
        # dark it is (B2_apt/0_apartment_tall_DG2_nw.png). The megascans
        # brick is world-projected, so it lands on a UV-less patch mesh at
        # metric scale and gives it courses — which is what says "the facing
        # has come off and you are looking at the backing". A quarter stay
        # flat `scar` for the ones that are dark mortar bed rather than block.
        def inner_mat():
            return mats["brick"] if rng.random() < 0.5 else mats["scar"]
        for k in range(rng.randrange(patches[0], patches[1] + 1)):
            if n_p >= max_patches:
                break
            # 0.7-2.1 m across. The round-1 patches were 0.5-1.4 m and on a
            # 22 m wall they read as blemishes; the reconnaissance photos of
            # render loss are whole-pier or half-spandrel areas.
            ra = rng.uniform(0.35, 1.05)
            rv = rng.uniform(0.30, 0.80)
            # PIER OR SPANDREL, not the middle of the module. Every façade
            # module in these families centres its opening, so the solid
            # masonry is the outer ~30 % of the width (the piers) and the
            # top / bottom ~20 % of the storey (the spandrels). Aiming there
            # keeps the patch off glazing without needing to know where the
            # glazing is, and it is also where render actually comes off —
            # the pier is what carries the shear.
            if rng.random() < 0.65:
                band = rng.uniform(0.06, 0.30)
                if rng.random() < 0.5:
                    band = 1.0 - band
                ra = min(ra, width * 0.11)
                rv = min(rv, ra * 1.7)     # else a 0.4 m pier gives a 1.6 m
                #                            tall sliver that reads as a flame
                u = width * band
                v = e["z"] + rng.uniform(rv + 0.4, max(rv + 0.5, height - rv - 0.4))
            else:
                rv = min(rv, height * 0.10)
                u = rng.uniform(ra + 0.25, max(ra + 0.3, width - ra - 0.25))
                v = e["z"] + (rng.uniform(rv + 0.15, height * 0.20)
                              if rng.random() < 0.5
                              else rng.uniform(height * 0.80, height - rv - 0.15))
            u = min(max(u, ra + 0.2), max(ra + 0.25, width - ra - 0.2))
            _face_patch(fr, u, v, ra, rv, ctx, inner_mat(),
                        rim_mat=mats["scar_shadow"])
            n_p += 1
            # what fell off is on the pavement below — but only for a patch
            # low enough that the eye connects the two, and only on the
            # storeys whose foot is the street.
            if crumb_p and e["storey"] <= 1 and rng.random() < crumb_p:
                _b_crumbs(ctx, fr, m, u)
        # DIAGONAL SHEAR CRACKS, IN THE PIER. EMS-98 grade 2-3 on masonry is
        # "large cracks in most walls", and in the field photographs they run
        # from the corner of one opening to the corner of the one above or
        # beside it — i.e. across the SOLID pier between openings, at 55-75
        # deg. Running one from 0.15 to 0.85 of the module width, as the
        # first pass did, draws it straight across the window.
        if rng.random() < crack_p:
            for j in range(rng.randrange(cracks[0], cracks[1] + 1)):
                if n_c >= max_cracks:
                    break
                sgn = 1.0 if rng.random() < 0.5 else -1.0
                u0 = width * rng.uniform(0.05, 0.14)
                u1 = width * rng.uniform(0.22, 0.34)
                if rng.random() < 0.5:              # the other pier
                    u0, u1 = width - u0, width - u1
                v0 = e["z"] + height * rng.uniform(0.10, 0.24)
                v1 = e["z"] + height * rng.uniform(0.74, 0.92)
                if sgn < 0:
                    v0, v1 = v1, v0
                _b_crack(ctx, fr, u0, v0, u1, v1, mats["crack"],
                         width=rng.uniform(0.010, 0.020),
                         jag=height * rng.uniform(0.02, 0.05))
                n_c += 1
    ctx["notes"].append("facade_scars: {0} patch(es), {1} crack(s)".format(n_p, n_c))


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


TANK_LEG_H = 1.1          # stand height under the barrel, m (NYC roof tanks
#                           sit 1.0-1.5 m up so the outlet can gravity-feed)


def _tank(ctx, x, y, z, r=1.2, h=2.6, yaw=0.0):
    """A cedar water tank on four legs, as ONE mesh; returns [path].

    ONE PRIM, NOT FIVE. The first version authored the barrel and the four
    legs as separate prims, and every consumer then had to keep them
    together by hand: `r_rooftop_fail` tipped the barrel and left the legs
    standing, a broken roof edge left legs hanging in the air under a tank
    that had been carried away, and as rigid bodies a barrel balanced on
    four thin separate boxes is a stack the solver has to hold up. Merged,
    the tank tips, falls and lands as the single object it is, and its
    convex hull is one cook.

    Local frame: the mesh is authored about (x, y, z) with translate /
    rotateZ ops, so `_transform_prims` and PhysX can both move it.
    """
    from pxr import Gf, Sdf, UsdGeom, Vt
    rng = ctx["rng"]
    pts, faces, counts = [], [], []

    def _add_box(cx, cy, cz, sx, sy, sz):
        hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
        b = len(pts)
        for dz in (-hz, hz):
            for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
                pts.append(Gf.Vec3f(cx + dx, cy + dy, cz + dz))
        for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
                  (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
            # .extend, NOT `faces +=`: an augmented assignment inside a
            # closure rebinds the name and Python makes it local
            faces.extend([b + i for i in f])
            counts.append(4)

    def _add_tube(rad, za, zb, n=18):
        b = len(pts)
        for k in range(n):
            a = 6.283 * k / n
            pts.append(Gf.Vec3f(rad * math.cos(a), rad * math.sin(a), za))
            pts.append(Gf.Vec3f(rad * math.cos(a), rad * math.sin(a), zb))
        for k in range(n):
            i, j = b + 2 * k, b + 2 * ((k + 1) % n)
            faces.extend([i, j, j + 1, i + 1])
            counts.append(4)
        return b

    leg_h = TANK_LEG_H
    z0, z1 = leg_h, leg_h + h
    for k in range(4):
        a = 0.785 + 1.571 * k
        _add_box((r * 0.72) * math.cos(a), (r * 0.72) * math.sin(a),
                 leg_h / 2.0, 0.14, 0.14, leg_h)
    N = 18
    base = _add_tube(r, z0, z1, N)
    top = len(pts)
    pts.append(Gf.Vec3f(0.0, 0.0, z1 + 0.35))            # shallow conical lid
    for k in range(N):
        faces += [base + 2 * k + 1, base + 2 * ((k + 1) % N) + 1, top]
        counts.append(3)
    bot = len(pts)
    pts.append(Gf.Vec3f(0.0, 0.0, z0))
    for k in range(N):
        faces += [base + 2 * ((k + 1) % N), base + 2 * k, bot]
        counts.append(3)
    # two proud hoops: the steel bands round a stave tank. Silhouette only —
    # same material — but they are what says "tank" and not "drum" at 40 m.
    for f in (0.32, 0.74):
        _add_tube(r * 1.045, z0 + h * f - 0.06, z0 + h * f + 0.06, N)

    path = "{0}/tank_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
    mesh = UsdGeom.Mesh.Define(ctx["stage"], Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(faces))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr([Gf.Vec3f(-r * 1.05, -r * 1.05, 0.0),
                           Gf.Vec3f(r * 1.05, r * 1.05, z1 + 0.35)])
    xf = UsdGeom.Xformable(mesh)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(float(yaw))
    _bind(ctx["stage"], path, ctx["mats"]["tank_wood"])
    ctx["authored"].append(path)
    return [path]


def dress_roof(ctx, mass="main", tanks=None, acs=None):
    """Rooftop plant on the top roof of `mass`: 0-2 water tanks and 2-6 AC
    units. Called for EVERY level including pristine, so the damaged
    grades have something to tip. Returns the prim paths."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    if len(m["levels"]) < 3:
        return []
    z = m["top"] + 0.02
    made, kind = [], ctx.setdefault("roof_plant_kind", {})

    def _spot(margin):
        """A point on the roof whose whole footprint is `margin` inside the
        wall line. INSET BY THE OBJECT, not by a constant: a 1.4 m tank
        placed 2.0 m in had its legs over the edge, and when the recipe
        broke that edge the legs were left standing on nothing."""
        hx = max(0.0, m["W"] / 2.0 - margin)
        hy = max(0.0, m["D"] / 2.0 - margin)
        return _to_world(m, rng.uniform(-hx, hx), rng.uniform(-hy, hy))

    nt = tanks if tanks is not None else rng.choice((0, 1, 1, 2))
    for k in range(nt):
        r = rng.uniform(1.0, 1.4)
        wx, wy = _spot(r + 0.9)          # barrel radius + the parapet band
        for pth in _tank(ctx, wx, wy, z, r=r, h=rng.uniform(2.2, 3.0),
                         yaw=m["yaw"] + rng.uniform(0, 90)):
            made.append(pth)
            kind[pth] = "tank"
    na = acs if acs is not None else rng.randrange(2, 7)
    for k in range(na):
        wx, wy = _spot(1.8)              # the AC unit is ~1.0 x 0.5 m
        path = "{0}/ac_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        if _prop(ctx["stage"], path, _AC_UNIT, wx, wy, z, m["yaw"] + rng.choice((0, 90, 180, 270)),
                 1.0, rng):
            _b_bind_over(ctx["stage"], path, ctx["mats"]["plant_metal"])
            made.append(path)
            kind[path] = "ac"
            ctx["authored"].append(path)
    ctx["roof_plant"] = made
    ctx["roof_plant_mass"] = mass
    return made


def r_rooftop_fail(ctx, frac=0.5, tip_deg=(25.0, 45.0)):
    """Rooftop plant in a quake: `frac` of the tanks / AC units are kicked
    past balance so the solver lays them over. Run after `dress_roof`.

    IT ONLY TIPS. Making the plant a rigid body, and taking it out of the
    fit-out list so a later whole-body transform skips it, is
    `_b_settle_roof_plant`'s job at the end of `wreck_building` — because
    the roof under a prop can be dropped, holed or carried away by a recipe
    that runs AFTER this one, and a prop that was already loose here would
    have been left behind at the old roof height (the floating tanks over
    the pancaked block in quake_city9)."""
    rng = ctx["rng"]
    tipped = ctx.setdefault("roof_plant_tipped", set())
    for pth in list(ctx.get("roof_plant", [])):
        if rng.random() >= frac:
            continue
        M = _rot_about(_pivot_of(ctx, pth), (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0),
                       rng.uniform(*tip_deg))
        _transform_prims(ctx["stage"], [pth], M)
        tipped.add(pth)


# Recipes after which the roof plant is BURIED rather than merely dropped:
# the building came down as a whole and a cedar tank does not survive on top
# of the mound (Amatrice / Antakya: rooftop tanks are found crushed inside
# the pile, and only sometimes on it).
_B_TOTAL_COLLAPSE = ("masonry_collapse", "pancake")
B_ROOF_PLANT_BURIED = 0.55      # share deleted on a total collapse
B_ROOF_PLANT_TIP_DEG = 7.0      # idle shake tip for plant nobody kicked over


def _b_settle_roof_plant(ctx, recipes=()):
    """Make the rooftop plant FOLLOW ITS ROOF — the last thing a recipe run
    does to a building.

    `dress_roof` seats tanks and AC units on the roof slab in world space
    BEFORE any recipe runs, and every recipe that drops, holes, lowers,
    tips or removes that roof used to leave them exactly where they were:
    the user's "floating elements (water tanks, etc) on top of buildings".
    Recipes that move the whole shell (`settlement`, `tilt_severe`,
    `overturn`) carry the plant with `_everything`, which is right for the
    ride but wrong for the landing — a tank welded to a roof that is now
    vertical is just as floating.

    Rather than work out per recipe what happened to which slab, the plant
    is simply HANDED TO THE SOLVER: every piece becomes a rigid body over
    whatever the recipes left. If the roof survived, it falls its 2 cm and
    rests on it (kit roof tiles are zero-thickness quads but static
    colliders are cooked as triangle meshes, so they do hold — see
    `settle.prepare`); if the roof was holed, dropped a storey, fractured
    or carried away, it falls onto the slab, the pile or the ground; on a
    building that went over it slides off the face it is now stuck to.

    `recipes` is the recipe list the building was given, so a total
    collapse can bury a share of the plant instead."""
    plant = list(ctx.get("roof_plant", []))
    if not plant:
        return 0
    rng = ctx["rng"]
    names = {n for n, _kw in (recipes or ())}
    total = bool(names & set(_B_TOTAL_COLLAPSE))
    tipped = ctx.get("roof_plant_tipped", set())
    n_loose = n_buried = 0
    for pth in plant:
        if total and rng.random() < B_ROOF_PLANT_BURIED:
            _deactivate(ctx["stage"], pth)
            n_buried += 1
            continue
        if pth not in tipped:
            # nobody kicked this one over, but it still stood through the
            # shaking: a couple of degrees so the row of AC units on a
            # damaged roof is not a parade ground.
            M = _rot_about(_pivot_of(ctx, pth),
                           (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0),
                           rng.uniform(-B_ROOF_PLANT_TIP_DEG, B_ROOF_PLANT_TIP_DEG))
            _transform_prims(ctx["stage"], [pth], M)
        ctx["loose"].append(pth)
        n_loose += 1
    ctx["fit"]["all"] = [q for q in ctx["fit"]["all"] if q not in set(plant)]
    # A LEDGE ZONE for the post-settle cull (`settle.run(cull_ledges=...)`):
    # only for a building that is still standing. On a collapse the band
    # just outside the wall line IS the rubble windrow and culling there
    # would eat the debris field.
    if not total and "overturn" not in names:
        try:
            from . import settle as _settle
            m = ctx["info"]["masses"][ctx.get("roof_plant_mass", "main")]
            _settle.register_ledge_zone(m["cx"], m["cy"], m["W"], m["D"],
                                        m["yaw"], m["z0"])
        except Exception:
            pass
    ctx["notes"].append("roof_plant: {0} dropped to physics, {1} buried".format(
        n_loose, n_buried))
    return n_loose


SIGN_MATS = ("sign", "sign_red", "sign_blue")


def r_signage_fail(ctx, n=None, awnings=None, lean_p=0.4):
    """Shop signage down on the sidewalk, and awnings torn off the front.

    PAINTED SHEET, NOT PLASTER, AND FLAT OR LEANING, NOT ON END. The round-1
    version stood a 2-4 m plaster box on its end at 60-88 deg; on the bench
    that is a white plank propped in mid-air on the pavement, which is what
    the user called "random white debris". A fascia sign that comes off a
    shopfront lands face down on the pavement or slides down the glass and
    leans against the stallriser; either way it is a dark painted panel.
    """
    rng = ctx["rng"]
    m = ctx["info"]["masses"]["main"]
    k = n if n is not None else rng.randrange(1, 4)
    na = awnings if awnings is not None else rng.randrange(0, 3)

    def _spot(side, d):
        L = m["W"] if side in ("S", "N") else m["D"]
        t = rng.uniform(-0.42, 0.42) * L
        if side in ("S", "N"):
            lx, ly = t, (-m["D"] / 2.0 - d if side == "S" else m["D"] / 2.0 + d)
        else:
            lx, ly = (-m["W"] / 2.0 - d if side == "W" else m["W"] / 2.0 + d), t
        return _to_world(m, lx, ly)

    for i in range(k):
        side = rng.choice(["S", "S", "E", "W"])
        ox, oy = _outward(m, side)
        ax, ay = -oy, ox                     # along the wall (left perpendicular)
        Lp = rng.uniform(1.6, 3.6)           # fascia length
        Hp = rng.uniform(0.55, 1.15)         # fascia depth
        mat = ctx["mats"][SIGN_MATS[rng.randrange(len(SIGN_MATS))]]
        path = "{0}/sign_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        if rng.random() < lean_p:
            # LEANING on the shopfront: stood up, then rotated back INTO the
            # wall about its foot (-theta about the along-wall axis tips
            # inboard; +theta tips outward — the sign rule in the skill).
            th = rng.uniform(16.0, 34.0)
            d = Hp * math.sin(math.radians(th)) + rng.uniform(0.05, 0.3)
            wx, wy = _spot(side, d)
            _box(ctx["stage"], path, wx, wy, m["z0"] + Hp / 2.0,
                 Lp, 0.08, Hp, m["yaw"] + rng.uniform(-8, 8), mat)
            _transform_prims(ctx["stage"], [path],
                             _rot_about((wx, wy, m["z0"]), (ax, ay, 0.0), -th))
        else:
            # FLAT on the pavement, face down, with a couple of degrees of
            # roll where it has landed on its own bent frame.
            d = rng.uniform(0.5, 2.2)
            wx, wy = _spot(side, d)
            _box(ctx["stage"], path, wx, wy, m["z0"] + 0.045,
                 Lp, Hp, 0.08, m["yaw"] + rng.uniform(-40, 40), mat)
            _transform_prims(ctx["stage"], [path],
                             _rot_about((wx, wy, m["z0"]),
                                        (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0),
                                        rng.uniform(2.0, 9.0)))
        ctx["authored"].append(path)
        ctx["static_extra"].append(path)

    # AWNINGS: canvas over a bent frame. Half stay attached and sag off the
    # shopfront head at 30-60 deg, half are on the pavement in a heap.
    for i in range(na):
        side = rng.choice(["S", "S", "E", "W"])
        ox, oy = _outward(m, side)
        ax, ay = -oy, ox
        Wp = rng.uniform(2.0, 4.0)
        proj = rng.uniform(1.1, 2.2)
        path = "{0}/awning_{1}_{2}".format(ctx["parent"], ctx["tag"], _uid(ctx))
        if rng.random() < 0.5:
            zt = m["z0"] + rng.uniform(2.4, 3.2)
            th = rng.uniform(32.0, 62.0)
            wx, wy = _spot(side, proj / 2.0)
            _box(ctx["stage"], path, wx, wy, zt, Wp, proj, 0.06,
                 m["yaw"] + rng.uniform(-4, 4), ctx["mats"]["awning"])
            px, py = _spot(side, 0.0)
            _transform_prims(ctx["stage"], [path],
                             _rot_about((px, py, zt), (ax, ay, 0.0), th))
        else:
            # torn off: canvas off its frame folds up, so the footprint on the
            # pavement is well under the awning's opened size
            Wp, proj = Wp * rng.uniform(0.45, 0.7), proj * rng.uniform(0.5, 0.8)
            wx, wy = _spot(side, rng.uniform(0.4, 1.8))
            _box(ctx["stage"], path, wx, wy, m["z0"] + 0.05, Wp, proj, 0.07,
                 m["yaw"] + rng.uniform(-50, 50), ctx["mats"]["awning"])
            _transform_prims(ctx["stage"], [path],
                             _rot_about((wx, wy, m["z0"]),
                                        (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0),
                                        rng.uniform(3.0, 12.0)))
        ctx["authored"].append(path)
        ctx["static_extra"].append(path)


# ---------------------------------------------------------------------------
# BATCH D — BUILDINGS THAT TOUCH EACH OTHER
# ---------------------------------------------------------------------------
# Everything above damages ONE building in isolation, which is what an
# archetype bake can hold. Three of the most legible earthquake signatures
# belong to a PAIR and cannot be baked per building:
#
# * POUNDING. Neighbours separated by less than the seismic joint hammer each
#   other at every SLAB level — the floor diaphragm is the stiff part of a
#   frame and it lands on the neighbour's wall between its own floors. Mexico
#   City 1985: ~40 % of the damaged buildings had struck a neighbour and ~15 %
#   of the collapses were attributed to it; Christchurch 2011 repeated it on
#   the URM party walls of High / Colombo St. So the marks are HORIZONTAL
#   BANDS at the shorter building's storey pitch on BOTH facing walls, a
#   crushed cornice where the shorter roof line meets the taller wall, and a
#   wedge of the fallen material in the gap — not one vertical scar.
# * LEAN-ON. A bearing failure rotates a block about its far base edge until
#   its top corner meets the neighbour and it STOPS there, held up by it:
#   Adapazari 1999 (whole apartment blocks resting against each other), Kobe
#   1995, L'Aquila 2009. The angle is GEOMETRY, not a draw — it follows from
#   the gap, the depth and the two heights (`_d_contact_angle`) — and the
#   contact damages both: the leaner's top corner crushes, the neighbour
#   loses its cornice and a band of wall at the contact line.
# * COLLAPSE-ONTO. A taller building's upper storeys fail toward a lower
#   neighbour and land ON its roof, punching the diaphragm and burying the
#   top floor (Kobe 1995; Amatrice 2016, where one rowhouse unit's upper
#   floors came down on the next unit's roof).
#
# The bench builds the neighbour itself (`EQ_NEIGHBOUR=<style>,<gap>,<side>`)
# and `quake._d_interactions` runs these for a capped number of pairs at
# assembly. Neighbours reach a recipe through `ctx["neighbours"]`, filled
# from the registry below by `_d_nbs`, so no existing function has to change.
_D_NEIGHBOURS = {}


def d_set_neighbours(tag, nbs):
    """Register the neighbours of the building `tag`, BEFORE `wreck_building`
    runs on it. Callers: the bench launcher's `EQ_NEIGHBOUR` block and
    `quake._d_interactions`."""
    _D_NEIGHBOURS[str(tag)] = list(nbs or [])
    return _D_NEIGHBOURS[str(tag)]


def d_neighbour(style, placements, x, y, yaw, side, gap, parent=None, tag=None):
    """One neighbour record from a PLACED KIT building: its mass frame, its
    elements (so a recipe can break it too) and which side of the subject it
    stands on. `gap` is the clear distance between the two facing walls."""
    info = describe(style, placements, x, y, yaw)
    return {"side": side, "gap": float(gap), "info": info,
            "m": info["masses"]["main"], "H": info["H"], "style": style,
            "parent": parent, "tag": tag or "nb",
            "paths": [p.get("prim_path") for p in placements if p.get("prim_path")]}


def d_box_mass(cx, cy, yaw, W, D, H, storey_h=3.4):
    """A mass-like dict for a building known only as a BOX — a placed
    archetype in the city, whose kit pieces are inside a reference and can be
    neither classified nor fractured. Every geometric helper here takes one."""
    n = max(1, int(round(H / storey_h)))
    return dict(tag="main", spec=None, cx=float(cx), cy=float(cy),
                yaw=float(yaw), W=float(W), D=float(D), z0=0.0,
                levels=[i * (float(H) / n) for i in range(n)], top=float(H),
                module=5.0)


def d_box_neighbour(cx, cy, yaw, W, D, H, side, gap, storey_h=3.4):
    """A neighbour that is only a box (see `d_box_mass`). `info` is None, so
    the recipes author their geometric half and skip every fracture."""
    return {"side": side, "gap": float(gap), "info": None,
            "m": d_box_mass(cx, cy, yaw, W, D, H, storey_h), "H": float(H),
            "style": None, "parent": None, "tag": "nbbox", "paths": []}


def _d_nbs(ctx, side=None):
    """`ctx["neighbours"]`, filled from the registry on first use."""
    nbs = ctx.get("neighbours")
    if nbs is None:
        nbs = list(_D_NEIGHBOURS.get(str(ctx.get("tag")), ()))
        ctx["neighbours"] = nbs
    return [n for n in nbs if side is None or n["side"] == side]


def _d_pick(ctx, side=None, want="near"):
    """The neighbour a recipe should use: the closest, or the lowest /
    tallest when the recipe needs a height relation."""
    nbs = _d_nbs(ctx, side)
    if not nbs:
        return None
    if want == "lower":
        return min(nbs, key=lambda n: n["H"])
    if want == "taller":
        return max(nbs, key=lambda n: n["H"])
    return min(nbs, key=lambda n: n["gap"])


def _d_nctx(ctx, nb):
    """A ctx over the NEIGHBOUR that shares the subject's output lists, so its
    fragments fracture into the same settle. None for a box neighbour."""
    if nb.get("info") is None:
        return None
    n = nb.get("_ctx")
    if n is not None:
        return n
    n = dict(ctx)                  # shares loose / static_extra / velocity / ...
    n["info"] = nb["info"]
    n["parent"] = nb.get("parent") or ctx["parent"]
    n["tag"] = "{0}n".format(nb.get("tag") or ctx["tag"])
    n["n_uid"] = 0
    n["neighbours"] = []
    n["fit"] = nb.get("fit") or {"slabs": {}, "columns": {},
                                 "partitions": [], "props": {}, "all": []}
    nb["fit"] = n["fit"]
    nb["_ctx"] = n
    return n


# --- pair geometry ---------------------------------------------------------
def _d_frame(m, side):
    """(outward, lateral, B, L) for one side of a mass: the outward unit
    vector, the unit vector along that wall, the mass's depth in the outward
    direction (the lever arm when it rotates about the FAR base edge) and the
    length of the wall."""
    ox, oy = _outward(m, side)
    B = m["D"] if side in ("S", "N") else m["W"]
    L = m["W"] if side in ("S", "N") else m["D"]
    return (ox, oy), (-oy, ox), B, L


def _d_lat_of(e, side):
    """A kit element's coordinate ALONG the wall of `side`, in its own mass
    frame (local +x for S/N, local +y for E/W)."""
    return e["lx"] if side in ("S", "N") else e["ly"]


def _d_facing_side(m, ux, uy):
    """Which side of `m` faces the world direction (ux, uy)."""
    best, bd = "S", -9e9
    for s in ("S", "E", "N", "W"):
        ox, oy = _outward(m, s)
        d = ox * ux + oy * uy
        if d > bd:
            bd, best = d, s
    return best


def _d_span(m, side, nm, pad=0.0):
    """The interval along `side`'s wall (in m's local lateral coordinate) over
    which the two footprints face each other — the length of wall that can
    actually touch. None when they do not overlap."""
    (ox, oy), (ax, ay), B, L = _d_frame(m, side)
    c = (nm["cx"] - m["cx"]) * ax + (nm["cy"] - m["cy"]) * ay
    same = abs(math.cos(math.radians(nm["yaw"] - m["yaw"]))) > 0.7
    if side in ("S", "N"):
        Ln = nm["W"] if same else nm["D"]
    else:
        Ln = nm["D"] if same else nm["W"]
    lo = max(-L / 2.0, c - Ln / 2.0) - pad
    hi = min(L / 2.0, c + Ln / 2.0) + pad
    return (lo, hi) if hi - lo > 1.0 else None


def d_rect_gap(a, b):
    """(gap_m, unit normal from a to b) for two rectangles (cx, cy, yaw, W, D),
    by separating-axis distance over the four candidate axes; negative means
    the footprints overlap.

    YAW-AWARE, which the city's first pounding test was not: it compared
    |dx| against W and |dy| against D, and half the buildings on a downtown
    block are laid at yaw 90, where W is the depth. That test could only ever
    fire on the axis-aligned half of the city."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    best, bn = -1e9, (1.0, 0.0)
    for r in (a, b):
        for k in (0, 1):
            th = math.radians(r[2] + 90.0 * k)
            ux, uy = math.cos(th), math.sin(th)
            hs = 0.0
            for q in (a, b):
                e1 = math.radians(q[2])
                c1, s1 = math.cos(e1), math.sin(e1)
                hs += (abs(ux * c1 + uy * s1) * q[3] / 2.0
                       + abs(-ux * s1 + uy * c1) * q[4] / 2.0)
            d = dx * ux + dy * uy
            sep = abs(d) - hs
            if sep > best:
                best = sep
                bn = (ux, uy) if d >= 0 else (-ux, -uy)
    return best, bn


def d_rect_overlap(a, b, n):
    """How much of two rectangles face each other ACROSS the normal `n` — the
    length of shared façade. <= 0 means they are diagonal neighbours."""
    vx, vy = -n[1], n[0]
    hs = 0.0
    for q in (a, b):
        e1 = math.radians(q[2])
        c1, s1 = math.cos(e1), math.sin(e1)
        hs += (abs(vx * c1 + vy * s1) * q[3] / 2.0
               + abs(-vx * s1 + vy * c1) * q[4] / 2.0)
    return hs - abs((b[0] - a[0]) * vx + (b[1] - a[1]) * vy)


def _d_contact_angle(H, B, gap, Hn, max_deg=30.0, step=0.25):
    """The angle at which a block H tall and B deep, rotating about its FAR
    base edge, first TOUCHES a neighbour whose facing wall stands `gap` clear
    of its near wall and whose parapet is `Hn` high (both heights measured
    from the block's own base). Returns (deg, z_contact) or (None, None) when
    it never touches inside `max_deg` — too far, or so much lower that the
    block sails over its roof.

    Rotating +theta about the far base edge maps a point (u outward, h up) to
    (u cos + h sin, h cos - u sin), so the near face reaches the neighbour's
    wall plane at h = (B + gap - B cos) / sin, which FALLS as theta grows:
    first contact is the top corner, and past that the face slides down the
    neighbour's corner."""
    th = step
    while th <= max_deg + 1e-6:
        r = math.radians(th)
        c, s = math.cos(r), math.sin(r)
        h = (B + gap - B * c) / s
        if h <= H:
            z = h * c - B * s
            if z <= Hn:
                return th, z
        th += step
    return None, None


# --- authored pair debris --------------------------------------------------
def _d_rubble_mat(ctx, urm=None):
    """The dusty rubble palette, as `_heap` draws it."""
    rng, mats = ctx["rng"], ctx["mats"]
    if urm is None:
        urm = ctx["info"].get("type") == "urm"
    r = rng.random()
    if urm:
        return (mats["brick"] if r < 0.6 else
                mats["mortar"] if r < 0.9 else mats["plaster"])
    return (mats["concrete"] if r < 0.5 else
            mats["dark_concrete"] if r < 0.92 else mats["plaster"])


def _d_chunk(ctx, m, lx, ly, z, s, tag="d", mat=None, flat=False):
    """One authored rubble chunk, in a mass's local frame."""
    rng = ctx["rng"]
    wx, wy = _to_world(m, lx, ly)
    path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
    _box(ctx["stage"], path, wx, wy, z, s, s * rng.uniform(0.5, 1.0),
         (0.012 if flat else s * rng.uniform(0.35, 0.7)), rng.uniform(0, 180),
         mat if mat is not None else _d_rubble_mat(ctx))
    ctx["authored"].append(path)
    if not flat:
        ctx["static_extra"].append(path)
    return path


def _d_face_band(ctx, m, side, z, span, height=(0.45, 0.85), thick=0.11,
                 n_seg=(3, 7), tag="spall", dark=0.6):
    """SPALLED RENDER on a wall face, as a BROKEN horizontal band at height
    `z` — what a slab edge leaves on the wall it hammers, and what a lean-on
    contact leaves along the contact line. Three to seven short boxes with
    gaps between them, jittered in height and length, half-buried in the wall
    so the proud half reads as exposed brick / concrete. One continuous strip
    is the thing that reads as a painted stripe, so there is never one."""
    rng = ctx["rng"]
    mats = ctx["mats"]
    urm = ctx["info"].get("type") == "urm"
    made = []
    lo, hi = span
    if hi - lo < 0.8:
        return made
    n = rng.randrange(*n_seg)
    t = lo + rng.uniform(0.0, 0.25) * (hi - lo)
    for k in range(n):
        L = rng.uniform(0.14, 0.34) * (hi - lo)
        if t + L > hi:
            break
        h = rng.uniform(*height)
        zc = z + rng.uniform(-0.30, 0.10) * h
        mat = (mats["dark_concrete"] if rng.random() < dark
               else (mats["brick"] if urm else mats["concrete"]))
        if side in ("S", "N"):
            lx, ly = t + L / 2.0, (-m["D"] / 2.0 if side == "S" else m["D"] / 2.0)
            sx, sy = L, thick
        else:
            lx, ly = (-m["W"] / 2.0 if side == "W" else m["W"] / 2.0), t + L / 2.0
            sx, sy = thick, L
        wx, wy = _to_world(m, lx, ly)
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        _box(ctx["stage"], path, wx, wy, zc, sx, sy, h, m["yaw"], mat)
        ctx["authored"].append(path)
        ctx["static_extra"].append(path)
        made.append(path)
        t += L + rng.uniform(0.06, 0.55) * (hi - lo) / n
    return made


def _d_party_ghost(ctx, m, side, span, levels, tag="ghost"):
    """The GHOST OF THE FLOOR LINES on a party wall the neighbour has left:
    when one unit of a terrace comes down, the wall it shared is legible as
    horizontal plaster lines with the joist pockets and torn wall ties in
    them (Amatrice, L'Aquila). Thin proud strips plus a few stub ends."""
    rng = ctx["rng"]
    mats = ctx["mats"]
    made = []
    lo, hi = span
    for z in levels:
        if z < 1.5:
            continue
        made += _d_face_band(ctx, m, side, z, (lo + 0.3, hi - 0.3),
                             height=(0.16, 0.28), thick=0.09, n_seg=(2, 4),
                             tag=tag, dark=0.15)
        for k in range(rng.randrange(2, 5)):
            t = rng.uniform(lo + 0.5, hi - 0.5)
            s = rng.uniform(0.14, 0.26)
            if side in ("S", "N"):
                lx, ly = t, (-m["D"] / 2.0 if side == "S" else m["D"] / 2.0)
            else:
                lx, ly = (-m["W"] / 2.0 if side == "W" else m["W"] / 2.0), t
            made.append(_d_chunk(ctx, m, lx, ly, z - 0.12, s, tag=tag + "_tie",
                                 mat=(mats["timber"] if rng.random() < 0.6
                                      else mats["rebar"])))
    return made


def _d_corner_spans(m, side, corner_m=1.8):
    """The two short spans on the FLANKING walls right where a pounding gap
    meets the street. A 0.4 m slot between two buildings is a dark slit: the
    bands inside it are invisible from any camera, and the only part of the
    damage a street view can see is what wraps round the corner pier — which
    is also where the real spalling concentrates, because the pier is the
    stiff bit that takes the hit."""
    W, D = m["W"], m["D"]
    if side in ("E", "W"):
        end = W / 2.0 if side == "E" else -W / 2.0
        sp = (end - corner_m, end) if side == "E" else (end, end + corner_m)
        return [("S", sp), ("N", sp)]
    end = D / 2.0 if side == "N" else -D / 2.0
    sp = (end - corner_m, end) if side == "N" else (end, end + corner_m)
    return [("E", sp), ("W", sp)]


def _d_gap_debris(ctx, m, side, span, gap, depth_m=0.7, spill_m=2.6,
                  density=1.4, glass=0.12, tag="gapdeb"):
    """The WEDGE IN THE GAP: what came off both walls, packed into the slot
    between them and spilling out of its two open ends onto the street.

    The slot is only `gap` wide, and `_heap`'s windrow has a 1.5 m minimum
    reach — it would push half the pile through the neighbour. This fills the
    actual slot and fans out past its ends instead."""
    rng = ctx["rng"]
    lo, hi = span
    reach = max(0.35, gap)
    n = int(max(14, (hi - lo + 2 * spill_m) * (reach + 0.6) * density * 3.0))
    B = m["D"] if side in ("S", "N") else m["W"]
    made = []
    for k in range(n):
        t = rng.uniform(lo - spill_m, hi + spill_m)
        out = max(0.0, lo - t, t - hi)                 # how far past the slot end
        d = rng.uniform(0.1, reach + (1.6 if out > 0.05 else 0.0))
        zmax = depth_m * max(0.15, 1.0 - out / max(spill_m, 0.1))
        s = rng.uniform(0.14, 0.42 if out > 0.05 else 0.55)
        is_glass = rng.random() < glass
        if side in ("S", "N"):
            lx, ly = t, (-B / 2.0 - d if side == "S" else B / 2.0 + d)
        else:
            lx, ly = (-B / 2.0 - d if side == "W" else B / 2.0 + d), t
        made.append(_d_chunk(
            ctx, m, lx, ly,
            m["z0"] + (0.012 if is_glass else rng.uniform(0.0, zmax) + s * 0.15),
            s * (1.4 if is_glass else 1.0), tag=tag,
            mat=(ctx["mats"]["glass"] if is_glass else None), flat=is_glass))
    return made


def _d_roof_heap(ctx, m, base, lx, ly, r, h, tag="impact", n_max=420):
    """A LOCALISED heap on a roof (or a slab): a dome of radius `r` about the
    local (lx, ly), peak `h`. `_heap(fill=True)` covers the whole footprint
    and skirts past it — wrong for a mass that landed on one end of a
    neighbour's roof."""
    rng = ctx["rng"]
    n = int(min(n_max, max(40, math.pi * r * r * h * 0.55 / 0.9)))
    made = []
    for k in range(n):
        u, v = rng.uniform(-1, 1), rng.uniform(-1, 1)
        rr = math.hypot(u, v)
        if rr > 1.0:
            continue
        zmax = h * max(0.0, 1.0 - rr ** 1.5)
        s = rng.uniform(0.3, 1.5) * (1.0 if rr < 0.7 else 0.7)
        made.append(_d_chunk(ctx, m, lx + u * r, ly + v * r,
                             base + rng.uniform(0.0, zmax) + s * 0.15, s, tag=tag))
    return made


def _d_roof_dist(m, e, side):
    """A roof tile's distance in from the wall line of `side`, mass frame."""
    if side == "S":
        return e["ly"] + m["D"] / 2.0
    if side == "N":
        return m["D"] / 2.0 - e["ly"]
    if side == "W":
        return e["lx"] + m["W"] / 2.0
    return m["W"] / 2.0 - e["lx"]


def _d_roof_near(ctx, m, side, mass="main", reach=4.0):
    """The roof tiles within `reach` of one wall line — the only ones a recipe
    working on that side should turn into `_roof_box` slabs.

    A KIT ROOF TILE IS DARK; A `_roof_box` SLAB IS PALE CONCRETE. Converting
    every tile of a mass in order to break the edge along ONE side leaves the
    whole roof a pale checkerboard from the air, which is what the first
    lean-on and collapse-onto benches produced. (`_ragged_slabs` still does
    exactly that to every recipe that calls it, `out_of_plane` included —
    `_d_far_roof_hidden` is the local guard, not a fix.)"""
    return [e for e in _els(ctx, mass=mass, role="roof")
            if _d_roof_dist(m, e, side) <= reach]


class _d_far_roof_hidden(object):
    """Context manager: mark the roof tiles FURTHER than `reach` from `side`
    dead for the duration, so a helper that iterates every roof tile of the
    mass only touches the ones on the failed side. They are alive again on
    exit, so they stay kit tiles and `wreck_building` still finds them."""

    def __init__(self, ctx, m, side, mass="main", reach=4.0):
        self.hidden = [e for e in _els(ctx, mass=mass, role="roof")
                       if _d_roof_dist(m, e, side) > reach]

    def __enter__(self):
        for e in self.hidden:
            e["dead"] = True
        return self

    def __exit__(self, *exc):
        for e in self.hidden:
            e["dead"] = False
        return False


def _d_crush_band(ctx, m, side, z_lo, z_hi, span=None, mass="main",
                  loose_frac=0.55, parapet=True, out_dir=None, p_module=1.0,
                  n_pieces=(7, 12), vel=(0.3, 1.2), tag="crush"):
    """Break the KIT MODULES of one face inside a HEIGHT BAND — the contact
    line of a lean-on, a slab line of a pounding pair, the crushed cornice of
    either. A module the band only clips is cut PARTIALLY (a course off its
    top), one the band covers comes apart whole; `loose_frac` of the fragments
    fall and the rest stay where they are, so the band reads as a crushed,
    spalled wall and never as a rectangular cut-out. Returns modules touched."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    (ox, oy), _lat, _B, _L = _d_frame(m, side)
    if out_dir is not None:
        ox, oy = out_dir
    n = 0
    for e in list(_els(ctx, mass=mass, side=side,
                       role=("wall", "corner", "parapet", "parapet_corner",
                             "balcony"))):
        if not parapet and e["role"] in ("parapet", "parapet_corner"):
            continue
        if span is not None:
            t = _d_lat_of(e, side)
            if not (span[0] - 2.5 <= t <= span[1] + 2.5):
                continue
        z0e, z1e = e["z"], e["z"] + max(0.5, e["h"])
        if z1e < z_lo or z0e > z_hi:
            continue
        if p_module < 1.0 and rng.random() > p_module:
            continue
        path = e["p"].get("prim_path")
        if not path:
            continue
        cover = (min(z1e, z_hi) - max(z0e, z_lo)) / (z1e - z0e)
        partial = None
        if cover < 0.8:
            # A PARTIAL CUT CAN ONLY TAKE THE TOP OFF (`fracture_partial` cuts
            # at a height fraction and everything above it comes loose), so a
            # band that only clips a module's FOOT has to leave it alone — the
            # first pounding bench asked for a 0.9 m band at each slab line,
            # every module whose BASE sat on that line computed a 0.12 cut,
            # and 88 % of both facing walls came down: a black slot where a
            # spalled band should have been.
            if not (z_lo <= z1e <= z_hi):
                continue
            # keep what is below the band: cut at the band's lower edge
            partial = max(0.12, min(0.9, (max(z0e, z_lo) - z0e) / (z1e - z0e)))
        st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                        rng.randrange(*n_pieces), rng, nrng, ctx["mats"],
                        ctx["cache"], ctx["info"]["type"], inner_p=0.45,
                        partial=partial, consume=0.18)
        keep = [q for q in lo if rng.random() >= loose_frac]
        drop = [q for q in lo if q not in keep]
        for q in drop:
            v = rng.uniform(*vel)
            ctx["velocity"][q] = (ox * v, oy * v, -rng.uniform(0.1, 0.7))
        ctx["loose"] += drop
        ctx["static_extra"] += st + keep
        e["dead"] = True
        n += 1
    return n


def _d_ground_response(ctx, m, side, sink_m, lift_m=0.0):
    """The ground under a building that rotated: heaved soil on the side that
    went down, and on the side that came UP a berm as tall as the lift, so the
    raft does not hang over a clean void. Agent C owns the ground half of the
    foundation recipes — use theirs when it is on the module, otherwise the
    berm pair `r_tilt_severe` uses.

    Deliberately FEW buckled pavement slabs: at a 33 m lean they were the
    loudest thing in the frame — pale flat plates on black asphalt that read
    as torn paper — and the berm is what says "the ground failed"."""
    fn = globals().get("_c_ground_response")
    if callable(fn):
        try:
            # agent C's unified entry point: low side + how far it dropped +
            # how far the other side came up. Everything below is the
            # fallback for a tree where that does not exist yet.
            return fn(ctx, m, low_side=side, drop_m=sink_m, rise_m=lift_m)
        except TypeError:
            try:
                return fn(ctx, m, side, sink_m)
            except TypeError:
                pass
    rng = ctx["rng"]
    _berm(ctx, m, side, crest_m=0.35 + 0.5 * min(1.0, sink_m / 1.5), reach_m=2.4)
    _berm(ctx, m, _opposite(side), crest_m=max(0.25, min(1.1, 0.7 * lift_m)),
          reach_m=2.2)
    # the silt squeezed out on the low side, as `tilt_severe` does it
    _heap(ctx, m, m["z0"], 0.0, 0.10, fill=False, sides=(side,),
          depth_m=0.35 + 0.45 * min(1.0, sink_m / 1.5), tag="silt",
          mat_fn=lambda: ctx["mats"]["soil"])
    # boils stay CLOSE: reach_frac 1.2 puts them 20 m out on a 33 m block,
    # where they read as tan stains in the middle of the road
    _ejecta(ctx, m, 2 + rng.randrange(3), reach_frac=0.35, bias_side=side)
    _buckled_pavement(ctx, m, max(2, int((m["W"] + m["D"]) * 0.08)),
                      sides=(side, _opposite(side)))
    return None


# --- the recipes -----------------------------------------------------------
def r_lean_on(ctx, side=None, mass="main", max_deg=30.0, crush_m=0.35,
              sink_frac=0.55, crush_storeys=1.35, band_storeys=1.0,
              debris=True, lean_anyway=False):
    """The building leans onto its NEIGHBOUR and stops against it.

    A bearing failure under one side rotates the block about its far base
    edge; it comes to rest the moment its top corner meets the neighbour's
    wall, and the angle is whatever the gap and the two heights make it —
    Adapazari 1999, where blocks that would have gone over were caught by the
    next block along. The contact damages BOTH: the leaner's top corner and
    the neighbour's wall at the contact line spall and lose their cornice,
    glass on both faces goes, and what comes off lands in the slot between
    them and on the street at its open ends.

    `crush_m` is what the contact zone GRINDS AWAY before it locks: the block
    rotates past first touch by that much, and it is the only reason a party
    wall (gap ~ 0) leans at all. It also bounds the interpenetration — the two
    shells overlap by at most `crush_m` at the contact, which is what the
    crushed band there is covering up. Real masonry crushing at a pounding
    contact is 0.1-0.5 m, so 0.35 sits in the middle of it.
    """
    rng = ctx["rng"]
    info = ctx["info"]
    m = info["masses"][mass]
    nb = _d_pick(ctx, side)
    if nb is None:
        ctx["notes"].append("lean_on: no neighbour registered — nothing done")
        return
    side = nb["side"]
    (ox, oy), (ax, ay), B, L = _d_frame(m, side)
    H = m["top"] - m["z0"]
    st_h = (m["levels"][1] - m["levels"][0]) if len(m["levels"]) > 1 else 3.4
    nm = nb["m"]
    gap = max(0.02, float(nb["gap"]))
    # rotate past first touch by `crush_m` of ground-away contact material
    deg, zc = _d_contact_angle(H, B, gap + max(0.0, crush_m),
                               nb["H"] - m["z0"], max_deg=max_deg)
    touched = deg is not None
    crushing = touched and deg < 4.0        # a party wall: no room to rotate
    if not touched:
        # NOTHING TO LEAN ON. A block 40 m deep and 22 m tall (office_wide)
        # turns its top over by H sin - B (1 - cos), which at 30 deg is barely
        # its own height — it cannot reach a neighbour 6 m away, and leaning it
        # anyway is `tilt_severe` under another name. Say so and stop; the
        # city's pair search (`quake._d_pairs`) never offers such a pair.
        ctx["notes"].append(
            "lean_on: NOT DONE — {0} m tall on a {1} m base cannot reach a "
            "{2} m neighbour {3:.2f} m away inside {4:.0f} deg (it would have "
            "to slide, not rotate)".format(
                round(H), round(B), round(nb["H"]), gap, max_deg))
        if not lean_anyway:
            return
        deg = max_deg * 0.75
        zc = H * math.cos(math.radians(deg)) - B * math.sin(math.radians(deg))
    r = math.radians(deg)
    # The near base edge digs in and the far edge levers its raft out of the
    # ground (Adapazari). Rotating about the far edge buries the near edge by
    # B sin(theta) — a whole storey on a deep block — so `sink_frac` of that
    # is kept as burial and the rest given back as lift.
    dig = B * math.sin(r)
    lift = (1.0 - sink_frac) * dig
    z_contact = m["z0"] + zc + lift
    if touched and z_contact > nb["H"] - 0.3:
        lift = max(0.0, nb["H"] - 0.3 - m["z0"] - zc)
        z_contact = m["z0"] + zc + lift
    span = _d_span(m, side, nm) or (-L / 4.0, L / 4.0)

    # 1) WHAT THE CONTACT CRUSHES ON THE LEANER. Before the rigid transform,
    #    so the fragments ride round with the shell — and everything authored
    #    or left static by these breaks has to ride too (the overturn's
    #    floating-roof lesson), which is what the three marks below track.
    k_loose, k_static, k_auth = len(ctx["loose"]), len(ctx["static_extra"]), len(ctx["authored"])
    # THE LEANER'S BAND IS IN ITS OWN PRE-ROTATION FRAME. `zc` is the contact
    # height AFTER the rotation, which is what the (unmoved) neighbour needs;
    # the leaner's modules are still where they were built, and the contact
    # sits at height `h` up its near FACE — the top corner at first touch,
    # sliding down the face as the angle grows.
    h_face = (((B + gap + max(0.0, crush_m) - B * math.cos(r)) / math.sin(r))
              if math.sin(r) > 1e-6 else H)
    z_lo = max(m["z0"], m["z0"] + min(H, h_face)
               - (crush_storeys + (0.8 if crushing else 0.0)) * st_h)
    # and it stops a storey or so ABOVE the contact: when the face has slid
    # down the neighbour's corner, what is above the contact is hanging over
    # that neighbour's roof, not grinding into its wall.
    z_hi = m["z0"] + min(H + 6.0, h_face + (crush_storeys + 0.6) * st_h)
    n_self = _d_crush_band(ctx, m, side, z_lo, z_hi, span=span, mass=mass,
                           loose_frac=0.5 if not crushing else 0.62, tag="lean")
    carried = []
    for e in _d_roof_near(ctx, m, side, mass=mass, reach=4.0):
        box = _roof_box(ctx, e)
        if not box:
            continue
        em = ctx["info"]["masses"][e["mass"]]
        d = rng.uniform(0.9, 2.2)
        rem, strip = _split_strip(ctx, box, em, side, d + 0.8, ctx["mats"]["concrete"])
        stp, lop = _break_split(ctx, strip, 8 + rng.randrange(4),
                                _edge_judge(em, side, d, rng),
                                lambda: ctx["mats"]["concrete"],
                                min_volume_frac=0.0006)
        ctx["loose"] += lop
        ctx["static_extra"] += stp
        carried.append(rem)
        ctx["authored"].append(rem)
        e["dead"] = True
    n_lv = len(m["levels"])
    _disturb_interior(ctx, mass, set(range(max(0, n_lv - 2), n_lv)), side=side)
    _spall(ctx, mass, rate=0.07)

    # 2) THE RIGID BODY. +theta about the LEFT perpendicular of the outward
    #    vector tips the top OUTWARD (the skill's sign rule); the pivot is the
    #    FAR base edge, so the near edge goes down into the soil.
    far = _opposite(side)
    # THE PIVOT IS BUILT FROM THE LOCAL SIDE NORMAL. `_outward` is already
    # yaw-rotated and `_to_world` rotates again, so feeding one to the other
    # puts the pivot of a yawed building at twice its yaw. Invisible on the
    # bench and in the bake (both build at yaw 0) and wrong anywhere a mass
    # carries a yaw. (`r_tilt_sink`, `r_tilt_severe` and `r_overturn` are all
    # still written the other way; they only ever run at yaw 0.)
    fx, fy = _SIDE_NORMAL[far]
    edge = B / 2.0
    px, py = _to_world(m, fx * edge if far in ("E", "W") else 0.0,
                       fy * edge if far in ("S", "N") else 0.0)
    raft = _raft(ctx, m)
    ride = ctx["loose"][k_loose:] + ctx["static_extra"][k_static:] + ctx["authored"][k_auth:]
    seen, paths = set(), []
    for q in _everything(ctx) + [raft] + carried + ride:
        if q and q not in seen:
            seen.add(q)
            paths.append(q)
    M = _rot_about((px, py, m["z0"]), (ax, ay, 0.0), abs(deg)) * _translate(0.0, 0.0, lift)
    _transform_prims(ctx["stage"], paths, M)
    ctx["static_extra"] += [q for q in paths if q not in set(ctx["loose"])]
    for q in ctx["loose"][k_loose:]:
        ctx["velocity"][q] = (ox * rng.uniform(0.2, 1.0), oy * rng.uniform(0.2, 1.0),
                              -rng.uniform(0.2, 1.0))

    # 3) WHAT THE CONTACT DOES TO THE NEIGHBOUR: a band of wall at the contact
    #    line, the cornice above it, glass, and the wedge in the slot.
    nctx = _d_nctx(ctx, nb)
    nside = _d_facing_side(nm, -ox, -oy)
    nspan = _d_span(nm, nside, m) or (-4.0, 4.0)
    nst_h = (nm["levels"][1] - nm["levels"][0]) if len(nm["levels"]) > 1 else 3.4
    n_nb = 0
    if touched:
        if nctx is not None:
            # FIT OUT THE STOREYS THE CONTACT IS ABOUT TO OPEN. A neighbour is
            # placed pristine and hollow; crushing a band of its wall without
            # this leaves a black rectangle behind the hole, which is the
            # dollhouse failure the whole `fit_interior` pass exists to avoid.
            nlv = nm["levels"]
            hurt = {i for i in range(len(nlv))
                    if nlv[i] > z_contact - (band_storeys + 1.2) * nst_h}
            if hurt and not nctx["fit"]["all"]:
                nctx["fit"] = fit_interior(nctx["stage"], nctx["parent"],
                                           nb["info"], nctx["mats"], rng,
                                           storeys=hurt, tag=nctx["tag"])
                nb["fit"] = nctx["fit"]
            n_nb = _d_crush_band(nctx, nm, nside,
                                 z_contact - band_storeys * nst_h,
                                 z_contact + band_storeys * nst_h,
                                 span=nspan, loose_frac=0.4,
                                 out_dir=(-ox, -oy), tag="hit")
            if z_contact > nb["H"] - 1.6 * nst_h:
                n_nb += _d_crush_band(nctx, nm, nside, nb["H"] - 1.2 * nst_h,
                                      nb["H"] + 6.0, span=nspan, loose_frac=0.5,
                                      out_dir=(-ox, -oy), tag="cornice")
            if hurt:
                _disturb_interior(nctx, "main", hurt, side=nside)
                ctx["static_extra"] += [q for q in nctx["fit"]["all"]
                                        if q not in set(ctx["loose"])]
            _d_face_band(nctx, nm, nside, z_contact, nspan, height=(0.5, 1.0),
                         n_seg=(2, 5), tag="hitband")
        else:
            _d_face_band(ctx, nm, nside, z_contact, nspan, height=(0.6, 1.2),
                         n_seg=(3, 6), tag="hit")
            if z_contact > nb["H"] - 2.0:
                _d_face_band(ctx, nm, nside, nb["H"] - 0.5, nspan,
                             height=(0.5, 0.9), n_seg=(2, 5), tag="cornice")
        if gap < 0.6:
            _d_party_ghost(nctx or ctx, nm, nside, nspan, list(nm["levels"][1:]))
    _shard_field(ctx, 0.4, mass=mass, sides=(side,))
    if debris:
        _d_gap_debris(ctx, m, side, span, gap + 0.6,
                      depth_m=rng.uniform(0.5, 1.0),
                      spill_m=rng.uniform(2.0, 3.6))

    # 4) the ground, on the side that went down
    _d_ground_response(ctx, m, side, dig * sink_frac, lift_m=lift)
    ctx["notes"].append(
        "lean_on: {0:.1f} deg toward {1} onto {2} (gap {3:.2f} m, H {4:.0f} vs "
        "{5:.0f} m), contact z {6:.1f} m, {7} module(s) crushed on the leaner, "
        "{8} on the neighbour{9}".format(
            deg, side, nb.get("style") or "neighbour", gap, H, nb["H"], z_contact,
            n_self, n_nb,
            "" if touched else " — NO CONTACT, the neighbour is out of reach"))


def r_collapse_onto(ctx, side=None, mass="main", storeys=2, punch=True,
                    heap=True, spill_frac=0.18):
    """The upper storeys fail toward a LOWER neighbour and land on its roof.

    Kobe 1995, Amatrice 2016: the top of the taller building comes down on the
    next one's roof, punches the diaphragm and buries the top floor. The
    fragments are thrown with the velocity that actually lands them there
    (ballistic, from the drop and the distance), the neighbour's roof loses a
    hole about the impact, the floor under it is fitted out and disturbed, and
    a share goes over the far parapet into the street beyond."""
    rng = ctx["rng"]
    info = ctx["info"]
    m = info["masses"][mass]
    nb = _d_pick(ctx, side, want="lower")
    if nb is None:
        ctx["notes"].append("collapse_onto: no neighbour registered — nothing done")
        return
    side = nb["side"]
    (ox, oy), (ax, ay), B, L = _d_frame(m, side)
    nm, Hn = nb["m"], nb["H"]
    H = m["top"] - m["z0"]
    gap = max(0.05, float(nb["gap"]))
    Dn = nm["D"] if side in ("S", "N") else nm["W"]
    if Hn > H - 2.0:
        if gap < 0.8:
            # A TERRACE. Nothing can land on a neighbour of its own height, and
            # what actually happens when one unit of a row goes is that the
            # unit collapses INTO ITSELF and leaves the next one's party wall
            # standing and legible (Amatrice 2016, L'Aquila 2009). That is a
            # different picture, not a weaker lean.
            return _d_party_collapse(ctx, nb, mass=mass)
        ctx["notes"].append(
            "collapse_onto: the neighbour ({0:.0f} m) is not lower than this "
            "building ({1:.0f} m) — leaning on it instead".format(Hn, H))
        return r_lean_on(ctx, side=side, mass=mass)
    n_lv = len(m["levels"])
    first = max(0, n_lv - int(storeys))
    top_storeys = set(range(first, n_lv))
    z_fail = m["levels"][min(first, n_lv - 1)]
    span = _d_span(m, side, nm) or (-L / 4.0, L / 4.0)
    imp_t = (span[0] + span[1]) / 2.0 + rng.uniform(-0.15, 0.15) * (span[1] - span[0])
    imp_d = gap + Dn * rng.uniform(0.28, 0.5)

    def _throw(z, share):
        """Ballistic velocity for a fragment starting at height z: onto the
        neighbour's roof, or past its far parapet for `spill_frac` of them."""
        far = share < spill_frac
        d = (gap + Dn * rng.uniform(0.95, 1.25)) if far else \
            (gap + Dn * rng.uniform(0.12, 0.72))
        drop = max(0.6, z - (Hn + 0.4))
        t = math.sqrt(2.0 * drop / 9.81)
        return (ox * min(5.4, d / max(0.25, t)), oy * min(5.4, d / max(0.25, t)),
                rng.uniform(-0.4, 0.6))

    k_loose = len(ctx["loose"])
    n_hit = 0
    for e in list(_els(ctx, mass=mass, side=side)):
        if e["role"] not in ("wall", "corner", "parapet", "parapet_corner", "balcony"):
            continue
        if e["storey"] not in top_storeys and e["role"] not in ("parapet", "parapet_corner"):
            continue
        st, lo = _break(ctx["stage"], ctx["parent"], e, ctx["tag"],
                        7 + rng.randrange(5), rng, ctx["nrng"], ctx["mats"],
                        ctx["cache"], info["type"], inner_p=0.4, consume=0.22)
        for q in lo:
            ctx["velocity"][q] = _throw(e["z"] + e["h"] * 0.5, rng.random())
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True
        n_hit += 1
    # the ragged verticals, the broken slab and roof edges and the disturbed
    # interior the same way `out_of_plane` treats a hole it opened
    _ragged_neighbours(ctx, mass, side, top_storeys)
    # only the roof tiles ON the failed side may become `_roof_box` slabs, or
    # the whole roof comes out a pale checkerboard from the air
    with _d_far_roof_hidden(ctx, m, side, mass=mass, reach=4.0):
        # a DEEPER strip than the default (0.8-2.6 m): the storeys under this
        # roof are the ones that just left, so a 1 m nibble at the edge leaves
        # the roof plate cantilevered over the hole with nothing holding it
        _ragged_slabs(ctx, mass, side, top_storeys, depth=(2.0, 4.5))
    _disturb_interior(ctx, mass, top_storeys, side=side)
    _spall(ctx, mass, rate=0.09)
    for q in ctx["loose"][k_loose:]:
        if q not in ctx["velocity"]:
            ctx["velocity"][q] = _throw(z_fail + 1.5, rng.random())

    # 2) the neighbour's roof: a hole where the mass landed, the top floor
    #    under it fitted out and disturbed, its parapet crushed
    nctx = _d_nctx(ctx, nb)
    nside = _d_facing_side(nm, -ox, -oy)
    nst_h = (nm["levels"][1] - nm["levels"][0]) if len(nm["levels"]) > 1 else 3.4
    R = max(2.2, min(0.32 * min(nm["W"], nm["D"]), 0.5 * (span[1] - span[0])))
    if side == "S":
        ilw = _to_world(m, imp_t, -B / 2.0 - imp_d)
    elif side == "N":
        ilw = _to_world(m, imp_t, B / 2.0 + imp_d)
    elif side == "W":
        ilw = _to_world(m, -B / 2.0 - imp_d, imp_t)
    else:
        ilw = _to_world(m, B / 2.0 + imp_d, imp_t)
    ilx, ily = _to_local(nm, ilw[0], ilw[1])
    ilx = max(-nm["W"] / 2.0 + 1.0, min(nm["W"] / 2.0 - 1.0, ilx))
    ily = max(-nm["D"] / 2.0 + 1.0, min(nm["D"] / 2.0 - 1.0, ily))
    n_punch = 0
    if nctx is not None and punch:
        ntop = len(nm["levels"]) - 1
        if not nctx["fit"]["all"]:
            nctx["fit"] = fit_interior(nctx["stage"], nctx["parent"], nb["info"],
                                       nctx["mats"], rng,
                                       storeys={ntop, max(0, ntop - 1)},
                                       tag=nctx["tag"])
            nb["fit"] = nctx["fit"]
        wob = _wander(rng, 0.9)

        def judge(c):
            lx, ly = _to_local(nm, c[0], c[1])
            return math.hypot(lx - ilx, ly - ily) < R + wob(
                math.atan2(ly - ily, lx - ilx))

        urm_n = nb["info"]["type"] == "urm"
        for e in list(_els(nctx, role="roof")):
            # ONE TILE of margin, not three: at R + 3.2 a 5.8 m impact
            # radius fractured most of a 22 x 18 m roof (520 loose pieces) and
            # the whole-roof crack mosaic from the bug catalogue came back
            if math.hypot(e["lx"] - ilx, e["ly"] - ily) > R + 2.6:
                continue
            box = _roof_box(nctx, e, thick=(0.14 if urm_n else ROOF_T))
            if not box:
                continue
            from pxr import UsdShade
            bm = UsdShade.MaterialBindingAPI(
                nctx["stage"].GetPrimAtPath(box)).ComputeBoundMaterial()[0]
            stp, lop = _break_split(nctx, box, 14 + rng.randrange(6), judge,
                                    lambda: (ctx["mats"]["timber"] if urm_n
                                             else ctx["mats"]["concrete"]),
                                    min_volume_frac=0.0005,
                                    mode=("plank" if urm_n else "uniform"),
                                    aspect=((1.5, 4.0) if urm_n else None))
            if bm:
                for q in stp:
                    _bind(nctx["stage"], q, bm)
            ctx["loose"] += lop
            ctx["static_extra"] += stp
            ctx["authored"] += stp
            n_punch += len(lop)
            e["dead"] = True
        pth = nctx["fit"]["slabs"].get(("main", ntop))
        if pth:
            def judge2(c):
                lx, ly = _to_local(nm, c[0], c[1])
                return math.hypot(lx - ilx, ly - ily) < R * 0.8
            stp, lop = _break_split(nctx, pth, 12 + rng.randrange(6), judge2,
                                    lambda: (ctx["mats"]["timber"] if urm_n
                                             else ctx["mats"]["concrete"]),
                                    min_volume_frac=0.0006)
            nctx["fit"]["slabs"][("main", ntop)] = None
            nctx["fit"]["all"] = [q for q in nctx["fit"]["all"] if q != pth] + stp
            ctx["loose"] += lop
            ctx["static_extra"] += stp
        _disturb_interior(nctx, "main", {ntop})
        _d_crush_band(nctx, nm, nside, Hn - 1.3 * nst_h, Hn + 6.0,
                      span=_d_span(nm, nside, m), loose_frac=0.7,
                      out_dir=(-ox, -oy), tag="parapet")
        ctx["static_extra"] += [q for q in nctx["fit"]["all"]
                                if q not in set(ctx["loose"])]
    if gap < 0.6:
        _d_party_ghost(nctx or ctx, nm, nside,
                       _d_span(nm, nside, m) or (-4.0, 4.0), list(nm["levels"][1:]))
    # 3) the rubble that stays: the heap at the impact, the wedge in the gap,
    #    a windrow on the street beyond the far parapet
    if heap:
        # THE HEAP SITS BESIDE THE HOLE, NOT OVER IT. Centred and 1.25 R
        # wide it covered the hole and half the roof, and the punch-through —
        # the whole point of the recipe — could not be seen from the air.
        _d_roof_heap(ctx, nm, Hn - 0.15,
                     ilx + rng.uniform(0.7, 1.1) * R * (1 if ilx < 0 else -1),
                     ily + rng.uniform(-0.4, 0.4) * R, R * 0.85,
                     rng.uniform(0.8, 1.6) + 0.02 * H, tag="onroof")
        _d_gap_debris(ctx, m, side, span, gap + 0.5,
                      depth_m=rng.uniform(0.6, 1.1), spill_m=rng.uniform(2.0, 3.5))
        _heap(ctx, nm, nm["z0"], 0.0, 0.16, fill=False, sides=(_opposite(nside),),
              depth_m=rng.uniform(0.4, 0.9), tag="spill")
    ctx["notes"].append(
        "collapse_onto: {0} module(s) of the top {1} storey(s) thrown onto {2} "
        "({3:.0f} m vs {4:.0f} m, gap {5:.2f} m); roof punched at ({6:.1f}, "
        "{7:.1f}) r {8:.1f} m, {9} roof piece(s) through".format(
            n_hit, storeys, nb.get("style") or "the neighbour", H, Hn, gap,
            ilx, ily, R, n_punch))


def _d_party_collapse(ctx, nb, mass="main"):
    """One unit of a TERRACE comes down and the next one's party wall is left
    standing: the floor lines and joist pockets of the lost unit printed on it,
    a band of torn plaster where the two were tied, and the rubble of the unit
    banked against it. The subject takes its own construction type's total
    collapse (`masonry_collapse` for URM, `pancake` for a frame) — the pile is
    what it always is; the ghost on the neighbour is the read."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    nm = nb["m"]
    side = nb["side"]
    (ox, oy), _lat, _B, _L = _d_frame(m, side)
    nside = _d_facing_side(nm, -ox, -oy)
    nspan = _d_span(nm, nside, m) or (-4.0, 4.0)
    nctx = _d_nctx(ctx, nb)
    btype = ctx["info"]["type"]
    if btype == "urm":
        r_masonry_collapse(ctx, mass=mass)
    else:
        r_pancake(ctx, mass=mass)
    # the party wall: floor lines, ties, and a band of torn render along the
    # line where the lost unit's roof met it
    _d_party_ghost(nctx or ctx, nm, nside, nspan, list(nm["levels"][1:]))
    _d_face_band(nctx or ctx, nm, nside, min(nm["top"], m["top"]) - 0.4, nspan,
                 height=(0.5, 1.0), n_seg=(3, 6), thick=0.14, tag="tearline")
    if nctx is not None:
        _d_crush_band(nctx, nm, nside, min(nm["top"], m["top"]) - 1.0,
                      nm["top"] + 6.0, span=nspan, loose_frac=0.45,
                      p_module=0.6, out_dir=(-ox, -oy), tag="party")
    # and the unit's rubble banked against it
    _heap(ctx, nm, nm["z0"], 0.0, 0.18, fill=False, sides=(nside,),
          depth_m=rng.uniform(0.9, 1.7), tag="banked")
    ctx["notes"].append(
        "party_collapse: this unit came down and left the {0} party wall of "
        "its neighbour standing ({1:.0f} m of it), floor lines and ties "
        "showing".format(nside, nspan[1] - nspan[0]))


def r_pounding(ctx, side=None, mass="main", levels=None, cornice=True,
               debris=True, p_break=0.55, corner_m=1.8):
    """The two buildings HAMMER each other at the shorter one's slab levels.

    Both facing walls take a broken band of spalled render at every storey
    line of the SHORTER building (that is where its diaphragm is), a share of
    the kit modules at those lines lose a course for real, the shorter one's
    cornice is crushed where it meets the taller wall, glass on both faces
    goes, and the material lands in the slot."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    nb = _d_pick(ctx, side)
    if nb is None:
        ctx["notes"].append("pounding: no neighbour registered — nothing done")
        return
    side = nb["side"]
    (ox, oy), _lat, B, L = _d_frame(m, side)
    nm, Hn = nb["m"], nb["H"]
    H = m["top"] - m["z0"]
    gap = max(0.02, float(nb["gap"]))
    span = _d_span(m, side, nm) or (-L / 4.0, L / 4.0)
    nside = _d_facing_side(nm, -ox, -oy)
    nspan = _d_span(nm, nside, m) or (-4.0, 4.0)
    nctx = _d_nctx(ctx, nb)
    self_short = H <= Hn
    short_lv = m["levels"] if self_short else nm["levels"]
    zs = levels if levels is not None else \
        [z for z in list(short_lv[1:]) + [min(H, Hn)] if z > 1.5]
    n_band = n_mod = 0
    for z in zs:
        n_band += len(_d_face_band(ctx, m, side, z, span, tag="pound"))
        n_band += len(_d_face_band(nctx or ctx, nm, nside, z, nspan, tag="pound"))
        for c, mm, sd, sp, od in ((ctx, m, side, span, (ox, oy)),
                                  (nctx, nm, nside, nspan, (-ox, -oy))):
            if c is None or rng.random() > p_break:
                continue
            # a COURSE off the wall under the slab line, on a share of the
            # modules — pounding spalls a band, it does not remove the wall
            n_mod += _d_crush_band(c, mm, sd, z - 0.75, z + 0.25, span=sp,
                                   loose_frac=0.3, parapet=False, out_dir=od,
                                   p_module=0.55, n_pieces=(6, 10),
                                   vel=(0.2, 0.8), tag="pound")
            # and round the corner onto the street face, where it can be seen
            for fsd, fsp in _d_corner_spans(mm, sd, corner_m=corner_m):
                n_band += len(_d_face_band(c, mm, fsd, z, fsp, n_seg=(1, 3),
                                           height=(0.35, 0.7), tag="poundc"))
                if rng.random() < 0.4:
                    n_mod += _d_crush_band(c, mm, fsd, z - 0.75, z + 0.25,
                                           span=fsp, loose_frac=0.3,
                                           parapet=False, n_pieces=(6, 10),
                                           vel=(0.2, 0.8), tag="poundc")
    if cornice:
        zc = min(H, Hn)
        if self_short:
            n_mod += _d_crush_band(ctx, m, side, zc - 1.1, m["top"] + 6.0,
                                   span=span, loose_frac=0.7, out_dir=(ox, oy),
                                   tag="cornice")
            _d_face_band(nctx or ctx, nm, nside, zc, nspan, height=(0.8, 1.4),
                         n_seg=(2, 5), thick=0.15, tag="crushline")
        else:
            if nctx is not None:
                n_mod += _d_crush_band(nctx, nm, nside, zc - 1.1, Hn + 6.0,
                                       span=nspan, loose_frac=0.7,
                                       out_dir=(-ox, -oy), tag="cornice")
            _d_face_band(ctx, m, side, zc, span, height=(0.8, 1.4), n_seg=(2, 5),
                         thick=0.15, tag="crushline")
    _shard_field(ctx, 0.3, mass=mass, sides=(side,))
    if nctx is not None:
        _shard_field(nctx, 0.3, mass="main", sides=(nside,))
    if debris:
        # the slot is dark, so most of what the debris has to do is happen at
        # its two OPEN ENDS, out on the pavement
        _d_gap_debris(ctx, m, side, span, gap + 0.4, depth_m=rng.uniform(0.35, 0.8),
                      spill_m=rng.uniform(2.5, 4.2), density=1.7)
    ctx["notes"].append(
        "pounding: {0} band(s) and {1} crushed module(s) at {2} slab level(s) "
        "across a {3:.2f} m gap toward {4} ({5:.0f} m vs {6:.0f} m)".format(
            n_band, n_mod, len(zs), gap, side, H, Hn))


def d_pound_marks(ctx, a, b, gap=None, normal=None):
    """The AUTHORED half of `r_pounding`, for two buildings that are only
    boxes — the city, where every building is a referenced archetype and no
    kit module can be broken. `a` and `b` are mass dicts (`d_box_mass`).
    Same vocabulary as the bench recipe: bands at the shorter one's slab
    levels on both facing walls, the crushed cornice line where its roof meets
    the taller wall, the wedge in the slot. Returns the band count."""
    rng = ctx["rng"]
    if gap is None or normal is None:
        gap, normal = d_rect_gap((a["cx"], a["cy"], a["yaw"], a["W"], a["D"]),
                                 (b["cx"], b["cy"], b["yaw"], b["W"], b["D"]))
    a_side = _d_facing_side(a, normal[0], normal[1])
    b_side = _d_facing_side(b, -normal[0], -normal[1])
    a_span = _d_span(a, a_side, b) or (-4.0, 4.0)
    b_span = _d_span(b, b_side, a) or (-4.0, 4.0)
    short, tall = ((a, b) if a["top"] <= b["top"] else (b, a))
    t_side, t_span = (b_side, b_span) if short is a else (a_side, a_span)
    n = 0
    for z in [q for q in list(short["levels"][1:]) + [short["top"]] if q > 1.5]:
        n += len(_d_face_band(ctx, a, a_side, z, a_span, tag="pound"))
        n += len(_d_face_band(ctx, b, b_side, z, b_span, tag="pound"))
    _d_face_band(ctx, tall, t_side, short["top"], t_span, height=(0.9, 1.5),
                 n_seg=(2, 5), thick=0.16, tag="crushline")
    _d_gap_debris(ctx, a, a_side, a_span, max(0.35, gap) + 0.4,
                  depth_m=rng.uniform(0.3, 0.7), spill_m=rng.uniform(1.6, 3.0),
                  glass=0.18)
    return n


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
    # BATCH D — pair recipes; they need `ctx["neighbours"]`
    # (`quake_flow.d_set_neighbours`) and do nothing without one.
    "lean_on": r_lean_on,
    "collapse_onto": r_collapse_onto,
    "pounding": r_pounding,
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
    _b_settle_roof_plant(ctx, recipes)   # tanks / AC units follow their roof
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
