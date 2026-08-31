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
                ("storefront_glass", {"grade": 1})],
        "DG2": [("parapet_fall", {"sides": 1, "frac": 0.5}),
                ("facade_scars", {"frac": 0.18}), ("rooftop_fail", {"frac": 0.5}),
                ("signage_fail", {}),
                ("storefront_glass", {"grade": 2})],
        "DG3": [("parapet_fall", {"sides": 2, "frac": 0.8}),
                ("corner_fail", {"storeys": 2}),
                ("roof_hole", {"frac": 0.25}),
                ("facade_scars", {"frac": 0.22}), ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("storefront_glass", {"grade": 3})],
        "DG4": [("out_of_plane", {"sides": 1, "from_storey": 1}),
                ("parapet_fall", {"sides": 3, "frac": 0.9}),
                ("roof_hole", {"frac": 0.35}), ("rooftop_fail", {"frac": 0.8}),
                ("facade_scars", {"frac": 0.18}),
                ("storefront_glass", {"grade": 4})],
        "DG5": [("masonry_collapse", {})],
    },
    "rc": {
        "DG0": [],
        # `facade_scars` is URM-only since round 2 (plaster loss over brick);
        # a cracked RC frame shows as a few dropped infill panels instead.
        "DG1": [("infill_fail", {"storeys": 1, "frac": 0.12}), ("rooftop_fail", {"frac": 0.3}),
                ("storefront_glass", {"grade": 1})],
        "DG2": [("infill_fail", {"storeys": 1, "frac": 0.35}),
                ("rooftop_fail", {"frac": 0.5}),
                ("signage_fail", {}),
                ("balcony_fail", {"frac": 0.3}),
                ("parapet_fall", {"sides": 1, "frac": 0.4}),
                ("storefront_glass", {"grade": 2})],
        "DG3": [("infill_fail", {"storeys": 2, "frac": 0.55}),
                ("balcony_fail", {"frac": 0.6}),
                ("rooftop_fail", {"frac": 0.7}),
                ("signage_fail", {}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("storefront_glass", {"grade": 3})],
        # soft storey OR mid-storey: the bake draws one per style
        # _g_: the shopfront is authored BEFORE the storey moves (the glass
        # recipes read the building's pristine element records) and
        # `glass_follow` replays the storey's own delta onto that art.
        "DG4": [("balcony_fail", {"frac": 0.7}),
                ("storefront_glass", {"grade": 4}),
                ("storey_collapse", {}), ("rooftop_fail", {"frac": 0.8}),
                ("glass_follow", {})],
        "DG5": [("pancake", {})],
    },
    # _g_ (round 3) — the glass tower ladder, rebuilt on
    # `earthquake_research.md` §12 table 8.13. Two recipes now, because the
    # two systems fail by different mechanisms and at drifts an order of
    # magnitude apart: the CURTAIN WALL is hung off the slab edges and takes
    # relative movement (it tolerates 1.4-2 % drift and "as long as the
    # building does not collapse, the curtain wall will not fail"), while the
    # PODIUM SHOPFRONT is in plane with its wall and follows the wall's own
    # drift ("glass breaks in large numbers" at 0.2-0.33 %). That single
    # distinction is the whole tower-vs-street contrast in the field record —
    # one pane gone with the frame pristine up top, every pane gone with the
    # frame racked at street level (FEMA E-74 Figs. 6.3.1.4-1 and -6).
    # The old `glass_fallout(frac=0.06/0.2/0.45/0.7/0.85)` deleted kit modules
    # and its DG5 was outside the record by 30 points; the "out" fractions now
    # live in `G_GRADE` and top out at 40-55 %.
    "rc_glass": {
        "DG0": [],
        "DG1": [("curtain_wall", {"grade": 1}),
                ("storefront_glass", {"grade": 1}),
                ("glass_follow", {})],
        "DG2": [("curtain_wall", {"grade": 2}),
                ("storefront_glass", {"grade": 2}),
                ("parapet_fall", {"sides": 1, "frac": 0.3}),
                ("glass_follow", {})],
        "DG3": [("curtain_wall", {"grade": 3}),
                ("storefront_glass", {"grade": 3}),
                ("parapet_fall", {"sides": 2, "frac": 0.6}),
                ("infill_fail", {"storeys": 1, "frac": 0.4}),
                ("glass_follow", {})],
        # Türkiye 2023, recommendation #3: the glazed retail storey is what
        # MAKES the soft storey. So the podium is de-glazed first and leans
        # second, and the tower above it stays a banded cage.
        "DG4": [("curtain_wall", {"grade": 4}),
                ("storefront_glass", {"grade": 4}),
                ("soft_storey", {"storey": 0, "lean_deg": 2.5}),
                ("glass_follow", {})],
        "DG5": [("curtain_wall", {"grade": 5}),
                ("storefront_glass", {"grade": 5}),
                ("tilt_sink", {"tilt_deg": 9.0, "sink_m": 1.4}),
                ("glass_follow", {})],
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
        # _t_ (round 3): the two things `fracture.solidify` needs and nothing
        # downstream of `_break` can otherwise work out — where the INSIDE of
        # this piece's mass is (a wall is thickened toward it, so the façade
        # never moves) and which way the piece faces out (the façade/core
        # material split). Both are pure functions of `m`, which only this
        # loop has: every placement in the bench lives under one shared
        # `/World/stage/generated`, so a bbox of the prim's parent is the
        # whole scene, not the building.
        e["ref"] = (m["cx"], m["cy"], 0.5 * (m["z0"] + m["top"]))
        ox, oy = _outward(m, e["side"])
        e["out"] = (ox, oy, 0.0)
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
        # LINEAR albedo (damage._pbr): screen grey ~ linear^0.42, so the
        # first values (0.66 / 0.58) rendered 0.84 / 0.79 — the "white
        # debris" of round 1. 0.32 / 0.24 -> ~0.62 / ~0.55 on screen: a
        # light dusty plaster and a mortar bed, not paper.
        "plaster": ((0.32, 0.30, 0.265), 1.0),
        "mortar": ((0.24, 0.225, 0.205), 1.0),
        "rebar": ((0.30, 0.19, 0.13), 0.55),
        "glass": ((0.62, 0.78, 0.80), 0.12),
        "timber": ((0.48, 0.36, 0.24), 0.9),
        # 0.12 linear -> ~0.41 on screen (was 0.40 -> 0.68: the pancake lift shaft
        # and every "dark" chunk rendered pale grey)
        "dark_concrete": ((0.12, 0.115, 0.105), 1.0),
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
    """A thin tube from p0 to p1 — a rebar.

    POINTS ARE LOCAL, CENTRED ON THE TUBE'S OWN MIDPOINT, with an
    `xformOp:translate` carrying that midpoint's world position — the same
    pattern `_box` uses, and for the same reason: `UsdPhysics.RigidBodyAPI`
    treats the PRIM'S OWN transform as the body's origin/pose, and the
    mesh's local points as the shape's offset from it. A prim with NO xform
    ops has that origin at its parent's frame origin; if the points
    themselves already carry the absolute world position — this function's
    behaviour before this fix, and the reason the old docstring said
    "authored in world space with no xform; it is never simulated" — the
    shape sits wherever those points are, which can be a hundred-plus
    metres from the body's actual origin on a scene where the caller's
    building is nowhere near the stage origin.

    THIS WAS THE VENT BUG. `urban_fire.dress_roof_urban` builds its vent
    stacks with this function and — unlike every other `_cyl` caller in
    this codebase (rebar, joist and rafter stubs, all decorative statics
    kept out of `loose`) — hands them to `settle.prepare` via
    `roof_plant` -> `loose`, so PhysX picked up a rigid body whose origin
    (0, 0, 0) sat ~145 m from its own collision geometry (measured:
    `/World/vent_test` authored at world (120, 80, 15), local-to-world
    translation (0, 0, 0), local points centred on (120, 80, 15.5) — offline
    repro, no Kit, `scene_gen/tools/usd_python.sh`). That is a moment arm:
    any angular velocity omega moves the SHAPE (and therefore the translate
    op `settle.bake` reads back) by roughly `omega * arm` per step, which is
    how a `max_speed=6.0` linear-speed cap on the origin still produced a
    205 m 'worst mover' (`vent_b5_14`, uf_fix1, 2026-08-29) — the cap bounds
    the ORIGIN's speed, not the swept distance of a shape that is not where
    the origin is. It is also the likely reason so much of the same
    population tunnelled the floor despite `ground_plane_z` and `ccd=True`
    (already applied both scene- and actor-side, see `prepare` below): CCD's
    sweep and PhysX's depenetration both reason about the body's own pose,
    which here was nowhere near the geometry actually being resolved
    against the ground.

    Centring here fixes it for every current AND future caller, whether or
    not the path is ever added to `loose` — the rendered world position is
    unchanged (translate + local points reproduce the exact same geometry),
    only the split between 'where the object is' and 'what its shape looks
    like' is corrected. As a side effect it also moves these tiny meshes
    off single-precision coordinates of 100+ (where a `Vt.Vec3fArray`
    float32 already loses a millimetre or two) and onto coordinates near
    zero, where the same storage is exact to a fraction of a micron.
    """
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
    c = (p0 + p1) / 2.0
    p0, p1 = p0 - c, p1 - c
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
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    m.CreateExtentAttr([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    UsdGeom.Xformable(m).AddTranslateOp().Set(Gf.Vec3d(*c))
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
# A COLUMN CENTRE MUST BE THIS FAR INSIDE THE STOREY'S MEASURED PLAN before
# it is authored, when a caller passes `fit_interior(footprint=...)`. The
# column grid below is laid on the mass's `W x D` BOUNDING BOX, which is the
# plan only for a cuboid: on a sliced whole-asset building with a setback or
# an L plan, the corners of that grid land OUTSIDE the façade and poke
# through it (fire_dtc3 review, 2026-08-30, `gac_SM_Building_02_F5c_s193`
# `fit_g6/col_main_10_2_1`). 0.35 m clears the column's own half-diagonal
# (`COLUMN_W * sqrt(2) / 2` = 0.32 m) whatever the mass yaw, so the whole
# box lands inside the hull rather than just its centre.
FIT_FOOTPRINT_M = 0.35


def _inside_inset(poly, x, y, inset):
    """Is (x, y) at least `inset` metres inside the CONVEX polygon `poly`?

    `poly` is world-XY, either winding. Convexity is the caller's to keep:
    the only producer today is `gac_fire._storey_footprints`, which returns a
    convex hull per storey (itself already inset 0.35 m off the measured
    vertices) and hands it through `fire["footprints"]`. A degenerate polygon
    answers True — no footprint measured means no clamping, which is the
    behaviour every caller had before the kwarg existed.
    """
    n = len(poly)
    if n < 3:
        return True
    a2 = 0.0
    for i in range(n):
        j = (i + 1) % n
        a2 += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]
    s = 1.0 if a2 >= 0.0 else -1.0            # +1 when the winding is CCW
    for i in range(n):
        j = (i + 1) % n
        dx = poly[j][0] - poly[i][0]
        dy = poly[j][1] - poly[i][1]
        el = math.hypot(dx, dy)
        if el < 1e-9:
            continue
        # signed distance from the edge INTO the polygon
        if s * (dx * (y - poly[i][1]) - dy * (x - poly[i][0])) / el < inset:
            return False
    return True

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
                 columns=True, partitions=True, tag="b", footprint=None):
    """Author slabs (+ columns, partitions) for every mass of one building.

    Returns {"slabs": {(mass, storey): path}, "columns": {(mass, storey):
    [paths]}, "partitions": [...], "all": [...]}. Slab `storey` i is the
    floor OF storey i (its top at `levels[i]`); storey 0 has no slab (that
    is the ground). The roof is the kit's own roof piece.

    `storeys` limits which storeys get a fit-out (None = all): a recipe that
    only opens the top two floors of a twelve-storey block has no reason to
    pay for the other ten, and nothing can see them.

    `footprint` is an optional {storey: convex world-XY polygon} — the plan
    the COLUMN grid is clamped to (`FIT_FOOTPRINT_M`), for a caller that has
    measured one (`urban_fire.burn_building` passes
    `ctx["fire"]["footprints"]` on the sliced path). Default None clamps
    nothing and authors exactly the grid this has always authored; a storey
    missing from the mapping is likewise unclamped. No rng is drawn in the
    column loop, so a clamped run leaves every later draw untouched.
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
                # the storey's measured plan, when the caller has one: the
                # grid is a bounding-box grid and its corners fall outside an
                # irregular footprint (see `FIT_FOOTPRINT_M`)
                fp = (footprint or {}).get(i)
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
                        if fp and not _inside_inset(fp, wx, wy,
                                                    FIT_FOOTPRINT_M):
                            continue        # outside the storey's own plan
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
# ---------------------------------------------------------------------------
# ROUND 3 — WALL THICKNESS (agent T)
# ---------------------------------------------------------------------------
#
# The user, on the round-2 city: "I don't believe that we can't fracture
# zero-wall-thickness buildings." The premise turned out to be wrong in an
# instructive way. Measured (`tools/_t_shell_probe.py` + a ray probe on
# `bld_commercial_DG0.usd`): every kit mesh is OPEN — that much was true — but
# a wall module is DOUBLE-SIDED, 27.8 m2 of face pointing out and 28.0 m2
# pointing in, with a measured wall thickness of 0.14-0.68 m and the plain
# storey band exactly 0.70 m. The material was always there. What is missing is
# the RIM: 134 of 4433 edges on the main wall component belong to one face
# only, so `is_watertight` is False, and `slice_plane(cap=True)` then has to
# cap an OPEN polyline — which VTK's stripper fans into 2-3 m triangular
# sheets. Those sheets are the "foil plates", and they are also, literally, the
# "looks very TRIANGULAR" of the round-3 review: not a break pattern, a failed
# cap.
#
# `fracture.solidify` closes the module before it is cut, and these are the
# thicknesses it is given.
#
# THE NUMBERS. A Boston/NYC URM bearing wall is three or four wythes on the
# lower storeys at ~110 mm a wythe (_plans/earthquake_research.md, damage row
# 12: "outer wythe ~110 mm thick, peels in 2-10 m2 sheets"), so 0.33-0.44 m;
# a brownstone front is heavier still; a parapet is one or two wythes above the
# roof line; an RC frame's infill is a single block leaf at ~0.20 m; a floor
# slab is 150-250 mm (research §3.9, "horizontal shelves 150-250 mm thick");
# and glass is 6-12 mm, which is why it has its own entry — solidifying a pane
# to wall thickness would turn a curtain wall into a brick wall.
T_SOLID_M = {
    "urm": {"wall": 0.38, "corner": 0.40, "parapet": 0.25,
            "parapet_corner": 0.25, "balcony": 0.18, "roof": 0.20,
            "portico": 0.30, "pediment": 0.25, "ornament": 0.15,
            None: 0.30},
    "rc": {"wall": 0.20, "corner": 0.22, "parapet": 0.18,
           "parapet_corner": 0.18, "balcony": 0.16, "roof": 0.22,
           "portico": 0.22, "pediment": 0.18, "ornament": 0.12,
           None: 0.20},
}
# A glazed frame is the same frame; only the PANE is thin, and the pane is a
# prim of its own in this kit (agent G owns what happens to it).
T_SOLID_M["rc_glass"] = dict(T_SOLID_M["rc"], wall=0.16)
T_GLASS_M = 0.010            # 6-12 mm; a pane stays a plate
T_BROWNSTONE_M = 0.40        # a brownstone front is four wythes plus the stone
# Turn the whole round-3 thickness path off with EQ_SOLID=0 — the round-2 look
# and the round-2 timings come straight back, which is the comparison to make
# when something looks wrong.
T_SOLID_ON = _os.environ.get("EQ_SOLID", "1").strip() not in ("0", "false", "no")
# Solidifying multiplies nothing on this art (the back sheet and the 42
# sub-0.03 m2 trinkets in a wall module are dropped, so face counts come out
# 0.98-1.25x), but a SOLID cell does not need the plate-era refinement: the
# reason for 0.32 m teeth was that a 1 m plate cell read as a sheet of card.
# This scales `edge_cell_m` up for solidified pieces; 1.0 keeps round 2's.
T_EDGE_CELL_SCALE = float(_os.environ.get("EQ_SOLID_EDGE", "1.0"))
# The façade/core face split costs one GeomSubset prim and one material bind
# per fragment. EQ_SOLID_CORE=0 skips it (every fragment then takes one
# material over all its faces, as in round 2) — the knob to try first when a
# bench run is slower than it should be.
T_CORE_ON = _os.environ.get("EQ_SOLID_CORE", "1").strip() not in ("0", "false", "no")
# CHUNKS NEED FEWER CELLS THAN PLATES, and this is the lever for it. The
# round-2 seed counts were chosen for a SHEET, where a Voronoi cell has almost
# no bbox volume and `fracture_mesh`'s `min_volume_frac` throws most of them
# away; the same count on a solid keeps far more of them, which is where the
# +30% prim count of a solid DG5 column comes from. Seeds are multiplied by
# this on any solidified piece. Default 1.0 = round-2 counts (measured within
# the 1.5x prim budget anyway); `EQ_SOLID_N=0.85` buys ~15% back and, if
# anything, reads BETTER for masonry — a URM heap is whole bricks and brick
# clusters, not gravel.
T_SOLID_N_SCALE = float(_os.environ.get("EQ_SOLID_N", "1.0"))


def _t_is_glass(stage, path, texture=None):
    """Is this module a glazing panel? Name-based, because that is all the kit
    offers — the material is `glass`/`Glass_*` and the texture, if any, has it
    in the URL."""
    if texture and "glass" in str(texture).lower():
        return True
    try:
        from pxr import UsdShade
        pr = stage.GetPrimAtPath(path)
        if pr and pr.IsValid():
            bm = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
            if bm and "glass" in bm.GetPath().name.lower():
                return True
    except Exception:
        pass
    return "glass" in str(path).lower()


def _t_thickness(btype, role, style=None, stage=None, path=None, texture=None):
    """Wall thickness in metres for one piece. None = do not solidify."""
    if not T_SOLID_ON:
        return None
    if stage is not None and path and _t_is_glass(stage, path, texture):
        return T_GLASS_M
    tbl = T_SOLID_M.get(btype) or T_SOLID_M["urm"]
    t = tbl.get(role, tbl[None])
    if style and "brownstone" in str(style) and role in ("wall", "corner"):
        t = T_BROWNSTONE_M
    return t


def _t_el(ctx, path):
    """The element record for a prim path, or None."""
    for e in ctx["info"]["elements"]:
        if e["p"].get("prim_path") == path:
            return e
    return None


def _t_ref(ctx, e=None):
    """A point INSIDE the building — solidify thickens every piece toward it,
    which is what keeps the façade where it was."""
    if e is not None and e.get("ref"):
        return e["ref"]
    m = ctx["info"]["masses"].get((e or {}).get("mass") or "main") \
        or ctx["info"]["masses"]["main"]
    return (m["cx"], m["cy"], 0.5 * (m["z0"] + m["top"]))


def _t_core_mat(stage, parent, mats, btype, rng):
    """The inside of a wall: brick and mortar for masonry, dark concrete for a
    frame. Drawn per fragment so a heap is not one flat colour."""
    r = rng.random()
    if btype == "urm":
        # BRICK-HEAVY. The first solid bench (T_sol2_urm close-up) drew brick
        # 55% and flat mortar/plaster the rest, and the flat greys won the
        # read: the cut faces are the LARGEST faces on a chunk and they catch
        # the light, so a 45% share of untextured grey turns a brick chunk
        # into a concrete block. `plaster` is out of this mix entirely — it is
        # a finish, not a core, and at 0.62 on screen it is the palest thing
        # in the palette.
        key = "brick" if r < 0.70 else ("mortar" if r < 0.90 else "dark_concrete")
    else:
        # NOT `concrete`. That is the Worn_Pavement map, and on a 0.5 m chunk
        # its moss and joint grid read as a mossy boulder (visible all over
        # T_sol2_rc's close-up) — the same objection `_a_dustify` already
        # records against it. Mortar-grey with dark concrete for depth.
        # A SHARE OF BRICK EVEN IN A FRAME. `mortar`, `dark_concrete` and
        # `plaster_dusty` are all FLAT colours — a heap made only of those
        # reads as grey plastic (T_sol3_rc close-up), because nothing in it
        # has any surface at all. The infill panels of an RC frame of this
        # period ARE masonry (research §3.2, "RC frame with unreinforced
        # infill"), so a fifth of the core faces take the brick map and give
        # the pile something to catch the light on.
        key = ("mortar" if r < 0.35 else "dark_concrete" if r < 0.63
               else "brick" if r < 0.85 else "plaster_dusty")
    return mats.get(key) or mats.get("plaster")


def _t_core_bind(stage, parent, paths, out, mats, btype, rng, solid_m=None):
    """Give every fragment a BRICK CORE.

    A solidified fragment carries the module's own cladding on the faces that
    were the façade and pipeline-invented geometry everywhere else — the cut
    faces, the back, and the reveal round an opening. One prim takes one
    material, so the invented faces go into a `materialBind` GeomSubset and
    that subset gets the core. Without this a chunk is a brick-textured box on
    all six sides, and the broken edge of a window shows brickwork running the
    wrong way across the reveal instead of a mortar core."""
    from . import fracture
    if not T_CORE_ON or not out or not solid_m or solid_m <= T_GLASS_M:
        return 0
    n = 0
    for pth in paths:
        sub = fracture.face_subset(stage, pth, out, cos=0.30)
        if sub is None:
            continue
        _bind(stage, str(sub.GetPath()),
              _t_core_mat(stage, parent, mats, btype, rng))
        n += 1
    return n


def _chunk_material(stage, parent, cache, texture, mats, btype, rng,
                    inner_p):
    if rng.random() < inner_p or not texture:
        if btype == "urm":
            r = rng.random()
            key = "brick" if r < 0.45 else ("mortar" if r < 0.75 else "plaster")
        else:
            r = rng.random()
            # NOT `concrete` (round 3, agent T). That key is the Worn_Pavement
            # megascans map; on a 0.5 m chunk its moss and joint grid read as
            # a MOSSY BOULDER — visible all over the office close-ups
            # (T_sol3_rc, T_sol4_rc) — which is the same objection
            # `_a_dustify` already records against it ("the green-and-white
            # striped mattress"). `_a_dustify` only rescues ~70 % of a
            # collapse's fragments from it and nothing at all on a piece that
            # is still standing on the building.
            key = ("dark_concrete" if r < 0.5
                   else ("mortar" if r < 0.8 else "plaster"))
        return mats.get(key)
    return _clad_material(stage, parent, cache, texture)


def _break(stage, parent, el, tag, n, rng, nrng, mats, cache, btype,
           inner_p=0.35, partial=None, mode="uniform", rough=0.012,
           min_volume_frac=0.002, consume=0.0, max_piece_m=None,
           solid_m=None, style=None, core=True, **kw):
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
    # ROUND 3: close the module into a solid before cutting it, and thicken it
    # TOWARD THE BUILDING so the façade does not move (`el["ref"]`, set in
    # `describe`). `solid_m=0` from a caller disables it for that piece.
    if solid_m is None:
        solid_m = _t_thickness(btype, el.get("role"), style=style,
                               stage=stage, path=path, texture=tex)
    ref = el.get("ref")
    # _p_: the brick cluster is clamped against the MATERIAL thickness, not
    # the module's bbox (a kit wall with a projecting bay measures 1.07 m
    # across and is 0.38 m of brick). Only `_break` knows which is which.
    kw.setdefault("blocky_m", solid_m)
    if solid_m and T_SOLID_N_SCALE != 1.0:
        n = max(3, int(round(n * T_SOLID_N_SCALE)))
    if partial is not None:
        st, lo = fracture.fracture_partial(
            stage, path, out, n_pieces=n, rng=nrng, cut_frac=partial,
            mode=mode, rough=rough, consume=consume * 0.5,
            min_volume_frac=min_volume_frac, max_piece_m=max_piece_m,
            solid_m=solid_m, solid_ref=ref, **kw)          # _p_ pass-through
    else:
        st, lo = [], fracture.fracture_prim(
            stage, path, out, n_pieces=n, rng=nrng, mode=mode, rough=rough,
            verbose=False, consume=consume, consume_pool=1.6,
            min_volume_frac=min_volume_frac, max_piece_m=max_piece_m,
            solid_m=solid_m, solid_ref=ref, **kw)          # _p_ pass-through
    # WITH A CORE SUBSET THE PRIM BINDING IS THE FAÇADE, FULL STOP. `inner_p`
    # exists because a fragment could only carry one material and some of them
    # had to show the inside; now the inside has faces of its own, so drawing
    # an interior material for the whole prim would paint it over the cladding
    # as well and undo the split.
    # ...but NOT zero. `_a_dustify` greys a collapse's fragments by swapping
    # the PRIM binding (plaster/concrete/mortar -> dusty), so forcing every
    # prim to the cladding took the dust coat off the whole heap and left it
    # a clean brick quarry. A fifth still draw an interior material for the
    # prim; with the core subset on top that reads as a dust-coated chunk
    # with masonry showing where it broke.
    ip = (0.18 if (core and solid_m and T_CORE_ON and tex)
          else inner_p)
    for pth in list(st) + list(lo):
        _bind(stage, pth, _chunk_material(stage, parent, cache, tex, mats,
                                          btype, rng, ip))
    if core:
        _t_core_bind(stage, parent, list(st) + list(lo), el.get("out"), mats,
                     btype, rng, solid_m=solid_m)
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
# ...but a strip cut off a `_box` with `_split_strip` butts against a FLAT
# remainder box, and the remainder cannot be roughened (it has eight corner
# points; displacing them warps the box). A 2.8 cm scar on the strip's cells
# and none on the remainder is a tone step along the cut, and from nadir that
# step is a straight line right across the roof — the complaint again, in a
# mild form. 8 mm is invisible at 20 m and still breaks the specular at 2 m.
ROUGH_STRIP_M = 0.008


def _break_split(ctx, path, n, judge, mat_fn, rough=ROUGH_M,
                 min_volume_frac=0.0015, mode="uniform", aspect=None,
                 static_mat=None, refine=True, edge_cell_m=EDGE_CELL_M,
                 chew=(CHEW_OUT, CHEW_IN), crack_frac=CRACK_FRAC,
                 refine_max=REFINE_MAX, gap_static_m=GAP_STATIC_M,
                 max_loose_m=None, edge_consume=0.5, solid_m=None, core=True,
                 **kw):
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
    # ROUND 3 (agent T): the piece is closed into a solid of its real wall
    # thickness FIRST, so the two-scale split cuts a wall and not a sheet, and
    # the surviving stub shows masonry depth on its broken edge instead of a
    # paper rim. The role and the interior reference come from the element
    # record; a piece the recipes authored themselves (a `_box` slab) is
    # already watertight and `solidify` returns it untouched.
    _e = _t_el(ctx, path)
    if solid_m is None:
        solid_m = _t_thickness(ctx["info"]["type"],
                               (_e or {}).get("role"),
                               style=ctx["info"].get("style"), stage=stage,
                               path=path)
    # _p_ ROUND 3: NOTHING DEFAULTS TO `uniform` ANY MORE.
    # `uniform` is a 3-D Voronoi of random seeds, and in a member thinner than
    # the cell pitch that can only produce thin plates with acute corners —
    # the "TRIANGULAR" look, and 60 % of the flaky fragments in the first
    # round-3 bench came through callers that simply never passed a mode
    # (`_ragged_neighbours`, `_ragged_slabs`, `_a_slab_rim`, `_droop_strip`,
    # `r_parapet_fall`, ...). A caller that still wants it says so explicitly.
    # A kit ELEMENT breaks as its construction type does; anything else here
    # is an authored `_box` slab or strip, which is a plate and rafts.
    if mode == "uniform":
        _pk = (_p_frac_kw(ctx) if (_e is not None and _e.get("role") not in
                                   (None, "roof", "slab"))
               else _p_slab_kw(ctx))
        mode = _pk.pop("mode")
        if rough is None or abs(float(rough) - ROUGH_M) < 1e-9:
            rough = _pk.pop("rough")
        else:
            _pk.pop("rough", None)
        for _k, _v in _pk.items():
            kw.setdefault(_k, _v)
    kw.setdefault("blocky_m", solid_m)                          # _p_
    if solid_m:
        edge_cell_m = float(edge_cell_m) * T_EDGE_CELL_SCALE
        if T_SOLID_N_SCALE != 1.0:
            n = max(3, int(round(n * T_SOLID_N_SCALE)))
    st_m, lo_m = fracture.fracture_split(
        mesh, n, judge, ctx["nrng"], mode=mode, aspect=aspect, **kw,
        # _p_: floor 0.008 -> 0.002. A mortar joint is FLAT; 8 mm of noise on
        # an 0.068 m course is an eighth of a brick and turns the units into
        # pebbles. Nothing that passed >= 0.008 before is affected.
        rough_m=max(0.002, min(0.06, float(rough))),
        solid_m=solid_m, solid_ref=_t_ref(ctx, _e),
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
    if core and _e is not None:
        _t_core_bind(stage, ctx["parent"], st + lo, _e.get("out"),
                     ctx["mats"], ctx["info"]["type"], ctx["rng"],
                     solid_m=solid_m)
    src = stage.GetPrimAtPath(path)
    if src and src.IsValid():
        src.SetActive(False)
    # A LATER RECIPE CAN BREAK WHAT AN EARLIER ONE MADE STATIC (roof_hole
    # running on the slab corner_fail authored, for one), and a deactivated
    # prim left in `static_extra` becomes a collider request PhysX cannot
    # satisfy. Drop it here rather than at every call site.
    if path in ctx["static_extra"]:
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q != path]
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


# ---------------------------------------------------------------------------
# _p_  ROUND 3 — THE BREAK PATTERN *IS* THE PLANE OF WEAKNESS
# ---------------------------------------------------------------------------
#
# The user, on the two-city review: "it looks very TRIANGULAR now, like
# something was added to the edge of the rectangular breakage. Figure out the
# pattern in which it would break naturally and mimic that."
#
# That is an exact description of what round 2 did: a straight cut with teeth
# appended (`_a_torn` / `_a_stepped` wobbles added to a plane) over a 3-D
# Voronoi of random seeds, which in a wall thinner than the cell pitch can
# only make thin plates with acute corners. Real break lines are not decorated
# afterwards — they ARE the planes of weakness. Measured demolition debris is
# BLOCKY (Flakiness Index 7-13 %, Elongation 13-24 %, 55-74 % equidimensional,
# "almost no blades"); the only legitimate triangles are metres across.
#
#   MASONRY   the wall is a running-bond LATTICE (`fracture._p_brick_seeds`)
#             and the break line is a STAIRCASE QUANTISED TO IT: risers are k
#             whole courses (k 1-4), runs are m whole half-stretchers (m 1-8).
#             `_a_stepped` drew both CONTINUOUSLY — an 0.137 m riser is not a
#             number a brick wall can make.
#   CONCRETE  the member is cut into PRISMS through its full thickness
#             (`fracture._p_prism_seeds`), the line tears (`_a_torn` is right
#             for concrete and is called unchanged), and what flakes is the
#             COVER, 19-38 mm, and nothing else.
#
# Round 2's `_a_*` helpers are frozen: they are CALLED here, never edited.
# Sources: _plans/eq_round3_R.md §1-§2, _plans/earthquake_research.md §11.

# (stretcher, course, wythe) pitch in metres. US modular brick 194 x 92 x 57 mm
# actual with a 9.5 mm (3/8 in) joint -> three courses are 8 in exactly;
# heritage (NZ/UK/AU) brick 230 x 110 x 76 mm. Research §11.
P_BOND = {"modular": (0.203, 0.0677, 0.092),
          "heritage": (0.240, 0.086, 0.110)}
# Which stock a style is built of. The stone-fronted and older terrace styles
# take the larger heritage unit, which also gives the library two visibly
# different course heights instead of one.
P_STYLE_BOND = {"brownstone": "heritage", "brownstone_row": "heritage",
                "walkup": "heritage", "dw_terrace": "heritage",
                "church": "heritage"}
# R §1.5, verbatim: "Do NOT chew or roughen the faces. A mortar joint is flat.
# ROUGH_M = 0.028 is half a joint width and turns bricks into pebbles — use
# <= 0.003 m only to break the specular."
P_ROUGH_M = 0.003
P_ROUGH_RC_M = 0.006     # a concrete fracture surface is aggregate-rough at
                         # 5-10 mm; still a quarter of round 2's 0.028.
P_BRICK_KEEP = 0.62      # site dropout -> 2-8 brick clusters (R: 0.55-0.70)
P_PRISM_KEEP = 0.80
P_RUN_K = (1, 8)         # horizontal run = m x half stretcher (0.10-0.81 m)
P_RISE_K = (1, 4)        # riser          = k x course        (0.068-0.27 m)
P_VRUN_K = (2, 6)        # a line running UP the wall: run in z, in courses
P_VRISE_K = (1, 4)       #                             riser sideways, in halves
# Plastic-hinge length Lp = 0.08 L + 0.022 d_b f_y ~= 0.42 m for a 3 m column
# with 20 mm bars at f_y 400 (R §2) — the height of the spalled zone and of the
# bar lantern, and the only place a column loses its cover.
P_LP_M = 0.42
# ACI 318 §20.6 cover, measured to the tie. "Spalling is typically within the
# cover", so the spall shell IS the cover and nothing thicker ever flakes.
P_COVER_M = {"wall": 0.038, "corner": 0.038, "slab": 0.019, "roof": 0.019,
             "parapet": 0.038, "parapet_corner": 0.038, "balcony": 0.019,
             None: 0.038}
# Sliver rejection (R §2.4), passed through to `fracture._p_sliver_seeds`:
# b/a >= 0.6 on every piece, c/b >= 0.5 only under 1.2 m — see that function
# for why the large end is exempt (rafts are legitimately plate-like).
P_SLIVER = (0.6, 0.5, 1.2)
# Sliver rejection costs ONE extra cell pass per fracture. EQ_SLIVER=0 turns
# it off everywhere — the first knob to reach for if a bench run is over the
# fracture-time budget, since a `brick` or `prism` lattice is structurally
# blocky and the pass mostly removes the wafers the module's own rim clipped.
P_SLIVER_ON = _os.environ.get("EQ_SLIVER", "1").strip() not in ("0", "false", "no")


def _p_pitch(ctx=None, style=None):
    """(stretcher, course, wythe) pitch for this building's brick."""
    st = str(style if style is not None
             else ((ctx or {}).get("info", {}) or {}).get("style", ""))
    for k, bond in P_STYLE_BOND.items():
        if k in st:
            return P_BOND[bond]
    return P_BOND["modular"]


def _p_staircase(rng, amp, span_m, run_pitch, rise_pitch,
                 run_k=P_RUN_K, rise_k=P_RISE_K):
    """A staircase f(t), t in METRES along the line, QUANTISED TO THE BOND.

    Every value of f is an exact integer multiple of `rise_pitch` and every
    step happens at an exact integer multiple of `run_pitch`, because that is
    the only kind of line a bonded wall can tear along: the bed joints are the
    weak planes and the perpends are staggered half a unit, so the tear runs
    along a bed joint for a whole number of half-stretchers, drops a whole
    number of courses at a perpend, and repeats. (FEMA 306 Ch. 7: weak mortar
    and sound units -> "cracks stair-step through head and bed joints".)

    `_a_stepped` has the right RANGES and the wrong arithmetic — it draws run
    and riser from continuous uniforms, so the line lands between courses and
    the wall stops reading as bonded. This is that function with the two
    uniforms replaced by integers, and the reflect-don't-clamp behaviour kept
    (a clamped walk rides the rail and comes out as a straight line with
    notches)."""
    rise_pitch = max(1e-4, float(rise_pitch))
    run_pitch = max(1e-4, float(run_pitch))
    lim = max(1, int(math.floor(max(float(amp), rise_pitch) / rise_pitch)))
    ts, ys = [0.0], [0.0]
    t, j = 0.0, 0
    guard = 0
    while t < float(span_m) + run_pitch * run_k[1] and guard < 4000:
        guard += 1
        t += rng.randint(int(run_k[0]), int(run_k[1])) * run_pitch
        step = rng.randint(int(rise_k[0]), int(rise_k[1])) * (
            1 if rng.random() < 0.5 else -1)
        nj = j + step
        if nj > lim or nj < -lim:
            nj = j - step                     # turn round, do not ride the rail
            if nj > lim or nj < -lim:
                nj = max(-lim, min(lim, j))
        j = int(nj)
        ts.append(t)
        ys.append(j * rise_pitch)

    def f(tm):
        i = bisect.bisect_right(ts, float(tm)) - 1
        return ys[max(0, min(len(ys) - 1, i))]
    return f


def _p_wobble(rng, amp, span_m, btype="rc", vertical=False, pitch=None):
    """The break-line offset for a construction type — the `_p_` replacement
    for `_a_wobble`. Masonry gets the bond-quantised staircase; concrete keeps
    `_a_torn`, which is the right model for it (the crack follows the
    aggregate and the cover spalls ahead of it, so the line is smooth at the
    metre scale and rough at the decimetre scale)."""
    if btype != "urm":
        return _a_torn(rng, amp, span_m)
    p_l, p_c, _p_t = pitch or P_BOND["modular"]
    if vertical:
        # the line runs UP the wall: the runs are in COURSES and the risers
        # step sideways by half stretchers
        return _p_staircase(rng, amp, span_m, p_c, p_l * 0.5,
                            run_k=P_VRUN_K, rise_k=P_VRISE_K)
    return _p_staircase(rng, amp, span_m, p_l * 0.5, p_c,
                        run_k=P_RUN_K, rise_k=P_RISE_K)


def _p_zline_judge(m, side, z0, rng, btype="rc", amp=0.5, loose_above=True,
                   pitch=None):
    """`_a_zline_judge` on the bond: a HORIZONTAL break at height `z0` that
    staircases along the courses (urm) or tears (rc)."""
    wob = _p_wobble(rng, amp, _a_side_span(m, side), btype, vertical=False,
                    pitch=pitch)

    def judge(c):
        zb = float(z0) + wob(_a_side_t(m, side, c))
        return (c[2] > zb) if loose_above else (c[2] < zb)
    return judge


def _p_edge_judge(m, side, depth_m, rng, btype=None, pitch=None):
    """`_edge_judge` on the bond: within a staircasing/tearing depth of the
    mass's `side` wall line, measured inward."""
    W, D = m["W"], m["D"]
    wob = _p_wobble(rng, depth_m * 0.55, _a_side_span(m, side), btype or "rc",
                    vertical=False, pitch=pitch)

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


def _p_toward_judge(m, side, frac, rng, btype=None, z0=0.0, span_z=None,
                    pitch=None):
    """`_toward_judge` on the bond — the line runs UP the wall, so masonry
    tooths vertically (runs of a few courses, risers of half a brick)."""
    W, D = m["W"], m["D"]
    reach = (D if side in ("S", "N") else W)
    if span_z is None:
        span_z = max(6.0, float(m.get("top", 12.0)) - float(m.get("z0", 0.0)))
    wob = _p_wobble(rng, 0.45, max(3.0, float(span_z)), btype or "rc",
                    vertical=True, pitch=pitch)

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


def _p_vcrack_judge(m, side, t0, rng, btype="urm", amp=0.45, pitch=None,
                    loose_hi=True):
    """A VERTICAL crack `t0` metres along the `side` wall — the crack at a
    wall return or an opening reveal that bounds a macroblock. FEMA 306: at a
    corner the crack is vertical and the corner is "punched outward"."""
    z0 = float(m.get("z0", 0.0))
    span_z = max(3.0, float(m.get("top", 12.0)) - z0)
    wob = _p_wobble(rng, amp, span_z, btype, vertical=True, pitch=pitch)

    def judge(c):
        t = _a_side_t(m, side, c)
        line = float(t0) + wob(c[2] - z0)
        return (t > line) if loose_hi else (t < line)
    return judge


def _p_frac_kw(ctx, leaves=1, keep=None, rough=None):
    """The fracture kwargs for this construction type, in one place.

    Everything that touches masonry passes `mode="brick"` with the bond pitch;
    everything that touches concrete passes `mode="prism"`. Both get sliver
    rejection and near-zero surface roughening. Splat this into `_break` /
    `_break_split` — both grew a `**kw` pass-through for it."""
    btype = ctx["info"]["type"]
    tag = "{0}|{1}".format(ctx.get("tag", ""), btype)   # for EQ_DUMP_FRAGS
    if btype == "urm":
        return dict(mode="brick", brick=_p_pitch(ctx),
                    keep_frac=(P_BRICK_KEEP if keep is None else float(keep)),
                    leaves=int(leaves), dump_tag=tag,
                    sliver=(P_SLIVER if P_SLIVER_ON else None),
                    rough=(P_ROUGH_M if rough is None else float(rough)))
    return dict(mode="prism",
                keep_frac=(P_PRISM_KEEP if keep is None else float(keep)),
                dump_tag=tag, sliver=(P_SLIVER if P_SLIVER_ON else None),
                rough=(P_ROUGH_RC_M if rough is None else float(rough)))


def _p_wall_point(m, side, t):
    """Local (lx, ly) on the mass's `side` wall line, `t` metres along it."""
    W, D = m["W"], m["D"]
    if side == "S":
        return (t - W / 2.0, -D / 2.0)
    if side == "N":
        return (t - W / 2.0, D / 2.0)
    if side == "W":
        return (-W / 2.0, t - D / 2.0)
    return (W / 2.0, t - D / 2.0)


def _p_el_t(m, side, e):
    """Distance in metres along the `side` wall to a kit piece's LEFT end."""
    return ((e["lx"] + m["W"] / 2.0) if side in ("S", "N")
            else (e["ly"] + m["D"] / 2.0))


def _p_monolith(ctx, e, tag="mb"):
    """One kit module as ONE solid rigid body — a MACROBLOCK, not rubble.

    A URM wall failing out of plane is a RIGID-BODY MECHANISM, not a fracture:
    it rocks on three horizontal cracks (top, bottom, mid-height) and
    overturns, and 318 Canterbury URM buildings gave "cracking that delineates
    relatively undamaged masonry macroblocks" with out-of-plane failure at
    65 % of all observations. So a standing damaged URM wall is 1-4 big
    internally-intact blocks — it becomes a field of bricks only AFTER a block
    falls and lands. Dicing the whole wall into cells, which is what round 2
    did, is the one thing the mechanism says not to do.

    Returns the new prim path (the module solidified, written as one mesh, the
    source deactivated), or None."""
    from . import damage, fracture
    stage = ctx["stage"]
    path = e["p"].get("prim_path")
    if not path:
        return None
    mesh = fracture.prim_to_mesh(stage, path)
    if mesh is None or not len(mesh.faces):
        return None
    solid_m = _t_thickness(ctx["info"]["type"], e.get("role"),
                           style=ctx["info"].get("style"), stage=stage,
                           path=path)
    if solid_m:
        try:
            mesh = fracture.solidify(mesh, solid_m, ref=_t_ref(ctx, e))
        except Exception as exc:
            print("[quake] _p_monolith solidify failed: {0}".format(exc))
    out = "{0}/mb_{1}_{2}_{3}".format(ctx["parent"], ctx["tag"], tag,
                                      path.rsplit("/", 1)[-1])
    try:
        fracture._write_mesh(stage, out, mesh)
    except Exception as exc:
        print("[quake] _p_monolith write failed: {0}".format(exc))
        return None
    tex = damage.bound_texture(stage, path)
    mat = (_clad_material(stage, ctx["parent"], ctx["cache"], tex) if tex
           else ctx["mats"].get("brick"))
    _bind(stage, out, mat)
    _t_core_bind(stage, ctx["parent"], [out], e.get("out"), ctx["mats"],
                 ctx["info"]["type"], ctx["rng"], solid_m=solid_m)
    src = stage.GetPrimAtPath(path)
    if src and src.IsValid():
        src.SetActive(False)
    e["dead"] = True
    return out


def _p_macroblocks(ctx, mass, side, from_storey=1, panels_out=None):
    """Out-of-plane failure as the mechanism it is: cut the MACROBLOCKS first,
    dice only the edges and the part that hits the ground.

    Three horizontal cracks (top of wall, bottom, mid-height) plus vertical
    cracks at the corners and openings partition the wall into 1-4 blocks.
    Each block then draws its own fate — overturned into the street, still
    leaning out, or cracked and standing — which is exactly the spread the
    Canterbury survey reports (rocking cantilevers that went, and rocking
    cantilevers that did not). Only the modules the cracks RUN THROUGH are
    fractured, and those are fractured on the bond, so the edge that survives
    is a staircase of whole bricks.

    `panels_out` (round 4, v2 only — `None` in v1, exactly today's
    behaviour): a list to receive `(prim_path, size)` for every WHOLE
    monolith module in an OVERTURNED block, instead of that module riding
    the old ad hoc rotate-and-drop-into-`ctx["loose"]` treatment. The caller
    (`r_out_of_plane`) hands these to `_rubble`'s `panels=` — the fan's own
    large-element system poses and half-buries them, which is the round-4
    design table's "1-2 macroblock panels" row. The block's SHED fractured
    debris (the modules that broke on the way down) still gets the rigid
    overturn-and-drop treatment and lands in `ctx["loose"]` either way; only
    the whole pieces change hands.

    Returns a note string for `ctx["notes"]`."""
    from . import damage
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    btype = ctx["info"]["type"]
    pitch = _p_pitch(ctx)
    span = _a_side_span(m, side)
    lv = m["levels"]
    z_lo = lv[from_storey] if from_storey < len(lv) else m["z0"]
    ox, oy = _outward(m, side)

    # 1-2 interior vertical cracks -> 2-3 blocks across; one mid-height crack
    # in the taller half of the peel when there is room for one.
    n_v = 1 + (1 if rng.random() < 0.55 else 0)
    tv = sorted(rng.uniform(0.24, 0.76) * span for _ in range(n_v))
    n_up = len(lv) - from_storey
    z_mid = (lv[from_storey + max(1, n_up // 2)]
             if (n_up >= 3 and rng.random() < 0.6) else None)

    blocks, edge_els, feet = {}, [], {}
    for e in list(_els(ctx, mass=mass, side=side)):
        if e["role"] not in ("wall", "corner", "parapet", "parapet_corner",
                             "balcony"):
            continue
        if (e["storey"] < from_storey
                and e["role"] not in ("parapet", "parapet_corner")):
            continue
        t0 = _p_el_t(m, side, e)
        t1 = t0 + max(1.0, float(m["module"]))
        cross_v = any(t0 - 0.25 < q < t1 + 0.25 for q in tv)
        at_foot = abs(e["z"] - z_lo) < 0.6 and e["role"] == "wall"
        cross_h = (z_mid is not None
                   and e["z"] - 0.05 < z_mid < e["z"] + e["h"] + 0.05)
        key = (sum(1 for q in tv if t0 >= q),
               0 if (z_mid is None or e["z"] < z_mid) else 1)
        if cross_v or cross_h:
            edge_els.append((e, bool(cross_v and not cross_h)))
            continue
        # THE FOOT ROW BELONGS TO ITS BLOCK, NOT TO THE EDGE SET, AND THIS IS
        # A CORRECTNESS BUG NOT A TASTE ONE. Diced as an edge, a foot module
        # loses its top 8-42 %, so a block that then stays STANDING or LEANING
        # is supported by nothing and hangs 2-3 m in the air — agent T saw
        # exactly that ("a brick panel floating in the sky",
        # T_fin2_urm/1_commercial_DG5_street.png). A block only gets its
        # support cut away if it is going over.
        (feet if at_foot else blocks).setdefault(key, []).append(e)

    n_over = n_lean = n_stay = n_shed = n_cell = 0
    kw0 = kw = _p_frac_kw(ctx)

    def _dice(e, lo_frac, hi_frac, amp_frac=0.24):
        """One module cut by a bond staircase at `lo..hi` of its own height:
        above the line comes away, below it survives. Returns nothing; it
        books the pieces itself."""
        pth = e["p"].get("prim_path")
        if not pth:
            return
        tx = damage.bound_texture(ctx["stage"], pth)
        z0 = e["z"] + e["h"] * rng.uniform(lo_frac, hi_frac)
        st_d, lo_d = _break_split(
            ctx, pth, 10 + rng.randrange(4),
            _p_zline_judge(m, side, z0, rng, btype=btype,
                           amp=e["h"] * amp_frac, loose_above=True,
                           pitch=pitch),
            _mat_fn(ctx, tx, 0.35),
            static_mat=(_clad_material(ctx["stage"], ctx["parent"],
                                       ctx["cache"], tx) if tx else None),
            **kw)
        for q in lo_d:
            v = 0.4 + rng.uniform(0.0, 0.8)
            ctx["velocity"][q] = (ox * v, oy * v, 0.05 * v)
        _a_dustify(ctx, lo_d)
        ctx["loose"] += lo_d
        ctx["static_extra"] += st_d
        e["dead"] = True

    # DRAW EVERY FATE FIRST, THEN MAKE SURE THE PEEL ACTUALLY PEELS. This is
    # the DG4 recipe for a wall that came off; a draw in which no block goes
    # over is not a rarer outcome, it is the wrong recipe. (Benches P_urm4/5/6
    # all came out 0 overturned / 1 leaning / 2 standing on seed 4 — the same
    # three draws every time, because the seed is fixed.)
    order = sorted(set(list(blocks) + list(feet)))
    fate = {}
    for key in order:
        fate[key] = rng.random()
    if order and not any(fate[k] < 0.62 for k in order):
        fate[max(order, key=lambda k: len(blocks.get(k, ())))] = 0.0

    for key in order:
        els = blocks.get(key, [])
        r = fate[key]
        foot_els = feet.get(key, [])
        # a foot under a block that is going over is cut low (the peel's own
        # ragged base); a foot under one that stays up keeps nearly its whole
        # height, so it still carries the block
        for fe in foot_els:
            if r < 0.62 or not els:
                _dice(fe, 0.08, 0.42, amp_frac=0.26)
            else:
                _dice(fe, 0.72, 0.92, amp_frac=0.11)
            n_cell += 1
        if not els:
            continue
        t_lo = min(_p_el_t(m, side, e) for e in els)
        t_hi = max(_p_el_t(m, side, e) + float(m["module"]) for e in els)
        zb = min(e["z"] for e in els)
        px, py = _to_world(m, *_p_wall_point(m, side, 0.5 * (t_lo + t_hi)))
        # A BLOCK THAT GOES OVER DOES NOT LAND WHOLE. R, on the mechanism:
        # a wall "becomes a field of bricks only AFTER a block falls and
        # lands". So an overturning block sheds a share of its modules into
        # brick clusters — they travel with the block and arrive as rubble —
        # while the rest arrive as a recognisable section of wall lying in the
        # street, which is what the Christchurch photographs show. Round 3's
        # first bench kept every module whole and put two clean pale
        # rectangles on the pavement, which is the "rectangular breakage"
        # complaint back again in a bigger size.
        # `out_of_plane` is the recipe for a wall that PEELED OFF, so
        # overturning is the majority fate; leaning and standing are the
        # rocking cantilevers that did not complete. At 0.45 a three-block
        # wall came out 0 overturned / 1 leaning / 2 standing (P_urm4) and
        # nothing reached the street.
        shed = rng.uniform(0.3, 0.6) if r < 0.62 else 0.0
        whole, whole_sizes, brk = [], {}, []
        for e in els:
            if shed and rng.random() < shed:
                st_e, lo_e = _break(
                    ctx["stage"], ctx["parent"], e, ctx["tag"],
                    12 + rng.randrange(6), rng, ctx["nrng"], ctx["mats"],
                    ctx["cache"], btype, inner_p=0.4, consume=0.25, **kw0)
                brk += list(st_e) + list(lo_e)
                e["dead"] = True
                n_shed += 1
            else:
                q = _p_monolith(ctx, e, tag="{0}{1}".format(*key))
                if q:
                    whole.append(q)
                    whole_sizes[q] = _module_size(m, e)
        paths = whole + brk
        if not paths:
            continue
        if r < 0.62:
            if panels_out is not None and whole:
                # v2: the WHOLE monolith modules become RUBBLE PANELS —
                # `_rubble`'s large-element system poses and half-buries
                # them on the fan, instead of the ad hoc overturn rotation
                # below. Only the SHED fractured debris still gets that
                # rigid drop-and-settle treatment.
                for q in whole:
                    panels_out.append((q, whole_sizes[q]))
                if brk:
                    _transform_prims(ctx["stage"], brk,
                                     _rot_about((px, py, zb), (-oy, ox, 0.0),
                                                rng.uniform(76.0, 98.0)))
                    drop = max(0.0, zb - m["z0"] - rng.uniform(0.4, 1.1))
                    if drop > 0.05:
                        _transform_prims(ctx["stage"], brk,
                                        _translate(0.0, 0.0, -drop))
                    for q in brk:
                        ctx["velocity"][q] = (ox * rng.uniform(0.15, 0.5),
                                              oy * rng.uniform(0.15, 0.5), -0.2)
                    _a_dustify(ctx, brk, p=0.7)
                    ctx["loose"] += brk
                n_over += 1
                continue
            # OVERTURNED (v1, or nothing whole to hand off this key).
            # Rotating +theta about the LEFT perpendicular of the outward
            # run swings the wall's up-vector toward outward, i.e. the top
            # leads and the block ends flat in the street — which is what
            # the Christchurch photographs show, whole wall sections lying
            # on the footpath with their courses still legible.
            _transform_prims(ctx["stage"], paths,
                             _rot_about((px, py, zb), (-oy, ox, 0.0),
                                        rng.uniform(76.0, 98.0)))
            # A block whose foot was 8 m up is now lying flat 8 m in the air.
            # Set it down just over the windrow and let the settle seat it,
            # rather than dropping a 6 m rigid body into the pile.
            drop = max(0.0, zb - m["z0"] - rng.uniform(0.4, 1.1))
            if drop > 0.05:
                _transform_prims(ctx["stage"], paths,
                                 _translate(0.0, 0.0, -drop))
            for q in paths:
                ctx["velocity"][q] = (ox * rng.uniform(0.15, 0.5),
                                      oy * rng.uniform(0.15, 0.5), -0.2)
            _a_dustify(ctx, brk, p=0.7)
            ctx["loose"] += paths
            n_over += 1
        elif r < 0.86:
            # STILL LEANING — the rocking cantilever that did not go over.
            _transform_prims(ctx["stage"], paths,
                             _rot_about((px, py, zb), (-oy, ox, 0.0),
                                        rng.uniform(9.0, 26.0)))
            ctx["static_extra"] += paths
            n_lean += 1
        else:
            ctx["static_extra"] += paths        # cracked out, still standing
            n_stay += 1

    for e, is_v in edge_els:
        path = e["p"].get("prim_path")
        if not path:
            continue
        tex = damage.bound_texture(ctx["stage"], path)
        if is_v:
            q = min(tv, key=lambda z: abs(
                z - (_p_el_t(m, side, e) + float(m["module"]) * 0.5)))
            judge = _p_vcrack_judge(m, side, q, rng, btype=btype, amp=0.42,
                                    pitch=pitch, loose_hi=(rng.random() < 0.5))
        else:
            z_ref = (z_mid if (z_mid is not None
                               and e["z"] < z_mid < e["z"] + e["h"]) else z_lo)
            z0 = min(max(z_ref + e["h"] * rng.uniform(0.08, 0.42),
                         e["z"] + 0.2), e["z"] + e["h"] - 0.2)
            judge = _p_zline_judge(m, side, z0, rng, btype=btype,
                                   amp=e["h"] * 0.26, loose_above=True,
                                   pitch=pitch)
        st, lo = _break_split(
            ctx, path, 10 + rng.randrange(4), judge, _mat_fn(ctx, tex, 0.35),
            static_mat=(_clad_material(ctx["stage"], ctx["parent"],
                                       ctx["cache"], tex) if tex else None),
            **kw)
        for pth in lo:
            v = 0.4 + rng.uniform(0.0, 0.8)
            ctx["velocity"][pth] = (ox * v, oy * v, 0.05 * v)
        _a_dustify(ctx, lo)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True
        n_cell += 1
    return ("macroblocks {0}: {1} overturned, {2} leaning, {3} standing, "
            "{4} modules diced on the cracks, {5} shed into brick on the way "
            "down".format(side, n_over, n_lean, n_stay, n_cell, n_shed))


def _p_lintels(ctx, m, n=None, base=None, tag="lintel"):
    """The only large pieces in a masonry pile: LINTELS, QUOINS, sills, arch
    heads and cornice runs, which bypass the fracture entirely.

    R §1.7 and the acceptance test both make this explicit — "URM heap:
    largest piece = a lintel/quoin monolith, not a wall shard". Christchurch:
    the fallen debris "had collapsed into individual bricks rather than as
    larger chunks of masonry debris", so a pile whose biggest piece is a 1.5 m
    plate of wall is wrong twice over: too big for the bricks and too thin for
    the stones. These are authored, not cut, because that is what they are —
    single dressed stones that were never bonded into the field."""
    rng = ctx["rng"]
    n = int(n if n is not None else rng.randrange(3, 7))
    base = m["z0"] if base is None else float(base)
    mats = ctx["mats"]
    made = []
    for i in range(n):
        r = rng.random()
        if r < 0.55:                 # a LINTEL / cornice run: long and heavy
            sx = rng.uniform(1.1, 2.2)
            sy = rng.uniform(0.20, 0.32)
            sz = rng.uniform(0.18, 0.30)
        elif r < 0.85:               # a QUOIN / sill block: a dressed cube
            sx = rng.uniform(0.36, 0.58)
            sy = rng.uniform(0.28, 0.42)
            sz = rng.uniform(0.24, 0.38)
        else:                        # an arch head / coping stone
            sx = rng.uniform(0.7, 1.1)
            sy = rng.uniform(0.30, 0.45)
            sz = rng.uniform(0.22, 0.34)
        lx = rng.uniform(-0.44, 0.44) * m["W"]
        ly = rng.uniform(-0.44, 0.44) * m["D"]
        wx, wy = _to_world(m, lx, ly)
        path = "{0}/{1}_{2}_{3:02d}".format(ctx["parent"], ctx["tag"], tag, i)
        _box(ctx["stage"], path, wx, wy,
             base + sz * 0.5 + rng.uniform(0.05, 0.6),
             sx, sy, sz, yaw_deg=rng.uniform(0.0, 360.0),
             mat=_a_mat(ctx, "brick_dusty" if rng.random() < 0.5 else "dust"))
        made.append(path)
    ctx["loose"] += made
    return made


def _p_slab_kw(ctx, keep=None, rough=None):
    """A floor slab, a roof deck or a strip cut off one is NOT masonry however
    masonry the building is: it is a PLATE, and Fardis's slab rule is that the
    crack "extends into the slab at right angles to the beam, sometimes
    joining up with a similar crack from a parallel beam" — bay-sized
    rectangular RAFTS bounded by beam lines. So a slab is always `prism`:
    2-D seeds extruded through the full thickness, never a 3-D seed inside a
    0.2 m plate."""
    return dict(mode="prism",
                keep_frac=(P_PRISM_KEEP if keep is None else float(keep)),
                sliver=(P_SLIVER if P_SLIVER_ON else None),
                dump_tag="{0}|slab".format(ctx.get("tag", "")),
                rough=(P_ROUGH_RC_M if rough is None else float(rough)))


def _p_ragged_courses(ctx, mass, storey, sides=None, band=(0.28, 0.85),
                      above=True, below=True, n_seeds=None, p=0.92,
                      near=None):
    """`_a_ragged_courses` with the round-3 break pattern.

    Same job — destroy the kit's horizontal module seam at a removed storey,
    which is the top and the bottom of the "unnatural rectangular part broken
    off" — but the band edge is now a bond-quantised staircase on masonry
    (`_p_zline_judge`) and the cells are brick clusters rather than a 3-D
    Voronoi of a thin panel. `_a_ragged_courses` itself is frozen (round 2);
    the recipes this round owns call this instead.

    Returns the same dict of paths keyed `above_static` / `above_loose` /
    `below_static` / `below_loose`."""
    from . import damage
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    btype = ctx["info"]["type"]
    pitch = _p_pitch(ctx)
    kw = _p_frac_kw(ctx)
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
            z0 = z_ref + b if is_above else z_ref - b
            judge = _p_zline_judge(m, e["side"], z0, rng, btype=btype,
                                   amp=b * 0.8, loose_above=not is_above,
                                   pitch=pitch)
            keep_mat = (_clad_material(ctx["stage"], ctx["parent"],
                                       ctx["cache"], tex) if tex else None)
            # `refine_max=4` and 8 coarse cells: a band on a wall module is a
            # small feature and there are two whole storeys of them.
            s, l = _break_split(ctx, path, n_seeds or (7 + rng.randrange(3)),
                                judge, _mat_fn(ctx, tex, 0.4),
                                static_mat=keep_mat, refine_max=4,
                                edge_cell_m=0.38, **kw)
            if not s:
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
    if path in ctx["static_extra"]:
        ctx["static_extra"] = [q for q in ctx["static_extra"] if q != path]
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
                              static_mat=keep_m if keep_m else None,
                              rough=ROUGH_STRIP_M)
        fit["slabs"][(mt, i)] = rem
        fit["all"] = [q for q in fit["all"] if q != pth] + [rem] + st
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        _a_edge_bars(ctx, st, btype, m, side)
    # the roof over the failed side: the strip along it, no more
    W_, D_ = m["W"], m["D"]
    for box, _blx, _bly in _a_roofify(ctx, mass):
        # ONLY THE SLABS THAT REACH THE FAILED WALL. On a multi-tile roof
        # `_split_strip` cuts relative to the SLAB's own edge, so a slab in the
        # middle of the roof would lose a strip in the middle of the roof.
        try:
            _, _, _, _sx, _sy, _, _ = _box_dims(ctx["stage"], box)
        except Exception:
            continue
        half = (_sy if side in ("S", "N") else _sx) * 0.5
        near = ((_bly + D_ / 2.0) if side == "S" else
                (D_ / 2.0 - _bly) if side == "N" else
                (_blx + W_ / 2.0) if side == "W" else (W_ / 2.0 - _blx))
        if near - half > 1.0:
            continue
        d = rng.uniform(*depth)
        rem, strip = _split_strip(ctx, box, m, side, d + 2.2, mats["concrete"])
        bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(strip)).ComputeBoundMaterial()[0]
        st, lo = _break_split(ctx, strip, 12 + rng.randrange(5),
                              _edge_judge(m, side, d, rng, btype=btype),
                              lambda: (_a_mat(ctx, "timber_dusty")
                                       if (btype == "urm" and rng.random() < 0.5)
                                       else _a_mat(ctx, "concrete_dusty")),
                              min_volume_frac=0.0006, rough=ROUGH_STRIP_M,
                              static_mat=bm if bm else None, max_loose_m=3.2)
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        ctx["authored"].append(rem)


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
            refine_max=6, max_loose_m=2.6, rough=ROUGH_STRIP_M)
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
    corner there — so the standing sides are not showroom-clean.

    ROUND 2: `fracture_partial` (the single-scale path) cut this at a smooth
    wobble over 8-12 whole cells of a 4 m module, so a spall came out as a
    RECTANGULAR notch with one pale flap hanging in it — a machined hole, which
    is the complaint this whole round is about, on the one recipe that touches
    otherwise undamaged elevations and therefore appears on nearly every
    building in the city. It goes through `_break_split` with `_a_zline_judge`
    now: the line steps along the courses on masonry and tears on concrete, the
    cells the line crosses are refined, and the surviving wall keeps ITS OWN
    cladding (`static_mat`) so only the loss shows."""
    from . import damage
    rng = ctx["rng"]
    m = ctx["info"]["masses"][mass]
    btype = ctx["info"]["type"]
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
        # the top 8-26 % of the module goes: a course or two, not a storey
        z0 = e["z"] + e["h"] * rng.uniform(0.74, 0.92)
        # _p_ ROUND 3, AND THE TWO CONSTRUCTION TYPES DIVERGE HERE.
        #   urm: a spall is the top COURSES coming off, so the line staircases
        #        on the bond and the pieces are brick clusters.
        #   rc:  "spalling is typically within the cover" (ACI 318 §20.6
        #        cover 38 mm to the tie), so what comes off is a COVER FLAKE —
        #        `solid_m` is the cover, not the wall, and the flake is
        #        19-38 mm thick and 0.1-0.5 m across. Round 2 broke the full
        #        0.20 m member here and shed slabs of wall for a "spall".
        _kw = dict(_p_frac_kw(ctx))
        if btype != "urm":
            # the member keeps its real thickness; only its outer COVER is
            # sliced off and broken (`fracture._p_cover_shell`). Sliver
            # rejection is turned OFF for it: a cover flake fails c/b by
            # construction, and it is the one piece that is right to.
            _kw["cover_m"] = P_COVER_M.get(e.get("role"), P_COVER_M[None])
            _kw["cover_axis"] = e.get("out")
            _kw["sliver"] = None
        st, lo = _break_split(
            ctx, path, 9 + rng.randrange(4),
            _p_zline_judge(m, e["side"], z0, rng, btype=btype,
                           amp=e["h"] * 0.11, loose_above=True,
                           pitch=_p_pitch(ctx)),
            _mat_fn(ctx, tex, 0.55),
            static_mat=_clad_material(ctx["stage"], ctx["parent"],
                                      ctx["cache"], tex) if tex else None,
            refine_max=5, edge_cell_m=0.30, **_kw)
        if not st:
            # the whole module came away — that is a hole, not a spall
            continue
        # WHAT COMES OFF A WALL IS DUST-COLOURED, and what is left behind in
        # the recess is the wall's own inner face, not a bright flap.
        _a_dustify(ctx, lo, p=0.85)
        ox, oy = _outward(m, e["side"])
        for q in lo:
            v = rng.uniform(0.3, 0.9)
            ctx["velocity"][q] = (ox * v, oy * v, 0.0)
        ctx["loose"] += lo
        ctx["static_extra"] += st
        e["dead"] = True


def _break_box(stage, path, n, rng, nrng, mat, inner_mat=None, inner_p=0.5,
               mode="uniform", aspect=None, consume=0.0, consume_pool=1.6,
               max_piece_m=None, **kw):
    """Fracture an authored box (slab / column) into chunks. Returns paths.

    `consume_pool` is exposed (round 2): 1.6 lets middling pieces into the
    consumption draw, which is right for thinning a wall's shells, but a
    collapsed timber DECK has to lose its BIGGEST boards specifically or the
    heap comes out a lumber yard. Pass 1.0-1.1 for that."""
    from . import fracture
    out = path + "_brk"
    made = fracture.fracture_prim(stage, path, out, n_pieces=n, rng=nrng,
                                  mode=mode, aspect=aspect, rough=0.012,
                                  dump_tag="box|" + path.rsplit("/", 1)[-1],
                                  verbose=False, consume=consume,
                                  consume_pool=consume_pool,
                                  max_piece_m=max_piece_m,
                                  min_volume_frac=0.0008, **kw)  # _p_
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
            _dust_loose(ctx, lo)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
        # The windrow on the sidewalk — 1-4 m into the street, 0.5-1.5 m
        # deep (Christchurch parapet reconnaissance), under the pieces that
        # went (a piece is pivoted at its near end, so extend by one module).
        t0, t1 = min(ts) - 0.02, max(ts) + max(4.0, float(m["module"])) / L
        # v2: a fixed 0.45 m windrow depth (round-4 spec) with `elem_h_m` set
        # to the fallen parapet/course's own height so the planner's reach
        # heuristic uses the research's "reach ~= the fallen element's own
        # height" rule instead of a fraction of the whole building; v1 keeps
        # drawing `rng.uniform(0.5, 1.1)` exactly as it always did.
        depth = 0.45 if _RUBBLE_MODE == "v2" else rng.uniform(0.5, 1.1)
        elem_h_m = max((e["h"] for e in chosen), default=None)
        _rubble(ctx, m, "windrow", sides=(side,), depth_m=depth,
                along=(t0, t1), elem_h_m=elem_h_m,
                tag="parapet_{0}".format(side))


def r_glass_loss(ctx, frac=0.3):
    """DEPRECATED (agent G, round 3) — kept so old baked ladders still run.

    It only ever scattered a shard field on all four sides at a random depth,
    which is neither banded nor tied to any opening. `r_storefront_glass` is
    the replacement; `frac` is mapped onto its grade so a call site that has
    not been updated still gets the right severity."""
    grade = 1 + bisect.bisect_right([0.12, 0.28, 0.48, 0.65], float(frac))
    r_storefront_glass(ctx, grade=grade)


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
    """DEPRECATED (agent G, round 3) — a shim onto `r_curtain_wall`.

    The old body `_deactivate`d whole `Skyscraper*` kit MODULES with
    probability `frac x height x side`, which is what the user saw and
    objected to: "you just made some glass disappear, with random glass panes
    hanging in a few areas". Deleting a module deletes the mullion cage, the
    transoms and the 1 m ledge with it, and Bernoulli sampling cannot make a
    band. Both are answered in `r_curtain_wall`; `frac` maps onto its grade
    so any call site or baked ladder that still names `glass_fallout` gets
    the corrected behaviour at the severity it asked for."""
    grade = 1 + bisect.bisect_right([0.12, 0.32, 0.55, 0.78], float(frac))
    r_curtain_wall(ctx, grade=grade, mass=mass)


# ---------------------------------------------------------------------------
# ROUND 3, agent G — GLASS.  "What happens to glass buildings in earthquakes?"
#
# The round-2 answer (`r_glass_fallout`) deleted whole kit MODULES at random,
# which is wrong in the four ways `earthquake_research.md` §12 measures:
#
#  1. Only the GLASS leaves. The aluminium mullion cage, the transoms, the
#     glazing pockets, the gaskets and the spandrels stay: frame failure was
#     ONE façade system in 371 at Christchurch, and heavy cladding was 94 %
#     operational-or-IO [§12 8.7]. A de-glazed curtain wall is a CAGE, not a
#     hole, and the elevation stays STRIPED.
#  2. Loss is a contiguous BAND on the storeys that racked — Mexico City 1985,
#     90 Durango: "loss of glazing apart from top three floors (except one
#     panel)"; Wenchuan Qingchuan: the top storey only, from a roof-diaphragm
#     push; Tōhoku NILIM 647: 26 panes on ONE elevation and none opposite.
#     An i.i.d. scatter is right only for ACCELERATION-driven loss on a stiff
#     building (Aleppo 2023) [§12 8.6].
#  3. There is a cracked-but-retained state, and FEMA E-74 names four
#     (unbroken / cracked-and-retained / shattered-but-precarious / fallen
#     out). It belongs to ANNEALED and LAMINATED glass only: fully tempered
#     cracks and falls out at the SAME drift, 6/6 specimens — an empty frame
#     and a pile of dice on the footpath [§12 8.3, 8.4].
#  4. Cracks are CORNER-ROOTED. "Crack propagation starts along the edges near
#     diagonally opposed corners of glass panels" [§12 8.5]. A centre-rooted
#     spiderweb is an impact signature and reads as a thrown rock.
#
# And the quantities were far too high: at Christchurch's MMI IX "nearly half
# of all glazed lightweight claddings had glazing damage", 64 % of all façade
# systems were still operational, and NO modern curtain-wall tower has
# collapsed in any of the ten events reviewed. 85 % pane loss (the old DG5)
# is outside the record; the ceiling is ~55 % [§12 8.10, 8.13].
#
# HOW THE KIT IS BUILT, measured rather than assumed
# (scene_gen/tools/_g_glass_probe.py / _g_glass_rects.py / _g_uv.py):
#
# * `SM_MBuilding05_SkyscraperFacade_B` is a 5 x 1 x 3 m BOX shell: two glass
#   quads (outer at y = -1, inner at y = 0) and four `SkyscraperLedge` faces —
#   top, bottom, and the two vertical returns. So the module is already a 1 m
#   deep glazing pocket with a ledge at every storey line: exactly the cage and
#   the "stripe" the research says must survive. Nothing has to be built for
#   it; it only has to be LEFT ALONE.
# * The mullion grid is PAINTED, not modelled: material
#   `MI_MBuilding05_SkyscraperWindows`, a 128 x 128 tile holding TWO panes with
#   the mullion at 70/128 of its width and a transom along the top and bottom
#   edge. The mesh UVs put u_tex = 0.3326 per metre on every skyscraper face
#   (1.663 over a 5 m facade, 1.995 over the 6 m corner run) and exactly ONE
#   tile over the 3 m storey.
#   -> the painted pane grid is mullions at (k + 0.0)/0.3326 and
#      (k + 0.5469)/0.3326 metres from the piece's u-origin: panes 1.645 m and
#      1.362 m wide alternating, one row per storey, ~2.89 m tall.
#   -> AUTHOR ON THAT GRID and inset the opening by half a painted mullion, and
#      the painted cage survives around every hole in perfect register for
#      free. No mullion geometry is needed for the cage to read, and nothing
#      can drift out of alignment because the grid comes from the same UVs the
#      renderer samples.
# * Podium and shopfront glazing IS separate geometry (`M_MBuilding05_Glass`,
#   `M_MBuilding04_Glass`, `MI_Glass_Building_A`, opacity 0.698), measured into
#   `_G_SHOP_FACES`. Families 02 (office) and 01 (apartment) have NO glazing
#   geometry at all — their windows are painted into the façade map — so the
#   storefront recipe can only put the glass on the pavement there. Stated,
#   not hidden.
#
# ★ PRIM BUDGET (reviewer constraint: a scene must not take hours to load).
#   Everything below accumulates into MERGED meshes: one opening mesh, one
#   mullion-bar mesh, one crack mesh and one crazed mesh per (mass, side,
#   storey), one debris mesh per (mass, side), one gasket mesh per building.
#   A DG5 tower is ~200 lost panes and lands at well under 100 authored prims
#   — the same order as round 2's shard field, which was 68 loose boxes for a
#   result nobody wanted. Materials are five per building, in the building's
#   own `QuakeLooks` scope, never per pane.
# ---------------------------------------------------------------------------

# The painted curtain-wall pane grid, measured off the UVs (see above).
G_TILE_PER_M = 0.3326          # texture tiles per metre along the wall
G_TILE_MULLION = 0.5469        # 70/128: where the mullion sits inside a tile
G_PANE_INSET = 0.075           # m: keep the PAINTED mullion visible round a hole
G_MIN_PANE_M = 0.55            # m: merge the module-seam sliver into its neighbour
G_MULLION_W = 0.105            # m: an authored bar, for relief on the cage
G_MULLION_PROUD = 0.075        # m: how far it stands off the glass line
G_TRANSOM_H = 0.185            # m: the floor line. The elevation stays STRIPED.
G_SLAB_H = 0.24                # m: the slab edge you see through an empty frame
G_BENT_MM = (0.010, 0.040)     # mullions bent 10-40 mm at the band corners [8.7]
G_MAX_DEBRIS = 1500            # debris QUADS per building (they share meshes,
#                                so this costs vertices, not prims — but the
#                                vertices are what set the archetype USD size,
#                                and the round-2 tower DG5 archetype is 786 kB)
G_HEAP_PANES = 38              # panes that get a full granular heap; beyond
#                                that the belt is already continuous and more
#                                clumps only cost bytes
G_MAX_BARS = 900               # mullion bars per building (they share meshes)

# Glazed FACES of a kit piece, in `_piece_frame` / `_b_face_pt` coordinates:
#   (plane, u0, u1, v0, v1, out, recess, tex_at_u0, tex_per_m)
# `plane` "front" is the piece's own frame; "left" is the corner piece's second
# elevation (the x = xmin plane), built by `_g_left_frame`.
# `out` is where the glass plane is (0 = 2 cm proud of the piece face);
# `recess` is how far back the authored opening is pushed. v0/v1 are
# piece-local heights: the transom lines measured off the texture sit at
# z = 0.02 and z = 2.91 of every 3 m module.
#
# ★ THE RECESS MUST BE SHALLOW. The first bench pushed the opening 0.86 m back,
# into the module's own 1 m pocket, to get depth for free. Seen from a 40 deg
# oblique that quad PARALLAXES by 0.86 x tan(40) = 0.7 m — half a pane — so
# every hole slid off the painted mullion it was meant to sit behind, its
# silhouette was clipped by the box's ledges, and the band came out as black
# blobs with slanted, tapering edges (G_tow2/5_tower_g_glass5_sw.png). That is
# the "triangular" failure the user objected to, arriving by a route nobody had
# looked at. Depth now comes from the mullion bars standing 55 mm PROUD and
# from the module's own ledges; the dark quad sits ~0.1 m back, where it cannot
# drift off the grid.
_G_CW_FACES = {
    "SM_MBuilding05_SkyscraperFacade_B": [
        ("front", 0.0, 5.0, 0.02, 2.91, -0.02, 0.012, 0.001, 0.3326)],
    "SM_MBuilding05_SkyscraperFacade_A": [
        ("front", 0.0, 5.0, 0.02, 2.91, -0.02, 0.010, 0.001, 0.3326)],
    "SM_MBuilding05_SkyscraperCorner_B": [
        ("front", -1.0, 5.0, 0.02, 2.91, -0.02, 0.012, 0.3335, 0.3325),
        ("left", 0.0, 6.0, 0.02, 2.91, -0.02, 0.012, 1.996, -0.3325)],
    "SM_MBuilding05_SkyscraperCorner_A": [
        ("front", 0.0, 5.0, 0.02, 2.91, -0.02, 0.010, 0.001, 0.3326),
        ("left", 0.0, 5.0, 0.02, 2.91, -0.02, 0.010, 1.664, -0.3326)],
}

# IN-PLANE fixed glazing that exists as geometry, measured the same way:
#   name -> [(u0, u1, v0, v1, out)]  (one rectangle per opening)
# Shopfronts, arcade windows, lobby walls and sashes — Zhao Xi'an's in-plane
# ladder applies to these, NOT the curtain-wall medians [§12 8.3].
_G_SHOP_FACES = {
    # family 04 brick commercial: the stone arcade and the top floor
    "SM_MBuilding04_FirstFloor_A": [(1.118, 2.882, 2.652, 5.689, -0.909)],
    "SM_MBuilding04_FirstFloor_B": [(0.750, 3.250, 1.052, 5.610, -0.811)],
    # _g2_: `SM_MBuilding04_TopFloor_A` used to be here as ONE rectangle
    # (0.732, 3.268, 0.600, 2.400, -0.905) — the bounding box of the whole
    # glass SUBSET, which spans two separate windows AND the brick pier between
    # them, so emptying it blanked the pier too. It is now four measured
    # rectangles (two windows, two lights each) in `_G2_WIN_FACES`. Moved, not
    # duplicated: a module in both tables would get two dark quads 5 cm apart
    # and z-fight.
    # family 05 podium lobby — 4.8 m BEHIND the arcade face
    "SM_MBuilding05_FirstFloor_A": [(0.271, 4.729, 0.575, 3.075, -4.823)],
    "SM_MBuilding05_FirstFloor_B": [(0.270, 4.730, 0.790, 2.610, -4.820)],
    # CivilianArea sashes (civic hall / offices) and the church lights
    "SM_SingleWindow_01a": [(1.008, 1.492, 0.508, 1.728, -0.175)],
    "SM_SingleWindow_01b": [(1.008, 1.492, 0.508, 1.728, -0.175)],
    "SM_DoubleWindow_01a": [(0.643, 1.857, 0.508, 1.728, -0.195)],
    "SM_DoubleWindow_01b": [(0.643, 1.857, 0.508, 1.728, -0.195)],
    "SM_Church_Window_02": [(0.903, 1.847, 0.709, 2.983, -0.145)],
    "SM_Church_Window_03": [(0.934, 1.816, 0.705, 2.409, -0.145)],
}

# Per-grade curtain-wall damage, from `earthquake_research.md` §12 table 8.13.
# `out` and `crack` are fractions of ALL panes on the mass, NOT of the band —
# inside the band the local density is far higher, which is the point.
# `storeys` is the height of the contiguous band; `sides` the number of
# elevations it wraps. DG5's 0.40-0.55 replaces the old 0.85, which is outside
# the field record.
G_GRADE = {
    1: dict(out=(0.000, 0.010), crack=(0.010, 0.030), storeys=(1, 1), sides=1,
            bend=0, peel=False, gaskets=(2, 6)),
    2: dict(out=(0.020, 0.050), crack=(0.050, 0.120), storeys=(1, 2), sides=1,
            bend=0, peel=False, gaskets=(3, 9)),
    3: dict(out=(0.100, 0.200), crack=(0.150, 0.300), storeys=(2, 4), sides=2,
            bend=2, peel=False, gaskets=(3, 8)),
    4: dict(out=(0.250, 0.400), crack=(0.250, 0.400), storeys=(4, 8), sides=2,
            bend=4, peel=True, gaskets=(2, 6)),
    5: dict(out=(0.400, 0.550), crack=(0.200, 0.350), storeys=(6, 99), sides=3,
            bend=6, peel=True, gaskets=(1, 4)),
}

# Glass type modifiers [§12 8.13]. `crack_k` scales cracked-and-retained,
# `out_k` fallout, `precar` the share of cracked panes in FEMA E-74's state 3
# ("shatters but remains in its frame in a precarious position"). Fully
# tempered has NO retained state: 6/6 specimens cracked and fell at the same
# drift.
G_GLASS_KIND = {
    "tempered":  dict(crack_k=0.05, out_k=1.00, precar=0.00, debris="dice"),
    "annealed":  dict(crack_k=1.00, out_k=0.85, precar=0.10, debris="plates"),
    "laminated": dict(crack_k=1.35, out_k=0.17, precar=0.55, debris="blanket"),
}

# Whole-unit peel: 1 façade system in 371 in the field, 5.2 % drift in the lab
# [§12 8.8]. AT MOST ONE PER SCENE, low down, one elevation. The budget is a
# module global because a scene is many `wreck_building` calls in one process;
# the assembly resets it with `g_scene_reset`.
_G_PEEL_BUDGET = [1]


def g_scene_reset(peel_budget=1):
    """Call once per SCENE (city / bake / bench row) before wrecking anything.

    Rare events are capped per scene, not per building — one whole-unit
    curtain-wall peel in the whole city, which is already generous against
    1-in-371 façade systems."""
    _G_PEEL_BUDGET[0] = int(peel_budget)


def _g_mat(ctx, key):
    """Agent G's material palette — FIVE per building, never per pane.

    Linear albedo (screen grey ~ linear ** 0.42), same convention as
    `materials()`. Broken glass is DARK GREY-GREEN, never white: a 0.2 m
    fragment of float glass on asphalt is a dark fleck that occasionally
    glints, and a heap of tempered dice is darker still because every facet
    scatters."""
    from pxr import UsdShade
    from . import damage
    flat = {
        # the interior seen through an empty frame: a shadowed pocket, not a
        # black void — the ledge returns above and below catch some light
        # MEASURED AGAINST THE KIT. The painted mullion in
        # `MI_MBuilding05_SkyscraperWindows_BaseColor.png` is RGB 85/86/94
        # sRGB = 0.09 linear, and the first opening at 0.042 linear rendered
        # only a shade darker than it — so two adjacent empty panes merged
        # into one black blob with the painted grid invisible inside it
        # (G_tow3/5_tower_g_glass5_sw.png). The hole has to go DARKER than the
        # mullion and the mullion has to match the painted one, or the cage
        # cannot read against the hole.
        "open": ((0.020, 0.019, 0.017), 0.92),
        # anodised aluminium mullion, dusty — the kit's own painted value
        "mullion": ((0.095, 0.098, 0.102), 0.45),
        # the floor slab edge seen through an empty frame. This is what makes
        # a hole read as a hole rather than as a dark reflection, and it is
        # also the stripe the research insists survives [§12 8.7].
        "slab": ((0.165, 0.158, 0.145), 0.95),
        # a crazed laminated pane: thousands of cracks scatter light, so it
        # goes PALE — but 0.10 linear is ~0.37 on screen, not paper
        "crazed": ((0.100, 0.112, 0.118), 0.34),
        # a pile of 3-12 mm tempered dice. delta_mm = 122100 / U_D gives 5.5 mm
        # at 100 MPa surface compression [§12 8.4]; ~100 000 per 1.5 x 1.8 m
        # pane, so this is a merged CLUMP mesh, never the dice.
        "dice": ((0.030, 0.042, 0.038), 0.55),
        # an EPDM gasket extruded out of the pocket in a ribbon — the DG1
        # signature, and it fails 24 % below the cracking drift
        "gasket": ((0.013, 0.013, 0.012), 0.88),
    }
    rgb, rough = flat[key]
    path = ctx["parent"] + "/QuakeLooks/g_" + key
    m = UsdShade.Material.Get(ctx["stage"], path)
    if not m:
        m = damage._pbr(ctx["stage"], path, rgb, rough)
    return m


# --- the mesh accumulator --------------------------------------------------
# Every authored piece of glass art goes into one of these and is emitted as a
# single merged mesh. Vertices are never shared between faces, so the renderer
# gets flat per-face normals without a normals attribute.
def _g_acc():
    return {"P": [], "c": [], "i": []}


def _g_quad(acc, p0, p1, p2, p3):
    n = len(acc["P"])
    acc["P"] += [p0, p1, p2, p3]
    acc["c"].append(4)
    acc["i"] += [n, n + 1, n + 2, n + 3]


def _g_face_quad(acc, fr, u0, u1, v0, v1, out):
    _g_quad(acc, _b_face_pt(fr, u0, v0, out), _b_face_pt(fr, u1, v0, out),
            _b_face_pt(fr, u1, v1, out), _b_face_pt(fr, u0, v1, out))


def _g_face_soft(acc, fr, u0, u1, v0, v1, out, r=None, n=4):
    """An opening with ROUNDED corners, as a triangle fan.

    A measured glazing rectangle is the bounding box of the glass, and on the
    kit's arcades the glass is ARCHED — so a square-cornered quad laid over it
    is a rectangular hole punched through an arch head, which is exactly the
    "rectangular cut-out" the user has rejected twice
    (G_shop2/3_commercial_g_glass4_sw.png). Rounding the corners costs 8 extra
    triangles and reads correctly under an arch, a segmental head or a plain
    square frame."""
    w, h = u1 - u0, v1 - v0
    if w <= 0 or h <= 0:
        return
    r = min(0.45, 0.30 * min(w, h)) if r is None else r
    if r < 0.03:
        _g_face_quad(acc, fr, u0, u1, v0, v1, out)
        return
    ring = []
    for (cu, cv, a0) in ((u1 - r, v1 - r, 0.0), (u0 + r, v1 - r, math.pi / 2.0),
                         (u0 + r, v0 + r, math.pi), (u1 - r, v0 + r, 1.5 * math.pi)):
        for k in range(n + 1):
            a = a0 + 0.5 * math.pi * k / float(n)
            ring.append((cu + r * math.cos(a), cv + r * math.sin(a)))
    c = _b_face_pt(fr, 0.5 * (u0 + u1), 0.5 * (v0 + v1), out)
    P = [c] + [_b_face_pt(fr, uu, vv, out) for (uu, vv) in ring]
    N = len(ring)
    counts, idx = [], []
    for k in range(N):
        counts.append(3)
        idx += [0, 1 + k, 1 + (k + 1) % N]
    base = len(acc["P"])
    acc["P"] += P
    acc["c"] += counts
    acc["i"] += [base + q for q in idx]


def _g_box(acc, cx, cy, cz, sx, sy, sz, yaw_rad):
    """A box, merged. Six independent quads so the normals stay crisp."""
    ca, sa = math.cos(yaw_rad), math.sin(yaw_rad)
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    def P(dx, dy, dz):
        return (cx + ca * dx - sa * dy, cy + sa * dx + ca * dy, cz + dz)
    a = [P(-hx, -hy, -hz), P(hx, -hy, -hz), P(hx, hy, -hz), P(-hx, hy, -hz),
         P(-hx, -hy, hz), P(hx, -hy, hz), P(hx, hy, hz), P(-hx, hy, hz)]
    for f in ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
              (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)):
        _g_quad(acc, a[f[0]], a[f[1]], a[f[2]], a[f[3]])


def _g_ground_quad(acc, cx, cy, z, sx, sy, yaw_rad, tilt=0.0):
    """A flat plate lying on the ground, with a couple of degrees of tilt so a
    field of them does not glint all at once."""
    ca, sa = math.cos(yaw_rad), math.sin(yaw_rad)
    hx, hy = sx / 2.0, sy / 2.0
    pts = []
    for (dx, dy, dz) in ((-hx, -hy, -tilt), (hx, -hy, tilt),
                         (hx, hy, tilt * 0.5), (-hx, hy, -tilt * 0.5)):
        pts.append((cx + ca * dx - sa * dy, cy + sa * dx + ca * dy, z + dz))
    _g_quad(acc, *pts)


def _g_crack_strip(acc, fr, u0, v0, u1, v1, out, rng, width=0.016, n_seg=None,
                   jag=0.10):
    """One jagged crack line as a quad strip, merged.

    The same construction as `_b_crack` (agent B, round 2), but accumulating
    instead of defining a prim: a 200-pane building would otherwise be a
    thousand two-triangle meshes."""
    n = n_seg or rng.randrange(4, 8)
    du, dv = u1 - u0, v1 - v0
    L = math.hypot(du, dv) or 1.0
    line, off = [], 0.0
    for k in range(n + 1):
        t = k / float(n)
        off = (0.0 if k in (0, n)
               else max(-jag, min(jag, -0.55 * off + rng.uniform(-jag, jag))))
        pu, pv = -dv / L, du / L
        line.append((u0 + du * t + pu * off, v0 + dv * t + pv * off))
    prev = None
    for k, (uu, vv) in enumerate(line):
        j = min(len(line) - 1, k + 1)
        h = max(0, k - 1)
        tu, tv = line[j][0] - line[h][0], line[j][1] - line[h][1]
        tl = math.hypot(tu, tv) or 1.0
        qu, qv = -tv / tl, tu / tl
        # taper: widest at the corner it grew from, closing to nothing
        w = width * max(0.12, 1.0 - 0.85 * (k / float(n)))
        cur = (_b_face_pt(fr, uu + qu * w / 2.0, vv + qv * w / 2.0, out),
               _b_face_pt(fr, uu - qu * w / 2.0, vv - qv * w / 2.0, out))
        if prev is not None:
            _g_quad(acc, prev[0], cur[0], cur[1], prev[1])
        prev = cur


def _g_emit(ctx, acc, mat, kind, owner=None):
    """Turn one accumulator into a single mesh prim."""
    if not acc or not acc["c"]:
        return None
    path = _b_face_mesh(ctx, (acc["P"], acc["c"], acc["i"]), mat, kind)
    if owner:
        _g_follow(ctx, path, owner)
    return path


def _g_follow(ctx, path, owner):
    """Register an authored prim that is STUCK TO A WALL, with the wall's world
    transform at the moment it was authored.

    Every glass recipe authors from the ELEMENT RECORDS (`e["x"]`, `e["z"]`),
    which are the building's pristine coordinates and are never updated when a
    later recipe moves the building. So a curtain wall drawn before
    `soft_storey` or `tilt_sink` would be left hanging in the air where the
    tower used to be. `r_glass_follow` replays the owner's own delta onto these
    prims afterwards; it goes LAST in any ladder whose grade moves the shell.
    Ground debris is deliberately NOT registered — glass that has already
    landed stays on the pavement while the building sinks into it."""
    if not path or not owner:
        return path
    from pxr import UsdGeom
    fol = ctx.setdefault("g_follow", {})
    rec = fol.get(owner)
    if rec is None:
        prim = ctx["stage"].GetPrimAtPath(owner)
        if not prim or not prim.IsValid():
            return path
        rec = {"M0": UsdGeom.XformCache().GetLocalToWorldTransform(prim),
               "paths": []}
        fol[owner] = rec
    rec["paths"].append(path)
    return path


def r_glass_follow(ctx):
    """Move the authored glass art by whatever its wall did after it was drawn.

    `_transform_prims` post-multiplies (p' = p * local * M), so if a wall's
    world transform went W0 -> W1 the delta is M = W0^-1 * W1, and applying
    that same M to the art puts it back on the wall. If the wall has been
    deleted meanwhile (a peel, an infill blow-out) the art goes with it."""
    from pxr import Gf, UsdGeom
    fol = ctx.pop("g_follow", None)
    if not fol:
        return
    xf = UsdGeom.XformCache()
    moved = dropped = 0
    for owner, rec in fol.items():
        prim = ctx["stage"].GetPrimAtPath(owner)
        if not prim or not prim.IsValid() or not prim.IsActive():
            for p in rec["paths"]:
                _deactivate(ctx["stage"], p)
            dropped += len(rec["paths"])
            continue
        M = rec["M0"].GetInverse() * xf.GetLocalToWorldTransform(prim)
        if Gf.IsClose(M, Gf.Matrix4d(1.0), 1e-7):
            continue
        moved += _transform_prims(ctx["stage"], rec["paths"], M)
    if moved or dropped:
        ctx["notes"].append(
            "glass_follow: {0} art mesh(es) carried with their wall, {1} "
            "dropped with a deleted module".format(moved, dropped))


def _g_left_frame(e, meas, fr):
    """The frame of a CORNER piece's second elevation (its x = xmin plane).

    A `SkyscraperCorner_B` is an L-shaped box glazed on both exposed faces;
    `_piece_frame` only describes the first. The second runs along the piece's
    local +Y with local -X outward, so it is the same frame rotated -90 deg
    about the piece's far corner (mesh (0, ymax))."""
    if not meas:
        return None
    sx, sy, sz, xmin, ymin, zmin = meas
    ox, oy, yaw, w, h, depth, dw = fr
    if dw:
        return None
    ymax = ymin + sy
    ca, sa = math.cos(yaw), math.sin(yaw)
    return (e["x"] - sa * ymax, e["y"] + ca * ymax, yaw - math.pi / 2.0,
            sy, sz, xmin - 0.02, False)


def _g_face_side(m, fr):
    """Which wall of the mass a face looks out of. `_b_face_pt` pushes a point
    outward by (sin(yaw), -cos(yaw)), so that vector is the face normal."""
    yaw = fr[2]
    nx, ny = math.sin(yaw), -math.cos(yaw)
    best, bd = "S", -9.9
    for s in ("S", "E", "N", "W"):
        ox, oy = _outward(m, s)
        d = nx * ox + ny * oy
        if d > bd:
            best, bd = s, d
    return best


def _g_faces(ctx, e):
    """Every glazed FACE of one curtain-wall kit piece."""
    from detail import urban_building as ub
    spec = _G_CW_FACES.get(e["name"])
    if not spec:
        return []
    fr = _piece_frame(e)
    if not fr:
        return []
    meas = ub.PIECES.get(e["name"])
    m = ctx["info"]["masses"].get(e["mass"]) or ctx["info"]["masses"]["main"]
    out = []
    for (plane, u0, u1, v0, v1, o, recess, tex0, texm) in spec:
        f = fr if plane == "front" else _g_left_frame(e, meas, fr)
        if f is None:
            continue
        out.append({"fr": f, "u0": u0, "u1": u1, "v0": e["z"] + v0,
                    "v1": e["z"] + v1, "out": o, "recess": recess,
                    "tex0": tex0, "texm": texm, "e": e, "plane": plane,
                    "side": _g_face_side(m, f), "mass": e["mass"],
                    "storey": e["storey"]})
    return out


def _g_pane_edges(f):
    """Pane boundaries along a face, ON the painted mullion grid."""
    u0, u1, t0, tm = f["u0"], f["u1"], f["tex0"], f["texm"]
    if abs(tm) < 1e-6:
        return [u0, u1]
    lo, hi = sorted((t0 + tm * u0, t0 + tm * u1))
    edges = {u0, u1}
    k = int(math.floor(lo)) - 1
    while k <= int(math.ceil(hi)) + 1:
        for off in (0.0, G_TILE_MULLION):
            t = k + off
            if lo - 1e-9 <= t <= hi + 1e-9:
                u = (t - t0) / tm
                if u0 + 0.12 < u < u1 - 0.12:
                    edges.add(u)
        k += 1
    es = sorted(edges)
    # merge the module-seam sliver into its neighbour: a 0.35 m pane going dark
    # reads as a slot cut in the wall, and the kit's texture phase resets at
    # every module so one always falls at the joint
    keep = [es[0]]
    for u in es[1:]:
        if u - keep[-1] < G_MIN_PANE_M and u < es[-1] - 1e-6:
            continue
        keep.append(u)
    if len(keep) > 2 and keep[-1] - keep[-2] < G_MIN_PANE_M:
        keep.pop(-2)
    return keep


def _g_panes(ctx, mass=None):
    """Every curtain-wall PANE on the building, as records."""
    out = []
    for e in _els(ctx, role=("wall", "corner")):
        if e["name"] not in _G_CW_FACES:
            continue
        if mass is not None and e["mass"] != mass:
            continue
        for f in _g_faces(ctx, e):
            es = _g_pane_edges(f)
            for i in range(len(es) - 1):
                ua, ub_ = es[i], es[i + 1]
                if ub_ - ua < 0.25:
                    continue
                out.append({"f": f, "ua": ua, "ub": ub_, "va": f["v0"],
                            "vb": f["v1"], "side": f["side"],
                            "storey": f["storey"], "mass": f["mass"],
                            "w": ub_ - ua, "h": f["v1"] - f["v0"]})
    return out


def _g_profile(ctx, m, rng, profile=None):
    """Which storey racked hardest — the drift profile [§12 8.6].

    A moment frame peaks in the LOWER THIRD (not at the ground floor); a core
    tower in the upper third; a podium-and-tower at the transition (Plaza
    Mayor, Concepción); a roof-diaphragm push takes the top storey only
    (Wenchuan, Qingchuan TCM Hospital). Returns (peak_index, name)."""
    n = max(1, len(m["levels"]))
    if profile is None:
        r = rng.random()
        profile = ("frame_low" if r < 0.45 else
                   "transition" if r < 0.73 else
                   "core_upper" if r < 0.90 else "top_diaphragm")
    if profile == "frame_low":
        peak = int(n * rng.uniform(0.08, 0.33))
    elif profile == "transition":
        peak = 0 if n <= 2 else int(n * rng.uniform(0.30, 0.55))
    elif profile == "core_upper":
        peak = int(n * rng.uniform(0.55, 0.85))
    else:
        peak = n - 1
    return max(0, min(n - 1, peak)), profile


def _g_band(ctx, m, grade, rng, profile=None):
    """(set of storeys, [sides], profile name) — the contiguous band."""
    g = G_GRADE[grade]
    n = max(1, len(m["levels"]))
    peak, pname = _g_profile(ctx, m, rng, profile)
    lo_s, hi_s = g["storeys"]
    k = min(n, rng.randint(lo_s, min(hi_s, max(lo_s, n))))
    if grade >= 5:
        # "lower half of the tower + the whole podium" [8.13]
        s0, s1 = 0, max(0, int(math.ceil(n * 0.55)) - 1)
    else:
        s0 = max(0, min(n - k, peak - rng.randint(0, max(0, k - 1))))
        s1 = min(n - 1, s0 + k - 1)
    return set(range(s0, s1 + 1)), _g_pick_sides(ctx, g["sides"]), pname


def _g_pick_sides(ctx, n):
    """The elevations the band wraps — ADJACENT first, opposite last.

    `_pick_sides` draws the rest at random, so a two-sided band came out as
    S+N: the two faces that cannot both be the high-drift side of the same
    torsion. The field is the other way round — Chile 2010's confined-masonry
    school failed at the CORNER because full-height glazing on one face threw
    the building into torsion, and Mexico City's NBS survey found 42 % of
    failures were CORNER buildings [§12 8.6, 8.15]. So the second elevation is
    a neighbour of the first, and the far face is only reached at DG5."""
    rng = ctx["rng"]
    first = rng.choice(["S", "S", "E", "W", "N"])       # front-biased
    nb = {"S": ["E", "W"], "N": ["E", "W"], "E": ["S", "N"], "W": ["S", "N"]}[first]
    rng.shuffle(nb)
    order = [first] + nb + [_opposite(first)]
    return order[:max(1, min(4, int(n)))]


def _g_kind(ctx, glass, rng):
    """Glass type. Modern towers are mostly fully tempered or FT-laminated; the
    field's bad performers are older annealed infill glazing with a few
    millimetres of clearance [§12 8.12]. Drawn when the caller says nothing,
    which is also where the per-building variety comes from."""
    if glass in G_GLASS_KIND:
        return glass
    r = rng.random()
    return "tempered" if r < 0.55 else ("annealed" if r < 0.85 else "laminated")


def _g_side_w(side, sides):
    """Kobe 1995: southerly-oriented glazing above 70 % damaged, northerly
    below 15 % [§12 8.6]. One elevation carries the band; the neighbours get a
    little; the far side almost nothing."""
    if not sides:
        return 1.0
    try:
        i = list(sides).index(side)
    except ValueError:
        return 0.055
    return (1.0, 0.62, 0.34)[i] if i < 3 else 0.12


def _g_add_corner_cracks(acc, fr, ua, ub_, va, vb, out, rng, n=None):
    """A corner-rooted crack set on ONE pane.

    "Crack propagation starts along the edges near diagonally opposed corners
    of glass panels where glass-to-aluminium contacts are made" [§12 8.5]. So:
    two DIAGONALLY OPPOSED corners, 2-3 jagged lines fanning inward from each,
    reaching well past mid-pane. Never a centre spiderweb — that is an impact
    signature and reads as a thrown rock."""
    w, h = ub_ - ua, vb - va
    diag = rng.random() < 0.5
    corners = ([(ua, va, 1, 1), (ub_, vb, -1, -1)] if diag
               else [(ub_, va, -1, 1), (ua, vb, 1, -1)])
    if rng.random() < 0.35:                     # one corner only — the DS1 state
        corners = corners[:1]
    for (cu, cv, su, sv) in corners:
        # THE CRUSH ZONE. FEMA P-58's DS1 photograph is a short conchoidal
        # CRUSHING patch where the glass bore on the aluminium, with the
        # radials fanning out of it [§12 8.5]. Without it the fan reads as a
        # scratch or an insect on the glass (G_tow4/3_tower_g_glass3_ne.png);
        # with it, the eye finds the corner first and the fan explains itself.
        cw = min(0.22, 0.16 * w)
        ch = min(0.26, 0.10 * h)
        _g_quad(acc,
                _b_face_pt(fr, cu, cv, out + 0.016),
                _b_face_pt(fr, cu + su * cw, cv, out + 0.016),
                _b_face_pt(fr, cu + su * cw * 0.45, cv + sv * ch, out + 0.016),
                _b_face_pt(fr, cu, cv + sv * ch, out + 0.016))
        for k in range(n or rng.randint(1, 2)):
            reach = rng.uniform(0.45, 0.95)
            a = rng.uniform(0.20, 1.35)         # fan about the diagonal
            du = su * w * reach * math.cos(a)
            dv = sv * h * reach * math.sin(a)
            _g_crack_strip(acc, fr, cu + su * 0.015, cv + sv * 0.015,
                           cu + du, cv + dv, out + 0.014, rng,
                           width=rng.uniform(0.038, 0.062),
                           n_seg=rng.randint(3, 6), jag=0.05)


def _g_add_crazed(acc_pale, acc_crack, fr, ua, ub_, va, vb, out, rng):
    """FEMA E-74 state 3: "shatters but remains in its frame or anchorage in a
    precarious position, liable to fall out at any time" — the ONE legitimate
    hanging pane, and only on laminated / annealed / film-retained glazing. A
    crazed pane goes pale because every crack scatters, so it is a dense fine
    mesh over a pale overlay, not a hole."""
    _g_face_quad(acc_pale, fr, ua + 0.02, ub_ - 0.02, va + 0.02, vb - 0.02,
                 out + 0.006)
    for k in range(rng.randint(6, 10)):
        u0 = rng.uniform(ua, ub_)
        v0 = rng.uniform(va, vb)
        a = rng.uniform(0, 3.14)
        L = rng.uniform(0.25, 0.90)
        _g_crack_strip(acc_crack, fr, u0, v0, u0 + L * math.cos(a),
                       v0 + L * math.sin(a), out + 0.016, rng, width=0.016,
                       n_seg=3, jag=0.05)


def _g_add_gasket(acc, fr, u, va, vb, out, rng):
    """A dry EPDM gasket extruded out of its pocket and hanging in a ribbon.

    Gasket failure happens 24 % BELOW the cracking drift [§12 8.7], so this is
    the DG1 signature: Northridge's towers lost their rubber gaskets by the
    hundred and almost no glass at all."""
    v0 = rng.uniform(va + 0.2, vb - 0.9)
    L = rng.uniform(0.35, 0.95)
    lean = rng.uniform(-0.10, 0.10)
    _g_quad(acc, _b_face_pt(fr, u, v0, out + 0.02),
            _b_face_pt(fr, u + 0.045, v0, out + 0.02),
            _b_face_pt(fr, u + 0.045 + lean, v0 - L, out + 0.06),
            _b_face_pt(fr, u + lean, v0 - L, out + 0.06))


def _g_drop(ctx, m, side, u_world):
    """Where a fallen pane lands.

    No reconnaissance report anywhere gives a measured glass throw distance
    [§12 8.9]; the model is a derivation from construction drop-zone practice:
    mode 0.05-0.10 H, p90 0.33 H, tail to 0.75 H, hard clip H/2, radius
    inversely proportional to fragment mass-per-area — so tempered dice land
    almost straight down in a compact heap and whole panes flutter out.
    A tower standing on a podium sheds onto the PODIUM ROOF until the debris
    clears the setback, and only then reaches the street."""
    rng = ctx["rng"]
    # the DROP HEIGHT of this pane above the surface it lands on, not the
    # building height: a pane on the tower's first storey is three metres above
    # the podium roof and lands at its own foot, and using H for every pane
    # threw the low ones metres out
    H = max(3.0, m["top"] - m["z0"])
    drop = max(1.5, u_world[2] - m["z0"])
    q = rng.random()
    if q < 0.72:
        r = drop * rng.uniform(0.05, 0.14)
    elif q < 0.94:
        r = drop * rng.uniform(0.14, 0.36)
    else:
        r = drop * rng.uniform(0.36, 0.70)
    ox, oy = _outward(m, side)
    z = m["z0"]
    main = ctx["info"]["masses"].get("main")
    elevated = (main is not None and m is not main
                and m["z0"] > main["z0"] + 0.5)
    if elevated and rng.random() < 0.32:
        # A SETBACK ROOF IS NOT A SINK. Most of a tower's glass lands on the
        # podium roof and stays there — but a share of it goes over the edge
        # (whole panes flutter; the Tachikawa argument in [§12 8.9] says a
        # released pane tumbles, and the DG5 street is described as "glass over
        # the plaza"). Without this the whole tower's loss is invisible from
        # the street, which is the one view the reviewer looks at.
        setback = max(main["W"], main["D"]) * 0.5
        r = setback + drop * rng.uniform(0.10, 0.45)
    r = min(r, 0.5 * (H + drop), 22.0)
    wx, wy = u_world[0] + ox * r, u_world[1] + oy * r
    if elevated:
        lx, ly = _to_local(main, wx, wy)
        if abs(lx) > main["W"] / 2.0 or abs(ly) > main["D"] / 2.0:
            z = main["z0"]
    return wx, wy, z, r


def _g_add_debris(ctx, acc, m, side, u_world, area, kind, density=1.0):
    """The glass under an empty frame — flat, granular and LOW.

    Christchurch, verbatim: "Damage to toughened glass was typically observed
    as an empty frame and a pile of glass fragments on the footpath", and "the
    tempered glass fragments tend to fall from the frame or anchorage in
    CLUSTERS" [§12 8.4]. Footprint ~1.2 x the panel area (FEMA P-58's own
    consequence function), and NEVER 100 000 dice — a 1.5 x 1.8 m tempered pane
    is ~100 000 fragments and 40 kg.

    ★ SIZE. The first bench read `foot` (the SPREAD of the pile, ~2.3 m for a
    full pane) as the size of each lump and dropped 1.4 m dark-green
    polyhedra: a field of tents (G_tow2/5_tower_g_glass5_close.png). The pile
    is 3-8 cm DEEP over a couple of metres, so it is flat mats and flecks, not
    boulders."""
    rng = ctx["rng"]
    if ctx.get("g_debris", 0) >= G_MAX_DEBRIS:
        return 0
    wx, wy, z, r = _g_drop(ctx, m, side, u_world)
    foot = math.sqrt(max(0.4, 1.2 * area))          # 1.2 x panel area
    n = 0

    def _plate(cx, cy, sx, sy, zz, tilt=0.012):
        _g_ground_quad(acc, cx, cy, zz, sx, sy, rng.uniform(0, 6.283), tilt)

    if kind == "blanket":
        # FT-laminated: the interlayer holds thousands of crumbs and the unit
        # "tends to fold and fall like a heavy blanket" [§12 8.4]
        _plate(wx, wy, foot * rng.uniform(0.7, 1.0), foot * rng.uniform(0.4, 0.7),
               z + 0.05, 0.05)
        n += 1
        n_sk = max(1, int(round(3 * density)))
        sk = (0.06, 0.16)
    elif kind == "plates":
        # annealed: "large, jagged shards", cracks running at 45 deg
        for k in range(max(1, int(round(rng.randint(2, 4) * density)))):
            s = rng.uniform(0.22, 0.55)
            _plate(wx + rng.uniform(-0.5, 0.5) * foot,
                   wy + rng.uniform(-0.5, 0.5) * foot, s,
                   s * rng.uniform(0.35, 0.9), z + 0.012)
            n += 1
        n_sk = max(2, int(round(6 * density)))
        sk = (0.05, 0.14)
    else:
        # tempered: 3-12 mm dice. A GRANULAR mat, 3-8 cm deep — which means a
        # tight cluster of many small pieces, not two big plates: at
        # foot x 0.3 a "clump" is a 0.7 m sheet and the pile reads as tarpaulin
        # (G_tow4/5_tower_g_glass5_close.png). The quads share one mesh, so
        # the cost of granularity is vertices, not prims.
        for k in range(max(4, int(round(rng.randint(11, 17) * density)))):
            rr = abs(rng.gauss(0.0, 0.30)) * foot
            a = rng.uniform(0, 6.283)
            s = rng.uniform(0.08, 0.22)
            _plate(wx + rr * math.cos(a), wy + rr * math.sin(a),
                   s, s * rng.uniform(0.55, 1.0),
                   z + rng.uniform(0.008, 0.055) * max(0.2, 1.0 - rr / foot),
                   0.018)
            n += 1
        n_sk = max(2, int(round(7 * density)))
        sk = (0.04, 0.10)
    # the skirt: single dice and, on tempered glass, the few 100-250 mm SPIKES
    # certified toughened glass still produces (the 1988 Croydon fatality)
    for k in range(n_sk):
        d = foot * rng.uniform(0.6, 2.2)
        a = rng.uniform(0, 6.283)
        s = (rng.uniform(0.11, 0.25) if (kind == "dice" and rng.random() < 0.12)
             else rng.uniform(*sk))
        _plate(wx + d * math.cos(a), wy + d * math.sin(a), s,
               s * rng.uniform(0.3, 0.9), z + 0.008, 0.006)
        n += 1
    ctx["g_debris"] = ctx.get("g_debris", 0) + n
    return n


def r_curtain_wall(ctx, grade=3, glass=None, mass=None, profile=None,
                   sides=None, out_frac=None, crack_frac=None, peel=None,
                   scatter=False):
    """A curtain-wall tower loses its GLASS in a BAND, and keeps its cage.

    The replacement for `r_glass_fallout`. Every number is from
    `earthquake_research.md` §12 (table 8.13 for the per-grade fractions, 8.6
    for the band, 8.4 for the debris, 8.5 for the crack art, 8.7 for what
    stays). Nothing is deleted: the module, its ledges, its transoms and its
    painted mullion grid all survive, and a dark opening is authored inside
    each lost pane, inset by half a painted mullion so the cage still reads.

    grade      1..5 (EMS-98 damage grade for this building)
    glass      "tempered" | "annealed" | "laminated"; None draws one
    profile    "frame_low" | "transition" | "core_upper" | "top_diaphragm"
    scatter    True = acceleration-driven loss on a STIFF building: drop the
               band and scatter single panes (Aleppo 2023 [§12 8.6]). This is
               what the round-2 code drew by accident, and it is right only
               here.
    """
    rng = ctx["rng"]
    grade = int(max(1, min(5, grade)))
    g = G_GRADE[grade]
    panes = _g_panes(ctx, mass)
    if not panes:
        return
    kind = _g_kind(ctx, glass, rng)
    km = G_GLASS_KIND[kind]
    by_mass = {}
    for p in panes:
        by_mass.setdefault(p["mass"], []).append(p)
    n_out_all = n_crack_all = n_mesh = 0
    for tag, ps in sorted(by_mass.items()):
        m = ctx["info"]["masses"].get(tag) or ctx["info"]["masses"]["main"]
        band, bsides, pname = _g_band(ctx, m, grade, rng, profile)
        if sides:
            bsides = list(sides)
        if scatter:
            band, pname = set(range(len(m["levels"]))), "scatter"
        of = out_frac if out_frac is not None else rng.uniform(*g["out"])
        cf = crack_frac if crack_frac is not None else rng.uniform(*g["crack"])
        of *= km["out_k"]
        cf *= km["crack_k"]
        n_out = int(round(of * len(ps)))
        n_crack = int(round(cf * len(ps)))
        for p in ps:
            d = 0 if p["storey"] in band else min(
                abs(p["storey"] - s) for s in band)
            w = 1.0 if d == 0 else (0.16 if d == 1 else 0.04)
            if scatter:
                w = 0.45 + 0.55 * rng.random()
            # a wide ribbon pane is the most vulnerable pane on any facade:
            # D_clear grows with h_p/b_p, so squat panes fail first [§12 8.2]
            w *= max(0.75, min(1.35, p["w"] / 1.5))
            p["s"] = w * _g_side_w(p["side"], bsides) * rng.uniform(0.7, 1.3)
        ps.sort(key=lambda q: -q["s"])
        out = ps[:n_out]
        rest = ps[n_out:]
        # ISOLATED SURVIVORS INSIDE THE BAND. Mexico City 1985, 90 Durango:
        # "loss of glazing apart from top three floors (EXCEPT ONE PANEL)". A
        # band with no survivor in it reads as a cut-out. [§12 8.6]
        n_surv = min(len(out), rng.randint(1, 3) if n_out > 6 else 0)
        for k in range(n_surv):
            i = rng.randrange(0, max(1, len(out)))
            rest.append(out.pop(i))
        # and a few strays outside it — the band is not a stencil
        for k in range(min(len(rest), rng.randint(0, 2))):
            j = rng.randrange(0, len(rest))
            out.append(rest.pop(j))
        # CRACKED PANES FOLLOW THE SAME SCORE, not just the band's storeys.
        # Filtering on `storey in band` alone put cracks on all four
        # elevations (the band is a storey range, and every face has those
        # storeys), so a DG3 tower came out veiled in cobwebs on the faces
        # that never racked (G_final_tower/3_tower_DG3_nw.png). `rest` is
        # already sorted by the score, which carries the side weight.
        crack = rest[:n_crack]
        # --- author, merged --------------------------------------------------
        # one mesh per (kind, side, storey); debris one per (kind, side)
        accs, owners = {}, {}

        def A(k_, side, storey, owner=None):
            key = (k_, side, storey)
            a = accs.get(key)
            if a is None:
                a = accs[key] = _g_acc()
                owners[key] = owner
            return a

        band_lo, band_hi = min(band), max(band)
        bars, seen_bar = 0, set()
        # spread the heaps evenly along the band instead of thinning every one:
        # a continuous belt of good clumps reads better than a uniform drizzle,
        # and it is what the reconnaissance describes — "an empty frame and a
        # PILE of glass fragments on the footpath" [§12 8.4]
        step = max(1, int(math.ceil(len(out) / float(G_HEAP_PANES))))
        heaped = set(id(q) for q in sorted(
            out, key=lambda q: (q["side"], q["storey"], q["ua"]))[::step])
        dens = 1.0
        for p in out:
            f = p["f"]
            own = f["e"]["p"].get("prim_path")
            ua, ub_ = p["ua"] + G_PANE_INSET, p["ub"] - G_PANE_INSET
            va, vb = p["va"] + G_PANE_INSET, p["vb"] - G_PANE_INSET
            if ub_ - ua < 0.12 or vb - va < 0.12:
                continue
            _g_face_quad(A("open", p["side"], p["storey"], own), f["fr"],
                         ua, ub_, va, vb, f["out"] - f["recess"])
            # THE SLAB EDGE. Look through an empty curtain-wall frame and the
            # first thing you see is the floor slab: a pale horizontal band
            # across the bottom of the opening. It is the cue that turns a
            # dark rectangle into a HOLE (a mirror-glazed tower has plenty of
            # dark rectangles that are only reflections), and it is the stripe
            # the research says survives [§12 8.7].
            _g_face_quad(A("slab", p["side"], p["storey"], own), f["fr"],
                         ua, ub_, min(vb, va + G_TRANSOM_H),
                         min(vb, va + G_TRANSOM_H + G_SLAB_H),
                         f["out"] - f["recess"] + 0.004)
            # THE CAGE, in relief. The painted mullion already holds the grid;
            # these bars give it a shadow and a lit edge so two adjacent empty
            # panes cannot merge into one black blob.
            if bars < G_MAX_BARS:
                ab = A("mull", p["side"], p["storey"], own)
                for uu in (p["ua"], p["ub"]):
                    key = (id(f["fr"]), round(uu, 2), p["storey"])
                    if key in seen_bar:
                        continue
                    seen_bar.add(key)
                    cx, cy, _z = _b_face_pt(f["fr"], uu, p["va"],
                                            f["out"] + G_MULLION_PROUD)
                    _g_box(ab, cx, cy, 0.5 * (p["va"] + p["vb"]), G_MULLION_W,
                           G_MULLION_W * 1.05, p["vb"] - p["va"], f["fr"][2])
                    bars += 1
                # the TRANSOM is a flat strip, not a box: a box here is six
                # quads for a line that reads at 20-80 m as a shadow, and the
                # pale slab band directly above it already carries the depth.
                # 600 boxes vs 400 quads is the difference between 1.6x and
                # 1.2x the round-2 archetype size.
                for (vv, key2) in ((p["va"], "lo"), (p["vb"], "hi")):
                    key = (id(f["fr"]), round(0.5 * (p["ua"] + p["ub"]), 2),
                           p["storey"], key2)
                    if key in seen_bar:
                        continue
                    seen_bar.add(key)
                    v0 = vv if key2 == "lo" else vv - G_TRANSOM_H
                    _g_face_quad(ab, f["fr"], p["ua"], p["ub"], v0,
                                 v0 + G_TRANSOM_H, f["out"] + 0.030)
                    bars += 1
            if id(p) in heaped:
                _g_add_debris(ctx, A("dice", p["side"], None), m, p["side"],
                              _b_face_pt(f["fr"], 0.5 * (p["ua"] + p["ub"]),
                                         p["va"], f["out"]),
                              (p["ub"] - p["ua"]) * (p["vb"] - p["va"]),
                              km["debris"], density=dens)
        # bent mullions at the band corners — the glass plastically deforms the
        # aluminium as it rotates into the frame [§12 8.7]. 10-40 mm.
        bent = [q for q in out if q["storey"] in (band_lo, band_hi)]
        rng.shuffle(bent)
        for p in bent[:g["bend"]]:
            f = p["f"]
            cx, cy, _z = _b_face_pt(f["fr"], p["ua"], p["va"],
                                    f["out"] + G_MULLION_PROUD
                                    + rng.uniform(*G_BENT_MM))
            _g_box(A("mull", p["side"], p["storey"],
                     f["e"]["p"].get("prim_path")),
                   cx, cy, 0.5 * (p["va"] + p["vb"]), G_MULLION_W,
                   G_MULLION_W * 1.05, p["vb"] - p["va"], f["fr"][2])
        for p in crack:
            f = p["f"]
            own = f["e"]["p"].get("prim_path")
            if km["precar"] > 0 and rng.random() < km["precar"]:
                _g_add_crazed(A("crazed", p["side"], p["storey"], own),
                              A("crack", p["side"], p["storey"], own),
                              f["fr"], p["ua"], p["ub"], p["va"], p["vb"],
                              f["out"], rng)
            else:
                _g_add_corner_cracks(A("crack", p["side"], p["storey"], own),
                                     f["fr"], p["ua"], p["ub"], p["va"],
                                     p["vb"], f["out"], rng)
        # gaskets go first — 24 % below the cracking drift, and hundreds per
        # tower at Northridge with almost no broken glass [§12 8.7]
        pool = [q for q in ps if q["storey"] in band] or ps
        for k in range(rng.randint(*g["gaskets"])):
            q = pool[rng.randrange(len(pool))]
            _g_add_gasket(A("gasket", q["side"], None,
                            q["f"]["e"]["p"].get("prim_path")),
                          q["f"]["fr"], q["ua"] + 0.03, q["va"], q["vb"],
                          q["f"]["out"], rng)
        mats = {"open": _g_mat(ctx, "open"), "mull": _g_mat(ctx, "mullion"),
                "crazed": _g_mat(ctx, "crazed"), "dice": _g_mat(ctx, "dice"),
                "gasket": _g_mat(ctx, "gasket"), "crack": ctx["mats"]["crack"],
                "slab": _g_mat(ctx, "slab")}
        for (k_, side, storey), a in sorted(
                accs.items(), key=lambda kv: str(kv[0])):
            if _g_emit(ctx, a, mats[k_], "g" + k_, owners[(k_, side, storey)]):
                n_mesh += 1
        n_out_all += len(out)
        n_crack_all += len(crack)
        ctx["notes"].append(
            "curtain_wall DG{0} {1} {2} ({3}): band storeys {4}-{5} on {6}, "
            "{7}/{8} panes out ({9:.0f} %), {10} cracked/retained".format(
                grade, kind, tag, pname, band_lo, band_hi, "+".join(bsides),
                len(out), len(ps), 100.0 * len(out) / max(1, len(ps)),
                len(crack)))
    if peel is None:
        peel = g["peel"]
    if peel:
        _g_peel(ctx, mass=mass, kind=km["debris"])
    ctx["notes"].append(
        "curtain_wall: {0} out, {1} cracked, {2} debris quads, {3} authored "
        "mesh(es)".format(n_out_all, n_crack_all, ctx.get("g_debris", 0),
                          n_mesh))


def _g_peel(ctx, mass=None, kind="dice"):
    """The one rare event: a whole curtain-wall UNIT peels off, glass and
    mullions together.

    Christchurch, one system in 371: aluminium screwed to a timber sub-frame,
    "multiple sections completely detaching along one side of the building at
    the second floor fell to the ground"; Mexico City 1985: "glazing and
    mullions at first level lost and buckled, due to downward movement of the
    curtain wall system relative to the pavement" — whole-unit loss is a
    GROUND-FLOOR phenomenon [§12 8.8, 8.6]. So: at most one per SCENE, one
    storey, one elevation, as low as the curtain wall goes."""
    if _G_PEEL_BUDGET[0] <= 0:
        return
    rng = ctx["rng"]
    cands = [e for e in _els(ctx, role=("wall",))
             if e["name"] in _G_CW_FACES and (mass is None or e["mass"] == mass)]
    if not cands:
        return
    lo = min(e["storey"] for e in cands)
    side = rng.choice(sorted({e["side"] for e in cands}))
    run = [e for e in cands if e["storey"] == lo and e["side"] == side]
    if not run:
        return
    run.sort(key=lambda e: (e["lx"], e["ly"]))
    run = run[:max(1, min(2, len(run)))]
    m = ctx["info"]["masses"].get(run[0]["mass"]) or ctx["info"]["masses"]["main"]
    ox, oy = _outward(m, side)
    acc, dacc = _g_acc(), _g_acc()
    gone = 0
    for e in run:
        if not _deactivate(ctx["stage"], e["p"].get("prim_path")):
            continue
        e["dead"] = True
        gone += 1
        # the unit on the pavement: a flat aluminium lattice, still gridded,
        # lying just clear of the wall with its glass shed round it
        fr = _piece_frame(e)
        if fr is None:
            continue
        d = rng.uniform(1.4, 3.2)
        for k in range(4):
            x, y, _z = _b_face_pt(fr, 0.35 + k * 1.45, m["z0"], d)
            _g_box(acc, x + ox * rng.uniform(-0.4, 0.4),
                   y + oy * rng.uniform(-0.4, 0.4), m["z0"] + 0.06,
                   rng.uniform(2.2, 3.0), 0.09, 0.10,
                   fr[2] + math.radians(rng.uniform(-14, 14)))
        for k in range(2):
            x, y, _z = _b_face_pt(fr, 2.5, m["z0"], d + rng.uniform(-0.6, 0.6))
            _g_box(acc, x, y, m["z0"] + 0.10, 0.10, 0.09,
                   rng.uniform(3.4, 4.6),
                   fr[2] + math.pi / 2.0 + math.radians(rng.uniform(-10, 10)))
        _g_add_debris(ctx, dacc, m, side,
                      _b_face_pt(fr, 2.5, m["z0"] + 0.2, 0.0), 10.0, kind)
    if gone:
        _g_emit(ctx, acc, _g_mat(ctx, "mullion"), "gpeel")
        _g_emit(ctx, dacc, _g_mat(ctx, "dice"), "gpeeldice")
        _G_PEEL_BUDGET[0] -= 1
        ctx["g_peel"] = True
        ctx["notes"].append(
            "curtain_wall: ONE whole-unit peel ({0} module(s), storey {1}, {2} "
            "side) — scene budget now {3}".format(gone, lo, side,
                                                  _G_PEEL_BUDGET[0]))


# In-plane fixed glazing — shopfronts, arcade windows, lobby walls, URM sashes.
# Zhao Xi'an's Wenchuan survey keyed the glass to the adjacent brick wall's
# state, which is exactly our grade [§12 8.3]:
#   < 1/800   nothing            |  1/500-1/300  "glass breaks in large numbers"
#   1/300-1/150 "frames buckle, bulge outward, even flung out; glass basically
#               shattered and scattered; often only the empty window opening is
#               left"           |  > 1/150  windows vanish with the wall.
# Ferndale 2010 gives the only published cracked-share for a shopfront row:
# "50 % of the glazing on Main Street was cracked" [§12 8.10] -> DG2.
G_SHOP_GRADE = {
    1: dict(out=0.00, crack=0.10, rack=0.0, sill=0.25),
    2: dict(out=0.06, crack=0.50, rack=0.0, sill=0.55),
    3: dict(out=0.35, crack=0.35, rack=0.06, sill=0.90),
    4: dict(out=0.70, crack=0.15, rack=0.16, sill=1.00),
    5: dict(out=0.92, crack=0.04, rack=0.24, sill=1.00),
}


def _g_shop_openings(ctx, mass=None, sides=None, storeys=None):
    """Every measured in-plane glazed opening on the building."""
    out = []
    for e in _els(ctx, role=("wall", "corner")):
        rects = _G_SHOP_FACES.get(e["name"])
        if not rects:
            continue
        if mass is not None and e["mass"] != mass:
            continue
        if sides and e["side"] not in sides:
            continue
        if storeys is not None and e["storey"] not in storeys:
            continue
        fr = _piece_frame(e)
        if fr is None:
            continue
        m = ctx["info"]["masses"].get(e["mass"]) or ctx["info"]["masses"]["main"]
        for (u0, u1, v0, v1, o) in rects:
            out.append({"fr": fr, "ua": u0, "ub": u1, "va": e["z"] + v0,
                        "vb": e["z"] + v1, "out": o, "e": e, "m": m,
                        "side": e["side"], "storey": e["storey"]})
    return out


def _g_add_rack(acc, op, lean, rng):
    """Zhao's high-drift state: "frames buckle, bulge outward, even flung out".

    The frame is squeezed into a PARALLELOGRAM with the sash still rectangular
    inside it (Wenchuan, verbatim) [§12 8.15]. Leaning jamb/mullion bars across
    the opening plus a head bar bowed out of plane, standing proud of the glass
    line so the lean is visible against the reveal."""
    fr = op["fr"]
    h = op["vb"] - op["va"]
    w = op["ub"] - op["ua"]
    n = max(2, int(round(w / 1.3)) + 1)
    for k in range(n):
        u = op["ua"] + w * (k / float(n - 1)) if n > 1 else op["ua"] + w / 2.0
        # a leaning jamb: two stacked boxes offset along the wall, so the
        # parallelogram reads without a shear transform on the mesh
        for t in (0.28, 0.75):
            cx, cy, _z = _b_face_pt(fr, u + lean * h * t, op["va"],
                                    op["out"] + 0.10)
            _g_box(acc, cx, cy, op["va"] + h * t, G_MULLION_W, G_MULLION_W,
                   h * 0.48, fr[2])
    cx, cy, _z = _b_face_pt(fr, op["ua"] + w / 2.0, op["vb"] - 0.10,
                            op["out"] + 0.10 + abs(lean) * 1.6)
    _g_box(acc, cx, cy, op["vb"] - 0.10, w * 0.94, G_MULLION_W, G_MULLION_W,
           fr[2] + math.radians(rng.uniform(-3.0, 3.0)))


def _g_add_sill(ctx, acc, op, kind, rng, heavy=1.0):
    """"The sill buried in rubble and glass" — the storefront signature.

    FEMA E-74 Fig. 6.3.1.4-6, observed: every pane gone, the vertical mullions
    bowed one way, the sill buried, the interior visible. Storefront glass is
    also the dangerous kind: FEMA P-58 puts the serious-injury rate per fallout
    unit at 0.25 for storefront against 0.02 for curtain wall [§12 8.9]."""
    if ctx.get("g_debris", 0) >= G_MAX_DEBRIS:
        return 0
    fr, m = op["fr"], op["m"]
    w = op["ub"] - op["ua"]
    n = max(5, int(round(16 * heavy * max(0.6, w / 2.5))))
    for k in range(n):
        u = rng.uniform(op["ua"] - 0.25, op["ub"] + 0.25)
        # THE SILL IS BURIED — 3 in 4 pieces land inside the reveal, in a dense
        # line against the frame, and only the rest washes out onto the
        # pavement. Storefront glass falls almost straight down: it is a metre
        # or two above the footpath, not forty [§12 8.9].
        o = (op["out"] + rng.uniform(0.05, 0.55) if rng.random() < 0.75
             else rng.uniform(0.3, 1.8))
        x, y, _z = _b_face_pt(fr, u, m["z0"], o)
        # annealed shopfront glass is "large jagged shards" but 0.7 m plates
        # lying flat read as tarpaulin from the pavement; 0.10-0.35 m with a
        # few larger ones is what a swept-up shopfront actually looks like.
        s = (rng.uniform(0.10, 0.35) if rng.random() < 0.82
             else rng.uniform(0.35, 0.60))
        _g_ground_quad(acc, x, y, m["z0"] + 0.010 + rng.uniform(0.0, 0.03), s,
                       s * rng.uniform(0.3, 0.9), rng.uniform(0, 6.283), 0.012)
    ctx["g_debris"] = ctx.get("g_debris", 0) + n
    return n


def _g_sill_litter(ctx, mass="main", sides=None, storey=0, density=1.0):
    """Glass on the pavement under a storey whose windows are PAINTED.

    Families 01 (stone apartment), 02 (office) and 03 (brownstone) have no
    glazing geometry at all — the windows are in the façade map — so there is
    nothing to open there, and honesty is a windrow of glass at the foot of the
    wall instead of a guessed rectangle. Banded under the glazed storey, which
    is already better than the round-2 shard field's spray on all four sides at
    a random depth."""
    rng = ctx["rng"]
    m = ctx["info"]["masses"].get(mass) or ctx["info"]["masses"]["main"]
    acc = _g_acc()
    n = 0
    for side in (sides or ("S",)):
        L = m["W"] if side in ("S", "N") else m["D"]
        # A WINDROW, not a sprinkle. Shopfront glass falls a metre or two, so
        # it lies in a line against the wall under the glazed storey — Loma
        # Prieta's Watsonville Main Street, "the sill buried in rubble and
        # glass" [§12 8.9]. Density is per metre of frontage.
        for k in range(max(6, int(round(38 * density * L / 20.0)))):
            t = rng.uniform(-0.48, 0.48) * L
            d = abs(rng.gauss(0.0, 0.75)) + 0.15      # hugs the wall
            if side in ("S", "N"):
                lx, ly = t, (-m["D"] / 2.0 - d if side == "S" else m["D"] / 2.0 + d)
            else:
                lx, ly = (-m["W"] / 2.0 - d if side == "W" else m["W"] / 2.0 + d), t
            wx, wy = _to_world(m, lx, ly)
            s = rng.uniform(0.10, 0.40)
            _g_ground_quad(acc, wx, wy, m["z0"] + 0.010, s,
                           s * rng.uniform(0.3, 0.9), rng.uniform(0, 6.283),
                           0.010)
            n += 1
    _g_emit(ctx, acc, ctx["mats"]["glass_shard"], "glit")
    return n


def r_storefront_glass(ctx, grade=3, mass=None, sides=None, storeys=None,
                       glass="annealed", litter=True):
    """Shopfronts, arcade windows, lobby glazing and URM sashes.

    NOT the curtain-wall medians: these panes are IN PLANE with the wall, so
    they take the wall's own drift, and the record is harsh — Loma Prieta's
    Watsonville Main Street, Northridge's storefronts, Türkiye 2023 where the
    glazed retail storey is what MADE the soft storey. The ladder is Zhao
    Xi'an's (`G_SHOP_GRADE` above).

    _g2_ (round 3): the building's PUNCHED windows are done here too, by
    `r_window_glass` — so every ladder that already asks for `storefront_glass`
    gets them without a ladder change. The sill-litter fallback now fires only
    when the building has NEITHER shopfront glazing NOR a measured window, which
    on the 16 styles means the stone apartment's arcade ground floor and the
    civic portico and nothing else."""
    rng = ctx["rng"]
    grade = int(max(1, min(5, grade)))
    g = G_SHOP_GRADE[grade]
    km = G_GLASS_KIND.get(glass, G_GLASS_KIND["annealed"])
    ops = _g_shop_openings(ctx, mass=mass, sides=sides, storeys=storeys)
    n_win = 0
    if not ctx.get("g2_win_done"):
        ctx["g2_win_done"] = True
        n_win = r_window_glass(ctx, grade=grade, mass=mass, sides=sides,
                               storeys=storeys, glass=glass)
    if not ops:
        if litter and not n_win:
            n = _g_sill_litter(
                ctx, mass=mass or "main",
                sides=sides or _pick_sides(ctx, 2 if grade >= 3 else 1),
                density=g["sill"])
            ctx["notes"].append(
                "storefront_glass DG{0}: no glazing geometry on this kit family "
                "(painted windows, and no punched window either) — {1} glass "
                "pieces at the sill".format(grade, n))
        return
    # the ground storey is where the drift and the shopfronts are
    for op in ops:
        op["s"] = (1.0 if op["storey"] == 0 else 0.45) * rng.uniform(0.7, 1.3)
    ops.sort(key=lambda q: -q["s"])
    n_out = int(round(g["out"] * len(ops)))
    n_crack = int(round(g["crack"] * len(ops)))
    accs, owners = {}, {}

    def A(k_, side, owner=None):
        key = (k_, side)
        a = accs.get(key)
        if a is None:
            a = accs[key] = _g_acc()
            owners[key] = owner
        return a

    n_rack = 0
    for op in ops[:n_out]:
        own = op["e"]["p"].get("prim_path")
        _g_face_soft(A("open", op["side"], own), op["fr"], op["ua"] + 0.03,
                     op["ub"] - 0.03, op["va"] + 0.03, op["vb"] - 0.03,
                     op["out"] - 0.03)
        # the lit floor inside the empty shopfront — same job as the curtain
        # wall's slab edge: it says HOLE, not dark reflection
        _g_face_quad(A("slab", op["side"], own), op["fr"], op["ua"] + 0.06,
                     op["ub"] - 0.06, op["va"] + 0.04,
                     min(op["vb"], op["va"] + 0.18), op["out"] - 0.026)
        if g["rack"] > 0 and rng.random() < 0.75:
            _g_add_rack(A("mull", op["side"], own), op,
                        rng.choice((-1.0, 1.0)) * g["rack"], rng)
            n_rack += 1
        _g_add_sill(ctx, A("dice", op["side"]), op, km["debris"], rng,
                    heavy=g["sill"])
    for op in ops[n_out:n_out + n_crack]:
        own = op["e"]["p"].get("prim_path")
        if km["precar"] > 0 and rng.random() < km["precar"]:
            _g_add_crazed(A("crazed", op["side"], own),
                          A("crack", op["side"], own), op["fr"], op["ua"],
                          op["ub"], op["va"], op["vb"], op["out"], rng)
        else:
            _g_add_corner_cracks(A("crack", op["side"], own), op["fr"],
                                 op["ua"], op["ub"], op["va"], op["vb"],
                                 op["out"], rng)
        if rng.random() < 0.35 * g["sill"]:
            _g_add_sill(ctx, A("dice", op["side"]), op, km["debris"], rng,
                        heavy=0.3)
    mats = {"open": _g_mat(ctx, "open"), "mull": _g_mat(ctx, "mullion"),
            "crazed": _g_mat(ctx, "crazed"), "dice": _g_mat(ctx, "dice"),
            "crack": ctx["mats"]["crack"], "slab": _g_mat(ctx, "slab")}
    n_mesh = 0
    for key, a in sorted(accs.items(), key=lambda kv: str(kv[0])):
        if _g_emit(ctx, a, mats[key[0]], "s" + key[0], owners[key]):
            n_mesh += 1
    ctx["notes"].append(
        "storefront_glass DG{0}: {1}/{2} openings emptied, {3} cracked, {4} "
        "frames racked, {5} authored mesh(es)".format(
            grade, n_out, len(ops), n_crack, n_rack, n_mesh))


# ---------------------------------------------------------------------------
# _g2_ (round 3, agent G2) — PUNCHED WINDOWS
# ---------------------------------------------------------------------------
# Agent G's gap, item 1, verbatim: "families 01/02/03 (apartment / office /
# brownstone kits) have no glazing geometry in `_G_SHOP_FACES`, so an office
# DG3 gets 82 pieces of glass on the pavement and not one broken window."
# `_g_glass_rects.py` keys on the bound MATERIAL, and those three families
# paint their glass into the façade map, so it found nothing on them — which
# is most of the city.
#
# The REVEALS are real geometry, though, so the openings are MEASURABLE, not
# guessable. `scene_gen/tools/_g2_win_rects.py` rasterises every kit module in
# `_piece_frame` coordinates, takes the module's largest outward-facing plane
# as the wall, and finds the HOLES in it; a second pass picks up glass-material
# subsets for the modules that glaze a whole bay and therefore punch no hole at
# all (the Downtown_West shopfronts). `scene_gen/tools/_g2_curate.py` then
# throws out the arcades, plant screens, stoops and cornice reveals it also
# finds. Raw sweep + audit trail: `scene_gen/_plans/glazing_probe/`.
#
# Three measurement traps, all of which cost a sweep:
#  (a) The flood that finds "outside" must be blocked by geometry AT ANY DEPTH,
#      not just by the wall. A FRENCH window runs to the bottom edge of its own
#      module (`SM_MBuilding03_Facade_B_Upper`, u 0.79..3.21, v 0.00..2.61), so
#      a flood seeded on the raster border walks straight into it. The reveal
#      behind it does cover those cells.
#  (b) …and the component must be CLIPPED at the outside, not discarded on
#      contact with it, or every French window is thrown away again.
#  (c) The dark quad goes on the SHALLOWEST light of the deepest glazing
#      CLUSTER, not on the deepest plane. A kit window is a stepped reveal with
#      two lights at different depths (fam01 Facade_A: lower light 0.25 m back,
#      upper 0.20 m); a quad behind the deepest one leaves the upper light
#      still showing its painted glass, and a quad on the first reveal step
#      sits 8 cm behind the wall and reads flat.
#
# WHY THIS IS NOT `_G_CW_FACES`. A curtain-wall pane is hung off the slab edges
# and takes RELATIVE movement (1.4-2 % drift, "as long as the building does not
# collapse, the curtain wall will not fail"); a punched window is IN PLANE with
# its wall and takes the wall's own drift, and Zhao Xi'an's Wenchuan ladder is
# harsh from 0.2 % [§12 8.3]. Same building, two orders of severity — that is
# the whole tower-vs-street contrast in the field record.
_G2_WIN_FACES = {
    "SM_MBuilding01_Facade_A": [
        (0.95, 1.95, 0.629, 2.222, -0.37, 0.907, 1.987, 0.58, 2.28),
        (3.05, 4.05, 0.629, 2.222, -0.37, 3.007, 4.087, 0.58, 2.26),
    ],
    "SM_MBuilding01_Facade_B": [
        (0.802, 1.322, 0.629, 2.222, -0.37, 0.767, 1.367, 0.58, 2.28),
        (2.247, 2.767, 0.629, 2.222, -0.37, 2.207, 2.807, 0.58, 2.28),
        (3.7, 4.22, 0.629, 2.222, -0.37, 3.667, 4.267, 0.58, 2.28),
    ],
    "SM_MBuilding01_Facade_C": [
        (0.86, 2.04, 0.189, 0.918, -0.17, 0.86, 2.04, 0.18, 0.92),
        (2.96, 4.14, 0.2, 0.9, -0.17, 2.96, 4.14, 0.2, 0.9),
        (0.95, 1.95, 1.257, 2.85, -0.3, 0.9, 2.0, 1.2, 2.9),
        (3.05, 4.05, 1.257, 2.85, -0.3, 3.02, 4.1, 1.22, 2.9),
    ],
    "SM_MBuilding01_Facade_D": [
        (0.72, 1.42, 0.189, 0.918, -0.17, 0.72, 1.42, 0.18, 0.92),
        (2.16, 2.86, 0.189, 0.918, -0.17, 2.16, 2.86, 0.18, 0.92),
        (3.6, 4.3, 0.189, 0.918, -0.17, 3.6, 4.3, 0.18, 0.92),
        (0.802, 1.322, 1.257, 2.85, -0.3, 0.76, 1.36, 1.2, 2.9),
        (2.247, 2.767, 1.257, 2.85, -0.3, 2.2, 2.8, 1.2, 2.9),
        (3.7, 4.22, 1.257, 2.85, -0.3, 3.66, 4.26, 1.2, 2.9),
    ],
    "SM_MBuilding02_Facade_A": [
        (0.302, 1.102, 0.6, 2.33, -0.3, 0.2, 1.18, 0.52, 2.5),
        (1.6, 2.4, 0.6, 2.33, -0.3, 1.52, 2.48, 0.52, 2.5),
        (2.898, 3.698, 0.6, 2.33, -0.3, 2.82, 3.8, 0.52, 2.5),
    ],
    "SM_MBuilding02_Facade_B": [
        (0.85, 1.65, 0.6, 2.33, -0.3, 0.76, 1.74, 0.52, 2.5),
        (2.35, 3.15, 0.6, 2.33, -0.3, 2.26, 3.24, 0.52, 2.5),
    ],
    "SM_MBuilding02_Facade_C": [
        (0.52, 3.5, 0.52, 2.48, -0.2, 0.52, 3.5, 0.52, 2.48),
    ],
    "SM_MBuilding02_FirstFloor_B": [
        (1.52, 6.5, 0.699, 3.999, -0.46, 1.52, 6.5, -0.001, 3.999),
        (1.52, 6.5, 4.519, 5.499, -0.46, 1.52, 6.5, 4.519, 5.499),
    ],
    "SM_MBuilding02_FirstFloor_C": [
        (1.24, 2.8, -0.001, 2.619, -0.32, 1.24, 2.8, -0.001, 2.619),
        (3.255, 4.78, -0.0, 2.619, -0.39, 3.22, 4.78, -0.001, 2.619),
        (5.2, 6.76, -0.001, 2.619, -0.32, 5.2, 6.76, -0.001, 2.619),
        (1.327, 2.705, 3.675, 5.399, -0.4, 1.24, 2.8, 3.579, 5.499),
        (3.305, 4.695, 3.675, 5.399, -0.4, 3.22, 4.78, 3.579, 5.499),
        (5.295, 6.673, 3.675, 5.399, -0.4, 5.2, 6.76, 3.579, 5.499),
    ],
    "SM_MBuilding02_FirstFloor_E": [
        (1.238, 3.4, 0.799, 5.399, -0.55, 1.14, 3.5, -0.001, 5.499),
        (4.6, 6.763, 0.799, 5.399, -0.55, 4.52, 6.86, -0.001, 5.499),
    ],
    "SM_MBuilding03_Facade_A": [
        (0.963, 3.037, 0.735, 2.209, -0.63, 0.8, 3.2, 0.46, 2.52),
    ],
    "SM_MBuilding03_Facade_B_Bottom": [
        (0.963, 3.037, 0.735, 2.209, -0.25, 0.8, 3.2, 0.46, 3.02),
    ],
    "SM_MBuilding03_Facade_B_Middle": [
        (0.963, 3.037, 0.9, 2.374, -0.25, 0.8, 3.2, -0.0, 3.02),
    ],
    "SM_MBuilding03_Facade_B_Upper": [
        (0.963, 3.037, 0.815, 2.289, -0.63, 0.8, 3.2, -0.0, 2.6),
    ],
    "SM_MBuilding03_Facade_C": [
        (0.963, 1.859, 0.735, 2.248, -0.15, 0.94, 1.88, 0.7, 2.28),
        (2.141, 3.037, 0.735, 2.248, -0.15, 2.12, 3.06, 0.7, 2.28),
    ],
    "SM_MBuilding03_FirstFloor_A": [
        (1.3, 2.7, 0.0, 2.2, -1.6, 1.257, 2.737, -0.0, 2.24),
        (1.257, 2.737, 2.3, 2.86, -1.53, 1.257, 2.737, 2.3, 2.86),
        (1.257, 2.737, 2.94, 3.56, -1.55, 1.257, 2.737, 2.94, 3.56),
    ],
    "SM_MBuilding04_Facade_A": [
        (0.732, 1.572, 0.6, 1.46, -0.24, 0.64, 1.66, 0.52, 2.48),
        (2.412, 3.272, 0.6, 1.46, -0.24, 2.34, 3.36, 0.52, 2.48),
        (0.732, 1.572, 1.55, 2.41, -0.19, 0.64, 1.66, 0.52, 2.48),
        (2.412, 3.272, 1.55, 2.41, -0.19, 2.34, 3.36, 0.52, 2.48),
    ],
    "SM_MBuilding04_TopFloor_A": [
        (0.732, 1.572, 0.6, 1.46, -0.91, 0.64, 1.66, 0.52, 2.48),
        (2.412, 3.272, 0.6, 1.46, -0.91, 2.34, 3.36, 0.52, 2.48),
        (0.732, 1.572, 1.55, 2.41, -0.86, 0.64, 1.66, 0.52, 2.48),
        (2.412, 3.272, 1.55, 2.41, -0.86, 2.34, 3.36, 0.52, 2.48),
    ],
    "SM_build_b_mod_lvl1_storefront_b_wall3m": [
        (0.102, 2.902, 0.728, 4.188, -0.22, 0.102, 2.902, 0.728, 4.188),
    ],
    "SM_build_b_mod_lvl1_storefront_b_wall5m": [
        (0.115, 2.435, 0.728, 4.188, -0.22, 0.115, 2.435, 0.728, 4.188),
        (2.655, 4.935, 0.728, 4.188, -0.22, 2.655, 4.935, 0.728, 4.188),
    ],
    "SM_build_b_mod_lvl2_doublewindow": [
        (0.475, 2.235, 0.366, 2.206, -0.9, 0.36, 2.32, 0.26, 4.42),
        (2.775, 4.535, 0.366, 2.206, -0.9, 2.68, 4.64, 0.26, 4.42),
        (0.49, 2.21, 2.295, 4.275, -0.87, 0.36, 2.32, 0.26, 4.42),
        (2.79, 4.53, 2.295, 4.275, -0.87, 2.68, 4.64, 0.26, 4.42),
    ],
    "SM_build_b_mod_lvl2_singlewindow": [
        (0.479, 2.519, 0.38, 2.22, -0.9, 0.36, 2.64, 0.24, 4.42),
        (0.494, 2.514, 2.316, 4.256, -0.87, 0.36, 2.64, 0.24, 4.42),
    ],
    "SM_build_b_mod_lvl2_widewindow": [
        (0.822, 4.222, 0.357, 2.917, -0.82, 0.72, 4.28, 0.24, 4.42),
        (0.807, 4.207, 2.97, 4.29, -0.78, 0.72, 4.28, 0.24, 4.42),
    ],
    "SM_build_b_mod_lvl3_doublewindow": [
        (0.483, 2.223, 0.367, 1.967, -0.9, 0.36, 2.32, 0.26, 3.88),
        (2.783, 4.523, 0.367, 1.967, -0.9, 2.68, 4.64, 0.26, 3.88),
        (0.498, 2.198, 2.064, 3.704, -0.87, 0.36, 2.32, 0.26, 3.88),
        (2.798, 4.518, 2.064, 3.704, -0.87, 2.68, 4.64, 0.26, 3.88),
    ],
    "SM_build_b_mod_lvl3_singlewindow": [
        (0.481, 2.521, 0.381, 1.961, -0.9, 0.36, 2.64, 0.24, 3.86),
        (0.496, 2.516, 2.067, 3.687, -0.87, 0.36, 2.64, 0.24, 3.86),
    ],
}


# Zhao Xi'an's IN-PLANE ladder [§12 8.3], keyed to the adjacent wall's state —
# which is exactly what a DG number is:
#   1/800-1/500 fine wall cracks       -> glass intact
#   1/500-1/300 X-cracks               -> "glass breaks in large numbers"
#   1/300-1/150 bricks falling         -> "frames buckle, bulge outward, even
#                                         flung out; glass basically shattered
#                                         and scattered; often only the empty
#                                         window opening is left"
#   > 1/150     masonry collapses      -> the windows vanish with the wall
# plus the only published percentage for a shopfront row: Ferndale, 2010 M6.5
# Eureka, "50 % of the glazing on Main Street was CRACKED" [§12 8-6] — a state
# with far more panes in it than "out", which is why `crack` outruns `out`
# until DG3.
#
# `out` and `crack` are fractions of EVERY measured opening on the building.
# They are small because a building is not one storey: the openings are sorted
# by a score that is ~1.0 inside the racked band on the street face and 0.04
# on a face that never moved, so 0.18 of the whole building is most of the
# band. `crack_w` below spreads cracking WIDER than fallout on purpose —
# cracking starts at a lower drift, so it reaches storeys the band does not.
G2_WIN_GRADE = {
    1: dict(out=0.005, crack=0.04, storeys=1, sides=1, rack=0.00, sill=0.15),
    2: dict(out=0.030, crack=0.22, storeys=2, sides=1, rack=0.00, sill=0.50),
    3: dict(out=0.180, crack=0.22, storeys=2, sides=2, rack=0.20, sill=0.90),
    4: dict(out=0.340, crack=0.14, storeys=3, sides=2, rack=0.70, sill=1.00),
    5: dict(out=0.500, crack=0.06, storeys=99, sides=3, rack=0.85, sill=1.00),
}

# "Opening sashes fared better than fixed lights" — Mexico City 1985 [M9], and
# Wenchuan's "open casements survived" [W8]; the Chi-Chi survey says the same
# ("fixed lights damaged far more than opening sashes"). A casement or a
# double-hung sash can rotate a little inside its frame before it bears, so it
# has clearance a fixed light does not. A fixed share of the openings is drawn
# as openable and scored down by G2_SASH_K, which is what leaves survivors
# scattered through a band instead of a clean stencil.
G2_SASH_SHARE = 0.32
G2_SASH_K = 0.28

# The rack, from the Wenchuan photographs [§12 8.15]: the frame is squeezed
# into a PARALLELOGRAM with the sash still RECTANGULAR inside it, and at the
# high-drift end it "bulges outward". `lean` is u per unit v (a 0.10 lean over
# a 1.8 m opening is 180 mm of skew, which is what a 1/150 storey does to a
# 3 m storey height); `bulge` is how far the head bows out of plane.
G2_LEAN = (0.045, 0.130)
G2_BULGE = (0.04, 0.14)
G2_SASH_DROP = (0.03, 0.11)      # m: the jammed sash sits down in its frame
G2_BAR = 0.075                   # m: authored frame bar, square section
G2_PROUD = 0.045                 # m: how far it stands off the glass line

# ★ WHAT MAKES AN EMPTIED PUNCHED WINDOW READ — and it is NOT darkness.
# On a mirror curtain wall agent G found that half the façade is already dark,
# so an authored dark rectangle has to fight reflections; on these families it
# is worse. `SM_MBuilding02_Facade_A`'s painted window is very nearly BLACK in
# the façade map (G2_win1/0_office_pristine_nw.png), so a 0.020-linear quad
# laid over it is invisible at 40 m — the whole band of 32 emptied windows at
# DG3 read as "slightly darker windows".
# Two cues carry it instead, and both are in the record:
#   * the FLOOR of the room, a pale band across the bottom of the opening. The
#     five bench cameras that matter look DOWN, so an empty opening shows its
#     floor; an intact one cannot. Same job as agent G's slab edge.
#   * the CORNER REMNANT. FEMA E-74's tower photograph, per `eq_round3_R.md`
#     §3: "one pane missing out of ~12 … a dark TRIANGULAR REMNANT still lodged
#     at the head of the opening". A pale crazed triangle against a black hole
#     is the most legible thing on the whole façade at 40 m, and it is the one
#     triangle the research actually asks for — a remnant IN the frame, never a
#     fragment on the ground.
G2_FLOOR_BAND = (0.24, 0.40)     # m: the pale floor seen through the opening
G2_REMNANT_P = 0.45              # share of emptied openings keeping a remnant
G2_REMNANT_M = (0.14, 0.85)      # m: min/max leg of one — a FRAGMENT, and on a
#                                  wide shopfront bay a share of the pane is
#                                  metres across, which reads as an awning


def _g2_openings(ctx, mass=None, sides=None, storeys=None):
    """Every measured PUNCHED window on the building, as records.

    Same record shape as `_g_shop_openings` (so `_g_add_rack` / `_g_add_sill`
    still take one) plus the reveal rectangle `h*`, which is the hole in the
    wall rather than the glass inside it."""
    out = []
    for e in _els(ctx, role=("wall", "corner")):
        rects = _G2_WIN_FACES.get(e["name"])
        if not rects:
            continue
        if mass is not None and e["mass"] != mass:
            continue
        if sides and e["side"] not in sides:
            continue
        if storeys is not None and e["storey"] not in storeys:
            continue
        fr = _piece_frame(e)
        if fr is None:
            continue
        m = ctx["info"]["masses"].get(e["mass"]) or ctx["info"]["masses"]["main"]
        for r in rects:
            u0, u1, v0, v1, o = r[:5]
            hu0, hu1, hv0, hv1 = r[5:9] if len(r) >= 9 else (u0, u1, v0, v1)
            out.append({"fr": fr, "ua": u0, "ub": u1, "va": e["z"] + v0,
                        "vb": e["z"] + v1, "out": o, "e": e, "m": m,
                        "hua": hu0, "hub": hu1, "hva": e["z"] + hv0,
                        "hvb": e["z"] + hv1,
                        "side": e["side"], "storey": e["storey"],
                        "mass": e["mass"]})
    return out


def _g2_band(ctx, m, grade, rng, profile=None):
    """(storeys that racked, elevations, profile name) for a PUNCHED-window
    building.

    Unlike a curtain wall, an in-plane window follows its own storey's drift,
    and on URM/RC stock the drift concentrates LOW — the soft/open ground
    storey and the one above it (Zhao: "lower story obtained higher damage
    than higher story"; Chi-Chi: near-field lower storeys first) [§12 8.3,
    8.6]. So the profile draw is biased to `frame_low` where `_g_profile`
    spreads it evenly."""
    g = G2_WIN_GRADE[grade]
    n = max(1, len(m["levels"]))
    if profile is None:
        r = rng.random()
        profile = ("frame_low" if r < 0.72 else
                   "transition" if r < 0.92 else "top_diaphragm")
    peak, pname = _g_profile(ctx, m, rng, profile)
    k = min(n, max(1, min(int(g["storeys"]), n)))
    if grade >= 5:
        s0, s1 = 0, max(0, int(math.ceil(n * 0.6)) - 1)
    else:
        s0 = max(0, min(n - k, peak - rng.randint(0, max(0, k - 1))))
        s1 = min(n - 1, s0 + k - 1)
    return set(range(s0, s1 + 1)), _g_pick_sides(ctx, g["sides"]), pname


def _g2_is_sash(op):
    """Is this opening an OPENABLE sash rather than a fixed light?

    Drawn from the opening's own coordinates, not from the rng, so the same
    window is the same kind on every re-run and at every grade — a building
    does not swap its casements for fixed lights between DG2 and DG3. Rolled
    by hand and not with `hash()`, which Python 3 randomises per process
    (PYTHONHASHSEED), so `hash()` would make the bake non-reproducible."""
    h = 0
    for c in op["e"]["name"]:
        h = (h * 131 + ord(c)) & 0xFFFFFFFF
    for q in (op["ua"], op["va"], op["e"]["x"], op["e"]["y"]):
        h = (h * 131 + int(round(q * 100.0))) & 0xFFFFFFFF
    return (h % 1000) / 1000.0 < G2_SASH_SHARE


def _g2_add_rack(acc, op, lean, rng, bulge=0.0):
    """The high-drift state, from the Wenchuan photographs [§12 8.15]:

    *"frames buckle, bulge outward, even flung out; glass basically shattered
    and scattered; often only the empty window opening is left"* — and,
    verbatim, the frames are *"squeezed into parallelograms with the sashes
    still rectangular and intact inside"*.

    So: the JAMBS lean (the frame goes to a parallelogram, drawn as stacked
    boxes offset along the wall so no shear transform is needed on the mesh),
    the HEAD bows out of plane, and the SASH is a rectangle — four thin bars,
    unsheared, sitting a few centimetres down in its own frame because it has
    jammed. The sash is the point of the whole thing: a parallelogram alone
    reads as a modelling error, a rectangle inside a parallelogram reads as a
    building that moved."""
    fr = op["fr"]
    hu0, hu1 = op["hua"], op["hub"]
    hv0, hv1 = op["hva"], op["hvb"]
    w, h = hu1 - hu0, hv1 - hv0
    if w <= 0.2 or h <= 0.2:
        return
    o = op["out"] + G2_PROUD
    # the two jambs, leaning: three stacked boxes each
    for u_ in (hu0, hu1):
        for t in (0.18, 0.5, 0.82):
            cx, cy, _z = _b_face_pt(fr, u_ + lean * h * (t - 0.5), hv0, o)
            _g_box(acc, cx, cy, hv0 + h * t, G2_BAR, G2_BAR, h * 0.36, fr[2])
    # the head, bowed OUT of plane, and the sill
    for (v_, bo) in ((hv1 - G2_BAR, bulge), (hv0 + G2_BAR, 0.25 * bulge)):
        cx, cy, _z = _b_face_pt(fr, 0.5 * (hu0 + hu1) + lean * h * 0.5 * (
            1.0 if v_ > 0.5 * (hv0 + hv1) else -1.0), v_, o + bo)
        _g_box(acc, cx, cy, v_, w * 0.98, G2_BAR, G2_BAR,
               fr[2] + math.radians(rng.uniform(-2.5, 2.5)))
    # THE SASH: still rectangular, dropped and shoved to one side
    du = lean * h * rng.uniform(0.25, 0.55)
    dv = -rng.uniform(*G2_SASH_DROP)
    su0, su1 = hu0 + 0.10 + du, hu1 - 0.10 + du
    sv0, sv1 = hv0 + 0.10 + dv, hv1 - 0.10 + dv
    for (a, b, c, d) in ((su0, su0, sv0, sv1), (su1, su1, sv0, sv1),
                         (su0, su1, sv0, sv0), (su0, su1, sv1, sv1)):
        cu, cv = 0.5 * (a + b), 0.5 * (c + d)
        cx, cy, _z = _b_face_pt(fr, cu, cv, o + 0.02 + bulge * 0.5)
        _g_box(acc, cx, cy, cv, max(G2_BAR, b - a), G2_BAR * 0.8,
               max(G2_BAR, d - c), fr[2])


def _g2_add_remnant(acc, op, rng):
    """A jagged shard still lodged in a corner of an empty opening.

    FEMA E-74 Fig. 6.3.1.4-1, observed [`eq_round3_R.md` §3]: one pane gone,
    the mullions straight, and "a dark triangular remnant still lodged at the
    head of the opening". It is authored PALE (the crazed material) rather than
    dark, because a shattered edge scatters and because the hole behind it is
    already as dark as this kit gets — the remnant is the contrast, not the
    hole."""
    fr = op["fr"]
    ua, ub_ = op["ua"] + 0.02, op["ub"] - 0.02
    va, vb = op["va"] + 0.02, op["vb"] - 0.02
    w, h = ub_ - ua, vb - va
    if w < 0.25 or h < 0.25:
        return
    corners = [(ua, vb, 1, -1), (ub_, vb, -1, -1), (ua, va, 1, 1),
               (ub_, va, -1, 1)]
    rng.shuffle(corners)
    # the head corners first: glass hangs from the head, it does not sit on the
    # sill (the sill sweeps clear, which is where `_g2_add_sill` puts it)
    corners.sort(key=lambda c: 0 if c[3] < 0 else 1)
    for (cu, cv, su, sv) in corners[:rng.randint(1, 2)]:
        # CAPPED IN METRES, not only as a share. On a 5 x 2.6 m shopfront bay
        # 0.55 x width is a 2.7 m shard, which renders as an awning or a
        # curtain, not as broken glass (G2_win5/3_office_g2_win3_street.png,
        # first cut). A remnant lodged in a frame is a fragment: 0.3-1.0 m.
        du = su * min(G2_REMNANT_M[1], w * rng.uniform(0.22, 0.55))
        dv = sv * min(G2_REMNANT_M[1] * 1.2, h * rng.uniform(0.22, 0.60))
        if abs(du) < G2_REMNANT_M[0] or abs(dv) < G2_REMNANT_M[0]:
            continue
        # a jagged hypotenuse, 3 segments, so the shard is not a clean triangle
        pts = [(cu, cv), (cu + du, cv)]
        for k in (0.66, 0.33):
            pts.append((cu + du * k + rng.uniform(-0.05, 0.05) * w,
                        cv + dv * (1.0 - k) + rng.uniform(-0.05, 0.05) * h))
        pts.append((cu, cv + dv))
        # FANNED FROM THE CORNER, not emitted as one n-gon: the jagged
        # hypotenuse can leave the polygon slightly concave, and Hydra fans an
        # n-gon from its first vertex, which on a concave polygon folds a
        # sliver back over itself.
        P = [_b_face_pt(fr, uu, vv, op["out"] + 0.010) for (uu, vv) in pts]
        for k in range(1, len(P) - 1):
            base = len(acc["P"])
            acc["P"] += [P[0], P[k], P[k + 1]]
            acc["c"].append(3)
            acc["i"] += [base, base + 1, base + 2]


def _g2_add_sill(ctx, acc, op, rng, heavy=1.0):
    """The glass that stays ON the sill, inside the reveal.

    Not the same thing as the fall zone: a punched window's sill is a 0.1-0.3 m
    ledge and it catches a line of shards that never reach the street. It is
    also the only part of the debris that is visible from ABOVE on an upper
    storey, which matters because five of the seven bench cameras look down."""
    if ctx.get("g_debris", 0) >= G_MAX_DEBRIS:
        return 0
    fr = op["fr"]
    hu0, hu1 = op["hua"], op["hub"]
    n = max(3, int(round(7 * heavy * max(0.5, (hu1 - hu0) / 1.2))))
    for k in range(n):
        u = rng.uniform(hu0 + 0.04, hu1 - 0.04)
        o = op["out"] + rng.uniform(0.04, 0.9) * abs(op["out"]) * 0.9
        x, y, _z = _b_face_pt(fr, u, op["hva"], min(-0.02, o))
        s = rng.uniform(0.07, 0.24)
        _g_ground_quad(acc, x, y, op["hva"] + 0.012 + rng.uniform(0.0, 0.02),
                       s, s * rng.uniform(0.3, 0.9), fr[2] + rng.uniform(-0.5, 0.5),
                       0.010)
    ctx["g_debris"] = ctx.get("g_debris", 0) + n
    return n


def r_window_glass(ctx, grade=3, mass=None, sides=None, storeys=None,
                   glass="annealed", profile=None, scatter=False,
                   band_sides=None):
    """The four FEMA E-74 states on a building's PUNCHED windows.

    Families 01/02/03/04 and the Downtown_West terrace, i.e. every URM and RC
    building in the city that is not a curtain-wall tower. `r_storefront_glass`
    calls this, so every ladder that already asks for `storefront_glass` gets
    it with no ladder change.

    grade 1  a few CRACKED panes, corner-rooted, on one racked storey; 0-1 out.
             (FEMA E-74's tower photograph, and Baird's level 1: "some cracked
             panes; none broken".)
    grade 2  Ferndale: ~half the street face CRACKED, a few panes out. Baird's
             level 2: "extensive cracked glass; little broken glass".
    grade 3  Zhao's 1/500-1/300 band: "glass breaks in large numbers" on the
             storeys that racked — the FRAMES STAY.
    grade 4  Zhao's 1/300-1/150 band: "often only the empty window opening is
             left", frames racked into parallelograms and bulging outward with
             the sash jammed rectangular inside.
    grade 5  the same, over the lower 60 % of the building. (At DG5 proper the
             wall itself goes, and `masonry_collapse` / `pancake` take the
             windows with it.)

    `glass` defaults to ANNEALED, not to the curtain wall's tempered-heavy
    draw: the field's bad performers are old float glass in putty or a few
    millimetres of clearance [§12 8.12], which is what this stock is.
    `scatter=True` drops the band for i.i.d. single-pane loss — right only for
    a stiff, acceleration-driven building (Aleppo 2023, Türkiye's "scattered
    single panes" away from the collapsed soft storeys) [§12 8.6].

    `sides` and `band_sides` are NOT the same thing and the difference is a
    footgun. `sides` FILTERS the opening set — "this building is only glazed on
    the south and east" — so the per-grade fractions are then fractions of that
    subset. `band_sides` leaves every opening in play and only says which
    elevations the band lands on, which is what a caller who just wants the
    damage pointed at a camera means."""
    rng = ctx["rng"]
    grade = int(max(1, min(5, grade)))
    g = G2_WIN_GRADE[grade]
    km = G_GLASS_KIND.get(glass, G_GLASS_KIND["annealed"])
    ops = _g2_openings(ctx, mass=mass, sides=sides, storeys=storeys)
    if not ops:
        return 0
    by_mass = {}
    for op in ops:
        by_mass.setdefault(op["mass"], []).append(op)
    n_out_all = n_crack_all = n_rack = n_mesh = 0
    for tag, group in sorted(by_mass.items()):
        m = ctx["info"]["masses"].get(tag) or ctx["info"]["masses"]["main"]
        band, bsides, pname = _g2_band(ctx, m, grade, rng, profile)
        if band_sides or sides:
            bsides = list(band_sides or sides)
        if scatter:
            band, pname = set(range(len(m["levels"]))), "scatter"
        for op in group:
            d = 0 if op["storey"] in band else min(
                abs(op["storey"] - s) for s in band)
            # FALLOUT is banded hard; CRACKING is not. Cracking starts at a
            # lower drift than fallout on every curve in [§12 8.13], so it
            # reaches storeys the fallout band never touches — which is also
            # the only way Ferndale's "50 % of the glazing on Main Street"
            # comes out as a STREET rather than as one storey.
            w = 1.0 if d == 0 else (0.18 if d == 1 else 0.04)
            cw = 1.0 if d == 0 else (0.60 if d == 1 else 0.30)
            if scatter:
                w = cw = 0.45 + 0.55 * rng.random()
            sw = _g_side_w(op["side"], bsides)
            area = max(0.2, (op["ub"] - op["ua"]) * (op["vb"] - op["va"]))
            # a wide squat pane is the most vulnerable pane on any façade
            # (D_clear grows with h_p/b_p) [§12 8.2]
            ar = (op["ub"] - op["ua"]) / max(0.2, op["vb"] - op["va"])
            av = max(0.8, min(1.35, 0.75 + 0.5 * ar))
            sk = G2_SASH_K if _g2_is_sash(op) else 1.0
            j = rng.uniform(0.72, 1.32)
            op["s"] = w * sw * av * sk * j
            op["sc"] = cw * sw * av * sk * j
            op["area"] = area
        group.sort(key=lambda q: -q["s"])
        n_out = int(round(g["out"] * len(group)))
        out = group[:n_out]
        rest = group[n_out:]
        # survivors inside the band and a couple of strays outside it: a band
        # with no survivor in it reads as a cut-out [§12 8.6]
        if n_out > 5:
            for k in range(rng.randint(1, 3)):
                if out:
                    rest.append(out.pop(rng.randrange(len(out))))
            for k in range(rng.randint(0, 2)):
                if rest:
                    out.append(rest.pop(rng.randrange(len(rest))))
        rest.sort(key=lambda q: -q["sc"])
        crack = rest[:int(round(g["crack"] * len(group)))]
        accs, owners = {}, {}

        def A(k_, side, storey, owner=None):
            key = (k_, side, storey)
            a = accs.get(key)
            if a is None:
                a = accs[key] = _g_acc()
                owners[key] = owner
            return a

        for op in out:
            own = op["e"]["p"].get("prim_path")
            fr = op["fr"]
            # THE EMPTY OPENING. Rounded corners, because a measured rectangle
            # laid over an arched or segmental head is the square cut-out the
            # user has rejected twice; and it goes at the glazing depth, so
            # every reveal step, transom and mullion in FRONT of it survives —
            # "the empty window opening is left", not a hole in the wall.
            # a SMALL radius: `_g_face_soft`'s default (0.30 x the short side)
            # is sized for the fam04 arcade's arched heads and turns a 0.7 m
            # sash into an ellipse. A punched window has a square or a
            # segmental head, so it wants just enough rounding to stop the
            # quad reading as a stencil.
            _g_face_soft(A("open", op["side"], op["storey"], own), fr,
                         op["ua"] + 0.02, op["ub"] - 0.02,
                         op["va"] + 0.02, op["vb"] - 0.02, op["out"],
                         r=min(0.11, 0.13 * min(op["ub"] - op["ua"],
                                                op["vb"] - op["va"])), n=3)
            # THE FLOOR OF THE ROOM: a pale band across the bottom of the
            # opening. This, and not the darkness of the quad, is what says
            # HOLE on a façade whose painted windows are already black.
            fb = rng.uniform(*G2_FLOOR_BAND)
            _g_face_quad(A("slab", op["side"], op["storey"], own), fr,
                         op["ua"] + 0.05, op["ub"] - 0.05, op["va"] + 0.03,
                         min(op["vb"] - 0.05, op["va"] + 0.03 + fb),
                         op["out"] + 0.006)
            if rng.random() < G2_REMNANT_P:
                _g2_add_remnant(A("crazed", op["side"], op["storey"], own),
                                op, rng)
            if g["rack"] > 0 and rng.random() < g["rack"]:
                _g2_add_rack(A("mull", op["side"], op["storey"], own), op,
                             rng.choice((-1.0, 1.0)) * rng.uniform(*G2_LEAN),
                             rng, bulge=rng.uniform(*G2_BULGE))
                n_rack += 1
            _g2_add_sill(ctx, A("dice", op["side"], op["storey"], own), op,
                         rng, heavy=g["sill"])
            # THE FALL ZONE: 1.2 x the panel area on the pavement, annealed
            # jagged plates or tempered dice by kind, thrown by the drop-height
            # model in `_g_drop` (mode 0.05-0.10 H, p90 0.33 H) [§12 8.9].
            # Never registered with `_g_follow`: glass that has already landed
            # stays on the pavement while the building leans into it.
            _g_add_debris(ctx, A("dice", op["side"], None), m, op["side"],
                          _b_face_pt(fr, 0.5 * (op["hua"] + op["hub"]),
                                     op["hva"], 0.0),
                          op["area"], km["debris"], density=g["sill"])
        for op in crack:
            own = op["e"]["p"].get("prim_path")
            if km["precar"] > 0 and rng.random() < km["precar"]:
                _g_add_crazed(A("crazed", op["side"], op["storey"], own),
                              A("crack", op["side"], op["storey"], own),
                              op["fr"], op["ua"], op["ub"], op["va"],
                              op["vb"], op["out"], rng)
            else:
                _g_add_corner_cracks(A("crack", op["side"], op["storey"], own),
                                     op["fr"], op["ua"], op["ub"], op["va"],
                                     op["vb"], op["out"], rng)
            if rng.random() < 0.22 * g["sill"]:
                _g2_add_sill(ctx, A("dice", op["side"], op["storey"], own), op,
                             rng, heavy=0.25)
        mats = {"open": _g_mat(ctx, "open"), "mull": _g_mat(ctx, "mullion"),
                "crazed": _g_mat(ctx, "crazed"), "dice": _g_mat(ctx, "dice"),
                "crack": ctx["mats"]["crack"], "slab": _g_mat(ctx, "slab")}
        for key, a in sorted(accs.items(), key=lambda kv: str(kv[0])):
            if _g_emit(ctx, a, mats[key[0]], "w" + key[0], owners[key]):
                n_mesh += 1
        n_out_all += len(out)
        n_crack_all += len(crack)
        ctx["notes"].append(
            "window_glass DG{0} {1} {2} ({3}): band storeys {4}-{5} on {6}, "
            "{7}/{8} openings emptied, {9} cracked".format(
                grade, glass, tag, pname, min(band), max(band),
                "+".join(bsides), len(out), len(group), len(crack)))
    ctx["notes"].append(
        "window_glass: {0} out, {1} cracked, {2} frames racked, {3} authored "
        "mesh(es)".format(n_out_all, n_crack_all, n_rack, n_mesh))
    return len(ops)


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
        # the hollow-block rubble under the dropped panels: a thin apron
        # on all four sides (the infill drops straight down at its own
        # wall line, not outward past one favoured side), so `windrow` on
        # every side. `spread_frac=0.1` is passed through so `EQ_RUBBLE=v1`
        # draws the identical low apron it always did; the 0.3-0.6 m depth
        # is already a low-windrow number (round-4 table: "spread 0.10-0.18
        # x the building dimension at h=0 ... corresponds to a low windrow,
        # ~0.3-0.6 m deep"), so v2 keeps the same draw rather than a fixed
        # constant.
        _rubble(ctx, m, "windrow", sides=("S", "E", "N", "W"),
                depth_m=rng.uniform(0.3, 0.6), spread_frac=0.1, tag="windrow")


def _corner_break(ctx, m, cx, cy, c_sides, reach, path, mat_fn):
    """Strip along each of the corner's two sides, fractured against a
    wandering radius about the corner; the rest of the slab stays one clean
    piece. Hoisted out of `r_corner_fail` (round 4) — a module-level name so
    the routing test can find it with `inspect.getsource` — with every
    former closure variable (`m`, `cx`, `cy`, `c_sides`, `reach`) now an
    explicit parameter; the body is otherwise unchanged."""
    rng = ctx["rng"]
    mats = ctx["mats"]
    btype = ctx["info"]["type"]
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
        wob = _p_wobble(rng, 1.1, max(4.0, 1.6 * rr), btype,
                        pitch=_p_pitch(ctx))      # _p_

        def judge(c, _r=rr, _w=wob):
            lx, ly = _to_local(m, c[0], c[1])
            th = math.atan2(ly - cy, lx - cx)
            return math.hypot(lx - cx, ly - cy) < _r + _w(th * _r)
        from pxr import UsdShade
        bm = UsdShade.MaterialBindingAPI(ctx["stage"].GetPrimAtPath(strip)).ComputeBoundMaterial()[0]
        # _p_: the strip is a SLAB, not masonry — prisms through its
        # full thickness, whatever the building is built of.
        st, lo = _break_split(ctx, strip, 12 + rng.randrange(5), judge, mat_fn,
                              min_volume_frac=0.0008,
                              static_mat=bm if bm else None,
                              **_p_slab_kw(ctx, rough=ROUGH_STRIP_M))
        statics += st
        loose += lo
        _a_edge_bars(ctx, st, btype, m, sd)
    return rem, statics, loose


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
                        ctx["cache"], ctx["info"]["type"], inner_p=0.4,
                        **_p_frac_kw(ctx))            # _p_ brick / prism cells
        ox, oy = _outward(m, e["side"])
        for pth in lo:
            v = rng.uniform(0.2, 0.8)
            ctx["velocity"][pth] = (ox * v, oy * v, 0.0)
        _a_dustify(ctx, lo)
        _dust_loose(ctx, lo)
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
            # the line runs UP the wall, so masonry tooths vertically —
            # _p_: on the BOND now (runs of whole courses, risers of half
            # stretchers) rather than a continuous walk
            wob = _p_wobble(rng, 0.85, max(6.0, m["top"] - m["z0"]),
                            ctx["info"]["type"], vertical=True,
                            pitch=_p_pitch(ctx))
            keep_r = d - rng.uniform(1.0, 3.0)

            def judge(c, _k=keep_r, _w=wob, _z=m["z0"]):
                lx, ly = _to_local(m, c[0], c[1])
                return math.hypot(lx - cx, ly - cy) < _k + _w(c[2] - _z)
            st, lo = _break_split(
                ctx, path, 9 + rng.randrange(4), judge, _mat_fn(ctx, tex, 0.35),
                static_mat=_clad_material(ctx["stage"], ctx["parent"],
                                          ctx["cache"], tex) if tex else None,
                **_p_frac_kw(ctx))                    # _p_
            ox, oy = _outward(m, e["side"])
            for q in lo:
                v = rng.uniform(0.3, 1.0)
                ctx["velocity"][q] = (ox * v, oy * v, 0.0)
            _dust_loose(ctx, lo)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
    fit = ctx["fit"]
    btype = ctx["info"]["type"]
    c_sides = (("S" if "S" in corner else "N"), ("E" if "E" in corner else "W"))

    for (mt, i), pth in list(fit["slabs"].items()):
        if mt != mass or i not in top_storeys or not pth:
            continue
        rem, st, lo = _corner_break(
            ctx, m, cx, cy, c_sides, reach, pth,
            lambda: (_a_mat(ctx, "timber_dusty") if btype == "urm"
                    else _a_mat(ctx, "concrete_dusty")))
        fit["slabs"][(mt, i)] = rem
        fit["all"] = [q for q in fit["all"] if q != pth] + [rem] + st
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
    # THE WHOLE ROOF BECOMES SLABS (`_a_roofify`) even though only the ones
    # near the corner get broken: a roof that is half kit tile and half
    # authored slab has a straight material seam across it.
    roofs = _a_roofify(ctx, mass)
    for box, blx, bly in roofs:
        # only slabs that reach the corner are BROKEN; the rest stay whole
        if math.hypot(blx - cx, bly - cy) > reach + 7.0 and \
                max(m["W"], m["D"]) > 12.0 and len(roofs) > 1:
            continue
        rem, st, lo = _corner_break(
            ctx, m, cx, cy, c_sides, reach, box,
            lambda: (_a_mat(ctx, "timber_dusty")
                    if (btype == "urm" and rng.random() < 0.5)
                    else _a_mat(ctx, "concrete_dusty")))
        ctx["loose"] += lo
        ctx["static_extra"] += [rem] + st
        ctx["authored"].append(rem)
    # THE FLOOR OF THE NOTCH IS A KIT SEAM. The lowest removed storey leaves
    # the modules under it with a level top edge on the slab line — the
    # horizontal half of the "rectangular part broken off". Only near the
    # corner: the rest of that storey is undamaged and should stay so.
    k0 = min(top_storeys) if top_storeys else 0
    if k0 > 0:
        _p_ragged_courses(                            # _p_ (was _a_)
            ctx, mass, k0, sides=c_sides, above=False, below=True,
            band=(0.25, 0.75), p=0.9,
            near=lambda e: math.hypot(e["lx"] - cx, e["ly"] - cy) < reach + 5.0)
    _disturb_interior(ctx, mass, top_storeys)
    _spall(ctx, mass, rate=0.06)
    # the notch's own rubble FAN on the pavement below the corner — wider at
    # the toe than at the wall, which a straight windrow (v1's only shape)
    # cannot show — over the last `reach` metres of each of the two sides
    # that meet there. `elem_h_m` is the height of the storeys the corner
    # dropped (`k0`, computed above).
    corner_elem_h_m = max(1.0, m["top"] - m["levels"][k0])
    for sd in (("S" if "S" in corner else "N"), ("E" if "E" in corner else "W")):
        L = m["W"] if sd in ("S", "N") else m["D"]
        # side coordinate runs -0.5..0.5 from SW/SE toward the far end
        # (S/N: along +x; E/W: along +y), so the corner is at +-0.5
        at_hi = (("E" in corner) if sd in ("S", "N") else ("N" in corner))
        w = min(0.5, (reach + 2.0) / L)
        along = (0.5 - w, 0.5) if at_hi else (-0.5, -0.5 + w)
        depth = 0.8 if _RUBBLE_MODE == "v2" else rng.uniform(0.6, 1.2)
        _rubble(ctx, m, "fan", sides=(sd,), depth_m=depth, along=along,
                elem_h_m=corner_elem_h_m, tag="corner_{0}{1}".format(corner, sd))


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
                    consume_pool=None, max_piece_m=None, solid_m=None):
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
    # `solid_m` is plumbed for symmetry with `_break` / `_break_split` and is
    # normally a NO-OP here: `_roof_box` authors a closed `_box`, and
    # `solidify` returns a watertight mesh untouched. It matters only if a
    # caller ever points this at a kit roof tile directly (a zero-thickness
    # quad), which is exactly the case `_roof_box` exists to avoid.
    made = fracture.fracture_prim(ctx["stage"], box, out, n_pieces=n,
                                  rng=ctx["nrng"], solid_m=solid_m,
                                  dump_tag="roofbox|" + box.rsplit("/", 1)[-1],
                                  solid_ref=_t_ref(ctx, e),
                                  mode=("plank" if timber else "uniform"),
                                  aspect=((1.4, 3.0) if timber else None),
                                  rough=0.01, verbose=False, consume=consume,
                                  consume_pool=(consume_pool if consume_pool
                                                else (1.05 if dusty else 1.6)),
                                  max_piece_m=max_piece_m,
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
        # _p_ ROUND 3: FOR MASONRY THIS IS A MECHANISM, NOT A FRACTURE.
        # Out-of-plane failure is rigid-body rocking on three horizontal
        # cracks (top, bottom, mid-height) plus vertical cracks at the corners
        # and openings; 318 Canterbury URM buildings gave "cracking that
        # delineates relatively undamaged masonry MACROBLOCKS", with
        # out-of-plane at 65 % of all observations. A standing damaged URM
        # wall is 1-4 big intact blocks — it becomes a field of bricks only
        # after a block falls. Round 2 diced the whole wall into cells, which
        # is the one thing the mechanism says not to do.
        panels_for_side = [] if _RUBBLE_MODE == "v2" else None
        if btype == "urm":
            ctx["notes"].append(_p_macroblocks(ctx, mass, side,
                                               from_storey=from_storey,
                                               panels_out=panels_for_side))
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
                    _p_zline_judge(m, side, z0, rng, btype=btype,      # _p_
                                   amp=e["h"] * 0.30, loose_above=True,
                                   pitch=_p_pitch(ctx)),
                    _mat_fn(ctx, tex, 0.35),
                    static_mat=_clad_material(ctx["stage"], ctx["parent"],
                                              ctx["cache"], tex) if tex else None,
                    **_p_frac_kw(ctx))
                for pth in lo:
                    v = 0.4 + rng.uniform(0.0, 0.8)
                    ctx["velocity"][pth] = (ox * v, oy * v, 0.05 * v)
                _dust_loose(ctx, lo)
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
                            # _p_/round 3: 0.34 was tuned to HIDE plates —
                            # a kit fragment is a solid brick chunk now, so
                            # thinning it that hard just empties the fan.
                            partial=None, consume=0.22, **_p_frac_kw(ctx))
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
            _dust_loose(ctx, lo)
            ctx["loose"] += lo
            ctx["static_extra"] += st
            e["dead"] = True
        # FAN, not a straight windrow: an out-of-plane wall is wider at the
        # toe than at the wall (v1 has no such shape and maps this onto the
        # same straight windrow it always authored). `elem_h_m` is the
        # fallen run's own height (research: windrow/fan reach ~= the fallen
        # element's own height, not a fraction of the whole building), and
        # the macroblocks the URM branch above kept whole ride along as
        # `panels` instead of scattering as generic chunks.
        spread = rng.uniform(0.22, 0.4)
        depth = rng.uniform(1.0, 1.9)
        z_fail = m["levels"][from_storey] if from_storey < len(m["levels"]) else m["z0"]
        elem_h_m = max(1.0, m["top"] - z_fail)
        _rubble(ctx, m, "fan", sides=(side,), depth_m=depth,
                spread_frac=spread, elem_h_m=elem_h_m,
                panels=(panels_for_side or ()), tag="oop_{0}".format(side))
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
            _p_ragged_courses(ctx, mass, from_storey, sides=(side,),   # _p_
                              above=False, below=True, band=(0.3, 0.9), p=0.9)
        r_droop(ctx, mass=mass, side=side, storeys=opened, p=0.45)
        _disturb_interior(ctx, mass, opened, side=side)
    _spall(ctx, mass, rate=0.07)


# ---------------------------------------------------------------------------
# SOFT-STOREY MECHANICS (round 4) — G's finding (`quake_sliced.s_soft_storey`,
# reused as maths here, NOT imported: this module must stay importable
# without `quake_sliced`'s own dependencies and the two ladders are polished
# concurrently). A soft storey is not one mechanism:
#
# DIFFERENTIAL CRUSH (60 %): the columns crush MORE on one side, so the
# block above sits on a WEDGE — the angle is not free, it is
# (r_far - r_lean) / span, which is why a 30 m frame can lean several degrees
# without its low corner going underground (the far side is a taller
# standing stub, not the same crush as the low side).
# SIDESWAY (40 %): the columns hinge top and bottom and the storey RACKS —
# the block above stays PLUMB and slides sideways (Northridge Meadows /
# Antakya), invisible to a model that only knows how to tilt.
#
# The ORIGINAL code here pivoted on the LEAN side's own base edge with
# `sign = -1.0`, which — the bug — leans the block AWAY from the side it
# names and pushes the far base edge `span * sin(lean)` into the storey
# below (2.9 m on a 30 m frame at 5.5 deg). Pivoting the FAR base edge and
# letting the lean side drop to its own (lower) residual is what actually
# produces "the block above tilts toward the named side."
SS_P_SWAY = 0.40                    # share of soft storeys that rack rather than tilt
SS_R_LEAN_FRAC = (0.15, 0.40)       # residual height on the LEAN side, x storey h
SS_R_FAR_CEIL_FRAC = 0.95           # the far side cannot be taller than it started
SS_LEAN_DEG = (2.5, 6.5)            # the tilt a differential crush is drawn for
SS_SWAY_DEG = (8.0, 25.0)           # the column rack angle of a sidesway
SS_SWAY_CRUSH_FRAC = (0.15, 0.35)   # squash ON TOP of the rack's own shortening


def _soft_storey_geometry(m, lean_side, h_st, z_lo, rng, mode=None,
                          lean_deg=None, crush_frac=None):
    """One rigid displacement for everything above a failed storey, as pure
    numbers — no pxr, so this and its maths can be checked with plain numpy
    and no stage (`tests/test_quake_flow_rubble_routing.py`).

    CRUSH: `r_lean = U(0.15, 0.40) h_st` on `lean_side`, `r_far = min(0.95
    h_st, r_lean + span sin(lean_drawn))` on the opposite side, `lean =
    asin((r_far - r_lean) / span)`. Applying `_soft_storey_matrix`'s result
    to a point translates it down by `h_st - r_far` first (every point of
    the block's own base plane, which started level at `z_lo + h_st`, is now
    level at `z_lo + r_far` — the same height as the pivot), then rotates
    `+lean` about the FAR base edge (pivot `(fx, fy, z_lo + r_far)`, axis
    along that edge) — a point on the axis does not move, and the lean
    side's base (`span` away, in the outward direction) comes out at exactly
    `z_lo + r_lean` because `span * sin(lean) = r_far - r_lean` by
    construction.

    SWAY: plumb (`deg` 0, no `pivot`), offset `h_st sin(phi)` toward
    `lean_side`, `phi = U(8, 25) deg`, dropped `h_st (1 - cos phi) +
    U(0.15, 0.35) h_st`. `r_lean == r_far` (no wedge — the whole storey
    crushes uniformly under a pure rack).

    Explicit `lean_deg`/`crush_frac` forces CRUSH — matches
    `quake_sliced.s_soft_storey`'s "the tower ladder's podium asks for a
    named angle." `mode` overrides both the draw and the forcing when given
    directly (the routing test exercises both modes without depending on
    the 60/40 draw)."""
    span = max(1.0, m["D"] if lean_side in ("S", "N") else m["W"])
    ox, oy = _outward(m, lean_side)
    ax, ay = -oy, ox
    lnx, lny = _SIDE_NORMAL[lean_side]
    fx, fy = _to_world(m, -lnx * m["W"] / 2.0, -lny * m["D"] / 2.0)
    forced = lean_deg is not None or crush_frac is not None
    if mode is None:
        mode = "crush" if (forced or rng.random() >= SS_P_SWAY) else "sway"

    if mode == "crush":
        lean_drawn = float(lean_deg) if lean_deg is not None else rng.uniform(*SS_LEAN_DEG)
        r_lean = (float(crush_frac) if crush_frac is not None
                 else rng.uniform(*SS_R_LEAN_FRAC)) * h_st
        r_far = min(SS_R_FAR_CEIL_FRAC * h_st,
                    r_lean + span * math.sin(math.radians(lean_drawn)))
        lean = math.degrees(math.asin(
            min(1.0, max(0.0, (r_far - r_lean) / span))))
        return {"mode": "crush", "r_lean": r_lean, "r_far": r_far,
                "lean_deg": lean, "pivot": (fx, fy, z_lo + r_far),
                "axis": (ax, ay, 0.0), "deg": lean,
                "translate": (0.0, 0.0, -(h_st - r_far)),
                "offset_m": 0.0, "drop_m": h_st - r_far, "phi_deg": 0.0}

    phi = rng.uniform(*SS_SWAY_DEG)
    squash = rng.uniform(*SS_SWAY_CRUSH_FRAC) * h_st
    d = h_st * math.sin(math.radians(phi))
    drop = h_st * (1.0 - math.cos(math.radians(phi))) + squash
    r = max(0.05 * h_st, h_st - drop)
    return {"mode": "sway", "r_lean": r, "r_far": r, "lean_deg": 0.0,
            "pivot": None, "axis": (ax, ay, 0.0), "deg": 0.0,
            "translate": (ox * d, oy * d, -drop), "offset_m": d,
            "drop_m": drop, "phi_deg": phi}


def _soft_storey_matrix(geo):
    """`Gf.Matrix4d` for a `_soft_storey_geometry()` dict — translate first
    (matches every other pivoted transform in this file: `M = _translate(...)
    * _rot_about(...)`, row-vector convention, translate applied before the
    pivot rotation), then rotate about `pivot`/`axis` by `deg` when the
    mechanism has one (sway has none)."""
    M = _translate(*geo["translate"])
    if geo.get("pivot") is not None and abs(geo.get("deg") or 0.0) > 1e-9:
        M = M * _rot_about(geo["pivot"], geo["axis"], geo["deg"])
    return M


def _soft_storey_residual(geo, m, lean_side, side, lx, ly):
    """The residual standing height (m) of `side`'s wall band after a soft-
    storey failure: `geo["r_lean"]` on the lean side, `geo["r_far"]` on the
    far side, and a straight interpolation along the span for the two flank
    sides — a flank wall runs the whole span, so its own local position
    tells us where between the two residuals it sits."""
    r_lean, r_far = geo["r_lean"], geo["r_far"]
    far_side = _opposite(lean_side)
    if side == lean_side:
        return r_lean
    if side == far_side:
        return r_far
    if abs(r_far - r_lean) < 1e-9:
        return r_lean
    if lean_side in ("S", "N"):
        half = m["D"] / 2.0
        t = (ly + half) / (2.0 * half) if lean_side == "S" else (half - ly) / (2.0 * half)
    else:
        half = m["W"] / 2.0
        t = (lx + half) / (2.0 * half) if lean_side == "W" else (half - lx) / (2.0 * half)
    t = min(1.0, max(0.0, t))
    return r_lean + t * (r_far - r_lean)


def r_soft_storey(ctx, storey=0, mass="main", lean_deg=None, crush_m=None,
                  twist_deg=0.0, offset_m=0.0):
    """The columns of `storey` fail; everything above drops onto the crushed
    band and leans (differential crush) or racks (sidesway) a few degrees;
    that storey's walls are crushed into a rubble collar. Two mechanisms,
    `_soft_storey_geometry` above."""
    rng, nrng = ctx["rng"], ctx["nrng"]
    info = ctx["info"]
    m = info["masses"][mass]
    lv = m["levels"]
    if storey >= len(lv):
        return
    z_lo = lv[storey]
    z_hi = lv[storey + 1] if storey + 1 < len(lv) else m["top"]
    h_st = z_hi - z_lo
    lean_side = rng.choice(["S", "E", "N", "W"])
    crush_frac = (float(crush_m) / h_st) if crush_m is not None else None
    geo = _soft_storey_geometry(m, lean_side, h_st, z_lo, rng,
                                lean_deg=lean_deg, crush_frac=crush_frac)
    crush = 0.5 * (geo["r_lean"] + geo["r_far"])
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
    courses = _p_ragged_courses(ctx, mass, storey, band=(0.3, 0.95), p=0.92)
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
                        ctx["cache"], info["type"], inner_p=0.5, consume=0.45,
                        **_p_frac_kw(ctx))            # _p_ brick / prism cells
        sx, sy = _outward(m, e["side"])
        push = rng.uniform(0.8, 2.2)
        M = _translate(sx * push, sy * push, 0.0)
        # the squash follows the WEDGE, not a single uniform factor: full
        # residual on the lean side, the (taller) far-side residual on the
        # opposite wall, interpolated on the two flanks.
        residual = _soft_storey_residual(geo, m, lean_side, e["side"],
                                         e["lx"], e["ly"])
        _squash(ctx["stage"], lo, z_lo, residual / h_st)
        _transform_prims(ctx["stage"], lo, M)
        for pth in lo:
            ctx["velocity"][pth] = (sx * rng.uniform(0.3, 1.2),
                                    sy * rng.uniform(0.3, 1.2), 0.0)
        _a_dustify(ctx, lo + st)
        _dust_loose(ctx, lo + st)
        ctx["loose"] += lo + st
        e["dead"] = True
    # THE COLLAR: the crushed storey's material squeezed out round the
    # perimeter, 1-3 m wide (Northridge Meadows, Antakya) — solid chunks,
    # since the shells above are foil. `_pile_mass` moves the pile's base to
    # `z_lo` (the failed storey's own base, not the building's `m["z0"]`)
    # while keeping every derived height/reach calculation unchanged.
    collar_depth = rng.uniform(0.5, 0.9) + crush * 0.25
    ret = _rubble(ctx, _pile_mass(m, z_lo), "windrow",
                  sides=("S", "E", "N", "W"), depth_m=collar_depth,
                  elem_h_m=h_st, tag="collar_{0}".format(storey))
    collar = ret["all"]
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
    # its columns: in a CRUSH they shatter, as today; in a SWAY the columns
    # hinge rather than break — inclined by phi toward the lean side about
    # their own base — and stay static (rack, don't rubble).
    for cpath in fit["columns"].get((mass, storey), []):
        cx_, cy_, cz_, sx_, sy_, sz_, yaw_ = _box_dims(ctx["stage"], cpath)
        if geo["mode"] == "sway":
            M = _rot_about((cx_, cy_, z_lo), geo["axis"], geo["phi_deg"])
            _transform_prims(ctx["stage"], [cpath], M)
            ctx["static_extra"].append(cpath)
            continue
        made = _break_box(ctx["stage"], cpath, 4, rng, nrng,
                          _a_mat(ctx, "concrete_dusty"), _a_mat(ctx, "dust"))
        lx_, ly_ = _to_local(m, cx_, cy_)
        residual = _soft_storey_residual(geo, m, lean_side, _side_of(m, lx_, ly_),
                                         lx_, ly_)
        _squash(ctx["stage"], made, z_lo, residual / h_st)
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
    # the drawn mechanism's ONE rigid transform (`_soft_storey_geometry` /
    # `_soft_storey_matrix`): differential crush pivots the FAR base edge and
    # lets the lean side drop to its own lower residual; sidesway racks the
    # block sideways, plumb, no rotation.
    M = _soft_storey_matrix(geo)
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
    if geo["mode"] == "crush":
        ctx["notes"].append(
            "soft_storey: storey {0} crushed to {1:.2f} m on {2} / {3:.2f} m "
            "on {4} — the block above tilts {5:.1f} deg over a {6:.1f} m "
            "span".format(storey, geo["r_lean"], lean_side, geo["r_far"],
                         _opposite(lean_side), geo["lean_deg"],
                         (m["D"] if lean_side in ("S", "N") else m["W"])))
    else:
        ctx["notes"].append(
            "soft_storey: storey {0} racked {1:.0f} deg toward {2} — the "
            "block above stays plumb, offset {3:.2f} m, down {4:.2f} m"
            .format(storey, geo["phi_deg"], lean_side, geo["offset_m"],
                    geo["drop_m"]))


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
                            15 + rng.randrange(7), rng, nrng, ctx["mats"],
                            ctx["cache"], info["type"], inner_p=0.5,
                            # _p_/round 3: was 0.62 / 1.0 m against plates
                            partial=partial, consume=0.55, max_piece_m=1.2,
                            **_p_frac_kw(ctx))        # _p_ prisms, not shards
            ox, oy = _outward(m, e["side"])
            H = max(1.0, m["top"] - m["z0"])
            for pth in lo:
                zf = (e["z"] - m["z0"]) / H
                v = 0.3 + 1.6 * zf
                ctx["velocity"][pth] = (ox * v * rng.uniform(0.5, 1.2),
                                        oy * v * rng.uniform(0.5, 1.2), 0.0)
            _a_dustify(ctx, lo + st)
            _a_lay_flat(ctx, lo + st)
            _dust_loose(ctx, lo + st)
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
                                           consume=0.5, mode="prism")  # _p_
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
                    _p_edge_judge(m, sd, d, rng, btype=info["type"]),   # _p_
                    lambda: (_a_mat(ctx, "concrete_dusty") if rng.random() < 0.6
                             else _a_mat(ctx, "dust")),
                    min_volume_frac=0.0008, static_mat=km if km else None,
                    refine_max=6, max_loose_m=2.6,
                    **_p_slab_kw(ctx, rough=ROUGH_STRIP_M))
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
        # between and around them. The re-authored slab STACK above is the
        # layer cake and stays exactly as it was; only ITS heap becomes the
        # v2 dome, with `stub_h_m` 0 (nothing survives standing here) and a
        # `budget` capping the pile's own large elements to a handful — the
        # stack already carries most of what a pancake pile shows.
        # `_pile_mass` moves the dome's BASE to `base` (which, for a wing,
        # is where the main body's own stack top is, not the wing's nominal
        # `m["z0"]`) while keeping `top - z0` unchanged for every derived
        # height/reach calculation downstream.
        crown_h = pitch * n_lv * 0.85
        spread = rng.uniform(0.3, 0.45)
        # B's `budget` has no per-KIND ("raft") cap, only a total `n_large`
        # cap (kept in priority order panel/raft/rebar/sheet/column/
        # lintel/joist) and a total `n_instances` cap — so "2-4 rafts" is
        # approximated as "<=4 large elements total", which keeps rafts
        # first since nothing here supplies panels.
        budget = {"n_large": 4} if _RUBBLE_MODE == "v2" else None
        _rubble(ctx, _pile_mass(m, base), "dome", stub_h_m=0.0,
                crown_m=crown_h, spread_frac=spread, budget=budget,
                tag="pancake_{0}".format(mt))
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


# RUBBLE IS NOT A CRATE OF BOXES. `_heap` authored every chunk with `_box`,
# which can only take a YAW — so a pile of a thousand of them is a thousand
# upright rectangular blocks, and next to the thin plates the kit shells shed
# it reads as packing crates under broken crockery (round-2 review of the DG5
# city). A lump is the same eight points with each corner pulled about and the
# whole thing turned on all three axes, which costs the same and reads as
# broken masonry.
def _a_lump(stage, path, cx, cy, cz, s, rng, mat=None, jitter=0.3):
    from pxr import Gf, Sdf, UsdGeom, Vt
    hx = s * 0.5
    hy = s * rng.uniform(0.45, 1.0) * 0.5
    hz = s * rng.uniform(0.35, 0.85) * 0.5
    # a random orientation, built from two turns rather than a full rotation
    # matrix so this stays cheap over the ~1500 chunks a dome needs
    ya, pa, ra = (rng.uniform(0, 6.2832), rng.uniform(-0.9, 0.9),
                  rng.uniform(-0.9, 0.9))
    cy_, sy_ = math.cos(ya), math.sin(ya)
    cp, sp = math.cos(pa), math.sin(pa)
    cr, sr = math.cos(ra), math.sin(ra)
    pts = []
    for dz in (-hz, hz):
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            x = dx * (1.0 + rng.uniform(-jitter, jitter))
            y = dy * (1.0 + rng.uniform(-jitter, jitter))
            z = dz * (1.0 + rng.uniform(-jitter, jitter))
            y, z = y * cp - z * sp, y * sp + z * cp
            x, z = x * cr - z * sr, x * sr + z * cr
            x, y = x * cy_ - y * sy_, x * sy_ + y * cy_
            pts.append(Gf.Vec3f(float(x), float(y), float(z)))
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    me = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    me.CreatePointsAttr(Vt.Vec3fArray(pts))
    me.CreateFaceVertexCountsAttr(Vt.IntArray([4] * 6))
    me.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
    me.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    lo_ = [min(q[k] for q in pts) for k in range(3)]
    hi_ = [max(q[k] for q in pts) for k in range(3)]
    me.CreateExtentAttr([Gf.Vec3f(*map(float, lo_)), Gf.Vec3f(*map(float, hi_))])
    UsdGeom.Xformable(me).AddTranslateOp().Set(Gf.Vec3d(cx, cy, cz))
    if mat is not None:
        _bind(stage, path, mat)
    return path


def _a_lay_flat(ctx, paths, p=0.8, tilt=(3.0, 34.0), thin_frac=0.55):
    """Turn a share of these fragments FLAT before the settle starts.

    A kit wall is an open SHELL, so every cell of it is a thin PLATE, and a
    plate dropped with an outward kick lands on edge about as often as flat:
    the DG5 masonry heap came out as a field of tiles standing at angles
    (reviewer, `A_fin5_urm/2_commercial_DG5_ne.png` — "like shattered tiles").
    A slab of masonry that slid off a wall lies down, so each piece is turned
    so its THINNEST axis points roughly up, then tumbled a little; PhysX keeps
    it there because that is already its resting orientation. Pieces that are
    not plates (`thin_frac` of the longest axis or more) are left alone."""
    from pxr import UsdGeom
    rng = ctx["rng"]
    stage = ctx["stage"]
    xf = UsdGeom.XformCache()
    n = 0
    for pth in paths:
        if rng.random() > p:
            continue
        pr = stage.GetPrimAtPath(pth)
        if not pr or not pr.IsValid():
            continue
        pts = UsdGeom.Mesh(pr).GetPointsAttr().Get()
        if not pts or len(pts) < 4:
            continue
        ext = [max(q[k] for q in pts) - min(q[k] for q in pts) for k in range(3)]
        big = max(ext) or 1.0
        thin = int(min(range(3), key=lambda k: ext[k]))
        if ext[thin] > thin_frac * big:
            continue                       # already a chunk, leave it
        c = xf.GetLocalToWorldTransform(pr).ExtractTranslation()
        piv = (c[0], c[1], c[2])
        # thin axis -> +Z
        if thin == 0:
            M = _rot_about(piv, (0.0, 1.0, 0.0), 90.0)
        elif thin == 1:
            M = _rot_about(piv, (1.0, 0.0, 0.0), 90.0)
        else:
            M = _rot_about(piv, (0.0, 0.0, 1.0), 0.0)
        a = rng.uniform(0.0, 6.2832)
        M = M * _rot_about(piv, (math.cos(a), math.sin(a), 0.0),
                           rng.uniform(*tilt)) \
              * _rot_about(piv, (0.0, 0.0, 1.0), rng.uniform(0, 360))
        _transform_prims(stage, [pth], M)
        n += 1
    return n


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
        _a_lump(ctx["stage"], path, wx, wy, z, s, rng, _mat(dusty))
        made.append(path)

    if fill:
        reach = spread_frac * Hm
        RX, RY = W / 2.0 + reach, D / 2.0 + reach
        vol = (math.pi * RX * RY * h) / 2.0
        # DENSITY: heap chunks are STATIC colliders, so they are far cheaper
        # than a rigid body and they are the only thing that can outnumber the
        # shell plates. 1.6x the old density and a 4200 ceiling.
        n = int(min(5600, max(200, vol * 1.4 / 0.9)))
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
        # THE FINES SETTLE ON TOP. A scatter of small dust-coloured lumps ON
        # the surface of the dome, which is what makes a pile read as a pile
        # rather than as a heap of objects.
        if h > 0.5:
            # A SKIN, NOT A SPRINKLE. These sit ON the dome's surface and they
            # are what the camera sees first, so there have to be enough of
            # them to hold the surface between the shell fragments that land on
            # top: 0.42 n rather than 0.22, brick-sized, and drawn almost
            # entirely from the dust tints (the fines settle last and on top —
            # brick stays lower down, which is the mix the surveys describe).
            for k in range(int(n * 0.42)):
                u, v = rng.uniform(-1, 1), rng.uniform(-1, 1)
                r = math.hypot(u, v)
                if r > 1.0:
                    continue
                sz = 0.15 + 0.33 * rng.random() ** 1.5
                _chunk(u * RX, v * RY,
                       base + h * max(0.0, 1.0 - r ** 1.6) + sz * 0.45, sz, 0.85)
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


# ---------------------------------------------------------------------------
# RUBBLE v2 (round 4) — route a collapse recipe's pile through
# `quake_rubble.plan_pile` + `quake_rubble_usd.author` (a heightfield mound,
# a handful of large recognisable elements, a few PointInstancer scatters)
# instead of `_heap`'s thousand-box crate. `EQ_RUBBLE` (default "v2"; "v1" =
# the round-3 `_heap` path) is read ONCE at import so a whole run is one
# version. `_heap` ITSELF is not touched by any of this — `fire_collapse.py`
# (another live session) calls it directly, and every call site below goes
# through `_rubble` so the version switch lives in exactly one place.
# ---------------------------------------------------------------------------
_RUBBLE_MODE = _os.environ.get("EQ_RUBBLE", "v2").strip().lower()
if _RUBBLE_MODE not in ("v1", "v2"):
    _RUBBLE_MODE = "v2"


def _pile_mass(m, base_z):
    """A shallow copy of mass dict `m` whose pile sits at `base_z` instead of
    `m["z0"]` — a pancake wing settling onto the stack the main body already
    left, a soft-storey collar authored at the failed storey's own base
    rather than the building's. `top` is shifted by the same delta so
    `top - z0` (every derived height/reach calculation in both `_heap` and
    `quake_rubble.plan_pile` alike) is exactly what it was before the shift;
    `m` itself is returned unchanged when `base_z` already equals `m["z0"]`."""
    base_z = float(base_z)
    delta = base_z - float(m["z0"])
    if abs(delta) < 1e-9:
        return m
    m2 = dict(m)
    m2["z0"] = base_z
    m2["top"] = float(m["top"]) + delta
    return m2


def _rubble(ctx, m, kind, sides=None, along=None, depth_m=None, offset_m=0.0,
           stub_h_m=0.0, panels=(), elem_h_m=None, crown_m=None,
           spread_frac=None, tag="rubble", budget=None, seed_tag=""):
    """One collapsed mass's pile, `EQ_RUBBLE`-gated.

    v2: `quake_rubble.plan_pile(m, btype, rng, kind=kind, ...)` (pure
    numpy/python — agent B's planner, polished concurrently, never edited
    here) plans the mound/large-elements/instance-scatters, then
    `quake_rubble_usd.author(...)` (agent C's emitter) writes them to the
    stage. `ret["all"]` joins `ctx["authored"]`, `ret["static"]` joins
    `ctx["static_extra"]`, and `plan["stats"]` (plus `kind`/`sides`/`tag`)
    is recorded under `ctx["rubble"]` for the bake launcher (round-4 plan,
    package I: `fall_sides` / `reach_m` / `crown_m` into the manifest).

    v1: maps onto the old `_heap` box-crate pile — `kind="dome"` ->
    `fill=True` with `crown_m`/`spread_frac` as the peak height/spread;
    `kind in ("windrow", "fan")` -> `fill=False` with
    `sides`/`along`/`depth_m`/`offset_m` (v1's `_heap` has no separate "fan"
    shape, so both kinds map onto the same straight windrow it always had —
    the widening is a v2-only refinement). `stub_h_m`/`panels`/`elem_h_m`/
    `budget` have no v1 analogue and are ignored. `_heap` is NOT modified
    (`fire_collapse.py`, another live session, calls it directly), so
    `EQ_RUBBLE=v1` reproduces today's pile exactly.

    Returns a dict shaped like `quake_rubble_usd.author`'s: "mound", "apron",
    "static", "instancers", "large", "all". v1 fills only "static"/"all"
    (the flat list `_heap` made — `_heap` already appended it to
    `ctx["authored"]`/`ctx["static_extra"]` itself, so this does not do so
    again for v1)."""
    if _RUBBLE_MODE == "v1":
        base = float(m["z0"])
        h = float(crown_m or 0.0)
        sf = float(spread_frac if spread_frac is not None else 0.28)
        if kind == "dome":
            made = _heap(ctx, m, base, h, sf, fill=True, tag=tag)
        elif kind in ("windrow", "fan"):
            made = _heap(ctx, m, base, 0.0, sf, fill=False, sides=sides,
                        depth_m=depth_m, along=along, tag=tag,
                        offset_m=offset_m)
        else:
            raise ValueError("_rubble: unknown kind {0!r}".format(kind))
        return {"mound": None, "apron": None, "static": list(made),
                "instancers": [], "large": [], "all": list(made)}

    from . import quake_rubble, quake_rubble_usd

    rng = ctx["rng"]
    btype = ctx["info"]["type"]
    plan = quake_rubble.plan_pile(
        m, btype, rng, kind=kind, crown_m=crown_m, spread_frac=spread_frac,
        sides=sides, along=along, depth_m=depth_m, offset_m=offset_m,
        plate_ok=lambda x, y: _c_ok(ctx, x, y), stub_h_m=stub_h_m,
        panels=panels, budget=budget, seed_tag=seed_tag, elem_h_m=elem_h_m)
    ret = quake_rubble_usd.author(ctx["stage"], ctx["parent"], plan,
                                  mats=ctx["mats"],
                                  tag="{0}_{1}".format(ctx["tag"], tag),
                                  uid=lambda: _uid(ctx))
    ctx["authored"] += ret["all"]
    ctx["static_extra"] += ret["static"]
    stats = dict(plan["stats"])
    stats.update({"kind": kind, "sides": list(sides) if sides else None,
                  "tag": tag})
    ctx.setdefault("rubble", []).append(stats)
    return ret


# ---------------------------------------------------------------------------
# LOOSE-FRAGMENT DUST (round 5, `EQ_RUBBLE=v2` only). The rubble-v2 PILE
# (mound/apron/instancer scatter, `_rubble` above) already carries its own
# dust tint — `quake_rubble_usd._rubble_look`/`_textured_debris_look` tint
# every material THAT MODULE authors. What was still pristine is the SHELL
# side: a wall/slab/column fragment a collapse recipe fractures off the
# STANDING building keeps the standing building's own bound material.
# `_chunk_material`/`_mat_fn` (used by every `_break`/`_break_split` call
# below) draw straight from `materials()`'s shared palette, and some of that
# palette is a bare Nucleus/megascans REFERENCE with NO tint ever applied —
# `mats["brick"]` -> `Brick_Wall_Worn.usda`, `mats["concrete"]` ->
# `Worn_Pavement.usda` (see `materials()` above) — or a `_clad_material`
# cladding photo, also untinted. `_a_dustify` already re-tints a SHARE of a
# collapse's fragments, but only by SWAPPING a known palette identity for a
# `_dusty` twin, and its swap table has no entry for `brick` or for a
# cladding photo at all (see that function's own docstring for the table).
# Measured on `~/docker/isaac-sim/logs/eq500_gui/b2_office_DG5_obl.png` and
# `~/docker/isaac-sim/logs/r4_commercial/0_commercial_DG5_close.png`: the
# pile reads as a heap of clean toy bricks / pale grey boxes, not broken
# debris (the user, round 5: "too animated and not real").
#
# ONLY the call sites below whose fragments can actually draw a PRISTINE
# palette entry or a cladding photo are wired to this (`_chunk_material`/
# `_mat_fn`) — a fragment authored entirely from an already-`_dusty` `_a_mat`
# flat colour (a timber roof deck, floor joists, a plaster partition, a
# crushed column) is left alone: `diffuse_tint` MULTIPLIES, so tinting an
# already-dark calibrated flat colour a second time is the exact near-black
# bug round 4 already hit once (`_material_for_look`'s docstring) and round 2
# hit before that (`planks.wood_material`) — not a reason to do it a third
# time here.
# ---------------------------------------------------------------------------
_DUST_TINT = {
    # brick/stone under settled mortar dust: greyer, darker, NOT pink — a
    # neutral multiplier over a warm brick photo keeps its hue, so this also
    # desaturates.
    "urm": ((0.62, 0.55, 0.50), 0.35),
    # concrete is already fairly neutral; darkening alone reads as dusty.
    "rc": ((0.55, 0.54, 0.52), 0.0),
}
_DUST_SKIP_P = 0.15    # left on the fragment's OWN material — a pile that is
                       # ALL dusted reads as one flat texture, same as one
                       # that is not dusted at all; the mix is what reads as
                       # broken debris on a heap of loose pieces.


def _dust_tint_for(ctx):
    """(tint, desaturation) for `_dust_loose`, by `ctx["info"]["type"]` —
    `rc_glass` (a concrete-framed tower) falls onto the `rc` (concrete)
    tint, the closer of the two."""
    return _DUST_TINT.get(ctx["info"]["type"], _DUST_TINT["rc"])


def _dust_copy(ctx, mats, src_mat, tint, desat):
    """One dusted COPY of `src_mat`, cached in `mats["dust_" + <src's own
    prim name>]` — `<ctx parent>/QuakeLooks/dust_<name>` on the stage, the
    same cache-in-`ctx["mats"]` discipline `_a_mat`/`_clad_material` already
    keep so a pile of hundreds of fragments authors only a handful of extra
    materials.

    `Sdf.CopySpec` duplicates whatever is actually authored at the source
    prim's own path: a bare `references` arc for a megascans reference
    (`materials()`'s `brick`/`concrete`/`soil` — the copy composes the SAME
    referenced `Shader` child at its own new path, so tinting it below never
    touches the original), or a full `Shader` subtree for anything this
    module authors itself (`_a_mat`, `_clad_material`, `_t_core_mat`,
    `planks.wood_material`). This is never a KIT-MODULE prim — `bake.py`'s
    "`assetInfo` poisons `CopySpec`" trap is specifically about referenced
    KIT HOUSE MODULE meshes/materials/subsets, and nothing bound to a
    fragment here is one — but the copy is still guarded end to end and
    returns `None` on any failure, exactly like an unresolvable material:
    the caller leaves that fragment on its original binding rather than
    breaking the bake.

    `quake_rubble_usd._apply_diffuse_tint` (imported, not re-implemented)
    then multiplies the copy's `diffuse_tint` / sets `albedo_desaturation` —
    writing `diffuse_color_constant` instead would be the silent no-op that
    function's own docstring documents (a bound `diffuse_texture` replaces
    it outright, so a plain tint on the constant never reaches the screen
    once a photo is bound)."""
    from pxr import Sdf, UsdShade
    from . import quake_rubble_usd

    stage = ctx["stage"]
    src_path = src_mat.GetPath()
    name = src_path.name or "mat"
    key = "dust_" + name
    cached = mats.get(key)
    if cached is not None:
        return cached

    scope = Sdf.Path("{0}/QuakeLooks".format(ctx["parent"]))
    dst_path = scope.AppendChild("dust_" + name)
    n = 0
    while stage.GetPrimAtPath(dst_path).IsValid():
        n += 1
        dst_path = scope.AppendChild("dust_{0}_{1}".format(name, n))

    try:
        stage.DefinePrim(scope, "Scope")
        Sdf.CreatePrimInLayer(stage.GetRootLayer(), dst_path)
        ok = Sdf.CopySpec(stage.GetRootLayer(), src_path,
                          stage.GetRootLayer(), dst_path)
    except Exception:
        ok = False
    if not ok:
        return None

    dst_mat = UsdShade.Material.Get(stage, dst_path)
    if not dst_mat or not dst_mat.GetPrim().IsValid():
        return None
    sh = UsdShade.Shader.Get(stage, str(dst_path) + "/Shader")
    if not sh or not sh.GetPrim().IsValid():
        return None
    try:
        quake_rubble_usd._apply_diffuse_tint(stage, str(dst_path), tint, desat)
    except Exception:
        return None

    mats[key] = dst_mat
    return dst_mat


def _dust_loose(ctx, paths, tint=None, desat=None, skip_p=_DUST_SKIP_P):
    """Bind a DUSTED COPY of each fragment's own bound material onto it,
    `strongerThanDescendants`, `EQ_RUBBLE=v2` ONLY — v1 must stay byte-
    identical to round-3's `_heap`/`_a_dustify` pile, so every call site
    below is itself wrapped in `if _RUBBLE_MODE == "v2":`.

    For every prim in `paths`: resolves whatever material is ACTUALLY bound
    to it (`UsdShade.MaterialBindingAPI.ComputeBoundMaterial` — a referenced
    megascans material, a `_clad_material` cladding photo, an `_a_mat` flat
    colour, irrelevant which) and rebinds a copy of THAT material, darkened/
    desaturated by `tint`/`desat` (by construction type, `_dust_tint_for`,
    when not given explicitly). `strongerThanDescendants` beats not just the
    asset's own binding but any `_t_core_bind`/`_break_split` CORE SUBSET the
    fragment carries, so the whole fragment — the original cladding face
    AND the invented cut/core faces — comes out ONE uniform dust tone. That
    is also the physically right call: a fragment lying in a pile is dusty
    on every exposed face, not only the one that used to be the façade.

    A random `skip_p` share of fragments is left on their OWN original
    material — an untouched pile is one texture, a fully-dusted one is
    another; neither alone reads as broken debris, only the mix does.

    The standing (unfractured) shell is never touched — this only ever
    rebinds a prim already in the caller's own loose-fragment list. Any
    fragment whose material cannot be resolved, or copied (an unusual
    graph `Sdf.CopySpec` chokes on), is skipped silently."""
    if _RUBBLE_MODE != "v2" or not paths:
        return
    if tint is None or desat is None:
        auto_tint, auto_desat = _dust_tint_for(ctx)
        tint = auto_tint if tint is None else tint
        desat = auto_desat if desat is None else desat
    from pxr import UsdShade
    stage = ctx["stage"]
    rng = ctx["rng"]
    mats = ctx["mats"]
    for pth in paths:
        if not pth or rng.random() < skip_p:
            continue
        pr = stage.GetPrimAtPath(pth)
        if not pr or not pr.IsValid():
            continue
        try:
            src = UsdShade.MaterialBindingAPI(pr).ComputeBoundMaterial()[0]
        except Exception:
            continue
        if not src or not src.GetPrim().IsValid():
            continue
        dusted = _dust_copy(ctx, mats, src, tint, desat)
        if dusted is None:
            continue
        try:
            UsdShade.MaterialBindingAPI(pr).Bind(
                dusted, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        except Exception:
            continue


def _module_size(m, e):
    """Approximate (sx, sy, sz) for one classified kit element — the round-4
    rubble planner's bury-depth math (`quake_rubble.rotated_extent`) only
    needs a placement-scale size, not a post-`fracture.solidify` measurement.
    `ub.PIECES[e["name"]]` gives it directly; `sy` is 0 on a handful of
    zero-thickness kit entries (the plain `_Wall` variants), so those fall
    back to a plausible wall thickness rather than a degenerate box."""
    from detail import urban_building as ub
    meas = ub.PIECES.get(e.get("name"))
    if meas:
        sx, sy, sz = float(meas[0]), float(meas[1]), float(meas[2])
        return (sx, sy if sy > 0.05 else 0.3, sz)
    return (float(m.get("module", 4.0)), 0.3, float(e.get("h", 3.0)))


def _pick_opening_panels(ctx, mass, m, storey=1, n=(1, 2)):
    """1-2 whole wall modules at `storey` with window/door openings, picked
    BEFORE the fracture loop runs and left standing so `_rubble`'s "large"
    panels can lay them, still whole, on the pile (round-4 design table row
    2: "1-3 wall PANELS ... kept whole"). Marks each chosen element dead (so
    the fracture loop below skips it) and returns [(prim_path, (sx,sy,sz))].

    Prefers modules whose kit name reads as a window/facade band (the
    fracture ladder has no per-piece "has an opening" flag on this kit —
    that is a `quake_sliced`/GAC-slice concept, not a `urban_building` one —
    so this is a name-pattern heuristic, not a measurement) and falls back
    to any wall module at that storey when none match."""
    rng = ctx["rng"]

    def _cand():
        return [e for e in _els(ctx, mass=mass, role="wall", storey=storey)
                if e["p"].get("prim_path")]

    pool = [e for e in _cand() if any(
        k in (e.get("name") or "") for k in ("Facade", "acade", "window", "Window"))]
    if not pool:
        pool = _cand()
    if not pool:
        return []
    k = min(len(pool), 1 if len(pool) < 2 else rng.randint(n[0], n[1]))
    rng.shuffle(pool)
    out = []
    for e in pool[:k]:
        out.append((e["p"]["prim_path"], _module_size(m, e)))
        e["dead"] = True
    return out


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
        # v2 ONLY: pick 1-2 storey-1 wall modules with openings BEFORE the
        # fracture loop, so they can ride the pile whole as `_rubble` panels
        # instead of being cut up like every other wall (round-4 design
        # table: "1-3 wall PANELS ... kept whole"). Never called in v1 —
        # `_pick_opening_panels` draws from `ctx["rng"]`, and v1 must stay
        # byte-for-byte the round-3 behaviour.
        panel_entries = _pick_opening_panels(ctx, mt, m) if _RUBBLE_MODE == "v2" else []
        stub_hs = []
        for e in list(_els(ctx, mass=mt)):
            if e["role"] == "roof":
                # A MASONRY building's roof is a timber deck, and it comes
                # apart into boards and rafters, not into concrete plates:
                # many small pieces, most of them lost into the heap.
                _deck = _break_box_like(ctx, e, 52, timber=True,
                                        consume=0.85, dusty=True,
                                        max_piece_m=1.0)
                _a_lay_flat(ctx, _deck)
                ctx["loose"] += _deck
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
                stub_frac = rng.uniform(0.25, 0.62)
                stub_hs.append(e["h"] * stub_frac)
                z0 = e["z"] + e["h"] * stub_frac
                tex = damage.bound_texture(stage, path)
                st, lo = _break_split(
                    ctx, path, 10 + rng.randrange(4),
                    _p_zline_judge(m, e["side"], z0, rng, btype=info["type"],
                                   amp=e["h"] * 0.30, loose_above=True,
                                   pitch=_p_pitch(ctx)),               # _p_
                    _mat_fn(ctx, tex, 0.5),
                    static_mat=_clad_material(ctx["stage"], ctx["parent"],
                                              ctx["cache"], tex) if tex else None,
                    **_p_frac_kw(ctx))
            else:
                # SMALLER, FAR FEWER, AND CAPPED. 8-12 cells on a kit module
                # sheds 1.2-2 m plates; they land last, ON the crown of the
                # heap, and the pile reads as shattered tiles over rubble
                # however good the rubble under them is — no number of authored
                # lumps can win that, because the plates are always on top.
                # The only lever that works is how MANY there are: 18-24 cells
                # put the median near 0.8 m, the 0.9 m cap takes the big end
                # (it keeps the smallest half, so cap and consume compose), and
                # `consume` 0.75 leaves a quarter of them — enough to read as
                # recognisable façade among the rubble, too few to cover it.
                # The material is not lost, it is in the heap.
                # _p_: `brick` cells, so what lands is brick CLUSTERS with
                # joint faces rather than plates of wall. The cap and the
                # consume stay: they are what keeps the crown of the heap
                # rubble rather than façade.
                # v2: raised further (0.66 -> 0.8 consume, 1.2 -> 0.9 m cap)
                # now that the pile's own SURFACE carries the mass — the
                # crown only needs enough shell plate to read as
                # recognisable façade, not to cover a heap of toy blocks.
                consume_ = 0.8 if _RUBBLE_MODE == "v2" else 0.66
                max_piece_ = 0.9 if _RUBBLE_MODE == "v2" else 1.2
                st, lo = _break(stage, ctx["parent"], e, ctx["tag"],
                                18 + rng.randrange(7), rng, nrng, ctx["mats"],
                                ctx["cache"], info["type"], inner_p=0.45,
                                # _p_/round 3: 0.75 / 0.9 m were tuned
                                # against 1.2-2 m foil PLATES landing on the
                                # crown. The cells are brick clusters now, so
                                # the cap can rise and fewer need thinning.
                                partial=None, consume=consume_,
                                max_piece_m=max_piece_, **_p_frac_kw(ctx))
            ox, oy = _outward(m, e["side"])
            for pth in lo:
                zf = min(1.0, max(0.0, (e["z"] - m["z0"]) / H))
                # GENTLER THAN IT WAS (0.3 + 1.8 zf). A plate thrown at 2 m/s
                # arrives at the pile edge-first and stays there; a total
                # collapse drops nearly straight down anyway — the outward fan
                # belongs to `out_of_plane`, not to this.
                v = 0.2 + 0.9 * zf
                ctx["velocity"][pth] = (ox * v * rng.uniform(0.3, 1.0),
                                        oy * v * rng.uniform(0.3, 1.0),
                                        -0.4 * rng.random())
            _a_dustify(ctx, lo, p=0.9)
            _a_lay_flat(ctx, lo)
            _dust_loose(ctx, lo)
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
                                  mode="plank", aspect=(1.8, 3.6), consume=0.88,
                                  consume_pool=1.05, max_piece_m=1.0)
                # boards lie DOWN, and a board that came through a collapse is
                # under the dust like everything else
                _a_dustify(ctx, made, p=0.5)
                _a_lay_flat(ctx, made, p=0.9)
                ctx["loose"] += made
        for pth in fit["partitions"]:
            if "_{0}_".format(mt) in pth:
                ctx["loose"] += _break_box(stage, pth, 8, rng, nrng,
                                           _a_mat(ctx, "plaster_dusty"),
                                           consume=0.45, mode="prism")  # _p_
        for (pm, i), props in fit["props"].items():
            if pm == mt:
                _a_bury_props(ctx, props, m["z0"], H * 0.28)
        # THE PILE IS H/3 (FEMA's 0.33 air-space factor; Amatrice LiDAR gives
        # ~5 m heaps for 2-4 storey stone). Masonry spreads 0.4-0.7 H.
        # `stub_h_m` is the AVERAGE kept-stub height just fractured above, so
        # the mound doesn't dip below the standing stub it leans against;
        # `panel_entries` are the whole storey-1 modules picked before the
        # loop ran. v2's planner emits its own lintels/quoins/sills, so
        # `_p_lintels`'s own boxes only run in v1.
        crown_h = H * 0.28
        spread = rng.uniform(0.2, 0.34)
        stub_h_m = (sum(stub_hs) / len(stub_hs)) if stub_hs else 0.0
        _rubble(ctx, m, "dome", stub_h_m=stub_h_m, panels=panel_entries,
                crown_m=crown_h, spread_frac=spread,
                tag="collapse_{0}".format(mt))
        if _RUBBLE_MODE == "v1":
            # _p_: the LINTELS, QUOINS and sill stones — the only large
            # pieces in a masonry pile, and the acceptance test's own
            # criterion ("largest piece = a lintel/quoin monolith, not a
            # wall shard"). They bypass the fracture because that is what
            # they are: single dressed stones that were never bonded into
            # the field.
            _p_lintels(ctx, m, base=m["z0"] + H * 0.10)


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
        _bind(ctx["stage"], path, _c_ground_look(ctx, cx, cy, "silt"))
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
# (texture, TINT, roughness, repeats per metre, desaturation)
#
# THE TINT HAS TO GO ON `diffuse_tint`. `damage._pbr` puts it on
# `diffuse_color_constant`, and `planks.wood_material`'s comment says that
# multiplies the map — it does not. OmniPBR.mdl (kit/mdl/core/Base) is
# explicit: `diffuse_color_constant` is "the albedo base color" (what the map
# REPLACES), `diffuse_tint` is "multiplied over the final albedo color", and
# `albedo_desaturation` "desaturates the albedo map". Three benches at tint
# 0.40, 0.33 and 0.22 rendered the soil identically bright before this was
# found. Desaturation is not optional either: a neutral multiplier cannot take
# the orange out of an orange mud map, it only makes a darker orange.
# The tints below are multipliers over the map as it renders untinted, and
# they are BRACKETED rather than guessed: at 1.0 (the benches before the fix)
# the soil read as orange paint, at 0.09 (the bench that stacked both
# multipliers) as black mud. These sit near the geometric mean of the two.
_C_TEX = {
    "soil":  ("megascans/Soil_Mud/T_pjuph20_1K_B.jpg", (0.34, 0.33, 0.32), 0.98, (0.70, 0.70), 0.55),
    "silt":  ("megascans/Dirt_Rough/T_yd0lfcqcc_1k_B.png", (0.42, 0.42, 0.43), 0.96, (0.55, 0.55), 0.65),
    # NOT Worn_Pavement: its map carries green moss in the joints, and a 1.2 m
    # kerb block at 0.38 repeats/m showed one big square of it — a row of them
    # along a wall read as green mosaic tiles. Damaged_Asphalt is a plain grey
    # cracked surface; brightened it is concrete, darkened it is the road.
    "pave":  ("megascans/Damaged_Asphalt/T_vizcebf_2K_B.png", (0.50, 0.50, 0.49), 0.90, (0.55, 0.55), 0.25),
    "asph":  ("megascans/Damaged_Asphalt/T_vizcebf_2K_B.png", (0.32, 0.32, 0.32), 0.92, (0.30, 0.30), 0.35),
    "raft":  ("megascans/Damaged_Asphalt/T_vizcebf_2K_B.png", (0.30, 0.30, 0.30), 0.95, (0.28, 0.28), 0.35),
    "brick": ("megascans/Brick_Wall_Worn/T_sexkaitb_1K_B.jpg", (0.48, 0.40, 0.36), 0.92, (0.70, 0.70), 0.15),
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
    rel, rgb, rough, scale, desat = _C_TEX[key]
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
            sh.CreateInput("diffuse_tint",
                           Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
            sh.CreateInput("albedo_desaturation",
                           Sdf.ValueTypeNames.Float).Set(float(desat))
    except Exception as exc:
        print("[quake_flow] ground look {0} unavailable ({1})".format(key, exc))
        got = mats.get(_C_FALLBACK[key])
    mats[k] = got
    return got


# ---------------------------------------------------------------------------
# THE LOCAL GROUND CLASS (round 5, WP E). User: "For like bent/broken
# sidewalk/asphalt use the material of what's near where the broken ground is
# placed, not the grassy sidewalk one." Every pavement/kerb/spill piece below
# used to pick its look from a plain coin (`"pave" if rng.random() < 0.5 else
# "asph"`) or a fixed default, with no idea whether the ground under it is
# actually a carriageway, a sidewalk, a paved block interior or a lawn.
#
# `ctx["ground_at"]` is an OPTIONAL `(x_world, y_world) -> "road"|"sidewalk"|
# "paved"|"grass"` sampler — `disaster.ground_class.GroundClass.at`, built
# once per city from `city_layout` and wired in by `quake.py` at
# assembly time (`_c_tilt_ground`, `ground_effects`). Neither the per-building
# bench nor the archetype bake ever sets it, and `_c_ground_look` is written
# so that absence is a NO-OP: the exact same rng draw in the exact same place
# as before this existed, byte-identical materials.
# ---------------------------------------------------------------------------
def _c_ground_look(ctx, x, y, default_key, rng_coin=None):
    """The look for one authored ground piece centred at world `(x, y)`.

    With `ctx["ground_at"]` wired in: `look_for(ground_at(x, y))` — the
    piece's own material follows what is actually there (asphalt on a road,
    pavement on a sidewalk or a paved block, soil under a lawn).

    Without it (the ordinary bench/bake path, and any city that never built a
    `GroundClass`): `default_key`, or — if the call site still draws its own
    coin between two looks (`_c_pave_break`'s historic "pave" vs "asph") —
    `rng_coin()`, a zero-argument callable returning the key. `rng_coin` is
    called ONLY on this branch, so a caller with no `ground_at` still spends
    its `rng` draw in exactly the place it always did; the mixed-in ground
    class is a NEW execution path, not a reinterpretation of the old one, so
    it owes that draw nothing."""
    ga = ctx.get("ground_at")
    if ga is not None:
        from . import ground_class
        return _c_look(ctx, ground_class.look_for(ga(x, y)))
    return _c_look(ctx, rng_coin() if rng_coin is not None else default_key)


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
    # A caller with no opinion (no `mat`) gets the ground-class-aware look —
    # `_c_ground_response`'s high-side "pit" spill and `_c_overturn_ground`'s
    # spill both call this way — falling back to plain "soil" exactly as
    # before when `ctx["ground_at"]` is absent.
    _bind(ctx["stage"], path,
         mat if mat is not None else _c_ground_look(ctx, cx, cy, "soil"))
    ctx["authored"].append(path)
    return path


def _c_clods(ctx, m, samples, n, at=None, spread=0.55, size=(0.16, 0.55),
             z_frac=(0.45, 1.0), stone_p=0.18, tag="clod", big_p=0.22,
             big_mult=(1.4, 2.2), tilt_p=0.55, tilt_deg=(6.0, 24.0)):
    """Lumps of turned-up earth straddling a line. The round-1 berm was a
    smooth extrusion and read as a moulding; a crest needs loose material ON
    it, overlapping, at more than one size, to read as earth.

    ROUND 5, WP E: two things ported from `scour_relief._clod` (whose own
    docstring records taking these same lessons FROM this file's round-2
    bench notes, in the other direction — porting them back closes the
    loop):

      * `big_p`/`big_mult` draw a SECOND, larger size band on top of the
        base `size` range, so one call already scatters small grit next to
        the occasional bigger stone. This file's call sites already layer
        separate `_c_clods` passes at different `size` ranges on one crest
        (`_c_ground_response`'s "crest" + "toe" clods) — this is the
        within-one-call half of the same idea, and it means even a single
        pass reads as more than one size class.
      * `tilt_p`/`tilt_deg` tip a share of the lumps about a random
        HORIZONTAL axis after placing them (this file's own established tip
        idiom — `_buckled_pavement`, `_c_overturn_ground`'s footing stubs —
        rather than `scour_relief._clod`'s from-scratch pitch+roll seating
        math, which assumes a merged-mesh author path this file does not
        use). An untipped box sitting bolt upright on its own flat base is
        the "toy brick" look the round-5 review already flagged on the
        fracture fragments; a clod that fell out of a heave and came to
        rest on its own corner is not level. No z-reseat after the tip —
        the existing `sz * 0.2` lift already gives a small clod room to
        settle a little further into the ground on its low corner, which is
        the safe direction (a lump slightly bedded in reads as settled; one
        floating reads as a bug)."""
    rng = ctx["rng"]
    made = []
    if not samples:
        return made
    for _k in range(int(n)):
        lx, ly, nx, ny, dc, h, reach = samples[rng.randrange(len(samples))]
        d = (dc if at is None else at) + rng.gauss(0.0, spread)
        sz = rng.uniform(*size)
        if rng.random() < big_p:
            sz *= rng.uniform(*big_mult)
        px, py = lx + nx * d, ly + ny * d
        wx, wy = _to_world(m, px, py)
        if not _c_ok(ctx, wx, wy):
            continue
        mat = (_c_look(ctx, "soil" if rng.random() < 0.7 else "silt")
               if rng.random() > stone_p else
               _c_look(ctx, "pave" if rng.random() < 0.6 else "brick"))
        path = "{0}/{1}_{2}_{3}".format(ctx["parent"], tag, ctx["tag"], _uid(ctx))
        z = m["z0"] + h * rng.uniform(*z_frac) + sz * 0.2
        _box(ctx["stage"], path, wx, wy, z,
             sz, sz * rng.uniform(0.6, 1.1), sz * rng.uniform(0.45, 0.9),
             rng.uniform(0, 180), mat)
        if rng.random() < tilt_p:
            aa = rng.uniform(0.0, 6.2832)
            _transform_prims(
                ctx["stage"], [path],
                _rot_about((wx, wy, z), (math.cos(aa), math.sin(aa), 0.0),
                          rng.uniform(*tilt_deg)))
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
            tdeg, _c_ground_look(ctx, wx, wy, None,
                                 lambda: "pave" if rng.random() < 0.5 else "asph"),
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
             base + rng.uniform(-9, 9), _c_ground_look(ctx, wx, wy, "pave"))
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
            _c_ground_look(ctx, wx, wy, None,
                          lambda: "pave" if rng.random() < 0.5 else "asph"),
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
                     _c_ground_look(ctx, wx, wy, None,
                                   lambda: "pave" if rng.random() < 0.5 else "asph"),
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
                          mat=_c_ground_look(
                              ctx, wx, wy, None,
                              lambda: "silt" if rng.random() < 0.5 else "soil"),
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
                       spill=True, bounds=None, ground_at=None):
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

    `ground_at` (round 5, WP E): an optional `(x, y) -> "road"|"sidewalk"|
    "paved"|"grass"` sampler (`disaster.ground_class.GroundClass.at`) —
    stashed onto `ctx["ground_at"]` for every pavement/kerb/spill piece
    authored below to read via `_c_ground_look`. `None` (the default, and
    the only thing the bench or the bake ever pass) leaves `ctx` exactly as
    it always was.
    """
    ctx = ctx_or_stage if isinstance(ctx_or_stage, dict) else _c_ctx(
        ctx_or_stage, parent, mats, rng, tag)
    if ground_at is not None:
        ctx["ground_at"] = ground_at
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
                          mat=_c_ground_look(
                              ctx, wx, wy, None,
                              lambda: "silt" if rngl.random() < 0.6 else "soil"),
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
    # +angle about (-oy, ox) lays the top OUTWARD over `side` (the edge it
    # pivots on). The first version used -angle, which folded the block back
    # over its own footprint toward the opposite side — so the crater, the
    # shove and `quake._blocked`'s sweep all disagreed with the fall (found
    # by agent C's matrix check, 2026-08-27).
    M = _rot_about((px, py, m["z0"]), (ax, ay, 0.0), abs(angle))
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
    # the landing windrow, `spread_frac=0.10` kept so `EQ_RUBBLE=v1` draws
    # exactly what it always did; the shell itself is the rigid-body fall
    # (transformed above), so this is only the mark of debris at the
    # landing line, not the collapse mass, hence a plain windrow rather
    # than a fan.
    _rubble(ctx, m, "windrow", sides=(og["fall"],), depth_m=rng.uniform(0.6, 1.2),
            spread_frac=0.10, offset_m=max(0.0, land - 2.5), tag="landing")
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
            min_volume_frac=0.0008, static_mat=km if km else None,
            rough=ROUGH_STRIP_M)
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


def _a_roofify(ctx, mass):
    """Swap EVERY kit roof tile of `mass` for an authored slab; return
    [(path, lx, ly)] in the mass frame.

    ALL OF THEM, not just the one a recipe is about to break. A multi-tile
    roof (the office family has several) converted piecemeal comes out half kit
    tile and half authored slab with a dead-straight MATERIAL seam between
    them — the "straight line across the roof" complaint in its purest form
    (reviewer, round 2: `D_rc2/1_office_wide_collapse_onto_sw.png`, and again
    in `A_fin_rc/0_office_roof_hole_top.png` after the slab material had been
    changed once). Two greys that do not match cannot be made to match by
    picking a better grey; the roof has to be ONE material. Idempotent: called
    again it just re-reports the slabs (including the remainders
    `_split_strip` has since cut out of them)."""
    m = ctx["info"]["masses"][mass]
    thick = 0.14 if ctx["info"]["type"] == "urm" else ROOF_T
    for e in list(_els(ctx, mass=mass, role="roof")):
        if _roof_box(ctx, e, thick=thick):
            e["dead"] = True
    out = []
    for p in _a_roof_slabs(ctx):
        try:
            cx_, cy_ = _box_dims(ctx["stage"], p)[:2]
        except Exception:
            continue
        lx_, ly_ = _to_local(m, cx_, cy_)
        out.append((p, lx_, ly_))
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
    targets = _a_roofify(ctx, mass)

    n_hit, rim_st, own_loose = 0, [], []
    for tgt, tlx, tly in targets:
        # only pieces that actually reach INTO the hole are touched; a 6 m
        # margin fractured most of the roof and the crack mosaic came back
        if (abs(tlx - hcx) > hw * rmax + 2.6
                or abs(tly - hcy) > hd * rmax + 2.6):
            continue
        box = tgt
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
                              static_mat=bm if bm else None,
                              crack_frac=0.30, max_loose_m=3.2,
                              # _p_: a TIMBER deck really does come apart into
                              # boards, so `plank` stays for urm (and sliver
                              # rejection stays OFF there — a board IS an
                              # elongated piece). A concrete diaphragm does
                              # not: it cracks normal to its beams into
                              # rectangular rafts, so rc gets `prism`.
                              **(dict(mode="plank", aspect=(1.4, 2.8),
                                      rough=0.010) if btype == "urm"
                                 else _p_slab_kw(ctx)))
        ctx["loose"] += lo
        ctx["static_extra"] += st
        ctx["authored"] += st
        rim_st += st
        own_loose += lo
        n_hit += len(lo)
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
                              max_loose_m=3.2, **_p_slab_kw(ctx))    # _p_
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
    # ROOFTOP PLANT DOES NOT KNOW THE HOLE IS COMING. `dress_roof` /
    # `dress_roof_urban` lay out tanks, AC units and (on the fire ladder) a
    # bulkhead and its housekeeping pad BEFORE this recipe runs and before
    # the hole's outline exists, so a piece can land squarely inside the
    # polygon this call is about to cut purely by chance of the shared rng
    # draw. `_b_settle_roof_plant` still hands every one of them to the
    # solver at the end of the run, but being a rigid body does not
    # guarantee FALLING: a housekeeping pad spans several hole cells and
    # only needs to keep a fraction of ITS OWN footprint supported to stay
    # resting near its authored height, and an AC unit riding on that pad
    # never loses its own support because the pad under it does not move —
    # so the row reads as "the roof survived here" from directly above even
    # though most of the deck beneath it is gone. Same shape as the
    # "floating elements (water tanks, etc) on top of buildings" bug
    # `_b_settle_roof_plant` exists to fix (quake_city9), one level up: the
    # urban-fire case is a housekeeping pad and its condenser row riding out
    # a burn-through untouched (`ac_b2_3`, F4 commercial bench building,
    # 2026-08-29). MAJORITY OF THE FOOTPRINT, not the centre point alone — a
    # small AC unit is near enough to a point that it does not matter, but a
    # 10+ m pad needs the fairer test or an item hanging off its far end
    # never trips it. Anything mostly over the hole is dropped explicitly,
    # the same way a share of the broken roof cells already are just above —
    # authored at floor level, not left for a rigid neighbour to prop up.
    from pxr import Usd as _Usd, UsdGeom as _UsdGeom

    def _mostly_in_hole(pth):
        pr = ctx["stage"].GetPrimAtPath(pth)
        if not pr or not pr.IsValid():
            return False
        bc = _UsdGeom.BBoxCache(_Usd.TimeCode.Default(), [_UsdGeom.Tokens.default_])
        r = bc.ComputeWorldBound(pr).ComputeAlignedRange()
        if r.IsEmpty():
            return False
        lo, hi = r.GetMin(), r.GetMax()
        pts = ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]),
              ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0))
        return sum(1 for x, y in pts if judge((x, y, 0.0))) >= 3

    n_plant_in = 0
    for pth in list(ctx.get("roof_plant", [])) + list(ctx.get("roof_fixed", [])):
        pr = ctx["stage"].GetPrimAtPath(pth)
        if not pr or not pr.IsValid() or not pr.IsActive():
            continue
        if not _mostly_in_hole(pth):
            continue
        c = _pivot_of(ctx, pth)
        _transform_prims(ctx["stage"], [pth],
                         _translate(rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4),
                                    -(c[2] - z_floor - 0.4)))
        n_plant_in += 1
    if n_plant_in:
        ctx["notes"].append(
            "roof_hole: {0} rooftop plant item(s) fell with it".format(
                n_plant_in))
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
        # agent C's unified entry point: low side + how far it dropped + how
        # far the other side came up. NOT wrapped in `except TypeError` any
        # more: a TypeError raised INSIDE it (a complex number from a
        # negative base, round 2) silently degraded every lean to the old
        # berms. Everything below is the fallback for a tree without it.
        return fn(ctx, m, low_side=side, drop_m=sink_m, rise_m=lift_m)
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
        # what goes over the far parapet — a windrow on the NEIGHBOUR mass
        # `nm`, on the side away from the impact (`_opposite(nside)`), same
        # `spread_frac=0.16` so `EQ_RUBBLE=v1` reproduces the old apron
        # exactly.
        _rubble(ctx, nm, "windrow", sides=(_opposite(nside),),
                depth_m=rng.uniform(0.4, 0.9), spread_frac=0.16, tag="spill")
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
    # and the unit's rubble banked against it — on the NEIGHBOUR mass `nm`,
    # its own `nside` (the face the lost unit's rubble piles against).
    # `spread_frac=0.18` kept so `EQ_RUBBLE=v1` reproduces the old apron.
    _rubble(ctx, nm, "windrow", sides=(nside,), depth_m=rng.uniform(0.9, 1.7),
            spread_frac=0.18, tag="banked")
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
    # _g_ (round 3): `glass_loss` / `glass_fallout` are now shims onto these
    "curtain_wall": r_curtain_wall,
    "storefront_glass": r_storefront_glass,
    # _g2_ (round 3): `storefront_glass` already calls this, so no ladder
    # names it; it is here so a bench line and a future ladder can.
    "window_glass": r_window_glass,
    "glass_follow": r_glass_follow,
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


# _g_ (round 3) BENCH ENTRIES. `EQ_RECIPES` on the command line can only name
# a grade or a bare recipe, and a recipe with kwargs is not expressible there —
# so the glass alone, at every grade, gets five named wrappers. They are the
# only way to review the glass without the fracture/settle path in the frame,
# and they are what `scene_gen/tools/eq_bench.sh <snap> EQ_RECIPES=g_glass3`
# runs.
def _g_bench_recipe(grade):
    def _r(ctx, **kw):
        r_curtain_wall(ctx, grade=grade, **kw)
        r_storefront_glass(ctx, grade=grade)
        r_glass_follow(ctx)
    _r.__name__ = "r_g_glass{0}".format(grade)
    _r.__doc__ = "curtain_wall + storefront_glass at DG{0}, nothing else.".format(grade)
    return _r


for _g in range(1, 6):
    RECIPES["g_glass{0}".format(_g)] = _g_bench_recipe(_g)


# _g2_ (round 3) BENCH ENTRIES: the PUNCHED windows on their own, with no
# shopfront and no curtain wall in the frame, so a bench column shows exactly
# what `r_window_glass` authored.
def _g2_bench_recipe(grade):
    def _r(ctx, **kw):
        # THE BAND IS FORCED ONTO THE STREET ELEVATION for the bench, and only
        # for the bench. `_g_pick_sides` is front-biased but still draws N or W
        # about half the time, and the bench's `street` and `close` cameras are
        # fixed on the S face — so half the review columns showed an undamaged
        # wall and the reviewer had to go hunting in the obliques. The shipped
        # ladders call `r_window_glass` through `r_storefront_glass` with no
        # `sides`, so the city keeps the random draw (Kobe: south-facing > 70 %
        # damaged, north-facing < 15 %, so a per-building azimuth IS the field).
        kw.setdefault("band_sides", ("S", "E"))
        r_window_glass(ctx, grade=grade, **kw)
        r_glass_follow(ctx)
    _r.__name__ = "r_g2_win{0}".format(grade)
    _r.__doc__ = "window_glass at DG{0}, nothing else.".format(grade)
    return _r


for _g in range(1, 6):
    RECIPES["g2_win{0}".format(_g)] = _g2_bench_recipe(_g)


def _g2_follow_probe(ctx, **kw):
    """BENCH ONLY — the `r_glass_follow` proof for punched windows.

    Glazes every storey and every elevation at DG3 (so there is art ABOVE the
    storey that is about to be crushed, which a normal band would not
    guarantee), then crushes a mid storey, then follows. Read the two counters
    in the note: art on the crushed storey must be DROPPED with its module and
    art above it must be CARRIED. If "carried" is 0 while the block above
    visibly moved, the follow is broken."""
    r_window_glass(ctx, grade=3, band_sides=("S", "E", "N", "W"), scatter=True)
    r_mid_storey(ctx)
    r_glass_follow(ctx)


RECIPES["g2_follow"] = _g2_follow_probe


# ---------------------------------------------------------------------------
# ROUND 5 — THE `qc` COLLAPSE FAMILY (`disaster/quake_collapse.py`)
# ---------------------------------------------------------------------------
# The user, on the first 500 m M7.8 city (2026-08-30): "How does urban fire do
# partial collapse damage to modern city environment. I want you to use that,
# it looks more realistic ... lots of material mismatches. damaged part and
# undamaged part look like completely diff materials ... only use the partial
# collapse mechanism and expand to total collapse".
#
# `quake_collapse` is that: the urban-fire partial-collapse mechanism
# (`fire_collapse.plan_edges` / `_tear_perimeter`, `_break(mode="uniform")` so
# a fragment keeps its own cladding, per-fragment outward throw) carrying the
# earthquake's own failure modes and none of the fire's palette. It imports
# `pxr` nowhere at module scope and imports THIS module only from inside its
# functions, so this import is not circular.
#
# NOTHING BELOW CHANGES `LADDER` OR ANY EXISTING RECIPE. `LADDER_QC` is
# COMPOSED from `LADDER` plus `quake_collapse.LADDER_OVERRIDES`, so every row
# the override table does not name — DG0-DG2, the glass slots, the rooftop
# plant, the whole foundation family — is the legacy row and cannot drift from
# it; and `EQ_LADDER=legacy` selects `LADDER` itself, which reproduces today
# byte-for-byte.
from . import quake_collapse as _qc                           # noqa: E402
RECIPES.update(_qc.RECIPES)

LADDER_QC = dict((t, dict((g, list(r)) for g, r in lv.items()))
                 for t, lv in LADDER.items())
for _t, _over in _qc.LADDER_OVERRIDES.items():
    for _g, _recs in _over.items():
        LADDER_QC[_t][_g] = [(n, dict(kw or {})) for n, kw in _recs]

# `qc` (the default) runs the round-5 family; `legacy` runs the round-4 one.
_LADDER_MODE = _os.environ.get("EQ_LADDER", "qc").strip().lower()
if _LADDER_MODE not in ("qc", "legacy"):
    _LADDER_MODE = "qc"


def active_ladder():
    """The grade table `wreck_building` resolves a grade string against.

    `EQ_LADDER=qc` (default) -> `LADDER_QC`; `EQ_LADDER=legacy` -> `LADDER`.
    A caller that hands `wreck_building` an explicit `[(name, kwargs)]` list
    bypasses this entirely, which is what the bench's `EQ_RECIPES` does.
    """
    return LADDER_QC if _LADDER_MODE == "qc" else LADDER


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
        recipes = active_ladder()[info["type"]][recipes]
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


def level_for_intensity(i, btype, rng, jitter=0.05, duration_boost=1.0):
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
    # DURATION (research §13): a long record (M8+, 2-6 min) lowers the
    # collapse capacity of ENGINEERED frames — spectrally-equivalent 42 s vs
    # 6 s records: median collapse capacity -29 %, P(collapse) 11 % vs 1.4 %
    # — while brittle URM changes little. `duration_boost` (1..2.5, compiled
    # from the magnitude) multiplies the DG4/DG5 SHARE of the rc types:
    # cut' = 1 - boost * (1 - cut).
    if duration_boost > 1.0 and btype in ("rc", "rc_glass"):
        b_ = max(1.0, float(duration_boost))
        cuts = cuts[:3] + tuple(max(cuts[2] + 0.02, 1.0 - b_ * (1.0 - c)) for c in cuts[3:])
    g = 0
    for c in cuts:
        if v >= c:
            g += 1
    return GRADES[g]


def check(verbose=True):
    """Host-side: every ladder recipe exists, every family has a type."""
    from detail import urban_building as ub
    bad = []
    # BOTH TABLES, not just the legacy one. `EQ_LADDER=qc` is the DEFAULT, so
    # a typo in `quake_collapse.LADDER_OVERRIDES` is what a bake would hit
    # first, and a launch script that gates on this would have waved it
    # through.
    for tables, label in ((LADDER, "LADDER"), (LADDER_QC, "LADDER_QC")):
        for t, ladder in tables.items():
            for g, recs in ladder.items():
                for name, _kw in recs:
                    if name not in RECIPES:
                        bad.append("{0} {1}/{2}: unknown recipe {3}".format(
                            label, t, g, name))
    for s, spec in ub.STYLES.items():
        if spec.get("family") not in FAMILY_TYPE:
            bad.append("style {0}: family {1} has no type".format(s, spec.get("family")))
    if verbose:
        print("[quake_flow] check {0}".format("ok" if not bad else "FAILED"))
        for b in bad:
            print("  " + b)
    return bad
