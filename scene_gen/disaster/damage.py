"""damage stage — what a wildfire leaves behind on a building.

WHY THIS IS PLACEMENT WORK, NOT MESH WORK
-----------------------------------------
Isaac Sim has no runtime fracture. There is no Blast extension in this build —
`omni.convexdecomposition` is the only thing in that family — so meshes cannot
be shattered. Which turns out not to matter, because `modular_house` already
assembles a house from separately-placed modules: `floor`, `wall`, `roof`,
`bay_roof`, `door`, `porch`, `canopy`, `garage`. Damage is therefore a matter
of removing, felling and re-materialising those modules, and none of it needs
geometry the kit did not already ship.

SIX LEVELS, PLACED BY THE FIRE MODEL
------------------------------------
Ordered by how much of the house is gone, and matched to what a wildfire
actually leaves. The load-bearing observation is that the FOUNDATION SURVIVES:
a burned-out house from the air is a rectangular slab with debris inside its
own footprint, not a hole in the ground.

    pristine          control, untouched
    scorched          blackened but whole — the common case, and the one that
                      reads as "the fire came through here" with no structural
                      change at all
    roof_gone         roof consumed, walls standing. Roofs go first: they are
                      what catches the embers.
    partial_collapse  roof gone, some walls felled outward
    burned_out        floor plate and a few wall stubs
    slab              the floor plate alone

WHICH LEVEL A BUILDING GETS IS NOT RANDOM. `disaster.fire` already solves, for
every object, WHEN the front reaches it — so damage is a function of how long
ago it burned. `level_for_age` is that mapping, and it is what makes the damage
agree with the smoke: a house wrapped in flame is mid-collapse, a house the
front left twenty minutes ago is a cold slab. Scattering damage independently
of the fire is what makes a scene read as two unrelated effects.

CHAR IS BLACK, ASH IS GREY, AND THE AGE DECIDES
-----------------------------------------------
Fresh soot is near-black: the fire is still on the structure or only just left.
Once a structure has burned out and cooled, what remains is pale grey-white ash
and calcined masonry. Using one black for both is what makes a burnt row look
like a texture bug rather than a fire.

So `finish` follows burn age too — "char" at and just behind the front, "ash"
deep in the burn scar. An ashed building keeps black streaking, because cooled
ruins are mottled, never uniformly pale.

FELLING IS PLACEMENT-ONLY, FOR NOW
----------------------------------
A felled module is rotated onto its side, dropped to the ground and pushed
outward from the house centre. That is a stand-in for a physics settle — the
real version gives the modules rigid bodies and convex-decomposition colliders,
steps PhysX, and bakes the resulting transforms back as static prims. Doing it
this way first is deliberate: settling is the largest piece of work in the
damage plan and it is wasted if the vocabulary above changes once it is seen.

CHAR IS MOTTLED, NOT UNIFORM
----------------------------
Each GeomSubset is charred independently with probability `char_fraction`, so
a scorched wall is patchy the way a real one is. Binding one flat black over
everything reads as a texture error rather than as fire damage.

Windows are charred along with everything else on purpose — burned houses have
blown-out, soot-black openings, not clean glass.
"""

import math
import os

LEVELS = ("pristine", "scorched", "roof_collapsed", "partial_collapse",
          "burned_out", "rubble")

# Module subs that SURVIVE at each level. None means "everything".
#
# NOTHING IS DELETED UNTIL IT HAS FALLEN. An earlier cut removed the roof
# outright at `roof_gone`, which left a clean open-topped box that reads as an
# unfinished build rather than a fire — a burnt roof collapses INTO the house,
# it does not evaporate. So the roof is felled here, not removed, and only
# disappears once the level is severe enough that it would have burned through.
_SURVIVES = {
    "pristine":         None,
    "scorched":         None,
    "roof_collapsed":   None,
    "partial_collapse": {"floor", "wall", "roof", "bay_roof", "door",
                         "door_slot", "garage"},
    "burned_out":       {"floor", "wall", "roof", "bay_roof"},
    "rubble":           {"floor", "wall", "roof", "bay_roof"},
}

# Fraction of each module sub that ends up on the ground, per level.
#
# Doors fall as soon as anything else does. A door frame standing upright in
# front of a collapsed wall was the single most obviously wrong thing in the
# first bench — it is a thin timber panel, it is the first thing to go.
_FELL = {
    "pristine":         {},
    "scorched":         {},
    "roof_collapsed":   {"roof": 1.0, "bay_roof": 1.0},
    "partial_collapse": {"roof": 1.0, "bay_roof": 1.0, "wall": 0.35,
                         "door": 1.0},
    "burned_out":       {"roof": 1.0, "bay_roof": 1.0, "wall": 0.75,
                         "door": 1.0},
    "rubble":           {"roof": 1.0, "bay_roof": 1.0, "wall": 1.0},
}

# How much of each building is blackened.
_CHAR = {
    "pristine": 0.0, "scorched": 0.55, "roof_collapsed": 0.75,
    "partial_collapse": 0.85, "burned_out": 1.0, "rubble": 1.0,
}

# What the bench shows above each house, so structure and fire agree — a cold
# pile of rubble does not have flames on it.
FIRE_STATE = {
    "pristine": None, "scorched": None, "roof_collapsed": "smoke",
    "partial_collapse": "flame", "burned_out": "smoulder",
    "rubble": "residual",
}

_CHAR_RGB = (0.045, 0.040, 0.038)     # fresh soot
_SCORCH_RGB = (0.16, 0.13, 0.11)      # heat-darkened but not consumed
_ASH_RGB = (0.52, 0.51, 0.485)        # cooled ash / calcined masonry

# How a finish mixes its materials. Weights are (char, scorch, ash) and are
# what makes each read as mottled rather than as one flat colour.
_FINISH_MIX = {
    # Some ash even in the "fresh char" mix, so both burnt maps appear on
    # every pile rather than the ash one only showing up on cold ruins.
    "char": (0.55, 0.30, 0.15),
    # Ash is the MINORITY even on a cooled ruin. Pale ash reads as bright grey
    # at any distance and takes over the silhouette long before it should —
    # what actually dominates a burnt structure is black char with
    # heat-darkened timber around it, and ash as deposit on top.
    "ash":  (0.52, 0.30, 0.18),
}


def _sub_of(category):
    """`house_bay_roof` -> `bay_roof`. Categories are `<cat>_<sub>`."""
    c = str(category or "")
    for sub in ("bay_roof", "door_slot", "floor", "wall", "roof", "door",
                "porch", "canopy", "garage"):
        if c.endswith("_" + sub):
            return sub
    return c


def damage_placements(placements, level, rng, keep_stubs=0.25,
                      move_felled=True):
    """Remove and fell modules for *level*. Pure Python — no stage needed.

    Returns a NEW list; the input is not mutated, so the same pristine build
    can be run through several levels for comparison.

    *keep_stubs* is the fraction of walls left standing at `burned_out`, which
    is what stops that level reading as an empty slab with a different name.

    *move_felled* False marks a module as loose but LEAVES IT STANDING, for
    the case where physics is going to do the collapse. Felling by formula and
    then simulating is the worst of both: the piece is already flat on the
    ground when the solver starts, so nothing falls and the result looks like
    an intact wall that happens to be cut into pieces.
    """
    if level not in _SURVIVES:
        raise ValueError("unknown damage level {0!r}; expected one of {1}"
                         .format(level, ", ".join(LEVELS)))

    survives = _SURVIVES[level]
    fell_spec = _FELL[level]

    xs = [p["x_m"] for p in placements] or [0.0]
    ys = [p["y_m"] for p in placements] or [0.0]
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)

    # A COUNT PER SUB, not a per-module coin flip. A cottage carries five wall
    # modules, so `rng.random() < 0.35` on each rounds to zero felled walls
    # about one time in eight — `partial_collapse` then looked identical to the
    # level below it. Choosing how many up front, with at least one whenever
    # the level is meant to fell that sub at all, makes the level mean what it
    # says on every seed.
    felled = set()
    for sub, frac in fell_spec.items():
        if survives is not None and sub not in survives:
            continue
        idx = [i for i, p in enumerate(placements)
               if _sub_of(p.get("category")) == sub]
        if not idx or frac <= 0.0:
            continue
        k = max(1, min(len(idx), int(round(frac * len(idx)))))
        felled.update(rng.sample(idx, k))

    out = []
    for i, p in enumerate(placements):
        sub = _sub_of(p.get("category"))
        if survives is not None and sub not in survives:
            continue

        q = dict(p)
        if i in felled and not move_felled:
            q["felled"] = True          # loose, but physics will place it
        elif i in felled:
            is_roof = sub in ("roof", "bay_roof")
            # Falls outward, away from the middle of the house.
            dx, dy = q["x_m"] - cx, q["y_m"] - cy
            d = math.hypot(dx, dy) or 1.0
            ux, uy = dx / d, dy / d
            throw = rng.uniform(0.8, 2.4)
            q["x_m"] = q["x_m"] + ux * throw
            q["y_m"] = q["y_m"] + uy * throw
            # A roof drops INSIDE the walls rather than being thrown clear,
            # and lands shallower — it is a panel folding in, not a slab
            # tipping over.
            if is_roof:
                q["x_m"] = p["x_m"] + ux * rng.uniform(-0.6, 0.9)
                q["y_m"] = p["y_m"] + uy * rng.uniform(-0.6, 0.9)
                q["z_m"] = 0.55 + rng.uniform(0.0, 0.9)
                q["roll_deg"] = rng.choice([-1.0, 1.0]) * rng.uniform(18.0, 46.0)
                q["pitch_deg"] = rng.uniform(-22.0, 22.0)
            else:
                q["z_m"] = 0.15 + rng.uniform(0.0, 0.25)
                # Roll it flat; a little pitch so the pile is not one plane.
                q["roll_deg"] = rng.choice([-1.0, 1.0]) * rng.uniform(72.0, 96.0)
                q["pitch_deg"] = rng.uniform(-9.0, 9.0)
            q["yaw_deg"] = float(q.get("yaw_deg", 0.0)) + rng.uniform(-14.0, 14.0)
            q["felled"] = True
        elif sub == "wall" and level in ("burned_out", "rubble"):
            if rng.random() > keep_stubs:
                continue
        out.append(q)
    return out


# How long after ignition each level takes hold, as multiples of the fire
# model's own phase durations, so damage and smoke stay in step when the fire
# is retuned.
def level_for_age(dt, ignition_s=6.0, flame_s=180.0, smoulder_s=90.0,
                  ash_after_s=240.0):
    """`(level, finish, fire_state)` for a structure `dt` seconds post-ignition.

    `dt` is `elapsed - t_ignite` from `disaster.fire.plan_ignition`, so it is
    NEGATIVE for anything the front has not reached — those come back pristine
    with no fire, which is what leaves a clean unburnt margin to fly toward.

    The finish flips from char to ash once the structure has been cold for
    `ash_after_s`, which is what puts pale ruins deep in the burn scar and
    black ones at the front without either being placed by hand.
    """
    if dt < 0.0:
        return ("pristine", None, None)
    if dt < ignition_s:
        # NO FIRE ON AN INTACT HOUSE. `scorched` means the front went past and
        # marked the outside; nothing is burning, so a plume here reads as a
        # building alight when it plainly is not.
        return ("scorched", "char", None)
    d = dt - ignition_s
    if d < flame_s * 0.45:
        return ("roof_collapsed", "char", "flame")
    if d < flame_s:
        return ("partial_collapse", "char", "flame")
    d -= flame_s
    if d < smoulder_s:
        return ("burned_out", "char", "smoulder")
    if d < ash_after_s:
        return ("burned_out", "ash", "residual")
    return ("rubble", "ash", "residual")


# Which kit pieces stand in for broken fragments. All window-less, because a
# fragment with a window frame in it reads as a wall that fell over rather than
# as a piece of something that came apart.
def _fragment_assets():
    from detail import modular_house as mh
    return {
        "panel": mh._usd(mh.WALL_5["plain"]),      # Outer_Wall_Quart_01, 5 m
        "slab":  mh._usd(mh.FLOOR_5),              # Floor_Quart_01
    }


def break_up(placements, rng, roof_chance=0.65, wall_chance=0.45,
             spread_m=3.2):
    """Replace felled modules with several smaller fragments.

    A COLLAPSED ROOF IS NOT A ROOF AT AN ANGLE. Dropping the roof module whole
    leaves an intact, recognisable roof lying tilted on the house, which reads
    as a placement error. What a burnt roof actually leaves is a heap of
    unidentifiable panels, so a broken roof here becomes two or three window-
    less wall panels at scattered angles — the kit has no debris geometry, but
    it does have plain panels, and at that angle nobody reads them as walls.

    The same applies to a 10 m wall: one big slab tipping over looks staged,
    two 5 m pieces at different angles look like it came apart.

    Chances are per module, so two houses with the same damage level still
    collapse differently — which is the point.
    """
    from detail import modular_house as mh

    frag = _fragment_assets()
    out = []
    for p in placements:
        sub = _sub_of(p.get("category"))
        if not p.get("felled"):
            out.append(p)
            continue

        is_roof = sub in ("roof", "bay_roof")
        is_big_wall = sub == "wall" and "Half" in str(p.get("usd", ""))
        chance = roof_chance if is_roof else (wall_chance if is_big_wall else 0.0)
        if chance <= 0.0 or rng.random() > chance:
            out.append(p)
            continue

        n = rng.randint(2, 3) if is_roof else 2
        for _ in range(n):
            q = mh._place(
                frag["panel"],
                p["x_m"] + rng.uniform(-spread_m, spread_m),
                p["y_m"] + rng.uniform(-spread_m, spread_m),
                0.18 + rng.uniform(0.0, 0.55),
                rng.uniform(0.0, 360.0),
                category=p.get("category", "house_wall"))
            q["roll_deg"] = rng.uniform(-1.0, 1.0) * rng.uniform(58.0, 104.0)
            q["pitch_deg"] = rng.uniform(-26.0, 26.0)
            q["felled"] = True
            q["fragment"] = True
            out.append(q)
    return out


def scatter_debris(placements, rng, n_pieces=(8, 16), radius_m=9.0):
    """Loose rubble across and around the footprint.

    Small kit pieces rather than the `debris` asset pool on purpose: these are
    the same panels and slabs the house is made of, so they char with it and
    match its material. Imported rubble props would need their own burnt
    variants to not stand out as clean geometry in a burnt lot.
    """
    from detail import modular_house as mh

    frag = _fragment_assets()
    xs = [p["x_m"] for p in placements] or [0.0]
    ys = [p["y_m"] for p in placements] or [0.0]
    cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)

    out = []
    for _ in range(rng.randint(*n_pieces)):
        ang = rng.uniform(0.0, 2.0 * math.pi)
        # sqrt keeps the scatter area-uniform instead of bunched at the middle
        r = radius_m * math.sqrt(rng.random())
        which = "slab" if rng.random() < 0.45 else "panel"
        q = mh._place(frag[which],
                      cx + math.cos(ang) * r, cy + math.sin(ang) * r,
                      0.06 + rng.uniform(0.0, 0.30),
                      rng.uniform(0.0, 360.0),
                      category="house_debris",
                      scale=mh.SCALE * rng.uniform(0.35, 0.72))
        q["roll_deg"] = rng.uniform(-1.0, 1.0) * rng.uniform(66.0, 108.0)
        q["pitch_deg"] = rng.uniform(-30.0, 30.0)
        q["felled"] = True
        q["fragment"] = True
        out.append(q)
    return out


def summarise(level, before, after):
    n_fell = sum(1 for p in after if p.get("felled"))
    return ("{0:<17s} {1:3d} modules -> {2:3d} ({3} felled, char {4:.0%})"
            .format(level, len(before), len(after), n_fell, _CHAR[level]))


# ---------------------------------------------------------------------------
# Materials — needs a stage
# ---------------------------------------------------------------------------

# Absolute, derived from this file rather than from a config scheme: USD has
# never heard of `airstack://`, and the repo sits at a different path in the
# container (/isaac-sim/AirStack) than on the host. Same reasoning as
# `scene_generator.LOCAL_ASSET_ROOTS`, without the import.
_TEX_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "assets", "materials", "burn")


def _pbr(stage, path, rgb, roughness, texture=None,
         scale_uv=(2.0, 2.0), offset_uv=(0.0, 0.0),
         tint=(1.0, 1.0, 1.0)):
    """An OmniPBR material, textured when a map is given.

    A FLAT COLOUR IS THE PROBLEM, not the palette. Char is cracked and uneven
    and cooled ash is pale grey shot through with white calcined flecks — both
    are textures, and painting either as one constant reads as a shading bug
    rather than as fire damage. `tools/burn_textures.py` generates the maps.

    `rgb` stays as the tint so the texture can be pushed lighter or darker
    without regenerating it, and as the fallback if the map fails to resolve.
    """
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(float(roughness))
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.12)
    if texture:
        sh.CreateInput("diffuse_texture",
                       Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
        sh.GetInput("diffuse_color_constant").Set(Gf.Vec3f(*tint))
        # One tile per ~2 m of wall — small enough to carry detail close up,
        # large enough not to alias from the air. VARIED PER VARIANT, because
        # a single scale repeating across every panel of every house is what
        # reads as tiling rather than as burnt material.
        # TRIPLANAR, IN WORLD SPACE. Two reasons. Generated debris carries no
        # UVs at all, so a UV-space material falls back to a per-face default
        # and the map repeats inside every triangle — the "duplicated into
        # small rectangles" look. And even where UVs exist, UV-space scaling
        # ties feature size to a piece's UV layout, so a small fragment and a
        # whole wall show the grain at different sizes.
        #
        # Projecting from world coordinates instead makes the scale METRIC:
        # every piece in the scene shows char at the same physical size, and
        # a 0.4 m chip shows a 0.4 m crop of it rather than the whole tile.
        sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
        # texture_scale is now repeats-per-metre, so SMALLER means BIGGER
        # features. ~0.45 puts one tile across a bit over two metres.
        sh.CreateInput("texture_scale",
                       Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*scale_uv))
        sh.CreateInput("texture_translate",
                       Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(*offset_uv))
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


# Per-variant texture scale, uv offset and tint. Three of each is enough to
# stop a wall of panels reading as one repeated tile without tripling the
# material count for no gain.
# Per-variant scale (repeats per metre), world offset and tint. Three of each
# breaks up the repetition without tripling the material count.
_VARIANTS = (
    ((0.38, 0.38), (0.00, 0.00), (1.00, 1.00, 1.00)),
    ((0.52, 0.47), (3.70, 6.10), (0.92, 0.90, 0.90)),
    ((0.30, 0.34), (7.10, 2.30), (1.08, 1.05, 1.02)),
)

_MAPS = {
    "char":   ("Burn_Char_Ref.png", _CHAR_RGB, 0.92),
    "scorch": ("Burn_Scorch.png", _SCORCH_RGB, 0.85),
    "ash":    ("Burn_Ash_Over_Char.png", _ASH_RGB, 0.94),
}

def char_materials(stage, parent_path, maps=None, suffix=""):
    """The burn materials, three UV variants each, under `<parent>/BurnLooks`.

    *maps* overrides which texture backs each role, e.g.
    `{"ash": "Burn_Ash_Flake.png"}`, so a bench can stand several looks up
    side by side. *suffix* keeps their prim paths distinct when it does.
    """
    from pxr import Sdf

    looks = Sdf.Path(parent_path).AppendChild("BurnLooks" + suffix)
    stage.DefinePrim(looks, "Scope")
    spec = dict(_MAPS)
    for k, png in (maps or {}).items():
        if k in spec:
            spec[k] = (png, spec[k][1], spec[k][2])
    out = {}
    for name, (png, rgb, rough) in spec.items():
        out[name] = [
            _pbr(stage, looks.AppendChild("{0}_{1}".format(name.capitalize(), i)),
                 rgb, rough, os.path.join(_TEX_DIR, png),
                 scale_uv=sc, offset_uv=off, tint=tint)
            for i, (sc, off, tint) in enumerate(_VARIANTS)]
    return out


def char_placements(stage, placements, level, mats, rng, finish="char"):
    """Blacken a fraction of every placed module's GeomSubsets.

    Walks the same way `modular_house.apply_palette` does — Mesh prims, then
    their GeomSubsets, falling back to the mesh itself when it has none —
    because that is the granularity this kit binds materials at, and a
    per-Mesh bind would miss most of the house.
    """
    from pxr import Usd, UsdGeom, UsdShade

    frac = _CHAR[level]
    if frac <= 0.0:
        return 0

    n = 0
    for pl in placements:
        path = pl.get("prim_path")
        if not path:
            continue
        root = stage.GetPrimAtPath(path)
        if not root or not root.IsValid():
            continue
        # A felled module is on the ground in the fire's path; char it fully.
        p_char = 1.0 if pl.get("felled") else frac
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            subsets = list(
                UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
            for t in ([s.GetPrim() for s in subsets] or [prim]):
                if rng.random() > p_char:
                    continue
                UsdShade.MaterialBindingAPI(t).Bind(_pick(rng, finish, mats))
                n += 1
    return n


def _pick(rng, finish, mats):
    """One material: which map, by the finish's mix, then which variant."""
    w_char, w_scorch, w_ash = _FINISH_MIX.get(finish, _FINISH_MIX["char"])
    r = rng.random() * (w_char + w_scorch + w_ash)
    if r < w_char:
        key = "char"
    elif r < w_char + w_scorch:
        key = "scorch"
    else:
        key = "ash"
    return rng.choice(mats[key])


# ---------------------------------------------------------------------------
# Sooting — darken what a surface ALREADY is
# ---------------------------------------------------------------------------
#
# Distinct from `char_placements`, which REPLACES a surface with a burn
# texture. A wall that survived the fire is not made of charcoal: it is the
# same cream siding or red brick it always was, darkened by smoke. Swapping it
# for a char map throws away the building's identity and makes an intact wall
# read as a burnt one.
#
# So this duplicates whatever material a surface already binds and multiplies
# its albedo tint down. The texture, the hue and the material family all
# survive; only the value changes. The kit's UE-exported materials expose
# `Tint_AlbedoTexture_` as a float4 multiplier for exactly this, and OmniPBR
# has equivalents.

# THE DARKENING KNOB IS `albedo_brightness`, a FLOAT on OmniPBR. An earlier
# cut guessed at input names and force-set any it found, which went wrong two
# ways: the kit's UE-exported wall materials expose no tint input at all, so
# walls did not darken; and `albedo_add` is a float, so writing a colour into
# it corrupted doors and windows to blue.
#
# Guessing at someone else's shader network is the mistake. This instead reads
# only the one thing every one of these materials definitely has — its base
# colour TEXTURE — and builds our own OmniPBR around it, where the parameters
# are known because we authored them.

# Soot COVERAGE, not brightness: how much of a surface the deposit reaches.
# Scaling brightness darkens everything equally and reads as a dimmer; moving
# coverage grows the fouled AREA, which is what actually looks scorched.
SOOT_LEVELS = (0.30, 0.48, 0.66, 0.84)


def _basecolor_texture(mat_prim):
    """The base-colour map a material resolves to, or None.

    Same identification `modular_house.apply_palette` uses: walk the shader
    network for an asset-valued input whose file looks like a base colour.
    Some inputs in this kit raise on Get() rather than returning None, so the
    read cannot be bare.
    """
    from pxr import Sdf, Usd, UsdShade

    best = None
    # EXPIRED PRIMS RAISE, they do not return empty. A kit module's material
    # usually lives inside that module's own subtree, so once the module is
    # deactivated by a fracture the handle is dead and PrimRange throws
    # `Invalid range starting with expired 'Material' prim`.
    if not mat_prim or not mat_prim.IsValid():
        return None
    for p in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        for inp in sh.GetInputs():
            try:
                v = inp.Get()
            except Exception:
                continue
            if not isinstance(v, Sdf.AssetPath):
                continue
            f = (v.resolvedPath or v.path or "")
            low = f.lower()
            if not low:
                continue
            if any(k in low for k in ("basecolor", "albedo", "diffuse",
                                      "_bc", "_col", "_d.", "basecolour")):
                return f
            # THE FALLBACK WAS TAKING NORMAL MAPS. Anything not obviously a
            # base colour was accepted, so a material whose first texture is a
            # normal or ORM map got composited over the wrong image — which is
            # why brick and other non-cladding surfaces came out unscorched or
            # oddly tinted. Exclude the map types by name instead.
            if any(k in low for k in ("normal", "_n.", "_nrm", "orm", "rough",
                                      "metal", "height", "_h.", "occlusion",
                                      "_ao", "opacity", "mask", "emissive")):
                continue
            if best is None and low.endswith((".png", ".jpg", ".jpeg")):
                best = f
    return best


# INTERIOR IS DECIDED BY THE PLACEMENT CATEGORY, not by the material. Every
# material in this kit resolves to a prim literally named "UnrealMaterial"
# (e.g. /World/stage/generated/house_floor_0_13/Section0/UnrealMaterial), so
# the material says nothing about which side of a wall it is on. The category
# does: `house_floor` IS the interior — it is the floor plate you are looking
# down onto once the roof has gone.
INTERIOR_CATEGORIES = ("floor", "door_slot")

# THINGS THAT DO NOT BURN, SOOT OR COLLAPSE.
#
# Water most obviously — a pool cannot be scorched, and compositing a soot
# wash over its surface texture just corrupts it. The tile surround is wet
# masonry and survives a wildfire essentially untouched, which is exactly why
# a pool is one of the clearest surviving features in real burn-scar imagery.
# Metal street furniture is here for the same reason: hydrants, poles and
# signs come through a fire visibly intact.
#
# Matched as substrings, so `plot_pool_edge` is caught by `pool`.
INCOMBUSTIBLE = ("pool", "water", "fire_hydrant", "hydrant", "streetlight",
                 "sign", "manhole", "storm_drain")


def is_incombustible(category):
    c = str(category or "").lower()
    return any(k in c for k in INCOMBUSTIBLE)


def _is_interior(category):
    c = str(category or "")
    return any(c.endswith("_" + k) or c == k for k in INTERIOR_CATEGORIES)


# Scorch coverage buckets. Finer than the original four because coverage is
# now driven by a continuous field and needs somewhere to quantise to; the
# buckets exist purely so a street composites a handful of maps instead of one
# per surface.
SOOT_LEVELS = (0.52, 0.62, 0.72, 0.82, 0.90, 0.96, 1.00)


def bucket(coverage):
    """Nearest SOOT_LEVELS index for a continuous 0..1 coverage."""
    c = max(0.0, min(1.0, float(coverage)))
    return min(range(len(SOOT_LEVELS)),
               key=lambda i: abs(SOOT_LEVELS[i] - c))


def scorched_material(stage, parent_path, src_mat_prim, level,
                      from_above=False, triplanar=False, cache=None,
                      texture=None):
    """A material that is *this* surface, scorched. Cached by source and level.

    Split out of `soot_materials` because two other callers need it: fragments
    of a PARTIALLY collapsed wall, which should keep the wall's own texture
    rather than take a generic char map, and anything else that wants a
    specific coverage rather than a random one.
    """
    import numpy as np

    from pxr import Gf, Sdf, UsdShade

    from . import scorch

    cache = _CACHE if cache is None else cache
    soot_scope = Sdf.Path(parent_path).AppendChild("SootLooks")
    stage.DefinePrim(soot_scope, "Scope")

    # A pre-resolved path wins, so a caller can look the texture up while
    # the source prim is still alive and use it after fracturing kills it.
    tex = texture or _basecolor_texture(src_mat_prim)
    key = (str(parent_path), tex or "@none@", int(level), bool(from_above),
           bool(triplanar))

    if key in cache:
        return cache[key]

    cov = SOOT_LEVELS[int(level)]
    path = soot_scope.AppendChild("soot_{0:04d}".format(len(cache)))

    if not tex:
        _NO_TEXTURE.add(str(src_mat_prim.GetPath())
                        if src_mat_prim and src_mat_prim.IsValid()
                        else "<no source>")

    painted = None
    if tex:
        # Walls wash UP from the base, where a wildfire front arrives. A floor
        # is fouled by what lands on it, so top-down keeps the heavy end away
        # from the wall junction where it would read as a shadow.
        painted = scorch.scorched_texture(
            tex, cov, np.random.default_rng(abs(hash(key)) % (2 ** 31)),
            from_below=not from_above)

    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path.AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    if painted:
        sh.CreateInput("diffuse_texture",
                       Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(painted))
    elif tex:
        sh.CreateInput("diffuse_texture",
                       Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(tex))
        sh.CreateInput("albedo_brightness",
                       Sdf.ValueTypeNames.Float).Set(1.0 - 0.7 * cov)
    else:
        sh.CreateInput("diffuse_color_constant",
                       Sdf.ValueTypeNames.Color3f).Set(
                           Gf.Vec3f(0.55, 0.54, 0.52))
        sh.CreateInput("albedo_brightness",
                       Sdf.ValueTypeNames.Float).Set(1.0 - 0.7 * cov)
    if triplanar:
        # FRACTURE FRAGMENTS CARRY NO UVs. `fracture._write_mesh` authors
        # points and faces only, so a UV-space material has nothing to map
        # with and the scorch simply does not appear — which is why a partial
        # wall's pieces looked unscorched while the stub beside them did not.
        # Projecting from world coordinates needs no UVs, and on a vertical
        # face the texture's V axis lands on world Z, so the up-wash gradient
        # still runs the right way.
        sh.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("world_or_object", Sdf.ValueTypeNames.Bool).Set(True)
        sh.CreateInput("texture_scale",
                       Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.28, 0.28))
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(0.82)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    cache[key] = mat
    return mat


def bound_texture(stage, prim_path):
    """The base-colour texture a module binds, as a PATH STRING.

    Use this, not `bound_material`, whenever the module is about to be
    fractured: a prim handle does not survive its own subtree being
    deactivated, and a string does.
    """
    prim = bound_material(stage, prim_path)
    return _basecolor_texture(prim) if prim is not None else None


def bound_material(stage, prim_path):
    """The material a placed module currently binds, or None.

    Read BEFORE fracturing, because `fracture_prim` deactivates the source and
    the fragments that replace it have no binding of their own.
    """
    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        subsets = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        for t in ([s.GetPrim() for s in subsets] or [prim]):
            m = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            if m and m.GetPrim().IsValid():
                return m.GetPrim()
    return None


_CACHE = {}
# Materials we could not find a base colour for. They fall back to plain
# darkening, which is why a surface can come out dim rather than scorched —
# worth reporting rather than leaving to be spotted in a render.
_NO_TEXTURE = set()


def soot_materials(stage, placements, parent_path, rng, strength=(1, 3),
                   interior_bonus=2, coverage_at=None, skip_categories=(),
                   skip_incombustible=True):
    """Scorch surfaces in place, keeping their own colour. Returns a count.

    *coverage_at* is a callable `(x_m, y_m) -> 0..1`. Passing the disaster's
    own damage field here is what ties scorch to the FIRE rather than to a die
    roll: a house at the ignition point is fouled to its eaves while one at the
    edge of the burn is barely marked, and the gradient across a street reads
    as one event instead of as noise.

    Without it, levels are drawn from *strength* at random, which is only
    useful for a bench.
    """
    from pxr import Usd, UsdGeom, UsdShade

    n = 0
    lo, hi = strength
    for pl in placements:
        if str(pl.get("category", "")) in skip_categories:
            continue
        if skip_incombustible and is_incombustible(pl.get("category")):
            continue
        path = pl.get("prim_path")
        if not path:
            continue
        root = stage.GetPrimAtPath(path)
        if not root or not root.IsValid() or not root.IsActive():
            continue

        if coverage_at is not None:
            cov = coverage_at(float(pl.get("x_m", 0.0)),
                              float(pl.get("y_m", 0.0)))
            # ZERO COVERAGE MEANS OUTSIDE THE BURN. Bucket 0 does NOT — it is
            # the LIGHTEST scorch, not the absence of one. Skipping on
            # `level <= 0` therefore left everything the fire only just
            # reached completely clean, which is why heavily damaged buildings
            # still had pristine-looking walls on them.
            if cov <= 0.0:
                continue
            level = bucket(cov)
        else:
            level = rng.randint(int(lo), int(hi))
        inside = _is_interior(pl.get("category"))
        if inside:
            level = min(len(SOOT_LEVELS) - 1, level + int(interior_bonus))

        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            subsets = list(
                UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
            for t in ([s.GetPrim() for s in subsets] or [prim]):
                bound = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
                if not bound or not bound.GetPrim().IsValid():
                    continue
                bp = str(bound.GetPrim().GetPath())
                if "BurnLooks" in bp or "SootLooks" in bp:
                    continue
                m = scorched_material(stage, parent_path, bound.GetPrim(),
                                      level, from_above=inside)
                if m is not None:
                    UsdShade.MaterialBindingAPI(t).Bind(m)
                    n += 1
    if _NO_TEXTURE:
        print("[soot] {0} material(s) had no readable base colour and were "
              "only darkened, not scorched:".format(len(_NO_TEXTURE)))
        for path in sorted(_NO_TEXTURE)[:6]:
            print("[soot]   {0}".format(path))
    return n
