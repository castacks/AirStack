#!/usr/bin/env python
"""
Burnt-tree bench — six ways of burning a tree, side by side on grass.

    ISAAC_SIM_SCRIPT_NAME=tree_damage_test_launch_script.py airstack up isaac-sim

WHY THIS EXISTS
---------------
`suburb_mini_wildfire` has no vegetation at all — its docstring says so, and
gives the reason: "Vegetation is its own damage vocabulary and is not built;
leaving it green would fight everything else in frame." This is that
vocabulary, laid out so the methods can be compared before any of them goes
near a street.

THE ROW, LEFT TO RIGHT, IS THE BURN-AGE LADDER
----------------------------------------------
Each unit gets ONE method, and severity increases along +X, so the row doubles
as the same "one number drives everything" model the buildings already use
(`vegetation.level_for_age` mirrors `damage.level_for_age` deliberately).

  x=-115  0  CONTROL            untouched, for comparison. Judging "brown"
                                against nothing is hopeless.
  x= -70  1  CLUSTER: crown scorch
                                foliage entirely PRESENT and dead brown, bole
                                charred only to a third of its height. No
                                geometry changes at all. The most common real
                                outcome by area and the one nobody models.
  x= -22  2  torched skeleton   crown gone ENTIRELY — every instancer, leaf
                                and wood alike, since those branchlets are a
                                centimetre through and burn. What is left is
                                the one mesh named *_trunk / *_base, which on
                                these assets IS the bole and its primary
                                limbs. Standing. The iconic one.
  x= +22  3  CLUSTER: mixed severity
                                five trees at five different levels — a stand
                                EDGE, where the front weakened as it passed.
                                The one case a uniform cluster cannot show.
  x= +66  4  snapped snag       bole broken on a tilted plane, everything
                                above it consumed, broken trunk lengths at
                                its foot.
  x=+100  5  burned off + stump bole cut low, laid over and settled; the
                                charred stub stays. A CityPark stump prop
                                stands beside it as the cheap alternative.

SEVERITY IS A PROPERTY OF THE STAND. Crown fire carries between touching
canopies, so a dense clump burns to one severity while an isolated tree with
clearance gets only scorched — which is why unit 1 is uniform. Unit 3 is the
other half of that: at the EDGE of a run the front is weakening, and
neighbours end up genuinely different. Uniform and ragged are both real, and
a scene needs both.

AND THEY MOSTLY STAY UP. Only unit 5 goes down. Behind a real front a burnt
stand is a field of standing black poles — trees fall over months to years —
and a scene full of downed trunks reads as a tornado, not a fire.

GRASS, NOT A WHITE PLANE. The house bench paints its ground neutral grey
because its subject is char against ash and it needs honest light. Here the
subject is a brown or black tree against the ground it stands on, and a white
floor makes that impossible to judge.
"""

import math
import os
import random
import sys

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("omni.flowusd")
enable_extension("omni.kit.window.script_editor")

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade

_ISAAC_SIM_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_SCENE_GEN_DIR = os.path.normpath(
    os.path.join(_ISAAC_SIM_DIR, "..", "..", "scene_gen"))
sys.path.insert(0, os.path.join(_ISAAC_SIM_DIR, "utils"))
sys.path.insert(0, _SCENE_GEN_DIR)

import scene_generator as sg                                   # noqa: E402
from scene_prep import get_stage_meters_per_unit               # noqa: E402
from disaster import damage, fire, fracture, settle            # noqa: E402
from disaster import vegetation as veg                         # noqa: E402

PARENT = "/World/stage/generated"
SEED = int(os.environ.get("TREE_SEED", "5"))
# BAKED, like the house benches. Baking was suspected of freezing pieces in
# mid-air and it was not the cause — the floating was Voronoi fragments each
# holding several disconnected branch sections, which rest as one rigid body
# wherever their combined hull lands. Fix the fragmentation and baking is
# free: the scene ships as ordinary static geometry at zero runtime cost.
# `BAKE=0` leaves the bodies live for hand-testing.
BAKE = os.environ.get("BAKE", "1") not in ("0", "", "false")
# Enough for a bole tipped past its balance point at 13 m to finish falling.
# Baking freezes whatever it finds, so this has to outlast the longest fall in
# the scene, not the average one.
SETTLE_STEPS = int(os.environ.get("SETTLE_STEPS", "420"))
# TREE_UNITS=1,3,5 runs a subset. Fracture and texture compositing dominate
# start-up, so iterating on one method is much faster than rebuilding the row.
ONLY = [s.strip() for s in os.environ.get("TREE_UNITS", "").split(",")
        if s.strip()]

_VEG = "airstack://scene_gen/assets/aec"
BLACK_OAK = _VEG + "/tower/Assets/Vegetation/Black_Oak/Black_Oak.usd"
SHUMARD = _VEG + "/tower/Assets/Vegetation/Shumard_Oak/Shumard_Oak.usd"
APPLE = _VEG + "/tower/Assets/Vegetation/Common_Apple/Common_Apple.usd"
FIR = _VEG + "/brownstone/Assets/Vegetation/Trees/Douglas_Fir.usd"
ASPEN = _VEG + "/brownstone/Assets/Vegetation/Trees/Largetooth_Aspen.usd"
BEECH = _VEG + "/brownstone/Assets/Vegetation/Trees/American_Beech.usd"

_AEC_NAT = "airstack://scene_gen/assets/aec/brownstone/Materials/Base/Natural"
GRASS_PNG = _AEC_NAT + "/Grass_Cut/Grass_Cut_BaseColor.png"
BURNT_PNG = ("airstack://scene_gen/assets/materials/megascans/"
             "Burnt_Forest_Floor/T_uhwpehcdy_4K_B.png")
# The baked scar covers this square, centred on the row. Square because
# `ground_burn_map` bakes a square image over `region_m` — a non-square region
# would need per-axis UVs, and there is no reason to want one here.
# Grass -> light -> medium -> heavy scorch -> bare burnt ground. Four steps,
# because opaque geometry cannot fade and a single boundary carrying the whole
# change from green to black reads as a cut-out.
SCORCH_LEVELS = (0.34, 0.62, 0.88)

# OFF BY DEFAULT. The graded patch scar below is mechanically sound — it is
# the only approach that survives this renderer — but the transitions and the
# scorched-grass maps did not look good enough to keep on by default. Plain
# grass until the look is worth having.
GROUND_SCAR = os.environ.get("GROUND_SCAR", "0") not in ("0", "", "false")

BURN_SPAN_M = 320.0
BURN_CX, BURN_CY = -8.0, 24.0

GRASS_MAT = "airstack://scene_gen/assets/materials/Grass_Cut.usda"
ROUGH_MAT = "airstack://scene_gen/assets/materials/Grass_Countryside.usda"
BURNT_MAT = "airstack://scene_gen/assets/materials/megascans/Burnt_Forest_Floor.usda"
# One of the three CityPark stumps the suburban asset set blacklists. They were
# taken out of `trees` because every consumer drew from that pool WITHOUT
# filtering tags, so a stump got planted as a live tree in a lawn — the note in
# suburban.yaml says to re-add them only behind a pool of their own, and a
# burnt scene is the pool they were always for.
STUMP = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Library/Stages/"
         "CityPark/Props/SM_Stump.prop.usd")

# A cluster's species mix. Crowns have to TOUCH or the "burns as one stand"
# argument does not hold, so the radius is set against the widest crown in it.
CLUSTER_MIX = (SHUMARD, FIR, ASPEN, SHUMARD, BEECH)
CLUSTER_R = 7.5

# (id, label, kind, x_m, method)
UNITS = [
    ("0", "control (unburnt)",       "single",  -115.0, "pristine"),
    ("1", "cluster: crown scorch",   "cluster",  -70.0, "scorched"),
    ("2", "torched skeleton",        "single",   -22.0, "torched"),
    ("3", "cluster: mixed severity", "cluster",   22.0, "mixed"),
    ("4", "snapped snag",            "single",    66.0, "snag"),
    ("5", "burned off + stump",      "single",   100.0, "fallen"),
]

# BACK ROW — the same torched Black_Oak four times, and the ONLY thing that
# differs is what the wood is made of. Same species, same seed, same
# defoliation, so the row isolates the bark treatment the way the house
# bench's swatch boards isolate a texture.
#
#   soot        composite soot onto the species' OWN bark map. Keeps the tree's
#               identity; the map tiles up the trunk, which is what produced
#               the banding.
#   soot_fine   the same, at ~3x the triplanar frequency — smaller features,
#               so a repeat is harder to pick out.
#   REJECTED: binding Megascans Burnt_Forest_Floor directly. It tiled well
#               (one tile per ~9 m) and carried real normal/ORM maps, but a
#               photographed GROUND surface wrapped on a trunk reads as ground.
#               No tree in this dataset carries it; the row no longer offers it.
#   char_map    the house pipeline's own Burn_Char_Ref, world triplanar. The
#               material a collapsed building is made of, on a tree.
# Unit 3, tree by tree. WHY THIS REPLACED THE "GRADED CROWN" UNIT: a graded
# crown thinned the leaf instancers and kept them, and the rule for a burnt
# tree is now that every instancer goes — so graded collapsed into a copy of
# unit 2. What is worth a whole unit instead is the case a uniform cluster
# cannot show: a stand EDGE, where the front weakened as it passed and left
# neighbours at genuinely different levels. Unit 1 is the uniform cluster;
# this is the ragged one.
MIXED_LEVELS = ("scorched", "scorched", "torched", "snag", "fallen")

MAT_ROW_Y = 58.0
MAT_ROW = [
    ("A", "soot (own bark)",     -40.0, "soot"),
    ("B", "soot, fine grain",      0.0, "soot_fine"),
    ("C", "house char map",       40.0, "char_map"),
]

# Which units smoke, and how hard. A cold snag does not; a bed of burnt duff
# and a fallen bole do, at ground level — `fire._plume_z` already refuses to
# let a smoke state climb, which is right here for exactly the reason it is
# right on a collapsed house.
FIRE_STATE = {"scorched": None, "torched": "smoulder", "mixed": "smoulder",
              "snag": "residual", "fallen": "residual", "pristine": None}


def _mat(stage, key, url):
    """Reference a material layer and return its prim path."""
    path = "/World/ground/materials/" + key
    prim = stage.DefinePrim(Sdf.Path(path))
    prim.GetReferences().AddReference(sg._join_asset_root(url, ""))
    prim.Load()
    return path


def build_ground_and_light(stage, ssf):
    """Grass under the row, and even daylight over it."""
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground"))
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/ground/materials"))
    grass = _mat(stage, "grass", GRASS_MAT)
    rough = _mat(stage, "grass_rough", ROUGH_MAT)
    burnt = _mat(stage, "burnt", BURNT_MAT)

    # Two sheets: rough countryside grass everywhere, mown lawn as a band under
    # the row. Not decoration — a burnt tree is judged against the ground it
    # stands on, and one uniform green makes every unit look identical from
    # the waist down.
    sg._make_plane_mesh(stage, "/World/ground/base", -260.0, -260.0,
                        260.0, 260.0, -0.02, 3.0, ssf,
                        display_color=(0.21, 0.31, 0.15),
                        mat_prim_path=rough)
    sg._make_plane_mesh(stage, "/World/ground/lawn", -170.0, -40.0,
                        160.0, 92.0, 0.0, 3.0, ssf,
                        display_color=(0.24, 0.36, 0.17),
                        mat_prim_path=grass)

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/domeLight"))
    dome.CreateIntensityAttr(950.0)
    dome.CreateColorAttr(Gf.Vec3f(0.74, 0.78, 0.86))

    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/keyLight"))
    key.CreateIntensityAttr(2400.0)
    key.CreateAngleAttr(0.9)
    key.AddRotateXYZOp().Set(Gf.Vec3f(-46.0, 0.0, 32.0))
    return {"grass": grass, "rough": rough, "burnt": burnt}


def _tree(usd, x, y, yaw, scale=0.01):
    return {"usd": sg._join_asset_root(usd, ""), "x_m": float(x),
            "y_m": float(y), "z_m": 0.0, "yaw_deg": float(yaw),
            "scale": float(scale), "category": "tree"}


def build_placements(rng):
    """One placement list, plus the per-unit index ranges."""
    pl, units = [], []
    for uid, label, kind, x, method in UNITS:
        if ONLY and uid not in ONLY:
            continue
        i0 = len(pl)
        if kind == "single":
            # Black_Oak for the singles: a 25 m crown and a 17 m bole is the
            # most legible subject for the geometry methods, and it is the one
            # species whose crown carries a WOOD instancer as well as leaf
            # ones — which is what a torched skeleton is made of.
            pl.append(_tree(BLACK_OAK, x, 0.0, rng.uniform(0.0, 360.0)))
        else:
            for k, usd in enumerate(CLUSTER_MIX):
                a = 2.0 * math.pi * k / len(CLUSTER_MIX) + rng.uniform(-0.3, 0.3)
                r = CLUSTER_R * (0.35 + 0.65 * math.sqrt(rng.random()))
                pl.append(_tree(usd, x + math.cos(a) * r,
                                math.sin(a) * r * 0.8,
                                rng.uniform(0.0, 360.0)))
        units.append({"id": uid, "label": label, "kind": kind, "x": x,
                      "method": method, "i0": i0, "i1": len(pl)})

    if not ONLY or "mat" in ONLY:
        for mid, label, x, style in MAT_ROW:
            i0 = len(pl)
            # SAME YAW, same species, same everything. A different rotation
            # per tree would be enough to make one look darker than another
            # purely by which side the key light is on.
            pl.append(_tree(BLACK_OAK, x, MAT_ROW_Y, 35.0))
            units.append({"id": mid, "label": "bark: " + label,
                          "kind": "single", "x": x, "y": MAT_ROW_Y,
                          "method": "torched", "bark": style,
                          "i0": i0, "i1": len(pl)})
    return pl, units


def bark_material(style, mats):
    """Resolve a bark treatment to (material prim path, triplanar scale)."""
    if style == "char_map":
        return mats["char"], (0.28, 0.28)
    if style == "soot_fine":
        # Repeats-per-metre, so LARGER means smaller features: 0.85 puts a
        # tile across ~1.2 m instead of ~3.6 m.
        return "", (0.85, 0.85)
    return "", (0.28, 0.28)


def apply_method(stage, unit, trees, rng, out_parent, mats):
    """Run one unit's damage method over its trees. Returns (statics, loose)."""
    method = unit["method"]
    statics, loose = [], []
    bole_mat, bole_uv = bark_material(unit.get("bark", "soot"), mats)

    for k, q in enumerate(trees):
        path = q["prim_path"]
        # A mixed cluster is just per-tree levels — no separate code path, and
        # that is the point: if the stand edge needs its own machinery then
        # the level model is not carrying its weight.
        level = MIXED_LEVELS[k % len(MIXED_LEVELS)] if method == "mixed" \
            else method
        res = veg.burn_tree(stage, path, level, PARENT, out_parent, rng,
                            bole_material=bole_mat,
                            bole_scale_uv=bole_uv, verbose=True)
        q["_level"] = level
        statics += res["statics"]
        loose += res["loose"]
        q["_info"] = res["info"]
    return statics, loose


def bake_ground_scar(stage, units, placements, mats, ssf, size=2048):
    """One baked grass-to-burnt map over the whole row. Returns a prim path.

    WHAT ACTUALLY HAPPENS TO THE GROUND, which is the question behind the
    gaps. A surface fire burns the litter and grass across EVERYTHING it runs
    over — it does not stop at the trees. So a burnt stand has continuous
    black ground under and BETWEEN its trees, and the green strips left
    between the per-tree rings were saying the opposite: that only the ground
    touching each trunk burned, which no fire does.

    Severity is what varies, not presence. Under a tree and in heavy fuel the
    fire sits and burns down to pale ash and bare soil; out in open grass it
    is a fast flashy run that blackens and moves on. And a real scar is a
    MOSAIC — an irregular fingered outline with unburned islands the fire
    skipped where the ground was wetter or something broke the run.

    Which is exactly what `scorch.ground_burn_map` was written to bake and
    has never been called: it takes the fire's own field, mixes grass into
    burnt by it, and layers the mosaic and the islands on top. A tiled burnt
    material cannot express any of that, which is why painting the whole floor
    with one did not look right.

    The field here is per-tree rather than a fire front — this bench has no
    front — so it is the union of a soft disc at each BURNT tree, sized off
    its own crown. Overlapping discs in a cluster merge into one continuous
    scar, which is the gap closing.
    """
    import numpy as np

    from disaster import scorch

    if not GROUND_SCAR:
        print("[tree_damage] ground: plain grass (GROUND_SCAR=1 for the "
              "graded scar patches)")
        return []

    burnt = []
    for u in units:
        if u["method"] == "pristine":
            continue
        for q in placements[u["i0"]:u["i1"]]:
            prim = stage.GetPrimAtPath(q.get("prim_path") or "")
            r = 6.0
            if prim and prim.IsValid() and prim.IsActive():
                bb = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    [UsdGeom.Tokens.default_]).ComputeWorldBound(
                        prim).ComputeAlignedRange()
                if not bb.IsEmpty():
                    s = bb.GetSize()
                    r = max(3.5, 0.5 * max(float(s[0]), float(s[1])))
            # A crown-scorched tree stood in a fire that ran PAST it, so the
            # ground under it burned even though the tree barely did.
            burnt.append((q["x_m"], q["y_m"], r,
                          0.85 if q.get("_level", u["method"]) == "scorched"
                          else 1.0,
                          u["kind"] == "cluster"))
    if not burnt:
        return []

    # VECTORISED. `ground_burn_map` otherwise calls the field function once
    # per pixel through `np.vectorize`, which at a million pixels and fourteen
    # trees is tens of millions of Python calls.
    ys, xs = np.mgrid[0:size, 0:size]
    X = (xs / float(size) - 0.5) * BURN_SPAN_M + BURN_CX
    Y = (0.5 - ys / float(size)) * BURN_SPAN_M + BURN_CY

    # TWO FIELDS, BECAUSE THE GROUND HAS THREE STATES, NOT TWO.
    #
    #   grass            outside everything
    #   scorched grass   the transition, and the ONLY treatment a lone tree
    #                    gets — its own texture with soot composited onto it,
    #                    exactly what the walls and the boles get
    #   burnt floor      bare consumed ground, and only where a CLUSTER stood
    #
    # The physical argument is fuel continuity. A stand's crowns and litter
    # feed each other, so the fire sits in it and burns the ground out
    # completely; one tree in the open has nothing to sustain that, so the
    # grass under it chars and survives. Blending straight from grass to bare
    # burnt floor skips the state that most of a real burn scar is actually
    # in, which is why it read as a texture swap rather than as fire damage.
    f_scorch = np.zeros((size, size), dtype=np.float64)
    f_burnt = np.zeros((size, size), dtype=np.float64)
    for tx, ty, r, amp, in_cluster in burnt:
        d = np.hypot(X - tx, Y - ty)
        # Generous, with a long tail: the scorch is what has to reach out to
        # the grass and what has to merge between neighbours.
        f_scorch = np.maximum(
            f_scorch, np.clip((2.4 * r - d) / max(1e-6, 1.9 * r), 0.0, 1.0)
            * amp)
        if in_cluster:
            # Tight, so the bare ground sits INSIDE the scorched ring rather
            # than reaching its edge. Nesting is what makes it a transition.
            f_burnt = np.maximum(
                f_burnt,
                np.clip((1.25 * r - d) / max(1e-6, 0.85 * r), 0.0, 1.0) * amp)

    # PATCHES, NOT A MASKED OVERLAY. See `veg.scar_patch` — the overlay was
    # correct on paper and did not survive this renderer's USD-to-MDL
    # translation, which dropped the opacity and drew the whole plate opaque.
    #
    # A GRADED STACK, NOT TWO STATES. Opaque geometry cannot fade, so the
    # transition has to be built out of STEPS: light scorch furthest out,
    # then medium, then heavy, then bare burnt ground in the middle. Four
    # levels is enough that no single boundary carries the whole change, which
    # is what made grass-to-burnt read as a cut-out.
    #
    # And each boundary is INTERLEAVED rather than drawn: satellites of the
    # inner material are scattered out past the edge and satellites of the
    # outer one in behind it, so the two materials stipple through each other
    # over a couple of metres. That is how an opaque renderer gets a blend.
    made = []

    # SEVERAL MAPS PER LEVEL, EACH A DIFFERENT SEED, and different projection
    # scales and offsets on top. One map applied to one big patch is what
    # produced "a repeated pattern across the area": a single soot texture
    # tiling every ~3 m over a 30 m patch is ten copies of the same blotches.
    # Band-limiting kills the recognisable large features; varying the map,
    # the scale AND the world offset between neighbouring patches kills the
    # alignment that makes a repeat legible at all.
    levels = []
    for li, cov in enumerate(SCORCH_LEVELS):
        variants = []
        for vi in range(3):
            png = scorch.scorched_texture(
                sg._join_asset_root(GRASS_PNG, ""), cov,
                np.random.default_rng(SEED * 100 + li * 10 + vi),
                from_below=False, wash_weight=0.0, streak_stretch=1.0,
                band=(0.020, 0.34), salt="ground{0}-{1}".format(li, vi),
                verbose=(vi == 0))
            if not png:
                continue
            sc = 0.30 + 0.07 * vi
            variants.append(damage._pbr(
                stage, "{0}/GroundLooks/scorch_{1}_{2}".format(PARENT, li, vi),
                (1.0, 1.0, 1.0), 0.95, png, scale_uv=(sc, sc),
                offset_uv=(vi * 7.3, vi * 4.1)))
        levels.append(variants)

    burnt_mats = [damage._pbr(
        stage, "{0}/GroundLooks/burnt_{1}".format(PARENT, vi),
        (1.0, 1.0, 1.0), 0.94, sg._join_asset_root(BURNT_PNG, ""),
        scale_uv=(0.11 + 0.02 * vi, 0.11 + 0.02 * vi),
        offset_uv=(vi * 5.7, vi * 9.3)) for vi in range(3)]

    scope = "/World/ground/scar"
    UsdGeom.Scope.Define(stage, Sdf.Path(scope))
    prng = random.Random(SEED + 77)
    k = 0

    def _patch(mats_, x, y, rad, z, tag, n=64, jit=0.30):
        if not mats_:
            return
        made.append(veg.scar_patch(
            stage, "{0}/{1}".format(scope, tag), x, y, rad, prng,
            mat_prim_path=str(prng.choice(mats_).GetPath()),
            z_m=z, ssf=ssf, n=n, jitter=jit))

    for tx, ty, r, amp, in_cluster in burnt:
        # Outermost first and lowest, so an inner level always draws over the
        # one outside it rather than fighting it for depth.
        rings = [(levels[0], 1.95, 0.020), (levels[1], 1.45, 0.030),
                 (levels[2], 1.00, 0.040)]
        for ri, (mats_, fr, z0) in enumerate(rings):
            z = z0 + k * 0.0009
            _patch(mats_, tx, ty, r * fr * (0.88 + 0.24 * amp), z,
                   "s{0}_{1:03d}".format(ri, k))
            # Satellites straddling this ring's own edge, half outside and
            # half in, so the boundary is a scatter rather than a line.
            for j in range(prng.randint(5, 9)):
                ang = prng.uniform(0.0, 2.0 * math.pi)
                d = r * fr * prng.uniform(0.80, 1.32)
                _patch(mats_, tx + math.cos(ang) * d, ty + math.sin(ang) * d,
                       r * prng.uniform(0.14, 0.36), z,
                       "s{0}_{1:03d}_f{2}".format(ri, k, j), n=32, jit=0.44)
        # Bare burnt ground ONLY under a cluster, and NESTED INSIDE the
        # scorch — fuel continuity is what lets a fire sit long enough to
        # take the ground out completely, and a lone tree has none.
        if in_cluster:
            z = 0.050 + k * 0.0009
            _patch(burnt_mats, tx, ty, r * 0.70, z, "b{0:03d}".format(k),
                   jit=0.26)
            for j in range(prng.randint(4, 7)):
                ang = prng.uniform(0.0, 2.0 * math.pi)
                d = r * prng.uniform(0.55, 1.00)
                _patch(burnt_mats, tx + math.cos(ang) * d,
                       ty + math.sin(ang) * d, r * prng.uniform(0.12, 0.30),
                       z, "b{0:03d}_f{1}".format(k, j), n=32, jit=0.46)
        k += 1

    print("[tree_damage] ground: {0} patch(es) over {1} level(s), {2} burnt "
          "tree(s), {3} in clusters".format(
              len(made), len(SCORCH_LEVELS) + 1, len(burnt),
              sum(1 for b in burnt if b[4])))
    return made


def add_ash_rings(stage, unit, trees, rng, mats, ssf):
    """A burnt patch at each trunk base, plus ash mounds where fuel burned out.

    The ring is where the duff went; the MOUNDS are what is left of it, and
    of whatever heavy limb lay there smouldering. A ring on its own reads as a
    decal painted on the grass — it needs something with height standing in it
    to become part of the scene.
    """
    if unit["method"] == "pristine":
        return 0
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n = 0
    heavy = unit["method"] in ("torched", "snag", "fallen", "stump")
    for k, q in enumerate(trees):
        prim = stage.GetPrimAtPath(q.get("prim_path") or "")
        r_m = 2.2
        if prim and prim.IsValid() and prim.IsActive():
            rng_box = bc.ComputeWorldBound(prim).ComputeAlignedRange()
            if not rng_box.IsEmpty():
                s = rng_box.GetSize()
                # A third of the crown radius. The duff burns out under the
                # canopy, where the litter is, not out in the open grass.
                r_m = max(1.6, 0.30 * 0.5 * max(float(s[0]), float(s[1])))
        # NO FLAT RING ANY MORE. The baked ground scar covers what the rings
        # were for and covers it continuously, so a disc on top of it is both
        # redundant and the thing that drew the eye to where it ENDED. What is
        # still worth having is relief — ash has height, a painted map does
        # not — so the mounds stay.
        n += 1
        if not heavy:
            continue
        for j in range(rng.randint(2, 4)):
            a = rng.uniform(0.0, 2.0 * math.pi)
            d = r_m * rng.uniform(0.15, 0.85)
            veg.ash_pile(
                stage, "{0}/burn/pile_{1}_{2:02d}_{3}".format(
                    PARENT, unit["id"], k, j),
                q["x_m"] + math.cos(a) * d, q["y_m"] + math.sin(a) * d,
                rng.uniform(0.55, 1.35), rng.uniform(0.10, 0.30), rng,
                mat_prim_path=mats["ash"], ssf=ssf)
    return n


def add_labels(stage, units):
    """A low marker post at each unit, so the row is readable in the viewport."""
    UsdGeom.Scope.Define(stage, Sdf.Path("/World/markers"))
    for u in units:
        p = "/World/markers/unit_{0}".format(u["id"])
        m = UsdGeom.Cube.Define(stage, Sdf.Path(p))
        m.CreateSizeAttr(1.0)
        m.CreateDisplayColorAttr([Gf.Vec3f(0.92, 0.86, 0.20)] if not u.get("bark")
                                 else [Gf.Vec3f(0.25, 0.62, 0.95)])
        x = UsdGeom.Xformable(m)
        x.AddTranslateOp().Set(Gf.Vec3d(u["x"], u.get("y", 0.0) - 20.0, 0.9))
        x.AddScaleOp().Set(Gf.Vec3f(0.35, 0.35, 1.8))


def main():
    timeline = omni.timeline.get_timeline_interface()
    timeline.stop()

    usd_ctx = omni.usd.get_context()
    usd_ctx.new_stage()
    stage = usd_ctx.get_stage()
    if stage is None:
        raise RuntimeError("Failed to create a new stage")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Xform.Define(stage, Sdf.Path("/World/stage"))

    _mpu, ssf = get_stage_meters_per_unit(stage)
    mats = build_ground_and_light(stage, ssf)

    rng = random.Random(SEED)
    placements, units = build_placements(rng)

    # NO `instance_categories`. `tree` is one of the suburb's, and an
    # instanceable prim cannot be authored into and cannot be read through —
    # every pass below would silently do nothing, which is the oldest trap in
    # the bug catalogue. `resolver=None` likewise: a tree's origin is its
    # trunk base, which is exactly where we want it, and the centroid
    # correction would shift it by half a crown.
    sg.apply_placements(stage, placements, PARENT, ssf)
    for _ in range(30):
        omni.kit.app.get_app().update()

    fracture.ensure_deps()
    out_parent = PARENT + "/tree_debris"
    UsdGeom.Scope.Define(stage, Sdf.Path(out_parent))
    UsdGeom.Scope.Define(stage, Sdf.Path(PARENT + "/burn"))

    # The house pipeline's own burn maps, so the back row can put one on a
    # tree and be judged against the composited alternatives.
    burn_looks = damage.char_materials(stage, PARENT)
    mats["char"] = str(burn_looks["char"][0].GetPath())
    mats["ash"] = str(burn_looks["ash"][0].GetPath())

    # Unit ids are "0".."5" and "A".."D", so a per-unit seed cannot be int().
    def _useed(u, salt):
        return random.Random(SEED + salt * sum(ord(c) for c in u["id"]))

    all_statics, all_loose = [], []
    for u in units:
        trees = placements[u["i0"]:u["i1"]]
        print("\n[tree_damage] === unit {0}: {1} ({2} tree(s), x={3:+.0f}) ==="
              .format(u["id"], u["label"], len(trees), u["x"]))
        s, l = apply_method(stage, u, trees, _useed(u, 17), out_parent, mats)
        n_ring = add_ash_rings(stage, u, trees, _useed(u, 91), mats, ssf)
        u["statics"], u["loose"], u["rings"] = s, l, n_ring
        all_statics += s
        all_loose += l
        print("[tree_damage] unit {0}: {1} standing piece(s), {2} loose, "
              "{3} burn ring(s)".format(u["id"], len(s), len(l), n_ring))

    # The stump prop, beside unit 5 — the blacklisted asset in the one scene it
    # was always meant for. Nucleus-hosted, so it is placed on its own and a
    # failure to resolve costs the comparison, not the bench.
    u5 = [u for u in units if u["id"] == "5"]
    if u5:
        stump = _tree(STUMP, u5[0]["x"] + 11.0, -6.0,
                      rng.uniform(0.0, 360.0))
        stump["category"] = "tree_stump"
        sg.apply_placements(stage, [stump], PARENT, ssf)
        for _ in range(10):
            omni.kit.app.get_app().update()
        sp = stage.GetPrimAtPath(stump.get("prim_path") or "")
        if sp and sp.IsValid():
            m = damage.bound_material(stage, stump["prim_path"])
            tex = veg.material_texture(m) if m is not None else None
            mat = damage.scorched_material(stage, PARENT, m,
                                           damage.bucket(0.95),
                                           from_above=False, texture=tex)
            if mat is not None:
                for prim in Usd.PrimRange(sp):
                    if prim.IsA(UsdGeom.Mesh):
                        UsdShade.MaterialBindingAPI(prim).Bind(mat)
            veg.ash_ring(stage, PARENT + "/burn/ring_stump",
                         stump["x_m"], stump["y_m"], 2.4,
                         random.Random(SEED + 3),
                         mat_prim_path=mats["burnt"], ssf=ssf)

    scar = bake_ground_scar(stage, units, placements, mats, ssf)

    add_labels(stage, units)
    for _ in range(20):
        omni.kit.app.get_app().update()

    # SETTLE. Only the snag's splinters and the fallen bole are loose; every
    # standing tree is untouched geometry and stays out of the solver entirely.
    if all_loose:
        # HULLS FOR THE STICKS, DECOMPOSITION ONLY FOR THE BOLES. A fallen
        # bole is a trunk with limbs off it, and its convex hull is the blob
        # containing every limb tip — a hull collider rests the trunk on that
        # blob, floating clear of the ground, so those pieces genuinely need
        # decomposing. A debris stick is convex to within its own bark and
        # gains nothing from it; at three thousand pieces, cooking a
        # decomposition for each one is most of the start-up time.
        approx = {p: "convexDecomposition" for p in all_loose
                  if "/fallen_" in p or "/snag_" in p}
        settle.run(stage, all_loose,
                   all_statics + ["/World/ground/base"] + list(scar or []),
                   steps=SETTLE_STEPS, kick=0.10, rng=random.Random(SEED),
                   bake_result=BAKE, dynamic_approximation="convexHull",
                   approx_map=approx)
        print("[tree_damage] colliders: {0} decomposed, {1} hulls".format(
            len(approx), len(all_loose) - len(approx)))
        if not BAKE:
            print("[tree_damage] bodies left LIVE — the timeline is playing, "
                  "so they keep settling; BAKE=1 to freeze them")
    else:
        print("[tree_damage] nothing loose to settle")

    # Smoke. Measured off what is actually left standing, for the same reason
    # the house bench measures: a plume placed at the authored height of a tree
    # that has since come down hangs in the air over nothing.
    fire.setup_flow_stack(stage, density_cell_size_m=0.1, max_blocks=16384,
                          scene_scale_factor=1.0)
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n_em = 0
    for u in units:
        state = FIRE_STATE.get(u["method"])
        # NO SMOKE ON THE MATERIAL ROW. Its whole job is letting you judge a
        # bark treatment, and a smoulder plume drifting across it is the one
        # thing guaranteed to make two materials look different for a reason
        # that has nothing to do with the materials.
        if state is None or u.get("bark"):
            continue
        trees = placements[u["i0"]:u["i1"]]
        pts, top = [], 0.0
        for q in trees:
            prim = stage.GetPrimAtPath(q.get("prim_path") or "")
            if prim and prim.IsValid() and prim.IsActive():
                r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
                if not r.IsEmpty():
                    top = max(top, float(r.GetMax()[2]))
            pts.append({"x_m": q["x_m"], "y_m": q["y_m"], "z_m": 0.0,
                        "category": "tree"})
        n_em += fire.add_structure_fire(
            stage, pts, state, fire.FLOW_ROOT, "t{0}".format(u["id"]),
            _useed(u, 200), max_emitters=3 if u["kind"] == "single" else 5,
            base_z=0.0, top_z=max(2.0, top * 0.25),
            strength=1.0 if u["kind"] == "single" else 1.25)
    print("[tree_damage] {0} smoke emitters".format(n_em))

    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/benchCam"))
    cam.CreateFocalLengthAttr(16.0)
    cam.AddTranslateOp().Set(Gf.Vec3d(-5.0, -205.0, 74.0))
    cam.AddRotateXYZOp().Set(Gf.Vec3f(72.0, 0.0, 0.0))
    try:
        import omni.kit.viewport.utility as vp
        vp.get_active_viewport().camera_path = "/World/benchCam"
    except Exception as exc:
        carb.log_warn("could not retarget the viewport: {0}".format(exc))

    for _ in range(20):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 74)
    print("BURNT-TREE BENCH")
    print("  FRONT ROW (y=0)   — method, left to right, increasing severity")
    for u in units:
        if u.get("bark"):
            continue
        print("    x={0:+7.1f}  {1}  {2:<24s} {3:2d} tree(s), {4:2d} loose"
              .format(u["x"], u["id"], u["label"], u["i1"] - u["i0"],
                      len(u.get("loose", []))))
    print("  BACK ROW (y={0:+.0f})  — same torched oak, four bark treatments"
          .format(MAT_ROW_Y))
    for u in units:
        if not u.get("bark"):
            continue
        print("    x={0:+7.1f}  {1}  {2}".format(u["x"], u["id"], u["label"]))
    print("  TREE_UNITS=1,4 rebuilds only those units (or 'mat' for the back")
    print("  row). Physics is baked to static geometry; BAKE=0 leaves it live.")
    print("=" * 74 + "\n")

    timeline.play()
    app = omni.kit.app.get_app()
    while simulation_app.is_running():
        app.update()
    timeline.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
