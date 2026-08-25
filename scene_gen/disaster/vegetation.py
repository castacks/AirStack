"""vegetation stage — what a wildfire leaves behind on a tree.

WHY THIS IS INSTANCER WORK, NOT MESH WORK
-----------------------------------------
Every tree in the AEC library is built the same way, and it is the single fact
that makes all of this cheap:

    ONE solid Mesh for the bole (trunk plus primary limbs), bound to a BARK
    material, plus N UsdGeomPointInstancers, each holding exactly ONE
    prototype, and each prototype bound to either a LEAF/NEEDLE material or a
    BARK one.

    Black_Oak     Black_Oak_base  41,279 pts, z 0 -> 17.15 m
                  4 leaf instancers x 747 + 1 WOOD instancer x 747
    Shumard_Oak   Shumard_Oak_trunk 27,276 pts, 3 leaf instancers x 90
    Douglas_Fir   Douglas_Fir_trunk  3,108 pts,
                  3 needle instancers + 4 wood instancers (52-110 each)

A fire consumes FINE FUEL — leaves, needles, twigs under about 5 mm — and
leaves the wood. That maps exactly onto "hide the leaf instancers, keep the
wood ones", and `invisibleIds` does it with a single array write: no mesh
rebuild, no fracture, no boolean. Keeping the wood instancers is the part that
matters visually; hide everything and a torched oak is a bare pole instead of
a skeleton.

It is also a large RENDER SAVING rather than a cost. Black_Oak's four leaf
instancers expand to ~2.68M points against a 41k trunk, so a burnt stand is
roughly 30x cheaper than a green one. Every cost note in `suburban.yaml` is
about green trees and none of it applies here.

WHAT ACTUALLY HAPPENS TO A TREE, AND THE ANSWER IS: IT STAYS UP
---------------------------------------------------------------
The intuition that a burnt tree falls over is wrong on the timescale a capture
cares about. Immediately behind a front a burnt stand is a field of STANDING
black poles; trees come down over months to years, as roots rot and wind gets
at weakened boles. A scene full of downed trunks reads as windthrow — a
tornado or a microburst — not as a fire. So `fallen` is deliberately rare in
the level mix, and biased toward whatever burned longest at the base.

The ladder, matched to `damage.LEVELS` so one burn age drives both:

    pristine    control, untouched
    scorched    CROWN SCORCH. The plume kills foliage without burning it:
                leaves turn orange-brown from the bottom up over days and stay
                on the tree. Fully upright, FULL CROWN, char only on the lower
                bole. The most common outcome by area and the one nobody
                models — a "red belt" is most of what a mixed-severity fire
                leaves behind.
    torched     the crown burns. All fine fuel consumed, bole and primary
                limbs survive as a black skeleton. STANDING. The iconic image.
    snag        as torched, but the upper bole burned through or snapped —
                a jagged spar at a third to two thirds height, debris at its
                foot.
    fallen      burned through at the base, or root burnout. A charred stump
                plus a long charred bole on the ground, roughly radial from
                the stump.
    stump       only a low stump inside a bare ring where the duff burnt out.

TWO MORE THINGS REAL BURNS HAVE
-------------------------------
Char runs from the GROUND UP and stops: the "scorch height" on a bole is a
measured field quantity, and above it the bark is untouched. That is why
`char_bole` can split the bole at a height rather than washing the whole
thing — at low severity the black-bottom / clean-top bole IS the signature,
and a uniformly darkened trunk reads as a shading bug.

And a dense cluster burns UNIFORMLY SEVERE, because crown fire carries between
touching canopies, while an isolated tree with clearance usually gets only
scorched. Severity is therefore a property of the STAND, not of the tree, and
the bench is laid out to show that.

THE TEXTURE IS INSIDE THE MDL, NOT IN THE USD
---------------------------------------------
This cost a debugging round and it will cost the next one too. `damage.
_basecolor_texture` walks a material's shader network for an asset-valued USD
input, and on these packs there ISN'T ONE: the shader carries only
`info:mdl:sourceAsset = TreeBark_07.mdl`, and the texture lives inside that
MDL as `diffuse_texture: texture_2d("./textures/...")`. So `damage.
bound_texture()` returns None on EVERY tree, and the scorch path falls through
to plain darkening — a dim tree, not a burnt one, with no error anywhere.
`material_texture()` below reads the MDL when the USD has nothing.

LEAF MATERIALS HAVE NO OPACITY MAP
----------------------------------
Checked before writing the browning pass, because an alpha-cut leaf card whose
material gets rebuilt without its cutout turns into a solid rectangle. These
are plain OmniPBR with a diffuse texture and nothing else — the leaves are
modelled geometry, not cards — so rebuilding the material around the same
texture is safe.
"""

import hashlib
import math
import os
import re

TREE_LEVELS = ("pristine", "scorched", "torched", "snag", "fallen", "stump")

# Material identity, by MDL module name or texture filename. Checked in this
# order: a prototype called `Douglas_Fir_branchM` binds a material literally
# named `Default_Material` that resolves TreeBark_10.mdl (the defect recorded
# in assets/aec/README.md), so PRIM AND MATERIAL NAMES ARE USELESS HERE and
# only the resolved MDL module or the texture says what a surface is.
_LEAF_KEYS = ("leaf", "leaves", "needle", "foliage", "frond", "blossom",
              "petal")
_WOOD_KEYS = ("bark", "trunk", "wood", "timber", "stem", "branch")

# Dead-foliage colour. Crown-scorched leaves and needles go through a rusty
# orange "red stage" first and dry to a grey-brown over the following weeks;
# `browning` lerps between the two, so one knob covers both.
_RED_STAGE_RGB = (0.42, 0.19, 0.07)
_DRY_RGB = (0.27, 0.17, 0.11)

# level -> (foliage keep at crown base, keep at crown top, bole char coverage,
#           browning, scorch height as a fraction of bole height, geometry)
#
# `keep_base` vs `keep_top` is what makes a thinned crown read: fire enters a
# crown from below, so the lower third goes first and the top is merely
# thinned. Setting both equal gives a uniform thin, which looks like an
# unhealthy tree rather than a burnt one.
_PLAN = {
    "pristine": (1.00, 1.00, 0.00, 0.00, 0.00, None),
    # Foliage entirely PRESENT and brown. The bole is charred only up to a
    # third of its height, which is the whole tell.
    "scorched": (1.00, 1.00, 0.85, 0.85, 0.34, None),
    # Crown consumed. A few scraps survive at the very top, because a crown
    # that goes to exactly zero looks deleted rather than burnt.
    "torched":  (0.00, 0.06, 0.94, 1.00, 1.00, None),
    "snag":     (0.00, 0.00, 0.96, 1.00, 1.00, "snap"),
    "fallen":   (0.00, 0.00, 0.96, 1.00, 1.00, "topple"),
    "stump":    (0.00, 0.00, 1.00, 1.00, 1.00, "stump"),
}


def plan_for(level):
    """`(keep_base, keep_top, bole_cov, browning, scorch_h, geometry)`."""
    if level not in _PLAN:
        raise ValueError("unknown tree level {0!r}; expected one of {1}"
                         .format(level, ", ".join(TREE_LEVELS)))
    return _PLAN[level]


def level_for_age(dt, ignition_s=6.0, flame_s=180.0, smoulder_s=90.0,
                  ash_after_s=240.0):
    """`(level, fire_state)` for a tree `dt` seconds post-ignition.

    Deliberately the same signature and the same phase constants as
    `damage.level_for_age`, so a scene solves arrival times ONCE and hands the
    same number to buildings and to vegetation. Scattering tree damage
    independently is what would make a street read as a burnt row of houses
    standing in a healthy park.

    The mix is weighted to STANDING outcomes on purpose — see the module
    docstring. `fallen` needs both a long burn and a caller willing to roll
    for it, which `stand_outcome` does.
    """
    if dt < 0.0:
        return ("pristine", None)
    if dt < ignition_s:
        return ("scorched", None)
    d = dt - ignition_s
    if d < flame_s * 0.45:
        return ("scorched", "flame")
    if d < flame_s:
        return ("torched", "flame")
    d -= flame_s
    if d < smoulder_s:
        return ("torched", "smoulder")
    if d < ash_after_s:
        return ("snag", "residual")
    return ("snag", "residual")


def stand_outcome(level, rng, fall_chance=0.12, stump_chance=0.05):
    """Roll `snag` down to `fallen` or `stump` for a minority of trees.

    Kept OUT of `level_for_age` because burn age does not decide this — how
    long a tree stands after it dies is about rot, root burnout and wind, none
    of which the fire model knows. Defaults are low for the reason in the
    module docstring: a burnt stand is standing.
    """
    if level != "snag":
        return level
    r = rng.random()
    if r < stump_chance:
        return "stump"
    if r < stump_chance + fall_chance:
        return "fallen"
    return "snag"


# ---------------------------------------------------------------------------
# Material identity — reading through the MDL
# ---------------------------------------------------------------------------

_TEX2D_RE = re.compile(r'texture_2d\s*\(\s*"([^"]+)"')


def _mdl_texture(mdl_path, param="diffuse_texture"):
    """The texture a `.mdl` module assigns to *param*, as an absolute path.

    The MDL's own paths are relative to the MODULE, not to the USD layer that
    names it, so they are anchored here rather than by the USD resolver, which
    will never see them at all.
    """
    if not mdl_path or not os.path.exists(mdl_path):
        return None
    try:
        with open(mdl_path, "r", errors="ignore") as fh:
            src = fh.read()
    except OSError:
        return None
    m = re.search(re.escape(param) + r"\s*:\s*" + _TEX2D_RE.pattern, src)
    if not m:
        return None
    rel = m.group(1)
    if not rel:
        return None
    path = os.path.normpath(os.path.join(os.path.dirname(mdl_path), rel))
    return path if os.path.exists(path) else None


def _mdl_module(mat_prim):
    """Resolved path of the `.mdl` a material's shader names, or ""."""
    from pxr import Usd, UsdShade

    if not mat_prim or not mat_prim.IsValid():
        return ""
    for p in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        src = sh.GetSourceAsset("mdl")
        if src:
            return src.resolvedPath or src.path or ""
    return ""


def material_texture(mat_prim):
    """Base-colour texture for a material, THROUGH the MDL when needed.

    `damage._basecolor_texture` first, because a USD-authored input is the
    resolved, correct answer where it exists. Trees never have one.
    """
    from . import damage

    tex = damage._basecolor_texture(mat_prim)
    if tex:
        return tex
    return _mdl_texture(_mdl_module(mat_prim))


def _kind(mat_prim):
    """`"leaf"`, `"wood"` or `""` for a bound material."""
    hay = " ".join([
        os.path.basename(_mdl_module(mat_prim) or "").lower(),
        os.path.basename(material_texture(mat_prim) or "").lower(),
    ])
    if not hay.strip():
        return ""
    # Leaf first: `Pine_needles.mdl` would also match nothing in _WOOD_KEYS,
    # but a texture called `branch_leaves.png` matches both and the fine fuel
    # is what decides.
    if any(k in hay for k in _LEAF_KEYS):
        return "leaf"
    if any(k in hay for k in _WOOD_KEYS):
        return "wood"
    return ""


def _tag(info, mesh_path):
    """A legal, unique-per-tree prim name for something cut off *mesh_path*.

    Slicing the end off a path is not safe — a USD prim name must start with a
    letter or an underscore, and `"..._tree_0_1/Black_Oak_base"[-40:]` can
    start with a digit. `apply_placements` already names every placement
    `<category>_<group>_<index>`, which is a legal identifier by construction,
    so compose that with the mesh name instead of slicing.
    """
    tree = str(info.get("path", "tree")).rsplit("/", 1)[-1]
    leaf = str(mesh_path).rsplit("/", 1)[-1]
    name = "{0}__{1}".format(tree, leaf)
    return "".join(c if (c.isalnum() or c == "_") else "_" for c in name)


def _bound(prim):
    """The material a prim (or its first GeomSubset) binds, or None."""
    from pxr import UsdGeom, UsdShade

    subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
    for t in ([s.GetPrim() for s in subs] or [prim]):
        m = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
        if m and m.GetPrim().IsValid():
            return m.GetPrim()
    return None


# ---------------------------------------------------------------------------
# Survey — which prim is what
# ---------------------------------------------------------------------------

def survey(stage, tree_path, verbose=False):
    """Classify one placed tree. Everything else here takes this dict.

    Returns:
        bole        [(mesh path, material path, texture)] — the woody trunk,
                    i.e. meshes NOT inside a PointInstancer
        leaf_pi     [(instancer path, n instances)]
        wood_pi     [(instancer path, n instances)]
        unknown     prim paths whose material could not be classified
        leaf_mats   {material path: texture} for the crown
        z           (lo, hi) world z of the whole tree, metres
        crown_z     (lo, hi) world z spanned by the leaf instancers

    UNKNOWN DEFAULTS TO WOOD, and is reported rather than guessed at. Guessing
    "leaf" would DELETE geometry nobody identified, and the failure mode of
    that — a tree with a chunk silently missing — is far worse than the
    failure mode of guessing wood, which is a bit of green left on a burnt
    tree and visible immediately.
    """
    from pxr import Gf, Usd, UsdGeom

    root = stage.GetPrimAtPath(tree_path)
    out = {"bole": [], "leaf_pi": [], "wood_pi": [], "unknown": [],
           "unbound": [], "leaf_mats": {}, "bark_mat": "",
           "z": (0.0, 0.0), "crown_z": (0.0, 0.0), "path": tree_path}
    if not root or not root.IsValid():
        return out
    if root.IsInstance():
        # SKILL.md's oldest trap, and it applies here verbatim: a prim marked
        # instanceable cannot be authored into and PrimRange will not descend
        # it, so every pass below would silently do nothing.
        print("[veg] {0} IS INSTANCEABLE — nothing can be authored into it. "
              "Place burnable trees un-instanced.".format(tree_path))
        return out

    xf = UsdGeom.XformCache()
    pi_paths, proto_paths = [], set()
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != "PointInstancer":
            continue
        pi_paths.append(prim.GetPath())
        for t in UsdGeom.PointInstancer(prim).GetPrototypesRel().GetTargets():
            proto_paths.add(str(t))

    def _is_proto(path):
        s = str(path)
        if any(s == p or s.startswith(p + "/") for p in proto_paths):
            return True
        return any(s.startswith(str(p) + "/") for p in pi_paths)

    zs, crown_zs = [], []
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "PointInstancer":
            pi = UsdGeom.PointInstancer(prim)
            n = len(pi.GetProtoIndicesAttr().Get() or [])
            kinds = set()
            for t in pi.GetPrototypesRel().GetTargets():
                pp = stage.GetPrimAtPath(t)
                if not pp or not pp.IsValid():
                    continue
                for q in Usd.PrimRange(pp):
                    if not q.IsA(UsdGeom.Mesh):
                        continue
                    mp = _bound(q)
                    k = _kind(mp)
                    kinds.add(k)
                    if mp is None:
                        # BINDS NOTHING AT ALL, so it renders untextured —
                        # white or flat grey, whatever the fallback is. This
                        # is not a classification failure, it is a defect in
                        # the asset: Black_Oak's woody branchlet prototype
                        # ships with no material. `bind_bark` repairs it from
                        # the tree's own bark.
                        out["unbound"].append(str(q.GetPath()))
                    if k == "leaf" and mp is not None:
                        out["leaf_mats"][str(mp.GetPath())] = \
                            material_texture(mp)
            if "leaf" in kinds:
                out["leaf_pi"].append((str(prim.GetPath()), n))
                m = xf.GetLocalToWorldTransform(prim)
                for p in (pi.GetPositionsAttr().Get() or []):
                    crown_zs.append(m.Transform(Gf.Vec3d(*p))[2])
            else:
                out["wood_pi"].append((str(prim.GetPath()), n))
                if "" in kinds and not kinds - {""}:
                    out["unknown"].append(str(prim.GetPath()))
            continue

        if not prim.IsA(UsdGeom.Mesh) or _is_proto(prim.GetPath()):
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if not r.IsEmpty():
            zs += [r.GetMin()[2], r.GetMax()[2]]
        mp = _bound(prim)
        k = _kind(mp)
        if k == "leaf":
            # A tree whose foliage is a plain mesh rather than instanced. None
            # in the AEC packs, but the CityPark/Nucleus trees are not all
            # built this way, so it has somewhere to go.
            if mp is not None:
                out["leaf_mats"][str(mp.GetPath())] = material_texture(mp)
            out["bole"].append((str(prim.GetPath()),
                                str(mp.GetPath()) if mp else "",
                                material_texture(mp), "leaf"))
            continue
        if k == "":
            out["unknown"].append(str(prim.GetPath()))
        out["bole"].append((str(prim.GetPath()),
                            str(mp.GetPath()) if mp else "",
                            material_texture(mp), "wood"))

    if zs:
        out["z"] = (float(min(zs)), float(max(zs)))
    out["crown_z"] = ((float(min(crown_zs)), float(max(crown_zs)))
                      if crown_zs else out["z"])
    for _p, mat, _tex, kind in out["bole"]:
        if kind == "wood" and mat:
            out["bark_mat"] = mat
            break

    if verbose:
        n_leaf = sum(n for _, n in out["leaf_pi"])
        n_wood = sum(n for _, n in out["wood_pi"])
        print("[veg] {0}: bole {1} mesh, crown {2} leaf inst / {3} wood inst, "
              "z {4:.1f}-{5:.1f} m".format(
                  tree_path.rsplit("/", 1)[-1], len(out["bole"]),
                  n_leaf, n_wood, out["z"][0], out["z"][1]))
    if out["unknown"]:
        print("[veg] {0}: {1} prim(s) with no identifiable material, kept as "
              "wood: {2}".format(tree_path.rsplit("/", 1)[-1],
                                 len(out["unknown"]),
                                 ", ".join(p.rsplit("/", 1)[-1]
                                           for p in out["unknown"][:4])))
    if out["unbound"]:
        print("[veg] {0}: {1} mesh(es) bind NO material (asset defect; "
              "bind_bark gives them the tree's own bark): {2}".format(tree_path.rsplit("/", 1)[-1], len(out["unbound"]),
                            ", ".join(p.rsplit("/", 1)[-1]
                                      for p in out["unbound"][:4])))
    return out


def bind_bark(stage, info, verbose=False):
    """Give every unbound mesh in the tree the tree's own bark material.

    An ASSET DEFECT, not a pipeline one: Black_Oak's woody branchlet
    prototype — the 123-point mesh its 747-instance crown scatterer points at
    — ships with no material binding, so those branchlets render untextured
    while the trunk beside them is bark. It is invisible on a green tree
    because the leaves hide the branchlets; strip the leaves for a torched
    skeleton and suddenly the entire crown is the unshaded thing you notice.

    Repaired from the tree's OWN bark material, so the branchlets match the
    trunk and then take the same char pass as everything else woody.
    """
    from pxr import UsdShade

    if not info.get("unbound") or not info.get("bark_mat"):
        return 0
    mat = UsdShade.Material(stage.GetPrimAtPath(info["bark_mat"]))
    if not mat:
        return 0
    n = 0
    for path in info["unbound"]:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
            n += 1
    if verbose:
        print("[veg]   bind_bark: {0} unbound mesh(es) repaired from {1}"
              .format(n, info["bark_mat"].rsplit("/", 1)[-1]))
    return n


# ---------------------------------------------------------------------------
# Defoliation
# ---------------------------------------------------------------------------

# What survives on a tree the fire actually got into. Decided by PRIM NAME,
# because the name is what says which part of the tree a mesh is, and the
# material only says what it is made of — `Black_Oak_branch2` binds bark and is
# still a twig.
_TRUNK_NAMES = ("trunk", "base", "bark", "bole", "stem")
_BURNS_AWAY = ("branch", "leaf", "leaves", "needle", "twig", "foliage")


def strip_to_trunk(stage, info, verbose=False):
    """Leave the trunk and burn everything else off. Returns how many went.

    THE WHOLE CROWN GOES, WOOD INCLUDED. The first cut kept the woody branch
    instancers on the argument that fire takes fine fuel and leaves wood — but
    "wood" is not the line, THICKNESS is, and every one of those scatterers
    holds branchlets a centimetre or two through. They burn. What is left of a
    torched tree is the bole and the primary limbs that are part of it, which
    on these assets is exactly the one mesh named `*_trunk` or `*_base`.

    So the rule is by name, and it is deliberately blunt:

        keep      trunk, base, bark, bole, stem
        remove    every PointInstancer, and any mesh named branch/leaf/
                  leaves/needle/twig/foliage

    Blunt is the point. The material-kind classifier was cleverer and got this
    wrong in the visible direction — it kept 747 woody branchlets per oak,
    which is what left the crown looking like a cloud of disconnected sticks.

    Anything matching neither list falls back to its material: a mesh bound to
    a leaf material goes, one bound to bark stays. That is what catches
    Common_Apple, whose meshes are named after their materials
    (`Apple_bark_Mat` / `Apple_leaf_Mat`) rather than after the part.
    """
    from pxr import Usd, UsdGeom

    root = stage.GetPrimAtPath(info.get("path") or "")
    if not root or not root.IsValid():
        return 0

    # COLLECT FIRST, THEN DEACTIVATE. `Usd.PrimRange`'s default predicate
    # skips inactive prims, so deactivating during the walk changes the
    # traversal underneath itself — the moment an instancer goes, the range
    # stops descending into it and its prototype is never examined. That is
    # invisible while the instancer is the thing being removed and becomes a
    # bug the instant anything else needs looking at down there.
    doomed = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsActive():
            continue
        name = prim.GetName().lower()
        if prim.GetTypeName() == "PointInstancer":
            doomed.append(prim)
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        # SUBSTRING, NOT A WORD MATCH. These names run the part straight into
        # a suffix with no separator — `branchL`, `branchSpR3`, `branchMwdn`,
        # `branchShape_MASH1_Instancer_0jEI` — so anything that keys off word
        # boundaries or a trailing `_` misses most of the library. Same on the
        # keep side: `Black_Oak_base`, `Shumard_Oak_trunk`, `Apple_bark_Mat`.
        if any(k in name for k in _TRUNK_NAMES):
            continue
        if any(k in name for k in _BURNS_AWAY):
            doomed.append(prim)
            continue
        if _kind(_bound(prim)) == "leaf":
            doomed.append(prim)

    n = 0
    for prim in doomed:
        if prim.IsValid() and prim.SetActive(False):
            n += 1

    # The survey's view of the tree is now stale in the one way that matters:
    # a later pass must not try to fracture or re-bind a prim that no longer
    # exists.
    info["bole"] = [e for e in info.get("bole", [])
                    if (stage.GetPrimAtPath(e[0])
                        and stage.GetPrimAtPath(e[0]).IsActive())]
    info["leaf_pi"] = []
    info["wood_pi"] = []
    info["leaf_mats"] = {}
    info["unbound"] = [p for p in info.get("unbound", [])
                       if (stage.GetPrimAtPath(p)
                           and stage.GetPrimAtPath(p).IsActive())]
    if verbose:
        print("[veg]   strip_to_trunk: {0} crown prim(s) removed, {1} trunk "
              "mesh(es) left".format(n, len(info["bole"])))
    return n


def reseat(stage, info, ground_z=0.0, verbose=False):
    """Drop the tree so what is LEFT of it sits on the ground. Returns metres.

    THE LIFT WAS MEASURED OFF GEOMETRY WE THEN DELETE. `suburb_scene` seats a
    prop with `z_m = fp["base"]`, and `base` is minus the asset's own bbox
    min-z — the offset that puts its LOWEST POINT on the ground. On a tree
    that lowest point is a drooping branch or a leaf card, not the trunk:

        Shumard_Oak   whole-tree min z  -0.533 m
                      TRUNK      min z  -0.344 m

    so the tree is lifted 0.533 m to seat the foliage, and the bole starts
    0.189 m up. Green, nobody sees it — the crown covers the gap. Strip the
    crown for a burnt tree and the trunk is left hanging in the air over its
    own missing stump, which is exactly what "the stumps of the trees seem to
    be missing so they look like they're floating" is.

    So re-seat AFTER stripping, against what actually remains. Must run before
    anything writes world-space geometry OUTSIDE the tree prim — `_char_split`
    and the geometry passes do — because moving the tree's transform does not
    move those.
    """
    from pxr import Gf, Usd, UsdGeom

    prim = stage.GetPrimAtPath(info.get("path") or "")
    if not prim or not prim.IsValid():
        return 0.0
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return 0.0
    dz = float(ground_z) - float(r.GetMin()[2])
    if abs(dz) < 1e-4:
        return 0.0

    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() != UsdGeom.XformOp.TypeTranslate:
            continue
        v = op.Get()
        if v is None:
            continue
        # Match whatever precision is already authored — `_set_xform_ops` has
        # the same problem and the same reason.
        if isinstance(v, Gf.Vec3f):
            op.Set(Gf.Vec3f(v[0], v[1], float(v[2] + dz)))
        else:
            op.Set(Gf.Vec3d(v[0], v[1], float(v[2] + dz)))
        if verbose:
            print("[veg]   reseat: {0:+.3f} m".format(dz))
        return dz
    return 0.0


def keep_instances(stage, pi_path, keep_idx, verbose=False):
    """Rewrite a PointInstancer to hold ONLY *keep_idx*. Returns how many went.

    `invisibleIds` DOES NOT HIDE ANYTHING HERE, and that cost a full round of
    "the leaves are still on all the trees". It is the correct USD answer —
    the attribute exists on every one of these instancers and setting it is a
    single array write — but this Kit's Hydra delegate does not honour it, so
    2,894 of 2,988 instances were marked invisible and every one of them still
    rendered. Nothing errors; the write succeeds and is ignored.

    So the arrays themselves are rewritten instead. Every per-instance array
    the instancer authors has to be filtered TOGETHER or they desynchronise
    and instances take each other's prototypes and poses — which is why this
    is one function rather than a line at each call site. When nothing
    survives, the instancer prim is deactivated outright: that always works,
    and it is what you would do by hand.
    """
    from pxr import UsdGeom, Vt

    prim = stage.GetPrimAtPath(pi_path)
    if not prim or not prim.IsValid():
        return 0
    pi = UsdGeom.PointInstancer(prim)
    pos = pi.GetPositionsAttr().Get()
    if not pos:
        return 0
    n_before = len(pos)
    keep = sorted(set(int(i) for i in keep_idx if 0 <= int(i) < n_before))
    if not keep:
        prim.SetActive(False)
        return n_before
    if len(keep) == n_before:
        return 0

    # EVERY per-instance array, or they desynchronise.
    for attr, ctor in (
            (pi.GetPositionsAttr(), Vt.Vec3fArray),
            (pi.GetProtoIndicesAttr(), Vt.IntArray),
            (pi.GetOrientationsAttr(), Vt.QuathArray),
            (pi.GetScalesAttr(), Vt.Vec3fArray),
            (pi.GetVelocitiesAttr(), Vt.Vec3fArray),
            (pi.GetAngularVelocitiesAttr(), Vt.Vec3fArray),
            (pi.GetIdsAttr(), Vt.Int64Array)):
        vals = attr.Get()
        if vals is None or len(vals) != n_before:
            continue
        attr.Set(ctor([vals[i] for i in keep]))
    # A stale invisibleIds indexes instances that no longer exist.
    inv = pi.GetInvisibleIdsAttr()
    if inv and inv.HasAuthoredValue():
        inv.Set(Vt.Int64Array([]))
    if verbose:
        print("[veg]     {0}: {1} -> {2} instances".format(
            pi_path.rsplit("/", 1)[-1], n_before, len(keep)))
    return n_before - len(keep)


def _smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def defoliate(stage, info, rng, keep=0.0, keep_top=None, gradient=(0.0, 0.9),
              verbose=False):
    """Hide foliage. Returns `(hidden, total, hidden_meshes)`.

    Removal goes through `keep_instances`, which rewrites the instancer's
    per-instance arrays. `invisibleIds` would be the tidier answer and this
    Kit's Hydra silently ignores it, so the leaves stayed on every tree.

    `keep` is survival at the crown BASE, `keep_top` at the top (defaults to
    `keep`, i.e. a uniform thin). `gradient` is the fractional height window
    over which one becomes the other. Fire enters a crown from below, so the
    interesting case is keep=0, keep_top>0.

    Instance heights are taken in WORLD space through the instancer's own
    transform, so the window means the same thing on a tree that has been
    scaled, rotated or dropped into a lot at an angle.

    NOT EVERY TREE IS INSTANCED, which is the trap here. Black_Oak, Shumard
    and Douglas_Fir scatter their crowns with PointInstancers; Largetooth
    Aspen, American Beech and Common Apple ship ONE PLAIN MESH literally
    called `leaves`, and Black_Oak carries four extra leaf meshes at its
    crown top ON TOP of its instancers. Walking only `leaf_pi` therefore left
    three of the six species fully green with no error anywhere. Mesh foliage
    is all-or-nothing — there are no per-instance ids to thin — so it takes a
    single roll against the survival at its own height.
    """
    from pxr import Gf, Usd, UsdGeom, Vt

    keep_top = keep if keep_top is None else keep_top
    lo_f, hi_f = gradient
    z0, z1 = info.get("crown_z", (0.0, 0.0))
    span = max(1e-6, z1 - z0)
    xf = UsdGeom.XformCache()

    def _survival(z):
        t = _smoothstep(((z - z0) / span - lo_f) / max(1e-6, hi_f - lo_f))
        return keep + (keep_top - keep) * t

    hidden = total = 0
    for path, _n in info.get("leaf_pi", []):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        pi = UsdGeom.PointInstancer(prim)
        pos = pi.GetPositionsAttr().Get() or []
        if not len(pos):
            continue
        m = xf.GetLocalToWorldTransform(prim)
        keep_idx = [i for i, p in enumerate(pos)
                    if rng.random() < _survival(m.Transform(Gf.Vec3d(*p))[2])]
        hidden += keep_instances(stage, path, keep_idx, verbose=verbose)
        total += len(pos)

    n_mesh = 0
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for entry in info.get("bole", []):
        path, _mat, _tex, kind = entry
        if kind != "leaf":
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        z = (0.5 * (r.GetMin()[2] + r.GetMax()[2]) if not r.IsEmpty()
             else 0.5 * (z0 + z1))
        if rng.random() >= _survival(z):
            # DEACTIVATE, not MakeInvisible. Same lesson as `invisibleIds`:
            # only removal is reliably removal here, and a leaf mesh that is
            # merely invisible is also still paying for itself.
            prim.SetActive(False)
            n_mesh += 1

    if verbose:
        print("[veg]   defoliate: {0}/{1} leaf instances hidden, {2} leaf "
              "mesh(es) hidden".format(hidden, total, n_mesh))
    return hidden, total, n_mesh


# ---------------------------------------------------------------------------
# Foliage that died without burning
# ---------------------------------------------------------------------------

_DEAD_CACHE = {}


def dead_foliage_texture(url, browning, rng, char=0.0, out_dir=None,
                         verbose=False):
    """Recolour a leaf map to crown-scorched brown. Returns a local path.

    LUMINANCE IS KEPT, HUE IS REPLACED. A flat brown multiply loses the veins,
    the leaf edges and the gaps between leaves, and the crown goes to mush at
    any distance — the texture's own light and shade is most of what makes a
    canopy read as a canopy. So the map's luminance is re-tinted rather than
    the colour being scaled, which keeps every bit of that detail and only
    moves the hue.

    `char` composites soot on top for a crown that partly burned as well as
    died. Zero for pure crown scorch, which is the common case and involves no
    combustion in the crown at all.
    """
    import numpy as np
    from PIL import Image

    from . import scorch

    out_dir = out_dir or scorch.OUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    key = hashlib.md5("{0}|{1:.3f}|{2:.3f}|{3}|{4}".format(
        url, browning, char, _RED_STAGE_RGB, _DRY_RGB).encode("utf-8")
    ).hexdigest()[:16]
    path = os.path.join(out_dir, "foliage_{0}.png".format(key))
    if path in _DEAD_CACHE:
        return _DEAD_CACHE[path]
    if os.path.exists(path):
        _DEAD_CACHE[path] = path
        return path

    base = scorch.read_texture(url)
    if base is None:
        if verbose:
            print("[veg] could not read leaf texture {0}".format(url))
        return None

    b = max(0.0, min(1.0, float(browning)))
    # Red stage first, drying after — one knob walks both, so a crown at the
    # front of the burn is rust and one well behind it is grey-brown.
    dry = _smoothstep((b - 0.45) / 0.55)
    tint = np.array([_RED_STAGE_RGB[i] * (1.0 - dry) + _DRY_RGB[i] * dry
                     for i in range(3)], dtype=float)

    lum = base.mean(axis=2, keepdims=True)
    # Normalise the luminance about its own mean so a dark leaf map does not
    # come back near-black; the tint sets the level, the map sets the detail.
    lum = np.clip(lum / max(1e-6, float(lum.mean())) * 0.55, 0.0, 1.35)
    dead = np.clip(lum * tint[None, None, :], 0.0, 1.0)
    out = base * (1.0 - b) + dead * b

    if char > 0.0:
        h, w = out.shape[:2]
        # SCORCH WANTS A NUMPY GENERATOR, not a `random.Random` — `_noise`
        # calls `rng.normal()`. Derived from the cache key so the same map is
        # byte-identical across runs, which is the same trick
        # `damage.scorched_material` uses for exactly the same reason.
        nrng = np.random.default_rng(int(key, 16) % (2 ** 31))
        out = scorch.composite(
            out, scorch.soot_mask(h, w, nrng, coverage=float(char),
                                  from_below=True))

    Image.fromarray((np.clip(out, 0.0, 1.0) * 255.0 + 0.5).astype("uint8"),
                    "RGB").save(path)
    _DEAD_CACHE[path] = path
    if verbose:
        print("[veg] foliage {0} browning {1:.2f} -> {2}".format(
            os.path.basename(url), b, os.path.basename(path)))
    return path


def _leaf_pbr(stage, path, texture):
    """An OmniPBR carrying a leaf map in the asset's OWN UV space.

    NOT `damage._pbr`, and the difference is the one thing that matters: that
    builder turns on `project_uvw` (world triplanar) whenever it is given a
    texture, which is right for fracture debris that carries no UVs and wrong
    for a leaf. A leaf cluster's UVs are how the atlas lands on each individual
    leaf; project it from world coordinates instead and every leaf gets a
    slice of whatever the plane happens to cross, which reads as coloured
    noise rather than as foliage.
    """
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path).AppendChild("Shader"))
    sh.CreateIdAttr("OmniPBR")
    sh.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.CreateInput("diffuse_texture",
                   Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture))
    sh.CreateInput("diffuse_color_constant",
                   Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    # Dead foliage is matt. Live leaves have a waxy cuticle and a visible
    # sheen; it is the first thing they lose, and leaving the original
    # roughness on brown leaves makes them look like painted plastic.
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(0.88)
    sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("specular_level", Sdf.ValueTypeNames.Float).Set(0.08)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(),
                                                        "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def scorch_foliage(stage, info, parent_path, rng, browning=0.85, char=0.0,
                   verbose=False):
    """Rebind every surviving leaf material to a dead-foliage version.

    Rebinds on the PROTOTYPE, so one write covers every instance of it.
    """
    from pxr import Sdf

    n = 0
    scope = Sdf.Path(parent_path).AppendChild("FoliageLooks")
    stage.DefinePrim(scope, "Scope")
    for i, (mat_path, tex) in enumerate(
            sorted((info.get("leaf_mats") or {}).items())):
        if not tex:
            print("[veg] no readable leaf texture for {0}".format(mat_path))
            continue
        painted = dead_foliage_texture(tex, browning, rng, char=char,
                                       verbose=verbose)
        if not painted:
            continue
        name = "dead_{0}_{1:02d}".format(
            os.path.splitext(os.path.basename(painted))[0][-8:], i)
        mat = _leaf_pbr(stage, scope.AppendChild(name), painted)
        # Rebind every mesh that used the original. The prototypes are the
        # only consumers, and they are reached from the instancer rather than
        # from the material, so this walks the tree rather than the material.
        n += _rebind(stage, info, mat_path, mat)
    if verbose:
        print("[veg]   foliage: {0} prototype bind(s) browned".format(n))
    return n


def _rebind(stage, info, old_mat_path, new_mat):
    """Bind *new_mat* wherever *old_mat_path* is currently bound in the tree."""
    from pxr import Usd, UsdGeom, UsdShade

    root = stage.GetPrimAtPath(info["path"])
    if not root or not root.IsValid():
        return 0
    n = 0
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        subs = list(UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)))
        for t in ([s.GetPrim() for s in subs] or [prim]):
            b = UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            if not b or not b.GetPrim().IsValid():
                continue
            if str(b.GetPrim().GetPath()) != old_mat_path:
                continue
            UsdShade.MaterialBindingAPI(t).Bind(new_mat)
            n += 1
    return n


# ---------------------------------------------------------------------------
# Charring the bole
# ---------------------------------------------------------------------------

def char_bole(stage, info, parent_path, rng, coverage=0.9, scorch_height=1.0,
              material_path="", wash_weight=0.0, scale_uv=(0.28, 0.28),
              brightness=0.55, verbose=False):
    """Soot the trunk. Returns the number of binds.

    `material_path` BINDS A READY-MADE MATERIAL instead of compositing one.
    NO TREE IN THIS DATASET USES IT — leave it empty and take the composite.

    It existed for `Burnt_Forest_Floor`, and that was judged and REJECTED. The
    megascans surface tiles beautifully at trunk scale (world-triplanar at
    `texture_scale` 0.11, one tile per ~9 m, with real normal and ORM maps a
    composited diffuse-only map cannot carry) — and it still reads wrong,
    because a photographed piece of GROUND wrapped around a trunk reads as
    ground. Losing the species' bark identity was the stated cost; looking
    like the forest floor standing up was the one that decided it.

    `wash_weight` defaults to 0 here and to 0.52 on a wall. A wall's map
    covers it once and the up-wash carries real information; a bark map tiles
    several times up a trunk, so a gradient baked into it comes back as
    repeating bands — "it looks like repeated cylinders".

    SCORCH HEIGHT IS GEOMETRY, NOT TEXTURE. A bark map tiles several times up
    a trunk, so a directional wash composited into it repeats as a stack of
    bands — the gradient has to run once over the real bole or not at all.
    When `scorch_height` < 1 the bole is therefore CUT at that height and the
    two pieces take different coverages: black below, near-clean above. That
    black-bottom / clean-top bole is the signature of a surface fire that
    never got into the crown, and it is exactly what a uniform darkening
    cannot express.

    At `scorch_height` >= 1 nothing is cut — a torched bole really is charred
    end to end, so there is no gradient to draw and no reason to pay for a
    slice.
    """
    from pxr import UsdShade

    from . import damage

    # A ready-made material replaces the whole composite path, including the
    # height split: there is one surface to bind and no gradient to draw.
    ready = None
    if material_path:
        ready = UsdShade.Material(stage.GetPrimAtPath(material_path))
        if not ready:
            print("[veg] bole material {0} did not resolve; compositing "
                  "instead".format(material_path))

    n = 0
    for entry in info.get("bole", []):
        path, mat_path, tex, kind = entry
        if kind != "wood":
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        if ready:
            UsdShade.MaterialBindingAPI(prim).Bind(ready)
            n += 1
            continue
        src = stage.GetPrimAtPath(mat_path) if mat_path else None
        if scorch_height < 0.999:
            n += _char_split(stage, info, path, src, tex, parent_path,
                             coverage, scorch_height, wash_weight, scale_uv,
                             brightness, verbose=verbose)
            continue
        mat = damage.scorched_material(
            stage, parent_path, src, damage.bucket(coverage),
            from_above=False, texture=tex, wash_weight=wash_weight,
            scale_uv=scale_uv, brightness=brightness)
        if mat is not None:
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
            n += 1

    # The unbound branchlets take whatever the trunk took, so a repaired crown
    # matches rather than staying raw bark against a charred trunk.
    if ready:
        for path in info.get("unbound", []):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                UsdShade.MaterialBindingAPI(prim).Bind(ready)
                n += 1
    elif info.get("unbound") and info.get("bark_mat"):
        src = stage.GetPrimAtPath(info["bark_mat"])
        tex = material_texture(src)
        mat = damage.scorched_material(
            stage, parent_path, src, damage.bucket(coverage),
            from_above=False, texture=tex, wash_weight=wash_weight,
            scale_uv=scale_uv)
        for path in info["unbound"]:
            prim = stage.GetPrimAtPath(path)
            if mat is not None and prim and prim.IsValid():
                UsdShade.MaterialBindingAPI(prim).Bind(mat)
                n += 1

    if verbose:
        print("[veg]   bole: {0} bind(s), {1}, coverage {2:.2f}, scorch "
              "height {3:.0%}".format(
                  n, material_path.rsplit("/", 1)[-1] if ready else "soot",
                  coverage, scorch_height))
    return n


def _char_split(stage, info, bole_path, src_mat, tex, parent_path, coverage,
                height_frac, wash_weight=0.0, scale_uv=(0.28, 0.28),
                brightness=0.55, verbose=False):
    """Cut the bole at scorch height and char the two halves differently."""
    from pxr import UsdShade

    from . import damage, fracture

    mesh = fracture.prim_to_mesh(stage, bole_path)
    if mesh is None:
        return 0
    
    lo, hi = mesh.bounds
    z = float(lo[2] + (hi[2] - lo[2]) * float(height_frac))
    try:
        low = fracture.slice_plane(mesh, [0.0, 0.0, -1.0], [0.0, 0.0, z],
                               cap=True)
        high = fracture.slice_plane(mesh, [0.0, 0.0, 1.0], [0.0, 0.0, z],
                                cap=True)
    except Exception as exc:
        print("[veg] bole split failed on {0}: {1}".format(bole_path, exc))
        return 0
    if low is None or high is None or not len(low.faces) or not len(high.faces):
        return 0

    from pxr import UsdGeom, Sdf
    out = "{0}/bole_{1}".format(parent_path, _tag(info, bole_path))
    UsdGeom.Scope.Define(stage, Sdf.Path(out))
    p_low = fracture._write_mesh(stage, out + "/scorched", low)
    p_high = fracture._write_mesh(stage, out + "/clean", high)
    stage.GetPrimAtPath(bole_path).SetActive(False)
    info.setdefault("made", []).extend([p_low, p_high])

    # TRIPLANAR ON BOTH. The cut pieces are rewritten by `_write_mesh`, which
    # authors points and faces only — no UVs — so a UV-space material has
    # nothing to map with and the char simply would not appear.
    # 0.55, NOT 0.22, ON THE UPPER PIECE. A bare standing skeleton with clean
    # bark above the scorch line reads as a healthy tree somebody stripped,
    # not a burnt one — the crown is gone, so the bole is carrying the whole
    # story and it cannot be half untouched.
    for path, cov in ((p_low, coverage), (p_high, coverage * 0.55)):
        mat = damage.scorched_material(
            stage, parent_path, src_mat, damage.bucket(cov),
            from_above=False, triplanar=True, texture=tex,
            wash_weight=wash_weight, scale_uv=scale_uv,
            brightness=brightness if path == p_low else min(
                1.0, brightness * 1.35))
        prim = stage.GetPrimAtPath(path)
        if mat is not None and prim and prim.IsValid():
            UsdShade.MaterialBindingAPI(prim).Bind(mat)
    if verbose:
        print("[veg]   bole split at {0:.1f} m: {1:.2f} below / {2:.2f} above"
              .format(z, coverage, coverage * 0.22))
    return 2


# ---------------------------------------------------------------------------
# Geometry — snapping and toppling
# ---------------------------------------------------------------------------

def _nprng(rng):
    """A numpy Generator from whatever the caller passed.

    `disaster.fracture` and `disaster.scorch` are numpy code — `_seeds` calls
    `rng.uniform(lo, hi, size=...)` and `_noise` calls `rng.normal()`, neither
    of which a `random.Random` has — while `damage`, `settle` and every launch
    script pass a stdlib `random.Random`. The house benches resolve that by
    making the CALLER build both, which means every new caller gets to
    rediscover it as a TypeError several seconds into a container launch.
    Converting at the boundary keeps one rng in the callers' hands.
    """
    import numpy as np

    if isinstance(rng, np.random.Generator):
        return rng
    return np.random.default_rng(rng.randrange(2 ** 31))


def prune_above(stage, info, z_cut, verbose=False):
    """Remove everything still standing above *z_cut* metres. Returns a count.

    A snag whose crown is still hanging in the air over the break is the
    single most obviously wrong thing this pass can produce, and it is easy to
    miss: the BOLE is what gets cut, while the woody twig instancer scattering
    747 branchlets from 2.5 to 16.8 m is a separate prim that knows nothing
    about it. Black_Oak also keeps a small woody branch mesh 12 m up.

    Applies to WOOD as well as leaves, which is the difference between this
    and `defoliate`. Fine twigs above a burn-through are gone by definition —
    they are the first fuel to go, and they are why the top came off.
    """
    from pxr import Gf, Usd, UsdGeom

    xf = UsdGeom.XformCache()
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    n = 0
    for path, _cnt in (info.get("leaf_pi", []) + info.get("wood_pi", [])):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        pi = UsdGeom.PointInstancer(prim)
        pos = pi.GetPositionsAttr().Get() or []
        if not len(pos):
            continue
        m = xf.GetLocalToWorldTransform(prim)
        # `keep_instances`, not `invisibleIds` — this Hydra ignores the latter.
        n += keep_instances(
            stage, path,
            [i for i, p in enumerate(pos)
             if m.Transform(Gf.Vec3d(*p))[2] <= z_cut])

    for path, _mat, _tex, _kind in info.get("bole", []):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        if not r.IsEmpty() and float(r.GetMin()[2]) > z_cut:
            prim.SetActive(False)
            n += 1
    if verbose:
        print("[veg]   prune above {0:.1f} m: {1} piece(s) removed".format(
            z_cut, n))
    return n


def main_bole(stage, info):
    """The one wood mesh that IS the trunk. `(path, mat, tex, kind)` or None.

    `info["bole"]` holds every woody mesh, and on Black_Oak that is two: the
    17 m bole and `Black_Oak_branch2`, a 123-point twig sitting 12 m up. The
    material passes want both — every woody surface chars. The GEOMETRY passes
    emphatically do not: cutting a twig at "1.15 m above its own base" and
    laying it over produces a stray branch lying in mid-air, which is exactly
    the sort of small wrong thing that is hard to see and impossible to
    explain. Tallest woody mesh wins.
    """
    from pxr import Usd, UsdGeom

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    best, best_h = None, -1.0
    for entry in info.get("bole", []):
        path, _mat, _tex, kind = entry
        if kind != "wood":
            continue
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        h = 0.0 if r.IsEmpty() else float(r.GetMax()[2] - r.GetMin()[2])
        if h > best_h:
            best, best_h = entry, h
    return best


def consume_thin(stage, paths, keep_frac=0.16, floor_pts=140, verbose=False):
    """Delete the twiggy fragments and keep the substantial ones.

    THE SKEW IS THE OPPOSITE WAY ROUND FROM A HOUSE, and the reason is worth
    keeping. `fracture_mesh`'s own consumption is LARGE-biased, because on a
    collapsed building a big surviving panel reads as a wall that fell over
    and taking it out is what makes the rest read as debris.

    A tree is the other case entirely. What breaks off a burnt bole and
    actually lies on the ground is TRUNK SECTIONS AND MAJOR LIMBS; the thin
    outer branches do not fall, they are CONSUMED — fine fuel under about a
    centimetre is the first thing a fire takes, and it is most of why the top
    came off in the first place. Leaving a scatter of twigs on the ground and
    calling it debris is therefore wrong twice: the wrong material survived,
    and the pieces that should have survived are missing.

    Size is judged by POINT COUNT, not by bounding box. These fragments are
    Voronoi cells, so a cell holding nothing but a few branch tips still has
    a bbox metres across — the bbox measures the cell, and what matters is how
    much wood is inside it.
    """
    from pxr import UsdGeom

    counts = {}
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        counts[path] = len(pts) if pts else 0
    if not counts:
        return list(paths)

    cut = max(int(floor_pts), int(max(counts.values()) * float(keep_frac)))
    keep, burnt = [], []
    for path, n in counts.items():
        (keep if n >= cut else burnt).append(path)
    for path in burnt:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            prim.SetActive(False)
    # Never consume everything: if one piece dwarfs the rest, the threshold
    # can swallow the whole break and the tree loses its top with nothing to
    # show for it.
    if not keep and burnt:
        best = max(burnt, key=lambda p: counts[p])
        stage.GetPrimAtPath(best).SetActive(True)
        keep, burnt = [best], [p for p in burnt if p != best]
    if verbose:
        print("[veg]   consume_thin: {0} substantial piece(s) kept, {1} "
              "twiggy one(s) burnt away (cut at {2} pts)".format(
                  len(keep), len(burnt), cut))
    return keep


def _column_radius(mesh, z, probe=0.6):
    """Roughly how thick the bole is at height *z*, in metres."""
    from . import fracture

    
    try:
        s = fracture.slice_plane(mesh, [0.0, 0.0, 1.0], [0.0, 0.0, z], cap=True)
        s = fracture.slice_plane(s, [0.0, 0.0, -1.0], [0.0, 0.0, z + probe],
                             cap=True)
    except Exception:
        return None
    s = largest_component(s)
    if s is None or not len(s.faces):
        return None
    e = s.extents
    return 0.5 * float(max(e[0], e[1]))


def clip_to_column(mesh, cx, cy, radius, n_planes=8):
    """Cut the limbs off a bole, leaving the trunk column. Returns a mesh.


    A FALLEN TREE WITH ITS BRANCHES ON DOES NOT LIE DOWN. The limbs hold the
    trunk up off the ground — several metres of it, on a mature oak — and the
    result reads as a log hovering in the air, which is what "the fallen trees
    kinda look like they are floating since branches don't break off" is
    describing. It is not a physics bug; the tree really is resting on its
    branch tips, because nothing broke them.

    Real burnt trees lose their limbs going over: they are thin, already
    weakened by the fire, and they take the whole impact. So the bole is
    clipped to a prism about its own axis before it falls. The limb stubs
    inside the radius survive, which is what keeps it looking like a tree
    rather than a sawn pole.
    """
    from . import fracture

    
    out = mesh
    for i in range(int(n_planes)):
        a = 2.0 * math.pi * i / int(n_planes)
        nx, ny = math.cos(a), math.sin(a)
        try:
            out = fracture.slice_plane(
                out, [-nx, -ny, 0.0],
                [cx + nx * radius, cy + ny * radius, 0.0], cap=True)
        except Exception:
            return mesh
        if out is None or not len(out.faces):
            return mesh
    return out


def snap(stage, info, out_parent, rng, height_frac=0.34, tilt_deg=(9.0, 26.0),
         verbose=False):
    """Break the bole so a spar is left standing. `(statics, loose)`.

    NO VORONOI HERE ANY MORE. Fracturing the bole was the right tool for a
    wall and the wrong one for a tree: a Voronoi cell is a region of SPACE,
    and a tree is mostly air, so a cell routinely held sections of three
    different limbs that never touched each other. Authored as one mesh with
    one rigid body, that is a clump of branches hanging in formation in
    mid-air — and it is unfixable by tuning, because the cells are doing
    exactly what they are defined to do.

    So the break is a PLANE, tilted a few degrees off level so the spar is not
    guillotined flat, and everything above it is DELETED rather than dropped.
    What falls instead is `wood_debris` — constructed lengths of trunk, every
    one a single connected object by construction. The user's framing was the
    right one: break the tree up, delete the broken components, and put
    something we know reads as broken tree in their place.

    Deleting rather than dropping the top is also the physical answer. A snag
    is a tree whose top BURNED THROUGH; the fine material up there is gone,
    not lying at the foot.
    """
    import numpy as np
    
    from pxr import Sdf, UsdGeom

    from . import fracture

    statics = []
    entry = main_bole(stage, info)
    if entry is None:
        return [], []
    path = entry[0]
    mesh = fracture.prim_to_mesh(stage, path)
    if mesh is None:
        return [], []

    lo, hi = mesh.bounds
    z = float(lo[2]) + float(hi[2] - lo[2]) * float(height_frac)
    tilt = math.radians(rng.uniform(*tilt_deg))
    a = rng.uniform(0.0, 2.0 * math.pi)
    # Tilted break plane: level enough to read as a break, off-level enough
    # not to read as a saw cut.
    nrm = [math.sin(tilt) * math.cos(a), math.sin(tilt) * math.sin(a),
           math.cos(tilt)]
    cx = 0.5 * float(lo[0] + hi[0])
    cy = 0.5 * float(lo[1] + hi[1])
    try:
        spar = fracture.slice_plane(mesh, [-n for n in nrm], [cx, cy, z], cap=True)
    except Exception as exc:
        print("[veg] snap slice failed on {0}: {1}".format(path, exc))
        return [], []
    spar = largest_component(spar)
    if spar is None or not len(spar.faces):
        return [], []

    prune_above(stage, info, z, verbose=verbose)
    out = "{0}/snag_{1}".format(out_parent, _tag(info, path))
    UsdGeom.Scope.Define(stage, Sdf.Path(out))
    statics.append(fracture._write_mesh(stage, out + "/spar", spar))
    stage.GetPrimAtPath(path).SetActive(False)

    info.setdefault("made", []).extend(statics)
    if verbose:
        print("[veg]   snap: broken at {0:.1f} m on a {1:.0f} deg plane, top "
              "consumed".format(z, math.degrees(tilt)))
    return statics, []


def topple(stage, info, out_parent, rng, cut_m=1.15, lean_deg=34.0,
           azimuth_deg=None, n_segments=2, verbose=False):
    """Burn the bole through near the base and let it FALL. `(statics, loose)`.

    The stub STAYS PUT and keeps its char. That is the whole point: a tree
    that burned off at the base leaves a charred stump AND a bole lying
    roughly radially from it, and a downed trunk with no stump reads as one
    that was cut and carted in.

    TWO THINGS THAT WERE WRONG IN THE FIRST CUT, and they compound.

    It laid the bole over at 84 degrees — already flat — and handed physics a
    piece with nowhere left to fall. The pose was therefore authored by a
    formula rather than found by the solver, and it read exactly as that:
    "fallen in an unnatural way". Now the lean only tips the bole PAST ITS
    BALANCE POINT and gravity does the rest, which is the same argument
    `damage.damage_placements(move_felled=False)` makes about walls — felling
    by formula and then simulating is the worst of both.

    And it fell as one body with a CONVEX HULL collider. A branching bole is
    wildly concave and its hull is a 20 m blob, so the trunk rested on the
    hull of its own limb tips, floating. The fix for that is
    `settle.run(dynamic_approximation="convexDecomposition")`, NOT cutting the
    bole into many pieces: four segments removed the floating and removed the
    fallen tree with it — a line of separate logs on the ground does not read
    as a tree that went over. `n_segments=2` breaks it once, which is what a
    burnt trunk does going down, and keeps the silhouette.
    """
    import numpy as np
    from trimesh.transformations import rotation_matrix

    from pxr import Sdf, UsdGeom

    from . import fracture

    az = (rng.uniform(0.0, 360.0) if azimuth_deg is None
          else float(azimuth_deg))
    a = math.radians(az)
    # Perpendicular to the fall direction, so tipping about it carries the
    # bole's top toward that azimuth.
    axis = (-math.sin(a), math.cos(a), 0.0)

    statics, loose = [], []
    entry = main_bole(stage, info)
    for path, _mat, _tex, kind in ([entry] if entry else []):
        mesh = fracture.prim_to_mesh(stage, path)
        if mesh is None:
            continue
        lo, hi = mesh.bounds
        z = float(lo[2]) + float(cut_m)
        if z >= float(hi[2]) - 0.2:
            continue
        try:
            stub = fracture.slice_plane(mesh, [0.0, 0.0, -1.0], [0.0, 0.0, z],
                                    cap=True)
        except Exception as exc:
            print("[veg] topple slice failed on {0}: {1}".format(path, exc))
            continue
        stub = largest_component(stub)
        if stub is None:
            continue

        # Everything above the burn-through goes. The crown does NOT come down
        # with the bole here: rotating a PointInstancer's positions is easy but
        # its per-instance ORIENTATIONS are quaternions, and getting those
        # subtly wrong scatters twigs at impossible angles all down the fallen
        # trunk. Fine twigs are also the first fuel to go and are most of why
        # the tree burned through at all, so removing them is the honest
        # reading as well as the cheap one.
        prune_above(stage, info, z, verbose=verbose)

        out = "{0}/fallen_{1}".format(out_parent, _tag(info, path))
        UsdGeom.Scope.Define(stage, Sdf.Path(out))
        statics.append(fracture._write_mesh(stage, out + "/stump", stub))

        cx = 0.5 * float(lo[0] + hi[0])
        cy = 0.5 * float(lo[1] + hi[1])
        # STRIP THE LIMBS BEFORE IT GOES OVER. A bole that keeps its branches
        # lands on the branch tips and holds the trunk metres clear of the
        # ground — "the fallen trees kinda look like they are floating since
        # branches don't break off". That is not a physics failure: the tree
        # really is resting on its limbs, because nothing broke them. Real
        # ones snap on impact, and on a burnt tree they are half gone already.
        r_trunk = _column_radius(mesh, z) or 0.5
        radius = max(0.45, r_trunk * 1.9)
        bare = clip_to_column(mesh, cx, cy, radius)

        c = np.asarray(mesh.centroid, dtype=float)
        pivot = (float(c[0]), float(c[1]), z)
        rot = rotation_matrix(math.radians(lean_deg), axis, pivot)
        edges = np.linspace(z, float(hi[2]), int(n_segments) + 1)
        for k in range(int(n_segments)):
            z0, z1 = float(edges[k]), float(edges[k + 1])
            try:
                seg = fracture.slice_plane(bare, [0.0, 0.0, 1.0],
                                       [0.0, 0.0, z0], cap=True)
                if seg is not None and k < int(n_segments) - 1:
                    seg = fracture.slice_plane(seg, [0.0, 0.0, -1.0],
                                           [0.0, 0.0, z1], cap=True)
            except Exception:
                seg = None
            seg = largest_component(seg)
            if seg is None or not len(seg.faces):
                continue
            seg.apply_transform(rot)
            # The same hairline inset the fracture path uses: segments share
            # their cut planes exactly, and PhysX resolves a zero-gap contact
            # with a separating impulse.
            sc = np.asarray(seg.centroid, dtype=float)
            seg.vertices = sc + (np.asarray(seg.vertices, dtype=float)
                                 - sc) * 0.99
            loose.append(fracture._write_mesh(
                stage, "{0}/bole_{1:02d}".format(out, k), seg))
        stage.GetPrimAtPath(path).SetActive(False)

    info.setdefault("made", []).extend(statics + loose)
    if verbose:
        print("[veg]   topple: azimuth {0:.0f} deg, tipped {1:.0f} deg past "
              "balance, {2} stub(s) + {3} falling segment(s)".format(
                  az, lean_deg, len(statics), len(loose)))
    return statics, loose


def fell_branches(stage, info, out_parent, rng, count=12, prefer_large=True,
                  verbose=False):
    """Bake some woody crown instances into real limbs that FALL. `loose`.

    The crown scatterers are `UsdGeomPointInstancer`s, and an instance is not
    a prim: it has no transform to author, no collider and no rigid body, so
    the branchlets left after a bole is cut just hang in the air exactly where
    they were — "disconnected branches which is good but they haven't fallen".

    There is no way to simulate an instance. What there is, is
    `ComputeInstanceTransformsAtTime`, which gives every instance's matrix —
    so a handful can be BAKED into ordinary meshes at their own transforms,
    handed to the solver as rigid bodies, and their instances hidden. The rest
    of the crown stays instanced and costs nothing.

    `prefer_large` picks off the biggest instances by their authored `scales`.
    A limb that comes down is a limb, not a twig: the fine stuff burns (see
    `consume_thin`), so the pieces that reach the ground are the substantial
    ones.
    """
    import numpy as np

    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

    made = []
    out = "{0}/limbs_{1}".format(out_parent,
                                 str(info.get("path", "t")).rsplit("/", 1)[-1])
    UsdGeom.Scope.Define(stage, Sdf.Path(out))
    xf = UsdGeom.XformCache()
    bark = (UsdShade.Material(stage.GetPrimAtPath(info["bark_mat"]))
            if info.get("bark_mat") else None)

    for path, _n in info.get("wood_pi", []):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        pi = UsdGeom.PointInstancer(prim)
        try:
            xforms = pi.ComputeInstanceTransformsAtTime(
                Usd.TimeCode.Default(), Usd.TimeCode.Default())
        except Exception as exc:
            print("[veg] instance transforms unavailable on {0}: {1}"
                  .format(path, exc))
            continue
        if xforms is None or not len(xforms):
            continue
        scales = pi.GetScalesAttr().Get()

        # Prototype geometry, welded once and reused for every instance.
        protos = []
        for t in pi.GetPrototypesRel().GetTargets():
            pp = stage.GetPrimAtPath(t)
            for q in (Usd.PrimRange(pp) if pp and pp.IsValid() else []):
                if not q.IsA(UsdGeom.Mesh):
                    continue
                m = UsdGeom.Mesh(q)
                pts = m.GetPointsAttr().Get()
                cnt = m.GetFaceVertexCountsAttr().Get()
                idx = m.GetFaceVertexIndicesAttr().Get()
                if pts and cnt and idx:
                    protos.append((np.asarray([[p[0], p[1], p[2]] for p in pts],
                                              dtype=float),
                                   list(cnt), list(idx)))
                break
        if not protos:
            continue

        live = list(range(len(xforms)))
        if not live:
            continue
        if prefer_large and scales is not None and len(scales) == len(xforms):
            live.sort(key=lambda i: -float(np.linalg.norm(
                [scales[i][0], scales[i][1], scales[i][2]])))
            pool = live[:max(int(count) * 3, int(count))]
        else:
            pool = live
        rng.shuffle(pool)
        picks = pool[:int(count)]

        l2w = np.array(xf.GetLocalToWorldTransform(prim), dtype=float)
        pidx = pi.GetProtoIndicesAttr().Get() or []
        for j, i in enumerate(picks):
            v0, cnt, idx = protos[min(int(pidx[i]) if i < len(pidx) else 0,
                                      len(protos) - 1)]
            m4 = np.array(xforms[i], dtype=float) @ l2w
            v = v0 @ m4[:3, :3] + m4[3, :3]
            c = v.mean(axis=0)
            p = "{0}/limb_{1:03d}".format(out, len(made))
            mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(p))
            mesh.CreatePointsAttr(Vt.Vec3fArray(
                [Gf.Vec3f(*map(float, q - c)) for q in v]))
            mesh.CreateFaceVertexCountsAttr(Vt.IntArray([int(x) for x in cnt]))
            mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(
                [int(x) for x in idx]))
            lo, hi = (v - c).min(0), (v - c).max(0)
            mesh.CreateExtentAttr([Gf.Vec3f(*map(float, lo)),
                                   Gf.Vec3f(*map(float, hi))])
            mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
            UsdGeom.Xformable(mesh).AddTranslateOp().Set(Gf.Vec3d(*c))
            if bark:
                UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(bark)
            made.append(p)
        # The instances that became real limbs are REMOVED from the instancer,
        # or the same branch renders twice: once hanging in the crown and once
        # lying on the ground under it.
        taken = set(picks)
        keep_instances(stage, path,
                       [i for i in range(len(xforms)) if i not in taken])

    info.setdefault("made", []).extend(made)
    if verbose:
        print("[veg]   fell_branches: {0} limb(s) baked out of the crown "
              "instancers and handed to physics".format(len(made)))
    return made


def largest_component(mesh, min_frac=0.0):
    """The biggest connected island in *mesh*, or None.

    THE FLOATING PIECES ALL CAME FROM HERE. A Voronoi cell is a region of
    SPACE, not a piece of an object: cut a branching bole with one and the
    cell contains a section of this limb, a section of that one and a stub of
    a third, none of them touching. Authored as a single mesh and given a
    rigid body, that flies and rests as one thing — so you see three
    unconnected branch chunks hanging in mid-air at fixed distances from each
    other, which is exactly what "multiple branches that aren't connected but
    are part of the same fragment" is.

    `trimesh.split` separates the islands. `only_watertight=False` because kit
    meshes are open shells and the watertight filter would reject every one.
    """
    if mesh is None or not len(mesh.faces):
        return None
    try:
        parts = mesh.split(only_watertight=False)
    except Exception:
        return mesh
    if parts is None or len(parts) <= 1:
        return mesh
    parts = sorted(parts, key=lambda m: len(m.faces), reverse=True)
    best = parts[0]
    if min_frac > 0.0 and len(best.faces) < len(mesh.faces) * float(min_frac):
        return None
    return best


# Debris spec per level: (count, piece length range m, scatter radius m).
#
# SIZE HAS TO FOLLOW SEVERITY. A tree that merely lost part of its crown has
# not shed a metre of trunk — what is under it is bark plates, branch ends and
# litter. Metre-long split logs under a lightly damaged tree read as though
# something far worse happened to it than actually did, which is the whole
# problem with using one debris recipe for every level.
_DEBRIS = {
    "scorched": (108, (0.25, 0.70), 7.5),
    "torched":  (198, (0.30, 1.20), 9.0),
    "snag":     (252, (0.35, 2.00), 10.0),
    "fallen":   (252, (0.35, 2.20), 10.5),
    "stump":    (270, (0.30, 1.80), 10.5),
}


def debris_spec(level):
    return _DEBRIS.get(level)


def wood_debris(stage, info, out_parent, rng, n_pieces=9, radius_m=6.5,
                band=(0.02, 0.30), piece_len=(0.30, 1.90),
                thick_m=(0.05, 0.20), ground_z=0.0, simulate_above_m=0.8,
                len_bias=2.2, min_aspect=2.0, heading_deg=None,
                spread_deg=70.0, verbose=False):
    """Broken lengths of the tree's OWN trunk, round its foot. `loose`.

    `piece_len` is the length range, drawn with a bias to the SMALL end set by
    `len_bias` (the exponent on a 0..1 draw): 2.2 puts roughly three quarters
    of the pieces in the bottom third, which is what a burnt stand sheds. A
    WIND event sheds the opposite — whole limbs and long splits rather than
    charcoal — so `disaster.tornado`'s tree path passes ~1.1 and a wider,
    thicker range with it.

    `min_aspect` IS A GATE THAT WILL SILENTLY EAT PLANK-SHAPED PIECES. It
    rejects anything stubbier than `min_aspect:1`, and the 2.0 default is
    there to throw away the chips a clip landing on the trunk's edge produces.
    A board is wide by definition, so a caller asking for wide pieces has to
    lower it as well as raising `thick_m` — otherwise every attempt is
    rejected, the `n_pieces * 6` budget is burnt, and the function returns
    nothing with no error anywhere.

    `heading_deg` BIASES WHERE THE PIECES LAND. None (the default) scatters
    them isotropically about the trunk, which is what falls off a burning
    tree. Given a heading, pieces are thrown within `spread_deg` of it — the
    signature of wind rather than of gravity, and the reason a fallen stand in
    a tornado track reads as having been blown rather than as having rotted.

    `thick_m` IS THE ONE THAT MATTERS, and splitting the trunk lengthwise was
    the wrong way to get it. A wedge of a trunk is as thick as the trunk is
    wide, so the same recipe gave slender sticks off a Douglas Fir (0.3 m
    bole) and barrels off a Black Oak (over a metre) — the pieces that looked
    right all came from the SMALL-trunked species. Thickness has to be set
    directly rather than inherited from whatever it was cut out of.

    So a piece is cut as a thin off-axis COLUMN: pick a point somewhere in the
    trunk's cross-section, clip to a few centimetres either side of it, and
    take the full length of the slab through it. What comes out is a stick
    with the trunk's own curvature and bark, at a thickness we chose, from any
    species.

    KNOWN-GOOD GEOMETRY, NOT FRACTURE OUTPUT. Voronoi-cutting the bole gives
    cells full of disconnected branch sections (see `largest_component`), and
    no amount of tuning fixes that — a cell is a region of space and a tree is
    mostly air. So the debris is CONSTRUCTED instead: take a short length of
    the LOW trunk, where the bole is a single connected column about a metre
    through, keep only its largest connected island, and split it lengthwise
    into long wedges. Every piece is then guaranteed to be one solid object,
    and a split length of charred trunk is unmistakably a broken tree.

    Read the bole BEFORE any geometry pass runs: `snap` and `topple` both
    deactivate it, and `prim_to_mesh` on a deactivated prim returns nothing.

    RETURNS `(loose, static)`. Anything under `simulate_above_m` is SEATED
    rather than simulated — laid flat with its lowest point on the ground and
    handed back as static geometry. A 0.2 m stick lying flat looks the same
    whether PhysX put it there or arithmetic did, and it is the count that
    hurts: 15,000 of them is 15,000 colliders to cook and 15,000 bodies to
    carry to rest, which is most of a build. Pieces big enough for the landing
    to read — a metre of trunk, which can come to rest against a stump or
    across another log — still fall.

    `n_pieces` COUNTS BOTH, and it used to count only the simulated ones. The
    loop condition was `len(made) < n_pieces`, where `made` is the SIMULATED
    list, so a seated stick — which is most of what this makes, and all of
    what it makes at the light levels — never counted towards the target. At
    `scorched` the longest piece the recipe can cut is 0.70 m, under the 0.8 m
    `simulate_above_m` threshold, so `made` stayed empty by construction and
    the loop always ran to its `n_pieces * 6` attempt cap. The result was 224
    to 570 debris meshes under a tree asked for about forty, at six times the
    slicing cost — and it read as a woodpile rather than as what fell off a
    burnt tree. Counting every accepted piece makes `n_pieces` mean what its
    name says and makes the attempt cap the fallback it was meant to be.
    """
    import numpy as np
    from trimesh.transformations import rotation_matrix

    from pxr import Sdf, UsdGeom

    from . import fracture

    entry = main_bole(stage, info)
    if entry is None:
        return [], []
    mesh = fracture.prim_to_mesh(stage, entry[0])
    if mesh is None:
        return [], []
    lo, hi = mesh.bounds
    h = float(hi[2] - lo[2])
    cx = 0.5 * float(lo[0] + hi[0])
    cy = 0.5 * float(lo[1] + hi[1])

    # CUT THE STOCK ONCE. Every piece used to be sliced out of the whole
    # 41,000-point bole, three planes at a time — fine at ten pieces and
    # minutes of start-up at thirty. The band is a small fraction of the bole
    # and the limbs outside the trunk column are not wanted anyway, so isolate
    # it once and cut the pieces from THAT. Same geometry, a fraction of the
    # work, and it also guarantees the stock is one connected column.
    r_trunk = _column_radius(mesh, float(lo[2]) + h * float(band[0])) or 0.6
    z_lo = float(lo[2]) + h * float(band[0])
    z_hi = float(lo[2]) + h * float(band[1])
    try:
        stock = fracture.slice_plane(mesh, [0.0, 0.0, 1.0], [0.0, 0.0, z_lo],
                                 cap=True)
        stock = fracture.slice_plane(stock, [0.0, 0.0, -1.0], [0.0, 0.0, z_hi],
                                 cap=True)
    except Exception as exc:
        print("[veg] debris stock failed: {0}".format(exc))
        return [], []
    stock = largest_component(clip_to_column(stock, cx, cy, r_trunk * 1.6))
    if stock is None or not len(stock.faces):
        return [], []
    s_lo, s_hi = stock.bounds

    out = "{0}/debris_{1}".format(out_parent,
                                  str(info.get("path", "t")).rsplit("/", 1)[-1])
    UsdGeom.Scope.Define(stage, Sdf.Path(out))
    made, seated = [], []
    tries = 0
    p_lo, p_hi = float(piece_len[0]), float(piece_len[1])
    n_want, n_try = int(n_pieces), int(n_pieces) * 6
    while len(made) + len(seated) < n_want and tries < n_try:
        tries += 1
        # Biased to the small end: `** 2.2` puts roughly three quarters of the
        # draws in the bottom third of the range.
        seg_h = p_lo + (p_hi - p_lo) * (rng.random() ** float(len_bias))
        z0 = rng.uniform(float(s_lo[2]),
                         max(float(s_lo[2]), float(s_hi[2]) - seg_h))
        try:
            piece = fracture.slice_plane(stock, [0.0, 0.0, 1.0], [0.0, 0.0, z0],
                                     cap=True)
            piece = fracture.slice_plane(piece, [0.0, 0.0, -1.0],
                                     [0.0, 0.0, z0 + seg_h], cap=True)
        except Exception:
            continue
        piece = largest_component(piece)
        if piece is None or not len(piece.faces):
            continue

        # THIN IT TO A CHOSEN THICKNESS, not to a fraction of the trunk. Four
        # vertical planes around a point somewhere in the cross-section: the
        # result is a stick that keeps the bark and the trunk's curvature but
        # whose thickness we set, so an oak and a fir shed the same debris.
        t_w = rng.uniform(*thick_m)
        t_d = rng.uniform(*thick_m)
        pa = rng.uniform(0.0, 2.0 * math.pi)
        # sqrt keeps the sample area-uniform, so pieces come off the whole
        # cross-section rather than clustering on the pith.
        pr = max(0.0, r_trunk - 0.5 * max(t_w, t_d)) * math.sqrt(rng.random())
        px = cx + math.cos(pa) * pr
        py = cy + math.sin(pa) * pr
        ok = True
        for nrm, orig in (([1.0, 0.0, 0.0], px - t_w * 0.5),
                          ([-1.0, 0.0, 0.0], px + t_w * 0.5),
                          ([0.0, 1.0, 0.0], py - t_d * 0.5),
                          ([0.0, -1.0, 0.0], py + t_d * 0.5)):
            o = [orig if nrm[0] else px, orig if nrm[1] else py, 0.0]
            try:
                piece = fracture.slice_plane(piece, nrm, o, cap=True)
            except Exception:
                ok = False
                break
            if piece is None or not len(piece.faces):
                ok = False
                break
        if not ok:
            continue
        piece = largest_component(piece)
        if piece is None or len(piece.faces) < 8:
            continue
        # A stick, not a chip: reject anything that came back stubbier than
        # about 2:1, which is what a clip landing on the very edge produces.
        e = sorted(float(v) for v in piece.extents)
        if e[2] < 0.10 or e[2] < e[1] * float(min_aspect):
            continue

        c = np.asarray(piece.centroid, dtype=float)
        big = float(max(piece.extents)) > float(simulate_above_m)
        # Lie it down. A simulated piece keeps a full random roll; a seated one
        # is tipped only a few degrees off flat, because nothing is going to
        # come along and knock it over.
        piece.apply_transform(rotation_matrix(
            rng.uniform(1.15, 1.95) if big else (0.5 * math.pi
                                                 + rng.uniform(-0.12, 0.12)),
            (rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0), c))
        if heading_deg is None:
            ang = rng.uniform(0.0, 2.0 * math.pi)
        else:
            # A LOBE, NOT A RAY. Debris from a tree that went over in a wind
            # lies downwind of it in a fan; a hard heading with no spread
            # puts every piece on one line, which reads as a fence.
            hs = math.radians(float(spread_deg))
            ang = math.radians(float(heading_deg)) + rng.uniform(-hs, hs)
        # Small pieces land close in and big ones get thrown further, which is
        # what a pile of debris actually grades like.
        bias = 0.35 + 0.65 * min(1.0, seg_h / max(1e-6, p_hi))
        r = float(radius_m) * bias * math.sqrt(rng.random())
        px_, py_ = cx + math.cos(ang) * r, cy + math.sin(ang) * r
        if big:
            # Start it clear of the ground so the solver beds it rather than
            # resolving an initial interpenetration.
            tgt = np.array([px_, py_, float(lo[2]) + rng.uniform(0.4, 1.2)])
            piece.apply_translation(tgt
                                    - np.asarray(piece.centroid, dtype=float))
            made.append(fracture._write_mesh(
                stage, "{0}/log_{1:03d}".format(out, len(made) + len(seated)),
                piece))
        else:
            # SEATED: move it so its own lowest point rests on the ground, and
            # bury a millimetre so it does not read as hovering.
            pc_ = np.asarray(piece.centroid, dtype=float)
            piece.apply_translation(
                np.array([px_ - pc_[0], py_ - pc_[1],
                          float(ground_z) - float(piece.bounds[0][2])
                          - 0.001]))
            seated.append(fracture._write_mesh(
                stage, "{0}/log_{1:03d}".format(out, len(made) + len(seated)),
                piece))

    info.setdefault("made", []).extend(made + seated)
    if verbose:
        print("[veg]   wood_debris: {0} simulated + {1} seated in {2} "
              "attempt(s)".format(len(made), len(seated), tries))
    return made, seated


def wood_material(stage, info, parent_path, coverage=0.94, material_path="",
                  scale_uv=(0.28, 0.28), brightness=1.0):
    """The charred-wood material every generated piece should carry.

    `fracture._write_mesh` authors points and faces and NOTHING ELSE — no
    material, no UVs. So every splinter, every fallen limb, every snapped
    segment came out bound to nothing and rendered as untextured grey next to
    a charred trunk. The house bench binds its fragments explicitly straight
    after fracturing for exactly this reason; this is that step, in one place,
    for everything the vegetation pass makes.

    Triplanar, because those meshes have no UVs to map with.
    """
    from pxr import UsdShade

    from . import damage

    if material_path:
        m = UsdShade.Material(stage.GetPrimAtPath(material_path))
        if m:
            return m
    src = (stage.GetPrimAtPath(info["bark_mat"])
           if info.get("bark_mat") else None)
    return damage.scorched_material(
        stage, parent_path, src, damage.bucket(coverage), from_above=False,
        triplanar=True, texture=material_texture(src), wash_weight=0.0,
        scale_uv=scale_uv, brightness=brightness)


def bind_all(stage, paths, mat, verbose=False):
    """Bind *mat* to every prim in *paths* that is still alive."""
    from pxr import Usd, UsdGeom, UsdShade

    if mat is None:
        return 0
    n = 0
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.IsActive():
            continue
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(p).Bind(mat)
                n += 1
    if verbose:
        print("[veg]   bind_all: {0} generated piece(s) materialised".format(n))
    return n


def scar_patch(stage, path, x_m, y_m, radius_m, rng, mat_prim_path="",
               z_m=0.02, n=64, jitter=0.30, ssf=1.0):
    """An irregular ground patch — the scar's outline, as GEOMETRY.

    WHY GEOMETRY AND NOT A MASKED OVERLAY. A translucent overlay is the
    obvious way to paint a soft-edged scar onto grass, and it does not survive
    this renderer: Hydra here translates USD materials to MDL (`UsdToMdl`),
    and a `UsdPreviewSurface` whose `opacity` is driven by a texture comes out
    the other side without it. The plane then draws fully opaque over the
    whole plate — "you've made the whole ground ash color" — while the numbers
    said the mask was 88% transparent. Measuring the mask and the texture is
    what separated "the material is wrong" from "the material is not being
    applied at all".

    So the shape is a POLYGON instead. It costs nothing, it uses only the
    plain-mesh-plus-OmniPBR path every other material here already proves
    works, and a fire scar's edge is genuinely fairly sharp — a metre or two,
    not a long fade — so a hard irregular boundary is closer to the truth than
    a soft one anyway.

    THREE FREQUENCIES OF WOBBLE, because one gives a wavy circle and a circle
    is what the eye picks out. Low harmonics pull the outline into lobes, mid
    ones finger it in and out, and per-vertex noise roughens the edge.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    ph = [rng.uniform(0.0, 6.283) for _ in range(3)]
    fr = [rng.uniform(1.6, 2.6), rng.uniform(3.4, 5.2), rng.uniform(7.0, 11.0)]
    amp = [0.62, 0.28, 0.12]

    pts, uvs = [], []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        w = sum(amp[k] * math.sin(a * fr[k] + ph[k]) for k in range(3))
        r = radius_m * (1.0 + jitter * w) * rng.uniform(0.95, 1.05)
        r = max(0.25 * radius_m, r)
        px, py = x_m + math.cos(a) * r, y_m + math.sin(a) * r
        pts.append(Gf.Vec3f(px * ssf, py * ssf, z_m * ssf))
        uvs.append(Gf.Vec2f(float(px) / 4.0, float(py) / 4.0))

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([n]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(list(range(n))))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * n))
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.13, 0.12, 0.10)])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), z_m * ssf),
                           Gf.Vec3f(max(xs), max(ys), z_m * ssf)])
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying).Set(Vt.Vec2fArray(uvs))
    if mat_prim_path:
        m = UsdShade.Material(stage.GetPrimAtPath(mat_prim_path))
        if m:
            UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(m)
    return path


def ash_pile(stage, path, x_m, y_m, radius_m, height_m, rng,
             mat_prim_path="", n=16, ssf=1.0):
    """A low mound of ash. Returns the prim path.

    What is actually left where a heavy branch or a snag burned out: a soft
    grey-black cone, wider than it is tall, with no hard edge. A triangle fan
    with a jittered rim, because a smooth cone reads as a tent.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    pts = [Gf.Vec3f(x_m * ssf, y_m * ssf, height_m * ssf)]
    uvs = [Gf.Vec2f(0.5, 0.5)]
    for i in range(n):
        a = 2.0 * math.pi * i / n
        r = radius_m * rng.uniform(0.72, 1.22)
        px, py = x_m + math.cos(a) * r, y_m + math.sin(a) * r
        pts.append(Gf.Vec3f(px * ssf, py * ssf, 0.01 * ssf))
        uvs.append(Gf.Vec2f(float(px) / 3.0, float(py) / 3.0))

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * n))
    idx = []
    for i in range(n):
        idx += [0, 1 + i, 1 + (i + 1) % n]
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.13, 0.12, 0.11)])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), 0.0),
                           Gf.Vec3f(max(xs), max(ys), height_m * ssf)])
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying).Set(Vt.Vec2fArray(uvs))
    if mat_prim_path:
        m = UsdShade.Material(stage.GetPrimAtPath(mat_prim_path))
        if m:
            UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(m)
    return path


def consume(stage, info, verbose=False):
    """Remove the tree entirely — for the `stump` level, where a prop replaces it."""
    n = 0
    for path, _mat, _tex, _kind in info.get("bole", []):
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid() and prim.SetActive(False):
            n += 1
    if verbose:
        print("[veg]   consume: {0} bole mesh(es) removed".format(n))
    return n


# ---------------------------------------------------------------------------
# The burnt ring at the foot of a tree
# ---------------------------------------------------------------------------

def ash_ring(stage, path, x_m, y_m, radius_m, rng, mat_prim_path="",
             z_m=0.03, n=26, jitter=0.26, ssf=1.0):
    """An irregular burnt patch around a trunk base.

    NOT A CIRCLE. A fire scar's outline is irregular and fingered — the
    duff around a trunk burns out over hours and follows the fuel, not a
    compass — and a clean disc reads as a decal dropped on the grass. The
    radius is therefore walked with a smooth wobble plus noise.
    """
    from pxr import Gf, Sdf, UsdGeom, Vt

    phase = rng.uniform(0.0, 6.283)
    freq = rng.uniform(1.6, 3.4)
    pts, uvs = [], []
    for i in range(n):
        a = 2.0 * math.pi * i / n
        w = (math.sin(a * freq + phase) * 0.6
             + math.sin(a * freq * 2.3 + phase * 1.7) * 0.4)
        r = radius_m * (1.0 + jitter * w) * rng.uniform(0.94, 1.06)
        px, py = x_m + math.cos(a) * r, y_m + math.sin(a) * r
        pts.append(Gf.Vec3f(px * ssf, py * ssf, z_m * ssf))
        uvs.append(Gf.Vec2f(float(px) / 6.0, float(py) / 6.0))

    mesh = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([n]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(list(range(n))))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * n))
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.09, 0.08)])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    mesh.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), z_m * ssf),
                           Gf.Vec3f(max(xs), max(ys), z_m * ssf)])
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying).Set(Vt.Vec2fArray(uvs))
    if mat_prim_path:
        from pxr import UsdShade
        m = UsdShade.Material(stage.GetPrimAtPath(mat_prim_path))
        if m:
            UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(m)
    return path


# ---------------------------------------------------------------------------
# One tree, one level
# ---------------------------------------------------------------------------

def burn_tree(stage, tree_path, level, parent_path, out_parent, rng,
              bole_material="", bole_scale_uv=(0.28, 0.28), debris=True,
              debris_scale=1.0, limbs=True, ground_z=0.0,
              seated_collide=True, verbose=False):
    """Take one placed tree to *level*. Returns a dict of what it produced.

    `{"statics": [...], "loose": [...], "seated": [...], "info": <survey>}` —
    the caller hands `loose` to `disaster.settle` and `statics` to it as
    collision geometry, exactly as the house benches do. `seated` is the small
    ground debris: geometry that is already at rest and is never simulated.

    `bole_material` binds a ready-made material to the wood instead of
    compositing soot onto the species' own bark — see `char_bole`.

    `seated_collide=False` KEEPS THE STICKS AND DROPS THEIR COLLIDERS. Every
    seated piece used to be appended to `statics`, which means `settle` cooks
    a triangle-mesh collider for each one — hundreds per tree, 36,734 static
    colliders on the last full block — purely so that a handful of loose logs
    could land on them. Nothing else in the scene ever touches them: they are
    laid flat on the ground by arithmetic and the solver never moves them. A
    log that lands on the bare ground instead of on top of a 5-20 cm stick
    ends up within a stick's thickness of where it would otherwise have been,
    which is not a difference anyone can see, and the cooking cost is real.
    The pieces themselves are unaffected — they are still authored, still
    bound, still exported — so this changes the collision set only, and it is
    the right default for the archetype bake where the whole point is to pay
    the physics cost once and keep it small.
    """
    info = survey(stage, tree_path, verbose=verbose)
    res = {"statics": [], "loose": [], "seated": [], "info": info,
           "level": level}
    if not info.get("bole") and not info.get("leaf_pi"):
        return res

    keep, keep_top, cov, brown, scorch_h, geom = plan_for(level)

    # REPAIR FIRST. Black_Oak's woody branchlet prototype binds no material at
    # all; leave it and the char pass has nothing to re-bind, so a torched
    # crown of 747 branchlets renders untextured against a charred trunk.
    bind_bark(stage, info, verbose=verbose)

    # DEBRIS BEFORE GEOMETRY, because both `snap` and `topple` deactivate the
    # bole and the splinters are cut FROM it. Size and count come from the
    # LEVEL — see `_DEBRIS`; one recipe for every severity is what put
    # metre-long split logs under lightly damaged trees.
    # `debris_scale` because the counts in `_DEBRIS` are tuned for a
    # close-up bench where one tree fills the frame. A 250 m block carries
    # dozens of trees and is read from the air; at full count that is tens of
    # thousands of rigid bodies for detail no camera up there can resolve.
    spec = debris_spec(level) if debris else None
    if spec:
        n_d, plen, rad = spec
        n_d = max(0, int(round(n_d * float(debris_scale))))
    if spec and n_d > 0:
        _loose, _seated = wood_debris(stage, info, out_parent, rng,
                                      n_pieces=n_d, piece_len=plen,
                                      radius_m=rad, ground_z=ground_z,
                                      verbose=verbose)
        res["loose"] += _loose
        # Seated debris is static geometry, not a body: it never settles. It
        # only reaches the solver at all as something to land ON, and under
        # `seated_collide=False` it does not even do that — see the docstring.
        res["seated"] += _seated
        if seated_collide:
            res["statics"] += _seated

    # ORDER MATTERS. Geometry LAST: `snap` and `topple` deactivate the bole,
    # and a material bound to a prim that no longer exists is not an error,
    # it is silence — the piece just renders unburnt. Foliage before the bole
    # for the same reason, since `_char_split` also deactivates.
    #
    # EVERY BURNT LEVEL LOSES THE CROWN, `scorched` INCLUDED.
    #
    # `scorched` used to keep its foliage and brown it, because crown scorch —
    # heat-kill without combustion, leaves turning orange-brown and staying on
    # — is real and is the most common outcome by area. That argument is
    # sound and it lost on sight: a burnt block with sixty full crowns in it
    # reads as trees that were missed, whatever colour they are. The scene is
    # the judge.
    #
    # What still separates `scorched` from `torched` is the BOLE: charred to
    # a third of its height rather than end to end, plus a third of the
    # ground debris. That is a real difference and it survives.
    if level in ("scorched", "torched", "snag", "fallen", "stump"):
        strip_to_trunk(stage, info, verbose=verbose)
        # AND RE-SEAT. The placed height was measured off the whole tree,
        # including crown geometry that has just been deleted — see `reseat`.
        # Must happen before `char_bole` or the geometry passes, because those
        # write meshes OUTSIDE the tree prim and moving its transform will not
        # move them.
        reseat(stage, info, ground_z=ground_z, verbose=verbose)
    elif keep < 1.0 or keep_top < 1.0:
        defoliate(stage, info, rng, keep=keep, keep_top=keep_top,
                  verbose=verbose)
    # Nothing survives at `snag` and below, so browning them would composite a
    # texture per leaf material for geometry that is entirely hidden.
    if brown > 0.0 and info.get("leaf_mats") and max(keep, keep_top) > 0.0:
        # A crown that BURNED is charred as well as dead; one that was only
        # scorched by the plume never saw flame at all.
        char = 0.0 if level == "scorched" else 0.55
        scorch_foliage(stage, info, parent_path, rng, browning=brown,
                       char=char, verbose=verbose)
    if cov > 0.0 and geom != "stump":
        char_bole(stage, info, parent_path, rng, coverage=cov,
                  scorch_height=scorch_h, material_path=bole_material,
                  scale_uv=bole_scale_uv, verbose=verbose)

    # `fell_branches` is NOT called any more, and the reason is worth keeping:
    # baking crown instances into falling limbs was solving the wrong problem.
    # Those branchlets are fine fuel — they burn, they do not fall — so
    # `strip_to_trunk` deletes them and `wood_debris` supplies the timber that
    # really does end up on the ground. The function stays for a wind or
    # storm-damage pass, where limbs coming down IS the effect.
    del limbs

    if geom == "snap":
        s, l = snap(stage, info, out_parent, rng, verbose=verbose)
        res["statics"] += s
        res["loose"] += l
    elif geom == "topple":
        s, l = topple(stage, info, out_parent, rng, verbose=verbose)
        res["statics"] += s
        res["loose"] += l
    elif geom == "stump":
        consume(stage, info, verbose=verbose)

    # EVERYTHING GENERATED, IN ONE PLACE. `_write_mesh` binds nothing, so a
    # per-pass bind is a per-pass chance to forget — and the symptom (grey
    # untextured debris) looks like a lighting problem, not a missing bind.
    #
    # `seated` is listed EXPLICITLY rather than being reached through
    # `statics`: with `seated_collide=False` it is not in `statics` any more,
    # and binding by collision role would have quietly unbound every stick the
    # moment its collider went away. Deduped because with `seated_collide=True`
    # the same paths are in both lists.
    made, _seen = [], set()
    for _p in res["statics"] + res["loose"] + res["seated"]:
        if _p not in _seen:
            _seen.add(_p)
            made.append(_p)
    if made:
        bind_all(stage, made,
                 wood_material(stage, info, parent_path, coverage=max(cov, 0.9),
                               material_path=bole_material,
                               scale_uv=bole_scale_uv, brightness=0.55),
                 verbose=verbose)

    return res


# ===========================================================================
# WIND DAMAGE — the tornado path
#
# A separate entry point from `burn_tree`, and the reason is one branch. That
# function's stage 4a is gated on the LEVEL NAME rather than on an argument:
#
#     if level in ("scorched", "torched", "snag", "fallen", "stump"):
#         strip_to_trunk(...)
#
# and `strip_to_trunk` deactivates every PointInstancer in the tree plus every
# mesh named branch/leaf/needle/twig/foliage. There is no parameter that
# reaches it. Since "the leaves stay on" is the entire difference between a
# windthrown tree and a burnt one, a wind path cannot go through `burn_tree`
# without editing the branch that makes the burnt path correct.
#
# THE DOMAIN FACT THIS ENCODES, and it is the mirror of the one the wildfire
# skill records: a burnt stand STANDS. Immediately behind a fire front you are
# looking at a field of black poles, and a scene full of downed trunks reads
# as windthrow rather than as fire. Run that backwards and it is the rule
# here — a tornado track is downed trunks, root plates in the air, and green
# crowns lying on the ground, because the foliage is still on the tree when
# the photograph is taken. `fall_chance` is 0.12 over there and the majority
# case over here.
# ===========================================================================

WIND_LEVELS = ("pristine", "limbed", "leaning", "fallen", "snapped")

# level -> (defoliate_keep, defoliate_keep_top, limbs_down, geometry)
#
# `limbs_down` is a count of crown instances baked into real falling limbs by
# `fell_branches` — the function the wildfire path deliberately never calls,
# whose docstring reserves it for "a wind or storm-damage pass". This is that
# pass.
_WIND_PLAN = {
    "pristine": (1.00, 1.00, 0,  None),
    # EF0-EF1. Standing and whole; the crown is thinned and a few limbs are
    # on the ground under it. Leaves are still GREEN — the browning pass is
    # a fire effect and is not called anywhere on this path.
    "limbed":   (0.80, 0.60, 10, None),
    # Root-sprung. Tipped but alive and entirely intact, which is a real and
    # very common outcome and the cheapest one to author.
    "leaning":  (1.00, 1.00, 4,  "lean"),
    # Uprooted. The whole tree on the ground with its crown, root plate
    # levered up at the base.
    "fallen":   (0.94, 0.88, 8,  "uproot"),
    # Bole broken partway up. THE ONE LEVEL THAT LOSES ITS CROWN, and that is
    # correct rather than a compromise: a snapped bole's crown is somewhere
    # else entirely. It is also the rarest, because it takes the most wind.
    "snapped":  (1.00, 1.00, 0,  "snap"),
}

# level -> (n_pieces, piece_len_m, scatter_radius_m)
#
# LARGER THAN THE BURNT RECIPE ON EVERY AXIS, and that is the request:
# `_DEBRIS` runs 0.25-2.2 m because a fire reduces wood to charcoal and
# splinters. Wind BREAKS it, so what is on the ground is limb-sized. Fewer
# pieces, each of them bigger, thrown further.
_WIND_DEBRIS = {
    "limbed":  (26,  (0.55, 2.40), 9.0),
    "leaning": (16,  (0.50, 2.00), 8.0),
    "fallen":  (54,  (0.70, 3.60), 14.0),
    "snapped": (78,  (0.80, 4.20), 17.0),
}

# Thicker than the burnt stock too. A burnt stick is 5-20 cm through; a limb
# torn off a live tree is 10-30 cm, and the aspect gate has to come down with
# it or every attempt is rejected — see `wood_debris`.
_WIND_THICK = (0.10, 0.30)


def wind_plan(level):
    """`(keep_base, keep_top, limbs_down, geometry)` for a wind level."""
    if level not in _WIND_PLAN:
        raise ValueError("unknown wind level {0!r}; expected one of {1}"
                         .format(level, ", ".join(WIND_LEVELS)))
    return _WIND_PLAN[level]


def plain_wood(stage, parent_path, texture="", scale_uv=(0.30, 0.30),
               brightness=1.0):
    """A NOT-CHARRED wood material for wind debris. Returns the Material.

    `wood_material` cannot produce this and it is worth saying why, because
    the obvious fix does not work: it routes through
    `damage.scorched_material`, whose `level` indexes `damage.SOOT_LEVELS`,
    and the LOWEST entry in that table is 0.52 coverage. There is no bucket
    below "half sooted" — the composite path is a burn compositor and cannot
    be asked for clean timber. So this builds the material directly.

    With no `texture`, falls back to `disaster.planks`' own sawn-lumber
    surface, which is the same material the plank debris field carries: a
    snapped limb and a broken stud lying beside it then agree.
    """
    from . import damage, planks

    if texture:
        return damage._pbr(stage, "{0}/WindLooks/wood_{1}".format(
            parent_path, abs(hash(texture)) % 10000),
            (brightness, brightness * 0.97, brightness * 0.93), 0.80,
            texture=texture, scale_uv=scale_uv)
    return planks.wood_material(stage, parent_path + "/WindLooks/timber",
                                tile_m=1.1)


def tip_tree(stage, tree_path, lean_deg, azimuth_deg=0.0, lift_m=0.0):
    """Rotate a WHOLE placed tree about its own base. Returns the lean applied.

    THIS IS THE CHEAPEST CORRECT THING IN THE WHOLE PIPELINE, and it only
    works because of what a tornado does and a fire does not. A burnt tree has
    to be cut: its crown is gone, its bole is charred to a different degree
    over its height, and the piece that falls is a section rather than the
    tree. A windthrown tree is the SAME TREE, rotated — crown, instancers,
    materials and all — so there is no mesh surgery, no fracture, no physics
    and no per-instance quaternion arithmetic. `topple` exists because the
    burnt case cannot do this; the wind case can.

    Rotating the PARENT is also what carries the crown down with the trunk,
    which `topple` explicitly gives up on: its docstring records that rotating
    a PointInstancer's own per-instance `orientations` "scatters twigs at
    impossible angles". Nothing here touches them — they are children of the
    prim being rotated and they come along.

    THE PIVOT IS THE LOCAL ORIGIN, which on every tree in this library is the
    base of the trunk, so the tree hinges where a real one hinges. The op
    order is rebuilt as translate -> orient -> scale: rotation must come after
    scale in application order (i.e. before it in the list) or the tree is
    scaled along the tilted axes and comes out sheared.

    `lift_m` raises the base, which is what a root plate levering out of the
    ground actually does — and it is also what keeps the far side of the crown
    from sinking metres below grade, since rotating about the base drives
    anything that was on the downwind side of the trunk under the lawn.
    """
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(tree_path)
    if not prim or not prim.IsValid():
        return 0.0
    xf = UsdGeom.Xformable(prim)

    # Read what is there by NAME rather than by position: the tree may have
    # been placed by `apply_placements` (translate + rotateZ + scale) or by a
    # bake grid (translate + scale), and assuming an order is how a scale gets
    # silently dropped.
    vals = {}
    for op in xf.GetOrderedXformOps():
        vals[op.GetOpName().split(":")[-1]] = op.Get()
    xf.SetXformOpOrder([])

    t = vals.get("translate") or Gf.Vec3d(0.0, 0.0, 0.0)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(t[0]), float(t[1]),
                                     float(t[2]) + float(lift_m)))
    if "rotateZ" in vals and vals["rotateZ"] is not None:
        xf.AddRotateZOp().Set(float(vals["rotateZ"]))
    a = math.radians(float(azimuth_deg))
    # Perpendicular to the fall direction, so tipping about it carries the
    # crown toward that azimuth — the same construction `topple` uses.
    axis = Gf.Vec3d(-math.sin(a), math.cos(a), 0.0)
    # BUILT FROM COMPONENTS. `Gf.Rotation(...).GetQuat()` is a Quatd and
    # `AddOrientOp()` defaults to FLOAT precision, so handing the Quatd
    # straight over relies on a cross-precision constructor. Reading the real
    # and imaginary parts out and rebuilding a Quatf cannot depend on that.
    _q = Gf.Rotation(axis, float(lean_deg)).GetQuat()
    _im = _q.GetImaginary()
    xf.AddOrientOp().Set(Gf.Quatf(float(_q.GetReal()),
                                  float(_im[0]), float(_im[1]),
                                  float(_im[2])))
    sc = vals.get("scale")
    if sc is not None:
        xf.AddScaleOp().Set(Gf.Vec3f(float(sc[0]), float(sc[1]), float(sc[2])))
    return float(lean_deg)


def root_plate(stage, path, x_m, y_m, radius_m, azimuth_deg, rng,
               mat_prim_path="", tilt_deg=68.0, ssf=1.0, n=14):
    """The disc of soil and roots a windthrown tree levers out of the ground.

    THE SIGNATURE FEATURE, and it is what tells a fallen tree from a felled
    one at any distance: a tree that was cut leaves a stump, a tree that blew
    over leaves a two-to-four-metre wheel of earth standing on edge with the
    trunk running away from it. In `tornado.jpeg` these are the dark spots at
    the head of each downed trunk.

    Nearly vertical rather than flat, tilted back against the fall. An
    irregular rim for the same reason `scar_patch` wobbles its outline at
    three frequencies — a clean circle is exactly the shape the eye picks out
    as authored.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    a = math.radians(float(azimuth_deg))
    ux, uy = math.cos(a), math.sin(a)
    # Plate normal is the fall direction tipped back from vertical.
    t = math.radians(float(tilt_deg))
    # In-plane axes: `e1` across the fall (horizontal), `e2` up-and-back.
    e1 = (-uy, ux, 0.0)
    e2 = (-ux * math.cos(t), -uy * math.cos(t), math.sin(t))

    # A TRIANGLE FAN ABOUT A REAL CENTRE VERTEX, not one n-gon. A single
    # concave-ish 14-gon is legal USD and every renderer triangulates it
    # differently; a fan is unambiguous and costs one extra point.
    cz = float(radius_m) * 0.45
    pts = [Gf.Vec3f(x_m * ssf, y_m * ssf, cz * ssf)]
    for i in range(int(n)):
        th = 2.0 * math.pi * i / float(n)
        r = float(radius_m) * (1.0 + 0.20 * math.sin(3.0 * th + 0.7)
                               + 0.11 * math.sin(7.0 * th + 2.1)
                               + rng.uniform(-0.07, 0.07))
        c, s2 = math.cos(th) * r, math.sin(th) * r
        # Sit the disc's own centre a little above grade so its lower half is
        # buried — a root plate is a hole in the ground with the plug beside
        # it, not a coin balanced on the lawn.
        px = x_m + e1[0] * c + e2[0] * s2
        py = y_m + e1[1] * c + e2[1] * s2
        pz = e1[2] * c + e2[2] * s2 + cz
        pts.append(Gf.Vec3f(px * ssf, py * ssf, pz * ssf))

    m = UsdGeom.Mesh.Define(stage, Sdf.Path(path))
    m.CreatePointsAttr(pts)
    counts, idx = [], []
    for i in range(int(n)):
        counts.append(3)
        idx += [0, 1 + i, 1 + (i + 1) % int(n)]
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(idx)
    nrm = Gf.Vec3f(ux * math.sin(t), uy * math.sin(t), math.cos(t))
    m.CreateNormalsAttr([nrm] * len(pts))
    m.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    m.CreateDisplayColorAttr([Gf.Vec3f(0.20, 0.15, 0.10)])
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    m.CreateExtentAttr([Gf.Vec3f(min(xs), min(ys), min(zs)),
                        Gf.Vec3f(max(xs), max(ys), max(zs))])
    if mat_prim_path:
        mat = UsdShade.Material(stage.GetPrimAtPath(mat_prim_path))
        if mat:
            UsdShade.MaterialBindingAPI(m.GetPrim()).Bind(mat)
    return path


def wind_tree(stage, tree_path, level, parent_path, out_parent, rng,
              azimuth_deg=0.0, wood_material_path="", soil_material_path="",
              debris=True, debris_scale=1.0, ground_z=0.0,
              seated_collide=True, verbose=False):
    """Take one tree apart with WIND. The tornado counterpart of `burn_tree`.

    Returns the same dict shape — `{"statics", "loose", "seated", "info",
    "level"}` — so a bake launcher can treat the two interchangeably.

    `azimuth_deg` is which way the wind was going. Everything directional
    reads it: the lean, the fall, and the lobe the debris lands in. Baking an
    archetype should leave it at 0 (fall toward +X) and let the assembly's own
    yaw aim the finished tree, because a baked archetype is yawed into place
    and an azimuth authored inside it comes out pointing wherever the
    placement happens to face.

    ORDER IS LOAD-BEARING, and it is the same order `burn_tree` documents:

      1. DEBRIS IS CUT FIRST. `snap` deactivates the bole, and
         `fracture.prim_to_mesh` on a deactivated prim returns nothing — so a
         debris pass after the geometry pass silently yields zero pieces.
      2. `tip_tree` comes LAST for the tipped levels, because it moves the
         whole tree and anything already written in world space (debris,
         limbs, the root plate) would be left behind if it ran first.

    NOTHING HERE CHARS. `scorch_foliage` and `char_bole` are not called and
    the level table has zeros where `burn_tree`'s has coverages, so the only
    materials this authors are plain timber and soil.
    """
    res = {"statics": [], "loose": [], "seated": [], "info": None,
           "level": level}
    info = survey(stage, tree_path, verbose=verbose)
    res["info"] = info
    if not info.get("bole") and not info.get("leaf_pi"):
        return res
    keep, keep_top, n_limbs, geom = wind_plan(level)

    # Black_Oak's woody branchlet prototype ships with NO material binding, so
    # repair it before anything is rotated into view — the same asset defect
    # `burn_tree` opens with.
    bind_bark(stage, info, verbose=verbose)

    # ---- 1. debris, BEFORE any geometry pass -----------------------------
    spec = _WIND_DEBRIS.get(level) if debris else None
    if spec:
        n_d, plen, rad = spec
        n_d = max(0, int(round(n_d * float(debris_scale))))
        if n_d > 0:
            _loose, _seated = wood_debris(
                stage, info, out_parent, rng, n_pieces=n_d, piece_len=plen,
                radius_m=rad, thick_m=_WIND_THICK, ground_z=ground_z,
                # NEARLY UNIFORM lengths, against the burnt path's 2.2. What
                # a wind leaves under a tree is limbs, and a distribution
                # tuned to produce mostly charcoal produces mostly nothing
                # you can see.
                len_bias=1.15,
                # The gate has to come down with the thickness — see
                # `wood_debris`. At 2.0 a 0.30 m-thick piece would have to be
                # 0.6 m long to survive, which throws away most of the range
                # just asked for.
                min_aspect=1.25,
                heading_deg=float(azimuth_deg), spread_deg=62.0,
                verbose=verbose)
            res["loose"] += _loose
            res["seated"] += _seated
            if seated_collide:
                res["statics"] += _seated

    # ---- 2. crown thinning (NOT stripping) -------------------------------
    # `strip_to_trunk` is never called on this path. `defoliate` removes a
    # SHARE of the leaf instances with a height gradient, which is what wind
    # does — it strips the exposed top harder than the sheltered interior —
    # and leaves a thinner but still green crown.
    if keep < 1.0 or keep_top < 1.0:
        defoliate(stage, info, rng, keep=keep, keep_top=keep_top,
                  verbose=verbose)

    # ---- 3. limbs down ---------------------------------------------------
    if n_limbs > 0:
        res["loose"] += fell_branches(stage, info, out_parent, rng,
                                      count=int(n_limbs), prefer_large=True,
                                      verbose=verbose)

    # ---- 4. geometry -----------------------------------------------------
    plate = None
    if geom == "snap":
        s, l = snap(stage, info, out_parent, rng,
                    # HIGHER THAN THE BURNT BREAK. A fire burns through a
                    # trunk low, where the flame front sat; wind snaps it
                    # where the section can no longer carry the crown's
                    # moment, which is up in the merchantable length. A stub
                    # at a third of the tree reads as a burnt snag, one at
                    # half to two thirds reads as a break.
                    height_frac=rng.uniform(0.42, 0.66),
                    tilt_deg=(4.0, 16.0), verbose=verbose)
        res["statics"] += s
        res["loose"] += l
    elif geom in ("lean", "uproot"):
        if geom == "lean":
            lean = rng.uniform(19.0, 38.0)
            lift = 0.0
            r_plate = 0.0
        else:
            # NOT FLAT. 72-82 degrees leaves the trunk a few degrees off
            # horizontal, which is where a real one comes to rest — its crown
            # is holding it up. Laying it at 90 puts the bole IN the ground
            # and reads as a log that was placed there.
            lean = rng.uniform(72.0, 82.0)
            # The lift is the root plate's own radius, which is both why the
            # base is off the ground and how far up it is.
            r_plate = rng.uniform(1.6, 3.0)
            lift = r_plate * 0.62
        tip_tree(stage, tree_path, lean, azimuth_deg=float(azimuth_deg),
                 lift_m=lift)
        if r_plate > 0.0:
            from pxr import UsdGeom as _UG

            # THE BASE, NOT THE BOUNDING-BOX CENTRE. The bbox of a tree lying
            # down is centred halfway along its trunk, which would put the
            # root plate in the middle of the crown. `tip_tree` pivots about
            # the local origin and leaves the translate op holding it, so the
            # translate IS the base — read it back rather than measuring.
            base = stage.GetPrimAtPath(tree_path)
            cx = cy = 0.0
            for op in _UG.Xformable(base).GetOrderedXformOps():
                if op.GetOpName().split(":")[-1] == "translate":
                    v = op.Get()
                    cx, cy = float(v[0]), float(v[1])
                    break
            plate = root_plate(
                stage, "{0}/rootplate_{1}".format(
                    out_parent, _tag(info, tree_path)),
                cx, cy, r_plate, float(azimuth_deg) + 180.0, rng,
                mat_prim_path=soil_material_path, ssf=1.0)
            res["statics"].append(plate)

    # ---- 5. one material for everything generated ------------------------
    made, seen = [], set()
    for p in res["statics"] + res["loose"] + res["seated"]:
        if p not in seen and p != plate:
            seen.add(p)
            made.append(p)
    if made:
        mat = None
        if wood_material_path:
            from pxr import UsdShade as _US
            mat = _US.Material(stage.GetPrimAtPath(wood_material_path)) or None
        if not mat:
            mat = plain_wood(stage, parent_path,
                             texture=material_texture(
                                 stage.GetPrimAtPath(info["bark_mat"])
                                 if info.get("bark_mat") else None) or "")
        bind_all(stage, made, mat, verbose=verbose)
    return res
