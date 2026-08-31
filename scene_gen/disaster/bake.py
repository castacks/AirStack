"""bake stage — write each damaged object to a SELF-CONTAINED USD, and reload.

WHY
---
Fracture (CPU/trimesh) and settle (GPU PhysX) are the whole cost of a burnt
block and do NOT scale to a full plat (see `build-wildfire-scenes`). The way
out is to do that work ONCE per object, save it, and reference it. This module
is the "save" half: export each damaged house/tree to its own USD, and read a
manifest back to rebuild the scene by reference.

WHY BY VALUE, NOT Sdf.CopySpec
------------------------------
The obvious export is `Sdf.CopySpec` of each object's flattened subtree. It
does not work on the KIT HOUSE MODULES: every kit mesh/subset/material carries
an `assetInfo` metadata dict whose value is a crate type core USD cannot unpack
(`Usd_CrateFile::_UnpackValue: unsupported type enum value 0`). Reading,
copying, clearing OR overwriting that field all raise — so any CopySpec that
touches such a prim dies. Freshly authored fracture fragments and tree trunk
meshes have no `assetInfo`, which is why they alone copied.

So geometry is rebuilt BY VALUE: `UsdAttribute.Get()` reads every geometry
attribute fine (it never unpacks `assetInfo`, which is prim metadata), so we
author fresh meshes from those values and never touch the poisoned field. Each
mesh gets its world transform baked onto it, so the hierarchy can be flattened
and a reference lands the object where it was. Materials are rebuilt as a fresh
Material shell (no `assetInfo`) whose clean Shader children ARE CopySpec-able,
with the bindings and connections remapped to the new paths.
"""

import json
import os

_GEOM_SKIP = {"xformOpOrder"}   # transforms are re-baked as a world matrix


def _reanchor_assets(v):
    """Rewrite asset-valued attribute values to their RESOLVED ABSOLUTE path.

    Kit shaders author `info:mdl:sourceAsset` and texture `inputs:*:file` as
    paths RELATIVE to the original kit asset (e.g. `../Materials/MI_Road.mdl`).
    Baking a material BY VALUE moves it into `assets/archetypes/`, so a stored
    relative path now anchors to the archetype's directory — it happens to
    resolve on the local mount but breaks the moment the archetype is opened
    from the Nucleus root, and the render shows grey roofs/walls/mailboxes
    (the `[EntityResolver] FAILED ...MI_Road / vMaterials_2...` log lines).

    `attr.Get()` runs inside the full Kit app during the bake, so each
    `Sdf.AssetPath` already carries `resolvedPath` — the absolute URL the
    resolver found (an `omniverse://…` for a Nucleus-sourced kit asset, an
    absolute mount path for a local one). Storing THAT instead of the authored
    relative path makes the archetype resolve from any asset root. Falls back
    to the authored path when the resolver left `resolvedPath` empty.
    """
    from pxr import Sdf
    if isinstance(v, Sdf.AssetPath):
        return Sdf.AssetPath(v.resolvedPath or v.path)
    if isinstance(v, (Sdf.AssetPathArray, list, tuple)) and len(v) \
            and all(isinstance(e, Sdf.AssetPath) for e in v):
        return Sdf.AssetPathArray([Sdf.AssetPath(e.resolvedPath or e.path)
                                   for e in v])
    return v


def _bound_material_prim(prim):
    from pxr import UsdShade
    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if m and m.GetPrim().IsValid():
        return m.GetPrim()
    return None


# DEAD WEIGHT ON EVERY BAKED MESH (agent O). Measured with
# `.agent_tmp/_o_attrs.py` on `bld_office_DG5.usd`: all 4824 meshes carry
# `cornerIndices`, `creaseIndices`, `creaseLengths`, `creaseSharpnesses`,
# `holeIndices`, `interpolateBoundary`, `faceVaryingLinearInterpolation`,
# `triangleSubdivisionRule`, `orientation`, `purpose`, `visibility` and
# `doubleSided` — every one of them at the SCHEMA FALLBACK — and 994 of them
# carry twenty-two `physics:*` / `physxRigidBody:*` attributes left over from
# the settle. The prim has **no applied API schemas at all** (the by-value
# export copies attributes, not `apiSchemas` metadata), so none of the physics
# ones do anything: PhysX only looks at prims with `PhysicsRigidBodyAPI` /
# `CollisionAPI` applied. That is ~40 authored attribute specs per mesh that
# cost composition time and crate bytes and change nothing.
_DEAD_PREFIXES = ("physics:", "physx")


def _copy_attrs_by_value(src_prim, dst_prim, skip_xform, strip_dead=False):
    """Author every attribute of src onto dst by VALUE. Skips xformOps when
    `skip_xform` (a world matrix is set instead).

    `strip_dead` drops the inert physics leftovers and every attribute still
    sitting at its schema fallback — provably a no-op, see above."""
    from pxr import UsdGeom
    pdef = src_prim.GetPrimDefinition() if strip_dead else None
    for a in src_prim.GetAttributes():
        name = a.GetName()
        if name in _GEOM_SKIP:
            continue
        if skip_xform and name.startswith("xformOp"):
            continue
        if strip_dead:
            if name.startswith(_DEAD_PREFIXES):
                continue
            try:
                fb = pdef.GetAttributeFallbackValue(name)
            except Exception:
                fb = None
            if fb is not None:
                try:
                    if a.Get() == fb:
                        continue
                except Exception:
                    pass
        try:
            v = a.Get()
        except Exception:
            v = None
        v = _reanchor_assets(v)
        na = dst_prim.CreateAttribute(a.GetName(), a.GetTypeName(),
                                      custom=a.IsCustom())
        if v is not None:
            na.Set(v)
        interp = a.GetMetadata("interpolation")
        if interp:
            na.SetMetadata("interpolation", interp)


def _rebuild_material(out, dst_path, src_mat_prim):
    """A fresh Material at dst_path reproducing src, entirely BY VALUE.

    Reads the material and every shader/nodegraph under it with `Get()` and
    re-authors them, remapping connections to the new paths. Touches no prim
    metadata, so the poisoned `assetInfo` is never read — and it needs no
    flattened source layer, so it works on the LIVE composed stage (kit
    material shaders live in referenced sublayers a root-layer `CopySpec`
    would miss). `UsdAttribute.Get()` resolves the composed value regardless.
    """
    from pxr import Sdf, Usd, UsdShade

    UsdShade.Material.Define(out, dst_path)
    old = src_mat_prim.GetPath()
    new = Sdf.Path(dst_path)

    def fix(p):
        return p.ReplacePrefix(old, new) if p.HasPrefix(old) else p

    for src in Usd.PrimRange(src_mat_prim):
        if src == src_mat_prim:
            dp = out.GetPrimAtPath(dst_path)
        else:
            rel = src.GetPath().MakeRelativePath(old).pathString
            dp = out.DefinePrim(dst_path + "/" + rel, src.GetTypeName())
        for a in src.GetAttributes():
            na = dp.CreateAttribute(a.GetName(), a.GetTypeName(),
                                    custom=a.IsCustom())
            try:
                v = a.Get()
            except Exception:
                v = None
            v = _reanchor_assets(v)
            if v is not None:
                na.Set(v)
            conns = a.GetConnections()
            if conns:
                na.SetConnections([fix(c) for c in conns])
    return dst_path


# ---------------------------------------------------------------------------
# POINT INSTANCERS (round-4 rubble, agent E) — carried into the archetype
# instead of being flattened into thousands of authored prims. See
# `_author_instancer` (in `export_object`) for the transform split and
# `_copy_prototype_tree` for how a prototype survives.
# ---------------------------------------------------------------------------
def _direct_references(src_prim):
    """[(resolved target layer identifier, target prim path), ...] for every
    REFERENCE arc authored DIRECTLY on `src_prim`'s own path (never one it
    merely inherited from an ancestor prim).

    `Usd.PrimCompositionQuery`'s arc already carries the RESOLVED target
    layer — `arc.GetTargetLayer().identifier` — the same way `attr.Get()`
    hands back a resolved `Sdf.AssetPath` for `_reanchor_assets`. So a
    prototype authored as `references = @../rubble/chunk_04.usd@` (relative
    to ITS OWN source file, wherever that is) resolves to that file's real
    path here, however far the exported archetype ends up from it — the
    identical problem `_reanchor_assets` solves for material attribute
    values, solved the same way for a composition arc instead of an
    attribute.
    """
    from pxr import Pcp, Usd
    out = []
    try:
        q = Usd.PrimCompositionQuery(src_prim)
    except Exception:
        return out
    for arc in q.GetCompositionArcs():
        if arc.GetArcType() != Pcp.ArcTypeReference or arc.IsAncestral():
            continue
        layer = arc.GetTargetLayer()
        if layer is None:
            continue
        out.append((layer.identifier, arc.GetTargetPrimPath()))
    return out


def _copy_local_xform(src_prim, dst_prim):
    """Reproduce src's OWN xformOps on dst_prim exactly — same op types,
    precision, suffix and values, in the same order.

    `_copy_attrs_by_value` always drops `xformOpOrder` (see `_GEOM_SKIP`)
    because every OTHER caller in this file rebakes a prim's transform to one
    fresh world-matrix op instead of preserving its original ops. A
    PointInstancer PROTOTYPE is not baked to world — it stays in its own
    local frame (see `_author_instancer`) — so if it carries a corrective
    local transform of its own, that transform has to be reproduced
    structurally or the copy is silently inert: the `xformOp:*` attribute
    values would exist on the new prim, authored by `_copy_attrs_by_value`,
    but with no `xformOpOrder` naming them nothing would ever apply them.
    """
    from pxr import UsdGeom
    sx = UsdGeom.Xformable(src_prim)
    ops = sx.GetOrderedXformOps() if sx else []
    if not ops:
        return
    dx = UsdGeom.Xformable(dst_prim)
    for op in ops:
        parts = op.GetOpName().split(":")
        suffix = ":".join(parts[2:]) if len(parts) > 2 else ""
        new_op = dx.AddXformOp(op.GetOpType(), op.GetPrecision(), suffix,
                               op.IsInverseOp())
        if op.IsInverseOp():
            continue
        try:
            v = op.Get()
        except Exception:
            v = None
        if v is not None:
            new_op.Set(v)


def _carry_direct_binding(out, src_prim, dst_prim, resolve_material):
    """If `src_prim` binds a material DIRECTLY on its own path, reproduce
    that binding on `dst_prim`, with the SAME binding strength token.

    `material:binding` is a RELATIONSHIP, so `_copy_attrs_by_value` (which
    only copies attributes) silently drops it — this is the bug: the
    rubble emitter's per-look override
    (`quake_rubble_usd._bind_override`, authored `strongerThanDescendants`
    on a PointInstancer prototype's wrapper Xform so it beats the
    referenced asset's own material deeper in the subtree) vanished on
    export, leaving every instanced debris piece render with its raw
    catalogue material instead.

    `GetDirectBinding()`, NOT `ComputeBoundMaterial()` (what
    `_bound_material_prim`/`material_for` use for the ordinary per-mesh
    path): a prototype subtree is exported in isolation from the rest of
    the source stage, so a binding it only INHERITS from some ancestor
    that is not part of the exported object must never be picked up here
    — only a binding relationship authored on `src_prim` itself counts.
    (For `src_prim` itself, `ComputeBoundMaterial()` would in fact resolve
    to the same material as `GetDirectBinding()` — a prim's own direct
    binding always wins for ITSELF regardless of any ancestor's strength —
    but going through the direct binding here keeps that guarantee
    explicit rather than incidental.)

    `resolve_material` takes an ALREADY-RESOLVED material prim (unlike
    `material_for`, which resolves one from an arbitrary prim first) — it
    is `export_object`'s `_cached_material` closure, so a material
    shared by many prototypes (a whole rubble `look`) is rebuilt into the
    exported stage's `Looks` scope exactly once.
    """
    from pxr import UsdShade
    db = UsdShade.MaterialBindingAPI(src_prim).GetDirectBinding()
    mat = db.GetMaterial()
    mp = mat.GetPrim() if mat else None
    if not (mp and mp.IsValid()):
        return
    dst_mat_path = resolve_material(mp)
    if not dst_mat_path:
        return
    strength = UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(
        db.GetBindingRel())
    UsdShade.MaterialBindingAPI(dst_prim).Bind(
        UsdShade.Material(out.GetPrimAtPath(dst_mat_path)),
        bindingStrength=strength)


def _copy_prototype_tree(out, dst_path, src_prim, strip_dead=False,
                         resolve_material=None):
    """Deep-copy ONE PointInstancer prototype subtree into the exported
    stage, in its OWN LOCAL FRAME — never world-baked, see `_author_instancer`.

    A prim carrying its own REFERENCE arc is copied AS A REFERENCE: the
    resolved target file + prim path (`_direct_references`) is re-referenced
    onto the new prim, so the referenced geometry is never flattened by
    value. This is the same pattern `gac_slice.rehome_materials` already uses
    to re-home a material onto its own file (`GetPrimStack()` +
    `AddReference`), applied here to a prototype instead of a material.

    A prim with NO reference of its own — an INLINE prototype, e.g. a bare
    Mesh authored directly on the layout stage rather than pulled in from
    Nucleus — is copied attribute-by-value like everything else in this
    file, and its children are walked recursively so a multi-prim inline
    prototype (a mesh plus GeomSubsets, say) survives whole.

    `resolve_material`, when given, is `export_object`'s cache-and-rebuild
    closure (see `_carry_direct_binding`). It is threaded through every
    level of the recursion — the prototype ROOT (the common case: the
    wrapper Xform the rubble emitter binds its override onto, which ALSO
    carries the reference and would otherwise return before any child is
    ever visited) as well as any INLINE child that carries its own direct
    binding — so a prototype's own material binding survives the copy
    instead of being dropped the way every relationship silently is under
    `_copy_attrs_by_value`. Left `None`, this function's behaviour (and
    every non-instancer caller's) is exactly what it was before.
    """
    from pxr import Sdf

    newp = out.DefinePrim(Sdf.Path(dst_path), src_prim.GetTypeName())
    _copy_attrs_by_value(src_prim, newp, skip_xform=True, strip_dead=strip_dead)
    _copy_local_xform(src_prim, newp)
    if resolve_material is not None:
        _carry_direct_binding(out, src_prim, newp, resolve_material)
    refs = _direct_references(src_prim)
    if refs:
        for ident, tpath in refs:
            if tpath and str(tpath) not in ("", "/"):
                newp.GetReferences().AddReference(ident, Sdf.Path(str(tpath)))
            else:
                newp.GetReferences().AddReference(ident)
        return newp
    for child in src_prim.GetChildren():
        _copy_prototype_tree(out, dst_path + "/" + child.GetName(), child,
                             strip_dead=strip_dead,
                             resolve_material=resolve_material)
    return newp


# ---------------------------------------------------------------------------
# MERGE — one mesh per material per archetype (agent O, round 3)
# ---------------------------------------------------------------------------
# WHY. A round-3 DG5 archetype is ~2300 settled fragments + ~6300 authored
# static pieces, and EVERY one of them was written here as its own
# `UsdGeom.Mesh` with its own transform, extent, subdivision scheme and
# material binding. Measured on the round-2 library (`scene_gen/tools/
# _o_usd_stat.py`): `bld_apartment_tall_DG4.usd` = 32.8 MB / 7993 prims of
# which 6403 meshes, and `bld_commercial_mid_DG5.usd` = 7729 meshes of which
# **6184 are 8-point heap lumps**. The city references one of these PER
# BUILDING and nothing is instanced (`scene_generator.apply_placements`
# docstring), so a 35-building plate composes and Hydra-syncs a quarter of a
# million prims for geometry that never moves again.
#
# Everything in a baked archetype is STATIC — `settle.bake` froze it — so the
# per-prim split buys nothing at all downstream. Nothing reads inside an
# archetype: `quake.assemble` only re-points the reference, `_tilt_prim`
# transforms the holder, `ground_effects` works off `archetypes.json`'s
# W/D/H and the records, and `_d_interactions` rebuilds the kit from
# `urban_building` rather than reading the bake (all verified by grep before
# this was written). So the pieces can be welded into one mesh per material.
#
# WHAT IS PRESERVED, EXACTLY. The merge is a pure re-packing:
#   * points are transformed by the same world matrix the per-prim path would
#     have authored, so nothing moves;
#   * a bucket key carries the material AND every attribute that decides
#     SHADING (normals interpolation, the primvar signature, subdivision
#     scheme, orientation, doubleSided, visibility, and the values of any
#     constant primvar), so two meshes only ever merge when the renderer
#     would have treated them identically. A mesh with no authored normals
#     never merges with one that has them — Hydra generates normals for the
#     first, and generating them over a merged mesh gives the same answer
#     because merged vertices are never welded across sources;
#   * a GeomSubset's faces go to the subset's OWN material bucket, so agent
#     T's `_t_core_bind` façade/core split survives as two merged meshes
#     rather than as 4600 subsets.
#
# MATERIALS ARE DEDUPED BY FINGERPRINT. `matmap` used to key on the SOURCE
# prim path, so a chair referenced 20 times by the layout rebuilt its whole
# material network 20 times: `bld_apartment_tall_TILT.usd` has 800 meshes and
# **334 Materials / 2463 Shaders** (312 distinct bindings). Hashing the
# rebuilt network's shader ids and input values collapses those to the
# handful that actually differ — which also lets the mesh merge do its job,
# because two identical-but-separate materials would otherwise be two
# buckets.
# REPEATED GEOMETRY IS NOT MERGED, AND THIS IS THE MEASUREMENT THAT SETTLED
# THE POINTINSTANCER QUESTION. A merge bakes the world transform into the
# points, which makes every copy of a repeated mesh UNIQUE — and USD crate
# stores one copy of an identical value array however many prims point at it.
# The kit modules are exactly that case: the same wall/window/chair geometry
# placed N times, differing only by their transform op. Measured by
# re-exporting the round-2 library both ways (`scene_gen/tools/_o_remerge.py`):
#
#   bld_apartment_tall_TILT  (223 kit LOD0 + 63 prop meshes, few fragments)
#       merge everything ->  1.20 MB grows to 11.99 MB   (10.0x)
#   bld_apartment_tall_DG4   (107 kit + 63 prop + 4626 fragments)
#       merge everything -> 32.79 MB grows to 36.86 MB   (1.12x)
#   bld_commercial_mid_DG5   (2 kit + 6184 unique heap lumps + 1525 frags)
#       merge everything -> 17.69 MB SHRINKS to 16.41 MB (0.93x)
#
# i.e. the growth tracks the share of REPEATED geometry exactly, and the
# authored damage — which is where the prims are — shrinks. So repeated
# geometry keeps its own prim (crate dedupes it, and there are only ~200 of
# them per building) and everything unique is merged. A `PointInstancer` for
# those ~200 would save the same bytes crate already saves and cost the
# drone stack's `add_colliders` / PhysX a prototype-instancing path it does
# not have today — no net win, so it is not built. The heap lumps that DO
# dominate the count cannot be instanced anyway: `_a_lump` jitters every one
# of its eight corners independently, so no two are the same prototype.
MERGE_REPEAT_MIN = int(os.environ.get("BAKE_MERGE_REPEAT") or "2")
MERGE_UNIFORM_NORMALS = (os.environ.get("BAKE_MERGE_UNIFORM_N", "1").strip()
                         not in ("0", "off", "false", "no"))
MERGE_MAX_FACES = 200000    # a bucket is split past this: one 500k-face mesh
#                             is a worse BLAS than three 200k ones, and it
#                             keeps a single crate value array from dominating


def merge_mode(explicit=None):
    """"off" | "on" | "both" — `both` writes the unmerged file as well, which
    is the only way to A/B the same settled geometry (a second bake would
    diverge on the rng).

    UNSET MEANS OFF HERE, and the QUAKE bake driver passes "on" itself. Three
    other bakes call `export_object` (wildfire, tornado, suburb_mini) and
    their archetypes have not been looked at through this; flipping their
    default from under them is not an optimisation, it is an untested change.
    """
    v = explicit if explicit is not None else os.environ.get("BAKE_MERGE", "")
    v = str(v).strip().lower()
    if v in ("both", "2"):
        return "both"
    if v in ("1", "on", "true", "yes"):
        return "on"
    return "off"


def _safe_name(s):
    out = "".join(c if (c.isalnum() or c == "_") else "_" for c in str(s))
    return ("m_" + out) if not out[:1].isalpha() and out[:1] != "_" else out


def _val_key(v):
    """A stable, cheap repr for an attribute value (arrays included)."""
    try:
        n = len(v)
    except TypeError:
        return repr(v)
    if n > 64:                       # long arrays: hash rather than repr
        import hashlib
        return "#{0}:{1}".format(n, hashlib.sha1(
            repr(list(v)).encode("utf-8", "replace")).hexdigest()[:16])
    return repr(list(v))


def _mat_fingerprint(src_mat_prim):
    """sha1 of a material network's SHAPE and VALUES, path-independent."""
    import hashlib

    from pxr import Usd
    h = hashlib.sha1()
    old = src_mat_prim.GetPath()
    for src in Usd.PrimRange(src_mat_prim):
        rel = src.GetPath().MakeRelativePath(old).pathString
        h.update(("|P " + rel + " " + str(src.GetTypeName())).encode())
        for a in sorted(src.GetAttributes(), key=lambda q: q.GetName()):
            try:
                v = a.Get()
            except Exception:
                v = None
            v = _reanchor_assets(v)
            h.update(("|A " + a.GetName() + "=" + _val_key(v)).encode(
                "utf-8", "replace"))
            for c in a.GetConnections():
                cp = c.ReplacePrefix(old, "/M") if c.HasPrefix(old) else c
                h.update(("|C " + cp.pathString).encode())
    return h.hexdigest()


def _np():
    import numpy as np
    return np


def _mat4(m):
    np = _np()
    return np.array([[float(m[i][j]) for j in range(4)] for i in range(4)])


def _geom_key(mesh):
    """A hash of a mesh's LOCAL geometry — points and topology, nothing else.

    Two prims with the same key are the same asset placed twice, which is the
    one case where merging costs more than it saves (see MERGE_REPEAT_MIN).
    Normals and primvars are deliberately not hashed: a collision only ever
    means "author this one per-prim, as before", which is always correct.
    """
    import hashlib

    np = _np()
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not counts:
        return None
    h = hashlib.sha1()
    h.update(np.asarray(pts, dtype=np.float32).tobytes())
    h.update(np.asarray(counts, dtype=np.int32).tobytes())
    h.update(np.asarray(idx if idx is not None else [], dtype=np.int32).tobytes())
    return h.digest()


def _mesh_signature(prim, mesh, pvars):
    """Everything except the material that must match for two meshes to be
    rendered identically. Two meshes with different answers here are never
    merged."""
    na = mesh.GetNormalsAttr()
    n_interp = (mesh.GetNormalsInterpolation()
                if na and na.HasAuthoredValue() else None)
    sig = [n_interp,
           str(mesh.GetSubdivisionSchemeAttr().Get()),
           str(mesh.GetOrientationAttr().Get()),
           bool(mesh.GetDoubleSidedAttr().Get()),
           str(mesh.GetVisibilityAttr().Get()),
           str(mesh.GetPurposeAttr().Get())]
    for pv in pvars:
        sig.append("{0}:{1}:{2}:{3}:{4}".format(
            pv.GetName(), pv.GetInterpolation(), pv.GetTypeName(),
            # indexed and flat primvars concatenate differently, so a bucket
            # may not mix them
            int(bool(pv.IsIndexed())),
            # a CONSTANT primvar cannot be concatenated, so its VALUE is part
            # of the key (displayColor/displayOpacity on authored debris)
            _val_key(pv.Get()) if pv.GetInterpolation() == "constant" else ""))
    return tuple(sig)


class _Bucket(object):
    """Accumulates transformed geometry for one (material, signature)."""

    __slots__ = ("pts", "counts", "indices", "normals", "pvars", "n_pts",
                 "n_faces", "n_src", "sig", "mat")

    def __init__(self, mat, sig):
        self.mat, self.sig = mat, sig
        self.pts, self.counts, self.indices = [], [], []
        self.normals = []
        self.pvars = {}          # name -> {"values": [...], "indices": [...]}
        self.n_pts = self.n_faces = self.n_src = 0


def _gather(prim, mesh, world, faces_sel, bucket, pvars, flip):
    """Append the selected faces of one source mesh to `bucket`, in world
    space. `faces_sel` is None for "all faces".

    EVERY COMPUTATION HAPPENS BEFORE THE FIRST MUTATION. The whole result is
    staged in `add` and spliced onto the bucket at the end, where nothing can
    raise. A half-gathered bucket would be geometry with no primvars — and
    because the caller falls back to the per-prim path on an exception, it
    would ALSO be the same faces authored twice.
    """
    np = _np()
    pts = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not pts or not counts or idx is None:
        return False
    P = np.asarray(pts, dtype=np.float64)
    C = np.asarray(counts, dtype=np.int64)
    I = np.asarray(idx, dtype=np.int64)
    starts = np.concatenate(([0], np.cumsum(C)))[:-1]
    if faces_sel is None:
        fsel = np.arange(len(C), dtype=np.int64)
    else:
        fsel = np.asarray(faces_sel, dtype=np.int64)
        if not len(fsel):
            return False
    # flat face-vertex positions of the selected faces, in face order
    fv_pos = np.concatenate([np.arange(starts[f], starts[f] + C[f],
                                       dtype=np.int64) for f in fsel])         if len(fsel) else np.zeros(0, dtype=np.int64)
    sel_idx = I[fv_pos]
    uniq, inv = np.unique(sel_idx, return_inverse=True)
    # world transform: USD is row-vector, translation in row 3
    A = _mat4(world)
    Pw = P[uniq] @ A[:3, :3] + A[3, :3]
    new_counts = C[fsel]
    new_idx = inv.astype(np.int64) + bucket.n_pts
    if flip:
        # a mirroring transform reverses winding; put it back per face
        out, o = [], 0
        for c in new_counts:
            out.append(new_idx[o:o + c][::-1])
            o += c
        new_idx = np.concatenate(out) if out else new_idx
    add = {"pts": Pw, "counts": new_counts, "indices": new_idx,
           "normals": None, "pvars": []}

    na = mesh.GetNormalsAttr()
    if na and na.HasAuthoredValue():
        N = np.asarray(na.Get(), dtype=np.float64)
        Ainv3 = np.linalg.inv(A[:3, :3]).T
        interp = mesh.GetNormalsInterpolation()
        Ns = N[fv_pos] if interp == "faceVarying" else (
            N[fsel] if interp == "uniform" else N[uniq])
        Nw = Ns @ Ainv3
        ln = np.linalg.norm(Nw, axis=1, keepdims=True)
        Nw = Nw / np.where(ln > 1e-12, ln, 1.0)
        if flip and interp == "faceVarying":
            out, o = [], 0
            for c in new_counts:
                out.append(Nw[o:o + c][::-1])
                o += c
            Nw = np.concatenate(out) if out else Nw
        add["normals"] = Nw

    for pv in pvars:
        name = pv.GetName()
        interp = pv.GetInterpolation()
        slot = bucket.pvars.setdefault(name, {"values": [], "indices": [],
                                              "interp": interp,
                                              "type": pv.GetTypeName(),
                                              "indexed": False})
        if interp == "constant":
            add["pvars"].append((name, {"const": pv.Get()}))
            continue
        vals = pv.Get()
        pvi = pv.GetIndices() if pv.IsIndexed() else None
        if vals is None:
            continue
        if pvi is not None:
            # keep it indexed: remap the index array, keep the value array
            Vi = np.asarray(pvi, dtype=np.int64)
            take = (fv_pos if interp == "faceVarying"
                    else (fsel if interp == "uniform" else uniq))
            base = sum(len(v) for v in slot["values"])
            sel = Vi[take] + base
            if flip and interp == "faceVarying":
                out, o = [], 0
                for c in new_counts:
                    out.append(sel[o:o + c][::-1])
                    o += c
                sel = np.concatenate(out) if out else sel
            add["pvars"].append((name, {"indexed": True,
                                        "values": list(vals),
                                        "indices": sel}))
        else:
            V = list(vals)
            take = (fv_pos if interp == "faceVarying"
                    else (fsel if interp == "uniform" else uniq))
            sel = [V[int(k)] for k in take]
            if flip and interp == "faceVarying":
                out, o = [], 0
                for c in new_counts:
                    out.append(sel[o:o + int(c)][::-1])
                    o += int(c)
                sel = [q for chunk in out for q in chunk]
            add["pvars"].append((name, {"values": sel}))

    # ---- commit: nothing below here can raise -------------------------------
    bucket.pts.append(add["pts"])
    bucket.counts.append(add["counts"])
    bucket.indices.append(add["indices"])
    if add["normals"] is not None:
        bucket.normals.append(add["normals"])
    for name, d in add["pvars"]:
        slot = bucket.pvars[name]
        if "const" in d:
            slot["const"] = d["const"]
            continue
        if d.get("indexed"):
            slot["indexed"] = True
            slot["indices"].append(d["indices"])
        slot["values"].append(d["values"])
    bucket.n_pts += int(len(uniq))
    bucket.n_faces += int(len(fsel))
    bucket.n_src += 1
    return True


def _write_bucket(out, path, bucket):
    from pxr import Gf, UsdGeom, Vt
    np = _np()

    P = np.concatenate(bucket.pts) if bucket.pts else np.zeros((0, 3))
    C = np.concatenate(bucket.counts) if bucket.counts else np.zeros(0, int)
    I = np.concatenate(bucket.indices) if bucket.indices else np.zeros(0, int)
    if not len(C):
        return None
    m = UsdGeom.Mesh.Define(out, path)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(float(a), float(b), float(c))
                                      for a, b, c in P]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([int(q) for q in C]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(q) for q in I]))
    lo = P.min(axis=0)
    hi = P.max(axis=0)
    m.CreateExtentAttr([Gf.Vec3f(*[float(q) for q in lo]),
                        Gf.Vec3f(*[float(q) for q in hi])])
    (n_interp, subdiv, orient, dsided, vis, purpose) = bucket.sig[:6]
    if subdiv and subdiv != "None":
        m.CreateSubdivisionSchemeAttr(subdiv)
    if orient and orient != "None":
        m.CreateOrientationAttr(orient)
    if dsided:
        m.CreateDoubleSidedAttr(True)
    if vis and vis not in ("None", "inherited"):
        m.CreateVisibilityAttr(vis)
    if purpose and purpose not in ("None", "default"):
        m.CreatePurposeAttr(purpose)
    if n_interp and bucket.normals:
        N = np.concatenate(bucket.normals)
        # FLAT NORMALS COMPACT TO `uniform`, AND THAT IS WHERE THE FILE IS.
        # A fracture fragment is flat-shaded: all of a face's face-varying
        # normals are the same vector. Measured on `bld_apartment_tall_DG4`,
        # 696554 faces average 3.5 vertices, so `faceVarying` normals are
        # 2.44M Vec3f = 29 MB of a 31 MB file — one per FACE is 8 MB. It is
        # the same value at every face vertex either way, so nothing shades
        # differently; `BAKE_MERGE_UNIFORM_N=0` turns it off if a renderer
        # ever disagrees.
        if (n_interp == "faceVarying" and MERGE_UNIFORM_NORMALS
                and len(N) == int(C.sum())):
            starts = np.concatenate(([0], np.cumsum(C)))[:-1].astype(np.int64)
            first = N[starts]
            # every face-vertex equal to its face's first normal?
            flat = np.all(np.abs(N - np.repeat(first, C.astype(np.int64),
                                               axis=0)) < 1e-6)
            if flat:
                N = first
                n_interp = "uniform"
        m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(float(a), float(b), float(c))
                                           for a, b, c in N]))
        m.SetNormalsInterpolation(n_interp)
    api = UsdGeom.PrimvarsAPI(m.GetPrim())
    for name, slot in bucket.pvars.items():
        base = name.split(":", 1)[-1] if name.startswith("primvars:") else name
        pv = api.CreatePrimvar(base, slot["type"], slot["interp"])
        if "const" in slot:
            pv.Set(slot["const"])
            continue
        vals = [q for chunk in slot["values"] for q in chunk]
        if not vals:
            continue
        # BUILD THE Vt ARRAY EXPLICITLY. `attr.Set(python_list)` converts for
        # some element types and silently refuses for others (TexCoord2f),
        # which loses the UVs on a kit wall without raising.
        try:
            arr = slot["type"].type.pythonClass(vals)
        except Exception:
            arr = vals
        pv.Set(arr)
        if slot["indexed"]:
            idxs = np.concatenate(slot["indices"])
            pv.SetIndices(Vt.IntArray([int(q) for q in idxs]))
    return m.GetPrim()


_SINK_TOL_M = 0.02      # below this a root counts as "sank", not as "resting"
# ...AND ABOVE THIS ONE IT COUNTS AS AIRBORNE. A fragment resting on the pile
# has a few centimetres of daylight under its axis-aligned box simply because
# the box is bigger than the shape; a fragment the solver froze in mid-flight
# has tens of centimetres. 0.10 m is comfortably past the first and well short
# of the second — measured on the tornado archetypes, the airborne population
# has a MEDIAN gap of 0.18-0.24 m and a tail to 2.98 m.
_AIR_TOL_M = 0.10


def _reseat_roots(bc, roots, rz, sink_tol=_SINK_TOL_M, air_tol=_AIR_TOL_M,
                  min_ovl=0.20, freeze=(), pre=None):
    """Per-root Z corrections that put a settled pile back on the ground.

    PURE GEOMETRY, NO SOLVER. Returns `{root path: dz}` for the roots that
    need moving, and it does two separate things:

      RAISE what fell through the floor. The bake's harness ground is a
      four-vertex quad added to the static set, and at `THROW_MPS = 9` the
      loose fragments reach ~20 m/s — fast enough to tunnel it. Measured on
      the 2026-08-27 tornado archetypes, 21-42% of the meshes in every wrecked
      house sat below -0.05 m and whole-object `z_min` ran to -2.9 m. That is
      material simply lost from the pile.

      LOWER what was frozen in the air. `settle.run` treats `steps` as a
      CEILING with an early exit, not as a convergence test — the last bake
      used all 1200 steps with 4 bodies still moving, and earlier ones left
      182 of 7,219 "frozen mid-flight". Measured on the same archetypes,
      11-29% of the meshes in a `leveled` or `partial_collapse` house had NO
      other mesh under them within 0.10 m: a median 0.19 m of daylight and a
      tail to 2.6 m. With the scene's sun at 24.4 degrees of elevation that is
      0.42 m of detached shadow at the median, which is what "floating debris"
      looks like from the air.

    THE SUPPORT TEST IS AXIS-ALIGNED, so it OVER-counts support and therefore
    UNDER-counts floaters. That is the safe direction: a piece wrongly judged
    supported is left where the solver put it, which is the status quo.

    Roots are placed from the bottom up and each one's box is updated as it
    lands, so a stack settles rather than every piece being dropped onto the
    original surface.

    `freeze` NAMES ROOTS THAT ARE SUPPORT BUT NEVER MOVE, and the tree path is
    why it exists. A windthrown tree's pose is AUTHORED, not settled:
    `tip_tree` bisects the lean down until the crown is just into the turf
    (`seat_band = (-1.1, -0.15)`) and `wind_tree` lifts the base by the root
    plate's own radius on purpose. Both of those look exactly like "sank
    through the floor" to the test above, so reseating a tree along with its
    debris would stand every fallen trunk back up on the lawn and undo the one
    thing that makes it read as windthrow. A frozen root still contributes its
    box, so a log CAN come to rest on a fallen trunk.

    NO `pxr` IMPORT. Everything below is arithmetic on six floats a box, and
    `bc` is only ever asked for a world bound — so this function runs, and is
    unit-tested, on the host with a stand-in cache. That matters: it is the
    one piece of the bake whose correctness is not obvious (the support test
    got the intact roofs wrong on the first try) and it is the only piece that
    can be checked without a GPU.
    """
    boxes = []
    pre = pre or {}
    for r in roots:
        wr = bc.ComputeWorldBound(r).ComputeAlignedRange()
        if wr.IsEmpty():
            continue
        mn, mx = wr.GetMin(), wr.GetMax()
        # `pre` is a shift the caller has ALREADY decided for this root, so
        # measure it where it is going rather than where it is.
        dz = float(pre.get(r.GetPath().pathString, 0.0))
        boxes.append([r.GetPath().pathString,
                      float(mn[0]), float(mn[1]), float(mn[2]) + dz,
                      float(mx[0]), float(mx[1]), float(mx[2]) + dz])
    out = {}
    held_fast = set(str(p) for p in freeze)
    # Bottom up, so a piece lands on what has already been placed under it.
    boxes.sort(key=lambda b: b[3])
    for i, b in enumerate(boxes):
        path, x0, y0, z0, x1, y1, z1 = b
        if path in held_fast:
            continue
        if z0 < rz - sink_tol:
            dz = rz - z0                       # lost through the floor
            out[path] = dz
            b[3] += dz
            b[6] += dz
            continue
        # WHAT COUNTS AS HOLDING US UP, and this took two goes to get right.
        #
        # v1 asked only for boxes lying ENTIRELY below (`q_top <= z0`). A gable
        # roof sitting on its walls FAILS that — the roof's box includes its
        # overhanging soffit, so the walls' tops are above the roof's own
        # min-z — and every intact roof in the library would have been dropped
        # to the lawn.
        #
        # v2 asked for anything overlapping in plan that REACHES our underside
        # (`q_top >= z0 - air_tol`). That fixed the roofs and did almost
        # nothing else: measured on the rebake, `house_ranch_leveled` went
        # 17.0% airborne to 15.1%. The reason is that a levelled pile is full
        # of tall wall stubs, and a stub spanning 0 to 2.5 m "supports"
        # everything within its plan footprint at every height — including a
        # fragment frozen at 1.75 m with nothing but air under it.
        #
        # v3, here: the support's top has to land IN OUR VERTICAL SPAN —
        # `z0 - air_tol <= q_top <= z1`. Something whose top is at our
        # underside, or inside us, is plausibly what we came to rest on. A box
        # that simply TOWERS PAST us is not: it might be a wall we are wedged
        # against, and an axis-aligned test cannot tell, so the tie is broken
        # toward dropping. That is the right way to break it — a fragment
        # lying on the ground is a pose the solver could plausibly have
        # reached, and a fragment hanging in mid-air never is.
        #
        # And the overlap has to be a SEAT rather than a corner clip: at least
        # `min_ovl` of the smaller of the two plan areas. Measured against the
        # smaller so a small stub can still hold up a large sheet, which is
        # exactly how a wall holds up a roof.
        a_self = max(1e-6, (x1 - x0) * (y1 - y0))
        support, held = rz, False
        for j, q in enumerate(boxes):
            if j == i:
                continue
            ox = min(q[4], x1) - max(q[1], x0)
            oy = min(q[5], y1) - max(q[2], y0)
            if ox <= 0.0 or oy <= 0.0:
                continue                       # no plan overlap
            a_q = max(1e-6, (q[4] - q[1]) * (q[5] - q[2]))
            if (ox * oy) / min(a_self, a_q) < min_ovl:
                continue                       # a corner clip, not a seat
            if z0 - air_tol <= q[6] <= z1:
                held = True
                break
            if q[6] <= z0 and q[6] > support:
                support = q[6]
        if not held and (z0 - support) > air_tol:
            dz = -(z0 - support)
            out[path] = dz
            b[3] += dz
            b[6] += dz
    return out


def world_point_bounds(prim, xcache):
    """TIGHT world AABB of a mesh, computed from its POINTS.

    **`UsdGeom.BBoxCache` IS NOT USABLE FOR SEATING, AND THIS IS THE SINGLE
    MOST EXPENSIVE THING IN THIS FILE TO NOT KNOW.** It returns the AABB of an
    AABB: it takes the prim's LOCAL extent box, transforms its eight corners,
    and re-axis-aligns. For a piece whose geometry is a thin sliver lying
    DIAGONALLY inside its local box — which is most Voronoi debris — that
    inflates enormously, and it inflates DOWNWARD as much as upward.

    Measured on `tree_Black_Oak_snag/log_017` in a built scene::

        local points fill  x +-0.471  y +-0.284  z +-0.240  (extent exact)
        world z from POINTS      0.4208 .. 0.5649   span 0.144
        world z from BBoxCache   0.0000 .. 0.9857   span 0.986

    The piece hangs 42 cm in the air and its bbox bottom reads 0.000. So every
    pass that seats or audits through `BBoxCache` — `_reseat_roots`,
    `audit_archetype`, `_seat_plan`, `vegetation.wood_debris`' own
    `piece.bounds[0][2] = ground_z - 0.001` — calls it grounded, and a whole
    library audits clean while the scene is full of floating debris.

    Reading points is more expensive. It is also the only number that is true.
    """
    from pxr import Gf, UsdGeom

    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get() if mesh else None
    if not pts:
        return None
    m = xcache.GetLocalToWorldTransform(prim)
    lo = [1e30, 1e30, 1e30]
    hi = [-1e30, -1e30, -1e30]
    for q in pts:
        w = m.Transform(Gf.Vec3d(float(q[0]), float(q[1]), float(q[2])))
        for k in range(3):
            v = float(w[k])
            if v < lo[k]:
                lo[k] = v
            if v > hi[k]:
                hi[k] = v
    return lo, hi


def _shift_z(prim, dz):
    """Move *prim* by *dz* in WORLD z, whatever transform it already carries.

    **DO NOT APPEND A TRANSLATE OP.** These fragments carry an
    `xformOp:transform` WITH SCALE, and an appended translate composes INSIDE
    it — measured on `house_l_family_partial_collapse`: an authored -1.579 m
    moved the piece -1.279 m, exactly `dz * 0.8098`, its own z scale; a second
    fragment with a 0.3616 z scale moved -0.327 m against an authored -0.904.
    A repair that silently applies a fraction of what it computed is worse than
    none, because the audit after it reports a smaller number and looks like
    progress.

    The matrix's translation is applied after the linear part, so adding `dz`
    to it is exactly `dz` in the parent frame — and `/Baked` is identity, so
    the parent frame is world. A prim with no ops at all gets a plain
    translate, which for that case is unambiguous.

    The `extent` hint is updated with it. Nothing here reads it (every cache is
    built `useExtentsHint=False`), but a stale hint hands a consumer that DOES
    trust it the pre-repair box.
    """
    from pxr import Gf, UsdGeom

    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    hit = False
    for o in ops:
        if o.GetOpType() == UsdGeom.XformOp.TypeTransform:
            m = Gf.Matrix4d(o.Get())
            t = m.ExtractTranslation()
            m.SetTranslateOnly(Gf.Vec3d(t[0], t[1], t[2] + dz))
            o.Set(m)
            hit = True
            break
    if not hit:
        for o in ops:
            if o.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                v = o.Get()
                o.Set(type(v)(v[0], v[1], v[2] + dz))
                hit = True
                break
    if not hit:
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,
                          "reseat").Set(Gf.Vec3d(0.0, 0.0, dz))
    ext = prim.GetAttribute("extent")
    if ext and ext.HasAuthoredValue():
        v = ext.Get()
        if v:
            ext.Set([Gf.Vec3f(v[0][0], v[0][1], v[0][2] + dz),
                     Gf.Vec3f(v[1][0], v[1][1], v[1][2] + dz)])
    return hit


def reseat_meshes_in_file(path, grade=0.0, air_tol=_AIR_TOL_M, min_ovl=0.20,
                          names=("log_", "debris", "frag_", "brk_", "bole_"),
                          ground_only=("bole_",),
                          dry_run=False, verbose=True):
    """Lower genuinely airborne DEBRIS MESHES in an already-baked archetype.

    WHY THIS EXISTS SEPARATELY FROM `_reseat_roots`, which already runs at bake
    time and is supposed to prevent exactly this. The two disagree on
    GRANULARITY, and the disagreement is the bug:

      * `_reseat_roots` compares ROOT boxes. A root that holds several meshes
        is measured by its COMBINED bbox, so a module spanning 0 to 3 m
        "supports" anything whose underside falls in that span — even when no
        actual surface is under the piece. A tall box is not a surface.
      * `audit_archetype` compares MESH boxes and does find them. On the
        2026-08-27 wildfire bake it reported "2 of 4691 meshes with nothing
        under them (0.0%)" — the finding was there and the percentage rounded
        it away.

    Measured consequence on the 1 km plate: 18 fragments hanging at 2.15-3.08 m
    across the scene, every one of them from `house_l_family_partial_collapse`
    or `house_l_family_roof_collapsed`. Four discrete heights, because two
    files are instanced many times — which is the signature of a defect in a
    FILE rather than in a placement.

    PURE GEOMETRY AND PURE `pxr`. No solver, no Kit, no SimulationApp — it
    opens the archetype, measures, authors a translate on the offending mesh
    and saves. So an existing library can be repaired in place instead of
    re-baked, and the repair can be verified by re-running the same audit that
    found the fault.

    The support rule is `_reseat_roots`' v3 test, unchanged: a supporter's top
    must land IN our vertical span (something that merely TOWERS PAST us might
    be a wall we are wedged against, and the tie breaks toward dropping), and
    the plan overlap must be a SEAT — `min_ovl` of the SMALLER of the two plan
    areas, so a small stub can still hold up a large sheet.

    Returns `[(prim path, dz), ...]`.
    """
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(path)
    if stage is None:
        raise RuntimeError("cannot open " + path)
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    root = stage.GetPrimAtPath("/Baked") or stage.GetPseudoRoot()
    boxes = []
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        # POINTS, NOT `BBoxCache` — see `world_point_bounds`. Measuring this
        # through the bbox cache is what let a library of floating debris audit
        # as clean.
        b = world_point_bounds(prim, xc)
        if b is None:
            continue
        (mnx, mny, mnz), (mxx, mxy, mxz) = b
        boxes.append([prim, mnx, mny, mnz, mxx, mxy, mxz])

    # BOTTOM UP, and the boxes are mutated as pieces land, so a stack settles
    # rather than every piece being dropped onto the original surface.
    #
    # `ground_only` PIECES GO FIRST, whatever their height. A felled bole is
    # the heaviest thing in the archetype and it rests on the GROUND; the loose
    # wood rests on IT. Ordering by z alone put the logs down first and then
    # dropped a 7.8 m Black_Oak trunk onto the only thing under it — the
    # 1.15 m `stump` — which is a support the test accepts and gravity does
    # not: a trunk balanced across a stump top with its far end 2.3 m in the
    # air slides off in reality. Seat it on grade and let the debris follow.
    _first = tuple(k.lower() for k in (ground_only or ()))
    boxes.sort(key=lambda b: (0 if any(k in b[0].GetName().lower()
                                       for k in _first) else 1, b[3]))
    moved = []
    for i, b in enumerate(boxes):
        prim, x0, y0, z0, x1, y1, z1 = b
        nm = prim.GetName().lower()
        if not any(k in nm for k in names):
            continue                       # structure stays where it is
        # `bole_*` IS LOOSE, and leaving it out of the family list cost a
        # round. It is the felled trunk of a `*_fallen` tree — cut by
        # `vegetation.topple` and settled by PhysX, so it is exactly the
        # population a settle can freeze mid-flight. All twelve in the library
        # were airborne, Black_Oak's `bole_00` by 2.07 m. It never showed in a
        # name census because there are only two per archetype and the census
        # was cut off at the commonest 25 names.
        a_self = max(1e-6, (x1 - x0) * (y1 - y0))
        support, held = grade, False
        # A `ground_only` piece ignores every candidate support: it is defined
        # to rest on the earth, not on whatever the settle happened to leave
        # beneath it.
        scan = () if any(k in nm for k in _first) else enumerate(boxes)
        for j, q in scan:
            if j == i:
                continue
            ox = min(q[4], x1) - max(q[1], x0)
            oy = min(q[5], y1) - max(q[2], y0)
            if ox <= 0.0 or oy <= 0.0:
                continue
            a_q = max(1e-6, (q[4] - q[1]) * (q[5] - q[2]))
            if (ox * oy) / min(a_self, a_q) < min_ovl:
                continue
            if z0 - air_tol <= q[6] <= z1:
                held = True
                break
            if q[6] <= z0 and q[6] > support:
                support = q[6]
        if held or (z0 - support) <= air_tol:
            continue
        dz = -(z0 - support)
        moved.append((prim.GetPath().pathString, round(dz, 4)))
        b[3] += dz
        b[6] += dz
        if dry_run:
            continue
        _shift_z(prim, dz)
    if moved and not dry_run:
        stage.GetRootLayer().Save()
    if verbose:
        print("[reseat] {0}: {1} mesh(es) lowered{2}".format(
            os.path.basename(path), len(moved), " (dry run)" if dry_run else ""))
        for pth, dz in moved:
            print("           {0:<44} {1:+.3f} m".format(pth, dz))
    return moved


def _seat_plan(bc, roots, recenter=None, drop_to_ground=False,
               reseat=False, reseat_first=True, reseat_freeze=None):
    """Decide `(rz, {root path: dz})` — every Z correction one export makes.

    SLICED OUT OF `export_object` SO IT CAN BE TESTED WITHOUT KIT. `bc` is
    only ever asked for `ComputeWorldBound(prim).ComputeAlignedRange()`, and
    a root is only ever asked for `GetPath().pathString`, so a dozen lines of
    stand-in stand in for the whole of USD — see `tests/test_bake_reseat.py`.
    The composition of the two corrections is the part worth pinning: it is
    where the floating-debris bug lived, and it is invisible in a render until
    it is wrong.

    `rz` is what the caller subtracts from EVERY mesh; `dz` is the extra shift
    for one root. So a root finishes at `z_src - rz + dz`.
    """
    rx, ry, rz = (recenter or (0.0, 0.0, 0.0))
    lift = {}
    if not (drop_to_ground or reseat):
        return rz, lift
    # THE GRADE IS THE DATUM EVERYTHING WAS AUTHORED AGAINST — the harness
    # ground, `recenter`'s own Z, normally 0. `drop_to_ground` moves the
    # OBJECT relative to it; it must never move the ground.
    grade = rz

    def _min_z(prim):
        wr = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        return None if wr.IsEmpty() else wr.GetMin()[2]

    def _add(path, dz):
        lift[path] = lift.get(path, 0.0) + dz

    if drop_to_ground:
        # The object itself sets the offset. When its own geometry is gone
        # (a `stump`, or a `snap`/`topple` that deactivated the bole) there is
        # nothing to measure and `recenter`'s Z stands — everything was
        # authored against a harness ground at z=0, so that is right.
        z0 = _min_z(roots[0])
        if z0 is not None:
            rz = z0
        # DROP THE OBJECT, NOT ITS DEBRIS — the second half of the lesson in
        # `export_object`'s docstring, arriving from the other direction. The
        # offset is subtracted from EVERY mesh, so a first root whose own
        # geometry dips BELOW grade (a root flare under the turf, a low branch
        # past the trunk's base) makes `rz` negative and LIFTS the whole file,
        # including every stick `wood_debris` placed analytically at exactly
        # `ground_z - 0.001`. Measured 2026-08-27 on the shipped library: in
        # `tree_Shumard_Oak_snapped` and `_limbed` the seated sticks sat at
        # EXACTLY 0.530 m with clear air under them and 35-40% of all meshes
        # were unsupported, while every species whose base is at its origin was
        # clean and both `_NO_DROP` levels were clean at every species. The
        # float appeared exactly, and only, where the drop was applied.
        #
        # So the datum shift is compensated back out for every root that is NOT
        # the object: debris keeps the absolute Z it was authored at, and the
        # reseat pass below is what corrects a piece the solver misplaced.
        if rz != grade:
            for r in roots[1:]:
                _add(r.GetPath().pathString, rz - grade)
        if not reseat:
            # The old per-root rescue, for callers that do not ask for the full
            # geometric pass: raise anything left under the grade.
            for r in roots[1:]:
                zr = _min_z(r)
                if zr is not None and (zr - grade) < -_SINK_TOL_M:
                    _add(r.GetPath().pathString, grade - zr)

    if reseat:
        # A wrecked building is not DROPPED — it was authored on the harness
        # ground and that is where it belongs — but its individual fragments
        # still have to be put back on the pile. `drop_to_ground` cannot be
        # reused for that: its first act is to redefine the datum from the
        # first root, and for a house the first root is one module of many.
        #
        # `reseat_first=False` is the TREE case: the first root's pose is
        # authored deliberately (see `_reseat_roots(freeze=...)`) and only what
        # it shed needs seating.
        #
        # `pre` measures a root WHERE IT WILL ACTUALLY END UP rather than where
        # the solver left it, so a log lying across a trunk that is about to be
        # dropped comes down with it. ONLY THE FIRST ROOT NEEDS ONE: the pass
        # works in source coordinates against `grade`, which is already where
        # the debris finishes — its entry above exists purely to cancel the
        # datum shift, so its net movement is zero and pre-shifting it too
        # would count the same offset twice.
        pre = {}
        if drop_to_ground and rz != grade:
            pre[roots[0].GetPath().pathString] = grade - rz
        frozen = set(str(q) for q in (reseat_freeze or ()))
        if not reseat_first:
            frozen.add(roots[0].GetPath().pathString)
        for path, dz in _reseat_roots(
                bc, roots, grade, pre=pre, freeze=frozen).items():
            _add(path, dz)
    return rz, lift


def export_object(src_stage, _flat_unused, obj_paths, out_path, root="/Baked",
                  recenter=None, drop_to_ground=False, merge=None,
                  stats_out=None, reseat=False, reseat_first=True,
                  reseat_freeze=None):
    """Write one object's meshes + materials to a self-contained USD by value.

    `src_stage` is a stage (typically the Kit-flattened scene) that has the
    object at `obj_paths`. `_flat_unused` is ignored (kept for call
    compatibility). `recenter=(x,y,z)` subtracts that offset from every mesh's
    world translation. `drop_to_ground=True` seats the object on z=0 — needed
    for trees, whose fallen boles sink through the harness ground during settle
    (measured to -2.4 m) and end up buried once referenced.
    Returns True if any mesh OR PointInstancer was written.

    POINT INSTANCERS (round-4 rubble) ARE CARRIED, NOT FLATTENED. A
    `UsdGeom.PointInstancer` under an exported path is copied whole —
    `_author_instancer` — rather than being expanded into one authored prim
    per instance: it is already the cheap representation the mesh MERGE above
    exists to approximate for everything else, so it is authored directly and
    never enters the merge, whatever `merge` is set to. See
    `_author_instancer`'s docstring for exactly which frame its
    `positions`/`orientations`/`scales` end up in after export, and
    `_copy_prototype_tree` for how its prototypes — referenced Nucleus assets
    or inline meshes alike — survive the move.

    DROP THE OBJECT, NOT THE WHOLE PILE. `drop_to_ground` used to take the
    MINIMUM world min-Z over EVERY root it was handed — the tree AND all of
    its ground debris — and subtract that from all of them together. One log
    that PhysX pushed through the harness floor therefore lifted the entire
    archetype: measured on the previous bake, every seated stick in
    `tree_American_Beech_torched` sat at exactly 0.567 m, every one in
    `tree_Black_Oak_snag` at 0.80 m and every one in `tree_Black_Oak_fallen`
    at 3.56 m, with the trunk floating by the same amount. The shift was
    uniform per file, which is the signature of a single global offset rather
    than of anything physical. `scorched` files were clean only because
    nothing in them is simulated, so nothing could sink.

    So the offset comes from the FIRST root alone — the caller passes
    `[tree] + debris`, and the tree is the thing whose base defines where the
    object sits. Any OTHER root that is still below ground after that offset
    is treated individually: it is a piece the solver lost through the floor,
    so it is raised on its own until its lowest point is at 0. That is a
    handful of pieces per file rather than all of them, it leaves every
    correctly-seated stick exactly where it was analytically placed, and a log
    lifted to rest flat on the ground is a pose it could plausibly have
    settled into anyway. Raising is preferred over dropping the piece because
    the debris count is part of what the level looks like; a missing log is a
    more visible loss than a log lying in a slightly different place.

    AND THE DATUM SHIFT IS COMPENSATED BACK OUT FOR THE DEBRIS. That fix above
    stopped one sunk LOG lifting the file; the same defect arrives again from
    the tree itself. `_off` is subtracted from every mesh, so a first root
    whose own geometry dips below grade makes `rz` negative and RAISES
    everything — including sticks `wood_debris` placed analytically at exactly
    `ground_z - 0.001`. Both halves are now separated: the drop moves the
    object, the debris keeps the absolute Z it was authored at, and `reseat`
    is what corrects a piece the solver actually misplaced.

    `reseat_first=False` leaves the FIRST root alone during that pass — the
    tree case, where the pose is authored (`tip_tree`'s seat band, the root
    plate's lift) rather than settled. `reseat_freeze` names any OTHER root
    that is authored rather than settled; `vegetation.wind_tree` publishes
    exactly that list as `res["anchored"]`, and the root ball is on it — it is
    centred on `lift = r_plate * 0.5` so that it straddles the tipped trunk's
    open base, which puts it below grade on purpose. See
    `_reseat_roots(freeze=...)`.

    `merge` (agent O): `False`/"off" writes one prim per source mesh, the way
    this always did; `True`/"on" (the default, `BAKE_MERGE`) welds the meshes
    into ONE per material per shading signature — see the MERGE block above.
    `stats_out`, if given, is filled with the counts for a before/after table.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

    # DEDUPE FIRST. Callers assemble the path list from several overlapping
    # sources — the bake launcher passes `statics + loose + info["made"]`, and
    # `info["made"]` already contains every piece the vegetation passes wrote —
    # so most debris arrived here TWICE and was authored twice, once as
    # `log_007` and once as `log_007_1`, in the same place. That silently
    # doubled the geometry of every tree archetype (the previous bake's
    # per-file mesh counts are all even for this reason). Order is preserved
    # because the first root is what `drop_to_ground` measures.
    _seen, _paths = set(), []
    for p in obj_paths:
        k = str(p)
        if k not in _seen:
            _seen.add(k)
            _paths.append(k)

    roots = [src_stage.GetPrimAtPath(p) for p in _paths]
    roots = [p for p in roots if p and p.IsValid() and p.IsActive()]
    if not roots:
        return False

    _off = None
    _lift = {}          # root path -> extra Z shift; NEGATIVE means dropped
    if recenter or drop_to_ground or reseat:
        # X and Y only: the Z comes back from `_seat_plan`, which owns every
        # decision about it.
        rx, ry, _ = (recenter or (0.0, 0.0, 0.0))
        # BOTH PURPOSES, to agree with the renderer and with
        # `audit_archetype`. A cache built over `default_` alone misses
        # anything authored `purpose = render`, and a root it cannot measure is
        # a root it silently declines to correct — which audits as floating
        # debris that the bake reported as clean.
        _bc = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        rz, _lift = _seat_plan(_bc, roots, recenter=recenter,
                               drop_to_ground=drop_to_ground, reseat=reseat,
                               reseat_first=reseat_first,
                               reseat_freeze=reseat_freeze)
        _off = Gf.Vec3d(rx, ry, rz)

    out = Usd.Stage.CreateNew(out_path)
    # CARRY THE SOURCE STAGE'S UP AXIS AND SCALE. `CreateNew` defaults to
    # Y-up, while everything baked here is authored Z-up (the bake script sets
    # `UsdGeom.SetStageUpAxis(stage, z)`). Losing it makes every archetype
    # DECLARE Y-up over Z-up geometry, and Kit then applies a 90 deg X
    # correction when the plat references it — houses arrive turned on their
    # side. Nothing downstream re-checks this, and the geometry itself is
    # perfectly fine, so it reads as a layout or yaw bug rather than metadata.
    UsdGeom.SetStageUpAxis(out, UsdGeom.GetStageUpAxis(src_stage))
    UsdGeom.SetStageMetersPerUnit(out, UsdGeom.GetStageMetersPerUnit(src_stage))
    # AN XFORM, NOT A SCOPE. `scene_generator.apply_placements` references
    # an asset onto a TYPELESS holder prim so the asset's own root type wins
    # — and a Scope is not Xformable, so every transform op was skipped and
    # 46 earthquake archetypes composed on top of each other at the origin.
    # `scene_api._ref` never saw this because it defines its holder as an
    # Xform first. An Xform root composes correctly under either.
    UsdGeom.Xform.Define(out, root)
    out.SetDefaultPrim(out.GetPrimAtPath(root))
    UsdGeom.Scope.Define(out, root + "/Looks")

    xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matmap, used = {}, {}

    def unique(parent, name):
        base = parent + "/" + name
        n = used.get(base, 0)
        used[base] = n + 1
        return base if n == 0 else "{0}_{1}".format(base, n)

    # MATERIALS BY FINGERPRINT, NOT BY SOURCE PATH. `matmap` used to key on
    # the source prim's path, so the same chair referenced twenty times by
    # the layout rebuilt its material network twenty times — 334 Materials
    # and 2463 Shaders in `bld_apartment_tall_TILT.usd` for 800 meshes.
    # Keying on the network's own content collapses those, and it is what
    # makes the mesh merge effective (two byte-identical materials at
    # different paths would otherwise be two buckets).
    fpmap = {}                # source path -> fingerprint (avoid rehashing)

    def _cached_material(mp):
        """Rebuild-and-cache ONE ALREADY-RESOLVED material prim `mp` into
        this object's `Looks` scope, keyed by content fingerprint — the
        half of `material_for` below that does not care HOW `mp` was
        found. Shared with the PointInstancer prototype path
        (`_carry_direct_binding`, called from `_copy_prototype_tree`),
        which resolves `mp` from a prototype's DIRECT binding rather than
        `material_for`'s `ComputeBoundMaterial`, so both paths rebuild
        through the SAME cache — a look bound on 9 prototypes, or on a
        prototype and an ordinary mesh, is rebuilt exactly once either way.
        """
        if mp is None:
            return None
        key = mp.GetPath().pathString
        fp = fpmap.get(key)
        if fp is None:
            try:
                fp = _mat_fingerprint(mp)
            except Exception:
                fp = "path:" + key        # never let a hash failure lose a bind
            fpmap[key] = fp
        if fp not in matmap:
            matmap[fp] = _rebuild_material(
                out, unique(root + "/Looks", mp.GetName()), mp)
        return matmap[fp]

    def material_for(prim):
        return _cached_material(_bound_material_prim(prim))

    n_mesh = 0
    object_mats = []          # every out-material path used in this object
    wood_mat = [None]         # a bark/wood/char material, for the fallback bind
    unbound = []              # out Mesh prims that ended up with no material

    def _note(mp, name):
        if mp:
            object_mats.append(mp)
            if wood_mat[0] is None and any(
                    k in name.lower() for k in
                    ("trunk", "bole", "bark", "base", "stem", "wood", "char")):
                wood_mat[0] = mp

    mode = merge_mode(None if merge is None else
                      ("on" if merge is True else
                       ("off" if merge is False else merge)))
    do_merge = mode in ("on", "both")
    buckets = {}              # (mat_or_None, sig) -> [_Bucket, ...]
    n_src_merged = n_src_kept = 0

    def _bucket_for(mat, sig):
        lst = buckets.setdefault((mat, sig), [])
        if not lst or lst[-1].n_faces >= MERGE_MAX_FACES:
            lst.append(_Bucket(mat, sig))
        return lst[-1]

    def _author_one(prim, m, dst_name):
        """The original per-prim path: one Mesh + its subsets."""
        dst = unique(root, dst_name)
        dm = UsdGeom.Mesh.Define(out, dst)
        _copy_attrs_by_value(prim, dm.GetPrim(), skip_xform=True,
                             strip_dead=do_merge)
        dm.AddTransformOp().Set(m)
        bound_here = False
        mp = material_for(prim)
        if mp:
            UsdShade.MaterialBindingAPI(dm.GetPrim()).Bind(
                UsdShade.Material(out.GetPrimAtPath(mp)))
            _note(mp, prim.GetName()); bound_here = True
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            sp = sub.GetPrim()
            ds = UsdGeom.Subset.Define(out, dst + "/" + sp.GetName())
            _copy_attrs_by_value(sp, ds.GetPrim(), skip_xform=False,
                                 strip_dead=do_merge)
            smp = material_for(sp)
            if smp:
                UsdShade.MaterialBindingAPI(ds.GetPrim()).Bind(
                    UsdShade.Material(out.GetPrimAtPath(smp)))
                _note(smp, sp.GetName()); bound_here = True
        if not bound_here:
            unbound.append(dm.GetPrim())

    def _merge_one(prim, m):
        """Split one source mesh's faces across the buckets its materials
        name, and accumulate them in world space. Returns False when the mesh
        is something the merge cannot express, so the caller authors it whole.
        """
        mesh = UsdGeom.Mesh(prim)
        counts = mesh.GetFaceVertexCountsAttr().Get()
        pts = mesh.GetPointsAttr().Get() or []
        if not counts or not len(pts):
            return False
        # A NORMALS ARRAY THAT DOES NOT MATCH ITS DECLARED INTERPOLATION is a
        # malformed mesh; Hydra guesses, and the merge would have to guess the
        # same way. Author it whole and let it keep whatever it renders as.
        na = mesh.GetNormalsAttr()
        if na and na.HasAuthoredValue():
            want = {"faceVarying": len(mesh.GetFaceVertexIndicesAttr().Get() or []),
                    "uniform": len(counts)}.get(
                        mesh.GetNormalsInterpolation(), len(pts))
            if len(na.Get() or []) != want:
                return False
        pvars = [v for v in UsdGeom.PrimvarsAPI(prim).GetPrimvars()
                 if v.HasAuthoredValue()]
        sig = _mesh_signature(prim, mesh, pvars)
        prim_mat = material_for(prim)
        if prim_mat:
            _note(prim_mat, prim.GetName())
        # per-face material: the prim's own binding, overridden by any
        # `materialBind` GeomSubset (agent T's façade/core split)
        subs = UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim))
        face_mat = None
        for sub in subs:
            if sub.GetElementTypeAttr().Get() != UsdGeom.Tokens.face:
                return False              # not expressible: author it whole
            smp = material_for(sub.GetPrim())
            if not smp:
                continue
            _note(smp, sub.GetPrim().GetName())
            if face_mat is None:
                face_mat = [prim_mat] * len(counts)
            for f in (sub.GetIndicesAttr().Get() or []):
                if 0 <= int(f) < len(face_mat):
                    face_mat[int(f)] = smp
        det = Gf.Matrix3d(m.ExtractRotationMatrix()).GetDeterminant()
        flip = det < 0.0
        if face_mat is None:
            b = _bucket_for(prim_mat, sig)
            return _gather(prim, mesh, m, None, b, pvars, flip)
        groups = {}
        for f, mt in enumerate(face_mat):
            groups.setdefault(mt, []).append(f)
        # A MULTI-GROUP MESH MAY NOT PARTIALLY COMMIT AND THEN FALL BACK: the
        # caller's fallback authors the WHOLE prim, so a group that already
        # landed in a bucket would be drawn twice. Failures are absorbed here;
        # only a mesh that committed nothing goes back to the per-prim path.
        ok = False
        failed = []
        for mt, fs in groups.items():
            b = _bucket_for(mt, sig)
            try:
                ok = _gather(prim, mesh, m, fs, b, pvars, flip) or ok
            except Exception as exc:
                failed.append("{0}: {1}".format(mt, exc))
        if failed and not ok:
            return False
        if failed:
            print("[bake] merge LOST {0} face group(s) on {1} — {2}".format(
                len(failed), prim.GetPath(), "; ".join(failed)))
        return ok

    def _author_instancer(prim, m, dst_name):
        """Copy one PointInstancer whole: NEVER through the mesh merge (see
        the PASS 0 / main-loop pruning below — an instancer is already the
        cheap representation the merge exists to approximate for everything
        else), its schema attributes by value, its `prototypes` targets
        copied and remapped, and its own transform baked exactly like every
        Mesh above.

        THE FRAME, AFTER EXPORT: `positions`/`orientations`/`scales`/
        `velocities`/`accelerations`/`angularVelocities` are copied
        UNCHANGED — they stay in the INSTANCER PRIM'S OWN LOCAL FRAME, i.e.
        relative to THIS prim, exactly as authored on the source. Only the
        instancer prim's OWN xformOp (below) is replaced with its baked,
        recentred WORLD matrix. Composing unchanged local instance data
        through that one fresh xform reproduces the original world-space
        instance placements without double-transforming — the identical
        split every Mesh in this file already uses (local points, one
        rebaked world xform), just one level up the schema.
        """
        from pxr import Sdf
        dst = unique(root, dst_name)
        di = UsdGeom.PointInstancer.Define(out, dst)
        # strip_dead=False, UNCONDITIONALLY (not `do_merge`, unlike a Mesh):
        # an instancer's schema attributes (protoIndices, positions, ...)
        # must never be run through the merge's dead-physics-attribute /
        # schema-fallback stripping, whatever BAKE_MERGE is set to.
        _copy_attrs_by_value(prim, di.GetPrim(), skip_xform=True,
                             strip_dead=False)
        di.AddTransformOp().Set(m)

        src_rel = UsdGeom.PointInstancer(prim).GetPrototypesRel()
        dst_rel = di.CreatePrototypesRel()   # THE OUTPUT relationship — the
        # one `SetTargets` below must land on. Reading and writing the same
        # relationship object here would silently re-point the SOURCE
        # stage's `prototypes` at the archetype's own (not-yet-existing)
        # paths instead of authoring anything on the export.
        proto_root = prim.GetPath()
        new_targets = []
        for t in src_rel.GetTargets():
            tprim = src_stage.GetPrimAtPath(t)
            if not (tprim and tprim.IsValid()):
                continue
            if t.HasPrefix(proto_root):
                # THE COMMON CASE: the prototype is a child of the instancer
                # (round-4 rubble nests them at `<instancer>/Prototypes/
                # <name>`) — mirror that same relative path under the new
                # instancer so it lands at the same, discoverable place.
                dst_p = dst + "/" + t.MakeRelativePath(proto_root).pathString
            else:
                # NOT a child of this instancer — either elsewhere under the
                # same exported object, or outside every exported path
                # entirely. Both are folded into ONE destination convention:
                # copied under THIS instancer's own Prototypes scope, since
                # nothing downstream needs the original absolute location —
                # only the `prototypes` relationship has to resolve.
                dst_p = unique(dst + "/Prototypes", _safe_name(tprim.GetName()))
            _copy_prototype_tree(out, dst_p, tprim, strip_dead=False,
                                 resolve_material=_cached_material)
            new_targets.append(Sdf.Path(dst_p))
        if new_targets:
            dst_rel.SetTargets(new_targets)
        return di.GetPrim()

    # POINT INSTANCERS ARE GATHERED BY RELATIONSHIP, NOT BY POSITION, and
    # BEFORE either pass below. `prototypes` targets do not have to be
    # children of their instancer — the schema allows anything — so a target
    # sorting earlier in the tree than its own instancer would otherwise be
    # authored TWICE: once folded into the instancer by `_author_instancer`,
    # and once again when the ordinary per-mesh walk reaches it on its own.
    proto_skip = set()
    for r in roots:
        for prim in Usd.PrimRange(r):
            if not prim.IsA(UsdGeom.PointInstancer):
                continue
            for t in UsdGeom.PointInstancer(prim).GetPrototypesRel().GetTargets():
                proto_skip.add(t.pathString)

    def _under_skip(path_str):
        return any(path_str == s or path_str.startswith(s + "/")
                  for s in proto_skip)

    # PASS 0: how many times does each piece of LOCAL geometry occur? Anything
    # that occurs more than once is a placed asset whose points crate already
    # stores once, so it keeps its own prim (see MERGE_REPEAT_MIN).
    gkey, repeats = {}, {}
    n_repeat_kept = 0
    if do_merge and MERGE_REPEAT_MIN > 0:
        for r in roots:
            it0 = iter(Usd.PrimRange(r))
            for prim in it0:
                # PointInstancers (and any prototype living outside its own
                # instancer's subtree) are never candidates for the repeat
                # count OR the merge itself — see the main loop below.
                if prim.IsA(UsdGeom.PointInstancer):
                    it0.PruneChildren()
                    continue
                if _under_skip(prim.GetPath().pathString):
                    it0.PruneChildren()
                    continue
                if not prim.IsA(UsdGeom.Mesh):
                    continue
                try:
                    k = _geom_key(UsdGeom.Mesh(prim))
                except Exception:
                    k = None
                gkey[prim.GetPath().pathString] = k
                if k is not None:
                    repeats[k] = repeats.get(k, 0) + 1

    n_instancers = n_instances = 0
    for r in roots:
        rname = r.GetName()
        # Per-root rescue for a piece the solver pushed under the floor; zero
        # for everything else, which therefore keeps its settled pose exactly.
        rdz = Gf.Vec3d(0.0, 0.0, _lift.get(r.GetPath().pathString, 0.0))
        it = iter(Usd.PrimRange(r))
        for prim in it:
            if prim.IsA(UsdGeom.PointInstancer):
                # NEVER through the merge (requirement: an instancer and its
                # prototypes are skipped entirely, not gathered into a
                # bucket) — authored directly, and its whole subtree pruned
                # from this walk so no prototype is ALSO authored as a loose
                # top-level mesh.
                m = Gf.Matrix4d(xf_cache.GetLocalToWorldTransform(prim))
                if _off is not None:
                    m = m.SetTranslateOnly(m.ExtractTranslation() - _off + rdz)
                _author_instancer(prim, m, rname if prim == r else prim.GetName())
                idx = UsdGeom.PointInstancer(prim).GetProtoIndicesAttr().Get()
                n_instancers += 1
                n_instances += len(idx) if idx else 0
                it.PruneChildren()
                continue
            if _under_skip(prim.GetPath().pathString):
                # a prototype target that lives OUTSIDE its instancer's own
                # subtree: already folded into that instancer's copy above.
                it.PruneChildren()
                continue
            if not prim.IsA(UsdGeom.Mesh):
                continue
            m = Gf.Matrix4d(xf_cache.GetLocalToWorldTransform(prim))
            if _off is not None:                          # recentre to origin
                m = m.SetTranslateOnly(
                    m.ExtractTranslation() - _off + rdz)
            done = False
            k = gkey.get(prim.GetPath().pathString)
            repeated = (k is not None
                        and repeats.get(k, 0) >= MERGE_REPEAT_MIN)
            if repeated:
                n_repeat_kept += 1
            if do_merge and not repeated:
                try:
                    done = _merge_one(prim, m)
                except Exception as exc:
                    # NEVER LOSE GEOMETRY TO THE OPTIMISER. Anything the merge
                    # cannot express falls back to the prim-per-mesh path, and
                    # says so once.
                    print("[bake] merge fell back on {0}: {1}".format(
                        prim.GetPath(), exc))
                    done = False
            if done:
                n_src_merged += 1
            else:
                _author_one(prim, m, rname if prim == r else prim.GetName())
                n_src_kept += 1
            n_mesh += 1

    n_merged_prims = 0
    for (mat, _sig), lst in buckets.items():
        for k, b in enumerate(lst):
            nm = _safe_name(
                (out.GetPrimAtPath(mat).GetName() if mat else "unbound"))
            p = _write_bucket(out, unique(root, "merged_" + nm), b)
            if p is None:
                continue
            n_merged_prims += 1
            if mat:
                UsdShade.MaterialBindingAPI(p).Bind(
                    UsdShade.Material(out.GetPrimAtPath(mat)))
            else:
                unbound.append(p)

    # UNBOUND MESHES TAKE THE OBJECT'S OWN BARK. Some kit prototypes ship with
    # NO material binding at all — Black_Oak's woody branchlet prototype is the
    # known one (`bind_bark` repairs the trunk-level meshes but not the
    # instancer prototypes). They render grey. Bind them to a bark/wood/char
    # material already in this object, or any material as a last resort, which
    # is exactly what `vegetation.bind_bark` does at build time.
    fallback = wood_mat[0] or (object_mats[0] if object_mats else None)
    if fallback:
        for prim in unbound:
            UsdShade.MaterialBindingAPI(prim).Bind(
                UsdShade.Material(out.GetPrimAtPath(fallback)))

    out.GetRootLayer().Save()
    if stats_out is not None:
        stats_out.update(seat_rz=float(rz if _off is not None else 0.0),
                         seat_lifted=len(_lift),
                         seat_lift_max=(max((abs(v) for v in _lift.values()),
                                            default=0.0)),
                         src_meshes=n_mesh, merged_src=n_src_merged,
                         kept_src=n_src_kept, merged_prims=n_merged_prims,
                         repeat_kept=n_repeat_kept,
                         materials=len(matmap), mode=mode,
                         instancers=n_instancers, instances=n_instances,
                         out_prims=sum(1 for _ in out.Traverse()))
    return n_mesh > 0 or n_instancers > 0


def validate(out_path, root="/Baked"):
    """(meshes, ok, miss) — material-binding health, unchanged shape for the
    one existing caller (`tools/_o_remerge.py`). PointInstancers are counted
    and printed separately (not folded into `meshes`/`ok`/`miss`, and their
    prototype subtrees are pruned from this walk) — this function has never
    audited material binding on instanced/referenced content, and folding a
    handful of typically-unbound prototypes into the same ratio would read as
    a regression in an archetype that has none."""
    from pxr import Usd, UsdGeom, UsdShade

    st = Usd.Stage.Open(out_path)
    if st is None:
        return (0, 0, 0)
    meshes = ok = miss = 0
    n_instancers = n_instances = 0
    it = iter(Usd.PrimRange(st.GetPrimAtPath(root)))
    for prim in it:
        if prim.IsA(UsdGeom.PointInstancer):
            n_instancers += 1
            idx = UsdGeom.PointInstancer(prim).GetProtoIndicesAttr().Get()
            n_instances += len(idx) if idx else 0
            it.PruneChildren()
            continue
        if not prim.IsA(UsdGeom.Mesh):
            continue
        meshes += 1
        targets = [prim] + [s.GetPrim() for s in
                            UsdGeom.Subset.GetAllGeomSubsets(
                                UsdGeom.Imageable(prim))]
        bound = any(
            UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            and UsdShade.MaterialBindingAPI(t).ComputeBoundMaterial()[0]
            .GetPrim().IsValid() for t in targets)
        ok += 1 if bound else 0
        miss += 0 if bound else 1
    if n_instancers:
        print("[bake] validate {0}: {1} PointInstancer(s), {2} instance(s) "
              "total".format(os.path.basename(out_path), n_instancers,
                             n_instances))
    return (meshes, ok, miss)


def write_manifest(path, records):
    with open(path, "w") as fh:
        json.dump(records, fh, indent=1)


# LEVELS WHOSE JOB IS TO BE A LOW PILE. A `pristine` house has a roof with air
# under it and so does a `roof_stripped` one — reading those rows as floating
# debris is how an investigation nearly chased a non-bug. These are the only
# rows where "nothing underneath" is a defect. Both the wind and the fire
# ladders are here; the names do not collide.
PILE_LEVELS = ("leveled", "swept", "partial_collapse", "fallen", "snapped",
               "burned_out", "rubble", "stump")


def audit_archetype(path, grade=-0.05, air_min=0.10, air_tol=0.10,
                    seat_frac=0.2):
    """What fraction of one exported archetype's meshes has nothing under it.

    THE MEASUREMENT THE COMPLAINT IS ABOUT, produced by the bake that caused
    it rather than by a probe run afterwards. Per mesh, from the exported USD
    (so this is what actually ships, `bake._reseat_roots` included):

      * SUNK — world bbox min-z below `grade`. Material the solver pushed
        through the ground.
      * AIRBORNE — sits more than `air_min` up and no other mesh both
        overlaps it in plan by at least `seat_frac` of the smaller of the two
        footprints AND has its top inside [z0 - air_tol, z1].

    The support test is `bake._reseat_roots` v3, and the two rules matter for
    opposite reasons. Requiring the support's top to land IN OUR VERTICAL SPAN
    is what stops a 2.5 m wall stub "supporting" everything inside its plan
    footprint at every height; requiring a SEAT rather than a corner clip is
    what stops a passing overlap counting. Measured against the SMALLER of the
    two areas so a stub can still hold up a sheet — that is how a wall holds
    up a roof.

    Axis-aligned boxes still over-count support (a mesh can be inside another
    mesh's box and touch nothing), so every number here is a LOWER bound on
    what is really floating.
    """
    import numpy as np
    from pxr import Usd, UsdGeom

    st = Usd.Stage.Open(path)
    if not st:
        return None
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    # INSTANCE PROXIES INCLUDED, or an archetype that ships its debris as
    # instances audits as empty and reports a clean bill of health.
    try:
        walk = Usd.PrimRange.Stage(st, Usd.TraverseInstanceProxies())
    except Exception:
        walk = st.Traverse()
    box = []
    for prim in walk:
        if not prim.IsA(UsdGeom.Mesh) or not prim.IsActive():
            continue
        try:
            r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
        except Exception:
            continue
        if r.IsEmpty():
            continue
        mn, mx = r.GetMin(), r.GetMax()
        box.append((mn[0], mn[1], mn[2], mx[0], mx[1], mx[2]))
    if not box:
        return None
    return _air_and_sunk(np.asarray(box, dtype=float), grade=grade,
                         air_min=air_min, air_tol=air_tol,
                         seat_frac=seat_frac)


def _air_and_sunk(a, grade=-0.05, air_min=0.10, air_tol=0.10, seat_frac=0.2):
    """The measurement itself, over an (N, 6) array of x0,y0,z0,x1,y1,z1.

    Split out from `audit_archetype` so the support test can be exercised on
    the host with hand-built boxes and no Isaac — it is the part that has been
    wrong twice (see the docstring above) and it is worth pinning.
    """
    import numpy as np

    if not len(a):
        return None
    x0, y0, z0, x1, y1, z1 = a.T
    area = np.maximum(x1 - x0, 1e-6) * np.maximum(y1 - y0, 1e-6)
    air = 0
    for i in range(len(a)):
        if z0[i] <= air_min:
            continue
        ox = np.minimum(x1, x1[i]) - np.maximum(x0, x0[i])
        oy = np.minimum(y1, y1[i]) - np.maximum(y0, y0[i])
        ov = np.maximum(ox, 0.0) * np.maximum(oy, 0.0)
        sup = (ov >= seat_frac * np.minimum(area, area[i])) \
            & (z1 >= z0[i] - air_tol) & (z1 <= z1[i])
        sup[i] = False
        if not sup.any():
            air += 1
    return dict(meshes=int(len(a)), airborne=air,
                sunk=int((z0 < grade).sum()), z_min=float(z0.min()),
                air_max=float(z0[z0 > air_min].max()) if (z0 > air_min).any()
                else 0.0)


def read_manifest(path):
    with open(path) as fh:
        return json.load(fh)
