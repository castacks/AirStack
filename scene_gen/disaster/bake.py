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


def export_object(src_stage, _flat_unused, obj_paths, out_path, root="/Baked",
                  recenter=None, drop_to_ground=False, merge=None,
                  stats_out=None):
    """Write one object's meshes + materials to a self-contained USD by value.

    `src_stage` is a stage (typically the Kit-flattened scene) that has the
    object at `obj_paths`. `_flat_unused` is ignored (kept for call
    compatibility). `recenter=(x,y,z)` subtracts that offset from every mesh's
    world translation. `drop_to_ground=True` seats the object on z=0 — needed
    for trees, whose fallen boles sink through the harness ground during settle
    (measured to -2.4 m) and end up buried once referenced.
    Returns True if any mesh was written.

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
    _lift = {}          # root path -> extra Z lift, for pieces below ground
    if recenter or drop_to_ground:
        rx, ry, rz = (recenter or (0.0, 0.0, 0.0))
        if drop_to_ground:
            _bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                    [UsdGeom.Tokens.default_])

            def _min_z(prim):
                wr = _bc.ComputeWorldBound(prim).ComputeAlignedRange()
                return None if wr.IsEmpty() else wr.GetMin()[2]

            # The object itself sets the offset. When its own geometry is gone
            # (a `stump`, or a `snap`/`topple` that deactivated the bole) there
            # is nothing to measure and `recenter`'s Z stands — everything was
            # authored against a harness ground at z=0, so that is right.
            z0 = _min_z(roots[0])
            if z0 is not None:
                rz = z0
            for r in roots[1:]:
                zr = _min_z(r)
                if zr is not None and (zr - rz) < -_SINK_TOL_M:
                    _lift[r.GetPath().pathString] = rz - zr
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

    def material_for(prim):
        mp = _bound_material_prim(prim)
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

    # PASS 0: how many times does each piece of LOCAL geometry occur? Anything
    # that occurs more than once is a placed asset whose points crate already
    # stores once, so it keeps its own prim (see MERGE_REPEAT_MIN).
    gkey, repeats = {}, {}
    n_repeat_kept = 0
    if do_merge and MERGE_REPEAT_MIN > 0:
        for r in roots:
            for prim in Usd.PrimRange(r):
                if not prim.IsA(UsdGeom.Mesh):
                    continue
                try:
                    k = _geom_key(UsdGeom.Mesh(prim))
                except Exception:
                    k = None
                gkey[prim.GetPath().pathString] = k
                if k is not None:
                    repeats[k] = repeats.get(k, 0) + 1

    for r in roots:
        rname = r.GetName()
        # Per-root rescue for a piece the solver pushed under the floor; zero
        # for everything else, which therefore keeps its settled pose exactly.
        rdz = Gf.Vec3d(0.0, 0.0, _lift.get(r.GetPath().pathString, 0.0))
        for prim in Usd.PrimRange(r):
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
        stats_out.update(src_meshes=n_mesh, merged_src=n_src_merged,
                         kept_src=n_src_kept, merged_prims=n_merged_prims,
                         repeat_kept=n_repeat_kept,
                         materials=len(matmap), mode=mode,
                         out_prims=sum(1 for _ in out.Traverse()))
    return n_mesh > 0


def validate(out_path, root="/Baked"):
    from pxr import Usd, UsdGeom, UsdShade

    st = Usd.Stage.Open(out_path)
    if st is None:
        return (0, 0, 0)
    meshes = ok = miss = 0
    for prim in Usd.PrimRange(st.GetPrimAtPath(root)):
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
    return (meshes, ok, miss)


def write_manifest(path, records):
    with open(path, "w") as fh:
        json.dump(records, fh, indent=1)


def read_manifest(path):
    with open(path) as fh:
        return json.load(fh)
