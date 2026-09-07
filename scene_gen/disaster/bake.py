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


def _bound_material_prim(prim):
    from pxr import UsdShade
    m = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
    if m and m.GetPrim().IsValid():
        return m.GetPrim()
    return None


def _copy_attrs_by_value(src_prim, dst_prim, skip_xform):
    """Author every attribute of src onto dst by VALUE. Skips xformOps when
    `skip_xform` (a world matrix is set instead)."""
    from pxr import UsdGeom
    for a in src_prim.GetAttributes():
        name = a.GetName()
        if name in _GEOM_SKIP:
            continue
        if skip_xform and name.startswith("xformOp"):
            continue
        try:
            v = a.Get()
        except Exception:
            v = None
        na = dst_prim.CreateAttribute(a.GetName(), a.GetTypeName(),
                                      custom=a.IsCustom())
        if v is not None:
            na.Set(v)
        interp = a.GetMetadata("interpolation")
        if interp:
            na.SetMetadata("interpolation", interp)


def _portable_asset(ap, out_dir: str):
    """An asset path that still resolves on the OTHER side of the mount, or None.

    `anchor` below prefers `resolvedPath` because a Nucleus-relative texture
    would otherwise re-anchor to the archetype's own directory and drop. That is
    right for Nucleus and WRONG for two cases, both of which bake a path that
    exists only on the machine that did the baking:

    1. **A bare MDL module name must stay bare.** `OmniPBR.mdl` resolves through
       Kit's own MDL search path wherever it runs; `resolvedPath` is whichever
       `isaacsim` package the BAKING INTERPRETER imported from. Baked on the
       host that is `<repo>/.venv/lib/python3.11/site-packages/isaacsim/kit/
       mdl/core/Base/OmniPBR.mdl`, a path with no meaning inside the container —
       Kit mangles it into a module name (`::Z73file_3A::home::...`), fails to
       load it, and every fracture core falls back to the missing-MDL colour.
       Measured 2026-08-28: 113 of 146 archetypes, and a whole city rendering as
       red confetti while its intact shells looked perfect.

    2. **A path inside the repo is written RELATIVE to the output layer.** The
       repo is bind-mounted at `/isaac-sim/AirStack` in the container and at
       `~/coasei/AirStack` on the host, so an absolute host path breaks on one
       side and an absolute container path on the other. A relative path is
       resolved against the layer and means the same thing from both.

    Anything else — a Nucleus URL, a path outside the repo — is left to
    `anchor`.
    """
    # LOCAL, like every other function here: this module has no module-level
    # `pxr` import, because it is also read on the host where Kit's USD build
    # is not on the path until Kit has started. Omitting it here made every
    # DAMAGED archetype fail to export with `NameError: name 'Sdf' is not
    # defined`, while `pristine` exported fine — the three uses below are all
    # on paths only a fracture material reaches (case 1 is the bare
    # `OmniPBR.mdl` every `FractureCore_*` carries), so nothing that skipped
    # the damage pipeline ever touched them.
    from pxr import Sdf

    raw = (ap.path or "").strip()
    if raw and "/" not in raw and "\\" not in raw and raw.lower().endswith(".mdl"):
        return Sdf.AssetPath(raw)

    got = ap.resolvedPath or raw
    if not got or "://" in got:
        return None
    got = os.path.abspath(got)
    # Kit's own MDL library, however it was reached: keep only the module name.
    if got.lower().endswith(".mdl") and (os.sep + "mdl" + os.sep) in got.lower():
        return Sdf.AssetPath(os.path.basename(got))
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # scene_gen
    repo = os.path.dirname(repo)                                         # repo root
    if not got.startswith(repo + os.sep):
        return None
    if not out_dir:
        return None
    try:
        rel = os.path.relpath(got, out_dir)
    except ValueError:
        return None
    return Sdf.AssetPath(rel.replace(os.sep, "/"))


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

    # WHERE THIS ARCHETYPE IS BEING WRITTEN, so a repo-internal texture can be
    # made relative to it. Taken from the stage rather than threaded in as an
    # argument: `_rebuild_material` is called from two places and only one of
    # them knows the path.
    _root_layer = out.GetRootLayer()
    _out_dir = os.path.dirname(_root_layer.realPath or _root_layer.identifier
                               or "")

    def anchor(v, attr=None):
        """Asset paths re-anchored to where they resolve NOW.

        `a.Get()` hands back the AUTHORED string, and for these packs that is
        relative to the source layer — `Textures/Foo_BaseColor.png` sitting
        next to the building on Nucleus. Re-authoring it verbatim into an
        archetype under `assets/archetypes/` silently re-anchors it to THAT
        directory, where no `Textures/` exists, so every texture drops and the
        building renders untextured. `resolvedPath` is the absolute location
        the source resolved to, which stays correct from anywhere.

        AND `resolvedPath` IS EMPTY for these Nucleus-relative paths under
        Kit, so the first library was baked with the relative paths verbatim
        and every archetype-backed building rendered as a black box — the
        UsdUVTexture diffuse `fallback` is (0, 0, 0). When nothing resolved, a
        relative path is anchored against the LAYER that authored it
        (`Sdf.ComputeAssetPathRelativeToLayer`), which is what a relative
        asset path means. Absolute paths and URLs pass through untouched.
        """
        def one(ap):
            portable = _portable_asset(ap, _out_dir)
            if portable is not None:
                return portable
            if ap.resolvedPath:
                return Sdf.AssetPath(ap.resolvedPath)
            raw = ap.path
            if not raw or "://" in raw or raw.startswith(("/", "~")):
                return Sdf.AssetPath(raw)
            stack = (attr.GetPropertyStack(Usd.TimeCode.Default())
                     if attr is not None else [])
            for spec in stack:
                try:
                    full = Sdf.ComputeAssetPathRelativeToLayer(spec.layer, raw)
                except Exception:                           # noqa: BLE001
                    continue
                if full and full != raw:
                    return Sdf.AssetPath(full)
            return Sdf.AssetPath(raw)
        if isinstance(v, Sdf.AssetPath):
            return one(v)
        if isinstance(v, Sdf.AssetPathArray):
            return Sdf.AssetPathArray([one(a) for a in v])
        return v

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
            if v is not None:
                na.Set(anchor(v, a))
            conns = a.GetConnections()
            if conns:
                na.SetConnections([fix(c) for c in conns])
    return dst_path


#: Prim paths with this component are fracture output (`mesh_damage`
#: authors fragments under `<building>/fragments/`), merged on export.
FRAGMENT_SCOPE = "fragments"

#: Kept in step with `mesh_damage.INTERIOR_PREFIX`; imported lazily there
#: because this module must stay importable without `pxr`.
_INTERIOR_PREFIX = "InteriorStructure"


def export_object(src_stage, _flat_unused, obj_paths, out_path, root="/Baked",
                  recenter=None, merge_fragments=True):
    """Write one object's meshes + materials to a self-contained USD by value.

    `src_stage` is a stage (typically the Kit-flattened scene) that has the
    object at `obj_paths`. `_flat_unused` is ignored (kept for call
    compatibility). Returns True if any mesh was written.

    FRAGMENTS ARE MERGED, one mesh per material (`merge_fragments`). A settled
    archetype is static at scene load — its prims never enter the scene's
    settle, and findability keys off the `_archetype` flag, not the mesh
    structure — so ~60 rigid-body-shaped meshes per wreck bought nothing and
    cost twice: the renderer composites and binds every one of them (warm-up
    4.3 s -> 25.8 s on tiny with the same 18 materials) and PhysX cooks a
    collider per mesh on every launch (`settle cook 6.7 s`) for geometry that
    is fixed at bake time. Merged, a wreck is a handful of static trimeshes
    with their colliders authored here, cooked once. Points go to world space
    (re-centred), `st` rides along per vertex, and the retained shell keeps
    its own mesh and subsets exactly as before.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade
    _off = Gf.Vec3d(*recenter) if recenter else None

    roots = [src_stage.GetPrimAtPath(p) for p in obj_paths]
    roots = [p for p in roots if p and p.IsValid() and p.IsActive()]
    if not roots:
        return False

    out = Usd.Stage.CreateNew(out_path)
    # AN XFORM, NOT A SCOPE. `apply_placements` references an archetype onto a
    # typeless holder so the asset's own type wins (a single-Mesh asset must
    # stay a Mesh), and then authors translate/rotate/scale on that holder. A
    # Scope is not Xformable, so a Scope-rooted archetype composed fine and
    # was then SKIPPED by the placement — every archetype-backed building came
    # up as an empty lot with a debris ring around it.
    UsdGeom.Xform.Define(out, root)
    out.SetDefaultPrim(out.GetPrimAtPath(root))
    UsdGeom.Scope.Define(out, root + "/Looks")
    # What the geometry below actually is — the world transform is baked and
    # the source was measured in metres, Z-up. Left unauthored, the layer
    # reports USD's fallbacks (Y-up, centimetres) and anything that measures
    # the archetype from its metadata scales it a hundredfold.
    UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(out, 1.0)

    xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    matmap, used = {}, {}

    def unique(parent, name):
        base = parent + "/" + name
        n = used.get(base, 0)
        used[base] = n + 1
        return base if n == 0 else "{0}_{1}".format(base, n)

    def material_for(prim):
        mp = _bound_material_prim(prim)
        if mp is None:
            return None
        key = mp.GetPath().pathString
        if key not in matmap:
            matmap[key] = _rebuild_material(
                out, unique(root + "/Looks", mp.GetName()), mp)
        return matmap[key]

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

    frags = []                # fragment meshes, merged after the loop
    for r in roots:
        rname = r.GetName()
        for prim in Usd.PrimRange(r):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            if merge_fragments and FRAGMENT_SCOPE in prim.GetPath().pathString.split("/"):
                frags.append(prim)
                continue
            dst = unique(root, rname if prim == r else prim.GetName())
            dm = UsdGeom.Mesh.Define(out, dst)
            _copy_attrs_by_value(prim, dm.GetPrim(), skip_xform=True)
            m = Gf.Matrix4d(xf_cache.GetLocalToWorldTransform(prim))
            if _off is not None:                          # recentre to origin
                m = m.SetTranslateOnly(m.ExtractTranslation() - _off)
            dm.AddTransformOp().Set(m)
            bound_here = False
            mp = material_for(prim)
            if mp:
                UsdShade.MaterialBindingAPI.Apply(dm.GetPrim()).Bind(
                    UsdShade.Material(out.GetPrimAtPath(mp)))
                _note(mp, prim.GetName()); bound_here = True
            for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
                sp = sub.GetPrim()
                ds = UsdGeom.Subset.Define(out, dst + "/" + sp.GetName())
                _copy_attrs_by_value(sp, ds.GetPrim(), skip_xform=False)
                smp = material_for(sp)
                if smp:
                    UsdShade.MaterialBindingAPI.Apply(ds.GetPrim()).Bind(
                        UsdShade.Material(out.GetPrimAtPath(smp)))
                    _note(smp, sp.GetName()); bound_here = True
            if not bound_here:
                unbound.append(dm.GetPrim())
            n_mesh += 1

    if frags:
        n_mesh += _merge_fragments(out, root, frags, xf_cache, _off,
                                   material_for, unique, _note, unbound)

    # UNBOUND MESHES TAKE THE OBJECT'S OWN BARK. Some kit prototypes ship with
    # NO material binding at all — Black_Oak's woody branchlet prototype is the
    # known one (`bind_bark` repairs the trunk-level meshes but not the
    # instancer prototypes). They render grey. Bind them to a bark/wood/char
    # material already in this object, or any material as a last resort, which
    # is exactly what `vegetation.bind_bark` does at build time.
    # NOT THE FRACTURE CORE. `object_mats` is every material this object used,
    # and on a damaged rung the core is in there -- picking it dresses the
    # rubble in the look of the BREAK rather than in the building's own brick,
    # which is the complaint this fallback is supposed to answer. Prefer any
    # source material; fall back to the core only if there is nothing else.
    _outer = [m for m in object_mats
              if not m.rsplit("/", 1)[-1].startswith(
                  ("FractureCore_", "InteriorStructure_"))]
    fallback = wood_mat[0] or (_outer[0] if _outer
                               else (object_mats[0] if object_mats else None))
    if fallback:
        for prim in unbound:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                UsdShade.Material(out.GetPrimAtPath(fallback)))

    out.GetRootLayer().Save()
    return n_mesh > 0


def _merge_fragments(out, root, frags, xf_cache, off, material_for, unique,
                     note, unbound=None) -> int:
    """One `rubble_<material>` mesh per material from *frags*. Returns count."""
    import numpy as np
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

    buckets = {}                  # out material path (or None) -> lists
    for prim in frags:
        mesh = UsdGeom.Mesh(prim)
        pts = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        P = np.asarray(pts, dtype=np.float64)
        m = Gf.Matrix4d(xf_cache.GetLocalToWorldTransform(prim))
        if off is not None:
            m = m.SetTranslateOnly(m.ExtractTranslation() - off)
        M = np.array(m).reshape(4, 4)
        P = np.c_[P, np.ones(len(P))] @ M
        P = P[:, :3]
        counts = np.asarray(counts, dtype=np.int64)
        idx = np.asarray(idx, dtype=np.int64)
        pv = UsdGeom.PrimvarsAPI(prim).GetPrimvar("st")
        st = None
        if pv and pv.GetInterpolation() == UsdGeom.Tokens.vertex:
            v = pv.Get()
            if v is not None and len(v) == len(P):
                st = np.asarray(v, dtype=np.float64)
        # per-face material: the prim's binding, overridden by its subsets
        fmat = np.full(len(counts), -1, dtype=np.int64)
        mats = [material_for(prim)]
        for sub in UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            faces = sub.GetIndicesAttr().Get()
            mp = material_for(sub.GetPrim())
            if faces is None or mp is None:
                continue
            mats.append(mp)
            fmat[np.asarray(faces, dtype=np.int64)] = len(mats) - 1
        fmat[fmat < 0] = 0
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
        for k, mp in enumerate(mats):
            sel = np.nonzero(fmat == k)[0]
            if not len(sel):
                continue
            b = buckets.setdefault(mp, {"P": [], "st": [], "counts": [],
                                        "idx": [], "n": 0})
            fv = np.concatenate([idx[starts[f]:starts[f] + counts[f]]
                                 for f in sel])
            used = np.unique(fv)
            remap = np.empty(len(P), dtype=np.int64)
            remap[used] = np.arange(len(used)) + b["n"]
            b["P"].append(P[used])
            b["st"].append(st[used] if st is not None
                           else np.zeros((len(used), 2)))
            b["counts"].append(counts[sel])
            b["idx"].append(remap[fv])
            b["n"] += len(used)

    n = 0
    for mp, b in buckets.items():
        # A FACE WHOSE MATERIAL DID NOT RESOLVE lands in its own bucket, and
        # that bucket has to reach the caller's fallback or it stays bound to
        # nothing and renders WHITE. The fallback below `_merge_fragments`
        # exists for exactly this and never saw merged rubble, because only the
        # copy path collected into `unbound`. Measured 2026-08-30:
        # `SM_Building_21_partial_collapse` was 50.8% unbound geometry --
        # 2,580,926 of 5,077,472 verts -- and read as a white wreck.
        leaf = mp.rsplit("/", 1)[-1] if mp else "unbound"
        # INTERIOR STRUCTURE IS NOT RUBBLE. `fill_interior` binds its floors
        # and columns to their own material precisely so they survive this
        # merge as a separate mesh, instead of being poured into
        # `rubble_<core>` with the shell's cut faces. Bucketing here is per
        # FACE, so a Voronoi cell holding both facade and floor slab splits
        # correctly down the middle.
        if leaf.startswith(_INTERIOR_PREFIX + "_"):
            name = "interior_" + leaf[len(_INTERIOR_PREFIX) + 1:]
        else:
            name = "rubble_" + leaf
        dm = UsdGeom.Mesh.Define(out, unique(root, name))
        dm.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(
            np.concatenate(b["P"]).astype(np.float32)))
        dm.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
            np.concatenate(b["counts"]).astype(np.int32)))
        dm.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
            np.concatenate(b["idx"]).astype(np.int32)))
        dm.CreateDoubleSidedAttr(True)
        dm.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        if mp is None and unbound is not None:
            unbound.append(dm.GetPrim())
        UsdGeom.PrimvarsAPI(dm).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(
                np.concatenate(b["st"]).astype(np.float32)))
        # A static collider, authored once here rather than cooked from a
        # convex hull per fragment on every launch.
        UsdPhysics.CollisionAPI.Apply(dm.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(dm.GetPrim()) \
            .CreateApproximationAttr().Set(UsdPhysics.Tokens.none)
        if mp:
            UsdShade.MaterialBindingAPI.Apply(dm.GetPrim()).Bind(
                UsdShade.Material(out.GetPrimAtPath(mp)))
            note(mp, name)
        n += 1
    return n


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


def unresolved_textures(out_path, root="/Baked") -> list:
    """Texture inputs in the exported file that resolve to nothing.

    `validate` says a mesh is BOUND; it cannot say the material is TEXTURED,
    and the first library passed it with every map pointing at a `Textures/`
    directory that does not exist beside the archetype. This is the check
    that would have caught it: every `asset`-typed shader input under *root*
    that has a path and no `resolvedPath`, reported so the bake can refuse.
    """
    from pxr import Sdf, Usd, UsdShade

    st = Usd.Stage.Open(out_path)
    if st is None:
        return []
    bad = []
    for prim in Usd.PrimRange(st.GetPrimAtPath(root)):
        if not prim.IsA(UsdShade.Shader):
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            if inp.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            v = inp.Get()
            if isinstance(v, Sdf.AssetPath) and v.path and not v.resolvedPath:
                bad.append(f"{prim.GetPath()}.{inp.GetBaseName()} = {v.path}")
    return bad


def write_manifest(path, records):
    with open(path, "w") as fh:
        json.dump(records, fh, indent=1)


def read_manifest(path):
    with open(path) as fh:
        return json.load(fh)
