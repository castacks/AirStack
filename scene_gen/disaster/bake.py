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


def export_object(src_stage, _flat_unused, obj_paths, out_path, root="/Baked",
                  recenter=None):
    """Write one object's meshes + materials to a self-contained USD by value.

    `src_stage` is a stage (typically the Kit-flattened scene) that has the
    object at `obj_paths`. `_flat_unused` is ignored (kept for call
    compatibility). Returns True if any mesh was written.
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

    for r in roots:
        rname = r.GetName()
        for prim in Usd.PrimRange(r):
            if not prim.IsA(UsdGeom.Mesh):
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

    # UNBOUND MESHES TAKE THE OBJECT'S OWN BARK. Some kit prototypes ship with
    # NO material binding at all — Black_Oak's woody branchlet prototype is the
    # known one (`bind_bark` repairs the trunk-level meshes but not the
    # instancer prototypes). They render grey. Bind them to a bark/wood/char
    # material already in this object, or any material as a last resort, which
    # is exactly what `vegetation.bind_bark` does at build time.
    fallback = wood_mat[0] or (object_mats[0] if object_mats else None)
    if fallback:
        for prim in unbound:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                UsdShade.Material(out.GetPrimAtPath(fallback)))

    out.GetRootLayer().Save()
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
