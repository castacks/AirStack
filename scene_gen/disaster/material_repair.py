"""Targeted repairs for known third-party asset material defects.

These are source-asset defects repeated by instancing, not disaster-specific
fallbacks.  The repair is authored as a collection binding above the instance
proxies, so hundreds of affected meshes can be fixed without de-instancing or
duplicating their geometry.  Correct source materials are cloned by value into
the scene; only the bollard's intrinsically absent emissive material is
synthesised.
"""


REPAIR_SCOPE = "/World/stage/_AssetMaterialRepairs"
COLLECTION_ROOT = "/World/stage"
RUNTIME_ASSET_PREFIXES = ("/isaac-sim/kit/mdl/",)


def _authored_targets(prim):
    out = []
    for rel in prim.GetRelationships():
        if rel.GetName().startswith("material:binding"):
            out.extend(rel.GetTargets())
    return out


def _bound(prim):
    from pxr import UsdShade
    try:
        return UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
            materialPurpose=UsdShade.Tokens.full)[0]
    except Exception:  # pragma: no cover - malformed vendor assets
        return None


def _usable(material):
    from pxr import UsdShade
    return bool(material and material.GetPrim().IsValid()
                and material.GetPrim().IsA(UsdShade.Material)
                and any(out.HasConnectedSource()
                        for out in material.GetSurfaceOutputs()))


def _clone_material(stage, source, name, *, scope=REPAIR_SCOPE):
    """Clone one composed material spec into the current edit layer."""
    import os
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    UsdGeom.Scope.Define(stage, scope)
    dst = Sdf.Path(scope).AppendChild(name)
    existing = UsdShade.Material(stage.GetPrimAtPath(dst))
    if _usable(existing):
        return existing
    if not _usable(source):
        return None
    dst_layer = stage.GetEditTarget().GetLayer()
    source_root = source.GetPrim().GetPath()

    # CopySpec preserves a relative spelling but changes its anchoring layer.
    # A vendor material's ``./materials/TreeBark_07.mdl`` therefore resolved
    # beside the generated review scene after cloning instead of beside the
    # source tree asset. Capture the source resolution before copying and
    # author the resolved path on the clone; the subsequent portability pass
    # can then map/collect it like every other real local dependency.
    reanchored_assets = {}
    for source_prim in Usd.PrimRange(source.GetPrim()):
        relative = source_prim.GetPath().MakeRelativePath(source_root)
        for attr in source_prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            raw = getattr(value, "path", "") or ""
            if (not raw or os.path.isabs(raw)
                    or raw.startswith(("omniverse://", "http://", "https://"))
                    or "/" not in raw):
                continue
            resolved = getattr(value, "resolvedPath", "") or ""
            if not resolved:
                for spec in attr.GetPropertyStack():
                    authored = getattr(spec, "default", None)
                    if getattr(authored, "path", None) != raw:
                        continue
                    resolved = Sdf.ComputeAssetPathRelativeToLayer(
                        spec.layer, raw)
                    if resolved:
                        break
            if resolved:
                reanchored_assets[(relative.pathString,
                                   attr.GetName())] = resolved

    def _finished_copy():
        copied = UsdShade.Material(stage.GetPrimAtPath(dst))
        if not _usable(copied):
            return None
        # Unreal-exported materials can carry an assetInfo value whose type
        # this USD build can read but cannot serialise.  Sdf.CopySpec raises
        # while unpacking that metadata *after* copying the usable shader
        # network.  Never let the poisoned metadata enter our output layer.
        copied.GetPrim().ClearMetadata("assetInfo")
        for (relative, attr_name), resolved in reanchored_assets.items():
            prim_path = (dst if relative in ("", ".")
                         else dst.AppendPath(relative))
            attr = stage.GetPrimAtPath(prim_path).GetAttribute(attr_name)
            if attr:
                attr.Set(Sdf.AssetPath(resolved))
        return copied

    for spec in source.GetPrim().GetPrimStack():
        try:
            if Sdf.CopySpec(spec.layer, spec.path, dst_layer, dst):
                copied = _finished_copy()
                if copied:
                    return copied
        except Exception:  # pragma: no cover - try the next contributing spec
            # CopySpec is not transactional.  The Kit-crate failure described
            # above leaves a complete material behind before it throws.
            copied = _finished_copy()
            if copied:
                return copied
            continue
    return None


def repair_cross_scope(stage, verbose=True):
    """Move direct-child bindings into their independently composed scope.

    Frozen cells reference ``/World/<overlay>`` scopes independently.  A
    binding from such a scope to a material below ``/World/stage`` is dropped
    when the overlay is composed by itself.  Clone the *composed* source
    material into the overlay's own ``Looks`` scope and rebind the child.  The
    clone reads from the source prim stack, so this also works when the source
    material lives in a weaker Nucleus payload rather than the edit layer.
    """
    import hashlib
    from pxr import UsdGeom, UsdShade

    moved = rebound = unresolved = 0
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        return {"looks_moved": 0, "rebound": 0, "unresolved": 0}

    for scope_prim in world.GetChildren():
        if scope_prim.GetName() in ("PhysicsScene", "stage"):
            continue
        root = scope_prim.GetPath().pathString
        looks_scope = root + "/Looks"
        for child in scope_prim.GetChildren():
            material = _bound(child)
            if not _usable(material):
                continue
            source_path = material.GetPrim().GetPath().pathString
            if source_path.startswith(root + "/"):
                continue
            digest = hashlib.sha1(source_path.encode("utf-8")).hexdigest()[:8]
            name = "{0}_{1}".format(material.GetPrim().GetName(), digest)
            cloned = _clone_material(stage, material, name, scope=looks_scope)
            if not _usable(cloned):
                unresolved += 1
                continue
            moved += 1
            UsdShade.MaterialBindingAPI.Apply(child).Bind(
                cloned, UsdShade.Tokens.strongerThanDescendants)
            rebound += 1

    report = {"looks_moved": moved, "rebound": rebound,
              "unresolved": unresolved}
    if verbose and (moved or unresolved):
        print("[material_repair] cross_scope={0}".format(report), flush=True)
    return report


def _preview_material(stage, name, *, diffuse, roughness,
                      emissive=None, opacity=1.0):
    from pxr import Sdf, UsdGeom, UsdShade

    UsdGeom.Scope.Define(stage, REPAIR_SCOPE)
    path = REPAIR_SCOPE + "/" + name
    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(diffuse)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    if emissive is not None:
        shader.CreateInput("emissiveColor",
                           Sdf.ValueTypeNames.Color3f).Set(emissive)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface")
    return material


def _collection_bind(stage, name, paths, material, *, root_path=COLLECTION_ROOT):
    from pxr import Sdf, Usd, UsdShade

    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0
    collection = Usd.CollectionAPI.Apply(root, name)
    collection.CreateExpansionRuleAttr().Set(Usd.Tokens.explicitOnly)
    unique = sorted({str(path) for path in paths})
    collection.CreateIncludesRel().SetTargets([Sdf.Path(path)
                                               for path in unique])
    UsdShade.MaterialBindingAPI.Apply(root).Bind(
        collection, material, name, UsdShade.Tokens.strongerThanDescendants)
    return len(unique)


def _instance_anchor(prim):
    """Return the editable instance root above an instance-proxy prim."""
    if not prim.IsInstanceProxy():
        return None
    cursor = prim
    parent = cursor.GetParent()
    while parent and parent.IsValid() and parent.IsInstanceProxy():
        cursor = parent
        parent = cursor.GetParent()
    return parent if parent and parent.IsValid() and parent.IsInstance() \
        else None


def _direct_bind(stage, path, material):
    """Author one stronger binding on an ordinary (non-proxy) prim."""
    from pxr import UsdShade

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid() or prim.IsInstanceProxy():
        return 0
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.strongerThanDescendants)
    return 1


def repair_local_material_paths(stage, resolver, verbose=True,
                                scope=REPAIR_SCOPE + "/PortableLooks",
                                collection_root=COLLECTION_ROOT):
    """Override instance-proxy materials that retain local texture paths.

    ``make_portable`` can edit ordinary shaders after targeted de-instancing,
    but a small number of nested vendor prototypes may remain read-only.  A
    collection binding is both smaller and safer than expanding their shared
    geometry: clone each affected composed material once, rewrite only its
    Asset inputs through ``resolver(path)``, then bind every affected gprim to
    that clone.  An unresolved path is never hidden; its group is left alone
    so the portability gate still fails.
    """
    import hashlib
    import os
    from pxr import Sdf, Usd, UsdShade

    material_local = {}
    material_objects = {}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        # MDL vendor assets are inconsistent: some author texture inputs on a
        # Shader, others directly on the Material prim.  Inspect every prim
        # below a Material instead of silently missing the latter form.
        paths = []
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            raw = getattr(value, "path", "") or ""
            if (raw and not raw.startswith(("/Game/", "omniverse://",
                                            "http://", "https://"))
                    and not raw.startswith(RUNTIME_ASSET_PREFIXES)
                    and os.path.isabs(raw)):
                paths.append(raw)
        if not paths:
            continue
        owner = prim
        while owner and owner.IsValid() and not owner.IsPseudoRoot():
            if owner.IsA(UsdShade.Material):
                key = owner.GetPath().pathString
                material_local.setdefault(key, set()).update(paths)
                material_objects[key] = UsdShade.Material(owner)
                break
            owner = owner.GetParent()

    groups = {}
    for path, local_paths in material_local.items():
        material = material_objects[path]
        prim = material.GetPrim()
        prototype_prim = prim.GetPrimInPrototype() if prim.IsInstanceProxy() \
            else None
        identity = (prototype_prim.GetPath().pathString
                    if prototype_prim else path)
        key = (identity, tuple(sorted(local_paths)))
        groups.setdefault(key, {"source": material, "material_paths": set(),
                                "local_paths": set()})
        groups[key]["material_paths"].add(path)
        groups[key]["local_paths"].update(local_paths)

    affected_paths = {path: group_key for group_key, group in groups.items()
                      for path in group["material_paths"]}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        # Match the authored binding site directly.  Calling
        # ComputeBoundMaterial on every one of 150k instance-proxy meshes is
        # both unnecessary and, for old vendor assets lacking an applied API
        # schema, emits one USD warning per prim.  Direct and inherited binds
        # are both covered: for an inherited bind the binding site is the
        # ancestor Xform itself, which is exactly where the stronger override
        # belongs.  Collection-authored source binds are deliberately not
        # widened to a direct bind; if one appears it remains unresolved.
        for relationship in prim.GetRelationships():
            name = relationship.GetName()
            if not name.startswith("material:binding"):
                continue
            for target in relationship.GetTargets():
                group_key = affected_paths.get(target.pathString)
                if group_key is not None:
                    if ":collection:" in name:
                        groups[group_key]["collection_bound"] = True
                    else:
                        anchor = _instance_anchor(prim)
                        groups[group_key].setdefault("binding_sites", set()).add(
                            (prim.GetPath().pathString,
                             anchor.GetPath().pathString if anchor else ""))

    report = {"materials_found": len(groups), "materials_overridden": 0,
              "gprims_rebound": 0, "unresolved_paths": [],
              "direct_bindings": 0, "instance_collections": 0,
              "shadowed_paths": [], "unused_paths": []}
    for index, (group_key, group) in enumerate(sorted(
            groups.items(), key=lambda item: repr(item[0]))):
        resolved = {path: resolver(path) for path in group["local_paths"]}
        missing = sorted(path for path, target in resolved.items()
                         if not target)
        if missing:
            report["unresolved_paths"].extend(missing)
            continue
        digest = hashlib.sha1(repr(group_key).encode("utf-8")).hexdigest()[:10]
        cloned = _clone_material(
            stage, group["source"], "Portable_{0}_{1}".format(index, digest),
            scope=scope)
        if not _usable(cloned):
            report["unresolved_paths"].extend(sorted(group["local_paths"]))
            continue
        rewritten = set()
        for prim in Usd.PrimRange(cloned.GetPrim()):
            for attr in prim.GetAttributes():
                if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                    continue
                value = attr.Get()
                raw = getattr(value, "path", "") or ""
                target = resolved.get(raw)
                if target:
                    attr.Set(Sdf.AssetPath(target))
                    rewritten.add(raw)
        if rewritten != set(group["local_paths"]):
            report["unresolved_paths"].extend(sorted(
                set(group["local_paths"]) - rewritten))
            continue
        sites = sorted(group.get("binding_sites", ()))
        direct = [path for path, anchor in sites if not anchor]
        by_instance = {}
        for path, anchor in sites:
            if anchor:
                by_instance.setdefault(anchor, []).append(path)
        count = 0
        for path in direct:
            rebound = _direct_bind(stage, path, cloned)
            count += rebound
            report["direct_bindings"] += rebound
        for instance_index, (anchor, paths) in enumerate(
                sorted(by_instance.items())):
            rebound = _collection_bind(
                stage,
                "portableMaterial_{0}_{1}_{2}".format(
                    index, instance_index, digest),
                paths, cloned, root_path=anchor)
            count += rebound
            if rebound:
                report["instance_collections"] += 1
        if not count:
            if group.get("collection_bound"):
                # Never widen an existing collection binding into a direct
                # subtree bind; its membership needs a dedicated clone of the
                # original collection and remains a hard gate for now.
                report["unresolved_paths"].extend(
                    sorted(group["local_paths"]))
            else:
                # A material prim can be shipped in an asset's Looks library
                # without any binding targeting it.  Its path is inert: no
                # renderer can request it, so record it separately and allow
                # the composed dependency gate to ignore it.
                report["unused_paths"].extend(sorted(group["local_paths"]))
                report["shadowed_paths"].extend(sorted(group["local_paths"]))
            continue
        report["materials_overridden"] += 1
        report["gprims_rebound"] += count
        report["shadowed_paths"].extend(sorted(group["local_paths"]))

    report["unresolved_paths"] = sorted(set(report["unresolved_paths"]))
    report["shadowed_paths"] = sorted(set(report["shadowed_paths"]))
    report["unused_paths"] = sorted(set(report["unused_paths"]))
    if verbose and (report["materials_found"] or report["unresolved_paths"]):
        print("[material_repair] portable_materials={0}".format(report),
              flush=True)
    return report


def repair_known(stage, verbose=True):
    """Repair the four measured asset defects; return auditable counts.

    Safe to call repeatedly.  A repair is considered complete only when the
    composed binding resolves to a real Material with a connected surface.
    Unknown failures are deliberately left untouched for the hard audit gate.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    groups = {"aec_glass": [], "black_oak_bark": [],
              "bollard_emissive": [], "factory_section10": []}
    sources = {}
    for prim in list(Usd.PrimRange.Stage(stage,
                                         Usd.TraverseInstanceProxies())):
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Subset)):
            continue
        path = str(prim.GetPath())
        targets = _authored_targets(prim)
        target_strings = [str(target) for target in targets]

        if ("Brownstone02_Instanced" in path and
                any(Sdf.Path(t).name == "Glazed_Glass"
                    for t in target_strings)):
            if _usable(_bound(prim)):
                continue
            groups["aec_glass"].append(path)
            for target in target_strings:
                if Sdf.Path(target).name != "Glazed_Glass":
                    continue
                alternate = target.replace("/Looks_01/Glazed_Glass",
                                           "/Looks/Glazed_Glass")
                candidate = UsdShade.Material(stage.GetPrimAtPath(alternate))
                if _usable(candidate):
                    sources.setdefault("aec_glass", candidate)
                    break
            continue

        if ("Black_Oak_branch1" in path and
                any(Sdf.Path(t).name == "Black_Oak_bark_Mat"
                    for t in target_strings)):
            if _usable(_bound(prim)):
                continue
            groups["black_oak_bark"].append(path)
            for target in target_strings:
                if Sdf.Path(target).name != "Black_Oak_bark_Mat":
                    continue
                alternate = str(Sdf.Path(target).GetParentPath().AppendChild(
                    "TreeBark_7"))
                candidate = UsdShade.Material(stage.GetPrimAtPath(alternate))
                if _usable(candidate):
                    sources.setdefault("black_oak_bark", candidate)
                    break
            continue

        if (prim.GetName() == "g_Bollard_Emissive" and
                any(Sdf.Path(t).name == "Default"
                    for t in target_strings)):
            if _usable(_bound(prim)):
                continue
            groups["bollard_emissive"].append(path)
            continue

        if (prim.IsA(UsdGeom.Subset) and prim.GetName() == "Section10"
                and len(UsdGeom.Subset(prim).GetIndicesAttr().Get() or []) == 64):
            if _usable(_bound(prim)):
                continue
            parent = prim.GetParent()
            sibling = parent.GetChild("Section7") if parent else None
            if (parent and parent.IsA(UsdGeom.Mesh) and sibling
                    and sibling.IsA(UsdGeom.Subset)
                    and len(UsdGeom.Subset(sibling).GetIndicesAttr().Get()
                            or []) == 128):
                candidate = _bound(sibling)
                if _usable(candidate):
                    groups["factory_section10"].append(path)
                    sources.setdefault("factory_section10", candidate)

    materials = {}
    for key, name in (("aec_glass", "AEC_Glazed_Glass"),
                      ("black_oak_bark", "Black_Oak_Bark"),
                      ("factory_section10", "Factory_Corrugated_Metal_B")):
        if groups[key]:
            materials[key] = _clone_material(stage, sources.get(key), name)
    if groups["bollard_emissive"]:
        materials["bollard_emissive"] = _preview_material(
            stage, "Bollard_Emissive", diffuse=(0.16, 0.13, 0.08),
            roughness=.24, emissive=(1.0, .62, .18))

    repaired = {}
    for key, paths in groups.items():
        material = materials.get(key)
        repaired[key] = (_collection_bind(stage, "assetRepair_" + key,
                                          paths, material)
                         if paths and _usable(material) else 0)
    unresolved = {key: len(groups[key]) - repaired[key] for key in groups}
    report = {"repaired": repaired, "unresolved_known": unresolved,
              "total_repaired": sum(repaired.values()),
              "total_unresolved_known": sum(unresolved.values())}
    if verbose and (report["total_repaired"] or
                    report["total_unresolved_known"]):
        print("[material_repair] repaired={0} unresolved_known={1}".format(
            repaired, unresolved), flush=True)
    return report
