#!/usr/bin/env python3
"""Repair known material defects in already-frozen Nucleus dataset cells.

Kit-written crates in the frozen dataset are intentionally never re-saved:
some contain vendor ``assetInfo`` values that this USD build can read but
cannot serialise.  Instead, this tool copies the immutable original to a
permanent Nucleus payload and replaces the canonical URL with a tiny layer
that sublayers that payload and authors only material overrides.

The replacement is transactional at the scene level: a staging wrapper must
cold-open, retain the exact mesh count, and pass the material audit before the
canonical URL is changed.  The payload is also the rollback copy and is never
deleted.  No local asset path is authored into the wrapper.
"""
import argparse
import gc
import hashlib
import json
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import omni.client
from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdShade

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scene_gen.disaster import freeze, material_audit, material_repair
from scene_gen.tools.dataset_upload import _iter_remote


DEFAULT_DATASET = (
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "final_disaster_dataset")
DEFAULT_PAYLOADS = (
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "final_disaster_dataset_payloads/material_repair_v1")
REPAIR_VERSION = 1
MARKER_VERSION = "scene_gen_material_repair_version"
MARKER_PAYLOAD = "scene_gen_material_repair_payload"
MARKER_SOURCE = "scene_gen_material_repair_source"
MARKER_SHADOWED = "scene_gen_material_repair_shadowed_local_assets"


def _ok(result):
    return result == omni.client.Result.OK


def _shadowed_paths(value):
    """Decode the JSON-string form accepted by USDA customLayerData."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except (TypeError, ValueError):
            value = [value]
    return [str(path) for path in (value or ())]


def _stat(url):
    result, entry = omni.client.stat(url)
    return (int(entry.size) if _ok(result) else None)


def _mesh_count(stage):
    return sum(prim.IsA(UsdGeom.Mesh) for prim in
               Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))


def _sky_count(stage):
    return sum(prim.IsA(UsdLux.DomeLight) or prim.IsA(UsdLux.DistantLight)
               for prim in stage.Traverse())


def _portability_report(stage, shadowed=()):
    """Return the strict defects a Nucleus-only wrapper must not contain."""
    shadowed = set(str(path) for path in shadowed)
    local_paths = set()
    shadowed_paths = set()
    runtime_assets = set()
    missing_assets = set()
    cross_scope = []
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            raw = getattr(value, "path", "") or ""
            resolved = getattr(value, "resolvedPath", "") or ""
            if not raw or raw.startswith("/Game/"):
                continue
            if attr.GetName() == "info:unreal:sourceAsset" and not resolved:
                continue
            if raw.startswith(material_repair.RUNTIME_ASSET_PREFIXES):
                runtime_assets.add(raw)
                continue
            if os.path.isabs(raw) and not raw.startswith("omniverse://"):
                (shadowed_paths if raw in shadowed else local_paths).add(raw)
            elif not resolved and (raw.startswith("omniverse://") or
                                   "/" in raw):
                missing_assets.add(raw)

    world = stage.GetPrimAtPath("/World")
    if world and world.IsValid():
        for scope in world.GetChildren():
            if scope.GetName() in ("PhysicsScene", "stage"):
                continue
            root = scope.GetPath().pathString
            for child in scope.GetChildren():
                material, _ = UsdShade.MaterialBindingAPI(
                    child).ComputeBoundMaterial(
                        materialPurpose=UsdShade.Tokens.full)
                if material and material.GetPrim().IsValid():
                    target = material.GetPrim().GetPath().pathString
                    if not target.startswith(root + "/"):
                        cross_scope.append("{0} -> {1}".format(
                            child.GetPath(), target))
    return {"build_local": sorted(local_paths),
            "shadowed_build_local": sorted(shadowed_paths),
            "runtime_builtin_assets": sorted(runtime_assets),
            "missing_assets": sorted(missing_assets),
            "cross_scope_bindings": cross_scope}


def _portability_failed(report):
    return bool(report["build_local"] or report["missing_assets"] or
                report["cross_scope_bindings"])


def _upload_collected(stage, work_dir, dependency_root):
    """Publish make_portable's local fallbacks and author Nucleus URLs."""
    collected = {}
    prefix = "Materials/"
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            raw = getattr(value, "path", "") or ""
            if not raw.startswith(prefix):
                continue
            local = os.path.join(work_dir, *raw.split("/"))
            if not os.path.isfile(local):
                raise RuntimeError("collected dependency missing locally: " +
                                   local)
            target = dependency_root.rstrip("/") + "/" + raw
            if target not in collected:
                result = omni.client.copy(
                    local, target,
                    behavior=omni.client.CopyBehavior.OVERWRITE)
                if not _ok(result) or _stat(target) != os.path.getsize(local):
                    raise RuntimeError("dependency upload failed: {0} [{1}]"
                                       .format(target, result))
                collected[target] = os.path.getsize(local)
            attr.Set(Sdf.AssetPath(target))
    return {"files": len(collected), "bytes": sum(collected.values()),
            "urls": sorted(collected)}


def _wrapper_metadata(layer, relative, payload):
    data = dict(layer.customLayerData or {})
    data.update({MARKER_VERSION: REPAIR_VERSION,
                 MARKER_PAYLOAD: payload,
                 MARKER_SOURCE: relative})
    layer.customLayerData = data


def _already_repaired(layer):
    data = dict(layer.customLayerData or {})
    return (int(data.get(MARKER_VERSION, 0) or 0) >= REPAIR_VERSION
            and str(data.get(MARKER_PAYLOAD, "")).startswith("omniverse://"))


def _build_wrapper(payload, relative, output, dependency_root,
                   max_examples=25):
    """Build and validate one new wrapper locally; never modify payload."""
    source_layer = Sdf.Layer.FindOrOpen(payload)
    if source_layer is None:
        raise RuntimeError("could not open immutable payload: " + payload)
    layer = Sdf.Layer.CreateNew(output)
    if layer is None:
        raise RuntimeError("could not create wrapper: " + output)
    layer.subLayerPaths = [payload]
    layer.defaultPrim = source_layer.defaultPrim
    _wrapper_metadata(layer, relative, payload)

    stage = Usd.Stage.Open(layer)
    if stage is None:
        raise RuntimeError("could not compose payload through wrapper")
    stage.SetEditTarget(layer)
    before_meshes = _mesh_count(stage)
    before_default = str(stage.GetDefaultPrim().GetPath()) \
        if stage.GetDefaultPrim() else ""
    portable = freeze.make_portable(
        stage, out_dir=os.path.dirname(output), collect_bake_local=True,
        # Existing prototype geometry stays instanced.  The general material
        # collection override below repairs its read-only shader paths once
        # per look without expanding thousands of trees/buildings.
        deinstance_offenders=False, waive_above_instances=None)
    repairs = portable["material_repairs"]
    if repairs["total_unresolved_known"]:
        raise RuntimeError("known material repair remained unresolved: " +
                           repr(repairs))
    if repairs["total_repaired"] <= 0:
        raise RuntimeError("source failed its audit but matched no known repair")

    mirror_cache = {}

    def resolve_mirror(path):
        # Core MDL modules are addressed by module name and resolved through
        # Kit's standard MDL search path on every consumer.  An absolute
        # build-container spelling is neither needed nor portable.
        if path == "/isaac-sim/kit/mdl/core/Base/OmniPBR.mdl":
            return "OmniPBR.mdl"
        if path in mirror_cache:
            return mirror_cache[path]
        target = freeze._nucleus_target(
            path, freeze.ASSET_LOCAL_PREFIX, freeze.ASSET_MIRROR,
            freeze.LOCAL_MIRROR_ROOTS)
        # A 1 km scene can reference a shared material thousands of times.
        # Cache by source path and tolerate a transient Nucleus stat miss;
        # never turn one busy-server response into a false missing-asset gate.
        if target:
            for _attempt in range(3):
                if _stat(target) is not None:
                    mirror_cache[path] = target
                    return target
                time.sleep(0.15)
            # Keep the deterministic mirror URL and let the composed-stage
            # gate decide.  A busy Nucleus server can return transient stat
            # misses while the resolver still opens the same asset moments
            # later; the cold gate checks ``resolvedPath`` and cannot pass a
            # genuinely absent target.
            mirror_cache[path] = target
            return target
        mirror_cache[path] = None
        return None

    portable_materials = material_repair.repair_local_material_paths(
        stage, resolve_mirror)
    if portable_materials["unresolved_paths"]:
        raise RuntimeError("local material override unresolved: " +
                           repr(portable_materials["unresolved_paths"]))
    # A portable clone for an independently composed overlay (burn ground,
    # flood water, etc.) initially lives in the shared repair scope.  Move the
    # final rewritten material back inside that overlay after the local-path
    # pass; otherwise the wrapper would merely replace one cross-scope bind
    # with another.
    portable_cross_scope = material_repair.repair_cross_scope(stage)
    if portable_cross_scope["unresolved"]:
        raise RuntimeError("portable cross-scope material unresolved: " +
                           repr(portable_cross_scope))
    data = dict(layer.customLayerData or {})
    # A Python ``list[str]`` in customLayerData serialises to text that this
    # USD build cannot parse back.  JSON text is portable across USDA/USDC and
    # remains human/audit readable.
    data[MARKER_SHADOWED] = json.dumps(
        portable_materials["shadowed_paths"], separators=(",", ":"))
    layer.customLayerData = data
    dependencies = _upload_collected(
        stage, os.path.dirname(output), dependency_root)
    materials = material_audit.audit(stage, max_examples=max_examples)
    if not materials["ok"]:
        raise RuntimeError("unknown material failures remain: " +
                           repr(materials["examples"]))
    if not before_meshes:
        raise RuntimeError("wrapper composed no meshes")
    if not _sky_count(stage):
        raise RuntimeError("wrapper composed no plate-scale sky light")
    portability = _portability_report(
        stage, portable_materials["shadowed_paths"])
    if _portability_failed(portability):
        raise RuntimeError("wrapper remains non-portable: " +
                           repr(portability))
    if not layer.Save():
        raise RuntimeError("could not save local wrapper")
    return {"repairs": repairs, "portable": portable,
            "portable_materials": portable_materials,
            "portable_cross_scope": portable_cross_scope,
            "portability": portability, "dependencies": dependencies,
            "materials": materials,
            "mesh_count": before_meshes, "default_prim": before_default,
            "wrapper_bytes": os.path.getsize(output)}


def _cold_gate(url, expected_meshes, max_examples=25, reload=False):
    # Sdf caches layers by identifier. After the transactional canonical
    # replacement, explicitly reload that cached object or a same-process
    # check could accidentally inspect the pre-swap crate and report a false
    # result.
    layer = Sdf.Layer.FindOrOpen(url)
    if layer is None:
        raise RuntimeError("cold layer open failed: " + url)
    if reload and not layer.Reload():
        raise RuntimeError("cold layer reload failed: " + url)
    stage = Usd.Stage.Open(layer)
    if stage is None:
        raise RuntimeError("cold open failed: " + url)
    meshes = _mesh_count(stage)
    if meshes != expected_meshes:
        raise RuntimeError("geometry changed: expected {0} meshes, got {1}"
                           .format(expected_meshes, meshes))
    if not _sky_count(stage):
        raise RuntimeError("cold wrapper has no plate-scale sky light")
    shadowed = _shadowed_paths(
        dict(layer.customLayerData or {}).get(MARKER_SHADOWED, ()))
    portability = _portability_report(stage, shadowed)
    if _portability_failed(portability):
        raise RuntimeError("cold wrapper is not portable: " +
                           repr(portability))
    materials = material_audit.audit(stage, max_examples=max_examples)
    if not materials["ok"]:
        raise RuntimeError("cold material audit failed: " +
                           repr(materials["examples"]))
    return {"mesh_count": meshes, "materials": materials,
            "portability": portability,
            "default_prim": str(stage.GetDefaultPrim().GetPath())
            if stage.GetDefaultPrim() else ""}


def _upload_json(data, url):
    fd, path = tempfile.mkstemp(prefix="material_repair_", suffix=".json")
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(data, stream, indent=2, sort_keys=True)
            stream.write("\n")
        result = omni.client.copy(
            path, url, behavior=omni.client.CopyBehavior.OVERWRITE)
        if not _ok(result):
            raise RuntimeError("could not upload repair report: {0}"
                               .format(result))
    finally:
        if os.path.exists(path):
            os.unlink(path)


def _candidate_paths(dataset, only):
    return sorted(relative for relative, _size in
                  _iter_remote(dataset, only or None)
                  if relative.lower().endswith((".usd", ".usdc")))


def repair_one(dataset, payload_root, relative, *, apply=False,
               max_examples=25):
    canonical = dataset.rstrip("/") + "/" + relative
    payload = payload_root.rstrip("/") + "/" + relative
    source_layer = Sdf.Layer.FindOrOpen(canonical)
    if source_layer is None:
        raise RuntimeError("could not open canonical scene: " + canonical)
    if _already_repaired(source_layer):
        current = Usd.Stage.Open(source_layer)
        cold = _cold_gate(canonical, _mesh_count(current),
                          max_examples=max_examples, reload=True)
        return {"relative_path": relative, "status": "already_repaired",
                "canonical": canonical, "payload": payload, "cold": cold}

    source_stage = Usd.Stage.Open(source_layer)
    source_audit = material_audit.audit(source_stage,
                                        max_examples=max_examples)
    if source_audit["ok"]:
        return {"relative_path": relative, "status": "already_clean",
                "canonical": canonical, "source_audit": source_audit}
    if not apply:
        return {"relative_path": relative, "status": "would_repair",
                "canonical": canonical, "payload": payload,
                "source_audit": source_audit}

    source_size = _stat(canonical)
    if source_size is None:
        raise RuntimeError("canonical disappeared before copy: " + canonical)
    payload_size = _stat(payload)
    if payload_size is None:
        result = omni.client.copy(
            canonical, payload, behavior=omni.client.CopyBehavior.OVERWRITE)
        if not _ok(result) or _stat(payload) != source_size:
            raise RuntimeError("immutable payload copy failed: {0}"
                               .format(result))
    elif payload_size != source_size:
        raise RuntimeError("refusing to overwrite existing payload with a "
                           "different size: " + payload)

    digest = hashlib.sha256(relative.encode("utf-8")).hexdigest()[:16]
    staging = payload_root.rstrip("/") + "/_staging/" + digest + ".usd"
    dependency_root = (payload_root.rstrip("/") + "/_dependencies/" +
                       digest)
    work_dir = tempfile.mkdtemp(prefix="material_wrapper_")
    local = os.path.join(work_dir, "wrapper.usda")
    swapped = False
    try:
        built = _build_wrapper(payload, relative, local, dependency_root,
                               max_examples=max_examples)
        result = omni.client.copy(
            local, staging, behavior=omni.client.CopyBehavior.OVERWRITE)
        if not _ok(result):
            raise RuntimeError("staging-wrapper upload failed: {0}"
                               .format(result))
        staged = _cold_gate(staging, built["mesh_count"],
                            max_examples=max_examples)

        result = omni.client.copy(
            local, canonical, behavior=omni.client.CopyBehavior.OVERWRITE)
        if not _ok(result):
            raise RuntimeError("canonical-wrapper upload failed: {0}"
                               .format(result))
        swapped = True
        cold = _cold_gate(canonical, built["mesh_count"],
                          max_examples=max_examples, reload=True)
        report = {"relative_path": relative, "status": "repaired",
                  "version": REPAIR_VERSION, "canonical": canonical,
                  "payload": payload, "source_bytes": source_size,
                  "built": built, "staging_gate": staged,
                  "canonical_gate": cold,
                  "completed_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                                 time.gmtime())}
        cell = canonical.rsplit("/", 1)[0]
        _upload_json(report, cell + "/material_repair_report.json")
        return report
    except Exception:
        # Once the permanent payload exists, it is a byte-for-byte rollback
        # source. Restore the canonical only if replacement had begun.
        if swapped:
            result = omni.client.copy(
                payload, canonical,
                behavior=omni.client.CopyBehavior.OVERWRITE)
            if not _ok(result) or _stat(canonical) != source_size:
                raise RuntimeError("repair failed AND canonical rollback failed")
        raise
    finally:
        if os.path.isdir(work_dir):
            shutil.rmtree(work_dir)
        # Staging is disposable; the permanent payload is deliberately kept.
        omni.client.delete(staging)
        gc.collect()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default=DEFAULT_DATASET)
    parser.add_argument("--payload-root", default=DEFAULT_PAYLOADS)
    parser.add_argument("--only", default="")
    parser.add_argument("--max-examples", type=int, default=25)
    parser.add_argument("--apply", action="store_true",
                        help="perform the gated canonical replacement")
    parser.add_argument("--keep-going", action="store_true")
    parser.add_argument("--out", default="")
    args = parser.parse_args(argv)

    omni.client.initialize()
    rows = []
    try:
        paths = _candidate_paths(args.dataset, args.only)
        for index, relative in enumerate(paths, 1):
            print("[material_repair] {0}/{1} {2}".format(
                index, len(paths), relative), flush=True)
            try:
                row = repair_one(
                    args.dataset, args.payload_root, relative,
                    apply=args.apply, max_examples=args.max_examples)
            except Exception as exc:  # noqa: BLE001 - optional batch continue
                row = {"relative_path": relative, "status": "failed",
                       "error": str(exc)}
                rows.append(row)
                print("[material_repair] FAILED {0}: {1}".format(
                    relative, exc), flush=True)
                if not args.keep_going:
                    break
                continue
            rows.append(row)
            print("[material_repair] {0} {1}".format(
                row["status"].upper(), relative), flush=True)
    finally:
        omni.client.shutdown()

    summary = {
        "dataset": args.dataset, "payload_root": args.payload_root,
        "apply": args.apply, "rows": rows,
        "failed": sum(row["status"] == "failed" for row in rows),
        "repaired": sum(row["status"] == "repaired" for row in rows),
        "clean": sum(row["status"] in ("already_clean", "already_repaired")
                     for row in rows),
    }
    if args.out:
        with open(args.out, "w") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
    print("[material_repair] COMPLETE repaired={repaired} clean={clean} "
          "failed={failed}".format(**summary), flush=True)
    return 1 if summary["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
