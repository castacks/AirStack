#!/usr/bin/env python3
"""Cold-open a published cell at its Nucleus URL and gate portability there."""
import argparse
import json
import os
import tempfile

import omni.client
from pxr import Sdf, Usd, UsdGeom, UsdLux, UsdShade


def _ok(url):
    return omni.client.stat(url)[0] == omni.client.Result.OK


def _listop_items(op):
    out = []
    for name in ("explicitItems", "addedItems", "prependedItems", "appendedItems"):
        out.extend(getattr(op, name, ()) or ())
    return out


def verify(url):
    url = url.rstrip("/")
    result, entries = omni.client.list(url)
    if result != omni.client.Result.OK:
        raise RuntimeError("cannot list published cell: %s [%s]" % (url, result))
    names = {e.relative_path for e in entries}
    usds = sorted(n for n in names if n.lower().endswith((".usd", ".usdc")))
    required = {"GT_people.json", "GT_hints.json", "build_stats.json",
                "freeze_report.json"}
    missing_files = sorted(required - names)
    if len(usds) != 1:
        raise RuntimeError("expected exactly one cell USD at %s; found %s" % (url, usds))
    usd_url = url + "/" + usds[0]
    stage = Usd.Stage.Open(usd_url)
    if stage is None:
        raise RuntimeError("Usd.Stage.Open failed for " + usd_url)

    prims = list(Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()))
    meshes = sum(p.IsA(UsdGeom.Mesh) for p in prims)
    sky = [str(p.GetPath()) for p in stage.Traverse()
           if p.IsA(UsdLux.DomeLight) or p.IsA(UsdLux.DistantLight)]
    # Instance-preserving freezes legitimately retain internal references.
    # External arcs are also allowed when their target is portable and exists
    # on Nucleus; only local or missing targets are defects.
    local_arcs, missing_arcs, checked_arcs = set(), set(), set()
    layer = stage.GetRootLayer()

    def check_arc(path):
        if not path:                 # internal reference to a prototype
            return
        if os.path.isabs(path) and not path.startswith("omniverse://"):
            local_arcs.add(path)
            return
        target = Sdf.ComputeAssetPathRelativeToLayer(layer, path)
        if target.startswith("omniverse://") and target not in checked_arcs:
            checked_arcs.add(target)
            if not _ok(target):
                missing_arcs.add(target)

    for path in layer.subLayerPaths:
        check_arc(path)
    for prim in prims:
        if prim.HasAuthoredReferences():
            for ref in _listop_items(prim.GetMetadata("references")):
                check_arc(ref.assetPath)
        if prim.HasPayload():
            for payload in _listop_items(prim.GetMetadata("payload")):
                check_arc(payload.assetPath)

    local_paths, missing_assets, checked = set(), set(), set()
    for prim in prims:
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            raw = getattr(value, "path", "") or ""
            resolved = getattr(value, "resolvedPath", "") or ""
            if not raw or raw.startswith("/Game/"):
                continue
            if os.path.isabs(raw) and not raw.startswith("omniverse://"):
                local_paths.add(raw)
                continue
            # `resolvedPath` is the USD resolver's answer while the stage is
            # anchored on Nucleus. Do not `stat()` package members such as
            # `foo.usdz[0/texture.png]`: omni.client cannot stat that syntax
            # even though Ar successfully opened it. An empty resolution is
            # the portable failure. Bare MDL module identifiers are searched
            # through Kit's MDL paths and are not file references.
            if resolved:
                if resolved.startswith("omniverse://"):
                    checked.add(resolved)
            elif raw.startswith("omniverse://") or "/" in raw:
                missing_assets.add(raw)

    cross = []
    world = stage.GetPrimAtPath("/World")
    if world and world.IsValid():
        for scope in world.GetChildren():
            if scope.GetName() in ("PhysicsScene", "stage"):
                continue
            root = scope.GetPath().pathString
            for child in scope.GetChildren():
                mat, _ = UsdShade.MaterialBindingAPI(child).ComputeBoundMaterial()
                if mat and mat.GetPrim().IsValid():
                    mp = mat.GetPrim().GetPath().pathString
                    if not mp.startswith(root + "/"):
                        cross.append("%s -> %s" % (child.GetPath(), mp))

    report = {
        "url": usd_url, "opened_from_nucleus": True, "meshes": meshes,
        "sky_lights": sky, "build_local": sorted(local_paths),
        "missing_nucleus_assets": sorted(missing_assets),
        "checked_nucleus_assets": len(checked),
        "cross_scope_bindings": cross, "missing_cell_files": missing_files,
        "checked_nucleus_arcs": len(checked_arcs),
        "local_composition_arcs": sorted(local_arcs),
        "missing_nucleus_arcs": sorted(missing_arcs),
    }
    report["ok"] = bool(meshes and sky and not local_paths and
                        not missing_assets and not cross and not missing_files
                        and not local_arcs and not missing_arcs)
    print(json.dumps(report, indent=2))
    if not report["ok"]:
        raise RuntimeError("published Nucleus cell failed cold portability gate")
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("cell_url")
    args = ap.parse_args()
    report = verify(args.cell_url)
    fd, tmp = tempfile.mkstemp(prefix="nucleus_verification_", suffix=".json")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
        dst = args.cell_url.rstrip("/") + "/nucleus_verification.json"
        result = omni.client.copy(
            tmp, dst, behavior=omni.client.CopyBehavior.OVERWRITE)
        if result != omni.client.Result.OK or not _ok(dst):
            raise RuntimeError("could not publish verification report: %s" % result)
        print("published verification report:", dst)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
