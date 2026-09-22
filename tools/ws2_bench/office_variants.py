"""CPU-only Office prop editing. Never imports Isaac or starts a simulator.

Translations use the source stage's world axes, in metres. Only selected, complete
root-child props are edited; structural columns may be copied but never moved.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import random

import yaml
from pxr import Gf, Sdf, Usd, UsdGeom


MOVABLE = ("SM_Plant", "SM_Chair", "SM_Armchair", "SM_TrashCan", "SM_FileCabinet")
COPYABLE = MOVABLE + ("SM_Column",)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vector(value, name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must be a three-element list")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
        raise ValueError(f"{name} must contain finite numbers")
    return [float(x) for x in value]


def bounds(cache, prim):
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        return None
    return [list(box.GetMin()), list(box.GetMax())]


def intersects(a, b, gap=0.0):
    return all(a[0][i] < b[1][i] + gap and b[0][i] < a[1][i] + gap for i in range(3))


def open_source(path):
    stage = Usd.Stage.Open(str(Path(path).resolve()))
    if not stage or not stage.GetDefaultPrim():
        raise ValueError("source USD must have a default prim")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or UsdGeom.GetStageMetersPerUnit(stage) != 1.0:
        raise ValueError("only Z-up, metre-based Office stages are supported")
    root = stage.GetDefaultPrim()
    if UsdGeom.Xformable(root).GetLocalTransformation() != Gf.Matrix4d(1):
        raise ValueError("Office root transform must be identity")
    return stage


def inventory(source):
    stage = open_source(source)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    return [{"path": str(p.GetPath()), "movable": p.GetName().startswith(MOVABLE),
             "bounds_m": bounds(cache, p)}
            for p in stage.GetDefaultPrim().GetChildren()
            if p.GetName().startswith(COPYABLE)]


def build_variant(source, config, output):
    """Author a separate layer and a manifest; refuse overwrites and overlaps.

    AABB rejection is deliberately conservative. It does not prove support,
    navigability or PhysX contacts; those require subsequent simulator validation.
    """
    source, output = Path(source).resolve(), Path(output).resolve()
    manifest_path = output.with_suffix(".manifest.json")
    if output.exists() or manifest_path.exists():
        raise ValueError("output or manifest already exists; choose a new output")
    if output.suffix not in (".usd", ".usda", ".usdc"):
        raise ValueError("output must be a USD file")
    seed = config.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ValueError("seed must be an integer")
    floor = config.get("floor_z_m", 0.0)
    gap = config.get("clearance_m", 0.05)
    vector([floor, gap, 0], "floor and clearance")
    if gap < 0:
        raise ValueError("clearance must be nonnegative")
    operations = config.get("operations")
    if not isinstance(operations, list) or not operations:
        raise ValueError("operations must be a nonempty list")
    protected = config.get("protected_regions_m")
    if not isinstance(protected, list) or not protected:
        raise ValueError("supply protected start/takeoff/goal boxes in Office coordinates")
    for box in protected:
        if not isinstance(box, list) or len(box) != 2:
            raise ValueError("protected region must be [min_xyz, max_xyz]")
        lo, hi = vector(box[0], "region min"), vector(box[1], "region max")
        if any(a >= b for a, b in zip(lo, hi)):
            raise ValueError("protected region min must be below max")

    original = open_source(source)
    root = original.GetDefaultPrim()
    root_path = root.GetPath()
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    occupied, floors = {}, []
    for p in root.GetChildren():
        b = bounds(cache, p)
        if b and p.GetName().startswith("SM_Floor") and abs(b[1][2] - floor) < 0.03:
            floors.append(b)
        # Floor/ceiling slabs touch props by design. Rejecting their full AABBs
        # would reject the whole room. Other structural bounds stay obstacles.
        if p.GetName() == "SM_Buildings":
            # The enclosing background group spans the entire map. Its individual
            # buildings are separate bounds, rather than one giant solid box.
            for child in p.GetChildren():
                child_bounds = bounds(cache, child)
                if child_bounds:
                    occupied[str(child.GetPath())] = child_bounds
        elif b and not p.GetName().startswith(("SM_Floor", "SM_Ceiling")):
            occupied[str(p.GetPath())] = b
    if not floors:
        raise ValueError("no SM_Floor support bounds found at floor_z_m")

    layer = Sdf.Layer.CreateAnonymous("office_variant.usda")
    layer.subLayerPaths = [str(source)]
    stage = Usd.Stage.Open(layer)
    stage.SetDefaultPrim(stage.GetPrimAtPath(root_path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    rng = random.Random(seed)
    changes, edited = [], set()
    for index, op in enumerate(operations):
        kind, path = op.get("action"), op.get("source")
        if kind not in ("move", "duplicate") or not isinstance(path, str):
            raise ValueError("each operation needs action=move|duplicate and source prim path")
        prim = original.GetPrimAtPath(path)
        if not prim or prim.GetParent() != root or not prim.IsA(UsdGeom.Xform):
            raise ValueError(f"select a whole root-child Xform prop: {path}")
        allowed = MOVABLE if kind == "move" else COPYABLE
        if not prim.GetName().startswith(allowed):
            raise ValueError(f"structural or unsupported prop edit: {path}")
        before = bounds(cache, prim)
        if not before or abs(before[0][2] - floor) > 0.03:
            raise ValueError(f"select a floor-supported prop (not a tabletop object): {path}")
        if kind == "move" and path in edited:
            raise ValueError(f"prop already moved: {path}")
        if ("delta_m" in op) == ("delta_range_m" in op):
            raise ValueError("provide exactly one of delta_m or delta_range_m")
        if "delta_m" in op:
            lower = upper = vector(op["delta_m"], "delta_m")
        else:
            interval = op["delta_range_m"]
            if not isinstance(interval, list) or len(interval) != 2:
                raise ValueError("delta_range_m must be [min_xyz, max_xyz]")
            lower, upper = vector(interval[0], "delta min"), vector(interval[1], "delta max")
            if any(a > b for a, b in zip(lower, upper)):
                raise ValueError("delta range min exceeds max")
        if lower[2] != 0 or upper[2] != 0:
            raise ValueError("only horizontal displacement is supported")
        target = path if kind == "move" else str(root_path.AppendChild(f"WS2_added_{index:03d}"))
        if kind == "duplicate" and stage.GetPrimAtPath(target):
            raise ValueError(f"duplicate destination already exists: {target}")
        selected = None
        for _ in range(256):
            delta = [rng.uniform(a, b) for a, b in zip(lower, upper)]
            after = [[v + d for v, d in zip(corner, delta)] for corner in before]
            others = [b for key, b in occupied.items() if not (kind == "move" and key == path)]
            supported = all(any(f[0][0] <= x <= f[1][0] and f[0][1] <= y <= f[1][1]
                                for f in floors)
                            for x in (after[0][0], after[1][0])
                            for y in (after[0][1], after[1][1]))
            if supported and not any(intersects(after, b, gap) for b in others + protected):
                selected = delta, after
                break
        if selected is None:
            raise ValueError(f"no nonoverlapping placement in requested range for {path}")
        delta, after = selected
        if kind == "duplicate":
            dst = stage.DefinePrim(target, "Xform")
            dst.GetReferences().AddReference(str(source), path)
        else:
            dst = stage.GetPrimAtPath(target)
            edited.add(path)
        xf = UsdGeom.Xformable(dst)
        matrix = UsdGeom.Xformable(prim).GetLocalTransformation()
        matrix.SetTranslateOnly(matrix.ExtractTranslation() + Gf.Vec3d(*delta))
        xf.MakeMatrixXform().Set(matrix)
        occupied[target] = after
        changes.append({"action": kind, "source": path, "target": target,
                        "delta_m": delta, "before_bounds_m": before, "after_bounds_m": after})
    # Relative references keep the bundle movable when assets and variants travel
    # together. Never flatten/rewrite the original material and texture layers.
    relative_source = os.path.relpath(source, output.parent)
    output.parent.mkdir(parents=True, exist_ok=True)
    if not layer.Export(str(output)):
        raise RuntimeError("USD export failed")
    saved_layer = Sdf.Layer.FindOrOpen(str(output))
    with Sdf.ChangeBlock():
        saved_layer.subLayerPaths = [relative_source]
        for change in changes:
            if change["action"] == "duplicate":
                saved_layer.GetPrimAtPath(change["target"]).referenceList.explicitItems = [
                    Sdf.Reference(relative_source, change["source"])]
    saved_layer.Save()
    manifest = {"schema_version": 1, "seed": seed, "source": relative_source,
                "source_sha256": digest(source), "variant_sha256": digest(output),
                "config": config, "changes": changes,
                "validation": "CPU composition and coarse AABB only; flight/contact validation pending"}
    manifest_path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--inventory", action="store_true")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.inventory:
        print(json.dumps(inventory(args.source), indent=2))
    elif args.config and args.output:
        print(json.dumps(build_variant(args.source, yaml.safe_load(args.config.read_text()), args.output), indent=2))
    else:
        parser.error("use --inventory or both --config and --output")


if __name__ == "__main__":
    main()
