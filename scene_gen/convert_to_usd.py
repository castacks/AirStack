#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["bpy", "trimesh", "pycollada"]
# ///
"""Convert 3D assets (any Blender-importable format) to USD.

Built for Objaverse assets, which ship as a mix of glb/gltf/obj/fbx/dae/ply/stl
and so cannot be referenced by the scene generator directly. Uses headless
Blender (the `bpy` module) as the conversion engine so that materials and
textures survive the round-trip — something trimesh/usd-core cannot do on their
own. `objaverse_assets.py` drives this script; run it directly to convert
arbitrary meshes.

Blender is Z-up and its glTF importer rotates Y-up source art on import, so
converted USDs come out **Z-up** — the scene generator's default. They do
*not* need `axis-up: "Y"` in an asset pack.

The `bpy` wheel only ships for CPython 3.13 and pins an older numpy, so this is
written as a PEP 723 single-file script: `uv run --script` builds an isolated
3.13 environment for it and never disturbs the repo's 3.10 env.

Usage
-----
    # single file
    uv run --script convert_to_usd.py model.glb

    # many files / whole directories, into an output tree
    uv run --script convert_to_usd.py assets/ -o usd_out --recursive

    # pick the USD flavor (default: usdc, the compact binary crate)
    uv run --script convert_to_usd.py model.fbx --format usdz

Options
-------
    -o, --out DIR      output directory (default: alongside each input)
    -f, --format EXT   usdc | usda | usd | usdz   (default: usdc)
    -r, --recursive    recurse into input directories
    --overwrite        re-convert even if the USD already exists
    --no-textures      skip copying/packing textures
    --selected-only    export only mesh/curve/light objects (drop cameras etc.)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import bpy

# extension -> (operator module, operator name). Every importer here was probed
# present in bpy 5.2; the legacy obj importer is kept as a fallback.
IMPORTERS: dict[str, tuple[str, str]] = {
    ".glb":  ("import_scene", "gltf"),
    ".gltf": ("import_scene", "gltf"),
    ".obj":  ("wm", "obj_import"),
    ".fbx":  ("import_scene", "fbx"),
    ".ply":  ("wm", "ply_import"),
    ".stl":  ("wm", "stl_import"),
    ".dae":  ("wm", "collada_import"),
    ".abc":  ("wm", "alembic_import"),
    ".x3d":  ("import_scene", "x3d"),
    ".usd":  ("wm", "usd_import"),
    ".usda": ("wm", "usd_import"),
    ".usdc": ("wm", "usd_import"),
    ".usdz": ("wm", "usd_import"),
}
SUPPORTED = tuple(IMPORTERS)


def reset_scene() -> None:
    """Return Blender to a completely empty scene (no default cube/camera/light)."""
    bpy.ops.wm.read_factory_settings(use_empty=True)


def valid_kwargs(op, wanted: dict) -> dict:
    """Keep only the kwargs this operator actually declares — guards against
    property renames across Blender versions."""
    props = op.get_rna_type().properties.keys()
    return {k: v for k, v in wanted.items() if k in props}


def import_via_trimesh(path: Path) -> None:
    """Geometry-only fallback for formats this Blender build can't open (e.g.
    Collada — the bpy wheel is built without OpenCollada). Loads with trimesh
    and rebuilds meshes in Blender; materials/textures are NOT preserved."""
    import trimesh

    scene = trimesh.load(str(path), force="scene")
    geoms = scene.dump(concatenate=False)  # transforms baked into world coords
    made = 0
    for i, g in enumerate(geoms):
        faces = getattr(g, "faces", None)
        if faces is None or len(faces) == 0:
            continue  # skip paths / point clouds
        me = bpy.data.meshes.new(f"{path.stem}_{i}")
        me.from_pydata(g.vertices.tolist(), [], faces.tolist())
        me.update()
        ob = bpy.data.objects.new(me.name, me)
        bpy.context.collection.objects.link(ob)
        made += 1
    if made == 0:
        raise RuntimeError("trimesh fallback found no faces")


def import_asset(path: Path) -> str:
    """Import `path` into the current scene. Returns the fidelity achieved:
    'full' (via Blender, materials preserved) or 'geometry' (trimesh fallback)."""
    ext = path.suffix.lower()
    mod, name = IMPORTERS[ext]
    op = getattr(getattr(bpy.ops, mod), name)
    try:
        op(filepath=str(path))
        return "full"
    except (RuntimeError, AttributeError):
        # legacy .obj importer if the new one chokes
        if ext == ".obj":
            try:
                bpy.ops.import_scene.obj(filepath=str(path))
                return "full"
            except (RuntimeError, AttributeError):
                pass
        reset_scene()  # clear any partial import before the fallback
        import_via_trimesh(path)
        return "geometry"


def export_usd(out_path: Path, textures: bool, selected_only: bool) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    wanted = {
        "filepath": str(out_path),
        "export_materials": True,
        "export_textures": textures,
        "overwrite_textures": True,
        "relative_paths": True,
        "selected_objects_only": selected_only,
        # keep Y-up meshes upright in USD's Y-up world
        "convert_orientation": False,
    }
    kwargs = valid_kwargs(bpy.ops.wm.usd_export, wanted)
    bpy.ops.wm.usd_export(**kwargs)


def scene_is_empty() -> bool:
    return len(bpy.context.scene.objects) == 0


def world_bbox():
    """World-space bounding box of everything renderable, as
    ``(min, max)`` 3-tuples.

    Reported so callers can size an asset without reopening the USD: these are
    Blender's world coordinates, and `export_usd` writes them through unchanged
    (``convert_orientation=False``), so they *are* the USD's extents. Guessing
    instead — by measuring the source mesh and predicting how the importer
    rotates it — gets the up-axis wrong on any asset whose glTF carries its own
    root transform.
    """
    import mathutils

    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    for ob in bpy.context.scene.objects:
        if ob.type not in {"MESH", "CURVE", "SURFACE", "META", "FONT"}:
            continue
        for corner in ob.bound_box:
            v = ob.matrix_world @ mathutils.Vector(corner)
            for k in range(3):
                lo[k] = min(lo[k], v[k])
                hi[k] = max(hi[k], v[k])
    if lo[0] == float("inf"):
        return None
    return lo, hi


def apply_target_size(target_m: float, fit: str) -> float:
    """Uniformly scale the scene so *fit* measures *target_m*, and return the
    factor applied.

    glb carries no canonical unit, so an Objaverse asset arrives at whatever
    size its author used. Baking the metric scale in here — rather than
    recording it for the asset pack to re-apply — means the cached USD is
    already in meters, so a config just names the asset and nothing has to keep
    a scale factor in sync with it.

    ``fit`` picks the dimension being pinned: ``footprint`` (longest of x/y —
    right for buildings, which are sized by their plan), ``height`` (z), or
    ``max``.
    """
    import mathutils

    box = world_bbox()
    if box is None:
        raise RuntimeError("nothing renderable to scale")
    lo, hi = box
    sx, sy, sz = (hi[k] - lo[k] for k in range(3))
    ref = {"footprint": max(sx, sy), "height": sz, "max": max(sx, sy, sz)}[fit]
    if ref <= 0:
        raise RuntimeError(f"degenerate bounding box {(sx, sy, sz)}")

    factor = float(target_m) / ref
    S = mathutils.Matrix.Scale(factor, 4)
    # Only roots: children inherit the transform, so scaling them too would
    # apply the factor twice.
    for ob in bpy.context.scene.objects:
        if ob.parent is None:
            ob.matrix_world = S @ ob.matrix_world
    # Blender defers matrix_world writes to the next depsgraph evaluation, so
    # without this any bbox read back here still reports the pre-scale size.
    bpy.context.view_layer.update()
    return factor


def gather_inputs(inputs: list[str], recursive: bool) -> list[Path]:
    files: list[Path] = []
    for raw in inputs:
        p = Path(raw).expanduser()
        if p.is_dir():
            it = p.rglob("*") if recursive else p.glob("*")
            files += sorted(f for f in it if f.suffix.lower() in IMPORTERS)
        elif p.suffix.lower() in IMPORTERS:
            files.append(p)
        else:
            print(f"skip (unsupported): {p}", file=sys.stderr)
    return files


def out_path_for(src: Path, roots: list[Path], out_dir: Path | None,
                 ext: str, bundle: bool) -> Path:
    if out_dir is None:
        return src.with_suffix(f".{ext}")
    # mirror the tree relative to whichever input root contains src. `roots` are
    # directories (a file input contributes its parent), so a loose file maps to
    # out_dir/<name> and a dir input reproduces its subtree under out_dir.
    rel = Path(src.name)
    for root in roots:
        try:
            rel = src.relative_to(root)
            break
        except ValueError:
            continue
    base = out_dir / rel
    if bundle:
        # give each asset its own folder so sidecar textures (which Blender
        # names Image_0, Image_1, … restarting per export) can't collide with
        # another asset's. USD assets are conventionally shipped this way.
        base = base.parent / src.stem / src.stem
    return base.with_suffix(f".{ext}")


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Convert 3D assets to USD via headless Blender.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("inputs", nargs="+", help="files and/or directories")
    ap.add_argument("-o", "--out", help="output directory (default: beside each input)")
    ap.add_argument("-f", "--format", default="usdc",
                    choices=["usdc", "usda", "usd", "usdz"], help="USD flavor")
    ap.add_argument("-r", "--recursive", action="store_true",
                    help="recurse into input directories")
    ap.add_argument("--overwrite", action="store_true",
                    help="re-convert even if the output already exists")
    ap.add_argument("--no-textures", dest="textures", action="store_false",
                    help="do not copy/pack textures")
    ap.add_argument("--selected-only", action="store_true",
                    help="export renderable objects only (drop cameras/empties)")
    ap.add_argument("--report", action="store_true",
                    help="print a machine-readable BBOX <json> line per asset")
    ap.add_argument("--target-size", type=float, default=0.0,
                    help="scale the asset so --fit measures this many meters "
                         "(0 = keep the source's own units)")
    ap.add_argument("--fit", default="footprint",
                    choices=["footprint", "height", "max"],
                    help="which dimension --target-size pins")
    ap.add_argument("--flat", action="store_true",
                    help="write USDs directly into --out instead of one folder "
                         "per asset (textures from multiple assets may collide)")
    args = ap.parse_args()

    # bundle each asset in its own folder by default when writing to --out with
    # sidecar textures; usdz packs textures internally so it needs no bundling.
    bundle = bool(args.out) and not args.flat and args.textures and args.format != "usdz"

    roots = [p if p.is_dir() else p.parent
             for p in (Path(i).expanduser() for i in args.inputs)]
    out_dir = Path(args.out).expanduser() if args.out else None
    files = gather_inputs(args.inputs, args.recursive)

    if not files:
        print("No convertible files found. Supported: " + ", ".join(SUPPORTED),
              file=sys.stderr)
        return 1

    print(f"Converting {len(files)} file(s) -> .{args.format}\n")
    ok = geom_only = skipped = failed = 0
    t0 = time.time()

    for i, src in enumerate(files, 1):
        dst = out_path_for(src, roots, out_dir, args.format, bundle)
        tag = f"[{i}/{len(files)}] {src.name}"
        if dst.exists() and not args.overwrite:
            print(f"{tag} -> exists, skip")
            skipped += 1
            continue
        try:
            reset_scene()
            fidelity = import_asset(src)
            if scene_is_empty():
                raise RuntimeError("importer produced no objects")
            scale = apply_target_size(args.target_size, args.fit) \
                if args.target_size else 1.0
            bbox = world_bbox()
            export_usd(dst, args.textures, args.selected_only)
            note = "  (geometry-only — no materials)" if fidelity == "geometry" else ""
            size = [bbox[1][k] - bbox[0][k] for k in range(3)] if bbox else None
            extra = f"  {size[0]:.2f} x {size[1]:.2f} x {size[2]:.2f} m" \
                if (size and args.target_size) else ""
            print(f"{tag} -> {dst}{note}{extra}")
            if args.report and bbox:
                lo, hi = bbox
                print("BBOX " + json.dumps({
                    "src": str(src), "usd": str(dst),
                    "min": lo, "max": hi, "size": size,
                    "scale": scale,
                }))
            ok += 1
            geom_only += fidelity == "geometry"
        except Exception as e:  # noqa: BLE001 — one bad asset must not stop the batch
            print(f"{tag} -> FAILED: {type(e).__name__}: {e}", file=sys.stderr)
            failed += 1

    dt = time.time() - t0
    extra = f" ({geom_only} geometry-only)" if geom_only else ""
    print(f"\nDone in {dt:.1f}s — {ok} converted{extra}, "
          f"{skipped} skipped, {failed} failed.")
    return 1 if failed and ok == 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
