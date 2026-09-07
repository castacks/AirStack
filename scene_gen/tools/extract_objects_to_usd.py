#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["bpy", "numpy", "scipy"]
# ///
"""extract_objects_to_usd.py — one USD per object out of a JOINED scene export.

WHAT THIS IS FOR
----------------
`convert_to_usd.py` converts a file to a file: one asset in, one asset out.
That is the right shape for Objaverse, where a download IS one prop, and the
wrong shape for a city export, where a single `Grid System.fbx` holds thirteen
buildings, the ground they stand on, the grass, the fences and the footpaths —
all "joined" into one scene by whoever exported it. Referencing that lands the
entire district as one prim: it cannot be placed, scaled, damaged or picked
from a pool, which is everything the scene generator does with a building.

So this splits on the boundary the ARTIST drew — the named object — and writes
each one as its own USD, re-centred so it behaves like every other asset in a
pack.

    # what is in there, and how big (no export, seconds)
    extract_objects_to_usd.py "city/Grid System.fbx" --list

    # the buildings, each to its own USD
    extract_objects_to_usd.py "city/Grid System.fbx" -o out/ --match '^Building \\d+$'

MORE THAN ONE WAY TO SPLIT (2026-08-26, for the `standalone` pack)
--------------------------------------------------------------------
The artist's split is not always "one object = one asset":

    --group '^(Building_\d+)_'   parts named by prefix -> one asset per prefix
    --group-by-material          a pack whose exporter chopped every prop at
                                 65k verts -> one asset per material
    --join NAME                  a car exported as 124 parts -> one asset
    --loose-parts                a debris scatter that is ONE mesh of many
                                 pieces -> one asset per connected component
    --exclude REGEX              drop the ground slab that came with a building

And the asset is written METRES, Z-UP, FACING +X so the pack needs no fixups:

    --scale 0.0254 | --fit-length 4.6     inch/cm art, or "a car is 4.6 m"
    --yaw -90                             bake the front onto +X
    --decimate-to 50000                   a 1.5M-tri AI-generated house
    --flat-material R,G,B[,rough,metal]   an FBX whose maps never shipped
    --pbr-maps DIR                        loose BaseColor/Normal/Roughness jpgs
    --per-asset-dir                       <out>/<name>/<name>.usdc + textures/,
                                          for sources that reuse texture names

WHY NOT `tools/split_usd_pack.py`
---------------------------------
That one finds sub-prims inside an already-converted USD and measures them so a
config can reference `file.usd</Sub/Prim>`. It never writes a file, and the
asset packs cannot express a sub-prim reference anyway. Different job: this is
the import-side split, and its output is ordinary assets.

RE-CENTRING IS NOT COSMETIC
---------------------------
A joined export authors every object at its position in the district, so
"Building 7" carries a 300 m translation baked into its points. Referenced by
the generator — which places by setting a transform — it would land 300 m from
where it was put, and its measured footprint would be the bbox of a building
that is nowhere near the origin. Each object is therefore moved so its
footprint centre is at (0, 0) and its lowest point at z = 0, and the transform
is APPLIED, so the geometry itself is authored about the origin.

CARVING BUILDINGS OUT OF A MESH THAT DOES NOT SAY IT HAS ANY
------------------------------------------------------------
Splitting on object names only works when the objects are the split the artist
made. In the DowntownCity pack they are not: `Building 1` through
`Building 10` are objects with **zero vertices in the file itself** (read out of
the binary FBX, so this is not an importer failing), while the mesh named
`Path and Imperfections` is 3.2M triangles, carries 104 materials and is 30.6 m
TALL. The buildings were merged into the paths and their own objects left
empty.

`--carve NAME` recovers them. It rasterises the named mesh into a top-down
height grid, takes the cells standing more than `--carve-min-h` above the
mesh's own floor, labels the connected regions of those cells, and separates
the faces whose centroid falls in each region. On this pack that finds twelve
buildings of 840-3,124 m2 and 16-31 m tall, laid out on a block grid — which is
what "Grid System" was always the name of.

Footprint columns, not the tall cells alone: a region is used as a MASK over
the whole column, so a building comes out with its plinth and the slab it
stands on rather than sheared off at 8 m.

LOOSE GEOMETRY IS NOT HARMLESS
-----------------------------
A joined export carries stray vertices and edges that belong to no face —
leftovers of whatever the artist deleted. They render as nothing, so they are
invisible in a preview, and they are still in the bounding box: `Building 12`
measured 140.2 x 75.0 m because of TWO face-less vertices 135 m from the
building, which is really 44.3 x 18.7 m. The generator sizes and places every
asset by its measured bbox, so one of those makes a building land displaced
inside a footprint three times too big, and no amount of looking at it would
have shown why. They are deleted before anything is measured (`--keep-loose`
to see the damage for yourself).

TEXTURES
--------
`export_textures` copies the maps beside the USD, and every object that shares
a material gets its own copy — the price of one-file-per-building. Pass
`--no-textures` for a geometry-only pass when you are still deciding what is
worth keeping.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import bpy

IMPORTERS = {
    ".fbx": ("import_scene", "fbx"),
    ".glb": ("import_scene", "gltf"),
    ".gltf": ("import_scene", "gltf"),
    ".obj": ("wm", "obj_import"),
    ".dae": ("wm", "collada_import"),
    ".usd": ("wm", "usd_import"),
    ".usdc": ("wm", "usd_import"),
    ".usda": ("wm", "usd_import"),
    ".usdz": ("wm", "usd_import"),
}


def reset_scene() -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_asset(path: Path) -> None:
    ext = path.suffix.lower()
    if ext not in IMPORTERS:
        raise SystemExit(f"no importer for {ext}")
    mod, op = IMPORTERS[ext]
    getattr(getattr(bpy.ops, mod), op)(filepath=str(path))


def mesh_objects():
    return [o for o in bpy.context.scene.objects if o.type == "MESH"]


def drop_loose(obj) -> int:
    """Delete vertices and edges that belong to no face. Returns how many.

    Precise on purpose: "belongs to no face" can never be real building
    geometry, so this needs no distance heuristic and cannot eat a thin railing
    or a flagpole. See the header for what it costs to leave them in.
    """
    import bmesh

    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    stray = [v for v in bm.verts if not v.link_faces]
    if stray:
        bmesh.ops.delete(bm, geom=stray, context="VERTS")
        bm.to_mesh(me)
        me.update()
    bm.free()
    return len(stray)


def tri_count(obj) -> int:
    """Triangles after triangulation, without modifying the object."""
    me = obj.data
    return sum(max(len(p.vertices) - 2, 1) for p in me.polygons)


def world_bounds(obj):
    """(min, max) world-space corners as two 3-lists.

    FROM THE VERTICES, not from `obj.bound_box`. The bound box is cached on the
    object and is NOT refreshed by a bmesh edit, so measuring it straight after
    `drop_loose` reports the size the object had BEFORE the strays were
    deleted — which looks exactly like the cleanup having done nothing, and
    cost an export to work out.
    """
    import numpy as np
    from mathutils import Vector

    me = obj.data
    n = len(me.vertices)
    if not n:
        return [0.0] * 3, [0.0] * 3
    co = np.empty(n * 3, dtype=np.float32)
    me.vertices.foreach_get("co", co)
    co = co.reshape(n, 3)
    m = np.array(obj.matrix_world).astype(np.float64)
    world = co @ m[:3, :3].T + m[:3, 3]
    return world.min(axis=0).tolist(), world.max(axis=0).tolist()


def recentre(obj) -> tuple:
    """Move *obj* so its footprint centre is at the origin and its base at z=0.

    Returns the (x, y, z) it was moved by, which is where it stood in the
    district — worth printing, because it is the only record left of the
    layout once the objects are separate files.
    """
    import mathutils

    lo, hi = world_bounds(obj)
    shift = mathutils.Vector((-(lo[0] + hi[0]) / 2.0,
                              -(lo[1] + hi[1]) / 2.0,
                              -lo[2]))
    obj.matrix_world.translation += shift
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return (-shift.x, -shift.y, -shift.z)


def export_one(obj, out_path: Path, textures: bool) -> None:
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    out_path.parent.mkdir(parents=True, exist_ok=True)
    kwargs = dict(filepath=str(out_path), selected_objects_only=True,
                  export_materials=True, export_textures=textures,
                  export_animation=False, root_prim_path="/root")
    # bpy keeps renaming these; drop anything this build does not know rather
    # than failing the whole run on a keyword.
    props = bpy.ops.wm.usd_export.get_rna_type().properties.keys()
    bpy.ops.wm.usd_export(**{k: v for k, v in kwargs.items() if k in props})


def carve_regions(obj, min_h: float, cell: float, min_area: float) -> list:
    """Split *obj* into one object per tall footprint region. Returns them.

    See the header for why this exists. The mask is built from the VERTEX
    height field rather than from faces, because a wall's faces are vertical
    and their centroids sit halfway up — a face-height mask puts a hole through
    the middle of every building.
    """
    # `bmesh` AFTER `bpy`: it is registered by bpy's native module, so a
    # module-scope `import bmesh` sorted above `import bpy` fails outright.
    import bmesh
    import numpy as np
    from scipy import ndimage

    me = obj.data
    nv = len(me.vertices)
    co = np.empty(nv * 3, dtype=np.float32)
    me.vertices.foreach_get("co", co)
    co = co.reshape(nv, 3)
    m = np.array(obj.matrix_world)
    w = co @ m[:3, :3].T + m[:3, 3]

    x0, y0, z0 = w[:, 0].min(), w[:, 1].min(), w[:, 2].min()
    nx = int(np.ceil(np.ptp(w[:, 0]) / cell)) + 1
    ny = int(np.ceil(np.ptp(w[:, 1]) / cell)) + 1
    ix = np.clip(((w[:, 0] - x0) / cell).astype(np.int32), 0, nx - 1)
    iy = np.clip(((w[:, 1] - y0) / cell).astype(np.int32), 0, ny - 1)
    top = np.full((nx, ny), -1e9, dtype=np.float32)
    np.maximum.at(top, (ix, iy), w[:, 2])

    tall = ndimage.binary_closing((top - z0) > min_h, np.ones((3, 3)))
    lab, n = ndimage.label(tall, structure=np.ones((3, 3)))
    keep = [i for i in range(1, n + 1)
            if (lab == i).sum() * cell * cell >= min_area]
    print(f"[extract] carve: {n} tall region(s), {len(keep)} over "
          f"{min_area:.0f} m2", flush=True)
    if not keep:
        return []

    nf = len(me.polygons)
    ctr = np.empty(nf * 3, dtype=np.float32)
    me.polygons.foreach_get("center", ctr)
    ctr = (ctr.reshape(nf, 3) @ m[:3, :3].T + m[:3, 3])
    fx = np.clip(((ctr[:, 0] - x0) / cell).astype(np.int32), 0, nx - 1)
    fy = np.clip(((ctr[:, 1] - y0) / cell).astype(np.int32), 0, ny - 1)
    face_lab = lab[fx, fy]

    # BMESH, NOT `mesh.separate(type="SELECTED")`. The operator route sets the
    # face flags in object mode and then has to survive a mode toggle and a
    # `select_mode` change, both of which re-flush the selection — the first
    # region came out holding all 3.2M faces and the other eleven found nothing
    # left. Building each region's mesh directly has no selection state to lose.
    made = []
    mats = list(me.materials)
    for i in keep:
        drop = [k for k in range(len(face_lab)) if face_lab[k] != i]
        if len(drop) >= nf:
            continue
        bm = bmesh.new()
        bm.from_mesh(me)
        bm.faces.ensure_lookup_table()
        bmesh.ops.delete(bm, geom=[bm.faces[k] for k in drop], context="FACES")
        sub = bpy.data.meshes.new(f"Carved_{i:02d}")
        bm.to_mesh(sub)
        bm.free()
        for mat in mats:
            sub.materials.append(mat)
        o2 = bpy.data.objects.new(f"Carved {i:02d}", sub)
        o2.matrix_world = obj.matrix_world.copy()
        bpy.context.scene.collection.objects.link(o2)
        made.append(o2)
        print(f"[extract]   region {i:02d}: {len(sub.polygons):,} faces",
              flush=True)
    return made


def _slug(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9]+", "_", name).strip("_")


def _join(objs, name: str):
    """Join *objs* into one mesh object called *name*. Returns it."""
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    out = bpy.context.view_layer.objects.active
    # A USD import leaves an EMPTY for every Xform prim, and its name is the
    # one the artist gave the asset — take it, or the mesh comes out `car1.001`.
    other = bpy.data.objects.get(name)
    if other is not None and other is not out:
        other.name = name + "_xform"
    out.name = name
    return out


def _group(objs, pattern: str) -> list:
    """One joined object per distinct first capture group of *pattern*.

    For a pack whose parts are named `Building_03_Roof`, `Building_03_Shops`,
    ... — the artist's split is the NAME PREFIX, not the object. Objects the
    pattern does not match are dropped.
    """
    rx = re.compile(pattern)
    groups: dict = {}
    for o in objs:
        m = rx.search(o.name)
        if m:
            groups.setdefault(m.group(1), []).append(o)
    return [_join(v, k) for k, v in sorted(groups.items())]


def _group_by_material(objs) -> list:
    """One joined object per material — the split a vertex-capped export lost.

    A GLES / `materialmerger` exporter chops a mesh at the 16-bit index limit,
    so one prop arrives as `Object_2` (65,532 verts) plus `Object_3` (the
    remainder): same material, and one bbox nested inside the other rather
    than beside it. The artist's split survives only in the material
    assignment, so that is what is grouped on. Objects with no material are
    dropped.
    """
    groups: dict = {}
    for o in objs:
        slot = o.material_slots[0].material if o.material_slots else None
        if slot is not None:
            groups.setdefault(slot.name, []).append(o)
    return [_join(v, k) for k, v in sorted(groups.items())]


def _loose_parts(obj) -> list:
    """Split *obj* into its connected components. Returns them.

    For a debris scatter authored as ONE mesh of many pieces lying on the
    ground: there is no name to split on, but the pieces do not share
    vertices, which is the split.
    """
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.select_all(action="SELECT")
    bpy.ops.mesh.separate(type="LOOSE")
    bpy.ops.object.mode_set(mode="OBJECT")
    return [o for o in bpy.context.selected_objects if o.type == "MESH"]


def _decimate(obj, target_tris: int) -> None:
    """Collapse *obj* to about *target_tris* triangles, UVs kept."""
    n = tri_count(obj)
    if n <= target_tris:
        return
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    mod = obj.modifiers.new("decimate", "DECIMATE")
    mod.ratio = target_tris / n
    bpy.ops.object.modifier_apply(modifier=mod.name)
    print(f"[extract]   decimated {obj.name}: {n:,} -> {tri_count(obj):,} tris",
          flush=True)


def _flat_material(obj, rgb, roughness: float = 0.85, metallic: float = 0.0):
    """Replace every material on *obj* with one plain Principled BSDF.

    For sources whose materials came through as nothing — a 3ds Max FBX whose
    VRay maps were never in the file, so every piece renders black or flat.
    """
    mat = bpy.data.materials.new(f"{obj.name}_flat")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
    bsdf.inputs["Roughness"].default_value = roughness
    bsdf.inputs["Metallic"].default_value = metallic
    obj.data.materials.clear()
    obj.data.materials.append(mat)


def _pbr_maps(obj, folder: Path):
    """One Principled material from `*BaseColor*`/`*albedo*`, `*Normal*`,
    `*Roughness*` images in *folder* — for an FBX shipped beside loose maps
    that the importer did not wire up."""
    def find(*keys):
        for f in sorted(folder.iterdir()):
            if any(k in f.name.lower() for k in keys) and f.suffix.lower() in (".jpg", ".png"):
                return f
        return None
    mat = bpy.data.materials.new(f"{obj.name}_pbr")
    mat.use_nodes = True
    nt = mat.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    for keys, sock, colorspace in ((("basecolor", "albedo"), "Base Color", "sRGB"),
                                   (("roughness",), "Roughness", "Non-Color")):
        f = find(*keys)
        if f:
            tex = nt.nodes.new("ShaderNodeTexImage")
            tex.image = bpy.data.images.load(str(f))
            tex.image.colorspace_settings.name = colorspace
            nt.links.new(tex.outputs["Color"], bsdf.inputs[sock])
    f = find("normal")
    if f:
        tex = nt.nodes.new("ShaderNodeTexImage")
        tex.image = bpy.data.images.load(str(f))
        tex.image.colorspace_settings.name = "Non-Color"
        nm = nt.nodes.new("ShaderNodeNormalMap")
        nt.links.new(tex.outputs["Color"], nm.inputs["Color"])
        nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    obj.data.materials.clear()
    obj.data.materials.append(mat)


def _prepose(obj, scale: float, fit_len: float, yaw_deg: float) -> float:
    """Scale (or fit the longest footprint side to *fit_len* m) and yaw
    *obj* about Z, BEFORE recentring. Returns the scale used.

    Baked into the geometry by `recentre`'s transform_apply, so the written
    asset is metres and faces +X and the pack needs no `scale`/`yaw-offset`.
    """
    import math
    lo, hi = world_bounds(obj)
    if fit_len > 0.0:
        scale = fit_len / max(hi[0] - lo[0], hi[1] - lo[1], 1e-6)
    if scale != 1.0:
        obj.matrix_world = (
            __import__("mathutils").Matrix.Scale(scale, 4) @ obj.matrix_world)
    if yaw_deg:
        obj.matrix_world = (
            __import__("mathutils").Matrix.Rotation(math.radians(yaw_deg), 4, "Z")
            @ obj.matrix_world)
    return scale


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("source")
    ap.add_argument("-o", "--out", default="", help="output directory")
    ap.add_argument("--match", default="",
                    help="regex an object NAME must match to be exported")
    ap.add_argument("--min-tris", type=int, default=0,
                    help="skip objects below this triangle count")
    ap.add_argument("--list", action="store_true",
                    help="print the inventory and exit")
    ap.add_argument("--format", default="usdc", choices=("usdc", "usda", "usd"))
    ap.add_argument("--no-textures", action="store_true")
    ap.add_argument("--carve", default="",
                    help="carve this object into one per tall footprint "
                         "region — for a mesh the buildings were merged into")
    ap.add_argument("--carve-min-h", type=float, default=8.0)
    ap.add_argument("--carve-cell", type=float, default=2.0)
    ap.add_argument("--carve-min-area", type=float, default=200.0)
    ap.add_argument("--exclude", default="",
                    help="regex; objects whose NAME matches are dropped first")
    ap.add_argument("--group", default="",
                    help="regex with one capture group; objects sharing the "
                         "capture are JOINED into one asset named by it")
    ap.add_argument("--group-by-material", action="store_true",
                    help="one asset per MATERIAL — for an export that chopped "
                         "each prop into 65k-vertex chunks")
    ap.add_argument("--join", default="",
                    help="join every (matching) object into ONE asset with "
                         "this name")
    ap.add_argument("--loose-parts", action="store_true",
                    help="split each object into connected components first "
                         "(a debris scatter authored as one mesh)")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="uniform scale baked into the geometry (cm art: 0.01)")
    ap.add_argument("--fit-length", type=float, default=0.0,
                    help="scale so the longest footprint side is this many m")
    ap.add_argument("--yaw", type=float, default=0.0,
                    help="deg about Z baked into the geometry, so the written "
                         "asset faces +X")
    ap.add_argument("--decimate-to", type=int, default=0,
                    help="collapse each asset to about this many triangles")
    ap.add_argument("--flat-material", default="",
                    help="R,G,B[,roughness[,metallic]] — replace all materials "
                         "with one plain colour (untextured sources)")
    ap.add_argument("--pbr-maps", default="",
                    help="folder of loose BaseColor/Normal/Roughness maps to "
                         "wire into one material")
    ap.add_argument("--per-asset-dir", action="store_true",
                    help="write <out>/<name>/<name>.usdc so each asset has "
                         "its own textures/ (sources that reuse texture names)")
    ap.add_argument("--keep-loose", action="store_true",
                    help="do NOT delete face-less vertices/edges (see the "
                         "module docstring — they corrupt the measured size)")
    args = ap.parse_args()

    src = Path(args.source).expanduser()
    if not src.exists():
        raise SystemExit(f"no such file: {src}")

    reset_scene()
    print(f"[extract] importing {src.name} ({src.stat().st_size / 1e6:.0f} MB)"
          " — a large joined export takes a few minutes", flush=True)
    import_asset(src)

    objs = mesh_objects()
    rx = re.compile(args.match) if args.match else None
    print(f"[extract] {len(objs)} mesh object(s)")
    if not args.keep_loose:
        n = sum(drop_loose(o) for o in objs)
        if n:
            print(f"[extract] dropped {n:,} face-less vert(s) — the bbox is "
                  f"only honest once they are gone")
    if args.carve:
        target = next((o for o in objs if args.carve in o.name), None)
        if target is None:
            raise SystemExit(f"no object matching {args.carve!r}")
        print(f"[extract] carving {target.name}", flush=True)
        objs = carve_regions(target, args.carve_min_h, args.carve_cell,
                             args.carve_min_area)
        if not args.keep_loose:
            for o in objs:
                drop_loose(o)
        rx = re.compile(args.match) if args.match else None

    if args.exclude:
        ex = re.compile(args.exclude)
        objs = [o for o in objs if not ex.search(o.name)]
    if args.loose_parts:
        objs = [p for o in objs for p in _loose_parts(o)]
        print(f"[extract] {len(objs)} loose part(s)")
    if args.group:
        objs = _group(objs, args.group)
        rx = None
    elif args.group_by_material:
        objs = _group_by_material(objs)
        rx = None
    elif args.join:
        objs = [_join([o for o in objs if rx is None or rx.search(o.name)],
                      args.join)]
        rx = None

    rows = []
    for o in sorted(objs, key=lambda o: o.name):
        lo, hi = world_bounds(o)
        size = [hi[i] - lo[i] for i in range(3)]
        rows.append((o, tri_count(o), size, lo))
    for o, tris, size, lo in rows:
        keep = (rx is None or rx.search(o.name)) and tris >= args.min_tris
        print(f"  {'*' if keep else ' '} {o.name:34s} tris={tris:9,d}  "
              f"{size[0]:8.1f} x {size[1]:7.1f} x {size[2]:7.1f} m  "
              f"at ({lo[0]:8.1f}, {lo[1]:8.1f})  mats={len(o.material_slots)}")
    if args.list:
        return 0

    out_dir = Path(args.out or src.parent / "usd").expanduser()
    n = 0
    for o, tris, size, _lo in rows:
        if rx is not None and not rx.search(o.name):
            continue
        if tris < args.min_tris:
            continue
        if args.decimate_to:
            _decimate(o, args.decimate_to)
        if args.flat_material:
            v = [float(x) for x in args.flat_material.split(",")]
            _flat_material(o, v[:3], *(v[3:5]))
        if args.pbr_maps:
            _pbr_maps(o, Path(args.pbr_maps).expanduser())
        sc = _prepose(o, args.scale, args.fit_length, args.yaw)
        was = recentre(o)
        size = [sc * v for v in size]
        stem = _slug(o.name)
        dst = (out_dir / stem if args.per_asset_dir else out_dir) \
            / f"{stem}.{args.format}"
        export_one(o, dst, not args.no_textures)
        n += 1
        print(f"[extract] {dst.name:34s} {size[0]:6.1f} x {size[1]:6.1f} x "
              f"{size[2]:6.1f} m  (stood at {was[0]:.0f}, {was[1]:.0f})",
              flush=True)
    print(f"[extract] wrote {n} USD(s) -> {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
