#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["bpy", "pillow", "pyyaml", "numpy"]
# ///
"""building_gallery.py — every building in an asset pack, rendered in Cycles.

    ~/coasei/scenegen/.venv/bin/python tools/building_gallery.py \
        --pack urban_v2 --out galleries/urban_v2 --res 2048

Writes, under --out:

    all_buildings.png                 every pool laid out in rows, one shot
    groups/<pool>.png                 one row per pool (intact_tower, ...)
    individual/<pool path>/<name>.png hero view, high resolution
    individual/<pool path>/<name>_views.png   four-azimuth sheet

The pool path mirrors the yaml (`intact/tower`, `damaged`, `destroyed/midrise`).

Assets must resolve on the HOST: `airstack://` paths directly, Nucleus paths
through the mirror under `assets/nucleus/` (see `tools/localize_nucleus_assets.py`
and `tools/nucleus.py`). Unresolvable entries are listed and skipped.

Scale: Blender applies a stage's `metersPerUnit`, so most cm-authored Nucleus
art arrives in metres already; an asset that declares no unit and whose pack
entry says `scale: 0.01` is scaled here — detected as "taller than 400 units",
which no building is in metres.
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
from pathlib import Path

import bpy
import numpy as np
import yaml
from mathutils import Vector

REPO = Path(__file__).resolve().parents[2]
SCENE_GEN = REPO / "scene_gen"
NUCLEUS_MIRRORS = [SCENE_GEN / "assets" / "nucleus", SCENE_GEN / "assets" / "nucleus" / "Muyang"]


#: The Nucleus project root the packs resolve against. A pack may name an
#: asset as an absolute `omniverse://` URL under it, as `urban_v2` now does for
#: everything — those files also exist in the checkout (they were uploaded FROM
#: it), so the gallery maps the URL back rather than losing the render.
NUCLEUS_ROOT = "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"


def resolve(usd: str) -> Path | None:
    """The local file for a pack entry, or None if it is not on this machine.

    Cycles cannot open an `omniverse://` stage, so every scheme has to come
    back to a path: `airstack://` is the checkout, and a Nucleus asset — named
    relative to the pack's `asset_root` or as a full URL — is tried BOTH ways,
    as the checkout copy it was uploaded from, then as the mirror.

    The two Nucleus forms take the SAME candidates on purpose. `asset_root` is
    `NUCLEUS_ROOT`, so `scene_gen/assets/x.usdc` and `<NUCLEUS_ROOT>scene_gen/
    assets/x.usdc` name one file, and `_join_asset_root` treats them as one.
    Trying the checkout only for the full URL meant that rewriting a pack to
    the relative form — as `urban_v3` did on 2026-08-29 — silently dropped
    every repo-published asset from the gallery: `assets/nucleus/` holds the
    art mirrored FROM the server, never the art published TO it.
    """
    if usd.startswith("airstack://"):
        p = REPO / usd[len("airstack://"):]
        return p if p.exists() else None
    rest = usd[len(NUCLEUS_ROOT):] if usd.startswith(NUCLEUS_ROOT) else usd
    if "://" in rest or rest.startswith("/"):
        return None
    for cand in [REPO / rest] + [m / rest for m in NUCLEUS_MIRRORS]:
        if cand.exists():
            return cand
    return None


def condition_of(entry) -> str:
    """The condition tag on an entry — `intact` unless it says otherwise.

    Mirrors `scene_generator.condition_of`, kept local because this script runs
    under `bpy`'s 3.13 interpreter, where `scene_generator` (and `pxr`) are not
    importable.
    """
    if isinstance(entry, dict):
        for t in entry.get("tags") or ():
            if str(t) in ("intact", "damaged", "destroyed"):
                return str(t)
    return "intact"


def walk_pools(node, path=()):
    """Yield (pool_path_tuple, [entries]) for every leaf list under buildings."""
    if isinstance(node, list):
        yield path, node
    elif isinstance(node, dict):
        for k, v in node.items():
            yield from walk_pools(v, path + (k,))


def bounds(objs):
    """World bbox of what *objs* actually RENDER, instances included.

    Through the evaluated depsgraph rather than `obj.bound_box`: the AEC
    brownstones are a point-instancer whose prototype meshes sit unscaled in
    a hidden `proto` collection while the visible copy is a collection
    instance at 0.01 — measuring the prototypes reports a 2 km building and
    scales the real one away to a speck.
    """
    names = {o.name for o in objs}
    dg = bpy.context.evaluated_depsgraph_get()
    lo = np.full(3, np.inf); hi = np.full(3, -np.inf)
    for inst in dg.object_instances:
        ob = inst.object
        if ob.type != "MESH":
            continue
        owner = inst.parent.original if (inst.is_instance and inst.parent) else ob.original
        if owner.name not in names or owner.hide_render:
            continue
        m = inst.matrix_world
        for c in ob.bound_box:
            w = np.array(m @ Vector(c))
            lo = np.minimum(lo, w); hi = np.maximum(hi, w)
    if not np.isfinite(lo).all():
        lo = np.zeros(3); hi = np.zeros(3)
    return lo, hi



#: Flat stand-ins for MDL-only materials (the AEC brownstones bind
#: `Materials/Base/*.mdl` and nothing else), keyed on the material NAME.
#: Cycles cannot read MDL, so without this they render black. First match wins.
MDL_COLOURS = [
    ("Red_Clinker", (0.42, 0.18, 0.13)), ("Brick_Wall_Red", (0.45, 0.20, 0.15)),
    ("Brick", (0.50, 0.42, 0.36)), ("Default_Wall", (0.80, 0.76, 0.70)),
    ("Plaster", (0.80, 0.76, 0.70)), ("Concrete", (0.55, 0.53, 0.50)),
    ("Glass", (0.55, 0.70, 0.80)), ("Cherry", (0.35, 0.18, 0.10)),
    ("Door", (0.35, 0.22, 0.12)), ("Wood_Bark", (0.30, 0.22, 0.15)),
    ("Wood", (0.45, 0.30, 0.16)), ("Iron", (0.22, 0.22, 0.24)),
    ("Metal", (0.30, 0.30, 0.32)), ("Grass", (0.25, 0.40, 0.15)),
    ("Paint", (0.70, 0.70, 0.68)), ("Rubber", (0.12, 0.12, 0.12)),
    ("plastic", (0.85, 0.85, 0.85)), ("Plastic", (0.20, 0.20, 0.20)),
    ("Miscellaneous", (0.85, 0.85, 0.82)), ("Invalid", (0.90, 0.90, 0.90)),
    ("Light", (0.95, 0.85, 0.65)), ("Phase", (0.75, 0.75, 0.75)),
    ("Aluminum", (0.25, 0.25, 0.27)), ("Eggshell_White", (0.88, 0.88, 0.85)),
    ("Eggshell_Ash", (0.70, 0.70, 0.68)),
]


def approximate_mdl(objs):
    """Colour every material Blender imported as an EMPTY node tree — what an
    MDL-only binding becomes (pure black otherwise). Returns how many."""
    mats = {}
    for o in objs:
        if o.type == "MESH":
            for slot in o.material_slots:
                if slot.material is not None:
                    mats[slot.material.name] = slot.material
    if not mats:
        return 0
    n = 0
    for m in mats.values():
        # Only a material Blender could make NOTHING of: MDL-only binds import
        # as an empty node tree. Anything with nodes — textured or authored
        # flat colours — is rendered as imported.
        if m.use_nodes and len(m.node_tree.nodes) > 0:
            continue
        m.use_nodes = True
        nt = m.node_tree
        bsdf = next((nd for nd in nt.nodes if nd.type == "BSDF_PRINCIPLED"), None)
        if bsdf is None:
            nt.nodes.clear()
            bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
            outn = nt.nodes.new("ShaderNodeOutputMaterial")
            nt.links.new(bsdf.outputs["BSDF"], outn.inputs["Surface"])
        rgb, rough, metal = (0.62, 0.60, 0.58), 0.8, 0.0
        for key, c in sorted(MDL_COLOURS, key=lambda kc: -len(kc[0])):
            if key.lower() in m.name.lower():
                rgb = c
                rough = 0.15 if "Glass" in key else 0.8
                metal = 0.8 if key in ("Metal", "Iron", "Aluminum") else 0.0
                break
        bsdf.inputs["Base Color"].default_value = (*rgb, 1.0)
        bsdf.inputs["Roughness"].default_value = rough
        bsdf.inputs["Metallic"].default_value = metal
        n += 1
    return n


def import_building(path: Path, scale: float, name: str):
    before = set(bpy.data.objects)
    bpy.ops.wm.usd_import(filepath=str(path))
    new = [o for o in bpy.data.objects if o not in before]
    root = bpy.data.objects.new(name, None)
    bpy.context.scene.collection.objects.link(root)
    for o in new:
        if o.parent is None or o.parent not in new:
            o.parent = root
    k = approximate_mdl(new)
    if k:
        print(f"[gallery]   {name}: {k} MDL-only material(s) given flat colours", flush=True)
    lo, hi = bounds(new)
    if hi[2] - lo[2] > 400.0 and scale != 1.0:      # unit-less cm art
        root.scale = (scale,) * 3
        bpy.context.view_layer.update()
        lo, hi = bounds(new)
    # base on the ground, footprint centred on the ORIGIN. The root may sit
    # elsewhere (a Nucleus asset's mesh is offset inside its own hierarchy),
    # so `off` = bbox centre - root: everything that aims at or labels the
    # building uses root.location + off, never the root alone.
    root.location = Vector((-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]))
    bpy.context.view_layer.update()
    lo, hi = bounds(new)
    off = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0]) - np.array(root.location)
    root["gallery_off"] = [float(v) for v in off]
    return root, new, hi - lo


def setup_world():
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 48
    scene.cycles.use_denoising = True
    prefs = bpy.context.preferences.addons["cycles"].preferences
    for backend in ("OPTIX", "CUDA"):
        try:
            prefs.compute_device_type = backend
            prefs.get_devices()
        except Exception:
            continue
        if any(d.type == backend for d in prefs.devices):
            for d in prefs.devices:
                d.use = d.type == backend
            scene.cycles.device = "GPU"
            break
    world = bpy.data.worlds.new("gallery")
    world.use_nodes = True
    bg = world.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.62, 0.68, 0.78, 1.0)
    bg.inputs[1].default_value = 0.7
    scene.world = world
    sun = bpy.data.lights.new("sun", "SUN"); sun.energy = 3.5; sun.angle = math.radians(2)
    so = bpy.data.objects.new("sun", sun); so.rotation_euler = (math.radians(50), 0, math.radians(215))
    scene.collection.objects.link(so)
    fill = bpy.data.lights.new("fill", "SUN"); fill.energy = 1.0
    fo = bpy.data.objects.new("fill", fill); fo.rotation_euler = (math.radians(65), 0, math.radians(40))
    scene.collection.objects.link(fo)
    bpy.ops.mesh.primitive_plane_add(size=1.0)
    ground = bpy.context.active_object; ground.name = "ground"
    m = bpy.data.materials.new("ground"); m.use_nodes = True
    m.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.32, 0.32, 0.33, 1)
    m.node_tree.nodes["Principled BSDF"].inputs["Roughness"].default_value = 0.95
    ground.data.materials.append(m)
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    scene.collection.objects.link(cam); scene.camera = cam
    cam.data.clip_start = 0.5; cam.data.clip_end = 100000
    return ground, cam


def label(text: str, at, size: float):
    """A flat text object on the ground in front of *at*."""
    curve = bpy.data.curves.new(text, "FONT")
    curve.body = text; curve.size = size; curve.align_x = "CENTER"
    o = bpy.data.objects.new("lbl_" + text, curve)
    o.location = Vector((at[0], at[1], 0.05)); o.rotation_euler = (0, 0, 0)
    mat = bpy.data.materials.get("label") or bpy.data.materials.new("label")
    mat.use_nodes = True
    mat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.95, 0.95, 0.9, 1)
    o.data.materials.append(mat)
    bpy.context.scene.collection.objects.link(o)
    return o


def aim(cam, centre, radius, az_deg, el_deg, fov_deg=40.0, aspect=1.0):
    cam.data.angle = math.radians(fov_deg)
    # fit the bounding sphere in the NARROWER field
    half = math.radians(fov_deg) / 2
    if aspect < 1.0:
        half = math.atan(math.tan(half) * aspect)
    dist = radius / math.sin(half) * 1.05
    az, el = math.radians(az_deg), math.radians(el_deg)
    d = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    cam.location = Vector(centre) + d * dist
    cam.rotation_euler = (Vector(centre) - cam.location).to_track_quat("-Z", "Y").to_euler()


def aim_ortho(cam, centre, width, height, az_deg, el_deg, aspect):
    """Orthographic view of a *width* x *height* (projected) extent."""
    cam.data.type = "ORTHO"
    cam.data.ortho_scale = max(width, height * aspect)
    az, el = math.radians(az_deg), math.radians(el_deg)
    d = Vector((math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)))
    cam.location = Vector(centre) + d * 5000.0
    cam.rotation_euler = (Vector(centre) - cam.location).to_track_quat("-Z", "Y").to_euler()


def add_legend(png: Path, entries, cols: int = 4):
    """Append a numbered legend strip under *png* in place."""
    from PIL import Image, ImageDraw, ImageFont
    im = Image.open(png).convert("RGB")
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", max(14, im.width // 110))
    except OSError:
        font = ImageFont.load_default()
    rows = math.ceil(len(entries) / cols)
    lh = int(font.size * 1.6)
    strip = Image.new("RGB", (im.width, rows * lh + lh), (28, 28, 30))
    d = ImageDraw.Draw(strip)
    colw = im.width // cols
    for i, (num, name, size) in enumerate(entries):
        x = (i % cols) * colw + lh // 2
        y = (i // cols) * lh + lh // 2
        d.text((x, y), f"{num:>2}  {name}   {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} m", fill=(235, 235, 230), font=font)
    out = Image.new("RGB", (im.width, im.height + strip.height))
    out.paste(im, (0, 0)); out.paste(strip, (0, im.height))
    out.save(png)


def render(path: Path, w: int, h: int):
    s = bpy.context.scene
    s.render.resolution_x, s.render.resolution_y = w, h
    s.render.resolution_percentage = 100
    s.render.image_settings.file_format = "PNG"
    path.parent.mkdir(parents=True, exist_ok=True)
    s.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


def show_only(roots_visible, all_roots, extras=()):
    vis = set(id(r) for r in roots_visible)
    for r in all_roots:
        hide = id(r) not in vis
        r.hide_render = hide
        for c in r.children_recursive:
            c.hide_render = hide
    for o in extras:
        o.hide_render = False



def stitch_grids(out: Path, pack_info: dict = None, tile: int = 512, cols: int = 6):
    """Grids of the INDIVIDUAL hero renders — no scene re-render.

    `grids/<pool>.png` per pool and `grid_all.png` with one section per pool,
    each tile captioned with the name, condition, CONSTRUCTION MATERIAL and
    size. Condition and material come from *pack_info* (the pack, read fresh)
    so re-tagging an asset needs only a re-stitch; size comes from
    `individual/index.json`, which only a render can update.
    This is how the gallery is refreshed when one asset is added: render that
    one with `--only`, then re-stitch. The same-scene shots (`all_buildings.png`,
    `groups/`) are left as they are.
    """
    import json
    from PIL import Image, ImageDraw, ImageFont
    idx_path = out / "individual" / "index.json"
    index = json.load(open(idx_path)) if idx_path.exists() else {}
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", max(12, tile // 26))
        hfont = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", max(16, tile // 16))
    except OSError:
        font = hfont = ImageFont.load_default()
    cap = int(font.size * 2.6)
    info = pack_info or {}

    def _entry(index, pool, name):
        """(size, condition, material) for one render. Index entries are
        `{"size": [...], "condition": ...}`, or a bare size list if written
        before the condition became a tag; the PACK is preferred for the
        condition and is the only source for the material."""
        v = index.get(f"{pool}/{name}")
        size = v.get("size") if isinstance(v, dict) else v
        cond = (v.get("condition", "intact") if isinstance(v, dict) else "intact")
        pi = info.get(name)
        if pi:
            cond, mat = pi
        else:
            mat = None
        return size, cond, mat

    pools = {}
    for png in sorted((out / "individual").rglob("*.png")):
        if png.name.endswith("_views.png"):
            continue
        pool = "/".join(png.relative_to(out / "individual").parts[:-1])
        pools.setdefault(pool, []).append(png)
    order = ["highrise", "tower", "midrise", "lowrise", "townhouse", "rowhouse",
             # the pre-2026-08-26 condition-keyed folders, for a gallery that
             # has not been re-sorted yet
             "intact/tower", "intact/midrise", "intact/rowhouse",
             "damaged", "destroyed/tower", "destroyed/midrise"]
    pools = dict(sorted(pools.items(), key=lambda kv: (order.index(kv[0]) if kv[0] in order else 99, kv[0])))

    for pool in pools:
        pools[pool].sort(key=lambda p: (_entry(index, pool, p.stem)[1] != "intact",
                                        _entry(index, pool, p.stem)[1], p.stem))

    def section(pool, pngs):
        rows = math.ceil(len(pngs) / cols)
        im = Image.new("RGB", (cols * tile, rows * (tile + cap)), (30, 30, 32))
        d = ImageDraw.Draw(im)
        for i, png in enumerate(pngs):
            x, y = (i % cols) * tile, (i // cols) * (tile + cap)
            im.paste(Image.open(png).convert("RGB").resize((tile, tile)), (x, y))
            name = png.stem
            size, cond, mat = _entry(index, pool, name)
            label_ = name if cond == "intact" else f"{name}  [{cond}]"
            d.text((x + 6, y + tile + 4), label_,
                   fill=(235, 235, 230) if cond == "intact" else (245, 205, 150),
                   font=font)
            sub = f"{size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} m" if size else ""
            if mat:
                sub = f"{mat}" + (f"   {sub}" if sub else "")
            if sub:
                d.text((x + 6, y + tile + 4 + font.size + 4), sub,
                       fill=(150, 190, 205) if mat else (170, 170, 165), font=font)
        return im

    (out / "grids").mkdir(parents=True, exist_ok=True)
    sections = []
    for pool, pngs in pools.items():
        im = section(pool, pngs)
        im.save(out / "grids" / (pool.replace("/", "_") + ".png"))
        sections.append((pool, im))
    hh = int(hfont.size * 2.2)
    W = cols * tile
    H = sum(hh + im.height for _, im in sections)
    allim = Image.new("RGB", (W, H), (30, 30, 32))
    d = ImageDraw.Draw(allim)
    y = 0
    for pool, im in sections:
        d.text((10, y + hh // 4), f"{pool}  ({sum(1 for _ in pools[pool])})", fill=(255, 255, 255), font=hfont)
        y += hh
        allim.paste(im, (0, y)); y += im.height
    allim.save(out / "grid_all.png")
    print(f"[gallery] stitched {len(sections)} grid(s) + grid_all.png ({sum(len(v) for v in pools.values())} renders)")


def _pack_info(pack: str) -> dict:
    """`{asset name: (condition, material)}` straight from the pack file.

    Read at stitch time rather than baked into the index at render time, so a
    re-tagged or re-materialled asset shows correctly after a `--stitch`
    without re-rendering anything.
    """
    doc = yaml.safe_load(open(SCENE_GEN / "config" / "asset_packs" / f"{pack}.yaml"))
    out = {}
    for _pool_path, entries in walk_pools((doc.get("usds") or {}).get("buildings") or {}):
        for e in entries:
            usd = e["usd"] if isinstance(e, dict) else e
            name = Path(str(usd)).stem
            mat = e.get("material") if isinstance(e, dict) else None
            out[name] = (condition_of(e), mat)
    return out


def _index_update(out: Path, key: str, size, condition="intact"):
    """Record a rendered asset's size and condition for the grid captions."""
    import json
    p = out / "individual" / "index.json"
    p.parent.mkdir(parents=True, exist_ok=True)
    d = json.load(open(p)) if p.exists() else {}
    d[key] = {"size": [round(float(v), 1) for v in size], "condition": condition}
    json.dump(d, open(p, "w"), indent=1, sort_keys=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pack", default="urban_v2")
    ap.add_argument("--out", default="")
    ap.add_argument("--res", type=int, default=2048)
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--skip-individual", action="store_true",
                    help="only the group and all-buildings shots")
    ap.add_argument("--only", default="",
                    help="regex on asset names: import and render ONLY these "
                         "individually (no scene shots), then re-stitch the grids")
    ap.add_argument("--stitch", action="store_true",
                    help="only rebuild grids/ and grid_all.png from the "
                         "individual renders already on disk")
    args = ap.parse_args()
    out = Path(args.out or SCENE_GEN / "galleries" / args.pack)
    if args.stitch:
        stitch_grids(out, _pack_info(args.pack))
        return 0
    only = re.compile(args.only) if args.only else None
    pack = yaml.safe_load(open(SCENE_GEN / "config" / "asset_packs" / f"{args.pack}.yaml"))
    default_scale = float(pack.get("asset_scale", 1.0))

    bpy.ops.wm.read_factory_settings(use_empty=True)
    ground, cam = setup_world()
    bpy.context.scene.cycles.samples = args.samples

    # ---- import everything, one root per building -----------------------
    groups = []                         # [(pool_path, [(name, root, size)])]
    missing = []
    for pool_path, entries in walk_pools(pack["usds"]["buildings"]):
        items = []
        for e in entries:
            usd = e["usd"] if isinstance(e, dict) else e
            sc = float(e.get("scale", default_scale)) if isinstance(e, dict) else default_scale
            p = resolve(usd)
            name = Path(usd).stem
            if only is not None and not only.search(name):
                continue
            if p is None:
                missing.append((pool_path, usd)); continue
            print(f"[gallery] import {'/'.join(pool_path)}/{name}", flush=True)
            root, objs, size = import_building(p, sc, name)
            items.append((name, root, size, condition_of(e)))
        if items:
            groups.append((pool_path, items))
    all_roots = [it[1] for _, items in groups for it in items]
    for pp, usd in missing:
        print(f"[gallery] SKIPPED (not on host): {'/'.join(pp)}: {usd}")

    # ---- lay out: each pool a block of LINES of at most `per_line` buildings,
    # shortest line in front; pools front to back by height, so nothing hides
    # what stands behind it. Numbers on the ground, names in the legend. ----
    gap, per_line = 12.0, 8
    y = 0.0
    labels = []
    row_extent = []      # (pool_path, items, width, depth, y0, numbered, labels)
    groups.sort(key=lambda g: max(it[2][2] for it in g[1]))
    for pool_path, items in groups:
        items.sort(key=lambda t: t[2][2])                 # ascending height
        lines = [items[i:i + per_line] for i in range(0, len(items), per_line)]
        y0, width, numbered, row_labels = y, 0.0, [], []
        n = 0
        for line in lines:
            depth = max(it[2][1] for it in line)
            x = 0.0
            for name, root, size, _cond in line:
                n += 1
                root.location.x += x + size[0] / 2
                root.location.y += y + depth / 2 - size[1] / 2      # front edges aligned
                offx, offy = root["gallery_off"][0], root["gallery_off"][1]
                root.location.x -= offx
                root.location.y -= offy
                lab = label(str(n), (x + size[0] / 2, y - 7.0),
                            min(12.0, max(3.5, 0.4 * size[0])))
                labels.append(lab); row_labels.append(lab)
                numbered.append((n, name, size))
                x += size[0] + gap
            width = max(width, x - gap)
            y += depth + 28.0
        row_extent.append((pool_path, items, width, y - 28.0 - y0, y0, numbered, row_labels))
        y += 60.0
    bpy.context.view_layer.update()
    ground.scale = (20000, 20000, 1)

    # ---- individual --------------------------------------------------------
    for pool_path, items, _, _, _, _, _ in row_extent:
        for name, root, size, cond in items:
            if args.skip_individual:
                break
            show_only([root], all_roots)
            for l in labels: l.hide_render = True
            c = np.array(root.location) + np.array(root["gallery_off"]) + np.array([0, 0, size[2] / 2])
            r = float(np.linalg.norm(size) / 2)
            sub = out / "individual" / Path(*pool_path)
            # hero from the FRONT: terrace houses face -X, everything else +X
            aim(cam, c, r,
                215 if ("rowhouse" in pool_path
                        or "townhouse" in pool_path) else 35, 22)
            render(sub / f"{name}.png", args.res, args.res)
            tiles = []
            for k, az in enumerate((35, 125, 215, 305)):
                aim(cam, c, r, az, 18)
                fp = out / "_tiles" / f"{name}_{k}.png"
                render(fp, args.res // 2, args.res // 2); tiles.append(fp)
            from PIL import Image
            sheet = Image.new("RGB", (args.res, args.res))
            for k, fp in enumerate(tiles):
                sheet.paste(Image.open(fp), ((k % 2) * args.res // 2, (k // 2) * args.res // 2))
            sheet.save(sub / f"{name}_views.png")
            _index_update(out, f"{'/'.join(pool_path)}/{name}", size, cond)
            print(f"[gallery] {'/'.join(pool_path)}/{name}  {size.round(1)}  {cond}",
                  flush=True)
    if only is not None:
        stitch_grids(out, _pack_info(args.pack))
        return 0

    # ---- groups: one orthographic shot per pool block -----------------------
    aspect = 16 / 9
    for pool_path, items, width, depth, y0, numbered, row_labels in row_extent:
        show_only([it[1] for it in items], all_roots, extras=row_labels)
        h = max(it[2][2] for it in items)
        el = 30.0
        proj_h = h * math.cos(math.radians(el)) + (depth + 20.0) * math.sin(math.radians(el))
        c = (width / 2, y0 + depth / 2 - 6.0, h * 0.3)
        aim_ortho(cam, c, width * 1.08 + 16.0, proj_h * 1.12, 276, el, aspect)
        png = out / "groups" / ("_".join(pool_path) + ".png")
        render(png, int(args.res * 2), int(args.res * 2 / aspect))
        add_legend(png, numbered)
        print(f"[gallery] group {'/'.join(pool_path)}: {len(items)}", flush=True)

    # ---- everything: rows front to back, orthographic ----------------------
    show_only(all_roots, all_roots, extras=labels)
    W = max(w for _, _, w, _, _, _, _ in row_extent)
    H = y
    tallest = max(it[2][2] for _, items, _, _, _, _, _ in row_extent for it in items)
    el = 32.0
    proj_h = tallest * math.cos(math.radians(el)) + H * math.sin(math.radians(el))
    aspect = 16 / 9
    aim_ortho(cam, (W / 2, H / 2 - 30.0, tallest * 0.25), W * 1.06 + 20.0, proj_h * 1.08, 268, el, aspect)
    png = out / "all_buildings.png"
    render(png, args.res * 2, int(args.res * 2 / aspect))
    add_legend(png, [(f"{'/'.join(pp)} #{n}", name, size)
                     for pp, _, _, _, _, numbered, _ in row_extent for n, name, size in numbered], cols=3)
    print(f"[gallery] wrote -> {out}  ({len(all_roots)} buildings, {len(missing)} skipped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
