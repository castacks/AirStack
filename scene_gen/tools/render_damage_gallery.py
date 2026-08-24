#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["bpy", "pillow"]
# ///
"""Render the USDs `damage_gallery.py` wrote into a before/after contact sheet.

Rows are buildings, columns are disaster types, pristine on the far left. Reads
the manifest that tool emits; writes `sheet.png` beside it, plus the individual
tiles under `tiles/` so any one cell can be looked at full size.

Runs in an isolated Python 3.13 env via `uv run --script`, the same arrangement
`render_usd.py` uses and for the same reason: `bpy` ships only for 3.13 and
bundles its own USD, so it cannot share a process with the `usd-core` half of
the pipeline. Cycles renders on the GPU (OptiX here).

ONE CAMERA PER ROW, FRAMED ON THE PRISTINE CELL
-----------------------------------------------
The camera is computed once from the undamaged building and then reused for
every disaster in that row. Framing each cell on its own bounds instead makes
the sheet actively misleading: debris and thrown fragments widen the scene, the
camera pulls back to fit them, and the building shrinks — so a *more* damaged
cell renders a *smaller* building and the eye reads the difference as scale
rather than as damage. The frame is padded to leave room for the debris the
pristine cell does not have.

Usage
-----
    uv run --script render_damage_gallery.py galleries/damage/manifest.json
    ./render_damage_gallery.py .../manifest.json --res 640 --samples 96
    ./render_damage_gallery.py .../manifest.json --cpu
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import bpy
from mathutils import Vector

#: How much wider than the pristine building to frame, so a tornado's downwind
#: debris trail and an explosion's thrown fragments stay inside the tile.
FRAME_PAD = 1.55


# --------------------------------------------------------------------------- #
# engine
# --------------------------------------------------------------------------- #
def setup_engine(samples: int, use_gpu: bool) -> str:
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    scene.render.film_transparent = False
    if use_gpu:
        prefs = bpy.context.preferences.addons["cycles"].preferences
        for backend in ("OPTIX", "CUDA", "HIP", "METAL", "ONEAPI"):
            try:
                prefs.compute_device_type = backend
                prefs.get_devices()
            except Exception:
                continue
            gpus = [d for d in prefs.devices if d.type == backend]
            if gpus:
                for d in prefs.devices:
                    d.use = d.type == backend
                scene.cycles.device = "GPU"
                return f"GPU/{backend} ({gpus[0].name})"
    scene.cycles.device = "CPU"
    return "CPU"


# --------------------------------------------------------------------------- #
# scene assembly
# --------------------------------------------------------------------------- #
def import_usd(path: Path) -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(path))


def geometry_bounds():
    """World-space (center, radius) over vertices — not `bound_box`.

    `bound_box` is the *local* AABB, so transforming its corners on a tumbled
    fragment describes a box around a rotated box and reports geometry that is
    not there. On a fractured building that overstates the extent badly enough
    to frame the camera on empty ground.
    """
    mn = Vector((math.inf,) * 3)
    mx = Vector((-math.inf,) * 3)
    found = False
    for ob in bpy.context.scene.objects:
        if ob.type != "MESH" or not len(ob.data.vertices):
            continue
        for v in ob.data.vertices:
            p = ob.matrix_world @ v.co
            mn = Vector(map(min, mn, p))
            mx = Vector(map(max, mx, p))
        found = True
    if not found:
        return Vector((0, 0, 0)), 1.0
    return (mn + mx) * 0.5, max((mx - mn).length * 0.5, 1e-4)


def add_ground(center: Vector, radius: float) -> None:
    """A matte plane at z=0, so nothing floats and debris casts onto something.

    The generator places buildings with their base at z=0 and settles debris
    onto real ground; without a floor here the tiles read as models on a
    backdrop rather than as a wrecked lot.
    """
    bpy.ops.mesh.primitive_plane_add(size=radius * 40.0,
                                     location=(center.x, center.y, 0.0))
    ground = bpy.context.active_object
    ground.name = "gallery_ground"
    mat = bpy.data.materials.new("gallery_ground")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    if bsdf:
        bsdf.inputs["Base Color"].default_value = (0.19, 0.185, 0.17, 1.0)
        bsdf.inputs["Roughness"].default_value = 0.95
    ground.data.materials.append(mat)


def add_lighting(bg: float = 0.06) -> None:
    world = bpy.data.worlds.new("gw")
    world.use_nodes = True
    node = world.node_tree.nodes["Background"]
    node.inputs[0].default_value = (bg, bg, bg + 0.01, 1.0)
    node.inputs[1].default_value = 0.9
    bpy.context.scene.world = world

    def sun(name, rot, energy, angle=6.0):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(angle)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)

    # Low key from the camera side so damage relief reads; broad fill opposite
    # so the openings punched through walls are not solid black.
    sun("key", (math.radians(52), 0, math.radians(35)), 4.5)
    sun("fill", (math.radians(70), 0, math.radians(215)), 1.6)


def place_camera(center: Vector, radius: float, az: float, el: float,
                 fov: float = math.radians(40)):
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = fov
    cam_data.clip_start = max(radius * 0.005, 0.01)
    cam_data.clip_end = radius * 200
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)

    dist = radius / math.tan(fov * 0.5) * 1.15
    azr, elr = math.radians(az), math.radians(el)
    d = Vector((math.cos(elr) * math.cos(azr),
                math.cos(elr) * math.sin(azr),
                math.sin(elr)))
    cam.location = center + d * dist
    cam.rotation_euler = (center - cam.location).to_track_quat("-Z",
                                                              "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def render_to(path: Path, res: int) -> None:
    scene = bpy.context.scene
    scene.render.resolution_x = scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.render.render(write_still=True)


def apply_soot(soot: dict) -> int:
    """Re-apply the charring Blender's USD importer dropped. Returns how many.

    `scorch` authors soot as `inputs:scale` on the `UsdUVTexture` feeding
    albedo — the one place USD composition will consult, since the shader's
    `diffuseColor` is *connected* and a value there is ignored. Hydra and Isaac
    honour that scale. Blender's importer does not: it wires the image straight
    into Base Color and drops it, so a burnt-out building imports at full
    brightness and the fire column would show a fire that chars nothing.

    So the multiplier is carried through the manifest and spliced in here, as a
    MULTIPLY between the texture and the shader — which is exactly what the
    Blender prototype does, for exactly the same reason. Materials arrive named
    after their USD prim, with a `.001` suffix on collision.
    """
    if not soot:
        return 0
    done = 0
    for mat in bpy.data.materials:
        keep = soot.get(mat.name.split(".")[0])
        if keep is None or not mat.use_nodes or not mat.node_tree:
            continue
        bsdf = next((n for n in mat.node_tree.nodes
                     if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is None:
            continue
        sock = bsdf.inputs.get("Base Color")
        if sock is None:
            continue
        if not sock.is_linked:
            c = list(sock.default_value)
            sock.default_value = (c[0] * keep, c[1] * keep, c[2] * keep, c[3])
            done += 1
            continue
        tree = mat.node_tree
        src = sock.links[0].from_socket
        tint = (keep, keep, keep, 1.0)
        try:                       # legacy MixRGB, still present in Blender 5
            mix = tree.nodes.new("ShaderNodeMixRGB")
            mix.blend_type = "MULTIPLY"
            mix.inputs["Fac"].default_value = 1.0
            mix.inputs["Color2"].default_value = tint
            tree.links.new(src, mix.inputs["Color1"])
            tree.links.new(mix.outputs["Color"], sock)
        except RuntimeError:       # newer unified Mix node
            mix = tree.nodes.new("ShaderNodeMix")
            mix.data_type = "RGBA"
            mix.blend_type = "MULTIPLY"
            mix.inputs["Factor"].default_value = 1.0
            mix.inputs["B"].default_value = tint
            tree.links.new(src, mix.inputs["A"])
            tree.links.new(mix.outputs["Result"], sock)
        done += 1
    return done


def render_cell(usd: Path, out: Path, res: int, frame, az: float, el: float,
                soot: dict = None):
    """Render one tile. *frame* is (center, radius), or None to measure it."""
    import_usd(usd)
    apply_soot(soot or {})
    measured = geometry_bounds()
    center, radius = frame or (measured[0], measured[1] * FRAME_PAD)
    add_ground(center, radius)
    add_lighting()
    place_camera(center, radius, az, el)
    render_to(out, res)
    return center, radius


# --------------------------------------------------------------------------- #
# the sheet
# --------------------------------------------------------------------------- #
def _wrap(text: str, width: int) -> list:
    """Greedy word wrap, so a long asset name survives the row gutter."""
    lines, cur = [], ""
    for word in str(text).split():
        cand = f"{cur} {word}".strip()
        if len(cand) <= width or not cur:
            cur = cand
        else:
            lines.append(cur)
            cur = word
    if cur:
        lines.append(cur)
    return lines[:3]


def compose(manifest: dict, tiles: dict, out: Path, title: str) -> None:
    from PIL import Image, ImageDraw, ImageFont

    cols = manifest["columns"]
    rows = manifest["rows"]
    tw, th = Image.open(next(iter(tiles.values()))).size
    pad, head, gutter, top = 8, 34, 250, 64

    def font(size, bold=True):
        for name in (f"DejaVuSans{'-Bold' if bold else ''}.ttf",
                     "LiberationSans-Regular.ttf"):
            try:
                return ImageFont.truetype(name, size)
            except Exception:
                continue
        return ImageFont.load_default()

    W = gutter + len(cols) * (tw + pad) + pad
    H = top + head + len(rows) * (th + pad) + pad
    sheet = Image.new("RGB", (W, H), (17, 17, 16))
    draw = ImageDraw.Draw(sheet)
    draw.text((pad + 6, 14), manifest.get("title", title),
              fill=(238, 238, 232), font=font(26))
    draw.text((pad + 6, 42), manifest.get("subtitle", ""),
              fill=(140, 140, 134), font=font(14, False))

    # Three headings, because a column is one of three things and the pairing
    # is unreadable if they all look alike: the pristine reference, a
    # debris-free twin, and the full cell. The twin is deliberately the cooler
    # colour — it is the control, not the result.
    bare_suffix = manifest.get("bare_suffix") or ""
    for c, col in enumerate(cols):
        x = gutter + c * (tw + pad)
        if col == "pristine":
            accent = (235, 235, 228)
        elif bare_suffix and col.endswith(bare_suffix):
            accent = (128, 176, 208)
        else:
            accent = (232, 168, 96)
        draw.text((x + 4, top + 8), col.upper(), fill=accent, font=font(17))

    for r, row in enumerate(rows):
        y = top + head + r * (th + pad)
        # Wrap rather than clip: these names are asset titles scraped from
        # the set ("Bungalow The Gilchrist") and the tail is what tells two
        # of them apart.
        for k, line in enumerate(_wrap(row["name"], 22)):
            draw.text((pad + 6, y + 8 + k * 19), line, fill=(226, 226, 220),
                      font=font(15))
        for c, col in enumerate(cols):
            x = gutter + c * (tw + pad)
            key = (r, col)
            if key in tiles:
                sheet.paste(Image.open(tiles[key]).convert("RGB"), (x, y))
            st = row["stats"].get(col) or {}
            # A caller that already knows what its cell should say puts it in
            # `note`. `damage_spread.py` reports cells/loose/seconds; the
            # debris counts below mean nothing there.
            if st.get("note") and col != "pristine":
                note = st["note"]
            elif col == "pristine":
                note = (f"{st.get('footprint_m', ['?', '?'])[0]}"
                        f"x{st.get('footprint_m', ['?', '?'])[1]} m, "
                        f"h {st.get('height_m', '?')} m")
            else:
                note = (f"{st.get('fragments', 0)} frag · "
                        f"{st.get('debris', 0)}+{st.get('debris_piles', 0)} "
                        f"debris")
            draw.text((x + 6, y + th - 19), note, fill=(206, 206, 198),
                      font=font(13, False))

    sheet.save(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("manifest")
    ap.add_argument("--res", type=int, default=520)
    ap.add_argument("--samples", type=int, default=48)
    ap.add_argument("--az", type=float, default=38.0)
    ap.add_argument("--el", type=float, default=20.0)
    ap.add_argument("--cpu", action="store_true")
    args = ap.parse_args()

    mpath = Path(args.manifest).resolve()
    manifest = json.loads(mpath.read_text())
    root = mpath.parent
    device = setup_engine(args.samples, not args.cpu)
    print(f"[render] {device}  {args.res}px  {args.samples} samples")

    tiles: dict = {}
    t0 = time.time()
    for r, row in enumerate(manifest["rows"]):
        frame = None
        for col in manifest["columns"]:
            rel = row["cells"].get(col)
            if not rel:
                continue
            usd = root / rel
            if not usd.exists():
                print(f"  ! missing {usd}")
                continue
            out = root / "tiles" / f"{r:02d}_{row['name'][:20]}" / f"{col}.png"
            # The pristine column is first, and its frame is kept for the rest
            # of the row — see the note at the top.
            center, radius = render_cell(
                usd, out, args.res, frame, args.az, args.el,
                soot=(row["stats"].get(col) or {}).get("soot"))
            if frame is None:
                frame = (center, radius)
            tiles[(r, col)] = out
            print(f"  [{r}] {row['name'][:22]:22s} {col:11s} "
                  f"{time.time() - t0:6.1f}s")

    if not tiles:
        print("[render] nothing rendered", file=sys.stderr)
        return 1
    sheet = root / "sheet.png"
    compose(manifest, tiles, sheet, "Procedural building damage")
    print(f"[render] {len(tiles)} tiles in {time.time() - t0:.1f}s -> {sheet}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
