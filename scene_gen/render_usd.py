#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.13"
# dependencies = ["bpy", "pillow"]
# ///
"""View USD files by rendering them to preview images (and optional turntables).

There is no interactive USD viewer in the pip `bpy`/`usd-core` wheels (usdview
ships only with a full source build of USD), so this "viewer" renders each USD
from several angles into a contact sheet you can open in any image viewer, plus
a combined `index.png` of all models. Rendering uses headless Blender/Cycles on
the GPU (OptiX on this box), which imports USD natively and honors its
materials and textures.

Runs in an isolated Python 3.13 env via `uv run --script`, so it never touches
this repo's 3.10 environment.

The point of rendering the **USD** rather than the source mesh is that it is
the only check that materials and textures actually survived
`convert_to_usd.py` — a converted asset with lost textures still loads in Isaac
Sim, it just shows up as untextured grey.

Usage
-----
    uv run --script view_usd.py city_usds/                 # all USDs under a dir
    uv run --script view_usd.py model.usdc -o previews     # a single file
    uv run --script view_usd.py city_usds/ --turntable     # + spinning GIFs
    ./view_usd.py city_usds/ --views 4 --res 768           # shebang works too

Options
-------
    -o, --out DIR     where previews go            (default: ./usd_previews)
    --views N         orbit angles per model        (default: 6)
    --elev DEG        camera elevation              (default: 20)
    --res PX          square render resolution      (default: 640)
    --samples N       Cycles samples (quality)      (default: 48)
    --bg F            background gray 0..1           (default: 0.16)
    --turntable       also render a spinning GIF per model
    --frames N        turntable frame count         (default: 24)
    --cpu             force CPU rendering (default: GPU if available)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import bpy
from mathutils import Vector

USD_EXTS = {".usd", ".usda", ".usdc", ".usdz"}


# --------------------------------------------------------------------------- #
# render engine / device
# --------------------------------------------------------------------------- #
def setup_engine(samples: int, use_gpu: bool) -> str:
    scene = bpy.context.scene
    scene.render.engine = "CYCLES"
    scene.cycles.samples = samples
    scene.cycles.use_denoising = True
    device = "CPU"
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
                device = f"GPU/{backend} ({gpus[0].name})"
                break
    if device == "CPU":
        scene.cycles.device = "CPU"
    return device


# --------------------------------------------------------------------------- #
# scene assembly
# --------------------------------------------------------------------------- #
def import_usd(path: Path) -> None:
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.wm.usd_import(filepath=str(path))


def scene_bounds() -> tuple[Vector, float]:
    """World-space center and bounding-sphere radius of all mesh geometry."""
    mn = Vector((math.inf,) * 3)
    mx = Vector((-math.inf,) * 3)
    found = False
    for ob in bpy.context.scene.objects:
        if ob.type not in {"MESH", "CURVE", "SURFACE"}:
            continue
        for corner in ob.bound_box:
            p = ob.matrix_world @ Vector(corner)
            mn = Vector(map(min, mn, p))
            mx = Vector(map(max, mx, p))
            found = True
    if not found:
        raise RuntimeError("no renderable geometry in USD")
    center = (mn + mx) * 0.5
    radius = max((mx - mn).length * 0.5, 1e-4)
    return center, radius


def add_lighting(bg: float) -> None:
    world = bpy.data.worlds.new("vw")
    world.use_nodes = True
    bg_node = world.node_tree.nodes["Background"]
    bg_node.inputs[0].default_value = (bg, bg, bg, 1.0)
    bg_node.inputs[1].default_value = 0.6  # ambient strength
    bpy.context.scene.world = world

    def sun(name, rot, energy):
        d = bpy.data.lights.new(name, "SUN")
        d.energy = energy
        d.angle = math.radians(3)
        o = bpy.data.objects.new(name, d)
        o.rotation_euler = rot
        bpy.context.collection.objects.link(o)

    sun("key", (math.radians(55), 0, math.radians(40)), 4.0)
    sun("fill", (math.radians(65), 0, math.radians(210)), 1.5)


def place_camera(center: Vector, radius: float, az: float, el: float,
                 fov: float = math.radians(42)) -> bpy.types.Object:
    cam_data = bpy.data.cameras.new("cam")
    cam_data.angle = fov
    cam_data.clip_start = radius * 0.01
    cam_data.clip_end = radius * 100
    cam = bpy.data.objects.new("cam", cam_data)
    bpy.context.collection.objects.link(cam)

    dist = radius / math.tan(fov * 0.5) * 1.25
    azr, elr = math.radians(az), math.radians(el)
    direction = Vector((math.cos(elr) * math.cos(azr),
                        math.cos(elr) * math.sin(azr),
                        math.sin(elr)))
    cam.location = center + direction * dist
    cam.rotation_euler = (center - cam.location).to_track_quat("-Z", "Y").to_euler()
    bpy.context.scene.camera = cam
    return cam


def render_to(path: Path, res: int) -> None:
    scene = bpy.context.scene
    scene.render.resolution_x = scene.render.resolution_y = res
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = str(path)
    bpy.ops.render.render(write_still=True)


# --------------------------------------------------------------------------- #
# image assembly (PIL)
# --------------------------------------------------------------------------- #
def contact_sheet(tiles: list[Path], out: Path, title: str) -> None:
    from PIL import Image, ImageDraw, ImageFont

    imgs = [Image.open(t).convert("RGB") for t in tiles]
    n = len(imgs)
    cols = min(3, n)
    rows = math.ceil(n / cols)
    tw, th = imgs[0].size
    pad, header = 10, 46
    W = cols * tw + (cols + 1) * pad
    H = header + rows * th + (rows + 1) * pad
    sheet = Image.new("RGB", (W, H), (20, 20, 19))
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 24)
    except Exception:
        font = ImageFont.load_default()
    draw.text((pad, 12), title, fill=(235, 235, 230), font=font)
    for i, im in enumerate(imgs):
        r, c = divmod(i, cols)
        x = pad + c * (tw + pad)
        y = header + pad + r * (th + pad)
        sheet.paste(im, (x, y))
    sheet.save(out)


def turntable_gif(frames: list[Path], out: Path, ms: int = 80) -> None:
    from PIL import Image

    imgs = [Image.open(f).convert("RGB") for f in frames]
    imgs[0].save(out, save_all=True, append_images=imgs[1:], loop=0,
                 duration=ms, optimize=True)


def index_grid(heroes: list[tuple[str, Path]], out: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont

    if not heroes:
        return
    imgs = [(name, Image.open(p).convert("RGB")) for name, p in heroes]
    cols = min(3, len(imgs))
    rows = math.ceil(len(imgs) / cols)
    tw, th = imgs[0][1].size
    pad, header, cap = 12, 52, 30
    W = cols * tw + (cols + 1) * pad
    H = header + rows * (th + cap) + (rows + 1) * pad
    grid = Image.new("RGB", (W, H), (20, 20, 19))
    draw = ImageDraw.Draw(grid)
    try:
        tfont = ImageFont.truetype("DejaVuSans-Bold.ttf", 26)
        cfont = ImageFont.truetype("DejaVuSans.ttf", 18)
    except Exception:
        tfont = cfont = ImageFont.load_default()
    draw.text((pad, 14), f"USD preview index — {len(imgs)} models", fill=(235, 235, 230), font=tfont)
    for i, (name, im) in enumerate(imgs):
        r, c = divmod(i, cols)
        x = pad + c * (tw + pad)
        y = header + pad + r * (th + cap + pad)
        grid.paste(im, (x, y))
        draw.text((x + 4, y + th + 4), name, fill=(200, 200, 195), font=cfont)
    grid.save(out)


# --------------------------------------------------------------------------- #
def gather_usds(inputs: list[str]) -> list[Path]:
    files: list[Path] = []
    for raw in inputs:
        p = Path(raw).expanduser()
        if p.is_dir():
            files += sorted(f for f in p.rglob("*") if f.suffix.lower() in USD_EXTS)
        elif p.suffix.lower() in USD_EXTS:
            files.append(p)
        else:
            print(f"skip (not USD): {p}", file=sys.stderr)
    return files


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Render USD files to preview images via headless Blender.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    ap.add_argument("inputs", nargs="+", help="USD files and/or directories")
    ap.add_argument("-o", "--out", default="usd_previews", help="output directory")
    ap.add_argument("--views", type=int, default=6, help="orbit angles per model")
    ap.add_argument("--elev", type=float, default=20, help="camera elevation (deg)")
    ap.add_argument("--res", type=int, default=640, help="square render resolution")
    ap.add_argument("--samples", type=int, default=48, help="Cycles samples")
    ap.add_argument("--bg", type=float, default=0.16, help="background gray 0..1")
    ap.add_argument("--turntable", action="store_true", help="also render a spin GIF")
    ap.add_argument("--frames", type=int, default=24, help="turntable frame count")
    ap.add_argument("--cpu", action="store_true", help="force CPU rendering")
    args = ap.parse_args()

    usds = gather_usds(args.inputs)
    if not usds:
        print("No USD files found (.usd/.usda/.usdc/.usdz).", file=sys.stderr)
        return 1

    out_dir = Path(args.out).expanduser()
    (out_dir / "_frames").mkdir(parents=True, exist_ok=True)

    device = setup_engine(args.samples, use_gpu=not args.cpu)
    print(f"Rendering {len(usds)} USD(s) on {device}\n")

    heroes: list[tuple[str, Path]] = []
    ok = failed = 0
    t0 = time.time()

    for i, usd in enumerate(usds, 1):
        name = usd.stem
        tag = f"[{i}/{len(usds)}] {name}"
        try:
            import_usd(usd)
            center, radius = scene_bounds()
            add_lighting(args.bg)

            tiles: list[Path] = []
            for k in range(args.views):
                az = 360.0 * k / args.views + 25.0
                place_camera(center, radius, az, args.elev)
                fp = out_dir / "_frames" / f"{name}_v{k:02d}.png"
                render_to(fp, args.res)
                tiles.append(fp)

            sheet = out_dir / f"{name}_preview.png"
            contact_sheet(tiles, sheet, name)
            heroes.append((name, tiles[0]))

            if args.turntable:
                tframes: list[Path] = []
                for k in range(args.frames):
                    place_camera(center, radius, 360.0 * k / args.frames, args.elev)
                    fp = out_dir / "_frames" / f"{name}_t{k:02d}.png"
                    render_to(fp, args.res)
                    tframes.append(fp)
                turntable_gif(tframes, out_dir / f"{name}_turntable.gif")

            extra = " + turntable" if args.turntable else ""
            print(f"{tag} -> {sheet.name}{extra}")
            ok += 1
        except Exception as e:  # noqa: BLE001
            print(f"{tag} -> FAILED: {type(e).__name__}: {e}", file=sys.stderr)
            failed += 1

    if heroes:
        index_grid(heroes, out_dir / "index.png")

    dt = time.time() - t0
    print(f"\nDone in {dt:.1f}s — {ok} rendered, {failed} failed. Output: {out_dir}/")
    if heroes:
        print(f"Open {out_dir}/index.png for an overview, or *_preview.png per model.")
    return 1 if failed and ok == 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
