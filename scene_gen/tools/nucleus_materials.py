#!/usr/bin/env python3
"""nucleus_materials.py — preview Nucleus materials and textures, fast.

    # by keyword: a labelled contact sheet of everything that matches
    python3 tools/nucleus_materials.py concrete
    python3 tools/nucleus_materials.py wood planks --limit 40

    # by link: one material, or every material under a directory
    python3 tools/nucleus_materials.py omniverse://…/Base/Wood/Ash.mdl
    python3 tools/nucleus_materials.py omniverse://…/vMaterials_2/Stone/

    python3 tools/nucleus_materials.py --reindex        # refresh the search index

Runs on the host through `nucleus.py` — no Kit, no Isaac Sim container. Use the
3.11 interpreter the omni.client binding is built for:

    AirStack/.venv/bin/python scene_gen/tools/nucleus_materials.py …

WHY THIS IS FAST
----------------
It does not render anything. NVIDIA's material libraries ship a pre-rendered
256x256 preview next to every asset — `<dir>/.thumbs/256x256/<filename>.png`,
the same sphere render the Omniverse browser shows — so previewing a material
is one `read_file`, not an MDL compile or a Cycles job. Measured on this box, a
60-material sheet lands in a couple of seconds against the ~80 s a headless Kit
boot costs before it renders anything at all.

The other half is concurrency. `omni.client` reads are network-bound and the
round trip dominates, so they parallelise almost perfectly. Measured, 23
thumbnails off `Base/Wood`:

    1 worker    15.30 s
    8 workers    0.32 s
    16 workers   0.09 s

Directory listings get the same treatment: the crawl walks each depth level as
one parallel batch rather than one blocking `list` at a time.

WHEN THERE IS NO THUMBNAIL
--------------------------
Authored project material like `Projects/SEI-COA/StandaloneMaterials/` has
thumbnails for its *textures* but none for the `MI_*.usd` that binds them, so
the preview falls back a tier at a time and reports which tier it used:

    thumb     the pre-rendered sphere            (a real shaded render)
    image     the file is itself an image        (the texture as authored)
    texture   base-colour map pulled out of the material   (NOT a render)

Tier 3 reads the binding rather than guessing from filenames — `.usd` through
`pxr` (downloaded first; `Usd.Stage.Open` cannot open `omniverse://` on the
host, see `nucleus.py`), `.mdl` by pulling its `texture_2d()` arguments, since
an MDL keeps its albedo inside the module where the USD cannot see it. A tile
labelled `texture` is a flat map, not a shaded material — worth knowing before
you judge a material by it.

Anything that resolves to none of the three renders as a `no preview` tile
instead of being dropped, so a gap in the sheet is visible rather than silent.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import re
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nucleus                                                  # noqa: E402

import numpy as np                                               # noqa: E402
from PIL import Image, ImageDraw, ImageFont                      # noqa: E402

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_TOOLS)

#: Where materials live on the AirLab server. `--root` adds more.
ROOTS = (
    "/NVIDIA/Materials/Base",
    "/NVIDIA/Materials/vMaterials_2",
    "/Projects/SEI-COA/StandaloneMaterials",
)

MATERIAL_EXTS = {".mdl", ".usd", ".usda", ".usdc"}
IMAGE_EXTS = {".png", ".jpg", ".jpeg", ".tga", ".bmp"}

#: The sibling directory every NVIDIA material library keeps its previews in.
THUMB_DIR = "/.thumbs/256x256/"

INDEX = os.path.join(_SCENE_GEN, "assets", "nucleus_material_index.json")

#: Which of a material's textures is the one worth looking at.
_BASECOLOR = re.compile(r"base_?colou?r|albedo|diffuse", re.I)

#: Trailing tokens that mark a texture as a non-colour channel. These preview as
#: flat purple/yellow noise and crowd out real materials, so a keyword search
#: drops them unless `--maps` asks for them. Matched on the LAST token only, so
#: `Concrete_Rough_BaseColor.png` survives while `Concrete_Rough_ORM.png` does not.
_CHANNELS = {
    "n", "norm", "normal", "nrm", "orm", "rough", "roughness", "metal",
    "metallic", "metalness", "h", "height", "disp", "displacement", "ao",
    "occlusion", "opacity", "mask", "alpha", "spec", "specular", "emissive",
    "emission", "transmission", "bump", "gloss", "cavity", "curvature", "id",
}


#: Packed/utility maps, which name their channels instead of using one suffix:
#: `multi_moss.jpg`, `concrete_wall_aged_R_rough_G_ao_B_cavity.jpg`, `*_mask_*`.
_PACKED = re.compile(r"(^|_)multi_|_r_[a-z]+_g_|(^|_)mask(_|$)", re.I)


def is_channel_map(url: str) -> bool:
    """True for a texture that is a data channel rather than something to look at."""
    stem, ext = os.path.splitext(url.rsplit("/", 1)[-1])
    if ext.lower() not in IMAGE_EXTS:
        return False
    return bool(_PACKED.search(stem)) or stem.rpartition("_")[2].lower() in _CHANNELS


# ---------------------------------------------------------------------------
# crawl / index
# ---------------------------------------------------------------------------

def _safe_ls(url: str) -> list:
    try:
        return nucleus.ls(url)
    except Exception:                                            # noqa: BLE001
        return []


def crawl(roots, workers: int = 16, verbose: bool = False) -> tuple:
    """``(entries, thumbs)`` under *roots*, walked one parallel level at a time.

    `thumbs` is collected during the same walk so that previewing an indexed
    entry never has to probe for its thumbnail — it is a set lookup.
    """
    server = nucleus.server_root()
    entries, thumbs = set(), set()
    frontier = [server + r.rstrip("/") + "/" for r in roots]

    with ThreadPoolExecutor(workers) as pool:
        depth = 0
        while frontier:
            listings = list(pool.map(_safe_ls, frontier))
            nxt = []
            for base, listing in zip(frontier, listings):
                in_thumbs = ".thumbs/" in base
                for name, _size, is_dir in listing:
                    if is_dir:
                        # Inside `.thumbs` only the size we actually read.
                        if base.endswith(".thumbs/") and name != "256x256":
                            continue
                        nxt.append(base + name + "/")
                    elif os.path.splitext(name)[1].lower() in (
                            MATERIAL_EXTS | IMAGE_EXTS):
                        (thumbs if in_thumbs else entries).add(base + name)
            if verbose:
                print(f"  depth {depth}: {len(frontier)} dirs -> "
                      f"{len(entries)} files", file=sys.stderr)
            frontier, depth = nxt, depth + 1

    return sorted(entries), thumbs


def load_index(roots, workers: int, reindex: bool = False) -> tuple:
    """The cached crawl, rebuilding it when missing or when asked."""
    if not reindex and os.path.exists(INDEX):
        with open(INDEX) as fh:
            data = json.load(fh)
        if data.get("roots") == list(roots):
            age = (time.time() - data.get("built", 0)) / 86400
            print(f"[index] {len(data['entries'])} entries, "
                  f"{age:.1f} days old  ({INDEX})", file=sys.stderr)
            return data["entries"], set(data["thumbs"])

    print(f"[index] crawling {len(roots)} roots…", file=sys.stderr)
    t = time.time()
    entries, thumbs = crawl(roots, workers, verbose=True)
    os.makedirs(os.path.dirname(INDEX), exist_ok=True)
    with open(INDEX, "w") as fh:
        json.dump({"built": time.time(), "roots": list(roots),
                   "entries": entries, "thumbs": sorted(thumbs)}, fh)
    print(f"[index] {len(entries)} entries, {len(thumbs)} thumbnails "
          f"in {time.time() - t:.1f}s -> {INDEX}", file=sys.stderr)
    return entries, thumbs


def search(entries, terms, limit: int, maps: bool = False) -> list:
    """Entries matching every term. Real materials sort ahead of raw maps."""
    terms = [t.lower() for t in terms]
    hits = [u for u in entries if all(t in u.lower() for t in terms)]
    if not maps:
        hits = [u for u in hits if not is_channel_map(u)]
    hits.sort(key=lambda u: (os.path.splitext(u)[1].lower() in IMAGE_EXTS, u))
    return hits[:limit]


# ---------------------------------------------------------------------------
# preview resolution — cheapest tier first
# ---------------------------------------------------------------------------

def _decode(data: bytes, tile: int) -> Image.Image:
    im = Image.open(io.BytesIO(data))
    im.draft("RGB", (tile * 2, tile * 2))       # fast path for big JPEGs
    # 16-bit and float maps (`I;16`, `F`) must be scaled down by hand: PIL's
    # convert("RGB") saturates them, which renders the whole texture pure white
    # rather than failing — a silently wrong tile. Several vMaterials_2 height
    # and diffuse maps are 16-bit.
    if im.mode.startswith("I") or im.mode == "F":
        arr = np.asarray(im).astype("float32")
        if arr.max() > 255:
            arr *= 255.0 / arr.max()
        im = Image.fromarray(np.clip(arr, 0, 255).astype("uint8"))
    im = im.convert("RGB")
    # Scale to fit the tile in BOTH directions — `thumbnail` only shrinks, and a
    # 128px texture next to a 256px thumbnail leaves the grid ragged.
    scale = tile / max(im.size)
    return im.resize((max(1, round(im.width * scale)),
                      max(1, round(im.height * scale))), Image.LANCZOS)


def _usd_textures(url: str) -> list:
    """Texture asset paths bound anywhere in a USD material.

    Downloaded first: the host has no `omni.usd_resolver`, so `Usd.Stage.Open`
    cannot take an `omniverse://` path — see `nucleus.py`.
    """
    from pxr import Sdf, Usd, UsdShade

    fd, tmp = tempfile.mkstemp(suffix=os.path.splitext(url)[1] or ".usd")
    try:
        with os.fdopen(fd, "wb") as fh:
            fh.write(nucleus.read(url))
        stage = Usd.Stage.Open(tmp)
        if stage is None:
            return []
        out = []
        for prim in stage.Traverse():
            shader = UsdShade.Shader(prim)
            if not shader:
                continue
            for inp in shader.GetInputs():
                val = inp.Get()
                if isinstance(val, Sdf.AssetPath) and val.path:
                    out.append(val.path)
        return out
    finally:
        os.unlink(tmp)


def base_texture(url: str) -> str | None:
    """The base-colour map a material binds, as a full Nucleus URL."""
    ext = os.path.splitext(url)[1].lower()
    try:
        if ext == ".mdl":
            text = nucleus.read(url).decode("utf8", "replace")
            paths = re.findall(r'texture_2d\s*\(\s*"([^"]+)"', text)
        elif ext in MATERIAL_EXTS:
            paths = _usd_textures(url)
        else:
            return None
    except Exception:                                            # noqa: BLE001
        return None
    if not paths:
        return None
    best = next((p for p in paths if _BASECOLOR.search(p)), paths[0])
    if "://" in best:
        return best
    return f"{url.rsplit('/', 1)[0]}/{best.lstrip('./')}"


def preview(url: str, tile: int, thumbs: set | None = None) -> tuple:
    """``(image, tier)`` for one asset. Never raises."""
    directory, name = url.rsplit("/", 1)
    thumb = f"{directory}{THUMB_DIR}{name}.png"
    # An indexed crawl already knows whether the thumbnail exists; an ad-hoc
    # URL does not, so just try it — a miss costs one failed read.
    if thumbs is None or thumb in thumbs:
        try:
            return _decode(nucleus.read(thumb), tile), "thumb"
        except Exception:                                        # noqa: BLE001
            pass

    if os.path.splitext(name)[1].lower() in IMAGE_EXTS:
        try:
            return _decode(nucleus.read(url), tile), "image"
        except Exception:                                        # noqa: BLE001
            pass

    tex = base_texture(url)
    if tex:
        try:
            return _decode(nucleus.read(tex), tile), "texture"
        except Exception:                                        # noqa: BLE001
            pass

    return None, "missing"


# ---------------------------------------------------------------------------
# contact sheet
# ---------------------------------------------------------------------------

_FONTS = ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
          "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf")


def _font(size: int, bold: bool = False):
    try:
        return ImageFont.truetype(_FONTS[1 if bold else 0], size)
    except Exception:                                            # noqa: BLE001
        return ImageFont.load_default()


def _fit(text: str, font, width: int) -> str:
    """*text* shortened with an ellipsis until it fits *width*."""
    if font.getlength(text) <= width:
        return text
    while text and font.getlength(text + "…") > width:
        text = text[:-1]
    return text + "…"


def _wrap(text: str, font, width: int, max_lines: int) -> list:
    """Greedy character wrap — paths have few break points, so wrap anywhere."""
    lines, cur = [], ""
    for ch in text:
        if font.getlength(cur + ch) <= width:
            cur += ch
        else:
            lines.append(cur)
            cur = ch
            if len(lines) == max_lines:
                return lines[:-1] + [lines[-1][:-1] + "…"]
    if cur:
        lines.append(cur)
    return lines[:max_lines]


#: Tier -> the colour its caption is drawn in, so a sheet reads at a glance.
_TIER_COLOR = {"thumb": (110, 190, 130), "image": (120, 170, 220),
               "texture": (215, 165, 90), "missing": (150, 150, 156)}


def sheet(items, tile: int, cols: int | None, label_lines: int = 3):
    """A grid of ``(url, image, tier)``, each tile labelled with its path."""
    name_font, dir_font = _font(12, bold=True), _font(10)
    pad, line_h = 12, 13
    label_h = 6 + line_h * (1 + label_lines)

    cols = cols or max(1, min(len(items), round(len(items) ** 0.5 * 1.35)))
    rows = -(-len(items) // cols)
    cw, ch = tile + pad, tile + label_h + pad
    img = Image.new("RGB", (cols * cw + pad, rows * ch + pad), (22, 22, 25))
    draw = ImageDraw.Draw(img)
    server = nucleus.server_root()

    for i, (url, im, tier) in enumerate(items):
        x = pad + (i % cols) * cw
        y = pad + (i // cols) * ch
        draw.rectangle([x, y, x + tile, y + tile], fill=(38, 38, 42))
        if im is None:
            draw.text((x + tile // 2, y + tile // 2), "no preview",
                      fill=(120, 120, 126), font=dir_font, anchor="mm")
        else:
            img.paste(im, (x + (tile - im.width) // 2,
                           y + (tile - im.height) // 2))

        # `thumb` is the norm and needs no shouting; the other tiers are NOT a
        # shaded render, so badge those on the tile itself where they can't be
        # missed and can't collide with a long filename.
        if tier != "thumb":
            bw = dir_font.getlength(tier) + 8
            draw.rectangle([x, y, x + bw, y + 15], fill=(0, 0, 0))
            draw.text((x + 4, y + 3), tier, font=dir_font,
                      fill=_TIER_COLOR.get(tier, (220, 220, 226)))

        rel = url[len(server):].lstrip("/")
        head, _, base = rel.rpartition("/")
        ty = y + tile + 4
        draw.text((x, ty), _fit(base, name_font, tile), font=name_font,
                  fill=_TIER_COLOR.get(tier, (220, 220, 226)))
        for j, line in enumerate(_wrap(head, dir_font, tile, label_lines)):
            draw.text((x, ty + line_h * (j + 1)), line,
                      font=dir_font, fill=(140, 140, 148))

    return img


# ---------------------------------------------------------------------------

def resolve_targets(query, roots, workers, limit, reindex, maps):
    """``(urls, thumbs, slug)`` for either a link or a set of keywords."""
    if len(query) == 1 and "://" in query[0]:
        url = query[0].rstrip("/")
        if _safe_ls(url + "/"):                       # a directory: sheet it
            entries, thumbs = crawl([url[len(nucleus.server_root()):]],
                                    workers, verbose=True)
            if not maps:
                entries = [u for u in entries if not is_channel_map(u)]
            return entries[:limit], thumbs, os.path.basename(url)
        # A link to one file is shown whatever it is — no filtering.
        return [url], None, os.path.splitext(os.path.basename(url))[0]

    entries, thumbs = load_index(roots, workers, reindex)
    hits = search(entries, query, limit, maps)
    return hits, thumbs, "-".join(q.lower() for q in query)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("query", nargs="*",
                    help="keywords (AND-ed), or one omniverse:// URL")
    ap.add_argument("-o", "--out", help="output PNG "
                    "(default: galleries/materials/<query>.png)")
    ap.add_argument("--tile", type=int, default=256, help="tile size px")
    ap.add_argument("--cols", type=int, help="columns (default: near-square)")
    ap.add_argument("--limit", type=int, default=64, help="max materials")
    ap.add_argument("--workers", type=int, default=16, help="parallel reads")
    ap.add_argument("--root", action="append", default=[],
                    help="extra server-relative root to index")
    ap.add_argument("--maps", action="store_true",
                    help="also show normal/ORM/height channel maps")
    ap.add_argument("--reindex", action="store_true", help="rebuild the index")
    args = ap.parse_args()

    roots = list(ROOTS) + args.root
    nucleus.connect()          # load the runtime once, before any threads

    if args.reindex and not args.query:
        load_index(roots, args.workers, reindex=True)
        return 0
    if not args.query:
        ap.print_help()
        return 0

    t0 = time.time()
    urls, thumbs, slug = resolve_targets(
        args.query, roots, args.workers, args.limit, args.reindex, args.maps)
    if not urls:
        print(f"no materials matched {' '.join(args.query)!r}", file=sys.stderr)
        return 1

    # No size bump for a single link: the thumbnails are 256px, so enlarging the
    # tile just upscales one and makes the material look softer than it is.
    with ThreadPoolExecutor(args.workers) as pool:
        got = list(pool.map(lambda u: preview(u, args.tile, thumbs), urls))
    tile = args.tile
    items = [(u, im, tier) for u, (im, tier) in zip(urls, got)]

    out = args.out or os.path.join(_SCENE_GEN, "galleries", "materials",
                                   f"{slug or 'materials'}.png")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    sheet(items, tile, args.cols).save(out)

    tiers = {}
    for _u, _im, tier in items:
        tiers[tier] = tiers.get(tier, 0) + 1
    summary = ", ".join(f"{n} {k}" for k, n in sorted(tiers.items()))
    print(f"{len(items)} materials ({summary}) in {time.time() - t0:.1f}s")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
