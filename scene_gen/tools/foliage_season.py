#!/usr/bin/env python
"""
foliage_season.py — green / autumn / bare, decided from the leaf texture.

NEVER USE THIS ALONE. Run tools/material_binding.py FIRST.

This measures the colour of a texture FILE. It does not and cannot tell you
whether any mesh in the asset references that file. An asset whose leaf albedo
measures a perfect green renders WHITE if its geometry binds no material, and
this tool will happily call it GREEN. That mistake was made: trees were
recommended on hue alone and would have turned a whole park canopy white.

Order of operations, always:
    1. material_binding.py  — is the geometry bound at all, and what share of
                              its faces? Unbound is disqualifying, full stop.
    2. size, 3. point cost
    4. this tool — and only for the texture the BOUND material actually
       resolves to. For MDL materials the albedo is inside the .mdl, not in the
       USD, so read the mdl's texture_2d() path rather than trusting a filename.

Asset and texture NAMES do not say the season (Nucleus's UrbanWest oaks and
the Muyang trees are named identically whatever colour they are), so this reads
the actual leaf base-colour image off Nucleus and reports its mean hue. Runs in
the isaac-sim container behind SimulationApp, same as nucleus_catalogue.py.

    HUE   ~60-160 deg  green foliage
          ~15-55  deg  autumn (yellow/orange/red)
          <15 / >330   red autumn or bare bark

Reads a JSON list of USD urls from $JOBS, writes to $OUT.
"""

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})

import colorsys
import io
import json
import os

import carb
import omni.client
from PIL import Image
from pxr import Usd

OUT = os.environ.get("OUT", "/tmp/foliage.txt")
JOBS = os.environ.get("JOBS", "/tmp/leafjobs.json")
LEAFY = ("leaf", "leaves", "branch", "foliage", "tree")


def asset_paths(url):
    st = Usd.Stage.Open(url)
    if st is None:
        return []
    out = []
    for prim in st.Traverse():
        for attr in prim.GetAttributes():
            if attr.GetTypeName().type.typeName != "SdfAssetPath":
                continue
            v = attr.Get()
            if v is None:
                continue
            p = getattr(v, "resolvedPath", "") or getattr(v, "path", "")
            if not p.lower().endswith((".png", ".jpg", ".jpeg")):
                continue
            b = os.path.basename(p).lower()
            # Packs disagree on naming: UE exports say *_BaseColor.png, the
            # Quixel-style ones just T_Branch_02.png. Take any albedo-looking
            # leaf texture and exclude the non-colour channels by suffix.
            if not any(k in b for k in LEAFY):
                continue
            if any(k in b for k in ("_normal", "_opacity", "_roughness",
                                    "_mask", "_ao", "ambientocclusion",
                                    "_metallic", "_specular", "_height",
                                    "subsurface")):
                continue
            out.append(p)
    return sorted(set(out))


def stats(tex_url):
    res, _ver, content = omni.client.read_file(tex_url)
    if res != omni.client.Result.OK:
        return None
    im = Image.open(io.BytesIO(memoryview(content).tobytes()))
    im.thumbnail((128, 128))
    rgba = im.convert("RGBA")
    px = list(rgba.getdata())
    # Alpha-cut out the transparent card background; leaf atlases are mostly
    # empty otherwise and the mean reads as black.
    lit = [p for p in px if p[3] > 128 and (p[0] + p[1] + p[2]) > 45]
    if not lit:
        lit = [p for p in px if (p[0] + p[1] + p[2]) > 45]
    if not lit:
        return None
    n = len(lit)
    r = sum(p[0] for p in lit) / n / 255.0
    g = sum(p[1] for p in lit) / n / 255.0
    b = sum(p[2] for p in lit) / n / 255.0
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    return (int(r * 255), int(g * 255), int(b * 255), h * 360.0, s, v, n)


def dir_leaf_images(url, depth=0):
    """Leaf-ish images under a directory. Packs whose colour lives inside an
    .mdl expose nothing through the USD, so the texture folder is the only
    way in."""
    found = []
    res, entries = omni.client.list(url)
    if res != omni.client.Result.OK:
        return found
    for e in entries:
        name = e.relative_path
        low = name.lower()
        if e.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN:
            if depth < 2:
                found += dir_leaf_images(f"{url}{name}/", depth + 1)
        elif low.endswith((".png", ".jpg", ".jpeg")):
            if any(k in low for k in LEAFY) and not any(
                    k in low for k in ("_normal", "_opacity", "_roughness",
                                       "_mask", "_ao", "ambientocclusion",
                                       "_metallic", "_specular", "_height",
                                       "subsurface", "_orm", "_n.", "_m.")):
                found.append(url + name)
    return found


def main():
    root = os.environ.get("OMNI_SERVER", "").strip().strip('"').rstrip("/")
    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", root)
    urls = json.load(open(JOBS))
    with open(OUT, "w") as out:
        for u in urls:
            out.write(f"\n### {u}\n")
            try:
                paths = (dir_leaf_images(u) if u.endswith("/")
                         else asset_paths(u))
            except Exception as exc:
                out.write(f"  !! {type(exc).__name__}: {exc}\n")
                continue
            if not paths:
                out.write("  (no leaf basecolor texture found)\n")
            for p in paths:
                try:
                    s = stats(p)
                except Exception as exc:
                    out.write(f"  !! {os.path.basename(p)}: {exc}\n")
                    continue
                if s is None:
                    out.write(f"  ?? {os.path.basename(p)}: unreadable\n")
                    continue
                r, g, b, h, sat, val, n = s
                verdict = ("GREEN" if 60 <= h <= 170 else
                           "AUTUMN" if 10 <= h < 60 else
                           "RED/BARE")
                out.write(f"  {os.path.basename(p)[:70]:<70} "
                          f"rgb({r},{g},{b}) hue={h:6.1f} sat={sat:.2f} "
                          f"val={val:.2f} n={n}  {verdict}\n")
            out.flush()


main()
app.close()
