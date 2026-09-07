#!/usr/bin/env python3
"""Rewrite baked-in absolute asset paths so an archetype loads on both sides.

    AirStack/.venv/bin/python scene_gen/tools/portable_archetype_paths.py \\
        scene_gen/assets/archetypes_urban_v2/earthquake --audit
    ... same without --audit to rewrite in place

An archetype baked on the HOST records where its materials resolved THERE:

    /home/<user>/coasei/AirStack/.venv/lib/python3.11/site-packages/isaacsim/
        kit/mdl/core/Base/OmniPBR.mdl
    /home/<user>/coasei/AirStack/scene_gen/assets/materials/megascans/...jpg

Neither path exists inside the isaac-sim container, where the repo is bind
mounted at `/isaac-sim/AirStack`. Kit turns the absolute MDL path into a module
name (`::Z73file_3A::home::...`), fails to load it, and every FRACTURE CORE —
the cut face of every fragment — falls back to the missing-MDL colour: a city
of bright red confetti with perfect intact shells beside it. Measured
2026-08-28 on `archetypes_urban_v2`: 113 of 146 archetypes affected.

`disaster/bake.py` no longer writes these (see `_portable_asset`). This is the
repair for libraries baked before that, and it is worth having separately
because the fix is a few hundred bytes of material binding per file — re-baking
the same 146 archetypes would be another 1.9 hours of fracture for geometry
that is already correct.

THE MIRROR PROBLEM IS A DIFFERENT TOOL. `tools/localize_archetype_textures.py`
handles `omniverse://` paths that only Kit can read; this handles repo paths
that only the host can read. They are not interchangeable.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_REPO = os.path.dirname(_SCENE_GEN)


def _portable(raw: str, out_dir: str):
    """The portable form of *raw*, or None if it is already fine."""
    if not raw or "://" in raw:
        return None
    low = raw.lower()
    # Kit's own MDL library: the module name alone is what travels.
    if low.endswith(".mdl") and ("/" in raw or "\\" in raw):
        return os.path.basename(raw)
    if not os.path.isabs(raw):
        return None
    if not os.path.abspath(raw).startswith(_REPO + os.sep):
        return None
    return os.path.relpath(os.path.abspath(raw), out_dir).replace(os.sep, "/")


def fix_stage(path: str, dry: bool) -> tuple:
    """``(n_fixed, [examples])`` for one archetype."""
    from pxr import Usd, UsdShade, Sdf

    stage = Usd.Stage.Open(path)
    if stage is None:
        return 0, []
    out_dir = os.path.dirname(os.path.abspath(path))
    fixed, shown = 0, []
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        shader = UsdShade.Shader(prim)
        src = shader.GetSourceAsset("mdl")
        if src:
            got = _portable(str(src.path), out_dir)
            if got and got != str(src.path):
                fixed += 1
                if len(shown) < 2:
                    shown.append(f"{src.path} -> {got}")
                if not dry:
                    shader.SetSourceAsset(Sdf.AssetPath(got), "mdl")
        for inp in shader.GetInputs():
            val = inp.Get()
            if not isinstance(val, Sdf.AssetPath):
                continue
            got = _portable(str(val.path), out_dir)
            if got and got != str(val.path):
                fixed += 1
                if len(shown) < 2:
                    shown.append(f"{os.path.basename(val.path)} -> {got}")
                if not dry:
                    inp.Set(Sdf.AssetPath(got))
    if fixed and not dry:
        stage.GetRootLayer().Save()
    return fixed, shown


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("library", help="a Stage A <root>/<disaster> directory")
    ap.add_argument("--audit", action="store_true",
                    help="report what would change and write nothing")
    args = ap.parse_args()

    files = sorted(glob.glob(os.path.join(os.path.abspath(args.library),
                                          "*.usd")))
    if not files:
        print(f"no archetypes under {args.library}", file=sys.stderr)
        return 1
    touched = total = 0
    for f in files:
        n, shown = fix_stage(f, args.audit)
        if n:
            touched += 1
            total += n
            print(f"  {os.path.basename(f):46} {n:3d}  {'; '.join(shown)}"[:150])
    verb = "would fix" if args.audit else "fixed"
    print(f"\n{verb} {total} path(s) across {touched} of {len(files)} archetypes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
