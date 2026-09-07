#!/usr/bin/env python3
"""Give each archetype a sidecar whose textures resolve outside Kit.

    AirStack/.venv/bin/python scene_gen/tools/localize_archetype_textures.py \\
        scene_gen/assets/archetypes_urban_v2/earthquake

Stage A records a texture where it found it, which for this pack is an
`omniverse://` URL. Only Kit can resolve that: plain `pxr` and Blender's USD
both see a path that does not exist, and Blender's importer silently drops the
image — so every preview render of a baked archetype comes out untextured grey,
which is the one state a look check cannot be run in.

WHAT IS WRITTEN, AND WHY IT IS NOT AN EDIT
------------------------------------------
`<name>.local.usda`, beside the archetype: a few hundred bytes that sublayer
the archetype and carry nothing but `over` opinions on the shader inputs whose
maps are mirrored on disk. A sublayer's own opinions are stronger than the
layer it pulls in, so opening the sidecar gives the same scene with local
textures, and opening the archetype gives exactly what Stage A baked.

The archetype is never touched. That matters: it is the artifact Stage B
references and the one that would be published back to Nucleus, where the
`omniverse://` paths are the CORRECT ones and a local `/home/...` path would be
broken for everyone else.

Textures with no mirror are left alone rather than dropped — a partly textured
preview is strictly more informative than a grey one. The NVIDIA base materials
(`omniverse://.../NVIDIA/Materials/...`) that the fracture cores are bound to
are usually in that class, and their absence is why a cut face may still render
flat while the building's own cladding does not.
"""
from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
for _p in (_SCENE_GEN, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import local_mirror as M                                        # noqa: E402

SUFFIX = ".local.usda"


def fixes_for(usd: str) -> list:
    """``[(attribute path, local file), ...]`` for one archetype."""
    from pxr import Usd, UsdShade, Sdf

    stage = Usd.Stage.Open(usd)
    if stage is None:
        return []
    out = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        for inp in UsdShade.Shader(prim).GetInputs():
            val = inp.Get()
            if not isinstance(val, Sdf.AssetPath) or not val.path:
                continue
            if "://" not in val.path:
                continue
            got = M.local_for(val.path)
            if got:
                out.append((inp.GetAttr().GetPath(), got))
    return out


def write_sidecar(usd: str, fixes: list) -> str:
    """Author `<name>.local.usda`. Returns its path, or "" if nothing to do."""
    from pxr import Sdf

    if not fixes:
        return ""
    dst = os.path.splitext(usd)[0] + SUFFIX
    if os.path.exists(dst):
        os.remove(dst)
    layer = Sdf.Layer.CreateNew(dst)
    # RELATIVE, so the pair can be moved or copied together.
    layer.subLayerPaths.append("./" + os.path.basename(usd))
    # STAGE METADATA IS ROOT-LAYER-ONLY, and the sidecar is now the root.
    # `upAxis` and `metersPerUnit` are NOT inherited from a sublayer, so a
    # sidecar without them silently falls back to USD's defaults — Y-up — and
    # every archetype rendered through it came out lying on its side, at the
    # right size, with no error anywhere. `defaultPrim` goes across for the
    # same reason: without it an importer picks its own root.
    src = Sdf.Layer.FindOrOpen(usd)
    if src is not None:
        for key in ("upAxis", "metersPerUnit", "defaultPrim",
                    "timeCodesPerSecond", "kilogramsPerUnit"):
            if src.pseudoRoot.HasInfo(key):
                layer.pseudoRoot.SetInfo(key, src.pseudoRoot.GetInfo(key))
    for attr_path, local in fixes:
        prim = Sdf.CreatePrimInLayer(layer, attr_path.GetPrimPath())
        prim.specifier = Sdf.SpecifierOver
        spec = prim.attributes.get(attr_path.name) or Sdf.AttributeSpec(
            prim, attr_path.name, Sdf.ValueTypeNames.Asset)
        spec.default = Sdf.AssetPath(local)
    layer.Save()
    return dst


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("library", help="a Stage A <root>/<disaster> directory")
    ap.add_argument("--force", action="store_true",
                    help="rewrite sidecars that already exist")
    args = ap.parse_args()

    root = os.path.abspath(args.library)
    names = sorted(f for f in os.listdir(root)
                   if f.endswith(".usd") and not f.endswith(SUFFIX))
    made = skipped = unresolved = 0
    for name in names:
        usd = os.path.join(root, name)
        dst = os.path.splitext(usd)[0] + SUFFIX
        if os.path.exists(dst) and not args.force:
            skipped += 1
            continue
        try:
            fixes = fixes_for(usd)
        except Exception as exc:                                # noqa: BLE001
            print(f"  ! {name}: {type(exc).__name__}: {exc}")
            continue
        if write_sidecar(usd, fixes):
            made += 1
            print(f"  {name}: {len(fixes)} texture(s) localised", flush=True)
        else:
            unresolved += 1
    print(f"\n{made} sidecar(s) written, {skipped} already present, "
          f"{unresolved} archetype(s) with nothing to localise")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
