#!/usr/bin/env python3
"""retint_archetypes.py — re-point a baked library's fracture cores at the
current `mesh_damage.CORE_LOOKS`, without re-cutting anything.

    python3 tools/retint_archetypes.py --disaster earthquake
    python3 tools/retint_archetypes.py --disaster earthquake --dry-run

WHY THIS IS NOT A RE-BAKE
-------------------------
A cut face's look is a MATERIAL, and `bake.export_object` writes it into every
archetype by value — so changing `CORE_LOOKS` fixes the live path and every
future bake and leaves the library on disk exactly as it was. Re-cutting 86
cells to change one colour is four hours to alter nine floats.

The geometry, the bindings and the settled poses are untouched here. All that
changes is the shader inputs on `/Baked/Looks/FractureCore_<kind>`, which is
the same prim `mesh_damage.core_material` would have authored had the look been
right at bake time.

WHAT IT WILL NOT FIX
--------------------
Anything baked into the GEOMETRY. `STRUCTURE_OF` decides the fragment shape and
the wall thickness as well as the core material, so a library cut as the wrong
MATERIAL is still wrong after this — the pieces are the wrong shape and no
amount of retinting changes that. This is for the look alone.
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)


def retint(path: str, looks: dict, dry: bool = False) -> dict:
    """Sync one archetype's cores. Returns ``{kind: (was, now)}`` for changes."""
    from pxr import Gf, Sdf, Usd, UsdShade

    stage = Usd.Stage.Open(path)
    if stage is None:
        return {}
    changed: dict = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if not name.startswith("FractureCore_"):
            continue
        kind = name[len("FractureCore_"):]
        look = looks.get(kind)
        if not look:
            continue
        for sub in Usd.PrimRange(prim):
            shader = UsdShade.Shader(sub)
            if not shader:
                continue
            inp = shader.GetInput("diffuse_color_constant")
            if not inp:
                continue
            was = inp.Get()
            now = Gf.Vec3f(*look["color"])
            if was is not None and Gf.IsClose(was, now, 1e-4):
                continue
            changed[kind] = (tuple(round(v, 3) for v in was) if was else None,
                             tuple(look["color"]))
            if not dry:
                inp.Set(now)
                r = shader.GetInput("reflection_roughness_constant")
                if r:
                    r.Set(float(look["roughness"]))
    if changed and not dry:
        stage.GetRootLayer().Save()
    return changed


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--disaster", default="earthquake")
    ap.add_argument("--root", default="",
                    help="library root (default: scene_gen/assets/archetypes)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    from disaster.mesh_damage import CORE_LOOKS

    root = args.root or os.path.join(_SCENE_GEN, "assets", "archetypes")
    d = os.path.join(root, args.disaster)
    files = sorted(glob.glob(os.path.join(d, "*.usd")))
    if not files:
        raise SystemExit(f"no archetypes in {d}")

    print(f"[retint] {len(files)} archetype(s) in {d}")
    for k, look in sorted(CORE_LOOKS.items()):
        print(f"[retint]   target {k:9s} -> {tuple(look['color'])}")

    n = 0
    seen: dict = {}
    for f in files:
        got = retint(f, CORE_LOOKS, args.dry_run)
        if got:
            n += 1
            seen.update(got)
    verb = "would change" if args.dry_run else "changed"
    print(f"[retint] {verb} {n}/{len(files)} archetype(s)")
    for k, (was, now) in sorted(seen.items()):
        print(f"[retint]   {k:9s} {was} -> {now}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
