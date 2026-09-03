#!/usr/bin/env python
"""Warm disaster-independent whole-building GAC slice caches.

Unlike a fire region slice, these records have ``signature=None`` and contain
no damage, soot, physics or disaster metadata.  Tornado and earthquake can
therefore load the same geometry through ``kit_bake.have_kit(name)``.
"""

import argparse
import importlib.util
import os
import sys
import time

SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if SCENE_GEN not in sys.path:
    sys.path.insert(0, SCENE_GEN)

from detail import kit_bake as kb  # noqa: E402


def _load_baker():
    path = os.path.join(SCENE_GEN, "tools", "bake_gac_kits.py")
    spec = importlib.util.spec_from_file_location("bake_gac_kits", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--assets", required=True, help="comma-separated GAC asset names")
    args = ap.parse_args()
    names = list(dict.fromkeys(x.strip() for x in args.assets.split(",") if x.strip()))
    baker = _load_baker()
    from disaster import fracture
    fracture.ensure_vtk(verbose=True)
    made = hit = failed = 0
    t0 = time.time()
    for name in names:
        if kb.have_kit(name, None):
            print("[canonical_kit] HIT %s" % name)
            hit += 1
            continue
        print("[canonical_kit] BUILD %s" % name)
        try:
            made += int(baker.bake_one(name, kb.KIT_DIR) is not None)
        except Exception as exc:
            failed += 1
            print("[canonical_kit] FAILED %s: %s" % (name, exc))
    print("canonical kits: %d built, %d cached, %d failed in %.1f s" %
          (made, hit, failed, time.time() - t0))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
