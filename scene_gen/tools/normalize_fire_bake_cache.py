#!/usr/bin/env python3
"""Normalize already-built fire bake layers to portable Nucleus asset paths.

This is a cache migration, not a rebake: geometry, damage, physics transforms,
textures and sidecars are unchanged. Only authored asset paths under the
one-for-one ``scene_gen/assets`` mirror (plus malformed single-slash Omniverse
URLs) are rewritten. Use ``--backup-dir`` for recoverable in-place migration.
"""

import argparse
import glob
import os
import shutil
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN = os.path.dirname(HERE)
if SCENE_GEN not in sys.path:
    sys.path.insert(0, SCENE_GEN)

from disaster import fire_bake as fb  # noqa: E402


def _bad_paths(layer):
    """Return paths the same all-fields USD walker still considers unsafe."""
    from pxr import UsdUtils

    bad = []

    def _inspect(path):
        if path.startswith(fb.SHARED_ASSET_LOCAL_PREFIX):
            bad.append(path)
        elif path.startswith(fb.SHARED_KIT_LEGACY_MIRROR):
            bad.append(path)
        elif path.startswith("omniverse:/") and not path.startswith("omniverse://"):
            bad.append(path)
        return path

    UsdUtils.ModifyAssetPaths(layer, _inspect)
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cache_dir")
    ap.add_argument("--backup-dir", default="")
    args = ap.parse_args()

    from pxr import Sdf

    files = sorted(glob.glob(os.path.join(args.cache_dir, "*.usd")))
    if not files:
        raise SystemExit("no .usd bakes in " + args.cache_dir)
    if args.backup_dir:
        os.makedirs(args.backup_dir, exist_ok=True)

    changed_files = changed_paths = 0
    for path in files:
        layer = Sdf.Layer.FindOrOpen(path)
        if layer is None:
            raise RuntimeError("cannot open " + path)
        before = _bad_paths(layer)
        if not before:
            continue
        if args.backup_dir:
            backup = os.path.join(args.backup_dir, os.path.basename(path))
            if not os.path.exists(backup):
                shutil.copy2(path, backup)
        n = fb.rewrite_shared_asset_paths(layer, verbose=False)
        if not layer.Save():
            raise RuntimeError("failed to save " + path)
        # Reopen from disk; a clean in-memory layer is not evidence that the
        # crate actually serialized the overrides.
        layer.Reload()
        after = _bad_paths(layer)
        if after:
            raise RuntimeError("{0}: {1} unsafe path(s) survived, e.g. {2}"
                               .format(path, len(after), after[0]))
        changed_files += 1
        changed_paths += n
        print("portable {0}: {1} path(s)".format(os.path.basename(path), n))

    print("portable fire cache: {0} file(s), {1} path(s), {2} total bake(s)"
          .format(changed_files, changed_paths, len(files)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
