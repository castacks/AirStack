#!/usr/bin/env python3
"""Normalize reusable earthquake archetypes to portable Nucleus paths.

This is a cache migration, not a rebake: mesh points, damage, settled poses,
bindings and sidecars are unchanged.  Only asset-valued fields and composition
arcs are rewritten.  Run with Kit's USD Python so Omniverse URLs are supported.

    bash scene_gen/tools/usd_python.sh \
      scene_gen/tools/normalize_quake_archetypes.py \
      scene_gen/assets/archetype --names-file /tmp/changed.txt \
      --backup-dir /tmp/quake-archetypes-before-portable
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

from disaster import bake  # noqa: E402


def _unsafe_paths(layer):
    """Return authored paths that cannot resolve from the Nucleus layer."""
    from pxr import UsdUtils

    bad = []

    def _inspect(path):
        p = str(path or "")
        if p.startswith("/Game/"):
            return p                 # logical Unreal material identifier
        if (p.startswith(("/", "airstack://", "omniverse:/"))
                and not p.startswith("omniverse://")):
            bad.append(p)
        elif (p and not p.startswith(("/", "omniverse://", "http://",
                                      "https://"))
              and not p.endswith(".mdl")):
            bad.append(p)
        return p

    UsdUtils.ModifyAssetPaths(layer, _inspect)
    return bad


def _files(cache_dir, names_file):
    if not names_file:
        return sorted(glob.glob(os.path.join(cache_dir, "*.usd")))
    names = [q.strip() for q in open(names_file) if q.strip()]
    return [os.path.join(cache_dir, os.path.basename(q)) for q in names]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cache_dir")
    ap.add_argument("--names-file", default="")
    ap.add_argument("--backup-dir", default="")
    ap.add_argument(
        "--check-only", action="store_true",
        help="evaluate every rewrite/repair and gate, but do not save")
    ap.add_argument(
        "--changed-names-out", default="",
        help="write basenames of changed layers for an exact upload")
    args = ap.parse_args()

    from pxr import Sdf, Usd

    files = _files(args.cache_dir, args.names_file)
    if not files:
        raise SystemExit("no .usd archetypes selected in " + args.cache_dir)
    if args.backup_dir and not args.check_only:
        os.makedirs(args.backup_dir, exist_ok=True)

    changed_files = changed_paths = changed_bindings = 0
    changed_names = []
    for path in files:
        layer = Sdf.Layer.FindOrOpen(path)
        if layer is None:
            raise RuntimeError("cannot open " + path)
        if args.backup_dir and not args.check_only:
            backup = os.path.join(args.backup_dir, os.path.basename(path))
            if not os.path.exists(backup):
                shutil.copy2(path, backup)
        n = bake.normalize_archetype_asset_paths(layer, verbose=False)
        stage = Usd.Stage.Open(layer)
        if stage is None:
            raise RuntimeError("cannot compose " + path)
        n_api = bake.apply_material_binding_api(stage, verbose=False)
        if n or n_api:
            changed_files += 1
            changed_paths += n
            changed_bindings += n_api
            changed_names.append(os.path.basename(path))
        if (n or n_api) and not args.check_only:
            if not layer.Save():
                raise RuntimeError("failed to save " + path)
            layer.Reload()
        bad = _unsafe_paths(layer)
        if bad:
            raise RuntimeError("{0}: {1} unsafe path(s) survived, e.g. {2}"
                               .format(path, len(bad), bad[0]))
        print("portable {0}: {1} path(s), {2} binding API(s){3}".format(
            os.path.basename(path), n, n_api,
            " (check only)" if args.check_only else ""))

    if args.changed_names_out and not args.check_only:
        with open(args.changed_names_out, "w") as fh:
            for name in changed_names:
                fh.write(name + "\n")
    print("portable quake archetypes: {0} changed file(s), {1} path(s), "
          "{2} binding API(s), {3} selected{4}".format(
              changed_files, changed_paths, changed_bindings, len(files),
              " (check only)" if args.check_only else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
