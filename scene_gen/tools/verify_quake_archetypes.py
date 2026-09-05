#!/usr/bin/env python3
"""Cold-check reusable earthquake archetypes from their Nucleus location.

This is deliberately different from checking the local bake cache before an
upload: every selected layer is opened through the Omniverse resolver, every
authored asset path is inspected in that remote layer, every Nucleus
dependency is ``stat``-checked, and (optionally) every rooftop-plant cluster is
checked against the geometry composed from Nucleus.

Run with Isaac's bare USD Python (no SimulationApp/GPU required)::

    bash scene_gen/tools/usd_python.sh \
      scene_gen/tools/verify_quake_archetypes.py \
      --names-file /tmp/quake_roof_changed_names.txt --roof-support
"""
import argparse
import json
import os
import sys

import omni.client
from pxr import Sdf, UsdUtils


HERE = os.path.dirname(os.path.abspath(__file__))
SCENE_GEN = os.path.dirname(HERE)
if SCENE_GEN not in sys.path:
    sys.path.insert(0, SCENE_GEN)


DEFAULT_ROOT = (
    "omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
    "scene_gen/assets/archetype")


def _names(path):
    if not path:
        raise ValueError("--names-file is required; a cold audit must be explicit")
    return sorted({os.path.basename(q.strip()) for q in open(path) if q.strip()})


def _unsafe(path):
    """Whether an authored dependency cannot travel with a Nucleus layer."""
    p = str(path or "")
    if not p or p.endswith(".mdl"):
        return False
    # Unreal-imported MDL networks retain their source material identifier
    # in an Asset-typed metadata field (for example
    # ``/Game/ModularNeighborhoodPack/Materials/Wood_01.Wood_01``).  It is a
    # logical shader identifier, not a filesystem dependency for USD to
    # open.  Do not confuse it with an absolute local build-machine path.
    if p.startswith("/Game/"):
        return False
    return not p.startswith(("omniverse://", "http://", "https://"))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=DEFAULT_ROOT)
    ap.add_argument("--names-file", required=True)
    ap.add_argument(
        "--manifest", default="",
        help="also read this JSON manifest under --root and stat every "
             "record's usd basename (for example gac_quake.json)")
    ap.add_argument("--roof-support", action="store_true")
    args = ap.parse_args(argv)

    names = _names(args.names_file)
    bad = []
    dependencies = set()
    opened = 0
    for name in names:
        url = args.root.rstrip("/") + "/" + name
        layer = Sdf.Layer.FindOrOpen(url)
        if layer is None:
            bad.append((url, "layer did not open"))
            continue
        opened += 1

        def inspect(path):
            p = str(path or "")
            if _unsafe(p):
                bad.append((url, p))
            elif p.startswith("omniverse://"):
                dependencies.add(p)
            return p

        # This walks asset-valued attributes AND composition arcs. Returning
        # the original spelling makes it a read-only inspection in memory;
        # the remote layer is never saved or changed.
        UsdUtils.ModifyAssetPaths(layer, inspect)

    missing = []
    for dep in sorted(dependencies):
        result, _entry = omni.client.stat(dep)
        if result != omni.client.Result.OK:
            missing.append((dep, str(result)))

    manifest_records = manifest_assets = 0
    if args.manifest:
        manifest_url = args.root.rstrip("/") + "/" + args.manifest
        result, _version, content = omni.client.read_file(manifest_url)
        if result != omni.client.Result.OK:
            missing.append((manifest_url, str(result)))
        else:
            doc = json.loads(bytes(memoryview(content)).decode("utf-8"))
            rows = doc.get("records", []) if isinstance(doc, dict) else doc
            manifest_records = len(rows)
            targets = sorted({args.root.rstrip("/") + "/" +
                              os.path.basename(str(row.get("usd")))
                              for row in rows if row.get("usd")})
            manifest_assets = len(targets)
            for target in targets:
                result, _entry = omni.client.stat(target)
                if result != omni.client.Result.OK:
                    missing.append((target, str(result)))

    roof_failures = []
    roof_clusters = 0
    if args.roof_support:
        from roof_plant_seat_probe import check_archetype
        for name in names:
            url = args.root.rstrip("/") + "/" + name
            rows = check_archetype(url, verbose=False)
            roof_clusters += len(rows)
            roof_failures.extend(q for q in rows if not q["ok"])

    print("remote quake archetypes: {0}/{1} opened, {2} unique dependencies, "
          "{3} unsafe, {4} missing".format(
              opened, len(names), len(dependencies), len(bad), len(missing)))
    if args.manifest:
        print("remote manifest: {0} record(s), {1} unique USD asset(s)"
              .format(manifest_records, manifest_assets))
    if args.roof_support:
        print("remote roof support: {0} cluster(s), {1} failure(s)".format(
            roof_clusters, len(roof_failures)))
    for label, rows in (("unsafe", bad), ("missing", missing),
                        ("roof", roof_failures)):
        for row in rows[:20]:
            print("  {0}: {1}".format(label, row))
    return 1 if bad or missing or roof_failures or opened != len(names) else 0


if __name__ == "__main__":
    raise SystemExit(main())
