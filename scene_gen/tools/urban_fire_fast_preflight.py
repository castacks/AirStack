#!/usr/bin/env python3
"""Fail-fast checks and content-addressed layout proof for the fast fire path."""
import argparse
import hashlib
import json
import os
from pathlib import Path


LAYOUT_INPUTS = (
    "scene_gen/generate_scene.py",
    "scene_gen/scene_generator.py",
    "scene_gen/layout/city_layout.py",
    "scene_gen/detail/districts.py",
    "scene_gen/detail/city_detail.py",
    "scene_gen/tools/plan_to_fc_dump.py",
)


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def source_fingerprint(repo, preset):
    repo = Path(repo).resolve()
    paths = [repo / p for p in LAYOUT_INPUTS]
    paths.append(repo / "scene_gen/config/presets" / (preset + ".yaml"))
    # Asset-set and low-level configuration affect pool contents and placement.
    paths += sorted((repo / "scene_gen/config/asset_sets").glob("*.yaml"))
    paths += sorted((repo / "scene_gen/config/low_level").rglob("*.yaml"))
    h = hashlib.sha256()
    for p in paths:
        if not p.is_file():
            raise SystemExit("missing layout input: %s" % p)
        rel = p.relative_to(repo).as_posix().encode()
        h.update(len(rel).to_bytes(4, "big")); h.update(rel)
        h.update(bytes.fromhex(sha(p)))
    return h.hexdigest(), len(paths)


def inspect(repo, dump_path, manifest_path):
    dump_path, manifest_path = Path(dump_path), Path(manifest_path)
    dump = json.load(open(dump_path))
    manifest = json.load(open(manifest_path))
    expected = (manifest.get("placements_dump") or {}).get("sha256")
    actual = sha(dump_path)
    errors = []
    if expected != actual:
        errors.append("manifest placement-dump sha256 is stale")
    if str(manifest.get("preset")) != str(dump.get("preset")):
        errors.append("manifest and dump preset differ")
    if int(manifest.get("seed", -1)) != int(dump.get("seed", -2)):
        errors.append("manifest and dump seed differ")

    rows = manifest.get("records") or []
    allowed = {"kit", "gac", "dtc", "aec"}
    for n, row in enumerate(rows):
        kind, url = row.get("kind"), str(row.get("usd") or "")
        if kind not in allowed:
            errors.append("record %d has unroutable kind %r" % (n, kind))
        if not url:
            errors.append("record %d has no USD" % n)
        if os.path.isabs(url):
            errors.append("record %d has host-local absolute USD: %s" % (n, url))
        if "scene_gen/assets/aec/" in url and kind != "aec":
            errors.append("record %d is an AEC asset routed as %r" % (n, kind))
        if kind == "aec" and not row.get("asset"):
            errors.append("record %d is AEC but has no asset key" % n)

    fp, n_inputs = source_fingerprint(repo, str(manifest.get("preset")))
    proof = {
        "schema": "urban-fire-layout-proof-v1",
        "preset": manifest.get("preset"),
        "seed": manifest.get("seed"),
        "offline_dump_sha256": actual,
        "manifest_sha256": sha(manifest_path),
        "layout_source_sha256": fp,
        "layout_input_files": n_inputs,
    }
    return proof, errors, len(rows)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--stamp")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check-stamp", action="store_true")
    mode.add_argument("--write-stamp", action="store_true")
    args = ap.parse_args()
    proof, errors, n_rows = inspect(args.repo, args.dump, args.manifest)
    if errors:
        for e in errors:
            print("PRE-FLIGHT ERROR:", e)
        return 1
    if args.check_stamp:
        if not args.stamp or not Path(args.stamp).is_file():
            print("layout proof absent")
            return 2
        try:
            saved = json.load(open(args.stamp))
        except Exception as e:
            print("layout proof is not valid JSON:", e)
            return 2
        if saved != proof:
            print("layout proof stale: dump, manifest, preset, or layout source changed")
            return 2
        print("layout proof current:", args.stamp)
    elif args.write_stamp:
        if not args.stamp:
            ap.error("--write-stamp requires --stamp")
        Path(args.stamp).parent.mkdir(parents=True, exist_ok=True)
        with open(args.stamp, "w") as f:
            json.dump(proof, f, indent=2, sort_keys=True)
            f.write("\n")
        print("wrote content-addressed layout proof:", args.stamp)
    else:
        print("preflight OK: %d fire records; manifest/dump/routing are coherent" % n_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
