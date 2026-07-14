#!/usr/bin/env python3
"""
batch_inject_skinned_patches.py

Batch wrapper around inject_skinned_patch_stable.py.

Mode A: scan character directories

  /isaac-sim/python.sh batch_inject_skinned_patches.py \
    --character-root ../assets/characters \
    --patched-suffix _patched.usd \
    --output-suffix _with_skinned_patch.usd \
    --patch-path "/root/Plane/Plane"

For a file like:
  ../assets/characters/F_Business_02/F_Business_02_patched.usd
it expects the original:
  ../assets/characters/F_Business_02/F_Business_02.usd
and writes:
  ../assets/characters/F_Business_02/F_Business_02_with_skinned_patch.usd

Mode B: use a CSV with explicit pairs

CSV columns:
  original,patched,output

Optional columns:
  patch_path,patch_keyword,target_mesh_path,target_mesh_keyword,offset_distance

Example:
  /isaac-sim/python.sh batch_inject_skinned_patches.py --pairs-csv pairs.csv
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from inject_skinned_patch_stable import inject_patch


def discover_pairs(
    character_root: Path,
    patched_suffix: str,
    output_suffix: str,
) -> List[Dict[str, str]]:
    pairs: List[Dict[str, str]] = []
    for patched in sorted(character_root.rglob(f"*{patched_suffix}")):
        stem = patched.name[: -len(patched_suffix)]
        original = patched.with_name(stem + ".usd")
        output = patched.with_name(stem + output_suffix)

        if not original.exists():
            print(f"[WARN] Skipping {patched}: expected original not found: {original}")
            continue

        pairs.append({
            "original": str(original),
            "patched": str(patched),
            "output": str(output),
        })
    return pairs


def read_pairs_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    required = {"original", "patched", "output"}
    if not rows:
        raise SystemExit(f"[ERROR] CSV has no rows: {path}")

    missing = required - set(rows[0].keys())
    if missing:
        raise SystemExit(f"[ERROR] CSV missing required columns: {sorted(missing)}")

    return rows


def as_float(value: Optional[str], default: float) -> float:
    if value is None or value == "":
        return default
    return float(value)


def as_int(value: Optional[str], default: int) -> int:
    if value is None or value == "":
        return default
    return int(value)


def empty_to_none(value: Optional[str]) -> Optional[str]:
    if value is None or value == "":
        return None
    return value


def main() -> None:
    ap = argparse.ArgumentParser()
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--character-root", default=None, help="Root directory to scan for *_patched.usd")
    source.add_argument("--pairs-csv", default=None, help="CSV with original,patched,output columns")

    ap.add_argument("--patched-suffix", default="_patched.usd")
    ap.add_argument("--output-suffix", default="_with_skinned_patch.usd")

    ap.add_argument("--patch-path", default=None, help="Default patch path, e.g. /root/Plane/Plane")
    ap.add_argument("--patch-keyword", default=None)
    ap.add_argument("--target-mesh-path", default=None)
    ap.add_argument("--target-mesh-keyword", default=None)

    ap.add_argument("--dst-patch-name", default="BackPatch")
    ap.add_argument("--offset-distance", type=float, default=0.003)
    ap.add_argument("--k-nearest", type=int, default=4)
    ap.add_argument("--max-influences", type=int, default=None)
    ap.add_argument("--flip-winding", action="store_true")

    ap.add_argument("--dry-run", action="store_true")

    args = ap.parse_args()

    if args.pairs_csv:
        pairs = read_pairs_csv(Path(args.pairs_csv))
    else:
        pairs = discover_pairs(
            character_root=Path(args.character_root),
            patched_suffix=args.patched_suffix,
            output_suffix=args.output_suffix,
        )

    print(f"[INFO] Found {len(pairs)} pair(s)")

    for i, row in enumerate(pairs, 1):
        original = row["original"]
        patched = row["patched"]
        output = row["output"]

        patch_path = empty_to_none(row.get("patch_path")) or args.patch_path
        patch_keyword = empty_to_none(row.get("patch_keyword")) or args.patch_keyword
        target_mesh_path = empty_to_none(row.get("target_mesh_path")) or args.target_mesh_path
        target_mesh_keyword = empty_to_none(row.get("target_mesh_keyword")) or args.target_mesh_keyword
        offset_distance = as_float(row.get("offset_distance"), args.offset_distance)
        k_nearest = as_int(row.get("k_nearest"), args.k_nearest)

        print()
        print(f"=== [{i}/{len(pairs)}] ===")
        print(f"original: {original}")
        print(f"patched : {patched}")
        print(f"output  : {output}")
        print(f"patch   : {patch_path or patch_keyword or '<auto>'}")
        print(f"target  : {target_mesh_path or target_mesh_keyword or '<auto>'}")
        print(f"offset  : {offset_distance}")

        if args.dry_run:
            continue

        inject_patch(
            original_path=original,
            patched_path=patched,
            output_path=output,
            patch_path=patch_path,
            patch_keyword=patch_keyword,
            target_mesh_path=target_mesh_path,
            target_mesh_keyword=target_mesh_keyword,
            dst_patch_name=args.dst_patch_name,
            offset_distance=offset_distance,
            k_nearest=k_nearest,
            max_influences=args.max_influences,
            flip_winding=args.flip_winding,
        )

    print()
    print("[OK] Batch finished")


if __name__ == "__main__":
    main()
