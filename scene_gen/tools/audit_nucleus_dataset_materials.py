#!/usr/bin/env python3
"""Cold-audit material resolution in every exported Nucleus dataset USD."""
import argparse
import gc
import json
import sys
from pathlib import Path

import omni.client
from pxr import Usd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scene_gen.disaster import material_audit
from scene_gen.tools.dataset_upload import _iter_remote


DEFAULT_ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/"
                "SEI-COA/final_disaster_dataset")


def run(root, only="", max_examples=25):
    rows = []
    paths = sorted(path for path, _size in _iter_remote(root, only or None)
                   if path.lower().endswith((".usd", ".usdc")))
    for index, relative in enumerate(paths, 1):
        url = root.rstrip("/") + "/" + relative
        print("[material_audit] {0}/{1} OPEN {2}".format(
            index, len(paths), relative), flush=True)
        try:
            stage = Usd.Stage.Open(url)
            if stage is None:
                raise RuntimeError("Usd.Stage.Open returned None")
            report = material_audit.audit(stage, max_examples=max_examples)
            row = {"relative_path": relative, "url": url, **report}
        except Exception as exc:  # noqa: BLE001 - batch must continue
            row = {"relative_path": relative, "url": url, "ok": False,
                   "open_error": str(exc)}
        rows.append(row)
        counts = row.get("counts") or {}
        print("[material_audit] {0} {1}: dangling={2} typeless={3} "
              "unbound={4} surface_less={5} visible_meshes={6}".format(
                  "PASS" if row.get("ok") else "FAIL", relative,
                  counts.get("dangling_targets", 0),
                  counts.get("typeless_targets", 0),
                  counts.get("unbound_uncolored", 0),
                  counts.get("surface_less_materials", 0),
                  counts.get("visible_meshes", 0)), flush=True)
        try:
            del stage
        except UnboundLocalError:
            pass
        gc.collect()
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=DEFAULT_ROOT)
    parser.add_argument("--only", default="")
    parser.add_argument("--out", default="")
    parser.add_argument("--max-examples", type=int, default=25)
    parser.add_argument("--fail-on-error", action="store_true")
    args = parser.parse_args(argv)
    omni.client.initialize()
    try:
        rows = run(args.root, args.only, args.max_examples)
    finally:
        omni.client.shutdown()
    summary = {"root": args.root, "usd_count": len(rows),
               "passed": sum(bool(row.get("ok")) for row in rows),
               "failed": sum(not row.get("ok") for row in rows),
               "rows": rows}
    if args.out:
        with open(args.out, "w") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
    print("[material_audit] COMPLETE {0}/{1} passed".format(
        summary["passed"], summary["usd_count"]), flush=True)
    return 1 if args.fail_on_error and summary["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
