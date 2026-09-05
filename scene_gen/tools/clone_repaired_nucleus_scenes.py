#!/usr/bin/env python3
"""Clone a validated Nucleus material-repair wrapper onto an identical scene.

This is for legacy duplicate exports whose large source crates have the exact
same Nucleus content hash.  The source wrapper has already paid the expensive
material-repair and cold-gate cost.  We copy only that tiny wrapper, update its
provenance metadata, quick-open it at a staging URL, then cold-gate the final
canonical destination.  A failed destination gate restores the byte-identical
immutable source payload.

Nothing in this tool re-saves or flattens a finished scene crate.
"""
import argparse
import gc
import hashlib
import json
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import omni.client
from pxr import Sdf, Usd

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scene_gen.tools import repair_nucleus_dataset_materials as repair


MARKER_CLONE_SOURCE = "scene_gen_material_repair_clone_source"


def _entry(url):
    result, value = omni.client.stat(url)
    if not repair._ok(result):
        raise RuntimeError("could not stat Nucleus asset: " + url)
    return value


def _read_json(url):
    result, _version, content = omni.client.read_file(url)
    if not repair._ok(result):
        raise RuntimeError("could not read Nucleus JSON: " + url)
    return json.loads(bytes(memoryview(content)).decode("utf-8"))


def _expected_meshes(source_url):
    report_url = source_url.rsplit("/", 1)[0] + "/material_repair_report.json"
    report = _read_json(report_url)
    if report.get("canonical") != source_url or report.get("status") != "repaired":
        raise RuntimeError("source repair report does not describe canonical URL")
    value = report.get("canonical_gate", {}).get("mesh_count")
    if not isinstance(value, int) or value <= 0:
        raise RuntimeError("source repair report has no valid mesh count")
    return value


def _quick_gate(url, expected_payload):
    layer = Sdf.Layer.FindOrOpen(url)
    if layer is None:
        raise RuntimeError("quick layer open failed: " + url)
    data = dict(layer.customLayerData or {})
    if int(data.get(repair.MARKER_VERSION, 0) or 0) < repair.REPAIR_VERSION:
        raise RuntimeError("quick gate found no repair marker: " + url)
    if str(data.get(repair.MARKER_PAYLOAD, "")) != expected_payload:
        raise RuntimeError("quick gate payload disagrees: " + url)
    stage = Usd.Stage.Open(layer)
    if stage is None or not stage.GetDefaultPrim():
        raise RuntimeError("quick composed-stage open failed: " + url)


def _parse_pair(value):
    source, separator, destination = value.partition("=")
    if not separator or not source or not destination:
        raise argparse.ArgumentTypeError("pair must be SOURCE_REL=DEST_REL")
    if source.startswith("/") or destination.startswith("/"):
        raise argparse.ArgumentTypeError("pair paths must be dataset-relative")
    return source, destination


def clone_one(dataset, payload_root, source_relative, destination_relative,
              *, apply=False, max_examples=25):
    source_url = dataset.rstrip("/") + "/" + source_relative
    destination_url = dataset.rstrip("/") + "/" + destination_relative
    source_layer = Sdf.Layer.FindOrOpen(source_url)
    if source_layer is None or not repair._already_repaired(source_layer):
        raise RuntimeError("source is not a repaired wrapper: " + source_url)
    source_data = dict(source_layer.customLayerData or {})
    payload = str(source_data[repair.MARKER_PAYLOAD])
    expected_meshes = _expected_meshes(source_url)

    destination_layer = Sdf.Layer.FindOrOpen(destination_url)
    if destination_layer is None:
        raise RuntimeError("could not open destination: " + destination_url)
    if repair._already_repaired(destination_layer):
        cold = repair._cold_gate(destination_url, expected_meshes,
                                 max_examples=max_examples, reload=True)
        return {"source": source_relative, "destination": destination_relative,
                "status": "already_repaired", "cold": cold}

    payload_entry = _entry(payload)
    destination_entry = _entry(destination_url)
    if (payload_entry.size != destination_entry.size or
            payload_entry.hash != destination_entry.hash):
        raise RuntimeError(
            "refusing clone: destination source is not byte-identical to "
            "the repaired wrapper payload")
    if not apply:
        return {"source": source_relative, "destination": destination_relative,
                "status": "would_clone", "payload": payload,
                "source_hash": payload_entry.hash}

    digest = hashlib.sha256(destination_relative.encode("utf-8")).hexdigest()[:16]
    staging = payload_root.rstrip("/") + "/_clone_staging/" + digest + ".usd"
    work_dir = tempfile.mkdtemp(prefix="material_wrapper_clone_")
    local = os.path.join(work_dir, "wrapper.usda")
    swapped = False
    try:
        if not source_layer.Export(local):
            raise RuntimeError("could not export repaired source wrapper")
        clone = Sdf.Layer.FindOrOpen(local)
        if clone is None:
            raise RuntimeError("could not reopen local wrapper clone")
        data = dict(clone.customLayerData or {})
        data[repair.MARKER_SOURCE] = destination_relative
        data[MARKER_CLONE_SOURCE] = source_relative
        clone.customLayerData = data
        if not clone.Save():
            raise RuntimeError("could not save local wrapper clone")

        result = omni.client.copy(
            local, staging, behavior=omni.client.CopyBehavior.OVERWRITE)
        if not repair._ok(result):
            raise RuntimeError("staging clone upload failed: {0}".format(result))
        _quick_gate(staging, payload)

        result = omni.client.copy(
            local, destination_url,
            behavior=omni.client.CopyBehavior.OVERWRITE)
        if not repair._ok(result):
            raise RuntimeError("canonical clone upload failed: {0}".format(result))
        swapped = True
        cold = repair._cold_gate(destination_url, expected_meshes,
                                 max_examples=max_examples, reload=True)
        report = {
            "source": source_relative,
            "destination": destination_relative,
            "status": "cloned_repair",
            "canonical": destination_url,
            "payload": payload,
            "source_hash": payload_entry.hash,
            "expected_meshes": expected_meshes,
            "cold": cold,
            "completed_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                           time.gmtime()),
        }
        repair._upload_json(
            report, destination_url.rsplit("/", 1)[0] +
            "/material_repair_report.json")
        return report
    except Exception:
        if swapped:
            result = omni.client.copy(
                payload, destination_url,
                behavior=omni.client.CopyBehavior.OVERWRITE)
            restored = _entry(destination_url)
            if (not repair._ok(result) or
                    restored.size != destination_entry.size or
                    restored.hash != destination_entry.hash):
                raise RuntimeError("clone failed AND destination rollback failed")
        raise
    finally:
        if os.path.isdir(work_dir):
            shutil.rmtree(work_dir)
        omni.client.delete(staging)
        gc.collect()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", default=repair.DEFAULT_DATASET)
    parser.add_argument("--payload-root", default=repair.DEFAULT_PAYLOADS)
    parser.add_argument("--pair", action="append", required=True,
                        type=_parse_pair, metavar="SOURCE_REL=DEST_REL")
    parser.add_argument("--max-examples", type=int, default=25)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--keep-going", action="store_true")
    parser.add_argument("--out", default="")
    args = parser.parse_args(argv)

    omni.client.initialize()
    rows = []
    try:
        for source, destination in args.pair:
            print("[material_clone] {0} -> {1}".format(source, destination),
                  flush=True)
            try:
                row = clone_one(
                    args.dataset, args.payload_root, source, destination,
                    apply=args.apply, max_examples=args.max_examples)
            except Exception as exc:  # noqa: BLE001 - optional batch continue
                row = {"source": source, "destination": destination,
                       "status": "failed", "error": str(exc)}
                rows.append(row)
                print("[material_clone] FAILED {0}: {1}".format(
                    destination, exc), flush=True)
                if not args.keep_going:
                    break
                continue
            rows.append(row)
            print("[material_clone] {0} {1}".format(
                row["status"].upper(), destination), flush=True)
    finally:
        omni.client.shutdown()

    summary = {
        "dataset": args.dataset,
        "rows": rows,
        "failed": sum(row["status"] == "failed" for row in rows),
        "cloned": sum(row["status"] == "cloned_repair" for row in rows),
        "clean": sum(row["status"] == "already_repaired" for row in rows),
    }
    if args.out:
        with open(args.out, "w") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
    print("[material_clone] COMPLETE cloned={cloned} clean={clean} "
          "failed={failed}".format(**summary), flush=True)
    return 1 if summary["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
