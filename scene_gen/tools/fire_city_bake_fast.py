#!/usr/bin/env python3
"""Experimental deduplicating city-bake driver.

This is deliberately separate from fire_city_bake.sh.  It classifies with the
same fire_city_manifest.py helper, launches each output stem at most once, and
uses experimental_fast_fire_bake_launch_script.py inside Isaac Sim.
"""

import argparse
import json
import os
import shlex
import subprocess
import sys
import time


HERE = os.path.dirname(os.path.abspath(__file__))
REPO_HOST = os.path.dirname(os.path.dirname(HERE))
MANIFEST_TOOL = os.path.join(HERE, "fire_city_manifest.py")


def run(cmd, **kw):
    print("+", " ".join(shlex.quote(str(x)) for x in cmd), flush=True)
    return subprocess.run(cmd, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("manifest")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--verify-only", action="store_true")
    args = ap.parse_args()

    container = os.environ.get("CONTAINER", "isaac-sim")
    repo = os.environ.get("REPO", "/isaac-sim/AirStack")
    out_base = os.environ.get("FB_OUT", "/isaac-sim/.cache/fire_bakes")
    host_cache = os.environ.get(
        "FB_CACHE_HOST", os.path.expanduser("~/docker/isaac-sim/cache/main"))
    seed_base = int(os.environ.get("FB_SEED", "7"))
    timeout_s = int(os.environ.get("TIMEOUT_S", "5400"))
    launcher = os.path.join(
        repo, "simulation/isaac-sim/launch_scripts/experimental_fast_fire_bake_launch_script.py")

    seed = subprocess.check_output(
        [sys.executable, MANIFEST_TOOL, args.manifest, "--print-seed"], text=True).strip()
    city_out = os.path.join(out_base, "city_" + seed)
    if city_out.startswith("/isaac-sim/.cache"):
        city_host = host_cache + city_out[len("/isaac-sim/.cache"):]
    else:
        city_host = city_out

    if args.verify_only:
        return run(["docker", "exec", container, "bash", "-lc",
                    "%s/scene_gen/tools/usd_python.sh %s/scene_gen/tests/test_fire_bake.py --verify %s"
                    % (repo, repo, shlex.quote(city_out))]).returncode

    raw = subprocess.check_output(
        [sys.executable, MANIFEST_TOOL, args.manifest, "--out-dir", city_host], text=True)
    rows = []
    for raw_index, line in enumerate(x for x in raw.splitlines()
                                     if x and not x.startswith("SUMMARY")):
        fields = line.split("\t")
        if len(fields) >= 3:
            rows.append((raw_index,) + tuple(fields[:3]))

    # Classification is a snapshot.  The old driver subsequently launched
    # every NEED row, including repeated stems.  Preserve manifest order but
    # allow only the first occurrence of each output stem to launch.
    # Keep the LAST occurrence.  The old driver baked duplicates in manifest
    # order and overwrote the same stem each time, so its final artifact used
    # the last row's FB_BUILD_SEED.  Selecting that row preserves the exact
    # stochastic result while eliminating all earlier wasted launches.
    last = {stem: (raw_index, entry, stem, status)
            for raw_index, entry, stem, status in rows}
    unique = sorted(last.values())
    duplicates = [(entry, stem) for raw_index, entry, stem, status in rows
                  if last[stem][0] != raw_index]

    print("fast bake plan: %d manifest rows -> %d unique output stems; "
          "%d duplicate Kit launches removed" %
          (len(rows), len(unique), len(duplicates)), flush=True)
    for entry, stem in duplicates:
        print("  DEDUP %s -> %s" % (entry, stem))
    if args.dry_run:
        for _raw_index, entry, stem, status in unique:
            print("  %s %s -> %s" % (status, entry, stem))
        return 0

    os.makedirs(city_host, exist_ok=True)
    json_host = os.path.join(REPO_HOST, "scene_gen/_plans/_fire_city_json", "city_" + seed)
    os.makedirs(json_host, exist_ok=True)
    if run([sys.executable, MANIFEST_TOOL, args.manifest,
            "--write-city-json", json_host], stdout=subprocess.DEVNULL).returncode:
        return 2
    json_container = os.path.join(repo, "scene_gen/_plans/_fire_city_json", "city_" + seed)

    failures = []
    total_start = time.monotonic()
    for unique_index, (raw_index, entry, stem, status) in enumerate(unique):
        if status == "ERROR":
            failures.append((stem, "manifest classification error"))
            continue
        if status == "HAVE" and not args.force:
            print("[%d/%d] cached %s" % (unique_index + 1, len(unique), stem))
            continue
        try:
            kind, name, level, origin, sides, damage_seed = entry.split(":", 5)
        except ValueError:
            failures.append((stem, "bad entry"))
            continue
        if level in ("F0", "F1", "F2", "F3"):
            steps, quiet = 600, 150
        elif level == "F4":
            steps, quiet = 1600, 300
        else:
            steps = int(os.environ.get("SETTLE_STEPS", "2400"))
            quiet = int(os.environ.get("SETTLE_QUIET", "400"))
        env = {
            "ISAAC_SIM_HEADLESS": "true", "PYTHONUNBUFFERED": "1",
            "PYTHONHASHSEED": "0", "FB_KIND": kind, "FB_NAME": name,
            "FB_LEVEL": level, "FB_ORIGIN": origin, "FB_SIDES": sides,
            "FB_SEED": damage_seed, "FB_BUILD_SEED": str(seed_base + 7 * raw_index),
            "FB_INDEX": str(raw_index), "FB_OUT": city_out, "FB_BAKED_KITS": "1",
            "FB_VERIFY": "1", "FB_CITY_JSON": os.path.join(json_container, stem + ".city.json"),
            "SETTLE_STEPS": str(steps), "SETTLE_QUIET": str(quiet),
            # The measured good configuration.  Hull-only was slower and did
            # not converge on SM_Building_30 F5.
            "SETTLE_DECOMP_M": os.environ.get("SETTLE_DECOMP_M", "0.8"),
            "SETTLE_FABRIC": "1", "SETTLE_REST_V2": "0" if kind == "kit" else "1",
        }
        passthrough = ("ISAAC_SIM_PYTHONPATH", "SOOT_TEX_COMPRESS")
        for key in passthrough:
            if key in os.environ:
                env[key] = os.environ[key]
        env_args = sum((["-e", "%s=%s" % item] for item in env.items()), [])
        command = ["docker", "exec"] + env_args + [container, "bash", "-lc",
            "cd /isaac-sim && PYTHONPATH=\"$ISAAC_SIM_PYTHONPATH\" timeout %ds "
            "/isaac-sim/python.sh %s --ext-folder /root/.local/share/ov/data/documents/Kit/shared/exts --no-window"
            % (timeout_s, shlex.quote(launcher))]
        t0 = time.monotonic()
        print("[%d/%d] baking %s" % (unique_index + 1, len(unique), stem), flush=True)
        rc = run(command).returncode
        print("  %.1f s, exit %d" % (time.monotonic() - t0, rc), flush=True)
        sidecar = os.path.join(city_host, stem + ".json")
        usd = os.path.join(city_host, stem + ".usd")
        why = None
        if rc:
            why = "exit %d" % rc
        elif not os.path.isfile(usd) or not os.path.isfile(sidecar):
            why = "missing USD or sidecar"
        else:
            try:
                doc = json.load(open(sidecar))
                rest = doc.get("settle") or {}
                if rest.get("still_moving") or rest.get("converged") is False:
                    why = "settle did not converge"
            except Exception as exc:
                why = "invalid sidecar: %s" % exc
        if why:
            failures.append((stem, why))

    print("fast city bake: %.1f s, %d unique, %d deduplicated, %d failed" %
          (time.monotonic() - total_start, len(unique), len(duplicates), len(failures)))
    for stem, why in failures:
        print("FAILED", stem, why, file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
