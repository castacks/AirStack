#!/usr/bin/env python3
"""Fast-path freeze entry point with a mandatory content-addressed layout gate."""
import os
import json
import runpy
import subprocess
import sys
import threading
import time


repo = os.environ.get("REPO", "/isaac-sim/AirStack")
required = {k: os.environ.get(k) for k in
            ("FC_DUMP", "FC_MANIFEST", "FC_LAYOUT_STAMP")}
missing = [k for k, v in required.items() if not v]
if missing:
    raise SystemExit("fast freeze refused: missing " + ", ".join(missing))
cmd = [sys.executable, os.path.join(repo, "scene_gen/tools/urban_fire_fast_preflight.py"),
       "--repo", repo, "--dump", required["FC_DUMP"],
       "--manifest", required["FC_MANIFEST"], "--stamp",
       required["FC_LAYOUT_STAMP"], "--check-stamp"]
subprocess.run(cmd, check=True)


def _batch_exit_after_report():
    """Exit after the new report is durable, avoiding Kit's shutdown hang.

    This watchdog exists only in the experimental entry point.  Some large
    headless cells finish export, write a complete report and then hang inside
    ``SimulationApp.close()``.  Process exit is safe at this boundary: the USD
    and all sidecars have already been closed, and the report is the strict
    success/failure oracle used by the driver.
    """
    path = os.path.join(os.environ["FREEZE_OUT"], "freeze_report.json")
    try:
        st = os.stat(path)
        baseline = (st.st_mtime_ns, st.st_size)
    except OSError:
        baseline = None
    while True:
        time.sleep(1.0)
        try:
            st = os.stat(path)
            signature = (st.st_mtime_ns, st.st_size)
            if signature == baseline:
                continue
            with open(path) as fh:
                report = json.load(fh)
        except (OSError, ValueError):
            continue
        portable = report.get("portable_ok")
        if portable is None:
            continue
        # Give buffered diagnostics from the exporting thread a moment to
        # reach the tee log, then terminate without entering Kit cleanup.
        time.sleep(2.0)
        print("[fast-freeze] durable report observed; hard batch exit "
              "(portable_ok={0})".format(portable), flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0 if portable else 2)


if os.environ.get("EXPERIMENTAL_FREEZE_HARD_EXIT", "1").lower() not in (
        "0", "false", "no"):
    threading.Thread(target=_batch_exit_after_report, daemon=True).start()

target = os.path.join(repo, "simulation/isaac-sim/launch_scripts",
                      "freeze_urban_fire_city_launch_script.py")
runpy.run_path(target, run_name="__main__")
