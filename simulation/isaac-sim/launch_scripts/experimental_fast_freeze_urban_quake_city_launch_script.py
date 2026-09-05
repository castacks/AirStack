#!/usr/bin/env python3
"""Batch-safe entry point for the opt-in urban-earthquake freeze path.

Large headless Kit jobs occasionally finish the USD/report and then hang in
``SimulationApp.close``.  Once a new, parseable portability report is durable,
all required quake sidecars and review images already exist.  This wrapper
therefore exits at that exact success/failure boundary; it never changes how
the scene itself is built.
"""

import json
import os
import runpy
import sys
import threading
import time


repo = os.environ.get("REPO", "/isaac-sim/AirStack")
out = os.environ.get("FREEZE_OUT", "").strip()
if not out:
    raise SystemExit("urban quake fast freeze requires FREEZE_OUT")


def _exit_after_report():
    path = os.path.join(out, "freeze_report.json")
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
        time.sleep(2.0)
        print("[quake-fast-freeze] durable report observed; hard batch exit "
              "(portable_ok={0})".format(portable), flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0 if portable else 2)


threading.Thread(target=_exit_after_report, daemon=True).start()
target = os.path.join(
    repo, "simulation", "isaac-sim", "launch_scripts",
    "downtown_quake_launch_script.py")
runpy.run_path(target, run_name="__main__")
