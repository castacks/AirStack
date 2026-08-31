"""_burn_rss — run another probe script in-process and report its PEAK RSS.

    usd_python.sh _burn_rss.py <script.py> [args...]

`gac_burn_probe.py` deliberately prints nothing about memory (its stdout is
the byte-for-byte artefact the GAC/kit FREEZE is diffed on, so a line added
there would show up as a regression). This wrapper keeps that stdout intact
and appends one line of its own after it.
"""
import os
import resource
import runpy
import sys
import time

script = sys.argv[1]
sys.argv = [script] + sys.argv[2:]
t0 = time.time()
code = 0
try:
    runpy.run_path(script, run_name="__main__")
except SystemExit as e:
    code = int(e.code or 0)
print("[rss] PEAK RSS %.0f MB, wall %.1f s, exit %d"
      % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0,
         time.time() - t0, code))
