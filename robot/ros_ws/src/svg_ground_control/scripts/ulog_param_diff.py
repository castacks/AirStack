#!/usr/bin/env python3
"""Diff PX4 parameters between two .ulg logs (e.g. two drones of the fleet).

    pip install pyulog
    python3 ulog_param_diff.py drone_1.ulg drone_3.ulg [PREFIX ...]

Only parameters that differ are printed. With no PREFIX the whole parameter set
is compared; otherwise only names starting with one of the prefixes (e.g.
MPC_ EKF2_ RC). Per-drone calibration (CAL_*), UUIDs and RC trims are expected
to differ — anything else that differs is a candidate for "why does only this
drone do X". See experiment.md "one drone behaves differently".
"""
import sys
from pyulog import ULog

if len(sys.argv) < 3:
    sys.exit(__doc__)
a, b = sys.argv[1], sys.argv[2]
prefixes = tuple(sys.argv[3:])
pa, pb = ULog(a).initial_parameters, ULog(b).initial_parameters
w = max(len(k) for k in set(pa) | set(pb))
print(f"{'param':{w}s}  {a}  |  {b}")
for k in sorted(set(pa) | set(pb)):
    if prefixes and not k.startswith(prefixes):
        continue
    if pa.get(k) != pb.get(k):
        print(f"{k:{w}s}  {pa.get(k)}  |  {pb.get(k)}")
