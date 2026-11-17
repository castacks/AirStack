# Passive receipt-window check

2026-11-17 14:52 UTC, `ore_proj`, starting at `e3ec006d4` with the recorded source
changes. The current live default scene was read without actions, services,
actuation publishers or stage edits. All 20 exact-header triples were classified
ABSENT. Two records warmed up the three-frame window; all 18 subsequent windows
passed receipt-age, source-progression and calibration consistency checks.

Maximum age of any receipt at its recorded window check: 0.131582 s; maximum
three-frame receipt span: 0.130492 s. These are measured receipt timings, not source
clock verification, acquisition synchronization or current permission on replay.
No target identity, scene-wide absence or motion qualification follows.

All 20 metadata records are saved; only records 0, 10 and 19 have raw PNG/NPZ pairs.
The audit checks headers and window arithmetic independently, replays the helper
at the recorded monotonic times, verifies hashes and recomputes those three raw
observations. The other 17 raw observations cannot be independently recomputed.
Capture and helper snapshots are pinned by provenance. Older passive records keep
their original source versions and were not rewritten.

From the RRM root, with optional image dependencies installed:

```bash
PYTHONPATH=. python docs/uav-results/passive-window-20261117/audit.py
```

The compact report is [saved separately](../passive-window-audit-20261117.json).
