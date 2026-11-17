# Frozen depth detector: resolution and viewpoint limits

2026-11-17 14:55 UTC, `ore_proj`, source checkpoint `d76bf4602`. A predeclared
Cartesian matrix used three resolutions and three camera heights, with the same
nominal red cube, blue distractor, optics, light and camera +X orientation. The
0.1 m local-depth-step rule and all detector bytes were frozen before capture.
Each case saved three separately rendered frames; none commanded a robot.

| Resolution | Camera z (m) | Observation statuses | Max adjacent step (m) | Window policy |
| --- | --- | --- | --- | --- |
| 240×150 | 0.02 | OBSERVED, INCONSISTENT, INCONSISTENT | 0.0982, 0.1674, 0.1674 | ABSTAIN |
| 240×150 | 0.60 | 3 OBSERVED | <0.000001 | SELECTED |
| 240×150 | 1.20 | 3 OBSERVED | <0.000001 | SELECTED |
| 480×300 | 0.02 | 3 OBSERVED | 0.0918 | SELECTED |
| 480×300 | 0.60 | 3 OBSERVED | <0.000001 | SELECTED |
| 480×300 | 1.20 | 3 OBSERVED | <0.000001 | SELECTED |
| 960×600 | 0.02 | 3 OBSERVED | 0.0482 | SELECTED |
| 960×600 | 0.60 | 3 OBSERVED | <0.000001 | SELECTED |
| 960×600 | 1.20 | 3 OBSERVED | <0.000001 | SELECTED |

Eight of nine authored positive controls pass all three observations. This is a
development envelope result, not a perception success rate: the same geometry is
reused and these are not independent trials. The low-resolution, low-camera case
exposes a conservative false abstention on the legitimate cube. The visually
similar first frame passes while later frames reject; the renderer's RGB/depth
output changes across those static-scene frames. A unique rendering cause was not
established. DLSS minimum internal resolution warnings were present and retained
locally, alongside the earlier SDK projection/cache/cleanup warnings. No threshold
or camera setting was changed after the result to force a pass.

All 27 PNG/depth/K inputs, per-frame camera metadata, authored geometry and stages
are retained. The [compact report](../render-envelope-20261117.json) records all
statuses, diagnostics, policy decisions and 81 raw input hashes. `audit.py` checks
hashes and calibration against renderer projection and requires the exact frozen
detector source hash; use this source checkpoint or the saved detector snapshot
when reproducing. It never starts the renderer or dispatches a goal:

```bash
PYTHONPATH=. python docs/uav-results/render-envelope-20261117/audit.py
```

The private capture terminated with exit0, a complete nine-case result and no
error record. Original live Kit210 and PX4 PID978 retained their earlier start
times. No scene mutation in the live application, ROS publication, flight or
learned model inference occurred. These views do not establish general object
recognition, synchronized robot-camera acquisition, identity or physical safety.
