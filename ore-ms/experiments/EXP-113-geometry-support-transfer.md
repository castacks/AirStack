# EXP113: geometry versus simple support on an additional recording

2026-11-02 America/New_York. Freeze85eabf07c, posthoc context declarationdecb17b89.
**Fixed primary test FAIL; no method adopted or threshold tuned.**

[Full findings and figure](../docs/research/geometry-support-transfer-findings-2026-11-02.md).
Additional physical but already exposed recording `night_thermal_so_salon2`;
all three established20framewindows retained. Six matched captures/12native
expert attempts/240frame executions, with native late RGB zero-match failures
preserved. All3164arrays off/on bitwise equal;85tests pass. 57decisions retained,
18both-supported rows (one early window) form the primary quality population.

Both cameras exceed the fixed information floor on all18rows; gate chooses RGB,
thermal lower translation proxy error on18/18. Gate/RGB/count T RMSE0.102837m,
image coverage0.089175, information-rank diagnostic0.065534, thermal-only0.046304.
Per-factor information normalization makes no choice change here. No sensitivity
replaces the primary. First ranking mistake early/3: modeled informationRGB2097
versus thermal1792, yet T proxy errors0.115234versus0.019930m.

Retrospective unchanged-control context on274EXP112rows/3priorrecordings shows
mixed transfer: information rank slightly worsens Day0RGB and does not consistently
beat count on Day1. These are posthoc exposed-data results, not independent tests.
Geometry adequacy does not establish comparative error risk; cause remains open.

Primary independent audit8997checks/76native information events/72transports;
context audit4815checks/274control decisions. Source/reference hashes unchanged.
No training/reserve/default/architecture change, physical accuracy or third-sensor
claim. Backup: backups/2026-11-02-geometry-support-transfer; original model/compiler
runtime prerequisites EXP112, base runtime EXP106.
