# EXP120: synthetic availability transitions

2026-11-03. Freeze 45f60353a; parent EXP119. [Full findings](../docs/research/availability-transitions-findings-2026-11-03.md).

All 36 exposed traces were audited. Brief natural returns exist, but none has a supported>=3 / failed>=3 / recovered>=3 cycle. Five fresh dual captures therefore test explicit requested-input outages on exposed day100: RGB20–34, thermal50–64, both75–79. No fabricated native health, dropped decode/run or false endpoint advance. Immediate transport feedback is synthetic, not a silent-loss detector.

Adaptive full-read and bounded probing deliver 85/95 slots; fixed RGB and fixed thermal deliver 71/95. Active thermal failure produces alternate preselection at 51 and eligible alternate delivery at 52. Bounded pair calls 133 versus 158 perturbed full-read (15.8% fewer); native processing 46.698 versus 55.539 seconds (15.9% lower). Fixed-camera costs include common dual startup/guards.

All four arms share 53 matched eligible proxy intervals. Bounded/full translation RMSE 0.067631/0.067574m; rotation 0.584021/0.583995 degrees. Inactive restoration can regain fresh native cache state without on-time adjacent output within its horizon. Joint return 80 yields first preselected eligible RGB delivery 83. Pre-outage late writebacks and every abstention remain explicit.

Thirty-three tests and 34,779 independent checks pass; 946 pins unchanged; all 5,226 unperturbed arrays exact. Synthetic offline engineering, not natural-fault or physical-accuracy proof. No adoption. Next: separately freeze an output-readiness handshake or continuation and its cost budget. Existing inputs suffice; natural sustained return evidence would strengthen claims.

Backup: backups/2026-11-03-availability-transitions. Inputs/weights/compiler EXP112, base runtime EXP106, control EXP119, coverage EXP117.
