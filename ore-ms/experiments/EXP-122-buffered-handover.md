# EXP122: bounded retained-input handover repair passes

2026-11-03. Freeze 32720b0b1; parent EXP121. [Full findings](../docs/research/buffered-handover-findings-2026-11-03.md).

Six fresh dual captures preserve day100, source outages and native solver. Transport retains actual encoded current/prior inputs, including arrivals skipped by neural processing; dropped inputs are absent. Catch-up allows one missing predecessor per group and at most two queries per run. Original selection and readiness caps remain unchanged.

The frozen contract PASSES. Exact failing-trace off reproduces EXP121 arrays and the target51 gap. Catch-up processes retained RGB50 during group51 and delivers adjacent50→51 by group52, improving 84/95 to 85/95 for one added native invocation (139→140 calls). The live buffered candidate also uses one catch-up and delivers 85/95, restoring adjacent output in all four return episodes. Live calls141 versus full-read158; native processing49.647 versus55.670 seconds.

Live uses thirteen readiness extras and one buffered invocation, within all caps. Encoded peak is 4,217,128 bytes; ingress156,530,123 bytes; retrieved payload1,392,345 bytes. These component costs are not total RSS or sensor/bandwidth/energy savings. Across 84 common eligible slots, candidate translation/rotation proxy RMSE is 0.066306m/0.561533deg, with unchanged nominal/reference qualifications.

Fifty-two tests and 48,655 independent checks pass; 961 pins remain unchanged. All 8,918 unperturbed/fixed-trace arrays match, including a separate byte-pattern check. Synthetic serial-offline mechanism repair does not establish natural-fault, acquisition-deadline or physical-accuracy validation. No production default adoption.

Backup: backups/2026-11-03-buffered-handover. Inputs/weights/compiler EXP112, runtime EXP106, fixture/control/inherited coverage EXP121.
