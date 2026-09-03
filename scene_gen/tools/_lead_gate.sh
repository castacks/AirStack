#!/bin/bash
cd /isaac-sim/AirStack
bash scene_gen/tools/usd_python.sh scene_gen/tools/_lead_bench_offline.py >/tmp/bench_offline.log 2>&1
BUILT=$(grep -c 'BUILT' /tmp/bench_offline.log || true)
FAILED=$(grep -c '^\[bench\] FAILED' /tmp/bench_offline.log || true)
echo "BUILT lines: $BUILT  FAILED cells: $FAILED"
S=/isaac-sim/.cache/tornado_probe/bench_offline.usd
AUDIT=/tmp/bench_float_audit.log
bash scene_gen/tools/usd_python.sh scene_gen/tools/_lead_bench_audit.py "$S" >"$AUDIT" 2>&1
FLOATS=$(grep -c FLOAT "$AUDIT" || true)
echo "floats: $FLOATS"
bash scene_gen/tools/usd_python.sh scene_gen/tools/_lead_overshoot.py $S 2>/dev/null | tail -4
bash scene_gen/tools/usd_python.sh scene_gen/tools/tornado_fit_probe.py $S 2>/dev/null | tail -4
bash scene_gen/tools/usd_python.sh -c "
from pxr import Usd
stage = Usd.Stage.Open('$S')
n = sum(1 for p in stage.Traverse() if 'substrate' in p.GetName().lower() or p.GetName().lower().endswith('_lip'))
print('substrate/lip prims:', n)
" 2>/dev/null | tail -1
if [ "$FAILED" -ne 0 ]; then
    echo "FAILED: $FAILED bench cell(s) did not build:"
    grep '^\[bench\] FAILED' /tmp/bench_offline.log
    exit 1
fi
if [ "$FLOATS" -ne 0 ]; then
    echo "FAILED: floating subtrees (including unsupported roof trim):"
    grep FLOAT "$AUDIT"
    exit 1
fi
