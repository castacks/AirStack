#!/bin/bash
cd /isaac-sim/AirStack
bash scene_gen/tools/usd_python.sh scene_gen/tools/_lead_bench_offline.py >/tmp/bench_offline.log 2>&1
echo "BUILT lines: $(grep -c 'BUILT' /tmp/bench_offline.log)  FAILED cells: $(grep -c '^\[bench\] FAILED' /tmp/bench_offline.log)"
S=/isaac-sim/.cache/tornado_probe/bench_offline.usd
echo "floats: $(bash scene_gen/tools/usd_python.sh scene_gen/tools/_lead_bench_audit.py $S 2>/dev/null | grep -c FLOAT)"
bash scene_gen/tools/usd_python.sh scene_gen/tools/_lead_overshoot.py $S 2>/dev/null | tail -4
bash scene_gen/tools/usd_python.sh scene_gen/tools/tornado_fit_probe.py $S 2>/dev/null | tail -4
bash scene_gen/tools/usd_python.sh -c "
from pxr import Usd
stage = Usd.Stage.Open('$S')
n = sum(1 for p in stage.Traverse() if 'substrate' in p.GetName().lower() or p.GetName().lower().endswith('_lip'))
print('substrate/lip prims:', n)
" 2>/dev/null | tail -1
