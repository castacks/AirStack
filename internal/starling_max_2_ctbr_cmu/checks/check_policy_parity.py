import json
from pathlib import Path
import sys
import time
import numpy as np
from gamma_zero_controller.policy_worker import PolicyClient

root=Path(sys.argv[1])
p=PolicyClient(root/'starling_cmu_inputs.json',str(root/'policy.sock'))
aerror=np.zeros(4);cerror=np.zeros(4);times=[]
for row in json.loads((root/'policy_golden.json').read_text()):
    if row['reset']: p.reset_history()
    t=time.perf_counter();a,c=p.act(row['state'],row['goal'],row['previous']);times.append(time.perf_counter()-t)
    aerror=np.maximum(aerror,np.abs(a-row['action']));cerror=np.maximum(cerror,np.abs(c-row['physical']))
result=dict(steps=64,action_max_error=aerror.tolist(),physical_max_error=cerror.tolist(),
            median_ms=float(np.median(times)*1000),p95_ms=float(np.percentile(times,95)*1000),max_ms=float(max(times)*1000),
            cross_cpu_absolute_tolerance=1e-4)
(root/'policy-parity.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
assert max(aerror.max(),cerror.max())<1e-4
