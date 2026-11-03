"""Unchanged v3 native runner with opt-in synchronized call/timing observations."""
import argparse
import json
from pathlib import Path
import time
import torch
from Scripts.Setup import run_td3_execution_v3 as runner


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('dataset','manifest','output'):p.add_argument('--'+n,type=Path,required=True)
    p.add_argument('--modality',choices=['rgb','thermal'],required=True)
    p.add_argument('--frames',type=int,required=True)
    a=p.parse_args();a.start=0;a.acknowledge_provisional_geometry=True
    a.record_uncertainty=True;a.record_arbiter_events=True;a.record_root_initialization=True
    calls=[];phases=[]
    original_factory=runner.MACVO.from_config
    def factory(config):
        system=original_factory(config)
        def wrap(obj,name,kind):
            original=getattr(obj,name)
            def measured(*args,**kwargs):
                torch.cuda.synchronize();start=time.perf_counter_ns();status='FAILED'
                try:
                    result=original(*args,**kwargs);status='PASS';return result
                finally:
                    torch.cuda.synchronize();end=time.perf_counter_ns()
                    record={'kind':kind,'start_ns':start,'end_ns':end,'duration_ns':end-start,'status':status}
                    if kind=='run':record['sensor_time_ns']=int(args[0].time_ns[0])
                    (calls if kind.startswith('frontend_') else phases).append(record)
            setattr(obj,name,measured)
        wrap(system.Frontend,'estimate_depth','frontend_depth')
        wrap(system.Frontend,'estimate_pair','frontend_pair')
        wrap(system,'run','run');wrap(system,'terminate','terminate')
        return system
    # Process-local factory only: estimator source/config and algorithms stay unchanged.
    runner.MACVO.from_config=staticmethod(factory)
    start=time.perf_counter_ns();status='FAILED'
    try:runner.run(a,a.modality);status='PASS'
    finally:
        end=time.perf_counter_ns();dest=a.output/a.modality;dest.mkdir(parents=True,exist_ok=True)
        payload={'status':status,'scope':'CUDA-synchronized offline processing; no live throughput/energy claim',
                 'host_elapsed_ns':end-start,'frontend_calls':calls,'native_phases':phases,
                 'note':'frontend calls are nested inside run; do not add durations together; first-depth probe is separate'}
        with (dest/'native-cost.json').open('x') as f:json.dump(payload,f,indent=2,allow_nan=False);f.write('\n')

if __name__=='__main__':main()
