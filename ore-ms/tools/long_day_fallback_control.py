"""Explicit missing-thermal adapter around the frozen longer-day runner.

No solver or capture loop is copied. Existing NativeThermalGraph(packet=None)
implements the RGB-only graph; source-pinned adapters supply only declared fallback.
"""
from pathlib import Path
from collections import defaultdict
import sys
import numpy as np
import torch
import long_day_strong_controls as run

ROOT=run.ROOT;native=run.native;base=run.base
OUT=ROOT/'results/long-day-fallback-control'
PROTOCOL=ROOT/'docs/research/long-day-fallback-protocol-2026-11-15.md'
FALLBACK=[25,27,28,32]


def preflight():
    torch.set_num_threads(1)
    source,saved,events=run.load()
    parent=native.read(ROOT/'results/long-day-strong-controls/run-001/freeze.json')
    for kind in ['source_pins','input_pins']:
        for p,h in parent[kind].items():assert native.sha(ROOT/p)==h,p
    E=native.geometry(native.read(run.REC/'inputs/day/calibration.json'));rows=[]
    for i in range(599):
        row={'pair':i}
        for cam,sensor in [('rgb','rgb'),('thermal','thr')]:
            try:
                packet,_=native.bind_camera(saved[cam],events[cam].get((i,i+1)),i,source['records'],sensor,E[cam])
                row[cam]={'supported':True,'points':len(packet['points'])}
            except ValueError as error:
                assert cam=='thermal' and i in FALLBACK and str(error)=='no_native_supported_first_write'
                row[cam]={'supported':False,'reason':str(error)}
        rows.append(row)
    assert all(r['rgb']['supported'] for r in rows)
    assert [r['pair'] for r in rows if not r['thermal']['supported']]==FALLBACK
    checks=[]
    for i in FALLBACK:
        e=events['rgb'][(i,i+1)];state=torch.tensor(e['parent_pose_camera_map_xyzw'],dtype=torch.float32)
        def data():return base.feedback.feedback_input(saved['rgb'],e,i,source['records'],state)
        first,_=base.timed_solve(native.Analytic_ReprojDisp_TwoFramePGO(data()))
        graphs=[base.component.FixedRotationGraph(g, native.pp.SE3(first.double()[None])) for g in
                [native.Analytic_ReprojDisp_TwoFramePGO(data()),native.NativeThermalGraph(data(),None,native.pose(state.numpy()),thermal_precision=.1)]]
        assert torch.equal(graphs[0](),graphs[1]())
        assert torch.equal(graphs[0].covariance_array(),graphs[1].covariance_array())
        assert torch.equal(graphs[0].build_jacobian(),graphs[1].build_jacobian())
        a,ra=base.timed_solve(graphs[0]);b,rb=base.timed_solve(graphs[1])
        assert torch.equal(a,b) and ra['losses']==rb['losses']
        checks.append({'pair':i,'initial_arrays_jacobian_exact':True,'committed_solution_and_losses_exact':True})
    OUT.mkdir(parents=True,exist_ok=False)
    native.write(OUT/'preflight.json',{'rows':rows,'rgb_supported':599,'thermal_supported':595,'fallback_pairs':FALLBACK,'fallback_equivalence':checks,'reference_scores_read':False,'adapter_sha256':native.sha(__file__),'protocol_sha256':native.sha(PROTOCOL),'input_pins':parent['input_pins']})
    print('PREFLIGHT599RGB595thermal4fallback; equivalence4/4 exact',flush=True)


def capture():
    p=native.read(OUT/'preflight.json')
    assert p['adapter_sha256']==native.sha(__file__) and p['protocol_sha256']==native.sha(PROTOCOL)
    for name,h in p['input_pins'].items():assert native.sha(ROOT/name)==h,name
    original_load=run.load;original_bind=native.bind_camera;original_write=native.write
    def load_adapter():
        source,saved,events=original_load()
        events['thermal']=defaultdict(lambda:None,events['thermal'])
        return source,saved,events
    def bind_adapter(saved,event,parent,source_rows,sensor,E):
        try:return original_bind(saved,event,parent,source_rows,sensor,E)
        except ValueError as error:
            assert parent in FALLBACK and sensor=='thr' and str(error)=='no_native_supported_first_write'
            return None,{'thermal_used':False,'fallback':'RGB_ONLY_TRANSLATION','reason':str(error),'pair':parent}
    def write_adapter(path,data):
        if Path(path)==OUT/'run-001/freeze.json':
            data['source_pins'].update({'tools/long_day_fallback_control.py':native.sha(__file__),str(PROTOCOL.relative_to(ROOT)):native.sha(PROTOCOL)})
            data.update({'preflight_sha256':native.sha(OUT/'preflight.json'),'fallback_pairs':FALLBACK,'thermal_supported':595,'fallback_policy':'RGB_ONLY_TRANSLATION_ON_UNSUPPORTED_NATIVE_THERMAL','scope':'Exposed first600day; own-state thermal refinement with four predeclared RGB fallback intervals'})
        return original_write(path,data)
    run.OUT=OUT;run.PROTOCOL=PROTOCOL;run.load=load_adapter;native.bind_camera=bind_adapter;native.write=write_adapter
    try:run.capture()
    finally:run.load=original_load;native.bind_camera=original_bind;native.write=original_write


def score():
    run.OUT=OUT;run.score()


if __name__=='__main__':{'preflight':preflight,'capture':capture,'score':score}[sys.argv[1]]()
