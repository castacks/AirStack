"""Real v3 entry point with controlled zero-observation failure; no GPU/model."""
from argparse import Namespace
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest
import torch

from Scripts.Setup import run_td3_execution_v3 as runner


@pytest.mark.parametrize('enabled',[False,True])
def test_v3_failure_reporting_and_opt_in_events(tmp_path,monkeypatch,enabled):
    source=tmp_path/'MAC-VO-main';source.mkdir();weights=tmp_path/'models/macslam';weights.mkdir(parents=True)
    (weights/'MACVO_FrontendCov.pth').write_bytes(b'controlled fixture, not a checkpoint')
    class Sequence:
        def __init__(self):
            self.rows=[{'rgb_left':{'time_ns':n}} for n in [100,200]];self.indices=np.arange(2)
            self.signed_baseline=-.2;self.baseline=.2;self.K=np.eye(3)[None]
        def __len__(self):return 2
        def clip(self,**kwargs):pass
        def __getitem__(self,i):return NS(stereo=NS(frame_ns=[100,200][i]))
        def __iter__(self):return iter([self[0],self[1]])
    class Refiner:
        def elaborate_map(self,frames):frames.data['need_interp'].tensor[1]=False
    class System:
        def __init__(self):
            self.terminated=False
            self.graph=NS(frames=NS(data={'need_interp':NS(tensor=torch.tensor([False,True])),
                'pose':NS(tensor=torch.tensor([[0.,0.,0.,0.,0.,0.,1.]]*2)),
                'time_ns':NS(tensor=torch.tensor([100,200]))}),match=NS(index=NS(tensor=torch.empty(0,dtype=torch.int64))))
            self.Frontend=NS(estimate_depth=lambda _:NS(depth=torch.ones(1,1,2,2),cov=torch.ones(1,1,2,2)))
            self.MapRefiner=Refiner();self.Optimizer=NS(start_optimize=lambda data:None,write_graph_data=lambda result,graph:None)
            self.KeypointSelector=NS(select_point=lambda *args:None);self.OutlierFilter=NS(filter=lambda *args:None)
        def run(self,frame):pass
        def run_pair(self,first,second):pass
        def terminate(self):
            self.terminated=True;self.Optimizer.write_graph_data(None,self.graph);self.MapRefiner.elaborate_map(self.graph.frames)
    system=System()
    monkeypatch.setattr(runner,'ROOT',source);monkeypatch.setattr(runner,'TD3ManifestSequence',lambda *a,**k:Sequence())
    monkeypatch.setattr(runner,'load_config',lambda _:(None,{'Odometry':{'args':{},'frontend':{'args':{}},'optimizer':{'args':{}}}}))
    monkeypatch.setattr(runner.MACVO,'from_config',lambda _:system);monkeypatch.setattr(torch.cuda,'get_device_name',lambda _:'CPU controlled fixture')
    args=Namespace(dataset=tmp_path,manifest=tmp_path/'manifest.json',output=tmp_path/'run',start=0,frames=2,
        acknowledge_provisional_geometry=True,record_uncertainty=False,record_arbiter_events=enabled)
    with pytest.raises(RuntimeError,match='No visual matches'):runner.run(args,'rgb')
    path=args.output/'rgb';r=json.loads((path/'report.json').read_text())
    assert r['status']=='FAILED' and r['completed_frames']==2
    assert r['tracking_diagnostics']['unsupported_pairs_before_refinement']==1
    assert r['tracking_diagnostics']['stored_flags_after_refinement']==0
    assert not (path/'trajectory.npz').exists()
    if enabled:
        assert r['arbiter_event_recording']['status']=='COMPLETE'
        events=[json.loads(line) for line in (path/'arbiter-events/events.jsonl').read_text().splitlines()]
        assert [e['kind'] for e in events]==['refinement_begin','refinement_end']
    else:
        assert 'arbiter_event_recording' not in r and not (path/'arbiter-events').exists()
