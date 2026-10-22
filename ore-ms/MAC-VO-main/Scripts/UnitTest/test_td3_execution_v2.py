"""Exercise real runner failure reporting without a model or GPU inference."""
from argparse import Namespace
import json
from types import SimpleNamespace
import numpy as np
import pytest
import torch
from Scripts.Setup import run_td3_execution_v2 as runner


def test_zero_observation_failure_keeps_pre_refinement_diagnostics(tmp_path,monkeypatch):
    source=tmp_path/'MAC-VO-main';source.mkdir();weights=tmp_path/'models/macslam';weights.mkdir(parents=True);(weights/'MACVO_FrontendCov.pth').write_bytes(b'fixture')
    rows=[{'rgb_left':{'time_ns':n}} for n in [100,200]]
    class Sequence:
        def __init__(self):
            self.rows=rows;self.indices=np.arange(2);self.signed_baseline=-.2;self.baseline=.2;self.K=np.eye(3)[None]
        def __len__(self):return 2
        def clip(self,**kwargs):pass
        def __getitem__(self,i):return SimpleNamespace(stereo=object())
        def __iter__(self):return iter([self[0],self[1]])
    class System:
        def __init__(self):
            self.terminated=False
            data={'need_interp':SimpleNamespace(tensor=torch.tensor([False,True])),
                  'pose':SimpleNamespace(tensor=torch.tensor([[0.,0.,0.,0.,0.,0.,1.]]*2)),
                  'time_ns':SimpleNamespace(tensor=torch.tensor([100,200]))}
            self.graph=SimpleNamespace(frames=SimpleNamespace(data=data),match=SimpleNamespace(index=SimpleNamespace(tensor=torch.empty(0,dtype=torch.int64))))
            self.Frontend=SimpleNamespace(estimate_depth=lambda _:SimpleNamespace(depth=torch.ones(1,1,2,2),cov=torch.ones(1,1,2,2)))
        def run(self,frame):pass
        def terminate(self):
            self.graph.frames.data['need_interp'].tensor[1]=False;self.terminated=True
    system=System()
    monkeypatch.setattr(runner,'ROOT',source)
    monkeypatch.setattr(runner,'TD3ManifestSequence',lambda *args,**kwargs:Sequence())
    monkeypatch.setattr(runner,'load_config',lambda _: (None,{'Odometry':{'args':{},'frontend':{'args':{}},'optimizer':{'args':{}}}}))
    monkeypatch.setattr(runner.MACVO,'from_config',lambda _:system)
    monkeypatch.setattr(torch.cuda,'get_device_name',lambda _: 'controlled CPU fixture')
    args=Namespace(dataset=tmp_path,manifest=tmp_path/'manifest.json',output=tmp_path/'output',start=0,frames=2,acknowledge_provisional_geometry=True,record_uncertainty=False)
    with pytest.raises(RuntimeError,match='No visual matches'):runner.run(args,'rgb')
    report=json.loads((args.output/'rgb/report.json').read_text())
    assert report['status']=='FAILED' and report['completed_frames']==2
    assert report['tracking_diagnostics']['support_state']=='NO_PAIRS_SUPPORTED'
    assert report['tracking_diagnostics']['unsupported_pairs_before_refinement']==1
    assert report['tracking_diagnostics']['stored_flags_after_refinement']==0
    assert report['tracking_diagnostics']['retained_observations']==0
    with np.load(args.output/'rgb/tracking-flags.npz') as data:
        assert data['before_refinement'].tolist()==[False,True]
        assert data['after_refinement'].tolist()==[False,False]
    assert not (args.output/'rgb/trajectory.npz').exists()
