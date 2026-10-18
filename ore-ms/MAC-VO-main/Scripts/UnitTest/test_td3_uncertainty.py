from types import SimpleNamespace
import torch
import pypose as pp
from DataLoader import StereoData
from Module.Frontend.StereoDepth import IStereoDepth
from Module.Covariance.Project2to3 import MatchCovariance


def test_observer_floor_crossing_and_mutation_isolation():
    model = MatchCovariance(SimpleNamespace(kernel_size=3, min_flow_cov=.25,
        min_depth_cov=.05, device='cpu', match_cov_default=.25))
    frame = StereoData(T_BS=pp.identity_SE3(1), K=torch.tensor([[[100.,0,4.],[0,100.,4.],[0,0,1.]]]), baseline=torch.tensor([.2]), time_ns=[0], height=9, width=9, imageL=torch.zeros(1,3,9,9), imageR=torch.zeros(1,3,9,9))
    depth = IStereoDepth.Output(depth=torch.ones(1,1,9,9)*5)
    kp = torch.tensor([[4.,4.]])
    raw = torch.tensor([[.001,.5,0.]])
    expected = model.estimate(frame,kp,depth,None,raw.clone())
    captures = []
    def observe(values):
        captures.append({k: v.clone() for k,v in values.items() if v is not None})
        for value in values.values():
            if value is not None: value.fill_(999)
    model.diagnostic_observer = observe
    actual = model.estimate(frame,kp,depth,None,raw.clone())
    torch.testing.assert_close(actual,expected,rtol=0,atol=0)
    assert captures[0]['flow_cov_before'][0,0] == raw[0,0]
    assert captures[0]['flow_cov_after'][0,0] == .0625
    assert captures[0]['depth_variance_before'][0] < .05
    assert captures[0]['depth_variance_after'][0] == .05


def test_retention_mask_joins_back_to_selected_candidates(tmp_path):
    import numpy as np
    from Scripts.Setup.td3_uncertainty_check import raw_association
    directory = tmp_path/'uncertainty'
    directory.mkdir()
    data = {'time0_ns':np.array(10), 'K':np.eye(3)[None],
            'outlier_keep_inbound':np.array([False,True])}
    for prefix in ['grid','selected']:
        data.update({prefix+'_uv0':np.zeros((3,2)),prefix+'_uv1':np.zeros((3,2)),
            prefix+'_depth0':np.array([4.,5.,6.]),prefix+'_depth1':np.array([4.,5.,6.]),
            prefix+'_inbound':np.array([True,True,True] if prefix=='grid' else [True,False,True]),
            prefix+'_raw_flow_cov':np.ones((3,3))})
    for i in range(2):
        data.update({f'endpoint{i}_flow_cov_before':np.ones((2,3)),
                     f'endpoint{i}_depth_variance_before':np.array([0.,.1]),
                     f'endpoint{i}_depth_variance_after':np.array([.05,.1])})
    np.savez(directory/'pair-000.npz',**data)
    camera = np.repeat(np.eye(4)[None],2,axis=0)
    result = raw_association(tmp_path,camera,np.array([10,20]))
    assert result['selection_counts_candidate_inbound_retained']==[3,2,1]
    assert [result['populations'][p]['count'] for p in ['grid','selected','retained']]==[3,2,1]
    assert result['covariance_construction']['1']['floor_application_verified']
