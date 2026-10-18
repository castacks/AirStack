import numpy as np
import torch
import pypose as pp
from Module.Map import MatchObs,PointNode
from Module.Optimization.TwoFramePGO.Graphs import GraphInput
from Scripts.Setup.td3_backend_ablation import solve,compose
from Scripts.Setup.td3_reprojection import project_ned


def synthetic_input():
    rng=np.random.default_rng(4)
    p0=np.column_stack([rng.uniform(4,15,40),rng.uniform(-2,2,40),rng.uniform(-1,1,40)])
    camera0_from_camera1=np.eye(4);camera0_from_camera1[:3,3]=[.3,.04,-.02]
    p1=p0-camera0_from_camera1[:3,3]
    k=np.array([[200.,0,160.],[0,200.,120.],[0,0,1.]])
    def f(a):return torch.tensor(a,dtype=torch.float32)
    obs=MatchObs.init({'pixel2_uv':f(project_ned(p1,k)),
        'pixel2_uv_cov':f(np.tile([.25,.25,0],(len(p0),1))),
        'pixel2_disp':f((200*.3/p1[:,0])[:,None]),'pixel2_disp_cov':torch.ones(len(p0),1)*.5})
    pts=PointNode.init({'pos_Tw':f(p0),'cov_Tw':torch.eye(3).repeat(len(p0),1,1)})
    return GraphInput(torch.tensor([1]),torch.tensor([0]),pp.identity_SE3(1),torch.tensor([.3]),
        obs,pts,f(k),torch.zeros(len(p0),dtype=torch.long),'cpu'),camera0_from_camera1


def test_both_backends_recover_consistent_synthetic_motion_without_input_mutation():
    from Scripts.Setup.td3_reference import matrices
    from Scripts.Setup.td3_backend_ablation import fingerprint
    hashes=[]
    for backend in ['disp','reproj']:
        data,expected=synthetic_input();before=fingerprint(data)
        pose,metrics=solve(data,backend)
        hashes.append(metrics['input_sha256'])
        np.testing.assert_allclose(matrices(pose[None])[0],expected,atol=1e-3)
        assert fingerprint(data)==before
        assert metrics['conditioning']['condition_number']>1
    assert hashes[0]==hashes[1]


def test_composition_uses_camera0_from_camera1_order():
    from scipy.spatial.transform import Rotation
    first=np.eye(4);first[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    second=np.eye(4);second[0,3]=2
    result=compose([first,second])
    np.testing.assert_allclose(result[-1,:3,3],[0,2,0],atol=1e-12)
