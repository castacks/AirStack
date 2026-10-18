import numpy as np
import pytest
import torch
from Scripts.UnitTest.test_td3_backend import synthetic_input
from Scripts.Setup.td3_backend_ablation import fingerprint,solve
from Scripts.Setup.td3_reference import matrices
from Scripts.Setup.td3_weight_ablation import set_weights,geometry_fingerprint


def test_weight_arms_change_only_requested_fields():
    signatures=[];control,_=synthetic_input();source_hash=fingerprint(control)
    for arm in ['learned','fixed_uv','fixed_disp','fixed_both']:
        data,_=synthetic_input();signature=set_weights(data,arm,[.4,.6,2.])
        signatures.append(signature)
        uv=data.observations.data['pixel2_uv_cov'];disp=data.observations.data['pixel2_disp_cov']
        expected=[.4,.6] if arm in ['fixed_uv','fixed_both'] else [.25,.25]
        torch.testing.assert_close(uv[:,:2],torch.tensor(expected).repeat(len(uv),1))
        torch.testing.assert_close(disp,torch.full_like(disp,2. if arm in ['fixed_disp','fixed_both'] else .5))
        assert (uv[:,2]==0).all()
        assert geometry_fingerprint(data)==geometry_fingerprint(control)
    assert len(set(signatures))==1 and fingerprint(control)==source_hash


@pytest.mark.parametrize('values',[[0,1,1],[-1,1,1],[np.nan,1,1],[1,np.inf,1],[1,1]])
def test_invalid_constants_are_rejected(values):
    data,_=synthetic_input()
    with pytest.raises(ValueError):set_weights(data,'fixed_both',values)


def test_fixed_weights_recover_consistent_motion():
    data,expected=synthetic_input();set_weights(data,'fixed_both',[.4,.6,2.])
    pose,_=solve(data,'disp')
    np.testing.assert_allclose(matrices(pose[None])[0],expected,atol=1e-3)
