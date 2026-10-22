import numpy as np
import pytest
from Scripts.Setup.td3_tracking_report import tracking_report


def test_supported_and_mixed_flags():
    supported=np.zeros(5,dtype=bool)
    assert tracking_report(supported,supported)['support_state']=='ALL_PAIRS_SUPPORTED'
    mixed=np.array([False,True,False,True,False])
    result=tracking_report(mixed,mixed)
    assert result['support_state']=='SOME_PAIRS_UNSUPPORTED'
    assert result['unsupported_frame_indices']==[1,3]


def test_boundary_clearing_does_not_hide_original_loss():
    before=np.ones(100,dtype=bool);before[0]=False
    after=before.copy();after[1:3]=False;after[-2:]=False
    original=before.copy();stored=after.copy();result=tracking_report(before,after)
    assert result['support_state']=='NO_PAIRS_SUPPORTED'
    assert result['unsupported_pairs_before_refinement']==99 and result['stored_flags_after_refinement']==95
    assert result['changed_flag_frame_indices']==[1,2,98,99]
    np.testing.assert_array_equal(before,original);np.testing.assert_array_equal(after,stored)


def test_invalid_flags_fail_explicitly():
    for a,b in [(np.zeros((3,1),bool),np.zeros((3,1),bool)),(np.zeros(1,bool),np.zeros(1,bool)),(np.zeros(3,bool),np.zeros(4,bool)),(np.zeros(3),np.zeros(3))]:
        with pytest.raises(ValueError):tracking_report(a,b)
