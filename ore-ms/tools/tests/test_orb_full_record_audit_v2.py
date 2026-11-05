from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from orb_full_record_audit_v2 import rigid_inverse

def test_rigid_block_inverse_handles_native_rotation_roundoff_without_tolerance_change():
    t=np.eye(4);t[0,0]+=1e-7;t[0,3]=1000
    got=rigid_inverse(t)
    assert got[0,3]==-(1+1e-7)*1000
    assert abs(got[0,3]-np.linalg.inv(t)[0,3])>1e-5
    assert np.linalg.norm(t[:3,:3].T@t[:3,:3]-np.eye(3))<1e-5

def test_exact_rigid_control():
    t=np.eye(4);t[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];t[:3,3]=[100,20,-7]
    np.testing.assert_array_equal(rigid_inverse(t),np.linalg.inv(t))

def test_gross_nonrigid_native_matrix_stays_rejected():
    t=np.eye(4);t[0,0]+=1e-4
    with pytest.raises(ValueError):rigid_inverse(t)
