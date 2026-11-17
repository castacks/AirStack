"""Canonical corrected audit suite; original frozen tests/source remain intact."""
import unittest,contextlib,io
import numpy as np,cv2
from scipy.spatial.transform import Rotation
import test_pre_essential_exclusion as original
import audit_pre_essential_exclusion_svd as corrected
original.audit=corrected
class SvdContract(unittest.TestCase):
    def test_nonideal_raw_matrix_has_consistent_decomposition(self):
        e=np.array([[0.,-1,0],[2,0,0],[0,0,.03]]);r,_,t=cv2.decomposeEssentialMat(e);t=t.reshape(-1);self.assertLess(corrected.svd_gap(e,r,t),1e-10);x,y,z=t;b=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])@r;a=e/np.linalg.norm(e);b=b/np.linalg.norm(b);self.assertGreater(min(np.linalg.norm(a-b),np.linalg.norm(a+b)),1e-3)
    def test_wrong_rotation_fails_same_svd_contract(self):
        e=np.array([[0.,-1,0],[2,0,0],[0,0,.03]]);r,_,t=cv2.decomposeEssentialMat(e);r=r@Rotation.from_rotvec([.2,0,0]).as_matrix();self.assertGreater(corrected.svd_gap(e,r,t.reshape(-1)),1e-5)
class FullValid(unittest.TestCase):
    def test_unmodified_full_result_passes(self):
        with contextlib.redirect_stdout(io.StringIO()):r=corrected.main(corrected.ROOT/'results/pre-essential-exclusion/source/config.json')
        self.assertEqual(r['status'],'pass');self.assertEqual(r['original_raw_E_equality_gate'],'fail_retained')
if __name__=='__main__':
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(c) for c in [original.Finite,SvdContract,original.Corruption,FullValid]]);result=unittest.TextTestRunner(verbosity=2).run(suite);raise SystemExit(not result.wasSuccessful())
