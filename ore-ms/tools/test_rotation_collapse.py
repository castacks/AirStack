"""Identity, wrong-axis, missing-output and corruption controls."""
import contextlib,io,json,tempfile,shutil,unittest
import numpy as np
from scipy.spatial.transform import Rotation
from rotation_collapse import angle_row,join_row,stats,ROOT,sha,degrees
import audit_rotation_collapse as audit

def pred(angle=0,status='ok'):
    return dict(frontend='learned',arm='rgb',index=0,status=status,R=Rotation.from_euler('x',angle,degrees=True).as_matrix().tolist())
def score(truth,error):return dict(frontend='learned',arm='rgb',index=0,status='ok',reference_rotation_deg=truth,R_error=error,R1=error<=1,R5=error<=5)
class Finite(unittest.TestCase):
    def test_identity_can_be_correct_or_wrong(self):
        for truth,error in [(.05,.05),(10,10)]:
            r=join_row(angle_row(pred()),score(truth,error));s=stats([r],{0});self.assertTrue(r['near_identity']);self.assertEqual(s['R5_wrong_near_identity'],int(truth>5));self.assertEqual(r['error_minus_identity_deg'],0)
    def test_rejected_intermediate_R_is_not_an_output(self):
        r=angle_row(pred(12,'failed'));self.assertIsNone(r['estimated_deg']);self.assertFalse(r['near_identity'])
    def test_equal_amount_wrong_axes_are_wrong_orientation(self):
        p=pred(10);truth=Rotation.from_euler('y',10,degrees=True).as_matrix();error=degrees(np.array(p['R'])@truth.T);r=join_row(angle_row(p),score(10,error));self.assertAlmostEqual(r['motion_ratio'],1);self.assertFalse(r['near_identity']);self.assertGreater(r['error_deg'],5)
    def test_fixed_boundary_and_impossible_triangle(self):
        a=angle_row(pred(.05));a['estimated_deg']=.1;a['near_identity']=True;s=stats([join_row(a,score(10,10))],{0});self.assertEqual(s['near_identity_outputs'],1)
        with self.assertRaises(AssertionError):join_row(angle_row(pred()),score(10,0))
class Corruption(unittest.TestCase):
    def setUp(self):
        source=ROOT/'results/rotation-collapse/source/config.json'
        if not source.exists():self.skipTest('Real diagnostic not scored yet')
        cfg=json.loads(source.read_text());original=ROOT/cfg['output']
        if not (original/'completion.json').exists():self.skipTest('Real diagnostic not scored yet')
        self.temp=tempfile.TemporaryDirectory(dir=ROOT/'results',prefix='rotation-collapse-tests-');self.out=__import__('pathlib').Path(self.temp.name)/'out';shutil.copytree(original,self.out);cfg['output']=str(self.out.relative_to(ROOT));self.config=self.out.parent/'config.json';self.config.write_text(json.dumps(cfg));f=json.loads((self.out/'input-freeze.json').read_text());f['pins'][str(self.config.relative_to(ROOT))]=sha(self.config);(self.out/'input-freeze.json').write_text(json.dumps(f))
    def tearDown(self):
        if hasattr(self,'temp'):self.temp.cleanup()
    def test_valid(self):
        with contextlib.redirect_stdout(io.StringIO()):self.assertEqual(audit.main(self.config)['status'],'pass')
    def corrupt(self,file,mutation):
        p=self.out/(file+'.json');d=json.loads(p.read_text());mutation(d);p.write_text(json.dumps(d));r=self.out/('angle-lock.json' if file=='angles' else 'completion.json');d=json.loads(r.read_text());d[file+'_sha256']=sha(p);r.write_text(json.dumps(d))
        with contextlib.redirect_stdout(io.StringIO()),self.assertRaises(AssertionError):audit.main(self.config)
    def test_reject_changed_angle(self):self.corrupt('angles',lambda d:d['FlashLight'][0].update(estimated_deg=99))
    def test_reject_identity_error_gap(self):self.corrupt('rows',lambda d:d['FlashLight'][0].update(error_minus_identity_deg=999))
    def test_reject_summary(self):self.corrupt('summary',lambda d:d['SmokeRoom']['learned']['rgb']['paired_rotation_ge2deg'].update(R5_wrong_near_identity=999))
if __name__=='__main__':unittest.main()
