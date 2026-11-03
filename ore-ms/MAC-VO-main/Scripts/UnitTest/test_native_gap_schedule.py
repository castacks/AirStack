"""Exact source identities and gap semantics; no native/GPU dependencies."""
import copy
import unittest
from Scripts.Setup.native_gap_schedule import select_frames,mapped_manifest

class MappingTests(unittest.TestCase):
    def setUp(self):
        self.rows=[{c+'_left':{'time_ns':100+i*10,'path':str(i)+'.png','index':50+i} for c in ['rgb','thermal']} for i in range(8)]
        self.decisions=[{'pair':i,'requests':['rgb','thermal'] if i!=3 else ['rgb']} for i in range(7)]
    def test_exact_rows_and_gap_identity(self):
        ids=select_frames(self.rows,self.decisions,'thermal');self.assertEqual(ids,[0,1,2,3,5,6,7])
        original={'pairs':self.rows,'metadata':{'preserved':True}}
        manifest,mapping=mapped_manifest(original,ids,'thermal')
        self.assertEqual(manifest['pairs'],[self.rows[i] for i in ids]);self.assertEqual(original['pairs'],self.rows)
        gap=mapping['intervals'][3];self.assertEqual(gap['sensor_interval_ns'],[130,150]);self.assertEqual(gap['skipped_source_frames'],1);self.assertFalse(gap['is_adjacent_source_pair'])
        manifest['pairs'][0]['thermal_left']['index']=999;self.assertEqual(self.rows[0]['thermal_left']['index'],50)
    def test_dense_identity(self):
        m,p=mapped_manifest({'pairs':self.rows},list(range(8)),'rgb');self.assertEqual(m['pairs'],self.rows);self.assertTrue(all(i['is_adjacent_source_pair'] for i in p['intervals']))
    def test_invalid_request_rejected(self):
        for req in [['rgb','rgb'],['unknown']]:
            d=copy.deepcopy(self.decisions);d[3]['requests']=req
            with self.assertRaises(ValueError):select_frames(self.rows,d,'rgb')
    def test_missing_or_reordered_pair_rejected(self):
        for d in [self.decisions[:-1],list(reversed(self.decisions))]:
            with self.assertRaises(ValueError):select_frames(self.rows,d,'rgb')
    def test_bootstrap_guard_cannot_disappear(self):
        for pair in [0,1,5,6]:
            d=copy.deepcopy(self.decisions);d[pair]['requests']=['rgb']
            with self.assertRaises(ValueError):select_frames(self.rows,d,'thermal')
    def test_nonmonotonic_clock_rejected(self):
        self.rows[4]['rgb_left']['time_ns']=110
        with self.assertRaises(ValueError):select_frames(self.rows,self.decisions,'rgb')
    def test_bad_indices_rejected(self):
        for ids in [[1,2],[0,2,2],[0,3,2],[0,8],[0,-1]]:
            with self.assertRaises(ValueError):mapped_manifest({'pairs':self.rows},ids,'rgb')

if __name__=='__main__':unittest.main()
