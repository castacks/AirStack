import json,sys,time,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from route_search import CollisionWorld,BoundedClearance,lattice_connection,plan_job
from aoi_dispatch import Dispatcher
from damage_aoi import inspection

class DispatchTests(unittest.TestCase):
 def test_backward_detour(self):
  world=CollisionWorld([dict(bbox_min=[1,-10,0],bbox_max=[3,10,20])])
  path=lattice_connection([0,0,3.5],[8,0,3.5],BoundedClearance(world,3));self.assertTrue(path)
  previous=[0,0,3.5]
  for p in path:self.assertTrue(world(previous,p));self.assertLessEqual(p[2],12);previous=p
 def test_spatial_index_matches_full_slab(self):
  rng=np.random.default_rng(21);lo=rng.uniform(-40,40,(100,3));hi=lo+rng.uniform(1,5,(100,3));w=CollisionWorld([dict(bbox_min=a,bbox_max=b) for a,b in zip(lo,hi)])
  for a,b in rng.uniform(-60,60,(200,2,3)):
   d=b-a;t1=(w.lo-a)/d;t2=(w.hi-a)/d
   expected=not (np.maximum(np.minimum(t1,t2).max(1),0)<=np.minimum(np.maximum(t1,t2).min(1),1)).any()
   self.assertEqual(w(a,b),expected)
 def test_exclusive_claim_and_failed_candidate_progress(self):
  regions=[dict(region_id=i,stable_updates=2,center=[i,0],lo=[i,0,0],hi=[i+1,1,1]) for i in range(3)]
  with patch('aoi_dispatch.plan_job',return_value=([],[],'no_route_found')):
   d=Dispatcher(None,2);self.assertTrue(d.submit(3,regions,[],[-10,-10,10,10],[0,0,3.5]));self.assertTrue(d.submit(4,regions,[],[-10,-10,10,10],[0,0,3.5]))
   self.assertNotEqual(d.jobs[3][1]['region_id'],d.jobs[4][1]['region_id']);d.pool.shutdown(wait=True)
   self.assertEqual(len(d.collect()),2)
   self.assertNotEqual(d.eligible(3,regions,[],[-10,-10,10,10],[0,0,3.5])[0]['region_id'],0)
   self.assertEqual(len(d.eligible(3,regions,[],[-10,-10,10,10],[12,0,3.5])),3)
 def test_budget_failure_not_unreachable(self):
  world=CollisionWorld([dict(bbox_min=[100,100,0],bbox_max=[110,110,12])])
  r=dict(polygon=[[0,0],[1,0],[1,1],[0,1]],center=[.5,.5])
  self.assertEqual(plan_job(r,[0,0,3.5],world,seconds=-1)[2],'search_budget_exhausted')

if __name__=='__main__':unittest.main()
