"""Nonblocking, exclusive AOI route jobs with geometry/position keyed retry memory."""
import copy,time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from aoi_queue import disposition
from route_search import plan_job

class Dispatcher:
 def __init__(self,world,workers=4,seconds=3.):
  self.world=world;self.seconds=seconds;self.pool=ThreadPoolExecutor(max_workers=workers)
  self.jobs={};self.failed={};self.diagnostics=[]
 def key(self,rid,region,start):
  return (rid,region['region_id'],tuple(np.round(np.r_[region['lo'],region['hi']],1)),tuple((np.asarray(start)/4).astype(int)))
 def eligible(self,rid,candidates,assignments,bounds,start):
  claimed={j[1]['region_id'] for j in self.jobs.values()}
  return sorted((r for r in candidates if disposition(r,assignments,bounds)=='pending' and r['region_id'] not in claimed and self.key(rid,r,start) not in self.failed),key=lambda r:np.linalg.norm(np.asarray(r['center'])-np.asarray(start)[:2]))
 def submit(self,rid,candidates,assignments,bounds,start):
  if rid in self.jobs:return False
  options=self.eligible(rid,candidates,assignments,bounds,start)
  if not options:return False
  region=copy.deepcopy(options[0]);key=self.key(rid,region,start)
  self.jobs[rid]=(self.pool.submit(plan_job,region,start,self.world,self.seconds),region,key,time.time())
  return True
 def collect(self):
  result=[]
  for rid,(future,region,key,started) in list(self.jobs.items()):
   if not future.done():continue
   route,views,reason=future.result();del self.jobs[rid]
   diagnostic=dict(robot=rid,region_id=region['region_id'],reason=reason,elapsed_s=time.time()-started)
   self.diagnostics.append(diagnostic)
   if not route:self.failed[key]=reason
   result.append((rid,region,route,views,diagnostic))
  return result
 def close(self):self.pool.shutdown(wait=True,cancel_futures=True)
