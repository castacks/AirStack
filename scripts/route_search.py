"""Bounded collision search. Grid nodes are for routing, never AOI partitioning."""
import heapq,time
from functools import lru_cache
import numpy as np
from scipy.spatial import cKDTree

class SearchBudgetExceeded(RuntimeError):pass

class CollisionWorld:
 def __init__(self,objects,margin=.7):
  self.lo=np.array([o['bbox_min'] for o in objects])-margin;self.hi=np.array([o['bbox_max'] for o in objects])+margin
  self.centers=(self.lo+self.hi)/2;self.tree=cKDTree(self.centers);self.radius=float(np.linalg.norm((self.hi-self.lo)/2,axis=1).max())
 def __call__(self,a,b):
  a=np.asarray(a,float);b=np.asarray(b,float);ids=self.tree.query_ball_point((a+b)/2,np.linalg.norm(b-a)/2+self.radius)
  if not ids:return True
  lo=self.lo[ids];hi=self.hi[ids];d=b-a;parallel=np.abs(d)<1e-12;div=np.where(parallel,1.,d);t1=(lo-a)/div;t2=(hi-a)/div
  near=np.where(parallel,-np.inf,np.minimum(t1,t2));far=np.where(parallel,np.inf,np.maximum(t1,t2))
  valid=~(((a<lo)|(a>hi))&parallel).any(1);valid&=np.maximum(near.max(1),0)<=np.minimum(far.min(1),1)
  return not valid.any()

class BoundedClearance:
 def __init__(self,world,seconds=2.):self.world=world;self.deadline=time.monotonic()+seconds
 def __call__(self,a,b):
  if time.monotonic()>self.deadline:raise SearchBudgetExceeded('route_search_time_budget')
  return self.world(a,b)

def lattice_connection(start,goal,clearance,max_altitude=12.,step=4.,padding=40.,max_nodes=6000):
 """A* can move sideways/backward around obstacles, unlike forward-only bends."""
 start=np.asarray(start,float);goal=np.asarray(goal,float)
 if not clearance(start,start) or not clearance(goal,goal):return None
 heights=sorted(set([float(start[2]),float(goal[2])]+[z for z in (3.5,6.,9.,12.) if z<=max_altitude]))
 if max(heights)>max_altitude+1e-6:return None
 z0=heights.index(float(start[2]));origin=start[:2];low=np.minimum(start[:2],goal[:2])-padding;high=np.maximum(start[:2],goal[:2])+padding
 def point(k):return np.r_[origin+step*np.array(k[:2]),heights[k[2]]]
 seed=(0,0,z0);cost={seed:0.};parent={};queue=[(float(np.linalg.norm(goal-start)),0.,seed)];done=set()
 while queue and len(done)<max_nodes:
  _,g,k=heapq.heappop(queue)
  if k in done:continue
  done.add(k);p=point(k)
  if np.linalg.norm(p-goal)<step*3 and clearance(p,goal):
   path=[goal.tolist(),p.tolist()]
   while k in parent:k=parent[k];path.append(point(k).tolist())
   path.reverse();path=path[1:];result=[];current=start
   while path:
    i=max(i for i,v in enumerate(path) if clearance(current,v));current=np.array(path[i]);result.append(current.tolist());path=path[i+1:]
   return result
  for dx,dy,dz in [(x,y,0) for x in (-1,0,1) for y in (-1,0,1) if x or y]+[(0,0,-1),(0,0,1)]:
   n=(k[0]+dx,k[1]+dy,k[2]+dz)
   if n in done or not 0<=n[2]<len(heights):continue
   q=point(n)
   if np.any(q[:2]<low) or np.any(q[:2]>high):continue
   ng=g+float(np.linalg.norm(q-p))
   if ng>=cost.get(n,float('inf')) or not clearance(p,q):continue
   cost[n]=ng;parent[n]=k;heapq.heappush(queue,(ng+float(np.linalg.norm(q-goal)),ng,n))
 return None

def plan_job(region,start,world,seconds=2.):
 from damage_aoi import inspection
 try:
  route,views=inspection(region,start,BoundedClearance(world,seconds),max_altitude=12.)
  return route,views,'ready' if route else 'no_route_found'
 except SearchBudgetExceeded:return [],[],'search_budget_exhausted'
