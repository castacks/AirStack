"""GT validates test opportunity only; this report never selects operational AOIs."""
import argparse,json,sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
from route_search import CollisionWorld

def audit(cfg,spawns,objects,obstacles):
 world=CollisionWorld(obstacles);humans=[o for o in objects if o['class']=='survivor']
 centers=np.array([(np.array(o['bbox_min'])+o['bbox_max'])/2 for o in humans]);assert len(centers),'No human GT available'
 b=cfg['inspection_bounds'];in_scope=(centers[:,0]>=b[0])&(centers[:,0]<=b[2])&(centers[:,1]>=b[1])&(centers[:,1]<=b[3])
 covered=np.zeros(len(humans),bool);by_robot={}
 for rid,route in cfg['survey_routes'].items():
  route=np.asarray(route);assert np.all(route[:,2]==25.)
  near=np.full(len(humans),np.inf)
  for a,z in zip(route[:-1],route[1:]):
   assert world(a,z),f'Blocked high survey segment: {rid}, {a}, {z}'
   d=z[:2]-a[:2];t=np.clip(((centers[:,:2]-a[:2])@d)/max(d@d,1e-9),0,1)
   near=np.minimum(near,np.linalg.norm(centers[:,:2]-(a[:2]+t[:,None]*d),axis=1))
  eligible=(near<=cfg['coverage_max_route_distance_m'])&in_scope;covered|=eligible
  by_robot[rid]=dict(humans_near_route=int(eligible.sum()),route_length_m=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum()))
  s=spawns[int(rid)-1];assert world([s['x_m'],s['y_m'],3.5],route[0]),f'Blocked initial survey transit: {rid}'
 for i,s in enumerate(spawns,1):assert world([s['x_m'],s['y_m'],s['z_m']],[s['x_m'],s['y_m'],25. if i in cfg['high_robots'] else 3.5]),f'Blocked spawn {i}'
 assert covered.sum()>=cfg['coverage_min_humans'],f'Insufficient GT survey opportunity: {covered.sum()}'
 return dict(passed=True,total_humans=len(humans),humans_in_inspection_scope=int(in_scope.sum()),humans_near_survey=int(covered.sum()),by_robot=by_robot,qualification='Geometric opportunity only; visibility, perception detections, and inspection coverage must be measured in the actual run. GT is not supplied to AOI selection.')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--directory',default=str(Path(__file__).parent));p.add_argument('--gt');p.add_argument('--output');a=p.parse_args();d=Path(a.directory)
 report=audit(json.loads((d/'mission.json').read_text()),json.loads((d/'spawns.json').read_text()),json.loads(Path(a.gt or d/'coverage_gt.json').read_text())['objects'],json.loads((d/'obstacle_bounds.json').read_text())['objects'])
 print(json.dumps(report,indent=2))
 if a.output:Path(a.output).write_text(json.dumps(report,indent=2))
