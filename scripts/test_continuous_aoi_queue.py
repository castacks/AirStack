"""Exercise the actual coordinator dispatch/completion blocks without ROS/flight."""
import ast,time
from unittest.mock import patch
from aoi_dispatch import Dispatcher
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from aoi_queue import disposition,finish_assignment,can_return
source=ast.parse(Path(__file__).with_name('three_aoi_mission.py').read_text())
loop=next(n for n in ast.walk(source) if isinstance(n,ast.While))
blocks=[n for n in loop.body if isinstance(n,ast.If) and ast.unparse(n.test).startswith("phase == 'survey_and_inspect'")]
assert len(blocks)==2
wrapper=ast.parse('for _tick in [0]: pass')
wrapper.body[0].body=blocks
code=compile(ast.fix_missing_locations(wrapper),'actual_coordinator_blocks','exec')
def trial(fail=False):
 regions=[dict(region_id=i,stable_updates=3,center=[i,0],lo=[i,0,0],hi=[i+2,2,2],damage_voxels=70,area_m2=30,view_count=3,polygon=[]) for i in range(12)]
 g=dict(np=np,disposition=disposition,finish_assignment=finish_assignment,can_return=can_return,phase='survey_and_inspect',states={i:[0,0,3.5] for i in range(1,7)},origins={i:np.zeros(3) for i in range(1,7)},robots=(1,2,3,4,5,6),low=(3,4,5,6),high=(1,2),tracker=SimpleNamespace(poses=list(range(6))),returned={1:['r1'],2:['r2']},inspection={r:[] for r in range(3,7)},budget_expired=False,last_attempt={},blocked={},candidates=regions,aois=[],cfg={'inspection_bounds':[-100,-100,100,100],'survey_drain_s':60},clearance=lambda *a:True,tasks=[],names={i:str(i) for i in range(1,7)},status={'r1':'completed','r2':'completed'},survey_finished_at=1.,survey_drained=False,log=lambda *a:None,inspection_footprint=lambda *a:{},connection=lambda *a,**k:[[0,0,3.5]],retrace_connection=lambda *a,**k:None)
 def plan(region,previous,clearance,**kw):
  v=dict(point=[region['region_id'],0,3.5],target=[region['region_id'],0],yaw=0.)
  return [v],[v]
 g['plan_inspection']=plan
 g['dispatcher']=Dispatcher(None,4)
 patcher=patch('aoi_dispatch.plan_job',side_effect=lambda region,start,world,seconds:(*plan(region,start,world),'ready'));patcher.start()
 def add(rid,points,kind,meta=None):
  ident=str(len(g['tasks']));g['tasks'].append(dict(task_id=ident,assigned_to=str(rid),metadata=dict(label=kind,world_waypoint=points[0],**(meta or {}))));return [ident]
 g['add']=add;g['done']=lambda ids:bool(ids) and all(g['status'].get(i) in ['completed','failed','cancelled'] for i in ids)
 for tick in range(40):
  g['now']=100+tick*20
  exec(code,g)
  time.sleep(.01)
  for r,ids in g['inspection'].items():
   for ident in ids:g['status'][ident]='failed' if fail and ident=='0' else 'completed'
  assert not any(r in g['returned'] for r in g['low']), 'Returned before survey drain'
 g['dispatcher'].close();patcher.stop()
 assert len(g['aois'])==12,len(g['aois'])
 assert len({a['region_id'] for a in g['aois']})==12
 assert all(sum(a['robot']==r for a in g['aois'])==3 for r in g['low'])
 assert all(a['inspection_status'] in ['completed','failed'] for a in g['aois'])
 if fail:assert g['aois'][0]['inspection_status']=='failed'
 g.update(survey_drained=True,now=500)
 exec(code,g)
 assert all(r in g['returned'] for r in g['low'])
 print('PASS actual dispatch/completion:12 AOIs,3 per inspector, exclusive claims, survey drain, failure=',fail)
trial();trial(True)
assert not can_return(['active'],True,True,True)
assert can_return([],False,False,True)
print('PASS budget return only when active inspection released')
