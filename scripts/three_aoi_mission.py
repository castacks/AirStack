#!/usr/bin/env python3
"""Finite three-robot validation using current semantic labels and existing executors.
AOIs use only predicted damage voxels. Geometry is used only for route clearance.
"""
import argparse,json,time,threading,math,zipfile
from pathlib import Path
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from survey_damage_regions import SurveyDamageRegions
from group_aois import group_regions
from aoi_queue import disposition,finish_assignment,can_return
from aoi_dispatch import Dispatcher
from route_search import CollisionWorld
from damage_aoi import regions, inspection as plan_inspection, connection, inspection_footprint, retrace_connection
p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--bounds',required=True);p.add_argument('--vocabulary',required=True);p.add_argument('--mission-config');a=p.parse_args()
cfg=json.load(open(a.mission_config)) if a.mission_config else {}
high=tuple(cfg.get('high_robots',[1]));low=tuple(cfg.get('low_robots',[2,3]));robots=high+low
nominal_offsets={int(k):np.array(v) for k,v in cfg.get('offsets',{'1':[0,0],'2':[0,5],'3':[0,-5]}).items()}
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
labels=json.load(open(a.vocabulary))['labels'];damage=[labels.index(v) for v in ['collapsed house','collapsed roof','debris']]
objects=json.load(open(a.bounds))['objects'];objects=[v for v in objects if v['class'] in ['house','tree','fallen_tree','streetlight','fence','car','truck','debris']]
world=CollisionWorld(objects);dispatcher=Dispatcher(world,workers=len(low));ros_resources=[]
states={};origins={};tasks=[];status={};nodes=[];pubs={};aois=[];cloud=None;started=time.time();seq=0
names={r:f'large_{i+1}' for i,r in enumerate(high)}|{r:f'small_{i+1}' for i,r in enumerate(low)}
def log(kind,data):
 with (out/'events.jsonl').open('a') as f:f.write(json.dumps(dict(wall_time=time.time(),kind=kind,data=data))+'\n')
clearance=world
def on_status(msg):
 d=json.loads(msg.data);status[d['task_id']]='completed' if d['status']=='complete' else d['status'];log('task_status',d)
def on_cloud(msg):
 global cloud
 arr=np.asarray(point_cloud2.read_points_numpy(msg,field_names=['x','y','z','label_id','state'],skip_nans=True)).reshape(-1,5)
 if 1 in origins:
  arr[:,:3]+=origins[1];cloud=arr
for rid in (0,)+robots:
 ctx=Context();rclpy.init(context=ctx,domain_id=rid);n=Node('three_run_'+str(rid),context=ctx);ex=SingleThreadedExecutor(context=ctx);ex.add_node(n)
 if rid:
  def odom(msg,r=rid):
   q=msg.pose.pose.position;states[r]=[q.x,q.y,q.z]
  def anchor(msg,r=rid):
   d=json.loads(msg.data)
   if d.get('anchored') and d.get('boot_enu') is not None:origins[r]=np.array(d['boot_enu'])
  n.create_subscription(Odometry,f'/robot_{rid}/odometry_conversion/odometry',odom,qos_profile_sensor_data)
  n.create_subscription(String,f'/robot_{rid}/rayfronts/status',anchor,10)
  n.create_subscription(String,'/dtc/rules_planner/task_status',on_status,10)
  pubs[rid]=n.create_publisher(String,'/dtc/task_queue_updates',10)
 else:pass  # AOIs use only the high drone's fresh observations below
 thread=threading.Thread(target=ex.spin,daemon=True);thread.start();nodes.append(n);ros_resources.append((ctx,n,ex,thread))
def add(rid,points,kind,meta=None):
 global seq
 new=[]
 for entry in points:
  extra = entry if isinstance(entry,dict) else {}
  point = extra.get("point",entry)
  seq+=1;point=np.array(point,float)
  # Fleet frame uses nominal relative spawn offsets; correct actual anchors per robot.
  local=point-origins[rid];local[:2]+=nominal_offsets[rid]
  t=dict(task_id=f'three_{seq:03d}',task_type='investigate',geometry_type='point',geometry={'center':local.tolist()},priority=10000-seq,status='assigned',assigned_to=names[rid],altitude_agl_m=float(point[2]),created_ts=time.time()+seq*.001,metadata=dict(label=kind,world_waypoint=point.tolist(),inspection_yaw=extra.get("yaw"),inspection_dwell_s=4.0 if extra.get("view") else 0.0,view_index=extra.get("view_index"),**(meta or {})))
  tasks.append(t);new.append(t['task_id'])
 log('assignment',dict(robot=rid,kind=kind,task_ids=new,world_waypoints=[p["point"] if isinstance(p,dict) else list(p) for p in points],metadata=meta));return new
def done(ids):return bool(ids) and all(status.get(i) in ['completed','failed','cancelled'] for i in ids)
tracker=SurveyDamageRegions(labels);processed=set();candidates=[]
phase='waiting';survey=[];inspection={r:[] for r in low};returned={};lastpub=0;survey_finished_at=None;blocked={};last_attempt={};queue_snapshot=[]
if (out/'mission_progress.json').exists():
 previous=json.load(open(out/'mission_progress.json'));tasks=previous['tasks'];status={k:('completed' if v=='complete' else v) for k,v in previous['status'].items()};seq=max([int(t['task_id'].split('_')[-1]) for t in tasks],default=0)
 statefile=json.load(open(out/'latest_state.json'));aois=statefile['aois'];phase=statefile['phase']
 survey=[t['task_id'] for t in tasks if t['metadata']['label']=='high survey']
 for rid in low:
  active=next((v for v in reversed(aois) if v['robot']==rid and v.get('inspection_status','active')=='active'),None)
  inspection[rid]=[t['task_id'] for t in tasks if t['assigned_to']==names[rid] and t['metadata']['label']=='low inspection' and active and t['metadata']['aoi']['id']==active['id']]
 for rid in robots:
  ids=[t['task_id'] for t in tasks if t['assigned_to']==names[rid] and t['metadata']['label']=='return to staging']
  if ids:returned[rid]=ids
 log('coordinator_resume',{'reason':'Restore saved tasks and mission state','tasks':len(tasks)})
try:
 while time.time()-started<cfg.get('wall_timeout_s',3000):
  now=time.time()
  budget_expired=now-started>=cfg.get('inspection_budget_s',cfg.get('wall_timeout_s',3000)-300)
  if done(survey) and survey_finished_at is None:survey_finished_at=now
  survey_drained=survey_finished_at is not None and now-survey_finished_at>=cfg.get('survey_drain_s',60)
  # Persistent evidence comes only from spatially independent high-drone views.
  changed=False
  for source in sorted((out/'fresh').glob('radseg_*.npz')):
   if source.name in processed:continue
   try:
    with np.load(source) as d:
     if int(d['robot_id']) in high and -float(d['pose_rdf'][0,1,3])>=23:
      q=d['pose_rdf'][0,:3,3];position=np.array([q[2],-q[0],-q[1]])
      changed=tracker.add(d['xyz'],d['chosen'],d['state'],position,source.name) or changed
    processed.add(source.name)
   except (OSError,ValueError,EOFError,zipfile.BadZipFile):pass
  if changed:
   candidates=group_regions(tracker.groups)
   log('region_evidence',dict(keyframes=len(tracker.poses),groups=candidates,rejected=tracker.rejected))
   for aoi in aois:
    region=next((v for v in candidates if v['region_id']==aoi['region_id']),None)
    if region and (region['damage_voxels']!=aoi['damage_voxels'] or region['lo']!=aoi['lo'] or region['hi']!=aoi['hi']):
     aoi.update(region);aoi['inspection_area']=inspection_footprint(aoi,aoi['views']);aoi['updated_wall_time']=now
     log('aoi_update',aoi.copy())
  if now-lastpub>3:
   for rid,pub in pubs.items():pub.publish(String(data=json.dumps({'tasks':[t for t in tasks if t['assigned_to']==names[rid] and status.get(t['task_id']) not in ['completed','failed','cancelled']]})))
   lastpub=now
  if len(states)==len(robots) and len(origins)==len(robots) and phase=='waiting':
   # Finite survey of the central tornado corridor. High transit clear of geometry.
   survey=[]
   for rid in high:
    targets=cfg.get('survey_routes',{}).get(str(rid),[[-45,-75,25],[-5,-75,25],[-5,-30,25],[-45,-30,25]])
    points=[]
    previous=np.array(states[rid])+origins[rid];previous[2]=max(previous[2],3.5)
    for point in targets:
     path=connection(previous,point,clearance,max_altitude=25.)
     if path is None:raise RuntimeError(f'Survey segment obstructed below25m: robot{rid}, {point}')
     points.extend(path);previous=point
    survey.extend(add(rid,points,'high survey'))
   phase='survey_and_inspect';log('started',{'origins':{k:v.tolist() for k,v in origins.items()},'scope':cfg.get('scope','central corridor; not whole-map coverage')})
  if phase=='survey_and_inspect' and len(states)==len(robots) and len(origins)==len(robots) and len(tracker.poses)>=6:
   for rid,region,route,viewpoints,diagnostic in dispatcher.collect():
    log('route_search',diagnostic)
    if not route:continue
    if budget_expired or rid in returned or disposition(region,aois,cfg['inspection_bounds'])!='pending':continue
    aoi=dict(**region,robot=rid,id=len(aois)+1,inspection_status='active',views=viewpoints,inspection_area=inspection_footprint(region,viewpoints),dispatch_evidence=dict(lo=region['lo'],hi=region['hi'],damage_voxels=region['damage_voxels'],area_m2=region['area_m2'],view_count=region['view_count']))
    aois.append(aoi);log('aoi',aoi.copy());inspection[rid]=add(rid,route,'low inspection',{'aoi':aoi.copy()})
   for rid in low:
    if rid in returned or inspection[rid] or budget_expired:continue
    previous=np.array(states[rid])+origins[rid];previous[2]=max(previous[2],3.5)
    dispatcher.submit(rid,candidates,aois,cfg['inspection_bounds'],previous)
  if phase=='survey_and_inspect' and len(states)==len(robots) and len(origins)==len(robots):
   for rid in robots:
    if rid in low and done(inspection[rid]) and rid not in returned:
     aoi=next((v for v in reversed(aois) if v['robot']==rid and v.get('inspection_status','active')=='active'),None)
     if aoi and finish_assignment(aoi,inspection[rid],status):
      aoi['finished_wall_time']=now;log('aoi_inspection_finished',aoi.copy())
      inspection[rid]=[];last_attempt[rid]=0
      continue  # next tick dispatches another region before any return decision
    own_survey=[t['task_id'] for t in tasks if t['assigned_to']==names[rid] and t['metadata']['label']=='high survey']
    start=np.array(states[rid])+origins[rid];start[2]=max(start[2],3.5)
    untried=rid in low and bool(dispatcher.eligible(rid,candidates,aois,cfg['inspection_bounds'],start))
    finished=done(own_survey) if rid in high else can_return(inspection[rid] or rid in dispatcher.jobs,survey_drained,not untried and not dispatcher.jobs,budget_expired)
    if finished and rid not in returned:
     current=np.array(states[rid])+origins[rid];home=origins[rid].copy();home[2]=25. if rid in high else 3.5
     path=connection(current,home,clearance,max_altitude=25. if rid in high else 12.)
     if path is None and rid in low:
      visited=[t['metadata']['world_waypoint'] for t in tasks if t['assigned_to']==names[rid] and t['metadata']['label']=='low inspection' and status.get(t['task_id'])=='completed']
      path=retrace_connection(current,home,visited,clearance,max_altitude=12.)
      if path is not None:log('retrace_return',dict(robot=rid,route=path))
     if path is not None:returned[rid]=add(rid,path,'return to staging')
   if len(returned)==len(robots) and all(done(v) for v in returned.values()):log('mission_complete',{'aois':aois,'status':status});break
  queue_snapshot=[dict(**v,queue_status=disposition(v,aois,cfg.get('inspection_bounds',[-70,-95,25,10])),route_failures=[dict(robot=k[0],reason=reason) for k,reason in dispatcher.failed.items() if k[1]==v['region_id']]) for v in candidates]
  metrics=dict(detected=len(candidates),queued=sum(v['queue_status']=='pending' for v in queue_snapshot),assigned=sum(v['queue_status']=='active' for v in queue_snapshot),inspected=sum(v['queue_status']=='completed' for v in queue_snapshot),inspections_completed_total=sum(v.get('inspection_status')=='completed' for v in aois),assignments_total=len(aois),planning=len(dispatcher.jobs))
  log('aoi_metrics',metrics)
  (out/'aoi_queue.json').write_text(json.dumps(dict(wall_time=now,survey_drained=survey_drained,budget_expired=budget_expired,regions=queue_snapshot,metrics=metrics),indent=2))
  with (out/'telemetry.jsonl').open('a') as f:f.write(json.dumps(dict(wall_time=now,phase=phase,positions={r:(np.array(v)+origins[r]).tolist() for r,v in states.items() if r in origins},status=status))+'\n')
  (out/'mission_progress.json').write_text(json.dumps(dict(tasks=tasks,status=status)))
  (out/'latest_state.json').write_text(json.dumps(dict(wall_time=now,phase=phase,aois=aois,aoi_queue=queue_snapshot,aoi_metrics=metrics,status=status,origins={r:v.tolist() for r,v in origins.items()}),indent=2))
  time.sleep(1)
 else:log('mission_timeout',{'status':status,'aois':aois})
 (out/'mission.json').write_text(json.dumps(dict(aois=aois,tasks=tasks,status=status,completed=len(returned)==len(robots) and all(status.get(t['task_id'])=='completed' for t in tasks)),indent=2))
finally:
 dispatcher.close()
 for ctx,node,executor,thread in ros_resources:executor.shutdown(timeout_sec=5.)
 for ctx,node,executor,thread in ros_resources:
  thread.join(timeout=5.);node.destroy_node();ctx.try_shutdown()
