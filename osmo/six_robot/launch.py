"""Dedicated six-robot pod runner. All docker operations are pod-local."""
import os,sys,subprocess,time,json,shutil
from pathlib import Path
root=Path('/root/AirStack');out=Path('/osmo/output/mission');out.mkdir(exist_ok=True)
os.chdir(root)
env=os.environ.copy();env.update(AUTOLAUNCH='false',NUM_ROBOTS='6',COMPOSE_PROFILES='desktop,isaac-sim,offboard-compute',ISAAC_SIM_LIVESTREAM='false',DISPLAY='',DEBUG_RVIZ='false',VERSION='0.19.0-alpha.1-disaster-dataset',FROZEN_DATASET_ROOT='omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/final_disaster_dataset_revisions/2026-10-23-coordinator',ROS_AUTOMATIC_DISCOVERY_RANGE='SUBNET')
def run(args,**kw):return subprocess.run(args,check=True,env=env,**kw)
def dex(c,code,detach=False):
 return run(['docker','exec']+(['-d'] if detach else [])+[c,'bash','-c',code])
def state(status,**extra):
 (out/'run_status.json').write_text(json.dumps(dict(status=status,wall_time=time.time(),**extra),indent=2))
for f in ['mission.json','fleet.yaml','vocabulary.json','description_banks.npz','obstacle_bounds.json','spawns.json','scene_hints.json','source_provenance.json']:
 shutil.copy(root/'osmo/six_robot'/f,out/f)
shutil.copy(root/'osmo/six_robot/mission.json',out/'mission_input.json')
try:
 state('bootstrapping')
 # Older workspace entrypoints ignore OSMO_AIRSTACK_UP and create this default stack.
 # Clean only the known bootstrap project in this dedicated workflow pod.
 ids=run(['docker','ps','-aq','--filter','label=com.docker.compose.project=airstack'],capture_output=True,text=True).stdout.split()
 if ids:
  info=json.loads(run(['docker','inspect',*ids],capture_output=True,text=True).stdout)
  allowed={'airstack-gcs-1','airstack-robot-desktop-1','isaac-sim'}
  if any(c['Name'].lstrip('/') not in allowed for c in info):raise RuntimeError('Unexpected bootstrap containers; refusing cleanup')
  (out/'bootstrap_containers.json').write_text(json.dumps([dict(id=c['Id'],name=c['Name'],image=c['Config']['Image']) for c in info],indent=2))
  run(['docker','stop','-t','20',*ids]);run(['docker','rm',*ids])
 nets=run(['docker','network','ls','-q','--filter','label=com.docker.compose.project=airstack'],capture_output=True,text=True).stdout.split()
 for net in nets:
  info=json.loads(run(['docker','network','inspect',net],capture_output=True,text=True).stdout)[0]
  if info['Containers']:raise RuntimeError('Bootstrap network still occupied; refusing removal')
  run(['docker','network','rm',net])
 own=run(['nvidia-smi','--query-gpu=uuid','--format=csv,noheader'],capture_output=True,text=True).stdout.strip().splitlines()
 if not own:raise RuntimeError('No assigned GPU')
 compose=['docker','compose','-p','sixradseg','-f','docker-compose.yaml','-f','osmo/six_robot/compose.yaml']
 # Source snapshot carries the local module as an actual directory.
 run(['python3','osmo/six_robot/preflight.py','--repair-empty'])
 run(compose+['up','-d','--no-build','robot-desktop','isaac-sim','offboard-compute'])
 table=run(['docker','exec','isaac-sim','nvidia-smi','--query-gpu=index,uuid','--format=csv,noheader'],capture_output=True,text=True).stdout
 index={u.strip():i.strip() for i,u in (line.split(',') for line in table.splitlines())}
 gpu=index[own[0]];compute_gpu=index[own[-1]]
 (out/'gpu_assignment.json').write_text(json.dumps(dict(owned=own,index=index,selected=gpu,compute=compute_gpu)))
 dex('sixradseg-robot-desktop-1','bash /root/AirStack/osmo/six_robot/test_snapshot.sh > /results/deployment_tests.log 2>&1')
 for i in range(1,7):
  dex(f'sixradseg-robot-desktop-{i}',f'export ROBOT_NAME=robot_{i} ROS_DOMAIN_ID={i}; bash /root/AirStack/osmo/six_robot/robot.sh > /results/robot_{i}.log 2>&1',True)
 dex('isaac-sim',f'export ISAAC_SIM_ACTIVE_GPU={gpu}; bash /isaac-sim/AirStack/osmo/six_robot/sim.sh > /results/sim.log 2>&1',True)
 dex('offboard-compute',f'export CUDA_VISIBLE_DEVICES={compute_gpu} ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET FASTRTPS_DEFAULT_PROFILES_FILE=/root/AirStack/common/fastdds.xml; bash /root/AirStack/osmo/six_robot/perception.sh',True)
 source='source /root/AirStack/robot/ros_ws/install/setup.bash; '
 dex('sixradseg-robot-desktop-1',source+'SIX_READY_TIMEOUT_S=2400 python3 /root/AirStack/osmo/six_robot/ready.py > /results/readiness.log 2>&1')
 dex('sixradseg-robot-desktop-1',source+'python3 /root/AirStack/osmo/six_robot/coverage_preflight.py --directory /results --gt /results/gt_objects.json --output /results/coverage_report.json')
 for i in range(1,7):
  dex(f'sixradseg-robot-desktop-{i}',source+f'export ROS_DOMAIN_ID={i}; python3 /root/AirStack/scripts/capture_inspection_frames.py /results/robot_{i} --robot robot_{i} --period 0.4 > /results/capture_{i}.log 2>&1',True)
 for _ in range(180):
  if all((out/f"robot_{i}/frames.jsonl").exists() for i in range(1,7)):break
  time.sleep(1)
 else:raise RuntimeError("Not all camera recorders produced a base frame")
 # Require the actual shared encoder socket before dispatching flight tasks.
 for _ in range(600):
  if subprocess.run(['docker','exec','offboard-compute','test','-S','/tmp/rayfronts/encoder.sock']).returncode==0:break
  time.sleep(2)
 else:raise RuntimeError('Encoder failed startup')
 state('running',robots=6,high=[1,2],low=[3,4,5,6])
 dex('sixradseg-robot-desktop-1',source+'export OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2; python3 /root/AirStack/scripts/three_aoi_mission.py /results --bounds /results/obstacle_bounds.json --vocabulary /results/vocabulary.json --mission-config /results/mission.json > /results/coordinator.log 2>&1')
 mission=json.loads((out/'mission.json').read_text())
 state('completed' if mission.get('completed') else 'incomplete',mission_completed=mission.get('completed',False))
except Exception as exc:
 state('failed',error=str(exc));print(str(exc),flush=True)
finally:
 # Always retain failure diagnostics; a startup error must not consume the entire allocation.
 subprocess.run(['docker','ps','-a','--no-trunc'],stdout=(out/'containers.txt').open('w'),env=env)
 for i in range(1,7):
  pidfile=out/f'robot_{i}/capture.pid'
  if pidfile.exists():
   subprocess.run(['docker','exec',f'sixradseg-robot-desktop-{i}','kill','-INT',str(int(pidfile.read_text()))],env=env)
 time.sleep(3)
 if 'compose' in globals():
  subprocess.run(compose+['logs','--no-color','--tail','300'],stdout=(out/'container_logs.txt').open('w'),stderr=subprocess.STDOUT,env=env)
  subprocess.run(compose+['stop','-t','20'],env=env)
 (out/'artifact_manifest.json').write_text(json.dumps([dict(path=str(p.relative_to(out)),bytes=p.stat().st_size) for p in out.rglob('*') if p.is_file()],indent=2))
 try:
  run(['tar','-cf','/root/AirStack/osmo/six_results.tar','-C','/osmo/output/mission','.'])
  run(['docker','run','--rm','--network','host','--runtime','runc','--entrypoint','bash','--env-file','/root/AirStack/simulation/isaac-sim/docker/omni_pass.env','-e','SIX_RUN_ID='+env['RUN_ID'],'-v','/root/AirStack:/isaac-sim/AirStack','-v','/osmo/output/mission:/results','airlab-docker.andrew.cmu.edu/airstack/airstack:v0.19.0-alpha.1-disaster-dataset_isaac-sim','-c','export PYTHONPATH=/isaac-sim/kit/extscore/omni.client.lib; /isaac-sim/python.sh /isaac-sim/AirStack/osmo/six_robot/upload_results.py'],timeout=300)
 except Exception as upload_exc:
  print('Artifact upload failed:',upload_exc,flush=True)
  (out/'upload_error.txt').write_text(str(upload_exc))
  sys.exit(1)
sys.exit(0 if json.loads((out/'run_status.json').read_text())['status']=='completed' else 1)
