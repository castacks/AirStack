"""Isolated Office episodes on the existing AirStack/PX4 control path."""
import argparse,datetime,fcntl,hashlib,json,math,os,re,shutil,subprocess,time,urllib.request
from pathlib import Path
import yaml
from conditions import validate
from run_conditions import RUNTIME,HERE,bridge_url,apply,scene_command
from ravi_metrics import summarize
from mission import defaults,motion_metrics,horizon_outcome
from operator_control import request,UserStop

WORKSPACE=HERE.parents[2]
ROBOT='airstack-robot-desktop-1'
CONTAINER_RUNTIME='/root/AirStack/robot/ros_ws/ws2_runtime'
IMAGES={'mononav':'mononav-demo:1.0','kim':'collision-avoidance-airstack:1.0'}
REPOS={'mononav':WORKSPACE/'MonoNav','kim':WORKSPACE/'Collision-avoidance'}
WORKERS={'mononav':'mononav_airstack.py','kim':'collision_avoidance_airstack.py'}

def resolved(raw):
    if not isinstance(raw,dict):raise ValueError('episode must be a mapping')
    allowed={'name','planner','condition','height','goal_distance','goal_radius','timeout','fault','patch_start_s','patch_duration_s',*defaults('kim')}
    if raw.keys()-allowed:raise ValueError('unknown episode keys: '+str(raw.keys()-allowed))
    c=dict(name='office',planner='kim',height=1.2,fault=None,**defaults(raw.get('planner','kim')))
    c.update(raw);c['condition']=validate(c.get('condition',{'layout':'stock','light':2200}))
    if c['planner'] not in IMAGES:raise ValueError('planner must be kim or mononav')
    if c['mission_mode']!=defaults(c['planner'])['mission_mode']:raise ValueError('mission mode must match model: Kim avoidance, MonoNav goal')
    if c['fault'] not in (None,'bridge_unavailable','contact_probe'):raise ValueError('unknown fault injection')
    if not isinstance(c['name'],str) or not c['name']:raise ValueError('name required')
    for key in ['patch_start_s','patch_duration_s']:
        if key in c:
            v=c[key]
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not 0<=v<=300:
                raise ValueError(key+' must be 0..300 simulation seconds')
    for key,lo,hi in [('height',.6,1.6),('goal_distance',.5,30),('goal_radius',.1,1),('timeout',.1,600),
                      ('minimum_travel',0,100),('minimum_displacement',0,30),('maximum_stationary_fraction',0,1),('initial_speed',0,.7),
                      ('maximum_speed',.1,.7),('trajectory_horizon',.5,3),('velocity',.1,.7)]:
        v=c[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError(key+' out of range')
        c[key]=float(v)
    return c

def fingerprint(c):return hashlib.sha256(json.dumps(c,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def patch_active(c,elapsed):
    return bool(c['condition']['patch_enabled'] and elapsed>=c.get('patch_start_s',0) and
                (not c.get('patch_duration_s',0) or elapsed<c.get('patch_start_s',0)+c['patch_duration_s']))
def atomic(path,value):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(path)
def sha256_file(path):
    digest=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()
def cmd(argv,log=None,timeout=45,check=True,env=None):
    r=subprocess.run(argv,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,timeout=timeout,env=env)
    if log:
        with log.open('a') as f:f.write('$ '+repr(argv)+'\n'+r.stdout+'\n')
    if check and r.returncode:raise RuntimeError(f'{argv[0]} exited {r.returncode}: {r.stdout[-1500:]}')
    return r.stdout
def robot(shell,*args,**kwargs):return cmd(['docker','exec',ROBOT,'bash','-lc',shell,'ws2',*args],**kwargs)
def stop_owned(log):
    running=cmd(['docker','ps','--format','{{.Names}}']).splitlines()
    owned=[x for x in ['ws2-episode-worker','collision-avoidance-airstack','mononav-airstack','isaac-sim',ROBOT] if x in running]
    if owned:cmd(['docker','stop','--timeout','3',*owned],log,timeout=30,check=False)
def state():return json.loads((RUNTIME/'scene_status.json').read_text())
def health(url):
    with urllib.request.urlopen(url+'/health',timeout=2) as r:return json.load(r)
def action(name,out,height=1.2):
    return robot('sws && python3 -u /tmp/ws2_flight_action.py "$1" --height "$2"',name,str(height),log=out/(name+'.log'),timeout=140)

def worker_command(c,run_id=''):
    method=c['planner'];repo=REPOS[method]
    command=['docker','run','--rm','--name','ws2-episode-worker','--gpus','all','--ipc=host','--network','airstack_airstack_network',
             '-v',str(repo)+':/workspace/planner','-w','/workspace/planner',
             '-v',str(RUNTIME/'inference')+':/ws2_inference',
             '-e','WS2_INFERENCE_DIR=/ws2_inference','-e','WS2_RUN_ID='+run_id]
    if method=='mononav':command+=['-v','mononav-torch-cache:/root/.cache/torch']
    command += [IMAGES[method],'python','-u',WORKERS[method],'--headless','--execute','--server','http://'+ROBOT+':8765']
    if method=='kim':command+=['--depth-source','fcrn','--rate','3','--initial-speed',str(c['initial_speed']),
                              '--maximum-speed',str(c['maximum_speed']),'--trajectory-horizon',str(c['trajectory_horizon'])]
    else:command+=['--depth-source','zoe','--zoe-depth-scale','1.68','--rate','1','--warmup-frames','6',
                   '--velocity',str(c['velocity']),'--goal-distance',str(c['goal_distance']),'--goal-radius',str(c['goal_radius']),
                   '--min-tsdf-points','1000','--tsdf-local-radius','8']
    return command

def run_episode(raw,output,wait_for_recording=False,camera='overview',record_bag=True,control_path=None):
    c=resolved(raw);out=Path(output).resolve()
    # Artifact/bag paths must remain inside the shared runtime mount.
    out.relative_to(RUNTIME.resolve());out.mkdir(parents=True,exist_ok=False)
    log=out/'lifecycle.log';events=(out/'events.jsonl').open('w')
    def event(name,**data):
        events.write(json.dumps(dict(event=name,wall_time=time.time(),**data))+'\n');events.flush()
        atomic(RUNTIME/'live_episode.json',{'planner':c['planner'],'scenario':c['name'],
               'event':name,'phase':phase,'condition':c['condition'],'wall_time':time.time(),
               'sample':data if name=='sample' else None,'result':data.get('result'),'directory':str(out)})
    lock=(RUNTIME/'sequence.lock').open('w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    atomic(out/'scenario.json',c);(out/'scenario.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
    result={'schema_version':3,'scenario_id':c['name'],'configuration_hash':fingerprint(c),'outcome':'infrastructure_error',
            'termination':{'source':'runner','reason':'not_started'},'result_dir':str(out),'metrics':{}}
    samples=[];goal=None;reached=None;commands=0;worker=None;bag=None;airborne=False;recording=False;mission_start=None
    phase='preflight';started=time.monotonic();workerlog=None
    (RUNTIME/'inference').mkdir(exist_ok=True)
    result['bag_recorded']=False
    event('starting')
    def terminate(outcome,reason,detail=None):
        result.update(outcome=outcome,termination={'source':phase,'reason':reason,'detail':detail})
    def operator_check(sim_ready=False):
        nonlocal phase
        choice=request(control_path)
        if choice=='stop':raise UserStop('operator requested stop')
        if choice!='pause':return False
        previous_phase=phase
        worker_paused=worker is not None and worker.poll() is None
        if worker_paused:cmd(['docker','pause','ws2-episode-worker'],log)
        if sim_ready:scene_command({'paused':True})
        phase='paused';event('paused')
        try:
            while request(control_path)=='pause':time.sleep(.2)
        finally:
            if sim_ready:scene_command({'paused':False})
            if worker_paused:cmd(['docker','unpause','ws2-episode-worker'],log,check=False)
            phase=previous_phase;event('resumed')
        if request(control_path)=='stop':raise UserStop('operator requested stop')
        return True
    try:
        operator_check()
        stop_owned(log)
        if (RUNTIME/'flight_guard.json').exists():
            shutil.move(RUNTIME/'flight_guard.json',out/'previous_guard.json')
        for f in ['scene_status.json','scene_reply.json','scene_command.json']:(RUNTIME/f).unlink(missing_ok=True)
        provenance={}
        for name,repo in {'AirStack':HERE.parents[1],**REPOS}.items():
            provenance[name]={'head':cmd(['git','-C',str(repo),'rev-parse','HEAD']).strip(),
                              'diff_sha256':hashlib.sha256(cmd(['git','-C',str(repo),'diff']).encode()).hexdigest()}
        provenance['worker_image']=json.loads(cmd(['docker','image','inspect',IMAGES[c['planner']]]))[0]['Id']
        provenance['runtime_images']={name:json.loads(cmd(['docker','inspect',name]))[0]['Image'] for name in ['isaac-sim',ROBOT]}
        provenance['runtime_sources']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')}
        provenance['layout_catalog_sha256']=sha256_file(HERE/'layouts.json')
        provenance['patch']=json.loads((HERE/'assets/patch_manifest.json').read_text())
        if sha256_file(HERE/'assets/learned_patch.png')!=provenance['patch']['sha256']:
            raise ValueError('Installed patch differs from its manifest')
        if c['planner']=='kim':
            model_files=[*REPOS['kim'].glob('airstack_models/NYU_FCRN-checkpoint/NYU_FCRN.ckpt.*'),REPOS['kim']/'save_model/D3QN_V_3_single.h5']
            provenance['model_weights']={str(p.relative_to(REPOS['kim'])):sha256_file(p) for p in model_files}
        else:
            hash_code="import hashlib,json,pathlib; p=pathlib.Path('/cache/hub/checkpoints/ZoeD_M12_N.pt'); h=hashlib.sha256(); f=p.open('rb'); [h.update(b) for b in iter(lambda:f.read(1048576),b'')]; print(json.dumps({p.name:h.hexdigest()}))"
            provenance['model_weights']=json.loads(cmd(['docker','run','--rm','--network','none','-v','mononav-torch-cache:/cache:ro',
                '--entrypoint','python',IMAGES['mononav'],'-c',hash_code],log,timeout=30))
        atomic(out/'provenance.json',provenance)
        initial_condition=dict(c['condition'],patch_enabled=patch_active(c,0))
        atomic(RUNTIME/'episode.json',{'condition':initial_condition,'spawn':[-4,0,.07],'fault':c['fault']})
        env=os.environ.copy();env.update(WS2_CONTROL_MODE='bench',WS2_EPISODE_CONFIG='/isaac-sim/AirStack/robot/ros_ws/ws2_runtime/episode.json',
            WS2_PATCH_TEXTURE='/isaac-sim/AirStack/tools/ws2_bench/assets/learned_patch.png',WS2_PATCH_KIND='Rui learned FCRN patch')
        phase='startup';event('starting_simulator');cmd(['bash',str(HERE/'start_existing.sh')],log,timeout=60,env=env)
        url=bridge_url();deadline=time.monotonic()+180;previous=None;ready_count=0
        while time.monotonic()<deadline:
            if request(control_path)=='stop':raise UserStop('operator requested stop during startup')
            try:
                s=state();h=health(url)
                if s['oracle']['available'] and h['ready'] and s['sim_time']>3 and s['sim_time']!=previous:
                    ready_count+=1;previous=s['sim_time']
                    if ready_count>=3:break
            except (OSError,ValueError,KeyError):pass
            time.sleep(.3)
        else:raise RuntimeError('fresh simulator/oracle/camera readiness timed out')
        event('ready',scene=s)
        operator_check(True)
        if c['fault']=='bridge_unavailable':
            robot("pkill -TERM -f '^/usr/bin/python3 /root/AirStack/robot/ros_ws/install/mononav_bridge/lib/mononav_bridge/mononav_bridge_node'",log=log)
            try:health(url)
            except OSError:raise RuntimeError('injected unavailable bridge detected before takeoff')
            raise RuntimeError('bridge fault injection did not stop bridge')
        metadata,jpeg=apply(initial_condition,url)
        scene_command({'camera':camera})
        atomic(out/'condition_evidence.json',{'requested':c['condition'],'frame':metadata,'scene':state()})
        (out/'initial_planner_input.jpg').write_bytes(jpeg)
        if wait_for_recording:
            event('waiting_for_recording')
            input('READY ON GROUND. Start your screen recording, then press Enter here to record bag and take off: ')
        cmd(['docker','cp',str(HERE/'flight_action.py'),ROBOT+':/tmp/ws2_flight_action.py'],log)
        phase='recording';relative=out.relative_to(RUNTIME.resolve());bagpath=CONTAINER_RUNTIME+'/'+str(relative)+'/bag'
        topics=['/clock','/tf','/tf_static','/ws2/ground_truth/pose','/ws2/ground_truth/oracle',
                '/robot_1/odometry_conversion/odometry','/robot_1/sensors/front_stereo/left/image_rect',
                '/robot_1/sensors/front_stereo/left/camera_info','/robot_1/sensors/front_stereo/left/depth_ground_truth',
                '/robot_1/vision_planner/status','/robot_1/trajectory_controller/trajectory_override',
                '/robot_1/interface/mavros/state','/robot_1/interface/cmd_roll_pitch_yawrate_thrust',
                '/vision_planner/planner_input/compressed','/vision_planner/input_metadata']
        if record_bag:
            baglog=(out/'bag.log').open('w')
            bag=subprocess.Popen(['docker','exec',ROBOT,'bash','-lc',
                    'sws && echo $$ > /tmp/ws2_episode_bag.pid && exec ros2 bag record --use-sim-time --storage mcap --storage-preset-profile zstd_fast -o "$1" "${@:2}"','ws2',bagpath,*topics],stdout=baglog,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+20
            while not list((out/'bag').glob('*.mcap')):
                if bag.poll() is not None:raise RuntimeError('bag recorder exited')
                if time.monotonic()>deadline:raise TimeoutError('bag recorder startup')
                time.sleep(.2)
            recording=True;result['bag_recorded']=True;event('recording_started',sim_time=state()['sim_time'])
        ground_until=state()['sim_time']+2
        while state()['sim_time']<ground_until:time.sleep(.1)
        phase='takeoff';event('taking_off');action('takeoff',out,c['height']);airborne=True
        s=state()
        if abs(s['position'][2]-c['height'])>.3:raise RuntimeError('takeoff action succeeded but GT height is wrong')
        if s['oracle']['collision']:raise RuntimeError('collision during takeoff: '+str(s['oracle']['collision']))
        if c['fault']=='contact_probe':
            phase='oracle_validation';scene_command({'oracle_validation':True})
            deadline=time.monotonic()+15
            while time.monotonic()<deadline:
                probe=state()['oracle']
                if (RUNTIME/'flight_guard.json').exists():probe=json.loads((RUNTIME/'flight_guard.json').read_text()).get('oracle',probe)
                if probe['collision']:
                    terminate('collision','injected_contact_probe',probe['collision'])
                    result['validation_only']=True
                    return result
                time.sleep(.05)
            raise RuntimeError('injected collider was not detected')
        goal=[s['position'][0]+c['goal_distance'],s['position'][1],s['position'][2]] if c['mission_mode']=='goal' else None
        atomic(out/'mission.json',{'start':s['position'],'goal':goal,'radius':c['goal_radius'] if goal else None,
                                  'mode':c['mission_mode'],'duration_s':c['timeout'],'takeoff_sim_time':s['sim_time']})
        phase='model_loading';event('loading_model')
        workerlog=(out/'worker.log').open('w');argv=worker_command(c,str(out));atomic(out/'worker_command.json',argv)
        worker=subprocess.Popen(argv,stdout=workerlog,stderr=subprocess.STDOUT)
        phase='planner';deadline=time.monotonic()+120;lastsim=s['sim_time'];last_advance=time.monotonic();last_command=None
        while True:
            if operator_check(True):
                last_advance=time.monotonic();deadline=time.monotonic()+120
            s=state();h=health(url);t=s['sim_time'];oracle=s['oracle']
            if mission_start is not None:
                enabled=patch_active(c,t-mission_start)
                if s['condition']['patch_enabled']!=enabled:
                    reply=scene_command({'condition':dict(c['condition'],patch_enabled=enabled)})
                    event('patch_activation',enabled=enabled,sim_time=reply['sim_time'],request_sim_time=t)
            if t!=lastsim:last_advance=time.monotonic();lastsim=t
            if time.monotonic()-last_advance>5:raise RuntimeError('simulation clock stopped')
            if not oracle['available']:raise RuntimeError('collision oracle unavailable')
            if t-oracle['sim_time']>.25:raise RuntimeError('collision oracle stopped updating')
            if h.get('image_stamp') is not None and t-h['image_stamp']>c['condition']['delay']+2:
                raise RuntimeError('camera/bridge data stale beyond configured delay')
            if recording and bag.poll() is not None:raise RuntimeError('bag recorder stopped during episode')
            command=h.get('last_command')
            if command and command.get('published') and command['stamp']!=last_command:
                last_command=command['stamp'];commands+=1;event('planner_command',command=command)
                if mission_start is None:mission_start=t;event('mission_started',sim_time=t)
            if oracle['collision']:
                if mission_start is not None:
                    samples.append({'sim_time_s':t,'position_m':s['position'],'clearance_m':oracle['clearance_m'],
                                    'clearance_censored':oracle['clearance_censored']})
                terminate('collision','physx_contact',oracle['collision']);break
            if (RUNTIME/'flight_guard.json').exists():
                guard=json.loads((RUNTIME/'flight_guard.json').read_text())
                collision=guard.get('oracle',{}).get('collision')
                terminate('collision' if collision else 'planner_stopped','physx_contact' if collision else 'flight_envelope_guard',guard);break
            if worker.poll() is not None:
                if mission_start is not None:
                    terminate('planner_stopped','planner_process_exit',worker.returncode);break
                raise RuntimeError('planner process exited before execution: '+str(worker.returncode))
            text=(out/'worker.log').read_text()
            if mission_start is not None:
                if not samples or samples[-1]['sim_time_s']!=t:
                    sample={'sim_time_s':t,'position_m':s['position'],'clearance_m':oracle['clearance_m'],
                            'clearance_censored':oracle['clearance_censored']}
                    samples.append(sample);event('sample',**sample)
                if goal is not None and math.dist(s['position'],goal)<=c['goal_radius']:
                    reached=len(samples)-1;terminate('goal_reached','goal_region');break
                if t-mission_start>=c['timeout']:terminate(horizon_outcome(c,samples),'simulation_time_budget');break
            elif time.monotonic()>deadline:
                if 'frame=' in text:terminate('planner_stopped','no_executable_plan');break
                raise RuntimeError('planner startup timeout without processed frames')
            if 'Mission stopping (' in text or 'ALTITUDE HOLD' in text:
                terminate('planner_stopped','planner_terminal',text[-1500:]);break
            time.sleep(.1)
        result['planner_command_count']=commands
        result['metrics']=summarize(samples,goal,[],.25,reached,text.count('HOLD'),text.count('RECOVERY'))
        result['metrics'].update(motion_metrics(samples))
        clearances=[x['clearance_m'] for x in samples]
        result['metrics'].update(minimum_obstacle_clearance_m=min(clearances) if clearances else None,
            mean_obstacle_clearance_m=(sum((a['clearance_m']+b['clearance_m'])/2*(b['sim_time_s']-a['sim_time_s']) for a,b in zip(samples,samples[1:]))/(samples[-1]['sim_time_s']-samples[0]['sim_time_s'])) if len(samples)>1 else (clearances[0] if clearances else None),
            clearance_method='PhysX collider distance minus 0.25m spherical envelope; ground/ceiling included',
            mission_duration_sim_s=None if mission_start is None else s['sim_time']-mission_start,
            planner_wall_duration_s=time.monotonic()-started)
        if result['outcome']!='goal_reached':result['metrics']['path_efficiency']=None
    except (UserStop,KeyboardInterrupt) as exc:
        terminate('user_stopped',str(exc) or 'operator interrupt');event('operator_stopped')
        result['planner_command_count']=commands
        result['metrics']=summarize(samples,goal,[],.25,None,0,0)
        result['metrics'].update(motion_metrics(samples))
        clearances=[x['clearance_m'] for x in samples if x.get('clearance_m') is not None]
        duration=samples[-1]['sim_time_s']-samples[0]['sim_time_s'] if len(samples)>1 else 0.
        result['metrics'].update(path_efficiency=None,
            mission_duration_sim_s=None if mission_start is None or not samples else samples[-1]['sim_time_s']-mission_start,
            minimum_obstacle_clearance_m=min(clearances) if clearances else None,
            mean_obstacle_clearance_m=(sum((a['clearance_m']+b['clearance_m'])/2*(b['sim_time_s']-a['sim_time_s']) for a,b in zip(samples,samples[1:]))/duration) if duration else (clearances[0] if clearances else None))
    except Exception as exc:
        terminate('infrastructure_error',str(exc));event('error',phase=phase,message=str(exc))
    finally:
        phase='cleanup';cleanup=[];event('evaluating_and_cleanup')
        if worker is not None:
            try:cmd(['docker','stop','--timeout','3','ws2-episode-worker'],log,check=False);worker.wait(timeout=10)
            except Exception as exc:cleanup.append(str(exc))
        if airborne and result['outcome']!='collision' and not (RUNTIME/'flight_guard.json').exists():
            try:action('pause',out);action('land',out);event('landed',scene=state())
            except Exception as exc:cleanup.append('landing: '+str(exc))
        if recording:
            try:
                robot('kill -INT "$(cat /tmp/ws2_episode_bag.pid)"',log=log,check=False)
                bag.wait(timeout=25)
                if not (out/'bag/metadata.yaml').exists():cleanup.append('bag metadata missing')
                elif airborne:
                    cmd(['docker','cp',str(HERE/'verify_bag.py'),ROBOT+':/tmp/ws2_verify_bag.py'],log)
                    robot('sws && python3 /tmp/ws2_verify_bag.py "$1"',bagpath,log=log,timeout=60)
            except Exception as exc:cleanup.append('bag stop: '+str(exc))
        for f in ['simulation_gt.log','robot_gt.log','bridge_gt.log','scene_status.json','flight_guard.json']:
            if (RUNTIME/f).exists():shutil.copyfile(RUNTIME/f,out/f)
        preview=RUNTIME/'inference'/(c['planner']+'.json')
        try:
            if preview.exists() and json.loads(preview.read_text()).get('run_id')==str(out):
                shutil.copyfile(preview,out/'inference.json')
                shutil.copyfile(preview.with_suffix('.jpg'),out/'inference.jpg')
        except (OSError,ValueError) as exc:cleanup.append('inference preview: '+str(exc))
        try:stop_owned(log)
        except Exception as exc:cleanup.append('container cleanup: '+str(exc))
        result['cleanup_errors']=cleanup;result['wall_duration_s']=time.monotonic()-started
        atomic(out/'result.json',result);atomic(out/'samples.json',samples);event('result',result=result)
        events.close();lock.close()
        if workerlog:workerlog.close()
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('scenario',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--resolve-only',action='store_true')
    p.add_argument('--wait-for-recording',action='store_true',help='Wait on ground for Enter before starting bag/takeoff')
    p.add_argument('--camera',choices=['overview','follow'],default='overview')
    a=p.parse_args();c=resolved(yaml.safe_load(a.scenario.read_text()))
    if a.resolve_only:print(yaml.safe_dump(c,sort_keys=False))
    else:
        output=a.output or RUNTIME/'episodes'/datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        print(json.dumps(run_episode(c,output,a.wait_for_recording,a.camera),indent=2))
