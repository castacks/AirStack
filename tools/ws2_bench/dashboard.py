"""Presentation console: actual inference and result-guided automatic flights."""
from model_adapters import REGISTRY, get_adapter
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import argparse,datetime,fcntl,json,os,subprocess,sys,threading,math,time
from run_conditions import RUNTIME,HERE
from episode import atomic
from mission import defaults
from conditions import DIFFICULTY_COUNTS

MONITOR=False;job=None;job_output=None;job_lock=threading.Lock()

def feedback_command(payload,output):
    if os.environ.get('WS2_EXECUTION_BACKEND','local')=='osmo':
        raise ValueError('OSMO currently supports Corridors; select Random, Search or LLM-guided tests with Corridors')
    if not isinstance(payload,dict) or set(payload)-{'planner','budget','timeout','goal_distance','profile','difficulty'} or not isinstance(payload.get('planner'),str) or payload.get('planner') not in REGISTRY:
        raise ValueError('Select one registered target model: '+', '.join(REGISTRY))
    budget=payload.get('budget',8);timeout=payload.get('timeout',defaults(payload['planner'])['timeout'])
    distance=payload.get('goal_distance',8);profile=payload.get('profile','sensors')
    if isinstance(budget,bool) or not isinstance(budget,int) or not 2<=budget<=100 or budget%2:raise ValueError('Flight budget must be even, 2..100')
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in [timeout,distance]):raise ValueError('Invalid numeric mission settings')
    if not 60<=timeout<=600 or not 2<=distance<=30:raise ValueError('Duration60..600 seconds, goal2..30 metres')
    if profile not in ['combined','sensors','noise','delay','patch']:raise ValueError('Invalid attack profile')
    if 'fcrn_patch' not in get_adapter(payload['planner']).attacks and profile in ('combined','patch'):raise ValueError('Rui patch is FCRN/Kim only; no ZoeDepth or DroNet patch is available; use a sensor profile')
    difficulty=payload.get('difficulty','easy')
    if not isinstance(difficulty,str) or difficulty not in DIFFICULTY_COUNTS:raise ValueError('Difficulty must be easy, medium or hard')
    return [sys.executable,str(HERE/'campaign.py'),'--backend','feedback','--profile',profile,
            '--budget',str(budget),'--planner',payload['planner'],'--timeout',str(timeout),
            '--goal-distance',str(distance),'--difficulty',difficulty,'--output',str(output)]

def adaptive_command(payload,output):
    """Build the saved-layout, no-noise adaptive campaign command."""
    if not isinstance(payload,dict) or set(payload)-{'planner','budget','timeout','goal_distance','policy','provider','qualified_layouts','clean_validation_runs','infrastructure_retries','clean_failure_policy','action_space'} or not isinstance(payload.get('planner'),str) or payload.get('planner') not in REGISTRY:
        raise ValueError('Select one registered target model: '+', '.join(REGISTRY))
    policy=payload.get('policy','search')
    action_space=payload.get('action_space','saved')
    if os.environ.get('WS2_EXECUTION_BACKEND','local')=='osmo' and action_space!='corridor':
        raise ValueError('Select Corridors for OSMO; Office assets are not included in the task image')
    if action_space not in ('saved','expanded','corridor'):raise ValueError('Unknown action space')
    budget=payload.get('budget',8);timeout=payload.get('timeout',defaults(payload['planner'])['timeout'])
    distance=payload.get('goal_distance',8)
    if policy not in ('random','search','agent_search'):raise ValueError('Invalid adaptive policy')
    provider=payload.get('provider',os.environ.get('WS2_AGENT_PROVIDER','openai'))
    if provider not in ('claude','codex','openai'):raise ValueError('Provider must be claude, codex or openai')
    if action_space in ('expanded','corridor') and policy=='agent_search' and provider not in ('claude','codex'):raise ValueError('Expanded agent mode requires Claude or Codex')
    if policy=='agent_search' and provider=='openai' and not all(os.environ.get(key) for key in ('WS2_AGENT_ENDPOINT','WS2_AGENT_API_KEY','WS2_AGENT_MODEL')):
        raise ValueError('Agent mode needs WS2_AGENT_ENDPOINT, WS2_AGENT_API_KEY, and WS2_AGENT_MODEL')
    if policy=='agent_search' and provider in ('claude','codex'):
        from agent_policies import provider_from_name
        try:provider_from_name(provider).check_auth()
        except (RuntimeError,OSError,subprocess.TimeoutExpired) as exc:raise ValueError(str(exc)) from None
    if isinstance(budget,bool) or not isinstance(budget,int) or not 2<=budget<=100 or budget%2:raise ValueError('Flight budget must be even, 2..100')
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in [timeout,distance]):raise ValueError('Invalid numeric mission settings')
    if not 60<=timeout<=600 or not 2<=distance<=30:raise ValueError('Duration60..600 seconds, goal2..30 metres')
    from agent_campaign import parse_qualified_layouts
    layouts=payload.get('qualified_layouts')
    if action_space=='saved':
        if not isinstance(layouts,list) or not layouts:raise ValueError('Enter clean-checked layout:seed values, e.g. easy:2')
        qualified=parse_qualified_layouts(layouts)
    else:qualified=()
    validations=payload.get('clean_validation_runs',2)
    retries=payload.get('infrastructure_retries',1)
    clean_policy=payload.get('clean_failure_policy','halt')
    if type(validations) is not int or validations not in (0,1,2,3):raise ValueError('Extra clean checks must be 0..3')
    if type(retries) is not int or retries not in (0,1,2):raise ValueError('Infrastructure retries must be 0..2')
    if clean_policy not in ('halt','record'):raise ValueError('Invalid clean failure policy')
    command=[sys.executable,str(HERE/'agent_campaign.py'),'--policy',policy,
            '--budget',str(budget),'--planner',payload['planner'],'--timeout',str(timeout),
            '--goal-distance',str(distance),'--output',str(output),
            '--clean-validation-runs',str(validations),'--infrastructure-retries',str(retries),
            '--clean-failure-policy',clean_policy,'--action-space',action_space]
    if action_space=='corridor':
        for flag in ('--timeout','--goal-distance'):
            i=command.index(flag);del command[i:i+2]
    for layout,seed in qualified:command+=['--qualified-layout',f'{layout}:{seed}']
    if policy=='agent_search':command+=['--provider',provider]
    return command

def standalone_remote_root():
    remote=read_json(RUNTIME/'remote_live.json') or {}
    if not remote.get('active') and os.environ.get('WS2_EXECUTION_BACKEND','local')!='osmo':return None
    if not remote.get('local_output') or not remote.get('control_path'):return None
    if not remote.get('active'):
        # A completed CLI flight must not hide a newly starting campaign before
        # its first OSMO submission replaces remote_live.json.
        finished=(RUNTIME/'remote_live.json').stat().st_mtime
        launch=read_json(RUNTIME/'dashboard_launch.json') or {}
        campaign=RUNTIME/'live_campaign.json'
        if launch.get('started_at',0)>finished or (campaign.exists() and campaign.stat().st_mtime>finished):
            return None
    root=Path(remote['local_output']).resolve();root.relative_to(RUNTIME.resolve())
    return root if Path(remote['control_path']).resolve().parent==root else None

def campaign_root():
    single=standalone_remote_root()
    if single is not None:return single
    launch=launch_status()
    value=launch if launch else read_json(RUNTIME/'live_campaign.json') or {}
    if not value.get('output'):raise ValueError('No campaign yet')
    root=Path(value['output']).resolve();root.relative_to((RUNTIME/'campaigns').resolve())
    return root

def comparison_root():
    from pathlib import Path
    current=read_json(RUNTIME/'live_comparison.json') or {}
    if not current.get('output'):raise ValueError('No comparison yet')
    root=Path(current['output']).resolve();root.relative_to((RUNTIME/'campaigns').resolve())
    return root

def set_view(payload):
    if not isinstance(payload,dict) or set(payload)-{'mode','distance','height','azimuth'}:raise ValueError('Invalid camera settings')
    view=read_json(RUNTIME/'view_settings.json') or {'mode':'follow','distance':2.5,'height':1.5,'azimuth':180.}
    view.update(payload)
    if view['mode'] not in ['follow','overview','free']:raise ValueError('Unknown camera mode')
    for key,lo,hi in [('distance',1.5,25),('height',.3,15),('azimuth',0,360)]:
        v=view[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or not lo<=v<=hi:raise ValueError('Invalid '+key)
    session=read_json(RUNTIME/'remote_live.json') or {}
    if session.get('active'):
        from osmo_transport import request
        # Local saved preferences can contain legacy/internal fields (for
        # example relative_heading). Send only the public camera API fields.
        request(session,'/view',{key:view[key] for key in ('mode','distance','height','azimuth')})
    atomic(RUNTIME/'view_settings.json',view);return view

def read_json(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return None

def flight_view():
    session=read_json(RUNTIME/'remote_live.json') or {}
    if session.get('active'):
        snapshot=read_json(RUNTIME/'remote_snapshot.json') or {}
        if snapshot.get('run_id')!=session['run_id']:snapshot={}
        return {**snapshot,'execution':{'backend':'osmo','workflow_id':session.get('workflow_id'),
                                      'received_at':snapshot.get('received_at')}}
    if os.environ.get('WS2_EXECUTION_BACKEND','local')=='osmo':
        return {'inference':{},'execution':{'backend':'osmo','active':False}}
    return {'scene':read_json(RUNTIME/'scene_status.json'),'view':read_json(RUNTIME/'live_view.json'),
            'view_settings':read_json(RUNTIME/'view_settings.json'),
            'inference':{m:read_json(RUNTIME/'inference'/(m+'.json')) for m in REGISTRY},
            'execution':{'backend':os.environ.get('WS2_EXECUTION_BACKEND','local')}}

def flight_frame(path):
    session=read_json(RUNTIME/'remote_live.json') or {}
    if session.get('active'):
        from osmo_transport import request
        return request(session,path)
    if os.environ.get('WS2_EXECUTION_BACKEND','local')=='osmo':
        raise OSError('No active remote flight preview')
    return (RUNTIME/('live_view.jpg' if path=='/scene.jpg' else path.lstrip('/'))).read_bytes()

def launch_status():
    """Keep early child exits visible even before campaign code writes its state."""
    record=read_json(RUNTIME/'dashboard_launch.json')
    if not record:return None
    live=RUNTIME/'live_campaign.json';campaign=read_json(live)
    if campaign and campaign.get('output')!=record.get('output') and live.stat().st_mtime>record['started_at']:
        return None  # A newer CLI campaign owns the viewer.
    if job is not None and str(job_output)==record['output']:
        code=job.poll()
        record.update(status='running' if code is None else 'failed' if code else 'exited',exit_code=code)
    elif record.get('status') in ('starting','running'):
        record.update(status='unknown',error='Viewer restarted; inspect the campaign log before starting another run.')
    return record

def launch_log_tail():
    try:
        with (RUNTIME/'dashboard_campaign.log').open('rb') as stream:
            stream.seek(0,2);stream.seek(max(0,stream.tell()-12000))
            value=stream.read().decode('utf-8',errors='replace')
        for key,secret in os.environ.items():
            if len(secret)>=8 and any(part in key.upper() for part in ('TOKEN','SECRET','PASSWORD','WEBHOOK','API_KEY')):
                value=value.replace(secret,'[REDACTED]')
        return value
    except OSError as exc:return str(exc)

def watch_launch(process,output):
    code=process.wait()
    with job_lock:
        record=read_json(RUNTIME/'dashboard_launch.json')
        if record and record.get('pid')==process.pid and record['output']==str(output):
            record.update(status='failed' if code else 'exited',exit_code=code)
            atomic(RUNTIME/'dashboard_launch.json',record)

def active():
    if job is not None and job.poll() is None:return True
    for name in ['feedback.lock','adaptive.lock','sequence.lock']:
        with (RUNTIME/name).open('a') as lock:
            try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:return True
    return False

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def send(self,data,kind='application/json',status=200):
        if not isinstance(data,bytes):data=json.dumps(data).encode()
        self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers()
        try:self.wfile.write(data)
        except (BrokenPipeError,ConnectionResetError):pass
    def do_GET(self):
        path=self.path.split('?')[0]
        try:
            if path=='/':self.send((HERE/'presentation.html').read_bytes(),'text/html; charset=utf-8')
            elif path=='/state':
                preview=flight_view()
                campaign=read_json(RUNTIME/'live_campaign.json')
                launch=launch_status()
                if launch and (not campaign or campaign.get('output')!=launch['output']):
                    campaign={'output':launch['output'],'target_planner':launch['planner'],
                              'phase':'preflight' if launch['status']=='running' else 'error'}
                comparison=read_json(RUNTIME/'live_comparison.json')
                try:
                    compare_root=comparison_root();comparison_report=read_json(compare_root/'comparison.json');comparison_protocol=read_json(compare_root/'protocol.json')
                except ValueError:comparison_report=None;comparison_protocol=None
                if standalone_remote_root() is not None:
                    campaign=None;launch=None;comparison=None;comparison_report=None;comparison_protocol=None
                try:
                    root=campaign_root();report=read_json(root/'vulnerability_report.json');control=read_json(root/'operator_control.json')
                    llm_analysis=read_json(root/'llm_analysis.json');campaign_error=read_json(root/'campaign_error.json')
                    campaign_config=read_json(root/'config.json')
                except ValueError:report=None;control=None;llm_analysis=None;campaign_error=None;campaign_config=None
                self.send({'campaign':campaign,'episode':read_json(RUNTIME/'live_episode.json'),
                           'running':active(),'read_only':MONITOR,
                           'view':preview.get('view'),'view_settings':preview.get('view_settings'),
                           'scene':preview.get('scene'),'analysis':report,'control':control,
                           'llm_analysis':llm_analysis,'campaign_error':campaign_error,
                           'campaign_config':campaign_config,
                           'launch':launch,
                           'comparison':comparison,'comparison_report':comparison_report,'comparison_protocol':comparison_protocol,
                           'inference':preview.get('inference',{}),'execution':preview['execution']})
            elif path=='/launch-log':self.send(launch_log_tail().encode(),'text/plain; charset=utf-8')
            elif path=='/scene.jpg':self.send(flight_frame(path),'image/jpeg')
            elif path in ['/report','/report.md','/report.json']:
                ext={'/report':'html','/report.md':'md','/report.json':'json'}[path]
                self.send((campaign_root()/('vulnerability_report.'+ext)).read_bytes(),
                          'text/html; charset=utf-8' if ext=='html' else 'text/plain; charset=utf-8')
            elif path in ['/llm-report','/llm-report.md','/llm-report.json']:
                ext={'/llm-report':'html','/llm-report.md':'md','/llm-report.json':'json'}[path]
                self.send((campaign_root()/('llm_analysis.'+ext)).read_bytes(),
                          'text/html; charset=utf-8' if ext=='html' else 'text/plain; charset=utf-8')
            elif path in ['/comparison','/comparison.md','/comparison.json']:
                ext={'/comparison':'html','/comparison.md':'md','/comparison.json':'json'}[path]
                self.send((comparison_root()/('comparison.'+ext)).read_bytes(),
                          'text/html; charset=utf-8' if ext=='html' else 'text/plain; charset=utf-8')
            elif path=='/llm-calls':
                self.send([read_json(p) for p in sorted((campaign_root()/'llm_calls').glob('*.json'),key=lambda p:p.stat().st_mtime)[-4:]])
            elif path in ['/inference/'+m+'.jpg' for m in REGISTRY]:
                self.send(flight_frame(path),'image/jpeg')
            else:self.send({'error':'not found'},status=404)
        except (OSError,ValueError) as e:self.send({'error':str(e)},status=503)
    def do_POST(self):
        global job,job_output
        if MONITOR:self.send({'error':'Read-only monitor'},status=403);return
        if self.path not in ['/start/feedback','/start/adaptive','/control','/view']:self.send({'error':'not found'},status=404);return
        try:
            size=int(self.headers.get('Content-Length','0'))
            if not 0<size<=4096:raise ValueError('Select a target model before starting')
            payload=json.loads(self.rfile.read(size))
            if self.path=='/view':
                with job_lock:self.send({'ok':True,'view':set_view(payload)})
                return
            if self.path=='/control':
                if not isinstance(payload,dict) or set(payload)!={'action'} or payload['action'] not in ['pause','run','stop']:raise ValueError('Unknown control action')
                with job_lock:
                    remote=read_json(RUNTIME/'remote_live.json') or {}
                    if remote.get('active') and remote.get('control_path'):
                        control_file=Path(remote['control_path']).resolve()
                        control_file.relative_to(RUNTIME.resolve())
                        if not control_file.is_file():raise ValueError('Remote flight is still preparing')
                        atomic(control_file,payload)
                        self.send({'ok':True,'requested':payload['action']});return
                    if not active():self.send({'error':'No active campaign'},status=409);return
                    root=campaign_root()
                    if not (root/'operator_control.json').exists():self.send({'error':'Campaign is still preparing'},status=409);return
                    atomic(root/'operator_control.json',payload)
                self.send({'ok':True,'requested':payload['action']});return
            kind='adaptive' if self.path=='/start/adaptive' else 'feedback'
            output=RUNTIME/'campaigns'/(kind+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            argv=adaptive_command(payload,output) if kind=='adaptive' else feedback_command(payload,output)
        except (ValueError,UnicodeError,OSError) as exc:self.send({'error':str(exc)},status=400);return
        with job_lock:
            try:
                if active():self.send({'error':'another bench run is active'},status=409);return
                from check_install import check_install
                check=check_install(payload['planner'])
                if not check['ok']:
                    self.send({'error':'Installation incomplete:\n'+'\n'.join(check['errors'])},status=400);return
                record={'output':str(output),'planner':payload['planner'],'started_at':time.time(),'status':'starting','exit_code':None}
                with (RUNTIME/'dashboard_campaign.log').open('w') as log:
                    job=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    job_output=output
                    record.update(pid=job.pid,status='running')
                    atomic(RUNTIME/'dashboard_launch.json',record)
                threading.Thread(target=watch_launch,args=(job,output),daemon=True).start()
                self.send({'ok':True,'output':str(output)})
            except OSError as exc:
                self.send({'error':f'Cannot start campaign: {exc}. Check runtime permissions and the Python executable.'},status=500)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--monitor',action='store_true');p.add_argument('--port',type=int,default=8892)
    p.add_argument('--execution-backend',choices=['local','osmo'],default=os.environ.get('WS2_EXECUTION_BACKEND','local'))
    p.add_argument('--osmo-config',help='OSMO settings YAML; local campaigns and LLM authentication stay on this host')
    a=p.parse_args();MONITOR=a.monitor;RUNTIME.mkdir(parents=True,exist_ok=True)
    os.environ['WS2_EXECUTION_BACKEND']=a.execution_backend
    if a.osmo_config:os.environ['WS2_OSMO_CONFIG']=os.path.abspath(a.osmo_config)
    print(f'WS2 presentation: http://127.0.0.1:{a.port}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()
