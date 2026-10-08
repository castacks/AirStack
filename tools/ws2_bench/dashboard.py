"""Presentation console: actual inference and result-guided automatic flights."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import argparse,datetime,fcntl,json,os,subprocess,sys,threading,math
from run_conditions import RUNTIME,HERE
from episode import atomic
from mission import defaults
from conditions import DIFFICULTY_COUNTS

MONITOR=False;job=None;job_output=None;job_lock=threading.Lock()

def feedback_command(payload,output):
    if not isinstance(payload,dict) or set(payload)-{'planner','budget','timeout','goal_distance','profile','difficulty'} or payload.get('planner') not in ('mononav','kim'):
        raise ValueError('Select one target model: mononav or kim')
    budget=payload.get('budget',8);timeout=payload.get('timeout',defaults(payload['planner'])['timeout'])
    distance=payload.get('goal_distance',8);profile=payload.get('profile','sensors')
    if isinstance(budget,bool) or not isinstance(budget,int) or not 2<=budget<=100 or budget%2:raise ValueError('Flight budget must be even, 2..100')
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in [timeout,distance]):raise ValueError('Invalid numeric mission settings')
    if not 60<=timeout<=600 or not 2<=distance<=30:raise ValueError('Duration60..600 seconds, goal2..30 metres')
    if profile not in ['combined','sensors','noise','delay','patch']:raise ValueError('Invalid attack profile')
    if payload['planner']=='mononav' and profile in ('combined','patch'):raise ValueError('No ZoeDepth patch is available; use a sensor profile')
    difficulty=payload.get('difficulty','easy')
    if not isinstance(difficulty,str) or difficulty not in DIFFICULTY_COUNTS:raise ValueError('Difficulty must be easy, medium or hard')
    return [sys.executable,str(HERE/'campaign.py'),'--backend','feedback','--profile',profile,
            '--budget',str(budget),'--planner',payload['planner'],'--timeout',str(timeout),
            '--goal-distance',str(distance),'--difficulty',difficulty,'--output',str(output)]

def adaptive_command(payload,output):
    """Build the saved-layout, no-noise adaptive campaign command."""
    if not isinstance(payload,dict) or set(payload)-{'planner','budget','timeout','goal_distance','policy','provider','qualified_layouts','clean_validation_runs','infrastructure_retries','clean_failure_policy','action_space'} or payload.get('planner') not in ('mononav','kim'):
        raise ValueError('Select one target model: mononav or kim')
    policy=payload.get('policy','search')
    action_space=payload.get('action_space','saved')
    if action_space not in ('saved','expanded'):raise ValueError('Unknown action space')
    budget=payload.get('budget',8);timeout=payload.get('timeout',defaults(payload['planner'])['timeout'])
    distance=payload.get('goal_distance',8)
    if policy not in ('random','search','agent_search'):raise ValueError('Invalid adaptive policy')
    provider=payload.get('provider',os.environ.get('WS2_AGENT_PROVIDER','openai'))
    if provider not in ('claude','openai'):raise ValueError('Provider must be claude or openai')
    if action_space=='expanded' and policy=='agent_search' and provider!='claude':raise ValueError('Expanded agent mode requires Claude')
    if policy=='agent_search' and provider=='openai' and not all(os.environ.get(key) for key in ('WS2_AGENT_ENDPOINT','WS2_AGENT_API_KEY','WS2_AGENT_MODEL')):
        raise ValueError('Agent mode needs WS2_AGENT_ENDPOINT, WS2_AGENT_API_KEY, and WS2_AGENT_MODEL')
    if policy=='agent_search' and provider=='claude':
        from claude_provider import ClaudeSubscriptionProvider
        try:ClaudeSubscriptionProvider().check_auth()
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
    for layout,seed in qualified:command+=['--qualified-layout',f'{layout}:{seed}']
    if policy=='agent_search':command+=['--provider',provider]
    return command

def campaign_root():
    value={'output':str(job_output)} if job_output is not None and job is not None and job.poll() is None else read_json(RUNTIME/'live_campaign.json') or {}
    if not value.get('output'):raise ValueError('No campaign yet')
    from pathlib import Path
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
    atomic(RUNTIME/'view_settings.json',view);return view

def read_json(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return None

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
                campaign=read_json(RUNTIME/'live_campaign.json')
                comparison=read_json(RUNTIME/'live_comparison.json')
                try:
                    compare_root=comparison_root();comparison_report=read_json(compare_root/'comparison.json');comparison_protocol=read_json(compare_root/'protocol.json')
                except ValueError:comparison_report=None;comparison_protocol=None
                try:
                    root=campaign_root();report=read_json(root/'vulnerability_report.json');control=read_json(root/'operator_control.json')
                    llm_analysis=read_json(root/'llm_analysis.json');campaign_error=read_json(root/'campaign_error.json')
                    campaign_config=read_json(root/'config.json')
                except ValueError:report=None;control=None;llm_analysis=None;campaign_error=None;campaign_config=None
                self.send({'campaign':campaign,'episode':read_json(RUNTIME/'live_episode.json'),
                           'running':active(),'read_only':MONITOR,
                           'view':read_json(RUNTIME/'live_view.json'),'view_settings':read_json(RUNTIME/'view_settings.json'),
                           'scene':read_json(RUNTIME/'scene_status.json'),'analysis':report,'control':control,
                           'llm_analysis':llm_analysis,'campaign_error':campaign_error,
                           'campaign_config':campaign_config,
                           'comparison':comparison,'comparison_report':comparison_report,'comparison_protocol':comparison_protocol,
                           'inference':{m:read_json(RUNTIME/'inference'/(m+'.json')) for m in ['mononav','kim']}})
            elif path=='/scene.jpg':self.send((RUNTIME/'live_view.jpg').read_bytes(),'image/jpeg')
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
            elif path in ['/inference/mononav.jpg','/inference/kim.jpg']:
                self.send((RUNTIME/path.lstrip('/')).read_bytes(),'image/jpeg')
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
                    if not active():self.send({'error':'No active campaign'},status=409);return
                    root=campaign_root()
                    if not (root/'operator_control.json').exists():self.send({'error':'Campaign is still preparing'},status=409);return
                    atomic(root/'operator_control.json',payload)
                self.send({'ok':True,'requested':payload['action']});return
            kind='adaptive' if self.path=='/start/adaptive' else 'feedback'
            output=RUNTIME/'campaigns'/(kind+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            argv=adaptive_command(payload,output) if kind=='adaptive' else feedback_command(payload,output)
        except (ValueError,UnicodeError) as exc:self.send({'error':str(exc)},status=400);return
        with job_lock:
            if active():self.send({'error':'another bench run is active'},status=409);return
            with (RUNTIME/'dashboard_campaign.log').open('w') as log:
                job=subprocess.Popen(argv,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                job_output=output
            self.send({'ok':True,'output':str(output)})

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--monitor',action='store_true');p.add_argument('--port',type=int,default=8892)
    a=p.parse_args();MONITOR=a.monitor;RUNTIME.mkdir(parents=True,exist_ok=True)
    print(f'WS2 presentation: http://127.0.0.1:{a.port}',flush=True)
    ThreadingHTTPServer(('127.0.0.1',a.port),Handler).serve_forever()
