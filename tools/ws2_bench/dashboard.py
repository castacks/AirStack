"""Presentation console: actual inference and result-guided automatic flights."""
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import argparse,datetime,fcntl,json,subprocess,sys,threading,math
from run_conditions import RUNTIME,HERE
from episode import atomic
from mission import defaults
from conditions import DIFFICULTY_COUNTS

MONITOR=False;job=None;job_output=None;job_lock=threading.Lock()

def feedback_command(payload,output):
    if not isinstance(payload,dict) or set(payload)-{'planner','budget','timeout','goal_distance','profile','difficulty'} or payload.get('planner') not in ('mononav','kim'):
        raise ValueError('Select one target model: mononav or kim')
    budget=payload.get('budget',8);timeout=payload.get('timeout',defaults(payload['planner'])['timeout'])
    distance=payload.get('goal_distance',8);profile=payload.get('profile','combined')
    if isinstance(budget,bool) or not isinstance(budget,int) or not 2<=budget<=100 or budget%2:raise ValueError('Flight budget must be even, 2..100')
    if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in [timeout,distance]):raise ValueError('Invalid numeric mission settings')
    if not 60<=timeout<=600 or not 2<=distance<=30:raise ValueError('Duration60..600 seconds, goal2..30 metres')
    if profile not in ['combined','sensors','noise','delay','patch']:raise ValueError('Invalid attack profile')
    difficulty=payload.get('difficulty','easy')
    if not isinstance(difficulty,str) or difficulty not in DIFFICULTY_COUNTS:raise ValueError('Difficulty must be easy, medium or hard')
    return [sys.executable,str(HERE/'campaign.py'),'--backend','feedback','--profile',profile,
            '--budget',str(budget),'--planner',payload['planner'],'--timeout',str(timeout),
            '--goal-distance',str(distance),'--difficulty',difficulty,'--output',str(output)]

def campaign_root():
    value={'output':str(job_output)} if job_output is not None and job is not None and job.poll() is None else read_json(RUNTIME/'live_campaign.json') or {}
    if not value.get('output'):raise ValueError('No campaign yet')
    from pathlib import Path
    root=Path(value['output']).resolve();root.relative_to((RUNTIME/'campaigns').resolve())
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
    for name in ['feedback.lock','sequence.lock']:
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
                try:report=read_json(campaign_root()/'vulnerability_report.json');control=read_json(campaign_root()/'operator_control.json')
                except ValueError:report=None;control=None
                self.send({'campaign':campaign,'episode':read_json(RUNTIME/'live_episode.json'),
                           'running':active(),'read_only':MONITOR,
                           'view':read_json(RUNTIME/'live_view.json'),'view_settings':read_json(RUNTIME/'view_settings.json'),
                           'scene':read_json(RUNTIME/'scene_status.json'),'analysis':report,'control':control,
                           'inference':{m:read_json(RUNTIME/'inference'/(m+'.json')) for m in ['mononav','kim']}})
            elif path=='/scene.jpg':self.send((RUNTIME/'live_view.jpg').read_bytes(),'image/jpeg')
            elif path in ['/report','/report.md','/report.json']:
                ext={'/report':'html','/report.md':'md','/report.json':'json'}[path]
                self.send((campaign_root()/('vulnerability_report.'+ext)).read_bytes(),
                          'text/html; charset=utf-8' if ext=='html' else 'text/plain; charset=utf-8')
            elif path in ['/inference/mononav.jpg','/inference/kim.jpg']:
                self.send((RUNTIME/path.lstrip('/')).read_bytes(),'image/jpeg')
            else:self.send({'error':'not found'},status=404)
        except (OSError,ValueError) as e:self.send({'error':str(e)},status=503)
    def do_POST(self):
        global job,job_output
        if MONITOR:self.send({'error':'Read-only monitor'},status=403);return
        if self.path not in ['/start/feedback','/control','/view']:self.send({'error':'not found'},status=404);return
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
            output=RUNTIME/'campaigns'/('feedback_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
            argv=feedback_command(payload,output)
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
