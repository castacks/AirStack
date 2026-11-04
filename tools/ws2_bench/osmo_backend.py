"""Local OSMO client: one validated episode per workflow, one in flight at a time."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import time
import uuid
import yaml
from osmo_transport import atomic_json, read_json, request, unpack_artifacts, mapped_paths
from operator_control import request as operator_request, UserStop
from run_conditions import HERE, RUNTIME

ACTIVE = {'PENDING', 'WAITING', 'INITIALIZING', 'READY', 'RUNNING', 'SCHEDULING',
          'SUBMITTING', 'PROCESSING', 'RESCHEDULED', 'WAITING_FOR_RESOURCES'}


def load_settings(path=None):
    path = path or os.environ.get('WS2_OSMO_CONFIG')
    if not path:
        raise ValueError('Set WS2_OSMO_CONFIG to your configured osmo YAML file')
    c = yaml.safe_load(Path(path).expanduser().read_text())
    allowed = {'pool', 'platform', 'registry_credential', 'data_credential', 'output_uri',
               'images', 'cpu', 'memory_gib', 'storage_gib', 'exec_minutes', 'queue_minutes',
               'port', 'poll_seconds'}
    if not isinstance(c, dict) or c.keys() - allowed:
        raise ValueError('Invalid OSMO configuration keys')
    c = {**dict(cpu=6, memory_gib=48, storage_gib=80, exec_minutes=90,
             queue_minutes=60, port=8894, poll_seconds=5), **c}
    for key in ('pool', 'platform', 'registry_credential', 'data_credential'):
        if not isinstance(c.get(key), str) or not re.fullmatch(r'[\w.-]+', c[key]):
            raise ValueError('Set OSMO ' + key)
    for key, lo, hi in [('cpu',1,10),('memory_gib',1,50),('storage_gib',10,500),
                        ('exec_minutes',5,180),('queue_minutes',1,1440),
                        ('port',1024,65535),('poll_seconds',1,30)]:
        if type(c[key]) is not int or not lo <= c[key] <= hi:
            raise ValueError('Invalid OSMO resource setting: ' + key)
    if not isinstance(c.get('output_uri'), str) or not re.fullmatch(r's3://[a-zA-Z0-9_.-]+/[a-zA-Z0-9_./-]+', c['output_uri']) or '..' in c['output_uri'].split('/'):
        raise ValueError('output_uri must be an S3 bucket plus a dedicated WS2 prefix')
    if not isinstance(c.get('images'), dict):
        raise ValueError('Configure images by model name')
    return c


def target_image(settings, planner):
    image = settings['images'].get(planner)
    if not isinstance(image, str) or not re.fullmatch(r'[a-zA-Z0-9_.:/-]+@sha256:[a-f0-9]{64}', image):
        raise ValueError(f'Set images.{planner} to a published image pinned by @sha256 digest')
    return image


def prepare(raw, settings, run_id, *, camera='overview', record_bag=False, record_video=False):
    from episode import resolved, fingerprint
    c = resolved(raw)
    if not re.fullmatch(r'[a-f0-9]{32}', run_id):
        raise ValueError('Invalid run identity')
    if camera not in ('follow', 'overview'):
        raise ValueError('Invalid observer camera')
    if c['condition']['layout'] != 'corridor':
        raise ValueError('The first OSMO image supports generated corridors; Office assets are not packaged')
    image = target_image(settings, c['planner'])
    token = uuid.uuid4().hex
    payload = {'schema_version':1, 'run_id':run_id, 'scenario':c,
               'configuration_hash':fingerprint(c), 'token':token,
               'camera':camera, 'record_bag':bool(record_bag), 'record_video':bool(record_video)}
    contents = json.dumps(payload)
    if '{{' in contents or '{%' in contents:
        raise ValueError('Scenario strings cannot contain OSMO template expressions')
    destination = settings['output_uri'].rstrip('/') + '/' + run_id
    spec = {'workflow': {'name':'ws2-' + run_id[:16], 'pool':settings['pool'],
        'timeout':{'exec_timeout':f"{settings['exec_minutes']}m", 'queue_timeout':f"{settings['queue_minutes']}m"},
        'resources':{'default':{'gpu':1, 'cpu':settings['cpu'],
            'memory':f"{settings['memory_gib']}Gi", 'storage':f"{settings['storage_gib']}Gi",
            'platform':settings['platform']}},
        'tasks':[{'name':'flight', 'image':image,
            # OSMO applies saved DATA/REGISTRY credentials by URL/registry.
            # imagePullCredentials is not a task field in OSMO 6.2/6.3.
            'exitActions':{'COMPLETE':'0','FAIL':'1-255,1001,3000-4000'},
            'command':['/usr/bin/python3'],
            'args':['/opt/ws2/AirStack/tools/ws2_bench/osmo_task.py',
                    '--request','/tmp/ws2-request.json','--output','{{output}}'],
            'files':[{'path':'/tmp/ws2-request.json','contents':contents}],
            'outputs':[{'url':destination}]}]}}
    return payload, spec, destination


class Client:
    def check_access(self, settings):
        """Read current user's credentials/pool access without exposing secrets."""
        self.check_credentials(settings)
        profile = self.call('profile', 'list', '--format-type', 'json')
        accessible = profile.get('pools', [])
        if settings['pool'] not in accessible:
            raise ValueError('Your OSMO account cannot access pool ' + settings['pool'])

    def check_credentials(self, settings):
        data=self.call('credential','--format-type','json','list')
        found={(v.get('cred_name'),v.get('cred_type')) for v in data.get('credentials',[])}
        for name,kind in [(settings['data_credential'],'DATA'),(settings['registry_credential'],'REGISTRY')]:
            if (name,kind) not in found:raise ValueError('Missing OSMO '+kind+' credential: '+name)

    def call(self, *args, timeout=45, json_output=True):
        result = subprocess.run(['osmo', *map(str,args)], capture_output=True, text=True, timeout=timeout)
        if result.returncode:
            # CLI errors can include credentials; never copy an arbitrary response to the UI.
            raise RuntimeError(f'OSMO {args[0]} {args[1]} failed (exit {result.returncode}); inspect the OSMO CLI log locally')
        if not json_output:
            return result.stdout
        try:
            return json.loads(result.stdout)
        except ValueError:
            raise RuntimeError('OSMO returned an unexpected response; no automatic resubmission') from None

    def submit(self, spec):
        response = self.call('workflow','submit',spec,'--format-type','json','--priority','NORMAL',timeout=90)
        name = response.get('name') or response.get('workflow_name') or response.get('workflow_id')
        if not isinstance(name,str) or not re.fullmatch(r'[\w.-]+',name):
            raise RuntimeError('OSMO submission response lacks a workflow ID; inspect workflow list before retrying')
        return name

    def status(self, name):
        data = self.call('workflow','query',name,'--format-type','json')
        status = data.get('status')
        if not isinstance(status, str):
            raise RuntimeError('OSMO query response lacks status')
        return status.upper(), data

    def cancel(self, name):
        self.call('workflow','cancel',name,'--format-type','json',json_output=False)

    def download(self, uri, path):
        self.call('data','download',uri,path,'--processes','1','--threads','2',timeout=600,json_output=False)

    def tunnel(self, name, port, logfile):
        with Path(logfile).open('w') as log:
            return subprocess.Popen(['osmo','workflow','port-forward',name,'flight','--host','127.0.0.1',
                    '--port',f'{port}:8894','--connect-timeout','15'],stdout=log,stderr=subprocess.STDOUT,
                    start_new_session=True)


def verify_port_available(port):
    # A completed tunnel leaves TCP TIME_WAIT sockets. They are not listeners
    # and must not reject the next flight in a sequential campaign.
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1',port))


def preflight(planner):
    errors=[]
    try:
        import shutil
        c=load_settings();target_image(c,planner)
        if not shutil.which('osmo'):errors.append('Install the OSMO CLI and log in using the lab guide')
        if (RUNTIME/'osmo_pending_cleanup.json').exists():errors.append('Resolve osmo_pending_cleanup.json before another submission')
        if (read_json(RUNTIME/'remote_live.json') or {}).get('active'):errors.append('An OSMO workflow is still marked active; recover/cancel it before another submission')
    except (OSError, ValueError) as exc:
        errors.append(str(exc))
    return {'ok':not errors,'errors':errors,'warnings':['Pool permissions, image pull and GPU compatibility require remote validation.'],
            'planner':planner,'scope':'OSMO configuration only; no GPU job submitted'}


def run_remote_episode(raw, output, *, camera='overview', record_bag=True, control_path=None, record_video=False, client=None):
    from episode import resolved, fingerprint
    c=resolved(raw);settings=load_settings()
    if client is None:
        client=Client();client.check_credentials(settings)
    if (RUNTIME/'osmo_pending_cleanup.json').exists():
        raise RuntimeError('Resolve osmo_pending_cleanup.json before another submission')
    if (read_json(RUNTIME/'remote_live.json') or {}).get('active'):
        raise RuntimeError('Recover/cancel the active remote_live.json workflow before another submission')
    out=Path(output).resolve();out.relative_to(RUNTIME.resolve())
    payload,spec,uri=prepare(c,settings,uuid.uuid4().hex,camera=camera,record_bag=record_bag,record_video=record_video)
    RUNTIME.mkdir(parents=True,exist_ok=True)
    with (RUNTIME/'sequence.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        out.mkdir(parents=True,exist_ok=False)
        if control_path is None:
            control_path=out/'operator_control.json'
            atomic_json(control_path,{'action':'run'})
        control_path=Path(control_path).resolve()
        control_path.relative_to(RUNTIME.resolve())
        transport=out/'osmo';transport.mkdir(mode=0o700)
        specfile=transport/'workflow.yaml';specfile.write_text(yaml.safe_dump(spec,sort_keys=False));specfile.chmod(0o600)
        atomic_json(transport/'submitted_scenario.json',c)
        session={'run_id':payload['run_id'],'token':payload['token'],
                 'control_path':str(control_path),
                 'url':f"http://127.0.0.1:{settings['port']}",'local_output':str(out),
                 'remote_output':'/opt/ws2/AirStack/robot/ros_ws/ws2_runtime/episodes/'+payload['run_id']}
        name=None;tunnel=None;terminal=False;started=time.monotonic();last_query=-1.;last_tunnel=-100.
        result=None;status='SUBMITTING';last_control=None;task_phase=None
        reported_phase=None
        def publish(phase, **extra):
            nonlocal reported_phase
            atomic_json(RUNTIME/'live_episode.json',{'planner':c['planner'],'scenario':c['name'],
                'directory':str(out),'phase':phase,'event':phase,'condition':c['condition'],
                'wall_time':time.time(),**extra})
            if phase != reported_phase:
                print(f'[OSMO] {name or "submission"}: {phase}', flush=True)
                reported_phase=phase
        try:
            verify_port_available(settings['port'])
            if operator_request(control_path)=='stop':raise UserStop('operator requested stop before submission')
            while operator_request(control_path)=='pause':
                publish('paused');time.sleep(.2)
            if operator_request(control_path)=='stop':raise UserStop('operator requested stop before submission')
            publish('osmo_submitting')
            try:name=client.submit(specfile)
            except Exception:
                atomic_json(RUNTIME/'osmo_pending_cleanup.json',{'workflow_name_prefix':spec['workflow']['name'],
                    'message':'Submission outcome unknown. Inspect workflow list before retrying; do not submit twice.'})
                raise
            atomic_json(transport/'job.json',{'workflow_id':name,'run_id':payload['run_id'],
                'output_uri':uri,'image':target_image(settings,c['planner']),'configuration_hash':fingerprint(c)})
            atomic_json(RUNTIME/'remote_live.json',dict(session,active=True,workflow_id=name))
            (RUNTIME/'remote_live.json').chmod(0o600)
            while True:
                now=time.monotonic()
                choice=operator_request(control_path)
                if now-started>(settings['exec_minutes']+settings['queue_minutes'])*60+300:
                    raise TimeoutError('OSMO workflow exceeded its execution/queue deadline')
                if now-last_query>=settings['poll_seconds']:
                    status,details=client.status(name);last_query=now
                    atomic_json(transport/'status.json',details)
                    tasks=[t for g in details.get('groups',[]) for t in g.get('tasks',[]) if t.get('name')=='flight']
                    task_phase=tasks[0].get('status') if tasks else None
                    if status=='COMPLETED':terminal=True;break
                    if status.startswith(('FAILED','CANCELED','CANCELLED')):
                        terminal=True;raise RuntimeError('OSMO workflow ended: '+status)
                    if status not in ACTIVE:raise RuntimeError('Unrecognized OSMO workflow status: '+status)
                if choice=='stop' and last_control!='stop':
                    # Graceful stop lets the task land and upload evidence. Fall back to cancel if disconnected.
                    try:
                        request(session,'/control',{'action':'stop'})
                        last_control='stop'
                    except (OSError,ValueError):raise UserStop('operator stopped remote workflow')
                if (tunnel is None or tunnel.poll() is not None) and now-last_tunnel>=10:
                    if tunnel is not None:
                        from runtime_process import stop_process
                        stop_process(tunnel)
                    tunnel=client.tunnel(name,settings['port'],transport/'tunnel.log');last_tunnel=now
                try:
                    snapshot=json.loads(request(session,'/snapshot'))
                    snapshot=mapped_paths(snapshot,session['remote_output'],str(out))
                    atomic_json(RUNTIME/'remote_snapshot.json',dict(snapshot,received_at=time.time(),run_id=payload['run_id']))
                    if choice!=last_control:
                        request(session,'/control',{'action':choice});last_control=choice
                    if snapshot.get('episode'):atomic_json(RUNTIME/'live_episode.json',snapshot['episode'])
                except (OSError,ValueError):
                    phase='initializing' if task_phase=='INITIALIZING' else status.lower()
                    publish('osmo_'+phase,viewer='connecting')
                time.sleep(.5)
            downloads=transport/'download';downloads.mkdir()
            publish('osmo_downloading')
            client.download(uri,downloads)
            receipts=list(downloads.rglob('receipt.json'));archives=list(downloads.rglob('artifacts.tar.gz'))
            if len(receipts)!=1 or len(archives)!=1:raise RuntimeError('Missing or ambiguous OSMO result archive')
            receipt=read_json(receipts[0])
            unpack_artifacts(archives[0],receipt,out,payload['run_id'],fingerprint(c))
            # Original bytes remain in the checksummed archive. Only local JSON path references are mapped.
            for path in out.rglob('*.json'):
                if transport in path.parents:continue
                value=json.loads(path.read_text())
                atomic_json(path,mapped_paths(value,receipt['remote_episode_dir'],str(out)))
            atomic_json(transport/'path_mapping.json',{'remote':receipt['remote_episode_dir'],'local':str(out)})
            result=read_json(out/'result.json')
            if not result or result.get('configuration_hash')!=fingerprint(c):
                raise ValueError('Remote result configuration hash mismatch')
            result['execution_backend']='osmo';result['osmo_workflow_id']=name
            atomic_json(out/'result.json',result)
            return result
        except (Exception,KeyboardInterrupt) as exc:
            result={'schema_version':3,'scenario_id':c['name'],'configuration_hash':fingerprint(c),
                    'outcome':'user_stopped' if isinstance(exc,(UserStop,KeyboardInterrupt)) else 'infrastructure_error',
                    'termination':{'source':'osmo','reason':str(exc) or 'operator interrupt'},
                    'metrics':{},'result_dir':str(out),'execution_backend':'osmo','osmo_workflow_id':name}
            atomic_json(transport/'transport_error.json',result)
            if not (out/'result.json').exists():atomic_json(out/'result.json',result)
            return result
        finally:
            if name and not terminal:
                try:
                    client.cancel(name)
                    cancel_deadline=time.monotonic()+20
                    while time.monotonic()<cancel_deadline:
                        cancel_status,_=client.status(name)
                        if cancel_status=='COMPLETED' or cancel_status.startswith(('FAILED','CANCELED','CANCELLED')):break
                        time.sleep(1)
                    else:raise TimeoutError('Cancellation has not completed')
                except Exception:
                    atomic_json(transport/'cancellation_required.json',{'workflow_id':name,
                        'message':'Automatic cancellation failed; cancel this workflow before submitting another.'})
                    atomic_json(RUNTIME/'osmo_pending_cleanup.json',{'workflow_id':name})
            if tunnel:
                from runtime_process import stop_process
                stop_process(tunnel)
            atomic_json(RUNTIME/'remote_live.json',{'active':False,'run_id':payload['run_id'],
                'workflow_id':name,'local_output':str(out),'control_path':str(control_path)})
            publish('result',result=result)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario',type=Path)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='Workflow YAML to prepare; does not submit')
    parser.add_argument('--record-video',action='store_true')
    args=parser.parse_args()
    _,spec,_=prepare(yaml.safe_load(args.scenario.read_text()),load_settings(args.config),uuid.uuid4().hex,
                     record_video=args.record_video)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(yaml.safe_dump(spec,sort_keys=False));args.output.chmod(0o600)
    print('Prepared only; no workflow submitted: '+str(args.output))


if __name__=='__main__':main()
