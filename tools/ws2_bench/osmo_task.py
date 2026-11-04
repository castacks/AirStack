"""One remote flight plus a bounded HTTP preview/control service."""
import argparse
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import threading
from osmo_transport import JSON_FILES, atomic_json, read_json, pack_artifacts


def handler_type(runtime, payload, connected):
    runtime=Path(runtime);planner=payload['scenario']['planner']
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def authorized(self):
            return hmac.compare_digest(self.headers.get('Authorization',''), 'Bearer '+payload['token'])
        def send(self,data,status=200,kind='application/json'):
            if not isinstance(data,bytes):data=json.dumps(data).encode()
            self.send_response(status);self.send_header('Content-Type',kind)
            self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store')
            self.send_header('X-WS2-Run',payload['run_id']);self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            if not self.authorized():self.send({'error':'unauthorized'},403);return
            path=self.path.split('?')[0]
            if path=='/snapshot':
                data={k:read_json(runtime/v) for k,v in JSON_FILES.items()}
                data['inference']={planner:read_json(runtime/'inference'/(planner+'.json'))}
                self.send(data);return
            allowed={'/scene.jpg':runtime/'live_view.jpg',
                     '/inference/'+planner+'.jpg':runtime/'inference'/(planner+'.jpg')}
            if path not in allowed:self.send({'error':'not found'},404);return
            try:self.send(allowed[path].read_bytes(),kind='image/jpeg')
            except OSError:self.send({'error':'frame not available'},503)
        def do_POST(self):
            if not self.authorized():self.send({'error':'unauthorized'},403);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=4096:raise ValueError('Invalid request size')
                data=json.loads(self.rfile.read(length))
                if self.path=='/control':
                    if not isinstance(data,dict) or set(data)!={'action'} or data['action'] not in ('pause','run','stop'):
                        raise ValueError('Invalid operator request')
                    atomic_json(runtime/'remote_control.json',data);connected.set()
                elif self.path=='/view':
                    from dashboard import set_view
                    set_view(data)
                else:self.send({'error':'not found'},404);return
                self.send({'ok':True})
            except (ValueError,TypeError,KeyError) as exc:self.send({'error':str(exc)},400)
    return Handler


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    payload=json.loads(args.request.read_text())
    os.environ.update(WS2_REMOTE_TASK='1',WS2_EXECUTION_BACKEND='process')
    from episode import resolved,fingerprint,run_episode
    from run_conditions import RUNTIME
    import runtime_process
    scenario=resolved(payload['scenario'])
    if fingerprint(scenario)!=payload['configuration_hash']:raise ValueError('Request checksum mismatch')
    import re
    if not re.fullmatch(r'[a-f0-9]{32}',payload['run_id']):raise ValueError('Invalid run identity')
    if scenario['planner']!=runtime_process.image_manifest()['planner']:
        raise ValueError('Task image does not contain the selected target model')
    out=RUNTIME/'episodes'/payload['run_id'];RUNTIME.mkdir(parents=True,exist_ok=True)
    connected=threading.Event()
    server=ThreadingHTTPServer(('0.0.0.0',8894),handler_type(RUNTIME,payload,connected))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        if not connected.wait(timeout=90):raise TimeoutError('Local controller did not connect before flight startup')
        run_episode(scenario,out,camera=payload['camera'],record_bag=payload['record_bag'],
                    record_video=payload['record_video'],control_path=RUNTIME/'remote_control.json')
    except Exception as exc:
        out.mkdir(parents=True,exist_ok=True)
        if not (out/'result.json').exists():
            atomic_json(out/'result.json',{'schema_version':3,'scenario_id':scenario['name'],
                'configuration_hash':fingerprint(scenario),'outcome':'infrastructure_error',
                'termination':{'source':'remote_task','reason':str(exc)},'metrics':{},'result_dir':str(out)})
    finally:
        runtime_process.stop_all()
        server.shutdown();server.server_close();thread.join(timeout=5)
    pack_artifacts(out,args.output,payload['run_id'],fingerprint(scenario))
    # Exit 0 means artifacts were exported, not that the UAV completed its mission.
    print(json.dumps({'run_id':payload['run_id'],'result':read_json(out/'result.json')['outcome']}),flush=True)


if __name__=='__main__':main()
