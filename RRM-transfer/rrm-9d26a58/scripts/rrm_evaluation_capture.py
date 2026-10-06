#!/usr/bin/env python3
"""Record evaluation independently of mission/review lifetime; never dispatch motion.

Write OUTPUT/finish.json with {"finish": true} after recovery. Completion requires
an inactive console mission and two distinct fresh ground-state receipts. Per-process
caps remain300s; control overlap and physical segment gaps are explicit artifacts.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import urlopen
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rrm.evaluation_capture import GroundCompletion, grounded_snapshot, physical_rollover, landing_protection, validate_control_finalization

READER = '''import json,sys,time
from pathlib import Path
p=Path(sys.argv[1]);found={}
if p.exists():
 with p.open('rb') as f:
  f.seek(max(0,p.stat().st_size-600000));lines=f.read().splitlines()
 for line in reversed(lines):
  try:r=json.loads(line)
  except ValueError:continue
  if r.get('channel') not in found:found[r.get('channel')]=r
print(json.dumps({'now':time.monotonic(),'rows':found}))
'''


def marker():
    return {'monotonic_s': time.monotonic(), 'wall_unix_s': time.time()}


def atomic(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2))
    tmp.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--truth-dir', type=Path, required=True)
    parser.add_argument('--console-url', default='http://127.0.0.1:8787')
    parser.add_argument('--session-seconds', type=float, default=1800)
    parser.add_argument('--physical-segment-seconds', type=float, default=120)
    parser.add_argument('--control-segment-seconds', type=float, default=240)
    parser.add_argument('--finish-after-s', type=float, help='grounded verification only; request finish after this delay')
    args = parser.parse_args()
    if not (0 < args.session_seconds <= 1800 and 5 <= args.physical_segment_seconds <= 120
            and 10 <= args.control_segment_seconds <= 240):
        parser.error('session/segment bounds invalid')
    if args.finish_after_s is not None and not 0 < args.finish_after_s < args.session_seconds:
        parser.error('finish delay must be within session budget')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    truth = args.truth_dir.resolve()
    source = Path(__file__).with_name('airstack_control_capture.py')
    session = 'rrm_eval_' + uuid.uuid4().hex
    source_snapshots = out/'source-snapshots'
    source_snapshots.mkdir()
    source_hashes = {}
    for source_path in (source, Path(__file__), Path(__file__).parents[1]/'rrm/evaluation_capture.py'):
        raw = source_path.read_bytes()
        (source_snapshots/source_path.name).write_bytes(raw)
        source_hashes[source_path.name] = hashlib.sha256(raw).hexdigest()
    atomic(out/'source-manifest.json', source_hashes)
    controls = []
    physical = []
    current = None
    controller = None
    started = time.monotonic()
    finish = GroundCompletion()
    result = 'INCOMPLETE'
    reason = 'session_budget'
    stop_requested = False

    def event(kind, **fields):
        with (out/'events.jsonl').open('a') as f:
            f.write(json.dumps({'event': kind, **marker(), **fields})+'\n')

    def docker(*command, timeout=15):
        return subprocess.check_output(['docker', 'exec', 'airstack-robot-desktop-1', *command],
                                       text=True, timeout=timeout)

    def begin_control():
        name = session + '_control_' + str(len(controls))
        logfile = (out/(name+'.log')).open('x')
        proc = subprocess.Popen(['docker', 'exec', 'airstack-robot-desktop-1', 'bash', '-lc',
            'sws >/dev/null; echo "$$" > /tmp/'+name+'.pid; exec python3 /tmp/'+session+'-capture.py'
            +' --duration 300 --output /tmp/'+name+'.jsonl --source-revision '+session],
            stdout=logfile, stderr=subprocess.STDOUT)
        item = {'name': name, 'start': marker(), 'process': proc, 'log': logfile}
        controls.append(item)
        event('control_started', name=name)
        return item

    def read_control(item):
        return json.loads(docker('python3', '/tmp/'+session+'-reader.py', '/tmp/'+item['name']+'.jsonl'))

    def end_control(item):
        if 'end' in item:
            return
        pid = docker('cat', '/tmp/'+item['name']+'.pid').strip()
        if not pid.isdigit():
            raise RuntimeError('Invalid recorder PID')
        subprocess.run(['docker','exec','airstack-robot-desktop-1','kill','-INT',pid], check=False,
                       capture_output=True, timeout=10)
        item['process'].wait(timeout=15)
        item['log'].close()
        item['end'] = marker()
        item['return_code'] = item['process'].returncode
        for suffix in ['.jsonl','.jsonl.summary.json']:
            subprocess.run(['docker','cp','airstack-robot-desktop-1:/tmp/'+item['name']+suffix,
                            str(out/(item['name']+suffix))], check=True, capture_output=True, timeout=15)
        summary = json.loads((out/(item['name']+'.jsonl.summary.json')).read_text())
        item['summary'] = summary
        validate_control_finalization(item['return_code'], summary, source_hashes[source.name])
        event('control_closed', name=item['name'], events=summary['events'], stop_reason=summary['stop_reason'])

    def request(name, enabled):
        atomic(truth/'capture.json', {'capture_id': name, 'enabled': enabled})

    def begin_physical():
        nonlocal current
        name = session+'_physical_'+str(len(physical))
        if (truth/(name+'.jsonl')).exists():
            raise RuntimeError('Capture ID already exists')
        current = {'capture_id':name, 'start':marker()}
        request(name, True)
        event('physical_requested', capture_id=name)

    def status():
        return json.loads((truth/'status.json').read_text())

    def end_physical():
        nonlocal current
        if current is None:
            return
        request(current['capture_id'],False)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            s=status()
            if s['capture_id']==current['capture_id'] and not s['recording']:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('Physical recorder did not acknowledge disable')
        current['end']=marker();current['status']=s
        raw=truth/(current['capture_id']+'.jsonl')
        (out/raw.name).write_bytes(raw.read_bytes())
        physical.append(current)
        atomic(out/'physical-segments.json', physical)
        event('physical_closed',capture_id=current['capture_id'],reason=s['reason'],records=s['records'])
        current=None
        if s.get('last_error') or s.get('reason') != 'request_disabled' or s.get('records',0) <= 0:
            raise RuntimeError('Physical finalization incomplete or failed: '+json.dumps(s))

    def shutdown(signum, frame):
        nonlocal stop_requested
        stop_requested=True
    signal.signal(signal.SIGINT,shutdown)
    signal.signal(signal.SIGTERM,shutdown)
    try:
        # Never take over another observer's active request.
        prior=status()
        if prior.get('recording'):
            raise RuntimeError('Another physical recording is active')
        subprocess.run(['docker','cp',str(source_snapshots/source.name),'airstack-robot-desktop-1:/tmp/'+session+'-capture.py'],check=True,capture_output=True)
        reader=out/'control-reader.py';reader.write_text(READER)
        subprocess.run(['docker','cp',str(reader),'airstack-robot-desktop-1:/tmp/'+session+'-reader.py'],check=True,capture_output=True)
        controller=begin_control();begin_physical()
        while time.monotonic()-started < args.session_seconds:
            if stop_requested:
                reason='interrupted';break
            if controller['process'].poll() is not None:
                raise RuntimeError('Control recorder exited before session completion')
            c=read_control(controller)
            try:
                with urlopen(args.console_url+'/api/mission',timeout=3) as response:
                    mission=json.load(response)
            except Exception as exc:
                mission={'active':None,'observation_error':str(exc)}
            landing = landing_protection(mission)
            landing_window = ((out/'landing-window.json').exists()
                and json.loads((out/'landing-window.json').read_text()).get('protect') is True)
            landing = landing or landing_window
            s=status()
            if s.get('capture_id')==current['capture_id']:
                if s.get('last_error') or not s['recording']:
                    raise RuntimeError('Physical recorder stopped or errored: '+json.dumps(s))
                if physical_rollover(elapsed_s=time.monotonic()-current['start']['monotonic_s'],
                                     bytes_written=s['bytes'],landing_active=landing,
                                     target_s=args.physical_segment_seconds):
                    end_physical();begin_physical()
            elif time.monotonic()-current['start']['monotonic_s']>8:
                raise RuntimeError('Physical recorder failed to start')
            if time.monotonic()-controller['start']['monotonic_s']>=args.control_segment_seconds:
                old=controller;new=begin_control();deadline=time.monotonic()+30
                while time.monotonic()<deadline:
                    if old['process'].poll() is not None or new['process'].poll() is not None:
                        raise RuntimeError('Control rollover lost a live recorder')
                    warm=read_control(new)
                    if len(warm['rows'])==18 and grounded_or_fresh(warm):
                        break
                    time.sleep(.5)
                else:
                    raise RuntimeError('New control segment did not warm within overlap budget')
                event('control_overlap_ready',old=old['name'],new=new['name'],overlap_s=time.monotonic()-new['start']['monotonic_s'])
                controller=new;end_control(old)
            if args.finish_after_s is not None and time.monotonic()-started>=args.finish_after_s:
                atomic(out/'finish.json',{'finish':True})
            requested=False
            if (out/'finish.json').exists():
                requested=json.loads((out/'finish.json').read_text()).get('finish') is True
            c["now"] = time.monotonic()
            snapshot={'session_id':session,'status':'RECORDING','mission_active':mission.get('active'),
                      'landing_active':landing,'grounded':grounded_snapshot(c['rows'],c['now']),
                      'landing_window_ready':bool(landing_window and s.get('capture_id') == current['capture_id']
                          and s.get('recording') and not s.get('last_error')
                          and time.monotonic()-current['start']['monotonic_s'] < 210
                          and s.get('bytes',0) < 52*1024**2),
                      'capture_ready':bool(s.get('capture_id') == current['capture_id'] and s.get('recording')
                          and s.get('records',0)>0 and len(c['rows'])==18 and grounded_or_fresh(c)),
                      'finish_requested':requested,'physical_status':s,'control_segment':controller['name'],**marker()}
            atomic(out/'status.json',snapshot)
            event('health',mission_active=mission.get('active'),grounded=snapshot['grounded'],finish_requested=requested)
            if finish.update(requested=requested,mission_active=mission.get('active'),rows=c['rows'],now=c['now']):
                result='COMPLETE';reason='fresh_ground_and_inactive_mission';break
            time.sleep(1)
    except Exception as exc:
        reason='error: '+str(exc)
        event('session_error',error=str(exc))
    finally:
        errors=[]
        for operation in [end_physical]+[lambda item=item:end_control(item) for item in controls]:
            try:operation()
            except Exception as exc:errors.append(str(exc))
        if errors:result='INCOMPLETE'
        exported=[{k:v for k,v in c.items() if k not in ('process','log')} for c in controls]
        atomic(out/'control-segments.json',exported)
        atomic(out/'session-result.json',{'status':result,'reason':reason,'cleanup_errors':errors,
            'session_id':session,'source_hashes':source_hashes,
            'started_monotonic_s':started,'ended':marker(),'physical_segments':len(physical),
            'control_segments':len(controls),'execution_dispatch':False})
    print(json.dumps({'status':result,'reason':reason,'output':str(out)}),flush=True)
    return 0 if result=='COMPLETE' else 1


def grounded_or_fresh(snapshot):
    """Warm subscriptions regardless of flight state; no freshness substitution."""
    return all(0 <= snapshot['now']-snapshot['rows'][name]['receipt_monotonic_s'] < 2
               for name in ('state','landed','armed','authority','odom','sim_clock'))


if __name__=='__main__':
    raise SystemExit(main())
