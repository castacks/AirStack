"""Exercise actual CLI orchestration with fake Docker/clock, without any ROS/flight."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPT=Path(__file__).parents[1]/'scripts/rrm_evaluation_capture.py'
spec=importlib.util.spec_from_file_location('evaluation_capture_cli',SCRIPT)
cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)


class FakeWorld:
    def __init__(self,truth, *, error=False, physical_error=False, warm=False):
        self.truth=truth;self.t=100.;self.processes={};self.copies={};self.error=error
        self.physical_error=physical_error;self.warm=warm;self.first_warm=None
        (truth/'status.json').write_text(json.dumps({'recording':False}))
    def monotonic(self):self.t+=.01;return self.t
    def sleep(self,n):self.t+=n
    def popen(self,command,**kwargs):
        name=command[-1].split(' > /tmp/')[1].split('.pid')[0]
        world=self
        class Process:
            returncode=None
            pid=len(world.processes)+100
            started=world.t
            def poll(self):return self.returncode
            def wait(self,timeout):return self.returncode
        p=Process();self.processes[name]=p;return p
    def atomic(self,path,value):
        cli_atomic(path,value)
        if path==self.truth/'capture.json':
            name=value['capture_id'];raw=self.truth/(name+'.jsonl')
            if value['enabled']:raw.write_text('{}\n')
            s={'capture_id':name,'recording':value['enabled'],'records':1,'bytes':raw.stat().st_size,
               'last_error':'test fault' if self.physical_error else None,
               'reason':'request_disabled','sampling_delivery':{'pending_completed_loop_sequence':1}}
            (self.truth/'status.json').write_text(json.dumps(s))
    def check_output(self,command,**kwargs):
        if 'cat' in command:
            name=command[-1].split('/')[-1][:-4];return str(self.processes[name].pid)
        if 'python3' in command:
            name=command[-1].split('/')[-1][:-6];p=self.processes[name]
            from test_evaluation_capture import rows
            r=rows(self.t)
            for i in range(13):r['extra'+str(i)]={'receipt_monotonic_s':self.t,'message':{}}
            # Required clock is part of18 streams.
            r.pop('extra12');r['sim_clock']={'receipt_monotonic_s':self.t,'message':{}}
            if self.warm and self.t-p.started<3:r.pop('state')
            return json.dumps({'now':self.t,'rows':r})
        raise AssertionError(command)
    def run(self,command,**kwargs):
        if command[:2]==['docker','cp']:
            src,dst=command[2:]
            if src.startswith('airstack-robot-desktop-1:/tmp/'):
                name=src.split('/')[-1].split('.jsonl')[0]
                if src.endswith('summary.json'):
                    snapshot=next(v for k,v in self.copies.items() if k.endswith('-capture.py'))
                    summary={'events':25,'channels':{str(i):{} for i in range(18)},'missing_channels':[],
                             'stop_reason':'error' if self.error else 'interrupted',
                             'recorder_sha256':hashlib.sha256(snapshot).hexdigest()}
                    Path(dst).write_text(json.dumps(summary))
                else:Path(dst).write_text('{}\n')
            else:self.copies[dst]=Path(src).read_bytes()
        elif 'kill' in command:
            pid=int(command[-1]);p=next(p for p in self.processes.values() if p.pid==pid);p.returncode=1 if self.error else 0
        else:raise AssertionError(command)
        return type('Result',(),{'returncode':0})()


cli_atomic=cli.atomic
class OrchestrationTests(unittest.TestCase):
    def exercise(self,*,finish=True,error=False,physical_error=False,warm=False):
        tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);root=Path(tmp.name);truth=root/'truth';truth.mkdir();out=root/'out'
        world=FakeWorld(truth,error=error,physical_error=physical_error,warm=warm)
        args=['capture','--output',str(out),'--truth-dir',str(truth),'--session-seconds','55',
              '--physical-segment-seconds','5','--control-segment-seconds','10']
        if finish:args+=['--finish-after-s','36']
        class Response(io.BytesIO):
            def __enter__(self):return self
            def __exit__(self,*args):self.close()
        with patch.object(sys,'argv',args),patch.object(cli.time,'monotonic',world.monotonic), \
             patch.object(cli.time,'sleep',world.sleep),patch.object(cli.subprocess,'Popen',world.popen), \
             patch.object(cli.subprocess,'run',world.run),patch.object(cli.subprocess,'check_output',world.check_output), \
             patch.object(cli,'atomic',world.atomic),patch.object(cli,'urlopen',return_value=Response(b'{"active":false}')), \
             patch.object(cli.signal,'signal'):
            # Each request requires a fresh response object.
            with patch.object(cli,'urlopen',side_effect=lambda *a,**k:Response(b'{"active":false}')):
                code=cli.main()
        result=json.loads((out/'session-result.json').read_text());events=[json.loads(x) for x in (out/'events.jsonl').read_text().splitlines()]
        self.assertFalse(result['execution_dispatch']);self.assertTrue(all(p.returncode is not None for p in world.processes.values()))
        return code,result,events,out

    def test_actual_rollover_warms_new_stream_before_closing_old(self):
        code,r,events,out=self.exercise(warm=True)
        self.assertEqual(code,0);self.assertEqual(r['status'],'COMPLETE');self.assertGreater(r['physical_segments'],2);self.assertGreater(r['control_segments'],2)
        overlap=[e for e in events if e['event']=='control_overlap_ready'];self.assertTrue(overlap)
        for e in overlap:
            close=next(x for x in events if x['event']=='control_closed' and x['name']==e['old'])
            self.assertGreaterEqual(e['overlap_s'],3);self.assertLess(e['monotonic_s'],close['monotonic_s'])
        manifest=json.loads((out/'source-manifest.json').read_text())
        for name,h in manifest.items():self.assertEqual(hashlib.sha256((out/'source-snapshots'/name).read_bytes()).hexdigest(),h)

    def test_budget_exhaustion_without_finish_stays_incomplete_and_cleans_up(self):
        code,r,events,out=self.exercise(finish=False)
        self.assertEqual(code,1);self.assertEqual(r['status'],'INCOMPLETE');self.assertEqual(r['reason'],'session_budget')
        self.assertFalse(r['cleanup_errors'])

    def test_final_recorder_error_racing_finish_cannot_publish_complete(self):
        code,r,events,out=self.exercise(error=True)
        self.assertEqual(code,1);self.assertEqual(r['status'],'INCOMPLETE');self.assertTrue(r['cleanup_errors'] or r['reason'].startswith('error:'))

    def test_physical_error_retains_segment_and_reports_incomplete(self):
        code,r,events,out=self.exercise(physical_error=True)
        self.assertEqual(code,1);self.assertEqual(r['status'],'INCOMPLETE');self.assertTrue(list(out.glob('*physical*.jsonl')))
