"""Spawned fake native experts exercise the actual pipe/controller boundaries."""
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from Scripts.Setup import closed_loop_sparse


def fake_worker(camera,options,pipe):
    rows=json.loads(Path(options['manifest']).read_text())['pairs'];sequence=0;last=0;pending=None
    def emit(event):
        nonlocal sequence
        pipe.send({'kind':'observer_event','event':dict(event,sequence=sequence,modality=camera)});sequence+=1
    for i in range(len(rows)):
        pipe.send({'kind':'ready','source':i});kind,source,requested=pipe.recv();assert kind=='step' and source==i
        if requested and i:
            if pending:
                a,b=pending;emit({'kind':'motion_result_written','stage':'pair_processing','time0_ns':a,'time1_ns':b,'first_write_for_submission':True,'original_need_interp':False,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]})
            a=rows[last][camera+'_left']['time_ns'];b=rows[i][camera+'_left']['time_ns']
            emit({'kind':'pair_end','stage':'pair_processing','time0_ns':a,'time1_ns':b,'execution_error':None,'original_need_interp':False});pending=(a,b);last=i
        pipe.send({'kind':'completed','source':i,'executed':requested,'processed_count':0})
    pipe.send({'kind':'drain_ready'});assert pipe.recv()==('drain',)
    if pending:
        a,b=pending;emit({'kind':'motion_result_written','stage':'termination','time0_ns':a,'time1_ns':b,'first_write_for_submission':True,'original_need_interp':False,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]})
    pipe.send({'kind':'finished'});pipe.close()


class ControllerTests(unittest.TestCase):
    def test_actual_spawned_prefix_feedback_and_skips(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp);rows=[{c+'_left':{'time_ns':100+i*10} for c in ['rgb','thermal']} for i in range(9)]
            manifest=base/'manifest.json';manifest.write_text(json.dumps({'pairs':rows}))
            original=closed_loop_sparse.worker;closed_loop_sparse.worker=fake_worker
            try:closed_loop_sparse.run(SimpleNamespace(manifest=manifest,dataset=base,output=base/'out',frames=9,mode='bounded_probe',timeout_seconds=10))
            finally:closed_loop_sparse.worker=original
            log=[json.loads(l) for l in (base/'out/timeline.jsonl').read_text().splitlines()]
            self.assertEqual(json.loads((base/'out/summary.json').read_text())['status'],'PASS_OFFLINE_NATIVE_FEEDBACK')
            self.assertTrue(any(e['kind']=='skip_release' for e in log))
            ignored=[e for e in log if e['kind']=='ingress' and e['reason']=='non_processing_stage']
            self.assertEqual(len(ignored),2)
            for e in log:
                if e['kind']=='ingress' and e['accepted']:self.assertLess(e['receipt_ns'],e['controller_monotonic_ns'])
            for group in range(1,9):
                decision=next(e for e in log if e['kind']=='preselection' and e['pair']==group-1)
                releases=[e for e in log if e['kind'] in ['release','skip_release'] and e['source']==group]
                self.assertTrue(all(decision['sequence']<e['sequence'] for e in releases))

if __name__=='__main__':unittest.main()
