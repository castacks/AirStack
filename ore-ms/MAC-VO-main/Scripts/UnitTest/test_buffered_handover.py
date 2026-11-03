"""Real encoded retention and spawned historical invocation/deadline semantics."""
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from Scripts.Setup.retained_inputs import EncodedInputBuffer,missing_previous_source,register_retained
from Scripts.Setup.buffered_feedback import FrozenRequestFeedback,BufferedReadinessFeedback
from Scripts.Setup import buffered_handover

def files_and_rows(root):
    rows=[]
    for i in range(100):
        row={}
        for c in ['rgb','thermal']:
            for side in ['left','right']:
                name=f'{c}_{side}_{i}.bin';(root/name).write_bytes(name.encode())
                row[c+'_'+side]={'path':name,'time_ns':100+10*i}
        rows.append(row)
    return rows

def fake_buffered_worker(camera,options,pipe):
    rows=json.loads(Path(options['manifest']).read_text())['pairs'];sequence=0;last=0;pending=None;count=0;context=None
    def emit(event,group,invocation):
        nonlocal sequence
        pipe.send({'kind':'observer_event','event':dict(event,sequence=sequence,modality=camera),
                   'invocation_source':invocation,'delivery_group':group});sequence+=1
    for group in range(len(rows)):
        pipe.send({'kind':'ready','source':group});kind,source,requested,payload=pipe.recv();assert kind=='step' and source==group
        batch=([payload['source']] if payload else [])+([group] if requested else [])
        for invocation in batch:
            if invocation:
                if pending:
                    a,b=pending
                    emit({'kind':'motion_result_written','stage':'pair_processing','time0_ns':a,'time1_ns':b,'first_write_for_submission':True,'original_need_interp':False,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]},group,invocation)
                a=rows[last][camera+'_left']['time_ns'];b=rows[invocation][camera+'_left']['time_ns']
                emit({'kind':'pair_end','stage':'pair_processing','time0_ns':a,'time1_ns':b,'execution_error':None,'original_need_interp':False},group,invocation)
                pending=(a,b);last=invocation
            count+=1
        pipe.send({'kind':'completed','source':group,'executed':requested,'buffered_count':int(payload is not None),'processed_count':count})
    pipe.send({'kind':'drain_ready'});assert pipe.recv()==('drain',)
    if pending:
        a,b=pending;emit({'kind':'motion_result_written','stage':'termination','time0_ns':a,'time1_ns':b,'first_write_for_submission':True,'original_need_interp':False,'derived_camera_delta_xyzw':[0,0,0,0,0,0,1]},None,None)
    pipe.send({'kind':'finished'});pipe.close()

class RetentionTests(unittest.TestCase):
    def test_actual_bytes_expire_by_source_age(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);rows=files_and_rows(root);b=EncodedInputBuffer(root,rows,False)
            b.advance('rgb',0);b.advance('rgb',1);old=(root/rows[1]['rgb_left']['path']).read_bytes()
            b.advance('rgb',2);(root/rows[1]['rgb_left']['path']).write_bytes(b'changed on disk')
            payload=b.retrieve('rgb',1,2);self.assertEqual(payload['images']['left']['data'],old)
            self.assertEqual(sorted(b.entries['rgb']),[1,2]);b.advance('rgb',3);self.assertNotIn(1,b.entries['rgb'])
    def test_physical_unavailability_stores_nothing(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);rows=files_and_rows(root);b=EncodedInputBuffer(root,rows,True)
            for i in range(22):b.advance('rgb',i)
            self.assertIsNone(b.retrieve('rgb',20,21));self.assertEqual(b.entries['rgb'],{})
    def test_query_cannot_read_future_old_or_wrong_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);rows=files_and_rows(root);b=EncodedInputBuffer(root,rows,False);b.advance('rgb',0)
            for source,group in [(0,0),(0,2),(1,2)]:
                with self.assertRaises(ValueError):b.retrieve('rgb',source,group)
    def test_strict_arrival_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            rows=files_and_rows(Path(tmp));b=EncodedInputBuffer(tmp,rows,False)
            with self.assertRaises(ValueError):b.advance('rgb',1)
    def test_one_missing_predecessor_and_two_query_budget(self):
        self.assertEqual(missing_previous_source(51,49,True,True,0),50)
        for args in [(51,48,True,True,0),(51,50,True,True,0),(51,49,False,True,0),(51,49,True,False,0),(51,49,True,True,2)]:
            self.assertIsNone(missing_previous_source(*args))
    def test_retained_registration_never_reexecutes(self):
        rows=[{c+'_left':{'time_ns':i*10} for c in ['rgb','thermal']} for i in range(100)]
        f=FrozenRequestFeedback(rows,[{'pair':i,'action':'RGB','requests':['rgb']} for i in range(99)])
        f.last_source['rgb']=49;f.plan(50,0)
        b=register_retained(f,'rgb',50);self.assertEqual(b['source_start'],49)
        with self.assertRaises(ValueError):register_retained(f,'rgb',50)
    def test_late_historical_invocation_cannot_satisfy_inactive_session(self):
        rows=[{c+'_left':{'time_ns':i*10} for c in ['rgb','thermal']} for i in range(100)]
        f=BufferedReadinessFeedback(rows,'bounded_probe');f.live['rgb']={'status':'pending'}
        with self.assertRaises(ValueError):f.ingest_group('rgb',{'kind':'pair_end'},1,50,51)

class SpawnedBufferTests(unittest.TestCase):
    def test_failing_request_fixture_off_on_and_late_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);rows=files_and_rows(root);manifest=root/'manifest.json';manifest.write_text(json.dumps({'pairs':rows}))
            fixture=[]
            for pair in range(99):
                source=pair+1;action='THERMAL' if source<=50 else 'RGB'
                request=['rgb','thermal'] if source<=2 or 47<=source<=49 or source>=98 else [action.lower()]
                fixture.append({'pair':pair,'action':action,'requests':request})
            path=root/'fixture.json';path.write_text(json.dumps({'decisions':fixture}))
            original=buffered_handover.worker;buffered_handover.worker=fake_buffered_worker
            try:
                for enabled in [False,True]:
                    buffered_handover.run(SimpleNamespace(manifest=manifest,dataset=root,output=root/str(enabled),frames=100,mode='bounded_probe',
                        timeout_seconds=10,perturb=True,readiness=False,catchup=enabled,fixture=path))
            finally:buffered_handover.worker=original
            off=[json.loads(l) for l in (root/'False/timeline.jsonl').read_text().splitlines()]
            on=[json.loads(l) for l in (root/'True/timeline.jsonl').read_text().splitlines()]
            def health(log):
                return next(e for e in log if e['kind']=='observer_event' and e['camera']=='rgb' and e['event']['kind']=='pair_end' and e['trigger_source']==51)['event']
            self.assertEqual(health(off)['time0_ns'],rows[49]['rgb_left']['time_ns'])
            self.assertEqual(health(on)['time0_ns'],rows[50]['rgb_left']['time_ns'])
            delayed=next(e for e in on if e['kind']=='observer_event' and e['camera']=='rgb' and e['trigger_source']==50 and e['event']['kind']=='motion_result_written')
            self.assertEqual(delayed['delivery_group'],51)
            self.assertGreater(delayed['delivery_group'],49+1)
            queries=[e for e in on if e['kind']=='buffer_query'];self.assertLessEqual(len(queries),2)
            report=json.loads((root/'True/retained-inputs.json').read_text());self.assertGreater(report['ingress_bytes'],0)

class DecodeTests(unittest.TestCase):
    def test_encoded_pixels_equal_existing_file_adapter(self):
        import cv2
        import numpy as np
        import torch
        from Scripts.Setup.decode_retained import image_bytes
        from DataLoader.Dataset.TD3 import read_uint8_rgb
        with tempfile.TemporaryDirectory() as tmp:
            for name,image in [('rgb',np.arange(12*10*3,dtype=np.uint8).reshape(12,10,3)),('gray',np.arange(12*10,dtype=np.uint8).reshape(12,10))]:
                path=Path(tmp)/(name+'.png');assert cv2.imwrite(str(path),image);data=path.read_bytes()
                packet={'data':data,'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
                torch.testing.assert_close(image_bytes(packet),read_uint8_rgb(path),atol=0,rtol=0)
                packet['sha256']='invalid'
                with self.assertRaises(ValueError):image_bytes(packet)

if __name__=='__main__':unittest.main()
