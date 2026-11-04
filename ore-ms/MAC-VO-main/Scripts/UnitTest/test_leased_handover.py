"""Spawned shared-payload lifetime, bounded control and native-like controller test."""
import hashlib
import json
import multiprocessing as mp
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from Scripts.Setup.leased_transport import EncodedLeaseArena,BoundedConnection,mapped_payload,lease_identity
from Scripts.Setup import leased_handover
from Scripts.UnitTest.test_buffered_handover import files_and_rows,fake_buffered_worker


def payload(data=b'left-bytes', right=b'right-bytes'):
    return {'camera':'rgb','source':50,'images':{s:{'data':d,'bytes':len(d),
            'sha256':hashlib.sha256(d).hexdigest(),'path':s+'.bin','time_ns':50}
            for s,d in [('left',data),('right',right)]}}


def read_in_child(descriptor, raw):
    pipe=BoundedConnection(raw)
    with mapped_payload(descriptor,'rgb',51) as p:
        result={s:hashlib.sha256(x['data']).hexdigest() for s,x in p['images'].items()}
    pipe.send({'hashes':result,'ack':lease_identity(descriptor)});pipe.close()


def fake_leased_worker(camera,options,raw):
    class Adapter:
        def __init__(self):self.pipe=BoundedConnection(raw);self.context=None;self.descriptor=None
        def recv(self):
            command=self.pipe.recv()
            if command[0]=='step':
                self.descriptor=command[3];self.context=mapped_payload(self.descriptor,camera,command[1])
                p=self.context.__enter__();return command[:3]+(p,)
            return command
        def send(self,value):
            if value['kind']=='completed':
                self.context.__exit__(None,None,None);self.context=None
                value=dict(value,lease_ack=lease_identity(self.descriptor))
            self.pipe.send(value)
        def close(self):self.pipe.close()
    fake_buffered_worker(camera,options,Adapter())


class LeaseTests(unittest.TestCase):
    def test_spawn_exact_hashes_ack_and_reuse(self):
        arena=EncodedLeaseArena(128);ctx=mp.get_context('spawn')
        try:
            p=payload();d=arena.publish(p,'rgb',50,51)
            a,b=ctx.Pipe();child=ctx.Process(target=read_in_child,args=(d,b));child.start();b.close()
            pipe=BoundedConnection(a);self.assertTrue(pipe.poll(10));response=pipe.recv()
            child.join(10);self.assertEqual(child.exitcode,0)
            self.assertEqual(response['hashes'],{s:x['sha256'] for s,x in p['images'].items()})
            arena.acknowledge(response['ack']);d2=arena.publish(p,'rgb',50,51)
            self.assertGreater(d2['token'],d['token']);arena.acknowledge(lease_identity(d2));pipe.close()
        finally:arena.close()

    def test_busy_wrong_and_stale_ack_do_not_release(self):
        arena=EncodedLeaseArena(128)
        try:
            d=arena.publish(payload(),'rgb',50,51)
            with self.assertRaises(RuntimeError):arena.publish(payload(),'rgb',50,51)
            bad=dict(lease_identity(d),camera='thermal')
            with self.assertRaises(ValueError):arena.acknowledge(bad)
            self.assertIsNotNone(arena.active);arena.acknowledge(lease_identity(d))
            with self.assertRaises(ValueError):arena.acknowledge(lease_identity(d))
        finally:arena.close()

    def test_oversize_rejected_without_live_lease(self):
        arena=EncodedLeaseArena(4)
        try:
            self.assertIsNone(arena.publish(payload(),'rgb',50,51));self.assertIsNone(arena.active)
            self.assertEqual(arena.report()['oversize_rejections'],1)
        finally:arena.close()

    def test_tampered_hash_and_bounds_fail_and_cleanup(self):
        arena=EncodedLeaseArena(128)
        try:
            d=arena.publish(payload(),'rgb',50,51)
            d['images']['left']['sha256']='wrong'
            with self.assertRaises(ValueError):
                with mapped_payload(d,'rgb',51):pass
            d['images']['left']['sha256']=payload()['images']['left']['sha256'];d['images']['right']['offset']=129
            with self.assertRaises(ValueError):
                with mapped_payload(d,'rgb',51):pass
            arena.acknowledge(lease_identity(d))
        finally:arena.close()

    def test_identity_validation(self):
        arena=EncodedLeaseArena(128)
        try:
            with self.assertRaises(ValueError):arena.publish(payload(),'thermal',50,51)
            d=arena.publish(payload(),'rgb',50,51)
            with self.assertRaises(ValueError):
                with mapped_payload(d,'thermal',51):pass
            arena.acknowledge(lease_identity(d))
        finally:arena.close()

    def test_control_ceiling_and_no_payload_on_pipe(self):
        with self.assertRaises(ValueError):BoundedConnection.encode(('step',51,True,payload()))
        with self.assertRaises(ValueError):BoundedConnection.encode(('step',51,True,{'name':'x'*5000}))
        with self.assertRaises(ValueError):BoundedConnection.encode({'event':'x'*70000})
        arena=EncodedLeaseArena(128)
        try:
            d=arena.publish(payload(),'rgb',50,51)
            self.assertLess(len(BoundedConnection.encode(('step',51,True,d))),4096)
            arena.acknowledge(lease_identity(d))
        finally:arena.close()

    def test_spawned_controller_release_before_ack_and_native_adjacency(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);rows=files_and_rows(root);manifest=root/'manifest.json';manifest.write_text(json.dumps({'pairs':rows}))
            fixture=[]
            for pair in range(99):
                source=pair+1;action='THERMAL' if source<=50 else 'RGB'
                req=['rgb','thermal'] if source<=2 or 47<=source<=49 or source>=98 else [action.lower()]
                fixture.append({'pair':pair,'action':action,'requests':req})
            path=root/'fixture.json';path.write_text(json.dumps({'decisions':fixture}))
            original=leased_handover.worker;leased_handover.worker=fake_leased_worker
            try:
                leased_handover.run(SimpleNamespace(manifest=manifest,dataset=root,output=root/'run',frames=100,
                    mode='bounded_probe',timeout_seconds=10,perturb=True,readiness=False,catchup=True,fixture=path,
                    store_bytes=4096,arena_bytes=1024))
            finally:leased_handover.worker=original
            service=json.loads((root/'run/lease-service.json').read_text())
            self.assertEqual(service['arena']['leases'],1);self.assertEqual(service['arena']['acks'],1)
            self.assertIsNone(service['arena']['active'])
            log=[json.loads(l) for l in (root/'run/timeline.jsonl').read_text().splitlines()]
            completed=next(e for e in log if e['kind']=='completed' and e['lease_ack'])
            self.assertEqual(completed['source'],51)
            h=next(e for e in log if e['kind']=='observer_event' and e['camera']=='rgb' and e['trigger_source']==51 and e['event']['kind']=='pair_end')
            self.assertEqual(h['event']['time0_ns'],rows[50]['rgb_left']['time_ns'])


if __name__=='__main__':unittest.main()
