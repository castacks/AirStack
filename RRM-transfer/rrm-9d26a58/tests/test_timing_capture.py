import importlib.util
import io
import json
from pathlib import Path
import struct
import unittest
from types import SimpleNamespace
spec=importlib.util.spec_from_file_location('timing',Path(__file__).parents[1]/'scripts/airstack_timing_capture.py')
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)

def envelope(msgid,payload,magic=253):
    length=len(payload); padded=payload+b'\0'*((-length)%8)
    return {'sysid':2,'compid':1,'framing_status':1,'magic':magic,'msgid':msgid,
            'len':length,'payload64':list(struct.unpack('<'+'Q'*(len(padded)//8),padded))}

class TimingTests(unittest.TestCase):
    def test_local_position_full_and_zero_tail(self):
        p=struct.pack('<I6f',12345,1.,2.,3.,.1,.2,0.)
        r=t.decode(envelope(32,p));self.assertEqual(r['time_boot_ms'],12345)
        self.assertEqual(r['position_ned_m'],[1.,2.,3.])
        self.assertEqual(t.decode(envelope(32,p.rstrip(b'\0'))),r)
    def test_system_time_and_signed_timesync(self):
        r=t.decode(envelope(2,struct.pack('<QI',1234567890123,50)))
        self.assertEqual(r['time_unix_usec'],1234567890123)
        r=t.decode(envelope(111,struct.pack('<qq',-1,123)))
        self.assertEqual((r['tc1_ns'],r['ts1_ns']),(-1,123))
    def test_all_known_timestamp_prefixes(self):
        for msgid in [30,31,105,331]:
            size=t.SPECS[msgid][1];fmt='<I' if msgid in [30,31] else '<Q'
            p=struct.pack(fmt,555).ljust(size,b'\0');r=t.decode(envelope(msgid,p,253))
            self.assertEqual(r['time_boot_ms' if msgid in [30,31] else 'time_usec'],555)
    def test_bad_framing_length_identity_and_nonfinite(self):
        for key,value in [('framing_status',2),('framing_status',True),('len',29),('sysid',True),('payload64',[])]:
            m=envelope(32,bytes(28));m[key]=value
            with self.assertRaises(ValueError):t.decode(m)
        with self.assertRaises(ValueError):t.decode(envelope(32,struct.pack('<I6f',1,float('nan'),0,0,0,0,0)))
        with self.assertRaises(ValueError):t.decode(envelope(32,b'\1',254))
    def test_unknown_preserved_and_decode_error_retained(self):
        out=io.StringIO();c=t.TimingCapture(out,10,['raw_mavlink','timesync_status'])
        m=envelope(99,b'\1');c.record('raw_mavlink',m,11,10,'aabb')
        m=envelope(32,bytes(28));m['framing_status']=2;c.record('raw_mavlink',m,12,11,'ccdd')
        rows=[json.loads(x) for x in out.getvalue().splitlines()]
        self.assertFalse(rows[0]['decoded']['supported']);self.assertIn('decode_error',rows[1])
        self.assertEqual(rows[1]['message'],m);self.assertEqual(rows[0]['publisher_gid_hex'],'aabb')
        self.assertEqual(c.summary()['missing_channels'],['timesync_status'])
    def test_publisher_info_dict_and_object(self):
        self.assertEqual(t.publisher_gid({'publisher_gid':[1,2,255]}),'0102ff')
        self.assertEqual(t.publisher_gid(SimpleNamespace(publisher_gid=b'\x01')),'01')
        self.assertIsNone(t.publisher_gid({'publisher_gid':[]}))
        self.assertIsNone(t.publisher_gid({'source_timestamp':1}))
    def test_v1_high_id_and_unwritten_counters(self):
        with self.assertRaises(ValueError):t.decode(envelope(331,bytes(230),254))
        c=t.TimingCapture(io.StringIO(),0,['raw_mavlink'],max_bytes=1)
        m=envelope(32,bytes(28));m['framing_status']=2
        c.record('raw_mavlink',m,1,1,'gid')
        self.assertEqual(c.summary()['raw_message_id_counts'],{})
        self.assertEqual(c.summary()['decode_errors'],0)
    def test_limits_retain_prior_stream(self):
        out=io.StringIO();c=t.TimingCapture(out,0,['state'],max_events=1)
        c.record('state',{'armed':False},1,1,'gid');c.record('state',{},2,2,'gid')
        self.assertEqual(c.total,1);self.assertEqual(c.limit,'event_limit')
        c=t.TimingCapture(io.StringIO(),0,['state'],max_bytes=1)
        c.record('state',{},1,1,'gid');self.assertEqual(c.total,0);self.assertEqual(c.limit,'byte_limit')
if __name__=='__main__':unittest.main()
