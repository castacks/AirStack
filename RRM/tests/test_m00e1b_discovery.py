"""Offline passive-evidence parser, sanitization and disposition tests."""
import copy
import csv
import importlib.util
import json
from pathlib import Path
import unittest
ROOT=Path(__file__).resolve().parents[2];E=ROOT/'RRM/evidence/M00-E1B'
def mod(name):
 s=importlib.util.spec_from_file_location(name,ROOT/('RRM/scripts/'+name+'.py'));m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
m=mod('m00e1b_analyze');collector=mod('m00e1b_discover')
def rows(p):
 with p.open() as f:return list(csv.DictReader(f))
class DiscoveryTests(unittest.TestCase):
 def setUp(self):
  self.d,self.a,self.h=[json.loads((E/p).read_text()) for p in ['passive-discovery.json','installed-artifacts.json','host-and-lifecycle.json']]
 def test_retained_current_process_separation(self):
  r=m.derive(self.d,self.a,self.h);self.assertEqual(r['observed_process_birth_separation_s'],'21.62')
  self.assertEqual(r['newly_read_existing_log_app_to_graph_s'],'74.228771713')
  self.assertEqual(r['target_designation'],'UNVERIFIED');self.assertFalse(r['execution_authorized']);self.assertFalse(r['live_instrumentation_ready'])
  self.assertEqual(r['whole_unresolved_prerequisites_closed'],[]);self.assertEqual(r['startup_trials_executed'],0)
 def test_socket_inode_attribution(self):
  r=m.derive(self.d,self.a,self.h)
  self.assertTrue(any(b['socket']['local_port']==14581 and any(p['comm']=='px4' for p in b['owners']) for b in r['socket_ownership']))
 def test_missing_tail_event_stays_missing(self):
  for c in self.d['containers']:
   c['existing_log_excerpt']['selected_lines']=[line for line in c['existing_log_excerpt']['selected_lines'] if 'PX4 node at' not in line]
  self.assertIsNone(m.derive(self.d,self.a,self.h)['newly_read_existing_log_app_to_graph_s'])
 def test_missing_or_duplicate_process_rejected(self):
  for duplicate in [False,True]:
   d=copy.deepcopy(self.d)
   for c in d['containers']:
    ps=[p for p in c['processes'] if p['comm']=='px4']
    if ps:
     if duplicate:c['processes'].append(copy.deepcopy(ps[0]))
     else:c['processes']=[p for p in c['processes'] if p['comm']!='px4']
   with self.subTest(duplicate=duplicate):
    with self.assertRaises(ValueError):m.derive(d,self.a,self.h)
 def test_invalid_tick_rates_rejected(self):
  for value in [0,-1,True,'100']:
   h=copy.deepcopy(self.h);h['clock_ticks_per_second']=value
   with self.subTest(value=value):
    with self.assertRaises(ValueError):m.derive(self.d,self.a,h)
 def test_missing_boot_rejected(self):
  self.d['host']['boot_id']=''
  with self.assertRaises(ValueError):m.derive(self.d,self.a,self.h)
 def test_malformed_proc_stat_rejected(self):
  for text in ['no parenthesis','1 (name) R 1 2','1 (name) R '+'bad '*20]:
   with self.subTest(text=text):
    with self.assertRaises(ValueError):m.start_ticks(text)
 def test_comm_with_spaces_parentheses(self):
  fields=['R']+['0']*18+['12345']
  self.assertEqual(m.start_ticks('1 (name with ) paren) '+' '.join(fields)),12345)
 def test_decode_endpoint_v4_v6(self):
  self.assertEqual(collector.decode_endpoint('0100007F:11D1'),('127.0.0.1',4561))
  self.assertEqual(collector.decode_endpoint('00000000000000000000000001000000:0016',True),('::1',22))
 def test_environment_credentials_excluded(self):
  data=b'ROS_DOMAIN_ID=1\0HF_TOKEN=secret\0AWS_SECRET_ACCESS_KEY=secret\0PASSWORD=secret\0ROS_DISTRO=jazzy\0'
  self.assertEqual(collector.selected_env(data),{'ROS_DOMAIN_ID':'1','ROS_DISTRO':'jazzy'})
 def test_unknown_inode_does_not_invent_owner(self):
  a=copy.deepcopy(self.a)
  for p in a['process_artifacts']:p['selected_fd_targets']=[]
  self.assertTrue(all(not s['owners'] for s in m.derive(self.d,a,self.h)['socket_ownership']))
class DispositionTests(unittest.TestCase):
 def test_all20_ids_17_unresolved_and_no_downgrade(self):
  old=rows(ROOT/'RRM/evidence/M00-E1A/M00E1A_PREREQUISITE_REGISTER.csv');new=rows(E/'M00E1B_PREREQUISITE_DELTA.csv')
  self.assertEqual(len(new),20);self.assertEqual(sum(x['status']!='SATISFIED' for x in old),17)
  for a,b in zip(old,new):
   self.assertEqual(a['prerequisite_id'],b['prerequisite_id']);self.assertEqual(a['status'],b['previous_disposition']);self.assertEqual(a['status'],b['result'])
   self.assertEqual(b['whole_prerequisite_closed_in_e1b'],'NO')
   for field in ['new_evidence','new_evidence_source','verification_method','remaining_uncertainty','responsible_party','next_closure_action']:self.assertTrue(b[field])
 def test_class_d_precedes_c(self):
  for r in rows(E/'M00E1B_PREREQUISITE_DELTA.csv'):
   if 'D' in r['gap_classes'].split(';'):self.assertEqual(r['safety_precedence'],'D_PRECEDES_ANY_CLASS_C_ACTIVITY')
if __name__=='__main__':unittest.main()
