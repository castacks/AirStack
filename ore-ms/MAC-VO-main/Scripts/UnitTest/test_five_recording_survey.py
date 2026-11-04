"""Fixed-source association and chronological-window counterexamples."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from five_recording_survey import stream_records,select_thirds,STREAMS,pair_streams

def rows(n=12):
 return [{k:{'time_ns':i*100000000,'index':i,'path':k+str(i)} for k in STREAMS} for i in range(n)]
class SamplingTests(unittest.TestCase):
 def test_invalid_missing_duplicate_keep_original_indices(self):
  text='1\nnan\n1\n1.3\n1.4\n';names={f'sensors/x/{i:08d}.png' for i in [0,1,2,4]}
  r,s=stream_records(text,'x',names);self.assertEqual([x['index'] for x in r],[0,4])
  self.assertEqual((s['nonfinite'],s['duplicate_ns'],s['missing_image']),(1,1,1))
 def test_exact_decimal_ns(self):
  r,s=stream_records('1761615290.010015011','x',{'sensors/x/00000000.png'})
  self.assertEqual(r[0]['time_ns'],1761615290010015011)
 def test_first_window_each_elapsed_third(self):
  r=rows();w=select_thirds(r,[a['rgb_left'] for a in r],2)
  self.assertEqual([x['manifest_start'] for x in w],[0,4,8])
 def test_gap_cannot_be_ignored(self):
  r=rows();r[1]['thermal_left']['time_ns']=200000000
  w=select_thirds(r,[a['rgb_left'] for a in r],2);self.assertEqual(w[0]['manifest_start'],2)
 def test_no_cross_third_substitute(self):
  r=rows(3);w=select_thirds(r,[a['rgb_left'] for a in r],2)
  self.assertTrue(all(x['rows']==[] for x in w))
 def test_empty_rgb_retained(self):
  self.assertEqual([x['status'] for x in select_thirds([],[])],['NO_RGB_ACQUISITIONS']*3)
 def test_insufficient_window_no_shorter_replacement(self):
  r=rows(99);self.assertTrue(all(not w['rows'] for w in select_thirds(r,[x['rgb_left'] for x in r])))
 def test_pairing_earlier_tie_and_no_reuse(self):
  stream={k:[] for k in STREAMS}
  stream['rgb_left']=[{'time_ns':100,'index':0},{'time_ns':102,'index':1}]
  stream['rgb_right']=[{'time_ns':99,'index':0},{'time_ns':101,'index':1}]
  stream['thermal_left']=[{'time_ns':100,'index':0},{'time_ns':102,'index':1}]
  stream['thermal_right']=[{'time_ns':100,'index':0},{'time_ns':102,'index':1}]
  p=pair_streams(stream,2,30);self.assertEqual([r['rgb_right']['index'] for r in p],[0,1])
if __name__=='__main__':unittest.main()
