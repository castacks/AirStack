"""Save fixed signal-based choices before any proxy/reference labels are opened."""
import argparse,itertools,os,sys
from pathlib import Path
from Scripts.Setup.flow_cycle_experiment import read,write,sha,verify
POLICIES=('rgb','thermal','variance','closure')


def choose(cameras,policy):
 if policy in ('rgb','thermal'):return policy if cameras[policy]['motion_available'] else None
 key='angular_covariance_median' if policy=='variance' else 'closure_median'
 candidates=[(cameras[c][key],i,c) for i,c in enumerate(('rgb','thermal')) if cameras[c]['motion_available'] and cameras[c][key] is not None]
 return min(candidates)[2] if candidates else None


def predict(root,output):
 assert not (output/'predictions.json').exists();denied=[]
 def guard(event,args):
  if event=='open':
   path,mode,flags=args
   if isinstance(path,(str,bytes)):
    p=Path(os.fsdecode(path));reading=('r' in mode or '+' in mode) if isinstance(mode,str) else not flags&os.O_WRONLY
    if reading and ('super_odometry' in p.parts or p.name in ('evaluation.json','pose-proxy.json','headroom.json')):denied.append(str(p));raise PermissionError('prediction label read blocked')
 sys.addaudithook(guard);p=read(output/'inference.json');execution=read(output/'execution.json');verify(root,p);assert execution['status']=='PASS_ALL_FORWARD_CONTROLS' and execution['inference_sha256']==sha(output/'inference.json')
 files=[output/'inference.json',output/'execution.json',Path(__file__)];streams={};captures={};stages={};health={}
 for spec in p['runs']:
  key=(spec['window'],spec['camera']);folder=root/spec['output_relative'];cap=read(folder/'capture.json');assert cap['status']=='PASS_CAPTURE_CONTROLS' and cap['completed_pairs']==spec['frames']-1 and not cap['denied_label_reads']
  source=root/spec['original_relative']/'arbiter-events/events.jsonl';events=[__import__('json').loads(x) for x in source.read_text().splitlines()];files += [source,folder/'capture.json']+sorted(folder.glob('pair-*.npz'));captures[key]=cap;refinement=min(e['sequence'] for e in events if e['kind']=='refinement_begin')
  for i in range(spec['frames']-1):
   times=cap['pairs'][i]['sensor_interval_ns'];h,=[e for e in events if e['kind']=='pair_end' and [e['time0_ns'],e['time1_ns']]==times];health[key+(i,)]=h
   first=[e for e in events if e['kind']=='motion_result_written' and e['first_write_for_submission'] and e['from_idx']==i and e['frame_idx']==i+1 and [e['time0_ns'],e['time1_ns']]==times]
   assert len(first)<=1
   streams[key+(i,)]=bool(first and not h['original_need_interp'] and h['execution_error'] is None and first[0]['sequence']<refinement and first[0]['derived_delta_unavailable_reason'] is None and not first[0]['original_need_interp'])
   stages[key+(i,)]=first[0]['stage'] if first else None
 frozen={str(f.relative_to(root)):sha(f) for f in sorted(set(files))};write(output/'prediction-protocol.json',{'input_sha256':frozen,'labels_used':False,'fitted':False,'scope':'offline sensor-causal signal choices; no shared-clock/deadline claim'})
 rows=[]
 for window in ('early','middle','day'):
  n=len(captures[window,'rgb']['pairs']);assert n==len(captures[window,'thermal']['pairs'])
  for i in range(n):
   cameras={}
   for c in ('rgb','thermal'):
    r=captures[window,c]['pairs'][i];cameras[c]={'motion_available':streams[window,c,i],'motion_stage':stages[window,c,i],'sensor_interval_ns':r['sensor_interval_ns'],'closure_median':r['closure_angular_q25_q50_q75'][1],'angular_covariance_median':r['angular_covariance_median'],'valid_closure_count':r['valid_closure_count'],'grid_count':r['grid_count']}
   rows.append({'window':window,'pair_ordinal':i,'cameras':cameras,'choices':{policy:choose(cameras,policy) for policy in POLICIES}})
 for path,h in frozen.items():assert sha(root/path)==h
 write(output/'predictions.json',{'prediction_protocol_sha256':sha(output/'prediction-protocol.json'),'rows':rows,'labels_used':False,'fitted':False,'label_guard_enabled':True,'denied_label_reads':denied})
 print({'predictions':len(rows),'denied_label_reads':denied})


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();predict(a.root,a.output)
