"""Score already saved label-free choices using unchanged qualified motion proxies."""
import argparse,itertools,math
from pathlib import Path
from Scripts.Setup.flow_cycle_experiment import read,write,sha,CASES
from Scripts.Setup.predict_flow_cycle import POLICIES


def stats(values):return {'count':len(values),'mean':math.fsum(values)/len(values),'rmse':math.sqrt(math.fsum(x*x for x in values)/len(values))} if values else {'count':0,'mean':None,'rmse':None}


def evaluate(root,output):
 assert not (output/'evaluation.json').exists();pred=read(output/'predictions.json');assert not pred['labels_used'] and not pred['fitted'] and not pred['denied_label_reads'] and pred['prediction_protocol_sha256']==sha(output/'prediction-protocol.json')
 inputs={str((output/n).relative_to(root)):sha(output/n) for n in ('inference.json','execution.json','prediction-protocol.json','predictions.json')};inputs[str(Path(__file__).relative_to(root))]=sha(Path(__file__));labels={}
 for window,_,previous,_ in CASES:
  b=root/'results'/previous/'run-001';report=read(b/'evaluation.json');proof=read(b/'validation.json');assert proof['status']=='PASS' and proof['evaluation_sha256']==sha(b/'evaluation.json') and not report['independent_ground_truth']
  for filename in ('evaluation.json','validation.json','evaluation-protocol.json'):inputs[str((b/filename).relative_to(root))]=sha(b/filename)
  labels[window]={(r['camera'],r['pair_ordinal']):r for r in report['rows'] if r['arm']=='original'}
 write(output/'evaluation-protocol.json',{'input_sha256':inputs,'predictions_saved_before_labels':True,'independent_ground_truth':False,'fit_performed':False,'primary':'translation_m','secondary':'rotation_deg','scope':'same nominal-body/reference-assisted SLAM proxy; original first writes, offline characterization'})
 rows=[];summaries={}
 for row in pred['rows']:
  window,i=row['window'],row['pair_ordinal'];scores={c:labels[window][c,i]['common_loss'] for c in ('rgb','thermal')}
  for c in ('rgb','thermal'):assert (scores[c] is not None)==row['cameras'][c]['motion_available'] and row['cameras'][c]['sensor_interval_ns']==labels[window][c,i]['sensor_interval_ns']
  rows.append({**row,'common_interval_ns':labels[window]['rgb',i]['common_interval_ns'],'camera_losses':scores,'policy_losses':{policy:scores[c] if c is not None else None for policy,c in row['choices'].items()}})
 for window in ('early','middle','day'):
  local=[r for r in rows if r['window']==window];both=[r for r in local if all(r['cameras'][c]['motion_available'] for c in ('rgb','thermal'))];union=[r for r in local if any(r['cameras'][c]['motion_available'] for c in ('rgb','thermal'))];processing=[r for r in both if all(r['cameras'][c]['motion_stage']=='pair_processing' for c in ('rgb','thermal'))];groups={}
  for name,subset in [('both',both),('union',union),('both_processing_only',processing)]:
   full={};matched={}
   for policy in POLICIES:
    selected=[r for r in subset if r['policy_losses'][policy] is not None];full[policy]={'available':len(selected),'missing':len(subset)-len(selected),'pair_ordinals':[r['pair_ordinal'] for r in selected],'choices':{c:sum(r['choices'][policy]==c for r in subset) for c in ('rgb','thermal',None)},'losses':{loss:stats([r['policy_losses'][policy][loss] for r in selected]) for loss in ('translation_m','rotation_deg')}}
   for a,b in itertools.combinations(POLICIES,2):
    selected=[r for r in subset if all(r['policy_losses'][p] is not None for p in (a,b))];matched[a+'/'+b]={'pair_ordinals':[r['pair_ordinal'] for r in selected],**{p:{loss:stats([r['policy_losses'][p][loss] for r in selected]) for loss in ('translation_m','rotation_deg')} for p in (a,b)}}
   groups[name]={'slots':len(subset),'pair_ordinals':[r['pair_ordinal'] for r in subset],'full':full,'matched':matched}
  summaries[window]={'all_slots':len(local),'populations':groups}
 for path,h in inputs.items():assert sha(root/path)==h
 write(output/'evaluation.json',{'evaluation_protocol_sha256':sha(output/'evaluation-protocol.json'),'rows':rows,'windows':summaries,'independent_ground_truth':False,'fit_performed':False,'scope':'offline diagnostic choices only; no trajectory, online availability or physical accuracy claim'})
 print({w:{p:s['populations']['both']['full'][p]['losses'] for p in POLICIES} for w,s in summaries.items()})


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();evaluate(a.root,a.output)
