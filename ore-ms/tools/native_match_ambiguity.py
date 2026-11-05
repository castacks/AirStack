"""EXP149 initialized native stereo replay; ambiguity/error diagnostic only."""
import argparse,json,hashlib,subprocess,time,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/native-match-ambiguity/run-001';BASE=ROOT/'results/native-depth-proxy/run-001';CONS=ROOT/'results/depth-projection-sensitivity/run-001';PLAN=ROOT/'experiments/native-match-ambiguity-2026-11-05.json'
PARAMS=dict(minimum_rank_points=10,patch_competitor_separation=2,variables=['descriptor_margin','descriptor_ratio','patch_relative_gap'],groups=['all_primary','locally_consistent'],timeout_seconds=600)

def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def write(p,v):
 p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')
def local(p):
 s=str(p)
 for prefix in ['/workspace/','/root/AirStack/ore-ms/']:
  if s.startswith(prefix):return ROOT/s[len(prefix):]
 return ROOT/s

def prepare():
 source=read(BASE/'native-selected.json');records=[];pins={}
 for mode in ['rgb','thermal']:
  old=ROOT/'results/native-stereo-history/run-002/captures'/(mode+'-paired');p=read(old/'plan.json');images={(Path(v['path']).parent.name,Path(v['path']).stem):v for v in p['prepared_images']};lines=[]
  settings=OUT/'settings'/(mode+'.yaml');settings.parent.mkdir(parents=True,exist_ok=True);settings.write_bytes((old/'settings.yaml').read_bytes())
  for r in [v for v in source['records'] if v['modality']==mode]:
   primary=read(BASE/'points'/mode/(r['stem']+'.json'));points=[v for v in primary['points'] if v['reason']=='compared'];groups={v['feature_index']:v['group'] for v in read(CONS/'points'/mode/(r['stem']+'-r2.json'))['points']};paths=[]
   for side in ['left','right']:
    rec=images[(side,r['stem'])];path=local(rec['path'])
    if sha(path)!=rec['sha256']:raise ValueError('canonical image changed')
    paths.append(str(path));pins[str(path.relative_to(ROOT))]=rec['sha256']
   targets=[dict(feature_index=v['feature_index'],native=v['native'],metrics=v['metrics'],reference_group=groups[v['feature_index']]) for v in points]
   records.append(dict(modality=mode,index=r['index'],stem=r['stem'],historical=r['native'],targets=targets,shape=r['shape'],K=r['K'],baseline_m=r['baseline_m']))
   lines.append('\t'.join([str(r['index']),*paths,str(r['native']['time_ns']),','.join(str(v['feature_index']) for v in targets)])+'\n')
  f=OUT/'inputs'/(mode+'.tsv');f.parent.mkdir(exist_ok=True);f.write_text(''.join(lines))
 write(OUT/'prepared.json',dict(records=records,canonical_sha256=pins));print('PREPARED40pairs/1398primary targets')

def replay(namespace=OUT):
 prepared=read(OUT/'prepared.json');pins={}
 for n,h in prepared['canonical_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('canonical source pin')
 for mode in ['rgb','thermal']:
  namespace.mkdir(parents=True,exist_ok=True);runner='/opt/ore-ms-ambiguity/build/orb_match_ambiguity_runner';cmd=[runner,str(OUT/'settings'/(mode+'.yaml')),str(OUT/'inputs'/(mode+'.tsv')),str(namespace/(mode+'.jsonl'))]
  write(namespace/(mode+'-execution.json'),dict(command=cmd,runner_sha256=sha(runner),native_library_sha256=sha('/opt/ORB_SLAM3/lib/libORB_SLAM3.so')))
  with (namespace/(mode+'.log')).open('x') as f:
   try:result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=600);code=result.returncode
   except subprocess.TimeoutExpired:code=124
  write(namespace/(mode+'-status.json'),dict(returncode=code,terminal_success=code==0))
  if code:raise ValueError('native replay failure '+mode)
 print('REPLAY40PAIRS COMPLETE')

def fidelity(namespace=OUT):
 prepared=read(OUT/'prepared.json');report=[]
 for mode in ['rgb','thermal']:
  native={v['index']:v for v in [json.loads(s) for s in (namespace/(mode+'.jsonl')).read_text().splitlines()]}
  for r in [v for v in prepared['records'] if v['modality']==mode]:
   row=native[r['index']];old=r['historical']['features'];new=row['features'];geometry=len(old)==len(new) and all(a[:3]==b[:3] for a,b in zip(old,new));same=[a[:5]==b for a,b in zip(old,new)] if len(old)==len(new) else []
   target_exact=sum(geometry and same[v['feature_index']] for v in r['targets'])
   report.append(dict(modality=mode,index=r['index'],features=len(old),coordinates_exact=geometry,all_geometry_exact=bool(geometry and all(same)),geometry_mismatches=len(old)-sum(same),primary_targets=len(r['targets']),primary_exact=target_exact))
 write(namespace/'fidelity.json',dict(runs=report,all_coordinates_exact=all(r['coordinates_exact'] for r in report),all_geometry_exact=all(r['all_geometry_exact'] for r in report),primary_targets=sum(r['primary_targets'] for r in report),primary_exact=sum(r['primary_exact'] for r in report),scope='initialized stereo replay compared to historical TrackStereo snapshots'))
 print('FIDELITY',sum(r['primary_exact'] for r in report),'/',sum(r['primary_targets'] for r in report),'primary targets;',sum(r['geometry_mismatches'] for r in report),'all-feature geometry differences')

def freeze(runtime):
 files=[Path(__file__),ROOT/'tools/native_match_ambiguity_audit.py',ROOT/'tools/tests/test_native_match_ambiguity.py',ROOT/'tools/native_depth_proxy.py',ROOT/'tools/native_depth_proxy_audit.py',ROOT/'tools/ms2_broader_intake.py',ROOT/'environment/orb_match_ambiguity_runner.cc',ROOT/'environment/orb_match_ambiguity_CMakeLists.txt',ROOT/'environment/Dockerfile.orb-match-ambiguity',ROOT/'docs/research/native-match-ambiguity-protocol-2026-11-05.md',OUT/'prepared.json',OUT/'runtime-identity.json',BASE/'summary.json',CONS/'summary.json']
 files.append(BASE/'native-selected.json')
 for r in read(OUT/'prepared.json')['records']:
  files += [BASE/'points'/r['modality']/(r['stem']+'.json'),CONS/'points'/r['modality']/(r['stem']+'-r2.json')]
 for mode in ['rgb','thermal']:files += [OUT/'settings'/(mode+'.yaml'),OUT/'inputs'/(mode+'.tsv')]
 pins={str(f.relative_to(ROOT)):sha(f) for f in files};pins.update(read(OUT/'prepared.json')['canonical_sha256'])
 p=dict(parent='f1aa3bf1dc0ffa9848d2e9f82c8140509f8a2866',parameters=PARAMS,runtime=runtime,score_runtime='sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c',source_sha256=pins,physical_reference_qualified=False,benchmark_execution_ready=False,matcher_tuned=False,new_trajectory=False)
 write(PLAN,p);write(OUT/'study-lock.json',p);print('FROZEN',len(pins),'pins',runtime)

def verify():
 p=read(PLAN)
 if sha(PLAN)!=sha(OUT/'study-lock.json') or p['parameters']!=PARAMS:raise ValueError('study/parameter lock')
 for n,h in p['source_sha256'].items():
  if sha(ROOT/n)!=h:raise ValueError('changed source '+n)
 return p

def ambiguity(point):
 costs=sorted(v[1] for v in point['candidates']);margin=costs[1]-costs[0] if len(costs)>1 else None;ratio=costs[0]/costs[1] if len(costs)>1 and costs[1]>0 else None
 values=point.get('costs');gap=None;tie=None
 if values is not None:
  i=min(range(len(values)),key=lambda j:values[j]);alternatives=[v for j,v in enumerate(values) if abs(j-i)>=2]
  if alternatives:gap=(min(alternatives)-values[i])/max(1,values[i]);tie=min(alternatives)==values[i]
 return dict(descriptor_margin=margin,descriptor_ratio=ratio,patch_relative_gap=gap,descriptor_tie=margin==0 if margin is not None else None,patch_separated_tie=tie)

def reconstruct(point,frame,feature):
 if point.get('patch_status')!='ok':return None
 costs=point['costs'];i=min(range(11),key=lambda j:costs[j])
 if i in [0,10]:return None
 a,b,c=[np.float32(costs[j]) for j in [i-1,i,i+1]];den=np.float32(2)*np.float32(np.float32(a+c)-np.float32(2)*b)
 if den==0:return None
 delta=np.float32(np.float32(a-c)/den)
 if not np.isfinite(delta) or delta<-1 or delta>1:return None
 shift=i-5;ur=np.float32(np.float32(frame['scale_factors'][int(feature[2])])*np.float32(np.float32(np.float32(point['scaled_right_center'])+np.float32(shift))+delta));d=np.float32(np.float32(feature[0])-ur)
 if not(0<=d<np.float32(frame['max_disparity'])):return None
 if d<=0:d=np.float32(.01);ur=np.float32(float(feature[0])-.01)
 return [float(ur),float(np.float32(np.float32(frame['bf'])/d))]

def correlation(points,key):
 from scipy.stats import spearmanr
 selected=[p for p in points if p['faithful'] and p['ambiguity'][key] is not None];xs=[p['ambiguity'][key] for p in selected];ys=[p['original_metrics']['abs_relative_error'] for p in selected]
 reason='fewer_than_10' if len(selected)<10 else 'constant' if len(set(xs))<2 or len(set(ys))<2 else None
 return dict(points=len(selected),rho=float(spearmanr(xs,ys).statistic) if reason is None else None,reason=reason)

def analyze():
 from native_depth_proxy import stats
 verify();prepared=read(OUT/'prepared.json');cases=[];rows=[]
 for mode in ['rgb','thermal']:
  frames={v['index']:v for v in [json.loads(s) for s in (OUT/(mode+'.jsonl')).read_text().splitlines()]}
  for r in [v for v in prepared['records'] if v['modality']==mode]:
   frame=frames[r['index']];native={v['feature_index']:v for v in frame['points']};points=[];geometry=len(frame['features'])==len(r['historical']['features']) and all(a[:3]==b[:3] for a,b in zip(frame['features'],r['historical']['features']))
   for target in r['targets']:
    idx=target['feature_index'];p=native[idx];historical=geometry and frame['features'][idx]==target['native'][:5];reconstructed=reconstruct(p,frame,frame['features'][idx]);native_exact=reconstructed==frame['features'][idx][3:5]
    points.append(dict(feature_index=idx,reference_group=target['reference_group'],historical_exact=historical,native_cost_reconstruction_exact=native_exact,faithful=bool(historical and native_exact),original_metrics=target['metrics'],ambiguity=ambiguity(p)))
   groups={}
   for group in PARAMS['groups']:
    subset=points if group=='all_primary' else [p for p in points if p['reference_group']=='locally_consistent'];valid=[p for p in subset if p['faithful']]
    groups[group]=dict(points=len(subset),faithful=len(valid),descriptor_ties=sum(p['ambiguity']['descriptor_tie'] is True for p in valid),patch_ties=sum(p['ambiguity']['patch_separated_tie'] is True for p in valid),correlations={key:correlation(subset,key) for key in PARAMS['variables']},variables={key:stats([p['ambiguity'][key] for p in valid if p['ambiguity'][key] is not None]) for key in PARAMS['variables']},all_original_absrel=stats([p['original_metrics']['abs_relative_error'] for p in subset]),faithful_original_absrel=stats([p['original_metrics']['abs_relative_error'] for p in valid]))
   case=dict(modality=mode,index=r['index'],groups=groups);write(OUT/'analysis'/mode/(r['stem']+'.json'),dict(case=case,points=points));cases.append(case)
 for mode in ['rgb','thermal']:
  for group in PARAMS['groups']:
   data=[c['groups'][group] for c in cases if c['modality']==mode];rows.append(dict(modality=mode,group=group,points=sum(v['points'] for v in data),faithful=sum(v['faithful'] for v in data),descriptor_ties=sum(v['descriptor_ties'] for v in data),patch_ties=sum(v['patch_ties'] for v in data),correlations={key:dict(images_defined=sum(v['correlations'][key]['rho'] is not None for v in data),images_unavailable=sum(v['correlations'][key]['rho'] is None for v in data),negative_images=sum(v['correlations'][key]['rho'] is not None and v['correlations'][key]['rho']<0 for v in data),stats=stats([v['correlations'][key]['rho'] for v in data if v['correlations'][key]['rho'] is not None])) for key in PARAMS['variables']}))
 write(OUT/'summary.json',dict(cases=cases,aggregates=rows,physical_reference_qualified=False,benchmark_execution_ready=False,matcher_tuned=False,new_trajectory=False,scope='initialized native stereo replay; label association only on faithful historical and native cost reconstruction'))
 verify();print('ANALYZED40cases')
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('action',choices=['prepare','replay','fidelity','freeze','verify','analyze']);a.add_argument('--runtime');a.add_argument('--smoke',action='store_true');x=a.parse_args();ns=OUT/'smoke-040' if x.smoke else OUT
 {'prepare':prepare,'replay':lambda:replay(ns),'fidelity':lambda:fidelity(ns),'freeze':lambda:freeze(x.runtime),'verify':verify,'analyze':analyze}[x.action]()
