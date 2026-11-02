"""Independent checkpoint, frozen-state, journal and saved-array accounting."""
from pathlib import Path
import hashlib
import json
import math
import numpy as np
import torch
from Module.Network.FlowFormerCov import build_flowformer
from Module.Network.FlowFormerCov.checkpoint import load_cov_checkpoint
from yacs.config import CfgNode

ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/objective-comparison/run-001'
p=json.loads((OUT/'protocol.json').read_text());checks=0

def check(condition, label):
    global checks
    if not condition:raise AssertionError(label)
    checks+=1

def equal(a,b,label):
    if isinstance(a,torch.Tensor):check(isinstance(b,torch.Tensor) and a.dtype==b.dtype and a.shape==b.shape and torch.equal(a,b),label)
    elif isinstance(a,dict):
        check(a.keys()==b.keys(),label)
        for k in a:equal(a[k],b[k],label+'/'+str(k))
    elif isinstance(a,(list,tuple)):
        check(type(a)==type(b) and len(a)==len(b),label)
        for i,(x,y) in enumerate(zip(a,b)):equal(x,y,label+'/'+str(i))
    else:check(type(a)==type(b) and a==b,label)

def close(a,b,label):check(math.isclose(a,b,rel_tol=2e-12,abs_tol=2e-12),label)

for name,digest in p['pins'].items():check(hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name)
m=json.loads((ROOT/p['training_manifest_relative']).read_text())
architecture=json.loads((ROOT/m['architecture']['path']).read_text());base=build_flowformer({'latentcostformer':CfgNode(architecture)},torch.float32,torch.float32)
load_cov_checkpoint(base,torch.load(ROOT/m['initialization']['file']['path'],map_location='cpu',weights_only=True));initial=base.state_dict();summaries={}
for arm in list(p['models'])[1:]:
    folder=OUT/arm;state=torch.load(folder/'step-00000048.pt',weights_only=True,map_location='cpu');replay=torch.load(OUT/(arm+'_replay')/'step-00000048.pt',weights_only=True,map_location='cpu');equal(state,replay,arm+'/fullcheckpoint')
    rows=[json.loads(line) for line in (folder/'steps.jsonl').read_text().splitlines()];rr=[json.loads(line) for line in (OUT/(arm+'_replay')/'steps.jsonl').read_text().splitlines()];equal(rows[24:],rr,arm+'/journals')
    check(state['step']==48 and len(rows)==48 and len(rr)==24,arm+'/budget');check(len(state['trainable_names'])==24,arm+'/trainables')
    changed=[];frozen=0
    for name,t in initial.items():
        value=state['model'][name];check(torch.isfinite(value).all().item(),name+'/finite')
        if name.startswith('memory_decoder.cov_update.'):
            if not torch.equal(t,value):changed.append(name)
        else:equal(t,value,name+'/frozen');frozen+=1
    check(len(changed)==24,arm+'/changed')
    train=[s for s in m['samples'] if s['partition']=='train']
    for j,row in enumerate(rows):
        check(row['step']==j+1 and row['sample_sha256']==train[j]['file']['sha256'] and row['recording_id']==train[j]['recording_id'],arm+'/order')
        check(math.isfinite(row['loss']) and math.isfinite(row['gradient_norm_before_clip']) and row['valid_components']>0,arm+'/numeric')
    report=json.loads((folder/'report.json').read_text());rep=json.loads((OUT/(arm+'_replay')/'report.json').read_text())
    check(report['status']==rep['status']=='COMPLETED',arm+'/status');equal(report['validation_nll'],rep['validation_nll'],arm+'/validation')
    if arm!='recurrent_diagonal_nll':check(json.loads((folder/'objective-contract.json').read_text())['objective_mode']==arm,arm+'/objective')
    summaries[arm]={'changed_tensors':len(changed),'frozen_state_entries':frozen,'exact_checkpoint_and_journal_replay':True,'clipped_steps':sum(r['gradient_norm_before_clip']>m['optimization']['clip_norm'] for r in rows),'gradient_min':min(r['gradient_norm_before_clip'] for r in rows),'gradient_max':max(r['gradient_norm_before_clip'] for r in rows),'first_loss':rows[0]['loss'],'last_loss':rows[-1]['loss'],'validation_selected_objective':report['validation_nll']}

phase='training'
if (OUT/'metrics.json').exists():
    phase='evaluation';metrics=json.loads((OUT/'metrics.json').read_text());ev=json.loads((OUT/'evaluation.json').read_text());capture=json.loads((OUT/'capture.json').read_text());check(len(capture['rows'])==36,'36pairs');equal(ev['model_state_digest_before'],ev['model_state_digest_after'],'evaluation/state')
    counts={}
    for row,mr in zip(capture['rows'],metrics['rows']):
        file=ROOT/row['path'];check(hashlib.sha256(file.read_bytes()).hexdigest()==row['sha256'],'arrayhash')
        with np.load(file,allow_pickle=False) as a:
            residual=a['prediction'].astype('float64')-a['target'].astype('float64');loss=np.einsum('ij,ij->i',residual,residual);n=len(loss);check(n==row['pixels']==mr['pixels'],'support');check(np.all(np.diff(a['flat_index'])>0),'pixelorder')
            for arm in p['models']:
                raw=a[arm].astype('float64');var=np.exp(raw*2);q=(residual**2/var).sum(1);score=var.sum(1)
                final=float(np.mean(residual**2/(2*var)+raw));close(final,mr['arms'][arm]['final_nll'],'arrayNLL');close(final,ev['arms'][arm]['rows'][row['index']]['numpy64_final_nll'],'evalNLL')
                for level in p['coverage_levels']:check(int((q<=-2*math.log1p(-level)).sum())==mr['arms'][arm]['coverage_counts'][str(level)],'coverage')
                for j,f in enumerate(p['retained_fractions']):
                    k=max(1,int(f*n));threshold=np.partition(score,k-1)[k-1];before=np.flatnonzero(score<threshold);ties=np.flatnonzero(score==threshold);selected=np.concatenate([before,ties[:k-len(before)]])
                    check(len(selected)==k,'retainedcount');close(float(loss[selected].sum()),mr['arms'][arm]['risks'][j]['selected_error_sum'],'rankedsum');close(float(np.partition(loss,k-1)[:k].sum()),mr['arms'][arm]['risks'][j]['oracle_error_sum'],'oraclesum')
                for stage,baseline_stage in zip(ev['arms'][arm]['rows'][row['index']]['stages'],ev['arms']['baseline']['rows'][row['index']]['stages']):check(stage['mean_sha256']==baseline_stage['mean_sha256'],'frozenmean')
                identity=row['recording_id'];c=counts.setdefault(identity,{}).setdefault(arm,{'n':0,'risk':0.,'k':0,'coverage':0,'nll':0.})
                r=mr['arms'][arm];c['n']+=n;c['risk']+=r['risks'][2]['selected_error_sum'];c['k']+=r['risks'][2]['retained'];c['coverage']+=r['coverage_counts']['0.95'];c['nll']+=final*n
    for group,g in metrics['groups'].items():
        for arm in p['models']:
            records=g['recordings'];close(sum(counts[i][arm]['risk']/counts[i][arm]['k'] for i in records)/len(records),g['arms'][arm]['risks'][2]['selected_mse'],'primarymacro');close(sum(counts[i][arm]['coverage']/counts[i][arm]['n'] for i in records)/len(records),g['arms'][arm]['coverage']['0.95'],'coveragemacro')
    summaries['primary_new_only']=metrics['groups']['new_only']
with (OUT/('audit-'+phase+'.json')).open('x') as stream:json.dump({'status':'PASS','checks':checks,'summaries':summaries},stream,indent=2);stream.write('\n')
print('PASS',phase,checks,flush=True)
