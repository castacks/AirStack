"""One fixed per-row count control; immutable crop/original parent outputs."""
import argparse,json,hashlib,contextlib
from pathlib import Path
import numpy as np,cv2
import pre_essential_exclusion as parent
from pre_essential_exclusion import Study as ParentStudy,fit,index_sha,POSE_FIELDS
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]
VARIANTS=['original','bottom','count']

def count_indices(n,k,name,front,arm,index):
    assert 0<=k<=n
    if k==0:return np.empty(0,dtype=np.int64)
    if k==n:return np.arange(n,dtype=np.int64)
    prefix=f'pre-E-matched-count-v1|seed=0|{name}|{front}|{arm}|{index}|'
    ranks=[(hashlib.sha256((prefix+str(j)).encode('utf-8')).digest(),j) for j in range(n)]
    return np.asarray(sorted(j for _,j in sorted(ranks)[:k]),dtype=np.int64)

@contextlib.contextmanager
def variant_scope():
    previous=parent.VARIANTS
    try:parent.VARIANTS=VARIANTS;yield
    finally:parent.VARIANTS=previous

class Study(ParentStudy):
    def freeze(self):
        prior=ROOT/self.cfg['parent'];audit=read(prior/'audit.json');assert audit['status']=='pass' and audit['original_raw_E_equality_gate']=='fail_retained';assert sha(prior/'predictions.jsonl')==read(prior/'prediction-lock.json')['predictions_sha256'];oldplan=read(prior/'preflight.json');assert sha(prior/'preflight.json')==read(prior/'input-freeze.json')['pins'][str((prior/'preflight.json').relative_to(ROOT))];planmap={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in oldplan};priorrows=[json.loads(l) for l in (prior/'predictions.jsonl').read_text().splitlines()];pm={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in priorrows};plan=[];ns={};files=[self.config,Path(__file__),ROOT/'tools/pre_essential_exclusion.py',ROOT/'tools/smokeseer_sift.py',ROOT/'tools/camera_priority_controls.py',ROOT/'tools/smoke_rgb_first_fallback.py',ROOT/'tools/subt_export_comparison.py',ROOT/'tools/deployment_lighting.py',ROOT/'MAC-VO-main/Scripts/Setup/compare_factor_proxy.py',ROOT/'tools/audit_matched_count_control.py',ROOT/'tools/test_matched_count_control.py',ROOT/self.cfg['protocol']]
        for name,c in self.cfg['records'].items():
            p,q=[ROOT/c[k] for k in ['matching','qualification']];raw,_=self.load(c);n=read(p/'input-freeze.json')['anchors'];assert len(raw)==len({(r['frontend'],r['arm'],r['index']) for r in raw})==4*n;ns[name]=n
            for r in raw:
                key=name,r['frontend'],r['arm'],r['index'];count=planmap[(name,'bottom')+key[1:]]['kept'];ids=count_indices(r.get('matches',0),count,*key)
                for variant in VARIANTS:
                    if variant=='count':kept=count;digest=index_sha(ids)
                    else:
                        old=planmap[(name,variant)+key[1:]];control=pm[(name,variant)+key[1:]];assert old['kept']==control['matches'] and old['kept_sha256']==control['kept_sha256'];kept=old['kept'];digest=old['kept_sha256']
                    plan.append({'recording':name,'variant':variant,'frontend':r['frontend'],'arm':r['arm'],'index':r['index'],'input_matches':r.get('matches',0),'kept':kept,'kept_sha256':digest,'qualified':r['qualified'],'fit_eligible':bool(r['qualified'] and kept>=8)})
            files.extend(p/f for f in ['predictions.jsonl','prediction-lock.json','audit.json','matcher-inputs.json','input-freeze.json','scores.json','summary.json','completion.json']);files.extend(q/f for f in ['intervals.json','population.json','audit.json']);files.append(ROOT/c['comparison_config']);files.append(ROOT/read(ROOT/c['comparison_config'])['intrinsics'])
        files.extend(ROOT/x for x in read(prior/'input-freeze.json')['pins']);files.extend([ROOT/'results/pre-essential-exclusion/source/audit-amendment-lock.json',ROOT/'tools/audit_pre_essential_exclusion_svd.py',ROOT/'docs/research/pre-essential-exclusion-audit-amendment-2026-11-16.md']);files.extend(prior/f for f in ['predictions.jsonl','prediction-lock.json','preflight.json','input-freeze.json','audit.json','baseline-parity.json','summary.json','scores.json','transitions.json','choices.json','choice-lock.json','completion.json']);self.out.mkdir(parents=True,exist_ok=False);write(self.out/'preflight.json',plan);files.append(self.out/'preflight.json');write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'anchors':ns,'expected_rows':len(plan),'runtime_image':self.cfg['runtime_image'],'ranking':'SHA256 per match, fixed seed0 tag; ascending original indices; one draw only'});print('All matched-count rows preflighted',len(plan))
    def run(self):
        self.verify();assert np.__version__=='2.2.6' and cv2.__version__=='5.0.0';cv2.setNumThreads(1);prior=ROOT/self.cfg['parent'];old=[json.loads(l) for l in (prior/'predictions.jsonl').read_text().splitlines() if json.loads(l)['variant'] in ['original','bottom']];pm={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in old};plan={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in read(self.out/'preflight.json')};rows=0;fresh=0;reuse=0
        with (self.out/'predictions.jsonl').open('x') as f:
            for r in old:f.write(json.dumps(dict(r,origin='parent'),allow_nan=False)+'\n');rows+=1
            for name,c in self.cfg['records'].items():
                raw,model=self.load(c)
                for p in raw:
                    key=name,p['frontend'],p['arm'],p['index'];target=plan[(name,'count')+key[1:]];indices=count_indices(p.get('matches',0),target['kept'],*key);digest=index_sha(indices);assert digest==target['kept_sha256'];prior_variant=next((v for v in ['original','bottom'] if pm[(name,v)+key[1:]]['kept_sha256']==digest),None)
                    if prior_variant is not None:control=pm[(name,prior_variant)+key[1:]];result={k:control[k] for k in POSE_FIELDS if k in control};call=False;seconds=0.;reuse+=1;origin='cache'
                    else:result,call,seconds=fit(p,model.camera(p['arm'])[3],indices);fresh+=call;origin='fresh'
                    r={k:p[k] for k in ['frontend','arm','index','qualified']};r.update(result,recording=name,variant='count',input_matches=p.get('matches',0),kept_sha256=digest,solver_called=call,reused=prior_variant is not None,reuse_variant=prior_variant,geometry_seconds=seconds,origin=origin);f.write(json.dumps(r,allow_nan=False)+'\n');rows+=1
                    if rows%500==0:print('Full control rows',rows,flush=True)
        assert rows==read(self.out/'input-freeze.json')['expected_rows'];self.verify();write(self.out/'prediction-lock.json',{'rows':rows,'predictions_sha256':sha(self.out/'predictions.jsonl'),'inherited_parent_rows':len(old),'new_solver_calls':fresh,'new_reused_rows':reuse,'new_learned_inference_calls':0,'reference_values_parsed':False});print('Count control predictions locked',rows)
    def select(self):
        with variant_scope():super().select()
    def score(self):
        with variant_scope():super().score()
        scores=read(self.out/'scores.json');choices=read(self.out/'choices.json');sm={(r['recording'],r['variant'],r['frontend'],r['arm'],r['index']):r for r in scores};dm={(r['recording'],r['variant'],r['frontend'],r['index']):r for r in choices};contrast={}
        for name,c in self.cfg['records'].items():
            n=read(self.out/'input-freeze.json')['anchors'][name];pop=read(ROOT/c['qualification']/'population.json');common=set(pop['common_reference']);trans=set(pop['translation_ge1cm_both']);groups={'all_declared':set(range(n)),'common_reference':common,'paired_rotation_ge2deg':set(pop['paired_rotation_ge2deg']),'translation_ge1cm_both':trans,**{'third'+str(j+1):{i for i in common if i*3//n==j} for j in range(3)}};contrast[name]={}
            for front in ['sift','learned']:
                contrast[name][front]={}
                for group,ids in groups.items():
                    result={}
                    for policy in ['rgb','thermal','rgb_first','thermal_first']:
                        def pick(v,i):
                            a=dm[name,v,front,i]['policies'][policy]['sensor'];return sm[name,v,front,a,i] if a else None
                        pairs=[(pick('count',i),pick('bottom',i),i) for i in ids];r={'anchors':len(ids),'crop_new_outputs':sum(a is None and b is not None for a,b,i in pairs),'crop_lost_outputs':sum(a is not None and b is None for a,b,i in pairs)}
                        for field,eligible in [('R1',common),('R5',common),('joint',trans)]:
                            bits=[(bool(a and a[field]),bool(b and b[field])) for a,b,i in pairs if i in eligible];r[field]={'eligible':len(bits),'count_correct':sum(a for a,b in bits),'crop_correct':sum(b for a,b in bits),'crop_gained':sum(not a and b for a,b in bits),'crop_lost':sum(a and not b for a,b in bits)}
                        result[policy]=r
                    contrast[name][front][group]=result
        write(self.out/'crop-count-contrast.json',contrast);write(self.out/'contrast-lock.json',{'crop_count_contrast_sha256':sha(self.out/'crop-count-contrast.json'),'scope':'Paired count→crop on identical original cohorts; no variant selection'})
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','run','select','score']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Study(a.config),a.stage)()
