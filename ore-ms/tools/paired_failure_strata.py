"""Descriptive correctness strata over restored caches; no classifier or inference."""
import argparse,json,math
from pathlib import Path
import numpy as np
from deployment_lighting import read,write,sha
ROOT=Path(__file__).resolve().parents[1]
FIELDS=['matches','inliers','inlier_fraction','gray_range_min','gray_std_min']
CELLS=['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']

def features(raw,inputs,images):
    meta={(r['arm'],r['index']):r for r in inputs};im={r['path']:r for r in images};rows=[]
    for r in raw:
        if r['frontend']!='learned':continue
        m=meta[r['arm'],r['index']];pair=[im[m[k]] for k in ['source','target']];n=r.get('matches',0);k=r.get('cheirality_inliers',0)
        assert 0<=k<=n and all(math.isfinite(v['metrics']['std']) for v in pair)
        rows.append({'arm':r['arm'],'index':r['index'],'status':r['status'],'qualified':r['qualified'],'anchor_ns':m['anchor_ns'],'source':m['source'],'target':m['target'],'times_ns':m['times_ns'],'matches':n,'inliers':k,'inlier_fraction':k/n if n else None,'gray_range_min':min(v['metrics']['p05_p95_range'] for v in pair),'gray_std_min':min(v['metrics']['std'] for v in pair)})
    return rows

def runs(ids):
    result=[]
    for i in sorted(ids):
        if result and i==result[-1][-1]+1:result[-1].append(i)
        else:result.append([i])
    return result

def describe(cases,feat,n):
    result={}
    for endpoint in ['R1','R5','joint']:
        result[endpoint]={}
        for cell in CELLS:
            selected=[r for r in cases if r['endpoint']==endpoint and r['cell']==cell];ids=[r['index'] for r in selected];d={'count':len(ids),'indices':ids,'both_supported':sum(r['both_supported'] for r in selected),'third_counts':[sum(i*3//n==j for i in ids) for j in range(3)],'runs':runs(ids),'features':{}}
            for a in ['rgb','thermal']:
                d['features'][a]={}
                for field in FIELDS:
                    values=[feat[a,i][field] for i in ids if feat[a,i][field] is not None];d['features'][a][field]={'n':len(values),'missing':len(ids)-len(values),'quartiles':np.percentile(values,[25,50,75]).tolist() if values else None}
            result[endpoint][cell]=d
    return result

class Study:
    def __init__(self,path):self.config=Path(path).resolve();self.cfg=read(self.config);self.out=ROOT/self.cfg['output']
    def verify(self):
        for p,h in read(self.out/'input-freeze.json')['pins'].items():assert sha(ROOT/p)==h,p
    def freeze(self):
        files=[self.config,Path(__file__),ROOT/'tools/audit_paired_failure_strata.py',ROOT/'tools/test_paired_failure_strata.py',ROOT/'tools/deployment_lighting.py',ROOT/self.cfg['protocol']];counts={}
        for name,c in self.cfg['records'].items():
            p,q,control=[ROOT/c[k] for k in ['matching','qualification','priority']];assert read(p/'audit.json')['status']==read(q/'audit.json')['status']==read(control/'audit.json')['status']=='pass';assert sha(p/'predictions.jsonl')==read(p/'prediction-lock.json')['predictions_sha256'];assert sha(p/'scores.json')==read(p/'completion.json')['scores_sha256'];assert sha(control/'summary.json')==read(control/'completion.json')['summary_sha256'];n=read(p/'input-freeze.json')['anchors'];counts[name]=n;inputs=read(p/'matcher-inputs.json');assert len(inputs)==len({(r['arm'],r['index']) for r in inputs})==2*n;images={r['path']:r for r in read(q/'images.json')}
            for m in inputs:
                for k,j in [('source',0),('target',1)]:assert m[k] in images and images[m[k]]['arm']==m['arm'] and images[m[k]]['time_ns']==m['times_ns'][j]
            files.extend(p/k for k in ['predictions.jsonl','prediction-lock.json','scores.json','completion.json','matcher-inputs.json','audit.json','input-freeze.json']);files.extend(q/k for k in ['images.json','population.json','audit.json']);files.extend(control/k for k in ['summary.json','completion.json','audit.json'])
        files.extend(ROOT/p for p in self.cfg['prior_nulls']);self.out.mkdir(exist_ok=False,parents=True);write(self.out/'input-freeze.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in files},'anchors':counts,'commit':'4408b7fb8','scope':'Exposed descriptive supplied-reference strata; no selector'})
    def extract(self):
        self.verify();allrows={}
        for name,c in self.cfg['records'].items():
            p,q=[ROOT/c[k] for k in ['matching','qualification']];raw=[json.loads(l) for l in (p/'predictions.jsonl').read_text().splitlines()];n=read(self.out/'input-freeze.json')['anchors'][name];assert len(raw)==4*n and len({(r['frontend'],r['arm'],r['index']) for r in raw})==4*n;rows=features(raw,read(p/'matcher-inputs.json'),read(q/'images.json'));assert len(rows)==2*n;allrows[name]=rows
        write(self.out/'features.json',allrows);write(self.out/'feature-lock.json',{'features_sha256':sha(self.out/'features.json'),'score_values_parsed_for_features':False});print('Feature rows locked',sum(len(r) for r in allrows.values()))
    def join(self):
        self.verify();assert sha(self.out/'features.json')==read(self.out/'feature-lock.json')['features_sha256'];features_by_record=read(self.out/'features.json');allcases={};summary={};samples={};parity={}
        for name,c in self.cfg['records'].items():
            p,q,control=[ROOT/c[k] for k in ['matching','qualification','priority']];feat={(r['arm'],r['index']):r for r in features_by_record[name]};n=read(self.out/'input-freeze.json')['anchors'][name];origin=min(r['anchor_ns'] for r in feat.values());pop=read(q/'population.json');scores={(r['arm'],r['index']):r for r in read(p/'scores.json') if r['frontend']=='learned'};cases=[]
            for endpoint,group in [('R1','paired_rotation_ge2deg'),('R5','paired_rotation_ge2deg'),('joint','translation_ge1cm_both')]:
                for i in pop[group]:
                    good=[scores[a,i][endpoint] for a in ['rgb','thermal']];cell=CELLS[0] if all(good) else CELLS[1] if good[0] else CELLS[2] if good[1] else CELLS[3];cases.append({'endpoint':endpoint,'index':i,'cell':cell,'both_supported':all(feat[a,i]['status']=='ok' for a in ['rgb','thermal']),'relative_seconds':(feat['rgb',i]['anchor_ns']-origin)/1e9,'third':i*3//n+1,'cameras':{a:{k:scores[a,i][k] for k in ['status','R_error','t_error','reference_rotation_deg']} for a in ['rgb','thermal']}})
            allcases[name]=cases;summary[name]=describe(cases,feat,n);samples[name]=[];prior=read(control/'summary.json')['frontends']['learned'];parity[name]={}
            for endpoint,g in [('R1','paired_rotation_ge2deg'),('R5','paired_rotation_ge2deg'),('joint','translation_ge1cm_both')]:
                s=prior[g];better=max(['rgb_first','thermal_first'],key=lambda policy:s[policy][endpoint]);cell='thermal_only_correct' if better=='rgb_first' else 'rgb_only_correct';ids=[r['index'] for r in cases if r['endpoint']==endpoint and r['cell']==cell and r['both_supported']];assert len(ids)==s['headroom'][endpoint]['oracle_increment_over_best_priority'];parity[name][endpoint]={'better_order':better,'recoverable_indices':ids,'count':len(ids)}
            for cell in CELLS:
                eligible=[r for r in cases if r['endpoint']=='R5' and r['cell']==cell and r['both_supported']];positions=sorted(set(np.rint(np.linspace(0,len(eligible)-1,min(3,len(eligible)))).astype(int))) if eligible else []
                samples[name].extend({'cell':cell,'index':eligible[j]['index']} for j in positions)
        write(self.out/'cases.json',allcases);write(self.out/'summary.json',summary);write(self.out/'ceiling-parity.json',parity);write(self.out/'samples.json',samples);write(self.out/'sample-lock.json',{'samples_sha256':sha(self.out/'samples.json'),'image_inspection_preceded_sample_lock':False});write(self.out/'completion.json',{name+'_sha256':sha(self.out/(name+'.json')) for name in ['cases','summary','ceiling-parity','samples']});print('Cases locked',sum(len(v) for v in allcases.values()))
    def render(self):
        from PIL import Image,ImageDraw,ImageFont
        self.verify();assert read(self.out/'audit.json')['status']=='pass';assert sha(self.out/'samples.json')==read(self.out/'sample-lock.json')['samples_sha256'];samples=read(self.out/'samples.json');feature=read(self.out/'features.json');cases=read(self.out/'cases.json');folder=self.out/'sheets';folder.mkdir(exist_ok=False);receipts=[];font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
        for name,c in self.cfg['records'].items():
            im={r['path']:r for r in read(ROOT/c['qualification']/'images.json')};meta={(r['arm'],r['index']):r for r in feature[name]};case={r['index']:r for r in cases[name] if r['endpoint']=='R5'}
            for cell in CELLS:
                selected=[r for r in samples[name] if r['cell']==cell]
                if not selected:continue
                canvas=Image.new('RGB',(1280,55+len(selected)*290),'white');draw=ImageDraw.Draw(canvas);draw.text((10,8),f'{name} — {cell} — fixed moving R≤5° sample; nominal reference errors',font=font,fill='black')
                for row,s in enumerate(selected):
                    i=s['index'];r=case[i];y=55+row*290;draw.text((8,y),f"anchor {i}, {r['relative_seconds']:.1f}s; RGB R={r['cameras']['rgb']['R_error']:.3f}°, thermal R={r['cameras']['thermal']['R_error']:.3f}°",font=font,fill='black')
                    for col,(a,key) in enumerate([('rgb','source'),('rgb','target'),('thermal','source'),('thermal','target')]):
                        m=meta[a,i];p=ROOT/m[key];h=sha(p);assert h==im[m[key]]['sha256'];array=np.load(p,allow_pickle=False);assert array.dtype==np.uint8;thumb=Image.fromarray(array).convert('RGB');thumb.thumbnail((315,240));x=col*320;canvas.paste(thumb,(x,y+48));draw.text((x+3,y+23),f"{a} {key} {Path(m[key]).name} @ {m['times_ns'][0 if key=='source' else 1]}",font=font,fill='black');receipts.append({'recording':name,'cell':cell,'index':i,'arm':a,'endpoint':key,'path':m[key],'time_ns':m['times_ns'][0 if key=='source' else 1],'sha256':h})
                canvas.save(folder/(name+'-'+cell+'.png'))
        write(self.out/'render-receipts.json',receipts);write(self.out/'render-lock.json',{'receipts_sha256':sha(self.out/'render-receipts.json'),'sheet_sha256':{p.name:sha(p) for p in folder.iterdir()}});print('Rendered source/target images',len(receipts))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['freeze','extract','join','render']);p.add_argument('config',type=Path);a=p.parse_args();getattr(Study(a.config),a.stage)()
