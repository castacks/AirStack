"""Independent full descriptor/stratum/sample audit, no production imports."""
import json,math,hashlib,itertools
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def quantiles(values):
    if not values:return None
    v=sorted(values);result=[]
    for q in [.25,.5,.75]:
        x=(len(v)-1)*q;lo=int(math.floor(x));hi=int(math.ceil(x));result.append(v[lo]+(v[hi]-v[lo])*(x-lo))
    return result

def main(config):
    cfg=read(config);out=ROOT/cfg['output'];count=0
    def check(value,label):
        nonlocal count
        assert value,label
        count+=1
    freeze=read(out/'input-freeze.json')
    for p,h in freeze['pins'].items():check(sha(ROOT/p)==h,'frozen '+p)
    for receipt,key,file in [('feature-lock.json','features_sha256','features.json'),('sample-lock.json','samples_sha256','samples.json')]:check(read(out/receipt)[key]==sha(out/file),'locked '+file)
    check(read(out/'feature-lock.json')['score_values_parsed_for_features'] is False,'feature labels absent');check(read(out/'sample-lock.json')['image_inspection_preceded_sample_lock'] is False,'samples before display')
    for key,h in read(out/'completion.json').items():check(sha(out/(key.removesuffix('_sha256')+'.json'))==h,'completion '+key)
    features=read(out/'features.json');cases=read(out/'cases.json');summary=read(out/'summary.json');parity=read(out/'ceiling-parity.json');samples=read(out/'samples.json');all_sample_images=[]
    for name,c in cfg['records'].items():
        p,q,control=[ROOT/c[k] for k in ['matching','qualification','priority']];n=freeze['anchors'][name];raw={(r['arm'],r['index']):r for r in [json.loads(l) for l in (p/'predictions.jsonl').read_text().splitlines()] if r['frontend']=='learned'};meta={(r['arm'],r['index']):r for r in read(p/'matcher-inputs.json')};images={r['path']:r for r in read(q/'images.json')};pop=read(q/'population.json');scores={(r['arm'],r['index']):r for r in read(p/'scores.json') if r['frontend']=='learned'};feat={(r['arm'],r['index']):r for r in features[name]};check(len(feat)==len(features[name])==len(raw)==2*n,'full feature population');origin=min(r['anchor_ns'] for r in meta.values())
        for key,r in raw.items():
            a,i=key;m=meta[key];im=[images[m[k]] for k in ['source','target']];matches=r.get('matches',0);inliers=r.get('cheirality_inliers',0);expected={k:m[k] for k in ['arm','index','anchor_ns','source','target','times_ns']};expected.update(status=r['status'],qualified=r['qualified'],matches=matches,inliers=inliers,inlier_fraction=inliers/matches if matches else None,gray_range_min=min(v['metrics']['p05_p95_range'] for v in im),gray_std_min=min(v['metrics']['std'] for v in im));check(feat[key]==expected,'own input descriptors/counts and null ratio')
        case_map={(r['endpoint'],r['index']):r for r in cases[name]};expected_total=2*len(pop['paired_rotation_ge2deg'])+len(pop['translation_ge1cm_both']);check(len(case_map)==len(cases[name])==expected_total,'complete case population');prior=read(control/'summary.json')['frontends']['learned'];expected_samples=[]
        for endpoint,group in [('R1','paired_rotation_ge2deg'),('R5','paired_rotation_ge2deg'),('joint','translation_ge1cm_both')]:
            partition={k:[] for k in ['both_correct','rgb_only_correct','thermal_only_correct','neither_correct']}
            for i in pop[group]:
                rows=[scores[a,i] for a in ['rgb','thermal']];good=[]
                for r in rows:
                    good.append(r['R_error'] is not None and r['R_error']<=(1 if endpoint=='R1' else 5) and (endpoint!='joint' or r['t_error'] is not None and r['t_error']<=15))
                cell='both_correct' if all(good) else 'rgb_only_correct' if good[0] else 'thermal_only_correct' if good[1] else 'neither_correct';r=case_map[endpoint,i];both=all(s['status']=='ok' for s in rows);expected={'endpoint':endpoint,'index':i,'cell':cell,'both_supported':both,'relative_seconds':(meta['rgb',i]['anchor_ns']-origin)/1e9,'third':(3*i)//n+1,'cameras':{a:{k:scores[a,i][k] for k in ['status','R_error','t_error','reference_rotation_deg']} for a in ['rgb','thermal']}};check(r==expected,'numeric correctness own errors and time');partition[cell].append(i)
            for cell,ids in partition.items():
                d=summary[name][endpoint][cell];bothids=[i for i in ids if all(raw[a,i]['status']=='ok' for a in ['rgb','thermal'])];runs=[[v for _,v in g] for _,g in itertools.groupby(enumerate(ids),lambda x:x[1]-x[0])];check(d['indices']==ids and d['count']==len(ids) and d['both_supported']==len(bothids),'stratum/status counts');check(d['third_counts']==[sum(i*3//n==j for i in ids) for j in range(3)] and d['runs']==runs,'fixed thirds and contiguous runs')
                for a in ['rgb','thermal']:
                    for field in ['matches','inliers','inlier_fraction','gray_range_min','gray_std_min']:
                        values=[feat[a,i][field] for i in ids if feat[a,i][field] is not None];r=d['features'][a][field];quartiles=quantiles(values);check(r['n']==len(values) and r['missing']==len(ids)-len(values),'feature missingness');check(r['quartiles'] is None if quartiles is None else all(abs(x-y)<1e-10 for x,y in zip(r['quartiles'],quartiles)) and len(r['quartiles'])==3,'independent linear quantiles')
                if endpoint=='R5' and bothids:
                    k=min(3,len(bothids));positions=[0] if k==1 else sorted(set(round(j*(len(bothids)-1)/(k-1)) for j in range(k)));expected_samples.extend({'cell':cell,'index':bothids[pos]} for pos in positions)
            s=prior[group];better=max(['rgb_first','thermal_first'],key=lambda p:s[p][endpoint]);candidate='thermal_only_correct' if better=='rgb_first' else 'rgb_only_correct';ids=[i for i in partition[candidate] if all(raw[a,i]['status']=='ok' for a in ['rgb','thermal'])];check(parity[name][endpoint]=={'better_order':better,'recoverable_indices':ids,'count':len(ids)} and len(ids)==s['headroom'][endpoint]['oracle_increment_over_best_priority'],'exact prior ceiling parity')
        check(samples[name]==expected_samples,'predeclared deterministic sample IDs')
        for s in expected_samples:
            for a,k,j in [('rgb','source',0),('rgb','target',1),('thermal','source',0),('thermal','target',1)]:
                m=meta[a,s['index']];all_sample_images.append({'recording':name,'cell':s['cell'],'index':s['index'],'arm':a,'endpoint':k,'path':m[k],'time_ns':m['times_ns'][j],'sha256':images[m[k]]['sha256']})
    if (out/'render-lock.json').exists():
        r=read(out/'render-lock.json');receipts=read(out/'render-receipts.json');check(r['receipts_sha256']==sha(out/'render-receipts.json') and receipts==all_sample_images,'render bindings');
        for row in receipts:check(sha(ROOT/row['path'])==row['sha256'],'render image hash')
        for name,h in r['sheet_sha256'].items():check(sha(out/'sheets'/name)==h,'render sheet hash')
    result={'status':'pass','checks':count,'feature_rows':sum(len(v) for v in features.values()),'case_rows':sum(len(v) for v in cases.values()),'sample_anchors':sum(len(v) for v in samples.values()),'render_verified':(out/'render-lock.json').exists(),'auditor_sha256':sha(__file__),'scope':'Independent descriptors, numeric strata/cohorts, thirds/runs/quantiles, ceiling parity, sample IDs and optional render hashes; no causal labels or selection validation'};(out/'audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
