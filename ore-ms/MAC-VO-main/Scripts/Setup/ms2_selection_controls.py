"""Exploratory fixed depth/location baselines and proportional-bin selective risk."""
import argparse
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo
import numpy as np
from Scripts.Setup import ms2_timing_metadata as timing
from Scripts.Setup import ms2_selective_risk as selective

EDGES=[0.,5.,10.,20.,40.,80.]
BASELINES=('far_depth','near_depth','central','peripheral')
COVERAGES=(1.,.75,.5,.25)


def grouped_risk(score,loss,groups,c):
    total=0.
    for g in range(54):
        mask=groups==g;n=int(mask.sum())
        if n:total+=n*selective.risk(score[mask],loss[mask],c)
    return total/len(loss)


def summary(values):
    valid=[v for v in values if v is not None]
    return {'available_images':len(valid),'min_median_max':np.quantile(valid,[0,.5,1]).tolist() if valid else None,
            'images_below_one':sum(v<1-1e-12 for v in valid)}


def run(root,output):
    root=Path(root).resolve();output=Path(output);output.mkdir(parents=True,exist_ok=False)
    started=datetime.now(ZoneInfo('America/New_York')).isoformat()
    frozen={};rows=[]
    for p in (Path(__file__),Path(timing.__file__),Path(selective.__file__)):
        frozen[str(p.resolve())]=timing.sha(p)
    prior=root/'results/ms2-selective-risk/run-001/report.json';frozen[str(prior)]=timing.sha(prior)
    for window,(mp,rp) in timing.sources(root).items():
        for p in (mp,rp):frozen[str(p)]=timing.sha(p)
        manifest=json.loads(mp.read_text());source=json.loads(rp.read_text());assert source['status']=='PASS'
        metadata={r['stem']:r for r in manifest['records']}
        primary={(r['modality'],r['stem']):r for r in source['records'] if r['variant']=='depth'}
        for rec in source['records']:
            m,stem,v=(rec[k] for k in ('modality','stem','variant'));h,w=metadata[stem]['cameras'][m]['left']['shape'][:2]
            path=rp.parent/rec['file'];assert timing.sha(path)==rec['sha256'];frozen[str(path)]=rec['sha256']
            with np.load(rp.parent/primary[m,stem]['file'],allow_pickle=False) as p:
                pm=p['common_support'];px=p['x'][pm];py=p['y'][pm];pd=p['depth_m'][pm]
            with np.load(path,allow_pickle=False) as a:
                for support in ('own_support','common_support'):
                    mask=np.ones(len(a['x']),dtype=bool) if support=='own_support' else a['common_support']
                    x,y=a['x'][mask],a['y'][mask];z=a['depth_m'][mask]
                    if support=='common_support':assert np.array_equal(x,px) and np.array_equal(y,py);z=pd
                    u=a['covariance_normalized'][mask].sum(axis=1);e=np.square(a['residual_normalized'][mask]).sum(axis=1)
                    assert len(e)==rec[support]['count'] and np.isfinite(z).all() and (z>0).all()
                    assert np.isfinite(u).all() and np.isfinite(e).all() and (u>=0).all() and float(e.mean())>0
                    assert ((x>=0)&(x<w)&(y>=0)&(y<h)).all()
                    radius=((x-(w-1)/2)/(w/2))**2+((y-(h-1)/2)/(h/2))**2
                    groups=(np.searchsorted(EDGES,z,side='right')-1)*9+(y.astype(np.int64)*3//h)*3+x.astype(np.int64)*3//w
                    counts=np.bincount(groups,minlength=54);assert len(counts)==54 and counts.sum()==len(e)
                    scores={'uncertainty':u,'oracle':e,'far_depth':-z,'near_depth':z,'central':radius,'peripheral':-radius}
                    curves=[]
                    for c in COVERAGES:
                        risks={k:selective.risk(s,e,c) for k,s in scores.items()};risks['random']=float(e.mean())
                        risks['stratified_uncertainty']=grouped_risk(u,e,groups,c)
                        risks['stratified_oracle']=grouped_risk(e,e,groups,c)
                        comparisons={b:risks['uncertainty']/risks[b] if risks[b]>0 else None for b in BASELINES}
                        curves.append({'coverage':c,'retained_mass':len(e)*c,'risks':risks,'uncertainty_to_baseline':comparisons,
                                       'stratified_ratio':risks['stratified_uncertainty']/risks['random']})
                    rows.append({'window':window,'modality':m,'variant':v,'stem':stem,'support':support,'count':len(e),
                                 'height':h,'width':w,'file':str(path),'group_depth_source':'depth' if support=='common_support' else v,
                                 'group_counts':counts.tolist(),'sparse_pixels_lt30':int(counts[(counts>0)&(counts<30)].sum()),'curve':curves})
    aggregate=[]
    for window in timing.sources(root):
        for m in ('rgb','thr'):
            for v in ('depth','depth_multi','depth_filtered'):
                for support in ('own_support','common_support'):
                    g=[r for r in rows if (r['window'],r['modality'],r['variant'],r['support'])==(window,m,v,support)];assert len(g)==20
                    for j,c in enumerate(COVERAGES):
                        aggregate.append({'window':window,'modality':m,'variant':v,'support':support,'coverage':c,
                                          'primary':v=='depth' and support=='own_support' and c==.5,
                                          'stratified_ratio':summary([r['curve'][j]['stratified_ratio'] for r in g]),
                                          'uncertainty_to_baseline':{b:summary([r['curve'][j]['uncertainty_to_baseline'][b] for r in g]) for b in BASELINES}})
    assert len(rows)==960 and len(aggregate)==192 and all(timing.sha(p)==h for p,h in frozen.items())
    result={'status':'PASS','started_at':started,'finished_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),
            'scope':'provided-reference selective risk;exploratory;no fitting or physical calibration','new_model_inference':False,
            'depth_edges_lower_inclusive':EDGES,'grid_shape':[3,3],'sparse_bins_excluded':False,'rows':rows,'aggregate':aggregate}
    (output/'frozen-inputs.json').write_text(json.dumps(frozen,indent=2)+'\n')
    (output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print('PASS960 cases/3840 controlled curve points',flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.output)
