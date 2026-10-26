"""Separate geometry propagation from learned horizontal flow variance, fixed support."""
import argparse
from datetime import datetime
import json
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np

from ms2_cross_modal_support import sha, quartiles
from ms2_cross_modal_metric import risk_record


def choice(a,b):
    return np.where(a<b,1.,np.where(a>b,0.,.5))


def run(root, source, output):
    if output.exists():
        raise FileExistsError(output)
    base=root/'results/ms2-rain-replication'
    c={k:np.array(v['values'],float) for k,v in json.loads((base/'calibration.json').read_text()).items()}
    report=json.loads((source/'report.json').read_text())
    r=c['R_nir2thr']@c['R_nir2rgb'].T;t=(c['T_nir2thr']-r@c['T_nir2rgb']).reshape(3)/1000
    frozen={str(source/'report.json'):sha(source/'report.json'),str(base/'calibration.json'):sha(base/'calibration.json')}
    for row in report['rows']:
        p=source/f"metric-{row['stem']}.npz";frozen[str(p)]=sha(p)
    output.mkdir(parents=True)
    protocol={'primary':'equal horizontal angular variance propagated through each predicted depth: norm(q)^2 Zhat^4 / B^2','secondary':['equal native pixel variance','unpropagated learned horizontal angular variance'],'constant_value':'one, same across cameras, cancels in ranking','targets':'all original midpoint/RGB/thermal anchors','population':'unchanged both-valid candidate pairs','script_sha256':sha(__file__)}
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[]
    for row in report['rows']:
        stem=row['stem']
        with np.load(source/f'metric-{stem}.npz') as d:a={k:d[k] for k in d.files}
        mask=a['both_valid_mask'];xyz={'rgb':a['rgb_predicted_xyz'][mask],'thr':a['thr_predicted_xyz_in_rgb'][mask]@r.T+t}
        geom={};pixel={};raw={}
        for mod in ('rgb','thr'):
            x=xyz[mod];b=-c[f'T_{mod}R'][0,0]/1000;f=c[f'K_{mod}L'][0,0]
            geom[mod]=np.sum(x*x,axis=1)*x[:,2]**2/b**2
            pixel[mod]=geom[mod]/f**2
            raw[mod]=a[f'{mod}_metric_variance'][mask]/geom[mod]
        choices={'learned_propagated':a['choose_rgb_on_both_valid'],'geometry_angular':choice(geom['rgb'],geom['thr']),'geometry_pixel':choice(pixel['rgb'],pixel['thr']),'learned_unpropagated_angular':choice(raw['rgb'],raw['thr'])}
        saved={name:value for name,value in choices.items()}
        saved.update({f'{mod}_geometry_angular':g for mod,g in geom.items()})
        scores={}
        for anchor in report['protocol']['reference_anchors']:
            scores[anchor]={}
            for loss,power in [('mse',2),('mean_l2',1)]:
                er=a[f'{anchor}_rgb_error_m']**power;et=a[f'{anchor}_thr_error_m']**power
                scores[anchor][loss]={name:risk_record(er,et,sel) for name,sel in choices.items()}
                ratio=scores[anchor][loss]['learned_propagated']['uncertainty']/scores[anchor][loss]['geometry_angular']['uncertainty']
                scores[anchor][loss]['learned_over_geometry_angular']=ratio
        rows.append({'stem':stem,'pairs':int(mask.sum()),'rgb_choice_fractions':{k:float(v.mean()) for k,v in choices.items()},'learned_geometry_choice_disagreements':int(np.count_nonzero(choices['learned_propagated']!=choices['geometry_angular'])),'scores':scores})
        np.savez_compressed(output/f'control-{stem}.npz',**saved)
    assert all(sha(p)==v for p,v in frozen.items())
    result={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'protocol':protocol,'frozen_input_sha256':frozen,'rows':rows}
    (output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Geometry control PASS; same20 frames/population.')
    for anchor in report['protocol']['reference_anchors']:
        for loss in ('mse','mean_l2'):
            ratios=[r['scores'][anchor][loss]['learned_over_geometry_angular'] for r in rows]
            print(anchor,loss,'learned/geometry min,quartiles,max',quartiles(ratios),'below1',sum(v<1 for v in ratios))
    print('Decision disagreement pairs',sum(r['learned_geometry_choice_disagreements'] for r in rows))
    print('Geometry RGB fraction medians',quartiles([r['rgb_choice_fractions']['geometry_angular'] for r in rows]))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['root','source','output']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();run(a.root.resolve(),a.source,a.output)
