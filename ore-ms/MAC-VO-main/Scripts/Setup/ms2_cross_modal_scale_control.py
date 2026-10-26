"""Control per-image camera uncertainty scales, fixed candidate support and losses."""
import argparse
import json
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo

import numpy as np
from ms2_cross_modal_support import sha, quartiles
from ms2_cross_modal_metric import risk_record


def run(base,output):
    if output.exists():raise FileExistsError(output)
    metric=base/'metric-run-001';geom=base/'geometry-control-run-001'
    mr=json.loads((metric/'report.json').read_text());frozen={str(metric/'report.json'):sha(metric/'report.json')}
    for row in mr['rows']:
        for folder,name in [(metric,'metric'),(geom,'control')]:
            p=folder/f"{name}-{row['stem']}.npz";frozen[str(p)]=sha(p)
    protocol={'baseline':'per-image/per-camera median learned horizontal angular variance times fixed angular geometry trace','population':'unchanged both-valid candidates','labels_used_for_scale':False,'local_variance_variation_removed':True,'script_sha256':sha(__file__)}
    output.mkdir(parents=True);(output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');rows=[]
    for row in mr['rows']:
        stem=row['stem']
        with np.load(metric/f'metric-{stem}.npz') as f:m={k:f[k] for k in f.files}
        with np.load(geom/f'control-{stem}.npz') as f:g={k:f[k] for k in f.files}
        scales={};scores={}
        for mod in ('rgb','thr'):
            angular=m[f'{mod}_metric_variance'][m['both_valid_mask']]/g[f'{mod}_geometry_angular']
            scales[mod]=float(np.median(angular));scores[mod]=g[f'{mod}_geometry_angular']*scales[mod]
        sel=np.where(scores['rgb']<scores['thr'],1.,np.where(scores['rgb']>scores['thr'],0.,.5));learned=m['choose_rgb_on_both_valid']
        result={'stem':stem,'median_angular_variances':scales,'constant_scale_choose_rgb_fraction':float(sel.mean()),'scores':{}}
        for anchor in ('midpoint','rgb','thr'):
            result['scores'][anchor]={}
            for loss,power in [('mse',2),('mean_l2',1)]:
                a=m[f'{anchor}_rgb_error_m']**power;b=m[f'{anchor}_thr_error_m']**power
                r=risk_record(a,b,sel);l=risk_record(a,b,learned)
                result['scores'][anchor][loss]={'constant_scale':r,'learned':l,'learned_over_constant_scale':l['uncertainty']/r['uncertainty']}
        np.savez_compressed(output/f'scale-{stem}.npz',choose_rgb=sel)
        rows.append(result)
    assert all(sha(p)==v for p,v in frozen.items())
    report={'status':'PASS','created_at':datetime.now(ZoneInfo('America/New_York')).isoformat(),'protocol':protocol,'frozen_input_sha256':frozen,'rows':rows}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for anchor in ('midpoint','rgb','thr'):
        for loss in ('mse','mean_l2'):
            vals=[r['scores'][anchor][loss]['learned_over_constant_scale'] for r in rows]
            print(anchor,loss,'learned/constant camera scale',quartiles(vals),'below1',sum(v<1 for v in vals))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.base,a.output)
