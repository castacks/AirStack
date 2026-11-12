"""Check full author-script serialized output against saved metric audit."""
from pathlib import Path
import json
import re
import numpy as np
from thermal_sfm_reproduction import OUT,verify


def main():
    verify();summary=json.loads((OUT/'summary.json').read_text());checks=0;reports={}
    keys=['abs_rel','sq_rel','rmse','rmse_log','log10','a1','a2','a3']
    for scene,report in summary['reports'].items():
        text=(OUT/(scene+'-author-depth-eval.log')).read_text()
        line=next(l for l in text.splitlines() if l.strip().startswith('&'))
        numbers=list(map(float,re.findall(r'&\s*([+-]?[0-9]+\.[0-9]+)',line)))
        assert len(numbers)==8
        for key,value in zip(keys,numbers):
            assert abs(value-report['depth'][key])<=.000500001
            checks+=1
        ratio=np.loadtxt(OUT/scene/'author-depth-ratios.txt')
        own=json.loads((OUT/scene/'depth-metrics-audit.json').read_text())['rows']
        np.testing.assert_allclose(ratio,[r['ratio'] for r in own],rtol=0,atol=.000050001)
        checks+=len(ratio)
        text=(OUT/(scene+'-pose.log')).read_text()
        for label,kind in [('mean','source_mean'),('std','source_std')]:
            line=next(l for l in text.splitlines() if l.strip().startswith(label+' ') or l.strip().startswith(label+'\t'))
            values=list(map(float,re.findall(r'[-+]?[0-9]+\.[0-9]+',line)))
            assert len(values)==2
            np.testing.assert_allclose(values,report['pose'][kind],rtol=0,atol=.000050001)
            checks+=2
        reports[scene]={'full_author_depth_eval_rounded_metrics_match':True,
                       'author_depth_ratios_match':True,'original_pose_log_mean_std_match':True}
    actual={'status':'PASS','checks':checks,'reports':reports}
    path=OUT/'author-control-audit.json'
    if path.exists():assert json.loads(path.read_text())==actual
    else:path.write_text(json.dumps(actual,indent=2)+'\n')
    print('PASS full-author-script controls',checks)


if __name__=='__main__':main()
