"""Independent integer joins and scalar tied-rank validation of timing metadata."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from Scripts.Setup.validate_ms2_stereo_depth_proxy import rank_correlation,quantile


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate(root,run,skip_historical=False):
    root,run=Path(root),Path(run);r=root/'results';report=json.loads((run/'report.json').read_text());checks=0
    defs={'night-opening':(r/'ms2-timestamp-semantics/source-protocol-input-manifest.json',r/'ms2-stereo-depth-proxy/run-001/report.json'),
          'day-opening':(r/'ms2-day-replication/source-manifest.json',r/'ms2-day-replication/proxy-run-001/report.json'),
          'day-middle':(r/'ms2-day-later-windows/middle/source-manifest.json',r/'ms2-day-later-windows/middle/proxy-run-001/report.json'),
          'day-late':(r/'ms2-day-later-windows/late/source-manifest.json',r/'ms2-day-later-windows/late/proxy-run-001/report.json')}
    def check(x):
        nonlocal checks
        assert x;checks+=1
    def close(x,y):
        if x is None or y is None:check(x is y)
        else:check(math.isclose(x,y,rel_tol=1e-10,abs_tol=1e-12))
    check(report['status']=='PASS' and report['frozen_inputs_unchanged'] and report['no_gates_changed'])
    check(rank_correlation([1,1,3,4],[4,4,2,1])==-1.)
    check(rank_correlation([1,1],[2,3]) is None)
    endpoint_keys={'l2_median':'l2_median','abs_v_median':'abs_v_median','covariance_trace_median':'covariance_trace_median','uncertainty_error_rho':'uncertainty_error_spearman'}
    seen=set()
    for window,(mp,rp) in defs.items():
        manifest=json.loads(mp.read_text());original=json.loads(rp.read_text());by_stem={x['stem']:x for x in manifest['records']}
        original={(x['modality'],x['variant'],x['stem']):x for x in original['records']}
        sync=(root/manifest['records'][0]['cameras']['rgb']['left']['path']).parents[2]
        lidar={s:[int(x) for x in (sync/'lidar'/(s+'_timestamp.txt')).read_text().split()] for s in ('left','right')}
        for row in [x for x in report['rows'] if x['window']==window]:
            m,v,s,stem=[row[k] for k in ('modality','variant','support','stem')];key=window,m,v,s,stem
            check(key not in seen);seen.add(key);camera=by_stem[stem]['cameras'][m];i=int(stem)
            left,right=[camera[k]['recorded_time_ns'] for k in ('left','right')]
            check(row['camera_left_ns']==left and row['camera_right_ns']==right)
            check(row['lidar_left_ns']==lidar['left'][i] and row['lidar_right_ns']==lidar['right'][i])
            for k,delta in [('stereo_abs_ns',right-left),('lidar_left_abs_ns',left-lidar['left'][i]),('lidar_right_abs_ns',left-lidar['right'][i])]:check(row['predictors'][k]==abs(delta))
            check(row['stereo_signed_ns']==right-left)
            cross=by_stem[stem]['cameras']['thr']['left']['recorded_time_ns']-by_stem[stem]['cameras']['rgb']['left']['recorded_time_ns']
            check(row['cross_left_signed_ns']==cross)
            prior=original[m,v,stem];check(row['inherited_joint_gate']==prior['timing']['inherited_joint_gate'])
            check(row['count']==prior[s]['count'])
            for k,pk in endpoint_keys.items():close(row['endpoints'][k],prior[s].get('normalized_plane',{}).get(pk))
    check(len(seen)==960)
    corr_seen=set()
    for c in report['correlations']:
        window,m,v,s,p,e=[c[k] for k in ('window','modality','variant','support','predictor','endpoint')];key=window,m,v,s,p,e
        check(key not in corr_seen);corr_seen.add(key)
        rows=[r for r in report['rows'] if (r['window'],r['modality'],r['variant'],r['support'])==(window,m,v,s)]
        check(len(rows)==20)
        valid=[r for r in rows if r['endpoints'][e] is not None]
        expected=rank_correlation([r['predictors'][p] for r in valid],[r['endpoints'][e] for r in valid])
        close(c['rho'],expected);check(c['available_images']==len(valid) and c['missing_images']==20-len(valid))
        check(c['availability']==('available' if expected is not None else 'constant-or-insufficient'))
        check(c['primary']==(v=='depth' and s=='own_support' and p=='stereo_abs_ns' and e=='l2_median'))
    check(len(corr_seen)==576 and sum(c['primary'] for c in report['correlations'])==8)
    for summary in report['timing_summary']:
        rows=[x for x in report['rows'] if x['window']==summary['window'] and x['modality']==summary['modality'] and x['variant']=='depth' and x['support']=='own_support']
        check(len(rows)==20)
        for key,p in [('stereo_abs_ms_min_median_max','stereo_abs_ns'),('lidar_left_abs_ms_min_median_max','lidar_left_abs_ns'),('lidar_right_abs_ms_min_median_max','lidar_right_abs_ns')]:
            for a,q in zip(summary[key],[0,.5,1]):close(a,quantile([r['predictors'][p] for r in rows],q)/1e6)
    frozen=json.loads((run/'frozen-inputs.json').read_text())
    for p,h in frozen.items():check(sha(p)==h)
    historical=0
    if not skip_historical:
        for name in ('exp-043','exp-044-complete','exp-046','exp-047','exp-048','exp-049'):
            old=json.loads((Path('/root/ore-ms-recovery')/name/'manifest.json').read_text())
            for p,h in old['member_sha256'].items():
                if p.startswith('ore-ms/results/'):check(sha(root.parent/p)==h);historical+=1
    return {'status':'PASS','independent_controls':checks,'endpoint_rows':len(seen),'correlations_checked':len(corr_seen),
            'frozen_inputs_checked':len(frozen),'historical_hash_checks':historical,'historical_skipped':skip_historical}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--skip-historical',action='store_true',help='Standalone input-only replay; explicitly omits prior payload preservation audit');a=p.parse_args()
    if a.output.exists():p.error('Refuse existing validation')
    result=validate(a.root.resolve(),a.run.resolve(),a.skip_historical);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
