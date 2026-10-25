"""Descriptive recorded-time associations; never estimates exposure offsets or fits clocks."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.stats import rankdata


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def sources(root):
    r=Path(root)/'results'
    return {'night-opening':(r/'ms2-timestamp-semantics/source-protocol-input-manifest.json',r/'ms2-stereo-depth-proxy/run-001/report.json'),
            'day-opening':(r/'ms2-day-replication/source-manifest.json',r/'ms2-day-replication/proxy-run-001/report.json'),
            'day-middle':(r/'ms2-day-later-windows/middle/source-manifest.json',r/'ms2-day-later-windows/middle/proxy-run-001/report.json'),
            'day-late':(r/'ms2-day-later-windows/late/source-manifest.json',r/'ms2-day-later-windows/late/proxy-run-001/report.json')}


def rho(x,y):
    if len(x)<2 or np.ptp(x)==0 or np.ptp(y)==0:return None
    return float(np.corrcoef(rankdata(x),rankdata(y))[0,1])


def run(root,output):
    root,output=Path(root).resolve(),Path(output);output.mkdir(parents=True,exist_ok=False)
    frozen={str(Path(__file__).resolve()):sha(__file__)};rows=[];summaries=[]
    for window,(manifest_path,report_path) in sources(root).items():
        for p in [manifest_path,report_path]:frozen[str(p)]=sha(p)
        manifest=json.loads(manifest_path.read_text());report=json.loads(report_path.read_text());assert report['status']=='PASS'
        sync=(root/manifest['records'][0]['cameras']['rgb']['left']['path']).parents[2]
        times={}
        for label,p in {**{m+'_'+s:sync/m/('img_'+s+'_timestamp.txt') for m in ('rgb','thr') for s in ('left','right')},
                         **{'lidar_'+s:sync/'lidar'/(s+'_timestamp.txt') for s in ('left','right')}}.items():
            a=[int(x) for x in p.read_text().split()];assert all(y>x for x,y in zip(a,a[1:]));times[label]=a;frozen[str(p)]=sha(p)
        for rec in report['records']:
            stem,m,variant=rec['stem'],rec['modality'],rec['variant'];i=int(stem)
            source=next(r for r in manifest['records'] if r['stem']==stem)
            camera=source['cameras'][m];left=camera['left']['recorded_time_ns'];right=camera['right']['recorded_time_ns']
            assert times[m+'_left'][i]==left and times[m+'_right'][i]==right
            predictors={'stereo_abs_ns':abs(right-left),'lidar_left_abs_ns':abs(left-times['lidar_left'][i]),'lidar_right_abs_ns':abs(left-times['lidar_right'][i])}
            for support in ('own_support','common_support'):
                s=rec[support];a=s.get('normalized_plane',{})
                rows.append({'window':window,'modality':m,'variant':variant,'support':support,'stem':stem,
                             'camera_left_ns':left,'camera_right_ns':right,'lidar_left_ns':times['lidar_left'][i],
                             'lidar_right_ns':times['lidar_right'][i],'stereo_signed_ns':right-left,
                             'cross_left_signed_ns':times['thr_left'][i]-times['rgb_left'][i],
                             'inherited_joint_gate':rec['timing']['inherited_joint_gate'],'count':s['count'],
                             'predictors':predictors,'endpoints':{'l2_median':a.get('l2_median'),'abs_v_median':a.get('abs_v_median'),
                                                               'covariance_trace_median':a.get('covariance_trace_median'),
                                                               'uncertainty_error_rho':a.get('uncertainty_error_spearman')}})
        for m in ('rgb','thr'):
            records=[r for r in rows if r['window']==window and r['modality']==m and r['variant']=='depth' and r['support']=='own_support']
            assert len(records)==20
            summaries.append({'window':window,'modality':m,'images':20,
                              'stereo_abs_ms_min_median_max':[float(v)/1e6 for v in np.quantile([r['predictors']['stereo_abs_ns'] for r in records],[0,.5,1])],
                              'lidar_left_abs_ms_min_median_max':[float(v)/1e6 for v in np.quantile([r['predictors']['lidar_left_abs_ns'] for r in records],[0,.5,1])],
                              'lidar_right_abs_ms_min_median_max':[float(v)/1e6 for v in np.quantile([r['predictors']['lidar_right_abs_ns'] for r in records],[0,.5,1])]})
    correlations=[]
    for window in sources(root):
        for m in ('rgb','thr'):
            for variant in ('depth','depth_multi','depth_filtered'):
                for support in ('own_support','common_support'):
                    group=[r for r in rows if (r['window'],r['modality'],r['variant'],r['support'])==(window,m,variant,support)];assert len(group)==20
                    for predictor in ('stereo_abs_ns','lidar_left_abs_ns','lidar_right_abs_ns'):
                        for endpoint in ('l2_median','abs_v_median','covariance_trace_median','uncertainty_error_rho'):
                            valid=[r for r in group if r['endpoints'][endpoint] is not None]
                            x=[r['predictors'][predictor] for r in valid];y=[r['endpoints'][endpoint] for r in valid];coefficient=rho(x,y)
                            correlations.append({'window':window,'modality':m,'variant':variant,'support':support,
                                                 'predictor':predictor,'endpoint':endpoint,'available_images':len(valid),'missing_images':20-len(valid),
                                                 'rho':coefficient,'availability':'available' if coefficient is not None else 'constant-or-insufficient',
                                                 'primary':variant=='depth' and support=='own_support' and predictor=='stereo_abs_ns' and endpoint=='l2_median'})
    assert len(rows)==960 and len(correlations)==576 and sum(c['primary'] for c in correlations)==8
    assert all(sha(p)==h for p,h in frozen.items())
    result={'status':'PASS','scope':'recorded metadata association only;no exposure/synchronization/causal verdict','new_model_inference':False,
            'no_gates_changed':True,'frozen_inputs_unchanged':True,'frozen_input_count':len(frozen),'rows':rows,'timing_summary':summaries,'correlations':correlations}
    (output/'frozen-inputs.json').write_text(json.dumps(frozen,indent=2)+'\n')
    (output/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('PASS8 primary and568 secondary/sensitivity descriptive correlations; no exposure inference.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.root,a.output)
