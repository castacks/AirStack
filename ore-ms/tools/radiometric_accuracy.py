"""Frozen native accuracy comparison, separated from the completed parity gate."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import run_radiometric_native as native

ROOT, OUT = native.ROOT, native.OUT
PLAN = ROOT/'experiments/radiometric-native-accuracy-freeze-2026-11-12.json'
ARMS = ('original', 'passthrough', 'refined', 'rgb')
CASES = ('day', 'night', 'rain')
LOADER = ROOT/'results/pose-covariance/cohort-preflight/snapshot/ore-ms/results/provided-reference-full-record/run-001/source-001/official-loader.py'


def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def write(p,j):
    with Path(p).open('x') as f:json.dump(j,f,indent=2,allow_nan=False)


def verify():
    plan=json.loads(PLAN.read_text())
    for n,d in plan['pins'].items():assert sha(ROOT/n)==d,n
    for c in CASES:assert json.loads((OUT/c/'parity.json').read_text())['status']=='PASS_ALL_EXPORTED_NPZ_BITWISE'


def freeze():
    native.verify()
    pins={}
    for c in CASES:
        p=OUT/c/'parity.json';assert json.loads(p.read_text())['status']=='PASS_ALL_EXPORTED_NPZ_BITWISE';pins[str(p.relative_to(ROOT))]=sha(p)
        meta=ROOT/'results/unweighted-longer/run-001/inputs'/c/'metadata.json'
        j=json.loads(meta.read_text())
        for p in [meta, ROOT/j['calibration'],*[ROOT/n for n in j['reference_rgb_poses']]]:
            pins[str(p.relative_to(ROOT))]=sha(p)
    for p in [LOADER,Path(__file__),ROOT/'tools/radiometric_native_hook.py',ROOT/'tools/radiometric_tracking.py',ROOT/'tools/pose_covariance_accuracy.py',ROOT/'tools/pose_covariance_replay.py',ROOT/'tools/score_pose_covariance_accuracy.py']:
        pins[str(p.relative_to(ROOT))]=sha(p)
    pins.update(json.loads((ROOT/'experiments/radiometric-native-parity-freeze-2026-11-12.json').read_text())['pins'])
    write(PLAN,{'population':'all 99 adjacent intervals of first100 groups, each exposed day/night/rain; no window search',
        'arms':ARMS,'covariance':'original control; not calibrated for refined endpoints','rejection':'retain original endpoint',
        'gate':'each record: lower translation mean AND RMSE than thermal, rotation mean AND RMSE no worse; no loss of eligible interval IDs; common nonempty',
        'rgb_claim':'each record: all four mean/RMSE strictly lower than RGB with no eligible interval ID loss',
        'failure':'CLOSE_THIS_CONFIGURATION_NO_RETUNING','runtime':'sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09','pins':pins})


def capture(c,arm):
    verify();assert arm in ('refined','rgb');native.capture(c,arm);verify()


def lock():
    verify();pins={}
    for c in CASES:
        for arm in ARMS:
            base=OUT/c/arm/('rgb' if arm=='rgb' else 'thermal')
            assert json.loads((base/'report.json').read_text())['status']=='PASS_EXECUTION_ONLY'
            for p in base.rglob('*'):
                if p.is_file():pins[str(p.relative_to(OUT))]=sha(p)
    write(OUT/'global-prediction-lock.json',{'freeze_sha256':sha(PLAN),'pins':pins,'reference_values_used':False})


def score():
    verify()
    from pose_covariance_accuracy import transforms,project
    from pose_covariance_replay import pose_xyzw
    from score_pose_covariance_accuracy import reference_pose,error,statistics
    lock=json.loads((OUT/'global-prediction-lock.json').read_text());assert lock['freeze_sha256']==sha(PLAN)
    for n,d in lock['pins'].items():assert sha(OUT/n)==d,n
    records={}
    for c in CASES:
        base=ROOT/'results/unweighted-longer/run-001/inputs'/c
        meta=json.loads((base/'metadata.json').read_text());source=json.loads((base/'source-manifest.json').read_text())
        frames=transforms(json.loads((ROOT/meta['calibration']).read_text()),LOADER)
        refs=[reference_pose(ROOT/n)[0] for n in meta['reference_rgb_poses']]
        events={}
        for arm in ARMS:
            p=OUT/c/arm/('rgb' if arm=='rgb' else 'thermal')/'arbiter-events/events.jsonl'
            writes={}
            for line in p.read_text().splitlines():
                e=json.loads(line)
                if e['kind']=='motion_result_written' and e['first_write_for_submission']:
                    key=(e['from_idx'],e['frame_idx']);assert key not in writes,'ambiguous first write';writes[key]=e
            events[arm]=writes
        rows=[];eligible={a:set() for a in ARMS}
        for i in range(99):
            ref=None if refs[i] is None or refs[i+1] is None else np.linalg.solve(refs[i],refs[i+1])
            common=[int(source['records'][k]['cameras']['rgb']['left']['recorded_time_ns']) for k in [i,i+1]]
            entry={}
            for arm in ARMS:
                mode='rgb' if arm=='rgb' else 'thermal';sensor='rgb' if arm=='rgb' else 'thr'
                times=[int(source['records'][k]['cameras'][sensor]['left']['recorded_time_ns']) for k in [i,i+1]]
                e=events[arm].get((i,i+1));mapped=None
                if e is not None and e['derived_camera_delta_xyzw'] is not None:
                    assert [e['time0_ns'],e['time1_ns']]==times
                    value=project(pose_xyzw(e['derived_camera_delta_xyzw']),None,frames[mode],times,common)
                    if value['valid']:mapped=value['transform']
                value=error(mapped,ref) if mapped is not None and ref is not None else None
                if value is not None:eligible[arm].add(i)
                entry[arm]={'transform':mapped.tolist() if mapped is not None else None,'error':value}
            rows.append({'pair':i,'reference_delta':ref.tolist() if ref is not None else None,'arms':entry})
        shared=sorted(set.intersection(*eligible.values()))
        metrics={a:{k:statistics([rows[i]['arms'][a]['error'][k] for i in shared]) for k in ['translation_m','rotation_deg']} for a in ARMS}
        checks={'common_nonempty':bool(shared),'thermal_coverage_preserved':eligible['original']<=eligible['refined']}
        rgb={'common_nonempty':bool(shared),'rgb_coverage_preserved':eligible['rgb']<=eligible['refined']}
        for k in ['translation_m','rotation_deg']:
            for stat in ['mean','rmse']:
                a,b,d=[metrics[x][k][stat] for x in ['refined','original','rgb']]
                checks[k+'_'+stat]=bool(shared) and (a<b if k=='translation_m' else a<=b)
                rgb[k+'_'+stat]=bool(shared) and a<d
        result={'common_pair_ids':shared,'coverage_pair_ids':{a:sorted(v) for a,v in eligible.items()},'metrics':metrics,'checks':checks,'pass':all(checks.values()),'rgb_checks':rgb,'rgb_pass':all(rgb.values()),'rows':rows}
        write(OUT/c/'accuracy-scores.json',result);records[c]={k:v for k,v in result.items() if k!='rows'}
    write(OUT/'accuracy-summary.json',{'records':records,'decision':'ADVANCE_TO_SEPARATE_RECORDING_VALIDATION' if all(v['pass'] for v in records.values()) else 'CLOSE_THIS_CONFIGURATION_NO_RETUNING','scope':'exposed development data; supplied references and nominal geometry/timing; no flow truth/generalization claim'})


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['freeze','capture','lock','score']);p.add_argument('--condition',choices=CASES);p.add_argument('--arm',choices=['refined','rgb']);a=p.parse_args()
    if a.action=='capture':assert a.condition and a.arm;capture(a.condition,a.arm)
    else:{'freeze':freeze,'lock':lock,'score':score}[a.action]()
