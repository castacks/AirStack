"""EXP137: selected exposed0..100 prefixes, identical producer and fixed DROID."""
import argparse
from copy import deepcopy
from pathlib import Path

from droid_ms2_development import ROOT,read,write,sha,SETTINGS
from droid_longer_worker import allocation_settings

SELECTION=ROOT/'experiments/droid-longer-prefix-selection-2026-11-04.json'
PLAN=ROOT/'experiments/droid-longer-prefix-2026-11-04.json'
OUT=ROOT/'results/droid-longer-prefix/run-001'


def prepare():
    from orb_ms2_development import prepare as canonical_prepare
    selection=read(SELECTION);recovery=read(ROOT/'results/droid-longer-prefix/source-001/recovery.json')
    if recovery['selection_sha256']!=sha(SELECTION):raise ValueError('selection changed during recovery')
    for name,digest in selection['metadata_sha256'].items():
        if sha(ROOT/name)!=digest:raise ValueError('sampling metadata changed')
    old=read(ROOT/'experiments/droid-ms2-development-2026-11-04-v2.json')
    recovered={r['path']:r for r in recovery['restored_camera_pngs']}
    OUT.mkdir(parents=True,exist_ok=False);cases=[];overlap=[]
    for case in selection['cases']:
        source=read(ROOT/case['source0_manifest']);prior=source['records'];rows=[]
        for index in range(101):
            row={'stem':f'{index:06d}','cameras':deepcopy(prior[0]['cameras'])}
            for sensor in ['rgb','thr']:
                for side in ['left','right']:
                    rec=row['cameras'][sensor][side]
                    if index==0:rec.update(prior[0]['cameras'][sensor][side])
                    else:
                        path=f"results/ms2-broader-range/run-001/source/{case['sequence']}/{sensor}/img_{side}/{index:06d}.png"
                        rec.update(path=path,sha256=recovered[path]['sha256'],recorded_time_ns=case['early_recorded_times_ns'][sensor+'_'+side][index-1])
                    if sha(ROOT/rec['path'])!=rec['sha256']:raise ValueError('raw camera source hash mismatch')
                    if index<20:
                        previous=prior[index]['cameras'][sensor][side]
                        for field in ['sha256','recorded_time_ns','shape','dtype']:
                            if rec[field]!=previous[field]:raise ValueError('first20source prefix mismatch '+field)
            rows.append(row)
        manifest=OUT/(case['condition']+'-source-manifest.json')
        write(manifest,{'sequence':case['sequence'],'records':rows,'source_selection_sha256':sha(SELECTION),
                        'scope':'exposed exact0..100 visual inputs only; no reference paths'})
        for modality,m in [('rgb','rgb'),('thermal','thr')]:
            previous=next(r for r in old['runs'] if r['condition']==case['condition'] and r['modality']==modality)
            dest=OUT/'canonical'/(case['condition']+'-'+modality)
            canonical_prepare(ROOT,manifest,m,dest,0)
            # Same producer/runtime: PNG bytes are expected to match, not just a
            # visually similar display-normalized or newly encoded image.
            for index in range(20):
                for side in ['left','right']:
                    name=f'{index:06d}.png';a=ROOT/previous['canonical']/side/name;b=dest/side/name
                    if sha(a)!=sha(b):raise ValueError('first20canonical bytes differ')
                    overlap.append({'case':previous['case'],'side':side,'stem':f'{index:06d}','sha256':sha(b)})
            cases.append({**previous,'manifest':str(manifest.relative_to(ROOT)),
                          'canonical':str(dest.relative_to(ROOT)),'prior_estimate':
                          f"results/droid-ms2-development/run-002/{previous['case']}/estimate.json",
                          'prior_tracking':f"results/droid-ms2-development/run-002/{previous['case']}/tracking.jsonl",
                          'prior_input_reads':f"results/droid-ms2-development/run-002/{previous['case']}/input-reads.json"})
    write(OUT/'canonical-lock.json',{'selection_sha256':sha(SELECTION),'cases':cases,
        'first20_canonical_byte_controls':overlap,'producer_runtime':selection['canonical_runtime'],
        'source_groups':101,'reference_values_parsed':False})
    print('PREPARED',len(cases),'101-frame canonical streams;',len(overlap),'exact first20PNG controls')


def freeze():
    selection=read(SELECTION);prepared=read(OUT/'canonical-lock.json')
    if prepared['selection_sha256']!=sha(SELECTION):raise ValueError('selection changed')
    old=read(ROOT/'experiments/droid-ms2-development-2026-11-04-v2.json')
    files=[SELECTION,ROOT/'tools/droid_longer_study.py',ROOT/'tools/droid_longer_worker.py',
           ROOT/'tools/droid_longer_audit.py',ROOT/'tools/tests/test_droid_longer_worker.py',
           ROOT/'tools/droid_ms2_development.py',ROOT/'tools/droid_ms2_audit.py',
           ROOT/'tools/orb_ms2_development.py',ROOT/'MAC-VO-main/DataLoader/Dataset/MS2.py',
           ROOT/'models/droid/droid.pth',ROOT/'experiments/droid-ms2-development-2026-11-04-v2.json',
           ROOT/'results/droid-longer-prefix/source-001/recovery.json',OUT/'canonical-lock.json']
    for case in prepared['cases']:
        source=read(ROOT/case['manifest']);files += [ROOT/case[key] for key in ['manifest','calibration','prior_estimate','prior_tracking','prior_input_reads']]
        files.append(ROOT/case['canonical']/'plan.json')
        for row in source['records']:
            for sensor in ['rgb','thr']:
                for side in ['left','right']:files.append(ROOT/row['cameras'][sensor][side]['path'])
            for side in ['left','right']:files.append(ROOT/case['canonical']/side/(row['stem']+'.png'))
    plan={**{key:old[key] for key in ['runtime_image','weights','resize_or_crop','uninitialized_backend_guard','internal_stereo_baseline','native_time_unit_ns','metric_translation_rule']},
          'date':'2026-11-04','parent_commit':'a81d8f306d4725eb86d293b2201e99f7129a4026',
          'selection_sha256':sha(SELECTION),'runs':prepared['cases'],'output':'results/droid-longer-prefix/run-001',
          'settings':allocation_settings(SETTINGS,101),'prior_settings':SETTINGS,
          'source_frames':101,'seed':0,'prior_prefix_groups':20,
          'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},
          'primary_engineering_population':'all6cases, every101source slot; initialization/failures retained',
          'prefix_controls':'exact first20actual tensors/clocks/K and categorical frontend state; divergences reported',
          'reference_used':False,'scale_fitted':False,'protected_payload_access':False,'benchmark_execution_ready':False}
    write(PLAN,plan);write(OUT/'plan.json',plan)
    print('FROZEN',len(plan['source_sha256']),'pins; buffer',plan['settings']['buffer'],'other settings unchanged')


def verify():
    p=read(PLAN)
    if sha(PLAN)!=sha(OUT/'plan.json'):raise ValueError('plan changed')
    for name,digest in p['source_sha256'].items():
        if sha(ROOT/name)!=digest:raise ValueError('source/input changed '+name)
    return p


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','freeze','verify'])
    args=parser.parse_args();globals()[args.action]()
