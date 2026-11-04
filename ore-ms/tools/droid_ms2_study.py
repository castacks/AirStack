"""Freeze canonical-input DROID stereo engineering runs before native outcomes."""
import argparse
import json
from pathlib import Path
import subprocess

from droid_ms2_development import SETTINGS,sha,read,write,ROOT,model_geometry

PLAN=ROOT/'experiments/droid-ms2-development-2026-11-04.json'
OUT=ROOT/'results/droid-ms2-development/run-001'


def freeze(runtime):
    old=read(ROOT/'experiments/orb-ms2-development-2026-11-04-v4.json')
    if not read(ROOT/'results/droid-ms2-development/preflight-001/stereo-contract.json')['pass']:
        raise ValueError('native CPU/CUDA metric contract must pass before smoke')
    files=[ROOT/'tools/droid_ms2_development.py',ROOT/'tools/droid_ms2_study.py',
           ROOT/'tools/droid_ms2_audit.py',ROOT/'tools/droid_stereo_contract.py',
           ROOT/'tools/tests/test_droid_ms2_development.py',ROOT/'environment/Dockerfile.droid-ms2',
           ROOT/'environment/prepare_droid_build.py',ROOT/'models/droid/droid.pth',
           ROOT/'results/droid-ms2-development/source-001/checkpoint.json',
           ROOT/'results/droid-ms2-development/preflight-001/stereo-contract.json',
           ROOT/'experiments/droid-stereo-contract-2026-11-04.json',
           ROOT/'experiments/independent-benchmark-2026-11-04.json']
    files += [p for p in (ROOT/'results/droid-ms2-development/source-001').iterdir() if p.is_file()]
    runs=[]
    for case in old['cases']:
        source=read(ROOT/case['manifest']);calibration=read(ROOT/case['calibration'])
        files += [ROOT/case['manifest'],ROOT/case['calibration']]
        for modality,m in [('rgb','rgb'),('thermal','thr')]:
            label=case['condition']+'-'+('rgb' if modality=='rgb' else 'thr')
            canonical=f'results/orb-ms2-development/run-004/{label}'
            for row in source['records']:
                for side in ['left','right']:
                    files.append(ROOT/canonical/side/(row['stem']+'.png'))
            files.append(ROOT/canonical/'plan.json')
            c=source['records'][0]['cameras'][m]
            if c['K']!=calibration['K_'+m+'L']['values'] or c['baseline_m']!=-calibration['T_'+m+'R']['values'][0][0]/1000:
                raise ValueError('source calibration unit/intrinsic mismatch')
            runs.append({'case':case['condition']+'-'+modality,'condition':case['condition'],
                         'recording':case['recording'],'modality':modality,'manifest':case['manifest'],
                         'calibration':case['calibration'],'canonical':canonical,
                         'baseline_m':c['baseline_m'],'shape':c['left']['shape'][:2],
                         'intrinsics':[c['K'][0][0],c['K'][1][1],c['K'][0][2],c['K'][1][2]]})
    plan={'date':'2026-11-04','parent_commit':'90f772ec2110cc6d072847e9927012d8e36c6f24',
          'runtime_image':runtime,'weights':'models/droid/droid.pth','runs':runs,
          'settings':SETTINGS,'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in files},
          'source_frames':20,'seed':0,'resize_or_crop':False,'canonical_recipe_changed':False,
          'native_time_unit_ns':100_000_000,'internal_stereo_baseline':.1,
          'metric_translation_rule':'provided_baseline_m / internal_baseline_0.1',
          'source_timestamps':'original integer left/right ns retained in all output slots',
          'expected_output_scope':'offline global backend plus native interpolated initialization/image-refined filler',
          'initialization_gate':'uninitialized frontend => null admitted poses, regardless of finite filler',
          'semantic_failures':'retain original slots and failed initialization/terminal status; no outcome-based retries',
          'reference_used':False,'scale_fitted':False,'protected_payload_access':False,
          'benchmark_execution_ready':False}
    OUT.mkdir(parents=True,exist_ok=False);write(PLAN,plan);write(OUT/'plan.json',plan)
    print('FROZEN',len(plan['source_sha256']),'pins;',len(runs),'native runs')


def verify(protocol=PLAN):
    protocol=Path(protocol);plan=read(protocol)
    out=ROOT/plan.get('output','results/droid-ms2-development/run-001')
    if sha(protocol)!=sha(out/'plan.json'):raise ValueError('plan changed')
    for path,expected in plan['source_sha256'].items():
        if sha(ROOT/path)!=expected:raise ValueError('changed frozen source/input '+path)
    return plan


def amend(runtime):
    p=read(PLAN)
    allowed={'tools/droid_ms2_development.py','tools/droid_ms2_study.py','tools/droid_ms2_audit.py','tools/tests/test_droid_ms2_development.py'}
    for path,digest in p['source_sha256'].items():
        if path not in allowed and sha(ROOT/path)!=digest:raise ValueError('unexpected changed prerequisite '+path)
    p['amendment_of_sha256']=sha(PLAN)
    p['amendment_reason']='Retain full-resolution int32 correlation failures. Use native demo196608pixel area cap, no upsampling; scale intrinsics by precrop resize dimensions. Guard missing initialization before offline backend; do not alter warmup/motion thresholds or admit filled uninitialized poses.'
    p['runtime_image']=runtime;p['output']='results/droid-ms2-development/run-002'
    p['resize_or_crop']=True;p['uninitialized_backend_guard']=True
    for case in p['runs']:
        geometry=model_geometry(case['shape']);case['source_shape']=case['shape']
        case['source_intrinsics']=case['intrinsics'];sx,sy=geometry['scale_xy']
        case['intrinsics']=[value*scale for value,scale in zip(case['source_intrinsics'],[sx,sy,sx,sy])]
        case['shape']=geometry['shape'];case['image_geometry']=geometry
    p['source_sha256']={path:sha(ROOT/path) for path in p['source_sha256']}
    for path in ['results/droid-ms2-development/run-001/audit.json','experiments/droid-stereo-contract-2026-11-04-v2.json','results/droid-ms2-development/preflight-002/stereo-contract.json']:
        p['source_sha256'][path]=sha(ROOT/path)
    if not read(ROOT/'results/droid-ms2-development/preflight-002/stereo-contract.json')['pass']:raise ValueError('renewed geometry contract failed')
    out=ROOT/p['output'];out.mkdir(parents=True,exist_ok=False)
    write(ROOT/'experiments/droid-ms2-development-2026-11-04-v2.json',p);write(out/'plan.json',p)
    print('AMENDED',len(p['source_sha256']),'pins; original runs/outputs retained')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['freeze','verify','amend']);parser.add_argument('--runtime');parser.add_argument('--protocol',type=Path,default=PLAN)
    args=parser.parse_args()
    if args.action in ('freeze','amend'):
        if not args.runtime or not args.runtime.startswith('sha256:'):raise ValueError('exact runtime digest required')
        (freeze if args.action=='freeze' else amend)(args.runtime)
    else:verify(args.protocol);print('All frozen source/input hashes unchanged')
