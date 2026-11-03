"""EXP112 exact exposed-window freeze and sequential matched native captures."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'results/sensor-admission-execution/run-002'
PLAN=ROOT/'experiments/sensor-admission-execution-2026-11-02-compiler.json'
IMAGE='sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def write(p,v):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False);f.write('\n')

def freeze():
    assert not PLAN.exists()
    old=read(ROOT/'results/prefix-model-pipeline/run-001/predictor-contract.json')
    third=read(ROOT/'results/third-recording-shared-capture/run-001/capture-plan.json')
    specs=[('night-early','early','new-night-validation','early','shared-arbiter-events','night_mac_salon_warehousefig8'),
        ('night-middle','middle','long-night-validation','middle','arbiter-support-collapse','night_mac_salon_warehousefig8'),
        ('day0','day','long-salon','middle','history-day-replication','0_salon_so_day')]
    cases=[];pins={};references={}
    for name,window,folder,datasetcase,original,record in specs:
        dataset=f'results/{folder}/prepared-001/datasets/{datasetcase}'
        rows=read(ROOT/f'results/{original}/run-001/protocol.json')['input_rows']
        assert read(ROOT/dataset/'manifest.json')['pairs']==rows
        cases.append({'case':name,'window':window,'dataset':dataset,'frames':len(rows),'rows':rows,
            'recording_id':'td3/20251027/'+record,'reference_dataset':'data/tartandrive3/20251027/'+record,
            'nominal_body_from_camera':old[window]['nominal_body_from_camera']})
    # Static nominal transforms for Day 1 are already frozen in its separate exports.
    for spec in third['windows']:
        window=spec['case'];base=ROOT/'results/third-recording-shared-capture/run-001/windows'/window
        transforms=read(base/'model-predictions.json')['nominal_transforms']
        dataset=spec['dataset_relative'];rows=spec['input_rows'];assert read(ROOT/dataset/'manifest.json')['pairs']==rows
        cases.append({'case':window,'window':window,'dataset':dataset,'frames':len(rows),'rows':rows,
            'recording_id':'td3/20251027/1_salon_mac_cov_day','reference_dataset':'data/tartandrive3/20251027/1_salon_mac_cov_day',
            'nominal_body_from_camera':transforms})
    for c in cases:
        base=ROOT/c['dataset'];paths=[base/'manifest.json',*base.glob('sensors/*_camera_info/data.yaml')]
        assert len(paths)==5
        for row in c['rows']:
            paths.extend(base/row[k]['path'] for k in ['rgb_left','rgb_right','thermal_left','thermal_right'])
        for path in paths:pins[str(path.relative_to(ROOT))]=sha(path)
        for name in ['info.yaml','interp_data.txt','interp_timestamps.txt']:
            path=ROOT/c['reference_dataset']/'super_odometry/odometry'/name;references[str(path.relative_to(ROOT))]=sha(path)
    assert sum(c['frames']-1 for c in cases)==274
    weights={'models/macslam/MACVO_FrontendCov.pth':'bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a',
        'models/macslam/MACT_Offroad.pth':'eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a'}
    for name,digest in weights.items():assert sha(ROOT/name)==digest;pins[name]=digest
    for path in (ROOT/'MAC-VO-main').rglob('*.py'):
        if any(x in path.parts for x in ['.git','__pycache__']):continue
        pins[str(path.relative_to(ROOT))]=sha(path)
    for path in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(path.relative_to(ROOT))]=sha(path)
    for path in (ROOT/'tools').glob('sensor_admission_*.py'):pins[str(path.relative_to(ROOT))]=sha(path)
    plan={'parent_commit':'db7aa7b51','kind':'development matched off/on preservation then fixed whole-motion selector comparison',
        'runtime_image':IMAGE,'runtime_amendment':'Compiler/C runtime packages added after original Torch Inductor C-compiler failure. Same Python packages, models, input windows, policy and solver. Original attempt retained.','original_plan_sha256':sha(ROOT/'experiments/sensor-admission-execution-2026-11-02.json'),'cases':cases,'input_source_sha256':pins,'scorer_only_reference_sha256':references,
        'policy_protocol_sha256':sha(ROOT/'docs/research/sensor-admission-comparison-protocol-2026-11-02.md'),
        'arms':['off','on'],'neural_frames':1120,'expected_decisions_per_arm':274,'training_updates':0,'reserve_access':False,
        'criterion':'EXP111 unchanged: candidate coverage >= raw and strictly lower matched translation RMSE than every control with no rotation regression, on every recording',
        'no_threshold_or_policy_search':True,'native_preservation_required':'all serialized arrays and mean/health writes exactly equal; geometry errors absent',
        'source_reference_scope':'nominal-frame recorded SLAM proxy; no independent physical accuracy, network training holdout or live deadline claim'}
    write(PLAN,plan);write(OUT/'plan.json',plan)
    print('FROZEN',len(cases),'cases;',len(pins),'source/input hashes;',len(references),'reference hashes')

def verify():
    p=read(PLAN)
    assert sha(PLAN)==sha(OUT/'plan.json')
    for name,digest in p['input_source_sha256'].items():assert sha(ROOT/name)==digest,name
    for name,digest in p['scorer_only_reference_sha256'].items():assert sha(ROOT/name)==digest,name
    return p

def capture():
    p=verify()
    for case in p['cases']:
        for arm in p['arms']:
            dest=OUT/'captures'/case['case']/arm
            if dest.exists():raise ValueError('capture exists, no automatic retry: '+str(dest))
            dest.parent.mkdir(parents=True,exist_ok=True)
            module='Scripts.Setup.shared_geometry_replay' if arm=='on' else 'Scripts.Setup.shared_arbiter_replay'
            cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',
                '-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',p['runtime_image'],'python','-m',module,
                '--dataset','/workspace/'+case['dataset'],'--manifest','/workspace/'+case['dataset']+'/manifest.json',
                '--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames',str(case['frames']),'--start','0',
                '--timeout-seconds','300','--record-root-initialization','--acknowledge-provisional-geometry']
            print('CAPTURE',case['case'],arm,flush=True)
            with (dest.parent/(arm+'-host.log')).open('x') as log:
                result=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
            write(dest.parent/(arm+'-execution.json'),{'command':cmd,'exit_code':result.returncode})
            if result.returncode:raise RuntimeError('retained capture failure: '+str(dest))
            verify();print('COMPLETE',case['case'],arm,flush=True)

if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__);a.add_argument('action',choices=['freeze','verify','capture']);args=a.parse_args()
    if args.action=='freeze':freeze()
    elif args.action=='capture':capture()
    else:print('PASS',len(verify()['input_source_sha256']),'frozen sources/inputs')
