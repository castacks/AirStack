"""Host stdlib orchestration. Freeze first, commit, then execute once."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'results/objective-comparison/run-001'
PROTOCOL = ROOT / 'docs/research/objective-comparison-execution-2026-11-02.json'

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')

def collect_pins(value, pins):
    if isinstance(value, dict):
        if set(('path', 'sha256')) <= value.keys():
            path = ROOT / value['path']
            assert path.is_relative_to(ROOT) and sha(path) == value['sha256'], path
            pins[value['path']] = value['sha256']
        for child in value.values(): collect_pins(child, pins)
    elif isinstance(value, list):
        for child in value: collect_pins(child, pins)

def validate(plan):
    for name, digest in plan['pins'].items():
        assert sha(ROOT / name) == digest, name
    assert subprocess.check_output(['docker', 'image', 'inspect', plan['runtime_image'], '--format', '{{.Id}}'], text=True).strip() == plan['runtime_image']

def freeze(image):
    assert not OUT.exists() and not PROTOCOL.exists()
    prepared = ROOT / 'docs/research/objective-comparison-prepared-commands-2026-11-02.json'
    p = json.loads(prepared.read_text())
    old = json.loads((ROOT/'results/objective-bound-runner/run-001/protocol.json').read_text())
    pins = dict(old['old_source_sha256'])
    for name in pins: assert sha(ROOT/name) == pins[name], name
    for path in (ROOT/'MAC-VO-main/Train/MatchingNet').glob('*.py'):
        pins[str(path.relative_to(ROOT))] = sha(path)
    for path in (ROOT/'tools').glob('objective_*.py'):
        pins[str(path.relative_to(ROOT))] = sha(path)
    manifests = ['experiments/covariance-coverage-broad-2026-11-01.json', 'experiments/frozen-multirecord-evaluation-2026-11-01.json']
    for name in manifests:
        pins[name] = sha(ROOT/name)
        collect_pins(json.loads((ROOT/name).read_text()), pins)
    pins[str(prepared.relative_to(ROOT))] = sha(prepared)
    for name in ['docs/research/trace-risk-objective-protocol-2026-11-02.json', 'docs/research/frozen-calibration-ranking-protocol-2026-11-02.json']:
        pins[name] = sha(ROOT/name)
    for command in p['commands']:
        command['argv'] = [image if arg == p['runtime_image'] else arg for arg in command['argv']]
        assert not (ROOT/command['output_relative']).exists()
    metric = json.loads((ROOT/'docs/research/frozen-calibration-ranking-protocol-2026-11-02.json').read_text())
    models={'baseline':json.loads((ROOT/manifests[0]).read_text())['initialization']['file']}
    for command in p['commands'][:3]:
        models[command['name']]={'path':command['output_relative']+'/step-00000048.pt','sha256':None}
    plan={'status':'FROZEN_BEFORE_TRAINING','created_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
          'pins':pins,'runtime_image':image,'historical_runtime_image':p['runtime_image'],
          'runtime_change':'rebuilt pinned-base Dockerfile.blackwell; fresh preflight and runtime binding required; no historical bitwise claim',
          'commands':p['commands'],'scientific_updates':144,'verification_updates':72,
          'training_manifest_relative':manifests[0],'evaluation_manifest':{'path':manifests[1],'sha256':pins[manifests[1]]},
          'models':models,'retained_fractions':metric['retained_fractions'],'coverage_levels':metric['coverage_levels'],
          'primary':'new_only equal-record macro within-pair trace-ranked MSE at50percent retention; finaltrace versus finaldiagonal',
          'secondary':['all risk curves/random/oracle','perrecord/group recurrent and final diagonal NLL','joint coverage and standardized moments','objective/gradient/clipping','exact24to48 model/optimizer/RNG/journal replay','frozen state/mean fields'],
          'aggregation':metric['aggregation'],'interpretation_gate':'report coverage/NLL tradeoffs; no dominance unless primary improves without worse perrecord95coverage distance-to-nominal or finalNLL',
          'failure_policy':'single attempt per declared command; retain failures; stop on failure; no outcome-dependent tuning',
          'scope':'observed synthetic RGB development prefixes; unknown basepretraining; no independent test/thermal/physical/VO claim','candidate_promoted':False}
    validate(plan); write(PROTOCOL,plan)
    print('Frozen',len(pins),'pins')

def execute():
    p=json.loads(PROTOCOL.read_text()); validate(p)
    assert subprocess.check_output(['git','status','--porcelain','--',str(PROTOCOL),str(ROOT/'tools')],cwd=ROOT,text=True).strip()==''
    OUT.mkdir(parents=True,exist_ok=False);write(OUT/'protocol.json',p)
    events=[]
    try:
        commands=p['commands']+[
            {'name':name,'argv':['docker','run','--rm','--gpus','all','-v',f'{ROOT.parent}:{ROOT.parent}','-v',f'{ROOT}:/workspace','-w',str(ROOT/'MAC-VO-main'),'-e',f'PYTHONPATH={ROOT}/MAC-VO-main','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1',p['runtime_image'],'python',str(ROOT/'tools'/script)],'log_relative':str((OUT/(name+'.log')).relative_to(ROOT))}
            for name,script in [('audit_training','objective_audit.py'),('evaluate','objective_evaluate.py'),('metrics','objective_metrics.py'),('audit_evaluation','objective_audit.py')]]
        for command in commands:
            validate(p)
            if command['name']=='evaluate':
                for model in p['models'].values(): model['sha256']=sha(ROOT/model['path'])
                # Produced checkpoint hashes supplement the immutable committed declaration.
                write(OUT/'produced-model-pins.json',p['models'])
            event={'name':command['name'],'started_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'argv':command['argv']};events.append(event)
            print('START',command['name'],flush=True)
            with (ROOT/command['log_relative']).open('x') as log:
                result=subprocess.run(command['argv'],stdout=log,stderr=subprocess.STDOUT)
            event['exit_code']=result.returncode;event['ended_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
            if result.returncode:raise RuntimeError('command failed: '+command['name'])
            print('PASS',command['name'],flush=True)
    finally:
        write(OUT/'execution.json',{'events':events,'completed':len(events)==10 and all(e.get('exit_code')==0 for e in events)})

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['freeze','execute']);parser.add_argument('--image');args=parser.parse_args()
    if args.action=='freeze':assert args.image;freeze(args.image)
    else:execute()
