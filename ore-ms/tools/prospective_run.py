"""Host-only one-attempt preparation, freeze, and evaluation orchestration."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/prospective-objective-evaluation/run-001'
DECLARATION=ROOT/'docs/research/prospective-objective-data-protocol-2026-11-02.json'
EXECUTION=ROOT/'docs/research/prospective-objective-execution-2026-11-02.json'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def write(path,value):
 with path.open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')

def validate(p):
 for group in ('protected_pins','ingestion_script_sha256','execution_pins'):
  for rel,digest in p.get(group,{}).items():assert sha(ROOT/rel)==digest,rel
 for pin in p['models'].values():assert sha(ROOT/pin['path'])==pin['sha256']
 assert subprocess.check_output(['docker','image','inspect',p['runtime_image'],'--format','{{.Id}}'],text=True).strip()==p['runtime_image']

def pin_walk(value,pins):
 if isinstance(value,dict):
  if {'path','sha256'}<=value.keys():
   path=(ROOT/value['path']).resolve();assert path.is_relative_to(ROOT) and sha(path)==value['sha256'],path;pins[value['path']]=value['sha256']
  for child in value.values():pin_walk(child,pins)
 elif isinstance(value,list):
  for child in value:pin_walk(child,pins)

def commands(p,stages,event_name):
 events=[]
 try:
  for name,gpu in stages:
   validate(p);argv=['docker','run','--rm']+(['--gpus','all'] if gpu else [])+['-v',f'{ROOT.parent}:{ROOT.parent}','-v',f'{ROOT}:/workspace','-w',str(ROOT/'MAC-VO-main'),'-e',f'PYTHONPATH={ROOT}/MAC-VO-main','-e','OMP_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-e','CUBLAS_WORKSPACE_CONFIG=:4096:8',p['runtime_image'],'python',str(ROOT/'tools'/f'prospective_{name}.py')]
   event={'stage':name,'argv':argv,'started_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()};events.append(event);print('START',name,flush=True)
   with (OUT/f'{event_name}-{name}.log').open('x') as stream:result=subprocess.run(argv,stdout=stream,stderr=subprocess.STDOUT)
   event['exit_code']=result.returncode;event['ended_at_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();assert result.returncode==0,'failed '+name
   print('PASS',name,flush=True)
 finally:write(OUT/f'{event_name}-execution.json',{'events':events,'completed':len(events)==len(stages) and all(e.get('exit_code')==0 for e in events),'optimizer_steps':0})

parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('action',choices=['prepare','freeze','evaluate']);args=parser.parse_args()
p=json.loads(DECLARATION.read_text());validate(p)
assert sha(OUT/'data-protocol.json')==sha(DECLARATION)
if args.action=='prepare':commands(p,[('prepare',False),('audit',False)],'preparation')
elif args.action=='freeze':
 assert json.loads((OUT/'audit-data.json').read_text())['status']=='PASS'
 manifest=ROOT/p['prepared_manifest_relative'];data=json.loads(manifest.read_text());assert len(data['samples'])==32 and data['optimizer_steps']==0
 pins={str(DECLARATION.relative_to(ROOT)):sha(DECLARATION),'tools/prospective_run.py':sha(__file__),p['prepared_manifest_relative']:sha(manifest)}
 pin_walk(data,pins)
 for f in [OUT/'data-validation.json',OUT/'audit-data.json',OUT/'acquisition.json',ROOT/'results/objective-comparison/run-001/evaluation.json']:pins[str(f.relative_to(ROOT))]=sha(f)
 p.update(status='FROZEN_NATIVE_EXECUTION_BEFORE_MODEL_SCORES',source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),evaluation_manifest={'path':p['prepared_manifest_relative'],'sha256':sha(manifest)},execution_pins=pins)
 validate(p);write(EXECUTION,p);print('FROZEN',len(pins),'execution pins')
else:
 p=json.loads(EXECUTION.read_text());validate(p)
 assert not subprocess.check_output(['git','status','--porcelain','--',str(EXECUTION),str(ROOT/'tools')],cwd=ROOT,text=True).strip()
 write(OUT/'protocol.json',p);write(OUT/'produced-model-pins.json',p['models'])
 commands(p,[('evaluate',True),('metrics',False),('audit',False)],'evaluation')
