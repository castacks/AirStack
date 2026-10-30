"""Frozen six-process original-grid forward/reverse flow experiment, no estimator."""
import argparse
from datetime import datetime,timezone
import hashlib,json,time,subprocess
from pathlib import Path

CASES=[('early','new-night-validation','end-to-end-solver-control',20),('middle','long-night-validation','solver-collapse-replication',100),('day','long-salon','solver-day-replication',100)]
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
def write(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def verify(root,p):
 for section in ('source_sha256','input_sha256','checkpoint_sha256'):
  for name,digest in p[section].items():
   if sha(root/name)!=digest:raise ValueError('frozen input changed: '+name)


def prepare(root,output):
 output.mkdir(parents=True,exist_ok=False);code=root/'MAC-VO-main';files=[]
 for folder in ('DataLoader','Odometry','Module','Utility','Config','Scripts/Setup','Scripts/UnitTest'):
  files += [p for p in (code/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.yaml','.yml','.json') and p.name not in ('evaluate_flow_cycle.py','validate_flow_cycle.py')]
 files += [code/'Docker/Dockerfile.solver-vo'];inputs={};runs=[];image=subprocess.check_output(['docker','image','inspect','macvo:solver-vo','--format','{{.Id}}'],text=True).strip()
 assert image=='sha256:e1e72a2dc43630ffe47cf45c29278bd879d36b9bfb9fd6d621c1e88f88455b82'
 for window,dataset_name,previous,n in CASES:
  ds=root/'results'/dataset_name/'prepared-001/datasets'/('early' if window=='early' else 'middle');inputs.update({str(p.relative_to(root)):sha(p) for p in ds.rglob('*') if p.is_file()});assert len(read(ds/'manifest.json')['pairs'])==n
  for camera in ('rgb','thermal'):
   prior=root/'results'/previous/'run-001/runs/original'/camera;selected=[prior/'effective-config.json']+sorted((prior/'uncertainty').glob('pair-*.npz'));assert len(selected)==n
   inputs.update({str(p.relative_to(root)):sha(p) for p in selected});spec={'window':window,'camera':camera,'frames':n,'dataset_relative':str(ds.relative_to(root)),'original_relative':str(prior.relative_to(root)),'output_relative':str((output/'runs'/window/camera).relative_to(root))}
   command=['docker','run','--rm','--gpus','all','--ipc=host','-e','TORCHDYNAMO_DISABLE=1','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-v',str(root)+':/workspace','-w','/workspace/MAC-VO-main',image,'python','-m','Scripts.Setup.flow_cycle_experiment','capture','--root','/workspace','--output','/workspace/'+str(output.relative_to(root)),'--window',window,'--camera',camera]
   spec['command']=command;runs.append(spec)
 checkpoint={}
 for name,expected in [('MACVO_FrontendCov.pth','bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a'),('MACT_Offroad.pth','eed20722b73c80f016c4646741def4e4e5e2999feaf2c2715c5a6293dc35287a')]:
  p=root/'models/macslam'/name;assert sha(p)==expected;checkpoint[str(p.relative_to(root))]=expected
 write(output/'inference.json',{'created_at':datetime.now(timezone.utc).isoformat(),'base_commit':'24efa5c32','image_id':image,'runs':runs,'source_sha256':{str(p.relative_to(root)):sha(p) for p in sorted(set(files))},'input_sha256':inputs,'checkpoint_sha256':checkpoint,'reverse_budget':434,'forward_budget':434,'labels_in_protocol':False,'fit':False,'sampler':'integer pixel centers; bilinear inclusive image edges; require four finite corners; original inbound mask retained'})
 print({'runs':6,'source_files':len(set(files)),'input_files':len(inputs),'reverse_budget':434})


def run(root,output):
 p=read(output/'inference.json');verify(root,p);assert not (output/'execution.json').exists();result={'inference_sha256':sha(output/'inference.json'),'status':'RUNNING','runs':[]};(output/'logs').mkdir()
 try:
  for spec in p['runs']:
   verify(root,p);log=output/'logs'/(spec['window']+'-'+spec['camera']+'.log');start=time.monotonic_ns()
   with log.open('wb') as f:process=subprocess.run(spec['command'],stdout=f,stderr=subprocess.STDOUT,timeout=1200)
   entry={'window':spec['window'],'camera':spec['camera'],'exit_code':process.returncode,'elapsed_ns':time.monotonic_ns()-start,'log_sha256':sha(log)};result['runs'].append(entry);write(output/'execution.json',result);print(entry,flush=True)
   if process.returncode:raise RuntimeError('capture/control failed; no signal evaluation permitted')
   report=read(root/spec['output_relative']/'capture.json');assert report['status']=='PASS_CAPTURE_CONTROLS' and report['completed_pairs']==spec['frames']-1 and not report['denied_label_reads']
  verify(root,p);result['status']='PASS_ALL_FORWARD_CONTROLS'
 except BaseException as e:result.update(status='FAILED',error_type=type(e).__name__,error=str(e));raise
 finally:write(output/'execution.json',result)


def capture(root,output,window,camera):
 from Scripts.Setup.flow_cycle_sampling import install_label_guard,bilinear,summaries
 denied=install_label_guard()
 import numpy as np
 import torch
 from DataLoader.Dataset.TD3 import TD3ManifestSequence
 from Module.Frontend.Frontend import FlowFormerCovFrontend
 from Utility.Config import asNamespace
 p=read(output/'inference.json');spec,=[s for s in p['runs'] if s['window']==window and s['camera']==camera];target=root/spec['output_relative'];target.mkdir(parents=True,exist_ok=False);start=time.monotonic_ns();report={'status':'RUNNING','completed_pairs':0,'forward_calls':0,'reverse_calls':0,'initial_depth_calls':0,'denied_label_reads':denied,'label_guard_enabled':True,'pairs':[]}
 try:
  torch.manual_seed(0);np.random.seed(0);cfg=read(root/spec['original_relative']/'effective-config.json')['Odometry']['frontend'];assert cfg['type']=='FlowFormerCovFrontend' and cfg['args']['enc_dtype']==cfg['args']['dec_dtype']=='fp32'
  model=FlowFormerCovFrontend(asNamespace(cfg['args']));ds=root/spec['dataset_relative'];sequence=TD3ManifestSequence(ds,ds/'manifest.json',camera,acknowledge_provisional_geometry=True);assert len(sequence)==spec['frames'];previous=sequence[0].stereo
  for _ in range(2):model.estimate_depth(previous);report['initial_depth_calls']+=1
  for i in range(spec['frames']-1):
   current=sequence[i+1].stereo;source=root/spec['original_relative']/'uncertainty'/f'pair-{i:03d}.npz'
   with np.load(source,allow_pickle=False) as raw:
    uv0=raw['grid_uv0'].copy();uv1=raw['grid_uv1'].copy();inbound=raw['grid_inbound'].copy();k=raw['K'].copy();cov=raw['grid_raw_flow_cov'].copy();hw=raw['height_width'].copy();time0=int(raw['time0_ns'])
   assert time0==int(sequence.rows[i][camera+'_left']['time_ns']);torch.cuda.synchronize();tick=time.monotonic_ns();_,forward=model.estimate_pair(previous,current);torch.cuda.synchronize();forward_ns=time.monotonic_ns()-tick;report['forward_calls']+=1
   grid=torch.from_numpy(uv0).to(forward.flow.device);fresh=(grid+model.retrieve_pixels(grid,forward.flow).T).detach().cpu().numpy()
   exact=fresh.shape==uv1.shape and fresh.dtype==uv1.dtype and fresh.tobytes()==uv1.tobytes()
   if not exact:
    report['failed_forward_pair']=i;report['forward_max_abs_difference']=float(np.nanmax(abs(fresh-uv1)));np.savez_compressed(target/'failed-forward.npz',original=uv1,fresh=fresh);raise ValueError('forward grid endpoints do not match bitwise')
   torch.cuda.synchronize();tick=time.monotonic_ns();_,reverse=model.estimate_pair(current,previous);torch.cuda.synchronize();reverse_ns=time.monotonic_ns()-tick;report['reverse_calls']+=1
   dense=reverse.flow.detach().cpu().numpy()[0].copy();assert dense.shape==(2,int(hw[0]),int(hw[1]));sample=bilinear(dense.transpose(1,2,0),uv1);closure,valid,summary=summaries(uv0,uv1,inbound,sample['sampled'],sample['sample_valid'],k,cov)
   end=int(sequence.rows[i+1][camera+'_left']['time_ns']);np.savez_compressed(target/f'pair-{i:03d}.npz',uv0=uv0,uv1=uv1,original_inbound=inbound,K=k,raw_covariance=cov,height_width=hw,time_ns=np.array([time0,end],dtype=np.int64),**sample,closure=closure,closure_valid=valid,reverse_field_sha256=np.array(hashlib.sha256(dense.tobytes()).hexdigest()))
   report['pairs'].append({'pair_ordinal':i,'sensor_interval_ns':[time0,end],'forward_bitwise':True,'forward_elapsed_ns':forward_ns,'reverse_elapsed_ns':reverse_ns,**summary});report['completed_pairs']+=1;previous=current
   print(window,camera,i+1,'/',spec['frames']-1,flush=True)
  report['status']='PASS_CAPTURE_CONTROLS'
 except BaseException as e:report.update(status='FAILED',error_type=type(e).__name__,error=str(e));raise
 finally:
  report['elapsed_ns']=time.monotonic_ns()-start;report['cuda_peak_allocated_bytes']=int(torch.cuda.max_memory_allocated()) if torch.cuda.is_initialized() else None;write(target/'capture.json',report)


if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=('prepare','run','capture'));p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--window',choices=('early','middle','day'));p.add_argument('--camera',choices=('rgb','thermal'));a=p.parse_args()
 if a.action=='capture':capture(a.root,a.output,a.window,a.camera)
 else:{'prepare':prepare,'run':run}[a.action](a.root,a.output)
