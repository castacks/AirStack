"""Validate selected pixels, prepare native inputs, freeze source, capture MS²."""
from pathlib import Path
import argparse,json,sys,subprocess,tarfile,struct,zlib
from ms2_broader_intake import ROOT,OUT,PLAN,sha,read,write
sys.path.insert(0,str(ROOT/'MAC-VO-main'))
EXEC=OUT/'execution-plan.json'
def prepare():
 import numpy as np,cv2
 from Scripts.Setup.audit_ms2_pilot import read_calibration
 from Scripts.Setup.ms2_execution_adapter import prepare as paired
 from Scripts.Setup.shared_arbiter_replay import schedule
 plan=read(PLAN);pins={};cases=[];references={};overlap=0
 assert sha(PLAN)==sha(OUT/'sampling-lock.json')
 for rel,h in plan['metadata_sha256'].items():assert sha(ROOT/rel)==h
 for rec in ['day','night','rain']:
  intake=read(OUT/f'{rec}-acquisition.json');assert len(intake['members'])==1200 and intake['tar_headers_validated'];overlap+=intake['archived_overlap_members_equal']
  for m in intake['members']:assert sha(ROOT/m['path'])==m['sha256'];pins[m['path']]=m['sha256']
 for c in plan['cases']:
  base=ROOT/c['source_base'];calpath=base/'pilot/sync_data'/c['sequence']/'calib.npy';cal={k:np.asarray(v['values']) for k,v in read_calibration(calpath).items()}
  def ext(sensor):
   e=np.eye(4);e[:3,:3]=cal['R_nir2'+sensor];e[:3,3]=cal['T_nir2'+sensor].reshape(3)*.001;assert np.allclose(e[:3,:3].T@e[:3,:3],np.eye(3),atol=1e-10);return e
  q=np.eye(4);q[:3,:3]=[[0,1,0],[0,0,1],[1,0,0]]
  transforms={'rgb':q.tolist(),'thermal':(ext('rgb')@np.linalg.inv(ext('thr'))@q).tolist()};records=[]
  for i,stem in enumerate(c['stems']):
   row={'stem':stem,'cameras':{}}
   for camera in ['rgb','thr']:
    stereo={'K':cal['K_'+camera+'L'].tolist(),'baseline_m':float(-cal['T_'+camera+'R'].reshape(3)[0]*.001)}
    assert stereo['baseline_m']>0 and np.max(abs(cal['T_'+camera+'R'].reshape(3)[1:]))<1e-9
    for side in ['left','right']:
     path=OUT/'source'/c['sequence']/camera/('img_'+side)/(stem+'.png');payload=path.read_bytes();assert payload[:8]==b'\x89PNG\r\n\x1a\n';offset=8
     while offset<len(payload):
      length=struct.unpack('>I',payload[offset:offset+4])[0];tag=payload[offset+4:offset+8];chunk=payload[offset+8:offset+8+length];expected=struct.unpack('>I',payload[offset+8+length:offset+12+length])[0];assert zlib.crc32(tag+chunk)&0xffffffff==expected;offset+=length+12
      if tag==b'IEND':break
     assert offset==len(payload);x=cv2.imread(str(path),cv2.IMREAD_UNCHANGED);assert x is not None
     assert list(x.shape)==([384,1224,3] if camera=='rgb' else [256,640]) and str(x.dtype)==('uint8' if camera=='rgb' else 'uint16')
     stereo[side]={'path':str(path.relative_to(ROOT)),'sha256':sha(path),'shape':list(x.shape),'dtype':str(x.dtype),'recorded_time_ns':c['recorded_times_ns'][camera+'_'+side][i]}
    row['cameras'][camera]=stereo
   records.append(row)
  dest=OUT/'inputs'/c['case'];dest.mkdir(parents=True,exist_ok=False);source=dest/'source-manifest.json';write(source,{'sequence':c['sequence'],'records':records,'scope':'fixed broader provided-source development benchmark; no reference labels in native input'})
  contract=paired(ROOT,source,dest/'paired-manifest.json');rows=contract['pairs'];assert [i for _,_,i in schedule(rows)]==[i for i in range(100) for _ in range(2)]
  cases.append({**{k:c[k] for k in ['case','sequence','recording','frames','start','stems']},'window':c['case'],'recording_id':'ms2/'+c['sequence'],'dataset':'.','manifest':str((dest/'paired-manifest.json').relative_to(ROOT)),'rows':rows,'nominal_body_from_camera':transforms})
  for p in dest.glob('*.json'):pins[str(p.relative_to(ROOT))]=sha(p)
 # Reference files are extracted but not interpreted or supplied to estimator/decision code.
 for rec in ['day','night','rain']:
  cc=[c for c in cases if c['recording']==rec];seq=cc[0]['sequence'];wanted={f'{seq}/rgb/{stem}.txt' for c in cc for stem in c['stems']};archive=ROOT/'results/public-data-audit/ms2-scout/odom'/seq/'archive.tar.bz2';references[str(archive.relative_to(ROOT))]=sha(archive);found=[]
  with tarfile.open(archive,'r:bz2') as tar:
   for m in tar:
    name=m.name.removeprefix('./')
    if name not in wanted:continue
    assert m.isfile() and m.size<10000;target=OUT/'reference'/name;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(tar.extractfile(m).read());references[str(target.relative_to(ROOT))]=sha(target);found.append(name)
  assert set(found)==wanted,(rec,len(found),list(wanted-set(found))[:3])
 for p in (ROOT/'MAC-VO-main').rglob('*.py'):
  if '.git' not in p.parts and '__pycache__' not in p.parts:pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'MAC-VO-main/Config').rglob('*.yaml'):pins[str(p.relative_to(ROOT))]=sha(p)
 for n in ['MACVO_FrontendCov.pth','MACT_Offroad.pth']:
  p=ROOT/'models/macslam'/n;pins[str(p.relative_to(ROOT))]=sha(p)
 for p in (ROOT/'tools').glob('ms2_broader*.py'):pins[str(p.relative_to(ROOT))]=sha(p)
 write(EXEC,{'sampling_lock_sha256':sha(PLAN),'runtime_image':plan['runtime_image'],'cases':cases,'input_source_sha256':pins,'scorer_only_reference_sha256':references,'expected_decisions':891,'expected_native_frame_executions':3600,'archived_overlap_members_equal':overlap,'arms':['off','on']});print('PREPARED',len(cases),'cases;',len(pins),'pins;',len(references),'reference files; overlap',overlap,flush=True)
def verify():
 p=read(EXEC);assert p['sampling_lock_sha256']==sha(PLAN)==sha(OUT/'sampling-lock.json')
 for rel,h in {**p['input_source_sha256'],**p['scorer_only_reference_sha256']}.items():assert sha(ROOT/rel)==h,rel
 return p
def capture():
 p=verify()
 for c in p['cases']:
  for arm in p['arms']:
   dest=OUT/'captures'/c['case']/arm
   if dest.exists():
    assert read(dest.parent/(arm+'-execution.json'))['exit_code']==0;print('RETAINED COMPLETED',c['case'],arm,flush=True);continue
   dest.parent.mkdir(parents=True,exist_ok=True)
   cmd=['docker','run','--rm','--gpus','all','--ipc=host','-e','OMP_NUM_THREADS=1','-e','OPENBLAS_NUM_THREADS=1','-e','MKL_NUM_THREADS=1','-v',str(ROOT)+':/workspace','-w','/workspace/MAC-VO-main',p['runtime_image'],'python','-m','Scripts.Setup.ms2_geometry_replay','--dataset','/workspace','--manifest','/workspace/'+c['manifest'],'--output','/workspace/'+str(dest.relative_to(ROOT)),'--frames',str(c['frames']),'--start','0','--timeout-seconds','300','--record-root-initialization','--acknowledge-provisional-geometry']
   if arm=='on':cmd+=['--geometry']
   print('CAPTURE',c['case'],arm,flush=True)
   with (dest.parent/(arm+'-host.log')).open('x') as f:result=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
   write(dest.parent/(arm+'-execution.json'),{'command':cmd,'exit_code':result.returncode})
   if result.returncode:raise RuntimeError('retained capture failure: '+str(dest))
   verify();print('COMPLETE',c['case'],arm,flush=True)
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['prepare','verify','capture']);a=p.parse_args()
 if a.action=='verify':print('PASS',len(verify()['input_source_sha256']),'pins')
 else:globals()[a.action]()
