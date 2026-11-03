"""Independent frozen-ID/calibration/frame/clock and source-file provenance audit."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'results/ms2-broader-range/run-001';sys.path.insert(0,str(ROOT/'MAC-VO-main'))
from Scripts.Setup.audit_ms2_pilot import read_calibration
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
 sampling=read(ROOT/'experiments/ms2-broader-range-2026-11-02.json');execution=read(OUT/'execution-plan.json');checks=0
 assert sha(ROOT/'experiments/ms2-broader-range-2026-11-02.json')==sha(OUT/'sampling-lock.json')==execution['sampling_lock_sha256']
 for rel,h in {**sampling['metadata_sha256'],**execution['input_source_sha256'],**execution['scorer_only_reference_sha256']}.items():assert sha(ROOT/rel)==h,rel;checks+=1
 q=np.array([[0,1,0],[0,0,1],[1,0,0]],float);expected=set();acquired=set();points=np.array([[0,0,0],[1,2,3],[-1,.2,1],[.3,-.1,8]],float)
 for c,s in zip(execution['cases'],sampling['cases']):
  assert c['case']==s['case'] and c['stems']==s['stems'] and len(c['rows'])==100
  source=ROOT/s['source_base']/'pilot/sync_data'/c['sequence'];cal={k:np.asarray(v['values']) for k,v in read_calibration(source/'calib.npy').items()}
  er=np.asarray(c['nominal_body_from_camera']['rgb']);et=np.asarray(c['nominal_body_from_camera']['thermal']);np.testing.assert_array_equal(er[:3,:3],q);np.testing.assert_array_equal(er[:3,3],[0,0,0]);rgb_from_thr=et[:3,:3]@q.T;tr=et[:3,3]
  # NIR test points independently travel NIR->THR->RGB versus direct NIR->RGB.
  thermal=points@cal['R_nir2thr'].T+cal['T_nir2thr'].reshape(3)*.001;rgb=points@cal['R_nir2rgb'].T+cal['T_nir2rgb'].reshape(3)*.001;np.testing.assert_allclose(thermal@rgb_from_thr.T+tr,rgb,rtol=1e-12,atol=1e-12);checks+=12
  manifest=read(ROOT/c['manifest']);source_manifest=read(ROOT/manifest['source_manifest_relative']);assert sha(ROOT/manifest['source_manifest_relative'])==manifest['source_manifest_sha256'];assert source_manifest['sequence']==c['sequence']
  for i,(stem,row,source_row) in enumerate(zip(c['stems'],c['rows'],source_manifest['records'])):
   assert row['source_stem']==source_row['stem']==stem
   for camera,native in [('rgb','rgb'),('thermal','thr')]:
    stereo=source_row['cameras'][native];np.testing.assert_array_equal(stereo['K'],cal['K_'+native+'L']);assert stereo['baseline_m']==-float(cal['T_'+native+'R'].reshape(3)[0])*.001
    for side in ['left','right']:
     t=int((source/native/f'img_{side}_timestamp.txt').read_text().split()[s['start']+i]);a=row[camera+'_'+side];b=stereo[side];assert a['time_ns']==b['recorded_time_ns']==t==s['recorded_times_ns'][native+'_'+side][i]
     assert Path(a['path']).parts[-4:]==(c['sequence'],native,'img_'+side,stem+'.png');assert a['path']==b['path'];expected.add(a['path']);checks+=4
 for rec in ['day','night','rain']:
  intake=read(OUT/f'{rec}-acquisition.json');assert intake['archived_overlap_members_equal']==76 and not intake['current_full_archive_sha256_verified']
  for m in intake['members']:assert sha(ROOT/m['path'])==m['sha256'];acquired.add(m['path']);checks+=1
 assert acquired==expected and len(acquired)==3600
 result={'status':'PASS_INDEPENDENT_FROZEN_SOURCE_AND_CALIBRATION_AUDIT','checks':checks,'selected_groups':900,'source_pngs':3600,'archived_overlap_equal':228,'official_calibration_composition_verified':True,'physical_exposure_clocks_verified':False,'full_current_sync_archive_hash_verified':False,'auditor_sha256':sha(__file__)}
 with (OUT/'source-audit.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
 print(result)
if __name__=='__main__':main()
