"""Qualification on saved development outputs; no labels or new estimator run."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
from scipy.linalg import expm,logm
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'MAC-VO-main'))
from Scripts.Setup.causal_interval_model import project
from Scripts.Setup.td3_reference import matrices,OPTICAL_FROM_NED
from rotation_signal_audit import feature_vector,fixed_choices
import selected_motion_study
from common_time_model import transport,identity_assumption
ROOT=Path(__file__).resolve().parents[1];BASE=ROOT/'results/bounded-asset-recovery/run-001';OUT=ROOT/'results/decisive-test-gates/run-001'
def read(p):return json.loads(Path(p).read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
 OUT.mkdir(parents=True,exist_ok=True);smoke=read(BASE/'smoke-validation.json');assert smoke['passed'];source=read(BASE/'source-manifest.json');calfile=ROOT/'results/ms2-night-pilot/calibration.json';cal=read(calfile)
 loader=ROOT/'results/provided-reference-full-record/run-001/source-001/official-loader.py';assert sha(loader)=='299250fcdc33548b6cab3bf585d3f5b205af3635eb038e7e113fee73447de9a1'
 selected_motion_study.SOURCE=loader;rgb_from_thermal=selected_motion_study.source_frame(cal);q=np.eye(4);q[:3,:3]=OPTICAL_FROM_NED;transforms=dict(rgb=q,thermal=rgb_from_thermal@q)
 def block(m):
  e=np.eye(4);e[:3,:3]=cal['R_nir2'+m]['values'];e[:3,3]=np.asarray(cal['T_nir2'+m]['values']).reshape(3)*.001;return e
 assert np.allclose(rgb_from_thermal,block('rgb')@np.linalg.inv(block('thr')),rtol=0,atol=1e-14)
 records={};pins={};focal={}
 for mode,camera in [('rgb','rgb'),('thermal','thr')]:
  file=BASE/'captures'/mode/'arbiter-events/events.jsonl';events=[json.loads(s) for s in file.read_text().splitlines()];pins[str(file.relative_to(ROOT))]=sha(file)
  health={e['time1_ns']:e for e in events if e['kind']=='pair_end'};motions={e['time1_ns']:e for e in events if e['kind']=='motion_result_written' and e['first_write_for_submission']};assert len(health)==len(motions)==19
  records[mode]=dict(health=health,motions=motions);k=np.asarray(source['records'][0]['cameras'][camera]['K']);focal[mode]=float((k[0,0]**2+k[1,1]**2)/2)
 rows=[];checks=0
 for i in range(19):
  common=[int(source['records'][j]['cameras']['rgb']['left']['recorded_time_ns']) for j in [i,i+1]];experts={};raw={}
  for mode,camera in [('rgb','rgb'),('thermal','thr')]:
   native=[int(source['records'][j]['cameras'][camera]['left']['recorded_time_ns']) for j in [i,i+1]];motion=records[mode]['motions'][native[1]];health=records[mode]['health'][native[1]]
   assert [motion['time0_ns'],motion['time1_ns']]==native==[health['time0_ns'],health['time1_ns']]
   if motion['stage']!='pair_processing' or motion['original_need_interp'] is not False:
    experts[mode]=dict(valid_model_projection=False,reason='source_motion_not_pair_processing' if motion['stage']!='pair_processing' else 'native_unsupported',native_interval_ns=native,target_interval_ns=common,native_motion_event=motion['sequence'],source_stage=motion['stage'],delta_model=None,admission_ready=False,physical_body_transform_verified=False,sensor_clock_verified=False,controller_deadline_verified=False)
    raw[mode]={k:health[k] for k in ['grid_trace_q10_q50_q90_px2','inbound_count','retained_count','occupied_3x3_cells']}
    continue
   d=matrices([motion['derived_camera_delta_xyzw']])[0];e=transforms[mode];converted=e@d@np.linalg.inv(e);result=project(converted,native,common)
   source_clock='provided-'+mode+'-left';target_clock='provided-rgb-left';clock=None if mode=='rgb' else identity_assumption(source_clock,target_clock)
   other=transport(d.tolist(),native,common,source_clock,target_clock,e.tolist(),clock)
   assert result['valid_model_projection']==other['accepted'];checks+=1
   if result['valid_model_projection']:
    expected=expm(logm(converted)*((common[1]-common[0])/(native[1]-native[0])));assert np.max(abs(expected.imag))<2e-8;assert np.allclose(result['delta_model'],expected.real,atol=2e-8,rtol=0) and np.allclose(result['delta_model'],other['delta_target'],atol=2e-8,rtol=0);checks+=2
   for k in ['admission_ready','physical_body_transform_verified','sensor_clock_verified','controller_deadline_verified']:assert result[k] is False
   experts[mode]=dict(result,native_camera_delta=d.tolist(),target_from_native_camera=e.tolist(),clock_assumption=clock,frame_evidence='pinned official set_extrinsics; declared native NED-to-optical basis',native_motion_event=motion['sequence'],health_event=health['sequence']);raw[mode]={k:health[k] for k in ['grid_trace_q10_q50_q90_px2','inbound_count','retained_count','occupied_3x3_cells']}
  features,reason=feature_vector(raw,focal);assert reason is None and features is not None
  choices=fixed_choices(dict(features=features,raw=raw));rows.append(dict(source_ids=[source['records'][j]['stem'] for j in [i,i+1]],common_interval_ns=common,experts=experts,features=features,feature_reason=reason,raw_angular_choice=choices['angular_uncertainty'],angular_choice=choices['angular_uncertainty'] if all(e['valid_model_projection'] for e in experts.values()) else 'ABSTAIN',reference_used=False))
 files=[Path(__file__),ROOT/'tools/rotation_signal_audit.py',ROOT/'tools/common_time_model.py',ROOT/'tools/selected_motion_study.py',ROOT/'MAC-VO-main/Scripts/Setup/causal_interval_model.py',ROOT/'MAC-VO-main/Scripts/Setup/arbiter_motion_output.py',ROOT/'MAC-VO-main/Scripts/Setup/td3_reference.py',ROOT/'MAC-VO-main/Scripts/Setup/arbiter_pose_proxy.py',ROOT/'tools/ms2_broader_execution.py',ROOT/'tools/ms2_broader_evaluate.py',loader,calfile,BASE/'source-manifest.json',BASE/'smoke-validation.json']
 pins.update({str(f.relative_to(ROOT)):sha(f) for f in files})
 report=dict(status='PASS_CONDITIONAL_NOMINAL_TARGET',development_pairs=len(rows),both_model_supported=sum(all(v['valid_model_projection'] for v in r['experts'].values()) for r in rows),complete_existing_uncertainty_features=len(rows),unavailable_source_pairs=[r['source_ids'] for r in rows if not all(e['valid_model_projection'] for e in r['experts'].values())],source_frame_formula_exact=True,existing_projection_implementations_agree=True,independent_matrix_log_comparisons=checks,reference_values_read=False,new_native_inference=False,physical_clock_verified=False,physical_frame_verified=False,shared_controller_deadline_claimed=False,online_claimed=False,benchmark_admission=False,scope='existing fixed nominal model target compatibility only; no accuracy/ranking/heldout result',source_sha256=pins,records=rows)
 with (OUT/'target-qualification.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
 print('PASS conditional nominal target:',len(rows),'development pairs;',report['both_model_supported'],'both-supported; all existing uncertainty features available. No labels/inference.')
if __name__=='__main__':main()
