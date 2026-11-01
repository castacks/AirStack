"""Readiness is not test execution; unknown training has a narrower claim boundary."""
from copy import deepcopy
import pytest
from Scripts.Setup.prospective_recording_contract import assess


def fixture():
 records=[dict(id='day',aliases=['day-alias'],unit_kind='temporal_recording',exposure_status='reported_reserved'),dict(id='night',aliases=['night-alias'],unit_kind='temporal_recording',exposure_status='reported_reserved'),dict(id='used',aliases=['used-alias'],unit_kind='temporal_recording',exposure_status='development_exposed'),dict(id='static',aliases=['static-alias'],unit_kind='static_scene',exposure_status='reported_reserved')]
 plan={'evidence_files':{'access.json':'a'*64,'adapter.json':'b'*64,'reference.json':'c'*64,'manifest.json':'d'*64,'split.json':'e'*64,'physical.json':'f'*64},
       'partitions':{'development':['used'],'validation':[],'test':['day','night']},
       'candidate_test_records':[{'recording_id':r,'project_exposure':{'pixels':'unexamined_attested','references':'unexamined_attested','evidence_ref':'access.json'}} for r in ('day','night')],
       'adapter':{'preservation_verified':True,'evidence_ref':'adapter.json','entrypoint':'frozen_adapter.py'},
       'reference':{'benchmark_target_verified':True,'evidence_ref':'reference.json','physical_independent_pose_verified':False,'physical_evidence_ref':None},
       'test_input_metadata':{'frozen':True,'manifest_ref':'manifest.json'},
       'sampling':{'frames_per_record':100,'start':0,'rule':'first_100_official_paired_groups_no_outcome_filter'},
       'method':{'parameters_frozen':True,'code_sha256':{'candidate.py':'1'*64},'runtime_image_id':'sha256:'+'2'*64,'checkpoint_sha256':{'rgb':'3'*64,'thermal':'4'*64}},
       'decision_policy':'group_lag_one_final_before_drain','methods':['raw','history','shadow','rgb_only','thermal_only'],
       'scope':{'physical_motion_output':False,'production_adoption':False},'checkpoint_membership':{'rgb':{'status':'unknown','evidence_ref':None},'thermal':{'status':'unknown','evidence_ref':None}}}
 return plan,{'records':records}


def test_unknown_training_allows_scoped_benchmark_but_no_network_or_physical_claim():
 p,l=fixture();before=deepcopy((p,l));r=assess(p,l)
 assert r['benchmark_execution_ready'] and r['project_benchmark_claim_supported']
 assert not r['checkpoint_training_heldout_claim_supported'] and not r['physical_accuracy_claim_supported']
 assert not r['test_payloads_opened'] and not r['test_executed'] and not r['assignments_created']
 assert (p,l)==before and not r['physical_motion_admitted']


@pytest.mark.parametrize('fault',['pixels','references','missing_access_ref','adapter','missing_adapter_ref','adapter_entrypoint','reference','input_manifest','assignment','frames','boolean_frames','boolean_start','sampling_rule','parameters','runtime','checkpoint','controller','controls','physical_output','adoption','truthy_adapter'])
def test_missing_or_changed_release_artifacts_prevent_test_execution(fault):
 p,l=fixture()
 if fault=='pixels':p['candidate_test_records'][0]['project_exposure']['pixels']='reported_unexamined'
 elif fault=='references':p['candidate_test_records'][0]['project_exposure']['references']='unknown'
 elif fault=='missing_access_ref':p['candidate_test_records'][0]['project_exposure']['evidence_ref']=None
 elif fault=='adapter':p['adapter']['preservation_verified']=False
 elif fault=='missing_adapter_ref':p['adapter']['evidence_ref']='absent.json'
 elif fault=='adapter_entrypoint':p['adapter']['entrypoint']=None
 elif fault=='reference':p['reference']['benchmark_target_verified']=False
 elif fault=='input_manifest':p['test_input_metadata']['frozen']=False
 elif fault=='assignment':p['partitions']['test']=[]
 elif fault=='frames':p['sampling']['frames_per_record']=99
 elif fault=='boolean_frames':p['sampling']['frames_per_record']=True
 elif fault=='boolean_start':p['sampling']['start']=False
 elif fault=='sampling_rule':p['sampling']['rule']='pick_best_support_window'
 elif fault=='parameters':p['method']['parameters_frozen']=False
 elif fault=='runtime':p['method']['runtime_image_id']='tag:latest'
 elif fault=='checkpoint':p['method']['checkpoint_sha256']['rgb']='unknown'
 elif fault=='controller':p['decision_policy']='after_drain'
 elif fault=='controls':p['methods'].remove('thermal_only')
 elif fault=='physical_output':p['scope']['physical_motion_output']=True
 elif fault=='adoption':p['scope']['production_adoption']=True
 else:p['adapter']['preservation_verified']=1
 r=assess(p,l);assert not r['benchmark_execution_ready'] and r['missing_requirements']


@pytest.mark.parametrize('fault',['alias_duplicate','development_alias','validation_leak','unknown_clip','static','one_record','dev_val_overlap'])
def test_whole_recording_identity_and_exposure_cannot_be_bypassed(fault):
 p,l=fixture()
 if fault=='alias_duplicate':p['candidate_test_records'][1]['recording_id']='day-alias';p['partitions']['test']=['day','day-alias']
 elif fault=='development_alias':p['candidate_test_records'][0]['recording_id']='used-alias';p['partitions']['test']=['used','night']
 elif fault=='validation_leak':p['partitions']['validation']=['day-alias']
 elif fault=='unknown_clip':p['candidate_test_records'][0]['recording_id']='new-day-clip'
 elif fault=='static':p['candidate_test_records'][0]['recording_id']='static';p['partitions']['test']=['static','night']
 elif fault=='one_record':p['candidate_test_records'].pop();p['partitions']['test']=['day']
 else:p['partitions']['validation']=['used-alias']
 assert not assess(p,l)['benchmark_execution_ready']


def test_training_disjointness_requires_both_checkpoint_sources_and_ready_benchmark():
 p,l=fixture()
 p['checkpoint_membership']['rgb']={'status':'recording_disjoint_verified','evidence_ref':'split.json'}
 assert not assess(p,l)['checkpoint_training_heldout_claim_supported']
 p['checkpoint_membership']['thermal']={'status':'recording_disjoint_verified','evidence_ref':'split.json'}
 assert assess(p,l)['checkpoint_training_heldout_claim_supported']
 p['adapter']['preservation_verified']=False
 assert not assess(p,l)['checkpoint_training_heldout_claim_supported']


def test_physical_claim_needs_separate_source_not_just_benchmark_target():
 p,l=fixture();p['reference']['physical_independent_pose_verified']=True
 assert not assess(p,l)['physical_accuracy_claim_supported']
 p['reference']['physical_evidence_ref']='physical.json'
 assert assess(p,l)['physical_accuracy_claim_supported']


def test_unknown_ledger_aliases_rejected():
 p,l=fixture();l['records'][1]['aliases'].append('day-alias')
 with pytest.raises(ValueError,match='ambiguous'):assess(p,l)


@pytest.mark.parametrize('fault',['none','changed_code','escaped_path','overlapping_wrong_digest'])
def test_cli_verifies_artifacts_and_writes_only_readiness(tmp_path,monkeypatch,fault):
 import json,hashlib,sys
 from Scripts.Setup.prospective_recording_contract import main
 p,l=fixture();candidate=tmp_path/'candidate.py';candidate.write_text('# synthetic source\n')
 sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
 p['method']['code_sha256']={'candidate.py':sha(candidate)}
 for name in p['evidence_files']:
  target=tmp_path/name;target.write_text('{"synthetic":true}\n');p['evidence_files'][name]=sha(target)
 if fault=='changed_code':candidate.write_text('# changed\n')
 elif fault=='escaped_path':p['method']['code_sha256']={'../outside.py':'a'*64}
 elif fault=='overlapping_wrong_digest':p['evidence_files']['candidate.py']='a'*64
 plan=tmp_path/'plan.json';ledger=tmp_path/'ledger.json';out=tmp_path/'readiness.json'
 plan.write_text(json.dumps(p));ledger.write_text(json.dumps(l))
 monkeypatch.setattr(sys,'argv',['checker','--root',str(tmp_path),'--plan',str(plan),'--ledger',str(ledger),'--output',str(out)])
 if fault!='none':
  with pytest.raises(ValueError,match='artifact hash/path mismatch'):main()
  assert not out.exists()
 else:
  main();result=json.loads(out.read_text())
  assert result['benchmark_execution_ready'] and not result['test_executed'] and not result['test_payloads_opened']
