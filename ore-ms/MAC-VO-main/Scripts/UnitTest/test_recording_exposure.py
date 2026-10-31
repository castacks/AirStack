"""Whole-recording leakage, alias collisions and evidence uncertainty controls."""
import hashlib
from pathlib import Path
import pytest
from Scripts.Setup.audit_recording_exposure import read, validate_ledger, partition_audit, audit

ROOT=Path(__file__).resolve().parents[3]
LEDGER=ROOT/'experiments/recording-exposure-ledger-2026-10-31.json'


@pytest.fixture
def case(tmp_path):
    ledger=read(LEDGER)
    # Hermetic schema fixtures: historical hashes are checked by the separate CLI
    # audit, while these unit tests construct bounded synthetic source snapshots.
    evidence=[e for r in ledger['records'] for e in r['evidence']]+ledger['checkpoint']['evidence']
    for name,source in ledger['sources'].items():
        text='\n'.join(dict.fromkeys(e['quote'] for e in evidence if e['source']==name))
        path=tmp_path/source['snapshot_relative'];path.parent.mkdir(parents=True,exist_ok=True);path.write_text(text)
        source['sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
    return ledger,tmp_path


@pytest.fixture
def ledger(case):return case[0]


@pytest.fixture
def source_root(case):return case[1]


@pytest.fixture
def box_metadata():
    names=('0_salon_so_day','1_salon_mac_cov_day','night_mac_salon_warehousefig8',
        'night_macthermal_salon_redcourse_waypointskip1','night_so_waypoint_skip2',
        'night_teleop_so_salon_fig8','night_thermal_so_salon','night_thermal_so_salon2','night_thermal_so_salon3')
    return [{'Name':name+'.zip'} for name in names]


def reasons(result):return {i['reason'] for i in result['issues']}


def test_scoped_ledger_schema_against_fixed_metadata_fixture(ledger,source_root,box_metadata):
    r=audit(ledger,source_root,box_metadata)
    assert r['records']==20 and r['pinned_note_sources']==15
    assert r['by_dataset']=={'td3':9,'ms2':9,'cats':1,'sthereo':1}
    assert r['exposure_categories']=={'development_exposed':18,'reported_reserved':2}
    assert r['td3_unexposed_in_listed_pool']==0 and not r['scientific_payloads_opened']


def test_alias_cannot_put_the_same_recording_in_two_partitions(ledger):
    result=partition_audit(ledger,[{'recording_id':'0_salon_so_day','partition':'development'},
        {'recording_id':'0_salon_so_day.zip','partition':'test'}])
    assert 'recording_crosses_partitions' in reasons(result)
    assert 'development_exposure_in_test' in reasons(result)
    assert not result['partition_structure_valid']


@pytest.mark.parametrize('identity',['td3/20251027/0_salon_so_day/window-999','brand_new_unlisted_recording'])
def test_unknown_clips_or_recordings_are_not_new_holdouts(ledger,identity):
    result=partition_audit(ledger,[{'recording_id':identity,'partition':'test'}])
    assert reasons(result)=={'unknown_recording_or_clip_alias'} and not result['partition_structure_valid']


def test_source_image_screening_counts_without_neural_inference(ledger):
    names=('night_macthermal_salon_redcourse_waypointskip1','night_teleop_so_salon_fig8','night_thermal_so_salon','night_thermal_so_salon3')
    result=partition_audit(ledger,[{'recording_id':n,'partition':'test'} for n in names])
    assert len(result['issues'])==4 and reasons(result)=={'development_exposure_in_test'}


def test_pose_only_inspection_counts_even_when_images_not_obtained(ledger):
    record=next(r for r in ledger['records'] if r['dataset']=='sthereo')
    assert record['image_observation']=='not_obtained_in_cited_audit' and record['reference_observation']=='observed'
    assert 'development_exposure_in_test' in reasons(partition_audit(ledger,[{'recording_id':record['id'],'partition':'test'}]))


def test_cats_static_scene_is_not_a_temporal_recording(ledger):
    result=partition_audit(ledger,[{'recording_id':'OUTDOOR/CARS/scene1/','partition':'test'}])
    assert 'static_scene_cannot_validate_temporal_arbiter' in reasons(result)


def test_reserved_is_a_reported_candidate_not_network_holdout_or_completed_freeze(ledger):
    r=partition_audit(ledger,[{'recording_id':'_2021-08-13-16-31-10','partition':'test'}])
    assert r['partition_structure_valid'] and len(r['reported_reserved_candidates'])==1
    assert not r['checkpoint_training_heldout_supported'] and not r['complete_prospective_evaluation_freeze']
    record=next(r for r in ledger['records'] if r['id']=='ms2/_2021-08-13-16-31-10')
    assert record['reference_observation']=='not_established_by_recording'


def test_previously_prospective_result_is_exposed_for_future_methods(ledger):
    r=partition_audit(ledger,[{'recording_id':'_2021-08-13-22-03-03','partition':'validation'}])
    assert 'development_exposure_in_validation' in reasons(r)


@pytest.mark.parametrize('fault',['hash','quote','alias','reference_claim','checkpoint_claim'])
def test_corrupt_or_overstated_evidence_rejected(ledger,source_root,fault):
    if fault=='hash':next(iter(ledger['sources'].values()))['sha256']='0'*64
    elif fault=='quote':ledger['records'][0]['evidence'][0]['quote']='fabricated observation'
    elif fault=='alias':ledger['records'][1]['aliases'].append(ledger['records'][0]['id'])
    elif fault=='reference_claim':next(r for r in ledger['records'] if r['exposure_status']=='reported_reserved')['reference_observation']='never_read'
    else:ledger['records'][0]['MACT_Offroad_training_membership']='heldout'
    with pytest.raises(ValueError):validate_ledger(ledger,source_root)


def test_new_box_archive_remains_unknown_until_ledger_updated(ledger,source_root,box_metadata):
    box_metadata.append({'Name':'new_unclassified.zip'})
    with pytest.raises(ValueError,match='inventory changed'):audit(ledger,source_root,box_metadata)
