from pathlib import Path
from types import SimpleNamespace as NS
import copy,json
import pytest
from Scripts.Setup.geometry_support_controls import choose,augment,evaluate
from Scripts.Setup.geometry_support_replay import retaining_run


def row():
    return {'sample_id':'x/0','actions':{'geometry_current':'RGB','candidate':'RGB','raw':'RGB','history':'ABSTAIN','rgb_only':'RGB','thermal_only':'THERMAL'},
        'native_available':{'rgb':True,'thermal':True},'normalized_information_min_eigenvalue':{'rgb':10.,'thermal':12.}}


def features():return {'rgb':{'retained_count':100,'occupied_cells':9,'retained_fraction':1.,'native_factors':100},'thermal':{'retained_count':120,'occupied_cells':8,'retained_fraction':.8,'native_factors':120}}


def test_directions_ties_and_factor_normalization():
    out=augment(row(),features());a=out['actions'];assert a['retained_count']=='THERMAL' and a['occupied_cells']=='RGB' and a['retained_fraction']=='RGB'
    assert a['information_rank']=='THERMAL' and a['information_per_factor']=='RGB'
    assert choose({'rgb':1,'thermal':1},{'rgb':True,'thermal':True})=='RGB'
    assert choose({'thermal':2},{'rgb':False,'thermal':True})=='THERMAL'
    assert choose({}, {'rgb':False,'thermal':False})=='ABSTAIN'
    with pytest.raises(ValueError):choose({'rgb':None},{'rgb':True,'thermal':False})


def test_both_supported_population_and_matched_coverage():
    a=augment(row(),features());b=copy.deepcopy(a);b.update(sample_id='x/1',both_supported=False)
    for m in b['actions']:b['actions'][m]='THERMAL' if m!='rgb_only' else 'ABSTAIN'
    labels={'x/0':{'rgb':{'translation_m':2.,'rotation_deg':1.},'thermal':{'translation_m':1.,'rotation_deg':1.}},'x/1':{'rgb':None,'thermal':{'translation_m':3.,'rotation_deg':2.}}}
    result=evaluate([a,b],labels);assert result['groups']['all']['population']==2 and result['groups']['both_supported']['population']==1
    assert result['groups']['both_supported']['methods']['retained_count']['translation_m']==1
    assert not result['primary_criterion_passed'] and not result['candidate_adopted']


def terminal(tmp_path,change=None):
    args=NS(output=tmp_path,frames=20);out=tmp_path/'rgb';out.mkdir()
    report={'status':'FAILED','completed_frames':20,'tracking_diagnostics':{'retained_observations':0,'support_state':'NO_PAIRS_SUPPORTED','unsupported_pairs_before_refinement':19},'arbiter_event_recording':{'status':'COMPLETE','observer_errors':[],'optimization_submissions':0}}
    if change:change(report)
    (out/'report.json').write_text(json.dumps(report))
    return args,out


def fail(args,camera):raise RuntimeError('No visual matches were retained')


def test_only_exact_completed_zero_match_failure_can_finish_capture(tmp_path):
    args,out=terminal(tmp_path);original=(out/'report.json').read_bytes()
    assert retaining_run(fail,args,'rgb') is None
    assert (out/'report.json').read_bytes()==original
    assert json.loads((out/'terminal-capture-contract.json').read_text())['motion_available'] is False


@pytest.mark.parametrize('change',[lambda r:r.update(completed_frames=19),lambda r:r['tracking_diagnostics'].update(retained_observations=1),lambda r:r['arbiter_event_recording'].update(optimization_submissions=1)])
def test_partial_or_nonzero_tracking_failure_is_not_accepted(tmp_path,change):
    args,out=terminal(tmp_path,change)
    with pytest.raises(RuntimeError):retaining_run(fail,args,'rgb')
    assert not (out/'terminal-capture-contract.json').exists()


def test_other_native_error_propagates(tmp_path):
    args,out=terminal(tmp_path)
    def error(*args):raise RuntimeError('actual optimizer error')
    with pytest.raises(RuntimeError,match='actual optimizer error'):retaining_run(error,args,'rgb')
