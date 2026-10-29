"""Known decisions/losses test diagnostic abstention and matched populations."""
import copy
import numpy as np
import pytest
from Scripts.Setup.compare_factor_proxy import preference, stats, association, summarize, FEATURES, OLD


@pytest.mark.parametrize('a,b,expected', [(1, 2, ('RGB', 'decisive')), (2, 1, ('THERMAL', 'decisive')), (2, 2, ('RGB', 'exact_tie_rgb')), (None, 1, ('ABSTAIN', 'missing_feature')), (1, float('nan'), ('ABSTAIN', 'missing_feature')), (float('inf'), 1, ('ABSTAIN', 'missing_feature'))])
def test_preferences(a, b, expected):
    assert preference({'rgb': a, 'thermal': b}, {'rgb': True, 'thermal': True}) == expected


@pytest.mark.parametrize('a,b,expected', [(True, False, 'RGB'), (False, True, 'THERMAL'), (False, False, 'ABSTAIN')])
def test_availability(a, b, expected):
    assert preference({'rgb': None, 'thermal': None}, {'rgb': a, 'thermal': b})[0] == expected


def test_higher_old_signal():
    assert preference({'rgb': .5, 'thermal': .8}, {'rgb': True, 'thermal': True}, 'higher') == ('THERMAL', 'decisive')


def test_empty_statistics():
    assert stats([]) == {'count': 0, 'mean': None, 'rmse': None}


def test_rank_reasons():
    assert association([1, 2], [2, 1])['reason'] == 'fewer_than_three'
    assert association([1, 1, 1], [1, 2, 3])['reason'] == 'constant_ranks'
    assert association([1, 2, 3], [3, 2, 1])['rho'] == -1


def row():
    return {'pair_ordinal': 0, 'availability': {'rgb': True, 'thermal': True},
            'values': {'rgb': {k: 2 for k in FEATURES}, 'thermal': {k: 1 for k in FEATURES}},
            'choices': {k: {'action': 'THERMAL', 'reason': 'decisive'} for k in FEATURES},
            'old_choices': {k: {'action': 'RGB', 'reason': 'decisive'} for k in OLD},
            'controls': {'rgb_only': 'RGB', 'thermal_only': 'THERMAL', 'fixed_rgb_preference': 'RGB', 'history': 'ABSTAIN'},
            'losses': {'rgb': {'translation_m': 1., 'rotation_deg': 3.}, 'thermal': {'translation_m': 2., 'rotation_deg': 1.}}}


def test_opposite_loss_winners_history_abstain():
    s = summarize([row()]); f = s['features'][FEATURES[0]]
    assert f['accuracy']['translation_m']['decisive_correct'] == 0
    assert f['accuracy']['rotation_deg']['decisive_correct'] == 1
    assert f['matched_controls']['history']['count'] == 0
    assert f['old_signal_disagreements']['flow_trace_px2']['differences'] == {'translation_m': {'worse': 1}, 'rotation_deg': {'better': 1}}


def test_reference_tie_and_missing_old():
    r = row(); r['losses']['thermal']['translation_m'] = 1.+1e-13
    for k in OLD: r['old_choices'][k]['action'] = 'ABSTAIN'
    s = summarize([r])['features'][FEATURES[0]]
    assert s['accuracy']['translation_m']['reference_ties'] == 1
    assert s['accuracy']['translation_m']['decisive_correct'] == 0
    assert s['old_signal_disagreements']['flow_trace_px2']['missing'] == 1


def test_matched_support_excludes_abstention():
    a = row(); b = copy.deepcopy(a); b['pair_ordinal'] = 1
    for k in FEATURES: b['choices'][k] = {'action': 'ABSTAIN', 'reason': 'missing_feature'}
    s = summarize([a, b]); f = s['features'][FEATURES[0]]
    assert s['full_available'][FEATURES[0]]['abstain'] == 1
    assert f['missing'] == 1 and f['pair_ordinals'] == [0]
    assert f['matched_controls']['rgb_only']['pair_ordinals'] == [0]
    assert f['matched_controls']['rgb_only']['first']['translation_m']['rmse'] == 2.
    assert f['matched_controls']['rgb_only']['second']['translation_m']['rmse'] == 1.
