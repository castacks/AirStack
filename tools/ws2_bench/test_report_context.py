import copy

import pytest

from report_context import compact_report_evidence


def fixture():
    diagnostics = {'sample_count': 100, 'snapshots': [
        {'sim_time_s': float(i), 'clearance_m': 1.-i/100,
         'position_m': [float(i), 0., 1.],
         'worker': {'steering': i/100, 'collision_probability': i/100}}
        for i in range(100)]}
    clean = {'outcome': 'route_completed', 'metrics': {'minimum_obstacle_clearance_m': .01},
             'diagnostics': diagnostics}
    attacked = {**copy.deepcopy(clean), 'outcome': 'collision'}
    report = {'paired_comparisons': [{'round': 1, 'clean': clean, 'perturbed': attacked}]}
    evidence = {'rounds': [{'id': 'round_01', 'condition': {'rgb_noise': 16},
                           'attack_effect_evaluable': False,
                           'clean': copy.deepcopy(clean), 'attacked': copy.deepcopy(attacked)}]}
    return evidence, report


def test_bounds_samples_preserves_outcomes_metrics_and_originals(tmp_path):
    evidence, report = fixture(); original = copy.deepcopy((evidence, report))
    reduced = compact_report_evidence(evidence, report, tmp_path)
    row = reduced['rounds'][0]
    assert row['attacked']['outcome'] == 'collision'
    assert row['attacked']['metrics'] == evidence['rounds'][0]['attacked']['metrics']
    assert row['attack_effect_evaluable'] is False
    details = row['attacked']['diagnostics']
    assert details['retention']['source'] == 'available_summary_snapshots'
    assert details['retention']['selected_samples'] <= 3
    assert details['retention']['minimum_scope'] == 'sparse available summary samples'
    assert (evidence, report) == original


def test_required_results_are_never_silently_discarded(tmp_path):
    evidence, report = fixture()
    evidence['rounds'][0]['condition']['mandatory_context'] = 'x' * 2000
    with pytest.raises(ValueError, match='Required report settings/results'):
        compact_report_evidence(evidence, report, tmp_path, byte_budget=1000)


@pytest.mark.parametrize('mismatch', ['outcome', 'metrics', 'duplicate', 'missing'])
def test_mismatched_source_identity_rejected(tmp_path, mismatch):
    evidence, report = fixture()
    if mismatch == 'outcome':
        evidence['rounds'][0]['attacked']['outcome'] = 'route_completed'
    elif mismatch == 'metrics':
        evidence['rounds'][0]['attacked']['metrics'] = {}
    elif mismatch == 'duplicate':
        evidence['rounds'].append(copy.deepcopy(evidence['rounds'][0]))
    else:
        evidence['rounds'] = []
    with pytest.raises(ValueError, match='source'):
        compact_report_evidence(evidence, report, tmp_path)
