"""Checks for outcome contrasts that change one sensor-window start.

This module does not select tests or change historical outcomes. Equal configured
amplitude and pulse duration do not prove equal delivered exposure, encounter
overlap, reproducibility, or an internal failure mechanism.
"""
from __future__ import annotations

import copy
import math

from conditions import validate_schedule
from corridor_space import FIELDS, VERSION as ACTION_VERSION
from factor_exposure import assess as assess_exposure
from mission import SUCCESSES

VERSION = 'sensor_timing_contrast_v1'
SENSORS = {'rgb_noise': ('rgb_noise_stddev', 'noise_schedule'),
           'delay': ('delay_s', 'delay_schedule')}
LIMITATION = (
    'Matched configured amplitude and pulse duration, not measured equal total dose. '
    'Termination and periodic pulse counts can differ. Encounter overlap requires '
    'separate sampled evidence. Two outcomes do not establish a reproducible effect '
    'or an internal causal mechanism.'
)


def match_actions(first, second):
    """Recognize canonical v2 actions differing only in ONE active start_s.

    Every other action field must match exactly, including inactive settings.
    Callers supply previously bench-validated actions; this checks matching and
    schedules, not a second complete geometry/action feasibility validation.
    Zero-duration windows last until termination and are excluded because a
    changed start then changes the configured exposure duration. A repeating
    schedule matches only at the pulse-parameter level, not total pulse count.
    """
    result = {'schema': VERSION, 'matched': False, 'reason': None,
              'causal_validation': False, 'limitation': LIMITATION}
    if (not isinstance(first, dict) or not isinstance(second, dict)
            or set(first) != FIELDS or set(second) != FIELDS
            or first.get('action_space_version') != ACTION_VERSION
            or second.get('action_space_version') != ACTION_VERSION):
        return dict(result, reason='requires_canonical_v2_actions')
    changed = []
    for factor, (amplitude_key, schedule_key) in SENSORS.items():
        try:
            left = validate_schedule(first[schedule_key])
            right = validate_schedule(second[schedule_key])
        except (ValueError, TypeError):
            return dict(result, reason='invalid_sensor_schedule')
        if left != right:
            changed.append((factor, amplitude_key, schedule_key, left, right))
    if len(changed) != 1:
        return dict(result, reason='requires_exactly_one_changed_sensor_schedule')
    factor, amplitude_key, schedule_key, left, right = changed[0]
    if left['start_s'] == right['start_s']:
        return dict(result, reason='start_time_did_not_change')
    if any(left[key] != right[key] for key in ('duration_s', 'period_s')):
        return dict(result, reason='pulse_duration_or_period_changed')
    if left['duration_s'] == 0:
        return dict(result, reason='open_ended_window_changes_exposure_duration')
    amplitude = first[amplitude_key]
    if (type(amplitude) not in (int, float) or not math.isfinite(amplitude)
            or amplitude <= 0):
        return dict(result, reason='changed_factor_is_not_active')
    expected = copy.deepcopy(first)
    expected[schedule_key]['start_s'] = right['start_s']
    if expected != second:
        return dict(result, reason='other_action_fields_changed')
    return dict(result, matched=True, reason=None, factor=factor,
                amplitude=amplitude, start_s=[left['start_s'], right['start_s']],
                duration_s=left['duration_s'], period_s=left['period_s'],
                single_finite_window=left['period_s'] == 0,
                equal_delivered_exposure_established=False)


def assess(first_action, first_pair, second_action, second_pair):
    """Attach eligible measured outcomes to a structurally matched contrast.

    Both complete pairs need passing clean controls and per-factor exposure.
    Patch co-exposure, when configured, needs the patch verifier's attestation;
    a sensor-only aggregate cannot substitute for patch visibility.
    """
    result = match_actions(first_action, second_action)
    result.update(eligible=False, exposure_checks=[], outcomes=None,
                  outcome_changed=None, completed_vs_failed=None,
                  encounter_overlap_verified=False)
    if not result['matched']:
        return result
    checks = [assess_exposure(first_action, first_pair),
              assess_exposure(second_action, second_pair)]
    result['exposure_checks'] = checks
    if not all(check['eligible'] for check in checks):
        result['reason'] = 'pair_or_active_factor_evidence_ineligible'
        return result
    outcomes = [first_pair['perturbed']['outcome'], second_pair['perturbed']['outcome']]
    result.update(eligible=True, reason=None, outcomes=outcomes,
                  outcome_changed=outcomes[0] != outcomes[1],
                  completed_vs_failed=(outcomes[0] in SUCCESSES) != (outcomes[1] in SUCCESSES))
    return result


def history_contrasts(history):
    """Enumerate matched timing comparisons, never independent discovery counts."""
    comparisons = []
    for index, second in enumerate(history):
        for first in history[:index]:
            a, b = first['decision']['action'], second['decision']['action']
            if not match_actions(a, b)['matched']:
                continue
            value = assess(a, first['pairs'][0], b, second['pairs'][0])
            comparisons.append({'rounds': [first['decision']['round'], second['decision']['round']],
                                **value, 'shared_run_comparisons_are_not_independent': True})
    return comparisons
