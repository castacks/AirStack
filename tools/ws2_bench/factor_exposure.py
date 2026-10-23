"""Eligibility rules for measured exposure to each active factor.

The sensor audit covers RGB noise and delivery delay. Its aggregate Boolean
does not measure a physical patch's visibility. A patch visibility attestation
must come from a bench verifier, never the LLM. This function checks that
attestation's contract; it does not read/render images or verify their hashes.
Existing outcomes and historical files are never changed by this module.
"""
from __future__ import annotations

import math
import re

from mission import SUCCESSES

VERSION = 'factor_exposure_eligibility_v1'
HASH = re.compile(r'[0-9a-f]{64}')
PERFORMANCE_OUTCOMES = SUCCESSES | {'collision', 'insufficient_progress', 'timeout', 'planner_stopped'}
CONSUMED_SCHEMA = 'observed_consumed_input_v1'
CONSUMED_POPULATION = 'observed unique worker-consumed source-frame updates'


def _number(value):
    return type(value) in (int, float) and math.isfinite(value) and value >= 0


def _hash(value):
    return isinstance(value, str) and HASH.fullmatch(value) is not None


def _consumed(factor, raw):
    """Sampled input updates, not every inference or the network's full state."""
    missing = {'verified': False, 'reason': 'worker_consumed_factor_not_verified',
               'measurement_scope': CONSUMED_POPULATION}
    if (not isinstance(raw, dict) or raw.get('schema') != CONSUMED_SCHEMA
            or raw.get('status') != 'complete' or raw.get('population') != CONSUMED_POPULATION
            or raw.get('configuration_checked') is not True):
        return missing
    total = raw.get('observed_unique_consumed_frames')
    counts = raw.get('factor_counts'); sources = raw.get('evidence_sources')
    key = {'rgb_noise': 'noise', 'delay': 'delay'}[factor]
    values = counts.get(key) if isinstance(counts, dict) else None
    if (type(total) is not int or total <= 0 or not isinstance(values, dict)
            or set(values) != {'active', 'inactive', 'unknown'}
            or any(type(value) is not int or value < 0 for value in values.values())
            or sum(values.values()) != total or not isinstance(sources, dict)
            or set(sources) != {'direct_worker_metadata', 'same_run_exact_frame_join', 'unavailable'}
            or any(type(value) is not int or value < 0 for value in sources.values())
            or sum(sources.values()) != total
            or values['active'] > sources['direct_worker_metadata']+sources['same_run_exact_frame_join']):
        return missing
    verified = values['active'] > 0
    return dict(missing, verified=verified,
                reason='observed_consumed_active_source_frames' if verified else 'no_observed_consumed_active_frames',
                active_frames=values['active'], inactive_frames=values['inactive'], unknown_frames=values['unknown'],
                observed_unique_frames=total, exact_consumption_or_delivery_latency_verified=False,
                limitation='Observed source-frame updates do not describe every inference or all temporal model state.')


def _sensor(factor, exposure, consumed):
    key = {'rgb_noise': 'noise', 'delay': 'delay'}[factor]
    if not isinstance(exposure, dict):
        return {'verified': False, 'status': 'unknown', 'reason': 'missing_sensor_exposure_audit'}
    # The aggregate covers every sensor in the legacy audit, including inactive
    # factors. It cannot establish OR veto this active factor's own evidence.
    # A global evidence error is different: the audit itself is not trusted.
    if exposure.get('status') == 'evidence_error':
        return {'verified': False, 'status': 'unknown', 'reason': 'sensor_audit_evidence_error'}
    factors = exposure.get('factors')
    raw = factors.get(key) if isinstance(factors, dict) else None
    if not isinstance(raw, dict):
        return {'verified': False, 'status': 'unknown', 'reason': 'missing_active_factor_coverage'}
    count = raw.get('active_frames')
    applied = (raw.get('requested') is True and raw.get('exposure_verified') is True
               and raw.get('status') == 'exposed' and type(count) is int and count > 0)
    worker = _consumed(factor, consumed)
    verified = applied and worker['verified']
    return {'verified': verified, 'status': 'exposed' if verified else 'unknown',
            'reason': 'applied_and_observed_consumed_factor' if verified else
                      'active_factor_application_not_verified' if not applied else worker['reason'],
            'application_verified': applied,
            'application_active_frames': count if type(count) is int and count >= 0 else None,
            'worker_input': worker,
            'measurement_scope': 'Applied source-camera factor AND observed worker-consumed source-frame updates'}


def _patch(raw):
    """Accept only an explicit, frame-bound rendered-visibility attestation.

    The future upstream verifier must validate files, hashes, exact patch prim
    identity and synchronized RGB/segmentation timestamps before setting these
    flags. Projection/in-frustum checks alone cannot rule out occlusion.
    FCRN depth changes are NOT required to establish visibility.
    """
    missing = {'verified': False, 'status': 'unknown', 'reason': 'patch_visibility_not_verified',
               'causal_validation': False}
    if not isinstance(raw, dict):
        return missing
    if (raw.get('status') != 'visible' or raw.get('visibility_verified') is not True
            or raw.get('method') != 'rendered_instance_segmentation'
            or raw.get('source_validation') != 'bench_verified'
            or not isinstance(raw.get('patch_prim_path'), str) or not raw['patch_prim_path'].startswith('/')
            or not _hash(raw.get('patch_texture_sha256'))):
        return missing
    frames = raw.get('frames')
    if not isinstance(frames, list) or not frames:
        return missing
    verified = []
    for frame in frames:
        if not isinstance(frame, dict):
            return missing
        count = frame.get('visible_patch_pixels')
        if (frame.get('patch_enabled') is not True or frame.get('frame_binding_verified') is not True
                or frame.get('worker_consumption_verified') is not True
                or type(count) is not int or count <= 0 or not _number(frame.get('input_stamp_sim_s'))
                or not _hash(frame.get('rgb_sha256')) or not _hash(frame.get('segmentation_sha256'))):
            return missing
        verified.append((frame['input_stamp_sim_s'], frame['rgb_sha256'], frame['segmentation_sha256']))
    return {'verified': True, 'status': 'visible', 'reason': 'bench_verified_rendered_patch_pixels',
            'visible_frames': len(set(verified)), 'method': raw['method'], 'causal_validation': False,
            'limitation': 'Visible patch pixels do not establish a depth change or causal attack effect.'}


def assess(action, pair):
    """Assess a canonical corridor action and pair without mutating either.

    Store this pair-level verdict in pair['factor_exposure'] only after both
    flight results exist. It depends on the clean control, so it is not a raw
    measurement attached to the perturbed flight. Raw sensor/visibility audits
    stay on that flight under attack_exposure, worker_input_exposure and
    patch_visibility. The consumed-input audit must have checked the executed
    scenario condition; merely released bridge frames cannot verify consumption.

    Legacy missing exposure is reported as unknown. A caller reproducing an
    old study may preserve its original historical score, but must not describe
    that score as newly verified per-factor exposure.
    """
    result = {'schema': VERSION, 'eligible': False, 'reason': None, 'active_factors': [],
              'factor_evidence': {}, 'causal_validation': False}
    if not isinstance(action, dict) or not isinstance(pair, dict):
        return dict(result, reason='invalid_input')
    for key in ('rgb_noise_stddev', 'delay_s'):
        if not _number(action.get(key, 0)):
            return dict(result, reason='invalid_attack_configuration')
    if type(action.get('patch_enabled', False)) is not bool:
        return dict(result, reason='invalid_attack_configuration')
    if action.get('depth_noise', 0) != 0 or action.get('depth_noise_stddev_m', 0) != 0:
        return dict(result, reason='unsupported_depth_perturbation')
    factors = [name for name, key in (('rgb_noise', 'rgb_noise_stddev'), ('delay', 'delay_s'),
                                     ('patch', 'patch_enabled')) if action.get(key, 0) > 0]
    result['active_factors'] = factors
    clean, attacked = pair.get('clean'), pair.get('perturbed')
    if not isinstance(clean, dict) or not isinstance(attacked, dict):
        return dict(result, reason='missing_pair_result')
    if not factors:
        return dict(result, reason='all_off_baseline')
    exposure = attacked.get('attack_exposure')
    for factor in factors:
        result['factor_evidence'][factor] = (_patch(attacked.get('patch_visibility')) if factor == 'patch'
                                              else _sensor(factor, exposure, attacked.get('worker_input_exposure')))
    if any('executed' in run and type(run['executed']) is not bool for run in (clean, attacked)):
        reason = 'invalid_execution_marker'
    elif any(run.get('executed') is False for run in (clean, attacked)):
        reason = 'unexecuted_flight'
    elif clean.get('outcome') not in SUCCESSES:
        reason = 'clean_control_failed'
    elif not isinstance(attacked.get('outcome'), str) or attacked['outcome'] not in PERFORMANCE_OUTCOMES:
        reason = 'excluded_or_missing_outcome'
    elif not all(value['verified'] for value in result['factor_evidence'].values()):
        reason = 'active_factor_exposure_unverified'
    else:
        reason = None
    return dict(result, eligible=reason is None, reason=reason)


def assess_condition(condition, pair):
    """Report-side adapter; conditions use rgb_noise/delay rather than action keys."""
    if not isinstance(condition, dict):
        return assess(None, pair)
    return assess({'rgb_noise_stddev': condition.get('rgb_noise', 0),
                   'delay_s': condition.get('delay', 0),
                   'patch_enabled': condition.get('patch_enabled', False),
                   'depth_noise': condition.get('depth_noise', 0)}, pair)
