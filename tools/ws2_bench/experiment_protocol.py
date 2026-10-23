"""Validate the scientific purpose of an agent's next paired experiment.

The model chooses the purpose; these checks prevent misleading comparisons.
Threshold probes are empirical brackets, not a monotonicity assumption.
"""
import copy
from mission import SUCCESSES, EXCLUDED

KINDS = ('explore', 'repeat', 'isolate', 'threshold')
FACTORS = {'rgb_noise': ('rgb_noise_stddev',), 'delay': ('delay_s',),
           'patch': ('patch_enabled', 'patch_size_m', 'patch_start_s', 'patch_duration_s')}


def schema():
    return {'type': 'object', 'additionalProperties': False,
            'required': ['kind', 'source_round', 'passing_round', 'factor'],
            'properties': {
                'kind': {'type': 'string', 'enum': list(KINDS)},
                'source_round': {'type': 'integer', 'minimum': 0},
                'passing_round': {'type': 'integer', 'minimum': 0},
                'factor': {'type': 'string', 'enum': ['none', *FACTORS]}}}


def active(a):
    return [f for f, keys in FACTORS.items() if a.get(keys[0],0) > 0]


def disable(a, factor):
    if factor == 'patch':
        a.update(patch_enabled=False, patch_start_s=0., patch_duration_s=0.)
    else:
        a[FACTORS[factor][0]] = 0.


def attack_evidence_eligible(action, pair):
    """An executed passing control and verified scheduled exposure are needed.

    Legacy studies predate captured-frame exposure fields and remain readable.
    New v2 trials fail closed when that measurement is missing or invalid.
    Eligibility supports a comparison, not a claim of attack causality.
    """
    if 'factor_exposure' in pair:
        from factor_exposure import VERSION, assess
        saved = pair['factor_exposure']
        return (isinstance(saved, dict) and saved.get('schema') == VERSION
                and saved.get('eligible') is True and assess(action, pair)['eligible'])
    if any(pair[role].get('executed') is False for role in ('clean','perturbed')):return False
    if pair['clean']['outcome'] not in SUCCESSES or pair['perturbed']['outcome'] in EXCLUDED:return False
    exposure=pair['perturbed'].get('attack_exposure')
    if exposure is None and action.get('action_space_version')!='corridor_agent_v2':return True
    return isinstance(exposure,dict) and exposure.get('exposure_verified') is True


def validate(experiment, action, history, action_key):
    if not isinstance(experiment, dict) or set(experiment) != {'kind', 'source_round', 'passing_round', 'factor'}:
        raise ValueError('experiment requires kind, source_round, passing_round and factor')
    kind, source, passing, factor = (experiment[k] for k in ('kind', 'source_round', 'passing_round', 'factor'))
    if kind not in KINDS or factor not in ('none', *FACTORS):
        raise ValueError('Unknown experiment kind/factor')
    if any(type(n) is not int or n < 0 for n in (source, passing)):
        raise ValueError('Evidence round IDs must be nonnegative integers')
    rounds = {r['decision']['round']: r for r in history}
    if kind == 'explore':
        if source or passing or factor != 'none':
            raise ValueError('Exploration has source_round=passing_round=0 and factor=none')
        if action_key(action) in {action_key(r['decision']['action']) for r in history}:
            raise ValueError('Use repeat for a previously tested configuration')
        return experiment
    if source not in rounds:
        raise ValueError('Follow-up source_round must refer to an executed round')
    previous = rounds[source]
    base = previous['decision']['action']
    pair = previous['pairs'][0]
    if kind == 'repeat':
        if passing or factor != 'none' or action_key(action) != action_key(base):
            raise ValueError('Repeat must preserve the source configuration, factor=none, passing_round=0')
        return experiment
    if pair['clean']['outcome'] not in SUCCESSES or pair['perturbed']['outcome'] in EXCLUDED:
        raise ValueError('Isolation/threshold require an executed pair with a passing clean control')
    if not attack_evidence_eligible(base,pair):
        raise ValueError('Isolation/threshold require verified attack exposure in the source pair')
    if kind == 'isolate':
        if passing or len(active(base)) < 2 or factor not in active(base):
            raise ValueError('Isolation retains ONE active factor from a combined pair; passing_round=0')
        expected = copy.deepcopy(base)
        for other in FACTORS:
            if other != factor:
                disable(expected, other)
        if action_key(action) != action_key(expected):
            raise ValueError('Isolation must keep geometry/light and the chosen factor unchanged, disable other attacks')
        return experiment
    if factor not in ('rgb_noise', 'delay') or factor not in active(base):
        raise ValueError('Threshold must vary an active noise or delay scalar; patch size is not calibrated strength')
    if pair['perturbed']['outcome'] in SUCCESSES:
        raise ValueError('Threshold source_round must be a failed attack with a passing clean control')
    key = FACTORS[factor][0]
    if passing == 0:
        if active(base) != [factor]:
            raise ValueError('A conditional threshold needs a measured passing_round with the other factors unchanged; the clean control disables all factors')
        low = 0.  # The source pair's clean control is the measured zero endpoint.
    else:
        if passing not in rounds:
            raise ValueError('passing_round is not in executed history')
        passed = rounds[passing]; pp = passed['pairs'][0]
        if any(pp[role]['outcome'] not in SUCCESSES for role in ('clean', 'perturbed')):
            raise ValueError('passing_round must have both flights pass')
        if not attack_evidence_eligible(passed['decision']['action'],pp):
            raise ValueError('passing_round requires verified attack exposure')
        pa = copy.deepcopy(passed['decision']['action']); low = pa[key]; pa[key] = base[key]
        if action_key(pa) != action_key(base):
            raise ValueError('Threshold endpoints must differ only in the selected scalar')
    expected = copy.deepcopy(base); expected[key] = action[key]
    if action_key(expected) != action_key(action) or not min(low, base[key]) < action[key] < max(low, base[key]):
        raise ValueError('Threshold must probe strictly between observed pass/fail endpoints, changing only its scalar')
    return experiment


INSTRUCTIONS = (
    'Choose experiment.kind as explore, repeat, isolate, or threshold. This is YOUR choice, not a fixed repeat rule. '
    'explore: new configuration, source_round=passing_round=0, factor=none. '
    'repeat: exact source action, source_round set, passing_round=0, factor=none. '
    'isolate: from a valid combined pair, keep exactly one active factor and disable the others, keep geometry/light unchanged; passing_round=0. '
    'threshold: source is a failed noise or delay pair with successful clean control; change ONLY that scalar strictly between '
    'the source failing value and a measured passing_round value. Other factors may remain active ONLY if the passing endpoint '
    'has exactly those same other factors/geometry/light. passing_round=0 means the source clean control, valid ONLY for a single-factor source. '
    'A combined-factor bracket is conditional on the fixed other factors, not a universal single-factor threshold. '
    'Do not assume monotonic failure probability or treat patch size as calibrated strength. '
    'Clean failure is environment/baseline evidence: the attack twin is skipped. Explore another layout or explicitly repeat the baseline. '
    'For new versioned trials, factor_exposure eligibility requires each active factor to have measured application and observed worker-consumed input evidence; patch visibility is separate. '
    'For historical scheduled records without factor_exposure, attack_exposure.exposure_verified=true is the legacy eligibility check. '
    'A flight ending before the planned window, or missing exposure telemetry, cannot establish an attack outcome. '
    'Use compact pre-failure diagnostics to distinguish sensor age, model output and motion observations; do not claim a proven internal cause. '
    'Spend remaining pairs on the most informative follow-up; all repeats/isolation/probes use the same budget. '
    'After the budget, the report may suggest future work but those suggestions are not additional authorized flights.'
)
