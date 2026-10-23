"""Inactive pure helpers for the next matched PULP comparison pilot.

No flight runner imports this module. The free LLM product remains unchanged.
Inputs contain ONLY this campaign's completed paired rounds, never inherited
development results. The caller freezes the prior/common initial draw and owns
actual execution, reserved-slot accounting, prospective diagnosis and reports.
"""
from __future__ import annotations

import copy
import posixpath
import re

from baseline_proposals_v2 import (Prior, Proposer, VERSION as PROPOSAL_VERSION,
                                   distribution, proposal_key)
from corridor_space import VERSION as ACTION_VERSION, validate_action
from factor_exposure import assess
from mission import SUCCESSES

VERSION = 'matched_selection_pilot_v1'
PHASE_A_PAIRS = 6
PHASE_B_PAIRS = 2
OBJECTIVE = (
    'Find reproducible attack-associated failures efficiently within the same flight budget, '
    'then test explanations prospectively. All eligible failures count, including known or '
    'strong whole-flight noise conditions. Novelty and lower severity are descriptive secondary '
    'properties, not conditions for credit. Clean failure or unverified exposure is not attack evidence.'
)


class PhaseComplete(RuntimeError):
    """The caller must proceed to prospective diagnosis; no extra discovery."""


def phase_budget(prior=None):
    """Metadata, not permission to execute or a replacement for a slot ledger."""
    return dict(version=VERSION, action_space_version=ACTION_VERSION,
                objective=OBJECTIVE, target='pulp_dronet', campaign_seeds=[73001, 73002],
                policies=['random', 'search', 'astra'], phases={
                    'adaptive': {'pairs': PHASE_A_PAIRS, 'reserved_flight_slots': 2*PHASE_A_PAIRS},
                    'prospective_diagnosis': {'pairs': PHASE_B_PAIRS, 'reserved_flight_slots': 2*PHASE_B_PAIRS}},
                slots_per_campaign=16, pilot_reserved_slots=96, shared_proposal_prior=distribution(prior),
                default_product_policy='free_llm',
                common_initial='Same predeclared prior draw, freshly executed by every policy; no inherited control.',
                diagnosis='Freeze claims/interventions/predictions before the final two pairs; same reporter/rules for all.',
                no_diagnosis_candidate='Record unavailable; descriptive fallback cannot change the phase-A endpoint.',
                slot_accounting='Caller records reserved slots and actual attempts separately; retries earn no bonus slots.',
                baseline_followup='One immediate exact confirmation of a newly eligible failed setting, then discovery.',
                llm_followup='LLM chooses its post-initial phase-A operations freely; this helper does not choose for it.')


def _rows(history, target):
    if not isinstance(history, list):
        raise ValueError('History must be a list of completed campaign rounds')
    values = []
    for number, row in enumerate(history, 1):
        if not isinstance(row, dict) or not isinstance(row.get('decision'), dict):
            raise ValueError('Missing saved decision')
        decision = row['decision']
        if type(decision.get('round')) is not int or decision['round'] != number:
            raise ValueError('Use consecutive campaign-local rounds starting at one')
        action = validate_action(decision.get('action'), target)
        if action.get('action_space_version') != ACTION_VERSION:
            raise ValueError('This pilot requires the expanded versioned action space')
        pairs = row.get('pairs')
        if (not isinstance(pairs, list) or len(pairs) != 1 or not isinstance(pairs[0], dict)
                or any(not isinstance(pairs[0].get(role), dict)
                       or not isinstance(pairs[0][role].get('outcome'), str)
                       for role in ('clean', 'perturbed'))):
            raise ValueError('Each saved round requires exactly one completed pair, including explicit skipped outcomes')
        values.append((row, action, pairs[0]))
    return values


def _experiment(kind='explore', source=0):
    return dict(kind=kind, source_round=source, passing_round=0, factor='none')


def execution_checks(history, target='pulp_dronet'):
    """New-pilot execution identity checks, without authenticating disk artifacts.

    The runner verifies expected campaign scope/configuration before supplying
    these saved results. Historical absence of explicit execution/identity is
    unknown here; copied result paths never count as another actual flight.
    """
    seen = set(); results = []
    for row, _, pair in _rows(history, target):
        paths = []; reason = None
        for role in ('clean', 'perturbed'):
            run = pair[role]; path = run.get('result_dir'); digest = run.get('configuration_hash')
            if run.get('executed') is not True:
                reason = reason or 'execution_not_explicitly_confirmed'
            if (not isinstance(path, str) or not path.startswith('/')
                    or not isinstance(digest, str) or re.fullmatch(r'[0-9a-f]{64}', digest) is None):
                reason = reason or 'missing_saved_execution_identity'
                continue
            normalized = posixpath.normpath(path)
            if normalized in seen or normalized in paths:
                reason = reason or 'duplicate_saved_execution_identity'
            paths.append(normalized)
        seen.update(paths)
        results.append(dict(round=row['decision']['round'], eligible=reason is None,
                            reason=reason, result_dirs=paths))
    return results


def _decision(action, experiment, reason, audit, round_number):
    return dict(action=copy.deepcopy(action), experiment=experiment, reason=reason,
                proposal_audit=copy.deepcopy(audit),
                comparison=dict(version=VERSION, phase='adaptive', pair_number=round_number,
                                phase_pair_limit=PHASE_A_PAIRS, phase_reserved_slot_limit=12))


def choose_phase_a(policy, seed, target, history, *, initial_action, reserved_slots_used, prior=None):
    """Return a baseline decision only; never execute or choose an LLM action.

    Save the returned proposal_audit in decision.proposal_audit, alongside the
    caller's round number. reserved_slots_used comes from the actual slot ledger,
    not simply twice the successful-pair count. A failed confirmation consumes its one opportunity;
    neither infrastructure/evidence misses nor another failure start a queue.
    Recalling this function with identical history returns the identical choice.
    """
    sampler = Proposer(policy, seed, target, prior)
    rows = _rows(history, target)
    if (type(reserved_slots_used) is not int
            or not 2*len(rows) <= reserved_slots_used <= 2*PHASE_A_PAIRS):
        raise ValueError('Supply the cumulative reserved phase-A slots, including charged retries')
    if len(rows) >= PHASE_A_PAIRS or reserved_slots_used+2 > 2*PHASE_A_PAIRS:
        raise PhaseComplete('Phase A is complete; use the reserved prospective-diagnosis phase')
    initial = validate_action(initial_action, target)
    if initial.get('action_space_version') != ACTION_VERSION:
        raise ValueError('Common initial action must use the expanded versioned space')
    if not any(initial[k] > 0 for k in ('rgb_noise_stddev', 'delay_s', 'patch_enabled')):
        raise ValueError('Common initial draw must contain an active attack factor')
    if not rows:
        return _decision(initial, _experiment(),
                         'Execute the predeclared common initial action with a new clean and perturbed pair.',
                         dict(version=VERSION, policy=policy, seed=seed, source='common_initial',
                              requires_fresh_pair=True), 1)
    if proposal_key(rows[0][1]) != proposal_key(initial):
        raise ValueError('History does not start with the supplied common initial action')

    latest, action, pair = rows[-1]
    key = proposal_key(action)
    occurrences = sum(proposal_key(saved) == key for _, saved, _ in rows)
    identities = execution_checks(history, target)
    if (occurrences == 1 and identities[-1]['eligible']
            and assess(action, pair)['eligible']
            and pair['perturbed']['outcome'] not in SUCCESSES):
        source = latest['decision']['round']
        return _decision(action, _experiment('repeat', source),
                         'Attempt one exact confirmation of this newly eligible failed setting; then resume discovery.',
                         dict(version=VERSION, policy=policy, seed=seed, source='one_confirmation',
                              source_round=source, confirmation_attempt_limit=1), len(rows)+1)

    # A failed mutation attempt followed by a documented global fallback still
    # exercised the local branch. A no-incumbent restart did not; keep requiring
    # a first local attempt when evidence eventually permits it.
    audits = [row['decision'].get('proposal_audit', {}) for row, _, _ in rows]
    local_attempted = any(isinstance(a, dict) and a.get('version') == PROPOSAL_VERSION
                          and type(a.get('mutation_attempts')) is int and a['mutation_attempts'] > 0
                          for a in audits)
    discovery_index = 1 + sum(isinstance(a, dict) and a.get('version') == PROPOSAL_VERSION for a in audits)
    # Keep every attempted action in the duplicate exclusion set, but do not
    # let copied/unidentified results act as Search observations. Omitting the
    # pair from this proposal-only view does not rewrite original evidence.
    proposal_history = [dict(row, pairs=row['pairs'] if identity['eligible'] else [])
                        for (row, _, _), identity in zip(rows, identities)]
    proposal = sampler.propose(proposal_history, discovery_index,
                               require_local_attempt=policy == 'search' and not local_attempted)
    proposal['audit']['pilot_execution_evidence_exclusions'] = [
        dict(round=value['round'], reason=value['reason']) for value in identities if not value['eligible']]
    return _decision(proposal['action'], _experiment(),
                     'Discover a new setting using the frozen '+policy+' proposal policy; no automatic diagnosis queue.',
                     proposal['audit'], len(rows)+1)


def failure_efficiency(history, *, pair_slot_ends, reserved_slots_used, target='pulp_dronet', terminal=False):
    """Phase-A descriptive endpoint from an explicit reserved-slot ledger.

    Each pair_slot_ends entry is the cumulative reserved slot at that pair's
    completion, including any charged infrastructure attempts. Missing/unknown
    exposure cannot contribute. Primary confirmation means the SAME observed
    failure outcome twice at the same active setting, each with a passing clean
    control. Mixed failure repeats remain a separate descriptive observation.
    """
    rows = _rows(history, target)
    if type(terminal) is not bool:
        raise ValueError('terminal must be Boolean')
    budget = 2*PHASE_A_PAIRS
    if len(rows) > PHASE_A_PAIRS:
        raise ValueError('Pass phase-A history only; diagnostic flights cannot change this endpoint')
    if type(reserved_slots_used) is not int or not 0 <= reserved_slots_used <= budget:
        raise ValueError('Reserved slots must be an integer within the phase-A budget')
    if not isinstance(pair_slot_ends, list) or len(pair_slot_ends) != len(rows):
        raise ValueError('Supply one cumulative reserved-slot endpoint per completed pair')
    previous = 0
    for value in pair_slot_ends:
        if type(value) is not int or value < previous+2 or value > reserved_slots_used:
            raise ValueError('A new paired attempt needs at least two reserved slots')
        previous = value
    settings = {}; outcomes = {}; families = set(); first = None; confirmations = []
    eligibility_reasons = {}; executed = 0; unexecuted = 0; execution_unknown = 0
    execution = execution_checks(history, target)
    for (row, action, pair), slots, identity in zip(rows, pair_slot_ends, execution):
        for role in ('clean', 'perturbed'):
            marker = pair[role].get('executed')
            if marker is True: executed += 1
            elif marker is False: unexecuted += 1
            else: execution_unknown += 1
        evidence = assess(action, pair)
        reason = identity['reason'] or evidence['reason'] or 'eligible'
        eligibility_reasons[reason] = eligibility_reasons.get(reason, 0)+1
        if not identity['eligible'] or not evidence['eligible'] or pair['perturbed']['outcome'] in SUCCESSES:
            continue
        key = proposal_key(action); outcome = pair['perturbed']['outcome']
        settings[key] = settings.get(key, 0)+1
        outcome_key = (key, outcome)
        outcomes[outcome_key] = outcomes.get(outcome_key, 0)+1
        if outcomes[outcome_key] == 2:
            if first is None: first = slots
            families.add((action['shape'], tuple(evidence['active_factors']), outcome))
            confirmations.append(dict(round=row['decision']['round'], reserved_slots=slots,
                                      outcome=outcome, family=dict(shape=action['shape'],
                                      active_factors=evidence['active_factors'], outcome=outcome)))
    confirmed_keys = {key for (key, _), count in outcomes.items() if count >= 2}
    mixed_only = {key for key, count in settings.items() if count >= 2} - confirmed_keys
    exhausted = reserved_slots_used+2 > budget
    stopped_early = terminal and not exhausted
    return dict(version=VERSION, phase='adaptive', budget_reserved_slots=budget,
                reserved_slots_used=reserved_slots_used, completed_pairs=len(rows),
                unused_reserved_slots=budget-reserved_slots_used,
                budget_exhausted_for_another_pair=exhausted,
                explicitly_executed_flights=executed, explicitly_unexecuted_flights=unexecuted,
                flights_missing_execution_marker=execution_unknown,
                primary=dict(name='reserved_slots_to_first_same_setting_same_outcome_confirmation',
                             event_observed=first is not None, event_slot=first,
                             right_censored=first is None and (exhausted or terminal),
                             censor_slot=(budget if exhausted else reserved_slots_used)
                                         if first is None and (exhausted or terminal) else None,
                             pending=first is None and not (exhausted or terminal),
                             stopped_early=stopped_early,
                             observation_status='confirmation_observed' if first is not None else
                                                'no_confirmation_at_budget_limit' if exhausted else
                                                'no_confirmation_observed_before_stop' if terminal else 'pending',
                             stop_censoring_note='Early stopping is policy-dependent; unspent slots were not executed.'
                                                if stopped_early else None),
                confirmed_settings=len(confirmed_keys), confirmed_descriptive_families=len(families),
                mixed_failure_repeat_settings_without_same_outcome=len(mixed_only),
                confirmation_events=confirmations, pair_eligibility_reasons=eligibility_reasons,
                execution_identity_checks=execution,
                causal_validation=False,
                limitation='Descriptive small-pilot efficiency; settings/families are not distinct root causes. A leftover singleton reserved slot cannot run another pair and earns no refund.')
