"""No filesystem, provider, simulator or notification side effects."""
import copy

import pytest

from campaign_versions import apply, planning_version, CURRENT, KEYS, LEGACY_PLANNING_VERSION


def config(**updates):
    return {'backend': 'agent_search', 'action_space': 'corridor', 'budget': 4, **updates}


def test_new_planning_agent_is_versioned_without_mutating_input():
    source = config(); original = copy.deepcopy(source)
    result = apply(source, supports_planning=True)
    assert {key: result[key] for key in KEYS} == CURRENT
    assert source == original


@pytest.mark.parametrize('updates,supports', [({'action_space': 'saved'}, True),
    ({'action_space': 'expanded'}, True)])
def test_non_corridor_new_campaigns_remain_unversioned(updates, supports):
    source = config(**updates)
    assert apply(source, supports_planning=supports) == source


@pytest.mark.parametrize('backend,supports', [('random', True), ('random', False),
    ('search', True), ('search', False), ('agent_search', False)])
def test_all_new_corridor_policies_share_evaluation_without_forced_llm_planning(backend, supports):
    source = config(backend=backend)
    result = apply(source, supports_planning=supports)
    assert result == {**source, 'evaluation_version': CURRENT['evaluation_version']}
    assert 'planning_version' not in result and 'context_version' not in result


def test_legacy_resume_missing_versions_stays_byte_identical_except_requested_budget():
    previous = config(); requested = config(budget=8)
    saved = copy.deepcopy(previous)
    result = apply(requested, previous, supports_planning=True)
    assert result == requested and previous == saved
    assert not any(key in result for key in KEYS)
    assert planning_version(result) == LEGACY_PLANNING_VERSION


def test_versioned_resume_carries_exact_recorded_versions():
    previous = apply(config(), supports_planning=True)
    result = apply(config(budget=8), previous, supports_planning=True)
    assert result == {**previous, 'budget': 8}
    assert planning_version(result) == CURRENT['planning_version']


@pytest.mark.parametrize('backend', ['random', 'search'])
def test_baseline_evaluation_only_resume_does_not_require_a_planning_provider(backend):
    previous = apply(config(backend=backend), supports_planning=False)
    result = apply(config(backend=backend, budget=8), previous, supports_planning=False)
    assert result == {**previous, 'budget': 8}
    assert set(result).intersection(KEYS) == {'evaluation_version'}


@pytest.mark.parametrize('backend', ['random', 'search'])
def test_old_baseline_resumes_without_retroactive_evaluation_upgrade(backend):
    previous = config(backend=backend)
    result = apply(config(backend=backend, budget=8), previous, supports_planning=False)
    assert result == {**previous, 'budget': 8} and not set(result).intersection(KEYS)


def test_partial_legacy_fields_are_not_backfilled():
    previous = config(planning_version=LEGACY_PLANNING_VERSION)
    result = apply(config(), previous, supports_planning=True)
    assert result == previous and 'context_version' not in result and 'evaluation_version' not in result


@pytest.mark.parametrize('key', KEYS)
def test_unknown_recorded_or_requested_version_is_rejected(key):
    with pytest.raises(ValueError, match='Unsupported campaign'):
        apply(config(), config(**{key: 'unknown'}), supports_planning=True)
    with pytest.raises(ValueError, match='Unsupported campaign'):
        apply(config(**{key: 'unknown'}), supports_planning=True)


def test_resume_cannot_silently_add_or_change_protocol():
    with pytest.raises(ValueError, match='Cannot change or add'):
        apply(config(**CURRENT), config(), supports_planning=True)
    previous = config(planning_version=LEGACY_PLANNING_VERSION)
    with pytest.raises(ValueError, match='Cannot change or add'):
        apply(config(planning_version=CURRENT['planning_version']), previous, supports_planning=True)


def test_versioned_resume_requires_same_capability_and_new_legacy_pin_is_rejected():
    previous = apply(config(), supports_planning=True)
    with pytest.raises(ValueError, match='planning-capable'):
        apply(config(), previous, supports_planning=False)
    with pytest.raises(ValueError, match='resume-only'):
        apply(config(planning_version=LEGACY_PLANNING_VERSION), supports_planning=True)


def test_other_resume_mismatches_remain_for_runners_exact_check():
    previous = apply(config(seed=42), supports_planning=True)
    changed = apply(config(seed=99), previous, supports_planning=True)
    assert changed['seed'] == 99 and changed != previous



def test_historical_v2_without_encounter_version_is_never_upgraded_on_resume():
    previous = config(**{key: value for key, value in CURRENT.items() if key != 'encounter_context_version'})
    resumed = apply(config(budget=8), previous, supports_planning=True)
    assert resumed == {**previous, 'budget': 8}
    assert 'encounter_context_version' not in resumed
    with pytest.raises(ValueError, match='Cannot change or add'):
        apply(config(encounter_context_version=CURRENT['encounter_context_version']), previous,
              supports_planning=True)


@pytest.mark.parametrize('backend,supports', [('random', True), ('search', True), ('agent_search', False)])
def test_encounter_projection_version_requires_planning_for_new_campaign(backend, supports):
    with pytest.raises(ValueError, match='planning-capable'):
        apply(config(backend=backend, encounter_context_version=CURRENT['encounter_context_version']),
              supports_planning=supports)
