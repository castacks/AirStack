"""Protocol-version selection for new versus resumed campaigns.

Missing historical fields stay missing. This helper does not rewrite config or
history files, migrate past decisions, or relax the runner's exact resume check.
"""
from __future__ import annotations

import copy

from agent_context import VERSION as CONTEXT_VERSION
from factor_exposure import VERSION as EVALUATION_VERSION
from planning_contract import VERSION as PLANNING_VERSION
from encounter_context import VERSION as ENCOUNTER_CONTEXT_VERSION

KEYS = ('planning_version', 'context_version', 'evaluation_version', 'encounter_context_version')
LEGACY_PLANNING_VERSION = 'visual_experiment_planning_v1'
CURRENT = {'planning_version': PLANNING_VERSION, 'context_version': CONTEXT_VERSION,
           'evaluation_version': EVALUATION_VERSION, 'encounter_context_version': ENCOUNTER_CONTEXT_VERSION}
SUPPORTED = {'planning_version': {LEGACY_PLANNING_VERSION, PLANNING_VERSION},
             'context_version': {CONTEXT_VERSION}, 'evaluation_version': {EVALUATION_VERSION},
             'encounter_context_version': {ENCOUNTER_CONTEXT_VERSION}}


def _validated_versions(config):
    if not isinstance(config, dict):
        raise ValueError('Campaign config must be an object')
    values = {key: config[key] for key in KEYS if key in config}
    for key, value in values.items():
        if not isinstance(value, str) or value not in SUPPORTED[key]:
            raise ValueError(f'Unsupported campaign {key}: {value!r}')
    return values


def apply(config, previous=None, *, supports_planning=False):
    """Return config with frozen historical or explicit new protocol versions.

    Call with the saved config before the runner compares all non-budget keys.
    Every NEW corridor policy gets the same exposure evaluation version. Only
    an agent with a planning-capable provider also gets planning/decision-context
    versions. Old unversioned campaigns are never silently upgraded. Bounded
    report context records its own version in the report's context_retention;
    it is not the decision-context version pinned here.
    """
    requested = _validated_versions(config)
    if type(supports_planning) is not bool:
        raise ValueError('supports_planning must be a Boolean')
    eligible = (config.get('action_space') == 'corridor' and config.get('backend') == 'agent_search'
                and supports_planning)
    result = copy.deepcopy(config)
    if previous is not None:
        recorded = _validated_versions(previous)
        if any(key not in recorded or recorded[key] != value for key, value in requested.items()):
            raise ValueError('Cannot change or add protocol versions when resuming a campaign')
        if ('planning_version' in recorded or 'encounter_context_version' in recorded) and not eligible:
            raise ValueError('Versioned planning resume requires the corridor agent and a planning-capable provider')
        for key in KEYS:
            result.pop(key, None)
        result.update(recorded)
        return result
    if config.get('action_space') != 'corridor':
        if requested:
            raise ValueError('Current protocol versions apply to corridor campaigns')
        return result
    if not eligible and ('planning_version' in requested or 'encounter_context_version' in requested):
        raise ValueError('Planning protocol versions require a planning-capable corridor agent')
    if any(CURRENT[key] != value for key, value in requested.items()):
        raise ValueError('New planning campaigns use the current protocol; legacy versions are resume-only')
    result['evaluation_version'] = EVALUATION_VERSION
    if eligible:
        result.update(CURRENT)
    return result


def planning_version(config):
    """Dispatch missing historical versions to unchanged v1 behavior."""
    values = _validated_versions(config)
    return values.get('planning_version', LEGACY_PLANNING_VERSION)
