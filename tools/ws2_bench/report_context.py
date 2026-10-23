"""Bound report diagnostics without changing measured outcomes or source files."""
from __future__ import annotations

import copy

from agent_context import compact_diagnostics, encoded_bytes

VERSION = 'bounded_report_context_v1'


def compact_report_evidence(evidence, report, runtime_root, byte_budget=160_000):
    """Retain exact rounds/settings/metrics; reduce only diagnostic detail.

    The original report remains the scoring authority. Trace samples may be
    sparse or unavailable, which the shared diagnostic reducer marks explicitly.
    This is an opt-in input transformation for a new report protocol version.
    """
    if type(byte_budget) is not int or byte_budget <= 0:
        raise ValueError('Report context byte budget must be positive')
    result = copy.deepcopy(evidence)
    sources = {f"round_{pair['round']:02d}": pair for pair in report['paired_comparisons']}
    ids = [row['id'] for row in result['rounds']]
    if (len(sources) != len(report['paired_comparisons']) or
            len(set(ids)) != len(ids) or set(ids) != set(sources)):
        raise ValueError('Report context must reference every unique source round exactly once')
    for row in result['rounds']:
        source = sources[row['id']]
        for name, role in [('clean', 'clean'), ('attacked', 'perturbed')]:
            original = source[role]
            if any(row[name].get(k) != original.get(k) for k in ('outcome', 'metrics')):
                raise ValueError('Report context outcome/metrics differ from the referenced source run')
            run = {'result_dir': original.get('directory'),
                   'outcome': original.get('outcome'),
                   'diagnostics': original.get('diagnostics') or {}}
            row[name]['diagnostics'] = compact_diagnostics(
                run, f"{row['id']}_{role}", runtime_root)
    result['context_retention'] = {
        'version': VERSION, 'byte_budget': byte_budget,
        'exact_settings_outcomes_metrics_and_round_ids_retained': True,
        'original_report_and_trace_files_unchanged': True,
        'detailed_round_ids': [r['id'] for r in result['rounds']],
        'limitation': 'Selected diagnostic samples do not establish a causal mechanism.'}
    for keep in dict.fromkeys((len(result['rounds']), 4, 2, 0)):
        cutoff = max(0, len(result['rounds'])-keep)
        for row in result['rounds'][:cutoff]:
            for role in ('clean', 'attacked'):
                row[role].pop('diagnostics', None)
                row[role]['diagnostics_omitted_from_prompt'] = True
        result['context_retention']['detailed_round_ids'] = [r['id'] for r in result['rounds'][cutoff:]]
        if encoded_bytes(result) <= byte_budget:
            return result
    raise ValueError('Required report settings/results exceed the context byte budget; '
                     'preserve the completed campaign and increase the explicit bound. '
                     'Measured results cannot be silently omitted.')
