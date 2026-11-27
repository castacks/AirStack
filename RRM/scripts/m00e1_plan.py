#!/usr/bin/env python3
"""Offline deterministic proposed trial schedule; cannot execute or authorize any trial."""
import argparse
import csv
import io
import json
from pathlib import Path
import random

ROOT = Path(__file__).resolve().parents[2]
REPORT = ROOT / 'RRM/verification/reports/M00-E1/M00E1_TRIAL_MATRIX.csv'
SEED = 20261127
HYPOTHESES = ';'.join('HD%02d' % n for n in range(1, 9))
INSTRUMENTATION = ';'.join('I%02d' % n for n in range(1, 7))


def schedule():
    rng = random.Random(SEED)
    blocks = [(cache, rep) for cache in ['FRESH_ISOLATED_CACHE', 'PINNED_PREWARMED_CACHE']
              for rep in range(1, 7)]
    rng.shuffle(blocks)
    trials = []
    for pair_number, (cache, repetition) in enumerate(blocks, 1):
        policies = ['P0_CONCURRENT_BASELINE', 'P1_DEPENDENCY_AWARE_CANDIDATE']
        rng.shuffle(policies)
        for position, policy in enumerate(policies, 1):
            trial_number = len(trials) + 1
            trial_id = 'M00E2-TB-%03d' % trial_number
            trials.append({
                'trial_id': trial_id, 'order': str(trial_number), 'matched_pair': 'PAIR-%02d' % pair_number,
                'pair_position': str(position), 'cache_condition': cache, 'cell_repetition': str(repetition),
                'launch_policy': policy, 'configuration_id': 'CFG-M00E2-STARTUP-01-PROPOSED',
                'environment_id': 'ENV-M00E2-ISOLATED-01-PROPOSED', 'schedule_seed': str(SEED),
                'paired_scene_seed': str(1000 + pair_number),
                'objective': 'Characterize startup phases and test ordering/cache associations at unchanged guard semantics; exploratory only',
                'controlled_change': 'P0 retains concurrent interface startup; P1 defers guard construction until reviewed fresh dependency predicate. Cache class is a separate blocked factor',
                'hypotheses': HYPOTHESES, 'required_instrumentation': INSTRUMENTATION,
                'required_observations': 'Workflow/container/scene/graph/first-Play/spawn/exec/TCP/UDP/heartbeat/MAVROS/DDS/guard-origin-callback-transaction-latch and resource/native-clock records',
                'expected_discrimination': 'Pair P0/P1 within cache block; inspect per-phase/callback/resource timelines before attributing mechanism; preserve alternative explanations',
                'required_preconditions': 'CHK01;CHK02;CHK03;CHK04;CHK05;CHK06;CHK07;CHK08;CHK09;CHK10;CHK11;CHK12;CHK13;CHK14;CHK15;CHK16;CHK17;CHK18',
                'active_operation_ids': 'OP01;OP02;OP03;OP04;OP05;OP06;OP07',
                'authorization': 'SEPARATE_PD_AUTHORIZATION_REQUIRED', 'execution_status': 'NOT_EXECUTED',
                'requirement_verdict': 'NOT_ASSESSED_EXPLORATORY',
                'evidence_directory': '<APPROVED_EXTERNAL_EVIDENCE_ROOT>/M00E2/<AUTH_ID>/CFG-M00E2-STARTUP-01/<TRIAL_ID>/ATTEMPT-001'.replace('<TRIAL_ID>', trial_id),
                'failure_retention': 'Retain failed, aborted, incomplete and setup attempts with original trial ID; no replacement or overwritten attempt. Unscheduled startup needs amended authority'})
    return trials


def prepared_decision_valid(decision):
    if decision.get('package_state') != 'PREPARED — NOT AUTHORIZED FOR EXECUTION':
        raise ValueError('Prepared package must not imply execution authority')
    if decision.get('pd_decision') != 'PENDING' or decision.get('pd_signature') is not None:
        raise ValueError('No approval or signature may be inferred by this preparation tool')
    pending_fields = ['pd_name', 'decision_utc', 'approval_reference', 'rejection_reason',
                      'approved_environment', 'approved_candidate_manifest',
                      'approved_command_manifest', 'approved_operational_limits', 'authority_expiry']
    if any(decision.get(field) is not None for field in pending_fields):
        raise ValueError('E1 decision fields must remain blank pending an actual PD decision')
    if decision.get('execution_authorized') is not False or decision.get('requested_maximum_startups') != 24:
        raise ValueError('Preparation cannot authorize execution or change the proposed startup cap')
    return {'package_state': decision['package_state'], 'execution_authorized': False,
            'scope': 'Offline document consistency only; no authority to start any service'}


def render():
    records = schedule()
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=list(records[0]), lineterminator='\n')
    writer.writeheader()
    writer.writerows(records)
    return stream.getvalue()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    text = render()
    if args.check:
        if REPORT.read_text() != text:
            raise ValueError('Proposed schedule differs from frozen generator output')
    else:
        with REPORT.open('x') as stream:
            stream.write(text)
    print(json.dumps({'trials_proposed': len(schedule()), 'trials_executed': 0,
                      'execution_authorized': False, 'schedule_seed': SEED}))


if __name__ == '__main__':
    main()
