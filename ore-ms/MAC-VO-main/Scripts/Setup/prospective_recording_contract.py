"""Read-only prospective benchmark readiness; declared evidence is not authentication.

No recording payloads are opened, assignments created, or test runs launched.
Unknown checkpoint overlap restricts the network-held-out claim, not a scoped
project benchmark. Physical accuracy requires its own source-backed contract.
"""
import argparse
import hashlib
import json
from pathlib import Path


def assess(plan, ledger):
    records = {r['id']: r for r in ledger['records']}
    aliases = {}
    for record in records.values():
        for name in [record['id'], *record['aliases']]:
            if name in aliases:
                raise ValueError('ambiguous recording identity')
            aliases[name] = record['id']
    evidence = plan['evidence_files']

    def backed(path):
        digest = evidence.get(path) if isinstance(path, str) else None
        return isinstance(digest, str) and len(digest) == 64 and all(c in '0123456789abcdef' for c in digest)

    issues = []
    development = {aliases.get(x, x) for x in plan['partitions']['development']}
    validation = {aliases.get(x, x) for x in plan['partitions']['validation']}
    if development & validation:
        issues.append('development_validation_recording_overlap')
    candidates = plan['candidate_test_records']
    canonical = []
    for candidate in candidates:
        name = candidate['recording_id']
        identity = aliases.get(name)
        if identity is None:
            issues.append(name + ':unclassified_recording_identity')
            continue
        canonical.append(identity)
        record = records[identity]
        if record['unit_kind'] != 'temporal_recording':
            issues.append(identity + ':not_temporal_recording')
        if record['exposure_status'] == 'development_exposed' or identity in development | validation:
            issues.append(identity + ':development_or_validation_exposure')
        access = candidate['project_exposure']
        if access['pixels'] != 'unexamined_attested' or not backed(access.get('evidence_ref')):
            issues.append(identity + ':project_pixel_exposure_not_established')
        if access['references'] != 'unexamined_attested' or not backed(access.get('evidence_ref')):
            issues.append(identity + ':project_reference_exposure_not_established')
    if len(canonical) != len(set(canonical)):
        issues.append('duplicate_recording_or_alias')
    if len(set(canonical)) < 2:
        issues.append('fewer_than_two_independent_recordings')
    assignment = [aliases.get(x, x) for x in plan['partitions']['test']]
    if sorted(assignment) != sorted(canonical) or len(assignment) != len(set(assignment)):
        issues.append('exact_test_assignment_not_frozen')
    adapter = plan['adapter']
    if adapter['preservation_verified'] is not True or not backed(adapter.get('evidence_ref')) or not adapter.get('entrypoint'):
        issues.append('dataset_temporal_adapter_preservation_not_verified')
    reference = plan['reference']
    if reference['benchmark_target_verified'] is not True or not backed(reference.get('evidence_ref')):
        issues.append('source_benchmark_target_not_verified')
    inputs = plan['test_input_metadata']
    if inputs['frozen'] is not True or not backed(inputs.get('manifest_ref')):
        issues.append('exact_test_input_metadata_not_frozen')
    if (type(plan['sampling']['frames_per_record']) is not int or
            plan['sampling']['frames_per_record'] != 100 or type(plan['sampling']['start']) is not int or plan['sampling']['start'] != 0 or
            plan['sampling']['rule'] != 'first_100_official_paired_groups_no_outcome_filter'):
        issues.append('sampling_differs_from_declared_rule')
    runtime = plan['method']['runtime_image_id']
    checkpoints = plan['method']['checkpoint_sha256']
    if (plan['method']['parameters_frozen'] is not True or not plan['method']['code_sha256'] or
            not isinstance(runtime, str) or not runtime.startswith('sha256:') or len(runtime) != 71 or
            not all(c in '0123456789abcdef' for c in runtime[7:]) or not checkpoints or
            not all(isinstance(d, str) and len(d) == 64 and all(c in '0123456789abcdef' for c in d) for d in checkpoints.values())):
        issues.append('method_artifacts_not_frozen')
    if plan['decision_policy'] != 'group_lag_one_final_before_drain':
        issues.append('controller_policy_changed')
    if plan['methods'] != ['raw', 'history', 'shadow', 'rgb_only', 'thermal_only']:
        issues.append('declared_controls_changed')
    if plan['scope']['physical_motion_output'] is not False or plan['scope']['production_adoption'] is not False:
        issues.append('unsupported_output_or_adoption_scope')
    ready = not issues
    membership = plan['checkpoint_membership']
    training_heldout = ready and all(
        membership.get(c, {}).get('status') == 'recording_disjoint_verified' and
        backed(membership[c].get('evidence_ref')) for c in plan['method']['checkpoint_sha256'])
    physical = ready and reference['physical_independent_pose_verified'] is True and backed(reference.get('physical_evidence_ref'))
    return {'status': 'READY_SCOPED_BENCHMARK' if ready else 'INCOMPLETE_PROSPECTIVE_CONTRACT',
            'candidate_recordings': canonical, 'benchmark_execution_ready': ready,
            'missing_requirements': issues, 'project_benchmark_claim_supported': ready,
            'checkpoint_training_heldout_claim_supported': training_heldout,
            'physical_accuracy_claim_supported': physical,
            'membership_unknown_does_not_alone_block_scoped_benchmark': True,
            'test_payloads_opened': False, 'assignments_created': False,
            'test_executed': False, 'physical_motion_admitted': False,
            'scope': 'artifact/identity readiness only; evidence contents/origin require source review; no superiority outcome assessed'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('root', 'plan', 'ledger', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    ledger = json.loads(args.ledger.read_text())
    for group in (plan['method']['code_sha256'], plan['evidence_files']):
        for relative, digest in group.items():
            path = (args.root / relative).resolve()
            if not path.is_relative_to(args.root.resolve()) or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError('artifact hash/path mismatch: ' + relative)
    result = assess(plan, ledger)
    result.update(plan_sha256=hashlib.sha256(args.plan.read_bytes()).hexdigest(),
                  ledger_sha256=hashlib.sha256(args.ledger.read_bytes()).hexdigest(),
                  checker_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2); stream.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
