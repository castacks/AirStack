#!/usr/bin/env python3
"""M00-D offline repository, evidence, graph, protocol and status validation."""
import ast
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / 'RRM'
EVIDENCE = WORK / 'evidence/M00-D'
REPORTS = WORK / 'verification/reports/M00-D'
checks = []


def git(*args):
    return subprocess.check_output(['git', '-C', str(ROOT), *args], text=True).rstrip('\n')


def doc(path):
    return json.loads(path.read_text())


def rows(path):
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check(name, passed, detail):
    checks.append({'check': name, 'passed': bool(passed), 'detail': detail})


def identity(path):
    if path.is_symlink():
        return {'type': 'symlink', 'target': os.readlink(path)}
    if path.is_file():
        return {'type': 'file', 'sha256': sha(path), 'mode': stat.S_IMODE(path.stat().st_mode)}
    if path.is_dir():
        return {'type': 'gitlink', 'head': subprocess.check_output(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()}
    return {'type': 'missing'}


def main():
    before = doc(EVIDENCE / 'repository-before.json')
    head = git('rev-parse', 'HEAD')
    check('repository_branch_start', git('remote', 'get-url', 'origin') == before['repository'] ==
          'https://github.com/castacks/AirStack.git' and git('branch', '--show-current') ==
          before['branch'] == 'ore_proj' and before['head'] ==
          'b4ac9467b073b6ae7927a47b939f46e9407cb6c0' and before['status_porcelain'] == '',
          'Required clean M00-C start; existing branch retained')
    check('history_not_rewritten', head == before['head'] or git('rev-parse', 'HEAD^') == before['head'],
          'At parent or one direct M00-D successor')
    allowed = set(before['authorized_mutable_prior_files'])
    drift = [p for p, value in before['tracked_entries'].items()
             if p not in allowed and identity(ROOT / p) != value]
    check('all_protected_tracked_files_preserved', not drift and allowed == {'RRM/PROJECT_STATUS.md'},
          {'initial_entries': len(before['tracked_entries']), 'authorized_exception': sorted(allowed), 'changed_protected': drift})
    initial = doc(WORK / 'evidence/M00/repository-before.json')
    reference = {str(p.relative_to(ROOT)): identity(p) for p in (ROOT / 'RRM-transfer').rglob('*')
                 if p.is_file() or p.is_symlink()}
    check('complete_reference_tree_preserved', reference == initial['protected_tree'],
          {'entries': len(reference), 'limit': 'Does not attest ignored runtime trees or submodule internal worktrees'})
    outside = [s for s in git('status', '--porcelain=v1', '--untracked-files=all').splitlines()
               if not s[3:].startswith('RRM/')]
    check('outside_workspace_unchanged', not outside and not git('diff', before['head'], '--', '.', ':(exclude)RRM'),
          {'outside_status': outside})
    package = doc(WORK / 'evidence/M00/controlled-package-manifest.json')
    base = Path(package['supplied_path'])
    actual = {str(p.relative_to(base)): {'path': str(p.relative_to(base)), 'bytes': p.stat().st_size,
                                       'sha256': sha(p)} for p in base.rglob('*') if p.is_file()}
    check('controlled_package_unchanged', actual == {p['path']: p for p in package['files']},
          {'files': len(actual)})
    source = doc(EVIDENCE / 'source-manifest.json')['sources']
    bad = [s['path'] for s in source if sha(ROOT / s['path']) != s['sha256']]
    check('source_evidence_hashes', not bad, {'sources': len(source), 'bad': bad})
    historic_bad = []
    historic_count = 0
    for line in (WORK / 'evidence/M00-C/SHA256SUMS').read_text().splitlines():
        expected, path = line.split('  ', 1)
        blob = subprocess.check_output(['git', '-C', str(ROOT), 'show', before['head'] + ':' + path])
        historic_count += 1
        if hashlib.sha256(blob).hexdigest() != expected:
            historic_bad.append(path)
    check('sealed_m00c_manifest_against_pinned_git_blobs', not historic_bad,
          {'entries': historic_count, 'bad': historic_bad, 'status_snapshot_rule': 'Historical status hash resolved at M00-C commit; current status intentionally updated'})
    check('starting_integrity_no_discrepancy', doc(EVIDENCE / 'starting-integrity.json')['preexisting_discrepancies'] == [] and
          all(c['exit_code'] == 0 for c in doc(EVIDENCE / 'starting-integrity.json')['checks']),
          'Prior checksum/replay passed before authorized status update')
    p = subprocess.run([sys.executable, '-B', str(WORK / 'scripts/m00d_characterize.py'), '--check'],
                       cwd=ROOT, capture_output=True, text=True)
    check('independent_timeline_replay', p.returncode == 0, {'exit_code': p.returncode, 'stderr': p.stderr})
    metrics = rows(EVIDENCE / 'latency-measurements.csv')
    summary = doc(EVIDENCE / 'characterization-summary.json')
    check('clock_domains_n1_and_no_false_variability', len(metrics) == 19 and
          summary['independent_deadline_interval_s'] == '19.276162179' and
          summary['startup_episode_count'] == 1 and summary['new_startup_trials'] == 0 and
          summary['measured_characterization_intervals'] == 12 and summary['missing_measurements'] == 7 and
          all(not m['seconds'] for m in metrics if m['status'] == 'MISSING') and
          not summary['acceptance_thresholds_created'], '12 characterizations/7 missing; producer, sink and kernel clock semantics retained; no distribution from n=1')
    spec = importlib.util.spec_from_file_location('m00d_graph', WORK / 'scripts/m00d_characterize.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    graph = doc(EVIDENCE / 'dependency-graph.json')
    order = module.graph_order(graph)
    edgeids = {x['id'] for x in graph['edges']}
    nodeids = set(order)
    check('dependency_graph_consistent', len(order) == 17 and len(edgeids) == len(graph['edges']) == 19 and
          all(all(edge[key] for key in ['expected_prerequisite', 'observed_readiness_signal',
                                       'timing_evidence', 'failure_propagation', 'relation']) for edge in graph['edges']),
          {'nodes': len(order), 'edges': len(edgeids), 'cycle_free': True})
    hypotheses = rows(REPORTS / 'M00D_HYPOTHESIS_MATRIX.csv')
    instrumentation = rows(EVIDENCE / 'instrumentation-catalog.csv')
    instids = {i['instrumentation_id'] for i in instrumentation}
    metricids = {m['measurement_id'] for m in metrics}
    prior_hyp_ids = {h['hypothesis_id'] for h in rows(WORK / 'verification/reports/M00-C/M00C_HYPOTHESIS_MATRIX.csv')}
    check('hypotheses_traceable_and_discriminating', len(hypotheses) == 8 and len(instids) == 6 and all(
          set(h['m00c_hypotheses'].split(';')) <= prior_hyp_ids and
          set(h['dependency_nodes'].split(';')) <= nodeids and
          set(h['latency_ids'].split(';')) <= metricids and
          set(h['instrumentation_ids'].split(';')) <= instids and
          all(h[k] for k in ['future_observation_supporting', 'future_observation_refuting',
                             'missing_measurements', 'confidence', 'contradictory_or_limiting_evidence']) for h in hypotheses),
          'Eight hypotheses trace to original hypotheses, graph nodes, measurements and proposed instrumentation')
    old = rows(WORK / 'evidence/M00-C/M00C_BLOCKER_DISPOSITION.csv')
    new = rows(EVIDENCE / 'M00D_BLOCKER_DISPOSITION.csv')
    check('all_original_blockers_and_closure_fields_preserved', len(old) == len(new) == 10 and all(
          all(a[k] == b[k] for k in a) and b['closed'] == 'NO' and b['m00d_disposition']
          for a, b in zip(old, new)), 'Ten original IDs, authorities, scope and closure conditions retained; no unsupported closure')
    missing = []
    for edge in graph['edges']:
        if not (ROOT / edge['timing_evidence']).is_file():
            missing.append(edge['timing_evidence'])
    for h in hypotheses:
        for name in h['evidence_reference'].split(';'):
            if not (ROOT / name).is_file():
                missing.append(name)
    check('graph_hypothesis_evidence_references_resolve', not missing, {'missing': missing})
    protocol = (WORK / 'verification/procedures/M00D_CONTROLLED_REPRODUCTION.md').read_text()
    status = (WORK / 'PROJECT_STATUS.md').read_text()
    roadmap = (WORK / 'ROADMAP_MANDATE.md').read_text()
    check('protocol_authorization_and_design_controls', all(s.lower() in protocol.lower() for s in
          ['Tier B NOT AUTHORIZED', 'Tier C OUT OF SCOPE', 'WP01 NOT AUTHORIZED',
           'PROPOSED', '24 startups', 'six matched repetitions', 'Abort', 'Clock', 'Preconditions',
           'NOT EXECUTED', 'not execution-ready', 'SetParameters', 'independent', 'RA01']),
          'Exact candidate/unknown prerequisites, three tiers, repeat design, clocks, instrumentation, abort/retention and PD authority explicit')
    check('roadmap_status_consistency', 'M00-D' in status and before['head'] in status and
          all(s in status for s in ['WP01 NOT AUTHORIZED', 'Tier B NOT AUTHORIZED',
                                    'Tier C OUT OF SCOPE', 'PENDING PD REVIEW', 'PROPOSED',
                                    '28 in-scope NOT_TESTED', '1 OUT_OF_SCOPE', 'M00-E-TIERB-PREP']) and
          'RA01 — PENDING PD REVIEW' in roadmap and all(b['blocker_id'] in status for b in new),
          'Status current/history/coverage/ten blockers/RA01 and Tier A-only authority agree with roadmap and new directive')
    tests = (EVIDENCE / 'diagnostic-tests.txt').read_text()
    check('offline_regression_test_results', 'Ran 17 tests' in tests and tests.rstrip().endswith('OK'),
          '17 historical/synthetic offline graph/clock/schema tests; all pass; no functional or qualification claim')
    forbidden_imports = {'subprocess', 'os', 'socket', 'requests', 'rclpy', 'rclcpp', 'ctypes'}
    unsafe = []
    for name in ['m00d_characterize.py', 'm00d_validate_events.py']:
        tree = ast.parse((WORK / 'scripts' / name).read_text())
        imports = {n.module.split('.')[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
        imports |= {a.name.split('.')[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        calls = {getattr(n.func, 'id', getattr(n.func, 'attr', '')) for n in ast.walk(tree) if isinstance(n, ast.Call)}
        if imports & forbidden_imports or calls & {'eval', 'exec', '__import__', 'system', 'popen'}:
            unsafe.append(name)
    audit = doc(EVIDENCE / 'diagnostic-scope-audit.json')
    check('offline_tool_safety_and_scope_audit', not unsafe and audit['tier_executed'] == 'A' and
          audit['startup_trials_executed'] == 0 and audit['active_operations_executed'] == [],
          {'unsafe_diagnostic_modules': unsafe, 'limit': 'No runtime APIs in new diagnostic modules; agent action inventory not exhaustive external monitoring'})
    before_runtime = doc(EVIDENCE / 'passive-metadata-before.json')['commands'][0]
    after_runtime = doc(EVIDENCE / 'passive-metadata-after.json')
    check('container_metadata_unchanged', before_runtime['exit_code'] == after_runtime['exit_code'] == 0 and
          before_runtime['stdout'] == after_runtime['stdout'],
          'Images/container StartedAt/restart counts stable; not a claim about internal process state or new readiness samples')
    files = (EVIDENCE / 'M00D_CHANGED_FILES.txt').read_text().splitlines()
    broken = []
    for name in files:
        path = ROOT / name
        if path.suffix == '.py':
            ast.parse(path.read_text())
        if path.suffix == '.md':
            for link in re.findall(r'\[[^\]]*\]\(([^)]+)\)', path.read_text()):
                if '://' not in link and not link.startswith('#') and not (path.parent / link.split('#')[0]).exists():
                    broken.append({'file': name, 'link': link})
    check('document_links_and_python_syntax', not broken, {'broken': broken})
    manifest = doc(EVIDENCE / 'M00D_EVIDENCE_MANIFEST.json')
    bad = [x['path'] for x in manifest['artifacts'] if sha(ROOT / x['path']) != x['sha256'] or
           (ROOT / x['path']).stat().st_size != x['bytes']]
    check('evidence_manifest_complete_and_intact', not bad and set(files) - set(manifest['excluded_tracked_paths']) ==
          {x['path'] for x in manifest['artifacts']}, {'artifacts': len(manifest['artifacts']), 'bad': bad})
    p = subprocess.run(['sha256sum', '-c', 'RRM/evidence/M00-D/SHA256SUMS'], cwd=ROOT,
                       text=True, capture_output=True)
    check('detached_sha256_checks', p.returncode == 0,
          {'entries_ok': p.stdout.count(': OK'), 'stderr': p.stderr})
    staged = git('diff', '--cached', '--name-only').splitlines()
    check('staged_scope_explicit', (not staged or set(staged) == set(files)) and all(
          f.startswith('RRM/') and not f.startswith('RRM/RRM-C001-F00-EXT/') for f in files),
          {'authorized_files': len(files), 'staged_files': len(staged)})
    p = subprocess.run(['git', '-C', str(ROOT), 'diff', '--cached', '--check'], text=True, capture_output=True)
    check('staged_whitespace', p.returncode == 0, p.stdout + p.stderr)
    if head != before['head']:
        committed = git('diff-tree', '--no-commit-id', '--name-only', '-r', head).splitlines()
        check('final_commit_scope_and_status_together', set(committed) == set(files) and
              'RRM/PROJECT_STATUS.md' in committed, {'commit': head, 'files': len(committed)})
    passed = all(c['passed'] for c in checks)
    print(json.dumps({'all_passed': passed, 'scope': 'M00-D offline evidence/plan/project-control integrity; not Tier B execution or F00 qualification', 'checks': checks}, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except Exception as error:
        print(json.dumps({'all_passed': False, 'error': str(error), 'checks': checks}, indent=2))
        sys.exit(1)
