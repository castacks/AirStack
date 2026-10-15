#!/usr/bin/env python3
"""Explicit missing-case continuation; no resampling, deployment or dispatch."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

sys.path.insert(0, str(Path(__file__).resolve().parent))
import independent_intent_probe as probe


def error_record(error):
    """Retain structured cause without URLs, response bodies or credentials."""
    result = {'error_type': type(error).__name__}
    if isinstance(error, HTTPError):
        result['http_status'] = error.code
    elif isinstance(error, URLError):
        result['reason_type'] = type(error.reason).__name__
        if isinstance(getattr(error.reason, 'errno', None), int):
            result['reason_errno'] = error.reason.errno
    return result


def read_json(path):
    return json.loads(path.read_bytes())


def load_previous(previous):
    """Verify original artifacts/source and select only cases with no response."""
    manifest = read_json(previous / 'manifest.json')
    for name, digest in manifest['artifacts'].items():
        path = (previous / name).resolve()
        if not path.is_relative_to(previous.resolve()) or probe.sha(path.read_bytes()) != digest:
            raise ValueError('previous_artifact_integrity')
    for required in ('report.json', 'corpus.json', 'neutral.png', 'worker-before.json', 'worker-after.json'):
        if required not in manifest['artifacts']:
            raise ValueError('previous_manifest_missing_required_artifact')
    if manifest['source'] != probe.source_identity():
        raise ValueError('previous_inference_source_changed')
    corpus = read_json(previous / 'corpus.json')
    if corpus != probe.specification():
        raise ValueError('previous_corpus_changed')
    report = read_json(previous / 'report.json')
    if (report['corpus_sha256'] != probe.sha(probe.canonical(corpus))
            or report.get('execution_dispatch') is not False
            or report.get('source_unchanged') is not True
            or report.get('worker_capabilities_unchanged') is not True
            or report['summary'] != probe.summarise(report['records'])):
        raise ValueError('previous_report_contract')
    if len(report['records']) != len(corpus['cases']):
        raise ValueError('previous_record_count')
    image = (previous / 'neutral.png').read_bytes()
    if image != probe.neutral_png():
        raise ValueError('previous_image_changed')
    selected = []
    for index, (case, record) in enumerate(zip(corpus['cases'], report['records'])):
        if (record['case_id'] != case['case_id'] or record['adjudication'] != case['adjudication']
                or record.get('execution_dispatch') is not False
                or record['status'] not in ('RESPONDED', 'ERROR', 'NOT_RUN')):
            raise ValueError('previous_case_identity')
        request_name = case['case_id'] + '/request.json'
        assessment_name = case['case_id'] + '/assessment.json'
        if request_name not in manifest['artifacts'] or assessment_name not in manifest['artifacts']:
            raise ValueError('previous_missing_case_artifact')
        request = read_json(previous / request_name)
        expected = probe.build_request(case, image, index)
        expected['cycle_id'] = request['cycle_id']
        if (request != expected or record['request_sha256'] != probe.sha(probe.canonical(request))
                or read_json(previous / assessment_name) != record):
            raise ValueError('previous_request_or_record_changed')
        if record['status'] == 'RESPONDED':
            response_name = case['case_id'] + '/response.json'
            if response_name not in manifest['artifacts']:
                raise ValueError('previous_missing_response')
            response = read_json(previous / response_name)
            probe.validate_response(request, response)
            if probe.canonical(record['assessment']) != probe.canonical(probe.assess(case, response)):
                raise ValueError('previous_assessment_changed')
        else:
            selected.append((index, case, record, request))
    before = read_json(previous / 'worker-before.json')
    if before != read_json(previous / 'worker-after.json'):
        raise ValueError('previous_worker_identity_changed')
    return corpus, report, before, selected


def endpoint_get(url):
    request = Request(url, headers={'Accept': 'application/json'})
    with urlopen(request, timeout=10) as response:
        raw = response.read(4 * 1024 * 1024 + 1)
        if response.status != 200 or len(raw) > 4 * 1024 * 1024:
            raise ValueError('qualification_response_contract')
    return json.loads(raw)


def qualify(url, expected):
    checks = []
    for index in range(3):
        entry = dict(index=index, at_utc=datetime.now(timezone.utc).isoformat())
        started = time.monotonic()
        try:
            health = endpoint_get(url + '/healthz')
            capabilities = endpoint_get(url + '/v1/capabilities')
            entry.update(health=health, capabilities=capabilities)
            if (health.get('status') != 'ready' or health.get('execution_dispatch') is not False
                    or capabilities != expected or capabilities.get('execution_dispatch') is not False
                    or '/v1/propose' not in capabilities.get('routes', [])):
                raise ValueError('qualification_identity_or_dispatch_contract')
            entry['status'] = 'PASS'
        except Exception as error:
            entry.update(status='ERROR', **error_record(error))
        entry['wall_s'] = time.monotonic() - started
        checks.append(entry)
        if entry['status'] != 'PASS':
            break
    return dict(passed=len(checks) == 3 and all(c['status'] == 'PASS' for c in checks), checks=checks)


def run(output, url, previous):
    probe.CosmosWorkerClient(url, timeout_s=60)
    corpus, prior_report, expected_caps, selected = load_previous(previous)
    output.mkdir(parents=True, exist_ok=False)
    sources = probe.source_identity()
    sources['scripts/continue_intent_probe.py'] = probe.sha(Path(__file__).read_bytes())
    probe.save(output / 'corpus.json', corpus)
    probe.save(output / 'parent.json', dict(report_sha256=probe.sha((previous / 'report.json').read_bytes()),
        manifest_sha256=probe.sha((previous / 'manifest.json').read_bytes()), summary=prior_report['summary'],
        selected_case_ids=[case['case_id'] for _, case, _, _ in selected]))
    qualification = qualify(url, expected_caps)
    probe.save(output / 'qualification.json', qualification)
    records = []
    stopped = not qualification['passed']
    for index, case, old_record, old_request in selected:
        folder = output / case['case_id']
        folder.mkdir()
        request = probe.build_request(case, probe.neutral_png(), index)
        if request['cycle_id'] == old_request['cycle_id']:
            raise ValueError('cycle_id_not_fresh')
        probe.save(folder / 'request.json', request)
        record = dict(case_id=case['case_id'], adjudication=case['adjudication'], execution_dispatch=False,
            prior_status=old_record['status'], prior_request_sha256=old_record['request_sha256'],
            request_sha256=probe.sha(probe.canonical(request)))
        if stopped:
            record.update(status='NOT_RUN', reason='qualification_failed_or_prior_request_error')
        else:
            started = time.monotonic()
            try:
                response = probe.rpc(url + '/v1/propose', request)
                probe.save(folder / 'response.json', response)
                probe.validate_response(request, response)
                record.update(status='RESPONDED', assessment=probe.assess(case, response),
                              worker_reported_inference_wall_s=response.get('inference_wall_s'))
            except Exception as error:
                record.update(status='ERROR', **error_record(error))
                stopped = True
            record['rpc_and_assessment_wall_s'] = time.monotonic() - started
        probe.save(folder / 'assessment.json', record)
        records.append(record)
        print(json.dumps(dict(case=case['case_id'], status=record['status'],
            assessment={k: v for k, v in record.get('assessment', {}).items() if k in
                ('declared_model_status', 'local_schema_status', 'pipeline_status', 'model_matches_authored_intent')})), flush=True)
    after = None
    try:
        after = endpoint_get(url + '/v1/capabilities')
        probe.save(output / 'worker-after.json', after)
    except Exception as error:
        probe.save(output / 'worker-after-error.json', error_record(error))
    current = probe.source_identity()
    current['scripts/continue_intent_probe.py'] = probe.sha(Path(__file__).read_bytes())
    report = dict(written_at_utc=datetime.now(timezone.utc).isoformat(),
        scope='APPROVED_MISSING_CASE_DEVELOPMENT_CONTINUATION', execution_dispatch=False,
        corpus_sha256=probe.sha(probe.canonical(corpus)), source_unchanged=sources == current,
        worker_capabilities_unchanged=after == expected_caps, qualification_passed=qualification['passed'],
        model_identity_attested=False, semantic_prompt_identity_attested=False, generation_settings_attested=False,
        summary=probe.summarise(records), records=records)
    probe.save(output / 'report.json', report)
    probe.save(output / 'manifest.json', dict(source=sources, artifacts={
        p.relative_to(output).as_posix(): probe.sha(p.read_bytes())
        for p in sorted(output.rglob('*')) if p.is_file()}))
    print(json.dumps(report['summary']), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--previous', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker-url', required=True)
    args = parser.parse_args()
    report = run(args.output, args.worker_url.rstrip('/'), args.previous)
    return 0 if (report['qualification_passed'] and report['summary']['errors'] == 0
                 and report['source_unchanged'] and report['worker_capabilities_unchanged']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
