"""Continuation preserves completed outputs and fails closed on evidence drift."""
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import socket
import tempfile
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError


class IntentContinuationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'scripts/continue_intent_probe.py'
        spec = importlib.util.spec_from_file_location('intent_continuation', path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.previous = self.root / 'previous'
        self.caps = dict(routes=['/v1/propose'], execution_dispatch=False, worker_source_sha256='test-only')
        self.calls = 0

        def original_rpc(url, body=None):
            if body is None:
                return self.caps
            self.calls += 1
            if self.calls == 7:
                raise URLError(socket.gaierror(-3, 'test only'))
            return self.response(body)

        with patch.object(self.module.probe, 'rpc', side_effect=original_rpc), redirect_stdout(io.StringIO()):
            self.module.probe.run(self.previous, 'http://private-worker:8090')

    def response(self, request):
        return dict(cycle_id=request['cycle_id'], step_index=request['step_index'],
            observation_sha256=request['observation_sha256'], execution_dispatch=False,
            candidate={'raw_response': json.dumps({'status': 'UNSUPPORTED', 'explanation': 'unit fixture'})})

    def endpoint(self, url):
        return dict(status='ready', execution_dispatch=False) if url.endswith('/healthz') else self.caps

    def test_selects_only_error_and_not_run_in_original_order(self):
        _, _, _, selected = self.module.load_previous(self.previous)
        self.assertEqual([index for index, *_ in selected], list(range(6, 12)))
        self.assertEqual([old['status'] for _, _, old, _ in selected], ['ERROR'] + ['NOT_RUN'] * 5)

    def test_artifact_source_and_corpus_drift_rejected(self):
        report_path = self.previous / 'report.json'
        original = report_path.read_bytes()
        report_path.write_bytes(original + b' ')
        with self.assertRaisesRegex(ValueError, 'artifact_integrity'):
            self.module.load_previous(self.previous)
        report_path.write_bytes(original)
        with patch.object(self.module.probe, 'source_identity', return_value={}):
            with self.assertRaisesRegex(ValueError, 'source_changed'):
                self.module.load_previous(self.previous)
        with patch.object(self.module.probe, 'specification', return_value={}):
            with self.assertRaisesRegex(ValueError, 'corpus_changed'):
                self.module.load_previous(self.previous)

    def test_fresh_cycles_preserve_inputs_steps_and_exclude_prior_responses(self):
        sent = []
        def rpc(url, body):
            sent.append(body)
            return self.response(body)
        with patch.object(self.module, 'endpoint_get', side_effect=self.endpoint), \
                patch.object(self.module.probe, 'rpc', side_effect=rpc), redirect_stdout(io.StringIO()):
            report = self.module.run(self.root / 'continuation', 'http://private-worker:8090', self.previous)
        self.assertEqual(report['summary']['responded'], 6)
        self.assertEqual([r['step_index'] for r in sent], list(range(6, 12)))
        for request, (_, _, _, old) in zip(sent, self.module.load_previous(self.previous)[3]):
            self.assertNotEqual(request['cycle_id'], old['cycle_id'])
            self.assertEqual({k: v for k, v in request.items() if k != 'cycle_id'},
                             {k: v for k, v in old.items() if k != 'cycle_id'})

    def test_first_request_error_stops_remaining_inference_and_retains_safe_cause(self):
        with patch.object(self.module, 'endpoint_get', side_effect=self.endpoint), \
                patch.object(self.module.probe, 'rpc', side_effect=URLError(socket.gaierror(-3, 'secret text'))) as rpc, \
                redirect_stdout(io.StringIO()):
            report = self.module.run(self.root / 'continuation', 'http://private-worker:8090', self.previous)
        self.assertEqual(rpc.call_count, 1)
        self.assertEqual(report['summary']['errors'], 1)
        self.assertEqual(report['summary']['not_run'], 5)
        self.assertEqual(report['records'][0]['reason_type'], 'gaierror')
        self.assertEqual(report['records'][0]['reason_errno'], -3)
        self.assertNotIn('secret text', json.dumps(report))

    def test_failed_health_or_changed_capability_prevents_all_inference(self):
        for name, endpoint in (('health', lambda _: {'status': 'not_ready'}),
                               ('caps', lambda url: dict(status='ready', execution_dispatch=False)
                                if url.endswith('/healthz') else dict(self.caps, extra='changed'))):
            with patch.object(self.module, 'endpoint_get', side_effect=endpoint), \
                    patch.object(self.module.probe, 'rpc') as rpc, redirect_stdout(io.StringIO()):
                report = self.module.run(self.root / name, 'http://private-worker:8090', self.previous)
            rpc.assert_not_called()
            self.assertFalse(report['qualification_passed'])
            self.assertEqual(report['summary']['not_run'], 6)

    def test_qualification_three_successes_and_http_error_metadata(self):
        with patch.object(self.module, 'endpoint_get', side_effect=self.endpoint) as endpoint:
            self.assertTrue(self.module.qualify('http://private-worker', self.caps)['passed'])
            self.assertEqual(endpoint.call_count, 6)
        error = HTTPError('http://secret', 503, 'secret', {}, None)
        self.assertEqual(self.module.error_record(error), {'error_type': 'HTTPError', 'http_status': 503})


if __name__ == '__main__':
    unittest.main()
