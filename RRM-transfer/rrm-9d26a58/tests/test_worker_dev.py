"""Private development control with real disposable child processes, never a model."""
import hashlib
from http.server import ThreadingHTTPServer
import importlib.util
import io
import json
import os
import signal
import subprocess
import sys
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import zipfile

from rrm.worker_provenance import model_manifest, startup_identity
from rrm.intent_grounding_fixtures import specification


REPO = Path(__file__).resolve().parents[3]


def module(name):
    path = REPO / 'osmo/cosmos-worker' / (name + '.py')
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


CONTROL = module('dev_control')
CLIENT = module('dev_client')
RRM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RRM_ROOT / 'scripts'))
import rrm_cosmos_worker as WORKER


def files(body=b'import time\ntime.sleep(60)\n'):
    return {'rrm/__init__.py': b'', 'scripts/rrm_cosmos_reason2.py': b'',
            'scripts/rrm_cosmos_worker.py': body}


def archive(contents):
    target = io.BytesIO()
    with zipfile.ZipFile(target, 'w') as stream:
        for name, raw in contents.items():
            stream.writestr(name, raw)
    return target.getvalue()


class WorkerDevelopmentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.supervisor = CONTROL.Supervisor(self.root / 'state', self.root / 'model', stop_timeout=1)
        self.addCleanup(self.supervisor.stop)

    def server(self):
        self.token = 'test-only-development-token-' + 'x' * 32
        server = ThreadingHTTPServer(('127.0.0.1', 0), CONTROL.make_handler(self.supervisor, self.token))
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        return f'http://127.0.0.1:{server.server_port}'

    def test_private_http_requires_auth_for_status_upload_and_activation(self):
        url = self.server()
        for path, body in (('/v1/status', None), ('/v1/releases', archive(files())),
                           ('/v1/activate', b'{}')):
            for token in ('', 'wrong'):
                with self.assertRaises(HTTPError) as caught:
                    CLIENT.rpc(url + path, token, body)
                self.assertEqual(caught.exception.code, 401)
        self.assertFalse(CLIENT.rpc(url + '/v1/status', self.token)['child_running'])

    def test_upload_does_not_activate_and_client_bundle_identity_matches(self):
        source = self.root / 'source'
        for name, raw in files().items():
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        raw, identity = CLIENT.bundle(source)
        url = self.server()
        uploaded = CLIENT.rpc(url + '/v1/releases', self.token, raw, 'application/zip')
        self.assertEqual(uploaded['release_id'], identity)
        self.assertFalse(uploaded['activated'])
        self.assertIsNone(self.supervisor.active)
        self.assertEqual(CONTROL.unpack(raw), files())

    def test_traversal_oversize_duplicate_bad_syntax_and_missing_files_rejected(self):
        for contents in ({'../../evil.py': b'pass'}, dict(files(), **{'rrm/../evil.py': b'pass'}),
                         dict(files(), **{'rrm/large.py': b'x' * (CONTROL.MAX_FILE_BYTES + 1)}),
                         files(b'invalid python !'), {'rrm/__init__.py': b''}):
            with self.assertRaises((ValueError, SyntaxError)):
                CONTROL.unpack(archive(contents))
        stream = io.BytesIO()
        with zipfile.ZipFile(stream, 'w') as duplicate:
            duplicate.writestr('rrm/a.py', b'pass')
            with self.assertWarns(UserWarning):
                duplicate.writestr('rrm/a.py', b'pass')
        with self.assertRaisesRegex(ValueError, 'archive_entries'):
            CONTROL.unpack(stream.getvalue())

    def test_tampered_release_cannot_stop_running_child(self):
        identity = self.supervisor.install(files())
        before = self.supervisor.activate(identity)
        (self.supervisor.releases / identity / 'rrm/__init__.py').write_bytes(b'# changed')
        with self.assertRaisesRegex(ValueError, 'integrity'):
            self.supervisor.activate(identity)
        self.assertEqual(self.supervisor.status()['child_pid'], before['child_pid'])
        self.assertTrue(self.supervisor.status()['child_running'])

    def test_symlink_and_untracked_release_file_rejected(self):
        identity = self.supervisor.install(files())
        target = self.supervisor.releases / identity / 'unexpected.py'
        target.write_text('pass')
        with self.assertRaisesRegex(ValueError, 'integrity'):
            self.supervisor.verify(identity)
        target.unlink()
        target.symlink_to('/tmp/irrelevant')
        with self.assertRaisesRegex(ValueError, 'directory'):
            self.supervisor.verify(identity)

    def test_restart_and_manual_rollback_only_replace_child_and_strip_tokens(self):
        body = (b'import json,os,time\nfrom pathlib import Path\n'
                b'Path("env-check.json").write_text(json.dumps({k:k in os.environ for k in '
                b'("RRM_WORKER_DEV_TOKEN","HF_TOKEN")}))\ntime.sleep(60)\n')
        identity = self.supervisor.install(files(body))
        with patch.dict(os.environ, RRM_WORKER_DEV_TOKEN='never-print-this', HF_TOKEN='also-secret'):
            first = self.supervisor.activate(identity)
        evidence = self.supervisor.releases / identity / 'env-check.json'
        for _ in range(100):
            if evidence.exists():
                break
            time.sleep(.01)
        self.assertEqual(json.loads(evidence.read_text()), {'RRM_WORKER_DEV_TOKEN': False, 'HF_TOKEN': False})
        # Child-created artifacts in release dirs correctly invalidate a release.
        evidence.unlink()
        second_id = self.supervisor.install(files(b'import time\ntime.sleep(59)\n'))
        second = self.supervisor.activate(second_id)
        third = self.supervisor.activate(identity)
        self.assertNotEqual(first['child_pid'], second['child_pid'])
        self.assertNotEqual(second['child_pid'], third['child_pid'])
        self.assertEqual(third['active_release'], identity)
        self.assertEqual(third['previous_release'], second_id)

    def test_child_crash_does_not_end_http_supervisor(self):
        url = self.server()
        identity = self.supervisor.install(files(b'raise SystemExit(7)\n'))
        self.supervisor.activate(identity)
        self.supervisor.process.wait(timeout=5)
        result = CLIENT.rpc(url + '/v1/status', self.token)
        self.assertFalse(result['child_running'])
        self.assertEqual(result['child_exit_code'], 7)
        self.assertEqual(result['active_release'], identity)

    def test_stuck_child_is_killed_with_bounded_timeout_without_ending_supervisor(self):
        marker = self.root / 'stubborn-ready'
        body = ('import signal,time\nfrom pathlib import Path\n'
                'signal.signal(signal.SIGTERM, signal.SIG_IGN)\n'
                f'Path({str(marker)!r}).write_text("ready")\ntime.sleep(60)\n').encode()
        identity = self.supervisor.install(files(body))
        self.supervisor.stop_timeout = .05
        self.supervisor.activate(identity)
        for _ in range(100):
            if marker.exists():
                break
            time.sleep(.01)
        self.assertTrue(marker.exists())
        self.supervisor.stop()
        self.assertEqual(self.supervisor.process.returncode, -signal.SIGKILL)

    def test_client_cli_upload_activate_status_and_private_token_file(self):
        url = self.server()
        token_file = self.root / 'token'
        token_file.write_text(self.token)
        token_file.chmod(0o600)
        source = self.root / 'source'
        for name, raw in files().items():
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        command = [sys.executable, str(REPO / 'osmo/cosmos-worker/dev_client.py'),
                   '--url', url, '--token-file', str(token_file)]
        uploaded = subprocess.run(command + ['upload', '--source', str(source)],
                                  check=True, capture_output=True, text=True)
        identity = json.loads(uploaded.stdout)['release_id']
        self.assertFalse(self.supervisor.status()['child_running'])
        activated = subprocess.run(command + ['activate', '--release', identity, '--quiesced'],
                                   check=True, capture_output=True, text=True)
        self.assertEqual(json.loads(activated.stdout)['active_release'], identity)
        status = subprocess.run(command + ['status'], check=True, capture_output=True, text=True)
        self.assertTrue(json.loads(status.stdout)['child_running'])
        token_file.chmod(0o644)
        unsafe = subprocess.run(command + ['status'], capture_output=True, text=True)
        self.assertNotEqual(unsafe.returncode, 0)
        self.assertNotIn(self.token, unsafe.stdout + unsafe.stderr)

    def test_activation_requires_explicit_quiescence_and_unknown_id_is_safe(self):
        url = self.server()
        identity = self.supervisor.install(files())
        with self.assertRaises(HTTPError):
            CLIENT.rpc(url + '/v1/activate', self.token, json.dumps({'release_id': identity}).encode())
        self.assertIsNone(self.supervisor.active)
        with self.assertRaises(ValueError):
            self.supervisor.activate('../../invalid')

    def test_redirects_not_followed_with_bearer_token(self):
        class Redirect(CONTROL.BaseHTTPRequestHandler):
            reached = []
            def log_message(self, *_):
                pass
            def do_GET(self):
                self.reached.append(self.path)
                self.send_response(302)
                self.send_header('Location', '/token-leak')
                self.end_headers()
        server = ThreadingHTTPServer(('127.0.0.1', 0), Redirect)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        with self.assertRaises(HTTPError):
            CLIENT.rpc(f'http://127.0.0.1:{server.server_port}/start', 'test-token')
        self.assertEqual(Redirect.reached, ['/start'])

    def test_public_and_metadata_destinations_rejected_before_token_send(self):
        for address in ('8.8.8.8', '169.254.169.254', '0.0.0.0', '2606:4700:4700::1111'):
            with patch.object(CLIENT.socket, 'getaddrinfo', return_value=[(2, 1, 6, '', (address, 8091))]), \
                    patch.object(CLIENT, 'build_opener') as opener:
                with self.assertRaisesRegex(ValueError, 'requires_private'):
                    CLIENT.rpc('http://worker:8091/v1/status', 'do-not-send')
                opener.assert_not_called()

    def test_model_and_source_manifests_track_bytes_and_generation_settings(self):
        model = self.root / 'model'
        model.mkdir()
        (model / 'config.json').write_text('{}')
        (model / 'model.safetensors').write_bytes(b'unit-fixture-not-weights')
        source = self.root / 'source'
        for name, raw in files().items():
            path = source / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        identity = startup_identity(source, model, 768)
        self.assertEqual(identity['source_manifest_sha256'], self.supervisor.install(files()))
        self.assertEqual(identity['model_artifacts']['files']['model.safetensors'],
                         hashlib.sha256(b'unit-fixture-not-weights').hexdigest())
        self.assertEqual(identity['generation']['do_sample'], False)
        before = model_manifest(model)['manifest_sha256']
        (model / 'model.safetensors').write_bytes(b'changed')
        self.assertNotEqual(model_manifest(model)['manifest_sha256'], before)
        self.assertEqual(identity['identity_scope'], 'SELF_REPORTED_STARTUP_FILES_NOT_REMOTE_ATTESTATION')

    def test_loaded_release_verification_requires_matching_provenance_and_stable_child(self):
        state = dict(child_running=True, active_release='a' * 64, child_pid=123, execution_dispatch=False)
        caps = dict(execution_dispatch=False, worker_provenance=dict(
            source_manifest_sha256='a' * 64, development_release_id='a' * 64,
            execution_dispatch=False, schema_version='rrm-worker-provenance/v1'))
        with patch.object(CLIENT, 'rpc', side_effect=[state, caps, state]) as rpc:
            self.assertEqual(CLIENT.verify('http://worker:8091', 'http://worker:8090', 'secret')['status'],
                             'LOADED_SOURCE_MATCHED')
            self.assertEqual(rpc.call_args_list[1].args[1], None)
        for changed in ({}, dict(caps, worker_provenance=dict(caps['worker_provenance'],
                         source_manifest_sha256='b' * 64))):
            with patch.object(CLIENT, 'rpc', side_effect=[state, changed, state]):
                with self.assertRaisesRegex(ValueError, 'not_verified'):
                    CLIENT.verify('http://worker:8091', 'http://worker:8090', 'secret')
        with patch.object(CLIENT, 'rpc', side_effect=[state, caps, dict(state, child_pid=999)]):
            with self.assertRaisesRegex(ValueError, 'not_verified'):
                CLIENT.verify('http://worker:8091', 'http://worker:8090', 'secret')

    def test_real_worker_response_reports_actual_semantic_prompt_and_frozen_identity(self):
        model = self.root / 'model'
        model.mkdir()
        (model / 'config.json').write_text('{}')
        (model / 'model.safetensors').write_bytes(b'fake-unit-weights')
        with patch.object(WORKER, 'CosmosGenerator') as generator:
            generator.return_value.generate.return_value = json.dumps(dict(
                status='NEEDS_CLARIFICATION', ambiguity_refs=['unit-test'], explanation='unit test'))
            worker = WORKER.CosmosWorker(str(model), max_new_tokens=768)
        before = json.dumps(worker.provenance, sort_keys=True)
        context = specification()['cases'][0]['context']
        request = dict(cycle_id='a' * 32, step_index=0, context=context,
            image_base64='dW5pdC1pbWFnZQ==', observation_sha256=hashlib.sha256(b'unit-image').hexdigest())
        result = worker.propose(request)
        prompt = worker.generator.generate.call_args.kwargs['prompt']
        self.assertEqual(result['semantic_prompt_sha256'], hashlib.sha256(prompt.encode()).hexdigest())
        (model / 'model.safetensors').write_bytes(b'changed-after-startup')
        self.assertEqual(json.dumps(result['worker_provenance'], sort_keys=True), before)
        self.assertFalse(result['execution_dispatch'])

    def test_workflow_structure_and_embedded_shell_syntax(self):
        import yaml
        value = yaml.safe_load((REPO / 'osmo/workflows/airstack-live-replan-dev.yaml').read_text())
        tasks = value['workflow']['groups'][0]['tasks']
        self.assertEqual([task['name'] for task in tasks], ['workspace', 'cosmos-worker'])
        self.assertEqual(tasks[1]['image'], '{{worker_dev_image}}')
        self.assertIn('REQUIRED', value['default-values']['worker_dev_image'])
        for task in tasks:
            self.assertIn('rrm-worker-dev', task['credentials'])
            subprocess.run(['bash', '-n'], input=task['args'][0], text=True, check=True)
        dockerfile = (REPO / 'osmo/cosmos-worker/Dockerfile.dev').read_text()
        self.assertIn('@sha256:', dockerfile)
        self.assertIn('/opt/rrm-dev/dev_control.py', dockerfile)


if __name__ == '__main__':
    unittest.main()
