#!/usr/bin/env python3
"""Trusted-developer private source deployment; NOT a hostile-code sandbox."""
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import re
import secrets
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import zipfile

MAX_BYTES = 16 * 1024 * 1024
MAX_FILE_BYTES = 2 * 1024 * 1024
REQUIRED = {'scripts/rrm_cosmos_worker.py', 'scripts/rrm_cosmos_reason2.py', 'rrm/__init__.py'}


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


CONTROL_SOURCE_SHA256 = digest(Path(__file__).read_bytes())


def validate_files(files):
    if not REQUIRED.issubset(files) or len(files) > 256 or sum(map(len, files.values())) > MAX_BYTES:
        raise ValueError('invalid_release_size_or_required_files')
    for name, raw in files.items():
        if not re.fullmatch(r'(rrm|scripts)/[A-Za-z_][A-Za-z_0-9]*\.py', name):
            raise ValueError('invalid_release_path')
        if len(raw) > MAX_FILE_BYTES:
            raise ValueError('release_file_too_large')
        compile(raw, name, 'exec')  # Syntax only; never execute upload in supervisor.
    return {name: digest(raw) for name, raw in sorted(files.items())}


def unpack(raw):
    if len(raw) > MAX_BYTES:
        raise ValueError('archive_too_large')
    with zipfile.ZipFile(io.BytesIO(raw)) as archive:
        infos = archive.infolist()
        if len(infos) > 256 or len({i.filename for i in infos}) != len(infos):
            raise ValueError('invalid_archive_entries')
        if any(i.file_size > MAX_FILE_BYTES or i.flag_bits & 1 for i in infos):
            raise ValueError('invalid_archive_file')
        if sum(i.file_size for i in infos) > MAX_BYTES:
            raise ValueError('expanded_archive_too_large')
        files = {i.filename: archive.read(i) for i in infos}
    validate_files(files)
    return files


def source_files(root):
    return {p.relative_to(root).as_posix(): p.read_bytes()
            for folder in ('rrm', 'scripts') for p in sorted((root / folder).glob('*.py'))
            if p.is_file() and not p.is_symlink()}


class Supervisor:
    def __init__(self, state, model_path, *, python=sys.executable, stop_timeout=30):
        self.state = state
        self.releases = state / 'releases'
        self.releases.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.model_path = str(model_path)
        self.python = python
        self.stop_timeout = stop_timeout
        self.lock = threading.RLock()
        self.process = None
        self.active = None
        self.previous = None
        self.log = None

    def install(self, files):
        manifest = validate_files(files)
        identity = digest(canonical(manifest))
        destination = self.releases / identity
        with self.lock:
            if destination.exists():
                self.verify(identity)
                return identity
            temporary = Path(tempfile.mkdtemp(prefix='.upload-', dir=self.releases))
            try:
                for name, raw in files.items():
                    path = temporary / name
                    path.parent.mkdir(exist_ok=True)
                    path.write_bytes(raw)
                (temporary / 'manifest.json').write_bytes(canonical(manifest) + b'\n')
                temporary.rename(destination)
            finally:
                if temporary.exists():
                    shutil.rmtree(temporary)  # Exact newly allocated upload directory only.
        return identity

    def verify(self, identity):
        if not isinstance(identity, str) or not re.fullmatch('[a-f0-9]{64}', identity):
            raise ValueError('invalid_release_id')
        root = self.releases / identity
        if root.is_symlink() or not root.is_dir() or any(p.is_symlink() for p in root.rglob('*')):
            raise ValueError('invalid_release_directory')
        files = source_files(root)
        manifest = validate_files(files)
        actual_paths = {p.relative_to(root).as_posix() for p in root.rglob('*') if p.is_file()}
        if (actual_paths != set(manifest) | {'manifest.json'} or digest(canonical(manifest)) != identity
                or json.loads((root / 'manifest.json').read_bytes()) != manifest):
            raise ValueError('release_integrity_failed')
        return root

    def stop(self):
        with self.lock:
            if self.process is not None:
                # Only our own child/session is targeted, never PID 1 or another task.
                if self.process.poll() is None:
                    try:
                        os.killpg(self.process.pid, signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    try:
                        self.process.wait(timeout=self.stop_timeout)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(self.process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        self.process.wait(timeout=5)
                if self.log:
                    self.log.close()
                    self.log = None

    def activate(self, identity):
        with self.lock:
            root = self.verify(identity)  # Invalid uploads never stop a healthy child.
            self.stop()
            env = {k: v for k, v in os.environ.items()
                   if k not in ('RRM_WORKER_DEV_TOKEN', 'HF_TOKEN') and not k.startswith('PYTHON')}
            env.update(PYTHONPATH=str(root) + ':' + str(root / 'scripts'),
                       PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1',
                       RRM_DEV_RELEASE_ID=identity)
            self.log = (self.state / 'worker.log').open('ab', buffering=0)
            self.process = subprocess.Popen([self.python, str(root / 'scripts/rrm_cosmos_worker.py'),
                '--host', '0.0.0.0', '--port', '8090', '--model-path', self.model_path],
                env=env, cwd=root, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            self.previous, self.active = self.active, identity
            return self.status()

    def status(self):
        with self.lock:
            code = self.process.poll() if self.process is not None else None
            return dict(schema_version='rrm-worker-dev/v1', active_release=self.active,
                control_source_sha256=CONTROL_SOURCE_SHA256,
                previous_release=self.previous, child_running=self.process is not None and code is None,
                child_pid=self.process.pid if self.process else None, child_exit_code=code,
                readiness='CHECK_WORKER_HEALTH_AND_PROVENANCE', execution_dispatch=False)


def make_handler(supervisor, token):
    if len(token) < 32 or any(c.isspace() for c in token):
        raise ValueError('development_token_must_be_at_least_32_nonspace_characters')

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass  # Do not log headers, paths or uploaded content.

        def respond(self, value, status=200):
            raw = canonical(value)
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def authorized(self):
            self.connection.settimeout(10)
            provided = self.headers.get('Authorization', '').encode()
            if not secrets.compare_digest(provided, ('Bearer ' + token).encode()):
                self.close_connection = True
                self.respond({'error': 'unauthorized'}, 401)
                return False
            return True

        def do_GET(self):
            if not self.authorized():
                return
            if self.path == '/v1/status':
                return self.respond(supervisor.status())
            self.respond({'error': 'not_found'}, 404)

        def do_POST(self):
            if not self.authorized():
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= MAX_BYTES or self.headers.get('Transfer-Encoding'):
                    raise ValueError('invalid_request_size')
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError('incomplete_body')
                if self.path == '/v1/releases':
                    identity = supervisor.install(unpack(raw))
                    return self.respond({'release_id': identity, 'activated': False, 'execution_dispatch': False})
                if self.path == '/v1/activate':
                    value = json.loads(raw)
                    if value.get('quiesced') is not True:
                        raise ValueError('caller_must_quiesce_inference_clients')
                    return self.respond(supervisor.activate(value['release_id']))
                self.respond({'error': 'not_found'}, 404)
            except Exception:
                # Uploaded source and exception text may contain developer secrets.
                self.respond({'error': 'development_operation_failed'}, 400)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', type=Path, default=Path('/var/lib/rrm/dev'))
    parser.add_argument('--seed-source', type=Path, default=Path('/opt/rrm-dev/seed'))
    parser.add_argument('--model-path', required=True)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8091)
    args = parser.parse_args()
    token = os.environ.pop('RRM_WORKER_DEV_TOKEN', '')
    supervisor = Supervisor(args.state, args.model_path)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(supervisor, token))
    seed = supervisor.install(source_files(args.seed_source))
    supervisor.activate(seed)
    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print('Private development supervisor listening; execution_dispatch=false', flush=True)
    try:
        server.serve_forever()
    finally:
        supervisor.stop()
        server.server_close()


if __name__ == '__main__':
    main()
