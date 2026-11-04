"""Bounded live-view/control transport and verified artifact transfer."""
import hashlib
import json
from pathlib import Path
import tarfile
import urllib.request

JSON_FILES = {'episode': 'live_episode.json', 'scene': 'scene_status.json',
              'view': 'live_view.json', 'view_settings': 'view_settings.json'}


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, indent=2))
    tmp.replace(path)


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def request(session, path, payload=None):
    """The caller supplies only a loopback tunnel URL, never an agent-chosen host."""
    from urllib.parse import urlsplit
    url = urlsplit(session['url'])
    if url.scheme != 'http' or url.hostname != '127.0.0.1' or url.username or url.path:
        raise ValueError('Remote viewer requires an OSMO loopback tunnel')
    data = None if payload is None else json.dumps(payload).encode()
    req = urllib.request.Request(session['url'] + path, data=data, headers={
        'Authorization': 'Bearer ' + session['token'], 'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=2) as response:
        if response.headers.get('X-WS2-Run') != session['run_id']:
            raise ValueError('Viewer returned a different run')
        body = response.read(8 * 1024 * 1024 + 1)
        if len(body) > 8 * 1024 * 1024:
            raise ValueError('Oversized remote viewer response')
        return body


def pack_artifacts(episode_dir, destination, run_id, config_hash):
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    files = {}
    archive = destination / 'artifacts.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        for path in sorted(Path(episode_dir).rglob('*')):
            if path.is_symlink():
                raise ValueError('Artifact symlinks are not supported')
            if path.is_file():
                name = path.relative_to(episode_dir).as_posix()
                files[name] = file_hash(path)
                tar.add(path, arcname=name, recursive=False)
    receipt = {'schema_version': 1, 'run_id': run_id, 'configuration_hash': config_hash,
               'archive_sha256': file_hash(archive), 'files': files,
               'remote_episode_dir': str(Path(episode_dir).resolve())}
    atomic_json(destination / 'receipt.json', receipt)
    return receipt


def unpack_artifacts(archive, receipt, destination, run_id, config_hash):
    """Verify all files before extraction; retain the archive as original evidence."""
    if receipt.get('run_id') != run_id or receipt.get('configuration_hash') != config_hash:
        raise ValueError('Result belongs to a different episode/configuration')
    if file_hash(archive) != receipt.get('archive_sha256'):
        raise ValueError('Artifact archive checksum mismatch')
    dest = Path(destination).resolve()
    files = receipt['files']
    with tarfile.open(archive, 'r:gz') as tar:
        members = tar.getmembers()
        if len(members) != len(files) or {m.name for m in members} != set(files):
            raise ValueError('Artifact manifest/member mismatch')
        for member in members:
            path = (dest / member.name).resolve()
            path.relative_to(dest)
            if not member.isfile() or member.name.startswith('/') or '..' in Path(member.name).parts:
                raise ValueError('Unsafe artifact entry')
            if path.exists():
                raise ValueError('Refusing to overwrite existing evidence: ' + member.name)
            h = hashlib.sha256()
            with tar.extractfile(member) as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    h.update(chunk)
            if h.hexdigest() != files[member.name]:
                raise ValueError('Artifact checksum mismatch: ' + member.name)
        for member in members:
            path = dest / member.name
            path.parent.mkdir(parents=True, exist_ok=True)
            with tar.extractfile(member) as source, path.open('xb') as output:
                import shutil
                shutil.copyfileobj(source, output)


def mapped_paths(value, remote, local):
    if isinstance(value, str) and (value == remote or value.startswith(remote + '/')):
        return local + value[len(remote):]
    if isinstance(value, list):
        return [mapped_paths(v, remote, local) for v in value]
    if isinstance(value, dict):
        return {k: mapped_paths(v, remote, local) for k, v in value.items()}
    return value
