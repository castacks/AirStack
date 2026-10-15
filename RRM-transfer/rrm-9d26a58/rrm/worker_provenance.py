"""Self-reported startup artifact identity, not remote attestation or authority."""
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path


def file_sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def manifest_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def model_manifest(root):
    """Hash snapshot artifacts, excluding hidden download caches/lock metadata."""
    files = {p.relative_to(root).as_posix(): file_sha256(p) for p in sorted(root.rglob('*'))
             if p.is_file() and not any(part.startswith('.') for part in p.relative_to(root).parts)}
    if 'config.json' not in files or not any(name.endswith('.safetensors') for name in files):
        raise ValueError('model_snapshot_requires_config_and_safetensors')
    return dict(files=files, manifest_sha256=manifest_sha256(files), hash_scope='nonhidden_snapshot_files')


def startup_identity(root, model_path, max_new_tokens):
    source = {p.relative_to(root).as_posix(): file_sha256(p)
              for folder in ('rrm', 'scripts') for p in sorted((root / folder).glob('*.py')) if p.is_file()}
    packages = {}
    for name in ('torch', 'transformers', 'pydantic', 'pillow', 'accelerate'):
        try:
            packages[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            packages[name] = None
    return dict(schema_version='rrm-worker-provenance/v1', source_files=source,
        source_manifest_sha256=manifest_sha256(source), model_artifacts=model_manifest(Path(model_path)),
        declared_model_revision=os.environ.get('COSMOS_MODEL_REVISION'),
        development_release_id=os.environ.get('RRM_DEV_RELEASE_ID'), runtime_packages=packages,
        generation=dict(max_new_tokens=max_new_tokens, do_sample=False, dtype='bfloat16',
                        device_map='auto', attn_implementation='sdpa'),
        identity_scope='SELF_REPORTED_STARTUP_FILES_NOT_REMOTE_ATTESTATION', execution_dispatch=False)
