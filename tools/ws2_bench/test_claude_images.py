"""CLI-only mocked tests: no model, credential file or subscription is called."""
import base64
import hashlib
import io
import json
from types import SimpleNamespace

import pytest
from PIL import Image

from claude_provider import ClaudeSubscriptionProvider


def image_file(root, suffix):
    buffer = io.BytesIO(); Image.new('RGB', (20, 10), 'red').save(buffer, format='PNG' if suffix == 'png' else 'JPEG')
    path = root/f'frame.{suffix}'; path.write_bytes(buffer.getvalue()); return path


@pytest.fixture
def provider(tmp_path, monkeypatch):
    monkeypatch.setattr('claude_provider.shutil.which', lambda _: '/mock/claude')
    instance = ClaudeSubscriptionProvider(tmp_path/'calls')
    monkeypatch.setattr(instance, 'check_auth', lambda: {'loggedIn': True})
    return instance


def test_image_stream_has_correct_mime_and_audit_has_hash_not_base64(tmp_path, monkeypatch, provider):
    images = [image_file(tmp_path, extension) for extension in ('png', 'jpg')]
    captured = []
    def run(argv, **kwargs):
        captured.append((argv, kwargs))
        return SimpleNamespace(returncode=0, stdout='\n'.join(json.dumps(x) for x in [
            {'type': 'system', 'subtype': 'init'},
            {'type': 'result', 'subtype': 'success', 'is_error': False,
             'structured_output': {'decision': 'test'}, 'usage': {'input_tokens': 20}}]))
    monkeypatch.setattr('claude_provider.subprocess.run', run)
    value = provider.complete('inspect', 'Return JSON.', {'run': 1}, {'type': 'object'}, lambda raw: raw, images=images)
    assert value == {'decision': 'test'}
    argv, kwargs = captured[0]
    assert argv[argv.index('--input-format')+1] == 'stream-json'
    assert argv[argv.index('--output-format')+1] == 'stream-json'
    assert '--verbose' in argv and argv[argv.index('--tools')+1] == ''
    message = json.loads(kwargs['input']); content = message['message']['content']
    assert content[0]['type'] == 'text'
    assert [x['source']['media_type'] for x in content[1:]] == ['image/png', 'image/jpeg']
    for block, path in zip(content[1:], images):
        assert base64.b64decode(block['source']['data']) == path.read_bytes()
    audit_path = next((tmp_path/'calls').glob('*.json')); audit_text = audit_path.read_text()
    audit = json.loads(audit_text)
    assert audit['status'] == 'validated'
    assert audit['request']['images'][0]['sha256'] == hashlib.sha256(images[0].read_bytes()).hexdigest()
    assert all(block['source']['data'] not in audit_text for block in content[1:])
    assert 'base64' not in audit_text


def test_text_only_keeps_existing_json_interface(tmp_path, monkeypatch, provider):
    captured = []
    def run(argv, **kwargs):
        captured.append((argv, kwargs))
        return SimpleNamespace(returncode=0, stdout=json.dumps({
            'subtype': 'success', 'is_error': False, 'structured_output': {'ok': True}}))
    monkeypatch.setattr('claude_provider.subprocess.run', run)
    assert provider.complete('selection', 'Return JSON.', {'round': 1}, {}, lambda raw: raw) == {'ok': True}
    argv, kwargs = captured[0]
    assert '--input-format' not in argv and '--verbose' not in argv
    assert argv[argv.index('--output-format')+1] == 'json'
    assert json.loads(kwargs['input']) == {'data': {'round': 1}, 'validation_feedback': None}


def test_image_bounds_and_format_fail_before_model_call(tmp_path, monkeypatch, provider):
    monkeypatch.setattr('claude_provider.subprocess.run', lambda *a, **k: pytest.fail('No CLI completion for invalid images'))
    image = image_file(tmp_path, 'png')
    with pytest.raises(ValueError, match='At most six'):
        provider.complete('inspect', '', {}, {}, lambda raw: raw, images=[image]*7)
    invalid = tmp_path/'text.png'; invalid.write_text('Not a PNG')
    with pytest.raises(ValueError, match='PNG or JPEG'):
        provider.complete('inspect', '', {}, {}, lambda raw: raw, images=[invalid])
