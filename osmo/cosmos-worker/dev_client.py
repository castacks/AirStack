#!/usr/bin/env python3
"""Private trusted-development client. Tokens are read from a file, never argv."""
import argparse
import hashlib
import io
import ipaddress
import json
from pathlib import Path
import stat
import socket
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener
import zipfile


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *_):
        return None


def bundle(root):
    stream = io.BytesIO()
    files = {}
    with zipfile.ZipFile(stream, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for folder in ('rrm', 'scripts'):
            for path in sorted((root / folder).glob('*.py')):
                if path.is_symlink():
                    raise ValueError('symlink_source_not_allowed')
                raw = path.read_bytes()
                compile(raw, path.name, 'exec')
                name = path.relative_to(root).as_posix()
                files[name] = hashlib.sha256(raw).hexdigest()
                archive.writestr(name, raw)
    identity = hashlib.sha256(json.dumps(files, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return stream.getvalue(), identity


def rpc(url, token, body=None, content_type='application/json'):
    parsed = urlsplit(url)
    if (parsed.scheme != 'http' or not parsed.hostname or parsed.username or parsed.password
            or parsed.query or parsed.fragment):
        raise ValueError('expected_private_plain_http_url')
    addresses = {item[4][0] for item in socket.getaddrinfo(parsed.hostname, parsed.port or 80,
                                                         type=socket.SOCK_STREAM)}
    allowed = tuple(map(ipaddress.ip_network, ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16',
                                              '127.0.0.0/8', 'fc00::/7', '::1/128')))
    if not addresses or any(not any(ipaddress.ip_address(address) in network for network in allowed)
                            for address in addresses):
        raise ValueError('development_control_requires_private_or_loopback_destination')
    # Pin the checked address so a second DNS lookup cannot redirect the token.
    address = sorted(addresses)[0]
    host = '[' + address + ']' if ':' in address else address
    target = parsed._replace(netloc=host + ':' + str(parsed.port or 80)).geturl()
    # Never forward the development token through an ambient proxy or redirect.
    opener = build_opener(ProxyHandler({}), NoRedirect())
    headers = {'Host': parsed.netloc, 'Content-Type': content_type}
    if token is not None:
        headers['Authorization'] = 'Bearer ' + token
    request = Request(target, data=body, headers=headers)
    with opener.open(request, timeout=60) as response:
        raw = response.read(1024 * 1024 + 1)
        if len(raw) > 1024 * 1024:
            raise ValueError('control_response_too_large')
    return json.loads(raw)


def verify(base, worker_url, token):
    if urlsplit(base).hostname != urlsplit(worker_url).hostname or urlsplit(worker_url).path.rstrip('/'):
        raise ValueError('worker_and_control_must_share_private_hostname')
    before = rpc(base + '/v1/status', token)
    capabilities = rpc(worker_url.rstrip('/') + '/v1/capabilities', None)
    after = rpc(base + '/v1/status', token)
    identity = capabilities.get('worker_provenance') or {}
    if (before != after or not after.get('child_running') or not after.get('active_release')
            or after.get('execution_dispatch') is not False or capabilities.get('execution_dispatch') is not False
            or identity.get('source_manifest_sha256') != after['active_release']
            or identity.get('development_release_id') != after['active_release']
            or identity.get('execution_dispatch') is not False
            or identity.get('schema_version') != 'rrm-worker-provenance/v1'):
        raise ValueError('loaded_worker_release_not_verified')
    return dict(status='LOADED_SOURCE_MATCHED', control=after, worker_capabilities=capabilities,
                identity_scope='SELF_REPORTED_NOT_REMOTE_ATTESTATION', execution_dispatch=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', required=True, help='Private worker control URL, port8091; never public')
    parser.add_argument('--token-file', type=Path, default=Path('/run/rrm-worker-dev.token'))
    sub = parser.add_subparsers(dest='operation', required=True)
    sub.add_parser('status')
    verification = sub.add_parser('verify')
    verification.add_argument('--worker-url', required=True)
    upload = sub.add_parser('upload')
    upload.add_argument('--source', type=Path, required=True)
    activate = sub.add_parser('activate')
    activate.add_argument('--release', required=True)
    activate.add_argument('--quiesced', action='store_true', required=True)
    args = parser.parse_args()
    mode = args.token_file.stat().st_mode
    if args.token_file.is_symlink() or not stat.S_ISREG(mode) or stat.S_IMODE(mode) & 0o077:
        raise ValueError('token_file_must_be_private_regular_file')
    token = args.token_file.read_text().strip()
    if len(token) < 32 or any(c.isspace() for c in token):
        raise ValueError('invalid_development_token')
    base = args.url.rstrip('/')
    if urlsplit(base).path:
        raise ValueError('control_base_url_must_have_no_path')
    if args.operation == 'status':
        result = rpc(base + '/v1/status', token)
    elif args.operation == 'verify':
        result = verify(base, args.worker_url, token)
    elif args.operation == 'upload':
        raw, expected = bundle(args.source)
        result = rpc(base + '/v1/releases', token, raw, 'application/zip')
        if result.get('release_id') != expected or result.get('activated') is not False:
            raise ValueError('uploaded_release_identity_mismatch')
    else:
        result = rpc(base + '/v1/activate', token,
                     json.dumps({'release_id': args.release, 'quiesced': args.quiesced}).encode())
        if result.get('active_release') != args.release:
            raise ValueError('activated_release_identity_mismatch')
    if result.get('execution_dispatch') is not False:
        raise ValueError('development_dispatch_contract')
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
