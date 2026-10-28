"""Inspect approved CATS ZIPs with bounded HTTP ranges; no model evaluation."""
import argparse
from datetime import datetime
import hashlib
from html.parser import HTMLParser
import io
import json
from pathlib import Path, PurePosixPath
import urllib.parse
import urllib.request
import zipfile
from zoneinfo import ZoneInfo


class DownloadForm(HTMLParser):
    def __init__(self):
        super().__init__()
        self.action, self.fields = None, {}

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'form':
            self.action = attrs.get('action')
        if tag == 'input' and attrs.get('name'):
            self.fields[attrs['name']] = attrs.get('value', '')


def download_url(file_id):
    url = 'https://drive.google.com/uc?' + urllib.parse.urlencode({'export': 'download', 'id': file_id})
    with urllib.request.urlopen(url, timeout=45) as response:
        if 'text/html' not in response.headers.get('Content-Type', ''):
            return response.url
        body = response.read(1024 * 1024 + 1)
        if len(body) > 1024 * 1024:
            raise ValueError('oversized download interstitial')
    form = DownloadForm()
    form.feed(body.decode('utf-8'))
    if not form.action or urllib.parse.urlsplit(form.action).hostname != 'drive.usercontent.google.com':
        raise ValueError('expected Drive download confirmation form not found')
    return form.action + '?' + urllib.parse.urlencode(form.fields)


class RangeFile(io.RawIOBase):
    def __init__(self, url, size, budget=64 * 1024 * 1024):
        self.url, self.size, self.budget, self.pos = url, size, budget, 0
        self.ranges = []

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=0):
        self.pos = offset if whence == 0 else self.pos + offset if whence == 1 else self.size + offset
        if self.pos < 0:
            raise ValueError('negative archive offset')
        return self.pos

    def read(self, n=-1):
        n = self.size - self.pos if n < 0 else min(n, self.size - self.pos)
        if n <= 0:
            return b''
        if n > 16 * 1024 * 1024 or n > self.budget:
            raise ValueError('bounded range budget exceeded')
        start, end = self.pos, self.pos + n - 1
        request = urllib.request.Request(self.url, headers={'Range': f'bytes={start}-{end}', 'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(request, timeout=45) as response:
            expected = f'bytes {start}-{end}/{self.size}'
            if response.status != 206 or response.headers.get('Content-Range') != expected:
                raise ValueError('server did not honor exact bounded byte range')
            data = response.read(n + 1)
            if len(data) != n:
                raise ValueError('range length mismatch')
        self.pos += n
        self.budget -= n
        self.ranges.append({'start': start, 'end': end, 'bytes': n,
                            'sha256': hashlib.sha256(data).hexdigest()})
        return data


def inspect(file_id, size, output, sample=False):
    if output.exists():
        raise FileExistsError(output)
    output.mkdir(parents=True)
    remote = RangeFile(download_url(file_id), size)
    with zipfile.ZipFile(remote) as archive:
        members = [{'name': m.filename, 'size': m.file_size, 'compressed_size': m.compress_size,
                    'crc32': f'{m.CRC:08x}', 'header_offset': m.header_offset} for m in archive.infolist()]
        index = {'drive_file_id': file_id, 'archive_bytes': size, 'members': members}
        (output / 'index.json').write_text(json.dumps(index, indent=2) + '\n')
        scenes = sorted({m['name'].split('/rectified/color/left_color_default.png')[0] + '/'
                         for m in members if m['name'].endswith('/rectified/color/left_color_default.png')})
        selected = []
        if sample:
            if not scenes:
                raise ValueError('no scene with expected color layout')
            prefix = scenes[0]
            selected = [m for m in members if m['size'] and m['name'].startswith(prefix) and
                        any(part in m['name'][len(prefix):] for part in ('rawImages/', 'rectified/', 'gt_disparity/'))]
            selected += [m for m in members if m['name'].startswith(prefix + 'gt/') and m['name'].endswith('.asc')]
            if sum(m['size'] for m in selected) > 64 * 1024 * 1024:
                raise ValueError('selected scene exceeds uncompressed sample limit')
            # Record selection before reading any scene pixel, reference or model output.
            declaration = {'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
                'scene': prefix, 'selection': 'first lexicographic indexed scene; original images/masks/disparities/ASC',
                'members': [m['name'] for m in selected], 'model_inference': False, 'model_scoring': False}
            (output / 'sample-declaration.json').write_text(json.dumps(declaration, indent=2) + '\n')
        hashes = {}
        for member in selected:
            relative = PurePosixPath(member['name'])
            if relative.is_absolute() or '..' in relative.parts:
                raise ValueError('unsafe ZIP path')
            data = archive.read(member['name'])  # zipfile verifies CRC before returning.
            dest = output / 'sample' / member['name']
            dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(data)
            hashes[member['name']] = hashlib.sha256(data).hexdigest()
    result = {'status': 'PASS', 'created_at': datetime.now(ZoneInfo('America/New_York')).isoformat(),
        'drive_file_id': file_id, 'archive_bytes': size, 'index_members': len(members), 'indexed_scenes': len(scenes),
        'payload_range_bytes': sum(r['bytes'] for r in remote.ranges), 'ranges': remote.ranges,
        'sample_members': len(selected), 'sample_sha256': hashes,
        'scope': 'access/schema inspection only; CRC verifies retrieved ZIP members, not full-archive cryptographic identity',
        'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    (output / 'access-validation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('ranges', 'sample_sha256')}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--file-id', required=True)
    p.add_argument('--archive-bytes', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--sample', action='store_true')
    args = p.parse_args()
    inspect(args.file_id, args.archive_bytes, args.output, args.sample)
