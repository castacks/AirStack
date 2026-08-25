#!/usr/bin/env python3
"""
Build a patched copy of NVIDIA's `osmo` CLI whose UDP port-forward actually works.

Why this exists
---------------
`osmo workflow port-forward ... --udp` fails immediately on every 6.x release:

    TypeError: Passing coroutines is forbidden, use tasks explicitly.
      File "src/lib/utils/port_forward.py", line 303, in run_udp
      File "asyncio/tasks.py", line 429, in wait

Upstream (https://github.com/NVIDIA/OSMO, src/lib/utils/port_forward.py) does:

    _, pending = await asyncio.wait([send_datagram_to_router(),
                                     receive_datagram_from_router(transport)],
                                    return_when=asyncio.FIRST_COMPLETED)

asyncio.wait() rejects bare coroutines since Python 3.11, and the 6.3.x
binaries embed CPython 3.14 -- so the UDP leg dies before forwarding a single
datagram.  The TCP leg in the same file is fine: it already wraps its
coroutines in asyncio.create_task().  Still unfixed on main as of 6.3.1.

For AirStack this is what makes `airstack osmo:webrtc` show a black screen:
TCP 49100 (WebRTC signaling) connects, UDP 49099 (SRTP media) never does.

What this script does
---------------------
The CLI ships as a PyInstaller onedir bundle, so there is no source to edit.
Instead we:

  1. copy the installed bundle to a user-owned directory (default
     ~/.airstack/osmo-patched/osmo-cli) -- the system install is left alone;
  2. read the bundle's version from _internal/src/lib/utils/version.yaml and
     fetch the matching port_forward.py from GitHub;
  3. apply the one-line create_task() fix and recompile it with the *same*
     CPython minor version the bundle embeds (the .pyc magic must match);
  4. splice the new module into the embedded PYZ archive in place -- the blob
     is written into the existing slot and zero-padded, so no offset in the
     executable moves;
  5. re-sign the result (`codesign -f -s -`), which is mandatory on arm64
     because any edit invalidates the existing ad-hoc signature.

Re-run it after every `osmo` upgrade; it always re-copies from the install.

Usage
-----
    python3 patch_osmo_udp.py                     # patch a copy (default)
    python3 patch_osmo_udp.py --exe /path/to/osmo-cli
    python3 patch_osmo_udp.py --dest ~/somewhere
    python3 patch_osmo_udp.py --in-place          # patch the install itself
"""
from __future__ import annotations

import argparse
import importlib.util
import marshal
import os
import re
import shutil
import struct
import subprocess
import sys
import urllib.request
import zlib

MODULE = 'src.lib.utils.port_forward'
SOURCE_NAME = 'src/lib/utils/port_forward.py'
RAW_URL = 'https://raw.githubusercontent.com/NVIDIA/OSMO/{version}/' + SOURCE_NAME

# PyInstaller CArchive layout (v6.x).
CARCHIVE_MAGIC = b'MEI\014\013\012\013\016'
COOKIE_FMT = '!8sIIii64s'
COOKIE_SIZE = struct.calcsize(COOKIE_FMT)
ENTRY_FMT = '!IIIIBc'
ENTRY_SIZE = struct.calcsize(ENTRY_FMT)

BUGGY = """        _, pending = await asyncio.wait([send_datagram_to_router(),
                                        receive_datagram_from_router(transport)],
                                        return_when=asyncio.FIRST_COMPLETED)"""
FIXED = """        _, pending = await asyncio.wait([asyncio.create_task(send_datagram_to_router()),
                                        asyncio.create_task(receive_datagram_from_router(transport))],
                                        return_when=asyncio.FIRST_COMPLETED)"""


def die(msg: str) -> None:
    sys.exit(f'error: {msg}')


def find_installed_exe() -> str:
    """Resolve the `osmo` on PATH down to the real PyInstaller executable."""
    osmo = shutil.which('osmo')
    if not osmo:
        die('no `osmo` on PATH; pass --exe /path/to/osmo-cli')
    real = os.path.realpath(osmo)
    if not os.path.isdir(os.path.join(os.path.dirname(real), '_internal')):
        die(f'{real} does not look like a PyInstaller onedir bundle')
    return real


def read_carchive(data: bytes):
    """Return (archive_start, {name: (type, compressed, offset, dlen, ulen)}, pyvers)."""
    pos = data.rfind(CARCHIVE_MAGIC)
    if pos < 0:
        die('not a PyInstaller executable (no CArchive cookie)')
    _, pkg_len, toc_pos, toc_len, pyvers, _ = struct.unpack(COOKIE_FMT, data[pos:pos + COOKIE_SIZE])
    start = pos + COOKIE_SIZE - pkg_len
    toc = data[start + toc_pos:start + toc_pos + toc_len]
    entries, p = {}, 0
    while p < len(toc):
        elen, epos, dlen, ulen, flag, typ = struct.unpack(ENTRY_FMT, toc[p:p + ENTRY_SIZE])
        name = toc[p + ENTRY_SIZE:p + elen].rstrip(b'\0').decode()
        entries[name] = (typ.decode(), flag, epos, dlen, ulen)
        p += elen
    return start, entries, pyvers


def bundle_version(exe: str) -> str:
    """Read major/minor/revision out of the bundle's version.yaml -> '6.3.1'."""
    path = os.path.join(os.path.dirname(exe), '_internal', 'src', 'lib', 'utils', 'version.yaml')
    try:
        text = open(path).read()
    except OSError as err:
        die(f'cannot read {path}: {err}')
    fields = {}
    for key in ('major', 'minor', 'revision'):
        match = re.search(rf'^{key}:\s*(\d+)', text, re.M)
        if not match:
            die(f'no `{key}` in {path}')
        fields[key] = match.group(1)
    return '{major}.{minor}.{revision}'.format(**fields)


def fetch_source(version: str) -> str:
    url = RAW_URL.format(version=version)
    print(f'fetching {url}')
    try:
        with urllib.request.urlopen(url, timeout=30) as resp:
            return resp.read().decode()
    except Exception as err:  # noqa: BLE001 - surface whatever urllib raises
        die(f'could not fetch upstream source for {version}: {err}')


def reexec_matching_python(pyvers: int) -> None:
    """PYZ modules are .pyc: our compiler's magic must match the bundle's."""
    want = (pyvers // 100, pyvers % 100)
    if sys.version_info[:2] == want:
        return
    name = 'python%d.%d' % want
    alt = shutil.which(name)
    if not alt:
        die(f'the bundle embeds Python {want[0]}.{want[1]} but this is '
            f'{sys.version_info.major}.{sys.version_info.minor}; install {name} '
            f'(e.g. `brew install python@{want[0]}.{want[1]}`) and re-run')
    print(f're-executing under {alt} to match the bundle')
    os.execv(alt, [alt, os.path.abspath(__file__), *sys.argv[1:]])


def patch(exe: str) -> None:
    data = bytearray(open(exe, 'rb').read())
    start, entries, pyvers = read_carchive(data)
    reexec_matching_python(pyvers)

    if 'PYZ.pyz' in entries:
        pyz_name = 'PYZ.pyz'
    else:
        pyz_name = next((n for n, e in entries.items() if e[0] == 'z'), None)
        if pyz_name is None:
            die('no PYZ archive in the bundle')
    _, compressed, epos, dlen, _ = entries[pyz_name]
    if compressed:
        die('PYZ is stored compressed; in-place patching is not supported')
    off = start + epos
    pyz = bytearray(data[off:off + dlen])

    if pyz[:4] != b'PYZ\0':
        die('PYZ header not found where the CArchive TOC says it is')
    if bytes(pyz[4:8]) != importlib.util.MAGIC_NUMBER:
        die(f'pyc magic mismatch: bundle {bytes(pyz[4:8]).hex()} vs this python '
            f'{importlib.util.MAGIC_NUMBER.hex()}')
    toc_pos = struct.unpack('!I', pyz[8:12])[0]
    toc = dict(marshal.loads(bytes(pyz[toc_pos:])))
    if MODULE not in toc:
        die(f'{MODULE} is not in the PYZ archive')
    kind, mod_pos, mod_len = toc[MODULE]
    old_code = marshal.loads(zlib.decompress(bytes(pyz[mod_pos:mod_pos + mod_len])))

    version = bundle_version(exe)
    print(f'osmo version: {version}')
    source = fetch_source(version)
    if BUGGY not in source:
        die(f'{version} no longer contains the buggy asyncio.wait() call -- '
            f'it may already be fixed upstream; this patch is not needed')
    code = compile(source.replace(BUGGY, FIXED), SOURCE_NAME, 'exec')
    if code.co_firstlineno != old_code.co_firstlineno or code.co_names != old_code.co_names:
        print('warning: recompiled module differs structurally from the bundled one; '
              'the upstream tag may not match this binary')

    blob = zlib.compress(marshal.dumps(code), 9)
    print(f'patched module: {len(blob)} bytes into a {mod_len} byte slot')
    if len(blob) > mod_len:
        die(f'recompiled module is {len(blob) - mod_len} bytes too large for its slot')

    # Overwrite the slot and record the shorter length; the PYZ keeps its exact
    # size so every offset in the executable stays valid.
    pyz[mod_pos:mod_pos + mod_len] = blob + b'\0' * (mod_len - len(blob))
    toc[MODULE] = (kind, mod_pos, len(blob))
    new_toc = marshal.dumps(toc, 4)
    tail = len(pyz) - toc_pos
    if len(new_toc) > tail:
        die('rewritten PYZ directory does not fit')
    pyz[toc_pos:] = new_toc + b'\0' * (tail - len(new_toc))
    assert len(pyz) == dlen

    # Read it back the way the frozen importer will, before touching the file.
    check = dict(marshal.loads(bytes(pyz[toc_pos:])))
    if check.keys() != toc.keys():
        die('PYZ directory round-trip lost entries')
    _, pos, length = check[MODULE]
    marshal.loads(zlib.decompress(bytes(pyz[pos:pos + length])))

    data[off:off + dlen] = pyz
    mode = os.stat(exe).st_mode
    with open(exe, 'wb') as handle:
        handle.write(data)
    os.chmod(exe, mode)
    if sys.platform == 'darwin':
        # Editing the executable invalidates its ad-hoc signature, and macOS
        # kills unsigned arm64 binaries outright.
        subprocess.run(['codesign', '--force', '--sign', '-', exe], check=True)
        print(f'patched and re-signed: {exe}')
    else:
        print(f'patched: {exe}')


def main() -> None:
    default_dest = os.path.expanduser('~/.airstack/osmo-patched')
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[1],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--exe', help='PyInstaller executable to patch (default: `osmo` on PATH)')
    parser.add_argument('--dest', default=default_dest,
                        help=f'where to place the patched copy (default: {default_dest})')
    parser.add_argument('--in-place', action='store_true',
                        help='patch the installed bundle directly instead of copying it')
    args = parser.parse_args()

    installed = args.exe or find_installed_exe()
    if args.in_place:
        target = installed
    else:
        src_dir = os.path.dirname(installed)
        dest_dir = os.path.join(os.path.expanduser(args.dest), os.path.basename(src_dir))
        if os.path.exists(dest_dir):
            shutil.rmtree(dest_dir)
        print(f'copying {src_dir} -> {dest_dir}')
        shutil.copytree(src_dir, dest_dir, symlinks=True)
        target = os.path.join(dest_dir, os.path.basename(installed))
    patch(target)


if __name__ == '__main__':
    main()
