#!/usr/bin/env python3
"""Capture a selected robot PID with a short-lived debugger sidecar.

The debugger image must already contain /usr/bin/gdb and /usr/bin/timeout.
No package is installed in the target. GDB briefly pauses the selected process;
use this for a diagnosed invalid/stalled trial, not a healthy measurement run.
"""
import argparse
import json
import subprocess
import time
import uuid
from pathlib import Path


def run(args, timeout=10):
    return subprocess.run(args, text=True, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=timeout)


def status(container, pid):
    result = run(['docker', 'exec', container, 'cat', f'/proc/{pid}/status'])
    if result.returncode:
        raise RuntimeError(result.stdout)
    return dict(line.split(':', 1) for line in result.stdout.splitlines() if ':' in line)


def capture(container, pid, image, output, timeout_s=8):
    if timeout_s <= 0 or timeout_s > 30:
        raise ValueError('capture timeout must be in (0,30] seconds')
    before = status(container, pid)
    if int(before['TracerPid']) or before['State'].strip().startswith(('T', 't')):
        raise RuntimeError('Target is already traced or stopped; refusing to attach')
    helper = 'raven-stack-' + uuid.uuid4().hex[:12]
    command = ['docker', 'run', '--rm', '--pull=never', '--name', helper,
               '--pid=container:' + container, '--cap-add=SYS_PTRACE',
               '--security-opt=seccomp=unconfined', '--entrypoint=/usr/bin/timeout',
               image, '--signal=TERM', '--kill-after=2s', str(timeout_s) + 's',
               '/usr/bin/gdb', '-nx', '-q', '-batch',
               '-ex', 'set pagination off', '-ex', 'set confirm off',
               '-ex', f'set sysroot /proc/{pid}/root',
               '-ex', f'file /proc/{pid}/exe', '-ex', f'attach {pid}',
               '-ex', 'thread apply all bt', '-ex', 'detach']
    started = time.monotonic()
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    result = None
    try:
        result = run(command, timeout_s + 12)
        output.write_text(result.stdout)
    except subprocess.TimeoutExpired as exc:
        value = exc.stdout or b''
        output.write_text(value.decode(errors='replace') if isinstance(value, bytes) else value)
    finally:
        # Also terminate the daemon-side container if the Docker client timed out.
        # This name was created by this call; never stop the target container.
        run(['docker', 'stop', '--time=2', helper], timeout=6)
        run(['docker', 'rm', '-f', helper], timeout=6)
    after = status(container, pid)
    resumed = False
    if int(after['TracerPid']) == 0 and after['State'].strip().startswith(('T', 't')):
        # Restore only a stop introduced by this attachment (pre-state checked).
        run(['docker', 'exec', container, 'kill', '-CONT', str(pid)])
        resumed = True
        after = status(container, pid)
    metadata = {'container': container, 'pid': pid, 'debugger_image': image,
                'exit_code': result.returncode if result else None,
                'elapsed_s': time.monotonic() - started,
                'before': before, 'after': after, 'restored_continuation': resumed}
    output.with_suffix(output.suffix + '.json').write_text(json.dumps(metadata, indent=2) + '\n')
    if int(after['TracerPid']) or after['State'].strip().startswith(('T', 't')):
        raise RuntimeError('Target remains traced/stopped after capture; inspect immediately')
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('container')
    parser.add_argument('pid', type=int)
    parser.add_argument('output')
    parser.add_argument('--debugger-image', required=True)
    parser.add_argument('--timeout', type=float, default=8)
    args = parser.parse_args()
    if args.pid <= 0:
        parser.error('PID must be positive')
    result = capture(args.container, args.pid, args.debugger_image, args.output, args.timeout)
    print(json.dumps({k: result[k] for k in ('exit_code', 'elapsed_s', 'restored_continuation')}))
    return 0 if result['exit_code'] == 0 else 1


if __name__ == '__main__':
    raise SystemExit(main())
