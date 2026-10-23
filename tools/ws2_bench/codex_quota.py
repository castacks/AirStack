"""Read official Codex subscription quota and notify Mason at <=1% weekly left.

No tokens or credential files are read. The official app-server owns login and
contacts account/rateLimits/read. This does not start a model turn. Missing or
unavailable weekly limits are UNKNOWN, never interpreted as unlimited usage.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import time


DEFAULT_STATE = Path(__file__).resolve().parents[2] / 'robot/ros_ws/ws2_runtime/codex_quota.json'
WEEK_MINUTES = 7 * 24 * 60
THRESHOLD_PERCENT = 1.0


class QuotaLowError(RuntimeError):
    """New Codex calls must wait for a subscription reset or provider change."""


class QuotaUnknownError(RuntimeError):
    """Quota cannot be confirmed; do not silently spend through the guard."""


def _atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False, suffix='.tmp') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')
        temporary = Path(f.name)
    temporary.replace(path)


def subscription_environment():
    conflicts = [key for key in ('OPENAI_API_KEY', 'CODEX_API_KEY', 'OPENAI_BASE_URL',
                                 'AZURE_OPENAI_API_KEY', 'CODEX_OSS_BASE_URL') if os.environ.get(key)]
    if conflicts:
        raise ValueError('Codex subscription mode conflicts with: ' + ', '.join(conflicts))
    return dict(os.environ)


class AccountClient:
    """Short-lived stdio JSON-RPC client; intentionally has no turn/start API."""

    def __init__(self, executable=None, timeout_s=30):
        self.executable = executable or shutil.which('codex')
        if not self.executable:
            raise RuntimeError('Install Codex CLI and run: codex login')
        self.timeout_s = timeout_s
        self._id = 0
        self._queue = queue.Queue()

    def __enter__(self):
        env = subscription_environment()
        # Keep account auth in Codex's normal location; unrelated thread state
        # and logs go to a disposable directory, not the active IDE database.
        self._temporary = tempfile.TemporaryDirectory(prefix='ws2-codex-account-')
        argv = [self.executable, 'app-server', '--listen', 'stdio://']
        config = {'sqlite_home': self._temporary.name, 'log_dir': self._temporary.name,
                  'forced_login_method': 'chatgpt', 'features.plugins': False,
                  'features.apps': False, 'features.hooks': False}
        for key, value in config.items():
            argv += ['-c', key + '=' + json.dumps(value)]
        try:
            self.process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL, text=True, env=env)
            self._thread = threading.Thread(target=self._read, daemon=True)
            self._thread.start()
            self.request('initialize', {'clientInfo': {'name': 'ws2_quota', 'version': '1.0'}})
            self._send({'method': 'initialized', 'params': {}})
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def _read(self):
        for line in self.process.stdout:
            try:
                self._queue.put(json.loads(line))
            except ValueError:
                continue
        self._queue.put(None)

    def _send(self, value):
        self.process.stdin.write(json.dumps(value) + '\n')
        self.process.stdin.flush()

    def request(self, method, params=None):
        if method not in {'initialize', 'account/read', 'account/rateLimits/read', 'model/list'}:
            raise ValueError('Only read-only account/model RPC methods are permitted')
        self._id += 1
        self._send({'id': self._id, 'method': method, 'params': params or {}})
        deadline = time.monotonic() + self.timeout_s
        while True:
            try:
                message = self._queue.get(timeout=max(.01, deadline - time.monotonic()))
            except queue.Empty:
                raise RuntimeError('Codex account lookup timed out') from None
            if message is None:
                raise RuntimeError('Codex app-server exited before the account lookup completed')
            if message.get('id') != self._id:
                continue
            if 'error' in message:
                # Do not persist raw account errors, which can include identity.
                raise RuntimeError(f'Codex {method} failed (code {message["error"].get("code")})')
            return message.get('result', {})

    def check_auth(self):
        result = self.request('account/read', {'refreshToken': False})
        account = result.get('account') or {}
        if account.get('type') != 'chatgpt':
            raise RuntimeError('A saved ChatGPT subscription login is required; run: codex login')
        return {'loggedIn': True, 'authMethod': 'chatgpt', 'planType': account.get('planType')}

    def __exit__(self, *_):
        process = getattr(self, 'process', None)
        if process:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
            for stream in (process.stdin, process.stdout):
                if stream:
                    stream.close()
        temporary = getattr(self, '_temporary', None)
        if temporary:
            temporary.cleanup()


def weekly_snapshot(response, *, limit_id='codex', now=None):
    """Select the documented weekly window, not an assumed 'secondary' slot."""
    now = time.time() if now is None else now
    snapshot = {'checked_at': now, 'source': 'codex_app_server_account/rateLimits/read',
                'limit_id': limit_id, 'status': 'unknown', 'remaining_percent': None,
                'resets_at': None, 'blocked': True}
    buckets = response.get('rateLimitsByLimitId') or {}
    bucket = buckets.get(limit_id)
    legacy = response.get('rateLimits') or {}
    if bucket is None and legacy.get('limitId') == limit_id:
        bucket = legacy
    if not isinstance(bucket, dict):
        return {**snapshot, 'reason': 'Requested subscription quota bucket is unavailable'}
    candidates = []
    for slot in ('primary', 'secondary'):
        window = bucket.get(slot) or {}
        if window.get('windowDurationMins') != WEEK_MINUTES:
            continue
        used, reset = window.get('usedPercent'), window.get('resetsAt')
        if (isinstance(used, (float, int)) and not isinstance(used, bool)
                and math.isfinite(used) and 0 <= used <= 100
                and isinstance(reset, (float, int)) and not isinstance(reset, bool)
                and math.isfinite(reset) and reset > now):
            candidates.append((100.0 - used, reset, slot))
    if not candidates:
        return {**snapshot, 'reason': 'No current weekly usage window was returned'}
    remaining, reset, slot = min(candidates)
    return {**snapshot, 'status': 'low' if remaining <= THRESHOLD_PERCENT else 'ok',
            'remaining_percent': remaining, 'resets_at': reset, 'window_slot': slot,
            'window_minutes': WEEK_MINUTES, 'blocked': remaining <= THRESHOLD_PERCENT}


def read_quota(executable=None):
    with AccountClient(executable) as client:
        client.check_auth()
        return weekly_snapshot(client.request('account/rateLimits/read'))


def send_slack(message):
    """Use the user-authorized helper; helper exit 0 alone does not mean sent."""
    helper = Path.home() / 'notify_slack.py'
    if not helper.is_file():
        return False
    if os.environ.get('SLACK_WEBHOOK_URL'):
        argv = [sys.executable, str(helper), message]
    else:
        # Pass message as argv, never interpolate it into shell code. The login
        # file owns its webhook and no credential is copied into our state/logs.
        argv = ['bash', '-ic', 'python3 "$HOME/notify_slack.py" "$1"', 'ws2-quota', message]
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return not result.returncode and not any(x in result.stderr for x in
            ('warning: SLACK_WEBHOOK_URL', 'warning: Slack notification failed', 'Traceback',
             'No such file or directory', 'command not found'))


@contextmanager
def _locked(path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.with_suffix(path.suffix + '.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        yield


def record_quota(snapshot, state_path=DEFAULT_STATE, *, notify=True, notifier=send_slack):
    """Persist only usage fields. Deduplicate alerts across watcher/providers."""
    path = Path(state_path)
    with _locked(path):
        try:
            previous = json.loads(path.read_text())
        except (FileNotFoundError, ValueError):
            previous = {}
        key = f'{snapshot.get("limit_id")}:{snapshot.get("resets_at")}'
        # A manual reset can restore quota without changing the window end.
        same_episode = (previous.get('alert_window') == key
                        and previous.get('alert_active', False))
        state = {**snapshot, 'alert_window': key, 'alert_active': snapshot['status'] == 'low',
                 'alerts_sent': previous.get('alerts_sent', 0) if same_episode else 0,
                 'notification_failures': previous.get('notification_failures', 0) if same_episode else 0}
        if snapshot['status'] == 'unknown':
            # A transient lookup failure must not rearm an already delivered alert.
            for field in ('alert_window', 'alert_active', 'alerts_sent', 'notification_failures'):
                if field in previous:
                    state[field] = previous[field]
        _atomic_json(path, state)
        if notify and snapshot['status'] == 'low':
            reset = datetime.fromtimestamp(snapshot['resets_at'], timezone.utc).isoformat()
            while state['alerts_sent'] < 3:
                number = state['alerts_sent'] + 1
                message = (f'Codex weekly quota alert {number}/3: '
                           f'{snapshot["remaining_percent"]:g}% remaining. '
                           'New WS2 Codex requests are paused. Please reset the quota or select Claude. '
                           f'Current reset time: {reset}.')
                if notifier(message):
                    state['alerts_sent'] += 1
                else:
                    state['notification_failures'] += 1
                    _atomic_json(path, state)
                    break  # Retry the missing notifications on the next observation.
                _atomic_json(path, state)
                if notifier is send_slack and state['alerts_sent'] < 3:
                    time.sleep(1.1)  # Respect Slack's per-webhook message cadence.
        return state


def check_quota(state_path=DEFAULT_STATE, *, executable=None, notify=True):
    try:
        snapshot = read_quota(executable)
    except (RuntimeError, ValueError, OSError) as exc:
        snapshot = {'checked_at': time.time(), 'status': 'unknown', 'remaining_percent': None,
                    'resets_at': None, 'limit_id': 'codex', 'blocked': True,
                    'reason': str(exc), 'source': 'codex_app_server_account/rateLimits/read'}
    return record_quota(snapshot, state_path, notify=notify)


def ensure_quota(state_path=DEFAULT_STATE, *, executable=None):
    state = check_quota(state_path, executable=executable)
    if state['status'] == 'low':
        raise QuotaLowError('Codex weekly quota has <=1% remaining; new requests are paused')
    if state['status'] != 'ok':
        raise QuotaUnknownError('Codex weekly quota could not be verified; no model call was made')
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', type=Path, default=DEFAULT_STATE)
    parser.add_argument('--watch', action='store_true', help='Monitor independently of any bench campaign')
    parser.add_argument('--interval', type=float, default=30, help='Polling interval in seconds (>=10)')
    parser.add_argument('--status-only', action='store_true', help='Observe without sending notifications')
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval < 10:
        parser.error('--interval must be at least 10 seconds')
    args.state.parent.mkdir(parents=True, exist_ok=True)
    # Avoid duplicate permanent watcher processes, even though alerts also have a lock.
    lock = None
    if args.watch:
        lock = args.state.with_suffix('.watch.lock').open('a+')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(0, 'A quota watcher for this state file is already running.\n')
        lock.seek(0); lock.truncate(); lock.write(str(os.getpid())); lock.flush()
    while True:
        state = check_quota(args.state, notify=not args.status_only)
        print(json.dumps({k: state.get(k) for k in ('checked_at', 'status', 'remaining_percent',
              'resets_at', 'alerts_sent', 'notification_failures', 'reason')}), flush=True)
        if not args.watch:
            return 0 if state['status'] == 'ok' else 2
        time.sleep(args.interval)


if __name__ == '__main__':
    raise SystemExit(main())
