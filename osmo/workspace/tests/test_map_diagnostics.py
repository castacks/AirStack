"""Run in the deployment image; includes a real tmux history-overflow check."""
import importlib.util
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace

import pytest
from raven_nav.map_diagnostics import profile_map_stage


def test_profile_preserves_results_and_errors(monkeypatch):
    messages = []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=messages.append))

    @profile_map_stage
    def operation(self, fail=False):
        if fail:
            raise ValueError('original error')
        return 42

    monkeypatch.delenv('RAVEN_MAP_PROFILE', raising=False)
    assert operation(node) == 42
    assert messages == []
    monkeypatch.setenv('RAVEN_MAP_PROFILE', 'true')
    assert operation(node) == 42
    with pytest.raises(ValueError, match='original error'):
        operation(node, fail=True)
    assert len(messages) == 2
    assert all('stage=operation wall_s=' in message for message in messages)


def test_console_capture_retains_onset_after_periodic_snapshot_and_history_overflow(monkeypatch, tmp_path):
    path = Path(__file__).resolve().parents[1] / 'mission_runner.py'
    spec = importlib.util.spec_from_file_location('mission_runner', path)
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    # Dedicated test container only; exercise the exact capture shell command.
    subprocess.run(['tmux', 'new-session', '-d', '-s', 'isaac'], check=True)
    monkeypatch.setattr(runner, 'list_containers', lambda *args, **kwargs: ['isaac-sim'])
    monkeypatch.setattr(runner, 'docker_exec', lambda name, command, timeout:
        subprocess.run(['bash', '-c', command], capture_output=True, text=True, timeout=timeout))
    def fake_sh(command, timeout=None):
        if command[:2] == ['docker', 'cp']:
            import shutil
            shutil.copyfile(command[2].split(':', 1)[1], command[3])
            return SimpleNamespace(returncode=0, stdout='', stderr='')
        if command[:2] == ['docker', 'top']:
            return SimpleNamespace(returncode=0, stdout='', stderr='')
        raise AssertionError(command)

    monkeypatch.setattr(runner, 'sh', fake_sh)
    try:
        runner.start_simulator_console_capture()
        # The real periodic collector used to stop the pipe before freeze onset.
        runner.snapshot_simulator_logs(tmp_path / 'periodic')
        assert subprocess.check_output(
            ['tmux', 'display-message', '-p', '-t', 'isaac', '#{pane_pipe}'],
            text=True).strip() == '1'
        subprocess.run(['tmux', 'send-keys', '-t', 'isaac',
            "echo FREEZE_ONSET_MARKER; seq 1 5000; echo STREAM_END_MARKER", 'Enter'], check=True)
        log = Path('/tmp/osmo_isaac_console.log')
        for _ in range(100):
            if '\n5000\n' in log.read_text() and '\nSTREAM_END_MARKER\n' in log.read_text():
                break
            time.sleep(.05)
        runner.snapshot_simulator_logs(tmp_path / 'periodic_second')
        assert subprocess.check_output(
            ['tmux', 'display-message', '-p', '-t', 'isaac', '#{pane_pipe}'],
            text=True).strip() == '1'
        runner.snapshot_simulator_logs(tmp_path / 'final', stop_capture=True)
        assert subprocess.check_output(
            ['tmux', 'display-message', '-p', '-t', 'isaac', '#{pane_pipe}'],
            text=True).strip() == '0'
        text = (tmp_path / 'final' / 'isaac-sim_console_full.log').read_text()
        assert '\nFREEZE_ONSET_MARKER\n' in text and '\n5000\n' in text
        pane = subprocess.check_output(['tmux', 'capture-pane', '-p', '-t', 'isaac', '-S', '-2000'], text=True)
        assert 'FREEZE_ONSET_MARKER' not in pane
    finally:
        subprocess.run(['tmux', 'kill-session', '-t', 'isaac'], check=False)
