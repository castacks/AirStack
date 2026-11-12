"""Mission lifecycle checks: debugger never touches a healthy trial."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('runner_native_test',
    Path(__file__).parents[1] / 'mission_runner.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.parametrize('ok, capture_error', [(True, False), (False, False), (False, True)])
def test_capture_before_cleanup_only_on_failure(tmp_path, monkeypatch, ok, capture_error):
    events = []
    class Stack:
        env = {}
        num_robots = 3
        setup_bash = '/test/setup.bash'
        def ensure_down(self): pass
        def up(self): return ['test_robot']
        def wait_ready(self, container): return {'ready': True}
        def down(self): events.append('down')
    class Recorder:
        def __init__(self, *args): pass
        def start(self): events.append('record_start')
        def stop(self): events.append('record_stop')
        def collect(self, path): events.append('record_collect')
    def capture(summary, dest, image):
        # Use actual gate rather than attaching anything in this lifecycle test.
        if summary['status'] == 'passed':
            runner.capture_startup_failure_native_original(summary, dest, image)
            return
        events.append('capture')
        if capture_error:
            raise RuntimeError('diagnostic tool unavailable')
    monkeypatch.setattr(runner, 'capture_startup_failure_native_original',
                        runner.capture_startup_failure_native, raising=False)
    monkeypatch.setattr(runner, 'capture_startup_failure_native', capture)
    monkeypatch.setattr(runner, 'prepare_native_debugger', lambda stack: 'fixture_image')
    monkeypatch.setattr(runner, 'Recorder', Recorder)
    monkeypatch.setattr(runner, 'transport_snapshot_loop', lambda *args: None)
    monkeypatch.setattr(runner, 'docker_exec', lambda *args, **kwargs: None)
    monkeypatch.setattr(runner, 'run_step', lambda *args: {'ok': ok})
    monkeypatch.setattr(runner, 'snapshot_container_logs', lambda *args: None)
    monkeypatch.setattr(runner, 'snapshot_simulator_logs', lambda *args, **kwargs: None)
    monkeypatch.setattr(runner, 'snapshot_task_logs', lambda *args: None)
    monkeypatch.setattr(runner, 'snapshot_raven_results', lambda *args: None)
    monkeypatch.setattr(runner, 'upload_iteration', lambda *args: None)
    result = runner.run_iteration(Stack(), {'steps': [{}], 'on_step_failure': 'abort_iteration'}, tmp_path)
    assert result['status'] == ('passed' if ok else 'failed')
    assert events[-3:] == ['record_stop', 'record_collect', 'down']
    if ok:
        assert 'capture' not in events
    else:
        assert events.index('capture') < events.index('record_stop')


def test_default_does_not_require_debugger(monkeypatch):
    monkeypatch.setattr(runner, 'sh', lambda *args: pytest.fail('Unexpected docker call'))
    assert runner.prepare_native_debugger(SimpleNamespace(env={})) is None


def test_stopped_trial_never_attached(tmp_path):
    # Import happens only after the status gate; no capture module is loaded.
    runner.capture_startup_failure_native({'status': 'stopped'}, tmp_path, 'unused')
