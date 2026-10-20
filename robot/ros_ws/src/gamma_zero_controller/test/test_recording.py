from threading import Event
from pathlib import Path
import json
import pytest
from gamma_zero_controller.recording import Recorder


def test_recorder_flushes_all_accepted_records(tmp_path):
    path=tmp_path/'flight.jsonl'
    recorder=Recorder(path)
    for i in range(100):recorder.append({'i':i})
    recorder.close()
    assert [json.loads(line)['i'] for line in path.read_text().splitlines()]==list(range(100))


def test_slow_disk_does_not_block_append_and_queue_overflow_fails_closed(monkeypatch):
    entered,release=Event(),Event()
    class SlowFile:
        def write(self,line):
            entered.set()
            assert release.wait(timeout=2.)
        def close(self):
            pass
    monkeypatch.setattr(Path,'open',lambda *a,**kw:SlowFile())
    recorder=Recorder('unused',capacity=2)
    try:
        recorder.append({'i':0})
        assert entered.wait(timeout=1.)
        recorder.append({'i':1});recorder.append({'i':2})
        with pytest.raises(OSError,match='queue full'):recorder.append({'i':3})
    finally:
        release.set();recorder.close()


def test_disk_error_is_latched(monkeypatch):
    class BrokenFile:
        def write(self,line):
            raise OSError('disk full')
        def close(self):
            pass
    monkeypatch.setattr(Path,'open',lambda *a,**kw:BrokenFile())
    recorder=Recorder('unused')
    recorder.append({'i':0});recorder.thread.join(timeout=1.)
    with pytest.raises(OSError,match='disk full'):recorder.append({'i':1})
    with pytest.raises(OSError,match='disk full'):recorder.close()
