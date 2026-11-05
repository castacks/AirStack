import threading
from coordination_bringup.map_snapshot import write_snapshot, read_snapshot


def test_growing_snapshot_handoff_is_atomic_and_detects_updates(monkeypatch, tmp_path):
    monkeypatch.setenv('RAVEN_MAP_SNAPSHOT_DIR', str(tmp_path))
    failures = []
    def writer():
        for size in [1000, 50000, 2_000_000, 20_000_000]:
            write_snapshot('robot_1', bytes([size % 251])*size)
    thread = threading.Thread(target=writer)
    thread.start()
    previous, observed = None, []
    while thread.is_alive():
        try:
            data, version = read_snapshot('robot_1', previous)
            if data is not None:
                assert data == bytes([len(data) % 251])*len(data)
                observed.append(len(data))
                previous = version
        except FileNotFoundError:
            pass
    thread.join()
    data, version = read_snapshot('robot_1', None)
    assert len(data) == 20_000_000
    assert read_snapshot('robot_1', version)[0] is None
