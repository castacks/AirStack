"""Atomic, same-container handoff from Raven to the network sender.

Do not send a growing full map across a best-effort local ROS subscription
before delta encoding. The disk snapshot is a single replaceable file, not a
queue, and is never counted as inter-robot traffic.
"""
import os
from pathlib import Path


def snapshot_path(robot):
    return Path(os.getenv('RAVEN_MAP_SNAPSHOT_DIR', '/tmp')) / f'raven_semantic_snapshot_{robot}.cdr'


def write_snapshot(robot, data):
    path = snapshot_path(robot)
    _write(path, data)


def _write(path, data):
    temporary = path.with_suffix(f'.{os.getpid()}.tmp')
    with temporary.open('wb') as stream:
        stream.write(data)
    temporary.replace(path)


def read_snapshot(robot, previous):
    return _read(snapshot_path(robot), previous)


def _read(path, previous):
    with path.open('rb') as stream:
        stat = os.fstat(stream.fileno())
        version = (stat.st_ino, stat.st_mtime_ns, stat.st_size)
        return (None, previous) if version == previous else (stream.read(), version)


def write_received_snapshot(robot, source, data):
    if not source.replace('_', '').isalnum():
        raise ValueError('invalid map source name')
    path = snapshot_path(robot).with_name(f'raven_received_map_{robot}_from_{source}.cdr')
    _write(path, data)


def received_snapshots(robot, previous):
    parent = snapshot_path(robot).parent
    prefix = f'raven_received_map_{robot}_from_'
    for path in parent.glob(prefix+'*.cdr'):
        source = path.name[len(prefix):-4]
        data, version = _read(path, previous.get(source))
        if data is not None:
            yield source, data, version
