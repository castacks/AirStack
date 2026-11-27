#!/usr/bin/env python3
"""Offline only: characterize immutable startup records; no runtime/service APIs."""
import argparse
import csv
from datetime import datetime, timezone
from decimal import Decimal
import io
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'RRM/evidence/M00-D'
SOURCE = ROOT / 'RRM/evidence/M00-C/startup-source-records.json'


def epoch(text):
    match = re.search(r'\[([0-9]+\.[0-9]+)\]', text)
    if not match:
        raise ValueError('Missing producer epoch timestamp')
    return Decimal(match.group(1))


def receipt(text):
    match = re.match(r'(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)\.(\d{1,9})Z', text)
    if not match:
        raise ValueError('Missing nanosecond UTC receipt stamp')
    whole = datetime.strptime(match.group(1), '%Y-%m-%dT%H:%M:%S').replace(tzinfo=timezone.utc)
    return Decimal(int(whole.timestamp())) + Decimal('0.' + match.group(2))


def delta(start, stop):
    if start['domain'] != stop['domain']:
        raise ValueError('Unmapped clock domains')
    if start.get('episode') != stop.get('episode'):
        raise ValueError('Different startup episodes')
    value = stop['value'] - start['value']
    if value < 0:
        raise ValueError('Reversed event or clock discontinuity')
    return value


def record(doc, container, needle):
    group = next(g for g in doc['filtered_logs'] if g['container'] == container)
    for index, item in enumerate(group['records']):
        if needle in item['text']:
            return {'domain': 'HOST_DOCKER_RECEIPT_UTC', 'value': receipt(item['text']),
                    'episode': '20261127_STARTUP_01', 'source':
                    'RRM/evidence/M00-C/startup-source-records.json#/filtered_logs/%d/records/%d' %
                    (doc['filtered_logs'].index(group), index)}
    return None


def graph_order(graph):
    ids = {n['id'] for n in graph['nodes']}
    if len(ids) != len(graph['nodes']):
        raise ValueError('Duplicate graph node')
    dependencies = {key: set() for key in ids}
    for edge in graph['edges']:
        if edge['from'] not in ids or edge['to'] not in ids:
            raise ValueError('Unknown dependency endpoint')
        dependencies[edge['to']].add(edge['from'])
    order = []
    while dependencies:
        ready = sorted(key for key, value in dependencies.items() if not value)
        if not ready:
            raise ValueError('Dependency cycle')
        order += ready
        for key in ready:
            del dependencies[key]
        for value in dependencies.values():
            value.difference_update(ready)
    return order


def characterize(doc):
    sim = 'isaac-sim-livestream'
    robot = 'airstack-robot-desktop-1'
    events = {
        'app_start': record(doc, sim, 'Simulation App Starting'),
        'app_ready': record(doc, sim, '] app ready'),
        'app_complete': record(doc, sim, 'Simulation App Startup Complete'),
        'graph': record(doc, sim, 'PX4 node at'),
        'px4_start_log': record(doc, sim, 'px4 starting.'),
        'tcp': record(doc, sim, 'Simulator connected on TCP port'),
        'pegasus_heartbeat': record(doc, sim, 'Received first hearbeat'),
        'udp': record(doc, sim, 'on udp port 14581 remote port 14541'),
        'px4_script_return': record(doc, sim, 'Startup script returned successfully'),
        'mavros_heartbeat_receipt': record(doc, robot, 'CON: Got HEARTBEAT'),
        'commander_ready': record(doc, sim, 'Ready for takeoff!')}
    metrics = []

    def measured(key, title, start, stop, meaning):
        a, b = events.get(start), events.get(stop)
        if not a or not b:
            metrics.append({'measurement_id': key, 'quantity': title, 'seconds': '', 'domain': '',
                            'n_startup_episodes': '0', 'status': 'MISSING', 'source_start': start,
                            'source_stop': stop, 'interpretation_limit': meaning})
            return
        value = delta(a, b)
        metrics.append({'measurement_id': key, 'quantity': title, 'seconds': str(value),
                        'domain': a['domain'], 'n_startup_episodes': '1', 'status': 'CHARACTERIZATION_ONLY',
                        'source_start': a['source'], 'source_stop': b['source'],
                        'interpretation_limit': meaning + '; log buffering/receipt delay unknown; not a source-acquisition duration or acceptance result'})

    measured('L01', 'Application starting log to app-ready log', 'app_start', 'app_ready',
             'Application messages only; not container startup, scene or PX4 readiness')
    measured('L02', 'App-ready to application-startup-complete log', 'app_ready', 'app_complete',
             'Separate application initialization phase; not flight readiness')
    measured('L03', 'Application-startup-complete to vehicle-graph-created log', 'app_complete', 'graph',
             'Uninstrumented interval includes scene/application work; cannot assign it entirely to scene loading')
    measured('L04', 'Vehicle-graph-created to PX4-starting log', 'graph', 'px4_start_log',
             'Includes graph evaluation/backend/spawn work; not pure PX4 executable initialization')
    measured('L05', 'PX4-starting to startup-script-returned log', 'px4_start_log', 'px4_script_return',
             'PX4 logged initialization phase; neither total launch-to-flight-ready duration nor pure CPU work')
    measured('L06', 'PX4-starting to simulator-TCP-connected log', 'px4_start_log', 'tcp',
             'Simulator transport connection after logged PX4 start; not companion UDP discovery')
    measured('L07', 'Simulator-TCP-connected to Pegasus-heartbeat-received log', 'tcp', 'pegasus_heartbeat',
             'Simulator/backend path; cannot identify heartbeat emission or loss latency')
    measured('L08', 'Companion-UDP-initialized to MAVROS-connection log receipts', 'udp', 'mavros_heartbeat_receipt',
             'Two producer containers share Docker receipt clock; buffering unknown; not wire transport or DDS latency')
    measured('L09', 'App-ready to PX4-starting log', 'app_ready', 'px4_start_log',
             'Shows early app-ready is insufficient; broad pre-PX4 application/scene/backend interval')
    measured('L10', 'PX4-starting to commander-ready log', 'px4_start_log', 'commander_ready',
             'PX4 message sequence; commander-ready does not clear AirStack failed initializer')
    logs = doc['original_ros_logs']['logs']
    starts = [(i, line) for i, p in enumerate(logs) for line in p['text'].splitlines()
              if 'Actuation NOT_READY: STARTUP_DEADLINE' in line]
    stops = [(i, line) for i, p in enumerate(logs) for line in p['text'].splitlines()
             if 'CON: Got HEARTBEAT' in line]
    if not starts or not stops:
        raise ValueError('Missing original deadline/heartbeat producer evidence')
    start = {'domain': 'ROS_LOG_EPOCH', 'episode': '20261127_STARTUP_01', 'value': epoch(starts[0][1])}
    stop = {'domain': 'ROS_LOG_EPOCH', 'episode': '20261127_STARTUP_01', 'value': epoch(stops[0][1])}
    metrics.append({'measurement_id': 'L11', 'quantity': 'Deadline error to first recorded MAVROS connection',
                    'seconds': str(delta(start, stop)), 'domain': 'ROS_LOG_EPOCH', 'n_startup_episodes': '1',
                    'status': 'CHARACTERIZATION_ONLY',
                    'source_start': 'RRM/evidence/M00-C/startup-source-records.json#/original_ros_logs/logs/%d/text' % starts[0][0],
                    'source_stop': 'RRM/evidence/M00-C/startup-source-records.json#/original_ros_logs/logs/%d/text' % stops[0][0],
                    'interpretation_limit': '19.276162179s is NOT total PX4 startup time; no historical clock-step calibration; producer log interval only'})
    a, b = doc['original_ros_logs'], doc['isaac_processes']
    if a['boot_id'] != b['boot_id'] or a['clk_tck'] != b['clk_tck'] or a['clk_tck'] <= 0:
        raise ValueError('Unmatched kernel boot or tick frequency')
    interface = next(p for p in a['processes'] if p['pid'] == 20295)
    px4 = next(p for p in b['processes'] if p['exe'].endswith('/px4'))
    ticks = px4['start_ticks'] - interface['start_ticks']
    if ticks < 0:
        raise ValueError('Unexpected process ordering')
    metrics.append({'measurement_id': 'L12', 'quantity': 'PX4 process birth after interface process birth',
                    'seconds': str(Decimal(ticks) / Decimal(a['clk_tck'])),
                    'domain': 'SAME_BOOT_KERNEL_TICKS_CONVERTED_TO_SECONDS', 'n_startup_episodes': '1',
                    'status': 'CHARACTERIZATION_ONLY',
                    'source_start': 'RRM/evidence/M00-C/startup-source-records.json#/original_ros_logs/processes',
                    'source_stop': 'RRM/evidence/M00-C/startup-source-records.json#/isaac_processes/processes',
                    'interpretation_limit': 'Process start separation, not initialization duration. 100Hz tick resolution; conservative delta quantization <20ms; no UTC mapping'})
    missing = [
        ('L13', 'Workflow submission/initialization latency', 'Submission, scheduling, pull/init and deployed workflow ID/timestamps'),
        ('L14', 'Scene-load exclusive duration', 'Explicit scene begin/end plus asset/shader/I/O subspans and epoch'),
        ('L15', 'Actual PX4 spawn-to-executable-ready duration', 'Spawner monotonic begin/end and child lifecycle/ready edge; kernel start is a different boundary'),
        ('L16', 'ROS2/DDS discovery-to-guard callback latency', 'Per-endpoint discovery and source/acquisition/receipt stamps on common mapped clock'),
        ('L17', 'Guard exact deadline-evaluation duration/jitter', 'Guard steady origin and each timer callback/lock contention/elapsed sample'),
        ('L18', 'Initial readiness-latch transitions', 'Complete safe-state receipt/transaction and Boolean/status transition history'),
        ('L19', 'Historical infrastructure/resource contributions', 'Per-phase CPU/GPU/scheduler/cgroup/I/O/network telemetry with clock map')]
    for key, title, needed in missing:
        metrics.append({'measurement_id': key, 'quantity': title, 'seconds': '', 'domain': 'UNOBSERVED',
                        'n_startup_episodes': '0', 'status': 'MISSING', 'source_start': 'EXTERNAL:UNRECORDED_STARTUP_INSTRUMENTATION',
                        'source_stop': '', 'interpretation_limit': 'Required instrumentation: ' + needed})
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Read-only compare with derived output')
    args = parser.parse_args()
    metrics = characterize(json.loads(SOURCE.read_text()))
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=list(metrics[0]), lineterminator='\n')
    writer.writeheader()
    writer.writerows(metrics)
    summary = {'startup_episode_count': 1, 'new_startup_trials': 0,
               'measured_characterization_intervals': sum(m['status'] == 'CHARACTERIZATION_ONLY' for m in metrics),
               'missing_measurements': sum(m['status'] == 'MISSING' for m in metrics),
               'independent_deadline_interval_s': next(m['seconds'] for m in metrics if m['measurement_id'] == 'L11'),
               'startup_variability': 'NOT_ESTIMABLE_N1; no standard deviation, p95 or confidence interval reported',
               'tier_executed': 'A_OFFLINE_AND_READ_ONLY_METADATA_ONLY', 'acceptance_thresholds_created': False}
    outputs = [(OUT / 'latency-measurements.csv', stream.getvalue()),
               (OUT / 'characterization-summary.json', json.dumps(summary, indent=2) + '\n')]
    for path, text in outputs:
        if args.check:
            if path.read_text() != text:
                raise ValueError('Nonreproducible output: ' + str(path))
        else:
            with path.open('x') as f:
                f.write(text)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
