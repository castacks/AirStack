#!/usr/bin/env python3
"""Offline evidence analysis. Rewrites only M00-C derived outputs; --check is read-only."""
import argparse
import csv
from datetime import datetime, timezone
from decimal import Decimal
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'RRM/evidence/M00-C'
REPORTS = ROOT / 'RRM/verification/reports/M00-C'


def ros_stamp(text):
    found = re.search(r'\[([0-9]+\.[0-9]+)\]', text)
    if not found:
        raise ValueError('No producer epoch timestamp')
    return Decimal(found.group(1))


def utc_stamp(text):
    found = re.match(r'(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)(?:\.(\d{1,9}))?Z', text)
    if not found:
        raise ValueError('No Docker UTC receipt timestamp')
    whole = datetime.strptime(found.group(1), '%Y-%m-%dT%H:%M:%S').replace(tzinfo=timezone.utc)
    return Decimal(int(whole.timestamp())) + Decimal('0.' + (found.group(2) or '0'))


def difference(later, earlier):
    if later['clock_domain'] != earlier['clock_domain']:
        raise ValueError('Clock-domain mismatch: explicit mapping required')
    return Decimal(later['timestamp_s']) - Decimal(earlier['timestamp_s'])


def first_line(lines, needle):
    hits = [(i, s) for i, s in enumerate(lines, 1) if needle in s]
    if not hits:
        raise ValueError('Missing required evidence: ' + needle)
    return hits[0]


def analyze(doc, prior):
    logs = doc['original_ros_logs']['logs']
    interface = next(x for x in logs if 'robot_interface_node_' in x['path'])
    mavros = next(x for x in logs if 'mavros_node_' in x['path'])
    deadline_line, deadline_text = first_line(interface['text'].splitlines(), 'Actuation NOT_READY: STARTUP_DEADLINE')
    heartbeat_line, heartbeat_text = first_line(mavros['text'].splitlines(), 'CON: Got HEARTBEAT')
    deadline, heartbeat = ros_stamp(deadline_text), ros_stamp(heartbeat_text)
    if heartbeat <= deadline:
        raise ValueError('Episode does not show heartbeat after deadline')
    rows = []

    def row(event, domain, value, locator, uncertainty, kind='OBSERVED', meaning=''):
        rows.append({'event_id': 'T%02d' % (len(rows) + 1), 'event': event,
                     'clock_domain': domain, 'timestamp_s': str(value) if value is not None else '',
                     'source_reference': locator, 'evidence_kind': kind,
                     'uncertainty_and_missing_observations': uncertainty, 'interpretation': meaning})

    row('OSMO workflow initialization', 'UNKNOWN', None,
        'EXTERNAL:OSMO_WORKFLOW_LIFECYCLE_RECORD', 'Workflow identity/time absent', 'MISSING')
    containers = [json.loads(s) for s in doc['commands'][0]['stdout'].splitlines()]
    for item in containers:
        row(item['name'] + ' container started', 'HOST_REALTIME_UTC', utc_stamp(item['started']),
            'RRM/evidence/M00-C/startup-source-records.json#/commands/0',
            'Docker State.StartedAt; container time is not process/readiness time')
    row('Guard deadline origin (member initialized)', 'PROCESS_STEADY', None,
        'robot/ros_ws/src/interface/mavros_interface/include/mavros_interface/px4_actuation_guard.hpp:42',
        'Exact semantic origin known; actual numeric steady timestamp uninstrumented; do not backdate error log by10s as an observation',
        'SOURCE_DEFINED_NOT_TIMESTAMPED', 'started_=Clock::now() with Clock=std::chrono::steady_clock')
    patterns = [
        ('Isaac Simulation App Starting', 'isaac-sim-livestream', 'Simulation App Starting'),
        ('Isaac application reports app ready', 'isaac-sim-livestream', '] app ready'),
        ('Isaac Simulation App Startup Complete', 'isaac-sim-livestream', 'Simulation App Startup Complete'),
        ('Robot interface launch process report', 'airstack-robot-desktop-1', '[robot_interface_node-4]:'),
        ('MAVROS process launch report', 'airstack-robot-desktop-1', '[mavros_node-3]: process started'),
        ('DDS router running', 'airstack-robot-desktop-1', 'DDS Router running.'),
        ('MAVROS FCU URL configured', 'airstack-robot-desktop-1', 'FCU URL:'),
        ('Isaac PX4 vehicle graph created', 'isaac-sim-livestream', 'PX4 node at'),
        ('PX4 startup log', 'isaac-sim-livestream', 'px4 starting.'),
        ('PX4 simulator TCP connected', 'isaac-sim-livestream', 'Simulator connected on TCP port'),
        ('Pegasus first received heartbeat log', 'isaac-sim-livestream', 'Received first hearbeat'),
        ('PX4 onboard MAVLink UDP initialized', 'isaac-sim-livestream', 'on udp port 14581 remote port 14541'),
        ('PX4 startup script returned', 'isaac-sim-livestream', 'Startup script returned successfully'),
        ('PX4 commander Ready for takeoff log', 'isaac-sim-livestream', 'Ready for takeoff!')]
    for name, container, needle in patterns:
        group = next(x for x in doc['filtered_logs'] if x['container'] == container)
        found = [(i, x) for i, x in enumerate(group['records']) if needle in x['text']]
        if found:
            i, item = found[0]
            row(name, 'HOST_DOCKER_RECEIPT_UTC', utc_stamp(item['text']),
                'RRM/evidence/M00-C/startup-source-records.json#/filtered_logs/%d/records/%d' %
                (doc['filtered_logs'].index(group), i),
                'Receipt/log buffering delay unknown; printed precision is not timing accuracy; no cross-domain conversion',
                meaning='First matching retained record; not proof of the first actual event')
        else:
            row(name, 'UNKNOWN', None, 'RRM/evidence/M00-C/startup-source-records.json',
                'Event not found in bounded selected logs', 'MISSING')
    row('Guard deadline expiration logged', 'ROS_LOG_EPOCH', deadline,
        'RRM/evidence/M00-C/startup-source-records.json#/original_ros_logs/logs/0/text line%d' % deadline_line,
        'Timestamp is log generation after steady condition; not exact expiration edge or ROS simulation /clock')
    row('MAVROS first recorded PX4 heartbeat/connection', 'ROS_LOG_EPOCH', heartbeat,
        'RRM/evidence/M00-C/startup-source-records.json#/original_ros_logs/logs/1/text line%d' % heartbeat_line,
        'First recorded MAVROS connection; not first PX4 heartbeat emission or Pegasus heartbeat')
    row('Startup readiness transitions before terminal failure', 'PROCESS_STEADY', None,
        'EXTERNAL:GUARD_STARTUP_CALLBACK_AND_TRANSACTION_TRACE',
        'No full state/ground callback/timer/service trace retained; status after failure does not reconstruct missing transitions', 'MISSING')
    for proc in doc['original_ros_logs']['processes']:
        row('Kernel start PID%d' % proc['pid'], 'KERNEL_BOOT_TICKS_100HZ',
            proc['start_ticks'], 'RRM/evidence/M00-C/startup-source-records.json#/original_ros_logs/processes',
            'Ticks since boot, not seconds/UTC;100Hz verified; each event tick quantized')
    isaac = next(x for x in doc['isaac_processes']['processes'] if x['pid'] == 210)
    row('Isaac Python kernel process start', 'KERNEL_BOOT_TICKS_100HZ', isaac['start_ticks'],
        'RRM/evidence/M00-C/startup-source-records.json#/isaac_processes/processes',
        'Kernel ticks in shared boot; process existence is not app/scene/vehicle readiness')
    px4 = next(x for x in doc['isaac_processes']['processes'] if x['exe'].endswith('/px4'))
    robot = next(x for x in doc['original_ros_logs']['processes'] if x['pid'] == 20295)
    ticks = doc['original_ros_logs']['clk_tck']
    if doc['original_ros_logs']['boot_id'] != doc['isaac_processes']['boot_id']:
        raise ValueError('Cannot compare process starts across different boot identities')
    if ticks != doc['isaac_processes']['clk_tck'] or ticks <= 0:
        raise ValueError('Invalid or mismatched tick frequency')
    process_delta = Decimal(px4['start_ticks'] - robot['start_ticks']) / Decimal(ticks)
    row('PX4 kernel process start', 'KERNEL_BOOT_TICKS_100HZ', px4['start_ticks'],
        'RRM/evidence/M00-C/startup-source-records.json#/isaac_processes/processes',
        'Kernel tick precision10ms; delta quantization bound less than20ms conservatively; UTC mapping unavailable')
    # Check prior retained excerpts against original log text, not prior summary arithmetic.
    prior_lines = [entry['text'] for group in prior['extracts'] for entry in group['matches']]
    prior_matches = all(any(line in item['text'] for item in logs) for line in prior_lines)
    a = {'clock_domain': 'ROS_LOG_EPOCH', 'timestamp_s': str(heartbeat)}
    b = {'clock_domain': 'ROS_LOG_EPOCH', 'timestamp_s': str(deadline)}
    return rows, {'heartbeat_after_deadline_s_exact_decimal': str(difference(a, b)),
                  'deadline_log_epoch_s': str(deadline), 'heartbeat_log_epoch_s': str(heartbeat),
                  'px4_process_after_interface_s': str(process_delta),
                  'prior_excerpt_lines_verified_in_original_logs': prior_matches,
                  'deadline_semantic_origin': 'Px4ActuationGuard::started_ steady-clock non-static member initialization',
                  'deadline_origin_numeric_observed': False,
                  'timeout_s_observed_in_M00B_GetParameters': '10.0',
                  'limitation': 'Producer log deltas assume no realtime discontinuity between events; no calibrated uncertainty/clock-step trace. Docker receipts and kernel ticks are not mixed; no inferred exact guard initialization UTC.'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='Compare generated outputs without writing')
    args = parser.parse_args()
    doc = json.loads((EVIDENCE / 'startup-source-records.json').read_text())
    prior = json.loads((ROOT / 'RRM/evidence/M00-B/startup-log-extract.json').read_text())
    rows, summary = analyze(doc, prior)
    live = json.loads((EVIDENCE / 'passive-readiness.json').read_text())
    ready = live['channels']['/robot_1/interface/actuation_ready']['samples']
    status = live['channels']['/robot_1/interface/actuation_status']['samples']
    summary['current_readiness_samples'] = len(ready)
    summary['current_false_count'] = sum(x['data'] is False for x in ready)
    summary['current_status_values'] = sorted({x['data'] for x in status})
    summary['current_observed_utc'] = live['captured_utc']
    rows.append({'event_id': 'T%02d' % (len(rows) + 1), 'event': 'Final passive readiness observation',
                 'clock_domain': 'COLLECTOR_UTC_LABEL_AND_SEPARATE_MONOTONIC_RECEIPTS',
                 'timestamp_s': '', 'source_reference': 'RRM/evidence/M00-C/passive-readiness.json',
                 'evidence_kind': 'OBSERVED', 'uncertainty_and_missing_observations':
                 'Late observation, not original startup transition or independent physical state',
                 'interpretation': '%d/%d false; %s' % (summary['current_false_count'], len(ready), summary['current_status_values'])})
    import io
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator='\n')
    writer.writeheader()
    writer.writerows(rows)
    outputs = [(REPORTS / 'M00C_STARTUP_TIMELINE.csv', stream.getvalue()),
               (EVIDENCE / 'timing-analysis.json', json.dumps(summary, indent=2) + '\n')]
    for path, text in outputs:
        if args.check:
            if path.read_text() != text:
                raise ValueError('Nonreproducible analysis output: ' + str(path))
        else:
            with path.open('w') as stream:
                stream.write(text)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
