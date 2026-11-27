#!/usr/bin/env python3
"""Offline validation of proposed diagnostic JSONL records; never attaches to a runtime."""
import argparse
import json
from pathlib import Path

VERSION = 'M00D-INSTRUMENTATION-PROPOSAL-v1'
UNITS = {'STEADY': {'ns'}, 'KERNEL_BOOT': {'ticks'}, 'REALTIME': {'ns'}, 'SIMULATION': {'ns'}}
REQUIRED = {'schema', 'episode_id', 'producer_id', 'process_instance_id', 'boot_id',
            'instrumentation_id', 'event', 'sequence', 'clock_domain', 'unit', 'native_time'}


def validate(records):
    previous = {}
    count = 0
    for record in records:
        count += 1
        if REQUIRED - record.keys():
            raise ValueError('Missing required fields')
        if record['schema'] != VERSION:
            raise ValueError('Wrong proposed schema version')
        for key in REQUIRED - {'sequence', 'native_time'}:
            if not isinstance(record[key], str) or not record[key]:
                raise ValueError('Missing string identity')
        if type(record['sequence']) is not int or record['sequence'] < 0:
            raise ValueError('Invalid sequence')
        if type(record['native_time']) is not int or record['native_time'] < 0:
            raise ValueError('Timestamp must be nonnegative integer native units')
        clock, unit = record['clock_domain'], record['unit']
        if clock not in UNITS or unit not in UNITS[clock]:
            raise ValueError('Clock/unit mismatch')
        if clock == 'KERNEL_BOOT' and (type(record.get('ticks_per_second')) is not int or
                                      record['ticks_per_second'] <= 0):
            raise ValueError('Missing positive kernel tick rate')
        key = (record['episode_id'], record['producer_id'], record['process_instance_id'],
               record['boot_id'], clock)
        if key in previous:
            seq, time = previous[key]
            if record['sequence'] <= seq:
                raise ValueError('Duplicate or reversed producer sequence')
            if clock != 'REALTIME' and record['native_time'] < time:
                raise ValueError('Clock reset without new episode/process identity')
            if clock == 'REALTIME' and record['native_time'] < time:
                raise ValueError('Realtime discontinuity must be declared in separate clock-map evidence')
        previous[key] = (record['sequence'], record['native_time'])
    if not count:
        raise ValueError('Empty trace cannot establish instrumentation coverage')
    return {'schema_valid_records': count, 'producer_clock_streams': len(previous),
            'coverage_qualified': False, 'requirement_verdict': 'NOT_ASSESSED',
            'limitation': 'Format/order checks only; no complete startup, clock mapping, safe state, source authenticity or requirement qualification inferred'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('jsonl', type=Path)
    args = parser.parse_args()
    with args.jsonl.open() as stream:
        records = [json.loads(line) for line in stream if line.strip()]
    print(json.dumps(validate(records), indent=2))


if __name__ == '__main__':
    main()
