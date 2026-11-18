"""Bounded camera fault programs, shared by the bench and ROS bridge.

Only rate-eligible live RGB arrivals are affected. No recorded frames, timestamp
rewriting, model edits or control commands are accepted by this interface.
"""
import copy
import hashlib
import json
import math
import random

VERSION = 'camera_program_v1'
FACTORS = ('rgb_noise', 'delay', 'frame_drop', 'frame_freeze')
FIELDS = ('factor', 'trigger', 'start_s', 'duration_s', 'period_s', 'turn_rate_rad_s')


def validate(raw):
    if not isinstance(raw, list) or len(raw) > 8:
        raise ValueError('camera_program must contain at most eight events')
    result = copy.deepcopy(raw)
    for event in result:
        if not isinstance(event, dict) or set(event) != set(FIELDS):
            raise ValueError('Camera event requires exactly ' + ', '.join(FIELDS))
        if event['factor'] not in FACTORS or event['trigger'] not in ('mission_time', 'first_turn'):
            raise ValueError('Unsupported camera factor/trigger; frame replay is not supported')
        for key, lo, hi in [('start_s', 0, 500), ('duration_s', .1, 500),
                            ('period_s', 0, 500), ('turn_rate_rad_s', .02, 1.5)]:
            value = event[key]
            if type(value) not in (int, float) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f'Camera event {key} outside [{lo}, {hi}]')
            event[key] = float(value)
        if event['factor'] in ('frame_drop', 'frame_freeze') and event['duration_s'] > 15:
            raise ValueError('Drop/freeze windows are limited to 15 simulation seconds')
        if event['period_s'] and event['period_s'] <= event['duration_s']:
            raise ValueError('Repeated windows need period_s > duration_s')
        if event['trigger'] == 'first_turn' and event['period_s']:
            raise ValueError('First-turn events are one-shot; add a separate event for another hypothesis')
    return result


def schema():
    return dict(type='array', maxItems=8, items=dict(type='object', additionalProperties=False,
        required=list(FIELDS), properties={
            'factor': dict(type='string', enum=list(FACTORS)),
            'trigger': dict(type='string', enum=['mission_time', 'first_turn']),
            'start_s': dict(type='number', minimum=0, maximum=500),
            'duration_s': dict(type='number', minimum=.1, maximum=500),
            'period_s': dict(type='number', minimum=0, maximum=500),
            'turn_rate_rad_s': dict(type='number', minimum=.02, maximum=1.5)}))


def identity(program):
    return hashlib.sha256(json.dumps(validate(program), sort_keys=True, separators=(',', ':')).encode()).hexdigest()


class CameraProgram:
    def __init__(self, program):
        self.program = validate(program)
        self.program_id = identity(self.program)
        self.reset()

    def reset(self):
        self.turns = {}
        self.last_elapsed = None
        self.candidates = 0
        self.dropped = 0
        self.frozen = 0
        self.active_counts = {f: 0 for f in FACTORS}
        self.first_active = {}
        self.last_active = {}

    def state(self, elapsed, yaw_rate=None):
        if elapsed is not None and self.last_elapsed is not None and elapsed < self.last_elapsed - 1e-6:
            self.reset()
        self.last_elapsed = elapsed
        active = []
        if elapsed is not None and elapsed >= 0:
            for index, event in enumerate(self.program):
                anchor = 0.
                if event['trigger'] == 'first_turn':
                    # Read-only observed yaw rate; not future GT route knowledge.
                    if index not in self.turns and elapsed >= event['start_s'] and yaw_rate is not None and abs(yaw_rate) >= event['turn_rate_rad_s']:
                        self.turns[index] = elapsed
                    anchor = self.turns.get(index)
                    if anchor is None:
                        continue
                    phase = elapsed - anchor
                else:
                    phase = elapsed - event['start_s']
                if phase < 0:
                    continue
                if event['period_s']:
                    phase %= event['period_s']
                if phase < event['duration_s']:
                    active.append(index)
        return {'schema': VERSION, 'program_id': self.program_id, 'elapsed_s': elapsed,
                'active_events': active, 'active_factors': sorted({self.program[i]['factor'] for i in active}),
                'triggered_at_s': {str(i): t for i, t in self.turns.items()}}

    def arrival(self, elapsed, yaw_rate, seed, drop_probability):
        state = self.state(elapsed, yaw_rate)
        sequence = self.candidates
        self.candidates += 1
        for factor in state['active_factors']:
            self.active_counts[factor] += 1
            self.first_active.setdefault(factor, elapsed)
            self.last_active[factor] = elapsed
        freeze = 'frame_freeze' in state['active_factors']
        drop = ('frame_drop' in state['active_factors'] and
                random.Random(f'{seed}:{sequence}:camera-drop').random() < drop_probability)
        self.frozen += int(freeze)
        self.dropped += int(drop and not freeze)
        state.update(suppress=freeze or drop, suppression='freeze' if freeze else 'drop' if drop else None,
                     arrival_index=sequence)
        return state

    def telemetry(self):
        return dict(schema=VERSION, program_id=self.program_id, eligible_arrivals=self.candidates,
                    dropped_arrivals=self.dropped, frozen_arrivals=self.frozen,
                    active_arrivals=dict(self.active_counts), first_active_s=dict(self.first_active),
                    last_active_s=dict(self.last_active), triggered_at_s={str(i): t for i, t in self.turns.items()})
