"""Compact measured input/output/motion evidence, never an automatic cause label."""
import json
import math
from collections import deque
from pathlib import Path


def numeric_stats(values):
    values = [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return {'count': len(values), 'min': min(values) if values else None,
            'mean': sum(values)/len(values) if values else None, 'max': max(values) if values else None}


class Recorder:
    def __init__(self, output, runtime, planner):
        self.output, self.preview = Path(output), Path(runtime)/'inference'/f'{planner}.json'
        self.records = []
        self.frames = deque(maxlen=6)
        self.last_image_time = -math.inf
        self.first_frame = None

    def capture(self, scene, bridge):
        worker = None
        try:
            candidate = json.loads(self.preview.read_text())
            if candidate.get('run_id') == str(self.output):
                worker = candidate
        except (OSError, ValueError):
            pass
        now = scene['sim_time']
        record = {'sim_time_s': now, 'position_m': scene['position'],
                  'clearance_m': scene['oracle'].get('clearance_m'),
                  'camera_sequence': bridge.get('sequence'), 'camera_stamp': bridge.get('image_stamp'),
                  'camera_age_sim_s': None if bridge.get('image_stamp') is None else now-bridge['image_stamp'],
                  'sensor_disturbance': bridge.get('sensor_disturbance'),
                  'command': bridge.get('last_command'), 'worker': worker,
                  'patch_scene_enabled': scene.get('condition', {}).get('patch_enabled', False)}
        if not self.records or now != self.records[-1]['sim_time_s']:
            self.records.append(record)
        # Only retain six most recent inference images; no accumulating video or bags.
        if worker and now-self.last_image_time >= 1:
            try:
                data=self.preview.with_suffix('.jpg').read_bytes()
                self.frames.append((now, worker.get('sequence'), data))
                if self.first_frame is None:
                    (self.output/'first_inference.jpg').write_bytes(data)
                    (self.output/'first_inference.json').write_text(json.dumps(worker,indent=2))
                    self.first_frame={'file':'first_inference.jpg','metadata':'first_inference.json',
                                      'capture_sim_s':now,'worker_sequence':worker.get('sequence')}
                self.last_image_time = now
            except OSError:
                pass

    def finish(self, mission_start, outcome):
        summary = summarize(self.records, mission_start, outcome)
        images = []
        for index, (stamp, sequence, data) in enumerate(self.frames):
            name = f'diagnostic_frame_{index:02d}.jpg'
            (self.output/name).write_bytes(data)
            images.append({'file': name, 'capture_sim_s': stamp, 'worker_sequence': sequence,
                           'alignment': 'latest preview, approximate alignment; not a synchronized sensor packet'})
        summary['preview_frames'] = images
        summary['first_inference'] = self.first_frame
        (self.output/'diagnostic_trace.json').write_text(json.dumps(self.records))
        (self.output/'diagnostics.json').write_text(json.dumps(summary, indent=2))
        return summary


def summarize(records, mission_start, outcome):
    end = records[-1]['sim_time_s'] if records else None
    tail = [r for r in records if r['sim_time_s'] >= max(mission_start or 0, end-5)] if end is not None else []
    unique = {}
    for r in tail:
        w = r.get('worker')
        if w:
            unique[w.get('sequence')] = w
    workers = list(unique.values())
    length = sum(math.dist(a['position_m'], b['position_m']) for a,b in zip(tail,tail[1:]))
    duration = tail[-1]['sim_time_s']-tail[0]['sim_time_s'] if tail else 0.
    snapshot = []
    for r in tail:
        if not snapshot or r['sim_time_s']-snapshot[-1]['sim_time_s'] >= .8 or r is tail[-1]:
            w = r.get('worker') or {}
            snapshot.append({k:r.get(k) for k in ('sim_time_s','position_m','clearance_m','camera_age_sim_s','patch_scene_enabled')} |
                            {'command': r.get('command'), 'model_output': {k:w.get(k) for k in
                             ('sim_stamp','sequence','inference_ms','primitive','blocked','recovery_mode','action','q_values',
                              'model_variant','policy_profile','steering','collision_probability','input_fov_deg','speed','policy_speed','target_speed','yaw_rate','estimated_forward_clearance_m','depth_summary_m','depth_summary_normalized')}})
    return {'schema_version': 1, 'outcome': outcome, 'window': 'last 5 simulation seconds before termination',
            'window_start_sim_s': tail[0]['sim_time_s'] if tail else None, 'window_end_sim_s': end,
            'mission_elapsed_sim_s': None if mission_start is None or end is None else end-mission_start,
            'sample_count': len(tail), 'unique_worker_frames': len(workers),
            'camera_age_sim_s': numeric_stats([r.get('camera_age_sim_s') for r in tail]),
            'worker_input_age_sim_s': numeric_stats([r['sim_time_s']-r['worker']['sim_stamp'] for r in tail
                                                    if r.get('worker') and isinstance(r['worker'].get('sim_stamp'), (int,float))]),
            'inference_ms': numeric_stats([w.get('inference_ms') for w in workers]),
            'clearance_m': numeric_stats([r.get('clearance_m') for r in tail]),
            'path_length_m': length, 'mean_motion_speed_m_s': length/duration if duration>0 else None,
            'snapshots': snapshot,
            'limitations': ['Model outputs, command acknowledgements and GT motion are observations, not causal attribution.',
                            'Worker telemetry is sampled asynchronously; missing or stale fields are not a zero value.',
                            'Patch enabled is scene state, not proof of pixel visibility or depth attack efficacy.',
                            'Depth summaries are predictions, not matched ground-truth depth error.']}
