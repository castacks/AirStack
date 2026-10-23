"""Compact measured input/output/motion evidence, never an automatic cause label."""
import json
import math
from collections import deque
from pathlib import Path
from visual_evidence import (MODEL_FIELDS, LIMITATIONS, trusted_preview,
                             digest, image_description, trajectory_image)


def numeric_stats(values):
    values = [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return {'count': len(values), 'min': min(values) if values else None,
            'mean': sum(values)/len(values) if values else None, 'max': max(values) if values else None}


def bridge_released_input(record):
    """Separate bridge availability from the worker's exact consumed input."""
    return {'sequence': record.get('camera_sequence'), 'sim_stamp': record.get('latest_input_stamp'),
            'sensor_disturbance': record.get('latest_input_disturbance'),
            'rgb_noise_rmse': record.get('latest_input_rgb_noise_rmse')}


class Recorder:
    def __init__(self, output, runtime, planner):
        self.output, self.preview = Path(output), Path(runtime)/'inference'/f'{planner}.json'
        self.planner = planner
        self.records = []
        self.frames = deque(maxlen=6)
        self.timeline = []
        self.timeline_interval = 8.
        self.last_timeline_time = -math.inf
        self.last_sequence = None
        self.latest_frame = None
        self.capture_rejections = {}
        self.last_image_time = -math.inf
        self.first_frame = None

    def capture(self, scene, bridge):
        worker, image, rejected = trusted_preview(self.preview, self.output)
        if rejected:
            self.capture_rejections[rejected] = self.capture_rejections.get(rejected, 0)+1
        now = scene['sim_time']
        record = {'sim_time_s': now, 'position_m': scene['position'],
                  'clearance_m': scene['oracle'].get('clearance_m'),
                  'camera_sequence': bridge.get('sequence'), 'camera_stamp': bridge.get('image_stamp'),
                  'camera_age_sim_s': None if bridge.get('image_stamp') is None else now-bridge['image_stamp'],
                  'sensor_disturbance': bridge.get('sensor_disturbance'),
                  'latest_input_disturbance': bridge.get('latest_input_disturbance'),
                  'latest_input_rgb_noise_rmse': bridge.get('latest_input_rgb_noise_rmse'),
                  'latest_input_stamp': bridge.get('latest_input_stamp'),
                  'command': bridge.get('last_command'), 'worker': worker,
                  'patch_scene_enabled': scene.get('condition', {}).get('patch_enabled', False)}
        if not self.records or now != self.records[-1]['sim_time_s']:
            self.records.append(record)
        # A bounded timeline plus six recent frames permits an agent to inspect
        # an earlier corner, not only termination. Hash/sequence checks exclude
        # prior-run frames and races between the JPEG and metadata publishers.
        if image is not None:
            frame = (now, worker, image, {k: record.get(k) for k in (
                'position_m', 'clearance_m', 'camera_age_sim_s',
                'sensor_disturbance', 'command', 'patch_scene_enabled')} |
                {'bridge_released_input': bridge_released_input(record)})
            self.latest_frame = frame
        if image is not None and now-self.last_image_time >= 1 and worker.get('sequence') != self.last_sequence:
            self.frames.append(frame)
            if now-self.last_timeline_time >= self.timeline_interval:
                self.timeline.append(frame); self.last_timeline_time = now
                if len(self.timeline) > 24:
                    self.timeline = self.timeline[::2]
                    self.timeline_interval *= 2
            if self.first_frame is None:
                (self.output/'first_inference.jpg').write_bytes(image)
                (self.output/'first_inference.json').write_text(json.dumps(worker, indent=2))
                self.first_frame = self._image_entry('first_inference.jpg', frame)
                self.first_frame['metadata'] = 'first_inference.json'
            self.last_image_time = now
            self.last_sequence = worker.get('sequence')

    @staticmethod
    def _image_entry(name, frame):
        stamp, worker, data = frame[:3]
        return {'file': name, 'sha256': digest(data), 'capture_sim_s': stamp,
                'worker_sequence': worker.get('sequence'), 'sensor_sim_s': worker.get('sim_stamp'),
                'kind': 'worker_inference_montage', 'verified_preview_hash': True,
                'model_output': {k: worker.get(k) for k in MODEL_FIELDS},
                'capture_observation': frame[3],
                'alignment': 'Exact worker JPEG/metadata hash; capture and GT samples are asynchronous.'}

    def finish(self, mission_start, outcome):
        summary = summarize(self.records, mission_start, outcome)
        if self.latest_frame and (not self.frames or self.frames[-1][1].get('sequence') != self.latest_frame[1].get('sequence')):
            self.frames.append(self.latest_frame)
        images = []
        for index, frame in enumerate(self.frames):
            name = f'diagnostic_frame_{index:02d}.jpg'
            (self.output/name).write_bytes(frame[2])
            images.append(self._image_entry(name, frame))
        evidence = list(images)
        sequences = {f['worker_sequence'] for f in evidence}
        for index, frame in enumerate(self.timeline):
            if frame[1].get('sequence') in sequences:
                continue
            name = f'evidence_frame_{index:02d}.jpg'; (self.output/name).write_bytes(frame[2])
            evidence.append(self._image_entry(name, frame)); sequences.add(frame[1].get('sequence'))
        if self.first_frame and self.first_frame['worker_sequence'] not in sequences:
            evidence.append(self.first_frame)
        evidence.sort(key=lambda f: f['capture_sim_s'])
        try:
            geometry = json.loads((self.output/'corridor_geometry.json').read_text())
        except (OSError, ValueError):
            geometry = None
        trajectory = trajectory_image(self.records, geometry, self.output/'diagnostic_trajectory.png')
        visual = {'schema_version': 2, 'run_id': str(self.output.resolve()), 'target': self.planner,
                  'outcome': outcome, 'mission_start_sim_s': mission_start,
                  'frames': evidence, 'trajectory': trajectory,
                  'description': image_description(self.planner), 'limitations': LIMITATIONS,
                  'capture_rejections': self.capture_rejections,
                  'retention': 'At most 24 time-distributed frames, last 6 distinct frames, and first frame; no full video.'}
        (self.output/'visual_evidence.json').write_text(json.dumps(visual, indent=2))
        summary['preview_frames'] = images
        summary['first_inference'] = self.first_frame
        summary['visual_evidence'] = {'manifest': 'visual_evidence.json', 'image_count': len(evidence),
                                      'trajectory_available': trajectory is not None,
                                      'capture_rejections': self.capture_rejections}
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
            snapshot.append({k:r.get(k) for k in ('sim_time_s','position_m','clearance_m','camera_age_sim_s','patch_scene_enabled',
                             'sensor_disturbance','latest_input_disturbance','latest_input_rgb_noise_rmse','latest_input_stamp')} |
                            {'command': r.get('command'), 'model_output': {k:w.get(k) for k in MODEL_FIELDS},
                             'bridge_released_input': bridge_released_input(r)})
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
                            'latest_input_* trace fields and bridge_released_input are bridge-available frames; worker.consumed_input is the actual current source update.',
                            'Source-frame noise RMSE is pre-JPEG, not network-tensor error. Observed input age includes processing and sampling, not exact delivery latency.',
                            'Patch enabled is scene state, not proof of pixel visibility or depth attack efficacy.',
                            'Depth summaries are predictions, not matched ground-truth depth error.']}
