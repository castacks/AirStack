"""Bounded, run-scoped visual evidence for experiment selection.

Images are existing worker previews or explicitly derived GT trajectory plots.
The model requests opaque evidence IDs; it never supplies filesystem paths.
Unavailable legacy images are reported as unavailable, not reconstructed.
"""
import hashlib
import json
import math
from pathlib import Path

MAX_REQUESTS = 3
MAX_IMAGES = 6
MODEL_FIELDS = (
    'sim_stamp', 'sequence', 'inference_ms', 'primitive', 'blocked',
    'recovery_mode', 'action', 'q_values', 'model_variant', 'policy_profile',
    'steering', 'collision_probability', 'input_fov_deg', 'speed',
    'policy_speed', 'target_speed', 'yaw_rate', 'estimated_forward_clearance_m',
    'depth_summary_m', 'depth_summary_normalized', 'depth_source',
    'consumed_input',
)
LIMITATIONS = [
    'Inference images are resized/annotated worker montages, not raw lossless network tensors.',
    'Worker timestamps refer to the model input. Capture time is later; GT telemetry is asynchronous.',
    'consumed_input describes the current worker source frame; bridge_released_input describes bridge availability, which may be newer.',
    'Source RGB noise RMSE is pre-JPEG whole-frame noise, not cropped or normalized network-tensor error. Neither field gives exact delivery latency.',
    'GT trajectory/geometry is privileged diagnostic evidence, never input to the evaluated reactive network.',
    'Image noise or a visible patch does not by itself establish the cause of a failure.',
    'Missing historical frames and raw sensor packets cannot be recovered from a montage.',
]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def _read(path):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None


def trusted_preview(path, run_id):
    """Read an image only when its bytes match that run's published metadata.

    Workers replace JPEG then JSON independently. Hash validation rejects a
    mixed pair even during a same-run frame transition, unlike run_id alone.
    """
    path = Path(path)
    try:
        metadata = json.loads(path.read_text())
        if metadata.get('run_id') != str(Path(run_id).resolve()):
            return None, None, 'different_or_missing_run_id'
        data = path.with_suffix('.jpg').read_bytes()
        expected = metadata.get('preview_sha256')
        if not expected:
            return metadata, None, 'missing_preview_hash'
        if digest(data) != expected:
            return metadata, None, 'preview_hash_mismatch'
        # Avoid loading malformed or unexpectedly large content into a model.
        if len(data) > 8 * 1024 * 1024 or not data.startswith(b'\xff\xd8'):
            return metadata, None, 'invalid_preview_jpeg'
        return metadata, data, None
    except (OSError, ValueError):
        return None, None, 'preview_unavailable'


def image_description(target):
    if target == 'mononav':
        return ('Worker RGB after bridge perturbation, ZoeDepth visualization, '
                'and TSDF/candidate paths. Depth is a prediction unless depth_source says ground-truth.')
    if target == 'kim':
        return ('Worker RGB after bridge perturbation, FCRN depth visualization, '
                'D3QN action/Q-values. RGB may be FOV-cropped by the adapter.')
    if target == 'pulp_dronet':
        return ('Worker RGB after bridge perturbation, actual grayscale crop, '
                'steering/collision probability. This model has no depth or optical-flow output.')
    return 'Worker-provided inference montage; consult model adapter for its panel meanings.'


def trajectory_image(records, geometry, destination):
    """Draw recorded GT motion and actual geometry; no predicted flight path."""
    from PIL import Image, ImageDraw, ImageFont
    positions = [r['position_m'][:2] for r in records
                 if len(r.get('position_m', [])) >= 2 and all(math.isfinite(v) for v in r['position_m'][:2])]
    if not positions:
        return None
    boxes = []
    for kind in ('walls', 'obstacles'):
        for box in (geometry or {}).get(kind, []):
            x, y = box['center'][:2]; sx, sy = box['size'][:2]; angle = box.get('yaw', 0.)
            c, s = math.cos(angle), math.sin(angle)
            corners = [(x + dx*c - dy*s, y + dx*s + dy*c)
                       for dx, dy in [(-sx/2, -sy/2), (sx/2, -sy/2), (sx/2, sy/2), (-sx/2, sy/2)]]
            boxes.append((kind, corners))
    points = positions + [p for _, box in boxes for p in box]
    xmin, xmax = min(p[0] for p in points)-1, max(p[0] for p in points)+1
    ymin, ymax = min(p[1] for p in points)-1, max(p[1] for p in points)+1
    scale = min(830/(xmax-xmin), 570/(ymax-ymin))
    ox, oy = 65+(830-scale*(xmax-xmin))/2, 85+(570-scale*(ymax-ymin))/2
    project = lambda p: (ox+(p[0]-xmin)*scale, oy+(ymax-p[1])*scale)
    canvas = Image.new('RGB', (960, 720), '#f8f9fa'); d = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 18)
    except OSError:
        font = ImageFont.load_default()
    d.text((20, 12), 'Recorded GT trajectory (XY, metres) - diagnostic only', fill='#16222b', font=font)
    d.text((20, 39), 'Blue: actual motion | Green: first sample | Red: last sample', fill='#34454f', font=font)
    for kind, corners in boxes:
        d.polygon([project(p) for p in corners], fill='#8b969e' if kind == 'walls' else '#a36d39')
    if len(positions) > 1:
        d.line([project(p) for p in positions], fill='#1269b2', width=4)
    for point, color in ((positions[0], '#17843c'), (positions[-1], '#d4252f')):
        x, y = project(point); d.ellipse((x-6, y-6, x+6, y+6), fill=color)
    origin = (70, 683); length = min(100, 2*scale)
    d.line((origin, (origin[0]+length, origin[1])), fill='#111111', width=3)
    d.text((origin[0]+length+6, 672), f'{length/scale:.1f} m; +X right, +Y up', fill='#16222b', font=font)
    destination = Path(destination); canvas.save(destination)
    return {'file': destination.name, 'sha256': digest(destination.read_bytes()),
            'kind': 'derived_gt_trajectory', 'samples': len(positions),
            'geometry_present': bool(geometry), 'coordinate_frame': 'ws2_world',
            'note': 'Sampled actual GT positions. Lines connect observations; they are not controller commands.'}


def request_schema():
    return {'type': 'array', 'maxItems': MAX_REQUESTS, 'items': {
        'type': 'object', 'additionalProperties': False,
        'required': ['evidence_id', 'view', 'sim_time_s'], 'properties': {
            'evidence_id': {'type': 'string', 'minLength': 1, 'maxLength': 60},
            'view': {'type': 'string', 'enum': ['inference', 'trajectory']},
            'sim_time_s': {'type': ['number', 'null']}}}}


def _runs(history, runtime_root):
    root = Path(runtime_root).resolve(); found = {}
    for index, item in enumerate(history):
        number = item.get('decision', {}).get('round', index+1)
        for pair_index, pair in enumerate(item.get('pairs', [])):
            for role in ('clean', 'perturbed'):
                result = pair.get(role) or {}
                folder = result.get('result_dir')
                if not folder:
                    continue
                path = Path(folder).resolve()
                try:
                    path.relative_to(root)
                except ValueError:
                    continue
                saved = _read(path/'result.json')
                if not isinstance(saved, dict) or saved.get('result_dir') != str(path):
                    continue
                evidence_id = f'r{number:03d}_{role}' if pair_index == 0 else f'r{number:03d}_p{pair_index+1:02d}_{role}'
                manifest = _read(path/'visual_evidence.json') or {}
                if manifest.get('run_id') != str(path):
                    manifest = {}
                found[evidence_id] = (path, saved, manifest)
    return found


def evidence_catalog(history, runtime_root):
    """Offer only recorded runs; no agent-controlled filesystem references."""
    result = []
    for evidence_id, (_, saved, manifest) in _runs(history, runtime_root).items():
        frames = manifest.get('frames', [])
        result.append({'evidence_id': evidence_id, 'outcome': saved.get('outcome'),
                       'target': manifest.get('target'),
                       'inference_times_sim_s': [f['capture_sim_s'] for f in frames],
                       'trajectory_available': bool(manifest.get('trajectory')),
                       'image_count': len(frames),
                       'availability': 'recorded' if frames or manifest.get('trajectory') else 'unavailable_legacy_or_missing_capture'})
    return result


def resolve_evidence_requests(history, requests, runtime_root, output_dir=None):
    """Resolve <=3 explicit requests into immutable images and their context.

    A null time means latest available. A numeric time selects nearest saved
    frame and reports its distance; it never implies a frame at the exact time.
    Missing evidence is returned as such so an agent can change its request.
    """
    if not isinstance(requests, list) or len(requests) > MAX_REQUESTS:
        raise ValueError(f'At most {MAX_REQUESTS} evidence requests are allowed')
    runs = _runs(history, runtime_root); images = []; observations = []; seen = set()
    for request in requests:
        if not isinstance(request, dict) or set(request) != {'evidence_id', 'view', 'sim_time_s'}:
            raise ValueError('Evidence request requires evidence_id, view, sim_time_s')
        evidence_id, view, stamp = (request[k] for k in ('evidence_id', 'view', 'sim_time_s'))
        if evidence_id not in runs or view not in ('inference', 'trajectory'):
            raise ValueError('Unknown recorded evidence ID or view')
        if stamp is not None and (type(stamp) not in (int, float) or not math.isfinite(stamp)):
            raise ValueError('Evidence time must be finite simulation seconds or null')
        folder, result, manifest = runs[evidence_id]
        entry = {'request': request, 'run_id': str(folder), 'outcome': result.get('outcome'),
                 'target': manifest.get('target'), 'status': 'unavailable'}
        artifact = None
        if view == 'trajectory':
            artifact = manifest.get('trajectory')
        else:
            candidates = manifest.get('frames', [])
            if candidates:
                artifact = max(candidates, key=lambda f: f['capture_sim_s']) if stamp is None else min(candidates, key=lambda f: abs(f['capture_sim_s']-stamp))
                entry['requested_time_delta_s'] = None if stamp is None else artifact['capture_sim_s']-stamp
                entry['description'] = image_description(manifest.get('target'))
        if artifact:
            path = (folder/artifact['file']).resolve()
            try:
                path.relative_to(folder)
            except ValueError:
                raise ValueError('Evidence artifact escapes its recorded episode') from None
            try:
                data = path.read_bytes()
                if len(data) > 8*1024*1024 or digest(data) != artifact.get('sha256'):
                    raise ValueError('Recorded evidence hash mismatch or oversized artifact')
            except OSError:
                entry['reason'] = 'Recorded image file is missing'
            else:
                entry.update(status='available', artifact=artifact)
                if str(path) not in seen and len(images) < MAX_IMAGES:
                    seen.add(str(path)); images.append(str(path))
        else:
            entry['reason'] = 'Requested image was not captured; no substitute was fabricated'
        observations.append(entry)
    bundle = {'image_paths': images, 'manifest': {'schema_version': 1, 'requests': observations,
                                               'limitations': LIMITATIONS}}
    if output_dir is not None:
        directory = Path(output_dir); directory.mkdir(parents=True, exist_ok=True)
        (directory/'evidence_manifest.json').write_text(json.dumps(bundle['manifest'], indent=2))
    return bundle
