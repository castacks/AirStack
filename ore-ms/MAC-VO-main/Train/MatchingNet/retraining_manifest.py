"""Pinned, recording-disjoint, prepared dense-flow data for opt-in head fitting."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch

from .covariance_contract import CONTRACT_ID


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def pinned(root, pin):
    if not isinstance(pin, dict) or set(pin) != {'path', 'sha256'}:
        raise ValueError('file pin requires path and sha256')
    path = (Path(root) / pin['path']).resolve()
    if not path.is_relative_to(Path(root).resolve()) or sha256(path) != pin['sha256']:
        raise ValueError('file path/hash mismatch: ' + str(pin['path']))
    return path


def load_pair(root, sample):
    """No transforms/augmentation; preserve prepared FP32 units and bool masks."""
    path = pinned(root, sample['file'])
    with np.load(path, allow_pickle=False) as data:
        if set(data.files) != {'image1', 'image2', 'flow', 'valid_mask'}:
            raise ValueError('prepared NPZ needs image1,image2,flow,valid_mask only')
        arrays = {name: data[name].copy() for name in data.files}
    first = arrays['image1']
    if first.ndim != 3 or first.shape[0] != 3:
        raise ValueError('image must be [3,H,W]')
    height, width = first.shape[1:]
    if min(height, width) < 64 or height % 8 or width % 8:
        raise ValueError('H/W must be >=64 and divisible by8')
    for name in ('image1', 'image2'):
        value = arrays[name]
        if (value.shape != (3, height, width) or value.dtype != np.float32
                or not np.isfinite(value).all() or not ((value >= 0) & (value <= 1)).all()):
            raise ValueError('prepared images must be matching finite FP32 RGB-unit arrays')
    if arrays['flow'].shape != (2, height, width) or arrays['flow'].dtype != np.float32:
        raise ValueError('flow must be FP32[2,H,W]')
    if arrays['valid_mask'].shape != (1, height, width) or arrays['valid_mask'].dtype != np.bool_:
        raise ValueError('valid_mask must be bool[1,H,W]')
    return {k: torch.from_numpy(v).unsqueeze(0) for k, v in arrays.items()}


def inspect_manifest(root, manifest_path):
    root = Path(root).resolve()
    path = Path(manifest_path).resolve()
    data = json.loads(path.read_text())
    required = {'schema_version', 'task', 'contract_id', 'ledger', 'architecture',
                'initialization', 'recordings', 'samples', 'optimization', 'modality', 'input_recipe'}
    if (set(data) != required or type(data['schema_version']) is not int or data['schema_version'] != 1
            or data['task'] != 'dense_temporal_flow_covariance'
            or data['contract_id'] != CONTRACT_ID or data['modality'] not in ('rgb', 'thermal')):
        raise ValueError('unsupported training manifest schema/task/modality')
    for key in ('ledger', 'architecture', 'input_recipe'):
        pinned(root, data[key])
    architecture = json.loads(pinned(root, data['architecture']).read_text())
    if architecture.get('pretrain') is not False:
        raise ValueError('architecture pretrain must be false; all initialization comes from pinned state')
    if not pinned(root, data['input_recipe']).read_text().strip():
        raise ValueError('input recipe evidence required')
    init = data['initialization']
    if (set(init) != {'kind', 'file', 'training_membership'}
            or init['kind'] != 'checkpoint_finetune' or init['training_membership'] != 'unknown'):
        raise ValueError('this runner supports explicit unknown-membership checkpoint finetuning only')
    pinned(root, init['file'])
    ledger = json.loads(pinned(root, data['ledger']).read_text())
    aliases, canonical = {}, {}
    for record in ledger['records']:
        canonical[record['id']] = record
        for alias in [record['id'], *record['aliases']]:
            if alias in aliases:
                raise ValueError('ambiguous recording ledger alias')
            aliases[alias] = record['id']
    records = {}
    for record in data['recordings']:
        identity = record['id']
        if identity not in canonical or identity in records:
            raise ValueError('unique canonical recording IDs required')
        entry = canonical[identity]
        if entry['exposure_status'] != 'development_exposed' or entry['unit_kind'] != 'temporal_recording':
            raise ValueError('only development-exposed temporal recordings may train/validate')
        if record['label_kind'] != 'provided_dense_temporal_flow':
            raise ValueError('dense temporal-flow label provenance required')
        if not pinned(root, record['label_evidence']).read_text().strip():
            raise ValueError('nonempty label source evidence required')
        records[identity] = record
    partitions = {'train': set(), 'validation': set()}
    image_hashes = {'train': set(), 'validation': set()}
    paths, hashes = set(), set()
    if not data['samples']:
        raise ValueError('training and validation samples required')
    for sample in data['samples']:
        identity = aliases.get(sample['recording_id'])
        if sample['recording_id'] != identity or identity not in records:
            raise ValueError('sample needs declared canonical recording ID')
        if sample['partition'] not in partitions:
            raise ValueError('only train/validation partitions accepted')
        partitions[sample['partition']].add(identity)
        file = pinned(root, sample['file'])
        digest = sample['file']['sha256']
        if file in paths or digest in hashes:
            raise ValueError('duplicate sample path/content across dataset')
        paths.add(file); hashes.add(digest)
        pair = load_pair(root, sample)
        for key in ('image1', 'image2'):
            image_hashes[sample['partition']].add(hashlib.sha256(pair[key].numpy().tobytes()).hexdigest())
    if not all(partitions.values()) or partitions['train'] & partitions['validation']:
        raise ValueError('nonempty disjoint whole-recording train/validation required')
    if image_hashes['train'] & image_hashes['validation']:
        raise ValueError('identical prepared image content crosses train/validation')
    if set(records) != set.union(*partitions.values()):
        raise ValueError('unused recording declarations rejected')
    options = data['optimization']
    if set(options) != {'seed', 'steps', 'checkpoint_interval', 'lr', 'betas', 'eps',
                        'weight_decay', 'clip_norm', 'gamma', 'max_flow'}:
        raise ValueError('all optimization options must be explicit')
    for name in ('seed', 'steps', 'checkpoint_interval'):
        if type(options[name]) is not int or options[name] < (0 if name == 'seed' else 1):
            raise ValueError('invalid integer option: ' + name)
    if options['seed'] >= 2**32:
        raise ValueError('seed outside numpy seed range')
    for name in ('lr', 'eps', 'weight_decay', 'clip_norm', 'gamma', 'max_flow'):
        value = options[name]
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0 or (name != 'weight_decay' and value == 0):
            raise ValueError('invalid numeric option: ' + name)
    if options['gamma'] > 1 or not isinstance(options['betas'], list) or len(options['betas']) != 2:
        raise ValueError('invalid gamma/betas')
    if any(type(b) not in (int, float) or not math.isfinite(b) or not 0 <= b < 1 for b in options['betas']):
        raise ValueError('invalid beta')
    for sample in data['samples']:
        pair = load_pair(root, sample)
        valid = (pair['valid_mask'][:, 0] & torch.isfinite(pair['flow']).all(dim=1)
                 & (torch.linalg.vector_norm(pair['flow'], dim=1) < options['max_flow']))
        if not valid.any():
            raise ValueError('sample has no valid supervised target pixels')
    return data
