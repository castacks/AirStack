"""Explicit FP32 covariance-head finetuning; independent of the legacy trainer."""
import argparse
import datetime
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import random
import platform

import numpy as np
import torch

from .covariance_contract import sequence_covariance_nll
from .retraining_manifest import inspect_manifest, load_pair, pinned, sha256


PREFIX = 'memory_decoder.cov_update.'


def configure(model, training=True):
    model.eval()
    # Only this flag selects recurrent full-resolution lists. Leave all other
    # decoder children in eval, so frozen dropout/norm buffers do not change.
    model.memory_decoder.training = True
    model.memory_decoder.cov_update.train(training)
    names = []
    for name, parameter in model.named_parameters():
        parameter.requires_grad_(name.startswith(PREFIX))
        if parameter.requires_grad:
            names.append(name)
    if not names:
        raise ValueError('model has no covariance-update parameters')
    return names


def build_model(root, manifest, device):
    from Module.Network.FlowFormerCov import build_flowformer
    from Module.Network.FlowFormerCov.checkpoint import load_cov_checkpoint
    from yacs.config import CfgNode
    architecture = json.loads(pinned(root, manifest['architecture']).read_text())
    model = build_flowformer({'latentcostformer': CfgNode(architecture)}, torch.float32, torch.float32)
    weights = torch.load(pinned(root, manifest['initialization']['file']), map_location='cpu', weights_only=True)
    migration = load_cov_checkpoint(model, weights)
    return model.to(device), migration


def source_identity(factory):
    import inspect
    base = Path(__file__).resolve().parents[2]
    paths = list((base / 'Module/Network/FlowFormer').rglob('*.py'))
    paths += list((base / 'Module/Network/FlowFormerCov').rglob('*.py'))
    paths += [Path(__file__), Path(__file__).with_name('retraining_manifest.py'),
              Path(__file__).with_name('covariance_contract.py')]
    paths.append(Path(inspect.getsourcefile(factory)))
    return {str(p.relative_to(base)) if p.is_relative_to(base) else 'factory/' + p.name: sha256(p)
            for p in sorted(set(paths))}


def rng_state():
    state = np.random.get_state()
    return {'python': random.getstate(), 'numpy': [state[0], state[1].tolist(), *state[2:]],
            'torch': torch.get_rng_state(),
            'cuda': torch.cuda.get_rng_state_all() if torch.cuda.is_available() else []}


def restore_rng(state):
    random.setstate(state['python'])
    numpy = state['numpy']
    np.random.set_state((numpy[0], np.array(numpy[1], dtype=np.uint32), *numpy[2:]))
    torch.set_rng_state(state['torch'].cpu())
    if state['cuda']:
        torch.cuda.set_rng_state_all([item.cpu() for item in state['cuda']])


def objective(model, pair, options):
    predictions, raw = model(pair['image1'], pair['image2'])
    return sequence_covariance_nll(predictions, raw, pair['flow'], pair['valid_mask'],
                                   gamma=options['gamma'], max_flow=options['max_flow'])


def assert_frozen(model, snapshot):
    current = model.state_dict()
    for name, original in snapshot.items():
        if not torch.equal(current[name], original):
            raise ValueError('frozen parameter/buffer changed: ' + name)


def atomic_save(payload, path):
    if path.exists():
        raise FileExistsError(path)
    temporary = path.with_suffix('.pending')
    try:
        with temporary.open('xb') as stream:
            torch.save(payload, stream); stream.flush(); os.fsync(stream.fileno())
        # Same-directory hard link publishes atomically and refuses overwrite.
        os.link(temporary, path)
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try: os.fsync(descriptor)
        finally: os.close(descriptor)
    finally:
        temporary.unlink(missing_ok=True)


def train(root, manifest_path, output, *, device='cuda', resume=None, stop_after=None,
          model_factory=build_model):
    """Batch1 cyclic input order, constant AdamW, exact contract-bound resume.

    model_factory is a software-test seam; CLI always uses strict FlowFormer.
    Caller-owned process: deterministic settings and RNG are set explicitly.
    """
    root, manifest_path, output = Path(root).resolve(), Path(manifest_path).resolve(), Path(output)
    if output.exists():
        raise FileExistsError(output)
    manifest = inspect_manifest(root, manifest_path)
    options = manifest['optimization']
    if device not in ('cpu', 'cuda') or (device == 'cuda' and not torch.cuda.is_available()):
        raise ValueError('available cpu/cuda device required')
    if stop_after is not None and (type(stop_after) is not int or not 0 < stop_after <= options['steps']):
        raise ValueError('stop_after must be an absolute positive step within budget')
    os.environ['CUBLAS_WORKSPACE_CONFIG'] = ':4096:8'
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.backends.cudnn.benchmark = False
    random.seed(options['seed']); np.random.seed(options['seed']); torch.manual_seed(options['seed'])
    if device == 'cuda': torch.cuda.manual_seed_all(options['seed'])
    dependencies = {}
    for package in ('timm', 'einops', 'yacs'):
        try: dependencies[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError: dependencies[package] = None
    identity = {'manifest_sha256': sha256(manifest_path), 'source_sha256': source_identity(model_factory),
                'torch': str(torch.__version__), 'numpy': np.__version__, 'device': device,
                'python': platform.python_version(), 'dependencies': dependencies,
                'cuda_version': torch.version.cuda, 'cudnn_version': torch.backends.cudnn.version(),
                'gpu': torch.cuda.get_device_name(0) if device == 'cuda' else None,
                'backend': model_factory.__module__ + '.' + model_factory.__qualname__,
                'ordering': 'manifest_order_cyclic_batch1', 'scheduler': 'constant_lr',
                'threads': 1, 'deterministic_algorithms': True, 'cublas_workspace': ':4096:8'}
    model, migration = model_factory(root, manifest, device)
    names = configure(model)
    parameters = [p for p in model.parameters() if p.requires_grad]
    snapshot = {k: v.detach().clone() for k, v in model.state_dict().items() if not k.startswith(PREFIX)}
    optimizer = torch.optim.AdamW(parameters, lr=options['lr'], betas=tuple(options['betas']),
                                 eps=options['eps'], weight_decay=options['weight_decay'])
    expected_group = {k: v for k, v in optimizer.param_groups[0].items() if k != 'params'}
    step = 0
    parent = None
    if resume is not None:
        parent = {'path': str(Path(resume).resolve()), 'sha256': sha256(resume)}
        state = torch.load(resume, map_location=device, weights_only=True)
        if (state['schema_version'] != 1 or state['identity'] != identity or state['manifest'] != manifest
                or state['trainable_names'] != names or type(state['step']) is not int
                or not 0 < state['step'] < options['steps']):
            raise ValueError('resume contract/source/data/runtime/trainable/step mismatch')
        model.load_state_dict(state['model'], strict=True)
        assert_frozen(model, snapshot)
        optimizer.load_state_dict(state['optimizer'])
        if len(optimizer.param_groups) != 1 or {k:v for k,v in optimizer.param_groups[0].items() if k!='params'} != expected_group:
            raise ValueError('resume optimizer configuration mismatch')
        step = state['step']
        for parameter in parameters:
            item = optimizer.state[parameter]
            if (set(item) != {'step', 'exp_avg', 'exp_avg_sq'} or item['step'].item() != step
                    or any(item[k].shape != parameter.shape or not torch.isfinite(item[k]).all()
                           for k in ('exp_avg', 'exp_avg_sq')) or (item['exp_avg_sq'] < 0).any()):
                raise ValueError('resume optimizer state mismatch/nonfinite')
        restore_rng(state['rng'])
    end = options['steps'] if stop_after is None else stop_after
    if end <= step:
        raise ValueError('resume must advance at least one step')
    if any(not torch.isfinite(value).all() for value in model.state_dict().values()):
        raise ValueError('nonfinite model state')
    # Revalidate all pins and source after constructing/loading before any output.
    if inspect_manifest(root, manifest_path) != manifest or sha256(manifest_path) != identity['manifest_sha256'] or source_identity(model_factory) != identity['source_sha256']:
        raise ValueError('inputs changed during preflight')
    output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'RUNNING', 'identity': identity, 'manifest': manifest, 'parent_checkpoint': parent,
              'initialization_migration': migration, 'trainable_names': names, 'start_step': step,
              'step': step, 'training_membership': 'unknown', 'training_heldout_claim': False,
              'created_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}
    train_rows = [s for s in manifest['samples'] if s['partition'] == 'train']
    try:
        with (output / 'steps.jsonl').open('x') as journal:
            while step < end:
                configure(model)
                sample = train_rows[step % len(train_rows)]
                pair = {k:v.to(device) for k,v in load_pair(root, sample).items()}
                optimizer.zero_grad(set_to_none=True)
                result = objective(model, pair, options)
                result.loss.backward()
                if any(p.grad is None or not torch.isfinite(p.grad).all() for p in parameters):
                    raise ValueError('missing/nonfinite trainable gradient before optimizer step')
                norm = torch.nn.utils.clip_grad_norm_(parameters, options['clip_norm'], error_if_nonfinite=True)
                optimizer.step()
                step += 1; report['step'] = step
                if any(not torch.isfinite(v).all() for v in model.state_dict().values()):
                    raise ValueError('nonfinite updated model state')
                assert_frozen(model, snapshot)
                journal.write(json.dumps({'step': step, 'sample_sha256': sample['file']['sha256'],
                    'recording_id': sample['recording_id'], 'loss': result.loss.item(),
                    'valid_components': result.valid_components, 'gradient_norm_before_clip': norm.item()})+'\n')
                journal.flush(); os.fsync(journal.fileno())
                if step % options['checkpoint_interval'] == 0 or step == end:
                    if inspect_manifest(root, manifest_path) != manifest or sha256(manifest_path) != identity['manifest_sha256'] or source_identity(model_factory) != identity['source_sha256']:
                        raise ValueError('inputs/source changed during run')
                    atomic_save({'schema_version': 1, 'identity': identity, 'manifest': manifest,
                        'model': model.state_dict(), 'optimizer': optimizer.state_dict(),
                        'rng': rng_state(), 'step': step, 'trainable_names': names},
                        output / f'step-{step:08d}.pt')
        configure(model, training=False)
        losses, components = 0., 0
        with torch.no_grad():
            for sample in manifest['samples']:
                if sample['partition'] != 'validation': continue
                pair = {k:v.to(device) for k,v in load_pair(root, sample).items()}
                value = objective(model, pair, options)
                losses += value.loss.item() * value.valid_components
                components += value.valid_components
        assert_frozen(model, snapshot)
        report.update(status='COMPLETED' if step == options['steps'] else 'STOPPED_AT_DECLARED_STEP',
                      validation_nll=losses/components, validation_components=components,
                      frozen_state_unchanged=True)
    except BaseException as error:
        report.update(status='FAILED', error_type=type(error).__name__, error=str(error))
        raise
    finally:
        report['ended_at_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
        with (output/'report.json').open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False); stream.write('\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('root', 'manifest', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--device', choices=('cpu', 'cuda'), default='cuda')
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--stop-after', type=int)
    args = parser.parse_args()
    report = train(args.root, args.manifest, args.output, device=args.device,
                   resume=args.resume, stop_after=args.stop_after)
    print(json.dumps({k:report[k] for k in ('status', 'start_step', 'step', 'validation_nll')}))


if __name__ == '__main__':
    main()
