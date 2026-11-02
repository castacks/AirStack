"""Opt-in final covariance-convolution policy for the unchanged training engine.

Process-local scoped bindings; run the CLI in its own process. All existing
trainer, loss, model and inference files remain independent of this wrapper.
"""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path

import torch

from . import retraining_runner as base
from .retraining_manifest import sha256


PREFIX = 'memory_decoder.cov_update.cov_head.conv4.'
POLICY = 'final-covariance-convolution-only-v1'


def configure(model, training=True):
    model.eval()
    model.memory_decoder.training = True
    layer = model.memory_decoder.cov_update.cov_head.conv4
    if not isinstance(layer, torch.nn.Conv2d) or layer.out_channels != 2 or layer.bias is None:
        raise ValueError('two-channel final covariance convolution with bias required')
    layer.train(training)
    names = []
    for name, parameter in model.named_parameters():
        parameter.requires_grad_(name.startswith(PREFIX))
        if parameter.requires_grad:
            names.append(name)
    if names != [PREFIX+'weight', PREFIX+'bias']:
        raise ValueError('exact final covariance weight/bias parameter names required')
    return names


@contextmanager
def final_layer_policy():
    if base.PREFIX != 'memory_decoder.cov_update.':
        raise RuntimeError('nested or conflicting training policy')
    previous = base.PREFIX, base.configure, base.source_identity
    def source_identity(factory):
        identity = previous[2](factory)
        identity['Train/MatchingNet/final_layer_retraining.py'] = sha256(Path(__file__))
        return identity
    try:
        base.PREFIX, base.configure, base.source_identity = PREFIX, configure, source_identity
        yield
    finally:
        base.PREFIX, base.configure, base.source_identity = previous


def train(root, manifest, output, *, device='cuda', resume=None, stop_after=None,
          model_factory=base.build_model):
    output = Path(output)
    existed = output.exists()
    try:
        with final_layer_policy():
            return base.train(root, manifest, output, device=device, resume=resume,
                              stop_after=stop_after, model_factory=model_factory)
    finally:
        if not existed and output.is_dir():
            contract = {'policy': POLICY, 'trainable_prefix': PREFIX,
                        'trainable_names': [PREFIX+'weight', PREFIX+'bias'],
                        'source_sha256': sha256(Path(__file__)),
                        'frozen_scope': 'every model parameter/buffer outside final convolution',
                        'production_adoption': False}
            with (output/'policy-contract.json').open('x') as stream:
                json.dump(contract, stream, indent=2); stream.write('\n')


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
    print(json.dumps({k:report[k] for k in ('status','start_step','step','validation_nll')}))


if __name__ == '__main__':
    main()
