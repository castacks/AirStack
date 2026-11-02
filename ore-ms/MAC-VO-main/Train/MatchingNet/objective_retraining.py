"""Opt-in full-covariance-block runner for two final-stage objectives.

Process-local bindings: use the CLI in its own process, not concurrently with
other training policies. Existing engine/defaults remain independent.
"""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path

from . import retraining_runner as base
from .covariance_contract import CovarianceObjective, sequence_covariance_nll
from .retraining_manifest import sha256
from .trace_risk_contract import CONTRACT_ID as TRACE_ID, final_trace_risk_score


OBJECTIVES = {
    'final_diagonal_nll': 'final-diagonal-log-std-fixed-mean-nll-v1',
    'final_trace_risk_score': TRACE_ID,
}
_ORIGINAL = base.objective, base.source_identity, base.configure, base.PREFIX


def _validate_mode(mode):
    if not isinstance(mode, str) or mode not in OBJECTIVES:
        raise ValueError('explicit final_diagonal_nll or final_trace_risk_score required')


@contextmanager
def objective_policy(mode):
    _validate_mode(mode)
    if (base.objective, base.source_identity, base.configure, base.PREFIX) != _ORIGINAL:
        raise RuntimeError('nested or conflicting training policy')

    def objective(model, pair, options):
        means, raw = model(pair['image1'], pair['image2'])
        if mode == 'final_diagonal_nll':
            return sequence_covariance_nll(means, raw, pair['flow'], pair['valid_mask'],
                                           gamma=options['gamma'], max_flow=options['max_flow'],
                                           final_only=True)
        result = final_trace_risk_score(means, raw, pair['flow'], pair['valid_mask'],
                                       max_flow=options['max_flow'])
        # Engine weights validation by 2N scalar flow components. The trace score
        # itself is averaged over N pixels; multiplying every support by2 does
        # not change its aggregate. This is support size, not observation count.
        return CovarianceObjective(result.loss, result.valid_pixels,
                                   2 * result.valid_pixels, 1, result.final_mean_trace / 2)

    def source_identity(factory):
        identity = _ORIGINAL[1](factory)
        for filename in ('objective_retraining.py', 'trace_risk_contract.py'):
            identity['Train/MatchingNet/' + filename] = sha256(Path(__file__).with_name(filename))
        # Semantic discriminator in the unchanged engine's exact identity map.
        # Both modes bind both files, but can never consume each other's state.
        identity['objective-policy/' + OBJECTIVES[mode]] = sha256(Path(__file__))
        return identity

    try:
        base.objective, base.source_identity = objective, source_identity
        yield
    finally:
        base.objective, base.source_identity = _ORIGINAL[:2]


def train(root, manifest, output, *, objective_mode, device='cuda', resume=None,
          stop_after=None, model_factory=base.build_model):
    _validate_mode(objective_mode)
    output = Path(output)
    existed = output.exists()
    try:
        with objective_policy(objective_mode):
            return base.train(root, manifest, output, device=device, resume=resume,
                              stop_after=stop_after, model_factory=model_factory)
    finally:
        if not existed and output.is_dir():
            contract = {
                'objective_mode': objective_mode, 'objective_id': OBJECTIVES[objective_mode],
                'trainable_prefix': _ORIGINAL[3], 'evaluated_stages': 'final_only',
                'manifest_contract_role': 'unchanged strict diagonal data/numeric admission',
                'identity_discriminator': 'objective-policy/' + OBJECTIVES[objective_mode],
                'source_sha256': sha256(Path(__file__)),
                'legacy_validation_nll_field_semantics': 'selected objective score; not comparable across objectives',
                'valid_components_semantics': '2N scalar flow support; trace score averages N pixels',
                'production_adoption': False,
            }
            with (output / 'objective-contract.json').open('x') as stream:
                json.dump(contract, stream, indent=2); stream.write('\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('root', 'manifest', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--objective-mode', choices=tuple(OBJECTIVES), required=True)
    parser.add_argument('--device', choices=('cpu', 'cuda'), default='cuda')
    parser.add_argument('--resume', type=Path)
    parser.add_argument('--stop-after', type=int)
    args = parser.parse_args()
    report = train(args.root, args.manifest, args.output, objective_mode=args.objective_mode,
                   device=args.device, resume=args.resume, stop_after=args.stop_after)
    print(json.dumps({**{k: report[k] for k in ('status', 'start_step', 'step')},
                      'objective_mode': args.objective_mode,
                      'validation_objective': report['validation_nll']}))


if __name__ == '__main__':
    main()
