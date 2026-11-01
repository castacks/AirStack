"""Independent pure-tensor numerical gate. No model, data, optimizer or fitting."""
import argparse
import datetime
import hashlib
import json
import math
from pathlib import Path
import platform
import random

import torch

from Train.MatchingNet import covariance_contract as contract


# Fixed before execution; CPU64 is the independent mathematical check, CUDA32
# checks production arithmetic. Neither tolerance is a calibration criterion.
TOLERANCES = {'cpu': {'rtol': 2e-10, 'atol': 2e-11},
              'cuda': {'rtol': 2e-5, 'atol': 2e-6}}


def validate(device):
    dtype = torch.float64 if device == 'cpu' else torch.float32
    tolerance = TOLERANCES[device]
    checks = 0
    max_absolute_error = 0.
    max_scaled_error = 0.

    def close(actual, expected):
        nonlocal checks, max_absolute_error, max_scaled_error
        actual = float(actual)
        assert math.isfinite(actual) and math.isfinite(expected)
        error = abs(actual - expected)
        limit = tolerance['atol'] + tolerance['rtol'] * abs(expected)
        assert error <= limit, (actual, expected, error, limit)
        max_absolute_error = max(max_absolute_error, error)
        max_scaled_error = max(max_scaled_error, error / limit)
        checks += 1

    generator = random.Random(0)
    cases = []
    # Independently computed scalar mean likelihood, output gradient and central
    # differences across heterogeneous recurrent stages and nonuniform masks.
    for case in range(64):
        stage_count = 1 + case % 3
        final_only = case % 4 == 0
        gamma = .85
        means = [[generator.uniform(-3, 3) for _ in range(8)] for _ in range(stage_count)]
        raws = [[generator.uniform(-1.5, 1.5) for _ in range(8)] for _ in range(stage_count)]
        labels = [generator.uniform(-1, 1) for _ in range(8)]
        valid = [True, case % 2 == 0, True, False]
        mask = torch.tensor(valid, dtype=torch.bool, device=device).reshape(1, 1, 4)
        tensor = lambda values: torch.tensor(values, dtype=dtype, device=device).reshape(1, 2, 1, 4)
        predictions = [tensor(p).requires_grad_() for p in means]
        target = tensor(labels).requires_grad_()
        raw_tensors = [tensor(p).requires_grad_() for p in raws]
        result = contract.sequence_covariance_nll(predictions, raw_tensors, target, mask,
            gamma=gamma, max_flow=100, final_only=final_only)
        result.loss.backward()
        selected = [i for i in range(8) if valid[i % 4]]
        evaluated = [stage_count-1] if final_only else list(range(stage_count))
        expected = 0.
        for stage in evaluated:
            weight = gamma ** (stage_count-stage-1)
            terms = []
            for index in selected:
                residual = means[stage][index] - labels[index]
                raw = raws[stage][index]
                variance = math.exp(2 * raw)
                term = lambda r: .5 * (residual**2 / math.exp(2*r) + 2*r)
                terms.append(term(raw))
                close(contract.log_std_to_variance(raw_tensors[stage]).flatten()[index].item(), variance)
                expected_gradient = weight / len(selected) * (1-residual**2/variance)
                close(raw_tensors[stage].grad.flatten()[index].item(), expected_gradient)
                step = 1e-5
                finite_difference = weight / len(selected) * (term(raw+step)-term(raw-step))/(2*step)
                assert math.isclose(expected_gradient, finite_difference, rel_tol=2e-7, abs_tol=2e-9)
                checks += 1
            expected += weight * sum(terms)/len(selected)
            for index in set(range(8))-set(selected):
                assert raw_tensors[stage].grad.flatten()[index].item() == 0
                checks += 1
        for stage in set(range(stage_count))-set(evaluated):
            assert raw_tensors[stage].grad is None
            checks += 1
        assert all(p.grad is None for p in predictions) and target.grad is None
        assert result.valid_pixels == sum(valid) and result.valid_components == len(selected)
        checks += 2
        close(result.loss.item(), expected)
        cases.append({'case': case, 'stages': stage_count, 'final_only': final_only,
                      'valid_pixels': result.valid_pixels, 'loss': result.loss.item(),
                      'reference_loss': expected})
    return {'status': 'PASS_NUMERICAL_CONTRACT_ONLY', 'device': device,
            'dtype': str(dtype), 'checks': checks, 'cases': cases,
            'max_absolute_error': max_absolute_error, 'max_tolerance_fraction': max_scaled_error,
            'tolerances': tolerance, 'finite_difference_rtol': 2e-7,
            'finite_difference_atol': 2e-9}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', choices=('cpu', 'cuda'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    if args.device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('declared CUDA numerical gate requires CUDA')
    result = validate(args.device)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    result.update(contract_id=contract.CONTRACT_ID,
        created_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        objective_sha256=sha(Path(contract.__file__)), probe_sha256=sha(Path(__file__)),
        torch=torch.__version__, python=platform.python_version(),
        device_name=torch.cuda.get_device_name(0) if args.device == 'cuda' else 'CPU',
        model_or_image_forward=False, optimizer_steps=0, training_performed=False,
        dataset_or_checkpoint_opened=False, historical_checkpoint_recipe_established=False)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False); stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='cases'}, indent=2))


if __name__ == '__main__':
    main()
