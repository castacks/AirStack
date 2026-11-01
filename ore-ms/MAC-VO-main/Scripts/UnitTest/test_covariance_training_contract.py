import math

import pytest
import torch

from Train.MatchingNet.covariance_contract import (
    log_std_to_variance, sequence_covariance_nll,
)


def objective(raw, prediction=None, target=None, mask=None, **options):
    prediction = torch.ones_like(raw) if prediction is None else prediction
    target = torch.zeros_like(raw) if target is None else target
    mask = torch.ones(raw.shape[0], 1, *raw.shape[2:], dtype=torch.bool,
                      device=raw.device) if mask is None else mask
    return sequence_covariance_nll([prediction], [raw], target, mask,
                                   gamma=options.pop('gamma', 0.85),
                                   max_flow=options.pop('max_flow', 100), **options)


def test_closed_form_nll_and_gradient():
    raw = torch.full((1, 2, 1, 1), 0.5, dtype=torch.float64, requires_grad=True)
    result = objective(raw, torch.full_like(raw, 2))
    assert result.loss.item() == pytest.approx(0.5 * (4 / math.e + 1), abs=1e-14)
    result.loss.backward()
    torch.testing.assert_close(raw.grad, torch.full_like(raw, (1 - 4 / math.e) / 2))
    assert (result.valid_pixels, result.valid_components, result.evaluated_stages) == (1, 2, 1)
    assert not result.final_mean_variance.requires_grad


def test_only_covariance_receives_gradients():
    raw = torch.zeros((1, 2, 1, 1), requires_grad=True)
    prediction = torch.ones_like(raw, requires_grad=True)
    target = torch.zeros_like(raw, requires_grad=True)
    objective(raw, prediction, target).loss.backward()
    assert raw.grad is not None and prediction.grad is None and target.grad is None


@pytest.mark.parametrize('residual', [0.25, 0.5, 2.0, 4.0])
def test_variance_optimum_is_squared_residual(residual):
    raw = torch.full((1, 2, 1, 1), math.log(residual), dtype=torch.float64, requires_grad=True)
    result = objective(raw, torch.full_like(raw, residual))
    result.loss.backward()
    assert raw.grad.abs().max().item() < 1e-14
    for offset, sign in ((-0.1, -1), (0.1, 1)):
        probe = (raw.detach() + offset).requires_grad_()
        objective(probe, torch.full_like(raw, residual)).loss.backward()
        assert torch.all(sign * probe.grad > 0)


def test_masking_before_math_and_valid_denominator():
    raw = torch.tensor([[[[0., float('nan'), float('inf'), -1.]],
                         [[0., float('nan'), float('inf'), -1.]]]], requires_grad=True)
    prediction = torch.tensor([[[[2., float('nan'), float('inf'), 9.]],
                                [[4., float('nan'), float('inf'), 9.]]]])
    target = torch.tensor([[[[0., 0., float('nan'), 100.]],
                           [[0., 0., 0., 0.]]]])
    result = objective(raw, prediction, target,
                       torch.tensor([[[True, False, True, True]]]))
    assert result.loss.item() == 5 and result.valid_pixels == 1
    result.loss.backward()
    torch.testing.assert_close(raw.grad[0, :, 0, 0], torch.tensor([-1.5, -7.5]))
    assert torch.equal(raw.grad[..., 1:], torch.zeros_like(raw.grad[..., 1:]))


@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
@pytest.mark.parametrize('field', ['prediction', 'raw'])
def test_invalid_model_values_on_valid_target_raise(invalid, field):
    raw = torch.zeros((1, 2, 1, 1))
    prediction = torch.ones_like(raw)
    (raw if field == 'raw' else prediction)[0, 0, 0, 0] = invalid
    with pytest.raises(ValueError):
        objective(raw, prediction)


@pytest.mark.parametrize('reason', ['mask', 'nan', 'inf', 'threshold'])
def test_empty_valid_set_is_explicit_error(reason):
    raw = torch.zeros((1, 2, 1, 1))
    target = torch.zeros_like(raw)
    mask = torch.ones((1, 1, 1), dtype=torch.bool)
    if reason == 'mask': mask[:] = False
    elif reason == 'threshold': target[:, 0] = 100
    else: target[:, 0] = float(reason)
    with pytest.raises(ValueError, match='no valid target'):
        objective(raw, target=target, mask=mask)


def test_stage_weight_order_and_final_only():
    raw = [torch.zeros((1, 2, 1, 1), requires_grad=True) for _ in range(3)]
    predictions = [torch.full_like(raw[0], v) for v in (1., 2., 3.)]
    target = torch.zeros_like(raw[0]);mask = torch.ones((1, 1, 1), dtype=torch.bool)
    result = sequence_covariance_nll(predictions, raw, target, mask, gamma=0.5, max_flow=10)
    assert result.loss.item() == .25 * .5 + .5 * 2 + 4.5
    assert result.evaluated_stages == 3
    result.loss.backward()
    for stage, (weight, error) in enumerate(zip((.25, .5, 1), (1, 4, 9))):
        torch.testing.assert_close(raw[stage].grad, torch.full_like(raw[stage], weight * (1 - error) / 2))
        raw[stage].grad = None
    result = sequence_covariance_nll(predictions, raw, target, mask, gamma=.5, max_flow=10, final_only=True)
    assert result.loss.item() == 4.5 and result.evaluated_stages == 1
    result.loss.backward()
    assert raw[0].grad is None and raw[1].grad is None and raw[2].grad is not None


@pytest.mark.parametrize('dtype', [torch.float16, torch.bfloat16, torch.float32, torch.float64])
def test_dtype_and_released_variance_transform(dtype):
    raw = torch.full((1, 2, 1, 1), -.5, dtype=dtype, requires_grad=True)
    expected = torch.exp(2 * (raw if dtype == torch.float64 else raw.float()))
    assert torch.equal(log_std_to_variance(raw), expected)
    result = objective(raw)
    assert result.loss.dtype == (torch.float64 if dtype == torch.float64 else torch.float32)
    result.loss.backward()
    assert torch.isfinite(raw.grad).all()


def test_direct_precision_avoids_variance_squared_backward_underflow():
    raw = torch.full((1, 2, 1, 1), -25., requires_grad=True)
    result = objective(raw)
    result.loss.backward()
    expected = (1 - math.exp(50)) / 2
    assert torch.isfinite(raw.grad).all()
    torch.testing.assert_close(raw.grad, torch.full_like(raw, expected), rtol=2e-6, atol=0)


def test_promoted_loss_rejects_gradient_that_overflows_half_output():
    with pytest.raises(ValueError, match='covariance-output gradient'):
        objective(torch.full((1, 2, 1, 1), -6., dtype=torch.float16))


@pytest.mark.parametrize('raw_value', [-1000., 1000., -50., 50.])
def test_unrepresentable_variance_or_precision_rejected(raw_value):
    with pytest.raises(ValueError, match='unrepresentable'):
        objective(torch.full((1, 2, 1, 1), raw_value))


def test_finite_prediction_with_overflowing_squared_error_rejected():
    with pytest.raises(ValueError, match='nonfinite covariance likelihood'):
        objective(torch.zeros((1, 2, 1, 1)), torch.full((1, 2, 1, 1), 1e30))


def test_gradcheck_with_masked_components():
    raw = torch.tensor([[[[-.5, .25]], [[.5, -.25]]]], dtype=torch.float64, requires_grad=True)
    assert torch.autograd.gradcheck(lambda value: objective(value, mask=torch.tensor([[[True, False]]])).loss,
                                   (raw,), eps=1e-6, atol=1e-5, rtol=1e-4)


@pytest.mark.parametrize('key,value', [('gamma', 0), ('gamma', 1.1), ('gamma', float('nan')),
    ('gamma', True), ('max_flow', 0), ('max_flow', float('inf')), ('final_only', 1)])
def test_bad_options_rejected(key, value):
    with pytest.raises(ValueError):
        objective(torch.zeros((1, 2, 1, 1)), **{key: value})


@pytest.mark.parametrize('mutation', ['empty', 'length', 'tensor_stages', 'shape', 'dtype',
                                    'mask_dtype', 'mask_shape', 'target_shape', 'empty_target'])
def test_invalid_input_contracts(mutation):
    target = torch.zeros((1, 2, 1, 1));pred = [torch.ones_like(target)];raw = [target.clone()]
    mask = torch.ones((1, 1, 1), dtype=torch.bool)
    if mutation == 'empty': pred = [];raw = []
    elif mutation == 'length': raw = []
    elif mutation == 'tensor_stages': pred = pred[0]
    elif mutation == 'shape': raw = [torch.zeros(1, 2, 1, 2)]
    elif mutation == 'dtype': raw = [raw[0].long()]
    elif mutation == 'mask_dtype': mask = mask.float()
    elif mutation == 'mask_shape': mask = torch.ones(1, 2, 1, 1, dtype=torch.bool)
    elif mutation == 'target_shape': target = torch.zeros(1, 3, 1, 1)
    elif mutation == 'empty_target': target = torch.zeros(1, 2, 0, 1)
    with pytest.raises(ValueError):
        sequence_covariance_nll(pred, raw, target, mask, gamma=.85, max_flow=100)
