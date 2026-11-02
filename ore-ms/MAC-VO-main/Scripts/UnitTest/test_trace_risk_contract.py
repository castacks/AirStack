"""Analytic mean-risk and numerical safety evidence, no neural model training."""
import math

import pytest
import torch

from Train.MatchingNet.trace_risk_contract import final_trace_risk_score as objective


def fixture(dtype=torch.float64):
    mean = torch.tensor([[[[1., 3.]], [[2., 1.]]]], dtype=dtype, requires_grad=True)
    target = torch.zeros_like(mean, requires_grad=True)
    raw = torch.tensor([[[[.1, -.2]], [[.4, .3]]]], dtype=dtype, requires_grad=True)
    mask = torch.ones(1, 1, 1, 2, dtype=torch.bool)
    return mean, target, raw, mask


def score(mean, target, raw, mask):
    return objective([mean], [raw], target, mask, max_flow=400.)


def test_independent_scalar_formula_and_analytic_raw_gradient():
    mean, target, raw, mask = fixture()
    result = score(mean, target, raw, mask)
    expected = 0.; gradients = torch.empty_like(raw)
    for pixel in range(2):
        s = sum(float(mean[0, axis, 0, pixel].detach())**2 for axis in range(2))
        vv = [math.exp(2*float(raw[0, axis, 0, pixel].detach())) for axis in range(2)]
        v = sum(vv); expected += .5*(s/v+math.log(v))/2
        for axis in range(2):gradients[0, axis, 0, pixel] = vv[axis]/v*(1-s/v)/2
    assert result.loss.item() == pytest.approx(expected, rel=1e-14)
    result.loss.backward()
    torch.testing.assert_close(raw.grad, gradients, rtol=1e-13, atol=1e-14)
    assert mean.grad is None and target.grad is None
    assert result.valid_pixels == 2 and result.evaluated_stages == 1


def test_float64_finite_difference_gradient():
    mean, target, raw, mask = fixture()
    assert torch.autograd.gradcheck(lambda x:score(mean, target, x, mask).loss,
                                    (raw,), eps=1e-6, atol=1e-6, rtol=1e-4)


def test_shared_trace_fits_mean_squared_error_not_mean_error_norm():
    # The two pixels have squared endpoint errors 5 and10. A shared v must fit7.5.
    mean, target, _, mask = fixture()
    optimum = 7.5
    def shared(log_v):
        t = torch.tensor(log_v, dtype=torch.float64, requires_grad=True)
        raw = (t/2 - math.log(2)/2).expand_as(mean)
        loss = score(mean, target, raw, mask).loss
        gradient, = torch.autograd.grad(loss, t)
        return loss.item(), gradient.item()
    loss, grad = shared(math.log(optimum))
    assert abs(grad)<1e-14
    for factor in (.5, 2.):assert shared(math.log(optimum*factor))[0] > loss
    assert shared(math.log(optimum/2))[1] < 0 < shared(math.log(optimum*2))[1]


def test_analytic_fixture_single_gradient_step_descends():
    mean, target, raw, mask = fixture()
    before = score(mean, target, raw, mask).loss
    grad, = torch.autograd.grad(before, raw)
    after = score(mean, target, raw.detach()-.01*grad, mask).loss
    assert after < before


def test_same_trace_different_anisotropy_is_unidentified():
    mean, target, _, mask = fixture()
    a = torch.tensor([1., 3.], dtype=torch.float64).reshape(1,2,1,1).expand_as(mean)
    b = torch.tensor([2., 2.], dtype=torch.float64).reshape(1,2,1,1).expand_as(mean)
    torch.testing.assert_close(score(mean,target,a.log()/2,mask).loss,
                               score(mean,target,b.log()/2,mask).loss,rtol=1e-14,atol=1e-14)


def test_final_stage_only_and_masked_nan():
    mean, target, raw, mask = fixture();mask[...,1] = False
    mean.data[...,1] = float('nan');raw.data[...,1] = float('nan')
    earlier = torch.full_like(raw,float('nan'),requires_grad=True)
    result = objective([mean,mean],[earlier,raw],target,mask,max_flow=400.)
    result.loss.backward();assert earlier.grad is None
    assert torch.isfinite(raw.grad).all() and torch.equal(raw.grad[...,1],torch.zeros_like(raw.grad[...,1]))
    assert result.valid_pixels == 1


@pytest.mark.parametrize('dtype',[torch.float16,torch.bfloat16,torch.float32])
def test_fp32_minimum(dtype):
    mean,target,raw,mask=fixture(dtype);out=score(mean,target,raw,mask)
    assert out.loss.dtype==torch.float32;out.loss.backward();assert torch.isfinite(raw.grad).all()


@pytest.mark.parametrize('failure',['empty','raw_nan','mean_inf','raw_overflow','mask_dtype','shape','max_flow'])
def test_invalid_domain_rejected(failure):
    mean,target,raw,mask=fixture()
    if failure=='empty':mask[:]=False
    elif failure=='raw_nan':raw.data[0,0,0,0]=float('nan')
    elif failure=='mean_inf':mean.data[0,0,0,0]=float('inf')
    elif failure=='raw_overflow':raw.data[0,0,0,0]=1000.
    elif failure=='mask_dtype':mask=mask.float()
    elif failure=='shape':raw=raw[:,:,:,:1]
    if failure=='max_flow':
        with pytest.raises(ValueError):objective([mean],[raw],target,mask,max_flow=0.)
    else:
        with pytest.raises(ValueError):score(mean,target,raw,mask)


def test_strict_flow_limit_and_nonfinite_target_excluded():
    mean,target,raw,mask=fixture();target=target.detach();target[0,0,0,1]=400.
    assert score(mean,target,raw,mask).valid_pixels==1
    target[0,0,0,1]=float('nan');assert score(mean,target,raw,mask).valid_pixels==1


def test_zero_error_has_no_positive_finite_optimum():
    mean,target,raw,mask=fixture();mean=target.detach().clone()
    high=score(mean,target,raw,mask).loss;low=score(mean,target,raw-1,mask).loss
    assert low<high


def test_trace_sum_overflow_rejected_even_when_axis_variances_finite():
    # Both axis variances fitFP32, their sum does not; common diagonal score fits.
    mean,target,raw,mask=fixture(torch.float32);raw=torch.full_like(raw,44.)
    with pytest.raises(ValueError,match='trace'):score(mean,target,raw,mask)
