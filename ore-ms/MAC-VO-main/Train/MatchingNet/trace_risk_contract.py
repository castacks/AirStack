"""Experimental final-stage trace risk score, independent of trainer defaults.

Predicts the conditional mean squared endpoint error through total variance.
This is a scalar mean-risk score, not a two-dimensional Gaussian likelihood.
It does not identify directional variance and does not guarantee better ranking.
"""
from dataclasses import dataclass

import torch

from .covariance_contract import sequence_covariance_nll


CONTRACT_ID = 'final-trace-fixed-mean-risk-score-v1'


@dataclass(frozen=True)
class TraceRiskObjective:
    loss: torch.Tensor
    valid_pixels: int
    evaluated_stages: int
    final_mean_trace: torch.Tensor


def final_trace_risk_score(predictions, raw_log_stds, target, valid_mask, *, max_flow):
    """Mean .5*(s/v + log(v)), s=||fixed error||², v=sum(exp(2*raw)).

    Shared diagonal-contract admission supplies shape, mask, target-flow limit,
    finite checks, and FP32 minimum. It deliberately restricts the common numeric
    domain to that of the diagonal comparator, including its output-gradient
    checks. Validation is detached; only the final raw stage receives gradients.
    No clipping, epsilon, weighting coefficient, pair mining, or empty-set skip.
    A conditional all-zero error has its infimum as v approaches zero; no finite
    optimum or coercivity is claimed. Only use as an explicitly admitted experiment.
    """
    with torch.no_grad():
        admitted = sequence_covariance_nll(predictions, raw_log_stds, target,
                                          valid_mask, gamma=1., max_flow=max_flow,
                                          final_only=True)
    tensors = [target, *predictions, *raw_log_stds]
    dtype = torch.float64 if any(t.dtype == torch.float64 for t in tensors) else torch.float32
    gt = target.detach().to(dtype)
    mask = valid_mask[:, 0] if valid_mask.ndim == 4 else valid_mask
    valid = (mask & torch.isfinite(gt).all(dim=1)
             & (torch.linalg.vector_norm(gt, dim=1) < max_flow))
    prediction = predictions[-1].detach().to(dtype).movedim(1, -1)[valid]
    raw = raw_log_stds[-1].to(dtype).movedim(1, -1)[valid]
    error_squared = (prediction - gt.movedim(1, -1)[valid]).square().sum(dim=-1)
    log_trace = torch.logsumexp(2 * raw, dim=-1)
    trace = torch.exp(log_trace)
    scaled = error_squared * torch.exp(-log_trace)
    score = .5 * (scaled + log_trace)
    if (not torch.isfinite(error_squared).all() or not torch.isfinite(trace).all()
            or not (trace > 0).all() or not torch.isfinite(trace.reciprocal()).all()
            or not torch.isfinite(score).all()):
        raise ValueError('unrepresentable trace risk score or trace variance/precision')
    output_gradient = (torch.softmax(2 * raw.detach(), dim=-1)
                       * (1 - scaled.detach()).unsqueeze(-1) / admitted.valid_pixels)
    if not torch.isfinite(output_gradient.to(raw_log_stds[-1].dtype)).all():
        raise ValueError('unrepresentable trace-output gradient')
    loss = score.mean()
    if not torch.isfinite(loss):
        raise ValueError('nonfinite accumulated trace risk score')
    return TraceRiskObjective(loss, admitted.valid_pixels, 1, trace.detach().mean())
