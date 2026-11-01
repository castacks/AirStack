"""Opt-in covariance-only objective v1; raw outputs are log standard deviations.

Independent of the legacy trainer/loss. No clipping, hidden epsilon, or automatic
empty-batch skip. This eager contract intentionally checks numerical failures.
"""
from dataclasses import dataclass
import math

import torch


FLOAT_DTYPES = (torch.float16, torch.bfloat16, torch.float32, torch.float64)
CONTRACT_ID = "diagonal-log-std-fixed-mean-nll-v1"


def _floating(value):
    if not isinstance(value, torch.Tensor) or value.dtype not in FLOAT_DTYPES:
        raise ValueError("floating tensor (fp16/bf16/fp32/fp64) required")


def log_std_to_variance(raw):
    """exp(2*raw), in FP32 minimum; reject nonfinite variance or precision.

    Call on valid entries only if the source contains masked invalid values.
    FP32/FP64 match the released inference transform, without changing that path.
    """
    _floating(raw)
    value = raw if raw.dtype == torch.float64 else raw.float()
    variance = torch.exp(2 * value)
    if (not torch.isfinite(value).all() or not torch.isfinite(variance).all()
            or not (variance > 0).all()
            or not torch.isfinite(variance.reciprocal()).all()):
        raise ValueError("log standard deviation has unrepresentable variance/precision")
    return variance


@dataclass(frozen=True)
class CovarianceObjective:
    loss: torch.Tensor
    valid_pixels: int
    valid_components: int
    evaluated_stages: int
    final_mean_variance: torch.Tensor


def sequence_covariance_nll(predictions, raw_log_stds, target, valid_mask, *,
                            gamma, max_flow, final_only=False):
    """Fixed-mean diagonal NLL: mean(.5 * (error²/exp(2*raw) + 2*raw)).

    Inputs are nonempty lists/tuples of matching [B,2,H,W] floating tensors.
    Boolean mask is [B,H,W] or [B,1,H,W]. A pixel needs two finite target
    components and norm(target)<max_flow. Invalid prediction/raw on a valid pixel
    is an error, not an additional mask. Residual/target receive no gradients.
    Recurrent losses are summed with gamma**(T-i-1), never weight-normalized.
    Empty valid sets raise: caller must explicitly record/handle that condition.
    """
    _floating(target)
    if target.ndim != 4 or target.shape[1] != 2 or any(n == 0 for n in target.shape):
        raise ValueError("nonempty target [B,2,H,W] required")
    if (not isinstance(predictions, (tuple, list)) or not predictions
            or not isinstance(raw_log_stds, (tuple, list))
            or len(predictions) != len(raw_log_stds)):
        raise ValueError("matching nonempty prediction/raw stage lists required")
    if (isinstance(gamma, bool) or not isinstance(gamma, (float, int))
            or not math.isfinite(gamma) or not 0 < gamma <= 1
            or isinstance(max_flow, bool) or not isinstance(max_flow, (float, int))
            or not math.isfinite(max_flow) or max_flow <= 0
            or type(final_only) is not bool):
        raise ValueError("finite 0<gamma<=1, max_flow>0 and boolean final_only required")
    tensors = [target, *predictions, *raw_log_stds]
    for value in tensors:
        _floating(value)
        if value.shape != target.shape or value.device != target.device:
            raise ValueError("all stages must match target shape/device")
    if (not isinstance(valid_mask, torch.Tensor) or valid_mask.dtype != torch.bool
            or valid_mask.device != target.device):
        raise ValueError("boolean mask on target device required")
    if valid_mask.shape == (target.shape[0], 1, *target.shape[2:]):
        valid_mask = valid_mask[:, 0]
    if valid_mask.shape != (target.shape[0], *target.shape[2:]):
        raise ValueError("mask must be [B,H,W] or [B,1,H,W]")

    dtype = torch.float64 if any(t.dtype == torch.float64 for t in tensors) else torch.float32
    gt = target.detach().to(dtype)
    valid = (valid_mask & torch.isfinite(gt).all(dim=1)
             & (torch.linalg.vector_norm(gt, dim=1) < max_flow))
    count = int(valid.sum().item())
    if count == 0:
        raise ValueError("no valid target pixels; caller must explicitly handle empty batch")
    selected_gt = gt.movedim(1, -1)[valid]
    indices = [len(predictions) - 1] if final_only else range(len(predictions))
    loss = torch.zeros((), dtype=dtype, device=target.device)
    for index in indices:
        prediction = predictions[index].detach().to(dtype).movedim(1, -1)[valid]
        raw = raw_log_stds[index].to(dtype).movedim(1, -1)[valid]
        if not torch.isfinite(prediction).all():
            raise ValueError("nonfinite predicted mean on a valid pixel")
        variance = log_std_to_variance(raw)
        error_squared = (prediction - selected_gt).square()
        # Direct log-domain precision avoids the reciprocal's variance**2
        # backward intermediate, which can underflow despite a finite NLL/grad.
        scaled_error = error_squared * torch.exp(-2 * raw)
        nll = 0.5 * (scaled_error + 2 * raw)
        if not torch.isfinite(nll).all():
            raise ValueError("nonfinite covariance likelihood on a valid pixel")
        weight = gamma ** (len(predictions) - index - 1)
        output_gradient = (weight / (2 * count)) * (1 - scaled_error.detach())
        if not torch.isfinite(output_gradient.to(raw_log_stds[index].dtype)).all():
            raise ValueError("unrepresentable covariance-output gradient")
        loss = loss + weight * nll.mean()
    if not torch.isfinite(loss):
        raise ValueError("nonfinite accumulated covariance likelihood")
    return CovarianceObjective(loss, count, 2 * count, len(indices), variance.detach().mean())
