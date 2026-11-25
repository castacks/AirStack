"""Analytical references and independent-ensemble statistics for M05."""
import math
import statistics


def spatial_covariance(distance, period):
    # cos(2θ)=2 cos²(θ)−1, independent of the solver's basis evaluation.
    cosine = math.cos(2 * math.pi * distance / period)
    return 0.6 * cosine**2 + 0.5 * cosine - 0.1


def temporal_covariance(lag, relaxation):
    return 1 / math.exp(lag / relaxation)


def forced_mean(time, parameters):
    start, stop = parameters["switch_s"]
    clear = parameters["clear_mean"]
    gap = parameters["driven_mean"] + parameters["driven_cosine"] - clear
    tau = parameters["tau_s"]
    if time <= start:
        return clear
    length = min(time, stop) - start
    return clear + gap * (1 - math.exp(-length / tau)) * math.exp(-max(0, time - stop) / tau)


def confidence(values):
    """Pointwise approximate 99% CI over independent realization contributions."""
    center = statistics.fmean(values)
    width = 2.576 * statistics.stdev(values) / math.sqrt(len(values))
    return center, center - width, center + width
