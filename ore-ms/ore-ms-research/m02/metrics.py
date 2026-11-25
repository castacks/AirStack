"""Metrics for aligned linear RGB captures with a fixed detector full scale."""
import math
import statistics


def michelson(light, dark):
    return (light - dark) / (light + dark) if light + dark else None


def rms(values):
    return math.sqrt(statistics.fmean(value**2 for value in values))


def rmse(actual, reference):
    return rms(a - b for a, b in zip(actual, reference, strict=True))


def snr_db(actual, ideal):
    signal, residual = rms(ideal), rmse(actual, ideal)
    return 20 * math.log10(signal / residual) if signal and residual else None


def local_ssim(actual, reference, width, height, window=11):
    """Mean SSIM over valid uniform windows on a fixed [0, 1] scale."""
    if (window < 3 or window % 2 == 0 or min(width, height) < window
            or len(actual) != width * height or len(reference) != width * height):
        raise ValueError("invalid SSIM shape or window")
    c1, c2 = 0.01**2, 0.03**2
    scores = []
    for top in range(height - window + 1):
        for left in range(width - window + 1):
            indices = [(top + y) * width + left + x for y in range(window) for x in range(window)]
            a = [actual[i] for i in indices]
            b = [reference[i] for i in indices]
            mean_a, mean_b = statistics.fmean(a), statistics.fmean(b)
            variance_a = statistics.fmean((x - mean_a)**2 for x in a)
            variance_b = statistics.fmean((y - mean_b)**2 for y in b)
            covariance = statistics.fmean(
                (x - mean_a) * (y - mean_b) for x, y in zip(a, b, strict=True))
            scores.append(((2 * mean_a * mean_b + c1) * (2 * covariance + c2))
                          / ((mean_a**2 + mean_b**2 + c1) * (variance_a + variance_b + c2)))
    return statistics.fmean(scores)
