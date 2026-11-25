"""Reference constructions maintained separately from the RGB forward model."""
from decimal import Decimal, localcontext


def analytic_slab(boundary, absorption, scattering, length, incident_integral):
    """Return transmission and radiance from the constant-source IVP."""
    with localcontext() as context:
        context.prec = 45
        incident, a, s, d, source = (
            Decimal(str(value)) for value in
            (boundary, absorption, scattering, length, incident_integral))
        total = a + s
        if not total:
            return 1.0, float(incident)
        t = (-total * d).exp()
        return float(t), float(incident * t + s * source * (1 - t) / total)


def manufactured_coefficients(distance):
    return (0.024 + 0.0024 * distance, 0.035 + 0.0016 * distance,
            0.003 + 0.00018 * distance)


def continuous_reference(steps):
    """Integrate the manufactured IVP directly with classical RK4."""
    h = 12 / steps
    radiance = 0.008

    def derivative(position, value):
        absorption, scattering, angular_radiance = manufactured_coefficients(position)
        return scattering * angular_radiance - (absorption + scattering) * value

    for i in range(steps):
        x = i * h
        first = derivative(x, radiance)
        second = derivative(x + h / 2, radiance + h * first / 2)
        third = derivative(x + h / 2, radiance + h * second / 2)
        fourth = derivative(x + h, radiance + h * third)
        radiance += h * (first + 2 * second + 2 * third + fourth) / 6
    return radiance


def piecewise_projection(wavelengths, values, sensitivity):
    """Simpson quadrature integrates each quadratic interpolant exactly."""
    terms = []
    for i in range(1, len(wavelengths)):
        width = wavelengths[i] - wavelengths[i - 1]
        left, right = values[i - 1:i + 1]
        low, high = sensitivity[i - 1:i + 1]
        at_middle = ((left + right) / 2) * ((low + high) / 2)
        terms.append(width * (left * low + 4 * at_middle + right * high) / 6)
    return sum(terms)
