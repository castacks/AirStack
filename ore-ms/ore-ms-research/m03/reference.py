"""Independent numerical reference constructions for M03."""
from decimal import Decimal as D, localcontext
import math

# CODATA 2022, W m^-2 K^-4, rounded at the published digits.
STEFAN_BOLTZMANN = 5.670374419e-8


def planck_reference(wavelength_um, temperature):
    """Evaluate directly in micrometre units using 45-digit arithmetic."""
    with localcontext() as context:
        context.prec = 45
        light = D(299792458)
        planck = D("6.62607015e-34")
        boltzmann = D("1.380649e-23")
        um = D(str(wavelength_um))
        amplitude = 2 * planck * light**2 * D("1e24")
        characteristic = planck * light * D("1e6") / boltzmann
        return float(amplitude / (um**5 * ((characteristic / (um * D(temperature))).exp() - 1)))


def integrate_simpson(function, low, high, intervals):
    if intervals < 2 or intervals % 2:
        raise ValueError("Simpson integration requires positive even intervals")
    width = (high - low) / intervals
    odd = math.fsum(function(low + i * width) for i in range(1, intervals, 2))
    even = math.fsum(function(low + i * width) for i in range(2, intervals, 2))
    return width / 3 * (function(low) + 4 * odd + 2 * even + function(high))


def rk4_layers(boundary, layer_conditions, steps):
    """Integrate dL/ds = κ(B−L) separately over each layer."""
    result = boundary
    for length, coefficient, source in layer_conditions:
        step = length / steps

        def slope(value):
            return coefficient * (source - value)

        for _ in range(steps):
            a = slope(result)
            b = slope(result + step * a / 2)
            c = slope(result + step * b / 2)
            d = slope(result + step * c)
            result += step / 6 * (a + 2 * b + 2 * c + d)
    return result


def interpolate(wavelength, values):
    nodes = (8, 11, 14)
    index = 0 if wavelength < nodes[1] else 1
    left, right = nodes[index:index + 2]
    return (values[index] * (right - wavelength) + values[index + 1] * (wavelength - left)) / 3


def combined_band_reference(config, case, intervals):
    """Integrate a direct sum of boundary and downstream-attenuated layer terms."""
    target_temperature = config["default_target"]["temperature_K"]

    def integrand(wavelength):
        eps = interpolate(wavelength, case["target_emissivity"])
        reflected = interpolate(wavelength, case["reflected_radiance"])
        boundary = eps * planck_reference(wavelength, target_temperature) + (1 - eps) * reflected
        terms = []
        total_depth = 0
        for index in reversed(case["layer_order"]):
            layer = config["layers"][index]
            depth = layer["length_m"] * interpolate(wavelength, layer["absorption_m_inv"])
            local = planck_reference(wavelength, layer["temperature_K"]) * (1 - math.exp(-depth))
            terms.append(local * math.exp(-total_depth))
            total_depth += depth
        return interpolate(wavelength, config["response"]) * (
            math.fsum(terms) + boundary * math.exp(-total_depth))

    return integrate_simpson(integrand, *config["wavelength_limits_um"], intervals)
