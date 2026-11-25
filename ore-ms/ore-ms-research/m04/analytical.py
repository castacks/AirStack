"""Independent screen-space and radiometric reference calculations."""
from fractions import Fraction as F
from itertools import product
import math


def rational(value):
    return F(str(value))


def outline(config, case, surface, time):
    """Project rectangle corners with rational arithmetic into a pixel interval."""
    camera = config["camera"]
    origin = tuple(map(rational, case.get("camera_origin", camera["origin"])))
    basis = case.get("camera_axes", camera["axes"])
    center = tuple(rational(value) + rational(rate) * time for value, rate in
                   zip(surface["center"], surface.get("velocity", [0, 0, 0]), strict=True))
    offsets = [(-rational(extent), rational(extent)) for extent in surface["half"]]
    points = []
    depths = set()
    for x, y in product(*offsets):
        displacement = (center[0] + x - origin[0], center[1] + y - origin[1],
                        center[2] - origin[2])
        horizontal, vertical, depth = (
            sum(component * axis for component, axis in zip(displacement, vector, strict=True))
            for vector in basis)
        assert depth > 0
        depths.add(depth)
        points.append((camera["cx"] + camera["fx"] * horizontal / depth,
                       camera["cy"] + camera["fy"] * vertical / depth))
    (depth,) = depths  # The frozen oracle only admits parallel projected planes.
    u, v = zip(*points)
    return min(u), max(u), min(v), max(v), depth


def frame(config, case, time):
    target = dict(config["target"])
    for field in ("center", "velocity"):
        if f"target_{field}" in case:
            target[field] = case[f"target_{field}"]
    objects = [target, *(config["obstacles"][key] for key in case.get("obstacles", []))]
    bounds = [(surface["id"], outline(config, case, surface, time)) for surface in objects]
    camera = config["camera"]
    for row in range(camera["height"]):
        for col in range(camera["width"]):
            occupied = {name: depth for name, (left, right, top, bottom, depth) in bounds
                        if left <= col <= right and top <= row <= bottom}
            relative_path = math.sqrt(1 + ((col - camera["cx"]) / camera["fx"])**2
                                      + ((row - camera["cy"]) / camera["fy"])**2)
            target_range = float(occupied["target"]) * relative_path if "target" in occupied else None
            if occupied:
                nearest = min(occupied, key=occupied.get)
                depth = float(occupied[nearest])
                yield row, col, target_range, nearest, depth, depth * relative_path
            else:
                yield row, col, target_range, "", None, None


def planck_reference(wavelength, kelvin):
    amplitude = 2 * 6.62607015e-34 * 299792458**2 * 1e24
    temperature_constant = 6.62607015e-34 * 299792458 * 1e6 / 1.380649e-23
    return amplitude / (wavelength**5 * math.expm1(temperature_constant / (wavelength * kelvin)))


def spectral_interp(w, samples, knots):
    i = 0 if w < knots[1] else 1
    a, b = knots[i:i+2]
    return (samples[i] * (b - w) + samples[i + 1] * (w - a)) / (b - a)


def rgb_integral(material, knots, depth=0, source=0):
    start, peak, end = knots
    rho, irradiance = (material["rgb_reflectance"], material["rgb_irradiance"])
    area = (end - start) / 2
    if len(set(rho)) == len(set(irradiance)) == 1:
        boundary = area * rho[0] * irradiance[0] / math.pi
    else:
        def surface(w):
            return (spectral_interp(w, rho, (400, 550, 700))
                    * spectral_interp(w, irradiance, (400, 550, 700)) / math.pi)

        def response(w):
            return (w - start) / (peak - start) if w <= peak else (end - w) / (end - peak)

        terms = []
        for lo in range(start, end, 5):
            hi = lo + 5
            left, right = surface(lo), surface(hi)
            # Simpson integrates the product of the two linear interpolants.
            terms.append(5 / 6 * (left * response(lo)
                                   + 2 * (left + right) * response(lo + 2.5)
                                   + right * response(hi)))
        boundary = math.fsum(terms)
    transmission = math.exp(-depth)
    return boundary * transmission, area * source * (1 - transmission)


def lwir_integrals(config, material, distance=0):
    medium = config["medium"]
    t = math.exp(-distance * medium["lwir_absorption_m_inv"])
    intervals = 3072
    terms = [[], [], []]
    for i in range(intervals + 1):
        w = 8 + i * 6 / intervals
        weight = (1 if i in (0, intervals) else 4 if i % 2 else 2)
        weight *= spectral_interp(w, config["lwir_response"], (8, 11, 14))
        eps = spectral_interp(w, material["emissivity"], (8, 11, 14))
        values = (t * eps * planck_reference(w, material["temperature_K"]),
                  t * (1 - eps) * planck_reference(w, material["reflected_K"]),
                  (1 - t) * planck_reference(w, medium["lwir_air_K"]))
        for term, value in zip(terms, values, strict=True):
            term.append(weight * value)
    return [math.fsum(values) * 2 / intervals for values in terms]
