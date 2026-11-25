"""Independent pixel-plane and optical-depth references for the frozen corpus."""
import math
import statistics


def expected_hit(config, case, time, row, col):
    camera = config["camera"]
    x = (col - camera["cx"]) / camera["fx"]
    y = (row - camera["cy"]) / camera["fy"]
    target, background = config["target"], config["background"]
    surfaces = [dict(target, center=[case.get("target_x", 0), 0, 10],
                     material=case.get("material", target["material"])),
                dict(background, center=[case.get("background_x", 0), 0, 15])]
    if "obstacle" in case:
        surfaces.append(config["obstacles"][case["obstacle"]])
    covered = []
    for obj in surfaces:
        cx, cy, z = [p + time * v for p, v in
                     zip(obj["center"], obj.get("velocity", [0, 0, 0]), strict=True)]
        du, dv = obj["half"]
        if cx-du <= x*z <= cx+du and cy-dv <= y*z <= cy+dv:
            covered.append((z, obj["id"], obj["material"]))
    if not covered:
        return False, "", "", None
    z, ident, material = min(covered)
    return any(a[1] == "target" for a in covered), ident, material, (
        x*z, y*z, z, z * math.sqrt(1 + x*x + y*y))


def latent(field, coefficients, z, time):
    phase = 2 * math.pi * (z - field["velocity_m_s"] * time) / field["period_m"]
    a, b, c, d, e = coefficients
    return (math.sqrt(.2)*a + math.sqrt(.5)*(math.cos(phase)*b + math.sin(phase)*c)
            + math.sqrt(.3)*(math.cos(2*phase)*d + math.sin(2*phase)*e))


def physical(config, coefficients, heat, z, time):
    field = config["field"]
    fraction = 1 / (1 + math.exp(-latent(field, coefficients, z, time)))
    temperature_fraction = 1 / (1 + math.exp(-latent(field, heat, z, time)))
    air = field["thermal"]
    temperature = air["min_K"] + (air["max_K"] - air["min_K"]) * temperature_fraction
    optics = field["optical"]
    rgb = optics["rgb_floor"] + optics["rgb_span"] * fraction
    lwir = [f + s*fraction for f, s in zip(optics["lwir_floor"], optics["lwir_span"], strict=True)]
    return fraction, temperature, rgb, lwir


def integrated_depths(config, coefficients, heat, z, length, time):
    """Simpson from camera to surface, independent of the production slabs."""
    count = 4096
    rgb_terms, lw8_terms, lw11_terms = [], [], []
    for i in range(count + 1):
        _, _, rgb, lwir = physical(config, coefficients, heat, z * i / count, time)
        weight = 1 if i in (0, count) else 4 if i % 2 else 2
        rgb_terms.append(weight * rgb)
        lw8_terms.append(weight * lwir[0])
        lw11_terms.append(weight * lwir[1])
    rgb, eight, eleven = [length / (3 * count) * math.fsum(values)
                          for values in (rgb_terms, lw8_terms, lw11_terms)]
    exponent = config["field"]["optical"]["rgb_exponent"]
    return rgb * (550 / 450)**exponent, rgb, eight, eleven


def interval(values):
    mean = statistics.fmean(values)
    width = 1.96 * statistics.stdev(values) / math.sqrt(len(values))
    return mean, mean - width, mean + width
