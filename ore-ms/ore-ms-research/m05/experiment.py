"""Generate seeded ensembles, environment trajectories and stress receipts."""
import argparse
import csv
import json
import math
from pathlib import Path
import random

from environment import Field, advance, evaluate, physical_sample, stationary, transition


def grid_index(time, dt):
    index = round(time / dt)
    if not math.isclose(index * dt, time, abs_tol=1e-12, rel_tol=0):
        raise ValueError("the requested sample or regime change is off the time grid")
    return index


def ensemble(config, field, writer):
    origin = field.basis(0)
    separations = [field.basis(h) for h in config["separations_m"]]
    for arm in config["stationary_arms"]:
        rng = random.Random(arm["seed"])
        update = transition(arm["dt_s"], arm["tau_s"], arm["sigma"])
        means = (0,) * field.size
        steps = [grid_index(lag, arm["dt_s"]) for lag in config["lags_s"]]
        for i in range(config["stationary_size"]):
            state = stationary(rng, arm["sigma"], field.size)
            temperature = stationary(rng, config["temperature"]["sigma"], field.size)
            spatial = [evaluate(state, basis) for basis in separations]
            temporal = [evaluate(state, origin)]
            for step in range(1, steps[-1] + 1):
                state = advance(state, means, update, rng)
                if step in steps:
                    temporal.append(evaluate(state, origin))
            writer.writerow((arm["name"], i, *temporal, *spatial, evaluate(temperature, origin)))


def forced_paths(config, field, writer):
    settings = config["forced"]
    update = transition(settings["dt_s"], settings["tau_s"], settings["sigma"])
    clear = field.mean_coefficients(settings["clear_mean"])
    driven = field.mean_coefficients(settings["driven_mean"], settings["driven_cosine"])
    origin = field.basis(0)
    rng = random.Random(config["forced_seed"])
    for i in range(config["forced_size"]):
        fluctuation = stationary(rng, settings["sigma"], field.size)
        state = tuple(a + b for a, b in zip(clear, fluctuation, strict=True))
        dt = settings["dt_s"]
        end = grid_index(settings["end_s"], dt)
        start, stop = [grid_index(time, dt) for time in settings["switch_s"]]
        checks = {grid_index(time, dt) for time in settings["check_s"]}
        for step in range(end + 1):
            if step in checks:
                time = step * dt
                writer.writerow((i, time, evaluate(state, origin)))
            means = driven if start <= step < stop else clear
            if step < end:
                state = advance(state, means, update, rng)


def samples(config, field, state, temperature, velocity, time, medium):
    for i in range(config["grid_cells"]):
        position = i * field.period_m / config["grid_cells"]
        basis = field.basis(position, time, velocity)
        latent, heat = evaluate(state, basis), evaluate(temperature, basis)
        yield position, latent, heat, physical_sample(latent, heat, velocity,
                                                      config["media"][medium], config["temperature"])


def serialize_sample(sample):
    return (sample.burden, sample.temperature_K, sample.rgb_extinction_550_m_inv,
            sample.rgb_absorption_550_m_inv, sample.rgb_scattering_550_m_inv,
            sample.rgb_exponent, sample.scattering_fraction, *sample.lwir_absorption_m_inv,
            sample.velocity_m_s)


def trajectories(config, field, writer):
    forced, heat = config["forced"], config["temperature"]
    dt = forced["dt_s"]
    end = grid_index(forced["end_s"], dt)
    start, stop = [grid_index(time, dt) for time in forced["switch_s"]]
    clear = field.mean_coefficients(forced["clear_mean"])
    driven = field.mean_coefficients(forced["driven_mean"], forced["driven_cosine"])
    zero = (0,) * field.size
    temperature_update = transition(dt, heat["tau_s"], heat["sigma"])
    for name, kind, sigma in (("forced_clear_cycle", "fog", 0), ("synthetic_fog", "fog", forced["sigma"]),
                              ("synthetic_dust", "dust", forced["sigma"])):
        rng = random.Random(config["trajectory_seed"])
        coeffs, temperature = clear, zero
        update = transition(dt, forced["tau_s"], sigma)
        for step in range(end + 1):
            time = step * dt
            for position, latent, thermal, sample in samples(
                    config, field, coeffs, temperature, forced["velocity_m_s"], time, kind):
                writer.writerow((name, time, position, latent, thermal, *serialize_sample(sample)))
            means = driven if start <= step < stop else clear
            if step < end:
                coeffs = advance(coeffs, means, update, rng)
                temperature = advance(temperature, zero, temperature_update, rng) if sigma else zero
    moving = config["transport"]
    coeffs = field.mean_coefficients(0, moving["fixed_cosine"])
    for time in range(0, moving["end_s"] + 1, moving["dt_s"]):
        for position, latent, thermal, sample in samples(
                config, field, coeffs, zero, moving["velocity_m_s"], time, "fog"):
            writer.writerow(("periodic_transport", time, position, latent, thermal,
                             *serialize_sample(sample)))


def stress(config, field, writer):
    settings, heat = config["stress"], config["temperature"]
    rng = random.Random(config["stress_seed"])
    means = (0,) * field.size
    update = transition(settings["dt_s"], settings["tau_s"], settings["sigma"])
    temperature_update = transition(settings["dt_s"], heat["tau_s"], heat["sigma"])
    for index in range(settings["paths"]):
        coefficients = stationary(rng, settings["sigma"], field.size)
        temperature = stationary(rng, heat["sigma"], field.size)
        ranges = [[math.inf, -math.inf] for _ in range(10)]
        count = invalid = 0
        for step in range(settings["steps"]):
            for _, _, _, state in samples(config, field, coefficients, temperature,
                                          settings["velocity_m_s"], step * settings["dt_s"], "dust"):
                values = serialize_sample(state)[:10]
                for endpoints, value in zip(ranges, values, strict=True):
                    endpoints[0] = min(endpoints[0], value)
                    endpoints[1] = max(endpoints[1], value)
                count += 1
                invalid += not (all(map(math.isfinite, values))
                                and 0 <= state.burden <= 1 and 250 <= state.temperature_K <= 330
                                and 0 <= state.scattering_fraction <= 1 and min(values[2:]) >= 0
                                and len(set(state.lwir_absorption_m_inv)) > 1
                                and abs(state.rgb_extinction_550_m_inv
                                        - state.rgb_absorption_550_m_inv
                                        - state.rgb_scattering_550_m_inv) <= 2e-12)
            coefficients = advance(coefficients, means, update, rng)
            temperature = advance(temperature, means, temperature_update, rng)
        writer.writerow((index, count, invalid, *(value for ends in ranges for value in ends)))


FIELDS = ("burden", "air_K", "rgb_extinction_m_inv", "rgb_absorption_m_inv",
          "rgb_scattering_m_inv", "rgb_exponent", "scattering_fraction", "lwir_8_m_inv",
          "lwir_11_m_inv", "lwir_14_m_inv", "velocity_m_s")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    model = Field(config["period_m"], tuple(config["weights"]))
    args.directory.mkdir()
    outputs = (
        ("ensemble.csv", ("arm", "realization", *(f"lag_{lag}" for lag in config["lags_s"]),
                          *(f"space_{h}" for h in config["separations_m"]), "temperature_latent"),
         ensemble),
        ("forced.csv", ("realization", "time_s", "burden_latent"), forced_paths),
        ("trajectories.csv", ("case", "time_s", "position_m", "burden_latent", "temperature_latent",
                             *FIELDS), trajectories),
        ("stress.csv", ("path", "evaluations", "invalid",
                        *(f"{field}_{bound}" for field in FIELDS[:10] for bound in ("min", "max"))),
         stress),
    )
    for filename, columns, execute in outputs:
        with (args.directory / filename).open("w", newline="") as destination:
            writer = csv.writer(destination, lineterminator="\n")
            writer.writerow(columns)
            execute(config, model, writer)


if __name__ == "__main__":
    main()
