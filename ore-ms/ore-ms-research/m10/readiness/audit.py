"""Recompute the readiness figures from immutable prior evidence."""
import csv
import math
from pathlib import Path
import re
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]


def select(file, **fields):
    with (ROOT / file).open(newline="") as stream:
        return [r for r in csv.DictReader(stream) if all(r[k] == v for k, v in fields.items())]


def value(file, column, **fields):
    row, = select(file, **fields)
    return float(row[column])


def regex_values(file, pattern):
    match = re.search(pattern, (ROOT / file).read_text())
    if not match:
        raise ValueError(f"missing expected evidence in {file}")
    return [float(text) for text in match.groups()]


def audit():
    out = csv.writer(sys.stdout, lineterminator="\n")
    out.writerow(("control", "estimate", "units", "immutable_source"))

    def emit(name, x, units, source):
        assert math.isfinite(x)
        out.writerow((name, format(x, ".10g"), units, source))

    m06 = "m06/evidence/statistics.csv"
    for name in ("fixed_geometry_pearson_r", "different_realization_comparator_r",
                 "rgb_tau550_matched_geometry_change", "lwir_tau11_matched_geometry_change",
                 "crossover_R_target_minus_background_W_m2_sr",
                 "crossover_lwir_target_minus_background_W_m2_sr"):
        emit(name, value(m06, "estimate", metric=name),
             "W m^-2 sr^-1" if "W_m2_sr" in name else "dimensionless", m06)
    m07 = "m07/evidence/observability.csv"
    for case in ("joint_known", "unknown_range_one_band", "unknown_range_two_bands",
                 "lwir_crossover", "target_occluded", "rail_plateau"):
        emit(case + "_rank", value(m07, "rank", case=case), "count", m07)
    m07 = "m07/evidence/oracle.csv"
    row, = select(m07, oracle_control="unknown_range")
    emit("range_extinction_equal_transmission_residual",
         abs(float(row["observation"]) - float(row["alternate_observation"])), "dimensionless", m07)
    m07 = "m07/evidence/metrics.csv"
    for case, mode in (("matched", "joint"), ("mismatched_air", "joint"),
                       ("mismatched_air", "LWIR")):
        row, = select(m07, corpus="M07", case=case, mode=mode, split="holdout")
        emit(f"m07_{case}_{mode}_coverage", float(row["coverage95"]), "fraction", m07)
        if (case, mode) == ("mismatched_air", "joint"):
            emit("m07_wrong_air_warning_fraction", int(row["mismatch"]) / int(row["n"]),
                 "fraction", m07)
    emit("m07_severe_rgb_weak", value(m07, "weak", corpus="M07", case="severe",
                                     mode="RGB", split="holdout"), "count", m07)
    m07 = "m07/evidence/verification.log"
    grid_z, fine = regex_values(m07, r"grid z=([0-9.]+);[^\n]*residual=([0-9.]+) LSB")
    emit("m07_nominal_grid_warning", grid_z, "nominal sigma", m07)
    emit("m07_finer_search_residual", fine, "LSB", m07)
    m08 = "m08/evidence/verification.log"
    diff, = regex_values(m08, r"predictive mean difference=([-0-9.]+);")
    emit("equal_initial_burden_6s_predictive_mean_gap", abs(diff), "fraction", m08)
    m08 = "m08/evidence/scores.csv"
    for case, method, h in (("matched", "joint", "2"), ("air_mismatch", "joint", "0"),
                             ("dynamics_mismatch", "joint", "2"), ("matched", "slow_dynamics", "2"),
                             ("matched", "low_amplitude", "48"), ("matched", "persistence", "48")):
        emit(f"m08_{case}_{method}_{h}s_coverage",
             value(m08, "coverage95", case=case, method=method, horizon_s=h, target="burden"),
             "fraction", m08)
    m09 = "m09/evidence/scores.csv"
    for case, h in (("matched", "2"), ("dynamics_mismatch", "2"), ("air_mismatch", "0")):
        values = select(m09, family="learned", model="physics", size="768", case=case,
                        mode="joint", horizon_s=h, target="burden")
        assert len(values) == 3
        emit(f"m09_{case}_{h}s_coverage",
             statistics.fmean(float(r["coverage95"]) for r in values), "fraction", m09)
    m09 = "m09/evidence/ood.csv"
    for case in ("dynamics_mismatch", "air_mismatch"):
        row, = select(m09, size="768", seed="901", mode="joint", case=case)
        for metric in ("auc", "matched_flag_rate", "shift_flag_rate"):
            emit(f"m09_{case}_{metric}", float(row[metric]), "fraction", m09)
    m09 = "m09/evidence/paired.csv"
    row, = select(m09, comparison="physics_minus_classical", size="768", mode="joint",
                  horizon_s="future_mean")
    for metric in ("delta_crps", "lower95", "upper95"):
        emit(f"m09_vs_classical_{metric}", float(row[metric]), "burden CRPS", m09)


if __name__ == "__main__":
    audit()
