"""Generate controlled detector inputs and separately export truth for scoring."""
import argparse
import csv
import json
from pathlib import Path
import random

from features import FEATURES, KEYS, m06_keys, project_m06
from forward import AnalogNoise, CASES, HERE, code_observation, context


def controlled(writers, private):
    features, labels = writers
    parameters = private["controlled"]
    truth_rng = random.Random(parameters["truth_seed"])
    statistical_state = [truth_rng.random() for _ in range(parameters["realizations"])]
    for number, uniform in enumerate(statistical_state):
        split = "development" if number < parameters["development"] else "holdout"
        seed = parameters["noise_seed"] + 2 * number
        for name, case in CASES.items():
            if name.startswith("ensemble_"):
                continue
            hidden = private["cases"].get(name, {})
            physical = tuple(hidden.get(f"actual_{key}", value) for key,value in
                             zip(("material","length_m","air_K"), context(case), strict=True))
            burden = hidden.get("burden", uniform)
            codes = code_observation(case, physical, burden, AnalogNoise(seed), random.Random(seed + 1))
            if hidden.get("drop_rgb"):
                codes[:3] = [""] * 3
            if hidden.get("invalidate_lwir"):
                codes[-1] = -1
            key = ("M07", name, number, split, 0)
            features.writerow((*key, *codes))
            labels.writerow((*key, burden))


def m06(writers, source):
    features, labels = writers
    with (source/"frames.csv").open(newline="") as file:
        frames = {(r["case"],r["realization"],r["time_s"]):r for r in csv.DictReader(file)
                  if r["case"].startswith("ensemble_")}
    with (source/"pairs.csv").open(newline="") as file:
        for pair in csv.DictReader(file):
            key = tuple(pair[k] for k in ("case","realization","time_s"))
            if key not in frames:
                continue
            frame = frames[key]
            features.writerow(project_m06(pair, frame))
            labels.writerow((*m06_keys(pair, frame), pair["burden_avg"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("m06_data", type=Path)
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    private = json.loads((HERE/"generation.json").read_text())
    args.out.mkdir()
    with ((args.out/"features.csv").open("w", newline="") as observations,
          (args.out/"labels.csv").open("w", newline="") as truth):
        writers = tuple(csv.writer(f, lineterminator="\n") for f in (observations,truth))
        writers[0].writerow(FEATURES)
        writers[1].writerow((*KEYS,"burden"))
        controlled(writers, private)
        m06(writers, args.m06_data)


if __name__ == "__main__":
    main()
