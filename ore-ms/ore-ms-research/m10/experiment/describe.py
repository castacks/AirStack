"""Produce descriptive case tables and frozen-result sensitivity statistics."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from model import dump, table


def run(source, destination):
    report = json.loads((source/"results/metrics.json").read_text())
    labels = table(source/"holdout/labels.csv")
    proposals = {(r["id"],r["arm"]):r for r in table(source/"holdout/proposals.csv")}
    choices = {(r["id"],r["arm"]):r for r in table(source/"holdout/issued.csv")}
    baseline = {(r["id"],r["arm"]):r for r in table(source/"holdout/baselines.csv")}
    keys = sorted(report["metrics"]["A01"])
    groups = [int(row["realization"])-256 for row in labels]
    scenarios = np.array([row["case"] for row in labels])
    # Eligibility is shared. The oracle uses this column only as a sensor summary.
    eligible_sensor = np.array([any(baseline[row["id"],"B03"][f"available_{sensor}"]=="1"
                                    for sensor in ("rgb","lwir")) for row in labels])
    with (destination/"scenario_metrics.csv").open("w",newline="") as file:
        fields = ["arm","stratum","n","independent_ids","at_least_one_eligible","rgb_eligible","lwir_eligible",
                  "candidate","feasible","issued","correct","abstained","coverage","correct_per_feasible",
                  "correct_ci_low","correct_ci_high","paired_A01_minus_arm_ci_low","paired_A01_minus_arm_ci_high",
                  "negative_transfer_vs_B03","gain_count_vs_B03","false_supported","upper95_negative"]
        writer = csv.DictWriter(file,fieldnames=fields,lineterminator="\n")
        writer.writeheader()
        for arm,results in report["metrics"].items():
            for case in keys:
                s = results[case]
                combined = report["intervals"][case]
                ci = combined[arm].get("correct_per_feasible",[None,None])
                paired = (combined["A01_paired_contrasts"] or {}).get(arm,[None,None])
                eligible = eligible_sensor if case=="overall" else eligible_sensor&(scenarios==case)
                row = {k:s[k] for k in ("n","candidate","feasible","issued","correct","abstained","coverage",
                                      "correct_per_feasible")}
                row.update(arm=arm,stratum=case,independent_ids=256,at_least_one_eligible=int(eligible.sum()),
                           rgb_eligible=s["available_rgb"],lwir_eligible=s["available_lwir"],
                           correct_ci_low=ci[0],correct_ci_high=ci[1],paired_A01_minus_arm_ci_low=paired[0],
                           paired_A01_minus_arm_ci_high=paired[1],negative_transfer_vs_B03=s["lost_B03_correct"],
                           gain_count_vs_B03=s["gained_over_B03"],
                           false_supported=sum(n["false"] for n in s["negative"].values()),
                           upper95_negative=next(iter(s["negative"].values()))["wilson_upper95"]
                           if case!="overall" and s["negative"] else None)
                writer.writerow(row)

    def correct(row, truth):
        return (bool(row["x"]) and truth["feasible"]=="1" and
                sum((float(row[k])-float(truth[k]))**2 for k in ("x","y"))<=1)

    omit = scenarios!="registration_hidden"
    feasible = np.array([r["feasible"]=="1" for r in labels])
    a = np.array([correct(choices[t["id"],"A01"],t) for t in labels])
    b = np.array([correct(baseline[t["id"],"B03"],t) and float(baseline[t["id"],"B03"]["score"])>=.3
                  for t in labels])
    gain = np.bincount(groups,weights=(a.astype(int)-b)*omit,minlength=256)
    n = np.bincount(groups,weights=feasible&omit,minlength=256)
    draw = np.random.default_rng(10091).integers(0,256,(10000,256))
    interval = np.percentile(gain[draw].sum(axis=1)/n[draw].sum(axis=1),[2.5,97.5])
    ref = {r["id"] for r in labels if r["case"] in ("reference","distractor")}
    context_pairs = sum(p["reason"]!="no_candidate" for (key,arm),p in proposals.items()
                        if key in ref and arm=="A01")
    dump(destination/"sensitivity.json",{"label":"post hoc, descriptive, fixed decisions",
        "omit_registration_hidden":{"feasible":int(n.sum()),"delta_correct":int(gain.sum()),
                                    "gain":float(gain.sum()/n.sum()),"paired_interval95":interval.tolist()},
        "candidate_reference_and_matched_distractor_count":context_pairs})


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source",type=Path)
    parser.add_argument("destination",type=Path)
    args = parser.parse_args()
    run(args.source,args.destination)
