"""Calibration receipts, frozen decisions and realization-clustered scoring."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from baseline import ARMS, CONFIG

LIMIT = CONFIG["risk_limit"]
PRIMARY = ("absent", "hidden")


def rows(path):
    with path.open() as file:
        return list(csv.DictReader(file))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+"\n")


def ratio(x, y):
    return x/y if y else None


def wilson(k, n):
    if not n:
        return None
    z = CONFIG["wilson_z"]
    p = k/n
    return (p+z*z/(2*n)+z*math.sqrt(p*(1-p)/n+z*z/(4*n*n)))/(1+z*z/n)


def partition(labels, predictions, destination):
    """Write physically separate inputs. Calibration itself sees only two splits."""
    destination.mkdir()
    truth = rows(labels)
    by_id = {r["id"]: name for name, (lo, hi) in CONFIG["split_bounds"].items()
             for r in truth if lo <= int(r["realization"]) < hi}
    for file, data in (("labels", truth), ("predictions", rows(predictions))):
        for split in CONFIG["split_bounds"]:
            with (destination/f"{split}_{file}.csv").open("w", newline="") as out:
                writer = csv.DictWriter(out, fieldnames=data[0], lineterminator="\n")
                writer.writeheader()
                writer.writerows(r for r in data if by_id[r["id"]] == split)


def join(location, split):
    labels = {r["id"]: r for r in rows(location/f"{split}_labels.csv")}
    grouped = defaultdict(list)
    seen = set()
    lo, hi = CONFIG["split_bounds"][split]
    for p in rows(location/f"{split}_predictions.csv"):
        key = (p["id"], p["arm"])
        if key in seen or p["arm"] not in ARMS:
            raise ValueError(f"invalid join key {key}")
        seen.add(key)
        truth = labels[p["id"]]
        if not lo <= int(truth["realization"]) < hi:
            raise ValueError("cross-partition join")
        feasible = truth["feasible"] == "1"
        candidate = bool(p["x"])
        err = math.hypot(float(p["x"])-float(truth["x"]), float(p["y"])-float(truth["y"])) if feasible and candidate else None
        grouped[p["arm"]].append({**truth, **p, "feasible": feasible, "candidate": candidate,
                                  "error": err, "correct_candidate": err is not None and err <= 1,
                                  "score": float(p["score"])})
    if len(seen) != len(labels)*len(ARMS) or len(labels) != (hi-lo)*len(CONFIG["cases"]):
        raise ValueError("incomplete partition")
    return grouped


def summarize(data, cutoff, mapping=None):
    feasible = sum(r["feasible"] for r in data)
    issued = [r for r in data if r["candidate"] and r["score"] >= cutoff]
    correct = sum(r["correct_candidate"] for r in issued)
    errors = [r["error"] for r in issued if r["error"] is not None]
    negatives = {}
    for kind in sorted({r["negative"] for r in data} - {"feasible"}):
        n = sum(r["negative"] == kind for r in data)
        k = sum(r["negative"] == kind for r in issued)
        negatives[kind] = {"false_supported": k, "n": n, "rate": k/n, "upper95": wilson(k,n)}
    candidates = [r for r in data if r["candidate"]]
    reliability = []
    if mapping is not None:
        for i, p in enumerate(mapping):
            events = [r["correct_candidate"] for r in candidates if index(r["score"]) == i]
            reliability.append({"n": len(events), "correct": sum(events), "p": p})
        loss = sum(r["correct"]*(1-r["p"])**2+(r["n"]-r["correct"])*r["p"]**2 for r in reliability)
    else:
        loss = 0
    return {"n": len(data), "feasible": feasible, "correct": correct, "issued": len(issued),
            "abstained": len(data)-len(issued), "available_rgb": sum(int(r["available_rgb"]) for r in data),
            "available_lwir": sum(int(r["available_lwir"]) for r in data),
            "negative": negatives, "coverage": ratio(len(issued), len(data)),
            "selective_risk": ratio(len(issued)-correct,len(issued)),
            "feasible_coverage": ratio(len(errors),feasible), "mean_error_px": ratio(sum(errors),len(errors)),
            "reasons": dict(Counter(r["reason"] for r in data)),
            "issued_reasons": dict(Counter(r["reason"] for r in issued)),
            "candidates": len(candidates), "reliability": reliability,
            "brier_candidate": ratio(loss,len(candidates)) if mapping is not None else None}


def index(score):
    n = CONFIG["score_bins"]
    return min(int(score*n), n-1)


def select(data):
    sweep = []
    for cutoff in CONFIG["thresholds"]:
        stats = summarize(data, cutoff)
        upper = {p: stats["negative"][p]["upper95"] for p in PRIMARY}
        qualifies = all(value <= LIMIT for value in upper.values())
        sweep.append({"cutoff": cutoff, "correct": stats["correct"], "issued": stats["issued"],
                      "primary_false": sum(stats["negative"][p]["false_supported"] for p in PRIMARY),
                      "upper95": upper, "qualifies": qualifies})
    usable = [r for r in sweep if r["qualifies"] and r["correct"]]
    selected = max(usable, key=lambda r: (r["correct"],-r["primary_false"],r["cutoff"])) if usable else sweep[-1]
    return {"chosen": selected, "sweep": sweep}


def calibrate(location, output):
    dev = join(location, "development")
    cal = join(location, "calibration")
    rank = {arm: select(dev[arm]) for arm in ARMS}
    strongest = max(ARMS, key=lambda arm: (rank[arm]["chosen"]["correct"], -rank[arm]["chosen"]["primary_false"]))
    result = {"development_comparator": strongest, "development": rank, "calibration": {},
              "input_sha256": {f"{s}_{kind}.csv":digest(location/f"{s}_{kind}.csv")
                               for s in ("development","calibration") for kind in ("labels","predictions")},
              "config_sha256": digest(Path(__file__).with_name("config.json")),
              "method_sha256": {file:digest(Path(__file__).with_name(file))
                                for file in ("baseline.py","score.py")}}
    for arm in ARMS:
        data = cal[arm]
        method = select(data)
        bins = [[] for _ in range(CONFIG["score_bins"])]
        for r in data:
            if r["candidate"]:
                bins[index(r["score"])].append(r["correct_candidate"])
        method["mapping"] = [(sum(b)+1)/(len(b)+2) for b in bins]
        method["bins"] = [{"n":len(b),"correct":sum(b)} for b in bins]
        result["calibration"][arm] = method
    save(output, result)


def dependence(rgb, lwir):
    table = np.zeros((2,2), dtype=int)
    for a,b in zip(rgb,lwir,strict=True):
        if a["id"] != b["id"]:
            raise ValueError("unaligned dependence comparison")
        if a["feasible"] and int(a["available_rgb"]) and int(b["available_lwir"]):
            table[int(a["correct_candidate"]),int(b["correct_candidate"])] += 1
    a,b,c,d = table.ravel().tolist()
    return {"table":table.tolist(), "n":int(table.sum()),
            "phi":ratio(a*d-b*c, math.sqrt((a+b)*(c+d)*(a+c)*(b+d)))}


def bootstrap(grouped, locks, strongest):
    lo, hi = CONFIG["split_bounds"]["holdout"]
    samples = np.random.default_rng(CONFIG["bootstrap_seed"]).integers(
        0, hi-lo, (CONFIG["bootstrap_draws"], hi-lo))
    cases = CONFIG["cases"]
    totals = np.zeros((len(cases),len(ARMS),hi-lo,2),dtype=int)
    case_index = {name: i for i,name in enumerate(cases)}
    for i,arm in enumerate(ARMS):
        cutoff = locks[arm]["chosen"]["cutoff"]
        for r in grouped[arm]:
            issue = r["candidate"] and r["score"] >= cutoff
            totals[case_index[r["case"]],i,int(r["realization"])-lo] += (
                r["feasible"],issue and r["correct_candidate"])

    def intervals(values):
        if not values[0,:,0].sum():
            return None
        performance = np.array([correct[samples].sum(axis=1)/feasible[samples].sum(axis=1)
                                for feasible,correct in values.transpose(0,2,1)])
        comparator = ARMS.index(strongest)
        return {arm: {"correct_per_feasible_ci95":np.percentile(performance[i],[2.5,97.5]).tolist(),
                      "gain_vs_development_comparator_ci95":
                          np.percentile(performance[i]-performance[comparator],[2.5,97.5]).tolist()}
                for i,arm in enumerate(ARMS)}

    return intervals(totals.sum(axis=0)), {case:intervals(totals[i]) for i,case in enumerate(cases)}


def evaluate(location, lock, receipt, out):
    if digest(lock) != receipt.read_text().strip():
        raise ValueError("calibration lock checksum mismatch")
    chosen = json.loads(lock.read_text())
    for file, expected in chosen["method_sha256"].items():
        if digest(Path(__file__).with_name(file)) != expected:
            raise ValueError(f"calibration method changed: {file}")
    grouped = join(location, "holdout")
    arms = {}
    for arm, data in grouped.items():
        method = chosen["calibration"][arm]
        cutoff = method["chosen"]["cutoff"]
        all_stats = summarize(data, cutoff, method["mapping"])
        qualifies = (method["chosen"]["qualifies"] and method["chosen"]["correct"] > 0
                     and all(all_stats["negative"][p]["upper95"] <= LIMIT for p in PRIMARY)
                     and all_stats["correct"] > 0)
        arms[arm] = {"cutoff":cutoff,"operating_point_qualified":qualifies,"overall":all_stats,
                     "cases":{case:summarize([r for r in data if r["case"] == case],cutoff,method["mapping"])
                              for case in CONFIG["cases"]}}
    correlations = {"overall":dependence(grouped["B01"],grouped["B02"])}
    for case in CONFIG["cases"]:
        correlations[case] = dependence(*[[r for r in grouped[arm] if r["case"] == case]
                                            for arm in ("B01","B02")])
    overall_ci, case_ci = bootstrap(grouped,chosen["calibration"],chosen["development_comparator"])
    result = {"calibration_sha256":digest(lock), "arms":arms, "conditional_dependence":correlations,
              "bootstrap":overall_ci,"stratum_bootstrap":case_ci,
              "development_comparator":chosen["development_comparator"]}
    save(out,result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name, fields in (("partition",("labels","predictions","destination")),
                         ("calibrate",("location","output")),("evaluate",("location","lock","receipt","out"))):
        sub = commands.add_parser(name)
        for field in fields:
            sub.add_argument(field,type=Path)
    args = vars(parser.parse_args())
    command = args.pop("command")
    {"partition":partition,"calibrate":calibrate,"evaluate":evaluate}[command](**args)
