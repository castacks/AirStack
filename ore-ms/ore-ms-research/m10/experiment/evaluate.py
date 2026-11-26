"""Fit task calibration and report paired holdout outcomes outside inference."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from model import ARMS, DECISION, FROZEN, HERE, OUTPUT, P, digest, dump, table

BASE = ("B01","B02","B03")
PRIMARY = ("absent","hidden")
LABELS = ("id","realization","case","projected","visible","fraction","x","y","feasible","negative")
BOUNDS = {"calibration":(128,256),"holdout":(256,512)}
LOCK_FILES = ("protocol.json","PROTOCOL.md","model.py","stages.py","isolate.py","evaluate.py","verify.py","run.sh")


def wilson(k,n):
    if not n:
        return None
    z = P["wilson_z"]
    p = k/n
    return (p+z*z/(2*n)+z*math.sqrt((p*(1-p)+z*z/(4*n))/n))/(1+z*z/n)


def ratio(n,d):
    return float(n/d) if d else None


def inputs(location, split):
    labels = table(location/"labels.csv",LABELS)
    lo,hi = BOUNDS[split]
    scenarios = {r["case"] for r in labels}
    pairs = {(int(r["realization"]),r["case"]) for r in labels}
    keys = {r["id"] for r in labels}
    if (pairs!={(i,case) for i in range(lo,hi) for case in scenarios} or
            len(scenarios)!=27 or len(labels)!=len(pairs) or len(keys)!=len(labels)):
        raise ValueError(f"incomplete {split} labels")
    result = {}
    for path,schema,arms in (("baselines.csv",FROZEN,BASE),("proposals.csv",OUTPUT,ARMS)):
        records = table(location/path,schema)
        for arm in arms:
            index = {r["id"]:r for r in records if r["arm"]==arm}
            if index.keys()!=keys:
                raise ValueError(f"unaligned {arm}")
            result[arm] = [index[r["id"]] for r in labels]
        if len(records)!=len(keys)*len(arms):
            raise ValueError("duplicate or unexpected prediction rows")
    return labels,result


def arrays(labels, preds):
    feasible = np.array([r["feasible"]=="1" for r in labels])
    candidate = np.array([bool(r["x"]) for r in preds])
    points = np.array([(float(r["x"]),float(r["y"])) if has else (np.nan,np.nan)
                       for r,has in zip(preds,candidate)])
    truth = np.array([(float(r["x"]),float(r["y"])) if f else (np.nan,np.nan)
                      for r,f in zip(labels,feasible)])
    error = np.hypot(*(points-truth).T)
    score = np.array([float(r["score"]) for r in preds])
    if not np.all((0<=score)&(score<=1)):
        raise ValueError("nonfinite or nonbounded candidate score")
    return {"candidate":candidate,"correct":error<=1,"error":error,"score":score,
            "reason":np.array([r["reason"] for r in preds])}


def bins(raw):
    return np.minimum((raw*P["bins"]).astype(int),P["bins"]-1)


def calibration_stats(labels, a):
    negative = np.array([r["negative"] for r in labels])
    supported = a["candidate"]
    correct = a["correct"]
    scores = a["score"]
    index = bins(scores)
    reliability = []
    for b in range(P["bins"]):
        cell = (index==b)&supported
        count = int(cell.sum())
        hits = int((correct&cell).sum())
        reliability.append({"n":count,"correct":hits,"p":(hits+1)/(count+2) if count else None})
    qualified = supported & np.array([reliability[i]["n"]>0 for i in index])
    sweep = []
    for limit in P["thresholds"]:
        issue = qualified & (scores>=limit)
        stats = {group:{"false":int((issue&(negative==group)).sum()),"n":int((negative==group).sum())}
                 for group in PRIMARY}
        for v in stats.values():
            v["wilson_upper95"] = wilson(v["false"],v["n"])
        good = all(v["wilson_upper95"] is not None and v["wilson_upper95"]<=P["safety_upper"]
                   for v in stats.values())
        sweep.append({"cutoff":limit,"correct":int((issue&correct).sum()),"issued":int(issue.sum()),
                      "safety":stats,"passes_safety":good})
    choices = [c for c in sweep if c["passes_safety"] and c["correct"]>0]
    chosen = max(choices,key=lambda c:(c["correct"],-sum(v["false"] for v in c["safety"].values()),
                                        c["cutoff"]),default=sweep[-1])
    return {"bins":reliability,"chosen":chosen,"sweep":sweep}


def calibrate(source, frozen_lock, model, lock):
    labels, preds = inputs(source,"calibration")
    historical = json.loads(frozen_lock.read_text())
    if (historical["development_comparator"]!="B03" or
            [historical["calibration"][arm]["chosen"]["cutoff"] for arm in BASE]!=[.2,.2,.3]):
        raise ValueError("frozen baseline changed")
    dump(lock,{"model_sha256":digest(model),"frozen_lock_sha256":digest(frozen_lock),
               "code_sha256":{f:digest(HERE/f) for f in LOCK_FILES},
               "calibration_input_sha256":{p.name:digest(p) for p in sorted(source.iterdir())},
               "historical":historical["calibration"],
               "new":{arm:calibration_stats(labels,arrays(labels,preds[arm])) for arm in ARMS}})


def validate_lock(lock, receipt, model):
    if digest(lock)!=receipt.read_text().strip():
        raise ValueError("calibration lock checksum mismatch")
    contents = json.loads(lock.read_text())
    if (contents["model_sha256"]!=digest(model) or set(contents["code_sha256"])!=set(LOCK_FILES) or
            any(digest(HERE/f)!=d for f,d in contents["code_sha256"].items())):
        raise ValueError("adaptive configuration changed after calibration")
    return contents


def deployment(model, lock, receipt, output):
    valid = validate_lock(lock,receipt,model)
    dump(output,{"model":json.loads(model.read_text()),"qualification":{
        arm:{"cutoff":c["chosen"]["cutoff"],"mapping":[b["p"] for b in c["bins"]]}
        for arm,c in valid["new"].items()}})


def decisions(a, calibrated, old):
    score_bins = bins(a["score"])
    mapping = calibrated["mapping"] if old else [b["p"] for b in calibrated["bins"]]
    a["p"] = np.array([np.nan if mapping[i] is None else mapping[i] for i in score_bins])
    a["p"][~a["candidate"]] = np.nan
    a["bin"] = score_bins
    a["issued"] = a["candidate"] & np.isfinite(a["p"]) & (a["score"]>=calibrated["chosen"]["cutoff"])


def summary(labels, a, where, available, reference):
    feasible, negative = labels
    issued, correct, candidate = (a[k] for k in ("issued","correct","candidate"))
    issue = where&issued
    hit = issue&correct
    positives = feasible&where
    known = where&candidate&np.isfinite(a["p"])
    n, k, f = int(where.sum()),int(issue.sum()),int(positives.sum())
    safety = {}
    for category in sorted(set(negative[where])-{"feasible"}):
        cell = where&(negative==category)
        count, false = int(cell.sum()),int((cell&issued).sum())
        safety[category] = {"false":false,"n":count,"rate":ratio(false,count),"wilson_upper95":wilson(false,count)}
    reliability = []
    for b in range(P["bins"]):
        mask = known&(a["bin"]==b)
        if not mask.any():
            continue
        reliability.append({"bin":b,"n":int(mask.sum()),"pred":float(a["p"][mask].mean()),
                            "correct":int(correct[mask].sum())})
    relevant = issue&feasible
    base_hits = reference["issued"]&reference["correct"]
    return {"n":n,"feasible":f,"issued":k,"abstained":n-k,"correct":int(hit.sum()),
            "candidate":int((where&candidate).sum()),"qualified_candidate":int(known.sum()),
            "coverage":ratio(k,n),"feasible_coverage":ratio(relevant.sum(),f),
            "correct_per_feasible":ratio(hit.sum(),f),"selective_risk":ratio(k-hit.sum(),k),
            "mean_error_px":float(a["error"][relevant].mean()) if relevant.any() else None,
            "negative":safety,"brier":float(np.mean((a["p"][known]-correct[known])**2)) if known.any() else None,
            "ece":ratio(sum(abs(b["n"]*b["pred"]-b["correct"]) for b in reliability),known.sum()),
            "reliability":reliability,"available_rgb":int((where&available[0]).sum()),
            "available_lwir":int((where&available[1]).sum()),
            "lost_B03_correct":int((positives&base_hits&~hit).sum()),
            "gained_over_B03":int((hit&~base_hits).sum()),
            "candidate_reasons":dict(Counter(a["reason"][where].tolist())),
            "issued_reasons":dict(Counter(a["reason"][issue].tolist()))}


def at_coverage(a,target,feasible,negative):
    eligible = a["candidate"] & np.isfinite(a["p"])
    n = len(eligible)
    desired = n*target
    if eligible.sum()<desired:
        return {"eligible":int(eligible.sum()),"requested":target,"supported":False}
    rank = np.sort(a["score"][eligible])[::-1]
    cutoff = rank[math.ceil(desired)-1]
    above = eligible&(a["score"]>cutoff)
    equal = eligible&(a["score"]==cutoff)
    mix = (desired-above.sum())/equal.sum()
    issue = above.astype(float)+mix*equal
    return {"eligible":int(eligible.sum()),"requested":target,"supported":True,
            "tie_fraction":float(mix),"cutoff_diagnostic_only":float(cutoff),
            "correct_per_feasible":ratio(issue@a["correct"],feasible.sum()),
            "selective_risk":ratio(desired-issue@a["correct"],desired),
            "primary_false_rate":{group:ratio(issue@(negative==group),(negative==group).sum())
                                  for group in PRIMARY}}


def paired_intervals(arms, labels, masks):
    realization = np.array([int(r["realization"])-256 for r in labels])
    feasible = np.array([r["feasible"]=="1" for r in labels])
    draws = np.random.default_rng(P["bootstrap_seed"]).integers(0,256,(P["bootstrap_draws"],256))
    weights = np.asarray([np.bincount(row,minlength=256) for row in draws],dtype=float)
    names = (*BASE,"B04","A01","A04","A05","A06","A07","A08","A09","O01","O01_B03_mask")
    results = {}
    for case, mask in masks.items():
        vars = [mask,mask&feasible]
        for name in names:
            a = arms[name]
            vars.extend((mask&a["issued"],mask&a["issued"]&a["correct"]))
        totals = np.stack([np.bincount(realization,weights=x,minlength=256) for x in vars],axis=-1)
        counts = weights@totals
        ci = lambda series:np.percentile(series,[2.5,97.5]).tolist()
        all_n = counts[:,0]
        feasible_n = counts[:,1]
        coverage = counts[:,2::2]/all_n[:,None]
        result = {name:{"coverage":ci(coverage[:,i]),
                        "coverage_gain_vs_B03":ci(coverage[:,i]-coverage[:,2])}
                  for i,name in enumerate(names)}
        if feasible_n.all():
            success = counts[:,3::2]/feasible_n[:,None]
            for i,name in enumerate(names):
                result[name]["correct_per_feasible"] = ci(success[:,i])
            result["A01_paired_contrasts"] = {name:ci(success[:,4]-success[:,i])
                                               for i,name in enumerate(names) if name in (*BASE,"B04")}
        else:
            result["A01_paired_contrasts"] = None
        results[case] = result
    return results


def dependence(labels, preds, decisions, mask, sensors):
    a,b = decisions["B01"],decisions["B02"]
    avail = mask & sensors[0] & sensors[1]
    feasible = np.array([r["feasible"]=="1" for r in labels])
    pair = avail & a["candidate"] & b["candidate"]
    joint = [[int((pair&(a["correct"]==i)&(b["correct"]==j)).sum()) for j in (0,1)] for i in (0,1)]
    joint_feasible = [[int((pair&feasible&(a["correct"]==i)&(b["correct"]==j)).sum())
                        for j in (0,1)] for i in (0,1)]
    indices = np.flatnonzero(pair)
    conflicts = np.zeros(len(labels),dtype=bool)
    for i in indices:
        p,q = preds["B01"][i],preds["B02"][i]
        conflicts[i] = math.hypot(float(p["x"])-float(q["x"]),float(p["y"])-float(q["y"]))>P["association_px"]
    warnings = np.array([p["warning"]=="1" for p in preds["A01"]])
    degradation = avail & feasible & (a["correct"]^b["correct"])
    return {"joint_correctness_candidate_pairs":joint,"joint_correctness_feasible_candidate_pairs":joint_feasible,
            "available_both":int(avail.sum()),"pair_candidates":int(pair.sum()),
            "rgb_candidate_only":int((avail&a["candidate"]&~b["candidate"]).sum()),
            "lwir_candidate_only":int((avail&~a["candidate"]&b["candidate"]).sum()),
            "neither_candidate":int((avail&~a["candidate"]&~b["candidate"]).sum()),
            "conflicts":int(conflicts.sum()),"warning_opportunities":int((avail&feasible).sum()),
            "degradation":int(degradation.sum()),"warnings":int((warnings&avail&feasible).sum()),
            "missed_degradation":int((degradation&~warnings).sum()),
            "false_warning":int((warnings&avail&feasible&~degradation).sum())}


def evaluate(source, model, lock, receipt, destination):
    frozen = validate_lock(lock,receipt,model)
    labels,preds = inputs(source,"holdout")
    a = {arm:arrays(labels,rows) for arm,rows in preds.items()}
    for arm,data in a.items():
        old = arm in BASE
        decisions(data,frozen["historical" if old else "new"][arm],old)
    issued_rows = table(source/"issued.csv",DECISION)
    if len(issued_rows)!=len(labels)*len(ARMS):
        raise ValueError("incomplete isolated decisions")
    for arm in ARMS:
        isolated = [r for r in issued_rows if r["arm"]==arm]
        if [r["id"] for r in isolated]!=[r["id"] for r in labels]:
            raise ValueError("unaligned isolated decisions")
        for i,r in enumerate(isolated):
            expected = a[arm]["issued"][i]
            if (r["status"]!=("issue" if expected else "abstain") or
                    (expected and (r["x"],r["y"])!=(preds[arm][i]["x"],preds[arm][i]["y"])) or
                    (not expected and (r["x"] or r["y"]))):
                raise ValueError("isolated decision disagrees with scoring reference")
    feasible = np.array([r["feasible"]=="1" for r in labels])
    negative = np.array([r["negative"] for r in labels])
    cases = np.array([r["case"] for r in labels])
    available = [np.array([r[f"available_{channel}"]=="1" for r in preds["B03"]])
                 for channel in ("rgb","lwir")]
    union = np.logical_or.reduce([a[arm]["candidate"] for arm in (*BASE,"B04","A01")])
    error = np.nanmin(np.stack([np.where(a[arm]["correct"],a[arm]["error"],np.inf)
                                for arm in (*BASE,"B04","A01")]),axis=0)
    oracle = np.isfinite(error)
    for arm,allowed in (("O01",np.ones(len(labels),bool)),("O01_B03_mask",a["B03"]["issued"])):
        shape = len(labels)
        a[arm] = {"candidate":union&allowed,"error":error,"correct":oracle,"issued":oracle&allowed,
                  "p":np.full(shape,np.nan),"bin":np.zeros(shape,dtype=int),
                  "reason":np.where(oracle&allowed,"privileged_correct","oracle_withheld")}
    masks = {"overall":np.ones(len(labels),bool),**{case:cases==case for case in sorted(set(cases))}}
    results = {arm:{case:summary((feasible,negative),data,mask,available,a["B03"])
                    for case,mask in masks.items()} for arm,data in a.items()}
    iid = paired_intervals(a,labels,masks)
    matched = {arm:[at_coverage(a[arm],q,feasible,negative) for q in P["coverage_slices"]]
               for arm in (*BASE,"B04","A01")}
    joint = {case:dependence(labels,preds,a,mask,available) for case,mask in masks.items()}
    destination.mkdir()
    dump(destination/"metrics.json",{"calibration_sha256":digest(lock),"realizations":256,"metrics":results,
         "intervals":iid,"matched_coverage":matched,"dependence":joint,
         "context_admission":{"A04":{"admitted":0,"n":len(labels),"reason":"unqualified_association_and_transfer"},
                              "A05":{"admitted":0,"n":len(labels),"reason":"missing_t0_measurements"},
                              "A06":{"admitted":0,"n":len(labels),"reason":"missing_t0_measurements"}}})


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command",required=True)
    for cmd,names in (("calibrate",("source","frozen_lock","model","lock")),
                      ("deployment",("model","lock","receipt","output")),
                      ("evaluate",("source","model","lock","receipt","destination"))):
        sub = commands.add_parser(cmd)
        for name in names:
            sub.add_argument(name,type=Path)
    args = vars(parser.parse_args())
    command = args.pop("command")
    {"calibrate":calibrate,"deployment":deployment,"evaluate":evaluate}[command](**args)
