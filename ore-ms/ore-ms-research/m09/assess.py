"""Score frozen classical and learned marginals on the exact same realization keys."""
import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
from statistics import NormalDist

import numpy as np

from common import (CFG, HORIZONS, IDS, ISSUED, LABELS, optical_contract, parse,
                    sigmoid, target_mapping, writer)

TARGETS = LABELS[4:]
CLASSICAL = (*IDS,"method","horizon_s","status","flags","ess","burden_samples")
PROBS = np.array([NormalDist().inv_cdf((i+.5)/CFG["samples"]) for i in range(CFG["samples"])])
SCORE = ("family","model","size","seed","case","mode","horizon_s","target","n","issued",
         "failure_rate","fallback_rate","bias","mae","rmse","spread","crps","crps_low95",
         "crps_high95","coverage95","coverage_low95","coverage_high95","width95","events",
         "mean_probability","brier","brier_low95","brier_high95","auc")
GROUP = SCORE[:6]


def crps(samples, y):
    count=samples.shape[-1]
    weights=np.arange(1,count)*np.arange(count-1,0,-1)/count**2
    return np.mean(np.abs(samples-y[...,None]),axis=-1)-np.diff(samples,axis=-1) @ weights


def interval(values):
    mean=float(np.mean(values))
    width=1.96*float(np.std(values,ddof=1))/math.sqrt(len(values)) if len(values)>1 else 0
    return mean,mean-width,mean+width


def wilson(count,size):
    p=count/size
    z=1.96
    center=(p+z*z/(2*size))/(1+z*z/size)
    width=z*math.sqrt(p*(1-p)/size+z*z/(4*size**2))/(1+z*z/size)
    return max(0,center-width),min(1,center+width)


def auc(values,outcome):
    yes,no=values[outcome],values[~outcome]
    if not len(yes) or not len(no):
        return ""
    return float(np.mean((yes[:,None]>no).astype(float)+.5*(yes[:,None]==no)))


def learned_group(records, prior):
    ordered=sorted(records,key=lambda r:int(r["realization"]))
    active=[r for r in ordered if r["status"]!="abstain"]
    result=[]
    for row in active:
        if row["status"]=="prior_fallback":
            result.append(prior)
            continue
        mean=np.array(json.loads(row["location"]))
        scale=np.array(json.loads(row["scale"]))
        samples=sigmoid(mean[...,None]+scale[...,None]*PROBS)
        if samples.shape[-2]==1:
            samples=target_mapping(samples[:,0,:])
        result.append(np.sort(samples,axis=-1))
    return ordered,active,np.array(result)


def paired_comparisons(contributions, destination):
    cases = [
        ("physics_minus_ablation","physics","ablation"),
        ("physics_minus_data","physics","data"),
        ("physics_minus_classical","physics",None),
        ("ablation_minus_classical","ablation",None),
        ("data_minus_classical","data",None),
        *(("large_minus_small_"+key,key,key) for key in CFG["models"]),
    ]
    all_ids=range(*CFG["test_ids"])
    with destination.open("w",newline="") as file:
        output=writer(file,("comparison","size","mode","horizon_s","paired_realizations","delta_crps",
                            "lower95","upper95","seed_min","seed_max"))
        for label,left,right in cases:
            for size in CFG["train_sizes"]:
                if label.startswith("large_minus_small") and size==CFG["train_sizes"][0]:
                    continue
                for mode in CFG["modes"]:
                    measurements=[]
                    for seed in CFG["seeds"]:
                        first=contributions[("learned",left,str(size),str(seed),"matched",mode)]
                        if right is None:
                            name="joint" if mode=="joint" else mode
                            second=contributions[("classical",name,"0","0","matched",name)]
                        else:
                            rhs_size=CFG["train_sizes"][0] if label.startswith("large_minus_small") else size
                            second=contributions[("learned",right,str(rhs_size),str(seed),"matched",mode)]
                        measurements.append(np.array([first[i]-second[i] for i in all_ids]))
                    data=np.array(measurements)
                    for j,h in enumerate([*HORIZONS,"future_mean"]):
                        selected=data[:,:,j] if h!="future_mean" else data[:,:,1:].mean(axis=-1)
                        points=selected.mean(axis=0)
                        means=selected.mean(axis=1)
                        output.writerow((label,size,mode,h,len(points),*interval(points),
                                         float(means.min()),float(means.max())))


def populations(learned, classical, prior):
    for group,records in sorted(learned.items()):
        yield group,*learned_group(records,prior)
    for group,records in sorted(classical.items()):
        ordered=[records[i][0] for i in sorted(records)]
        active=[r for r in ordered if r["burden_samples"]]
        runs=[[json.loads(records[int(r["realization"])][h]["burden_samples"]) for h in HORIZONS]
              for r in active]
        # Preserve the archived scalar transfer arithmetic for zero-width
        # oracle intervals, where a single ULP can change coverage.
        length, optics = optical_contract()
        def bands(values):
            return (values,
                    [math.exp(-length*(optics["rgb_floor"]+optics["rgb_span"]*b)) for b in values],
                    [math.exp(-length*(optics["lwir_floor"][1]+optics["lwir_span"][1]*b)) for b in values])
        samples=np.sort(np.array([[bands(values) for values in trial] for trial in runs]),
                        axis=-1) if runs else np.array([])
        yield group,ordered,active,samples


def evaluate(issued, raw, prior_file, destination):
    destination.mkdir()
    truths=defaultdict(dict)
    for row in parse(raw/"truth.csv",LABELS):
        if row["split"]=="holdout":
            ident=(row["case"],int(row["realization"]))
            truths[ident][int(row["horizon_s"])]=[float(row[field]) for field in TARGETS]
    groups=defaultdict(list)
    for row in parse(issued,ISSUED):
        groups[("learned",*(row[k] for k in ("model","size","seed","case","mode")))].append(row)
    classic=defaultdict(lambda: defaultdict(dict))
    for file in ("forecasts.csv","oracle_forecasts.csv"):
        for row in parse(raw/file,CLASSICAL):
            if row["split"]=="holdout":
                ident=("classical",row["method"],"0","0",row["case"],row["method"])
                classic[ident][int(row["realization"])][int(row["horizon_s"])]=row
    prior=np.sort(target_mapping(np.array(json.loads(prior_file.read_text()))),axis=-1)
    contributions={}
    with ((destination/"scores.csv").open("w",newline="") as report,
          (destination/"reliability.csv").open("w",newline="") as calibration):
        scores=writer(report,SCORE)
        bins=writer(calibration,(*GROUP,"horizon_s","target","bin","n","mean_p","events",
                                 "event_rate","low95","high95"))
        for key,ordered,active,samples in populations(groups,classic,prior):
            if len({r["realization"] for r in ordered})!=len(ordered):
                raise ValueError("duplicate forecast realization")
            case=key[4]
            n=len(ordered)
            valid=len(active)
            fallback=sum(r["status"]=="prior_fallback" for r in ordered)/n
            if valid:
                y=np.array([[truths[case,int(r["realization"])][h] for h in HORIZONS] for r in active])
                error=samples.mean(axis=-1)-y
                proper=crps(samples,y)
                bounds=np.quantile(samples,[.025,.975],axis=-1)
                included=(y>=bounds[0]) & (y<=bounds[1])
                width=bounds[1]-bounds[0]
                spread=samples.std(axis=-1)
                contributions[key]={int(r["realization"]):v[:,0] for r,v in zip(active,proper,strict=True)}
            for j,h in enumerate(HORIZONS):
                for k,target in enumerate(TARGETS):
                    prefix=(*key,h,target,n,valid,1-valid/n,fallback)
                    if not valid:
                        scores.writerow((*prefix,*[""]*(len(SCORE)-len(prefix))))
                        continue
                    yerr=error[:,j,k]
                    coverage=int(included[:,j,k].sum())
                    event_stats=("","","","","","")
                    if k:
                        threshold=CFG["thresholds"][k-1]
                        probability=(samples[:,j,k]<threshold).mean(axis=-1)
                        events=y[:,j,k]<threshold
                        event_stats=(int(events.sum()),float(probability.mean()),
                                     *interval(np.square(probability-events)),auc(probability,events))
                        indexes=np.minimum(4,np.floor(probability*5).astype(int))
                        for b in range(5):
                            selected=indexes==b
                            count=int(selected.sum())
                            hits=int(events[selected].sum())
                            tail=(float(probability[selected].mean()),hits,hits/count,*wilson(hits,count)
                                  ) if count else ("",0,"","","")
                            bins.writerow((*key,h,target,b,count,*tail))
                    scores.writerow((*prefix,float(yerr.mean()),float(np.abs(yerr).mean()),
                        float(np.sqrt(np.mean(yerr**2))),float(spread[:,j,k].mean()),
                        *interval(proper[:,j,k]),coverage/valid,*wilson(coverage,valid),
                        float(width[:,j,k].mean()),*event_stats))
    paired_comparisons(contributions,destination/"paired.csv")
    with (destination/"ood.csv").open("w",newline="") as file:
        output=writer(file,("size","seed","mode","case","matched_n","shift_n","matched_flag_rate",
                            "shift_flag_rate","auc"))
        for key, group in sorted(groups.items()):
            if key[1]!="physics" or key[4] not in ("dynamics_mismatch","air_mismatch"):
                continue
            base=groups[key[:4]+("matched",key[5])]
            x=np.array([float(r["novelty"]) for r in base+group])
            result=auc(x,np.array([False]*len(base)+[True]*len(group)))
            output.writerow((key[2],key[3],key[5],key[4],len(base),len(group),
                             np.mean([int(r["novel"]) for r in base]),
                             np.mean([int(r["novel"]) for r in group]),result))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("issued",type=Path)
    parser.add_argument("m08_raw",type=Path)
    parser.add_argument("prior",type=Path)
    parser.add_argument("out",type=Path)
    args=parser.parse_args()
    evaluate(args.issued,args.m08_raw,args.prior,args.out)
