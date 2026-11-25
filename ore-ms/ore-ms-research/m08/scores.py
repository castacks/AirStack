"""Proper scores and uncertainty, evaluated within horizons over whole realizations."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics

from contract import FORECASTS, IDS, LABELS, TARGETS, output, rows
from dynamics import CONFIG, targets

SCORES = ("case","method","horizon_s","target","n","issued","bias","mae","rmse","mean_spread",
          "coverage95","coverage_low","coverage_high","width95","crps","crps_low","crps_high",
          "categorical_log_score","event_count","mean_probability","brier","brier_low",
          "brier_high","auc")


def interval(values):
    n=len(values)
    mean=statistics.fmean(values)
    delta=1.96*statistics.stdev(values)/math.sqrt(n) if n>1 else 0
    return mean,mean-delta,mean+delta


def wilson(successes,n):
    p=successes/n
    z=1.96
    center=(p+z*z/(2*n))/(1+z*z/n)
    half=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/(1+z*z/n)
    return max(0,center-half),min(1,center+half)


def quantile(values,p):
    i=p*(len(values)-1)
    lo=int(i)
    fraction=i-lo
    return values[lo]+(values[min(lo+1,len(values)-1)]-values[lo])*fraction


def crps(values,truth):
    n=len(values)
    separation = math.fsum(i*(n-i)*(right-left)
                           for i,(left,right) in enumerate(zip(values,values[1:]),1))/n**2
    return statistics.fmean(abs(x-truth) for x in values)-separation


def categorical_score(values,truth):
    n=CONFIG["log_bins"]
    prior=CONFIG["log_pseudocount"]
    bucket=min(n-1,int(n*truth))
    count=sum(min(n-1,int(n*x))==bucket for x in values)
    return -math.log((count+prior)/(len(values)+prior*n))


def auc(events,probs):
    yes=[x for actual,x in zip(events,probs,strict=True) if actual]
    no=[x for actual,x in zip(events,probs,strict=True) if not actual]
    if not yes or not no:
        return ""
    return sum((a>b)+.5*(a==b) for a in yes for b in no)/(len(yes)*len(no))


def evaluate(folder):
    truth={tuple(r[k] for k in (*IDS,"horizon_s")):r for r in rows(folder/"truth.csv",LABELS)}
    grouped=defaultdict(list)
    for source in ("forecasts.csv","oracle_forecasts.csv"):
        for row in rows(folder/source,FORECASTS):
            if row["split"] != "holdout":
                continue
            observed=truth[tuple(row[k] for k in (*IDS,"horizon_s"))]
            grouped[row["case"],row["method"],int(row["horizon_s"])].append((row,observed))
    contributions={}
    with ((folder/"scores.csv").open("w",newline="") as summary,
          (folder/"reliability.csv").open("w",newline="") as reliability):
        scores=output(summary,SCORES)
        bins=output(reliability,("case","method","horizon_s","target","bin","n","mean_p",
                                 "events","event_rate","wilson_low","wilson_high"))
        for (case,method,h), records in sorted(grouped.items()):
            hits=[(r,t,list(zip(*(targets(b) for b in json.loads(r["burden_samples"])))))
                  for r,t in records if r["burden_samples"]]
            for j,target in enumerate(TARGETS):
                prefix=(case,method,h,target,len(records),len(hits))
                if not hits:
                    scores.writerow((*prefix,*[""]*(len(SCORES)-len(prefix))))
                    continue
                errors, spread, widths, ranked, category = [],[],[],[],[]
                probabilities, events = [],[]
                coverage=0
                threshold = CONFIG["rgb_event_below"] if j==1 else CONFIG["lwir_event_below"]
                for row,label,columns in hits:
                    sample=sorted(columns[j])
                    actual=float(label[target])
                    errors.append(statistics.fmean(sample)-actual)
                    spread.append(statistics.pstdev(sample))
                    lo,hi=(quantile(sample,p) for p in CONFIG["interval"])
                    coverage += lo<=actual<=hi
                    widths.append(hi-lo)
                    value=crps(sample,actual)
                    ranked.append(value)
                    if j==0:
                        category.append(categorical_score(sample,actual))
                        contributions[case,method,h,row["realization"]]=value
                    else:
                        probabilities.append(statistics.fmean(x<threshold for x in sample))
                        events.append(int(actual<threshold))
                brier, events_count, prob, discrimination = ("","",""),"","",""
                if probabilities:
                    prob=statistics.fmean(probabilities)
                    events_count=sum(events)
                    brier=interval([(p-y)**2 for p,y in zip(probabilities,events,strict=True)])
                    discrimination=auc(events,probabilities)
                    bucket_count=CONFIG["reliability_bins"]
                    for i in range(bucket_count):
                        member=[(p,y) for p,y in zip(probabilities,events,strict=True)
                                if min(bucket_count-1,int(p*bucket_count))==i]
                        count=len(member)
                        if count:
                            positives=sum(y for _,y in member)
                            bins.writerow((case,method,h,target,i,count,statistics.fmean(p for p,_ in member),
                                           positives,positives/count,*wilson(positives,count)))
                        else:
                            bins.writerow((case,method,h,target,i,0,"",0,"","",""))
                n=len(hits)
                scores.writerow((*prefix,statistics.fmean(errors),
                                 statistics.fmean(map(abs,errors)),
                                 math.sqrt(statistics.fmean(x*x for x in errors)),statistics.fmean(spread),
                                 coverage/n,*wilson(coverage,n),statistics.fmean(widths),
                                 *interval(ranked),statistics.fmean(category) if category else "",
                                 events_count,prob,*brier,discrimination))
    with (folder/"paired.csv").open("w",newline="") as file:
        writer=output(file,("reference","horizon_s","paired_realizations","joint_minus_reference_crps",
                            "lower95","upper95"))
        methods=CONFIG["cases"]["matched"]["methods"]
        for method in methods:
            if method=="joint":
                continue
            pooled=defaultdict(list)
            for h in CONFIG["horizons_s"]:
                diff={}
                for ident in range(CONFIG["development"],CONFIG["realizations"]):
                    a=contributions["matched","joint",h,str(ident)]
                    b=contributions["matched",method,h,str(ident)]
                    diff[ident]=a-b
                    pooled[ident].append(a-b)
                writer.writerow((method,h,len(diff),*interval(list(diff.values()))))
            # A single averaged score contribution per trajectory, with all
            # horizons of that trajectory kept together.
            writer.writerow((method,"all",len(pooled),
                             *interval([statistics.fmean(v) for v in pooled.values()])))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder",type=Path)
    evaluate(parser.parse_args().folder)
