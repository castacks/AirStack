"""Quantify the teacher's Gaussian moment projection on frozen M08 samples."""
import argparse
import json
from pathlib import Path

import numpy as np

from assess import CLASSICAL, PROBS
from common import HORIZONS, logit, parse, sigmoid, writer


def run(raw, output):
    deviations={h:[] for h in HORIZONS}
    for row in parse(raw/"forecasts.csv",CLASSICAL):
        if (row["case"],row["split"],row["method"])!=("matched","holdout","joint"):
            continue
        samples=np.array(json.loads(row["burden_samples"]))
        transforms=logit(samples)
        fitted=sigmoid(transforms.mean()+transforms.std()*PROBS)
        deviations[int(row["horizon_s"])].append(float(np.abs(fitted-samples).mean()))
    with output.open("w",newline="") as file:
        rows=writer(file,("horizon_s","realizations","mean_abs_quantile_difference",
                          "max_realization_difference"))
        for h,values in deviations.items():
            rows.writerow((h,len(values),float(np.mean(values)),max(values)))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    run(args.raw,args.output)
