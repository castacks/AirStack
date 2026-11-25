"""Issue conditional marginal parameters from admissible detector input only."""
import argparse
from functools import lru_cache
import json
from pathlib import Path
import resource
import statistics
import time

import numpy as np

from common import (CFG, IDS, ISSUE, ISSUED, THREE_MODE_CASES, dump, features, parse, writer)
from network import predict

ABSTAIN = {"range_unknown","material_unknown","domain_unknown","process_unknown"}
FALLBACK = {"occluded","weak"}


def load(path):
    checkpoint = json.loads(path.read_text())
    if checkpoint["model"] not in CFG["models"]:
        raise ValueError("unknown model")
    dimensions = 3 if checkpoint["model"]=="data" else 1
    count = 2*len(CFG["horizons_s"])*dimensions
    shape = [(8,CFG["hidden"]),(CFG["hidden"],),(CFG["hidden"],count),(count,)]
    params = [np.array(p,dtype=float) for p in checkpoint["parameters"]]
    if checkpoint["dimensions"]!=dimensions or [p.shape for p in params]!=shape:
        raise ValueError("invalid checkpoint architecture")
    if not all(np.isfinite(p).all() for p in params):
        raise ValueError("nonfinite checkpoint")
    checkpoint["parameters"] = params
    return checkpoint


@lru_cache(maxsize=240)
def precision(covariance, active):
    return np.linalg.inv(np.array(covariance)[np.ix_(active,active)])


def issue(row, mode, checkpoint):
    case = row["case"]
    if case in ABSTAIN:
        return ("abstain",case,"","","","")
    if case in FALLBACK:
        return ("prior_fallback",case,"","","","")
    scaling = checkpoint["feature_scaling"]
    x = features(row,mode,scaling)
    active = tuple(i for i in range(4) if x[4+i])
    if not active:
        return ("prior_fallback","no_admissible_codes","","","","")
    mask = str(sum(1<<i for i in active))
    inverse = precision(tuple(tuple(v) for v in scaling["covariance"]),active)
    values = x[list(active)]
    novelty = float(values @ inverse @ values)
    flag = int(novelty>scaling["thresholds"][mask])
    _,location,scale,_ = predict(checkpoint["parameters"],x[None,:],checkpoint["dimensions"])
    flags = "thermal_unconstrained" if case=="thermal_unknown" else ""
    return ("issued",flags,novelty,flag,
            json.dumps(location[0].tolist(),separators=(",",":")),
            json.dumps(scale[0].tolist(),separators=(",",":")))


def run(input_file, folder, destination, timing=None):
    records = [row for row in parse(input_file,ISSUE) if row["split"]=="holdout"]
    measured = []
    with destination.open("w",newline="") as source:
        output = writer(source,ISSUED)
        for path in sorted(folder.glob("*.json")):
            model = load(path)
            for row in records:
                modes = (CFG["modes"] if row["case"] in THREE_MODE_CASES else
                         ["LWIR"] if row["case"]=="weak" else ["joint"])
                for mode in modes:
                    output.writerow((*(row[key] for key in IDS),model["model"],model["size"],
                                     model["seed"],mode,*issue(row,mode,model)))
            if timing is not None:
                probe = next(r for r in records if r["case"]=="matched")
                issue(probe,"joint",model)
                times = []
                for _ in range(300):
                    start = time.perf_counter_ns()
                    issue(probe,"joint",model)
                    times.append(time.perf_counter_ns()-start)
                measured.append((path.stem,statistics.median(times)/1000,
                                  sorted(times)[284]/1000,path.stat().st_size,
                                  resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
    if timing is not None:
        with timing.open("w",newline="") as source:
            writer(source,("checkpoint","median_us","p95_us","checkpoint_bytes","peak_rss_kib")
                   ).writerows(measured)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("issues",type=Path)
    parser.add_argument("checkpoints",type=Path)
    parser.add_argument("issued",type=Path)
    parser.add_argument("--timing",type=Path)
    args=parser.parse_args()
    run(args.issues,args.checkpoints,args.issued,args.timing)
