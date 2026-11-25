"""Measure warm single-issue parametric and sampled forecast latency."""
import argparse
import json
from pathlib import Path
import resource
import statistics
import sys
import time

import numpy as np

from assess import PROBS
from common import ROOT, CFG, ISSUE, parse, sigmoid, target_mapping, writer
from infer import issue, load

sys.path.append(str(ROOT/"m08"))
from predict import forecast


def run(raw, checkpoints, destination):
    case = next(r for r in parse(raw/"issues.csv",ISSUE)
                if r["case"]=="matched" and r["split"]=="holdout")
    model = load(checkpoints/"physics-768-901.json")

    def learned():
        values = issue(case,"joint",model)
        mean,scale = (np.array(json.loads(value)) for value in values[-2:])
        return target_mapping(sigmoid(mean[:,0,None]+scale[:,0,None]*PROBS))

    def classical():
        return target_mapping(np.array([json.loads(r[-1]) for r in forecast(case,"joint")]))

    with destination.open("w",newline="") as file:
        out=writer(file,("implementation","iterations","median_us","p95_us","cumulative_peak_rss_kib"))
        for name,fn in (("physics_mlp_samples",learned),("m08_joint_samples",classical)):
            assert fn().shape==(5,3,CFG["samples"])
            timings=[]
            for _ in range(60):
                start=time.perf_counter_ns()
                fn()
                timings.append((time.perf_counter_ns()-start)/1000)
            out.writerow((name,len(timings),statistics.median(timings),sorted(timings)[56],
                          resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw",type=Path)
    parser.add_argument("checkpoints",type=Path)
    parser.add_argument("output",type=Path)
    args=parser.parse_args()
    run(args.raw,args.checkpoints,args.output)
