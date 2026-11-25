"""Separate privileged issue-state reference. No future labels are inputs."""
import argparse
import json
from pathlib import Path
import random

from contract import FORECASTS, IDS, ORACLE, output, rows
from dynamics import CONFIG, HORIZONS, evolve
from predict import LEVELS


def run(source,destination):
    with destination.open("w",newline="") as file:
        writer=output(file,FORECASTS)
        for row in rows(source,ORACLE):
            state = json.loads(row["coefficients"])
            rng = random.Random(CONFIG["oracle_seed"]+int(row["realization"]))
            trials = [evolve(state,rng,CONFIG["tau_s"],CONFIG["sigma"])
                      for _ in range(CONFIG["oracle_size"])]
            for h, column in zip(HORIZONS,zip(*trials),strict=True):
                ordered = sorted(column)
                samples = [ordered[min(len(ordered)-1,int(level*len(ordered)))]
                           for level in LEVELS]
                writer.writerow((*(row[k] for k in IDS),"oracle",h,"oracle_only",
                                 "privileged_issue_state","",json.dumps(samples,separators=(",",":"))))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("issue_truth",type=Path)
    parser.add_argument("out",type=Path)
    args=parser.parse_args()
    run(args.issue_truth,args.out)
