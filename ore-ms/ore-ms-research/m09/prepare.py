"""Generate independent training/validation and admissible physics teachers."""
import argparse
import json
from pathlib import Path
import random
import sys

import numpy as np

from common import CFG, ISSUE, LABELS, TEACHER, ROOT, dump, logit, writer

sys.path.append(str(ROOT/"m08"))
from dynamics import CONFIG, MODEL, burden, evolve, stationary, targets
from predict import forecast
from forward import AnalogNoise, CASES, code_observation, context


def prepare(directory):
    directory.mkdir()
    settings = CASES["matched"]
    with ((directory/"issues.csv").open("w",newline="") as issues,
          (directory/"labels.csv").open("w",newline="") as truth,
          (directory/"teacher.csv").open("w",newline="") as projection):
        observation = writer(issues,ISSUE)
        labels = writer(truth,LABELS)
        teaching = writer(projection,TEACHER)
        for split,key in (("train","train_ids"),("validation","validation_ids")):
            for i in range(*CFG[key]):
                initial = stationary(random.Random(CFG["state_seed_base"]+i),CONFIG["sigma"],MODEL.size)
                states = evolve(initial,random.Random(CFG["process_seed_base"]+i),
                                CONFIG["tau_s"],CONFIG["sigma"])
                noise = CFG["detector_seed_base"] + 2*i
                codes = code_observation(settings,context(settings),burden(initial,0),
                                         AnalogNoise(noise),random.Random(noise+1))
                record = ("matched",i,split,*codes)
                observation.writerow(record)
                for h,b in zip(CFG["horizons_s"],states,strict=True):
                    labels.writerow((*record[:3],h,*targets(b)))
                issue = dict(zip(ISSUE,map(str,record),strict=True))
                for mode in CFG["modes"]:
                    for h,status,_,_,raw in forecast(issue,mode):
                        values = logit(np.array(json.loads(raw),dtype=float))
                        teaching.writerow((i,split,mode,h,float(values.mean()),
                                           float(values.std()),status))
    probe = dict(case="matched",realization="0",split="prior",R_code="",G_code="",B_code="",lwir_code="")
    dump(directory/"prior.json", [json.loads(row[-1]) for row in forecast(probe,"climatology")])


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out",type=Path)
    prepare(parser.parse_args().out)
