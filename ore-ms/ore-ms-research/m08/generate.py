"""Write issue observations, future labels, and privileged inputs separately."""
import argparse
import json
from pathlib import Path
import random

from contract import FEATURES, LABELS, ORACLE, output
from dynamics import (CONFIG, HERE, HORIZONS, MODEL, burden, evolve, stationary, targets)
from forward import AnalogNoise, CASES, code_observation, context


def generate(folder):
    private = json.loads((HERE/"truth_config.json").read_text())
    folder.mkdir()
    with ((folder/"issues.csv").open("w",newline="") as observations,
          (folder/"truth.csv").open("w",newline="") as truth,
          (folder/"oracle_input.csv").open("w",newline="") as privileged):
        issues, labels, oracle = (output(file,columns) for file,columns in
                                ((observations,FEATURES),(truth,LABELS),(privileged,ORACLE)))
        for ident in range(CONFIG["realizations"]):
            split = "development" if ident < CONFIG["development"] else "holdout"
            seed = private["truth_seed_base"] + ident
            initial = stationary(random.Random(seed), CONFIG["sigma"], MODEL.size)
            matched = evolve(initial,random.Random(seed+1_000_000),CONFIG["tau_s"],CONFIG["sigma"])
            mismatch = evolve(initial,random.Random(seed+1_000_000),
                              private["mismatch_tau_s"], private["mismatch_sigma"])
            oracle.writerow(("matched",ident,split,json.dumps(initial,separators=(",",":"))))
            for name, intervention in CONFIG["cases"].items():
                spec = CASES[intervention["inverse"]]
                material, length, air = context(spec)
                if name == "occluded":
                    material,length = "foreground",5
                if name == "air_mismatch":
                    air = 318
                noise_seed = private["noise_seed_base"]+2*ident
                codes = code_observation(
                    spec,(material,length,air),burden(initial,0,length),
                    AnalogNoise(noise_seed),random.Random(noise_seed+1))
                if name == "missing":
                    codes = [""]*4
                issues.writerow((name,ident,split,*codes))
                for h,b in zip(HORIZONS,mismatch if name=="dynamics_mismatch" else matched,strict=True):
                    labels.writerow((name,ident,split,h,*targets(b)))


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out",type=Path)
    generate(parser.parse_args().out)
