"""Issue full predictive marginal samples using current codes only."""
import argparse
from bisect import bisect_left
from functools import lru_cache
from itertools import accumulate
import json
import math
from pathlib import Path
import random

from contract import FEATURES, FORECASTS, IDS, output, rows
from dynamics import CONFIG, HORIZONS, MODEL, evolve, stationary
from estimate import COUNT, GRID, estimate, model, observation
from forward import CASES

N = CONFIG["marginal_size"]
LEVELS = tuple((i+.5)/N for i in range(N))
GUARDS = {"unknown_range","unknown_material","out_of_domain","dynamics_unconstrained"}


@lru_cache(maxsize=None)
def bank(tau, sigma):
    rng = random.Random(CONFIG["bank_seed"])
    paths = tuple(evolve(stationary(rng,CONFIG["sigma"],MODEL.size),rng,tau,sigma)
                  for _ in range(CONFIG["bank_size"]))
    orders = tuple(sorted(range(len(paths)), key=lambda i:paths[i][j])
                   for j in range(len(HORIZONS)))
    return paths, orders


def log_likelihood(row, mode):
    selected, reasons = observation(row, CASES[row["case"]], mode)
    if not selected:
        return (), reasons
    curve = tuple(-.5*math.fsum(((y-prediction[i])/sigma)**2 for i,y,sigma in selected)
                  for prediction in model(row["case"]))
    return curve, reasons


def weights(logs, support):
    highest = max(logs)
    result = []
    for x in support:
        i = min(COUNT-2,max(0,int(x*COUNT-.5)))
        fraction = min(1,max(0,(x-GRID[i])*COUNT))
        result.append(math.exp((1-fraction)*logs[i]+fraction*logs[i+1]-highest))
    total = math.fsum(result)
    if not total:
        return (),0.0
    normalized = tuple(x/total for x in result)
    return normalized, 1/math.fsum(x*x for x in normalized)


def marginals(paths, orders, mass, method):
    for j in range(len(HORIZONS)):
        column = 0 if method in ("persistence","climatology") else j
        order = orders[column]
        cdf = list(accumulate(mass[i] for i in order))
        proportion = cdf[-1]
        yield [paths[order[bisect_left(cdf,level*proportion)]][column] for level in LEVELS]


def forecast(row, method):
    intervention = CONFIG["cases"][row["case"]]
    guard = intervention.get("guard")
    if guard in GUARDS:
        for h in HORIZONS:
            yield h,"abstain",guard,"",""
        return
    tau = CONFIG["slow_tau_s"] if method=="slow_dynamics" else CONFIG["tau_s"]
    sigma = CONFIG["low_sigma"] if method=="low_amplitude" else CONFIG["sigma"]
    paths, orders = bank(tau,sigma)
    uniform = (1/len(paths),)*len(paths)
    mass, ess, state, reasons = uniform, len(paths), "prior_only", []
    if method not in ("climatology","physics_prior"):
        mode = method if method in ("RGB","LWIR") else "joint"
        permitted = row | {"case": intervention["inverse"]}
        if guard == "thermal_unconstrained":
            permitted["lwir_code"] = ""
            reasons.append("thermal_unconstrained")
        logs, exclusions = log_likelihood(permitted,mode)
        reasons.extend(exclusions)
        fit = estimate(permitted, mode)
        if fit[0] == "informed":
            conditioned, ess = weights(logs,(path[0] for path in paths))
            if ess >= CONFIG["ess_min"]:
                mass,state = conditioned,"conditioned"
            else:
                state = "prior_fallback"
                reasons.append("latent_particles_insufficient")
        else:
            state = "prior_fallback"
            reasons.append(f"m07_{fit[0]}")
    for h, sample in zip(HORIZONS,marginals(paths,orders,mass,method),strict=True):
        yield h,state,",".join(reasons),ess,json.dumps(sample,separators=(",",":"))


def run(source,destination):
    with destination.open("w",newline="") as file:
        writer=output(file,FORECASTS)
        for row in rows(source,FEATURES):
            for method in CONFIG["cases"][row["case"]]["methods"]:
                for result in forecast(row,method):
                    writer.writerow((*(row[k] for k in IDS),method,*result))


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("issues",type=Path)
    parser.add_argument("out",type=Path)
    args=parser.parse_args()
    run(args.issues,args.out)
