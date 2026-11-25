"""Framewise bounded posterior from detector codes and a frozen nuisance registry."""
import argparse
import csv
from functools import lru_cache
import math
from pathlib import Path

from features import CODES, FEATURES, KEYS, rows
from forward import CASES, CONFIG, context, ideal, settings

MODES = ("prior", "RGB", "LWIR", "joint")
COUNT = CONFIG["grid_cells"]
GRID = tuple((i + .5)/COUNT for i in range(COUNT))
RESULTS = (*KEYS, "mode","status","used","excluded","estimate","lower95","upper95","sd","local_info",
           "min_max_residual")


@lru_cache(maxsize=None)
def model(case_id):
    args = context(CASES[case_id])
    return tuple(ideal(*args,b) for b in GRID)


def quantile(weights, mass):
    accumulated = 0
    for i, w in enumerate(weights):
        if accumulated + w >= mass and w:
            return (i + (mass-accumulated)/w)/COUNT
        accumulated += w
    return 1


def observation(row, case, mode):
    if not case.get("association",True):
        return [], ["target_not_associated"]
    if not case.get("geometry_known", True):
        return [], ["range_unconstrained"]
    if not case.get("material_known", True):
        return [], ["material_unconstrained"]
    limits = settings(case)
    rgb_scale, lwir_scale = limits["rgb_well"]/limits["rgb_gain"], limits["lwir_high"]
    max_code = 2**limits["bits"]-1
    available, excluded = [], []
    options = range(3) if mode=="RGB" else [3] if mode=="LWIR" else range(4)
    forward = model(row["case"])
    for i in options:
        label = CODES[i]
        if i==3 and not case.get("air_known", True):
            excluded.append("lwir_air_unconstrained")
            continue
        if abs(forward[-1][i]-forward[0][i]) <= 1e-10:
            excluded.append(f"{label}_flat")
            continue
        value = row[label]
        if not value:
            excluded.append(f"{label}_missing")
            continue
        try:
            code = int(value)
        except ValueError:
            code = -1
        if not 0 <= code <= max_code:
            excluded.append(f"{label}_invalid")
            continue
        if code in (0,max_code):
            excluded.append(f"{label}_rail")
            continue
        scale = lwir_scale if i==3 else rgb_scale
        sensor_sigma = limits["lwir_sigma"] if i==3 else limits["rgb_read_noise_e"]/limits["rgb_gain"]
        sigma = math.hypot(sensor_sigma, scale/max_code/math.sqrt(12))
        available.append((i, code*scale/max_code, sigma))
    return available, excluded


def estimate(row, mode):
    if mode not in MODES:
        raise ValueError("invalid mode")
    if mode=="prior":
        return ("prior_only","", "",.5,.025,.975,1/math.sqrt(12),0,"")
    case = CASES[row["case"]]
    observed, excluded = observation(row, case, mode)
    labels = ",".join(excluded)
    if not observed:
        return ("abstain","",labels,"","","","","","")
    forward = model(row["case"])
    costs, worst = [], []
    for expectation in forward:
        residuals = [abs(y-expectation[i])/sigma for i,y,sigma in observed]
        costs.append(-.5*sum(x*x for x in residuals))
        worst.append(max(residuals))
    top = max(costs)
    unnormalized = [math.exp(value-top) for value in costs]
    norm = math.fsum(unnormalized)
    weights = [weight/norm for weight in unnormalized]
    mean = math.fsum(w*x for x,w in zip(GRID,weights,strict=True))
    variance = math.fsum(w*(x-mean)**2 for x,w in zip(GRID,weights,strict=True)) + 1/(12*COUNT**2)
    index = min(COUNT-2,max(1,int(mean*COUNT)))
    information = math.fsum(((forward[index+1][i]-forward[index-1][i])*COUNT/(2*sigma))**2
                            for i,_,sigma in observed)
    mismatch = min(worst)
    status = ("mismatch" if mismatch > CONFIG["mismatch_z"] else
              "weak" if math.sqrt(variance) >= CONFIG["weak_sd"] or information < 1 else "informed")
    return (status,",".join(CODES[i] for i,_,_ in observed),labels,mean,
            quantile(weights,.025),quantile(weights,.975),math.sqrt(variance),information,mismatch)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("features",type=Path)
    parser.add_argument("estimates",type=Path)
    args = parser.parse_args()
    with args.estimates.open("w",newline="") as out:
        writer = csv.writer(out,lineterminator="\n")
        writer.writerow(RESULTS)
        for row in rows(args.features, FEATURES):
            for mode in MODES:
                writer.writerow((*(row[k] for k in KEYS),mode,*estimate(row,mode)))


if __name__=="__main__":
    main()
