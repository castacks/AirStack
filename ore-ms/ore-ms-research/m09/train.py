"""Train the frozen 18-run M09 matrix and hash selected checkpoints."""
import argparse
import csv
import json
from pathlib import Path
import resource
import time

import numpy as np

from common import (CFG, ISSUE, LABELS, TEACHER, digest, dump, features, logit, parse,
                    raw_codes, writer)
from network import Adam, initialize, normal_nll, objective, predict

CURVE = ("model","size","seed","epoch","objective","teacher_kl","validation_burden_nll","lr")


def read_dataset(directory):
    observations = list(parse(directory/"issues.csv",ISSUE))
    expected = {(str(i), split) for split, key in (("train","train_ids"),
                ("validation","validation_ids")) for i in range(*CFG[key])}
    if len(observations) != len(expected) or {
            (r["realization"],r["split"]) for r in observations} != expected:
        raise ValueError("training/validation population violates frozen split")
    targets = {(r["realization"],r["horizon_s"]):[float(r[k]) for k in LABELS[4:]]
               for r in parse(directory/"labels.csv",LABELS)}
    moments = {(r["realization"],r["mode"],r["horizon_s"]):
               [float(r["mean"]),max(CFG["min_scale"],float(r["scale"]))]
               for r in parse(directory/"teacher.csv",TEACHER)}
    return observations,targets,moments


def preprocess(rows):
    x = np.array([raw_codes(row,"joint") for row in rows])
    center = np.nanmean(x,axis=0)
    scale = np.maximum(CFG["feature_scale_floor"],np.nanstd(x,axis=0))
    standardized = (x-center)/scale
    complete = standardized[np.isfinite(standardized).all(axis=1)]
    covariance = np.cov(complete,rowvar=False)+CFG["novelty_ridge"]*np.eye(4)
    mask_thresholds = {}
    for mask in range(1,16):
        active = [i for i in range(4) if mask & (1<<i)]
        values = complete[:,active]
        precision = np.linalg.inv(covariance[np.ix_(active,active)])
        distance = np.einsum("ni,ij,nj->n",values,precision,values)
        mask_thresholds[str(mask)] = float(np.quantile(distance,CFG["novelty_quantile"]))
    return {"center":center.tolist(),"scale":scale.tolist(),"covariance":covariance.tolist(),
            "thresholds":mask_thresholds}


def matrices(rows,targets,moments,scaling):
    views=[(row,mode) for row in rows for mode in CFG["modes"]]
    return (
        np.array([features(r,m,scaling) for r,m in views]),
        logit(np.array([[targets[r["realization"],str(h)] for h in CFG["horizons_s"]]
                        for r,_ in views])),
        np.array([[moments[r["realization"],m,str(h)] for h in CFG["horizons_s"]]
                  for r,m in views]),
    )


def train(directory,destination):
    destination.mkdir()
    folder = destination/"checkpoints"
    folder.mkdir()
    observations,targets,moments = read_dataset(directory)
    fingerprints = {name:digest(directory/name) for name in ("issues.csv","labels.csv","teacher.csv")}
    all_train = [r for r in observations if r["split"]=="train"]
    validation = [r for r in observations if r["split"]=="validation"]
    measured=[]
    with (destination/"curves.csv").open("w",newline="") as file:
        output=writer(file,CURVE)
        for size in CFG["train_sizes"]:
            train_rows=all_train[:size]
            scaling=preprocess(train_rows)
            x,y,teachers=matrices(train_rows,targets,moments,scaling)
            vx,vy,_=matrices(validation,targets,moments,scaling)
            for name in CFG["models"]:
                dimensions=3 if name=="data" else 1
                strength=CFG["physics_weight"] if name=="physics" else 0.0
                for seed in CFG["seeds"]:
                    start=time.perf_counter()
                    parameters=initialize(seed,dimensions)
                    first=[p.tolist() for p in parameters]
                    optimizer=Adam(parameters)
                    shuffle=np.random.default_rng(seed+4032)
                    best=(float("inf"),None,None)
                    for epoch in range(1,CFG["epochs"]+1):
                        order=shuffle.permutation(len(x))
                        lr=CFG["learning_rate"]*(CFG["decay_factor"] if epoch>CFG["decay_epoch"] else 1)
                        total,kl_sum=0.0,0.0
                        for offset in range(0,len(order),CFG["batch_size"]):
                            idx=order[offset:offset+CFG["batch_size"]]
                            loss,kl,grad=objective(parameters,x[idx],y[idx],dimensions,
                                                   teachers[idx],strength)
                            optimizer.update(parameters,grad,lr)
                            total+=len(idx)*loss
                            kl_sum+=len(idx)*kl
                        _,loc,sigma,_=predict(parameters,vx,dimensions)
                        val=float(normal_nll(loc[...,0],sigma[...,0],vy[...,0]).mean())
                        output.writerow((name,size,seed,epoch,total/len(x),kl_sum/len(x),val,lr))
                        if val<best[0]:
                            best=(val,epoch,[p.copy() for p in parameters])
                    val,epoch,chosen=best
                    stem=f"{name}-{size}-{seed}"
                    dump(folder/f"{stem}.json",{"model":name,"size":size,"seed":seed,
                        "dimensions":dimensions,"epoch":epoch,"validation_burden_nll":val,
                        "parameters":[p.tolist() for p in chosen],"parameter_count":sum(p.size for p in chosen),
                        "feature_scaling":scaling,"training_files":fingerprints,"initial":first,
                        "shuffle_seed":seed+4032,"steps":optimizer.steps})
                    measured.append((stem,time.perf_counter()-start,resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))
    with (destination/"training_runtime.csv").open("w",newline="") as file:
        writer(file,("checkpoint","wall_s","cumulative_peak_rss_kib")).writerows(measured)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data",type=Path)
    parser.add_argument("out",type=Path)
    args=parser.parse_args()
    train(args.data,args.out)
