"""Independent synthetic fixtures, access tests and paired score audit."""
import argparse
from fractions import Fraction
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile

import numpy as np

from model import ARMS, Candidate, HERE, OBS, fit_logit, joint_success, propose, table


def require(condition, message, *detail):
    if not condition:
        raise AssertionError((message,*detail))


def smoke():
    model = {"models":[{"coefficients":[2,0,0,0,0],"constant":.75},
                       {"coefficients":[-2,0,0,0,0],"constant":.5}],
             "dependence":{"associated":{"shrunken_joint_failure_residual":.05}}}
    a = Candidate(4,8,10,.9,1,0)
    b = Candidate(6,8,8,.8,1,0)
    w = .8
    rows = propose((a,b),model)
    require(abs(rows["A01"][0]-(w*4+(1-w)*6))<1e-12 and rows["A01"][-1]==w,"bounded adaptive weight")
    require(abs(rows["B04"][0]-4.8)<1e-12 and rows["A08"][0]==5,"constant and equal reference")
    require(rows["A01"][3]<rows["A09"][3],"dependence lowers optimism")
    require(propose((a,Candidate(20,8,10,.9,1,0)),model)["A01"][4]=="conflict_rgb","conflict resolution")
    require(propose((None,b),model)["A01"][4]=="fallback_lwir","fallback")
    require(all(r[0]=="" for r in propose((None,None),model).values()),"no evidence")
    require(joint_success(.1,.4,8)==.4 and joint_success(.9,.9,-8)==1,"Frechet bounds")
    x = np.array([[1,-1],[1,-1],[1,1],[1,1]])
    y = np.array([0,0,1,1])
    coeff,_,_ = fit_logit(x,y)
    lo,hi = 0.,5.
    for _ in range(60):
        mid = (lo+hi)/2
        if mid<4/(1+math.exp(mid)):
            lo = mid
        else:
            hi = mid
    require(abs(coeff[0])<1e-12 and abs(coeff[1]-(lo+hi)/2)<1e-9,"independent optimizer reference")


def audit_isolation(out):
    observations = out/"holdout_obs"
    runtime = out/"deployment.json"
    with tempfile.TemporaryDirectory() as tmp:
        folder = Path(tmp)
        stage = folder/"observations"
        stage.mkdir()
        for name in ("frames.csv","candidates.csv"):
            lines = (observations/name).read_text().splitlines()
            (stage/name).write_text("\n".join(lines[:2 if name=="frames.csv" else 3])+"\n")
        np.save(stage/"masks.npy",np.load(observations/"masks.npy",mmap_mode="r")[:1])
        hidden = folder/"scoring.csv"
        hidden.write_text("true_x,visibility\n16,1\n")
        issued = folder/"decision.csv"
        command = [sys.executable,str(HERE/"isolate.py"),str(stage),str(runtime),str(issued),
                   "--issue","--deny-read",str(hidden)]
        subprocess.run(command,check=True,capture_output=True)
        before = issued.read_bytes()
        hidden.write_text("true_x,visibility\n4,0\n")
        subprocess.run(command,check=True,capture_output=True)
        require(before==issued.read_bytes(),"altered truth changed isolated decisions")
        frames = stage/"frames.csv"
        original = frames.read_text().splitlines()
        for field in ("true_x","first_hit","case","actual_burden","future_state","oracle"):
            frames.write_text(f"{original[0]},{field}\n{original[1]},999\n")
            result = subprocess.run(command,capture_output=True,text=True)
            require(result.returncode!=0 and "unapproved schema" in result.stderr,"privileged field accepted",field)
        require(tuple(original[0].split(","))==OBS,"observation allowlist")


def audit(out):
    smoke()
    audit_isolation(out)
    holdout = out/"holdout"
    labels = {r["id"]:r for r in table(holdout/"labels.csv")}
    proposals = {(r["id"],r["arm"]):r for r in table(holdout/"proposals.csv")}
    issued = {(r["id"],r["arm"]):r for r in table(holdout/"issued.csv")}
    report = json.loads((out/"results/metrics.json").read_text())
    historical = {"B0"+str(i):value for i,value in enumerate((4176,3324,4327),start=1)}
    for arm,k in historical.items():
        stats = report["metrics"][arm]["overall"]
        require((stats["correct"],stats["feasible"])==(k,5120),"PF1 regression",arm,stats)
    groups = {}
    for key,row in labels.items():
        groups.setdefault(row["realization"],{})[row["case"]] = key
    require(len(groups)==256 and all(len(group)==27 for group in groups.values()),"group population")
    for group in groups.values():
        for arm in ARMS:
            ref = proposals[group["reference"],arm]
            for control in ("blind_future","distractor","unknown_range","unknown_material"):
                other = proposals[group[control],arm]
                require(all(ref[k]==other[k] for k in ref if k!="id"),"observational equivalence",control,arm)
            missing = issued[group["missing_both"],arm]
            require(missing["status"]=="abstain" and not missing["x"],"missing both",arm)
            for control in ("absent","hidden"):
                require(issued[group[control],arm]["status"]=="abstain","unexpected negative issuance",control,arm)
    baseline = {(r["id"],r["arm"]):r for r in table(holdout/"baselines.csv")}
    counts = [0]*256
    oracle = 0
    for key,truth in labels.items():
        if truth["feasible"]!="1":
            continue

        def correct(row):
            return bool(row["x"]) and sum((Fraction(row[k])-Fraction(truth[k]))**2 for k in ("x","y"))<=1

        b03 = baseline[key,"B03"]
        delta = int(correct(issued[key,"A01"]))-int(correct(b03) and float(b03["score"])>=.3)
        counts[int(truth["realization"])-256] += delta
        oracle += any(correct(row) for row in
                      [*(baseline[key,arm] for arm in historical),proposals[key,"B04"],proposals[key,"A01"]])
    require(oracle==report["metrics"]["O01"]["overall"]["correct"],"oracle candidate opportunity")
    rng = np.random.default_rng(10091)
    distribution = []
    for _ in range(10000):
        indices = rng.integers(0,256,256)
        distribution.append(sum(counts[i] for i in indices)/5120)
    q = statistics.quantiles(distribution,n=40,method="inclusive")
    observed = report["intervals"]["overall"]["A01_paired_contrasts"]["B03"]
    error = max(abs(q[0]-observed[0]),abs(q[-1]-observed[1]))
    require(error<=1e-12,"independent grouped bootstrap",error)
    theoretical = 1.96**2/(256+1.96**2)
    for arm in (*historical,*ARMS):
        for category in ("absent","hidden"):
            actual = report["metrics"][arm]["overall"]["negative"][category]["wilson_upper95"]
            require(abs(actual-theoretical)<1e-12,"independent Wilson zero-event limit",arm,category)
    print(json.dumps({"status":"PASS","scenarios_per_realization":27,"realizations":256,
        "historical_candidate_and_score_integrity":"PASS","admissible_input_equivalence_sets":4,
        "forbidden_fields_rejected":6,"privileged_store_read_denied":True,"privileged_label_mutation_invariance":True,
        "math_fixtures":"PASS","independent_oracle_correct":oracle,
        "independent_primary_bootstrap_endpoint_error":error},indent=2))


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode",choices=("smoke","audit"))
    parser.add_argument("out",type=Path,nargs="?")
    args = parser.parse_args()
    if args.mode=="smoke":
        smoke()
        print("PASS: deterministic fit, weights, dependence, conflicts and fallback fixtures.")
    else:
        audit(args.out)
