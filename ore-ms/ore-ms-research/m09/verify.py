"""Independent checks for M09 gradients, physical maps, paired scores, and isolation."""
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from assess import CLASSICAL, GROUP, PROBS, SCORE, crps
from common import CFG, CODES, HERE, HORIZONS, ISSUE, ISSUED, LABELS, ROOT, digest, parse, sigmoid, target_mapping
from independent import empirical, energy_pair, interval
from infer import issue, load
from network import initialize, objective
from train import CURVE, preprocess


def simple(path):
    with path.open(newline="") as file:
        return list(csv.DictReader(file))


def analytic_sample(row):
    mean,scale=(np.array(json.loads(row[k])) for k in ("location","scale"))
    path=sigmoid(mean[1,0]+scale[1,0]*PROBS)
    return list(map(float,path))


class Verify(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.train=list(parse(OUT/"data"/"issues.csv",ISSUE))
        cls.raw=list(parse(RAW/"issues.csv",ISSUE))
        cls.forecast=list(parse(OUT/"issued.csv",ISSUED))
        cls.scores=list(parse(OUT/"evaluation"/"scores.csv",SCORE))
        cls.curves=simple(OUT/"train"/"curves.csv")
        cls.checkpoints={file.stem:load(file) for file in sorted(
            (OUT/"train"/"checkpoints").glob("*.json"))}

    def test_01_gradient_and_training_selection(self):
        rng=np.random.default_rng(811)
        x,y=rng.normal(size=(4,8)),rng.normal(size=(4,5,3))
        teacher=rng.normal(size=(4,5,2))
        teacher[...,1]=np.exp(teacher[...,1])+.1
        max_error=0
        for dimensions,strength in ((3,0),(1,0),(1,.5)):
            parameters=initialize(901,dimensions)
            _,_,analytic=objective(parameters,x,y,dimensions,teacher,strength)
            for j,arr in enumerate(parameters):
                for idx in list(np.ndindex(arr.shape))[::max(1,arr.size//15)]:
                    original=arr[idx]
                    arr[idx]=original+1e-6
                    upper=objective(parameters,x,y,dimensions,teacher,strength)[0]
                    arr[idx]=original-1e-6
                    lower=objective(parameters,x,y,dimensions,teacher,strength)[0]
                    arr[idx]=original
                    max_error=max(max_error,abs((upper-lower)/2e-6-analytic[j][idx]))
        self.assertLessEqual(max_error,2e-6)
        self.assertEqual(len(self.checkpoints),18)
        curves=defaultdict(list)
        for row in self.curves:
            stem=f"{row['model']}-{row['size']}-{row['seed']}"
            curves[stem].append(row)
            self.assertTrue(all(math.isfinite(float(row[k])) for k in CURVE[4:]))
        selected=[]
        source={name:digest(OUT/"data"/name) for name in ("issues.csv","labels.csv","teacher.csv")}
        for stem,checkpoint in self.checkpoints.items():
            track=curves[stem]
            self.assertEqual(len(track),CFG["epochs"])
            winner=min(track,key=lambda row:float(row["validation_burden_nll"]))
            self.assertEqual(checkpoint["epoch"],int(winner["epoch"]))
            self.assertEqual(checkpoint["validation_burden_nll"],float(winner["validation_burden_nll"]))
            self.assertEqual(checkpoint["steps"],CFG["epochs"]*math.ceil(
                checkpoint["size"]*len(CFG["modes"])/CFG["batch_size"]))
            self.assertEqual(checkpoint["training_files"],source)
            self.assertEqual(checkpoint["parameter_count"],966 if checkpoint["model"]=="data" else 466)
            selected.append(checkpoint["epoch"])
        for size in CFG["train_sizes"]:
            independent=[]
            for seed in CFG["seeds"]:
                p,c=(self.checkpoints[f"{n}-{size}-{seed}"] for n in ("physics","ablation"))
                for field in ("initial","feature_scaling","shuffle_seed","steps","dimensions"):
                    self.assertEqual(p[field],c[field])
                independent.append(p["shuffle_seed"])
                self.assertTrue(any(not np.array_equal(a,b) for a,b in
                                    zip(p["parameters"],c["parameters"],strict=True)))
            self.assertEqual(len(set(independent)),len(CFG["seeds"]))
        print(f"18 fits, exact validation checkpoint selection epochs {min(selected)}..{max(selected)}; "
              f"gradient max error={max_error:.10g}",flush=True)

    def test_02_physical_maps_and_scores(self):
        sys.path.extend(str(ROOT/name) for name in ("m02","m03"))
        from rgb_model import Layer,transfer
        from thermal_model import Slab,atmosphere
        max_error=0
        across_tables=0
        inconsistencies=[]
        for row in self.forecast:
            if row["status"]!="issued":
                self.assertEqual(row["location"],"")
                continue
            loc,scale=(np.array(json.loads(row[k])) for k in ("location","scale"))
            self.assertTrue(np.isfinite(loc).all())
            self.assertTrue((np.isfinite(scale)&(scale>=CFG["min_scale"])).all())
            values=sigmoid(loc[...,None]+scale[...,None]*PROBS)
            if row["model"]!="data":
                values=target_mapping(values[:,0,:])
            elif (row["case"],row["size"],row["mode"],row["seed"])==("matched","768","joint","901"):
                mapped=np.sort(target_mapping(values[:,0,:]),axis=-1)
                inconsistencies.append(float(np.abs(values[:,1:]-mapped[:,1:]).mean()))
            self.assertTrue(((values>=0)&(values<=1)).all())
            across_tables+=1
        for b in (0,.2,.55,1):
            predicted=target_mapping(np.array([b]))[:,0]
            rgb=transfer(1,[Layer(10,.003+.135*b,0,0,0)],0).transmission
            lw=atmosphere([Slab.absorbing(11,10,.002+.024*b,294)])[0]
            max_error=max(max_error,abs(float(predicted[1])-rgb),abs(float(predicted[2])-lw))
            self.assertNotEqual(rgb,lw)
        self.assertLessEqual(max_error,2e-12)
        samples=np.array([.1,.35,.55,.82])
        truth=.23
        crps_error=abs(float(crps(samples,np.array(truth)))-empirical(samples,truth))
        self.assertLessEqual(crps_error,2e-12)
        self.assertLessEqual(abs((.68-1)**2-energy_pair(.68,1)),2e-12)

        # Compare the M09 scorer against the independently implemented M08 report.
        previous={(r["case"],r["method"],r["horizon_s"],r["target"]):r
                  for r in simple(RAW/"scores.csv")}
        parity=0
        for row in self.scores:
            if row["family"]!="classical":
                continue
            ref=previous[tuple(row[k] for k in ("case","model","horizon_s","target"))]
            self.assertEqual((row["n"],row["issued"]),(ref["n"],ref["issued"]))
            for column in ("crps","brier","coverage95"):
                if ref[column]:
                    parity=max(parity,abs(float(row[column])-float(ref[column])))
                else:
                    self.assertEqual(row[column],"")
        self.assertLessEqual(parity,2e-12)
        print(f"{across_tables} finite learned distributions; mapping error={max_error:.10g}; "
              f"independent CRPS error={crps_error:.10g}; archived classical score delta={parity:.10g}; "
              f"unconstrained mean quantile-map deviation={np.mean(inconsistencies):.10g}",flush=True)

    def test_03_guard_and_novelty_cases(self):
        out=defaultdict(set)
        blind={}
        for row in self.forecast:
            out[row["case"]].add(row["status"])
            if row["case"] in ("matched","dynamics_mismatch"):
                identity=tuple(row[k] for k in ("realization","model","size","seed","mode"))
                blind.setdefault(identity,{})[row["case"]]=tuple(
                    row[k] for k in ("location","scale","novelty","novel"))
        for case in ("clipped","missing","occluded","weak"):
            self.assertEqual(out[case],{"prior_fallback"})
        for case in ("range_unknown","material_unknown","domain_unknown","process_unknown"):
            self.assertEqual(out[case],{"abstain"})
        self.assertEqual(out["thermal_unknown"],{"issued"})
        for paired in blind.values():
            self.assertEqual(paired["matched"],paired["dynamics_mismatch"])
        for row in simple(OUT/"evaluation"/"ood.csv"):
            if row["case"]=="dynamics_mismatch":
                self.assertEqual(float(row["auc"]),.5)
        r=next(r for r in self.raw if r["case"]=="matched" and r["split"]=="holdout")
        model=self.checkpoints["physics-768-901"]
        self.assertEqual(issue(r|{CODES[-1]:"-1"},"joint",model),issue(r,"RGB",model))
        self.assertEqual(issue(r|{CODES[-1]:""},"joint",model),issue(r,"RGB",model))
        print("all hard guard outcomes verified; dropped/invalid LWIR equals RGB-only; "
              "blind process shift has identical issue forecasts and novelty AUC 0.5",flush=True)

    def test_04_split_and_inference_isolation(self):
        partitions=defaultdict(set)
        for row in self.train:
            partitions[row["split"]].add(int(row["realization"]))
        partitions["holdout"]={int(r["realization"]) for r in self.raw if r["split"]=="holdout"}
        for split,cfg in (("train","train_ids"),("validation","validation_ids"),("holdout","test_ids")):
            self.assertEqual(partitions[split],set(range(*CFG[cfg])))
        self.assertEqual(len(set.union(*partitions.values())),1152)
        with tempfile.TemporaryDirectory() as directory:
            isolated=Path(directory)
            for name in ("common.py","network.py","infer.py","config.json"):
                shutil.copyfile(HERE/name,isolated/name)
            models=isolated/"models"
            models.mkdir()
            stem="physics-768-901.json"
            shutil.copyfile(OUT/"train"/"checkpoints"/stem,models/stem)
            selected=next(r for r in self.raw if r["case"]=="matched" and r["split"]=="holdout")
            source=isolated/"features.csv"
            with source.open("w",newline="") as file:
                writer=csv.DictWriter(file,ISSUE)
                writer.writeheader()
                writer.writerow(selected)
                writer.writerow(selected|{"realization":"999999"})
            destination=isolated/"issued.csv"
            cmd=[sys.executable,str(isolated/"infer.py"),str(source),str(models),str(destination)]
            subprocess.run(cmd,check=True)
            for result in parse(destination,ISSUED):
                expected=next(r for r in self.forecast if all(r[k]==result[k] for k in
                    ("case","model","size","seed","mode")) and r["realization"]==selected["realization"])
                self.assertEqual({k:v for k,v in result.items() if k!="realization"},
                                 {k:v for k,v in expected.items() if k!="realization"})
            source.write_text(",".join((*ISSUE,"future_burden"))+"\n")
            rejected=subprocess.run(cmd,capture_output=True,text=True)
            self.assertNotEqual(rejected.returncode,0)
            self.assertIn("unexpected input columns",rejected.stderr)
        print("768 train / 256 validation / 128 holdout IDs; isolated deployment has no truth, "
              "teachers or prior-milestone assets; ID mutation invariant",flush=True)

    def test_05_paired_independent_comparison(self):
        # Recompute the small-set physics-vs-ablation 2 s CRPS comparison
        # directly using the unoptimized all-pairs definition.
        truth={(r["realization"],r["horizon_s"]):float(r["burden"])
               for r in parse(RAW/"truth.csv",LABELS) if r["case"]=="matched" and r["split"]=="holdout"}
        scores={}
        for row in self.forecast:
            if (row["case"],row["size"],row["mode"])!=("matched","128","joint"):
                continue
            if row["model"] not in ("physics","ablation"):
                continue
            r=int(row["realization"])
            scores[row["model"],r,int(row["seed"])]=empirical(analytic_sample(row),truth[str(r),"2"])
        paired=[sum(scores["physics",r,s]-scores["ablation",r,s] for s in CFG["seeds"])/3
                for r in range(*CFG["test_ids"])]
        entries=simple(OUT/"evaluation"/"paired.csv")
        row=next(r for r in entries if (r["comparison"],r["mode"],r["size"],r["horizon_s"])==
                 ("physics_minus_ablation","joint","128","2"))
        max_error=max(abs(a-float(row[k])) for a,k in
                      zip(interval(paired),("delta_crps","lower95","upper95"),strict=True))
        self.assertLessEqual(max_error,2e-12)
        self.assertTrue(all(int(row["paired_realizations"])==128 for row in entries))
        print(f"{len(entries)} paired cells, 128 independent IDs per cell; independent "
              f"comparison max difference={max_error:.10g}",flush=True)


if __name__=="__main__":
    OUT=Path(sys.argv.pop(1))
    RAW=Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
