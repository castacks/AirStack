"""Independent stochastic, scoring, physical, and leakage checks for M08."""
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
import random
import shutil
import statistics
import subprocess
import sys
import tempfile
import unittest

from contract import FEATURES, FORECASTS, IDS, LABELS, rows
from dynamics import (CONFIG, HERE, HORIZONS, LENGTH, MODEL, OPTICS, ZERO, advance,
                      burden, stationary, targets, transition)
from estimate import GRID, estimate
from predict import bank, log_likelihood, weights
from reference import independent_burden, ou_reference
from scores import auc, categorical_score, crps, quantile

ROOT=HERE.parent


def key(row):
    return tuple(row[k] for k in IDS)


def unique(records, fn):
    items=list(records)
    result={fn(r):r for r in items}
    if len(result)!=len(items):
        raise ValueError("duplicate record")
    return result


class Verify(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.issues=unique(rows(OUT/"issues.csv",FEATURES),key)
        cls.truth=unique(rows(OUT/"truth.csv",LABELS),lambda row:(key(row),int(row["horizon_s"])))
        cls.forecasts=unique(rows(OUT/"forecasts.csv",FORECASTS),
                             lambda row:(key(row),row["method"],int(row["horizon_s"])))
        cls.oracle=unique(rows(OUT/"oracle_forecasts.csv",FORECASTS),
                          lambda row:(key(row),int(row["horizon_s"])))
        with (OUT/"scores.csv").open(newline="") as file:
            cls.scores=unique(csv.DictReader(file),lambda r:(r["case"],r["method"],
                                                              int(r["horizon_s"]),r["target"]))

    def row(self,method,h,case="matched",target="burden"):
        return self.scores[case,method,h,target]

    def test_01_exact_ou_spatial_and_transport(self):
        tau,sigma=CONFIG["tau_s"],CONFIG["sigma"]
        a,q=transition(6,tau,sigma)
        ref_mu,ref_var=ou_reference(.4,6,tau,sigma)
        error=max(abs(a*.4-ref_mu),abs(q*q-ref_var))
        short=transition(2,tau,sigma)
        long=transition(4,tau,sigma)
        combined=(short[0]*long[0],(long[0]*short[1])**2+long[1]**2)
        semigroup=max(abs(combined[0]-a),abs(combined[1]-q*q))
        self.assertLessEqual(max(error,semigroup),2e-12)
        rng=random.Random(85853)
        sample=[advance((.4,),(0,),(a,q),rng)[0] for _ in range(4096)]
        mean_error=abs(statistics.fmean(sample)-ref_mu)
        var_error=abs(statistics.variance(sample)-ref_var)
        self.assertLessEqual(max(mean_error,var_error),.07)
        samples=[stationary(rng,1,MODEL.size) for _ in range(4096)]
        left,right=MODEL.basis(0),MODEL.basis(8)
        covariance=statistics.fmean(math.fsum(a*x for a,x in zip(v,left,strict=True))*
                                     math.fsum(a*x for a,x in zip(v,right,strict=True)) for v in samples)
        cos=math.sqrt(.5)
        cov_error=abs(covariance-(.6*cos*cos+.5*cos-.1))
        self.assertLessEqual(cov_error,.07)
        for coeffs in samples[:10]:
            self.assertLessEqual(abs(burden(coeffs,0)-burden(coeffs,64)),2e-12)
        decay,innovation=transition(48,tau,sigma)
        self.assertLessEqual(decay,.001)
        self.assertLessEqual(abs(sigma*sigma-innovation**2),1e-6)
        self.assertGreater(float(self.row("joint",48)["width95"]),float(self.row("joint",0)["width95"]))
        print(f"OU formula error={error:.10g}; semigroup={semigroup:.10g}; "
              f"mean/variance MC errors={mean_error:.10g}/{var_error:.10g}; spatial cov error={cov_error:.10g}; "
              f"long decay={decay:.10g}, variance gap={abs(sigma*sigma-innovation**2):.10g}",flush=True)

    def test_02_insufficient_scalar_and_mapping(self):
        angle=math.pi*LENGTH/CONFIG["period_m"]
        scale=2/math.sqrt(.5)
        states=[(0,-sign*scale*math.sin(angle),sign*scale*math.cos(angle),0,0)
                for sign in (1,-1)]
        initial=[burden(state,0) for state in states]
        self.assertLessEqual(abs(initial[0]-initial[1]),2e-12)
        a,q=transition(6,CONFIG["tau_s"],CONFIG["sigma"])
        rng=random.Random(86701)
        signed=[]
        for _ in range(4096):
            noise=[rng.gauss(0,q) for _ in states[0]]
            signed.append(burden(tuple(a*x+n for x,n in zip(states[0],noise,strict=True)),6)-
                          burden(tuple(a*x+n for x,n in zip(states[1],noise,strict=True)),6))
        difference=statistics.fmean(signed)
        self.assertGreater(abs(difference),.02)
        reference_error=max(abs(burden(x,h)-independent_burden(x,h,CONFIG))
                            for x in states for h in HORIZONS)
        self.assertLessEqual(reference_error,2e-12)
        from rgb_model import Layer, transfer
        from thermal_model import Slab, atmosphere
        _,r,t=targets(.6)
        extinction=OPTICS["rgb_floor"]+.6*OPTICS["rgb_span"]
        result=transfer(1,[Layer(LENGTH,extinction,0,0,0)],0).transmission
        alpha=OPTICS["lwir_floor"][1]+.6*OPTICS["lwir_span"][1]
        thermal=atmosphere([Slab.absorbing(11,LENGTH,alpha,294)])[0]
        mapping=max(abs(r-result),abs(t-thermal))
        self.assertLessEqual(mapping,2e-12)
        print(f"scalar counterexample b0={initial}, predictive mean difference={difference:.10g}; "
              f"midpoint reference error={reference_error:.10g}; kernel mapping error={mapping:.10g}",flush=True)

    def test_03_conditioning_and_oracle(self):
        example=next(r for r in self.issues.values() if r["case"]=="matched")|{"case":"matched"}
        diff=[]
        for mode in ("RGB","LWIR","joint"):
            logs,_=log_likelihood(example,mode)
            mass=[math.exp(x-max(logs)) for x in logs]
            chosen=math.fsum(x*b for x,b in zip(mass,GRID,strict=True))/math.fsum(mass)
            result=estimate(example,mode)
            self.assertEqual(result[0],"informed")
            diff.append(abs(chosen-result[3]))
        self.assertLessEqual(max(diff),2e-12)
        paths,_=bank(CONFIG["tau_s"],CONFIG["sigma"])
        x=[row[0] for row in paths]
        mass,ess=weights(logs,x)
        mu=math.fsum(w*z for w,z in zip(mass,x,strict=True))
        sd=math.sqrt(math.fsum(w*(z-mu)**2 for w,z in zip(mass,x,strict=True)))
        self.assertLess(sd,statistics.pstdev(x))
        for identity,row in self.oracle.items():
            self.assertEqual(row["status"],"oracle_only")
            if identity[1]==0:
                value=float(self.truth[identity]["burden"])
                self.assertEqual(set(json.loads(row["burden_samples"])),{value})
        self.assertEqual(float(self.row("oracle",0)["coverage95"]),1)
        self.assertEqual(float(self.row("oracle",0)["crps"]),0)
        print(f"M07 likelihood reconstruction max mean difference={max(diff):.10g}; "
              f"initial prior/conditional SD={statistics.pstdev(x):.10g}/{sd:.10g}; ESS={ess:.10g}; "
              "oracle h=0 exact",flush=True)

    def test_04_population_calibration_and_failure(self):
        expected={(k,m,h) for k in self.issues for m in CONFIG["cases"][k[0]]["methods"]
                  for h in HORIZONS}
        self.assertEqual(set(self.forecasts),expected)
        self.assertEqual(set(self.truth),{(k,h) for k in self.issues for h in HORIZONS})
        self.assertEqual(set(self.oracle),{(k,h) for k in self.issues if k[0]=="matched" for h in HORIZONS})
        checks={"missing":"prior_fallback","clipped":"prior_fallback","occluded":"prior_fallback",
                "weak":"prior_fallback","range_unknown":"abstain","material_unknown":"abstain",
                "domain_unknown":"abstain","process_unknown":"abstain"}
        for (k,method,h),row in self.forecasts.items():
            if k[0] in checks:
                self.assertEqual(row["status"],checks[k[0]])
                self.assertTrue(row["flags"])
            if k[0]=="thermal_unknown":
                self.assertIn("thermal_unconstrained",row["flags"])
                self.assertIn("lwir_code_missing",row["flags"])
                matched_rgb = self.forecasts[("matched", *k[1:]),"RGB",h]
                self.assertEqual(row["burden_samples"],matched_rgb["burden_samples"])
            if row["status"]=="abstain":
                self.assertEqual(row["burden_samples"],"")
                continue
            samples=json.loads(row["burden_samples"])
            self.assertEqual(len(samples),CONFIG["marginal_size"])
            self.assertEqual(samples,sorted(samples))
            self.assertTrue(all(0<=v<=1 for b in samples for v in targets(b)))
            if k[0]=="matched" and method in ("persistence","climatology"):
                self.assertEqual(row["burden_samples"],self.forecasts[k,method,0]["burden_samples"])
        joint=[float(self.row("joint",h)["coverage95"]) for h in HORIZONS]
        self.assertTrue(all(.83<=coverage<=1 for coverage in joint))
        self.assertTrue(all(int(self.row("joint",h)["n"])==128 for h in HORIZONS))
        self.assertAlmostEqual(crps([0,1],.5),.25)
        self.assertAlmostEqual(categorical_score([.1,.8],.8),-math.log(1.5/12))
        self.assertEqual(quantile([.123]*5,.025),.123)
        self.assertEqual(auc([0,1],[0,1]),1)
        # Independently score the frozen two-second threshold from retained samples.
        brier_terms = []
        events = 0
        for (ident, method, h), row in self.forecasts.items():
            if ident[0] != "matched" or ident[2] != "holdout" or method != "joint" or h != 2:
                continue
            sample = json.loads(row["burden_samples"])
            threshold_burden = (-math.log(CONFIG["rgb_event_below"])/LENGTH
                                - OPTICS["rgb_floor"])/OPTICS["rgb_span"]
            probability = sum(b > threshold_burden for b in sample)/len(sample)
            event = float(self.truth[ident,h]["rgb_t550"]) < CONFIG["rgb_event_below"]
            events += event
            brier_terms.append((probability-event)**2)
        threshold_score = self.row("joint",2,target="rgb_t550")
        self.assertEqual(events,int(threshold_score["event_count"]))
        self.assertAlmostEqual(statistics.fmean(brier_terms),float(threshold_score["brier"]))
        wrong=[float(self.row("joint",h,"dynamics_mismatch")["coverage95"]) for h in HORIZONS]
        air=float(self.row("joint",0,"air_mismatch")["coverage95"])
        print(f"{len(self.issues)} issue rows, {len(self.truth)} truth rows, {len(self.forecasts)} deployable "
              f"rows, {len(self.oracle)} oracle rows; matched coverage={joint}; "
              f"wrong dynamics={wrong}; wrong-air h0={air:.8g}",flush=True)

    def test_05_leakage_and_seed_variability(self):
        split=defaultdict(set)
        for row in self.issues.values():
            split[row["split"]].add(row["realization"])
        self.assertEqual([len(split[x]) for x in ("development","holdout")],[64,128])
        self.assertFalse(split["development"]&split["holdout"])
        self.assertGreater(len({r["burden"] for r in self.truth.values()
                                if r["case"]=="matched" and r["horizon_s"]=="6"}),180)
        self.assertGreater(len({r["burden_samples"] for (k,m,h),r in self.forecasts.items()
                                if k[0]=="matched" and m=="joint" and h==2}),100)
        selected=next(row for row in self.issues.values() if row["case"]=="matched")
        with tempfile.TemporaryDirectory() as directory:
            isolated=Path(directory)
            for name in ("m02/rgb_model.py","m03/thermal_model.py","m04/geometry.py",
                         "m05/environment.py","m06/adapters.py","m07/estimate.py","m07/features.py",
                         "m07/forward.py","m07/config.json","m08/dynamics.py","m08/contract.py",
                         "m08/predict.py","m08/config.json"):
                target=isolated/name
                target.parent.mkdir(parents=True,exist_ok=True)
                shutil.copyfile(ROOT/name,target)
            issue=isolated/"issues.csv"
            with issue.open("w",newline="") as file:
                writer=csv.DictWriter(file,FEATURES)
                writer.writeheader()
                writer.writerow(selected)
                writer.writerow(selected|{"realization":"900","split":"holdout"})
            out=isolated/"out.csv"
            cmd=[sys.executable,str(isolated/"m08/predict.py"),str(issue),str(out)]
            subprocess.run(cmd,check=True)
            for row in rows(out,FORECASTS):
                initial=self.forecasts[key(selected),row["method"],int(row["horizon_s"])]
                self.assertEqual({k:v for k,v in initial.items() if k not in IDS},
                                 {k:v for k,v in row.items() if k not in IDS})
            issue.write_text(",".join((*FEATURES,"future_burden"))+"\n")
            rejected=subprocess.run(cmd,capture_output=True,text=True)
            self.assertNotEqual(rejected.returncode,0)
            self.assertIn("unexpected input columns",rejected.stderr)
        print("exact whole-realization split; isolated forecasts reproduced without labels, "
              "truth config or oracle; ID-only mutation invariant",flush=True)


if __name__=="__main__":
    OUT=Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
