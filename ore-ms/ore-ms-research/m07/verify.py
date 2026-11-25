"""Verify observable burden recovery, uncertainty, ambiguity, and data separation."""
from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
import shutil
import statistics
import subprocess
import sys
import tempfile
import unittest

from estimate import MODES, RESULTS, estimate
from features import FEATURES, KEYS, project_m06, rows
from forward import CASES, CONFIG, HERE, black_band, context, ideal
from observability import audit, oracle
from reference import inverse_band, rgb_band, wilson

ROOT = HERE.parent
METRICS = ("corpus","case","split","mode","n","emitted","informed","weak","mismatch",
           "abstained","failure_rate","bias","mae","rmse","max_abs","mean_relative_when_truth_ge_0p05",
           "coverage95","wilson_low","wilson_high","mean_width95")


def key(row):
    return tuple(row[k] for k in KEYS)


def index(records, make_key):
    records = list(records)
    indexed = {make_key(row):row for row in records}
    if len(indexed) != len(records):
        raise ValueError("duplicate record")
    return indexed


def evaluate(estimates, labels):
    groups = defaultdict(list)
    for row in estimates.values():
        groups[row["corpus"],row["case"],row["split"],row["mode"]].append(row)
    metrics = {}
    with (OUT/"metrics.csv").open("w",newline="") as file:
        out = csv.DictWriter(file, METRICS, lineterminator="\n")
        out.writeheader()
        for ident, group in sorted(groups.items()):
            counts = Counter(row["status"] for row in group)
            result = dict(zip(METRICS[:4],ident,strict=True))
            n = len(group)
            result.update(n=n, informed=counts["informed"], weak=counts["weak"],
                          mismatch=counts["mismatch"], abstained=counts["abstain"])
            result["failure_rate"] = sum(counts[k] for k in ("weak","mismatch","abstain"))/n
            emitted = [r for r in group if r["estimate"]]
            result["emitted"] = len(emitted)
            errors, relative, width = [], [], []
            covered = 0
            for row in emitted:
                truth = float(labels[key(row)]["burden"])
                b, low, high = (float(row[k]) for k in ("estimate","lower95","upper95"))
                if not 0 <= low <= high <= 1 or not 0 <= b <= 1:
                    raise ValueError("invalid posterior output")
                errors.append(b-truth)
                if truth >= .05:
                    relative.append(abs(b-truth)/truth)
                width.append(high-low)
                covered += low <= truth <= high
            if errors:
                result.update(bias=statistics.fmean(errors),
                              mae=statistics.fmean(map(abs,errors)),
                              rmse=math.sqrt(statistics.fmean(x*x for x in errors)),
                              max_abs=max(map(abs,errors)),coverage95=covered/len(errors),
                              mean_width95=statistics.fmean(width))
                if relative:
                    result["mean_relative_when_truth_ge_0p05"]=statistics.fmean(relative)
                # Only the prespecified matched calibration arm draws independent
                # states from the declared prior and noise model.
                if ident[:2]==("M07","matched"):
                    result["wilson_low"],result["wilson_high"]=wilson(covered,len(errors))
            out.writerow(result)
            metrics[ident]=result
    return metrics


class Verification(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.features=index(rows(OUT/"features.csv",FEATURES),key)
        cls.labels=index(rows(OUT/"labels.csv",(*KEYS,"burden")),key)
        cls.outputs=index(rows(OUT/"estimates.csv",RESULTS),lambda row:(key(row),row["mode"]))
        if set(cls.labels)!=set(cls.features):
            raise ValueError("feature/label population mismatch")
        if set(cls.outputs)!={(ident,mode) for ident in cls.features for mode in MODES}:
            raise ValueError("estimate population mismatch")
        if not all(0<=float(row["burden"])<=1 for row in cls.labels.values()):
            raise ValueError("invalid truth")
        cls.metrics=evaluate(cls.outputs,cls.labels)

    def result(self, mode, case="matched", corpus="M07", split="holdout"):
        return self.metrics[corpus,case,split,mode]

    def test_analytic_inverse_ambiguities_and_information(self):
        inverse, equivalent, epsilon = oracle(OUT/"oracle.csv",black_band)
        self.assertLessEqual(inverse,2e-12)
        self.assertLessEqual(equivalent,2e-10)
        self.assertTrue(0 < epsilon < .8)
        errors=[]
        for b in (.1,.5,.9):
            emitted=ideal(*context(CASES["matched"]),b)[0]
            retrieved=inverse_band(lambda x:rgb_band(CONFIG,x),emitted)
            errors.append(abs(b-retrieved))
        self.assertLessEqual(max(errors),2e-10)
        levels, information, difference=audit(OUT/"observability.csv")
        self.assertEqual(levels,dict(rgb_known=1,lwir_known=1,joint_known=1,
            unknown_range_one_band=1,unknown_range_two_bands=2,lwir_crossover=0,
            target_occluded=0,rail_plateau=0))
        self.assertLessEqual(difference,2e-6)
        total=information["rgb_known"]+information["lwir_known"]
        self.assertLessEqual(abs(information["joint_known"]/total-1),2e-10)
        print(f"inverse error={inverse:.9g}; band inverse max={max(errors):.9g}; "
              f"ambiguity output residual={equivalent:.9g}; Jacobian error={difference:.9g}; "
              f"scalar information={information}",flush=True)

    def test_matched_holdout_calibration_and_modes(self):
        for mode in ("RGB","LWIR","joint"):
            result=self.result(mode)
            self.assertEqual((result["n"],result["abstained"]),(256,0))
            self.assertLessEqual(abs(result["bias"]),.08 if mode=="LWIR" else .035)
            self.assertLessEqual(result["mae"],.21 if mode=="LWIR" else .065)
            self.assertTrue(.88<=result["coverage95"]<=.995)
        self.assertLessEqual(self.result("joint")["mae"],self.result("RGB")["mae"]+.02)
        self.assertLessEqual(self.result("RGB","ensemble_fixed","M06")["max_abs"],.006)
        widths = [(mode,self.result(mode)["mae"],self.result(mode)["coverage95"],
                   self.result(mode)["mean_width95"]) for mode in MODES]
        print(f"matched holdout (mode, MAE, coverage, width)={widths}",flush=True)
        # Diagnostic only: check whether the frozen grid can explain its
        # noiseless M06 residual warning without replacing the scored output.
        for ident, row in self.features.items():
            if row["case"] != "ensemble_fixed" or row["split"] != "holdout":
                continue
            result = self.outputs[ident,"RGB"]
            if result["status"] != "mismatch":
                continue
            center = float(result["estimate"])
            lsb = CONFIG["rgb_well"] / (CONFIG["rgb_gain"] * (2**CONFIG["bits"]-1))
            residual = min(
                max(abs(int(row[f"{channel}_code"]) - rgb_band(CONFIG,b,channel)/lsb)
                    for channel in "RGB")
                for b in (center + i*1e-5 for i in range(-180,181))
                if 0 <= b <= 1)
            print(f"M06 frozen-grid warning realization={row['realization']} t={row['time_s']}; "
                  f"grid z={result['min_max_residual']}; "
                  f"independent finer local search residual={residual:.6g} LSB",flush=True)

    def test_degraded_ambiguity_and_missingness(self):
        for case in ("partial_blocked","complete","saturated","uncertain_geometry","uncertain_material"):
            for mode in ("RGB","LWIR","joint"):
                result=self.result(mode,case)
                self.assertEqual((result["n"],result["abstained"],result["emitted"]),(256,256,0))
        self.assertEqual(self.result("LWIR","crossover")["abstained"],256)
        self.assertEqual(self.result("RGB","severe")["weak"]+self.result("RGB","severe")["abstained"],256)
        self.assertEqual(self.result("RGB","missing_rgb")["abstained"],256)
        self.assertEqual(self.result("LWIR","invalid_lwir")["abstained"],256)
        self.assertEqual(self.result("joint","missing_rgb")["abstained"],0)
        self.assertEqual(self.result("joint","invalid_lwir")["abstained"],0)
        sample=next(row for row in self.features.values() if row["case"]=="matched")
        all_missing=sample|{k:"" for k in FEATURES[-4:]}
        self.assertEqual(estimate(all_missing,"joint")[0],"abstain")
        matched, mismatch = self.result("joint"),self.result("joint","mismatched_air")
        self.assertGreater(mismatch["mae"],matched["mae"])
        for ident, row in self.features.items():
            if row["case"]=="ensemble_fixed":
                self.assertEqual(self.outputs[ident,"LWIR"]["status"],"abstain")
                fields=("estimate","lower95","upper95","sd","local_info","status")
                self.assertEqual([self.outputs[ident,"RGB"][k] for k in fields],
                                 [self.outputs[ident,"joint"][k] for k in fields])
        print(f"severe RGB weak/abstain=256; mismatch joint MAE={mismatch['mae']:.8g}; "
              f"coverage={mismatch['coverage95']:.8g}; warnings={mismatch['mismatch']}/256",flush=True)

    def test_leakage_and_time_partition(self):
        populations=defaultdict(set)
        timed=defaultdict(set)
        for row in self.features.values():
            cohort=row["corpus"],row["split"]
            populations[cohort].add(row["realization"])
            timed[row["corpus"],row["realization"],row["case"]].add(row["time_s"])
        for corpus, sizes in (("M07",(128,256)),("M06",(96,32))):
            a,b=(populations[corpus,p] for p in ("development","holdout"))
            self.assertEqual((len(a),len(b)),sizes)
            self.assertFalse(a&b)
        for (corpus,_,_), times in timed.items():
            self.assertEqual(times,{"0","6","12"} if corpus=="M06" else {"0"})

        mock_pair={field:"1" for field in FEATURES[-4:]}|dict(case="ensemble_fixed",
                         realization="1",time_s="6",burden_avg=".1",rgb_tau550=".8",first_range_m="10")
        frame={"split":"development","burden_coefficients":"[0]"}
        edited=mock_pair|dict(burden_avg=".99",rgb_tau550=".1",first_range_m="99")
        self.assertEqual(project_m06(mock_pair,frame),
                         project_m06(edited,frame|{"burden_coefficients":"[999]"}))
        # Deploy only runtime files. Neither label nor private generation inputs exist.
        with tempfile.TemporaryDirectory() as directory:
            sandbox=Path(directory)
            for name in ("m02/rgb_model.py","m03/thermal_model.py","m04/geometry.py",
                         "m05/environment.py","m06/adapters.py","m07/estimate.py","m07/features.py",
                         "m07/forward.py","m07/config.json"):
                destination=sandbox/name
                destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copyfile(ROOT/name,destination)
            input_file=sandbox/"features.csv"
            sample=next(row for row in self.features.values() if row["case"]=="matched")
            with input_file.open("w",newline="") as file:
                writer=csv.DictWriter(file,FEATURES)
                writer.writeheader()
                writer.writerow(sample)
            output=sandbox/"estimates.csv"
            cmd=[sys.executable,str(sandbox/"m07"/"estimate.py"),str(input_file),str(output)]
            subprocess.run(cmd,check=True)
            for row in rows(output,RESULTS):
                self.assertEqual(row,self.outputs[key(row),row["mode"]])
            with input_file.open("w") as file:
                file.write(",".join((*FEATURES,"burden_avg"))+"\n")
            rejected=subprocess.run(cmd,capture_output=True,text=True)
            self.assertNotEqual(rejected.returncode,0)
            self.assertIn("unexpected input columns",rejected.stderr)
        print(f"{len(self.features)} feature rows, {len(self.outputs)} mode outputs; "
              "96 M06 holdout fixed frames; isolated inference reproduced without labels",flush=True)


if __name__=="__main__":
    OUT=Path(sys.argv.pop(1))
    unittest.main(verbosity=2)
