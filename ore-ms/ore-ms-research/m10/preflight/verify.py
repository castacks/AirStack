"""Independent geometry, input-isolation, physical and scoring references."""
import argparse
from collections import defaultdict
import csv
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = Path(__file__).resolve().parent
CFG = json.loads((HERE/"config.json").read_text())


def read(path):
    with path.open() as file:
        return list(csv.DictReader(file))


def require(ok, *context):
    if not ok:
        raise AssertionError(context)


def rectangle_reference(patches, width, height):
    """Invert projection in image space and paint integer center samples far to near."""
    projected = set()
    surface = {}
    for identity, _, left, top, w, h, z in sorted(patches, key=lambda patch: -patch[-1]):
        pixels = {(x,y) for x in range(max(0,left),min(width,left+w))
                  for y in range(max(0,top),min(height,top+h))}
        if identity == "target":
            projected = pixels
        surface.update(dict.fromkeys(pixels, identity))
    hit = {point for point, identity in surface.items() if identity == "target"}
    reference = np.zeros((height,width,2),dtype=np.uint8)
    identities = ["background","target","foreground","distractor"]
    for (x,y), identity in surface.items():
        reference[y,x] = ((x,y) in projected, identities.index(identity))
    return projected, hit, reference


def geometry_and_controls(root):
    width,height = CFG["grid"]
    arrays = [np.load(root/p,mmap_mode="r",allow_pickle=False) for p in
              ("scoring/geometry.npy","inference/codes.npy","inference/masks.npy")]
    truth,codes,masks = arrays
    labels = read(root/"scoring/labels.csv")
    observations = read(root/"inference/frames.csv")
    scenes = read(root/"scoring/scene.csv")
    n = 512*len(CFG["cases"])
    require(len(labels)==len(observations)==len(scenes)==n, "row counts")
    require(truth.shape==(n,height,width,2) and codes.shape==masks.shape==(n,height,width,4), "shape")
    by_realization = defaultdict(dict)
    counts = defaultdict(int)
    max_delta = 0.
    min_transmission = 1.
    for i,(lab,obs,scene) in enumerate(zip(labels,observations,scenes)):
        require(lab["id"]==obs["id"]==scene["id"]==str(i), "opaque join",i)
        case,realization = lab["case"],int(lab["realization"])
        lo,hi = CFG["split_bounds"][obs["split"]]
        require(lo<=realization<hi and case not in by_realization[realization], "split/unique", i)
        require(obs["rgb_ms"]=="2000" and obs["lwir_ms"]==("1900" if case=="timing_offset" else "2000"), "time",i)
        require(obs["registration_valid"]==str(int(case!="registration_known")), "registration flag",i)
        require(obs["range_m"]==("" if case=="unknown_range" else "10") and
                obs["material_qualified"]==str(int(case!="unknown_material")), "constraints",i)
        expected_geometry = {"absent":(0,0,"absent"), "hidden":(48,0,"hidden"),
                             "distractor":(0,0,"distractor"), "contrast_without_support":(0,0,"distractor"),
                             "empty_support":(0,0,"empty_support"), "partial":(48,24,"feasible"),
                             "boundary_four_quarter":(16,4,"feasible"),
                             "boundary_three_quarter":(12,3,"insufficient_partial"),
                             "boundary_four_below":(20,4,"insufficient_partial")}
        target, hit, expected = rectangle_reference(json.loads(scene["patches"]),width,height)
        p,v = len(target),len(hit)
        physical = expected_geometry.get(case,(48,48,"feasible"))
        require((p,v,lab["negative"])==physical, "scenario geometry",i,physical,(p,v,lab["negative"]))
        require(np.array_equal(truth[i],expected), "projection or first-hit",i,np.count_nonzero(truth[i]!=expected))
        feasible = v >= 4 and p > 0 and Fraction(v,p) >= Fraction(1,4)
        require((int(lab["projected"]),int(lab["visible"]),lab["feasible"])==(p,v,str(int(feasible))),
                "counts or feasibility",i,(p,v,feasible),lab)
        require((float(lab["fraction"])==float(Fraction(v,p))) if p else not lab["fraction"],
                "fraction",i,lab)
        if v:
            for axis,field in enumerate(("x","y")):
                centroid = float(Fraction(sum(pt[axis] for pt in hit),v))
                delta = abs(float(lab[field])-centroid)
                require(delta<=1e-12,"centroid",i,field,centroid,lab[field])
                max_delta = max(max_delta,delta)
        else:
            require(not lab["x"] and not lab["y"],"empty centroid",i,lab)
        if case=="hidden":
            min_transmission = min(min_transmission,float(scene["foreground_rgb_T550"]))
        if case in ("missing_rgb","missing_lwir","missing_both"):
            channels = {"missing_rgb":slice(0,3),"missing_lwir":slice(3,4),"missing_both":slice(0,4)}[case]
            require(not masks[i,...,channels].any() and not codes[i,...,channels].any(),"missing codes",i)
        require(np.all(codes[i]<=4095) and np.array_equal(masks[i]==2,
                ((codes[i]==0)|(codes[i]==4095))&(masks[i]!=0)), "detector rails",i)
        by_realization[realization][case] = i
        counts[obs["split"]] += 1
    require(set(by_realization)==set(range(512)), "full split population")
    changed = {"blind_future":0,"shift_tau":0,"shift_innovation":0}
    for realization,cases in by_realization.items():
        require(set(cases)==set(CFG["cases"]),"case coverage",realization)
        ref = cases["reference"]
        for control in ("blind_future","distractor","unknown_range","unknown_material"):
            paired = cases[control]
            require(np.array_equal(codes[ref],codes[paired]) and np.array_equal(masks[ref],masks[paired]),
                    "paired identical codes and masks",control,ref,paired)
            if control in ("blind_future","distractor"):
                obs1,obs2 = observations[ref].copy(),observations[paired].copy()
                del obs1["id"],obs2["id"]
                require(obs1==obs2,"matched metadata",control,ref,paired)
        blind = cases["blind_future"]
        require(scenes[ref]["issue_burden"]==scenes[blind]["issue_burden"],"blind present burden",ref)
        changed["blind_future"] += scenes[ref]["future_burden"]!=scenes[blind]["future_burden"]
        for control in ("shift_tau","shift_innovation"):
            changed[control] += scenes[ref]["issue_burden"]!=scenes[cases[control]]["issue_burden"]
        for control in ("registration_known","registration_hidden"):
            shifted = cases[control]
            require(np.array_equal(codes[ref,:,:-3,3],codes[shifted,:,3:,3]) and
                    np.array_equal(codes[ref,:,:,:3],codes[shifted,:,:,:3]),"uncorrected shift",ref,shifted)
    require(changed==dict.fromkeys(changed,512),"distinct shifts",changed)
    require(min_transmission>.49,"hidden transmission",min_transmission)
    return by_realization,max_delta,min_transmission,dict(counts),changed


def isolation_and_frontends(root, by_realization):
    pred = {(r["id"],r["arm"]): r for r in read(root/"predictions.csv")}
    require(len(pred)==512*len(CFG["cases"])*3,"candidate coverage")
    for group in by_realization.values():
        for arm in ("B01","B02","B03"):
            ref = pred[str(group["reference"]),arm]
            for ctrl in ("distractor","blind_future","unknown_range","unknown_material"):
                other = pred[str(group[ctrl]),arm]
                require({k:v for k,v in ref.items() if k!="id"}=={k:v for k,v in other.items() if k!="id"},
                        "opaque ID or metadata influence",ref,other)
            require(not pred[str(group["missing_both"]),arm]["x"],"missing both",group["missing_both"],arm)
            require(not pred[str(group["hidden"]),arm]["x"],"hidden foreground",group["hidden"],arm)
            require(not pred[str(group["clipped"]),arm]["x"],"saturation abstention",group["clipped"],arm)
        require(pred[str(group["crossover"]),"B01"]["x"] and not pred[str(group["crossover"]),"B02"]["x"],
                "crossover",group["crossover"])
        for ctrl in ("registration_known","timing_offset"):
            require(pred[str(group[ctrl]),"B02"]["reason"]=="unavailable" and
                    pred[str(group[ctrl]),"B03"]["reason"]=="fallback_rgb","known acquisition control",group[ctrl])
        require(pred[str(group["registration_hidden"]),"B03"]["reason"]=="conflict","blind registration",group)
    with tempfile.TemporaryDirectory() as temp:
        sandbox = Path(temp)
        for file in ("baseline.py","config.json"):
            shutil.copy(HERE/file,sandbox)
        obsdir = sandbox/"inference"
        obsdir.mkdir()
        frame = read(root/"inference/frames.csv")[by_realization[0]["reference"]]
        frame["id"] = "0"
        fields = list(frame)
        width,height = CFG["grid"]
        codes = np.full((1,height,width,4),1000,dtype=np.uint16)
        codes[0,2:4,3:5,:3] = 1320
        codes[0,2:4,3:5,3] = 1500
        np.save(obsdir/"codes.npy",codes)
        np.save(obsdir/"masks.npy",np.ones_like(codes,dtype=np.uint8))
        frame_file = obsdir/"frames.csv"

        def run_frame(fieldnames):
            with frame_file.open("w") as file:
                writer = csv.DictWriter(file,fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow(frame)
            return subprocess.run([sys.executable,str(sandbox/"baseline.py"),str(obsdir),str(sandbox/"pred.csv")],
                                  capture_output=True,text=True,check=False)

        require(run_frame(fields).returncode==0,"standalone clean inference")
        before = (sandbox/"pred.csv").read_bytes()
        fused = read(sandbox/"pred.csv")[2]
        score = (320/4095/.035+500/4095/.045)/8
        require((float(fused["x"]),float(fused["y"]),int(fused["support"]),fused["reason"])==(3.5,2.5,4,"equal")
                and abs(float(fused["score"])-score)<1e-12,"independent equal-map fixture",fused,score)
        (sandbox/"scoring").mkdir()
        (sandbox/"scoring/labels.csv").write_text("id,x,y,feasible\n0,32,24,0\n")
        require(run_frame(fields).returncode==0 and (sandbox/"pred.csv").read_bytes()==before,
                "mutated privileged labels affected inference")
        frame["lwir_ms"] = "2001"
        require(run_frame(fields).returncode==0 and read(sandbox/"pred.csv")[1]["reason"]=="unavailable",
                "future acquisition accepted")
        for prohibited in ("visibility","oracle","future_state","case","actual_range","first_hit","true_x"):
            frame[prohibited] = "1"
            rejected = run_frame([*fields,prohibited])
            require(rejected.returncode!=0 and "unapproved inference schema" in rejected.stderr,
                    "forbidden field accepted",prohibited)
            del frame[prohibited]
    return len(pred)


def radiance_reference():
    import corpus
    from adapters import Segment, radiance
    config = {"rgb_ambient":.009,"rgb_solid_angle":.8,
              "rgb_response":dict(zip("RGB",corpus.RGB_TRIANGLES)),
              "lwir_response":(.65,1,.72),"lwir_intervals":192}
    error = 0.
    cases = 0
    for material,z in (("target",10),("foreground",5),("crossover",10),("background",15)):
        rho,temperature,emissivity,reflected = corpus.MATERIALS[material]
        spec = dict(irradiance=[.018]*3,rho=rho,temperature_K=temperature,
                    emissivity=emissivity,reflected_K=reflected)
        for ext,absorb,air in ((0,0,294),(.07,.014,294),(.17,.056,318)):
            for x,y in ((16,12),(4,8),(30,21)):
                length = z*math.sqrt(1+((x-16)/20)**2+((y-12)/20)**2)
                layer = Segment(0,length,z/2,0,air,.12*ext,.88*ext,.5,(.7*absorb,absorb,.65*absorb))
                rgb,lwir = radiance(config,spec,[layer])
                expected = np.array([*(sum(pair) for pair in rgb),sum(lwir)])
                actual = corpus.material_image(material,z,ext,absorb,air)[y,x]
                error = max(error,float(np.max(np.abs(expected-actual))))
                cases += 1
    require(error<=1e-10,"M06 scalar parity",error)
    return cases,error


def independent_scoring(root):
    import statistics
    lock_file = root/"calibration.json"
    lock = json.loads(lock_file.read_text())
    output = json.loads((root/"holdout.json").read_text())
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    require(sha(lock_file)==output["calibration_sha256"]==(root/"calibration.sha256").read_text().strip(),"lock hash")
    require(sha(HERE/"config.json")==lock["config_sha256"],"config hash")
    for file, expected in lock["method_sha256"].items():
        require(sha(HERE/file)==expected,"method provenance",file)
    by_arm = {}
    development = []
    for split in ("development","calibration","holdout"):
        folder = root/("partitions" if split=="holdout" else "calibration_only")
        files = [folder/f"{split}_{kind}.csv" for kind in ("labels","predictions")]
        if split!="holdout":
            for file in files:
                require(lock["input_sha256"][file.name]==sha(file),"calibration provenance",file.name)
        truth = {r["id"]:r for r in read(files[0])}
        source = read(files[1])
        for arm in ("B01","B02","B03"):
            samples = []
            for p in source:
                if p["arm"]!=arm:
                    continue
                t = truth[p["id"]]
                feasible = t["feasible"]=="1"
                score = Fraction(p["score"])
                candidate = bool(p["x"])
                good = feasible and candidate and sum((Fraction(p[f])-Fraction(t[f]))**2 for f in ("x","y"))<=1
                samples.append((t,score,candidate,good))
            if split!="holdout":
                sweep = []
                for cutoff in CFG["thresholds"]:
                    issued = [t for t,s,c,g in samples if c and s>=Fraction(str(cutoff))]
                    primary = [sum(t["negative"]==name for t in issued) for name in ("absent","hidden")]
                    upper = [wilson_root(k,128) for k in primary]
                    good = sum(g and s>=Fraction(str(cutoff)) for _,s,_,g in samples)
                    sweep.append((all(u<=.05 for u in upper),good,-sum(primary),cutoff))
                verified = max([t for t in sweep if t[0] and t[1]],default=sweep[-1])
                stage = lock[split][arm]
                require(verified[3]==stage["chosen"]["cutoff"] and verified[1]==stage["chosen"]["correct"],
                        "independent threshold",split,arm,verified)
                if split=="development":
                    development.append((verified[1],verified[2],arm))
                if split=="calibration":
                    for b,probability in enumerate(stage["mapping"]):
                        events = [g for _,s,c,g in samples if c and min(int(10*s),9)==b]
                        require(probability==(sum(events)+1)/(len(events)+2),"bin mapping",arm,b)
                continue
            stage = output["arms"][arm]
            cutoff = Fraction(str(stage["cutoff"]))
            require(stage["cutoff"]==lock["calibration"][arm]["chosen"]["cutoff"],"holdout threshold",arm)
            mapping = lock["calibration"][arm]["mapping"]
            measures = {case:np.zeros((256,2),dtype=int) for case in CFG["cases"]}
            for case in ["overall",*CFG["cases"]]:
                cell = [r for r in samples if case=="overall" or r[0]["case"]==case]
                issued = [t for t,s,c,g in cell if c and s>=cutoff]
                metric = stage["overall"] if case=="overall" else stage["cases"][case]
                good = sum(g and s>=cutoff for _,s,_,g in cell)
                require((len(cell),len(issued),good)==(metric["n"],metric["issued"],metric["correct"]),
                        "independent scoring",arm,case)
                losses = [(mapping[min(int(10*s),9)]-g)**2 for _,s,c,g in cell if c]
                require((abs(statistics.mean(losses)-metric["brier_candidate"])<=1e-12 if losses
                         else metric["brier_candidate"] is None),"candidate Brier",arm,case)
                for category, observed in metric["negative"].items():
                    k = sum(r["negative"]==category for r in issued)
                    n = sum(t["negative"]==category for t,_,_,_ in cell)
                    require((k,n)==(observed["false_supported"],observed["n"]) and
                            abs(wilson_root(k,n)-observed["upper95"])<=1e-12,"Wilson root",arm,case,category)
            for t,s,c,g in samples:
                measures[t["case"]][int(t["realization"])-256] += (t["feasible"]=="1",g and s>=cutoff)
            measures["overall"] = sum(measures.values())
            by_arm[arm] = measures
    comparator = output["development_comparator"]
    require(comparator==lock["development_comparator"]==
            max(development,key=lambda item:item[:2])[2],"development ranking")
    rand = np.random.default_rng(CFG["bootstrap_seed"])
    weights = np.stack([np.bincount(rand.integers(0,256,256),minlength=256)
                        for _ in range(CFG["bootstrap_draws"])])
    delta = 0.
    for case in ["overall",*CFG["cases"]]:
        estimates = output["bootstrap"] if case=="overall" else output["stratum_bootstrap"][case]
        counts = {arm:weights@measure[case] for arm,measure in by_arm.items()}
        if not counts[comparator][:,0].any():
            require(estimates is None,"undefined interval",case)
            continue
        performances = {arm:a[:,1]/a[:,0] for arm,a in counts.items()}
        for arm,p in performances.items():
            for key, series in (("correct_per_feasible_ci95",p),
                                ("gain_vs_development_comparator_ci95",p-performances[comparator])):
                q = statistics.quantiles(series,n=40,method="inclusive")
                delta = max(delta,abs(q[0]-estimates[arm][key][0]),abs(q[-1]-estimates[arm][key][1]))
    require(delta<=1e-12,"independent bootstrap",delta)
    return delta


def wilson_root(k,n):
    """Solve the inverted score-test inequality by bisection."""
    low,high = k/n,1.
    for _ in range(56):
        mid = (high+low)/2
        if n*(k/n-mid)**2 <= 1.96**2*mid*(1-mid):
            low = mid
        else:
            high = mid
    return (high+low)/2


def main(root):
    groups,centroid,transmission,counts,changed = geometry_and_controls(root)
    predictions = isolation_and_frontends(root,groups)
    probes,physical = radiance_reference()
    interval = independent_scoring(root)
    print(json.dumps({"partition_cases":counts,"label_mismatches":0,"centroid_max_px":centroid,
        "hidden_foreground_min_T550":transmission,"process_changes":changed,"prediction_rows":predictions,
        "identical_pairs_per_control":len(groups),"forbidden_fields_rejected":7,
        "radiance_probes":probes,"max_scalar_radiance_difference":physical,
        "max_bootstrap_endpoint_difference":interval,"independent_scoring":"PASS"},indent=2))


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root",type=Path)
    main(parser.parse_args().root)
