"""Current-evidence reliability model and deterministic candidate policy."""
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
P = json.loads((HERE/"protocol.json").read_text())
OBS = ("id","split","rgb_ms","lwir_ms","registration_valid","range_m","material_qualified")
FROZEN = ("id","arm","available_rgb","available_lwir","x","y","support","score","reason")
OUTPUT = ("id","arm","available_rgb","available_lwir","x","y","support","score","reason","warning","rgb_weight")
ARMS = ("B04","A01","A04","A05","A06","A07","A08","A09")
DECISION = ("id","arm","status","x","y","p_correct","reason")


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    support: int
    score: float
    valid: float
    rail: float

    def features(self):
        cap = math.log1p(P["width"]*P["height"])
        return np.array([1,*(2*x-1 for x in (self.score,math.log1p(self.support)/cap,self.valid,self.rail))])


def table(path, schema=None):
    with path.open() as file:
        reader = csv.DictReader(file)
        if schema is not None and tuple(reader.fieldnames or ()) != schema:
            raise ValueError(f"unapproved schema in {path.name}")
        return list(reader)


def digest(path):
    import hashlib
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+"\n")


def features(source):
    """The join key is kept outside the feature tuple passed to the policy."""
    records = table(source/"frames.csv",OBS)
    predictions = table(source/"candidates.csv",FROZEN)
    masks = np.load(source/"masks.npy",mmap_mode="r",allow_pickle=False)
    if (masks.shape != (len(records),P["height"],P["width"],4) or
            masks.dtype != np.uint8 or len(predictions) != 2*len(records)):
        raise ValueError("observation dimensions")
    for i, row in enumerate(records):
        if not (row["id"].isdigit() and row["registration_valid"] in ("0","1") and
                row["material_qualified"] in ("0","1")):
            raise ValueError("invalid observation metadata")
        ages = [P["decision_ms"]-int(row[field]) for field in ("rgb_ms","lwir_ms")]
        mask = masks[i]
        if mask.max() > 2:
            raise ValueError("invalid detector mask")
        valid = ((mask[...,:3]==1).all(axis=-1),mask[...,3]==1)
        rail = ((mask[...,:3]==2).any(axis=-1),mask[...,3]==2)
        eligible = tuple(int(0<=age<=20 and v.any() and (j==0 or row["registration_valid"]=="1"))
                         for j,(age,v) in enumerate(zip(ages,valid)))
        found = []
        for j, p in enumerate(predictions[2*i:2*i+2]):
            if ((p["id"],p["arm"]) != (row["id"],f"B0{j+1}") or
                    tuple(int(p[k]) for k in ("available_rgb","available_lwir")) != eligible):
                raise ValueError("incompatible frozen frontend")
            if not p["x"]:
                found.append(None)
                continue
            x,y,score,n = float(p["x"]),float(p["y"]),float(p["score"]),int(p["support"])
            if not (eligible[j] and 0<=x<P["width"] and 0<=y<P["height"] and
                    4<=n<=P["width"]*P["height"] and 0<=score<=1):
                raise ValueError("unqualified candidate")
            found.append(Candidate(x,y,n,score,float(valid[j].mean()),float(rail[j].mean())))
        yield row["id"], eligible, tuple(found)


def sigmoid(x):
    return 1/(1+np.exp(-np.clip(x,-30,30)))


def fit_logit(x,y):
    weights = np.zeros(x.shape[1])
    penalty = np.diag([0]+[P["ridge"]]*(x.shape[1]-1))
    for step in range(1,P["iterations"]+1):
        probs = sigmoid(x@weights)
        gradient = x.T@(probs-y)+penalty@weights
        if np.linalg.norm(gradient,np.inf) <= P["fit_tolerance"]:
            return weights.tolist(), step, float(np.linalg.norm(gradient,np.inf))
        hessian = (x.T*(probs*(1-probs)))@x+penalty
        weights -= np.linalg.solve(hessian,gradient)
    raise RuntimeError("development fit did not converge")


def correct(candidate, label):
    return (candidate is not None and label["feasible"]=="1" and
            math.hypot(candidate.x-float(label["x"]),candidate.y-float(label["y"]))<=1)


def pair_type(a,b):
    return "associated" if math.hypot(a.x-b.x,a.y-b.y)<=P["association_px"] else "conflict"


def fit(source, label_file, output):
    labels = {r["id"]:r for r in table(label_file)}
    if {int(r["realization"]) for r in labels.values()} != set(range(128)) or len(labels)!=3456:
        raise ValueError("model fitting requires exactly the development population")
    designs = [[],[]]
    events = [[],[]]
    pairs = {"associated":np.zeros((2,2),dtype=int),"conflict":np.zeros((2,2),dtype=int)}
    observed = set()
    for key,_,(a,b) in features(source):
        row = labels[key]
        observed.add(key)
        outcomes = [int(correct(c,row)) for c in (a,b)]
        for j,c in enumerate((a,b)):
            if c is not None:
                designs[j].append(c.features())
                events[j].append(outcomes[j])
        if a is not None and b is not None:
            pairs[pair_type(a,b)][tuple(outcomes)] += 1
    if observed != labels.keys():
        raise ValueError("incomplete development stages")
    fits = []
    for x,y in zip(designs,events):
        weights,iterations,gradient = fit_logit(np.asarray(x),np.asarray(y))
        fits.append({"coefficients":weights,"iterations":iterations,"gradient_inf":gradient,
                     "constant":(sum(y)+1)/(len(y)+2),"n":len(y),"correct":sum(y)})
    dependence = {}
    for name,counts in pairs.items():
        n = int(counts.sum())
        residual = (counts[0,0]/n-counts[0,:].sum()*counts[:,0].sum()/n**2)*n/(n+P["dependence_shrinkage"]) if n else 0.
        dependence[name] = {"counts":counts.tolist(),"shrunken_joint_failure_residual":float(residual)}
    dump(output,{"models":fits,"dependence":dependence,"groups":128,"label_sha256":digest(label_file),
                 "observation_sha256":{f:digest(source/f) for f in ("frames.csv","masks.npy","candidates.csv")}})


def crop_weight(a,b):
    lo = P["minimum_weight"]
    return float(np.clip(a/(a+b) if a+b else .5,lo,1-lo))


def weighted(a,b,weight,score,reason):
    return (weight*a.x+(1-weight)*b.x,weight*a.y+(1-weight)*b.y,min(a.support,b.support),
            score,reason,0,weight)


def single(c, index, score, reason, warning=0):
    return c.x,c.y,c.support,score,reason,warning,1-index


def abstain(reason,warning=0):
    return "","",0,0,reason,warning,""


def joint_success(a,b,residual):
    # Fréchet bounds keep the modeled paired failure compatible with the marginals.
    u,v = 1-a,1-b
    failure = min(min(u,v),max(max(0,u+v-1),u*v+residual))
    return 1-failure


def propose(pair, model):
    a,b = pair
    models = model["models"]
    if a is None and b is None:
        return dict.fromkeys(ARMS,abstain("no_candidate"))
    probs = [float(sigmoid(c.features()@m["coefficients"])) if c is not None else 0
             for c,m in zip(pair,models)]
    constants = [m["constant"] for m in models]
    if a is None or b is None:
        i = int(a is None)
        reason = ("fallback_rgb","fallback_lwir")[i]
        primary = single(pair[i],i,probs[i],reason)
        out = dict.fromkeys(ARMS,primary)
        out["B04"] = single(pair[i],i,constants[i],reason)
        out["A08"] = single(pair[i],i,pair[i].score,reason)
        return out
    if pair_type(a,b)=="conflict":
        i = int(probs[1]>probs[0])
        resolved = probs[i]>=P["conflict_min_probability"] and probs[i]-probs[1-i]>=P["conflict_min_margin"]
        primary = (single(pair[i],i,probs[i],("conflict_rgb","conflict_lwir")[i],1)
                   if resolved else abstain("conflict",1))
        out = dict.fromkeys(ARMS,primary)
        out["B04"] = out["A08"] = abstain("conflict",1)
        return out
    residual = model["dependence"]["associated"]["shrunken_joint_failure_residual"]
    weight = crop_weight(*probs)
    primary = weighted(a,b,weight,joint_success(*probs,residual),"adaptive")
    out = dict.fromkeys(ARMS,primary)
    out["B04"] = weighted(a,b,crop_weight(*constants),sum(constants)/2,"constant")
    out["A08"] = weighted(a,b,.5,(a.score+b.score)/2,"equal_ablation")
    out["A09"] = weighted(a,b,weight,joint_success(*probs,0),"independent_ablation")
    return out


def inference(source, state, output):
    model = json.loads(state.read_text())
    with output.open("w",newline="") as file:
        writer = csv.writer(file,lineterminator="\n")
        writer.writerow(OUTPUT)
        for key,available,pair in features(source):
            writer.writerows((key,arm,*available,*decision) for arm,decision in propose(pair,model).items())


def qualified(proposal, rule):
    x,y,_,score,reason,_,_ = proposal
    if x=="":
        return "abstain","","","",reason
    probability = rule["mapping"][min(int(score*P["bins"]),P["bins"]-1)]
    if probability is None:
        return "abstain","","","","unqualified_score_bin"
    if score<rule["cutoff"]:
        return "abstain","","",probability,"below_cutoff"
    return "issue",x,y,probability,reason


def issue(source, runtime, output):
    state = json.loads(runtime.read_text())
    if set(state["qualification"])!=set(ARMS):
        raise ValueError("incomplete qualification")
    with output.open("w",newline="") as file:
        writer = csv.writer(file,lineterminator="\n")
        writer.writerow(DECISION)
        for key,_,pair in features(source):
            rows = propose(pair,state["model"])
            writer.writerows((key,arm,*qualified(proposal,state["qualification"][arm]))
                             for arm,proposal in rows.items())
