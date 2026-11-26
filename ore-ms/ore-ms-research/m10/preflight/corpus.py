"""Frozen PF1 scene construction and separate observation/truth stores."""
import argparse
import csv
from functools import lru_cache
import json
import math
from pathlib import Path
import random
import sys

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
sys.path.append(str(ROOT/"m06"))
from adapters import planck_um, profile
from environment import Field, advance, evaluate, logistic, stationary, transition
from geometry import Camera, Rectangle, hits

CFG=json.loads((HERE/"config.json").read_text())
W,H=CFG["grid"]
F=CFG["focal_px"]
CX,CY=(W-1)/2,(H-1)/2
CAMERA=Camera(W,H,F,F,CX,CY,(0,0,0),((1,0,0),(0,1,0),(0,0,1)))
RAYS=tuple(CAMERA.ray(x,y) for y in range(H) for x in range(W))
SLANT=np.array([1/d[2] for d in RAYS]).reshape(H,W)
MODEL=Field(64,(.2,.5,.3))
OBS=("id","split","rgb_ms","lwir_ms","registration_valid","range_m","material_qualified")
LABELS=("id","realization","case","projected","visible","fraction","x","y","feasible","negative")
TRUTH=("id","patches","issue_burden","future_burden","actual_lwir_shift","actual_air_K",
       "rgb_extinction_550","lwir_absorption_11","foreground_rgb_T550")
MATERIALS={
    "background":([.18,.18,.18],300,[1,1,1],285),
    "target":([.4,.4,.4],320,[.8,.8,.8],285),
    "foreground":([.12,.12,.12],285,[.9,.9,.9],290),
    "crossover":([.4,.4,.4],300,[1,1,1],285),
}
RGB_WAVE=np.arange(400,701,5)
RGB_TRIANGLES=((550,610,700),(450,540,630),(400,460,540))
RGB_WEIGHTS=np.array([np.maximum(0,np.minimum((RGB_WAVE-a)/(b-a),(c-RGB_WAVE)/(c-b)))
                      for a,b,c in RGB_TRIANGLES])
LW_WAVE=np.linspace(8,14,193)
LW_RESPONSE=np.array([profile(float(w),(.65,1,.72)) for w in LW_WAVE])
HIT_CODES={"background":0,"target":1,"foreground":2,"distractor":3}


def csv_writer(file, fields):
    writer=csv.writer(file,lineterminator="\n")
    writer.writerow(fields)
    return writer


def layout(case,x,y):
    patches=[("background","background",-40,-30,113,85,15)]
    if case in ("absent","distractor","contrast_without_support"):
        if case!="absent":
            w,h=(2,2) if case=="contrast_without_support" else (8,6)
            patches.append(("distractor","target",x,y,w,h,10))
        return tuple(patches)
    dims={"boundary_four_quarter":(4,4,3,4),"boundary_three_quarter":(3,4,3,3),
          "boundary_four_below":(4,5,4,4),"partial":(8,6,4,6),"hidden":(8,6,8,6)}
    w,h,ow,oh=dims.get(case,(8,6,0,0))
    if case=="empty_support":
        x=60
    patches.append(("target","crossover" if case=="crossover" else "target",x,y,w,h,10))
    if ow:
        patches.append(("foreground","foreground",x,y,ow,oh,5))
    return tuple(patches)


@lru_cache(maxsize=280)
def geometry(patches):
    posed=[]
    for ident,material,x,y,w,h,z in patches:
        center=((x+(w-1)/2-CX)*z/F,(y+(h-1)/2-CY)*z/F,z)
        size=((w/2-.01)*z/F,(h/2-.01)*z/F)
        item=Rectangle(ident,center,size,material,ident=="target")
        posed.append((item,center))
    result=np.empty((H*W,2),dtype=np.uint8)
    for i,ray in enumerate(RAYS):
        target,first,_=hits(CAMERA,ray,posed)
        result[i]=(int(target is not None),HIT_CODES[first.id])
    return result.reshape(H,W,2)


def state(init,innovation,dt,tau,sigma):
    class Given:
        def gauss(self,a,b):
            return next(values)
    values=iter(innovation)
    return advance(init,(0,)*5,transition(dt,tau,sigma),Given())


def physical(realization,case):
    rng=random.Random(CFG["seed"]+realization)
    start,innov,future=[stationary(rng,1,MODEL.size) for _ in range(3)]
    tau=2 if case=="shift_tau" else 6
    sigma=1.6 if case=="shift_innovation" else 1
    current=state(start,innov,2,tau,sigma)
    later=state(current,future,4,2 if case=="blind_future" else tau,
                1.6 if case=="blind_future" else sigma)
    present=logistic(evaluate(current,MODEL.basis(5,2,1)))
    future_b=logistic(evaluate(later,MODEL.basis(5,6,1)))
    return present,future_b


def medium(case,burden):
    effective=round(4*burden)/4
    extinction=.003+.135*effective
    absorption=.002+.024*effective
    air=318 if case=="wrong_air" else 294
    if case=="rgb_attenuated":
        extinction*=2.5
    if case=="lwir_attenuated":
        absorption*=4
    if case=="crossover":
        absorption=0
    return extinction,absorption,air


@lru_cache(maxsize=40)
def thermal_body(temperature):
    return np.array([planck_um(float(w),temperature) for w in LW_WAVE])


@lru_cache(maxsize=300)
def material_image(material,z,extinction,absorption,air):
    rho,temperature,emissivity,reflected=MATERIALS[material]
    length=SLANT[...,None]*z
    rgb_depth=extinction*(550/RGB_WAVE)**.5
    t=np.exp(-length*rgb_depth)
    reflection=np.interp(RGB_WAVE,[400,550,700],rho)
    source=.88*.009*.8/(4*math.pi)
    observed=t*(.018*reflection/math.pi)+(1-t)*source
    left,right=observed[...,:-1],observed[...,1:]
    rgb=5/6*(left@(2*RGB_WEIGHTS[:,:-1]+RGB_WEIGHTS[:,1:]).T+
             right@(RGB_WEIGHTS[:,:-1]+2*RGB_WEIGHTS[:,1:]).T)
    eps=np.interp(LW_WAVE,[8,11,14],emissivity)
    # Coefficients at 8/11/14 µm are 0.7/1/0.65 times the 11 µm value.
    alpha=np.interp(LW_WAVE,[8,11,14],np.array([.7,1,.65])*absorption)
    t=np.exp(-length*alpha)
    boundary=eps*thermal_body(temperature)+(1-eps)*thermal_body(reflected)
    spectrum=LW_RESPONSE*(t*boundary+(1-t)*thermal_body(air))
    lw=(spectrum[...,0]/2+spectrum[...,1:-1].sum(axis=-1)+spectrum[...,-1]/2)*6/192
    return np.concatenate((rgb,lw[...,None]),axis=-1)


def image(patches,truth,case,realization,m):
    radiance=np.zeros((H,W,4))
    for ident,material,*_,z in patches:
        select=truth[...,1]==HIT_CODES[ident]
        radiance[select]=material_image(material,z,*m)[select]
    rng=np.random.default_rng(CFG["seed"]+1_000_000+realization)
    sigmas=[65/6500]*3+[.8] if case=="noisy" else [12/6500]*3+[.2]
    if case=="crossover":
        sigmas[-1]=.02
    analog=radiance+rng.normal(size=radiance.shape)*sigmas
    rails=np.array([160/6500]*3+[40] if case=="clipped" else [4000/6500]*3+[110])
    codes=np.rint(np.clip(analog/rails,0,1)*4095).astype(np.uint16)
    mask=np.where((codes==0)|(codes==4095),2,1).astype(np.uint8)
    missing=(slice(0,3) if case=="missing_rgb" else slice(3,4) if case=="missing_lwir"
             else slice(0,4) if case=="missing_both" else None)
    if missing is not None:
        codes[...,missing]=0
        mask[...,missing]=0
    if case in ("registration_known","registration_hidden"):
        codes[...,3]=np.roll(codes[...,3],3,axis=1)
        mask[...,3]=np.roll(mask[...,3],3,axis=1)
        codes[:,:3,3]=0
        mask[:,:3,3]=0
    return codes,mask


def label(patches,geom):
    p=int(geom[...,0].sum())
    y,x=np.where(geom[...,1]==1)
    n=len(x)
    feasible=n>=4 and 4*n>=p and p>0
    surface_names={patch[0] for patch in patches}
    negative=("feasible" if feasible else "distractor" if "distractor" in surface_names
              else "absent" if "target" not in surface_names else "empty_support" if not p
              else "hidden" if not n else "insufficient_partial")
    return p,n,n/p if p else "",float(x.mean()) if n else "",float(y.mean()) if n else "",int(feasible),negative


def run(out):
    inference=out/"inference"
    scoring=out/"scoring"
    inference.mkdir(parents=True)
    scoring.mkdir()
    n=512*len(CFG["cases"])
    permutation=np.random.default_rng(CFG["seed"]).permutation(n)
    codes=np.lib.format.open_memmap(inference/"codes.npy",mode="w+",dtype=np.uint16,shape=(n,H,W,4))
    masks=np.lib.format.open_memmap(inference/"masks.npy",mode="w+",dtype=np.uint8,shape=(n,H,W,4))
    geometry_truth=np.lib.format.open_memmap(scoring/"geometry.npy",mode="w+",dtype=np.uint8,shape=(n,H,W,2))
    observations,labels,scenes=([None]*n for _ in range(3))
    index=0
    for split,(begin,end) in CFG["split_bounds"].items():
        for realization in range(begin,end):
            rng=random.Random(CFG["seed"]+2_000_000+realization)
            x,y=rng.randint(4,20),rng.randint(4,15)
            for case in CFG["cases"]:
                ident=int(permutation[index])
                index+=1
                patches=layout(case,x,y)
                geom=geometry(patches)
                current,future=physical(realization,case)
                m=medium(case,current)
                codes[ident],masks[ident]=image(patches,geom,case,realization,m)
                geometry_truth[ident]=geom
                observations[ident]=(ident,split,2000,1900 if case=="timing_offset" else 2000,
                                     int(case!="registration_known"),"" if case=="unknown_range" else 10,
                                     int(case!="unknown_material"))
                labels[ident]=(ident,realization,case,*label(patches,geom))
                scenes[ident]=(ident,json.dumps(patches,separators=(",",":")),current,future,
                               3 if case.startswith("registration_") else 0,m[2],*m[:2],
                               math.exp(-5*m[0]))
    for path,fields,data in ((inference/"frames.csv",OBS,observations),
                             (scoring/"labels.csv",LABELS,labels),(scoring/"scene.csv",TRUTH,scenes)):
        with path.open("w",newline="") as f:
            csv_writer(f,fields).writerows(data)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out",type=Path)
    run(parser.parse_args().out)
