"""M05 coefficient propagation and fixed-ray quality projections."""
from functools import lru_cache
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.append(str(ROOT / "m07"))
from forward import CONFIG as INVERSE
from environment import Field, advance, logistic, stationary, transition

CONFIG = json.loads((HERE / "config.json").read_text())
MODEL = Field(CONFIG["period_m"], tuple(CONFIG["weights"]))
HORIZONS = tuple(CONFIG["horizons_s"])
LENGTH = CONFIG["length_m"]
ZERO = (0,) * MODEL.size
OPTICS = INVERSE["optical"]


@lru_cache(maxsize=None)
def basis(time, length):
    count = CONFIG["segments"]
    return tuple(MODEL.basis((i + .5)*length/count, time, CONFIG["velocity_m_s"])
                 for i in range(count))


def burden(coefficients, time, length=LENGTH):
    a,b,c,d,e = coefficients
    return sum(logistic(a*w+b*x+c*y+d*v+e*z) for w,x,y,v,z in basis(time,length))/CONFIG["segments"]


def targets(b):
    return (b, math.exp(-LENGTH*(OPTICS["rgb_floor"]+OPTICS["rgb_span"]*b)),
            math.exp(-LENGTH*(OPTICS["lwir_floor"][1]+OPTICS["lwir_span"][1]*b)))


@lru_cache(maxsize=None)
def updates(tau, sigma):
    return tuple(transition(b-a,tau,sigma) for a,b in zip(HORIZONS,HORIZONS[1:]))


def evolve(coefficients, rng, tau, sigma):
    result = [burden(coefficients,0)]
    for h, update in zip(HORIZONS[1:],updates(tau,sigma),strict=True):
        coefficients = advance(coefficients,ZERO,update,rng)
        result.append(burden(coefficients,h))
    return result
