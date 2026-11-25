"""Compact logistic-normal MLP, with an optional OU teacher KL objective."""
import numpy as np

from common import CFG, HORIZONS, sigmoid


def initialize(seed, dimensions):
    rng = np.random.default_rng(seed)
    width = CFG["hidden"]
    heads = 2*len(HORIZONS)*dimensions
    return [rng.normal(0,.2,(8,width)), np.zeros(width),
            rng.normal(0,.08,(width,heads)), np.zeros(heads)]


def predict(parameters, x, dimensions):
    w,b,v,c = parameters
    hidden = np.tanh(x @ w + b)
    raw = (hidden @ v + c).reshape(len(x),len(HORIZONS),dimensions,2)
    location, scale_raw = raw[...,0],raw[...,1]
    scale = CFG["min_scale"]+np.logaddexp(0,scale_raw)
    return hidden,location,scale,scale_raw


def normal_nll(location, scale, truth):
    return np.log(scale)+.5*((location-truth)/scale)**2


def objective(parameters, x, y, dimensions, teacher, strength):
    hidden,loc,scale,raw = predict(parameters,x,dimensions)
    y = y[...,:dimensions]
    error = loc-y
    variance = scale**2
    nll = normal_nll(loc,scale,y)
    dloc = error/variance/nll.size
    dscale = (1/scale-error**2/scale**3)/nll.size
    kl_mean = 0.0
    if strength:
        mean, teacher_scale = teacher[...,0], teacher[...,1]
        student_mean, student_scale = loc[...,0],scale[...,0]
        difference = student_mean-mean
        moment = teacher_scale**2+difference**2
        kl = np.log(student_scale/teacher_scale)+.5*moment/student_scale**2-.5
        kl_mean = float(kl.mean())
        dloc[...,0] += strength*difference/student_scale**2/kl.size
        dscale[...,0] += strength*(1/student_scale-moment/student_scale**3)/kl.size
    dheads = np.stack((dloc,dscale*sigmoid(raw)),axis=-1).reshape(len(x),-1)
    nonlinear = (dheads @ parameters[2].T)*(1-hidden**2)
    gradients=[x.T @ nonlinear,nonlinear.sum(axis=0),hidden.T @ dheads,dheads.sum(axis=0)]
    return float(nll.mean())+strength*kl_mean,kl_mean,gradients


class Adam:
    def __init__(self,parameters):
        self.first = [np.zeros_like(x) for x in parameters]
        self.second = [np.zeros_like(x) for x in parameters]
        self.steps = 0

    def update(self,parameters,gradients,learning_rate):
        self.steps += 1
        for param,gradient,a,b in zip(parameters,gradients,self.first,self.second,strict=True):
            a *= .9
            a += .1*gradient
            b *= .999
            b += .001*gradient**2
            direction = (a/(1-.9**self.steps))/(np.sqrt(b/(1-.999**self.steps))+1e-8)
            param -= learning_rate*direction
