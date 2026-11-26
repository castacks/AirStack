"""Independent numerical references. This module imports no ORE-MS code."""
import math

import numpy as np
from scipy.integrate import quad
from scipy.linalg import expm
from scipy.special import ndtr

H, C, K = 6.62607015e-34, 299792458, 1.380649e-23


def quadrature(function, start, end, points=None):
    return quad(function, start, end, points=points, epsabs=1e-11, epsrel=1e-11)[0]


def planck(wavelength_um, kelvin):
    wavelength_m = np.asarray(wavelength_um) * 1e-6
    frequency = C / wavelength_m
    per_hz = (2 * H / C**2) * frequency**3 / np.expm1(H * frequency / (K * kelvin))
    return per_hz * C / wavelength_m**2 * 1e-6


def transfer(boundary, layers):
    """Camera to surface. Input layers are (metres, m^-1, source per metre)."""
    transmission = 1.
    path = 0.
    for length, extinction, source in layers:
        path += transmission * quadrature(
            lambda distance: source * math.exp(-distance * extinction), 0, length)
        transmission *= math.exp(-length * extinction)
    return transmission, transmission * boundary, path


def two_stream(depth, albedo):
    """Unit normal stream incident at the near boundary, no far incidence."""
    a, b = 1 - albedo / 2, albedo / 2
    matrix = expm(depth * np.array([[-a, b], [-b, a]]))
    reflected = -matrix[1, 0] / matrix[1, 1]
    transmitted = 1 / matrix[1, 1]
    return float(reflected), float(transmitted)


def project(points, intrinsic, rotation, origin):
    homogeneous = (intrinsic @ rotation @ (np.asarray(points) - origin).T).T
    return homogeneous[..., :2] / homogeneous[..., 2:]


def raster(columns, rows, vertices):
    """Convex projected polygon with inclusive edge convention."""
    pixels = np.stack(np.broadcast_arrays(columns, rows), axis=-1)
    ends = np.roll(vertices, -1, axis=0)
    edges = ends - vertices
    delta = pixels[..., None, :] - vertices
    cross = edges[:, 0] * delta[..., 1] - edges[:, 1] * delta[..., 0]
    return np.all(cross >= -1e-12, axis=-1) | np.all(cross <= 1e-12, axis=-1)


def square(x, y, half_x, half_y):
    return np.array([[x-half_x,y-half_y], [x+half_x,y-half_y],
                     [x+half_x,y+half_y], [x-half_x,y+half_y]])


def ou(dt, tau, sigma):
    decay = float(expm(np.array([[-dt/tau]]))[0, 0])
    variance = quadrature(lambda t: 2*sigma**2/tau * math.exp(-2*t/tau), 0, dt)
    return decay, variance


def kernel(dx, dt, velocity, period, weights, tau, sigma):
    angle = math.tau * (dx - velocity * dt) / period
    return sigma**2 * math.exp(-abs(dt)/tau) * sum(
        weight * math.cos(n*angle) for n, weight in enumerate(weights))


def gaussian_scores(truth, mean, sd):
    error = truth - mean
    z = error / sd
    crps = sd * (z * (2 * ndtr(z) - 1) + np.sqrt(2/np.pi)*np.exp(-z*z/2)
                 - 1/np.sqrt(np.pi))
    success = int(np.count_nonzero(np.abs(z) <= 1.959963984540054))
    n = len(truth)
    return {'n': n, 'covered': success, 'coverage': success/n,
            'bias_truth_minus_prediction': float(np.mean(error)),
            'error_variance': float(np.var(error, ddof=1)),
            'crps': float(np.mean(crps)),
            'crps_se': float(np.std(crps, ddof=1)/np.sqrt(n)),
            'nll': float(np.mean(np.log(sd*np.sqrt(2*np.pi)) + z*z/2))}


def piecewise_band(function, knots):
    return sum(quadrature(function, lo, hi) for lo, hi in zip(knots[:-1], knots[1:]))


def homogeneous_lwir(config, material, length, air, burden):
    """Vectorized independent Gaussian quadrature, subdivided at response knots."""
    nodes, weights = np.polynomial.legendre.leggauss(16)
    wave = np.concatenate([mid + 1.5*nodes for mid in (9.5, 12.5)])
    profile = lambda values: np.interp(wave, (8, 11, 14), values)
    optics = config['optical']
    opacity = profile(optics['lwir_floor']) + np.asarray(burden)[..., None] * profile(
        optics['lwir_span'])
    trans = np.exp(-length*opacity)
    emissivity = profile(material['emissivity'])
    boundary = emissivity*planck(wave, material['temperature_K']) + (
        1-emissivity)*planck(wave, material['reflected_K'])
    values = trans*boundary + (1-trans)*planck(wave, air)
    return values @ (1.5*np.tile(weights, 2)*profile(config['lwir_response']))


def homogeneous_rgb(config, material, length, burden):
    nodes, weights = np.polynomial.legendre.leggauss(16)
    optics = config['optical']
    outputs = []
    for a,b,c in config['rgb_response'].values():
        knots = sorted({a,b,c,*([550] if a < 550 < c else [])})
        band = 0.
        for lo,hi in zip(knots,knots[1:]):
            wave = (lo+hi)/2+(hi-lo)/2*nodes
            response = np.interp(wave,(a,b,c),(0,1,0))
            incident = np.interp(wave,(400,550,700),material['irradiance'])
            boundary = incident*np.interp(wave,(400,550,700),material['rho'])/math.pi
            strength = optics['rgb_floor']+optics['rgb_span']*np.asarray(burden)[...,None]
            trans = np.exp(-length*strength*(550/wave)**optics['rgb_exponent'])
            source = config['rgb_ambient']*config['rgb_solid_angle']*optics['scatter_fraction']/(4*math.pi)
            band = band + (trans*boundary+(1-trans)*source) @ (weights*response*(hi-lo)/2)
        outputs.append(band)
    return np.stack(outputs,axis=-1)


def burden_projection(state, clock, config):
    positions = (np.arange(config['segments'])+.5)*config['length_m']/config['segments']
    angle = math.tau*(positions-config['velocity_m_s']*clock)/config['period_m']
    basis = [np.full_like(angle, np.sqrt(config['weights'][0]))]
    for i,w in enumerate(config['weights'][1:],1):
        basis.extend([np.sqrt(w)*np.cos(i*angle),np.sqrt(w)*np.sin(i*angle)])
    return (1/(1+np.exp(-(state @ np.array(basis))))).mean(axis=-1)


def wilson(success, n):
    if not n:
        return None
    z = 1.959963984540054
    p = success/n
    scale = 1+z*z/n
    center = (p+z*z/(2*n))/scale
    margin = z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/scale
    return [center-margin,center+margin]
