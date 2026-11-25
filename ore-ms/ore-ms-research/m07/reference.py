"""Independent inversion and statistical scoring references."""
import math


def rgb_band(config, burden, channel="R"):
    # Accepted M02 response integrates the product of linear interpolants
    # of sample radiance and sensitivity. Simpson is exact for that product.
    optics = config["optical"]
    source = (config["rgb_ambient"] * config["rgb_solid_angle"]
              * optics["scatter_fraction"] / (4*math.pi))
    surface = config["materials"]["target"]
    boundary = surface["rho"][0] * surface["irradiance"][0] / math.pi
    lo, peak, hi = config["rgb_response"][channel]
    depth = config["length_m"] * (optics["rgb_floor"] + optics["rgb_span"]*burden)

    def h(w):
        return source + (boundary-source)*math.exp(-depth*(550/w)**optics["rgb_exponent"])

    def response(w):
        return (w-lo)/(peak-lo) if w <= peak else (hi-w)/(hi-peak)

    return math.fsum(5/6*(h(w)*response(w)+2*(h(w)+h(w+5))*response(w+2.5)
                         +h(w+5)*response(w+5)) for w in range(lo,hi,5))


def inverse_band(function, measured):
    low, high = 0, 1
    for _ in range(50):
        middle = (low+high)/2
        if function(middle) > measured:
            low = middle
        else:
            high = middle
    return (low+high)/2


def wilson(successes, size):
    proportion = successes/size
    z = 1.96
    center = (proportion+z*z/(2*size))/(1+z*z/size)
    margin = z*math.sqrt(proportion*(1-proportion)/size+z*z/(4*size*size))/(1+z*z/size)
    return max(0,center-margin), min(1,center+margin)
