"""Small independent scoring and spectral formulas for verification."""
import math
import statistics


def empirical(sample,y):
    n=len(sample)
    return sum(abs(x-y) for x in sample)/n - sum(
        abs(x-z) for x in sample for z in sample)/(2*n*n)


def energy_pair(code_probability,observed):
    probability=(1-code_probability,code_probability)
    indicator=(1-observed,observed)
    return sum((a-b)**2 for a,b in zip(probability,indicator,strict=True))/2


def interval(differences):
    n=len(differences)
    center=statistics.fmean(differences)
    half=1.96*statistics.stdev(differences)/math.sqrt(n)
    return center, center-half, center+half
