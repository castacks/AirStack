"""Independent OU and frozen-path formulas for audit."""
import math


def ou_reference(x,h,tau,sigma):
    multiplier=1/math.exp(h/tau)
    return multiplier*x,sigma**2*(1-multiplier**2)


def independent_burden(coeffs,time,config):
    period=config["period_m"]
    horizon=config["velocity_m_s"]*time
    length=config["length_m"]
    count=config["segments"]
    a,b,c,d,e=coeffs
    total=0
    for i in range(count):
        theta=(length*(2*i+1)/(2*count)-horizon)*2*math.pi/period
        x=(math.sqrt(.2)*a+math.sqrt(.5)*(b*math.cos(theta)+c*math.sin(theta))+
           math.sqrt(.3)*(d*math.cos(2*theta)+e*math.sin(2*theta)))
        total+=1/(1+math.exp(-x))
    return total/count
