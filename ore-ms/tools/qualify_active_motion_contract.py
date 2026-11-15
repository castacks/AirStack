"""Finite kinematic/history fixture for active-perception design, not a planner."""
import datetime,hashlib,json
from fractions import Fraction as F
from pathlib import Path
import numpy as np
from numpy.polynomial import Polynomial as P
from numpy.polynomial.legendre import leggauss
from scipy.integrate import quad

ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def deriv(c):return [F(i)*x for i,x in enumerate(c)][1:]
def val(c,u):return sum(x*u**i for i,x in enumerate(c))
def product(a,b):
    c=[F(0)]*(len(a)+len(b)-1)
    for i,x in enumerate(a):
        for j,y in enumerate(b):c[i+j]+=x*y
    return c

def main():
    target=ROOT/'results/active-motion-contract';target.mkdir(parents=True,exist_ok=True)
    freeze=target/'freeze.json'
    if freeze.exists():raise FileExistsError('One fixed run only; retain existing output')
    plan={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'baseline':'a79658f6341194b0954852226f200313ccdf5408','branch':'ore-ms-orb-development','source_sha256':sha(__file__),'distance_m':.04,'horizon_s':.2,'beta':[-2,0,2],'virtual_tau_s':.008,'box_duration_s':.016,'speed_bound_mps':.5,'acceleration_bound_mps2':10.,'quad_tolerance_m':1e-11,'grid_points':10001,'scope':'Kinematic/control qualification only, no images/inference/poses/planner or measured sensor model.'}
    freeze.write_text(json.dumps(plan,indent=2)+'\n')
    checks=[]
    def ck(label,ok):
        assert ok,label
        checks.append(label)
    rows=[];energies={};polys={}
    H=plan['horizon_s'];D=plan['distance_m'];tau=plan['virtual_tau_s'];W=plan['box_duration_s']
    for beta in plan['beta']:
        c=list(map(F,[0,0,0,10+beta,-15-3*beta,6+3*beta,-beta]));dc=deriv(c);ddc=deriv(dc);polys[beta]=c
        ck(f'{beta}positions',val(c,F(0))==0 and val(c,F(1))==1)
        ck(f'{beta}endpointvelocity',val(dc,F(0))==val(dc,F(1))==0)
        ck(f'{beta}endpointacceleration',val(ddc,F(0))==val(ddc,F(1))==0)
        expected=product([F(0),F(0),F(1),F(-2),F(1)],[F(30+3*beta),F(-6*beta)])
        ck(f'{beta}analyticderivative',dc==expected)
        ck(f'{beta}monotoniccertificate',30-3*abs(beta)>=24)
        energy=sum(x/F(i+1) for i,x in enumerate(product(dc,dc)));energies[beta]=energy
        poly=P([float(x) for x in c]);vel=poly.deriv();acc=vel.deriv()
        u=np.linspace(0,1,plan['grid_points']);q=poly(u);v=vel(u)*D/H;a=acc(u)*D/H**2
        ck(f'{beta}positiongrid',q.min()>=-1e-13 and q.max()<=1+1e-13)
        # Real critical points of velocity and acceleration independently bound extrema.
        critical_v=[0.,1.]+[x.real for x in acc.roots() if abs(x.imag)<1e-10 and 0<x.real<1]
        critical_a=[0.,1.]+[x.real for x in acc.deriv().roots() if abs(x.imag)<1e-10 and 0<x.real<1]
        vmax=max(abs(vel(x)*D/H) for x in critical_v);amax=max(abs(acc(x)*D/H**2) for x in critical_a)
        ck(f'{beta}speedbound',vmax<=plan['speed_bound_mps']);ck(f'{beta}accbound',amax<=plan['acceleration_bound_mps2'])
        ck(f'{beta}gridextrema',max(abs(v))<=vmax+1e-12 and max(abs(a))<=amax+1e-10)
        # Numerically stable positive-domain polynomial: independent Bernstein form below.
        def q_bern(t):
            z=max(0.,min(1.,t/H));return D*(z**3*(10+z*(-15+6*z))+beta*z**3*(1-z)**3)
        def response(age):return np.exp(-age/tau)/tau*q_bern(H-age)
        reference,estimate=quad(response,0,H,epsabs=1e-13,epsrel=1e-12)
        results=[]
        for n in [64,128]:
            x,w=leggauss(n);ages=(x+1)*H/2
            # Different evaluator from reference; same mathematical polynomial.
            pred=D*H/2*np.sum(w*np.exp(-ages/tau)/tau*poly(1-ages/H));results.append(float(pred))
            ck(f'{beta}responsequad{n}',abs(pred-reference)<=plan['quad_tolerance_m'])
        box,box_est=quad(lambda age:q_bern(H-age)/W,0,W,epsabs=1e-13)
        recent_v2,_=quad(lambda age:float(vel(1-age/H)*D/H)**2/W,0,W,epsabs=1e-13)
        ck(f'{beta}integrationestimates',estimate<=plan['quad_tolerance_m'] and box_est<=plan['quad_tolerance_m'])
        rows.append({'beta':beta,'normalized_velocity_energy_exact':str(energy),'velocity_energy_m2ps':float(energy)*D**2/H,'max_speed_mps':float(vmax),'max_acceleration_mps2':float(amax),'endpoint_velocity_mps':0.,'response_position_m':float(reference),'response_lag_m':float(D-reference),'gauss_positions_m':results,'uniform_recent_position_m':float(box),'uniform_recent_lag_m':float(D-box),'recent_velocity_squared_mean_m2ps2':float(recent_v2),'quad_error_estimate_m':float(estimate)})
    ck('oppositesexactenergytie',energies[-2]==energies[2])
    for u in [F(0),F(1,8),F(1,4),F(1,2),F(3,4),F(7,8),F(1)]:
        ck('time reversal '+str(u),val(polys[-2],u)==1-val(polys[2],1-u))
    ck('orderedhistoryresponse',rows[0]['response_position_m']<rows[1]['response_position_m']<rows[2]['response_position_m'])
    ck('recentboxalsoorders',rows[0]['uniform_recent_position_m']<rows[1]['uniform_recent_position_m']<rows[2]['uniform_recent_position_m'])
    ck('sourceunchanged',sha(__file__)==plan['source_sha256'])
    report={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'status':'PASS','checks':len(checks),'passed':checks,'rows':rows,'scope':plan['scope'],'interpretation':'Whole-horizon energy ties for reversed schedules but recent box and exponential history both distinguish them. No proof of thermal-specific necessity, pose error improvement, covariance calibration or novel planner.'}
    (target/'qualification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':'PASS','checks':len(checks),'rows':rows}))
if __name__=='__main__':main()
