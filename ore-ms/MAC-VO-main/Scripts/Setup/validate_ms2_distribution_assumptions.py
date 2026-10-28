"""Independent numerical integration of assumed reciprocal-disparity risk."""
import argparse,json,math,hashlib
from pathlib import Path
from scipy.integrate import quad
from scipy.special import gammaln
from scipy.stats import gamma,lognorm
import numpy as np

def run(output):
    from .ms2_distribution_risk import scores,choose
    results=[]
    for c in [.01,.1,.25,.49]:
        s2=math.log1p(c);a=1/c;normalizer=float(gammaln(a)+a*math.log(c))
        for family in ['lognormal','gamma']:
            def integrand(t):
                if t==0 or t>700:return 0.
                # x=exp(t); integration uses density of log disparity and (1/x-1)^2.
                correction=math.log1p(-math.exp(t)) if t<0 else math.log1p(-math.exp(-t))
                if family=='gamma':
                    exponent=((a-2)*t if t<0 else a*t)+2*correction-math.exp(t)/c-normalizer
                else:
                    exponent=-(t+s2/2)**2/(2*s2)-.5*math.log(2*math.pi*s2)+2*correction+(-2*t if t<0 else 0)
                return math.exp(exponent)
            left,le=quad(integrand,-np.inf,0,epsabs=1e-10,epsrel=1e-10,limit=300)
            right,re=quad(integrand,0,np.inf,epsabs=1e-10,epsrel=1e-10,limit=300)
            expected=c+3*c*c+c**3 if family=='lognormal' else c*(1+2*c)/((1-c)*(1-2*c))
            assert np.isclose(left+right,expected,rtol=1e-8,atol=1e-10)
            dist=lognorm(s=math.sqrt(s2),scale=math.exp(-s2/2)) if family=='lognormal' else gamma(a=a,scale=c)
            mean,var=dist.stats(moments='mv');assert np.allclose([mean,var],[1,c])
            first=np.array([c]);s=scores(np.array([1.]),np.array([c]),first)
            assert np.isclose(np.exp(s['log_'+family][0]),expected,rtol=1e-12)
            results.append({'family':family,'c':c,'mean':float(mean),'variance':float(var),'quadrature':left+right,'quadrature_error':le+re,'analytic':expected})
    divergence=[]
    for c in [.5,1.]:
        a=1/c
        assert a-3<=-1 # near-zero density*inverse-square exponent; divergent integral
        def log_integrand(t):
            if t==0:return 0.
            return math.exp((a-2)*t+2*math.log1p(-math.exp(t))-math.exp(t)/c-gammaln(a)-a*math.log(c))
        truncated=[quad(log_integrand,math.log(eps),0,epsabs=1e-8,limit=300)[0] for eps in [1e-2,1e-4,1e-6]]
        assert truncated[0]<truncated[1]<truncated[2]
        test=scores(np.array([1.]),np.array([c]),np.array([c]));assert not test['gamma_finite'][0] and np.isinf(test['log_gamma'][0]) and np.isfinite(test['log_lognormal'][0])
        divergence.append({'c':c,'near_zero_integrand_power':a-3,'lower_cutoffs':[1e-2,1e-4,1e-6],'truncated_integrals':truncated,'full_gamma_risk':'infinite','lognormal_risk':'finite'})
    assert np.array_equal(choose(np.array([0,np.inf,1,np.inf]),np.array([0,np.inf,np.inf,1])),[.5,.5,1,0])
    tiny=scores(np.array([1.]),np.array([1e-10]),np.array([1e-10]));assert np.isclose(np.exp(tiny['log_lognormal'][0])/1e-10,1,rtol=1e-8)
    two_point=[]
    for c in [.01,.1,1.]:
        risks=[]
        for epsilon in [.1,.01,.001,1e-6]:
            upper=1+c/(1-epsilon);probability=c/(c+(1-epsilon)**2)
            mean=probability*epsilon+(1-probability)*upper
            variance=probability*(epsilon-1)**2+(1-probability)*(upper-1)**2
            assert np.allclose([mean,variance],[1,c])
            risk=probability*(1/epsilon-1)**2+(1-probability)*(1/upper-1)**2
            risks.append(risk)
            two_point.append({'c':c,'epsilon':epsilon,'upper':upper,'lower_probability':probability,'mean':mean,'variance':variance,'inverse_depth_risk':risk})
        assert all(a<b for a,b in zip(risks,risks[1:]))
    result={'status':'PASS','quadrature_cases':results,'divergence_cases':divergence,'two_point_nonidentification':two_point,'tie_extended_infinity_fixture':'PASS','small_c_limit':'PASS','script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'interpretation':'assumed-family formulas only, not a fitted or validated data distribution'}
    output.write_text(json.dumps(result,indent=2)+'\n');print('PASS eight finite quadratures, two divergent Gamma cases, 12 same-moment two-point fixtures, extended-risk ties and small-c limit')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);run(p.parse_args().output)
