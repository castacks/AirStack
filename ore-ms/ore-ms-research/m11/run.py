"""Compare immutable milestone kernels against independent M11 references."""
import argparse
import hashlib
import importlib
import json
import math
from pathlib import Path
import random
import resource
import sys
from time import perf_counter

import numpy as np
from scipy.special import expit
from scipy.stats import norm

import reference as ref

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.append(str(ROOT))
for folder in ('m02', 'm03', 'm04', 'm05', 'm06', 'm07', 'm08'):
    sys.path.append(str(ROOT / folder))

import rgb_model as rgb
import thermal_model as lw
import geometry as geo
import environment as env
import adapters
import estimate
import physics
import dynamics



def legacy_checks():
    errors = []
    for extinction in (0,.1,2):
        expected = sum(ref.transfer(.3,[(10,extinction,.1*extinction)])[1:])
        errors.append(abs(physics.transport(.3,.1,extinction,10)-expected))
    def weighted(w):
        boundary = .8*ref.planck(w,310)+.2*ref.planck(w,290)
        source = .04*ref.planck(w,300)
        return np.interp(w,(8,14),(.4,.9))*sum(ref.transfer(boundary,[(10,.04,source)])[1:])
    expected = ref.quadrature(weighted,8,14)
    actual = physics.thermal_band(310,.8,290,300,.04,10,(.4,.9),96)
    return {'rgb_absolute_errors':errors,'lwir_actual':actual,'lwir_reference':expected,
            'lwir_absolute_error':abs(actual-expected)}


def rgb_checks():
    worst = 0.
    rows = []
    phase = 1/(4*math.pi)
    for wave in (450, 550, 650):
        for depth in (0, .2, 1, 4):
            for illumination in (0., .8):
                scale = (550/wave)**1.4
                layers = [rgb.Layer(2, .3*depth*scale, .2*depth*scale, illumination, 1.2),
                          rgb.Layer(4, .175*depth*scale, .075*depth*scale, .5*illumination, .6)]
                expected = ref.transfer(.3, [(2,.5*depth*scale,.2*depth*scale*illumination*1.2*phase),
                    (4,.25*depth*scale,.075*depth*scale*.5*illumination*.6*phase)][::-1])
                actual = rgb.transfer(.3, layers, phase)
                error = max(abs(a-b) for a,b in zip(expected, (actual.transmission,
                            actual.direct, actual.path)))
                rows.append((wave,depth,illumination,error))
                worst = max(worst,error)
    noise = []
    sensor = rgb.Sensor(1, 300, 12, 3, True)
    for level in (50, 200):
        rng = random.Random(110001+level)
        draws = np.array([sensor.capture(level, rng)[0] for _ in range(20000)]) * 300/4095
        noise.append({'level_e':level, 'mean_e':float(draws.mean()),
                      'variance_e2':float(draws.var(ddof=1)), 'expected_variance_e2':level+9})
    streams = []
    for depth in (.1, 1., 4.):
        reflected, transmitted = ref.two_stream(depth, 1.)
        once = -.25*math.expm1(-2*depth)
        streams.append({'depth':depth,'two_stream_reflection':reflected,
                        'single_backscatter':once, 'energy_residual':abs(1-reflected-transmitted)})
    spectral = max(abs(rgb.spectral_coefficient(.15,w,550,1.4)-.15*(550/w)**1.4)
                   for w in (450,550,650))
    rails = [sensor.capture(x) for x in (0, 299, 300, 500)]
    return {'rows':rows, 'max_absolute':worst, 'noise':noise,'two_stream':streams,
            'spectral_error':spectral,'detector_rails':rails}


def lwir_checks():
    points = [(w,t,lw.planck_um(w,t)-float(ref.planck(w,t)))
              for w in (8,11,14) for t in (220,290,300,330)]
    response = (.20, .90, .35)
    depths = (.3, .6)
    temps = (280, 307)
    def terms(w, target):
        emissivity = np.interp(w, (8,11,14), (.68,.82,.9))
        emission = emissivity*ref.planck(w,target)
        reflect = (1-emissivity)*ref.planck(w,290)
        trans, _, path = ref.transfer(0, [(1,d,d*ref.planck(w,t))
                                         for d,t in zip(depths,temps)][::-1])
        return np.array([trans*emission,trans*reflect,path])
    expected = [ref.piecewise_band(lambda w: terms(w,310)[i]*np.interp(w,(8,11,14),response),
                                   (8,11,14)) for i in range(3)]
    refinements = []
    for n in (12,24,48,96):
        wave = np.linspace(8,14,n+1)
        values = []
        for w in wave:
            emission, reflect = lw.surface_components(lw.planck_um(w,310),
                lw.profile(w,(.68,.82,.9)),lw.planck_um(w,290))
            trans, path = lw.atmosphere([lw.Slab(d,lw.planck_um(w,t))
                                         for d,t in zip(depths,temps)])
            values.append([trans*emission,trans*reflect,path])
        band = [lw.weighted_band(v,[lw.profile(w,response) for w in wave],6/n)
                for v in zip(*values)]
        refinements.append({'intervals':n,'band':band,'absolute_error':max(
            abs(a-b) for a,b in zip(expected,band))})
    contrast = []
    for dt in (-1, -1e-9, 0, 1e-9, 1):
        def actual(t):
            trans,path = lw.atmosphere([lw.Slab(.6,lw.planck_um(11,300))])
            return trans*sum(lw.surface_components(lw.planck_um(11,t),.8,lw.planck_um(11,300)))+path
        difference = actual(300+dt)-actual(300)
        expected_delta = .8*math.exp(-.6)*(ref.planck(11,300+dt)-ref.planck(11,300))
        contrast.append((dt,difference,float(expected_delta)))
    detector = lw.Detector(1, 50, 12, 0)
    return {'planck_rows':points,'planck_relative_max':max(abs(e)/float(ref.planck(w,t))
            for w,t,e in points),'reference_terms':expected,'refinements':refinements,
            'crossover':contrast,'detector_rails':[detector.measure(x) for x in (0,1,20,50,60)]}


def geometry_checks():
    axes = ((1,0,0),(0,1,0),(0,0,1))
    camera = geo.Camera(33,25,20,20,16,12,(0,0,0),axes)
    intrinsic = np.array([[20,0,16],[0,20,12],[0,0,1]])
    target = geo.Rectangle('target',(0,0,10),(2,2),'target',True)
    block = geo.Rectangle('block',(-2.83,0,5),(.81,1.13),'block',velocity=(1,0,0))
    tie = geo.Rectangle('tied_block',(0,0,10),(2,2),'block')
    rows, cols = np.indices((25,33))
    max_range = max_pixel = 0.
    mismatch = 0
    counts = []
    for time, primitives in ((0,(target,block)),(2,(target,block)),(5,(target,block)),(0,(target,tie))):
        expected = np.full(rows.shape,'none',dtype='U15')
        depths = np.full(rows.shape,np.inf)
        for p in primitives:
            x,y,z = p.at(time)
            corners = np.column_stack((ref.square(x,y,*p.half),np.full(4,z)))
            vertices = ref.project(corners,intrinsic,np.eye(3),np.zeros(3))
            max_pixel = max(max_pixel,float(np.max(abs(vertices-np.array(
                [camera.project(vertex) for vertex in corners])))))
            present = ref.raster(cols,rows,vertices) & (z <= depths)
            expected[present] = p.id
            depths[present] = z
        counts.append({'time':time,'tied':len(primitives)==2 and primitives[1] is tie,
                       'visible_target':int(np.count_nonzero(expected=='target'))})
        for y,x in np.ndindex(rows.shape):
            _, first, distance = geo.hits(camera,camera.ray(x,y),[(p,p.at(time)) for p in primitives])
            mismatch += (first.id if first else 'none') != expected[y,x]
            if first and np.isfinite(depths[y,x]):
                independent_range = depths[y,x]*math.sqrt(1+((x-16)/20)**2+((y-12)/20)**2)
                max_range = max(max_range,abs(distance-independent_range))
    # Separate nonidentity camera projection with genuine world translation.
    rotation = np.array([[0,1,0],[-1,0,0],[0,0,1]])
    origin = np.array([1,2,3])
    turned = geo.Camera(33,25,20,20,16,12,tuple(origin),tuple(map(tuple,rotation)))
    point = np.array([3,4,9])
    max_pixel = max(max_pixel,float(np.max(abs(turned.project(point)-
        ref.project([point],intrinsic,rotation,origin)[0]))))
    return {'rays':3300,'first_hit_mismatch':mismatch,'max_range_m':max_range,
            'max_projection_px':max_pixel,'visibility':counts}


def stochastic_checks():
    tau, sigma = 6.,1.1
    steps = (.00000001,.1,2.,60.)
    errors = []
    for dt in steps:
        a,v = ref.ou(dt,tau,sigma)
        actual,scale = env.transition(dt,tau,sigma)
        errors.append((dt,abs(actual-a),abs(scale**2-v)))
    field = env.Field(64,(.2,.5,.3))
    rng = random.Random(110005)
    update = env.transition(2,tau,sigma)
    zero = (0,)*field.size
    n = 24000
    values = np.empty((n,4))
    for i in range(n):
        start = env.stationary(rng,sigma,field.size)
        end = env.advance(start,zero,update,rng)
        values[i] = [env.evaluate(start,field.basis(0)),
                     env.evaluate(start,field.basis(8)),
                     env.evaluate(end,field.basis(8,2,1)),
                     env.advance((1.,),(0.,),update,rng)[0]]
    cov = np.cov(values,rowvar=False)
    independent = np.array([ref.kernel(dx, dt, 1,64,field.weights,tau,sigma)
                             for dx,dt in ((0,0),(8,0),(8,2))])
    a,v = ref.ou(2,tau,sigma)
    euler = []
    for dt in (2.,1.,.5,.25):
        factor = 1-dt/tau
        n_step = round(2/dt)
        variance = 2*sigma*sigma*dt/tau * sum(factor**(2*i) for i in range(n_step))
        euler.append((dt,variance,abs(variance-v)))
    state = zero
    max_long = 0.
    for _ in range(10000):
        state = env.advance(state,zero,update,rng)
        max_long = max(max_long,max(map(abs,state)))
    profile = [env.logistic(x) for x in (-4,2,-4)]
    return {'transition_errors':errors,'n':n, 'means':values.mean(axis=0).tolist(),
            'covariance_row':cov[0,:3].tolist(),'reference_covariance':independent.tolist(),
            'conditional_variance':float(cov[3,3]),'conditional_reference':[a,v],
            'euler':euler,'long_run_max_abs':max_long,'clear_degraded_clear_burden':profile}


def multispectral_checks():
    config = json.loads((ROOT/'m06/config.json').read_text())
    def independent(path, material):
        def visible(w):
            incident = np.interp(w,(400,550,700),material['irradiance'])
            boundary = incident*np.interp(w,(400,550,700),material['rho'])/math.pi
            layers = [(s.length,(s.rgb_a+s.rgb_s)*(550/w)**s.rgb_power,
                       s.rgb_s*(550/w)**s.rgb_power*config['rgb_ambient']*
                       config['rgb_solid_angle']/(4*math.pi)) for s in path[::-1]]
            _,direct,indirect = ref.transfer(boundary,layers)
            return direct+indirect
        def thermal(w):
            e = np.interp(w,(8,11,14),material['emissivity'])
            boundary = e*ref.planck(w,material['temperature_K'])+(1-e)*ref.planck(w,material['reflected_K'])
            layers = []
            for s in path[::-1]:
                opacity = np.interp(w,(8,11,14),s.lwir_a)
                layers.append((s.length,opacity,opacity*ref.planck(w,s.air_K)))
            _,direct,source = ref.transfer(boundary,layers)
            return direct+source
        bands = []
        for key in 'RGB':
            a,b,c = config['rgb_response'][key]
            bands.append(ref.piecewise_band(lambda w: visible(w)*np.interp(w,(a,b,c),(0,1,0)),
                         sorted({a,b,c,*([550] if a<550<c else [])})))
        bands.append(ref.piecewise_band(lambda w: thermal(w)*np.interp(w,(8,11,14),
                                                        config['lwir_response']),(8,11,14)))
        return bands
    field = env.Field(config['field']['period_m'],tuple(config['field']['weights']))
    burden = field.mean_coefficients(-.7,2)
    output = []
    sensor = config['sensor']
    camera_config = config['camera']
    camera = geo.Camera(*(camera_config[key] for key in (
        'width','height','fx','fy','cx','cy','origin','axes')))
    pixel = (camera.cx,camera.cy)
    intrinsic = np.array([[camera.fx,0,camera.cx],[0,camera.fy,camera.cy],[0,0,1]])
    for regime,occluded,count in (
        ('vacuum',False,1),('dual',False,1),('dual',True,1),
        *(('advected',False,n) for n in (4,8,16,32))):
        objects = [config['target'],config['background']]
        if occluded:
            objects.append(config['obstacles']['full'])
        independently_visible = []
        for obj in objects:
            x,y,z = obj['center']
            corners = np.column_stack((ref.square(x,y,*obj['half']),np.full(4,z)))
            polygon = ref.project(corners,intrinsic,np.eye(3),np.zeros(3))
            if ref.raster(*pixel,polygon):
                independently_visible.append((z,obj['material']))
        reference_depth,reference_material = min(independently_visible)
        primitives = [geo.Rectangle(obj['id'],obj['center'],obj['half'],obj['material'],
                                    obj['id']=='target') for obj in objects]
        direction = camera.ray(*pixel)
        _,hit,length = geo.hits(camera,direction,[(obj,obj.center) for obj in primitives])
        visible_fraction = float(hit.target)
        depth_direction = direction[2]
        path = adapters.segments(config,regime,2,burden,(0,)*field.size,0,depth_direction,length,count)
        material_name = hit.material
        material = config['materials'][material_name]
        visible,lwir = adapters.radiance(config,material,path)
        actual = [*map(sum,visible),sum(lwir)]
        reference_path = adapters.segments(config,regime,2,burden,(0,)*field.size,0,1,
                                           reference_depth,count)
        expected = independent(reference_path,config['materials'][reference_material])
        codes = adapters.detect(visible,lwir,sensor,adapters.AnalogNoise(0),random.Random(0))
        normalized = np.array(expected)*sensor['rgb_gain']/sensor['rgb_well']
        normalized[3] = expected[3]/sensor['lwir_high']
        expected_codes = np.round(np.clip(normalized,0,1)*(2**sensor['bits']-1)).astype(int)
        output.append({'regime':regime,'length_m':length,'segments':count,
                       'RGB550_LWIR11_transmission':[math.exp(-adapters.optical_depth(path,m,w))
                                                    for m,w in (('RGB',550),('LWIR',11))],
                       'target_center_visible':bool(visible_fraction),'material':material_name,
                       'reference_hit_depth_m':reference_depth,'reference_material':reference_material,
                       'detector_codes':[codes[i] for i in (3,9,15,22)],
                       'independent_detector_codes':expected_codes.tolist(),
                       'actual':actual,'reference':expected,
                       'errors':[a-b for a,b in zip(actual,expected)]})
    # Validate the advected midpoint sample mapping separately from the transfer.
    cfg = config['field']
    sample_error = 0.
    for segment in adapters.segments(config,'advected',2,burden,(0,)*field.size,0,1,10,32):
        latent = -.7 + 2*math.cos(math.tau*(segment.z_m-2*cfg['velocity_m_s'])/cfg['period_m'])
        expected_burden = float(expit(latent))
        sample_error = max(sample_error,abs(expected_burden-segment.burden))
    # Shared driver creates a dependence control even though detector noises are independent.
    rng = np.random.default_rng(11000506)
    b = expit(rng.standard_normal(128))
    opt = cfg['optical']
    rgb_depth = 10*(opt['rgb_floor']+opt['rgb_span']*b)
    lw_depth = 10*(opt['lwir_floor'][1]+opt['lwir_span'][1]*b)
    trans = np.exp(-np.array([rgb_depth,lw_depth]))
    independent_noise = rng.standard_normal((2,len(b)))
    corr = np.corrcoef(trans)[0,1]
    noisy_corr = np.corrcoef(trans+.035*independent_noise)[0,1]
    disjoint_corr = np.corrcoef(trans[0],trans[1,rng.permutation(len(b))])[0,1]
    # In isothermal LTE B propagated through material at its own temperature is B.
    radiance = ref.planck(11.,300.)
    _,direct,path = ref.transfer(radiance,[(2,.5,.5*radiance),(1,.6,.6*radiance)])
    return {'cases':output,'advected_burden_mapping_error':sample_error,
            'dependence_n':len(b),'shared_pearson':float(corr),
            'noisy_shared_pearson':float(noisy_corr),'permuted_pearson':float(disjoint_corr),
            'isothermal_residual':float(abs(direct+path-radiance))}


def measurements(config, material, length, air, burdens, noise):
    rgb_ideal = ref.homogeneous_rgb(config,material,length,burdens)
    lwir_ideal = ref.homogeneous_lwir(config,material,length,air,burdens)
    analog = np.column_stack((rgb_ideal*config['rgb_gain'],lwir_ideal))
    scale = np.array([config['rgb_well']]*3+[config['lwir_high']])
    sigma = np.array([config['rgb_read_noise_e']]*3+[config['lwir_noise']])
    return np.round(np.clip((analog+noise*sigma)/scale,0,1)*(2**config['bits']-1)).astype(int)


def posterior_checks():
    cfg = estimate.CONFIG
    case = estimate.CASES['matched']
    from forward import context, settings
    name,length,air = context(case)
    material = cfg['materials'][name]
    limits = settings(case)
    grid = np.linspace(0,1,4001)
    projected = ref.homogeneous_lwir(cfg,material,length,air,grid)
    sigma = math.hypot(limits['lwir_sigma'],limits['lwir_high']/4095/math.sqrt(12))
    rng = np.random.default_rng(110006)
    n = 192
    truth = rng.uniform(0,1,n)
    noise = rng.normal(size=n)
    rows = []
    for regime,actual_air in (('matched',air),('wrong_air',air+15)):
        analog = ref.homogeneous_lwir(cfg,material,length,actual_air,truth) + noise*limits['lwir_sigma']
        codes = np.round(np.clip(analog/limits['lwir_high'],0,1)*4095).astype(int)
        errors, actual_covers, ref_covers, widths, biases, crps = [], [], [], [], [], []
        guards = {}
        for b,code in zip(truth,codes):
            data = dict.fromkeys(estimate.CODES,'')
            data.update(case='matched',lwir_code=str(code))
            result = estimate.estimate(data,'LWIR')
            guards[result[0]] = guards.get(result[0],0)+1
            if result[0]=='abstain':
                continue
            density = norm.pdf((code*limits['lwir_high']/4095-projected)/sigma)
            area = np.trapezoid(density,grid)
            pdf = density/area
            cdf = np.r_[0,np.cumsum((pdf[1:]+pdf[:-1])/8000)]
            mean = np.trapezoid(grid*pdf,grid)
            low,high = np.interp((.025,.975),cdf,grid)
            errors.append(max(abs(a-b) for a,b in zip(result[3:6],(mean,low,high))))
            actual_covers.append(result[4]<=b<=result[5])
            ref_covers.append(low<=b<=high)
            widths.append(high-low)
            biases.append(mean-b)
            crps.append(np.trapezoid((cdf-(grid>=b))**2,grid))
        count = len(errors)
        rows.append({'regime':regime,'generated':n,'scored':count,'status':guards,
                     'max_burden_discrepancy':max(errors),
                     'existing_coverage':sum(actual_covers)/count,
                     'existing_coverage_ci':ref.wilson(sum(actual_covers),count),
                     'reference_coverage':sum(ref_covers)/count,
                     'reference_coverage_ci':ref.wilson(sum(ref_covers),count),
                     'reference_mean_width':float(np.mean(widths)),
                     'reference_bias':float(np.mean(biases)),
                     'bias_se':float(np.std(biases,ddof=1)/math.sqrt(count)),
                     'reference_crps':float(np.mean(crps))})
    # Transmission has rank one under simultaneous unknown range and extinction.
    ambiguity = [math.exp(-length*k) for length,k in ((5,.12),(10,.06),(20,.03))]
    return {'rows':rows,'range_extinction_transmission':ambiguity}


FORECASTS = {'matched':(6.,1.), 'slow_tau':(14.,1.), 'low_amplitude':(6.,.6),
             'blind_future_shift':(2.,1.7)}


def forecast_checks():
    config = dynamics.CONFIG
    horizons = config['horizons_s']
    rng = np.random.default_rng(110008)
    n = 4096
    state = rng.standard_normal(n)
    innovations = rng.standard_normal((len(horizons)-1,n))
    forecast = {}
    covariance_error = 0.
    for name,(tau,sigma) in FORECASTS.items():
        future = state.copy()
        previous = 0
        forecast[name] = {}
        for i,h in enumerate(horizons[1:]):
            step = h-previous
            a, v = ref.ou(step,tau,sigma)
            future = a*future+math.sqrt(v)*innovations[i]
            previous = h
            nominal_a,variance = ref.ou(h,6,1)
            actual_a,actual_sd = env.transition(h,6,1)
            for earlier in horizons[:i+1]:
                expected_cross, _ = ref.ou(h-earlier,6,1)
                _, var_earlier = ref.ou(earlier,6,1)
                staged_a,_ = env.transition(h-earlier,6,1)
                staged_var = env.transition(earlier,6,1)[1]**2 if earlier else 0.
                covariance_error = max(covariance_error,abs(expected_cross*var_earlier-staged_a*staged_var))
            covariance_error = max(covariance_error,abs(nominal_a-actual_a),abs(variance-actual_sd**2))
            score = ref.gaussian_scores(future,nominal_a*state,math.sqrt(variance))
            score['coverage_ci'] = ref.wilson(score['covered'],n)
            forecast[name][h] = score
    parameter_rows = []
    # Discrete independent equally weighted parameter prior; no inferential fit.
    for h in horizons[1:]:
        values = [ref.ou(h,tau,sigma) for tau in (4,6,10) for sigma in (.8,1.,1.2)]
        parameter_rows.append({'horizon_s':h,'mean_conditional_aleatoric_variance':float(
            np.mean([variance for _,variance in values])),
            'parameter_variance_at_initial_state_1':float(np.var([a for a,_ in values]))})
    fixed = np.array([-.4,.7,.9,-1.2,.1])
    projection_error = max(abs(float(ref.burden_projection(fixed,h,config))-dynamics.burden(fixed,h))
                           for h in horizons)
    return {'n':n,'horizon_covariance_error':covariance_error,'projection_error':projection_error,
            'scores':forecast,'parameter_budget':parameter_rows}


def learned_checks():
    sys.path.insert(0,str(ROOT/'m09'))
    infer = importlib.import_module('infer')
    checkpoint_path = ROOT/'m09/checkpoints/physics-768-901.json'
    checkpoint = infer.load(checkpoint_path)
    cfg = estimate.CONFIG
    horizons = dynamics.CONFIG['horizons_s']
    material = cfg['materials']['target']
    rng = np.random.default_rng(110009)
    n = 256
    initial = rng.standard_normal((n,5))
    original = ref.burden_projection(initial,0,dynamics.CONFIG)
    noise = rng.standard_normal((n,4))
    innovation = rng.standard_normal((len(horizons)-1,n,5))
    quantiles = norm.ppf((np.arange(512)+.5)/512)
    normal_ends = norm.ppf([.025,.975])
    summaries = {}
    issue_results = {}
    for name in (*FORECASTS,'wrong_air'):
        tau,sigma = FORECASTS.get(name,FORECASTS['matched'])
        air = cfg['air_K'] + (15 if name=='wrong_air' else 0)
        codes = measurements(cfg,material,cfg['length_m'],air,original,noise)
        results = [infer.issue(dict(zip(estimate.CODES,map(str,row)),case='matched'),'joint',checkpoint)
                   for row in codes]
        issue_results[name] = results
        issued = np.array([r[0]=='issued' for r in results])
        count = int(issued.sum())
        location = np.array([json.loads(r[4]) for r in results if r[0]=='issued'])
        scale = np.array([json.loads(r[5]) for r in results if r[0]=='issued'])
        summaries[name] = {'generated':n,'issued':count,
                           'novel_flags':sum(r[3]==1 for r in results),'horizons':{}}
        state = initial.copy()
        for i,h in enumerate(horizons):
            if i:
                a,v = ref.ou(h-horizons[i-1],tau,sigma)
                state = a*state + math.sqrt(v)*innovation[i-1]
            truth = ref.burden_projection(state,h,dynamics.CONFIG)[issued]
            mu,sd = location[:,i,0],scale[:,i,0]
            draws = expit(mu[:,None]+sd[:,None]*quantiles)
            low,high = expit(mu[:,None]+sd[:,None]*normal_ends).T
            covered = int(((truth>=low)&(truth<=high)).sum())
            m = draws.shape[1]
            gini_half = draws @ ((2*np.arange(1,m+1)-m-1)/m**2)
            crps = abs(draws-truth[:,None]).mean(axis=1)-gini_half
            summaries[name]['horizons'][h] = {'coverage':covered/count,
                    'coverage_ci':ref.wilson(covered,count),'crps':float(crps.mean()),
                    'crps_se':float(crps.std(ddof=1)/np.sqrt(count)),
                    'mean_width':float(np.mean(high-low)),
                    'bias':float(np.mean(draws.mean(axis=1)-truth))}
    return {'checkpoint_sha256':hashlib.sha256(checkpoint_path.read_bytes()).hexdigest(),
            'same_issue_scores_before_future_shift':all(
                issue_results[name]==issue_results['matched']
                for name in FORECASTS),
            'scores':summaries}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    from fusion import fusion_checks
    timings = {}
    start = perf_counter()
    for check in (legacy_checks,rgb_checks,lwir_checks,geometry_checks,stochastic_checks,multispectral_checks,
                  posterior_checks,forecast_checks,learned_checks,fusion_checks):
        phase = perf_counter()
        result = check(args.output, timings) if check is fusion_checks else check()
        timings[check.__name__] = perf_counter()-phase
        (args.output/f'{check.__name__}.json').write_text(
            json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n')
    timings['total_s'] = perf_counter()-start
    timings['peak_rss_kib'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    (args.output/'timing.json').write_text(json.dumps(timings,indent=2,sort_keys=True)+'\n')


if __name__ == '__main__':
    main()
