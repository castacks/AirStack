"""Independent offline envelope assessment; action success is never an input.

Input samples use seconds, map metres and explicit mission phase. Position-only
coverage/contact/tracking facts are not fabricated. Integrals are left-held on
observed intervals, never extrapolated across missing or oversized gaps.
"""
import math


def assess(samples, *, min_altitude, max_altitude, bounds_xy, max_gap_s=0.5,
           collision=None, final_disarmed=None, truth_verified=False):
    values=(min_altitude,max_altitude,*bounds_xy,max_gap_s)
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in values):
        raise ValueError('finite numeric limits required')
    xmin,xmax,ymin,ymax=bounds_xy
    if min_altitude>=max_altitude or xmin>=xmax or ymin>=ymax or max_gap_s<=0:
        raise ValueError('invalid envelope')
    if not samples:return {'verdict':'INVALID','reason':'no_samples'}
    required=('t','x','y','z')
    if any(any(type(s.get(k)) not in (int,float) or not math.isfinite(s[k]) for k in required) for s in samples):
        return {'verdict':'INVALID','reason':'nonfinite_or_missing_sample'}
    gaps=[b['t']-a['t'] for a,b in zip(samples,samples[1:])]
    if any(dt<=0 for dt in gaps):return {'verdict':'INVALID','reason':'nonmonotonic_time'}
    below=[s['z']<min_altitude for s in samples]
    over=[s['z']>max_altitude for s in samples]
    violations=[max(xmin-s['x'],s['x']-xmax,ymin-s['y'],s['y']-ymax,0) for s in samples]
    outside=[v>0 for v in violations]
    result=dict(min_altitude_m=min(s['z'] for s in samples),max_altitude_m=max(s['z'] for s in samples),
                samples=len(samples),samples_below_min=sum(below),samples_above_max=sum(over),
                samples_outside=sum(outside),max_boundary_violation_m=max(violations),
                observed_duration_s=sum(dt for dt in gaps if dt<=max_gap_s),
                missing_intervals=sum(dt>max_gap_s for dt in gaps),
                below_min_observed_s=sum(dt for dt,bad in zip(gaps,below) if bad and dt<=max_gap_s),
                outside_observed_s=sum(dt for dt,bad in zip(gaps,outside) if bad and dt<=max_gap_s),
                distance_observed_m=sum(math.dist([a[k] for k in ('x','y','z')],[b[k] for k in ('x','y','z')])
                                        for a,b,dt in zip(samples,samples[1:],gaps) if dt<=max_gap_s),
                collision=collision,final_disarmed=final_disarmed,truth_verified=truth_verified,
                coverage_percent=None,tracking_rmse_m=None)
    if any(below) or any(over) or any(outside) or collision is True or final_disarmed is False:
        result['verdict']='FAIL'
    elif len(samples)<2 or result['missing_intervals'] or collision is not False or final_disarmed is not True or truth_verified is not True:
        result['verdict']='NOT DEMONSTRATED'
    else:result['verdict']='PASS'
    return result
