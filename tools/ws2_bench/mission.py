"""Model-specific evaluation; Kim remains a reactive policy without a goal."""
import math

SUCCESSES={'goal_reached','completed_horizon'}
EXCLUDED={'infrastructure_error','user_stopped'}

def defaults(planner):
    if planner=='kim':
        return dict(mission_mode='avoidance',timeout=120.,goal_distance=8.,goal_radius=.5,
                    minimum_travel=3.,minimum_displacement=1.,maximum_stationary_fraction=.5,initial_speed=.2,maximum_speed=.35,
                    trajectory_horizon=2.,velocity=.4)
    return dict(mission_mode='goal',timeout=180.,goal_distance=8.,goal_radius=.5,
                minimum_travel=3.,minimum_displacement=1.,maximum_stationary_fraction=.5,initial_speed=.4,maximum_speed=.5,
                trajectory_horizon=2.,velocity=.3)

def motion_metrics(samples):
    if not samples:return {'max_displacement_m':0.,'mean_speed_m_s':0.,'stationary_fraction':None}
    length=sum(math.dist(a['position_m'],b['position_m']) for a,b in zip(samples,samples[1:]))
    duration=samples[-1]['sim_time_s']-samples[0]['sim_time_s']
    # One-second windows prevent sub-frame position noise from dominating stop time.
    windows=[];anchor=samples[0]
    for s in samples[1:]:
        dt=s['sim_time_s']-anchor['sim_time_s']
        if dt>=1:
            windows.append((dt,math.dist(s['position_m'],anchor['position_m'])/dt));anchor=s
    return {'max_displacement_m':max(math.dist(samples[0]['position_m'],s['position_m']) for s in samples),
            'mean_speed_m_s':length/duration if duration>0 else 0.,
            'stationary_fraction':sum(dt for dt,v in windows if v<.03)/sum(dt for dt,v in windows) if windows else None}

def horizon_outcome(config,samples):
    if config['mission_mode']=='goal':return 'timeout'
    length=sum(math.dist(a['position_m'],b['position_m']) for a,b in zip(samples,samples[1:]))
    motion=motion_metrics(samples)
    return ('completed_horizon' if length>=config['minimum_travel'] and
            motion['max_displacement_m']>=config['minimum_displacement'] and
            motion['stationary_fraction'] is not None and motion['stationary_fraction']<=config['maximum_stationary_fraction']
            else 'insufficient_progress')
