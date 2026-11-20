"""Faithful black-box sensor attacks for the WS2 bench.

A real sensor spoofer makes the receiver *report* a false measurement; the flight
controller's estimator then fuses it. The honest place to model that is the output
of a Pegasus sensor model's ``update()`` — between the sensor and the PX4 MAVLink
backend (see vehicles/vehicle.py:update_sensors), below the estimator. We wrap the
sensor instance's bound ``update`` from the simulation launch script at spawn time,
so neither AirStack nor Pegasus source is edited: the attack lives entirely in the
simulation environment.

Perturbations are pure and replayable: ``perturb(reading, active_elapsed_s, params,
seed, sequence) -> reading``. numpy is imported lazily so the ramp path (no jitter)
runs without it.
"""
import math

_M_PER_DEG=111320.  # metres per degree of latitude (and of longitude at the equator)


def _rng(seed,sequence):
    import numpy as np  # lazy: only the jitter path needs it
    return np.random.default_rng(np.random.SeedSequence(
        [int(seed)&0xFFFFFFFF,(int(seed)>>32)&0xFFFFFFFF,int(sequence)&0xFFFFFFFF]))


def _number(name,value,lo,hi,*,low_open=False):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    if (value<=lo if low_open else value<lo) or value>hi:
        bound='(' if low_open else '['
        raise ValueError(f'{name} must be in {bound}{lo}, {hi}]')
    return float(value)


def _unit_axis(raw,name='axis',default=(0.,0.,1.)):
    axis=raw.get(name,list(default))
    if not (isinstance(axis,(list,tuple)) and len(axis)==3 and
            all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in axis)):
        raise ValueError(name+' must be three finite numbers')
    norm=math.sqrt(sum(v*v for v in axis))
    if norm==0:raise ValueError(name+' must be nonzero')
    return [v/norm for v in axis]


def _location(raw,name='location'):
    """A fixed world position for a close-proximity disruptor (ENU world frame)."""
    loc=raw.get(name)
    if not (isinstance(loc,(list,tuple)) and len(loc)==3 and
            all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in loc)):
        raise ValueError(name+' must be three finite numbers [x, y, z] in the ENU world frame')
    return [float(v) for v in loc]


def _proximity_factor(position,location,range_m):
    """Spatial gate for a localized disruptor: 1 at the device, tapering linearly to
    0 at range_m and beyond. A bounded proxy for acoustic/magnetic intensity (really
    ~1/r^2..1/r^3) whose point is that the effect is local, not global. An unknown
    drone position yields 0 (fail-safe: never claim a proximity hit we cannot place)."""
    if position is None:return 0.
    dist=math.sqrt(sum((float(position[i])-location[i])**2 for i in range(3)))
    return max(0.,1.-dist/range_m)


def _validate_gps_params(raw):
    """A slow positional spoof of the reported fix: drift along an ENU axis."""
    if not isinstance(raw,dict):raise ValueError('gps spoof params must be a mapping')
    extra=set(raw)-{'axis','rate_mps','max_offset_m','jitter_m'}
    if extra:raise ValueError('unknown gps spoof params: '+str(extra))
    axis=raw.get('axis',[1.,0.,0.])
    if not (isinstance(axis,(list,tuple)) and len(axis)==3 and
            all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in axis)):
        raise ValueError('axis must be three finite numbers [east, north, up]')
    norm=math.sqrt(sum(v*v for v in axis))
    if norm==0:raise ValueError('axis must be nonzero')
    return {'axis':[v/norm for v in axis],
            'rate_mps':_number('rate_mps',raw.get('rate_mps',.3),0,5,low_open=True),
            'max_offset_m':_number('max_offset_m',raw.get('max_offset_m',10.),0,200,low_open=True),
            'jitter_m':_number('jitter_m',raw.get('jitter_m',0.),0,10)}


def _perturb_gps(reading,active_elapsed_s,p,seed,sequence,factor=1.):
    """Shift the reported lat/lon/alt by a ramped ENU offset; leave *_gt untouched.
    ``factor`` is the spatial gate (1 for GPS: a remote RF spoofer reaches the scene)."""
    magnitude=min(p['rate_mps']*active_elapsed_s,p['max_offset_m'])*factor
    east,north,up=(component*magnitude for component in p['axis'])
    if p['jitter_m']:
        jitter=_rng(seed,sequence).normal(0.,p['jitter_m'],3)
        east+=float(jitter[0]);north+=float(jitter[1]);up+=float(jitter[2])
    latitude=reading['latitude']
    out=dict(reading)
    out['latitude']=latitude+north/_M_PER_DEG
    out['longitude']=reading['longitude']+east/(_M_PER_DEG*math.cos(math.radians(latitude)))
    out['altitude']=reading['altitude']+up
    return out


# --- B4 IMU (acoustic resonance injection) ---------------------------------
# Physically modelled on acoustic attacks against MEMS inertial sensors: Son et al.
# "Rocking Drones" (USENIX Security 2015) drove gyros at their resonant frequency;
# Trippel et al. "WALNUT" (EuroS&P 2017) injected a rectified DC bias into accels.
# So the resonant oscillation is the PRIMARY effect and the ramped bias is secondary
# and bounded; magnitudes stay sub-saturation for a plausible close-range source.
_IMU_FIELD={'gyroscope':'angular_velocity','accelerometer':'linear_acceleration'}


def _validate_imu_params(raw):
    if not isinstance(raw,dict):raise ValueError('imu spoof params must be a mapping')
    extra=set(raw)-{'target','axis','rate','max_bias','resonance_hz','resonance_amplitude','jitter','location','range_m'}
    if extra:raise ValueError('unknown imu spoof params: '+str(extra))
    target=raw.get('target','gyroscope')
    if target not in _IMU_FIELD:raise ValueError("imu target must be 'gyroscope' or 'accelerometer'")
    cap=5. if target=='gyroscope' else 40.  # rad/s (gyro) vs m/s^2 (accel); generous but sub-saturation
    p={'target':target,'axis':_unit_axis(raw),
       'rate':_number('rate',raw.get('rate',0.),0,cap),
       'max_bias':_number('max_bias',raw.get('max_bias',0.),0,cap),
       'resonance_hz':_number('resonance_hz',raw.get('resonance_hz',12.),0,100),
       'resonance_amplitude':_number('resonance_amplitude',raw.get('resonance_amplitude',.5 if target=='gyroscope' else 4.),0,cap),
       'jitter':_number('jitter',raw.get('jitter',0.),0,cap),
       # Close-range acoustic source at a fixed world position; effect is distance-gated.
       'location':_location(raw),'range_m':_number('range_m',raw.get('range_m',4.),0,50,low_open=True)}
    if p['max_bias']>0 and p['rate']<=0:raise ValueError('imu bias (max_bias>0) needs rate>0 to ramp')
    if not ((p['resonance_amplitude']>0 and p['resonance_hz']>0) or p['max_bias']>0 or p['jitter']>0):
        raise ValueError('imu attack must inject resonance, bias, or jitter')
    return p


def _perturb_imu(reading,active_elapsed_s,p,seed,sequence,factor=1.):
    """Add a resonant oscillation (+ optional ramped bias / jitter) to gyro or accel,
    scaled by ``factor`` (the proximity gate: the acoustic source is local)."""
    delta=min(p['rate']*active_elapsed_s,p['max_bias'])  # secondary rectified DC bias (WALNUT)
    if p['resonance_amplitude'] and p['resonance_hz']:   # primary: acoustic resonance (Son)
        delta+=p['resonance_amplitude']*math.sin(2*math.pi*p['resonance_hz']*active_elapsed_s)
    deltas=[component*delta for component in p['axis']]
    if p['jitter']:
        jitter=_rng(seed,sequence).normal(0.,p['jitter'],3)
        deltas=[d+float(jitter[i]) for i,d in enumerate(deltas)]
    field=_IMU_FIELD[p['target']]
    out=dict(reading)
    out[field]=[float(v)+factor*deltas[i] for i,v in enumerate(reading[field])]
    return out


# --- B5 Magnetometer (external magnetic field) -----------------------------
# A nearby magnet or energised coil adds a quasi-static field to the sensor, biasing
# the heading reference. Earth's field is ~0.25-0.65 gauss, so a comparable additive
# bias meaningfully corrupts yaw. An additive vector (not a pure rotation) is the
# honest model: an external field changes the measured magnitude as well as direction.
def _validate_mag_params(raw):
    if not isinstance(raw,dict):raise ValueError('mag spoof params must be a mapping')
    extra=set(raw)-{'axis','rate_gauss_per_s','max_bias_gauss','jitter_gauss','location','range_m'}
    if extra:raise ValueError('unknown mag spoof params: '+str(extra))
    return {'axis':_unit_axis(raw),
            'rate_gauss_per_s':_number('rate_gauss_per_s',raw.get('rate_gauss_per_s',.1),0,5,low_open=True),
            'max_bias_gauss':_number('max_bias_gauss',raw.get('max_bias_gauss',.5),0,5,low_open=True),
            'jitter_gauss':_number('jitter_gauss',raw.get('jitter_gauss',0.),0,1),
            # Magnet/coil at a fixed world position; field falls off fast, so gate by distance.
            'location':_location(raw),'range_m':_number('range_m',raw.get('range_m',2.5),0,50,low_open=True)}


def _perturb_mag(reading,active_elapsed_s,p,seed,sequence,factor=1.):
    """Add a ramped external-field bias to the body-frame field, scaled by ``factor``
    (the proximity gate: the magnet/coil is local)."""
    magnitude=min(p['rate_gauss_per_s']*active_elapsed_s,p['max_bias_gauss'])
    bias=[component*magnitude for component in p['axis']]
    if p['jitter_gauss']:
        jitter=_rng(seed,sequence).normal(0.,p['jitter_gauss'],3)
        bias=[b+float(jitter[i]) for i,b in enumerate(bias)]
    out=dict(reading)
    out['magnetic_field']=[float(v)+factor*bias[i] for i,v in enumerate(reading['magnetic_field'])]
    return out


# id -> {sensor_type, supported paths, param validator, perturbation, defense it stresses,
#        physical_mechanism + standoff_plausibility so every result stays honest about realism}
ATTACKS={
    'B1_gps_spoof':{'sensor_type':'GPS','paths':{'faithful'},'proximity':False,
                    'validate_params':_validate_gps_params,'perturb':_perturb_gps,
                    'defense_exercised':'EKF innovation gate / GPS-vision position consistency',
                    'physical_mechanism':'Counterfeit GNSS broadcast overriding the live fix (Humphreys/UT Austin UAV and vessel spoofing demonstrations).',
                    'standoff_plausibility':'high: remote standoff RF reaching the whole scene, demonstrated against real UAVs'},
    'B4_imu_spoof':{'sensor_type':'IMU','paths':{'faithful'},'proximity':True,
                    'validate_params':_validate_imu_params,'perturb':_perturb_imu,
                    'defense_exercised':'attitude/rate estimator consistency, vibration/clipping monitor',
                    'physical_mechanism':'Acoustic resonance injected into the MEMS gyro/accelerometer (Son et al. USENIX 2015; Trippel et al. WALNUT, EuroS&P 2017).',
                    'standoff_plausibility':'medium: in-flight close-range line-of-sight acoustic source at a fixed location (effect gated by distance; bounded by resonant frequency/coupling)'},
    'B5_mag_spoof':{'sensor_type':'Magnetometer','paths':{'faithful'},'proximity':True,
                    'validate_params':_validate_mag_params,'perturb':_perturb_mag,
                    'defense_exercised':'heading/yaw EKF consistency, mag-vs-GPS course cross-check',
                    'physical_mechanism':'External magnetic field from a magnet or energised coil placed on the flight path biasing the heading reference.',
                    'standoff_plausibility':'medium: in-flight magnet/coil at a fixed location (effect gated by distance; field falls off ~1/r^3)'},
}


def validate_attack(raw):
    """Normalise and range-check an episode ``attack`` block; replayable output."""
    if not isinstance(raw,dict):raise ValueError('attack must be a mapping')
    extra=set(raw)-{'id','path','params','schedule','seed'}
    if extra:raise ValueError('unknown attack fields: '+str(extra))
    spec=ATTACKS.get(raw.get('id'))
    if spec is None:raise ValueError('unknown attack id: '+repr(raw.get('id')))
    path=raw.get('path','faithful')
    if path not in spec['paths']:
        raise ValueError(f"{raw['id']} does not support path {path!r}; supported: {sorted(spec['paths'])}")
    seed=raw.get('seed',0)
    if isinstance(seed,bool) or not isinstance(seed,int) or not 0<=seed<2**31:
        raise ValueError('attack seed must be a nonnegative 31-bit integer')
    schedule=None
    if raw.get('schedule') not in (None,{}):
        from conditions import validate_schedule
        schedule=validate_schedule(raw['schedule'])
    return {'id':raw['id'],'path':path,'seed':seed,'schedule':schedule,
            'params':spec['validate_params'](raw.get('params',{}))}


def attack_metadata(cfg):
    """Metrics block (brief §6): fidelity, defense stressed, and the real-world
    attack this run emulates (physical_mechanism + standoff_plausibility), so the
    results never overclaim what a remote adversary could actually do."""
    cfg=validate_attack(cfg)  # idempotent on already-validated config
    spec=ATTACKS[cfg['id']]
    return {'id':cfg['id'],'fidelity':cfg['path'],'seed':cfg['seed'],
            'schedule':cfg['schedule'],'params':cfg['params'],
            'defense_exercised':spec['defense_exercised'],
            'physical_mechanism':spec['physical_mechanism'],
            'standoff_plausibility':spec['standoff_plausibility']}


def _schedule_active(schedule,elapsed_s):
    """Half-open windows on the attack clock; None means always active."""
    if schedule is None:return True
    if elapsed_s<schedule['start_s']:return False
    offset=elapsed_s-schedule['start_s']
    if schedule['period_s']:offset%=schedule['period_s']
    return schedule['duration_s']==0 or offset<schedule['duration_s']


class InstalledAttack:
    """Live state of a wrapped sensor; its summary is written to scene evidence."""
    def __init__(self,cfg):
        self.cfg=cfg;self.anchor=None;self.active_since=None;self.sequence=0;self.perturbed=0

    def summary(self):
        # 'active' tracks samples that actually perturbed the reading; for a proximity
        # attack that means the drone was within range, not merely that the device is on.
        return {'id':self.cfg['id'],'fidelity':self.cfg['path'],'seed':self.cfg['seed'],
                'schedule':self.cfg['schedule'],'params':self.cfg['params'],
                'perturbed_samples':self.perturbed,'active':self.perturbed>0}


def _find_sensor(vehicle,sensor_type):
    for sensor in getattr(vehicle,'_sensors',[]):
        if sensor.sensor_type==sensor_type:return sensor
    raise ValueError(f'no {sensor_type} sensor on the vehicle to attack')


def install_sensor_attack(vehicle,raw,clock):
    """Wrap the targeted sensor's update() in place. ``clock`` returns sim seconds.

    The attack clock anchors at the first update and the ramp starts from zero when
    the schedule window first opens, so ``schedule.start_s`` is the onset time.
    """
    cfg=validate_attack(raw)
    spec=ATTACKS[cfg['id']]
    sensor=_find_sensor(vehicle,spec['sensor_type'])
    original=sensor.update
    state=InstalledAttack(cfg)
    perturb,params,seed,schedule=spec['perturb'],cfg['params'],cfg['seed'],cfg['schedule']
    proximity=spec.get('proximity',False)  # a localized disruptor only affects a nearby drone

    def wrapped(vehicle_state,dt):
        reading=original(vehicle_state,dt)
        if reading is None:return None  # sensor not due this tick (update_at_rate)
        now=float(clock())
        if state.anchor is None:state.anchor=now
        elapsed=max(0.,now-state.anchor)
        if not _schedule_active(schedule,elapsed):return reading
        if state.active_since is None:state.active_since=now  # device turned on (ramp clock), independent of range
        factor=1.
        if proximity:
            factor=_proximity_factor(getattr(vehicle_state,'position',None),params['location'],params['range_m'])
            if factor<=0.:return reading  # device on but the drone is out of the disruptor's range this tick
        state.sequence+=1;state.perturbed+=1
        return perturb(reading,now-state.active_since,params,seed,state.sequence,factor)

    sensor.update=wrapped
    return state
