"""Unit tests for the faithful sensor-attack perturbations and installer.

Pure logic only: no simulator. A FakeSensor stands in for a Pegasus sensor so the
installer and wrapper are exercised without Isaac."""
import math
import pytest
from sensor_attacks import (validate_attack,attack_metadata,install_sensor_attack,
                            _perturb_gps,_perturb_imu,_perturb_mag,_schedule_active,_M_PER_DEG,ATTACKS)

GPS={'latitude':40.44,'longitude':-79.94,'altitude':5.,'latitude_gt':40.44,'longitude_gt':-79.94,'altitude_gt':5.}
IMU={'orientation':[1.,0.,0.,0.],'angular_velocity':[0.,0.,0.],'linear_acceleration':[0.,0.,0.]}
MAG={'magnetic_field':[0.2,0.0,0.4]}


class FakeSensor:
    """Returns a fresh reading copy; None on odd ticks models update_at_rate gaps."""
    def __init__(self,sensor_type='GPS',reading=None,skip_none=False):
        self.sensor_type=sensor_type;self._reading=reading or GPS;self.calls=0;self.skip_none=skip_none
    def update(self,state,dt):
        self.calls+=1
        if self.skip_none and self.calls%2==1:return None
        return dict(self._reading)


class FakeVehicle:
    def __init__(self,*sensors):self._sensors=list(sensors)


class FakeState:
    """Stands in for the Pegasus State the sensor.update() receives (ENU world position)."""
    def __init__(self,position=None):self.position=position


IMU_PROX={'location':[0.,0.,1.],'range_m':5.}  # disruptor placement for proximity-gated tests
MAG_PROX={'location':[0.,0.,1.],'range_m':5.}


# ---- validation -----------------------------------------------------------
def test_validate_defaults_and_normalises_axis():
    cfg=validate_attack({'id':'B1_gps_spoof','params':{'axis':[3,0,4]}})
    assert cfg['path']=='faithful' and cfg['seed']==0 and cfg['schedule'] is None
    assert cfg['params']['axis']==pytest.approx([.6,0.,.8])  # unit vector
    assert cfg['params']['rate_mps']==.3 and cfg['params']['max_offset_m']==10.

def test_validate_rejects_unknown_and_bad():
    for bad in [{},{'id':'nope'},{'id':'B1_gps_spoof','path':'shortcut'},
                {'id':'B1_gps_spoof','junk':1},{'id':'B1_gps_spoof','seed':-1},
                {'id':'B1_gps_spoof','params':{'axis':[0,0,0]}},
                {'id':'B1_gps_spoof','params':{'rate_mps':0}},
                {'id':'B1_gps_spoof','params':{'max_offset_m':0}},
                {'id':'B1_gps_spoof','params':{'bogus':1}}]:
        with pytest.raises(ValueError):validate_attack(bad)

def test_schedule_validated_through_conditions():
    cfg=validate_attack({'id':'B1_gps_spoof','schedule':{'start_s':20,'duration_s':0,'period_s':0}})
    assert cfg['schedule']=={'start_s':20.,'duration_s':0.,'period_s':0.}
    with pytest.raises(ValueError):
        validate_attack({'id':'B1_gps_spoof','schedule':{'start_s':20}})

def test_metadata_reports_defence_and_fidelity():
    meta=attack_metadata({'id':'B1_gps_spoof','params':{}})
    assert meta['fidelity']=='faithful'
    assert meta['defense_exercised']==ATTACKS['B1_gps_spoof']['defense_exercised']


# ---- perturbation maths ---------------------------------------------------
def test_gps_ramp_is_deterministic_and_bounded():
    p=validate_attack({'id':'B1_gps_spoof','params':{'axis':[0,1,0],'rate_mps':1.,'max_offset_m':5.}})['params']
    at2=_perturb_gps(GPS,2.,p,0,1)   # 2 m north
    assert at2['latitude']==pytest.approx(GPS['latitude']+2./_M_PER_DEG)
    assert at2['longitude']==GPS['longitude'] and at2['altitude']==GPS['altitude']
    at100=_perturb_gps(GPS,100.,p,0,2)  # ramp capped at max_offset_m
    assert at100['latitude']==pytest.approx(GPS['latitude']+5./_M_PER_DEG)
    assert _perturb_gps(GPS,2.,p,0,1)==at2  # replayable

def test_gps_leaves_ground_truth_fields():
    p=validate_attack({'id':'B1_gps_spoof','params':{'axis':[1,0,0],'rate_mps':1.}})['params']
    out=_perturb_gps(GPS,3.,p,0,1)
    assert out['latitude_gt']==GPS['latitude_gt'] and out['longitude_gt']==GPS['longitude_gt']
    assert out['longitude']!=GPS['longitude']  # east axis moves longitude

def test_gps_altitude_axis():
    p=validate_attack({'id':'B1_gps_spoof','params':{'axis':[0,0,1],'rate_mps':2.,'max_offset_m':50.}})['params']
    assert _perturb_gps(GPS,3.,p,0,1)['altitude']==pytest.approx(GPS['altitude']+6.)

def test_jitter_is_seeded():
    np=pytest.importorskip('numpy')
    p=validate_attack({'id':'B1_gps_spoof','params':{'rate_mps':1.,'jitter_m':2.}})['params']
    a=_perturb_gps(GPS,1.,p,7,1);b=_perturb_gps(GPS,1.,p,7,1);c=_perturb_gps(GPS,1.,p,7,2)
    assert a==b and a!=c  # same (seed,sequence) replays; different sequence differs


# ---- B4 IMU (acoustic resonance) ------------------------------------------
def test_imu_validation_defaults_resonance_centric():
    p=validate_attack({'id':'B4_imu_spoof','params':IMU_PROX})['params']
    assert p['target']=='gyroscope' and p['axis']==pytest.approx([0.,0.,1.])
    assert p['resonance_amplitude']==.5 and p['resonance_hz']==12.  # resonance on by default
    assert p['rate']==0. and p['max_bias']==0.                       # bias off by default
    assert p['location']==[0.,0.,1.] and p['range_m']==5.
    assert validate_attack({'id':'B4_imu_spoof','params':{'target':'accelerometer',**IMU_PROX}})['params']['resonance_amplitude']==4.

def test_imu_validation_rejects_bad():
    for bad in [{'target':'compass'},{'resonance_amplitude':0},            # no effect at all
                {'max_bias':1,'rate':0},                                   # bias cannot ramp
                {'rate':99},{'axis':[0,0,0]},{'bogus':1},
                {'location':[0,0]},{'range_m':0}]:                         # proximity params required/bounded
        with pytest.raises(ValueError):validate_attack({'id':'B4_imu_spoof','params':{**IMU_PROX,**bad}})
    with pytest.raises(ValueError):validate_attack({'id':'B4_imu_spoof','params':{}})  # location is required

def test_imu_resonance_is_deterministic_and_axis_targeted():
    p=validate_attack({'id':'B4_imu_spoof','params':{'axis':[1,0,0],'resonance_hz':1.,'resonance_amplitude':2.,**IMU_PROX}})['params']
    assert _perturb_imu(IMU,0.,p,0,1)['angular_velocity']==[0.,0.,0.]        # sin(0)=0
    out=_perturb_imu(IMU,0.25,p,0,1)                                         # quarter period -> peak
    assert out['angular_velocity']==pytest.approx([2.,0.,0.])
    assert out['linear_acceleration']==IMU['linear_acceleration'] and out['orientation']==IMU['orientation']
    assert _perturb_imu(IMU,0.25,p,0,1)==out                                 # replayable
    assert _perturb_imu(IMU,0.25,p,0,1,factor=0.5)['angular_velocity']==pytest.approx([1.,0.,0.])  # proximity scales

def test_imu_bias_ramp_is_bounded():
    p=validate_attack({'id':'B4_imu_spoof','params':{'axis':[0,0,1],'resonance_amplitude':0,'rate':1.,'max_bias':3.,**IMU_PROX}})['params']
    assert _perturb_imu(IMU,2.,p,0,1)['angular_velocity'][2]==pytest.approx(2.)
    assert _perturb_imu(IMU,100.,p,0,2)['angular_velocity'][2]==pytest.approx(3.)  # capped at max_bias


# ---- B5 Magnetometer (external field bias) --------------------------------
def test_mag_additive_bias_is_deterministic_and_bounded():
    p=validate_attack({'id':'B5_mag_spoof','params':{'axis':[1,0,0],'rate_gauss_per_s':1.,'max_bias_gauss':2.,**MAG_PROX}})['params']
    at1=_perturb_mag(MAG,1.,p,0,1)
    assert at1['magnetic_field']==pytest.approx([1.2,0.,0.4])                # +1 gauss east; other axes intact
    assert _perturb_mag(MAG,5.,p,0,2)['magnetic_field'][0]==pytest.approx(2.2)  # capped at max_bias_gauss
    assert _perturb_mag(MAG,1.,p,0,1)==at1                                   # replayable
    assert _perturb_mag(MAG,1.,p,0,1,factor=0.)['magnetic_field']==pytest.approx(MAG['magnetic_field'])  # out of range: no change

def test_mag_validation_rejects_bad():
    for bad in [{'rate_gauss_per_s':0},{'max_bias_gauss':0},{'axis':[0,0,0]},{'bogus':1},{'location':'x'}]:
        with pytest.raises(ValueError):validate_attack({'id':'B5_mag_spoof','params':{**MAG_PROX,**bad}})
    with pytest.raises(ValueError):validate_attack({'id':'B5_mag_spoof','params':{}})  # location is required


# ---- physical realism recorded --------------------------------------------
def test_metadata_records_physical_mechanism_and_plausibility():
    for aid,params in (('B1_gps_spoof',{}),('B4_imu_spoof',IMU_PROX),('B5_mag_spoof',MAG_PROX)):
        meta=attack_metadata({'id':aid,'params':params})
        assert meta['physical_mechanism']==ATTACKS[aid]['physical_mechanism']
        assert meta['standoff_plausibility']==ATTACKS[aid]['standoff_plausibility']


# ---- proximity gating -----------------------------------------------------
def test_proximity_attacks_only_fire_within_range_of_the_disruptor():
    # Bias/field terms make the change phase-independent (a bare resonance can sample sin=0).
    near=FakeState([0.,0.,1.]);far=FakeState([100.,0.,1.])  # disruptor at [0,0,1], range 5 m
    for aid,stype,field,extra in (('B4_imu_spoof','IMU','angular_velocity',{'rate':1.,'max_bias':2.,'resonance_amplitude':0}),
                                  ('B5_mag_spoof','Magnetometer','magnetic_field',{})):
        reading=IMU if stype=='IMU' else MAG
        sensor=FakeSensor(stype,reading=reading);clock=[0.]
        cfg={'id':aid,'params':{'location':[0.,0.,1.],'range_m':5.,**extra},
             'schedule':{'start_s':2.,'duration_s':0.,'period_s':0.}}
        state=install_sensor_attack(FakeVehicle(FakeSensor('GPS'),sensor),cfg,lambda:clock[0])
        clock[0]=0.;sensor.update(near,0.)          # pre-window: anchor only
        clock[0]=3.;out_far=sensor.update(far,0.)   # device on, drone far -> no effect
        assert out_far[field]==reading[field] and state.perturbed==0 and not state.summary()['active']
        clock[0]=4.;out_near=sensor.update(near,0.) # drone at the disruptor -> effect
        assert out_near[field]!=reading[field] and state.perturbed==1 and state.summary()['active']
        clock[0]=5.;assert sensor.update(FakeState(None),0.)[field]==reading[field]  # unknown position: fail-safe off

def test_installer_missing_magnetometer():
    with pytest.raises(ValueError):  # no magnetometer present to attack
        install_sensor_attack(FakeVehicle(FakeSensor('GPS')),{'id':'B5_mag_spoof','params':MAG_PROX},lambda:0.)


# ---- schedule windows -----------------------------------------------------
def test_schedule_windows():
    assert _schedule_active(None,999) is True
    s={'start_s':10.,'duration_s':0.,'period_s':0.}
    assert not _schedule_active(s,9.9) and _schedule_active(s,10.1)
    rep={'start_s':0.,'duration_s':2.,'period_s':5.}
    assert _schedule_active(rep,1.) and not _schedule_active(rep,3.) and _schedule_active(rep,5.5)


# ---- installer wrapping ---------------------------------------------------
def test_installer_wraps_and_ramps_from_window_open():
    sensor=FakeSensor('GPS');clock=[0.]
    state=install_sensor_attack(FakeVehicle(sensor,FakeSensor('IMU')),
        {'id':'B1_gps_spoof','params':{'axis':[0,1,0],'rate_mps':1.,'max_offset_m':100.},
         'schedule':{'start_s':10.,'duration_s':0.,'period_s':0.}},lambda:clock[0])
    clock[0]=0.;sensor.update(None,0.)   # arm: anchor at first update, window opens 10 s later
    clock[0]=5.;assert sensor.update(None,0.)['latitude']==GPS['latitude']  # before window: clean
    assert not state.summary()['active'] and state.perturbed==0
    clock[0]=13.;opened=sensor.update(None,0.)  # window just open → zero offset, but counted
    assert opened['latitude']==pytest.approx(GPS['latitude']) and state.summary()['active'] and state.perturbed==1
    clock[0]=16.;moved=sensor.update(None,0.)  # 3 s into the open window → 3 m north
    assert moved['latitude']==pytest.approx(GPS['latitude']+3./_M_PER_DEG)

def test_installer_passes_none_through():
    sensor=FakeSensor('GPS',skip_none=True)
    install_sensor_attack(FakeVehicle(sensor),{'id':'B1_gps_spoof','params':{}},lambda:1.)
    assert sensor.update(None,0.) is None      # odd tick: sensor not due
    assert sensor.update(None,0.) is not None   # even tick: perturbed reading

def test_installer_missing_sensor():
    with pytest.raises(ValueError):
        install_sensor_attack(FakeVehicle(FakeSensor('IMU')),{'id':'B1_gps_spoof','params':{}},lambda:0.)
