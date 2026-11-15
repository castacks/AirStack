import unittest

try:
    import pytest
except ImportError as error:
    raise unittest.SkipTest("optional UAV pytest suite: " + str(error))
from rrm.uav_acceptance import assess


def run(samples,**kw):
    return assess(samples,min_altitude=1,max_altitude=3,bounds_xy=(-2,2,-2,2),**kw)


def track():return [dict(t=i*.1,x=i*.1,y=0,z=2) for i in range(5)]


def test_no_physical_pass_with_missing_evidence():
    assert run(track())['verdict']=='NOT DEMONSTRATED'


def test_complete_independent_evidence():
    assert run(track(),collision=False,final_disarmed=True,truth_verified=True)['verdict']=='PASS'


def test_violation_takes_precedence_over_missing_evidence():
    s=track();s[1]['z']=.5
    v=run(s);assert v['verdict']=='FAIL' and v['samples_below_min']==1
    assert v['below_min_observed_s']==pytest.approx(.1)


def test_boundary_distance():
    s=track();s[2]['x']=2.4
    assert run(s)['max_boundary_violation_m']==pytest.approx(.4)


def test_no_integration_over_capture_gaps():
    s=[dict(t=0,x=0,y=0,z=2),dict(t=3,x=1,y=0,z=2)]
    v=run(s,collision=False,final_disarmed=True,truth_verified=True)
    assert v['verdict']=='NOT DEMONSTRATED' and v['distance_observed_m']==0 and v['missing_intervals']==1


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),True,None])
def test_bad_samples_invalid(bad):
    s=track();s[1]['x']=bad
    assert run(s)['verdict']=='INVALID'


def test_clock_reset_invalid():
    s=track();s[2]['t']=0
    assert run(s)['verdict']=='INVALID'


def test_collision_and_armed_fail():
    assert run(track(),collision=True)['verdict']=='FAIL'
    assert run(track(),final_disarmed=False)['verdict']=='FAIL'
