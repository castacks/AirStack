"""Expiry/pending/burst contracts and actual requested-observation masking."""
import pytest
from Scripts.Setup.probe_state_cache import ProbeCache,SECOND

def ready(cache,c,p,t):
 cache.health_event(c,p,True,t);cache.motion_event(c,p,True,t+1)

def test_unknown_refresh_and_pending_does_not_become_failure():
 s=ProbeCache('bounded_probe');d=s.plan(0,0);assert d['action']=='ABSTAIN' and d['requests']==['rgb','thermal']
 s.health_event('rgb',0,True,10);assert s.view('rgb',10)['status']=='UNKNOWN'
 s.motion_event('rgb',0,True,11);assert s.plan(1,12)['action']=='RGB'
 s.health_event('rgb',1,True,13);assert s.view('rgb',13)['value'] is True and s.view('rgb',13)['pair']==0

def test_late_old_write_cannot_replace_new_failure_or_freshen_it():
 s=ProbeCache('full_read_cache');s.plan(0,0);s.health_event('rgb',0,True,1);s.plan(1,2);s.health_event('rgb',1,False,3);s.motion_event('rgb',0,True,4)
 v=s.view('rgb',4);assert v['value'] is False and v['pair']==1 and v['receipt_ns']==3

def test_stale_boundary_and_refresh():
 s=ProbeCache('bounded_no_probe');s.plan(0,0);ready(s,'rgb',0,1)
 assert s.view('rgb',SECOND+1)['value'] is True
 assert s.view('rgb',SECOND+2)['status']=='STALE'
 d=s.plan(1,SECOND+2);assert d['action']=='ABSTAIN' and d['requests']==['rgb','thermal']

def test_unrequested_health_and_motion_do_not_change_state():
 s=ProbeCache('fixed_rgb');s.plan(0,0)
 assert not s.health_event('thermal',0,False,1);assert not s.motion_event('thermal',0,True,2);assert s.view('thermal',2)['status']=='UNKNOWN'

def test_two_request_burst_finishes_when_reply_pending():
 s=ProbeCache('bounded_probe');s.plan(0,0,True);ready(s,'rgb',0,1);s.health_event('thermal',0,False,3)
 s.plan(1,SECOND//2);ready(s,'rgb',1,SECOND//2+1)
 d=s.plan(2,SECOND+10);assert d['requests']==['rgb','thermal'] and d['burst_after']['thermal']==1
 ready(s,'rgb',2,SECOND+11);s.health_event('thermal',2,True,SECOND+13)
 d=s.plan(3,SECOND+14);assert 'thermal' in d['requests'] and d['burst_after']['thermal']==0

def test_motion_before_health_resolves_only_when_health_arrives():
 s=ProbeCache('full_read_cache');s.plan(0,0);s.motion_event('rgb',0,True,1);assert s.view('rgb',1)['value'] is None
 s.health_event('rgb',0,True,2);assert s.view('rgb',2)['receipt_ns']==2

def test_bounded_inactive_can_be_stale_without_silent_reuse():
 s=ProbeCache('bounded_no_probe');s.plan(0,0,True);ready(s,'thermal',0,1);ready(s,'rgb',0,3);d=s.plan(1,SECOND+10);assert d['action']=='ABSTAIN' and all(v['status']=='STALE' for v in d['views'].values())

def test_guard_forces_requests_without_using_future_truth():
 s=ProbeCache('fixed_rgb');d=s.plan(0,0,True);assert d['requests']==['rgb','thermal'] and d['action']=='RGB'

def test_invalid_clock_pair_and_duplicates():
 s=ProbeCache('full_read_cache');s.plan(0,0);s.health_event('rgb',0,False,1)
 with pytest.raises(ValueError):s.health_event('rgb',0,False,2)
 with pytest.raises(ValueError):s.plan(1,0)
 with pytest.raises(ValueError):s.plan(0,3)
