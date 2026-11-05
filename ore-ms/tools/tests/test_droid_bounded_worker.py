from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import droid_bounded_worker as b


def policy(slots=2048):
    return {'bounded_capacity':{'mode':'quarter_memory_persistent_state_cap',
       'preflight_gpu_total_bytes':50752389120,'worst_state_bytes_per_slot':4496290,
       'slots':slots,'reserved_slots':17},'settings':{**b.native.SETTINGS,'buffer':slots}}


def test_budget_is_not_an_observed_keyframe_extrapolation():
    assert b.budget_slots(50752389120,4496290)==2048


@pytest.mark.parametrize('a,c',[(True,1),(1.,1),(1,False),(-1,1),(1,1)])
def test_bad_budgets(a,c):
    with pytest.raises(ValueError):b.budget_slots(a,c)


def test_guard_boundary_and_no_increment_beyond_native():
    g=b.Guard(32);g.before_track(14);g.before_terminate(15)
    with pytest.raises(b.NativeCapacityExceeded):g.before_track(15)
    assert g.events[0]['attempt_index']==1 and g.limit==15
    assert g.events[0]['native_keyframe_count']==15


def test_termination_preserves_all16filler_and_next_slot():
    g=b.Guard(32);g.before_terminate(15)
    with pytest.raises(b.NativeCapacityExceeded):g.before_terminate(16)


@pytest.mark.parametrize('count',[True,-1,33,1.5])
def test_bad_counters(count):
    with pytest.raises(ValueError):b.Guard(32).before_track(count)


@pytest.mark.parametrize('key,value',[('filter_thresh',3.),('warmup',7),('stereo',False),('buffer',1024)])
def test_settings_changes_refused(key,value):
    p=policy();p['settings'][key]=value
    with pytest.raises(ValueError):b.settings_for(p,{'case':'day-rgb'})


def test_restores_on_native_exception_and_guards_before_mutation():
    class Base:
        def __init__(self):
            self.video=SimpleNamespace(counter=SimpleNamespace(value=2031),poses=SimpleNamespace(shape=(2048,7)))
            self.called=0
        def track(self,*a,**k):self.called+=1
    module=SimpleNamespace(Droid=Base);old=b.native.SETTINGS
    with pytest.raises(b.NativeCapacityExceeded):
        with b.bounded_context(module,policy(),{'case':'day-rgb'}):
            obj=module.Droid();obj.track(1)
    assert obj.called==0 and module.Droid is Base and b.native.SETTINGS is old


def test_below_capacity_forwards_exact_arguments():
    class Base:
        def __init__(self):self.video=SimpleNamespace(counter=SimpleNamespace(value=1),poses=SimpleNamespace(shape=(2048,7)))
        def track(self,*a,**k):self.arguments=(a,k);return 'original'
    g=b.Guard(2048);obj=b.guarded_class(Base,g)();x=object()
    assert obj.track(12,x,intrinsics=x)=='original'
    assert obj.arguments==((12,x),{'intrinsics':x}) and g.events==[]


def test_control_cannot_be_used_for_full_records():
    p=policy(32);p['bounded_capacity']['mode']=b.CONTROL_MODE
    assert b.settings_for(p,{'case':'day-rgb','source_frames':101})['buffer']==32
    with pytest.raises(ValueError):b.settings_for(p,{'case':'day-rgb','source_frames':5810})


def test_actual_allocation_must_match_policy():
    class Base:
        def __init__(self):self.video=SimpleNamespace(poses=SimpleNamespace(shape=(1024,7)))
    with pytest.raises(ValueError):b.guarded_class(Base,b.Guard(2048))()


@pytest.mark.parametrize('fails',[False,True])
def test_upstream_named_super_constructor_and_restore(fails):
    module=SimpleNamespace()
    class Base:
        def __init__(self):
            super(module.Droid,self).__init__()
            if fails:raise RuntimeError('constructor failure')
            self.video=SimpleNamespace(counter=SimpleNamespace(value=1),poses=SimpleNamespace(shape=(2048,7)))
    module.Droid=Base;old=b.native.SETTINGS
    if fails:
        with pytest.raises(RuntimeError):
            with b.bounded_context(module,policy(),{'case':'day-rgb'}):module.Droid()
    else:
        with b.bounded_context(module,policy(),{'case':'day-rgb'}):
            wrapped=module.Droid;module.Droid();assert module.Droid is wrapped
    assert module.Droid is Base and b.native.SETTINGS is old
