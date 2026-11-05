from pathlib import Path
import sys
from types import SimpleNamespace
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import droid_bounded_worker_v3 as b


def policy():
 return {'bounded_capacity':{'mode':'half_memory_persistent_state_cap','preflight_gpu_total_bytes':50752389120,
 'worst_state_bytes_per_slot':4496290,'slots':4096,'reserved_slots':17},'settings':{**b.native.SETTINGS,'buffer':4096}}


def test_half_memory_bound_and_unchanged_motion_settings():
 p=policy();assert b.budget_slots(50752389120,4496290)==4096
 assert b.settings_for(p,{'case':'night-rgb'})=={**b.native.SETTINGS,'buffer':4096}


def test_full_filler_reserve_at4096():
 g=b.Guard(4096);g.before_track(4078);g.before_terminate(4079)
 with pytest.raises(b.NativeCapacityExceeded):g.before_track(4079)
 with pytest.raises(b.NativeCapacityExceeded):g.before_terminate(4080)


@pytest.mark.parametrize('field,value',[('filter_thresh',2.),('warmup',9),('buffer',8192),('stereo',False)])
def test_no_other_setting_change(field,value):
 p=policy();p['settings'][field]=value
 with pytest.raises(ValueError):b.settings_for(p,{'case':'night-rgb'})


def test_original_named_super_and_native_exception_restore():
 module=SimpleNamespace()
 class Base:
  def __init__(self):
   super(module.Droid,self).__init__()
   self.video=SimpleNamespace(counter=SimpleNamespace(value=4079),poses=SimpleNamespace(shape=(4096,7)))
   self.mutations=0
  def track(self):self.mutations+=1
 module.Droid=Base;old=b.native.SETTINGS
 with pytest.raises(b.NativeCapacityExceeded):
  with b.bounded_context(module,policy(),{'case':'night-rgb'}):obj=module.Droid();obj.track()
 assert obj.mutations==0 and module.Droid is Base and b.native.SETTINGS is old
