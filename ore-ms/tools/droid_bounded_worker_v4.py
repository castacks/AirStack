"""Allocation-only DROID wrapper with explicit conservative capacity aborts.

No source dropping, threshold changes, resets, fallback or partial offline salvage.
Original adapter owns pose conversion and null failure output. Guard runs before
native track; a completed finalization must still fit the fixed filler reserve.
"""
import argparse
from contextlib import contextmanager
from pathlib import Path
import sys

import droid_ms2_development as native

RESERVE=17
CONTROL_MODE='declared_101_day_rgb_32_slot_overflow_control'


class NativeCapacityExceeded(RuntimeError):pass
class NativeFeatureIndexCapacityExceeded(RuntimeError):pass


def compact_feature_buffer(video,guard):
    """Remove only unused post-tracking feature capacity; preserve filler reserve.

    Pinned low-memory backend pools every allocated feature row, then uses 32-bit
    accessors. No factor may index padding; all actual and reserved rows retain
    exactly the same half-precision bits. Other state arrays/counter are unchanged.
    """
    import torch
    count=video.counter.value;original=video.fmaps
    guard.count(count)
    if original.ndim!=5 or original.shape[0]!=guard.slots or original.shape[1:3]!=(2,128):
        raise ValueError('unexpected native stereo feature storage')
    desired=count+RESERVE
    if desired>guard.slots:raise NativeCapacityExceeded('missing filler feature reserve')
    elements=desired*original[0].numel()
    if elements>=2**31-1:
        raise NativeFeatureIndexCapacityExceeded('active plus reserved features still exceed native32-bit accessor limit')
    compact=original[:desired].clone()
    if not torch.equal(compact.view(torch.int16),original[:desired].view(torch.int16)):
        raise ValueError('feature compaction changed preserved bits')
    if video.counter.value!=count:raise ValueError('counter changed during post-tracking compaction')
    guard.feature_compaction={'original_slots':guard.slots,'active_keyframes':count,'retained_feature_slots':desired,
      'removed_unused_slots':guard.slots-desired,'original_feature_elements':original.numel(),
      'retained_feature_elements':elements,'native32_bit_element_limit':2**31-1,
      'all_active_and_reserved_feature_bits_preserved':True,'counter_unchanged':True,
      'dtype':str(original.dtype),'shape_per_slot':list(original.shape[1:])}
    video.fmaps=compact



def budget_slots(total_bytes,worst_state_bytes):
    if any(type(x) is not int or x<=0 for x in [total_bytes,worst_state_bytes]):
        raise ValueError('positive exact byte counts required')
    slots=(total_bytes//2)//worst_state_bytes
    if slots<32:raise ValueError('half-memory state budget too small')
    return 1<<(slots.bit_length()-1)


def settings_for(plan,case):
    c=plan['bounded_capacity']
    expected=budget_slots(c['preflight_gpu_total_bytes'],c['worst_state_bytes_per_slot'])
    if c.get('mode')==CONTROL_MODE:
        if case['case']!='day-rgb' or case['source_frames']!=101 or c['slots']!=32:
            raise ValueError('overflow control is fixed to101dayRGB/32slots')
        expected=32
    elif c.get('mode')!='half_memory_persistent_state_cap':
        raise ValueError('unknown capacity policy')
    if c['slots']!=expected or c['reserved_slots']!=RESERVE:
        raise ValueError('capacity/reserve differs from fixed policy')
    settings={**native.SETTINGS,'buffer':expected}
    if plan['settings']!=settings:raise ValueError('only validated buffer allocation may change')
    return settings


class Guard:
    def __init__(self,slots):
        if type(slots) is not int or slots<32:raise ValueError('at least32exactslots required')
        self.slots=slots;self.limit=slots-RESERVE;self.attempts=0
        self.phase='construction';self.events=[];self.last_count=None;self.feature_compaction=None
    def count(self,value):
        if type(value) is not int or value<0 or value>self.slots:raise ValueError('invalid native counter')
        self.last_count=value
    def before_track(self,value):
        self.count(value);self.phase='tracking';index=self.attempts;self.attempts+=1
        if value>=self.limit:
            event={'kind':'capacity_blocked_before_native_track','attempt_index':index,
                   'native_keyframe_count':value,'slots':self.slots,'reserved_slots':RESERVE}
            self.events.append(event)
            raise NativeCapacityExceeded('conservative state capacity reached before source index '+str(index))
    def before_terminate(self,value):
        self.count(value);self.phase='offline_termination'
        if value>self.limit:
            self.events.append({'kind':'capacity_blocked_before_termination','native_keyframe_count':value})
            raise NativeCapacityExceeded('insufficient reserved filler capacity')


def guarded_class(base,guard,module=None):
    class BoundedDroid(base):
        def __init__(self,*args,**kwargs):
            # Pinned upstream uses super(Droid,self), resolving its module global.
            # Keep that name bound to the original class during construction.
            previous=module.Droid if module is not None else None
            if module is not None:module.Droid=base
            try:super().__init__(*args,**kwargs)
            finally:
                if module is not None:module.Droid=previous
            guard.phase='ready'
            if int(self.video.poses.shape[0])!=guard.slots:raise ValueError('actual native allocation differs')
        def track(self,*args,**kwargs):
            guard.before_track(self.video.counter.value)
            result=super().track(*args,**kwargs)
            guard.count(self.video.counter.value)
            return result
        def terminate(self,*args,**kwargs):
            guard.before_terminate(self.video.counter.value)
            compact_feature_buffer(self.video,guard)
            return super().terminate(*args,**kwargs)
    return BoundedDroid


@contextmanager
def bounded_context(module,plan,case):
    settings=settings_for(plan,case);guard=Guard(settings['buffer'])
    old_class,old_settings=module.Droid,native.SETTINGS
    module.Droid=guarded_class(old_class,guard,module);native.SETTINGS=settings
    try:yield guard
    finally:module.Droid=old_class;native.SETTINGS=old_settings


def execute(protocol,case_id,output):
    p=native.read(protocol);case=next(c for c in p['runs'] if c['case']==case_id)
    # Verify actual runtime source used for the guard's append/filler assumptions.
    for name,digest in p['native_source_sha256'].items():
        if native.sha(Path('/opt/DROID-SLAM/droid_slam')/name)!=digest:raise ValueError('native guard assumption source changed '+name)
    sys.path.insert(0,'/opt/DROID-SLAM/droid_slam')
    import droid
    with bounded_context(droid,p,case) as guard:
        success=native.execute(protocol,case_id,output)
    out=Path(output);result=native.read(out/'estimate.json')
    blocked=next((e['attempt_index'] for e in guard.events if e['kind']=='capacity_blocked_before_native_track'),None)
    returned=result['native_track_calls_returned']
    statuses=[{'source_index':i,'source_id':s,'time_ns':result['time_ns'][i],
               'native_track_returned':i<returned,
               'capacity_blocked_before_native_track':i==blocked,
               'not_attempted_after_capacity_abort':blocked is not None and i>blocked,
               'final_offline_pose_available':result['poses'][i] is not None}
              for i,s in enumerate(result['source_ids'])]
    native.write(out/'capacity-status.json',{'policy':p['bounded_capacity'],'events':guard.events,
      'native_track_attempts':guard.attempts,'feature_buffer_compaction':guard.feature_compaction,'last_observed_keyframe_count':guard.last_count,
      'failure_phase':guard.phase if not success else None,'source_statuses':statuses,
      'original_adapter_sha256':native.sha(Path(native.__file__)),'wrapper_sha256':native.sha(Path(__file__)),
      'all_requested_source_slots_retained':len(statuses)==case['source_frames'],
      'persistent_state_bound_only':True,'online_output_latency_available':False,
      'physical_frame_or_clock_verified':False,'reference_scoring':False})
    return success


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--protocol',type=Path,required=True)
    ap.add_argument('--case',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    raise SystemExit(0 if execute(a.protocol,a.case,a.output) else 1)
