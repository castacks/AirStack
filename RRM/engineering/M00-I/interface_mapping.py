"""Pure, typed mapping of supplied production observations to H diagnostics.

No ROS imports or endpoints are created. Source/type checks restore contract
compatibility; they do not authenticate a publisher, calibrate clocks or prove
OmniGraph construction/guard readiness from a ROS graph snapshot.
"""
from pathlib import Path
import sys
H=Path(__file__).resolve().parents[1]/'M00-H'
if str(H) not in sys.path:sys.path.append(str(H))
from observation_adapters import MavrosStateAdapter,VehicleGraphAdapter
from reliable_recorder import JournalError

ROOT='/robot_1/interface'
STATE=ROOT+'/mavros/state'
EXTENDED=ROOT+'/mavros/extended_state'
GET=ROOT+'/mavros/setpoint_raw/get_parameters'
SET=ROOT+'/mavros/setpoint_raw/set_parameters'
REQUIRED={STATE:'mavros_msgs/msg/State',EXTENDED:'mavros_msgs/msg/ExtendedState',
          GET:'rcl_interfaces/srv/GetParameters',SET:'rcl_interfaces/srv/SetParameters'}


class _AnnotatedEmitter:
    def __init__(self,emitter,extra):self.emitter,self.extra=emitter,extra
    def emit(self,kind,stamp,data):return self.emitter.emit(kind,stamp,{**data,**self.extra})


class StateMapping:
    def __init__(self,emitter):
        self.annotation=_AnnotatedEmitter(emitter,{})
        self.delegate=MavrosStateAdapter(self.annotation)
    def observe(self,topic,type_name,fields,stamp,deadline=None):
        if topic!=STATE or type_name!=REQUIRED[STATE]:raise JournalError('State topic/type mismatch')
        if not isinstance(fields,dict) or type(fields.get('connected')) is not bool or type(fields.get('armed')) is not bool:
            raise JournalError('State fields must preserve actual Boolean connected/armed values')
        # Header ROS time is deliberately not substituted for callback steady time.
        self.annotation.extra={'armed':fields['armed'],'source_topic':topic,'source_type':type_name}
        return self.delegate.observe(fields['connected'],stamp,deadline)


def extended_state(topic,type_name,fields):
    if topic!=EXTENDED or type_name!=REQUIRED[EXTENDED] or not isinstance(fields,dict):raise JournalError('ExtendedState topic/type mismatch')
    value=fields.get('landed_state')
    if type(value) is not int or value not in {0,1,2,3,4}:raise JournalError('Invalid MAVROS landed-state value')
    return {'landed_state':value,'on_ground':value==1,'in_air':value==2,
            'unknown_or_transition':value in {0,3,4},'observation_only':True}


class RosInterfacePresence:
    def __init__(self,emitter):
        annotated=_AnnotatedEmitter(emitter,{'readiness_scope':'ROS_INTERFACE_PRESENCE_ONLY_NOT_SIMULATOR_VEHICLE_GRAPH'})
        self.delegate=VehicleGraphAdapter(annotated,set(REQUIRED))
    def observe(self,snapshot,stamp):
        if not isinstance(snapshot,dict):raise JournalError('Typed graph snapshot required')
        matched={name for name,expected in REQUIRED.items() if snapshot.get(name) in (expected,[expected],(expected,))}
        # Keep H schema2 kinds; scope annotation is recorded before checksum/write.
        return self.delegate.observe(matched,stamp)
