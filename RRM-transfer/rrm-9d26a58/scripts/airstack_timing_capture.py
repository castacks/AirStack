#!/usr/bin/env python3
"""Read-only raw MAVLink, odometry and timing-status evidence capture."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import struct
import time

# Common-dialect wire sizes, including trailing extensions. Timestamps only except32.
SPECS = {2: ('SYSTEM_TIME', 12, 12), 30: ('ATTITUDE', 28, 28),
         31: ('ATTITUDE_QUATERNION', 32, 48), 32: ('LOCAL_POSITION_NED', 28, 28),
         105: ('HIGHRES_IMU', 62, 63), 111: ('TIMESYNC', 16, 18),
         331: ('ODOMETRY', 230, 233)}


def integer(v, low, high):
    if type(v) is not int or not low <= v <= high:
        raise ValueError('invalid MAVLink integer field')
    return v


def decode(message):
    """Trust bridge framing flag; CRC/signature are not independently recomputed."""
    integer(message['sysid'],0,255);integer(message['compid'],0,255)
    msgid=integer(message['msgid'],0,0xffffff)
    magic=integer(message['magic'],253,254)
    if integer(message['framing_status'],1,3) != 1:
        raise ValueError('MAVROS reports invalid framing')
    if magic==254 and msgid>255:raise ValueError('MAVLink1 message ID exceeds255')
    length=integer(message['len'],1,255)
    words=message['payload64']
    if not isinstance(words,list) or not (length+7)//8 <= len(words) <= 33:
        raise ValueError('short or oversized payload64')
    payload=b''.join(struct.pack('<Q',integer(w,0,2**64-1)) for w in words)[:length]
    if msgid not in SPECS:
        return {'supported':False,'message_id':msgid}
    name,minimum,maximum=SPECS[msgid]
    if length>maximum or (magic==254 and length!=minimum):
        raise ValueError('invalid known message length')
    # MAVLink2 omits trailing zero bytes, including core fields.
    payload=payload.ljust(maximum,b'\0')
    result={'supported':True,'name':name,'message_id':msgid}
    if msgid in (30,31,32,105,331):
        result['source_timestamp_role'] = ('sample' if msgid in (105,331) else 'publication')
        result['role_basis'] = 'inspected PX4 source94cb201; binary/source equivalence not proven'
    if msgid in (30,31,32):
        result['time_boot_ms']=struct.unpack_from('<I',payload)[0]
    elif msgid in (105,331):
        result['time_usec']=struct.unpack_from('<Q',payload)[0]
    elif msgid==2:
        result['time_unix_usec'],result['time_boot_ms']=struct.unpack_from('<QI',payload)
    elif msgid==111:
        result['tc1_ns'],result['ts1_ns']=struct.unpack_from('<qq',payload)
    if msgid==32:
        values=struct.unpack_from('<6f',payload,4)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('nonfinite LOCAL_POSITION_NED')
        result['position_ned_m']=list(values[:3]);result['velocity_ned_m_s']=list(values[3:])
    return result


def publisher_gid(info):
    """Jazzy supplies a dict; also accept MessageInfo objects on other releases."""
    value = info.get('publisher_gid') if isinstance(info, dict) else getattr(info,'publisher_gid',None)
    return bytes(value).hex() if value else None


class TimingCapture:
    def __init__(self,stream,start,channels,max_events=50_000,max_bytes=64*1024*1024):
        self.stream,self.start,self.channels=stream,start,list(channels)
        self.max_events,self.max_bytes=max_events,max_bytes
        self.counts=Counter();self.msgids=Counter();self.errors=0
        self.total=self.bytes=0;self.limit=None

    def record(self,channel,message,receipt,ros_now_ns,publisher_gid,transport=None):
        if self.limit:return
        if self.total>=self.max_events:self.limit='event_limit';return
        event={'channel':channel,'receipt_monotonic_s':receipt,'elapsed_s':receipt-self.start,
               'ros_now_ns':ros_now_ns,'publisher_gid_hex':publisher_gid,'message':message,
               'rmw_transport_info':transport}
        if channel=='raw_mavlink':
            try:event['decoded']=decode(message)
            except (ValueError,KeyError,TypeError,struct.error) as exc:
                event['decode_error']=str(exc)
        encoded=json.dumps(event,allow_nan=False)+'\n'
        size=len(encoded.encode())
        if self.bytes+size>self.max_bytes:self.limit='byte_limit';return
        self.stream.write(encoded);self.stream.flush()
        self.bytes+=size;self.total+=1;self.counts[channel]+=1
        if channel=='raw_mavlink':
            self.msgids[str(message['msgid'])]+=1
            self.errors+=int('decode_error' in event)

    def summary(self):
        return {'events':self.total,'bytes':self.bytes,'channel_counts':dict(self.counts),
                'missing_channels':[c for c in self.channels if not self.counts[c]],
                'raw_message_id_counts':dict(self.msgids),'decode_errors':self.errors,
                'limit':self.limit,'execution_dispatch':False,
                'limitations':'Packet clocks may have distinct epochs. Publication/header/receipt timestamps do not prove sensor or EKF acquisition timing. No offset inferred from absent timesync status. Bridge framing trusted; CRC/signature not recomputed. Synchronous I/O can stall.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=20)
    parser.add_argument('--robot',default='robot_1');parser.add_argument('--uas',default='uas2')
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=300:parser.error('duration must be (0,300]')
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from mavros_msgs.msg import Mavlink,TimesyncStatus,State,ExtendedState
    from sensor_msgs.msg import TimeReference
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    prefix=f'/{args.robot}/interface/mavros'
    specs={'raw_mavlink':(f'/{args.uas}/mavlink_source',Mavlink),
           'raw_odom':(prefix+'/local_position/odom',Odometry),
           'converted_odom':(f'/{args.robot}/odometry_conversion/odometry',Odometry),
           'timesync_status':(prefix+'/timesync_status',TimesyncStatus),
           'time_reference':(prefix+'/time_reference',TimeReference),
           'state':(prefix+'/state',State),'landed':(prefix+'/extended_state',ExtendedState),
           'armed':(f'/{args.robot}/interface/is_armed',Bool),
           'authority':(f'/{args.robot}/interface/has_control',Bool)}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:
        rclpy.init();node=rclpy.create_node('rrm_timing_capture',enable_rosout=False,
            start_parameter_services=False,parameter_overrides=[Parameter('use_sim_time',value=True),
                Parameter('start_type_description_service',value=False)])
        capture=TimingCapture(stream,time.monotonic(),specs)
        stop='duration';qos=QoSProfile(depth=200,reliability=ReliabilityPolicy.BEST_EFFORT)
        try:
            def callback_for(channel):
                def callback(message,info):
                    capture.record(channel,message_to_ordereddict(message),time.monotonic(),
                        node.get_clock().now().nanoseconds,publisher_gid(info),
                        {key:(info.get(key) if isinstance(info,dict) else getattr(info,key,None))
                         for key in ('source_timestamp','received_timestamp',
                                     'publication_sequence_number','reception_sequence_number')})
                return callback
            for channel,(topic,kind) in specs.items():
                node.create_subscription(kind,topic,callback_for(channel),qos)
            while rclpy.ok() and time.monotonic()-capture.start<args.duration and not capture.limit:
                rclpy.spin_once(node,timeout_sec=.05)
        except KeyboardInterrupt:stop='interrupted'
        except Exception:stop='error';raise
        finally:
            summary=capture.summary();summary.update({'schema':'rrm-timing-capture/v1',
                'captured_at_utc':datetime.now(timezone.utc).isoformat(),
                'elapsed_wall_s':time.monotonic()-capture.start,'stop_reason':capture.limit or stop,
                'recorder_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'topics':{c:v[0] for c,v in specs.items()},
                'publisher_endpoints':{c:[{'node':i.node_name,'namespace':i.node_namespace,
                    'gid_hex':bytes(i.endpoint_gid).hex()} for i in node.get_publishers_info_by_topic(v[0])]
                    for c,v in specs.items()},
                'publisher_identity_note':'Per-event GID is null when unavailable in installed rclpy; graph endpoints are candidates, not per-event attribution. RMW timestamps use middleware clock, not established acquisition time.',
                'capture_publishers':node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace()),
                'capture_services':node.get_service_names_and_types_by_node(node.get_name(),node.get_namespace())})
            args.output.with_suffix(args.output.suffix+'.summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            node.destroy_node();rclpy.shutdown()
    print(json.dumps(summary,sort_keys=True))

if __name__=='__main__':main()
