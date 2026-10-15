#!/usr/bin/env python3
"""Bounded read-only sensor geometry and separate simulator-depth assessment.

No action client, control publisher, C02 facts or waypoint export. The prototype
receives stereo inputs and an explicit operator region, never simulator depth.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rrm.stereo_localization import (CameraMapPose, PixelRegion, SensorBinding,
    StereoCalibration, localize_region)
from rrm.stereo_depth_assessment import assess_axial_depth

REGIONS = [('floor_patch',(200,220,80,60)),('central_view',(200,130,80,60)),
           ('low_texture',(120,60,100,60))]


def stamp(message):
    value=message.header.stamp
    return value.sec*10**9+value.nanosec


def decode(message):
    import numpy as np
    formats={'32FC1':('f4',1),'rgb8':('u1',3),'bgr8':('u1',3),
             'rgba8':('u1',4),'bgra8':('u1',4),'mono8':('u1',1)}
    if message.encoding not in formats:
        raise ValueError('unsupported image encoding '+message.encoding)
    name,channels=formats[message.encoding]
    dtype=np.dtype(('>' if message.is_bigendian else '<')+name)
    if (not 0 < message.width*message.height <= 4_000_000
            or message.step % dtype.itemsize
            or message.step < message.width*channels*dtype.itemsize
            or len(message.data) != message.height*message.step):
        raise ValueError('invalid image buffer dimensions')
    values=np.frombuffer(message.data,dtype=dtype).reshape(message.height,message.step//dtype.itemsize)
    values=values[:,:message.width*channels]
    shape=(message.height,message.width) if channels==1 else (message.height,message.width,channels)
    return np.ascontiguousarray(values.reshape(shape))


def calibration_for(messages):
    import numpy as np
    left,right,disparity=messages['left_info'],messages['right_info'],messages['disparity']
    if (left.header.frame_id!='camera_left' or right.header.frame_id!='camera_right'
            or disparity.header.frame_id!='camera_left' or disparity.image.header.frame_id!='camera_left'
            or disparity.image.encoding!='32FC1'
            or not np.allclose(left.r,np.eye(3).flatten(),atol=1e-6)
            or not np.allclose(right.r,np.eye(3).flatten(),atol=1e-6)
            or not np.isfinite([*left.p,*right.p]).all()
            or left.p[0] <= 0 or right.p[0] <= 0
            or left.p[3] != 0 or left.p[7] != 0):
        raise ValueError('invalid optical calibration/frame')
    dimensions=(left.width,left.height)
    if (dimensions!=(right.width,right.height) or dimensions!=(disparity.image.width,disparity.image.height)
            or any(dimensions!=(messages[side].width,messages[side].height) for side in ['left','right'])
            or not np.allclose([left.p[i] for i in range(12) if i!=3],
                               [right.p[i] for i in range(12) if i!=3],rtol=0,atol=1e-6)
            or not np.allclose(left.p[8:12],[0.,0.,1.,0.],rtol=0,atol=1e-6)
            or left.p[1]!=0 or left.p[4]!=0 or left.p[5]<=0 or right.p[7]!=0
            or not np.isclose(left.p[0],disparity.f,atol=1e-3)
            or not np.isclose(-right.p[3]/right.p[0],disparity.t,atol=1e-6)):
        raise ValueError('stereo calibration mismatch')
    roi=disparity.valid_window
    return StereoCalibration(*dimensions,float(left.p[0]),float(left.p[5]),float(left.p[2]),
        float(left.p[6]),float(disparity.t),float(disparity.min_disparity),float(disparity.max_disparity),
        float(disparity.delta_d),(roi.x_offset,roi.y_offset,roi.width,roi.height))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--episode-id',required=True,help='Verified simulator runtime identity for this diagnostic')
    parser.add_argument('--duration-s',type=float,default=15.)
    parser.add_argument('--max-frames',type=int,default=10)
    args=parser.parse_args()
    if not args.episode_id.strip() or not 0 < args.duration_s <= 30 or not 1 <= args.max_frames <= 20:
        parser.error('invalid bounded capture parameters')
    import cv2
    import numpy as np
    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import Image,CameraInfo
    from stereo_msgs.msg import DisparityImage
    from mavros_msgs.msg import State,ExtendedState
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from tf2_ros import Buffer,TransformListener

    out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    rclpy.init()
    node=rclpy.create_node('rrm_readonly_stereo_depth_assessment')
    tf=Buffer(cache_time=Duration(seconds=20))
    listener=TransformListener(tf,node)
    qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.BEST_EFFORT)
    kinds=['left','right','left_info','right_info','disparity','truth']
    caches={name:{} for name in kinds}
    counts={name:0 for name in kinds}
    records=[]
    states=[]
    pending=[]
    started=time.monotonic()
    next_attempt=started
    attempt_count=0
    clock_last=None
    reset=False
    session=uuid.uuid4().hex

    def receive(kind,message):
        nonlocal next_attempt,attempt_count
        now=time.monotonic()
        key=stamp(message)
        counts[kind]+=1
        caches[kind][key]=(message,now)
        while len(caches[kind])>32: del caches[kind][next(iter(caches[kind]))]
        if kind=='disparity' and not reset and now>=next_attempt and attempt_count<args.max_frames:
            pending.append(dict(sample_index=attempt_count,stamp_ns=key,requested=now,deadline=now+.5))
            attempt_count+=1
            next_attempt=now+1.

    def clock(message):
        nonlocal clock_last,reset
        value=message.clock.sec*10**9+message.clock.nanosec
        if clock_last is not None and value < clock_last-1_000_000_000: reset=True
        clock_last=value

    def state(kind,message):
        value=dict(kind=kind,receipt_monotonic_s=time.monotonic())
        if kind=='vehicle':value.update(connected=message.connected,armed=message.armed)
        elif kind=='landed':value['landed_state']=message.landed_state
        else:
            velocity=message.twist.twist.linear
            value.update(stamp_ns=stamp(message),linear_speed_m_s=float(np.linalg.norm([velocity.x,velocity.y,velocity.z])))
        states.append(value)

    prefix='/robot_1/sensors/front_stereo'
    specifications=[('left',Image,prefix+'/left/image_rect'),('right',Image,prefix+'/right/image_rect'),
        ('left_info',CameraInfo,prefix+'/left/camera_info'),('right_info',CameraInfo,prefix+'/right/camera_info'),
        ('disparity',DisparityImage,'/robot_1/perception/stereo_image_proc/disparity'),
        ('truth',Image,prefix+'/left/depth_ground_truth')]
    subscriptions=[node.create_subscription(cls,topic,lambda msg,kind=kind:receive(kind,msg),qos)
                   for kind,cls,topic in specifications]
    subscriptions.extend([node.create_subscription(State,'/robot_1/interface/mavros/state',lambda msg:state('vehicle',msg),qos),
        node.create_subscription(ExtendedState,'/robot_1/interface/mavros/extended_state',lambda msg:state('landed',msg),qos),
        node.create_subscription(Odometry,'/robot_1/odometry_conversion/odometry',lambda msg:state('odometry',msg),qos),
        node.create_subscription(Clock,'/clock',clock,qos)])

    def finalize(attempt,messages,pose):
        record=dict(sample_index=attempt['sample_index'],source_stamp_ns=attempt['stamp_ns'],
                    episode_id=args.episode_id,execution_dispatch=False,
                    receipt_monotonic_s=time.monotonic(),status='INCOMPLETE')
        missing=[kind for kind in kinds if kind not in messages]
        if missing or reset:
            return {**record,'missing_channels':missing,'clock_reset_detected':reset}
        sample=out/f"sample-{attempt['sample_index']:02}"
        sample.mkdir()
        try:
            calibration=calibration_for(messages)
            if (messages['truth'].header.frame_id!='camera_left' or messages['truth'].encoding!='32FC1'
                    or messages['left'].header.frame_id!='camera_left' or messages['right'].header.frame_id!='camera_right'
                    or stamp(messages['disparity'].image)!=attempt['stamp_ns']):
                raise ValueError('sample optical-frame/stamp mismatch')
            arrays={kind:decode(messages[kind].image if kind=='disparity' else messages[kind])
                    for kind in ['left','right','disparity','truth']}
            for kind,array in arrays.items():np.save(sample/(kind+'.npy'),array)
            rgb=arrays['left']
            encoding=messages['left'].encoding
            display=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR) if encoding=='rgb8' else rgb
            cv2.imwrite(str(sample/'left.png'),display)
            image_sha=hashlib.sha256(rgb.tobytes()).hexdigest()
            binding=SensorBinding(args.episode_id,image_sha,attempt['stamp_ns'],stamp(messages['right']),
                                  stamp(messages['left_info']),stamp(messages['disparity']),
                                  caches['left'][attempt['stamp_ns']][1])
            # Teacher arrays do not enter this function or its input records.
            previews={label:localize_region(arrays['disparity'],region=PixelRegion(label,*region,image_sha),
                calibration=calibration,binding=binding,pose=pose,current_episode_id=args.episode_id,
                now_monotonic_s=time.monotonic()) if pose is not None else
                dict(status='HOLD',reason='missing_exact_stamp_tf',execution_dispatch=False)
                for label,region in REGIONS}
            assessment={'whole_valid_window':assess_axial_depth(arrays['disparity'],arrays['truth'],calibration)}
            assessment.update({label:assess_axial_depth(arrays['disparity'],arrays['truth'],calibration,region=region)
                               for label,region in REGIONS})
            record.update(status='HEADER_PAIRED_ASSESSMENT',native_acquisition_binding_verified=False,
                image_sha256=image_sha,calibration=asdict(calibration),sensor_binding=asdict(binding),
                camera_map_pose=asdict(pose) if pose is not None else None,
                source_frames={kind:messages[kind].header.frame_id for kind in kinds},
                image_encodings={kind:messages[kind].encoding for kind in ['left','right','truth']},
                right_calibration=dict(stamp_ns=stamp(messages['right_info']),p=list(messages['right_info'].p)),
                teacher=dict(authority='EVALUATION_ONLY',depth_semantics='OPTICAL_AXIS_METERS',
                             source_stamp_ns=stamp(messages['truth']),source_topic=prefix+'/left/depth_ground_truth'),
                previews=previews,assessment=assessment)
        except (ValueError,TypeError,IndexError) as error:
            record['error']=str(error)
        (sample/'record.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
        return record

    error=None
    try:
        while time.monotonic()-started < args.duration_s:
            rclpy.spin_once(node,timeout_sec=.02)
            for attempt in list(pending):
                messages={kind:caches[kind][attempt['stamp_ns']][0] for kind in kinds
                          if attempt['stamp_ns'] in caches[kind]}
                pose=None
                try:
                    value=tf.lookup_transform('map','camera_left',Time(nanoseconds=attempt['stamp_ns']))
                    t,q=value.transform.translation,value.transform.rotation
                    pose=CameraMapPose(value.header.frame_id,value.child_frame_id,stamp(value),
                                       (t.x,t.y,t.z),(q.x,q.y,q.z,q.w))
                except Exception:pass
                if reset or (len(messages)==len(kinds) and pose is not None) or time.monotonic()>=attempt['deadline']:
                    records.append(finalize(attempt,messages,pose))
                    pending.remove(attempt)
        for attempt in pending:
            records.append(dict(sample_index=attempt['sample_index'],source_stamp_ns=attempt['stamp_ns'],
                                status='INCOMPLETE',reason='capture_ended',execution_dispatch=False))
    except Exception as exc:
        error=str(exc)
        raise
    finally:
        report=dict(schema_version='rrm-stereo-depth-capture/v1',session_id=session,episode_id=args.episode_id,
            written_at_utc=datetime.now(timezone.utc).isoformat(),duration_s=time.monotonic()-started,
            execution_dispatch=False,clock_reset_detected=reset,error=error,counts=counts,
            sample_attempts=attempt_count,records=records,vehicle_observations=states,
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            localization_source_sha256=hashlib.sha256((Path(__file__).parents[1]/'rrm/stereo_localization.py').read_bytes()).hexdigest())
        (out/'capture.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(dict(output=str(out),attempts=attempt_count,paired=sum(
            record['status']=='HEADER_PAIRED_ASSESSMENT' for record in records),error=error)))


if __name__=='__main__':main()
