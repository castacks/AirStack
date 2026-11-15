#!/usr/bin/env python3
"""Passive exact-stamp RGB / simulated depth / calibration fixture observation.
No task actions, services, actuation publishers or scene-manifest inputs.
"""
import argparse,json,time,hashlib,inspect
from pathlib import Path


def main():
    import cv2,numpy as np,rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from cv_bridge import CvBridge
    from rrm.uav_marker import detect_red_surface
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',required=True);ap.add_argument('--duration',type=float,default=20);ap.add_argument('--frames',type=int,default=20)
    args=ap.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    (out/'provenance.json').write_text(json.dumps(dict(
        capture_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        detector_sha256=hashlib.sha256(Path(inspect.getfile(detect_red_surface)).read_bytes()).hexdigest(),
        cv2_version=cv2.__version__,numpy_version=np.__version__),indent=2))
    rclpy.init();n=Node('uav_passive_marker_capture');bridge=CvBridge();cache={};records=[];seen=set();counts={}
    log=(out/'observations.jsonl').open('w',buffering=1)
    def cb(kind,msg):
        counts[kind]=counts.get(kind,0)+1
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        entry=cache.setdefault(stamp,{});entry[kind]=msg
        if stamp not in seen and all(k in entry for k in ('rgb','depth','info')):
            seen.add(stamp);rgb,depth,info=(entry[k] for k in ('rgb','depth','info'))
            v=dict(wall_unix=time.time(),stamp_sec=stamp[0],stamp_nanosec=stamp[1],frame=rgb.header.frame_id,exact_stamp_pair=True)
            try:
                if len({m.header.frame_id for m in (rgb,depth,info)})!=1:raise ValueError('frame mismatch')
                if depth.encoding!='32FC1':raise ValueError('requires explicit 32FC1 metres depth')
                if (info.width,info.height)!=(rgb.width,rgb.height):raise ValueError('calibration/image size mismatch')
                color=bridge.imgmsg_to_cv2(rgb,desired_encoding='rgb8');distance=bridge.imgmsg_to_cv2(depth,desired_encoding='passthrough')
                v['observation']=detect_red_surface(color,distance,[float(x) for x in info.k])
                if len(records) in (0,10,19):
                    prefix=f'frame-{len(records):03d}'
                    cv2.imwrite(str(out/(prefix+'.png')),cv2.cvtColor(color,cv2.COLOR_RGB2BGR))
                    np.savez_compressed(out/(prefix+'.npz'),depth_m=distance,intrinsic=np.array(info.k))
                    v['raw_prefix']=prefix
            except Exception as error:v['observation']=dict(status='INVALID',error=repr(error))
            records.append(v);log.write(json.dumps(v)+'\n')
        while len(cache)>15:cache.pop(next(iter(cache)))
    subs=[n.create_subscription(typ,'/robot_1/sensors/front_stereo/left/'+topic,lambda m,k=kind:cb(k,m),qos_profile_sensor_data)
          for kind,typ,topic in [('rgb',Image,'image_rect'),('depth',Image,'depth_ground_truth'),('info',CameraInfo,'camera_info')]]
    end=time.monotonic()+args.duration
    while time.monotonic()<end and len(records)<args.frames:rclpy.spin_once(n,timeout_sec=.05)
    statuses={}
    for v in records:
        status=v['observation']['status'];statuses[status]=statuses.get(status,0)+1
    summary=dict(scope='passive restricted red fixture; simulated oracle depth surface in pinhole optical convention, no world binding',
                 received=counts,exact_stamp_observations=len(records),statuses=statuses)
    (out/'summary.json').write_text(json.dumps(summary,indent=2));log.close();n.destroy_node();rclpy.shutdown()
    if not records:raise SystemExit(1)

if __name__=='__main__':main()
