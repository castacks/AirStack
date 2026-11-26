import json,time,collections,os
import rclpy
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rosidl_runtime_py.utilities import get_message
rclpy.init()
n=rclpy.create_node('readiness_observer')
spec={
'/clock':'rosgraph_msgs/msg/Clock','/tf':'tf2_msgs/msg/TFMessage','/tf_static':'tf2_msgs/msg/TFMessage',
'/robot_1/interface/mavros/state':'mavros_msgs/msg/State','/diagnostics':'diagnostic_msgs/msg/DiagnosticArray',
'/robot_1/odometry_conversion/odometry':'nav_msgs/msg/Odometry',
'/robot_1/vdb_mapping/vdb_map_visualization':'visualization_msgs/msg/Marker',
}
for suffix,typ in [('left/image_rect','Image'),('left/camera_info','CameraInfo'),('right/depth_ground_truth','Image')]:
 spec['/robot_1/sensors/front_stereo/'+suffix]='sensor_msgs/msg/'+typ
data={t:{'count':0} for t in spec}
def summarize(t,m):
 d=data[t]; now=time.monotonic()
 if d['count']: d['max_gap_s']=max(d.get('max_gap_s',0),now-d['last_monotonic_s'])
 else: d['first_monotonic_s']=now
 d['last_monotonic_s']=now; d['count']+=1
 h=getattr(m,'header',None)
 if h: d['frame']=h.frame_id
 stamp=m.clock if t=='/clock' else h.stamp if h else None
 if stamp:
  v=stamp.sec*10**9+stamp.nanosec
  if 'last_stamp_ns' in d:
   d['duplicates']=d.get('duplicates',0)+(v==d['last_stamp_ns'])
   d['regressions']=d.get('regressions',0)+(v<d['last_stamp_ns'])
  else: d['first_stamp_ns']=v
  d['last_stamp_ns']=v
 for key in ('width','height','encoding','step','child_frame_id','armed','connected','distortion_model'):
  if hasattr(m,key): d[key]=getattr(m,key)
 if hasattr(m,'k'): d['K']=list(m.k); d['D']=list(m.d)
 if hasattr(m,'points'): d['points']=len(m.points)
 if hasattr(m,'transforms'): d['last_tf_edges']=[[v.header.frame_id,v.child_frame_id] for v in m.transforms]
for t,typ in spec.items():
 q=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL) if t=='/tf_static' else qos_profile_sensor_data
 n.create_subscription(get_message(typ),t,lambda m,t=t:summarize(t,m),q)
start=time.monotonic()
while time.monotonic()-start<12: rclpy.spin_once(n,timeout_sec=.1)
elapsed=time.monotonic()-start
for d in data.values(): d['observed_hz']=d['count']/elapsed
print(json.dumps({'domain':os.environ.get('ROS_DOMAIN_ID','0'),'window_s':elapsed,'topics':data},indent=2))
n.destroy_node()
rclpy.shutdown()
