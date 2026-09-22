"""Read recorded GT poses to verify grounded lead-in and usable timestamps."""
import json,sys
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

bag=Path(sys.argv[1]);r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/ws2/ground_truth/pose']))
first=last=None;air=None;n=0;reversals=0;max_gap=0
while r.has_next():
    topic,data,record_time=r.read_next();m=deserialize_message(data,PoseStamped)
    t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    item={'sim_time':t,'position':[m.pose.position.x,m.pose.position.y,m.pose.position.z]}
    if first is None:first=item
    if last is not None:
        if t<last['sim_time']:reversals+=1
        max_gap=max(max_gap,t-last['sim_time'])
    if air is None and m.pose.position.z>.3:air=t
    last=item;n+=1
result={'poses':n,'first':first,'last':last,'timestamp_reversals':reversals,'max_pose_gap_sim_s':max_gap,
        'grounded_lead_in_sim_s':None if first is None or air is None else air-first['sim_time']}
result['passed']=bool(n and first['position'][2]<.3 and air is not None and air-first['sim_time']>=1 and reversals==0)
(bag.parent/'bag_verification.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
if not result['passed']:sys.exit(1)
