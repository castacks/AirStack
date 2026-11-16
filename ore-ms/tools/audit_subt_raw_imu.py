"""Independently reread raw ROS IMU fields; no import of raw-binding runner."""
import json,hashlib
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore
ROOT=Path(__file__).resolve().parents[1]
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main(config):
    cfg=read(config);binding=ROOT/cfg['output'];parent=ROOT/cfg['qualification'];report=read(binding/'qualification.json');assert report['status']=='pass'
    saved=np.load(parent/'export-imu.npz');times=saved['time_ns'];values=saved['values'];store=get_typestore(Stores.ROS1_NOETIC);seen=set();total=0;largest=0;checks=0
    for p in sorted(binding.glob('*-receipt.json')):
        receipt=read(p);bag=ROOT/receipt['path'];assert bag.stat().st_size==receipt['bytes'] and sha(bag)==receipt['sha256'];checks+=1
        with Reader(bag) as reader:
            conns=[c for c in reader.connections if c.topic==cfg['topics']['imu']];assert len(conns)==1 and conns[0].msgtype=='sensor_msgs/msg/Imu';checks+=1
            for conn,_,b in reader.messages(connections=conns):
                m=store.deserialize_ros1(b,conn.msgtype);t=int(m.header.stamp.sec)*1_000_000_000+int(m.header.stamp.nanosec);j=int(np.searchsorted(times,t));j=min([i for i in [j-1,j] if 0<=i<len(times)],key=lambda i:(abs(int(times[i])-t),i));delta=abs(int(times[j])-t)
                if delta>cfg['max_stamp_difference_ns']:continue
                q=m.orientation;w=m.angular_velocity;a=m.linear_acceleration;v=np.array([q.x,q.y,q.z,q.w,w.x,w.y,w.z,a.x,a.y,a.z]);assert np.array_equal(v,values[j]);assert m.header.frame_id==report['headers']['imu']['frame_id'];seen.add(j);total+=1;largest=max(largest,delta);checks+=2
        print('Independently checked IMU',bag.name,total,flush=True)
    assert seen==set(range(len(times)));assert total==sum(r['numeric_equal'] for r in read(binding/'imu-bindings.json'));checks+=2
    result={'status':'pass','checks':checks,'raw_matched_rows':total,'unique_export_rows':len(seen),'max_stamp_difference_ns':largest,'raw_imu_frame_id':report['headers']['imu']['frame_id'],'auditor_sha256':sha(Path(__file__)),'scope':'Independent raw ROS header and quaternion/gyro/acceleration numeric binding to export; no calibrated physical pose truth'}
    (binding/'independent-imu-audit.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    import sys
    main(Path(sys.argv[1]))
