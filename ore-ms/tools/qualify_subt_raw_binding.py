"""Bind SubT extracted camera/IMU exports to original ROS1 messages; no inference."""
import argparse, csv, io, json, hashlib, zipfile
from collections import Counter
from pathlib import Path
import numpy as np
import cv2
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore
from deployment_lighting import read, write, sha, stamp

ROOT=Path(__file__).resolve().parents[1]

def pixels(message,arm):
    assert len(message.data)==message.step*message.height
    if arm=='rgb':
        assert message.encoding in ['bgr8','rgb8'] and message.step>=message.width*3
        a=np.frombuffer(message.data,np.uint8).reshape(message.height,message.step)[:,:message.width*3].reshape(message.height,message.width,3)
        return cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if message.encoding=='rgb8' else a
    assert message.encoding in ['mono16','16UC1'] and message.step>=message.width*2
    b=np.frombuffer(message.data,np.uint8).reshape(message.height,message.step)[:,:message.width*2].copy()
    return b.view('>u2' if message.is_bigendian else '<u2').reshape(message.height,message.width).astype(np.uint16)

def numeric_imu(m):
    return np.array([m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z,m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z])

def nearest_index(times,t):
    j=int(np.searchsorted(times,t));return min([i for i in [j-1,j] if 0<=i<len(times)],key=lambda i:(abs(int(times[i])-t),i))

def run(config):
    cfg=read(config);out=ROOT/cfg['output'];out.mkdir(parents=True,exist_ok=False);cv2.setNumThreads(1)
    parent=ROOT/cfg['qualification'];images=read(parent/'images.json');frame_by_arm={a:sorted([r for r in images if r['arm']==a],key=lambda r:r['index']) for a in ['rgb','thermal']};times={a:np.array([r['time_ns'] for r in rows],np.int64) for a,rows in frame_by_arm.items()}
    frozen=read(parent/'input-freeze.json')['pins'];export=ROOT/cfg['export_archive'];assert sha(export)==frozen[cfg['export_archive']]
    transport=read(ROOT/cfg['raw_receipt']);raw=ROOT/cfg['raw_archive'];assert raw.stat().st_size==transport['bytes'] and sha(raw)==transport['sha256']
    with zipfile.ZipFile(export) as z:
        imu_member=next(n for n in z.namelist() if n.endswith('/imu_data.csv'));imu=[{k.strip():v.strip() for k,v in r.items()} for r in csv.DictReader(io.StringIO(z.read(imu_member).decode()))]
    fields=['q_x','q_y','q_z','q_w','ang_vel_x','ang_vel_y','ang_vel_z','lin_acc_x','lin_acc_y','lin_acc_z'];it=np.array([int(r['timestamp']) for r in imu],np.int64);iv=np.array([[float(r[k]) for k in fields] for r in imu]);assert np.all(np.diff(it)>0)
    pins={str(Path(config).resolve().relative_to(ROOT)):sha(config),cfg['raw_archive']:transport['sha256'],cfg['export_archive']:sha(export),'tools/qualify_subt_raw_binding.py':sha(Path(__file__)),cfg['protocol']:sha(ROOT/cfg['protocol']),str((parent/'images.json').relative_to(ROOT)):sha(parent/'images.json')}
    write(out/'input-freeze.json',{'pins':pins,'scope':'Raw header/pixel/IMU export identity only; no physical exposure or independent pose truth'})
    store=get_typestore(Stores.ROS1_NOETIC);seen={a:set() for a in ['rgb','thermal','imu']};counts=Counter();frames=[];imu_bindings=[];unmatched=[];different=[];duplicates=[];connections=[];headers={};topic_to_arm={v:k for k,v in cfg['topics'].items()};bagdir=ROOT/cfg['bag_output'];bagdir.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(raw) as rz,zipfile.ZipFile(export) as ez:
        members=sorted([m for m in rz.infolist() if m.filename.endswith('.bag')],key=lambda m:m.filename)
        for info in members:
            bag=bagdir/Path(info.filename).name
            assert not bag.exists(),bag
            digest=hashlib.sha256()
            with rz.open(info) as source,bag.open('xb') as dest:
                for b in iter(lambda:source.read(8*1024**2),b''):dest.write(b);digest.update(b)
            assert bag.stat().st_size==info.file_size
            with Reader(bag) as reader:
                for c in reader.connections:connections.append({'bag':bag.name,'topic':c.topic,'msgtype':c.msgtype,'msgcount':c.msgcount})
                selected=[c for c in reader.connections if c.topic in topic_to_arm]
                assert {c.topic for c in selected}==set(topic_to_arm),'Missing declared raw topic'
                for c,record_time,b in reader.messages(connections=selected):
                    arm=topic_to_arm[c.topic];m=store.deserialize_ros1(b,c.msgtype);t=stamp(m);counts[arm]+=1
                    if arm not in headers:
                        headers[arm]={'topic':c.topic,'msgtype':c.msgtype,'frame_id':m.header.frame_id}
                        if arm=='imu':headers[arm].update(orientation_covariance=m.orientation_covariance.tolist(),angular_velocity_covariance=m.angular_velocity_covariance.tolist())
                        else:headers[arm].update(encoding=m.encoding,width=m.width,height=m.height,step=m.step,is_bigendian=m.is_bigendian)
                    assert m.header.frame_id==headers[arm]['frame_id'],'Frame ID changed'
                    array_times=it if arm=='imu' else times[arm];i=nearest_index(array_times,t);delta=t-int(array_times[i])
                    if abs(delta)>cfg['max_stamp_difference_ns']:
                        unmatched.append({'arm':arm,'raw_ns':t,'nearest_export_ns':int(array_times[i]),'difference_ns':delta,'bag':bag.name});continue
                    if arm=='imu':
                        value=numeric_imu(m);same=bool(np.array_equal(value,iv[i]));row={'index':i,'raw_time_ns':t,'export_time_ns':int(it[i]),'difference_ns':delta,'numeric_equal':same,'bag':bag.name}
                        imu_bindings.append(row)
                    else:
                        f=frame_by_arm[arm][i];payload=ez.read(f['member']);assert hashlib.sha256(payload).hexdigest()==f['member_sha256'];expected=cv2.imdecode(np.frombuffer(payload,np.uint8),cv2.IMREAD_UNCHANGED);a=pixels(m,arm);same=bool(np.array_equal(a,expected));row={'arm':arm,'index':i,'raw_time_ns':t,'export_time_ns':int(times[arm][i]),'difference_ns':delta,'pixels_equal':same,'raw_pixel_sha256':hashlib.sha256(a.tobytes()).hexdigest(),'bag':bag.name,'record_time_ns':int(record_time)};frames.append(row)
                    if i in seen[arm]:duplicates.append({'arm':arm,'index':i,'raw_time_ns':t,'bag':bag.name})
                    seen[arm].add(i)
                    if not same:different.append(row)
            print('Bound raw bag',bag.name,dict(counts),flush=True)
            write(out/(bag.stem+'-receipt.json'),{'path':str(bag.relative_to(ROOT)),'bytes':bag.stat().st_size,'sha256':digest.hexdigest(),'zip_crc_verified':True})
    missing={a:sorted(set(range(len(it) if a=='imu' else len(times[a])))-seen[a]) for a in seen}
    write(out/'images.json',frames);write(out/'imu-bindings.json',imu_bindings);write(out/'connections.json',connections)
    report={'status':'pass' if not different and not any(missing.values()) else 'fail','raw_message_counts':dict(counts),'export_images':len(images),'export_imu_rows':len(it),'missing_export_indices':missing,'different_values':different,'raw_unmatched':unmatched,'duplicates':duplicates,'headers':headers,'max_stamp_difference_ns':{a:max([abs(r['difference_ns']) for r in (imu_bindings if a=='imu' else frames) if a=='imu' or r['arm']==a],default=None) for a in seen},'physical_exposure_binding':False,'independent_pose_truth':False}
    write(out/'qualification.json',report)
    for p,h in pins.items():assert sha(ROOT/p)==h,p
    write(out/'completion.json',{'pins':{str(p.relative_to(ROOT)):sha(p) for p in out.iterdir() if p.is_file()},'status':report['status']})
    print(json.dumps({k:v for k,v in report.items() if k not in ['raw_unmatched','duplicates','different_values','missing_export_indices']},indent=2));assert report['status']=='pass','Raw/export identity not established; stop gyro scoring'

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('config',type=Path);run(p.parse_args().config)
