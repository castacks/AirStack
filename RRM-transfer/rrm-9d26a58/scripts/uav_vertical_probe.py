#!/usr/bin/env python3
"""Bounded simulation-only public-action probe. Run inside the robot ROS overlay.

No direct actuator publication, forced arming, or airborne disarm. Output directory
must not exist. Success here is estimator-observed functionality, never physical
qualification without separately supplied simulator truth/contact evidence.
"""
import argparse
import hashlib
import os
import platform
import uuid
import json
import math
from pathlib import Path
import time
import copy
import signal
import subprocess


def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State, ExtendedState
    from std_msgs.msg import Bool, String
    from sensor_msgs.msg import CameraInfo, Image, BatteryState
    from rcl_interfaces.msg import Log
    from unique_identifier_msgs.msg import UUID
    from task_msgs.action import TakeoffTask, LandTask, NavigateTask, FixedTrajectoryTask
    from diagnostic_msgs.msg import KeyValue
    from airstack_msgs.msg import Odometry as Reference
    from airstack_msgs.srv import RobotCommand
    from geometry_msgs.msg import PoseStamped
    from rosidl_runtime_py.convert import message_to_ordereddict

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True)
    ap.add_argument('--execute-simulation', action='store_true')
    ap.add_argument('--observe-seconds', type=float, default=8)
    ap.add_argument('--inspect-offset-m', type=float, default=0.0, help='optional map +x viewpoint, max 1 m')
    ap.add_argument('--survey-size-m', type=float, default=0.0, help='bounded 3-lane survey route, max 0.75 m square')
    ap.add_argument('--fixed-circle-radius-m', type=float, default=0.0)
    ap.add_argument('--supervised-stop', action='store_true')
    ap.add_argument('--automatic-state-fault-stop', action='store_true')
    ap.add_argument('--marker-viewpoint', action='store_true', help='select one bounded viewpoint from observed red fixture')
    args = ap.parse_args()
    if args.marker_viewpoint and any((args.inspect_offset_m,args.survey_size_m,args.fixed_circle_radius_m,args.supervised_stop,args.automatic_state_fault_stop)):
        ap.error('marker-viewpoint is exclusive with other route/stop modes')
    if args.automatic_state_fault_stop and (not args.execute_simulation or not args.inspect_offset_m or args.supervised_stop):
        ap.error('automatic fault stop requires execute-simulation and inspect-offset-m, exclusive with supervised-stop')
    if args.supervised_stop and not args.inspect_offset_m:ap.error('supervised stop requires inspect-offset-m route')
    if not math.isfinite(args.fixed_circle_radius_m) or not 0 <= args.fixed_circle_radius_m <= .5 or sum(bool(v) for v in (args.fixed_circle_radius_m,args.survey_size_m,args.inspect_offset_m))>1:
        ap.error('circle radius in [0,0.5], select only one motion task')
    if not math.isfinite(args.survey_size_m) or not 0 <= args.survey_size_m <= .75 or (args.survey_size_m and args.inspect_offset_m):
        ap.error('survey size must be finite in [0,0.75], exclusive with inspect offset')
    if not math.isfinite(args.inspect_offset_m) or not 0 <= args.inspect_offset_m <= 1:
        ap.error('inspection offset must be finite in [0,1] m')
    out = Path(args.output); out.mkdir(parents=True, exist_ok=False)
    log = (out / 'events.jsonl').open('w', buffering=1)
    rclpy.init(); node = Node('rrm_uav_vertical_probe')
    latest = {}; phase = 'preflight'; samples = []; violations = []
    origin = None; handle = None; pending_goal = None
    inject_fault=False; qualification_process=None; qualification_log=None
    fault_namespace='/robot_1/watchdog_qualification'
    fault_relay=node.create_publisher(Odometry,fault_namespace+'/state',10) if args.automatic_state_fault_stop else None
    marker_records={};marker_window_starts={};marker_decision=None;tf_buffer=None

    def safe(value):
        if isinstance(value, float) and not math.isfinite(value):
            return {"nonfinite": str(value)}
        if isinstance(value, dict): return {k: safe(v) for k,v in value.items()}
        if isinstance(value, (list,tuple)): return [safe(v) for v in value]
        return value

    def emit(kind, **kw):
        log.write(json.dumps(dict(wall_unix=time.time(), monotonic=time.monotonic(),
                                  phase=phase, kind=kind, **safe(kw)), allow_nan=False) + '\n')

    def cb(name, msg):
        nonlocal origin
        now = time.monotonic()
        # Images retain metadata, not huge pixel arrays. Full image capture is separate.
        if name in ('image','marker_depth'):
            value = dict(stamp=message_to_ordereddict(msg.header), width=msg.width,
                         height=msg.height, encoding=msg.encoding, bytes=len(msg.data))
        else:
            value = message_to_ordereddict(msg)
        latest[name] = (now, msg)
        if name == 'odom':
            if fault_relay is not None:
                forwarded=copy.deepcopy(msg)
                original_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                if inject_fault:
                    forwarded.header.stamp.sec-=10
                fault_relay.publish(forwarded)
                emit('fault_relay',injected=inject_fault,original_source_s=original_stamp,
                     forwarded_source_s=forwarded.header.stamp.sec+forwarded.header.stamp.nanosec*1e-9)
            p = msg.pose.pose.position; v = msg.twist.twist.linear
            nums = (p.x,p.y,p.z,v.x,v.y,v.z)
            if not all(math.isfinite(x) for x in nums):
                violations.append('nonfinite_odometry'); return
            samples.append(dict(wall=now, phase=phase, sim=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                                x=p.x,y=p.y,z=p.z,speed=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)))
            if origin is not None:
                if math.hypot(p.x-origin[0],p.y-origin[1]) > 2: violations.append('horizontal_radius')
                if p.z-origin[2] > 3 or p.z-origin[2] < -0.3: violations.append('altitude_envelope')
                if samples[-1]['speed'] > 2: violations.append('speed_envelope')
        emit(name, data=value)

    subs=[]
    for name,typ,topic in [
        ('reference',Reference,'trajectory_controller/tracking_point'),
        ('odom',Odometry,'odometry_conversion/odometry'),
        ('state',State,'interface/mavros/state'),
        ('landed',ExtendedState,'interface/mavros/extended_state'),
        ('actuation',Bool,'interface/actuation_ready'),
        ('watchdog',Bool,'behavior/drone_safety_monitor/state_estimate_timed_out'),
        ('camera_info',CameraInfo,'sensors/front_stereo/left/camera_info'),
        ('image',Image,'sensors/front_stereo/left/image_rect'),
        ('battery',BatteryState,'interface/mavros/battery')]:
        subs.append(node.create_subscription(typ,'/robot_1/'+topic,lambda m,n=name:cb(n,m),
            10 if name in ('state','landed','watchdog','actuation') else qos_profile_sensor_data))
    takeoff=ActionClient(node,TakeoffTask,'/robot_1/tasks/takeoff')
    subs.append(node.create_subscription(Log,'/rosout',lambda m:cb('controller_log',m) if ('drone_safety_monitor' in m.name or 'trajectory_control' in m.name) else None,10))
    pause_pub=node.create_publisher(String,'/robot_1/behavior/drone_safety_monitor/command',10)
    if args.marker_viewpoint:
        subs.append(node.create_subscription(Image,'/robot_1/sensors/front_stereo/left/depth_ground_truth',lambda m:cb('marker_depth',m),qos_profile_sensor_data))
    if args.automatic_state_fault_stop:
        for name,typ,topic in [('fault_watchdog',Bool,'state_estimate_timed_out'),
                               ('fault_ack',Bool,'pause_acknowledged'),
                               ('fault_reason',String,'state_health_reason')]:
            subs.append(node.create_subscription(typ,fault_namespace+'/'+topic,lambda m,n=name:cb(n,m),10))
    robot_command=node.create_client(RobotCommand,'/robot_1/interface/robot_command')
    fixed=ActionClient(node,FixedTrajectoryTask,'/robot_1/tasks/fixed_trajectory')
    navigate=ActionClient(node,NavigateTask,'/robot_1/tasks/navigate')
    land=ActionClient(node,LandTask,'/robot_1/tasks/land')

    def fresh(names=('odom','state','landed','watchdog','actuation')):
        return all(n in latest and time.monotonic()-latest[n][0]<2 for n in names)
    def spin(duration, guard=False):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
            if guard and (violations or not fresh() or latest['watchdog'][1].data or not latest['actuation'][1].data):
                emit('guard_snapshot',ages_s={n:time.monotonic()-v[0] for n,v in latest.items()},
                     violations=sorted(set(violations)),watchdog=latest.get('watchdog',(None,None))[1].data if 'watchdog' in latest else None,
                     actuation=latest.get('actuation',(None,None))[1].data if 'actuation' in latest else None)
                raise RuntimeError('envelope_or_stale_state:'+','.join(sorted(set(violations))))
    def wait(future, seconds, guard=False):
        end=time.monotonic()+seconds
        while not future.done() and time.monotonic()<end: spin(0.05,guard)
        if not future.done(): raise TimeoutError('action response deadline')
        return future.result()
    def action(client, goal, guard, supervised_stop=False, admission_check=None):
        nonlocal handle,pending_goal,inject_fault
        from rrm.uav_probe_admission import wait_for_checked_server
        wait_for_checked_server(client,admission_check)
        goal_id=UUID(uuid=list(uuid.uuid4().bytes));emit('goal_submitted',goal_uuid=[int(v) for v in goal_id.uuid])
        pending_goal=client.send_goal_async(goal,feedback_callback=lambda f:emit('feedback',data=message_to_ordereddict(f.feedback)),goal_uuid=goal_id)
        handle=wait(pending_goal,8,guard);pending_goal=None
        emit('goal_acceptance',accepted=handle.accepted,goal_uuid=[int(v) for v in goal_id.uuid])
        if not handle.accepted:
            handle=None;raise RuntimeError('goal_rejected')
        future=handle.get_result_async();stop_observed=False
        if supervised_stop:
            start_sample=samples[-1];pause_at=None;stable=None;max_pause_displacement=0.;deadline=time.monotonic()+90
            while not future.done() and time.monotonic()<deadline:
                spin(.05,guard);sample=samples[-1];now=time.monotonic()
                if pause_at is None and sample['speed']>.1 and math.hypot(sample['x']-start_sample['x'],sample['y']-start_sample['y'])>=.2:
                    pause_at=now;pause_pose=(sample['x'],sample['y'],sample['z'])
                    if args.automatic_state_fault_stop:
                        inject_fault=True
                        emit('state_fault_injected',source_age_added_s=10,position_map_m=pause_pose,
                             observed_speed_m_s=sample['speed'],response_budget_wall_s=6)
                    else:
                        if pause_pub.get_subscription_count()<1:raise RuntimeError('supervisory_monitor_subscription_missing')
                        pause_pub.publish(String(data='pause'))
                        emit('supervisory_pause',position_map_m=pause_pose,observed_speed_m_s=sample['speed'],response_budget_wall_s=6)
                if pause_at is not None:
                    distance=math.dist((sample['x'],sample['y'],sample['z']),pause_pose)
                    max_pause_displacement=max(max_pause_displacement,distance)
                    if distance>.5:raise RuntimeError('supervisory_stopping_distance_exceeded')
                    stable=(stable or now) if sample['speed']<.1 else None
                    if stable and now-stable>=1:
                        if args.automatic_state_fault_stop and not (fresh(('fault_reason','fault_ack','fault_watchdog')) and
                            latest['fault_reason'][1].data=='stale_source_stamp' and latest['fault_ack'][1].data and
                            latest['fault_watchdog'][1].data and latest['fault_ack'][0]>pause_at):
                            raise RuntimeError('automatic_fault_detection_or_ack_unconfirmed')
                        stop_observed=True
                        results['supervised_stop_measurement']=dict(latency_including_dwell_wall_s=now-pause_at,endpoint_displacement_m=distance,max_extra_displacement_m=max_pause_displacement)
                        emit('supervisory_stop_observed',**results['supervised_stop_measurement'])
                        wait(handle.cancel_goal_async(),5,guard);break
                    if now-pause_at>6:raise RuntimeError('supervisory_physical_stop_deadline')
            if not stop_observed:
                if future.done():
                    terminal=future.result();emit('action_result',status=terminal.status,data=message_to_ordereddict(terminal.result));handle=None
                raise RuntimeError('moving_supervisory_stop_not_demonstrated')
        result=wait(future,10 if supervised_stop else 90,guard)
        emit('action_result',status=result.status,data=message_to_ordereddict(result.result))
        handle=None
        return (result.status==5 and stop_observed) if supervised_stop else (result.status==4 and result.result.success)

    def observe_marker(label):
        records=[];seen=set();started=time.monotonic();deadline=started+5
        marker_window_starts[label]=started
        while len(records)<3 and time.monotonic()<deadline:
            spin(.02,True)
            if not fresh(('image','marker_depth','camera_info')) or latest['image'][0]<started:continue
            rgb,depth,info=(latest[n][1] for n in ('image','marker_depth','camera_info'))
            stamps=[(m.header.stamp.sec,m.header.stamp.nanosec) for m in (rgb,depth,info)]
            if len(set(stamps))!=1 or stamps[0] in seen:continue
            if records and stamps[0]<=(records[-1]['stamp_sec'],records[-1]['stamp_nanosec']):
                raise RuntimeError('marker_source_stamp_regressed')
            source=stamps[0][0]+stamps[0][1]*1e-9
            odom_stamp=latest['odom'][1].header.stamp
            if abs(source-odom_stamp.sec-odom_stamp.nanosec*1e-9)>.2:continue
            pair=camera_pair_metadata(rgb,depth,info)
            channel_receipts={key:latest[name][0] for key,name in
                              (('rgb','image'),('depth','marker_depth'),('info','camera_info'))}
            if any(value<started for value in channel_receipts.values()):continue
            seen.add(stamps[0]);receipt=time.time()-(time.monotonic()-latest['image'][0])
            color=marker_bridge.imgmsg_to_cv2(rgb,desired_encoding='rgb8')
            distance=marker_bridge.imgmsg_to_cv2(depth,desired_encoding='passthrough')
            intrinsic=[float(x) for x in info.k]
            observation=detect_red_surface(color,distance,intrinsic)
            prefix=f'marker-{label}-{len(records):02d}'
            if not marker_cv2.imwrite(str(out/(prefix+'.png')),marker_cv2.cvtColor(color,marker_cv2.COLOR_RGB2BGR)):
                raise RuntimeError('marker_image_write_failed')
            marker_np.savez_compressed(out/(prefix+'.npz'),depth_m=distance,intrinsic=marker_np.array(intrinsic))
            record=dict(label=label,raw_prefix=prefix,frame=rgb.header.frame_id,image_size=[rgb.width,rgb.height],
                        intrinsic=intrinsic,stamp_sec=stamps[0][0],stamp_nanosec=stamps[0][1],
                        exact_stamp_pair=True,image_receipt_wall_unix=receipt,observation=observation,
                        headers=pair['headers'],receipt_monotonic_s=channel_receipts,
                        depth_encoding=depth.encoding)
            records.append(record);emit('marker_observation',**record)
        if len(records)!=3:raise RuntimeError('fresh_exact_marker_triplets_missing')
        evidence=camera_window_metadata(records,started_monotonic=started,now_monotonic=time.monotonic())
        emit('marker_window_checked',label=label,evidence=evidence)
        marker_records[label]=records
        return records

    def check_marker_dispatch():
        spin(.001,True)
        evidence=camera_window_metadata(marker_records['before'],
            started_monotonic=marker_window_starts['before'],now_monotonic=time.monotonic())
        emit('marker_dispatch_window_checked',evidence=evidence)

    results={}; error=None
    try:
        if args.marker_viewpoint:
            import cv2 as marker_cv2
            import numpy as marker_np
            import inspect
            from cv_bridge import CvBridge
            from tf2_ros import Buffer,TransformListener
            from rrm.uav_marker import detect_red_surface
            from rrm.uav_viewpoint import select_viewpoint,optical_forward_in_map
            from rrm.camera_pairing import camera_pair_metadata,camera_window_metadata
            marker_bridge=CvBridge();tf_buffer=Buffer(node=node);tf_listener=TransformListener(tf_buffer,node)
            emit('marker_policy_provenance',policy_sha256=hashlib.sha256(Path(inspect.getfile(select_viewpoint)).read_bytes()).hexdigest(),
                 detector_sha256=hashlib.sha256(Path(inspect.getfile(detect_red_surface)).read_bytes()).hexdigest(),
                 camera_window_sha256=hashlib.sha256(Path(inspect.getfile(camera_window_metadata)).read_bytes()).hexdigest())
        if args.automatic_state_fault_stop:
            qualification_log=(out/'qualification-monitor.log').open('w')
            qualification_process=subprocess.Popen([
                'ros2','run','drone_safety_monitor','drone_safety_monitor','--ros-args',
                '-p','use_sim_time:=true','-r','__ns:='+fault_namespace,
                '-r','state_estimate:='+fault_namespace+'/state',
                '-r','set_trajectory_mode:=/robot_1/trajectory_controller/set_trajectory_mode'],
                stdout=qualification_log,stderr=subprocess.STDOUT,start_new_session=True)
            emit('qualification_monitor_started',process_group=qualification_process.pid,
                 state_input=fault_namespace+'/state',mode_service='/robot_1/trajectory_controller/set_trajectory_mode')
        emit('provenance',probe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
             python=platform.python_version(),ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
             arguments=vars(args))
        emit('procedure',simulation_only=True,target_absolute_map_z_m=2.0,velocity_m_s=0.5,
             limits=dict(horizontal_radius_m=2,relative_max_z_m=3,max_speed_m_s=2),
             acceptance=dict(target_tolerance_m=0.3,hover_duration_wall_s=8,final_ground_height_tolerance_m=0.3),
             independent_physical_evidence='NOT_AVAILABLE')
        spin(args.observe_seconds)
        emit('observed_channels',channels=list(latest),graph=node.get_topic_names_and_types())
        if not args.execute_simulation:
            results['execution']='NOT_TESTED'; return
        if not fresh(): raise RuntimeError('preflight_missing_or_stale_state')
        if args.automatic_state_fault_stop and not (qualification_process.poll() is None and
            fresh(('fault_watchdog','fault_ack','fault_reason')) and not latest['fault_watchdog'][1].data and
            latest['fault_ack'][1].data and latest['fault_reason'][1].data=='healthy'):
            raise RuntimeError('qualification_monitor_preflight_not_ready')
        st=latest['state'][1]; od=latest['odom'][1]; p=od.pose.pose.position
        if not latest['actuation'][1].data or not st.connected or st.armed or latest['landed'][1].landed_state!=1 or latest['watchdog'][1].data:
            raise RuntimeError('preflight_not_connected_disarmed_ground_healthy')
        if od.header.frame_id!='map' or abs(p.z)>0.25: raise RuntimeError('unqualified_frame_or_floor')
        origin=(p.x,p.y,p.z)
        phase='takeoff'; goal=TakeoffTask.Goal();goal.target_altitude_m=2.;goal.velocity_m_s=0.5
        results['takeoff_action']=action(takeoff,goal,True)
        if not results['takeoff_action']: raise RuntimeError('takeoff_failed')
        phase='hover';spin(8,True)
        hover=[s for s in samples if s['phase']=='hover']
        results['hover_in_tolerance']=bool(hover) and all(abs(s['z']-2.)<=0.3 for s in hover)
        if not results['hover_in_tolerance']: raise RuntimeError('hover_altitude_failed')
        if args.marker_viewpoint:
            phase='marker_before';before=observe_marker('before')
            # Query at an actual observation stamp; do not substitute a latest TF.
            area=[r['observation'].get('bbox_xywh',[0,0,0,0])[2:4] for r in before]
            areas=[v[0]*v[1] for v in area]
            selected_index=min(range(3),key=lambda i:abs(areas[i]-sorted(areas)[1]))
            selected=before[selected_index]
            from rclpy.time import Time
            from rclpy.duration import Duration
            from builtin_interfaces.msg import Time as Stamp
            stamp=Time.from_msg(Stamp(sec=selected['stamp_sec'],nanosec=selected['stamp_nanosec']))
            deadline=time.monotonic()+3
            while not tf_buffer.can_transform('map',selected['frame'],stamp,timeout=Duration()):
                spin(.02,True)
                if time.monotonic()>deadline:raise RuntimeError('marker_acquisition_transform_unavailable')
            transform=tf_buffer.lookup_transform('map',selected['frame'],stamp)
            camera_window_metadata(before,started_monotonic=marker_window_starts['before'],now_monotonic=time.monotonic())
            q=transform.transform.rotation
            axis=optical_forward_in_map([q.x,q.y,q.z,q.w])
            marker_decision=select_viewpoint([r['observation'] for r in before],selected['image_size'],axis)
            emit('marker_viewpoint_decision',decision=marker_decision,before_records=before,
                 optical_axis_map=axis,transform=message_to_ordereddict(transform),
                 transform_observation_index=selected_index)
            if marker_decision['status']!='SELECTED':raise RuntimeError('marker_policy_abstained:'+marker_decision['reason'])
        if args.fixed_circle_radius_m:
            phase='fixed_circle';p=latest['odom'][1].pose.pose.position;circle_start=(p.x,p.y,p.z)
            sample_start=len(samples);goal=FixedTrajectoryTask.Goal();goal.trajectory_spec.type='Circle';goal.loop=False
            goal.trajectory_spec.attributes=[KeyValue(key=k,value=str(v)) for k,v in
                [('radius',args.fixed_circle_radius_m),('velocity',.3),('frame_id','base_link')]]
            emit('fixed_circle_goal',radius_m=args.fixed_circle_radius_m,velocity_m_s=.3,frame='base_link',return_position_map_m=circle_start)
            results['fixed_action']=action(fixed,goal,True)
            if not results['fixed_action']:raise RuntimeError('fixed_trajectory_failed')
            phase='fixed_settle';end=time.monotonic()+20;stable_since=None
            while time.monotonic()<end:
                spin(.1,True);a=samples[-1]
                near=math.dist((a['x'],a['y'],a['z']),circle_start)<=.3 and a['speed']<.2
                stable_since=(stable_since or time.monotonic()) if near else None
                if stable_since and time.monotonic()-stable_since>=2:break
            else:raise RuntimeError('fixed_trajectory_return_unverified')
            radius=max(math.hypot(a['x']-circle_start[0],a['y']-circle_start[1]) for a in samples[sample_start:])
            results['fixed_observed_radius_m']=radius
            if radius<.25:raise RuntimeError('fixed_trajectory_no_measured_movement')
        if args.inspect_offset_m or args.survey_size_m or args.marker_viewpoint:
            launch=latest['odom'][1].pose.pose.position
            anchor=(launch.x,launch.y)
            d=args.survey_size_m
            offsets=([(args.inspect_offset_m,0.)] if not d else
                     [(d,0.),(d,d/2),(0.,d/2),(0.,d),(d,d)])
            if args.marker_viewpoint:offsets=[tuple(marker_decision['delta_map_m'][:2])]
            results['viewpoints']=[]
            for waypoint_index,(dx,dy) in enumerate(offsets):
                phase='navigate_viewpoint'
                current=latest['odom'][1].pose.pose.position
                start=(current.x,current.y,current.z)
                target=(anchor[0]+dx,anchor[1]+dy,2.)
                emit('viewpoint_goal',target_map_m=target,physical_tolerance_m=0.25,max_settle_wall_s=20)
                goal=NavigateTask.Goal();goal.goal_tolerance_m=0.1;goal.global_plan.header.frame_id='map'
                goal.global_plan.header.stamp=node.get_clock().now().to_msg()
                for i in range(11):
                    pose=PoseStamped();pose.header=goal.global_plan.header
                    pose.pose.position.x=start[0]+(target[0]-start[0])*i/10
                    pose.pose.position.y=start[1]+(target[1]-start[1])*i/10;pose.pose.position.z=2.;pose.pose.orientation.w=1.
                    goal.global_plan.poses.append(pose)
                stop_mode=args.supervised_stop or args.automatic_state_fault_stop
                navigation_ok=action(navigate,goal,True,stop_mode,
                    admission_check=check_marker_dispatch if args.marker_viewpoint else None)
                if stop_mode:results['navigation_canceled_as_requested']=navigation_ok
                else:results['navigate_action']=navigation_ok
                if not navigation_ok: raise RuntimeError('navigation_failed')
                if stop_mode:
                    results['supervised_stop_and_cancel']=True
                    results['automatic_state_fault_stop']=args.automatic_state_fault_stop
                    break
                phase='viewpoint_settle';end=time.monotonic()+20;stable_since=None
                while time.monotonic()<end:
                    spin(.1,True);sample=samples[-1]
                    near=math.dist((sample['x'],sample['y'],sample['z']),target)<=.25 and sample['speed']<.2
                    stable_since=(stable_since or time.monotonic()) if near else None
                    if stable_since and time.monotonic()-stable_since>=2:break
                else:raise RuntimeError('actual_vehicle_did_not_settle_at_viewpoint')
                phase='inspection_capture';spin(1,True)
                if not fresh(('image','camera_info')): raise RuntimeError('fresh_camera_evidence_missing')
                from cv_bridge import CvBridge
                import cv2
                image=latest['image'][1]
                if not cv2.imwrite(str(out/f'inspection-{waypoint_index:02d}.png'),CvBridge().imgmsg_to_cv2(image,desired_encoding='bgr8')):
                    raise RuntimeError('image_write_failed')
                emit('inspection_capture',image=f'inspection-{waypoint_index:02d}.png',header=message_to_ordereddict(image.header),
                     pose=message_to_ordereddict(latest['odom'][1].pose.pose),
                     image_receipt_age_s=time.monotonic()-latest['image'][0],
                     semantic_asset_inspection=False)
                results['viewpoint_camera_capture']=True
                results['viewpoints'].append(dict(index=waypoint_index,target_map_m=target,image=f'inspection-{waypoint_index:02d}.png'))
                if args.marker_viewpoint:
                    phase='marker_after';after=observe_marker('after')
                    if any(r['observation'].get('status')!='OBSERVED' for r in after):raise RuntimeError('marker_after_abstained')
                    import statistics
                    before_area=statistics.median(r['observation']['bbox_xywh'][2]*r['observation']['bbox_xywh'][3] for r in marker_records['before'])
                    after_area=statistics.median(r['observation']['bbox_xywh'][2]*r['observation']['bbox_xywh'][3] for r in after)
                    before_z=statistics.median(r['observation']['surface_depth_z_m'] for r in marker_records['before'])
                    after_z=statistics.median(r['observation']['surface_depth_z_m'] for r in after)
                    same_calibration=all(r['intrinsic']==before[0]['intrinsic'] and r['frame']==before[0]['frame'] and r['image_size']==before[0]['image_size'] for r in after+before)
                    results['marker_viewpoint_quality']=dict(bbox_area_ratio=after_area/before_area,depth_reduction_m=before_z-after_z,
                        final_surface_depth_z_m=after_z,same_calibration=same_calibration)
                    emit('marker_viewpoint_quality',**results['marker_viewpoint_quality'])
                    if not(same_calibration and after_area/before_area>=1.2 and before_z-after_z>=.2 and after_z>=2.4):
                        raise RuntimeError('marker_resolution_gain_or_standoff_failed')
                    results['marker_viewpoint_action']=True
    except Exception as e:
        error=repr(e);emit('error',error=error)
    finally:
        if inject_fault:
            inject_fault=False
            emit('state_fault_cleared')
            spin(.5)
        # An accepted or pending request can arm after the latest disarmed sample.
        unresolved=False;terminal_future=None
        if pending_goal is not None:
            try:
                handle=wait(pending_goal,8);pending_goal=None
                emit('pending_goal_acceptance_reconciled',accepted=handle.accepted)
                if not handle.accepted:handle=None
                results['goal_submission_reconciled']=True
            except Exception as e:
                unresolved=True;emit('goal_submission_unresolved',error=repr(e))
        if handle is not None:
            try:
                wait(handle.cancel_goal_async(),5)
                terminal_future=handle.get_result_async();terminal=wait(terminal_future,8)
                emit('canceled_action_terminal',status=terminal.status,data=message_to_ordereddict(terminal.result))
                if terminal.status not in (4,5,6):raise RuntimeError('prior_action_terminal_unconfirmed')
                handle=None;terminal_future=None
            except Exception as e:
                unresolved=True;emit('prior_action_unresolved',error=repr(e))
        if args.execute_simulation and (unresolved or ('state' in latest and latest['state'][1].armed)):
            phase='recovery_land' if error or unresolved else 'land'
            try:
                if unresolved:raise RuntimeError('skip_task_trajectory_while_prior_goal_unknown')
                goal=LandTask.Goal();goal.velocity_m_s=0.5
                results['land_action']=action(land,goal,False)
                spin(4)
            except Exception as e:emit('recovery_error',error=repr(e));results['land_action']=False
            if not results.get('land_action') or not (fresh(('state','landed')) and not latest['state'][1].armed and latest['landed'][1].landed_state==1):
                try:
                    if not robot_command.wait_for_service(timeout_sec=3):raise RuntimeError('public_land_service_unavailable')
                    request=RobotCommand.Request();request.command=RobotCommand.Request.LAND
                    response=wait(robot_command.call_async(request),5)
                    results['fallback_land_requested']=response.success
                    emit('public_fallback_land',accepted=response.success,prior_task_terminal_unconfirmed=unresolved)
                    if not response.success:raise RuntimeError('public_land_rejected')
                    deadline=time.monotonic()+90
                    while time.monotonic()<deadline:
                        spin(.1)
                        if pending_goal is not None and pending_goal.done():
                            handle=pending_goal.result();pending_goal=None
                            if handle.accepted:
                                handle.cancel_goal_async();terminal_future=handle.get_result_async()
                            else:handle=None;unresolved=False
                        if terminal_future is not None and terminal_future.done():
                            terminal=terminal_future.result()
                            if terminal.status in (4,5,6):
                                emit('fallback_prior_goal_reconciled',status=terminal.status);handle=None;terminal_future=None;unresolved=False
                        if not unresolved and fresh(('state','landed')) and not latest['state'][1].armed and latest['landed'][1].landed_state==1:break
                    results['fallback_ground_disarm_observed']=fresh(('state','landed')) and not latest['state'][1].armed and latest['landed'][1].landed_state==1
                except Exception as e:emit('fallback_land_error',error=repr(e))
        results['pending_goal_unreconciled']=unresolved or pending_goal is not None or handle is not None
        if results['pending_goal_unreconciled']:error=error or 'pending_goal_terminal_unconfirmed'
        results['fresh_final_state']=fresh()
        results['final_disarmed']=fresh(('state',)) and not latest['state'][1].armed
        results['final_on_ground']=fresh(('landed',)) and latest['landed'][1].landed_state==1
        if qualification_process is not None:
            if results['final_disarmed'] and results['final_on_ground']:
                if qualification_process.poll() is None:
                    os.killpg(qualification_process.pid,signal.SIGINT)
                    try:qualification_process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(qualification_process.pid,signal.SIGKILL);qualification_process.wait()
                results['qualification_monitor_stopped']=True
            else:
                results['qualification_monitor_stopped']=False
                emit('qualification_monitor_retained',process_group=qualification_process.pid,
                     reason='final_ground_disarm_unconfirmed')
            qualification_log.close()
        if args.execute_simulation and not (results['fresh_final_state'] and results['final_disarmed'] and results['final_on_ground']):
            error=error or 'terminal_ground_disarm_unverified'
        results['violations']=sorted(set(violations));results['error']=error
        results['physical_qualification']='NOT_DEMONSTRATED: separate PhysX/contact evidence must be assessed independently'
        if samples:
            results['metrics']=dict(samples=len(samples),min_z=min(s['z'] for s in samples),max_z=max(s['z'] for s in samples),
                max_speed=max(s['speed'] for s in samples),final_z=samples[-1]['z'],
                distance_m=sum(math.dist((a['x'],a['y'],a['z']),(b['x'],b['y'],b['z'])) for a,b in zip(samples,samples[1:])))
        (out/'summary.json').write_text(json.dumps(results,indent=2))
        emit('summary',data=results);log.close();node.destroy_node();rclpy.shutdown()
    if error: raise SystemExit(1)

if __name__=='__main__':main()
