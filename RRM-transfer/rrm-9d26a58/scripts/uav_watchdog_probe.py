#!/usr/bin/env python3
"""Exercise real safety-monitor binary against mock controller, isolated DDS domain.
This tests detection and service requests, not physical recovery or flight safety.
"""
import json,os,subprocess,time,argparse
from pathlib import Path


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='88':raise SystemExit('requires isolated ROS_DOMAIN_ID=88')
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool,String
    from airstack_msgs.srv import TrajectoryMode
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);args=ap.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=Node('uav_watchdog_test');events=[];phase='startup'
    def stamp(kind,value):events.append(dict(t=time.monotonic(),phase=phase,kind=kind,value=value))
    def service(req,res):stamp('mode',req.mode);res.success=True;return res
    srv=n.create_service(TrajectoryMode,'/qualification/mode',service)
    sub=n.create_subscription(Bool,'/qualification/timedout',lambda m:stamp('timedout',m.data),10)
    odom=n.create_publisher(Odometry,'/qualification/state_estimate',10)
    command=n.create_publisher(String,'/qualification/command',10)
    log=(out/'monitor.log').open('w')
    proc=subprocess.Popen(['ros2','run','drone_safety_monitor','drone_safety_monitor','--ros-args',
        '-p','use_sim_time:=false','-p','state_estimate_timeout:=0.5',
        '-r','state_estimate:=/qualification/state_estimate','-r','command:=/qualification/command',
        '-r','state_estimate_timed_out:=/qualification/timedout','-r','set_trajectory_mode:=/qualification/mode'],stdout=log,stderr=subprocess.STDOUT)
    def spin(seconds,publish=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if publish:
                m=Odometry();m.header.stamp=n.get_clock().now().to_msg();odom.publish(m)
            rclpy.spin_once(n,timeout_sec=.03)
    result={}
    try:
        spin(3)
        phase='healthy';spin(3,True)
        phase='silence';spin(3)
        healthy=[x for x in events if x['phase']=='healthy' and x['kind']=='timedout']
        silence=[x for x in events if x['phase']=='silence']
        healthy_times=[x['t'] for x in healthy]
        healthy_intervals=[b-a for a,b in zip(healthy_times,healthy_times[1:])]
        result['healthy_cadence_ok']=bool(healthy_intervals) and max(healthy_intervals)<=.75
        result['healthy_clears_timeout']=any(x['value'] is False for x in healthy)
        result['silence_detected']=any(x['kind']=='timedout' and x['value'] is True for x in silence)
        result['pause_requested']=any(x['kind']=='mode' and x['value']==0 for x in silence)
        for name,mode in [('pause',0),('resume',1),('rewind',4)]:
            phase='command_'+name
            m=String();m.data=name;command.publish(m);spin(1)
            result[name+'_mode_requested']=any(x['phase']==phase and x['kind']=='mode' and x['value']==mode for x in events)
        result['verdict']='PASS' if all(result.values()) else 'FAIL'
        result['healthy_max_publish_interval_wall_s']=max(healthy_intervals,default=None)
        result['scope']='real monitor detection and mode request to mock service; no flight or controller intervention qualification'
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        log.close();(out/'events.json').write_text(json.dumps(events,indent=2));(out/'summary.json').write_text(json.dumps(result,indent=2))
        n.destroy_node();rclpy.shutdown()
    if result.get('verdict')!='PASS':raise SystemExit(1)

if __name__=='__main__':main()
