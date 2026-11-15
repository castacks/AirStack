#!/usr/bin/env python3
"""Read-only ROS stamp/receipt observer for the live UAV control chain."""
import argparse,json,time,statistics
from pathlib import Path


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry
    from airstack_msgs.msg import Odometry as Reference
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',required=True);ap.add_argument('--duration',type=float,default=30)
    args=ap.parse_args();out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=Node('uav_readonly_timing_probe');clock=None;rows=[]
    def on_clock(m):
        nonlocal clock
        clock=(m.clock.sec+m.clock.nanosec*1e-9,time.monotonic())
    cs=n.create_subscription(Clock,'/clock',on_clock,qos_profile_sensor_data)
    def callback(name,m):
        t=time.monotonic();stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        rows.append(dict(topic=name,wall_monotonic_s=t,source_stamp_s=stamp,
            latest_clock_s=clock[0] if clock else None,clock_receipt_age_s=t-clock[1] if clock else None,
            source_age_to_latest_clock_s=clock[0]-stamp if clock else None,frame=m.header.frame_id))
    subscriptions=[]
    for name,typ in [('odometry_conversion/odometry',Odometry),('trajectory_controller/look_ahead',Reference),('trajectory_controller/tracking_point',Reference)]:
        subscriptions.append(n.create_subscription(typ,'/robot_1/'+name,lambda m,k=name:callback(k,m),qos_profile_sensor_data))
    end=time.monotonic()+args.duration
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
    (out/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows));summary={}
    for topic in sorted({r['topic'] for r in rows}):
        selected=[r for r in rows if r['topic']==topic];ages=[r['source_age_to_latest_clock_s'] for r in selected if r['source_age_to_latest_clock_s'] is not None]
        receipt=[r['wall_monotonic_s'] for r in selected]
        summary[topic]=dict(samples=len(selected),min_source_age_s=min(ages) if ages else None,
            max_source_age_s=max(ages) if ages else None,median_source_age_s=statistics.median(ages) if ages else None,
            source_age_over_500ms_samples=sum(a>.5 for a in ages),
            max_receipt_gap_s=max((b-a for a,b in zip(receipt,receipt[1:])),default=None))
    (out/'summary.json').write_text(json.dumps(dict(scope='latest received /clock, not sensor acquisition or synchronized callback instrumentation',topics=summary),indent=2))
    n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
