#!/usr/bin/env python3
"""Run the actual probe against isolated mock action servers, never a real UAV.
Verifies mandatory guard -> cancel -> landing -> observed disarm ordering.
"""
import argparse,json,os,subprocess,time
from pathlib import Path


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='89':raise SystemExit('requires isolated DDS domain 89')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.action import ActionServer,CancelResponse,GoalResponse
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import State,ExtendedState
    from std_msgs.msg import Bool
    from task_msgs.action import TakeoffTask,LandTask
    from airstack_msgs.srv import RobotCommand
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--probe',required=True);ap.add_argument('--unresponsive-cancel',action='store_true');ap.add_argument('--delayed-admission',action='store_true');args=ap.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=Node('mock_uav_guard_contract');group=ReentrantCallbackGroup();actions=[]
    data={'armed':False,'armed_at':None,'violate':False,'z':0.,'abort':False,'accept_pending':False}
    def event(kind):actions.append(dict(t=time.monotonic(),kind=kind))
    pubs={name:n.create_publisher(typ,'/robot_1/'+topic,10) for name,typ,topic in [
        ('odom',Odometry,'odometry_conversion/odometry'),('state',State,'interface/mavros/state'),
        ('landed',ExtendedState,'interface/mavros/extended_state'),('watchdog',Bool,'behavior/drone_safety_monitor/state_estimate_timed_out'),('actuation',Bool,'interface/actuation_ready')]}
    def publish():
        if data['armed'] and time.monotonic()-data['armed_at']>1:data['violate']=True
        od=Odometry();od.header.frame_id='map';od.header.stamp=n.get_clock().now().to_msg();od.pose.pose.orientation.w=1.
        od.pose.pose.position.x=3. if data['violate'] else 0.;od.pose.pose.position.z=data['z'];pubs['odom'].publish(od)
        st=State();st.header.stamp=od.header.stamp;st.connected=True;st.armed=data['armed'];st.mode='OFFBOARD' if st.armed else 'AUTO.LOITER';pubs['state'].publish(st)
        ls=ExtendedState();ls.landed_state=2 if st.armed else 1;pubs['landed'].publish(ls)
        pubs['watchdog'].publish(Bool(data=False));pubs['actuation'].publish(Bool(data=not data['accept_pending']))
    timer=n.create_timer(.04,publish,callback_group=group)
    def ascent(goal):
        event('takeoff')
        if args.delayed_admission:
            end=time.monotonic()+.8
            while time.monotonic()<end:
                if goal.is_cancel_requested:
                    data['accept_pending']=False;event('takeoff_canceled_before_arm');goal.canceled();return TakeoffTask.Result(success=False,message='canceled before arm')
                time.sleep(.01)
        data['armed']=True;data['armed_at']=time.monotonic();data['z']=1.
        end=time.monotonic()+15
        while time.monotonic()<end:
            if data['abort']:
                event('takeoff_aborted');goal.abort();return TakeoffTask.Result(success=False,message='mock autopilot containment')
            if goal.is_cancel_requested:

                event('takeoff_canceled');goal.canceled();return TakeoffTask.Result(success=False,message='test canceled')
            time.sleep(.02)
        goal.abort();return TakeoffTask.Result(success=False,message='test timeout')
    def descent(goal):
        event('land');data['armed']=False;data['z']=0.;goal.succeed();return LandTask.Result(success=True,message='mock landing')
    def admission(request):
        if args.delayed_admission:data['accept_pending']=True;time.sleep(.4)
        return GoalResponse.ACCEPT
    takeoff=ActionServer(n,TakeoffTask,'/robot_1/tasks/takeoff',ascent,goal_callback=admission,cancel_callback=lambda _:CancelResponse.REJECT if args.unresponsive_cancel else CancelResponse.ACCEPT,callback_group=group)
    land=ActionServer(n,LandTask,'/robot_1/tasks/land',descent,callback_group=group)
    def fallback(req,res):
        res.success=req.command==RobotCommand.Request.LAND
        if res.success:
            event('fallback_land');data['armed']=False;data['z']=0.;data['abort']=True
        return res
    fallback_service=n.create_service(RobotCommand,'/robot_1/interface/robot_command',fallback,callback_group=group)
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(n)
    log=(out/'probe.log').open('w');proc=subprocess.Popen(['python3',args.probe,'--output',str(out/'probe'),'--execute-simulation'],stdout=log,stderr=subprocess.STDOUT)
    end=time.monotonic()+40
    try:
        while proc.poll() is None and time.monotonic()<end:executor.spin_once(timeout_sec=.05)
        if proc.poll() is None:raise RuntimeError('probe integration deadline')
        summary=json.loads((out/'probe/summary.json').read_text());order=[x['kind'] for x in actions]
        passed=(proc.returncode!=0 and order==['takeoff','takeoff_canceled','land'] and
            'horizontal_radius' in summary['violations'] and summary['land_action'] and summary['final_disarmed'] and summary['final_on_ground'])
        if args.unresponsive_cancel:
            passed=(proc.returncode!=0 and order[:2]==['takeoff','fallback_land'] and 'land' not in order and
                    summary.get('fallback_land_requested') and summary.get('fallback_ground_disarm_observed') and summary['final_disarmed'])
        if args.delayed_admission:
            passed=(proc.returncode!=0 and order==['takeoff','takeoff_canceled_before_arm'] and summary.get('goal_submission_reconciled') is True and summary.get('pending_goal_unreconciled') is False and summary['final_disarmed'])
        result=dict(verdict='PASS' if passed else 'FAIL',scope='isolated mock transport/action contract, no physical flight',action_order=order,probe_exit=proc.returncode)
        (out/'summary.json').write_text(json.dumps(result,indent=2));(out/'actions.json').write_text(json.dumps(actions,indent=2))
    finally:
        if proc.poll() is None:proc.terminate();proc.wait(timeout=5)
        log.close();executor.shutdown();n.destroy_node();rclpy.shutdown()
    if not passed:raise SystemExit(1)

if __name__=='__main__':main()
