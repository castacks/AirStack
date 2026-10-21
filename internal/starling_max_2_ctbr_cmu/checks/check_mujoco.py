import os
import json
from pathlib import Path
import sys
from collections import deque
import mujoco
import numpy as np
from gamma_zero_controller.inference import StarlingPolicy
from gamma_zero_controller.control import Controller

root=Path(sys.argv[1])
policy=StarlingPolicy(root/'starling_cmu_inputs.json',Path(os.environ['STARLING_BUNDLE']))
results=[]
for pattern in (sys.argv[2:] or ['flown','large']):
    for delay in [0.,.1]:
        config=json.loads((Path(__file__).parents[3]/'robot/ros_ws/src/gamma_zero_controller/config/simulation.json').read_text())
        config['trajectory']['pattern']=pattern
        model=mujoco.MjModel.from_xml_path(str(Path(os.environ['STARLING_MODEL_XML'])))
        data=mujoco.MjData(model);data.qpos[:]=[0.,0.,1.3,1.,0.,0.,0.]
        mujoco.mj_forward(model,data)
        c=Controller(policy,config)
        policy.reset_history()
        def state(t):
            rotation=data.xmat[1].reshape(3,3).copy()
            c.session.state={'position_enu':data.qpos[:3].copy(),'rotation_enu_flu':rotation,
                'linear_velocity_flu':data.sensor('imu_linear_velocity').data.copy(),
                'angular_velocity_flu':data.sensor('imu_angular_velocity').data.copy(),
                'projected_gravity_flu':rotation.T@np.array([0.,0.,-1.])}
            c.session.source_s=t;c.session.received_s=t
        state(0.);c.stable_since=-2.;c.engage(0.,0.,True);c.start_trajectory(0.,0.)
        queued=deque([np.array([.602*9.81,0.,0.,0.]) for _ in range(round(delay/.02))])
        thrust=.602*9.81;errors=[];yaw_errors=[];failed='';duration=c.session.trajectory.duration_s+2.
        for i in range(round(duration/.02)):
            t=i*.02;state(t)
            try:
                c.command(None,t,t)
            except (ValueError,RuntimeError) as exc:
                failed=str(exc);break
            cmd=np.asarray(c.last_record['ctbr_si'])
            queued.append(cmd); applied=queued.popleft()
            for _ in range(4):
                thrust+=(applied[0]-thrust)*(1-np.exp(-.005/.03))
                data.ctrl[:]=applied;data.ctrl[0]=thrust
                mujoco.mj_step(model,data)
            if t>=config['trajectory'].get('settle_s',10):
                desired_yaw=c.last_record['reference_yaw']
                actual_yaw=np.arctan2(c.session.state['rotation_enu_flu'][1,0],c.session.state['rotation_enu_flu'][0,0])
                yaw_errors.append(float(np.arctan2(np.sin(desired_yaw-actual_yaw),np.cos(desired_yaw-actual_yaw))))
                errors.append(float(np.linalg.norm(np.asarray(c.last_record['position_enu'])-c.last_record['reference_enu'])))
        results.append(dict(pattern=pattern,delay_s=delay,duration_s=duration,completed=not failed,
                            failure=failed,rms_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                            max_m=max(errors) if errors else None, yaw_rms_rad=float(np.sqrt(np.mean(np.square(yaw_errors)))) if yaw_errors else None))
        print(results[-1],flush=True)
(root/'mujoco-results.json').write_text(json.dumps(results,indent=2))
assert all(r['completed'] for r in results),results
