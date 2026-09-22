"""Use current PhysX pose for Pegasus state, independent of rendering frequency.

The installed Vehicle.update_state reads position from dynamic control but attitude
from USD. With 200 Hz physics / 20 Hz rendering, USD attitude is stale for nine of
ten physics steps. Sensor gravity compensation must use the same physical instant.
Adapted from the validated Novare capture; applied only by this Office launcher.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def update_physics_state(self, dt):
    dc=self.get_dc_interface()
    body=dc.get_rigid_body(self._stage_prefix+'/body')
    pose=dc.get_rigid_body_pose(body)
    rotation=Rotation.from_quat(list(pose.r))*self._body_local_rotation_inv
    velocity=np.asarray(dc.get_rigid_body_linear_velocity(body))
    angular_world=np.asarray(dc.get_rigid_body_angular_velocity(body))
    self._state.linear_acceleration=(velocity-self._state.linear_velocity)/dt
    self._state.position=np.asarray(pose.p)
    self._state.attitude=rotation.as_quat()
    self._state.linear_velocity=velocity
    self._state.linear_body_velocity=rotation.inv().apply(velocity)
    self._state.angular_velocity=rotation.inv().apply(angular_world)


def install():
    from pegasus.simulator.logic.vehicles.vehicle import Vehicle
    Vehicle.update_state=update_physics_state
