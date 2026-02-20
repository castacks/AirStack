# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api.objects import VisualSphere
from isaacsim.cortex.framework.cortex_world import CortexWorld
from isaacsim.cortex.framework.df import DfNetwork, DfState, DfStateMachineDecider
from isaacsim.cortex.framework.dfb import DfRobotApiContext
from isaacsim.cortex.framework.robot import add_franka_to_stage


class FollowState(DfState):
    """The context object is available as self.context. We have access to everything in the context
    object, which in this case is everything in the robot object (the command API and the follow
    sphere).
    """

    @property
    def robot(self):
        return self.context.robot

    @property
    def follow_sphere(self):
        return self.context.robot.follow_sphere

    def enter(self):
        self.follow_sphere.set_world_pose(*self.robot.arm.get_fk_pq().as_tuple())

    def step(self):
        target_position, _ = self.follow_sphere.get_world_pose()
        target_position[2] = max(target_position[2], 0.02)
        self.robot.arm.send_end_effector(target_position=target_position)
        return self  # Always transition back to this state.


class FollowContext(DfRobotApiContext):
    def __init__(self, robot):
        super().__init__(robot)
        self.reset()

        self.add_monitors(
            [FollowContext.monitor_end_effector, FollowContext.monitor_gripper, FollowContext.monitor_diagnostics]
        )

    def reset(self):
        self.is_target_reached = False

    def monitor_end_effector(self):
        eff_p = self.robot.arm.get_fk_p()
        target_p, _ = self.robot.follow_sphere.get_world_pose()
        self.is_target_reached = np.linalg.norm(target_p - eff_p) < 0.01

    def monitor_gripper(self):
        if self.is_target_reached:
            self.robot.gripper.close()
        else:
            self.robot.gripper.open()

    def monitor_diagnostics(self):
        print("is_target_reached: {}".format(self.is_target_reached))


def main():
    world = CortexWorld()
    robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

    # Add a sphere to the scene to follow, and store it off in a new member as part of the robot.
    robot.follow_sphere = world.scene.add(
        VisualSphere(
            name="follow_sphere", prim_path="/World/FollowSphere", radius=0.02, color=np.array([0.7, 0.0, 0.7])
        )
    )
    world.scene.add_default_ground_plane()

    # Add a simple state machine decider network with the single state defined above. This state
    # will be persistently stepped because it always returns itself.
    world.add_decider_network(DfNetwork(DfStateMachineDecider(FollowState()), context=FollowContext(robot)))

    world.run(simulation_app)
    simulation_app.close()


if __name__ == "__main__":
    main()
