# Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers import ParallelGripper

from .rmpflow import RMPFlowController


class PickPlaceController(manipulators_controllers.PickPlaceController):
    def __init__(
        self, name: str, gripper: ParallelGripper, robot_articulation: SingleArticulation, events_dt=None
    ) -> None:
        if events_dt is None:
            events_dt = [0.005, 0.002, 1, 0.05, 0.0008, 0.005, 0.0008, 0.1, 0.0008, 0.008]
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation
            ),
            gripper=gripper,
            events_dt=events_dt,
            end_effector_initial_height=0.6,
        )
        return
