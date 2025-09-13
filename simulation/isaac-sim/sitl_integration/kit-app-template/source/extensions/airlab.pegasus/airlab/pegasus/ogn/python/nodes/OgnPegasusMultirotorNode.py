import inspect
import os
import traceback
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import carb
import omni
import omni.client
import omni.graph.core as og
import omni.graph.tools.ogn as ogn
import omni.usd
import omni.replicator.core as rep
import omni.timeline
import usdrt.Sdf
from omni.isaac.core.prims import GeometryPrim, RigidPrim
from omni.isaac.core.utils import extensions, stage
from omni.isaac.core.world import World
from pxr import Gf, Usd, UsdGeom
import time
import threading
import struct
import socket
import asyncio

sys.path.append('/root/Documents/Kit/shared/exts/airlab.pegasus/airlab/pegasus/AirStackPegasusSimulator/extensions/pegasus.simulator')
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, BACKENDS, WORLD_SETTINGS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends import Backend, BackendConfig, PX4MavlinkBackend, PX4MavlinkBackendConfig, ArduPilotMavlinkBackend, ArduPilotMavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicle_manager import VehicleManager
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

from airlab.pegasus.ogn.OgnPegasusMultirotorNodeDatabase import OgnPegasusMultirotorNodeDatabase

# Global variables for drone simulation tracking
drone_sim_dict = {}
initialized_timeline_callback = False


def timeline_callback(event):
    """Timeline callback to handle simulation events"""
    global drone_sim_dict
    print("pegasus multirotor node timeline callback", event, event.type, dir(event))

    if event.type == int(omni.timeline.TimelineEventType.PLAY):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
        pass
    elif event.type == int(omni.timeline.TimelineEventType.STOP):
        # Reset orientation and position of the drone
        for k, v in drone_sim_dict.items():
            if 'multirotor' in v and hasattr(v['multirotor'], '_prim') and v['multirotor']._prim:
                prim = v['multirotor']._prim
                if prim.GetAttribute("xformOp:orient").Get() is not None:
                    prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(1, 0, 0, 0))
                prim.GetAttribute("xformOp:translate").Set((0, 0, 0))

        drone_sim_dict = {}


def incremental_rotate(prim):
    """Helper function to incrementally rotate a prim"""
    rot = Gf.Quatf(0.9702957272529602, 0, 0.2419208437204361, 0)  # 7 degrees
    o = prim.GetAttribute("xformOp:orient")
    if o.Get() != None:
        o.Set(o.Get() * rot)
    else:
        o = prim.GetAttribute("xformOp:rotateXYZ")
        if o.Get() != None:
            o.Set(o.Get() + Gf.Vec3d(0.0, 50, 0.0))


class OgnPegasusMultirotorNodeState:
    def __init__(self):
        self.node_initialized: bool = False  # Flag used to check if the per-instance node state is initialized.

class OgnPegasusMultirotorNode:
    @staticmethod
    def internal_state():
        return OgnPegasusMultirotorNodeState()

    @staticmethod
    def get_node_type():
        return "airlab.pegasus.PegasusMultirotorNode"

    @staticmethod
    def _is_initialized(node: og.Node) -> bool:
        return og.Controller.get(node.get_attribute("state:omni_initialized"))

    @staticmethod
    def _set_initialized(node: og.Node, init: bool):
        return og.Controller.set(node.get_attribute("state:omni_initialized"), init)

    @staticmethod
    def initialize(context, node: og.Node):
        """Initialize the node"""
        # Initialize state if shared_internal_state exists
        try:
            state = OgnPegasusMultirotorNodeDatabase.shared_internal_state(node)
            state.node_initialized = True
        except AttributeError:
            # If shared_internal_state doesn't exist, create a simple state
            pass

        OgnPegasusMultirotorNode._set_initialized(node, False)

    @staticmethod
    def release(node: og.Node):
        """Release node resources"""
        # Clean up drone simulation
        OgnPegasusMultirotorNode.try_cleanup(node)

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Overrides the release_instance method so that any per-script cleanup can happen before the per-node data
        is deleted.
        """
        # Same logic as when the reset button is pressed
        OgnPegasusMultirotorNode.try_cleanup(node)

    @staticmethod
    def try_cleanup(node: og.Node):
        """Clean up drone simulation resources"""
        global drone_sim_dict
        
        # Skip if not setup in the first place or already cleaned up
        if not OgnPegasusMultirotorNode._is_initialized(node):
            return

        node_id = node.node_id()
        
        # Clean up drone simulation if it exists
        if node_id in drone_sim_dict:
            try:
                drone_data = drone_sim_dict[node_id]
                if 'backend' in drone_data and drone_data['backend']:
                    # Clean up backend if it has cleanup methods
                    if hasattr(drone_data['backend'], 'cleanup'):
                        drone_data['backend'].cleanup()
                    
                if 'multirotor' in drone_data and drone_data['multirotor']:
                    # Clean up multirotor if it has cleanup methods
                    if hasattr(drone_data['multirotor'], 'cleanup'):
                        drone_data['multirotor'].cleanup()
                        
                del drone_sim_dict[node_id]
                print(f"Cleaned up drone simulation for node {node_id}")
                
            except Exception as e:
                print(f"Error during cleanup for node {node_id}: {e}")
                traceback.print_exc()
        
        OgnPegasusMultirotorNode._set_initialized(node, False)

    @staticmethod
    def _print_stacktrace(db: OgnPegasusMultirotorNodeDatabase):
        """Print stacktrace for debugging"""
        stacktrace = traceback.format_exc().splitlines(keepends=True)
        stacktrace_iter = iter(stacktrace)
        stacktrace_output = ""

        for stacktrace_line in stacktrace_iter:
            if "OgnPegasusMultirotorNode.py" in stacktrace_line:
                # The stack trace shows that the exception originates from this file
                # Removing this useless information from the stack trace
                next(stacktrace_iter, None)
            else:
                stacktrace_output += stacktrace_line

        if hasattr(db, 'log_error'):
            db.log_error(stacktrace_output)
        else:
            print("Error:", stacktrace_output)

    @staticmethod
    def compute(db) -> bool:
        """Main compute method that handles node spawning and execution"""
        global drone_sim_dict, initialized_timeline_callback
        
        # Initialize timeline callback if not already done
        if not initialized_timeline_callback:
            initialized_timeline_callback = True
            timeline = omni.timeline.get_timeline_interface()
            timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PLAY), timeline_callback
            )
            timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PAUSE), timeline_callback
            )
            timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP), timeline_callback
            )

        try:
            # Check if we have drone prim input
            if db.inputs.dronePrim:
                node_id = db.node.node_id()
                
                # Initialize drone if not already done
                if node_id not in drone_sim_dict.keys():
                    print('Creating new drone with VEHICLE ID:', db.inputs.vehicleID)
                    
                    # Create backend configuration
                    backend_config = PX4MavlinkBackendConfig({
                        "vehicle_id": db.inputs.vehicleID,
                        "px4_autolaunch": True,
                        "px4_dir": '/root/PX4-Autopilot',
                        "px4_vehicle_model": 'gazebo-classic_iris'
                    })
                    backend = PX4MavlinkBackend(config=backend_config)

                    # Create multirotor configuration
                    multirotor_config = MultirotorConfig()
                    multirotor_config.backends = [backend]
                    
                    # Create multirotor vehicle
                    selected_robot = 'Iris'
                    print('PASSED IN MODEL NAME:', db.inputs.dronePrim)
                    multirotor = Multirotor(
                        db.inputs.dronePrim,
                        ROBOTS[selected_robot],
                        0,
                        [0.0, 0.0, 0.07],
                        [0.0, 0.0, 0.0, 1.0],
                        config=multirotor_config,
                    )

                    # Store drone simulation data
                    drone_sim_dict[node_id] = {
                        'backend_config': backend_config,
                        'backend': backend,
                        'multirotor_config': multirotor_config,
                        'multirotor': multirotor,
                    }
                    
                    print(f"Successfully created drone simulation for node {node_id}")
                    
        except Exception as e:
            print('Error in compute method:', e)
            traceback.print_exc()
            return False

        # Set outputs:execOut if not hidden
        if db.node.get_attribute("outputs:execOut").get_metadata(ogn.MetadataKeys.HIDDEN) != "1":
            db.outputs.execOut = og.ExecutionAttributeState.ENABLED

        return True
