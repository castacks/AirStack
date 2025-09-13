"""
PX4-specific Pegasus Multirotor OmniGraph node.
Extends the base multirotor node with PX4-specific backend configuration.
"""

import sys
import traceback
import omni.graph.core as og
import omni.graph.tools.ogn as ogn

sys.path.append('/root/Documents/Kit/shared/exts/airlab.pegasus/airlab/pegasus/AirStackPegasusSimulator/extensions/pegasus.simulator')
from pegasus.simulator.logic.backends import PX4MavlinkBackend, PX4MavlinkBackendConfig

from airlab.pegasus.ogn.OgnPegasusPX4MultirotorNodeDatabase import OgnPegasusPX4MultirotorNodeDatabase
from airlab.pegasus.ogn.python.nodes.OgnPegasusMultirotorBaseNode import OgnPegasusMultirotorBaseNode


class OgnPegasusPX4MultirotorNodeState:
    def __init__(self):
        self.node_initialized: bool = False


class OgnPegasusPX4MultirotorNode:
    """PX4-specific Pegasus Multirotor OmniGraph node"""
    
    @staticmethod
    def internal_state():
        return OgnPegasusPX4MultirotorNodeState()

    @staticmethod
    def get_node_type():
        return "airlab.pegasus.PegasusPX4MultirotorNode"

    @staticmethod
    def initialize(context, node: og.Node):
        """Initialize the PX4 node"""
        return OgnPegasusMultirotorBaseNode.initialize(context, node, OgnPegasusPX4MultirotorNodeDatabase)

    @staticmethod
    def release(node: og.Node):
        """Release node resources"""
        return OgnPegasusMultirotorBaseNode.release(node)

    @staticmethod
    def release_instance(node, graph_instance_id):
        """Release instance resources"""
        return OgnPegasusMultirotorBaseNode.release_instance(node, graph_instance_id)

    @staticmethod
    def create_px4_backend_config(db):
        """Create PX4-specific backend configuration with explicit parameter mapping"""
        # Explicitly map each OmniGraph input to backend configuration parameter
        config_dict = {
            # Vehicle identification
            "vehicle_id": db.inputs.vehicleID,
            
            # MAVLink connection configuration
            "connection_type": db.inputs.connectionType,        # e.g., "tcpin", "udp"
            "connection_ip": db.inputs.connectionIP,            # e.g., "localhost"
            "connection_baseport": db.inputs.connectionBaseport, # e.g., 4560
            
            # PX4-specific autolaunch settings
            "px4_autolaunch": db.inputs.px4Autolaunch,          # True/False - launch PX4 automatically
            "px4_dir": db.inputs.px4Dir,                        # Path to PX4 installation
            "px4_vehicle_model": db.inputs.px4VehicleModel,     # PX4 vehicle model name
            
            # Simulation configuration
            "enable_lockstep": db.inputs.enableLockstep,        # True/False - synchronize simulation
            "num_rotors": db.inputs.numRotors,                  # Number of rotors (typically 4)
            "update_rate": db.inputs.updateRate,                # Backend update frequency in Hz
            
            # Per-rotor input configuration arrays
            "input_offset": [
                db.inputs.inputOffset0,                         # Rotor 0 input offset
                db.inputs.inputOffset1,                         # Rotor 1 input offset
                db.inputs.inputOffset2,                         # Rotor 2 input offset
                db.inputs.inputOffset3                          # Rotor 3 input offset
            ],
            "input_scaling": [
                db.inputs.inputScaling0,                        # Rotor 0 input scaling factor
                db.inputs.inputScaling1,                        # Rotor 1 input scaling factor
                db.inputs.inputScaling2,                        # Rotor 2 input scaling factor
                db.inputs.inputScaling3                         # Rotor 3 input scaling factor
            ],
            "zero_position_armed": [
                db.inputs.zeroPositionArmed0,                   # Rotor 0 zero position when armed
                db.inputs.zeroPositionArmed1,                   # Rotor 1 zero position when armed
                db.inputs.zeroPositionArmed2,                   # Rotor 2 zero position when armed
                db.inputs.zeroPositionArmed3                    # Rotor 3 zero position when armed
            ]
        }
        
        print(f"PX4 Backend Configuration:")
        print(f"  Vehicle ID: {config_dict['vehicle_id']}")
        print(f"  Connection: {config_dict['connection_type']}://{config_dict['connection_ip']}:{config_dict['connection_baseport']}")
        print(f"  PX4 Autolaunch: {config_dict['px4_autolaunch']}")
        print(f"  PX4 Directory: {config_dict['px4_dir']}")
        print(f"  PX4 Vehicle Model: {config_dict['px4_vehicle_model']}")
        print(f"  Lockstep Enabled: {config_dict['enable_lockstep']}")
        print(f"  Number of Rotors: {config_dict['num_rotors']}")
        print(f"  Update Rate: {config_dict['update_rate']} Hz")
        
        return PX4MavlinkBackendConfig(config_dict)

    @staticmethod
    def compute(db) -> bool:
        """Main compute method that handles PX4 drone spawning and execution"""
        try:
            # Use base compute functionality with PX4-specific backend
            return OgnPegasusMultirotorBaseNode.compute_base(
                db, 
                PX4MavlinkBackend, 
                PX4MavlinkBackendConfig,
                OgnPegasusPX4MultirotorNode.create_px4_backend_config
            )
                    
        except Exception as e:
            print('Error in PX4 compute method:', e)
            traceback.print_exc()
            OgnPegasusMultirotorBaseNode._print_stacktrace(db)
            return False
