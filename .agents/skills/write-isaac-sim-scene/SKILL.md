---
name: write-isaac-sim-scene
description: Create custom simulation environments in Isaac Sim using standalone Python scripts with Pegasus extension. Use when creating test scenarios, multi-robot simulations, or custom environments for testing autonomy modules.
license: Apache-2.0
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Write Isaac Sim Scene in Standalone Python Mode

## When to Use

Creating custom simulation environments for testing autonomy modules, multi-robot scenarios, or specific environmental conditions.

## Prerequisites

- Isaac Sim container running or accessible
- Understanding of Pegasus Simulator extension for drones
- Knowledge of required sensors and vehicle configuration
- Familiarity with Python and basic Isaac Sim concepts

## Isaac Sim Integration Overview

AirStack uses NVIDIA Isaac Sim with the **Pegasus Simulator extension** for high-fidelity drone simulation. There are two ways to define scenes:

1. **USD Files:** Static scene description files (`.usd` format)
2. **Standalone Python Scripts:** Dynamic scene creation with full programmatic control (recommended for complex scenarios)

This skill covers **standalone Python mode**.

## Script Structure Overview

Standalone Python scripts follow this pattern:

```
1. Start SimulationApp (BEFORE any omni imports)
2. Import required modules
3. Enable necessary extensions
4. Create PegasusApp class
   - Initialize Pegasus interface
   - Load environment
   - Spawn vehicles with sensors
   - Setup physics and backends
5. Run simulation loop
6. Clean up
```

## Steps

### 1. Create Script File

**Location:** `simulation/isaac-sim/launch_scripts/<your_scene_name>.py`

```bash
cd simulation/isaac-sim/launch_scripts/
touch your_scene_name.py
chmod +x your_scene_name.py
```

### 2. Script Header and SimulationApp Initialization

**Critical:** SimulationApp MUST be started before importing any `omni` modules.

```python
#!/usr/bin/env python
"""
Description: Brief description of your simulation scene
Author: Your Name
Date: YYYY-MM-DD

This script creates a simulation environment for testing <specific feature>.
- Number of drones: X
- Sensors: Camera, LiDAR, etc.
- Environment: Description
"""

import carb
from isaacsim import SimulationApp

# MUST start SimulationApp before importing omni modules
# Set headless=False for GUI, headless=True for automated testing
simulation_app = SimulationApp({"headless": False})

# Now safe to import omni and other modules
import rclpy
print(f"[Launcher] SUCCESS: rclpy imported from {rclpy.__file__}")
```

### 3. Import Required Modules

```python
import omni.kit.app
import omni.timeline
import omni.ui
from omni.isaac.core.world import World
from datetime import datetime
from pxr import UsdLux, Gf, UsdGeom

# Pegasus imports
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS, ROBOTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.ogn.api.spawn_multirotor import spawn_px4_multirotor_node
from pegasus.simulator.ogn.api.spawn_zed_camera import add_zed_stereo_camera_subgraph
from pegasus.simulator.ogn.api.spawn_ouster_lidar import add_ouster_lidar_subgraph
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig
)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from scipy.spatial.transform import Rotation
import numpy as np
import os
import subprocess
import threading
import signal
import atexit
import time

# Scene preparation utilities (scaling, collision, lighting, export)
# NOTE: importlib is used instead of a normal import because Isaac Sim's
# script runner does not reliably set __file__, making sys.path manipulation
# fragile. Loading the module by absolute file path is the robust approach.
import importlib.util as _ilu, os as _os
_scene_prep_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "utils", "scene_prep.py")
_spec = _ilu.spec_from_file_location("scene_prep", _os.path.normpath(_scene_prep_path))
_scene_prep = _ilu.module_from_spec(_spec); _spec.loader.exec_module(_scene_prep)
scale_stage_prim            = _scene_prep.scale_stage_prim
add_colliders               = _scene_prep.add_colliders
add_dome_light              = _scene_prep.add_dome_light
save_scene_as_contained_usd = _scene_prep.save_scene_as_contained_usd
```

### 4. Enable Required Extensions

```python
# Explicitly enable required extensions
ext_manager = omni.kit.app.get_app().get_extension_manager()

# Required extensions for Pegasus and OmniGraph
required_extensions = [
    "omni.graph.core",                  # Core runtime for OmniGraph engine
    "omni.graph.action",                # Action Graph framework
    "omni.graph.action_nodes",          # Built-in Action Graph node library
    "isaacsim.core.nodes",              # Core helper nodes for OmniGraph
    "omni.graph.ui",                    # UI scaffolding for graph tools
    "omni.graph.visualization.nodes",   # Visualization helper nodes
    "omni.graph.scriptnode",            # Python script node support
    "omni.graph.window.action",         # Action Graph editor window
    "omni.graph.window.generic",        # Generic graph UI tools
    "omni.graph.ui_nodes",              # UI node building helpers
    "pegasus.simulator",                # Pegasus Simulator extension
]

for ext in required_extensions:
    if not ext_manager.is_extension_enabled(ext):
        print(f"[Launcher] Enabling extension: {ext}")
        ext_manager.set_extension_enabled_immediate(ext, True)
        print(f"[Launcher] Successfully enabled extension: {ext}")        
    else:
        print(f"[Launcher] Extension already enabled: {ext}")
```

### 5. Create PegasusApp Class

```python
class YourSceneApp:
    """
    Simulation application for your specific scenario.
    """

    def __init__(self):
        print("[YourScene] Initializing simulation...")
        
        # Start Pegasus interface
        self.pg = PegasusInterface()
        
        # Create Isaac Sim world
        self.world = World(**self.pg.world_settings)
        self.pg.world = self.world
        
        # Dictionary to store vehicle instances
        self.vehicles = {}
        
        # PX4 process handles (if using PX4 SITL)
        self.px4_processes = []
        
        # Load environment
        self.load_environment()

        # Prepare environment (scale, colliders, lighting)
        stage = omni.usd.get_context().get_stage()
        self._prepare_environment(stage)

        # Spawn vehicles
        self.spawn_vehicles()
        
        # Setup simulation
        self.world.reset()
        
        print("[YourScene] Simulation initialized successfully")
    
    def load_environment(self):
        """Load or create the simulation environment."""
        print("[YourScene] Loading environment...")
        
        # Option 1: Load pre-defined environment
        # Available: "Grid", "Outdoor", "Office", etc.
        # See SIMULATION_ENVIRONMENTS in Pegasus for options
        stage = self.pg.load_environment(SIMULATION_ENVIRONMENTS["Grid"]["usd"])
        
        # Option 2: Add ground plane only
        # self.world.scene.add_default_ground_plane()
        
        # Option 3: Load custom USD environment
        # stage = self.pg.load_environment("/path/to/your/environment.usd")
        
        # Add obstacles or other static objects
        self._add_environment_objects()

    def _prepare_environment(self, stage):
        """Scale, add collisions, and light the environment."""
        stage_prim = stage.GetPrimAtPath("/World/stage")
        if stage_prim.IsValid():
            # STAGE_SCALE: use 0.01 for Nucleus assets authored in cm, 1.0 if already in meters
            scale_stage_prim(stage, "/World/stage", STAGE_SCALE)
            add_colliders(stage_prim)
            # Allow physics to settle after adding colliders
            for _ in range(10):
                omni.kit.app.get_app().update()
        # add_dome_light defaults: intensity=3500, exposure=-3
        # Override via kwargs, e.g. add_dome_light(stage, intensity=5000, exposure=-2)
        add_dome_light(stage)
    
    def _add_environment_objects(self):
        """Add obstacles or other objects to the environment."""
        # Example: Add a cube obstacle
        stage = omni.usd.get_context().get_stage()
        
        # cube_prim = stage.DefinePrim("/World/Obstacle1", "Cube")
        # UsdGeom.Xformable(cube_prim).AddTranslateOp().Set(Gf.Vec3d(5.0, 0.0, 0.5))
        # UsdGeom.Xformable(cube_prim).AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
        
        pass
    
    def spawn_vehicles(self):
        """Spawn drone vehicles with sensors and backends."""
        print("[YourScene] Spawning vehicles...")
        
        # Vehicle 1: Primary drone
        self._spawn_vehicle(
            vehicle_id=0,
            vehicle_name="drone1",
            position=[0.0, 0.0, 1.0],  # [x, y, z]
            orientation=[0.0, 0.0, 0.0, 1.0],  # quaternion [x, y, z, w]
            px4_autostart_id=4001,  # PX4 vehicle type (4001 = quadrotor)
            mavlink_tcp_port=4560,  # PX4 MAVLink port
            px4_instance=0,
            sensors={
                "camera": True,
                "lidar": False
            }
        )
        
        # Vehicle 2: Second drone (optional, for multi-robot)
        # self._spawn_vehicle(
        #     vehicle_id=1,
        #     vehicle_name="drone2",
        #     position=[5.0, 0.0, 1.0],
        #     orientation=[0.0, 0.0, 0.0, 1.0],
        #     px4_autostart_id=4001,
        #     mavlink_tcp_port=4561,
        #     px4_instance=1,
        #     sensors={"camera": True, "lidar": True}
        # )
    
    def _spawn_vehicle(self, vehicle_id, vehicle_name, position, orientation,
                       px4_autostart_id, mavlink_tcp_port, px4_instance,
                       sensors=None):
        """
        Spawn a single vehicle with specified configuration.
        
        Args:
            vehicle_id: Unique vehicle ID
            vehicle_name: Name for the vehicle
            position: [x, y, z] spawn position
            orientation: [x, y, z, w] quaternion orientation
            px4_autostart_id: PX4 vehicle type ID
            mavlink_tcp_port: MAVLink TCP port for PX4 communication
            px4_instance: PX4 instance number
            sensors: Dict of sensors to add {"camera": bool, "lidar": bool}
        """
        if sensors is None:
            sensors = {"camera": True, "lidar": False}
        
        # Configure multirotor
        config = MultirotorConfig()
        
        # PX4 MAVLink backend configuration
        px4_backend_config = PX4MavlinkBackendConfig({
            "vehicle_id": vehicle_id,
            "px4_autostart": px4_autostart_id,
            "px4_dir": os.environ.get("PX4_DIR", "/PX4-Autopilot"),
            "px4_instance": px4_instance,
            "mavlink_tcp_port": mavlink_tcp_port,
            "enable_lockstep": True,
            "update_rate": 250.0  # Hz
        })
        
        # Add ROS 2 backend for ROS communication
        ros2_backend = ROS2Backend(
            vehicle_id=vehicle_id,
            config={
                "namespace": vehicle_name,
                "pub_sensors": True,
                "pub_state": True
            }
        )
        
        # Attach backends
        config.backends = [
            PX4MavlinkBackend(px4_backend_config),
            ros2_backend
        ]
        
        # Create vehicle
        vehicle = Multirotor(
            stage_prefix="/World",
            prim_path=f"/World/{vehicle_name}",
            name=vehicle_name,
            usd_model=ROBOTS["Iris"]["usd"],  # or other model
            init_pos=position,
            init_orientation=orientation,
            config=config
        )
        
        # Add sensors
        if sensors.get("camera", False):
            self._add_camera_sensor(vehicle)
        
        if sensors.get("lidar", False):
            self._add_lidar_sensor(vehicle)
        
        # Initialize vehicle in world
        self.world.scene.add(vehicle)
        self.vehicles[vehicle_name] = vehicle
        
        print(f"[YourScene] Spawned vehicle: {vehicle_name}")
    
    def _add_camera_sensor(self, vehicle):
        """Add stereo camera to vehicle."""
        add_zed_stereo_camera_subgraph(
            camera_prim_path=vehicle.prim_path + "/ZedCamera",
            parent_prim_path=vehicle.prim_path,
            config={
                "graph_evaluator": "execution",  # or "push"
                "resolution": (1280, 720),
                "position": (0.3, 0.0, -0.1),  # Relative to vehicle
                "orientation": (0.0, 0.0, 0.0, 1.0),
            }
        )
    
    def _add_lidar_sensor(self, vehicle):
        """Add LiDAR sensor to vehicle."""
        add_ouster_lidar_subgraph(
            lidar_prim_path=vehicle.prim_path + "/OusterLidar",
            parent_prim_path=vehicle.prim_path,
            config={
                "graph_evaluator": "execution",
                "position": (0.0, 0.0, -0.15),  # Relative to vehicle
                "orientation": (0.0, 0.0, 0.0, 1.0),
            }
        )
    
    def run(self):
        """Main simulation loop."""
        print("[YourScene] Starting simulation loop...")
        
        # Optionally auto-start timeline
        # omni.timeline.get_timeline_interface().play()
        
        step_count = 0
        while simulation_app.is_running():
            # Step the simulation
            self.world.step(render=True)
            
            # Optional: Add periodic logic
            if step_count % 100 == 0:
                # print(f"[YourScene] Simulation step: {step_count}")
                pass
            
            step_count += 1
        
        print("[YourScene] Simulation loop ended")
    
    def cleanup(self):
        """Clean up resources."""
        print("[YourScene] Cleaning up...")
        
        # Stop PX4 processes
        for process in self.px4_processes:
            if process.poll() is None:  # Process still running
                process.terminate()
                process.wait()
        
        self.px4_processes.clear()
```

### 6. Main Entry Point

```python
def main():
    """Main entry point for the simulation."""
    try:
        # Create and run simulation
        app = YourSceneApp()
        app.run()
    except Exception as e:
        print(f"[YourScene] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up
        if 'app' in locals():
            app.cleanup()
        simulation_app.close()

if __name__ == "__main__":
    main()
```

### 7. Configure in .env

Update the main `.env` file to use your script:

```bash
# Set to standalone script mode
ISAAC_SIM_USE_STANDALONE="true"

# Specify your script name
ISAAC_SIM_SCRIPT_NAME="your_scene_name.py"
```

Alternatively, override from command line:
```bash
ISAAC_SIM_USE_STANDALONE=true ISAAC_SIM_SCRIPT_NAME="your_scene_name.py" airstack up isaac-sim
```

### 8. Test the Scene

Launch Isaac Sim with your script:

```bash
# Start Isaac Sim container with your scene
airstack up isaac-sim

# Check logs for errors
airstack logs isaac-sim

# If errors occur, connect to container for debugging
airstack connect isaac-sim
```

### 9. Document the Scene

Create a README.md next to your script:

**File:** `simulation/isaac-sim/launch_scripts/your_scene_name.md`

```markdown
# Your Scene Name

## Overview
Brief description of the simulation scene.

## Purpose
Why this scene was created and what it tests.

## Configuration

### Vehicles
- Number of drones: X
- Vehicle types: Quadrotor, fixed-wing, etc.
- Initial positions: List positions

### Sensors
- Cameras: Resolution, FoV
- LiDAR: Model, range
- Other sensors

### Environment
Description of the environment, obstacles, lighting.

## Usage

```bash
# Launch scene
ISAAC_SIM_SCRIPT_NAME="your_scene_name.py" airstack up isaac-sim

# With robot autonomy
airstack up isaac-sim robot
```

## Parameters
Any configurable parameters in the script.

## Known Issues
Any limitations or known problems.
```

## Advanced Topics

### Scene Preparation Utilities

**File:** `simulation/isaac-sim/utils/scene_prep.py`

Four reusable helpers that cover the most common environment setup tasks. Import them as shown in Step 3.

| Function | When to use |
|----------|-------------|
| `scale_stage_prim(stage, prim_path, scale)` | Nucleus assets authored in centimetres need `STAGE_SCALE=0.01`; assets already in metres use `1.0`. |
| `add_colliders(stage_prim)` | **Must** be called for physics to interact with environment meshes. Without it drones fall through the floor. Call after scaling. |
| `add_dome_light(stage, **kwargs)` | Adds uniform hemisphere lighting. Defaults: `intensity=3500`, `exposure=-3`. Pass kwargs to override, e.g. `add_dome_light(stage, intensity=5000)`. |
| `save_scene_as_contained_usd(src_url, output_dir)` | Copies a Nucleus-hosted stage (and all its textures/MDLs) to a local directory using `omni.kit.usd.collect.Collector`. Useful for archiving or offline replay. |

**Two-step save pattern** used internally by `save_scene_as_contained_usd`:
1. `export_as_stage_async` — writes a flat `.usd` of the live stage
2. `Collector` — resolves and copies all referenced Nucleus assets locally

Set `SAVE_SCENE_TO = None` in your script to skip saving entirely.

---

### Multi-Robot Scenarios

For multiple robots, spawn additional vehicles with unique IDs and ports:

```python
def spawn_vehicles(self):
    for i in range(num_robots):
        self._spawn_vehicle(
            vehicle_id=i,
            vehicle_name=f"drone{i}",
            position=[i * 5.0, 0.0, 1.0],  # Space them out
            orientation=[0.0, 0.0, 0.0, 1.0],
            px4_autostart_id=4001,
            mavlink_tcp_port=4560 + i,  # Unique port per vehicle
            px4_instance=i,
            sensors={"camera": True, "lidar": False}
        )
```

### Custom Sensor Configuration

Create custom sensor configurations:

```python
def _add_custom_camera(self, vehicle, config):
    """Add camera with custom parameters."""
    add_zed_stereo_camera_subgraph(
        camera_prim_path=vehicle.prim_path + "/CustomCamera",
        parent_prim_path=vehicle.prim_path,
        config={
            "resolution": config.get("resolution", (1920, 1080)),
            "horizontal_fov": config.get("fov", 90.0),
            "position": config.get("position", (0.3, 0.0, 0.0)),
            "orientation": config.get("orientation", (0.0, 0.0, 0.0, 1.0)),
        }
    )
```

### Dynamic Obstacles

Add moving obstacles:

```python
def _add_dynamic_obstacle(self):
    """Add a moving obstacle to the scene."""
    from omni.isaac.core.objects import DynamicCuboid
    
    obstacle = DynamicCuboid(
        prim_path="/World/DynamicObstacle",
        position=[10.0, 0.0, 1.0],
        scale=[1.0, 1.0, 1.0],
        color=[1.0, 0.0, 0.0]  # Red
    )
    self.world.scene.add(obstacle)
    
    # In simulation loop, update position
    # obstacle.set_world_pose(position=[x, y, z])
```

## Common Pitfalls

### SimulationApp Import Order
- ❌ **Importing omni modules before SimulationApp**
  - ✅ ALWAYS create SimulationApp first, then import omni modules

### Extension Loading
- ❌ **Missing required extensions**
  - ✅ Enable all required extensions before using their features
  - ✅ Check extension status with `ext_manager.is_extension_enabled()`

### PX4 Port Conflicts
- ❌ **Using same MAVLink port for multiple vehicles**
  - ✅ Each vehicle needs unique mavlink_tcp_port
  - ✅ Increment port number for each vehicle: 4560, 4561, 4562, ...

### Sensor Configuration
- ❌ **Incorrect sensor placement (inside vehicle mesh)**
  - ✅ Position sensors outside vehicle collision geometry
  - ✅ Typical camera position: forward of vehicle center

### Missing Colliders on Environment Meshes
- ❌ Loading a Nucleus environment without calling `add_colliders()`
  - ✅ Call `add_colliders(stage_prim)` after scaling — drones will fall through the floor otherwise

### World Reset
- ❌ **Not calling world.reset()**
  - ✅ Call world.reset() after adding all objects before stepping

## Debugging

### View Scene in GUI

Run with headless=False to see the scene:
```python
simulation_app = SimulationApp({"headless": False})
```

### Print Vehicle Info

```python
def run(self):
    while simulation_app.is_running():
        self.world.step(render=True)
        
        # Print vehicle state
        for name, vehicle in self.vehicles.items():
            pos, ori = vehicle.get_world_pose()
            print(f"{name}: pos={pos}, ori={ori}")
```

### Check ROS 2 Topics

```bash
# From another terminal, check topics are publishing
docker exec airstack-isaac-sim-1 bash -c "ros2 topic list"
docker exec airstack-isaac-sim-1 bash -c "ros2 topic hz /drone1/sensors/camera/image"
```

## References

- **Pegasus Simulator:**
  - [Pegasus GitHub](https://github.com/PegasusSimulator/PegasusSimulator)
  - [Pegasus Documentation](https://pegasussimulator.github.io/PegasusSimulator/)

- **Isaac Sim:**
  - [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
  - [USD Introduction](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html)

- **AirStack Examples:**
  - Single drone: `simulation/isaac-sim/launch_scripts/example_one_px4_pegasus_launch_script.py`
  - Multiple drones: `simulation/isaac-sim/launch_scripts/example_multi_px4_pegasus_launch_script.py`

- **Scene Preparation Utilities:**
  - `simulation/isaac-sim/utils/scene_prep.py`

- **Related Skills:**
  - [test-in-simulation](../test-in-simulation) - Testing modules in Isaac Sim
  - [debug-module](../debug-module) - Debugging simulation issues
