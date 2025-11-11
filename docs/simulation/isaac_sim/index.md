# Isaac Sim

The primary simulator we support is [NVIDIA Isaac Sim](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html). 
We chose Isaac Sim as the best balance between photorealism and physics simulation.

Isaac Sim is built on [NVIDIA Omniverse](https://developer.nvidia.com/omniverse), which provides a physically-based rendering engine and accurate rigid-body physics. This combination allows us to create realistic scenes that behave and look close to the real world.

## Why Isaac Sim

### ROS 2 Integration
Isaac Sim has native support for ROS 2 via the Isaac ROS bridge, enabling seamless communication of sensor topics, transforms, and commands.
Robots simulated in Isaac Sim can publish camera images, LiDAR scans, odometry, and TF data directly to ROS 2 nodes, while subscribing to control topics for motion commands or trajectory execution.
This makes it easy to test the same ROS 2 stack in simulation and later deploy it to real robots with minimal changes.

### Sensor Availability and Realism
Isaac Sim provides a wide range of high-fidelity, GPU-accelerated virtual sensors, including:

- RGB, depth, and segmentation cameras

- Stereo and fisheye cameras

- LiDARs and Radars

- IMUs, GPS, and odometry sensors

### Custom sensor scripting via OmniGraph or Python
These sensors produce data with realistic noise models and latency, allowing perception pipelines to be validated under realistic conditions.

### Scalability and Reusability
Through USD (Universal Scene Description), Isaac Sim supports modular world composition and scalable scene assembly.
Props, environments, and robots can be managed independently, versioned, and reused across projects.

AirStack leverages this architecture to promote the creation of reusable simulation components—for example, a robot defined in one project can be dropped into another scene without modification.
USD’s layer and reference system also makes it straightforward to build complex environments from smaller, composable files, reducing duplication and simplifying collaboration between teams.

This modularity enables rapid iteration, large-scale simulation generation, and consistent environment definitions across training, testing, and deployment workflows.

Together, these features make Isaac Sim a comprehensive platform for robotics simulation, bridging the gap between visual realism, physical accuracy, and ROS 2-based autonomy stacks.

## USD File Naming Conventions
AirStack uses the following file naming conventions:

**Purely 3D graphics**

- `*.prop.usd` ⟵ simply a 3D model with materials, typically encompassing just a single object. Used for individual assets or objects (as mentioned earlier), representing reusable props.

- `*.stage.usd` ⟵ an environment composed of many props, but with no physics, no simulation, no robots. simply scene graphics

**Simulation-ready**

- `*.robot.usd` ⟵ a prop representing a robot plus ROS2 topic and TF publishers, physics, etc.

- `*.scene.usd` ⟵ an environment PLUS physics, simulation, or robots