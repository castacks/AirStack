[//]: # "global"
# World Model

Global world models are responsible for maintaining a representation of the world that is used by the global planner to generate a plan. This representation is typically a map of the environment, but can also include other information such as the location of other robots, obstacles, and goals.

The current placeholder world model is a voxelized map representation called [VDB Mapping](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros2).