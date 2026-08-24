# Global Packages

The global packages include global world models and planners.


## Launch
The global layer is composed by the stack entry file: the trunk stacks
include `vdb_mapping_ros2.py` (with
`global_bringup/config/vdb_params.yaml`) and
`random_walk_planner.launch.xml` directly — see
`stacks/full_default/launch/stack.launch.xml`. The `global_bringup` package
owns the cross-package VDB config files.

