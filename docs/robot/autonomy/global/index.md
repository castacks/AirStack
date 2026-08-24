# Global Packages

The global packages include global world models and planners.


## Launch
The global layer is composed by the stack entry file (the legacy
`global_bringup` layer launch was removed with the AUTONOMY_ROLE dispatch):
the trunk stacks include `vdb_mapping_ros2.py` (with
`global_bringup/config/vdb_params.yaml`) and
`random_walk_planner.launch.xml` directly — see
`stacks/full_default/launch/stack.launch.xml`. The `global_bringup` package
remains as the owner of the cross-package VDB config files.

