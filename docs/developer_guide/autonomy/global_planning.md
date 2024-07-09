
Should publish a set of global waypoints of message type `mavros_msgs/WaypointList.msg`to the topic
`$(arg robot_name)/global_planner/waypoints`.

Should make sure to set the correct frame, e.g. `msg.frame=Waypoint.FRAME_LOCAL_ENU`.


The global planner can do whatever it wants internally.