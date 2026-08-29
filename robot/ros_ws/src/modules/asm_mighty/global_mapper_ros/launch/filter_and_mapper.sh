




rosnode kill /SQ01s/global_mapper_nodelet /SQ01s/global_mapper_ros /SQ01s/depthmap_filter_nodelet /SQ01s/global_mapper_nodelet /SQ01s/my_manager

sleep 1
roslaunch global_mapper_ros global_mapper_nodelet.launch manager:=global_mapper_ros&
sleep 1
roslaunch depthmap_filter depthmap_filter.launch manager:=global_mapper_ros

