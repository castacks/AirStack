# Mission Manager

This package handles the allocation of tasks for the multiple agents.
It assigned agents to search or track. For search it divides the search space.

```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Debugging this node
```
ros2 run --prefix 'gdb -ex run --args'  mission_manager mission_manager_node
```

```
ros2 launch mission_manager mission_manager_launch.py
ros2 run rviz2 rviz2
```