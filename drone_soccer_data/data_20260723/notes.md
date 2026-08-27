# Notes for 20260723

Using bv drone

- robot/bags/drone_4_VolleyBall_20260724_053214: comms lost at end
- robot/bags/drone_4_VolleyBall_20260724_055842: working
- robot/bags/drone_4_VolleyBall_20260724_085739: lost position mode
- robot/bags/drone_4_VolleyBall_20260724_090034: good
- robot/bags/drone_4_VolleyBall_20260724_091349: good
- robot/bags/drone_4_VolleyBall_20260724_110043: thrust command RL policy

Experiment day summary:

- More data collection trials with fixed VolleyBall recording from Ian and Bavin pilot
- Decided to delay recording thrust-related data (from px4) and downward camera (from voxl) -> TODO !
- Tested fmu_pose mode in trajectory_commander at 50Hz, figure8 and square
- Bavin's RL-trained altitude and thrust policy to follow 4x4 square waypoint
