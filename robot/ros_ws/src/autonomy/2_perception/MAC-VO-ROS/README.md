# MAC-VO ROS2 Wrapper

This repository is the ROS-2 wrapper for our work *MAC-VO: Metrics-aware Covariance for Learning-based Stereo Visual Odometry*. To visit our original codebase, see [https://github.com/MAC-VO/MAC-VO](https://github.com/MAC-VO/MAC-VO).

## Install and Configuration

To install as a ROS2 node, clone this directory in your ROS2 workspace and run 

> [!NOTE]
> Please clone the repository with `--recursive` flag to clone all the submodules automatically.

```bash
colcon build
source install/local_setup.bash
```

To launch the MAC-VO Node, use the following config, substitute `[PATH_TO_CONFIG]` with your own `.yaml` config path.

```
ros2 run MACVO_ROS2 MACVO --config [PATH_TO_CONFIG]
```

## Pretrained Model

Please follow the `README.md` on [https://github.com/MAC-VO/MAC-VO](https://github.com/MAC-VO/MAC-VO) to download the pre-trained model. The default path for the pre-trained model is set to be `MACVO_ROS2/src/Module`
