
# MAC-VO

## Summary

Currently, the state estimation of our robot relies on [MAC-VO](https://mac-vo.github.io/), a learning-based stereo visual odometry algorithm. This is purely camera based, and does not rely on additional sensors. On initialization, the node will load the model weights, and then allocate the required memory to store the model on first inference. This process may take some time. Once this process is complete, the inference time should be able to run at around 3 Hz. Documentation on the MAC-VO model can be found [here](https://mac-vo.github.io/wiki/category/extend-mac-vo/)

This node is also setup to retrieve camera info on node initialization. This includes camera intrinsics and baseline of the desired camera, and is setup using a service call to the camera parameter server.

The output from this node should give the pose estimates of the model, the feature points in 3D space used to estimate the pose, and a visualization image of the points projected onto the RGB image. **The pose is currently given in the perspective of the left camera frame.**

## Configuration

The wrapper that is currently used for interfacing with the non-ROS MAC-VO logic is modified from the one provided [here](https://github.com/MAC-VO/MAC-VO-ROS2). For our purposes, we wanted modularity in interfacing with the node, so we now have two configuration files:

- `interface_config.yaml`: This file specifies the desired camera name, the subscriber and publisher topics, and the size of the image when being fed through inference. This was designed specifically for Airstack
- `model_config.yaml`: This file is sourced from the official MAC-VO ROS wrapper and defines the structure for creating the MAC-VO model. It also specifies the location of the model weights, currently stored at /root/model_weights/MACVO_FrontendCov.pth within the Docker container.

## Parameters

Below are the parameters for the `interface_config.yaml`. To find out more about the parameters for the `model_config.yaml`, please consult the [MAC-VO Documentation](https://mac-vo.github.io/wiki/category/extend-mac-vo/).

| <div style="width:220px">Parameter</div>  | Description
|----------------------------|---------------------------------------------------------------
| `camera_name`                      | The name of the camera that the visual odometry should process from|
| `camera_param_server_client_topic` | Topic name for the camera parameter server|
| `imageL_sub_topic`                 | Topic name for the left stereo image, appended with the camera name|
| `imageR_sub_topic`                 | Topic name for the right stereo image, appended with the camera name|
| `pose_pub_topic`                   | Topic name for the pose estimate output from the MAC-VO model|
| `point_pub_topic`                  | Topic name for the point cloud of feature points with covariances used to estimate pose |
| `img_pub_topic`                    | Topic name for the visualization of the feature points over the rgb for debugging |
| `inference_dim_u`                  | The width of the images fed into the MAC-VO model, which affects inference rate |
| `inference_dim_v`                  | The height of the images fed into the MAC-VO model, which affects inference rate |

## Services
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `~/get_camera_params`     | sensor_interfaces/GetCameraParams | A service to get info about the desired camera|

## Subscriptions
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `~/left/image_rect`     | sensor_msgs/Image | The left RGB image from the stereo camera |
| `~right/image_rect`     | sensor_msgs/Image | The right RGB image from the stereo camera|

## Publications
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `~/visual_odometry_pose`  | nav_msgs/Path | Outputs the pose estimate output from the MAC-VO model.|
| `~/visual_odometry_points`  | nav_msgs/Path | Outputs the point cloud of feature points with covariances used to estimate pose. |
| `~/visual_odometry_img`  | nav_msgs/Path | Outputs the visualization of the feature points over the rgb for debugging. |