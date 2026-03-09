
# Camera Parameter Server

## Summary

The camera parameter server was designed to eliminate the need for multiple nodes to subscribe to each camera individually, reducing unnecessary subscribers and callbacks. Its sole purpose is to listen to the camera info topics, store relevant information about the cameras, and provide it on demand for other nodes to query.

## Configuration

The camera parameter server is currently configurable through a non-ROS configuration file. This file allows users to define a list of cameras, specifying their types and topic names. At the top level of the configuration, a base link name is provided to indicate the `tf` name for the robot's center. Additionally, a parameter called `camera_list` contains a list of dictionaries, with each dictionary representing an individual camera. Currently, two camera types are supported: monocular and stereo.

## Parameters

Below are the parameters needed for the meta level camera parameter server configuration, as well as the camera fields needed to specify individual camera types.

### Meta Level Parameters

| <div style="width:220px">Parameter</div>  | Description
|----------------------------|---------------------------------------------------------------
| `base_link_frame_id`  | The frame name of the base link, or center frame of the robot|
| `camera_list`         | A list of dictionaries that define each camera of the system|

### Monocular Camera Parameters

| <div style="width:220px">Parameter</div>  | Description
|----------------------------|---------------------------------------------------------------
| `camera_name`            | The name of the camera|
| `camera_type`            | The type of camera, for monocular being `mono` |
| `camera_info_sub_topic`  | The info topic name for the camera, normally `camera_info`|
| `camera_frame_id`        | The frame name of the camera to find its tf |

### Stereo Camera Parameters

| <div style="width:220px">Parameter</div>  | Description
|----------------------------|---------------------------------------------------------------
| `camera_name`            | The name of the camera|
| `camera_type`            | The type of camera, for stereo being `stereo` |
| `camera_info_sub_topic`  | The info topic name for the camera, normally `camera_info`|
| `left_camera_frame_id`   | The frame name of the left camera for find its tf |
| `right_camera_frame_id`  | The frame name of the right camera to find its tf |

## Services
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `~/get_camera_params`      | sensor_interfaces/GetCameraParams | The service to get info about the desired camera. This provides camera intrinsics, transform frame ids, and baseline if the camera type is a stereo|

## Subscriptions
| <div style="width:220px">Parameter</div> | Type | Description
|----------------------------|----------------------------------------|-----------------------|
| `tf/`           | tfMessage | Listens for the tf for the specified cameras |
| `~/camera_info` | sensor_msgs/CameraInfo | Listens for the info for specified cameras|