import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
from builtin_interfaces.msg import Time

import sys
import torch
import pypose as pp
import numpy as np

_name_to_dtypes = {
    "rgb8":    (np.uint8,  3),
    "rgba8":   (np.uint8,  4),
    "rgb16":   (np.uint16, 3),
    "rgba16":  (np.uint16, 4),
    "bgr8":    (np.uint8,  3),
    "bgra8":   (np.uint8,  4),
    "bgr16":   (np.uint16, 3),
    "bgra16":  (np.uint16, 4),
    "mono8":   (np.uint8,  1),
    "mono16":  (np.uint16, 1),

    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8":      (np.uint8,  1),
    "bayer_bggr8":      (np.uint8,  1),
    "bayer_gbrg8":      (np.uint8,  1),
    "bayer_grbg8":      (np.uint8,  1),
    "bayer_rggb16":     (np.uint16, 1),
    "bayer_bggr16":     (np.uint16, 1),
    "bayer_gbrg16":     (np.uint16, 1),
    "bayer_grbg16":     (np.uint16, 1),

    # OpenCV CvMat types
    "8UC1":    (np.uint8,   1),
    "8UC2":    (np.uint8,   2),
    "8UC3":    (np.uint8,   3),
    "8UC4":    (np.uint8,   4),
    "8SC1":    (np.int8,    1),
    "8SC2":    (np.int8,    2),
    "8SC3":    (np.int8,    3),
    "8SC4":    (np.int8,    4),
    "16UC1":   (np.uint16,   1),
    "16UC2":   (np.uint16,   2),
    "16UC3":   (np.uint16,   3),
    "16UC4":   (np.uint16,   4),
    "16SC1":   (np.int16,  1),
    "16SC2":   (np.int16,  2),
    "16SC3":   (np.int16,  3),
    "16SC4":   (np.int16,  4),
    "32SC1":   (np.int32,   1),
    "32SC2":   (np.int32,   2),
    "32SC3":   (np.int32,   3),
    "32SC4":   (np.int32,   4),
    "32FC1":   (np.float32, 1),
    "32FC2":   (np.float32, 2),
    "32FC3":   (np.float32, 3),
    "32FC4":   (np.float32, 4),
    "64FC1":   (np.float64, 1),
    "64FC2":   (np.float64, 2),
    "64FC3":   (np.float64, 3),
    "64FC4":   (np.float64, 4)
}


def to_stamped_pose(pose: pp.LieTensor | torch.Tensor, frame_id: str, time: Time) -> geometry_msgs.PoseStamped:
    pose_ = pose.detach().cpu()
    out_msg                 = geometry_msgs.PoseStamped()
    out_msg.header          = std_msgs.Header()
    out_msg.header.stamp    = time
    out_msg.header.frame_id = frame_id
    
    out_msg.pose.position.x = pose_[0].item()
    out_msg.pose.position.y = pose_[1].item()
    out_msg.pose.position.z = pose_[2].item()
    
    out_msg.pose.orientation.x = pose_[3].item()
    out_msg.pose.orientation.y = pose_[4].item()
    out_msg.pose.orientation.z = pose_[5].item()
    out_msg.pose.orientation.w = pose_[6].item()
    return out_msg

def to_nav_msgs_odmetry(pose: pp.LieTensor | torch.Tensor, frame_id: str, time: Time) -> nav_msgs.Odometry:
    pose_ = pose.detach().cpu()
    out_msg                 = nav_msgs.Odometry()
    out_msg.header          = std_msgs.Header()
    out_msg.header.stamp    = time
    out_msg.header.frame_id = frame_id
    out_msg.child_frame_id  = "base_link"  # TODO: UNHARDCODE
    
    out_msg.pose.pose.position.x = pose_[0].item()
    out_msg.pose.pose.position.y = pose_[1].item()
    out_msg.pose.pose.position.z = pose_[2].item()
    
    out_msg.pose.pose.orientation.x = pose_[3].item()
    out_msg.pose.pose.orientation.y = pose_[4].item()
    out_msg.pose.pose.orientation.z = pose_[5].item()
    out_msg.pose.pose.orientation.w = pose_[6].item()
    return out_msg

def from_image(msg: sensor_msgs.Image) -> np.ndarray:
    if msg.encoding not in _name_to_dtypes:
        raise KeyError(f"Unsupported image encoding {msg.encoding}")
    
    dtype_name, channel = _name_to_dtypes[msg.encoding]
    dtype = np.dtype(dtype_name)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    shape = (msg.height, msg.width, channel)
    
    data = np.frombuffer(msg.data, dtype=dtype).reshape(shape)
    data.strides = (msg.step, dtype.itemsize * channel, dtype.itemsize)
    return data


def to_image(arr: np.ndarray, frame_id: str, time: Time, encoding: str = "bgra8") -> sensor_msgs.Image:
    if not encoding in _name_to_dtypes:
        raise TypeError('Unrecognized encoding {}'.format(encoding))

    im = sensor_msgs.Image(encoding=encoding)

    # extract width, height, and channels
    dtype_class, exp_channels = _name_to_dtypes[encoding]
    dtype = np.dtype(dtype_class)
    if len(arr.shape) == 2:
        im.height, im.width, channels = arr.shape + (1,)
    elif len(arr.shape) == 3:
        im.height, im.width, channels = arr.shape
    else:
        raise TypeError("Array must be two or three dimensional")

    # check type and channels
    if exp_channels != channels:
        raise TypeError("Array has {} channels, {} requires {}".format(
            channels, encoding, exp_channels
        ))
    if dtype_class != arr.dtype.type:
        raise TypeError("Array is {}, {} requires {}".format(
            arr.dtype.type, encoding, dtype_class
        ))

    # make the array contiguous in memory, as mostly required by the format
    contig = np.ascontiguousarray(arr)
    im.data = contig.tobytes()
    im.step = contig.strides[0]
    im.header.stamp    = time
    im.header.frame_id = frame_id
    im.is_bigendian = (
        arr.dtype.byteorder == '>' or
        arr.dtype.byteorder == '=' and sys.byteorder == 'big'
    )

    return im


def to_pointcloud(position: torch.Tensor, keypoints: torch.Tensor | None, colors: torch.Tensor, frame_id: str, time: Time) -> sensor_msgs.PointCloud:
    """
    position    should be a Nx3 pytorch Tensor (dtype=float)
    keypoints   should be a Nx2 pytorch Tensor (dtype=float)
    """
    
    out_msg     = sensor_msgs.PointCloud()
    position_   = position.detach().cpu().numpy()
    colors_      = colors.detach().cpu().numpy()
    
    out_msg.header = std_msgs.Header()
    out_msg.header.stamp    = time
    out_msg.header.frame_id = frame_id
    
    
    out_msg.points = [
        geometry_msgs.Point32(x=float(position_[pt_idx, 0]), y=float(position_[pt_idx, 1]), z=float(position_[pt_idx, 2]))
        for pt_idx in range(position.size(0))
    ]
    out_msg.channels = [
        sensor_msgs.ChannelFloat32(
            name="r" , values=colors_[..., 2].tolist()
        ),
        sensor_msgs.ChannelFloat32(
            name="g" , values=colors_[..., 1].tolist()
        ),
        sensor_msgs.ChannelFloat32(
            name="b" , values=colors_[..., 0].tolist()
        )
    ]
    
    if keypoints is not None:
        assert position.size(0) == keypoints.size(0)
        keypoints_  = keypoints.detach().cpu().numpy()
        out_msg.channels.append(sensor_msgs.ChannelFloat32(
            name="kp_u", values=keypoints_[..., 0].tolist()
        ))
        out_msg.channels.append(sensor_msgs.ChannelFloat32(
            name="kp_v", values=keypoints_[..., 1].tolist()
        ))
    
    return out_msg
