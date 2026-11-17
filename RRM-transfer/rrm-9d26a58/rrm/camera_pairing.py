"""Exact RGB/depth/calibration header evidence behind the camera adapter boundary.

Header equality is not renderer synchronization, freshness or motion permission.
"""
import math


def camera_pair_metadata(rgb, depth, info):
    """Validate ROS-like camera messages and copy independently auditable metadata."""
    headers = {}
    for name, message in (('rgb', rgb), ('depth', depth), ('info', info)):
        stamp = message.header.stamp
        if (type(stamp.sec) is not int or type(stamp.nanosec) is not int or
                stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or
                (stamp.sec == 0 and stamp.nanosec == 0)):
            raise ValueError('positive valid source stamps required')
        frame = message.header.frame_id
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError('nonempty camera frame required')
        if any(type(value) is not int or value <= 0 for value in (message.width, message.height)):
            raise ValueError('positive image dimensions required')
        headers[name] = dict(frame_id=frame, stamp=dict(sec=stamp.sec, nanosec=stamp.nanosec))
    if any(header != headers['rgb'] for header in headers.values()):
        raise ValueError('exact source stamp and frame pairing required')
    if any((message.width, message.height) != (rgb.width, rgb.height) for message in (depth, info)):
        raise ValueError('RGB/depth/calibration dimensions must match')
    if depth.encoding != '32FC1':
        raise ValueError('requires explicit 32FC1 metres depth')
    if not isinstance(rgb.encoding, str) or not rgb.encoding:
        raise ValueError('explicit RGB source encoding required')
    intrinsic = list(info.k)
    if len(intrinsic) != 9 or not all(
            isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
            for value in intrinsic):
        raise ValueError('finite 3x3 calibration required')
    return dict(headers=headers, image_size=[rgb.width, rgb.height],
                frame=rgb.header.frame_id, intrinsic=[float(value) for value in intrinsic],
                rgb_encoding=rgb.encoding, depth_encoding=depth.encoding,
                exact_stamp_pair=True,
                pairing_scope='source-header equality only; acquisition synchronization unproven')
