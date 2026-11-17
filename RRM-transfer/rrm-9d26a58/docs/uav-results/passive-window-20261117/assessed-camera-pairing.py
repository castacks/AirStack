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


def camera_window_metadata(records, *, started_monotonic, now_monotonic, max_age_s=2.):
    """Validate three paired records at a same-process receipt-time boundary.

    Source progression and recent receipts do not prove source clock correctness,
    renderer synchronization or permission to execute a selected viewpoint.
    """
    def finite(value):
        return type(value) in (int, float) and math.isfinite(value)

    if (not all(finite(v) for v in (started_monotonic, now_monotonic, max_age_s)) or
            max_age_s <= 0 or started_monotonic > now_monotonic):
        raise ValueError('valid monotonic window and positive age limit required')
    if not isinstance(records, (list, tuple)) or len(records) != 3:
        raise ValueError('three paired camera records required')
    previous = None
    signature = None
    receipts = []
    stamps = []
    for record in records:
        if not isinstance(record, dict):
            raise ValueError('camera record required')
        headers = record.get('headers')
        if not isinstance(headers, dict) or set(headers) != {'rgb', 'depth', 'info'}:
            raise ValueError('three channel headers required')
        header = headers['rgb']
        if not isinstance(header, dict) or any(other != header for other in headers.values()):
            raise ValueError('exact camera headers required')
        stamp = header.get('stamp')
        if not isinstance(stamp, dict):
            raise ValueError('source stamp required')
        sec, ns = stamp.get('sec'), stamp.get('nanosec')
        if (type(sec) is not int or type(ns) is not int or sec < 0 or
                not 0 <= ns < 1_000_000_000 or sec*1_000_000_000+ns <= 0):
            raise ValueError('positive valid source stamps required')
        source_ns = sec*1_000_000_000+ns
        if previous is not None and source_ns <= previous:
            raise ValueError('strictly increasing camera source stamps required')
        previous = source_ns
        stamps.append(source_ns)
        frame = header.get('frame_id')
        size, intrinsic = record.get('image_size'), record.get('intrinsic')
        if (not isinstance(frame, str) or not frame.strip() or
                not isinstance(size, (list, tuple)) or len(size) != 2 or
                any(type(v) is not int or v <= 0 for v in size) or
                not isinstance(intrinsic, (list, tuple)) or len(intrinsic) != 9 or
                not all(finite(v) for v in intrinsic) or
                intrinsic[0] <= 0 or intrinsic[4] <= 0 or
                any(abs(intrinsic[i]) > 1e-9 for i in (1, 3, 6, 7)) or
                abs(intrinsic[8]-1) > 1e-9 or
                record.get('exact_stamp_pair') is not True or record.get('depth_encoding') != '32FC1'):
            raise ValueError('valid paired pinhole camera metadata required')
        current = (frame, tuple(size), tuple(intrinsic))
        if signature is not None and current != signature:
            raise ValueError('camera calibration/frame/size changed within window')
        signature = current
        times = record.get('receipt_monotonic_s')
        if not isinstance(times, dict) or set(times) != {'rgb', 'depth', 'info'}:
            raise ValueError('per-channel monotonic receipts required')
        for receipt in times.values():
            if (not finite(receipt) or not started_monotonic <= receipt <= now_monotonic or
                    now_monotonic-receipt >= max_age_s):
                raise ValueError('camera receipt outside current fresh window')
            receipts.append(receipt)
    return dict(source_span_s=(stamps[-1]-stamps[0])*1e-9,
                max_channel_receipt_age_s=now_monotonic-min(receipts),
                window_receipt_span_s=max(receipts)-min(receipts),
                max_age_s=float(max_age_s), checked_monotonic_s=now_monotonic,
                started_monotonic_s=started_monotonic,
                freshness_scope='same-process receipt freshness; acquisition synchronization unproven')
