#!/usr/bin/env python3
"""Exercise actual DisparityNode output on synthetic, isolated stereo topics.

Run after sourcing the candidate prefix in an unused ROS domain; no robot topics,
control messages, or simulator are involved.
"""
import argparse
import hashlib
import json
import os
import signal
from pathlib import Path
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage


def exercise(prefix, width, height, minimum, expected):
    namespace = f'/rrm_stereo_regression_{width}_{height}_{abs(minimum)}'
    node = rclpy.create_node('stereo_valid_window_regression')
    received = []
    subscription = node.create_subscription(
        DisparityImage, namespace + '/disparity', received.append, 10)
    publishers = {
        key: node.create_publisher(cls, namespace + '/' + key, 10)
        for key, cls in [('left/image_rect', Image), ('right/image_rect', Image),
                         ('left/camera_info', CameraInfo), ('right/camera_info', CameraInfo)]
    }
    pixels = np.random.default_rng(714).integers(0, 256, (height, width), dtype=np.uint8)
    shifted = np.roll(pixels, -16, axis=1)
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([
        str(prefix / 'lib/stereo_image_proc/disparity_node'), '--ros-args',
        '-r', '__ns:=' + namespace, '-p', 'min_disparity:=' + str(minimum),
        '-p', 'disparity_range:=64', '-p', 'correlation_window_size:=15',
    ], stdout=log, stderr=subprocess.STDOUT)
    result = None
    try:
        deadline = time.monotonic() + 15
        next_publish = 0
        while time.monotonic() < deadline and len(received) < 2:
            if process.poll() is not None:
                log.seek(0)
                raise RuntimeError('Disparity node exited: ' + log.read())
            if time.monotonic() >= next_publish:
                next_publish = time.monotonic() + .1
                timestamp = node.get_clock().now().to_msg()
                for side, data in [('left', pixels), ('right', shifted)]:
                    image = Image()
                    image.header.stamp, image.header.frame_id = timestamp, 'camera_' + side
                    image.width, image.height = width, height
                    image.encoding, image.step, image.data = 'mono8', width, data.tobytes()
                    info = CameraInfo()
                    info.header = image.header
                    info.width, info.height = width, height
                    info.distortion_model, info.d = 'plumb_bob', [0.] * 5
                    info.k = [184., 0., width/2, 0., 184., height/2, 0., 0., 1.]
                    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                    info.p = [184., 0., width/2, -22.08 if side == 'right' else 0.,
                              0., 184., height/2, 0., 0., 0., 1., 0.]
                    publishers[side + '/camera_info'].publish(info)
                    publishers[side + '/image_rect'].publish(image)
            rclpy.spin_once(node, timeout_sec=.02)
        if len(received) < 2:
            log.seek(0)
            raise RuntimeError('No paired output before deadline: ' + log.read())
        for message in received:
            roi = message.valid_window
            actual = (roi.x_offset, roi.y_offset, roi.width, roi.height)
            assert actual == expected, (width, height, minimum, actual, expected)
            assert roi.x_offset + roi.width <= message.image.width
            assert roi.y_offset + roi.height <= message.image.height
            assert (message.image.width, message.image.height) == (width, height)
        maps = Path(f'/proc/{process.pid}/maps').read_text()
        assert str(prefix / 'lib/libstereo_image_proc.so') in maps, maps
        result = dict(width=width, height=height, min_disparity=minimum,
                      valid_window=list(expected), callbacks=len(received),
                      loaded_candidate_library=True)
        return result
    finally:
        # Exercise normal ROS shutdown and expose shutdown failure in the result.
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        if result is not None:
            result['shutdown_returncode'] = process.returncode
        log.close()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    args = parser.parse_args()
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) in (0, 1):
        raise SystemExit('Use an isolated ROS_DOMAIN_ID, not the robot/GCS domains.')
    prefix = args.prefix.resolve()
    rclpy.init()
    try:
        results = [exercise(prefix, *case) for case in [
            (480, 300, 0, (70, 7, 402, 285)),
            (480, 300, -16, (54, 7, 409, 285)),
            (72, 32, 0, (0, 0, 0, 0)),
        ]]
        assert all(result['shutdown_returncode'] == 0 for result in results), results
        print(json.dumps(dict(cases=results, library_sha256=hashlib.sha256(
            (prefix / 'lib/libstereo_image_proc.so').read_bytes()).hexdigest()), indent=2))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
