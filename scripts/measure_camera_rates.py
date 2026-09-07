#!/usr/bin/env python3
"""Observe image arrivals and header stamps; never change camera settings."""
import argparse
import json
import statistics
import struct
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

parser = argparse.ArgumentParser()
parser.add_argument('--robot', default='robot_1')
parser.add_argument('--seconds', type=float, default=120)
args = parser.parse_args()
rclpy.init()
node = rclpy.create_node('camera_rate_probe')
samples = {'rgb': [], 'depth': []}
subscriptions = []

def receive(kind, data):
    endian = '<' if data[1] & 1 else '>'
    sec, ns = struct.unpack_from(endian + 'iI', data, 4)
    samples[kind].append((time.monotonic(), sec + ns * 1e-9))

for kind, suffix in [('rgb', 'image_rect'), ('depth', 'depth_ground_truth')]:
    topic = f'/{args.robot}/sensors/front_stereo/left/{suffix}'
    subscriptions.append(node.create_subscription(
        Image, topic, lambda data, k=kind: receive(k, data),
        qos_profile_sensor_data, raw=True))
start = time.monotonic()
while time.monotonic() - start < args.seconds:
    rclpy.spin_once(node, timeout_sec=0.2)
result = {'robot': args.robot, 'observation_wall_s': time.monotonic()-start}
for kind, seq in samples.items():
    row = {'messages': len(seq)}
    if len(seq) > 1:
        wall = seq[-1][0] - seq[0][0]
        sim = seq[-1][1] - seq[0][1]
        unique = sorted(set(s for _, s in seq))
        gaps = [b-a for a,b in zip(unique, unique[1:])]
        row.update(wall_fps=(len(seq)-1)/wall,
                   header_span_sim_s=sim,
                   unique_frames=len(unique),
                   sim_fps=(len(unique)-1)/sim if sim > 0 else None,
                   median_gap_sim_s=statistics.median(gaps) if gaps else None,
                   max_gap_sim_s=max(gaps) if gaps else None,
                   gaps_over_1_sim_s=sum(g > 1 for g in gaps))
    result[kind] = row
print(json.dumps(result), flush=True)
node.destroy_node()
rclpy.shutdown()
