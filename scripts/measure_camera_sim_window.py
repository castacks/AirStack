#!/usr/bin/env python3
"""Read-only RGB/depth cadence and RTF over a fixed simulation-clock window.

Unlike a first-image/last-image denominator, the whole clock window includes
empty scheduling slots. Run in the corresponding robot ROS domain.
"""
import argparse
import json
from pathlib import Path
import struct
import time


def summarize(samples, sim_seconds, wall_seconds, sim_start=0):
    result = {}
    for kind, stamps in samples.items():
        unique = sorted(set(stamps))
        gaps = [b - a for a, b in zip(unique, unique[1:])]
        boundaries = [sim_start, *unique, sim_start + sim_seconds]
        silence = [b - a for a, b in zip(boundaries, boundaries[1:])]
        result[kind] = dict(messages=len(stamps), unique_frames=len(unique),
                            sim_fps=len(unique) / sim_seconds,
                            wall_fps=len(unique) / wall_seconds,
                            max_gap_sim_s=max(gaps) if gaps else None,
                            max_silence_sim_s=max(silence))
    return result


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', required=True)
    ap.add_argument('--sim-seconds', type=float, default=50)
    ap.add_argument('--wall-timeout', type=float, default=7200)
    ap.add_argument('--wait-for-file')
    ap.add_argument('--output', help='atomically write the final JSON here')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('camera_sim_window_probe')
    samples = {'rgb': [], 'depth': []}
    begin = end = None
    latest = None
    started = time.monotonic()

    def clock(msg):
        nonlocal begin, end, latest
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if latest is not None and sim < latest:
            raise RuntimeError('Simulation clock reset during measurement')
        latest = sim
        if args.wait_for_file and not Path(args.wait_for_file).exists():
            return
        if begin is None:
            begin = (sim, time.monotonic(), time.time())
        if end is None and sim - begin[0] >= args.sim_seconds:
            end = (sim, time.monotonic(), time.time())

    def image(kind, data):
        if begin is None or end is not None:
            return
        endian = '<' if data[1] & 1 else '>'
        sec, ns = struct.unpack_from(endian + 'iI', data, 4)
        stamp = sec + ns * 1e-9
        if begin[0] <= stamp < begin[0] + args.sim_seconds:
            samples[kind].append(stamp)

    subs = [node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)]
    for kind, suffix in [('rgb', 'image_rect'), ('depth', 'depth_ground_truth')]:
        subs.append(node.create_subscription(Image,
            f'/{args.robot}/sensors/front_stereo/left/{suffix}',
            lambda data, k=kind: image(k, data), qos_profile_sensor_data, raw=True))
    try:
        while end is None and time.monotonic() - started < args.wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
        if end is None:
            raise SystemExit('No complete simulation-clock measurement window')
        elapsed_sim = end[0] - begin[0]
        elapsed_wall = end[1] - begin[1]
        result = dict(robot=args.robot, sim_start=begin[0], sim_end=end[0],
                      sim_seconds=elapsed_sim, wall_seconds=elapsed_wall,
                      rtf=elapsed_sim / elapsed_wall,
                      wall_start_unix_s=begin[2], wall_end_unix_s=end[2],
                      image_window_sim_seconds=args.sim_seconds,
                      streams=summarize(samples, args.sim_seconds, elapsed_wall, begin[0]))
        payload = json.dumps(result)
        if args.output:
            output = Path(args.output)
            temporary = output.with_name(output.name + '.tmp')
            temporary.write_text(payload + '\n')
            temporary.replace(output)
        else:
            print(payload, flush=True)
        if any(not values for values in samples.values()):
            raise SystemExit('Missing RGB or depth frames')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
