#!/usr/bin/env python3
"""Extract hardware velocity-command + pose tracks using MCAP log time."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    args = p.parse_args()

    cmd_t, cmd_v, pose_t, pose_xyz, pose_q = [], [], [], [], []
    cmd_frame = pose_frame = ""
    with AnyReader([args.bag]) as reader:
        start_ns = reader.start_time
        start_s = start_ns / 1e9
        for conn, timestamp, raw in reader.messages():
            t = (timestamp - start_ns) / 1e9
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic.endswith("/fmu/velocity_command"):
                cmd_t.append(t)
                lin = msg.twist.linear
                cmd_v.append([lin.x, lin.y, lin.z])
                cmd_frame = msg.header.frame_id or cmd_frame
            elif conn.topic.endswith("/pose") and "goal" not in conn.topic:
                pose_t.append(t)
                pos = msg.pose.position
                ori = msg.pose.orientation
                pose_xyz.append([pos.x, pos.y, pos.z])
                pose_q.append([ori.w, ori.x, ori.y, ori.z])
                pose_frame = msg.header.frame_id or pose_frame

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.out,
        command_times=np.asarray(cmd_t),
        velocity_command=np.asarray(cmd_v),
        pose_times=np.asarray(pose_t),
        pose_position=np.asarray(pose_xyz),
        pose_quat_wxyz=np.asarray(pose_q),
        command_frame=np.array(cmd_frame),
        pose_frame=np.array(pose_frame),
        bag_start_epoch_s=np.array(start_s),
    )
    print(f"wrote {args.out}: {len(cmd_t)} cmds, {len(pose_t)} poses, start={start_s:.3f}")


if __name__ == "__main__":
    main()
