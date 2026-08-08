#!/usr/bin/env python3

import subprocess
import signal
import sys
import os

def main():
    processes = []

    try:
        # Launch the ZED wrapper
        zed_wrapper = subprocess.Popen([
            "ros2", "launch", "zed_wrapper", "zed_dual_camera.launch.py",
            "pose_cam_serial:=41591402",
            "wire_cam_serial:=44405253",
            "camera_name:=robot_1/sensors",
            "node_name:=front_stereo",
        ])
        processes.append(zed_wrapper)

        # Launch the detection conversion script
        script_dir = os.path.expanduser("~/ros2_ws/src/zed_wrapper/scripts")  # Update if needed
        detection = subprocess.Popen(
            ["python3", "zed_to_detection2d.py"],
            cwd=script_dir
        )
        processes.append(detection)

        print("Both processes started. Press Ctrl+C to stop.")

        # Wait for both processes
        for p in processes:
            p.wait()

    except KeyboardInterrupt:
        print("\nStopping processes...")
        for p in processes:
            if p.poll() is None:  # Process still running
                p.send_signal(signal.SIGINT)

        for p in processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()

        print("Done.")
        sys.exit(0)


if __name__ == "__main__":
    main()
