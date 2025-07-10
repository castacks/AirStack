import subprocess
import socket
import time

def wait_for_mavlink(host='localhost', port=4560, timeout=15):
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            with socket.create_connection((host, port), timeout=1):
                print(f"MAVLink endpoint is up at {host}:{port}")
                return True
        except Exception:
            print(f"Waiting for MAVLink endpoint at {host}:{port}...")
            time.sleep(0.5)
    print("Timeout waiting for MAVLink endpoint.")
    return False

def run_mavros_launch_nonblocking(fcu_url="tcp://localhost:4560"):
    mavros_cmd = [
        "ros2", "launch", "mavros", "px4.launch",
        f"fcu_url:={fcu_url}"
    ]
    process = subprocess.Popen(mavros_cmd)
    print(f"Started MAVROS with PID {process.pid}")
    return process
