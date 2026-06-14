# VOXL2 USB Ethernet Setup & Hardware Test Guide

> **tl;dr** — Use the script: `./robot/ros_ws/src/interface/modalai_interface/scripts/run_hw_test.sh`
> (add `--wifi` if your machine is already on the same WiFi as the drone). This file documents what the script does and how to run it manually if needed.

---

## The Problem

When the development machine is on a different subnet from the VOXL2's WiFi router
(e.g. CMU ethernet vs lab WiFi on 192.168.123.x), there is no route between them and
DDS multicast never finds the drone.

ADB over USB works — but ADB only forwards TCP, not UDP, so DDS (which runs on UDP)
still can't get through even with ADB port-forwarding.

## The Fix: USB NCM Ethernet

The VOXL2's USB-C port exposes a USB gadget that normally only advertises ADB and a
Qualcomm diagnostic interface. The gadget also has an NCM (USB Ethernet) function
registered (`ncm.0`) but not linked into the active configuration by default.

Enabling it gives a direct Layer-2 Ethernet link over the existing USB cable — no WiFi,
no new hardware, no cable to the router.

---

## One-Time Setup (Services Persist on VOXL2)

These systemd services only need to be created once. After that, just run them on each
VOXL2 boot (see "Manual Procedure" below).

### Create the NCM enable script on VOXL2

```bash
adb shell "cat > /data/enable_ncm.sh << 'SCRIPT'
#!/bin/sh
UDC=\$(cat /sys/kernel/config/usb_gadget/g1/UDC)
echo \"\" > /sys/kernel/config/usb_gadget/g1/UDC
sleep 1
ln -sf /sys/kernel/config/usb_gadget/g1/functions/ncm.0 /sys/kernel/config/usb_gadget/g1/configs/c.1/f3 2>/dev/null
sleep 1
echo \"\$UDC\" > /sys/kernel/config/usb_gadget/g1/UDC
sleep 2
ip addr add 192.168.123.2/24 dev usb0 2>/dev/null || true
ip link set usb0 up 2>/dev/null || true
echo \"done \$(date)\" > /data/ncm_done.txt
SCRIPT
chmod +x /data/enable_ncm.sh"
```

### Create the two systemd services

```bash
# Service 1: add NCM function to gadget config
adb shell "printf '[Unit]\nDescription=Enable USB NCM Ethernet\n\n[Service]\nType=oneshot\nExecStart=/data/enable_ncm.sh\nRemainAfterExit=yes\n' > /etc/systemd/system/usb-ncm.service && systemctl daemon-reload"

# Service 2: clean unbind/rebind so host enumerates the new NCM function
adb shell "printf '[Unit]\nDescription=Rebind USB gadget to apply NCM\n\n[Service]\nType=oneshot\nExecStart=/bin/sh -c \"echo \\\"\\\" > /sys/kernel/config/usb_gadget/g1/UDC && sleep 2 && echo a600000.dwc3 > /sys/kernel/config/usb_gadget/g1/UDC && sleep 2 && ip addr add 192.168.123.2/24 dev usb0 2>/dev/null || true && ip link set usb0 up 2>/dev/null || true && echo rebind_done > /data/rebind.txt\"\nRemainAfterExit=yes\n' > /etc/systemd/system/usb-rebind.service && systemctl daemon-reload"
```

> **Why two services?** The first adds the `f3 → ncm.0` symlink but hits "Device or resource busy" on UDC re-enable because the system auto-rebinds too fast. The second does a clean unbind + rebind, causing the USB host to re-enumerate and create the `enx*` interface.

---

## Complete Manual Procedure (USB NCM)

What `run_hw_test.sh` does, step by step.

### 1. Physical setup

Sit the drone on a flat surface. **Remove propellers** for first-time testing. Connect USB-C and confirm ADB:

```bash
adb devices   # should list one device
```

### 2. PX4 bench params (one-time, already set on this VOXL2)

```bash
adb shell "px4-param show SYS_HAS_MAG"      # expect 0
adb shell "px4-param show EKF2_MAG_TYPE"    # expect 5
adb shell "px4-param show COM_DISARM_LAND"  # expect 0
adb shell "px4-param show EKF2_EV_NOISE_MD" # expect 1
```

If any differ:
```bash
adb shell "px4-param set SYS_HAS_MAG 0; px4-param set EKF2_MAG_TYPE 5; \
           px4-param set COM_DISARM_LAND 0; px4-param set EKF2_EV_NOISE_MD 1"
adb shell "systemctl restart voxl-px4.service"
sleep 15 && adb wait-for-device
```

### 3. Enable USB NCM Ethernet

```bash
adb shell "systemctl start usb-ncm.service"
sleep 10 && adb wait-for-device

adb shell "systemctl start usb-rebind.service"
sleep 12 && adb wait-for-device

# Verify VOXL2 got the IP
adb shell "ip addr show usb0 | grep 192.168.123"
# Expected: inet 192.168.123.2/24
```

### 4. Assign IP on the desktop side

```bash
# Find the NCM interface (name derived from MAC, changes on reconnect)
ip link show | grep enx

sudo ip addr add 192.168.123.1/24 dev enxXXXXXXXXXXXX
ping -c 3 192.168.123.2   # should be 0% loss
```

### 5. Bring wlan0 down on VOXL2, restart microdds-agent

VOXL2's WiFi (`wlan0`) gets an IP on the same 192.168.123.0/24 subnet as `usb0`. microdds-agent advertises both — the desktop can't reach the WiFi IP so DDS data flow silently breaks.

```bash
adb shell "ip link set wlan0 down"
adb shell "systemctl restart voxl-microdds-agent.service"
sleep 10 && adb wait-for-device
```

> `wlan0` comes back up on USB reconnect — repeat this step each time.

### 6. Start VOXL2 services

```bash
adb shell "systemctl is-active voxl-px4.service voxl-microdds-agent.service voxl-mpa-ros2.service"
# All three should print "active"
# If not:
adb shell "systemctl start voxl-px4.service voxl-mpa-ros2.service"
```

### 7. Build modalai_interface (once, or after code changes)

```bash
AUTOLAUNCH=false airstack up robot-desktop
docker exec airstack-robot-desktop-1 bash -c \
  "source ~/.bashrc && bws --packages-select px4_msgs modalai_interface"
```

### 8. Start the host-network container

The regular robot container is on Docker bridge `172.31.0.0/24`. VOXL2 also has its own Docker bridge on `172.31.0.0/24` — return routes are broken and Docker NAT hides the container's IP. `--network host` bypasses all of this.

```bash
docker run -d --network host --name modalai-hw-test \
  -v /home/kayla/AirStack:/root/AirStack \
  -e ROBOT_NAME=robot_1 -e ROS_DOMAIN_ID=0 \
  airlab-docker.andrew.cmu.edu/airstack/airstack:v0.18.0_robot-x86-64_dev \
  bash -c "source /root/AirStack/robot/ros_ws/install/local_setup.bash && \
           ros2 launch modalai_interface modalai_interface.launch.xml \
           voxl_qvio_topic:=/vvhub_body_wrt_fixed/pose > /tmp/modalai.log 2>&1"

sleep 8
```

### 9. Run prop_spin_test

```bash
docker exec modalai-hw-test bash -c "
  source /root/AirStack/robot/ros_ws/install/local_setup.bash
  export ROBOT_NAME=robot_1 ROS_DOMAIN_ID=0
  python3 /root/AirStack/robot/ros_ws/src/interface/modalai_interface/scripts/prop_spin_test.py 10 --force
"
```

### 10. Tear down

```bash
docker rm -f modalai-hw-test
```

---

## The `--force` Flag Explained

Normal arming requires PX4's EKF2 to complete yaw alignment (`cs_yaw_align: True`).
On the bench this fails because:

- `voxl-open-vins-server` (OpenVINS) produces VIO with timestamps ~0.8–1.1s behind wall clock
- `voxl-vision-hub` drops every VIO packet ("WARNING: VIO time X.Xs too old")
- EKF2 has no heading source → preflight fails → ARM rejected

The `--force` flag sends `VEHICLE_CMD_COMPONENT_ARM_DISARM` with `param2=21196.0`
(PX4's preflight-bypass key). Appropriate for bench testing only.

**For actual flight**, the OpenVINS timestamp issue must be resolved.

---

## Known Issues / Future Work

1. **OpenVINS timestamp drift**: `voxl-vision-hub` drops all VIO because OpenVINS timestamps with sensor time rather than wall clock. Fixing this enables proper EKF2 yaw and removes the need for `--force`.

2. **voxl_qvio_topic not publishing on bench**: `/vvhub_body_wrt_fixed/pose` only publishes when OpenVINS is actively tracking features. "interface odometry NOT RECEIVED" is expected on the bench.

3. **wlan0 comes back up on USB reconnect**: Not persistent across reconnects — repeat the `ip link set wlan0 down` step each time.

4. **Docker networking**: `--network host` is fine for bench testing. For the regular AirStack stack, the robot container's Docker network should be changed to avoid conflicting with VOXL2's internal Docker bridge (`172.31.0.0/24`).
