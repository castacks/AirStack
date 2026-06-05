# DiffPhysDrone Model Conversion and Starling Deployment Notes

This note records the working conversion path for the DiffPhysDrone checkpoint and the deployment interfaces needed for Starling / PX4 use. It is intentionally portable: the same model artifacts and runtime contract should work in AirStack, plain Pegasus, or a VOXL/Starling deployment as long as the surrounding code provides the same inputs and consumes the same outputs.

## Current Artifacts

The source checkpoint is:

```text
checkpoint0004.pth
```

The validated exported artifacts are:

```text
checkpoint0004.onnx
checkpoint0004_float32.tflite
checkpoint0004_float16.tflite
```

The ONNX export was checked against the PyTorch checkpoint with:

```text
max_action_error=2.3841858e-06
max_hidden_error=1.3113022e-06
ONNX parity check passed
```

That means the ONNX model is numerically equivalent to the PyTorch checkpoint for the tested recurrent inference path.

## Model Interface Contract

The exported policy is recurrent. Each inference step consumes the current depth image, a low-dimensional state vector, and the previous recurrent hidden state.

Inputs:

```text
depth:  float32 [1, 1, 12, 16]
state:  float32 [1, 10]
hidden: float32 [1, 192]
```

Outputs:

```text
action:      float32 [1, 6]
next_hidden: float32 [1, 192]
```

The runtime must store `next_hidden` and feed it back as `hidden` on the next inference step. Reset `hidden` to zeros when starting a new episode, rebooting the policy, or intentionally clearing policy memory.

## Conversion Path

### 1. Export PyTorch Checkpoint to ONNX

Inside the environment that has the DiffPhysDrone wrapper package and PyTorch installed:

```bash
source /root/AirStack/robot/ros_ws/install/setup.bash
mkdir -p /tmp/diffphysdrone_export

ros2 run diffphysdrone_px4_wrapper diffphysdrone_export_onnx \
  --checkpoint /airlab-storage/chiron/models/diffphysdrone/checkpoint0004.pth \
  --output /tmp/diffphysdrone_export/checkpoint0004.onnx
```

If the model directory is read-only, write to `/tmp` or another writable path and copy the artifact out afterward.

### 2. Copy Artifacts to a Machine With Normal Python Package Access

Example from a laptop:

```bash
scp ubuntu@lorenzo-rtx5000:/home/ubuntu/volume/home/ubuntu/dtc/airlab_ws/checkpoint0004.onnx .
scp ubuntu@lorenzo-rtx5000:/home/ubuntu/volume/home/ubuntu/dtc/airlab_ws/checkpoint0004.pth .
```

If the checkpoint only exists inside the robot container, copy it to the host first:

```bash
docker cp airstack-dtc-robot-desktop-1:/airlab-storage/chiron/models/diffphysdrone/checkpoint0004.pth \
  /home/ubuntu/volume/home/ubuntu/dtc/airlab_ws/checkpoint0004.pth
```

### 3. Check ONNX Parity

On a laptop or other machine where `torch`, `numpy`, and `onnxruntime` can be installed:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install torch numpy onnxruntime
```

Run:

```bash
KMP_DUPLICATE_LIB_OK=TRUE OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
python -m diffphysdrone_px4_wrapper.check_onnx_parity \
  --checkpoint checkpoint0004.pth \
  --onnx checkpoint0004.onnx
```

On macOS, `KMP_DUPLICATE_LIB_OK=TRUE` may be needed because PyTorch and ONNX Runtime can load conflicting OpenMP runtimes in the same process. This is acceptable for an offline parity check; it should not be treated as a final flight runtime configuration.

### 4. Convert ONNX to TFLite

Create a clean conversion environment:

```bash
conda create -n diffphysdrone_tf python=3.12 -y
conda activate diffphysdrone_tf
python -m pip install --upgrade pip
python -m pip install onnx onnx2tf tensorflow
```

Convert:

```bash
onnx2tf -i checkpoint0004.onnx -o checkpoint0004_saved_model
```

For this model, `onnx2tf` directly emits TFLite files:

```text
checkpoint0004_saved_model/checkpoint0004_float32.tflite
checkpoint0004_saved_model/checkpoint0004_float16.tflite
```

Do not run `tf.lite.TFLiteConverter.from_saved_model("checkpoint0004_saved_model")` unless a real TensorFlow SavedModel exists there. In the working conversion, `onnx2tf` created TFLite flatbuffers directly, not a SavedModel.

### 5. Smoke Test TFLite

```bash
python - <<'PY'
import numpy as np
import tensorflow as tf

model_path = "checkpoint0004_saved_model/checkpoint0004_float32.tflite"
interpreter = tf.lite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

print("inputs:")
for d in interpreter.get_input_details():
    print(d["name"], d["shape"], d["dtype"])

print("outputs:")
for d in interpreter.get_output_details():
    print(d["name"], d["shape"], d["dtype"])

for d in interpreter.get_input_details():
    x = np.zeros(d["shape"], dtype=d["dtype"])
    interpreter.set_tensor(d["index"], x)

interpreter.invoke()

print("ran inference")
for d in interpreter.get_output_details():
    y = interpreter.get_tensor(d["index"])
    print(d["name"], y.shape, y.dtype, "min", y.min(), "max", y.max())
PY
```

Keep both TFLite variants until the VOXL target runtime is confirmed:

```text
checkpoint0004_float32.tflite
checkpoint0004_float16.tflite
```

The float16 model is the likely deployment candidate because it is smaller and better aligned with embedded inference, but float32 is useful as a reference fallback.

## Runtime Architecture

The runtime should preserve this dataflow:

```text
Starling depth source
  -> depth preprocessing to [1, 1, 12, 16]
  -> policy state construction to [1, 10]
  -> recurrent hidden state [1, 192]
  -> policy inference
  -> action [1, 6]
  -> action-to-PX4 wrapper
  -> PX4 Offboard attitude/thrust or compatible setpoint interface
```

The policy itself does not command PX4 directly. It outputs a learned action vector. The wrapper is responsible for converting that action into the vehicle command interface.

## AirStack Runtime Mapping

In the AirStack integration, the current ROS graph is:

```text
/robot_1/sensors/front_stereo/right/depth_ground_truth or Starling ToF depth
  -> /diffphysdrone_policy
  -> /robot_1/diffphysdrone/accel_cmd
  -> /diffphysdrone_attitude_bridge
  -> /robot_1/interface/cmd_attitude_thrust
  -> /robot_1/interface/robot_interface
  -> MAVROS / PX4 Offboard
```

The same separation should be kept in other repos:

```text
policy inference backend: PyTorch / ONNX / TFLite
control wrapper: action or acceleration to PX4-compatible command
sim adapter: Pegasus or Isaac Sim sensor and odometry wiring
real adapter: Starling camera/depth and PX4 Offboard wiring
```

## Plain Pegasus or Other Repo Port

For a non-AirStack Pegasus repo, the minimum pieces to port are:

```text
diffphysdrone_px4_wrapper/inference.py
diffphysdrone_px4_wrapper/export_onnx.py
diffphysdrone_px4_wrapper/check_onnx_parity.py
diffphysdrone_px4_wrapper/voxl_policy_smoke.py
policy node or equivalent runtime loop
action-to-PX4 bridge logic
```

The simulator-specific code only needs to provide:

```text
depth image
vehicle odometry / velocity / attitude state
desired velocity or goal-derived command
PX4 Offboard output sink
```

If the other repo does not use ROS 2, keep the model contract and rewrite only the transport layer. The model does not care whether the inputs came from ROS topics, Pegasus Python callbacks, MAVSDK, or a VOXL service.

## Starling / VOXL Deployment Notes

For real Starling deployment, the likely final path is:

```text
checkpoint0004.pth
  -> checkpoint0004.onnx
  -> checkpoint0004_float32.tflite / checkpoint0004_float16.tflite
  -> VOXL inference runtime
  -> PX4 Offboard command bridge
```

The real drone runtime must check:

```text
depth stream source and rate
depth units and invalid-pixel behavior
camera optical frame orientation
body frame convention
state vector convention
action frame convention
PX4 frame convention
Offboard setpoint rate
arming and failsafe behavior
thrust scaling
maximum tilt / acceleration limits
```

The important frame conversions are:

```text
ROS ENU -> PX4 NED:
  (x_east, y_north, z_up) -> (y_north, x_east, -z_up)

ROS FLU -> PX4 FRD:
  (x_forward, y_left, z_up) -> (x_forward, -y_left, -z_up)
```

Do not assume that a model-format conversion fixes flight behavior. If the drone flies away from a waypoint or accelerates in the wrong direction, the likely issue is in frame conventions, action interpretation, attitude/thrust conversion, or PX4 Offboard command semantics.

## Validation Checklist

Before any real flight:

```text
[ ] PyTorch checkpoint loads.
[ ] ONNX export completes.
[ ] ONNX parity against PyTorch passes.
[ ] TFLite conversion completes.
[ ] TFLite dummy inference runs.
[ ] Policy receives the real Starling depth source, not a temporary ZED topic.
[ ] Policy depth debug image looks correct.
[ ] Policy output changes when an obstacle enters depth.
[ ] Wrapper output is bounded.
[ ] PX4 receives setpoints at a stable Offboard rate.
[ ] Vehicle frame and camera frame transforms are verified with a simple known command.
[ ] Bench test confirms no command is sent unless the safety gate is enabled.
[ ] First flight uses very small velocity/acceleration limits.
```

## Known Current Limitation

The AirStack robot container and SSH host could not install `onnxruntime` from PyPI because SSL/network access to Python package hosts timed out. That is why ONNX parity and TFLite conversion were run on a laptop instead. This does not invalidate the artifacts; it only means the AirStack container cannot currently run the ONNX backend until its Python dependency installation path is fixed or an `onnxruntime` wheel is copied in manually.
