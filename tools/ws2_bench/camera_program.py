"""Load the same ROS-independent camera contract used by the bridge."""
import importlib.util
from pathlib import Path

_path = Path(__file__).resolve().parents[2] / 'robot/ros_ws/src/local/planners/mononav_bridge/mononav_bridge/camera_program.py'
_spec = importlib.util.spec_from_file_location('_ws2_camera_program', _path)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
VERSION = _module.VERSION
FACTORS = _module.FACTORS
validate = _module.validate
schema = _module.schema
identity = _module.identity
CameraProgram = _module.CameraProgram
