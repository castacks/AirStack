"""Make `mighty_bridge` importable when these tests run on a bare host.

`colcon test` runs from the package root with the package already installed,
but the point of this suite is to run with NO ROS workspace at all
(`pytest robot/ros_ws/src/modules/asm_mighty/mighty_bridge/test`), so the
package directory has to be put on sys.path by hand.
"""
import os
import sys

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)
