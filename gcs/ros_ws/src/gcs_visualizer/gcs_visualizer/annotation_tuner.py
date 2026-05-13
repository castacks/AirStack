"""annotation_tuner — interactive REPL for dialing in annotation alignment.

Drives the live tuning parameters on /annotation_viz_node and bakes the
result into the JSON when you're happy.

Each command is one keystroke + Enter. Lower case decreases, upper case
increases. The current state and step sizes are reprinted before every
prompt.

    [x/X] tx -/+        [y/Y] ty -/+        [z/Z] tz -/+
    [r/R] roll -/+      [p/P] pitch -/+     [w/W] yaw -/+
    [+]  step *= 10     [-]  step /= 10     [0] zero all transform params
    [s] save to JSON    [n] switch scene    [q] quit

`save` rewrites the JSON pointed to by `get_package_share_directory`. If that
path is a symlink (--symlink-install), the source file is updated; otherwise
the install share is updated and a clear note is printed.
"""

import json
import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node


TARGET_NODE = '/annotation_viz_node'
TUNING_PARAMS = ('tx', 'ty', 'tz', 'roll_deg', 'pitch_deg', 'yaw_deg')


def _euler_deg_to_quat(roll_deg, pitch_deg, yaw_deg):
    """Intrinsic ZYX (yaw-pitch-roll) Euler angles in degrees → (x,y,z,w).

    Mirrors the helper in annotation_viz_node so the bake matches what the
    viz node currently displays.
    """
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class AnnotationTuner(Node):
    def __init__(self):
        super().__init__('annotation_tuner')
        self._set = self.create_client(SetParameters, f'{TARGET_NODE}/set_parameters')
        self._get = self.create_client(GetParameters, f'{TARGET_NODE}/get_parameters')
        if not self._set.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                f'service {TARGET_NODE}/set_parameters not available — '
                f'is annotation_viz_node running?')
        self._get.wait_for_service(timeout_sec=5.0)

        self.scene_name = self._get_string('scene_name') or 'FireAcademy'
        self.state = {k: 0.0 for k in TUNING_PARAMS}
        # Pull current values so we don't clobber a partially-tuned session.
        for k in TUNING_PARAMS:
            v = self._get_double(k)
            if v is not None:
                self.state[k] = v
        self.pos_step = 0.10
        self.ang_step = 1.0

    # --------------------------------------------------------- service IO

    def _get_string(self, name):
        req = GetParameters.Request()
        req.names = [name]
        fut = self._get.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            return None
        v = fut.result().values[0]
        return v.string_value if v.type == ParameterType.PARAMETER_STRING else None

    def _get_double(self, name):
        req = GetParameters.Request()
        req.names = [name]
        fut = self._get.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            return None
        v = fut.result().values[0]
        return v.double_value if v.type == ParameterType.PARAMETER_DOUBLE else None

    def _set_double(self, name, value):
        p = Parameter()
        p.name = name
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float(value)
        req = SetParameters.Request()
        req.parameters = [p]
        fut = self._set.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            print('  ! set_parameters timed out')
            return False
        res = fut.result().results[0]
        if not res.successful:
            print(f'  ! rejected: {res.reason}')
            return False
        return True

    def _set_string(self, name, value):
        p = Parameter()
        p.name = name
        p.value.type = ParameterType.PARAMETER_STRING
        p.value.string_value = str(value)
        req = SetParameters.Request()
        req.parameters = [p]
        fut = self._set.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            print('  ! set_parameters timed out')
            return False
        res = fut.result().results[0]
        if not res.successful:
            print(f'  ! rejected: {res.reason}')
            return False
        return True

    # ------------------------------------------------------------ helpers

    def _bump(self, name, delta):
        self.state[name] += delta
        self._set_double(name, self.state[name])

    def _reset_all(self):
        for k in TUNING_PARAMS:
            self.state[k] = 0.0
            self._set_double(k, 0.0)

    def _scene_path(self):
        share = get_package_share_directory('gcs_visualizer')
        return os.path.join(share, 'annotations', f'{self.scene_name}.json')

    def _save(self):
        path = self._scene_path()
        if not os.path.exists(path):
            print(f'  ! cannot save — file not found at {path}')
            return
        with open(path, 'r') as f:
            data = json.load(f)

        tx, ty, tz = self.state['tx'], self.state['ty'], self.state['tz']
        gq = _euler_deg_to_quat(self.state['roll_deg'],
                                self.state['pitch_deg'],
                                self.state['yaw_deg'])

        for entry in data:
            bw = entry['bbox_world']
            cx, cy, cz = bw['center_xyz_m']
            bw['center_xyz_m'] = [cx + tx, cy + ty, cz + tz]
            bw['quat_xyzw'] = list(gq)

        with open(path, 'w') as f:
            json.dump(data, f, indent=2)

        real = os.path.realpath(path)
        print(f'  saved → {path}')
        if real != path:
            print(f'    (resolves to source: {real})')
        else:
            print('    NOTE: this is the install share — copy to source repo to '
                  'survive a colcon rebuild.')

        self._reset_all()
        # Trigger the viz node to reload the freshly-baked file.
        self._set_string('scene_name', self.scene_name)

    def _switch_scene(self, new_name):
        if not new_name:
            return
        if not self._set_string('scene_name', new_name):
            return
        self.scene_name = new_name
        # New scene starts from zeroed transform.
        for k in TUNING_PARAMS:
            self.state[k] = 0.0

    def _print_state(self):
        print()
        print(f'== annotation tuner ==   scene: {self.scene_name}')
        print(f'  translate (m):    tx={self.state["tx"]:+8.3f}  '
              f'ty={self.state["ty"]:+8.3f}  tz={self.state["tz"]:+8.3f}')
        print(f'  rotate (deg):   roll={self.state["roll_deg"]:+8.3f}  '
              f'pitch={self.state["pitch_deg"]:+8.3f}  '
              f'yaw={self.state["yaw_deg"]:+8.3f}')
        print(f'  step:  pos={self.pos_step:.3f} m   ang={self.ang_step:.3f} deg')
        print('  [x/X y/Y z/Z]  [r/R p/P w/W]  [+/-] step  [0] reset  '
              '[s] save  [n] scene  [q] quit')

    # --------------------------------------------------------------- loop

    def run(self):
        while True:
            self._print_state()
            try:
                cmd = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if not cmd:
                continue
            c = cmd[0]
            if c == 'q':
                return
            elif c == 'x': self._bump('tx', -self.pos_step)
            elif c == 'X': self._bump('tx', +self.pos_step)
            elif c == 'y': self._bump('ty', -self.pos_step)
            elif c == 'Y': self._bump('ty', +self.pos_step)
            elif c == 'z': self._bump('tz', -self.pos_step)
            elif c == 'Z': self._bump('tz', +self.pos_step)
            elif c == 'r': self._bump('roll_deg', -self.ang_step)
            elif c == 'R': self._bump('roll_deg', +self.ang_step)
            elif c == 'p': self._bump('pitch_deg', -self.ang_step)
            elif c == 'P': self._bump('pitch_deg', +self.ang_step)
            elif c == 'w': self._bump('yaw_deg', -self.ang_step)
            elif c == 'W': self._bump('yaw_deg', +self.ang_step)
            elif c == '+':
                self.pos_step *= 10.0
                self.ang_step *= 10.0
            elif c == '-':
                self.pos_step /= 10.0
                self.ang_step /= 10.0
            elif c == '0': self._reset_all()
            elif c == 's': self._save()
            elif c == 'n':
                new = input('  new scene name: ').strip()
                self._switch_scene(new)
            else:
                print(f'  ? unknown command: {cmd!r}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = AnnotationTuner()
    except RuntimeError as e:
        print(f'error: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
