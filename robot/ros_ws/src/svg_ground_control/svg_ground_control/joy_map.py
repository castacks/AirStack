"""Find the axis/button numbers for a gamepad, to configure xbox_teleop.

Subscribes to sensor_msgs/Joy and prints only on change, so you wiggle one
control at a time and read the number off. Axis indices vary by pad and by
driver mode, so xbox_teleop's defaults are a guess — measure yours.

    ros2 run joy joy_node &
    ros2 run svg_ground_control joy_map

Buttons print both down and up with a press count, so a bouncing button is
visible instead of looking like a mapping bug. Ctrl-C prints a summary.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

MOVE_THRESHOLD = 0.25   # ignore stick slop / noise
REPRINT_DELTA = 0.10    # don't scroll while a stick is held
TRIGGER_REST = 0.9      # an axis resting past this is an analog trigger


class JoyMap(Node):

    def __init__(self):
        super().__init__('joy_map')
        self.rest = None
        self.last_printed = {}
        self.seen = {}              # axis -> [min, max] observed
        self.button_state = {}
        self.press_count = {}

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        print('\nWiggle ONE control at a time. Ctrl-C when done.\n')

    def _report_rest(self, msg: Joy):
        self.rest = list(msg.axes)
        print(f'{len(msg.axes)} axes, {len(msg.buttons)} buttons')
        print('rest: ' + ', '.join(f'[{i}]={v:+.2f}'
                                   for i, v in enumerate(msg.axes)))
        triggers = [i for i, v in enumerate(msg.axes) if abs(v) > TRIGGER_REST]
        if triggers:
            print(f'\n  WARNING: axes {triggers} sit at full scale untouched '
                  '(analog triggers).')
            print('  Using one as a velocity axis commands full speed with '
                  'nothing held.')
        print()

    def joy_callback(self, msg: Joy):
        if self.rest is None:
            self._report_rest(msg)
            return

        for i, value in enumerate(msg.axes):
            rest = self.rest[i] if i < len(self.rest) else 0.0
            if abs(value - rest) <= MOVE_THRESHOLD:
                continue
            if abs(value - self.last_printed.get(i, rest)) < REPRINT_DELTA:
                continue
            self.last_printed[i] = value
            lo, hi = self.seen.get(i, [value, value])
            self.seen[i] = [min(lo, value), max(hi, value)]
            direction = 'positive' if value > rest else 'negative'
            print(f'  AXIS {i:<2} {value:+.2f}  {direction:<8} '
                  + '#' * int(abs(value) * 20))

        for i, raw in enumerate(msg.buttons):
            was = self.button_state.get(i, 0)
            if raw and not was:
                self.press_count[i] = self.press_count.get(i, 0) + 1
                print(f'  BUTTON {i} down   (press #{self.press_count[i]})')
            elif was and not raw:
                print(f'  BUTTON {i} up')
            self.button_state[i] = raw

    def summary(self):
        if not self.seen and not self.press_count:
            print('\nNothing moved — is joy_node publishing /joy?')
            return
        print('\n--- axes that moved ---')
        for i in sorted(self.seen):
            lo, hi = self.seen[i]
            print(f'  axis {i}: {lo:+.2f} .. {hi:+.2f}')
        if self.press_count:
            print('--- buttons pressed ---')
            for i in sorted(self.press_count):
                print(f'  button {i}: {self.press_count[i]} press(es)')


def main(args=None):
    rclpy.init(args=args)
    node = JoyMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
