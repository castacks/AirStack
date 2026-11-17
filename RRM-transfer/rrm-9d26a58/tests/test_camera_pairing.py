"""Exact camera header evidence without ROS or optional image libraries."""
import copy
from types import SimpleNamespace as NS
import unittest

from rrm.camera_pairing import camera_pair_metadata


def messages():
    def message(encoding):
        return NS(header=NS(stamp=NS(sec=10, nanosec=20), frame_id='camera_left'),
                  width=160, height=120, encoding=encoding,
                  k=[100., 0., 80., 0., 100., 60., 0., 0., 1.])
    return [message('rgb8'), message('32FC1'), message('')]


class CameraPairingTests(unittest.TestCase):
    def test_metadata_preserves_each_header_without_aliasing(self):
        values = messages()
        before = copy.deepcopy(values)
        result = camera_pair_metadata(*values)
        self.assertEqual(values, before)
        self.assertEqual(set(result['headers']), {'rgb', 'depth', 'info'})
        self.assertEqual(result['image_size'], [160, 120])
        self.assertEqual(result['depth_encoding'], '32FC1')
        values[1].header.stamp.sec = 30
        values[2].k[0] = 300.
        self.assertEqual(result['headers']['depth']['stamp']['sec'], 10)
        self.assertEqual(result['intrinsic'][0], 100.)

    def test_each_channel_mismatch_rejected(self):
        for channel in range(3):
            for field in ('frame', 'stamp', 'width', 'height'):
                with self.subTest(channel=channel, field=field):
                    values = messages()
                    target = values[channel]
                    if field == 'frame':
                        target.header.frame_id = 'another_camera'
                    elif field == 'stamp':
                        target.header.stamp.nanosec += 1
                    else:
                        setattr(target, field, getattr(target, field)+1)
                    with self.assertRaises(ValueError):
                        camera_pair_metadata(*values)

    def test_invalid_equal_stamps_are_not_evidence(self):
        for sec, nanosec in ((0, 0), (-1, 2), (10, -1), (10, 1_000_000_000),
                             (True, 1), (10, 1.5)):
            with self.subTest(sec=sec, nanosec=nanosec):
                values = messages()
                for value in values:
                    value.header.stamp = NS(sec=sec, nanosec=nanosec)
                with self.assertRaises(ValueError):
                    camera_pair_metadata(*values)

    def test_invalid_dimensions_frames_units_and_calibration(self):
        for case in ('zero_width', 'bool_height', 'empty_frame', 'spaces_frame',
                     'integer_depth', 'empty_rgb_encoding', 'nan_k', 'bool_k', 'short_k'):
            with self.subTest(case=case):
                values = messages()
                if case in ('zero_width', 'bool_height', 'empty_frame', 'spaces_frame'):
                    for value in values:
                        if case == 'zero_width':
                            value.width = 0
                        elif case == 'bool_height':
                            value.height = True
                        else:
                            value.header.frame_id = '' if case == 'empty_frame' else '  '
                elif case == 'integer_depth':
                    values[1].encoding = '16UC1'
                elif case == 'empty_rgb_encoding':
                    values[0].encoding = ''
                elif case == 'short_k':
                    values[2].k.pop()
                else:
                    values[2].k[0] = float('nan') if case == 'nan_k' else True
                with self.assertRaises(ValueError):
                    camera_pair_metadata(*values)


if __name__ == '__main__':
    unittest.main()
