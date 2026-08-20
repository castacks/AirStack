import unittest
from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import Image

from mononav_bridge.bridge_node import MonoNavBridge, _transform_matrix


class BridgeHelpersTest(unittest.TestCase):
    def test_transform_matrix_identity_rotation_and_translation(self):
        transform = SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=-2.0, z=3.5),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        expected = np.eye(4)
        expected[:3, 3] = [1.0, -2.0, 3.5]
        np.testing.assert_allclose(_transform_matrix(transform), expected)

    def test_rgb8_image_conversion_ignores_row_padding_and_returns_bgr(self):
        message = Image()
        message.width = 2
        message.height = 1
        message.encoding = "rgb8"
        message.step = 8
        message.data = bytes([255, 0, 0, 0, 255, 0, 123, 123])
        converted = MonoNavBridge._image_to_bgr(message)
        np.testing.assert_array_equal(
            converted,
            np.asarray([[[0, 0, 255], [0, 255, 0]]], dtype=np.uint8),
        )

    def test_32fc1_depth_conversion_ignores_padding_and_sanitizes_nonfinite(self):
        message = Image()
        message.width = 2
        message.height = 1
        message.encoding = "32FC1"
        message.step = 12
        message.data = np.asarray([1.25, np.inf, 99.0], dtype=np.float32).tobytes()
        converted = MonoNavBridge._depth_to_meters(message)
        np.testing.assert_allclose(
            converted, np.asarray([[1.25, 0.0]], dtype=np.float32)
        )
