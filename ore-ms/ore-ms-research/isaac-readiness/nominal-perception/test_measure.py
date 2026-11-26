import struct
import unittest
from types import SimpleNamespace as Record

from measure import pairing, payload, timing


class MetricsTest(unittest.TestCase):
    def test_rates_and_gap(self):
        stamps = [10_000_000, 20_000_000, 30_000_000, 60_000_000]
        arrivals = [stamp * 2 for stamp in stamps]
        result = timing(stamps, arrivals)
        self.assertTrue(result["strict_positive_stamps"])
        self.assertEqual(result["sim"]["hz"], 60)
        self.assertEqual(result["wall"]["hz"], 30)
        self.assertEqual(result["sim"]["interval_ms"]["p95"], 30)
        self.assertEqual(result["sim"]["gaps_over_1_5_median"], 1)

    def test_nonmonotonic_and_empty(self):
        for stamps in ([], [0, 1], [5, 5], [5, 4]):
            result = timing(stamps, stamps)
            self.assertFalse(result["strict_positive_stamps"])
            self.assertIsNone(result["sim"]["hz"])

    def test_pairing_unique_and_nearest(self):
        pairs = pairing([1, 1, 2_000_001, 6_000_001], [1, 4_000_001])
        self.assertEqual(pairs["exact_pairs"], 1)
        self.assertEqual(pairs["left_unmatched_unique"], 2)
        self.assertEqual(pairs["right_unmatched_unique"], 1)
        self.assertEqual(pairs["left_to_nearest_right_abs_ms"]["max"], 2)
        self.assertIsNone(pairing([1], [])["left_to_nearest_right_abs_ms"])

    def test_padded_big_endian_depth_and_disparity(self):
        image = Record(width=2, height=2, step=12, encoding="32FC1",
                       is_bigendian=True, data=struct.pack(">ff4xff4x", 1, -1, 3, float("nan")))
        self.assertEqual(payload("depth", image)["valid_fraction"], 0.5)
        disparity = Record(image=image, min_disparity=0, max_disparity=2)
        self.assertEqual(payload("disparity", disparity)["valid_fraction"], 0.25)

    def test_cloud_xyz_offsets_and_truncation(self):
        cloud = Record(width=1, height=2, row_step=20, point_step=16, is_bigendian=False,
                       fields=[Record(name=axis, datatype=7, count=1, offset=offset)
                               for axis, offset in zip("xyz", (4, 8, 12))],
                       data=struct.pack("<4xfff4x4xfff4x", 1, 2, 3, 4, float("inf"), 6))
        self.assertEqual(payload("cloud", cloud)["valid_fraction"], 0.5)
        cloud.data = cloud.data[:-1]
        with self.assertRaisesRegex(ValueError, "strides"):
            payload("cloud", cloud)


if __name__ == "__main__":
    unittest.main()
