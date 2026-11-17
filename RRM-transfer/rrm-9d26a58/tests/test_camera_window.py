"""Camera evidence must remain fresh through server discovery, without ROS."""
import copy
from types import SimpleNamespace as NS
import unittest

from rrm.camera_pairing import camera_pair_metadata, camera_window_metadata
from rrm.uav_probe_admission import wait_for_checked_server


def window():
    records = []
    for index in range(3):
        def message(encoding):
            return NS(header=NS(stamp=NS(sec=20, nanosec=index*50_000_000), frame_id='left'),
                      width=160, height=120, encoding=encoding,
                      k=[100., 0., 80., 0., 100., 60., 0., 0., 1.])
        record = camera_pair_metadata(message('rgb8'), message('32FC1'), message(''))
        record['receipt_monotonic_s'] = dict(rgb=10.1+index*.1, depth=10.11+index*.1,
                                              info=10.12+index*.1)
        records.append(record)
    return records


def check(records, now=10.5, **kwargs):
    return camera_window_metadata(records, started_monotonic=10., now_monotonic=now, **kwargs)


class CameraWindowTests(unittest.TestCase):
    def test_recent_progressing_window_reports_bounds_without_mutation(self):
        records = window()
        saved = copy.deepcopy(records)
        result = check(records)
        self.assertEqual(records, saved)
        self.assertAlmostEqual(result['source_span_s'], .1)
        self.assertAlmostEqual(result['max_channel_receipt_age_s'], .4)
        self.assertAlmostEqual(result['window_receipt_span_s'], .22)
        self.assertIn('synchronization unproven', result['freshness_scope'])

    def test_duplicate_reverse_or_inconsistent_source_rejected(self):
        records = window()
        for invalid in ([records[0], records[0], records[2]], records[::-1]):
            with self.assertRaises(ValueError):
                check(invalid)
        for sec, ns in ((0,0), (-1,3), (20,1_000_000_000), (True,3), (20,1.5)):
            values = window()
            for header in values[1]['headers'].values():
                header['stamp'] = dict(sec=sec, nanosec=ns)
            with self.subTest(sec=sec, ns=ns), self.assertRaises(ValueError):
                check(values)
        values = window()
        values[1]['headers']['depth']['stamp']['nanosec'] += 1
        with self.assertRaises(ValueError):
            check(values)

    def test_every_channel_requires_current_receipt_and_strict_age(self):
        for index in range(3):
            for channel in ('rgb','depth','info'):
                for receipt in (9.99, 10.51, True, None, float('nan'), float('inf')):
                    values = window()
                    values[index]['receipt_monotonic_s'][channel] = receipt
                    with self.subTest(index=index, channel=channel, receipt=receipt):
                        with self.assertRaises(ValueError):
                            check(values)
        self.assertIsInstance(check(window(), now=12.09), dict)
        with self.assertRaises(ValueError):
            check(window(), now=12.1)
        with self.assertRaises(ValueError):
            check(window(), now=13.)

    def test_camera_changes_and_malformed_records_rejected(self):
        mutations = [
            lambda r: r.pop('headers'),
            lambda r: r['headers'].pop('info'),
            lambda r: r['receipt_monotonic_s'].pop('depth'),
            lambda r: r.update(exact_stamp_pair=1),
            lambda r: r.update(depth_encoding='16UC1'),
            lambda r: r.update(intrinsic=[200.,0.,80.,0.,100.,60.,0.,0.,1.]),
            lambda r: r.update(intrinsic=[float('nan')]*9),
            lambda r: r.update(intrinsic=[0.]*9),
            lambda r: r.update(image_size=[320,240]),
            lambda r: r.update(image_size=[True,120]),
            lambda r: [header.update(frame_id='different') for header in r['headers'].values()],
        ]
        for mutate in mutations:
            records = window()
            mutate(records[1])
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                check(records)
        for records in (None, {}, [], window()[:2], window()+window(), [None]*3):
            with self.subTest(records=records), self.assertRaises(ValueError):
                check(records)
        for value in (None, True, 0., -1., float('nan'), float('inf')):
            with self.subTest(max_age=value), self.assertRaises(ValueError):
                check(window(), max_age_s=value)
        for now in (9., None, True, float('nan')):
            with self.subTest(now=now), self.assertRaises(ValueError):
                check(window(), now=now)

    def test_expiry_while_waiting_for_server_is_rechecked(self):
        clock = [10.5]
        class Server:
            def wait_for_server(self, *, timeout_sec):
                clock[0] += timeout_sec
                return True
        records = window()
        check(records, now=clock[0])  # A pre-wait check passes.
        with self.assertRaises(ValueError):
            wait_for_checked_server(Server(), lambda: check(records, now=clock[0]))
        self.assertEqual(clock[0], 15.5)
        clock[0] = 10.5
        events = []
        wait_for_checked_server(Server(), lambda: events.append(check(records, now=clock[0])), timeout_sec=.1)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['checked_monotonic_s'], 10.6)

    def test_missing_server_does_not_reach_admission_callback(self):
        class MissingServer:
            def wait_for_server(self, *, timeout_sec):
                return False
        with self.assertRaisesRegex(RuntimeError, 'action_server_missing'):
            wait_for_checked_server(MissingServer(), lambda: self.fail('unavailable server'))


if __name__ == '__main__':
    unittest.main()
