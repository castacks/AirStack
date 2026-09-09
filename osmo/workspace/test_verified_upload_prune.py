import json
from pathlib import Path
import tempfile
import subprocess
import unittest
from unittest.mock import patch
import mission_runner as mr


class PruneTests(unittest.TestCase):
    def test_verification_failure_keeps_artifacts(self):
        for rc, output in [(0, '>fcs....... bags/gcs.mcap\n'), (23, '')]:
            with self.subTest(rc=rc), tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp)
                p=root/'mission'/'timestamp'/'iter_001'
                p.mkdir(parents=True)
                (p/'iteration.json').write_text('{"status":"passed"}')
                (p/'bag.mcap').write_bytes(b'valuable')
                env={'OSMO_MISSION_UPLOAD_DEST':'/backup', 'AIRLAB_STORAGE_USER':'test',
                     'AIRLAB_STORAGE_PASS':'test', 'OSMO_MISSION_NO_UPLOAD':'false',
                     'OSMO_UPLOAD_PER_ITERATION':'true', 'OSMO_PRUNE_UPLOADED_BAGS':'always'}
                results=[subprocess.CompletedProcess([],0,'',''),
                         subprocess.CompletedProcess([],rc,output,'')]
                with patch.dict(mr.os.environ,env), patch.object(mr,'_ensure_sshpass',return_value=True), patch.object(mr.subprocess,'run',side_effect=results):
                    self.assertFalse(mr.upload_iteration(p,root))
                self.assertEqual((p/'bag.mcap').read_bytes(),b'valuable')

    def test_preserves_receipt_and_metadata(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'iter_001__scene__method'
            p.mkdir()
            (p / 'iteration.json').write_text('{"status":"passed"}')
            (p / 'bags').mkdir()
            (p / 'bags' / 'gcs.mcap').write_bytes(b'payload')
            mr._prune_verified_iteration(p, 'host:/backup/run/')
            self.assertEqual({x.name for x in p.iterdir()}, {'iteration.json','upload_receipt.json'})
            self.assertTrue(json.loads((p/'upload_receipt.json').read_text())['artifacts_pruned'])
            with patch.object(mr.subprocess, 'run', side_effect=AssertionError('must not reupload')):
                self.assertTrue(mr.upload_iteration(p))

    def test_failed_iteration_never_pruned(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'iter_001'
            p.mkdir()
            (p/'iteration.json').write_text('{"status":"failed"}')
            (p/'bag.mcap').write_bytes(b'data')
            with self.assertRaises(ValueError):
                mr._prune_verified_iteration(p, 'host:/backup/')
            self.assertTrue((p/'bag.mcap').exists())

    def test_symlink_target_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'iter_link'
            p.symlink_to(tmp, target_is_directory=True)
            with self.assertRaises(ValueError):
                mr._prune_verified_iteration(p,'host:/backup/')


if __name__=='__main__':
    unittest.main()
