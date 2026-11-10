import tempfile
import unittest
from pathlib import Path

from run_native_information_weights import deny_held_labels


class FoldIOTests(unittest.TestCase):
    def test_held_reference_and_prior_scores_blocked_training_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            held = root/'held'
            held.mkdir()
            (held/'000000.txt').write_text('held')
            train = root/'training.txt'
            train.write_text('training')
            scores = root/'scores-old.json'
            scores.write_text('previous outcomes')
            with deny_held_labels(held):
                self.assertEqual(train.read_text(), 'training')
                with self.assertRaises(ValueError):
                    (held/'000000.txt').read_text()
                with self.assertRaises(ValueError):
                    open(held/'000000.txt')
                with self.assertRaises(ValueError):
                    scores.read_text()
            self.assertEqual((held/'000000.txt').read_text(), 'held')


if __name__ == '__main__':
    unittest.main()
