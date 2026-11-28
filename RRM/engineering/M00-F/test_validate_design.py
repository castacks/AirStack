"""Synthetic positive and mutation fixtures exercise fail-closed boundary classes."""
import unittest
from pathlib import Path
from validate_design import read_design, validate

HERE = Path(__file__).resolve().parent


class DesignPolicyTests(unittest.TestCase):
    def test_fixture_outcomes(self):
        for path in sorted((HERE / 'fixtures').glob('*.json')):
            with self.subTest(fixture=path.name):
                if path.name == 'duplicate_key.json':
                    with self.assertRaises(ValueError):
                        read_design(path)
                else:
                    self.assertEqual(not validate(read_design(path)), path.name == 'positive.json')

    def test_missing_every_required_field(self):
        source = read_design(HERE / 'fixtures/positive.json')
        for key in source:
            with self.subTest(missing=key):
                self.assertTrue(validate({k: v for k, v in source.items() if k != key}))

    def test_malformed_root(self):
        for value in [None, [], True, 'compose text']:
            with self.subTest(root=value):
                self.assertTrue(validate(value))

    def test_proposal_static_only(self):
        self.assertFalse(validate(read_design(HERE / 'M00F_NONEXECUTABLE_POLICY.json')))


if __name__ == '__main__':
    unittest.main()
