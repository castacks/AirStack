"""CLI safety-gate tests for the explicit live mission runner."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
import rrm_authorized_live_mission as runner


class AuthorizedLiveMissionRunnerTests(unittest.TestCase):
    def test_execution_requires_both_explicit_flags_before_any_runtime_setup(self):
        base = ["rrm_authorized_live_mission.py", "--context", "context.json", "--scene-manifest", "scene.json",
                "--entity-catalog", "catalog.json", "--authorization", "authorization.json",
                "--worker-url", "http://worker", "--run-dir", "new-run"]
        for flags in (("--execute",), ("--simulator-only",)):
            with self.subTest(flags=flags), patch.object(sys, "argv", [*base, *flags]):
                with self.assertRaisesRegex(SystemExit, "requires both"):
                    runner.main()

    def test_default_parser_is_shadow_only(self):
        parsed = runner.parser().parse_args([
            "--context", "context.json", "--scene-manifest", "scene.json", "--entity-catalog", "catalog.json",
            "--authorization", "authorization.json", "--worker-url", "http://worker", "--run-dir", "new-run",
        ])
        self.assertFalse(parsed.execute)
        self.assertFalse(parsed.simulator_only)


if __name__ == "__main__":
    unittest.main()
