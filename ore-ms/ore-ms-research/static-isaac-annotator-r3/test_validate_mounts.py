"""CPU-only checks for the R3 launcher's structured mount gate."""

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

from validate_mounts import validate_mounts

INPUT = "/evidence/input"
OUTPUT = "/evidence/run/capture"
HERE = Path(__file__).resolve().parent
RUNBOOK = HERE.parent / "STATIC_ISAAC_ANNOTATOR_READINESS_R3.md"


def mount(source, destination, writable, kind="bind"):
    return {"Type": kind, "Source": source, "Destination": destination, "RW": writable}


def approved(input_source=INPUT, output_source=OUTPUT):
    return [mount(input_source, "/capture-input", False),
            mount(output_source, "/capture-output", True)]


class MountTests(unittest.TestCase):
    def test_correct_mounts_in_either_order(self):
        for records in (approved(), approved()[::-1]):
            validate_mounts(records, INPUT, OUTPUT, exact=True)

    def test_cli_accepts_whitespace_and_blank_lines(self):
        with TemporaryDirectory() as root:
            inspection = Path(root) / "mounts.json"
            inspection.write_text("\n\n  " + json.dumps(approved()[::-1], indent=4) + "  \n\n")
            result = subprocess.run([sys.executable, str(HERE / "validate_mounts.py"),
                                     str(inspection), INPUT, OUTPUT, "--exact"],
                                    capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_required_mount(self):
        with self.assertRaisesRegex(ValueError, "missing required mount.*capture-output"):
            validate_mounts(approved()[:1], INPUT, OUTPUT)

    def test_incorrect_source_destination_or_type(self):
        for records in ([mount("/incorrect", "/capture-input", False), approved()[1]],
                        [mount(INPUT, "/wrong-target", False), approved()[1]],
                        [mount(INPUT, "/capture-input", False, "volume"), approved()[1]]):
            with self.subTest(records=records), self.assertRaises(ValueError):
                validate_mounts(records, INPUT, OUTPUT)

    def test_wrong_rw_mode(self):
        for index in (0, 1):
            records = approved()
            records[index]["RW"] = not records[index]["RW"]
            with self.subTest(index=index), self.assertRaisesRegex(ValueError, "incorrect"):
                validate_mounts(records, INPUT, OUTPUT)

    def test_duplicate_and_conflicting_mounts(self):
        for added in (approved()[0], mount("/different", "/capture-output", False),
                      mount("/shadow", "/capture-input/temp", True),
                      mount("/shadow", "/", True),
                      mount("/evidence", "/alias", False)):
            with self.subTest(added=added), self.assertRaisesRegex(ValueError, "duplicate|conflict"):
                validate_mounts(approved() + [added], INPUT, OUTPUT)

    def test_malformed_or_incomplete_data(self):
        wrong_flag = dict(approved()[0], RW="false")
        missing_source = dict(approved()[0])
        del missing_source["Source"]
        for records in (None, {"Mounts": approved()}, ["not a mount"], [missing_source],
                        [wrong_flag], [dict(approved()[0], Destination="/other/../capture-input")]):
            with self.subTest(records=records), self.assertRaises(ValueError):
                validate_mounts(records, INPUT, OUTPUT)
        with TemporaryDirectory() as root:
            inspection = Path(root) / "mounts.json"
            inspection.write_text("[{\n")
            result = subprocess.run([sys.executable, str(HERE / "validate_mounts.py"),
                                     str(inspection), INPUT, OUTPUT],
                                    capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 1)
        self.assertIn("Mount preflight failed", result.stderr)

    def test_unrelated_mounts_are_accepted_by_checker(self):
        records = approved() + [mount("/cache", "/logs", True), mount("", "/tmp", True, "tmpfs")]
        validate_mounts(records, INPUT, OUTPUT)
        with self.assertRaisesRegex(ValueError, "unexpected mount"):
            validate_mounts(records, INPUT, OUTPUT, exact=True)


class LauncherTests(unittest.TestCase):
    def run_launcher(self, *, writable_input):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            source, capture = root / "input", root / "run" / "capture"
            source.mkdir()
            capture.mkdir(parents=True)
            validator = shutil.copyfile(HERE / "validate_mounts.py", source / "validate_mounts.py")
            checksum = hashlib.sha256(validator.read_bytes()).hexdigest()
            (root / "input.sha256").write_text(f"{checksum}  validate_mounts.py\n")
            (root / "image.txt").write_text("fixture-image-reference\nfixture-image-config\n")
            launcher = root / "launch-once.sh"
            launcher.write_text(RUNBOOK.read_text().split("<<'SH'\n", 1)[1].split("\nSH\n", 1)[0])
            records = approved(str(source), str(capture))
            records[0]["RW"] = writable_input
            mock = root / "docker"
            header = f"fixture-image-config /ore-{root.name} created none private false"
            mock.write_text(f"""#!{sys.executable}
import sys

if sys.argv[1] == "create":
    print("fixture-container")
elif sys.argv[1] == "inspect":
    template = sys.argv[sys.argv.index("--format") + 1]
    expected_mounts = {json.dumps(records[::-1], indent=2)!r}
    if '"restart_count"' in template:
        print('{{"id":"fixture-container","restart_count":0,"state":{{"Status":"exited"}}}}')
    elif template == "{{{{json .Mounts}}}}":
        print("\\n" + expected_mounts + "\\n")
    elif template.endswith("{{{{json .Mounts}}}}"):
        print({header!r} + " " + expected_mounts)
    else:
        print({header!r})
elif sys.argv[1:] == ["start", "--attach", "fixture-container"]:
    print("mock start reached")
else:
    raise SystemExit("unexpected fake Docker command")
""")
            mock.chmod(0o700)
            env = {**os.environ, "PATH": f"{root}{os.pathsep}{os.environ['PATH']}"}
            result = subprocess.run(["bash", str(launcher)], env=env, capture_output=True, text=True, timeout=5)
            self.assertEqual(json.loads((root / "container-mounts.json").read_text()), records[::-1])
            exit_receipt = root / "run/container-exit.json"
            self.assertEqual(exit_receipt.exists(), not writable_input)
            if not writable_input:
                self.assertEqual(json.loads(exit_receipt.read_text())["id"], "fixture-container")
            return result

    def test_launcher_reaches_mock_start_only_after_valid_mounts(self):
        result = self.run_launcher(writable_input=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Structured mount validation passed\nmock start reached", result.stdout)

    def test_launcher_blocks_mock_start_on_mount_failure(self):
        result = self.run_launcher(writable_input=True)
        self.assertEqual(result.returncode, 1)
        self.assertIn("incorrect type, source or access for /capture-input", result.stderr)
        self.assertNotIn("mock start reached", result.stdout)


if __name__ == "__main__":
    unittest.main()
