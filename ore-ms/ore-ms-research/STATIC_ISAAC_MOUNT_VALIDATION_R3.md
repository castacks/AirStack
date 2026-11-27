# Static Isaac structured mount validation R3

27 November 2026 UTC. **PASS offline.** The corrected R3 launcher is ready for a separately authorized diagnostic. Native frame acquisition and durable shutdown remain live gates.

## Failed preflight

The earlier launcher formatted each Docker mount as a text row ending in a newline. Docker added a final newline, and `sort` moved the resulting empty row ahead of the mount rows. At retained `launch-once.sh:19`, `test "$actual" = "$expected"` rejected the extra leading newline. The saved structured records confirm that both required bind mounts had the correct source, destination and access mode. The container never started. [Live diagnostic report and retained evidence paths](STATIC_ISAAC_ANNOTATOR_LIVE_R3.md) · [Prior gate results and hashes](static-isaac-annotator-live-r3/verification.json)

## Correction

The launcher retains `docker inspect --format '{{json .Mounts}}'` as `container-mounts.json` and validates the JSON before asking Docker to start. The validator requires one read-only bind from the scratch `input` directory to `/capture-input` and one read-write bind from scratch `run/capture` to `/capture-output`. It validates the boolean access flag independently of display text. It rejects incomplete or invalid records, wrong types, path or access mismatches, duplicate destinations, and mounts that overlap a protected source or destination. JSON whitespace and mount order are immaterial. [Validator](static-isaac-annotator-r3/validate_mounts.py)

The reusable checker accepts well-formed unrelated records. The launcher passes `--exact` to retain the original isolation policy of exactly two scratch mounts. It still checks the pinned image, named stopped container, no network, private IPC, and disabled privileged mode. The validator is included in the pinned input manifest and runs under the original supervisor timeout. [Updated launcher procedure](STATIC_ISAAC_ANNOTATOR_READINESS_R3.md#one-frame-procedure-for-separate-authorization)

## Offline verification

| Check | Result |
| --- | --- |
| Replay the retained Docker mount array | Accepted in exact mode |
| Structured mount and mock launcher tests | 10 passed |
| R3 readiness regressions | 17 passed |
| R2 supervisor regressions | 10 passed |
| Runbook and launcher shell syntax, research documentation checks | Passed |
| Retained R1, R2 and R3 preflight manifests, frozen M05–M08 package and plan checksums | Passed |

The launcher tests replace Docker with a local test double. They verify that valid reordered records with blank lines reach the simulated start call and that a writable input mount stops before that call. The worker, readiness logic, eight-attempt bound, native `ReferenceTime` comparison, simulation-time resolution, shutdown gate, and completion-marker behavior retain their exact source from `221969215`. No real Docker command or Isaac process ran during qualification. [Deterministic tests](static-isaac-annotator-r3/test_validate_mounts.py)

Run the tests offline from the AirStack root.

```bash
research=ore-ms/ore-ms-research
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$research/static-isaac-recorder-r2" \
  python3 -m unittest discover -v -s "$research/static-isaac-annotator-r3"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -v -s "$research/static-isaac-recorder-r2"
python3 ore-ms/maintenance/check_docs.py
```
