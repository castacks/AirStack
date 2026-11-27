# Static Isaac annotator live diagnostic R3

27 November 2026 UTC. **INCONCLUSIVE.** The launcher failed its preflight mount comparison and stopped before `docker start`. Structured Docker evidence shows that the two expected bind mounts were correct. Isaac never started, so native frame association and durable shutdown remain unverified. No retry was made. [Machine-readable gate results](static-isaac-annotator-live-r3/verification.json)

## Preflight and failure

The checkout matched `7bce93c81f8751aa12fb12b855cc70d9204d8d92` on `ore-ms-orb-development`. The 17 R3 and 10 R2 CPU tests passed again. The launcher pinned the R1 isolated USD stage, exact committed worker and supervisor sources, and the locally cached Isaac Sim 5.1.0 image. The image manifest was `sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697` and the config was `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`. The worker's eight-attempt bound and 150-second supervisor timeout were retained. [Offline qualification and launcher procedure](STATIC_ISAAC_ANNOTATOR_READINESS_R3.md)

The launcher created one disposable container and saved its stopped configuration. The saved record shows the expected image, no network, private IPC, and exactly the declared read-only input and writable capture mounts. The shell mount comparison at `launch-once.sh:19` returned false. A read-only formatter check showed that Docker adds a final newline after the row newlines emitted by the template. Reconstructing the launcher comparison from the saved mount tuples reproduced the leading blank line after sorting. Comparing the structured tuples gave an exact match. The failure was in preflight formatting. No worker phase was reached.

| Gate | Outcome |
| --- | --- |
| Source, cached image and fixture provenance | Passed |
| Launcher mount assertion | Failed on an extra blank line |
| Native annotator readiness, common frame and resolved simulation time | Unexecuted |
| Raw paired buffers, worker lifecycle and native shutdown | Unexecuted |
| Supervisor | Returned `1` and wrote `FAILED.json`. No completion marker. |
| Operational isolation and retained evidence integrity | Passed |

The pre-start marker is absent. The diagnostic container was still in `created` state with unset start and finish times when it was inspected and removed. Its default exit-code field of zero carries no worker outcome. There are no readiness records, raw frames or lifecycle journal because no simulator process ran. The operational AirStack snapshots matched byte for byte, including container and image IDs, start times, running state and restart counts. The R1 and R2 evidence manifests and frozen M05–M08 checksums passed before and after.

## Retained evidence and next dependency

The evidence directory is `/root/ore-ms-artifacts/static-isaac-annotator-live-r3.Jh3AprK0`. The exact launcher, created and final container records, supervisor traceback, child launcher log, verification of the mount comparison, and operational snapshots are retained there. All 29 files pass `SHA256SUMS.txt`. The manifest SHA-256 is `c4016c895626758dfeda4e5e9c2577f7d36f6f313ed45bf0196687c26a3d4d05`.

Replace the formatted-text comparison with a structured check of mount type, source, destination and writability. Validate it offline against the retained mount record and against missing, extra and writable-input cases. Then obtain separate authorization for one R3 live diagnostic to verify a common native frame and recorder shutdown before returning to the 5 m and 15 m controls. The qualified R3 worker and supervisor sources were preserved.
