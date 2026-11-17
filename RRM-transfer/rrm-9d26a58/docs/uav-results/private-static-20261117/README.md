# Private static renderer development controls

Captured 2026-11-17 14:37 UTC on branch `ore_proj`, starting at `e442a0cad`,
with the saved renderer and fixture-author bytes. Replayed against the assessed
local-support detector snapshot; [report](replay.json) pins its source hash.

A separate bounded headless Kit process authored private stages and a static
camera. It launched no Pegasus vehicle, ROS graph, PX4 or physics stepping.
The existing live Kit PID210 and PX4 PID978 retained their original start times
at the later process check. No flight, live stage change or deployment occurred.
Three separately rendered frames per case are retained; depth arrays are identical
within each static case. They do not represent independent trials or establish
robot-camera acquisition synchronization. This is development evidence, not a
held-out perception accuracy estimate.

| Case | Initial total-spread detector | Local-support detector | Viewpoint |
| --- | --- | --- | --- |
| Nominal | DEPTH_INCONSISTENT | OBSERVED | SELECTED |
| Absent | ABSENT | ABSENT | ABSTAIN |
| Duplicate red | AMBIGUOUS | AMBIGUOUS | ABSTAIN |
| Split occlusion | AMBIGUOUS | AMBIGUOUS | ABSTAIN |
| Partial occlusion | DEPTH_INCONSISTENT | DEPTH_INCONSISTENT | ABSTAIN |
| Full occlusion | ABSENT | ABSENT | ABSTAIN |

The nominal cube's front and underside span0.401689m; its largest adjacent depth
step is0.091806m, one connected support. Partial cover's largest step is0.278654m,
including pixels at the occluder boundary. That abstention does not establish
general occlusion detection. Coherent partial coverage can still pass.

![Nominal private view](nominal/frame-0.png)

RGB PNGs and NPZs (`depth_m`, `intrinsic`) are the actual detector inputs.
Per-frame camera JSON, USD stages and authored fixture manifests are saved.
`audit.py` verifies all54input hashes, image shapes, pinhole projection/aperture
agreement and current detector dispositions; it also recomputes the failed baseline.
Run from the RRM root with its dependencies installed:

```bash
PYTHONPATH=. python docs/uav-results/private-static-20261117/audit.py
```

`renderer.py`, `fixture-author.py`, `baseline-uav-marker.py` and
`assessed-uav-marker.py` are evidence snapshots, not live scene entry points.
Fixture manifests retain their preparation-time `rendered_qualified=false` field;
the measured result is the separate replay report, not a rewritten intent label.

Two preceding attempts are preserved in the local experiment record: first saved
only nominal, and the next exposed annotator cleanup receiving a render-product
object where this SDK requires its path. Kit's default `app.close()` can exit0
even after an exception, so exit status alone did not count as capture success.
Final capture required all six cases, `capture_complete=true`, no error record and
terminal exit0. Final SDK warnings included shared KV database locking (private
process disabled that cache), unknown camera projection defaulting to pinhole,
DLSS minimum internal resolution, texture-copy performance and stage-reference
cleanup warnings. Saved camera metadata reports pinhole and the expected480×300
output; these checks do not qualify SDK internals or the running viewer.
