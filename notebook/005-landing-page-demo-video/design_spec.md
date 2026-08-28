# 005 — Landing page demo video (+ per-section docs videos)

## Problem context

The docs landing page (`docs/overrides/home.html`) uses a 37 MB animated GIF
(`docs/assets/media/splash-background.gif`) as its hero background — slow to
load and it doesn't show off current AirStack capabilities. Andrew wants a new
hero video showing the best of AirStack: **multi-agent flight, Foxglove GCS,
and cross-simulator (Isaac Sim + MS AirSim) usage**, with drones running real
autonomy (takeoff, trajectories/navigation).

Additionally (mid-task request): the individual clips should also be embedded
in their respective docs sections — Foxglove clip under the GCS/Foxglove docs,
Isaac clip under Isaac Sim docs, MS AirSim clip under its docs. If files are
too large for the repo, Andrew can upload to YouTube for embedding.

## Proposed implementation

1. **Isaac Sim footage** — `airstack up --sim isaac --robots 3 --scene
   construction-site --play` (Nucleus stage). Fly all 3 drones: parallel
   `TakeoffTask` to staggered altitudes, then concurrent `FixedTrajectoryTask`
   (Circle / Figure8 / Racetrack). Record the Isaac viewport via ffmpeg
   x11grab of the fullscreened window (EWMH control via python-xlib helper).
2. **Foxglove footage** — same run; the GCS container auto-launches Foxglove
   Studio with the 3-robot layout. Record its window during a second flight
   pass.
3. **MS AirSim footage** — `airstack up --sim airsim --robots 2 --scene
   neighborhood --play` (rebuilt alpha.12 image so the scene-fetching
   entrypoint is baked). Record the UE4 window during takeoff + trajectories.
4. **Post-production** — ffmpeg: trim best segments, scale to 1920×1080,
   crossfade-splice into one ~30–45 s hero video; encode H.264 (CRF ~28,
   faststart) + poster JPEG; target well under 10 MB. Per-section clips
   encoded individually (~2–5 MB each).
5. **Docs integration** — replace the GIF `<img>` in `home.html` with a muted
   autoplay looping `<video>`; embed per-section clips in
   `docs/simulation/isaac_sim/index.md`, MS AirSim docs, and
   `docs/gcs/foxglove.md`.

Note: local images retagged v0.20.0-alpha.11 → v0.20.0-alpha.12 (only diff in
range is the ms-airsim entrypoint, rebuilt separately) to avoid full rebuilds.

## Test plan

- **a-isaac-footage** — 3-drone flight recorded; drones visible and flying
  trajectories; no UI occlusion; ≥20 s usable footage.
- **b-foxglove-footage** — Foxglove fleet layout with live data recorded.
- **c-msairsim-footage** — UE4 scene with flying drone(s) recorded.
- **d-hero-video** — spliced video plays, loops, < ~10 MB, 1080p.
- **e-docs-integration** — `airstack docs` builds; landing page shows video;
  per-section pages show their clips.
