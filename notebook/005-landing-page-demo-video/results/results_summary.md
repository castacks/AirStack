# Results — 005 landing page demo video

All footage was captured live on 2026-08-24 on wildfire-inferno (RTX 5090,
Xorg `:1` at 2560×1440) via ffmpeg x11grab of the fullscreened sim/Foxglove
windows, flying the real autonomy stack (`TakeoffTask`, `FixedTrajectoryTask`,
`LandTask` action goals — the same interface the system tests use).

## Deliverables (committed under `docs/assets/media/`)

| File | Size | Content |
|------|------|---------|
| `splash-video.mp4` | 4.7 MB | 34 s hero loop, 1080p CRF28: Isaac 3-drone takeoff → Isaac trajectories → Foxglove live traces → AirSim Figure-8 between houses → AirSim street chase. 0.5 s crossfades. Replaces the 37 MB `splash-background.gif` (deleted) — ~8× smaller. |
| `splash-poster.jpg` | 88 KB | Poster frame for instant first paint |
| `isaac_sim_demo.mp4` | 2.2 MB | 38 s, 720p — embedded in `docs/simulation/isaac_sim/index.md` |
| `foxglove_demo.mp4` | 0.6 MB | 40 s, 720p — embedded in `docs/gcs/foxglove.md` |
| `ms_airsim_demo.mp4` | 3.6 MB | 30 s, 720p — embedded in `docs/simulation/ms-airsim/index.md` |

No YouTube needed — everything is small enough to commit.

## a — Isaac footage ✅

`airstack up --sim isaac --robots 3 --scene full-warehouse --play`,
`ISAAC_SIM_FOLLOW_CAM=1 ISAAC_SIM_FOLLOW_CAM_OFFSET=-8,-8,1.5`, Kit F11
fullscreen. Keeper: `isaac_warehouse_take5.mp4`. Flight log: 8/11 action goals
SUCCEEDED (3 land goals raced still-active trajectories — cosmetic, drones
landed after recording).

![3 drones on the warehouse floor, fullscreen viewport](a-isaac-footage/frame_spawn_fullscreen.png)
![drones airborne with the follow camera tracking](a-isaac-footage/frame_flight.png)

Scene/take journal:
- `construction-site`, `retro-neighborhood`, `abandoned-warehouse-day` all
  rejected: drones spawn at the world origin, which in these cm-authored
  Nucleus stages is unlit/enclosed (black viewport) — and on warehouse-day
  robots 1–2 never got a PX4 EKF origin. The NVIDIA `full-warehouse` catalog
  scene is lit, flat, and open at the origin.
- takes 1–2: follow-cam static — PhysX/fabric doesn't write poses back to
  USD; fixed by reading `VehicleManager` state (keyed by the full
  `/World/droneN/base_link` prim path, not the parent).
- take 3: camera collided with ceiling ducts at offset z=+3 → black
  occlusions; flattened to z=+1.5 tracking the lowest drone.
- take 4: all takeoffs rejected — a stuck-active TakeoffTask on the recreated
  sim (`another task is already active`); full `airstack down`/`up` cleared it.

## b — Foxglove footage ✅

Same 3-robot Isaac run; GCS Foxglove auto-loaded the 3-robot layout. Keeper:
`foxglove_take2.mp4` — 3D panel draws the three live trajectory traces while
robot tabs show Robot Tasks and live camera/depth feeds.

![live trajectory traces + camera feed](b-foxglove-footage/frame_traces.png)

## c — MS AirSim footage ✅

`airstack up --sim airsim --robots 2 --scene neighborhood` (AirSimNH
pre-fetched; ms-airsim image rebuilt so the alpha.12 scene-fetch entrypoint is
baked). UE4 chase cam (FlyWithMe), flight-ready in 33 s. 6/7 goals SUCCEEDED.

![Figure-8 between the houses](c-msairsim-footage/frame_figure8_houses.png)
![street chase](c-msairsim-footage/frame_street.png)

## d — Hero video ✅

5 segments crossfaded (xfade), 1920×1080@30, H.264 CRF28 + faststart, 34 s,
4.7 MB, poster JPEG. Corner HUD (AirSim REC/collision overlay) cropped out.

![Foxglove segment](d-hero-video/frame_foxglove_segment.png)
![AirSim street segment](d-hero-video/frame_airsim_segment.png)

## e — Docs integration ✅

- `docs/overrides/home.html`: GIF `<img>` → autoplay/muted/loop `<video>`
  with poster; `splash-background.gif` deleted.
- Clips embedded in the three section pages (raw-HTML paths are NOT rewritten
  by mkdocs: index pages need `../../assets/...`, non-index pages like
  `foxglove.md` need `../../` too because directory URLs add a level).
- `mkdocs build` clean (image `v0.19.0-alpha.18_mkdocs`); verified in the
  built site that every `<source>` path resolves to
  `site/docs/assets/media/*`.

## f — Landing page redesign (follow-up request) ✅

`docs/overrides/home.html` + `docs/stylesheets/extra.css` rebuilt around four
pillars (react.dev-style: real artifacts, no asserted claims):

1. **Hero** — headline + one-liner + the literal 3-command quickstart in a
   terminal card with a Copy button, over the demo video. CTAs above the fold.
2. **One command** — `airstack up` variants snippet + the real
   `airstack ready` ✓-checklist as a terminal card.
3. **Same code in sim and on the vehicle** — Isaac clip (lazy-loaded) beside
   the real compose excerpt showing robot-desktop and robot-l4t extending one
   `robot_base` and dispatching the same stack entry launch.
4. **CI flies the whole stack** — the seven real pytest marks as a matrix,
   the literal `airstack test` command, `/pytest` PR trigger.
5. **AI agents can drive this repo** — real `.agents/skills/` chips + the
   provable claim that an agent captured every video on the page.
6. **The whole system on one screen** — `workstation.webp` (144 KB), an
   unedited desktop capture taken live: Isaac (follow-cam) + Foxglove with
   three live trajectory traces + terminal with the flight-ready checklist;
   replaces the outdated `overview.png` on the landing page. Raw frame:
   `d-hero-video/workstation_raw.png`.
7. Provenance (AirLab/CMU) + final CTA with the OSMO no-GPU path.

Frost overlay lightened earlier (0.55→0.30 top, page-fade from 85%) and hero
text shadows strengthened to keep contrast. Copy buttons + IntersectionObserver
lazy video loading are inline JS in the template. Verified against the live
`mkdocs serve` (note: it mounts the site under the `site_url` path prefix, so
the local landing page is `/docs/docs/`).

## g — Hero v2: motion + scene diversity (follow-up request) ✅

Feedback: the AirSim hero segments read as hovering (FlyWithMe chase cam keeps
the drone centered), and everything Isaac was warehouse.

- **AirSim neighborhood re-shoot** (`airsim_nh_fast.mp4`): Racetrack 40×12 m at
  5 m/s + 30 m Line — real translation past houses/poles. Clean window 0–45 s
  (the desktop came foreground later; RustDesk session).
- **Second UE scene** (`airsim_zhangjiajie.mp4`): ZhangJiajie karst pillars.
  Blind trajectories graze trees/stone (collision counter in the corner HUD,
  cropped out); best windows t≈10–24 (two-drone climb) and t≈64–75 (steady
  pillar glide — used as the hero finale). UE console text lingers ~30 s
  top-left → aggressive crop on early windows.
- **UPDATE (h): office recovered via light boost.** `ISAAC_SIM_LIGHT_BOOST=16`
  (new `scene_prep.boost_scene_lights`: de-instances light-bearing subtrees,
  then `exposure += log2(factor)` on every scene light — 121 office ceiling
  lights) + `ISAAC_SIM_FOLLOW_CAM_LIGHT=8000` headlight made the office lobby
  filmable. `isaac_office_take1.mp4` t≈16–24 (two drones over the marble
  lobby) replaced the second warehouse segment in the hero (v3, 40 s, 5.4 MB).
  Remaining office caveat: the chase cam clipped into a wall corner ~t=32 when
  drone 2's Figure-8 crossed the lobby edge — keep indoor trajectories small
  or pick spawn spots with ≥4 m clearance.
- **Hospital attempt — dropped for now.** New tooling landed:
  `ISAAC_SIM_SPAWN_XY` (spawn-row center env, plumbed through compose +
  `example_multi`), `ISAAC_SIM_DOME_LIGHT` intensity/exposure override, and a
  USD spawn-finder (mirror payload deps from S3, bound instanceable prims via
  BBoxCache, grid-search clearest floor cell under a ceiling — office best
  indoor spot: (-11, 6.5), 3.9 m clearance). Spawn + camera placement worked,
  but both interiors are authored dark and the dome light cannot reach them —
  a proper clip needs a scene lighting pass. Hero keeps the two warehouse
  segments for Isaac.
- **Hero v2** (`splash-video.mp4`): warehouse takeoff → warehouse flight →
  Foxglove traces → NH street run → ZhangJiajie pillar glide; 39 s, 1600×900
  CRF30, **5.4 MB**. `ms_airsim_demo.mp4` docs clip also refreshed with the
  dynamic NH take (3.5 MB).
- Ops note: `airstack up --sim isaac` while ms-airsim is running fails with
  "Address already in use" (both sims bind 172.31.0.200) — the CLI could stop
  the conflicting sim service on `--sim` switch; filed as follow-up idea.

## Side feature: Isaac viewport follow-camera

`pegasus_app.py` now authors `/World/follow_cam` (16 mm, wide clip range),
switches the viewport to it, frames the spawn point pre-Play (fixes the
black-viewport-at-origin problem on cm stages), and chases the target drone
with exponential smoothing using live Pegasus `VehicleManager` state.
Env: `ISAAC_SIM_FOLLOW_CAM` (default drone 1; `off` disables),
`ISAAC_SIM_FOLLOW_CAM_OFFSET` (world-frame `x,y,z`, default `-5,-5,2.5`),
plumbed through `simulation/isaac-sim/docker/docker-compose.yaml`.
