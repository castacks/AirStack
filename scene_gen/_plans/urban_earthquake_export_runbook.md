# Urban earthquake fast export (L1–L3)

This is the opt-in export path for the reviewed urban-earthquake scene.  It
does not replace the existing launcher defaults or the current dataset tree.
It writes to `Earthquake/Urban_fast` locally and on Nucleus.

## Inputs

- L1 / L2 / L3 use M5.5 / M6.5 / M7.5.
- Damage seeds are 31 / 41 / 51.
- The city is 1000 × 1000 m with `urban_quake_v5`.
- The reviewed r15 archetypes are entirely on Nucleus at
  `omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype_r15`.
- Per-building GAC bakes are read from the sibling Nucleus `gac_quake` tree.

The r15 publisher first performs a server-side copy of the canonical library,
then uploads only overlay files whose byte size differs.  Run it from an Isaac
container only when promoting a new reviewed overlay:

```bash
bash scene_gen/tools/usd_python.sh \
  scene_gen/tools/publish_quake_overlay.py \
  --names-out /isaac-sim/.cache/quake_r15_all_usds.txt

bash scene_gen/tools/usd_python.sh \
  scene_gen/tools/verify_quake_archetypes.py \
  --root omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/scene_gen/assets/archetype_r15 \
  --names-file /isaac-sim/.cache/quake_r15_all_usds.txt \
  --manifest archetypes.json
```

The cold audit must report every selected layer opened with zero unsafe and
zero missing dependencies.

## Run on the OSMO pod

Run the driver on the pod host, not inside the container:

```bash
cd /root/AirStack
LEVELS="1 2 3" ISAAC_SIM_ACTIVE_GPU=2 \
  bash scene_gen/tools/urban_earthquake_cell_fast.sh
```

The driver runs levels sequentially and never tears the pod down.  An
incomplete prior `Urban_fast` cell is moved to a timestamped sibling before a
retry.  A locally complete cell skips the expensive rebuild and resumes at
upload/cold verification.

Each level is accepted only after all of these pass:

1. casualty-only population: no standing/walking people;
2. two geometry-verified exterior views per casualty;
3. whole-scene nadir plus four obliques;
4. epicentre and four district/quadrant views from two bearings;
5. up to 48 highest-severity building close-ups, top and oblique;
6. exact 2-D damage/casualty map;
7. portable USD gate (`portable_ok=true`, no local asset dependency);
8. complete cell upload with byte-size verification;
9. cold open from the final Nucleus URL.

Outputs are:

```text
/isaac-sim/final_disaster_dataset/Earthquake/Urban_fast/level_<L>/1/
omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/
  final_disaster_dataset/Earthquake/Urban_fast/level_<L>/1/
```

Each cell contains one USD, `GT_people.json`, `GT_hints.json`,
`build_stats.json`, `freeze_report.json`, `review_manifest.json`, and the full
`snaps/` tree.  Per-level timings are written under
`/root/docker/isaac-sim/logs/urban_quake_fast/`.
