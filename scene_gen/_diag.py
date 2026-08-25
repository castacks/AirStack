"""Throwaway: per-region building count and Stage A archetype cost."""
from isaacsim import SimulationApp
app = SimulationApp(launch_config={"headless": True})
import os, sys, yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compile_disaster as cd, scene_generator as sg
from generate_scene import build_scene
from archetypes import plan as P

with open(cd.resolve_config_path("urban_quake_showcase")) as fh:
    spec0 = yaml.safe_load(fh)
with open(cd.DEFAULT_BASE) as fh:
    base = yaml.safe_load(fh)

for region in [float(v) for v in sys.argv[1:]]:
    spec = yaml.safe_load(yaml.safe_dump(spec0))
    spec["region_m"] = [region, region]
    cfg = sg.resolve_asset_pack(cd.compile_spec(spec, base))
    placements, layout, _ = build_scene(cfg, sg._make_resolver(cfg))
    h = [p for p in placements if p.get("category") in ("house", "building")]
    types = {str(p.get("usd")) for p in h}
    dmg = [p for p in h if p.get("_damage_level")]
    print(f"PLAN region={region:.0f}: buildings={len(h)} damaged={len(dmg)} "
          f"distinct_building_types={len(types)} placements={len(placements)}")
sys.stdout.flush()
app.close()
