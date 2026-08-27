import sys, random
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from compile_disaster import load_scene_config
import scene_generator as sg
from layout import city_layout
from detail import districts
from collections import Counter
cfg = load_scene_config("downtown_earthquake")
cfg["measure_usds"] = False
# fallback sizes for the archetype pool, from urban_building
from detail import urban_building as ub
resolver = sg._make_resolver(cfg)
for k in list(resolver.__dict__.keys()):
    pass
with city_layout.patched(cfg):
    placements, layout = sg.build_city(cfg, resolver)
district_at, rings = districts.assign(cfg, layout)
if rings:
    districts.remap_buildings(cfg, layout, placements, resolver, random.Random(7), district_at)
houses = [p for p in placements if p.get("category") == "house"]
blocks = layout.get("blocks", [])
print("blocks", len(blocks), "buildings", len(houses), "rings", bool(rings))
print("sizes", [(round(b[2]-b[0]), round(b[3]-b[1])) for b in blocks])
print(Counter(p["usd"].rsplit("/",1)[-1] for p in houses).most_common(20))
print("disaster field", cfg["disaster"].get("field"), "tilt", cfg["disaster"]["debris"].get("tilt_chance"))
