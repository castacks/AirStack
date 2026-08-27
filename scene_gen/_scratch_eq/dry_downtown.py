import sys, random, os
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from compile_disaster import load_scene_config
import scene_generator as sg
from layout import city_layout
from detail import districts
for R in (250, 300, 400):
    cfg = load_scene_config("downtown", spec_overrides={"region_m": [R, R]})
    cfg["measure_usds"] = True
    resolver = sg._make_resolver(cfg)
    with city_layout.patched(cfg):
        placements, layout = sg.build_city(cfg, resolver)
    rng = random.Random(7)
    district_at, rings = districts.assign(cfg, layout)
    if rings:
        districts.remap_buildings(cfg, layout, placements, resolver, rng, district_at)
    houses = [p for p in placements if p.get("category") == "house"]
    blocks = layout.get("blocks", [])
    print("REGION", R, "blocks", len(blocks), "buildings", len(houses), "rings", bool(rings))
    from collections import Counter
    print("  ", Counter(p["usd"].rsplit("/",1)[-1] for p in houses).most_common(8))
    print("  block sizes", [(round(b[2]-b[0]), round(b[3]-b[1])) for b in blocks][:12])
