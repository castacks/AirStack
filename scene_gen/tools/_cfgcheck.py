import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from compile_disaster import load_scene_config
c = load_scene_config("/isaac-sim/AirStack/scene_gen/config/presets/downtown_gac.yaml")
print("building_props:", c.get("building_props"))
b = c["usds"]["buildings"]
print("pools:", {k: len(v) for k, v in b.items()})
print("trees:", len(c["usds"].get("trees") or []),
      "plants:", len(c["usds"].get("plants") or []),
      "rocks:", len(c["usds"].get("rocks") or []))
print("CityPark anywhere in usds:", "CityPark" in str(c["usds"]))
