import re
import omni.client
ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "GreatAmericanCity/Assets/Game/GreatAmericanCity/Meshes/")
res, ents = omni.client.list(ROOT)
names = sorted(e.relative_path[:-4] for e in ents
               if e.relative_path.lower().endswith(".usd"))
bld = [n for n in names if re.match(r"^SM_Building_\d", n)]
props = [n for n in names if n not in bld]
print("%d usd total: %d SM_Building_NN, %d other\n" % (len(names), len(bld), len(props)))
CAT = {
 "roof plant": ("air_machine","air_tubes","water_tank","generator","communication_tower",
                "tower","superior_construction","steel_pipe","glass_roof","construct_tubes"),
 "wall/vertical": ("stair","escape","ladder","cable","protect_tube","protective_grid",
                   "exit_door","wallback","tube"),
 "facade/street": ("awning","signs","plate","door","bus_stop","hydrant","light_pole",
                   "mailbox","road_sign","trash","bench","cone","block_barrier"),
 "ground/tile": ("asphalt","sidewalk","pavement","concrete","floor","slab","center_square",
                 "subway","stair_"),
 "greenery": ("bush","garden","leaves","plant_pot","tree"),
}
seen = set()
for cat, keys in CAT.items():
    hit = [n for n in props if any(k in n.lower() for k in keys) and n not in seen]
    seen |= set(hit)
    print("%-16s %2d  %s" % (cat, len(hit), ", ".join(hit)))
rest = [n for n in props if n not in seen]
print("%-16s %2d  %s" % ("uncategorised", len(rest), ", ".join(rest)))
