"""LOD2 for the plain buildings: a roof, eaves, doors, windows and (for sheds) a garage opening, instead of an extruded box.

    ~/.venvs/recon/bin/python gen_lod2.py && for f in data/recon/lod2/*_spec.yaml; do ~/.venvs/recon/bin/python build_hero.py $f; done

For every LOD1 building (buildings_lod1.yaml) that has no hand-made hero and whose outline is close to its
rectangle (>= 0.7 filled, one level), write data/recon/lod2/<ID>_spec.yaml for build_hero.py, in the
building's own frame (x along the long side). Two kinds, by name and size:
  house  hip roof (roof_brown) with a 0.4 m overhang over stucco walls; 1 storey, 2 above a 5.8 m roof;
         windows every ~3 m on every face, a door mid-front
  shed   (warehouses, yard buildings, barracks, offices) a low gable (rise 10% of the span) with 0.5 m eaves,
         gable ends closed; ribbed metal walls for sheds, stucco for barracks and offices; windows and a
         door, and on sheds a garage opening on the front
The front is the face nearest an OSM road. The height is the tile blob's (90th percentile = the ridge).
assemble_scene.py references data/recon/lod2/<ID>.usd in place of that building's LOD1 box; ground.py's
masking is unchanged (it works from the LOD1 footprints).
"""
import math
import numpy as np, yaml
from shapely.geometry import Polygon
from _paths import R, CODE

OUT = R / "lod2"; OUT.mkdir(exist_ok=True)
SKIP = {"B02", "B05", "B19"}                       # collapsed / courtyard / irregular: their LOD1 outline is better
heroes = eval(open(CODE / "assemble_scene.py").read().split("PIECES = ")[1].split("\n")[0])
osm = np.load(R / "osm.npz"); roads = np.concatenate([osm[k].reshape(-1, 3, 2).mean(1) for k in osm.files if k.startswith("road_")])
r3 = lambda v: [round(float(x), 3) for x in v]
n = 0
for b in yaml.safe_load(open(R / "buildings_lod1.yaml"))["buildings"]:
    if b["id"] in heroes or b["id"] in SKIP or len(b["levels"]) != 1: continue
    A = sum(Polygon(r).area for r in b["levels"][0]["rings"])
    if A / (b["size_m"][0] * b["size_m"][1]) < 0.7: continue
    X, Y = max(b["size_m"]), min(b["size_m"]); Hr = b["height_m"]
    yaw = b["yaw_deg"] + (90 if b["size_m"][1] > b["size_m"][0] else 0); t = math.radians(yaw)
    Rz = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    o = np.array(b["at"]) - Rz @ [X / 2, Y / 2]
    faces = {"ylo": ((0, 0), (X, 0)), "yhi": ((0, Y), (X, Y)), "xlo": ((0, 0), (0, Y)), "xhi": ((X, 0), (X, Y))}
    mid = {k: o + Rz @ ((np.array(a) + np.array(c)) / 2) for k, (a, c) in faces.items()}
    front = min(mid, key=lambda k: np.min(np.linalg.norm(roads - mid[k], axis=1)))
    sheddy = any(w in b["name"] for w in ("shed", "warehouse", "yard", "rig", "portable"))
    house = not sheddy and (b["name"].startswith("house") or (X * Y < 90 and "barracks" not in b["name"]))
    if house:
        rise = min(0.35 * Y, 2.0); eave = max(2.7, Hr - rise); wall_mat, roof_mat = "stucco", "roof_brown"
    else:
        rise = max(0.5, 0.1 * Y); eave = max(2.7, Hr - rise); wall_mat, roof_mat = ("metal_ribbed" if sheddy else "stucco"), ("metal_grey" if sheddy else "roof_brown")
    storeys = 2 if eave >= 5.2 else 1; fh = (eave - 0.15) / storeys
    parts = [{"box": {"name": "floor", "min": [0, 0, 0], "max": r3([X, Y, 0.15])}}]
    if storeys == 2: parts.append({"box": {"name": "slab_1", "min": r3([0.15, 0.15, 0.15 + fh - 0.2]), "max": r3([X - 0.15, Y - 0.15, 0.15 + fh])}})
    for k, (a, c) in faces.items():
        L = X if k[0] == "y" else Y; ops = []
        n_w = max(1, int(L // 3.2))
        for s_ in range(storeys):
            for i in range(n_w):
                at = (i + 0.5) * L / n_w - 0.55
                ops.append({"at": round(at, 2), "w": 1.1, "sill": round(s_ * fh + 0.9, 2), "h": 1.2})
        if k == front:
            ops = [o_ for o_ in ops if not (o_["sill"] < fh and abs(o_["at"] + 0.55 - L / 2) < (3.2 if sheddy else 1.4))]
            if sheddy and "portable" not in b["name"]:                       # the garage opening, mid-front
                gw = min(4.5, 0.3 * L); ops.append({"at": round(L / 2 - gw / 2, 2), "w": round(gw, 2), "h": round(min(3.6, fh * storeys - 0.6), 2)})
                ops = [o_ for o_ in ops if o_["at"] + o_["w"] < L / 2 - gw / 2 - 0.2 or o_["at"] > L / 2 + gw / 2 + 0.2 or o_ is ops[-1]]
            else:
                ops.append({"at": round(L / 2 - 0.5, 2), "w": 1.0, "h": 2.1})
        ops.sort(key=lambda o_: o_["at"])
        parts.append({"wall": {"name": f"wall_{k}", "from": r3(a), "to": r3(c), "z": r3([0.15, eave]), "mat": wall_mat, "openings": ops}})
    if house:
        parts.append({"hip": {"name": "roof", "min": r3([-0.4, -0.4, eave]), "max": r3([X + 0.4, Y + 0.4, Hr]), "mat": roof_mat}})
    else:
        ov = 0.5; sl = math.atan2(rise, Y / 2)
        for k_, (y0, y1) in (("lo", (-ov, Y / 2)), ("hi", (Y + ov, Y / 2))):
            z0 = eave - ov * math.tan(sl)
            parts.append({"beam": {"name": f"roof_{k_}", "from": r3([X / 2, y0, z0]), "to": r3([X / 2, y1, Hr]), "t": 0.12, "t2": r3([X + 2 * ov])[0], "mat": roof_mat}})
        for k_, x in (("xlo", 0.0), ("xhi", X - 0.2)):                         # the gable ends, closed
            parts.append({"prism": {"name": f"gable_{k_}", "plane": "yz", "y": r3([x, x + 0.2]), "poly": [r3(p) for p in ([0, eave], [Y, eave], [Y / 2, Hr])], "mat": wall_mat}})
    spec = {"id": b["id"], "name": b["name"], "semantic": "building", "out_dir": "lod2", "default_mat": wall_mat,
            "origin": r3([o[0], o[1], b["ground_z"]]), "yaw_deg": round(float(yaw), 2), "parts": parts}
    (OUT / f"{b['id']}_spec.yaml").write_text("# generated by gen_lod2.py -- edit that, not this\n" + yaml.safe_dump(spec, sort_keys=False, default_flow_style=None, width=200))
    n += 1; print(b["id"], b["name"], "house" if house else ("shed" if sheddy else "gable"), f"{X:.1f} x {Y:.1f}, eave {eave:.1f}, ridge {Hr:.1f}, {storeys} storey, front {front}")
print(n, "LOD2 specs ->", OUT)
