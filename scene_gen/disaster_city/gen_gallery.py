"""Write specs/gallery.yaml (the shot list gallery.py renders).

    ~/.venvs/recon/bin/python gen_gallery.py

Outside: orbit shots (target, azimuth, elevation, distance), the same framings as the
before/after renders. Inside: cameras in each hero's own frame. Survivors: one shot each from
specs/survivors.yaml -- an orbit for anyone on the ground, and for anyone on B01 (its decks, roof,
first floor) a camera standing on the same level 3.2 m away, since an orbit from above only sees
the grating over them. Each survivor links the drone frame that shows them (video_compare.py output).
"""
import math
import numpy as np, yaml
from _paths import SPECS

O = lambda n, c, t, az, el, d: {"name": n, "caption": c, "target": t, "az": az, "el": el, "dist": d}
outside = [
    O("site_overview", "The whole site: grafted heroes, rubble piles, LOD1 buildings, trees, vehicles", [95, -330, 58], 230, 42, 260),
    O("b01_and_pile", "B01 (the 133 building) and the R01 rubble pile", [42, -405, 60], 215, 28, 75),
    O("b01_close", "B01: the collapsed wing's slab in its steel frame, video-projected walls", [46, -410, 61], 200, 18, 35),
    O("b01_ylo_face", "B01's street face, textured from the drone frames", [49.0, -418.0, 62.0], 250, 12, 22),
    O("b01_wing_side", "The wing side: pancaked floors under the frame, rubble up against it", [42, -402, 61], 150, 15, 25),
    O("pile_top", "R01 from above", [38, -392, 59], 200, 65, 45),
    O("b35_house", "B35, the tan house inside R03's collapsed block", [63.0, -434.5, 61.0], 250, 20, 26),
    O("strip_mall", "B03 strip mall: each unit's roof state (intact / gone / pancaked)", [96, -416, 60], 300, 32, 45),
    O("drill_tower", "S01 drill tower, metal-clad as in the video", [3, -514, 66], 210, 18, 45),
    O("warehouse", "B06 south warehouse", [160, -499, 64], 140, 22, 60),
    O("industrial_pad", "The industrial pad: tank car, vessel, pipe rack, lattice tower", [55, -525, 62], 200, 25, 70),
    O("rail_derailment", "Rail yard and the R05 derailment", [295, -255, 62], 200, 30, 65),
    O("rail_cars_r02", "Crossed passenger cars by R02", [220, -385, 61], 250, 30, 55),
    O("r02_pile", "R02 east rubble pile", [188, -390, 61], 230, 28, 45),
    O("houses", "The street of houses (LOD1, tinted walls)", [150, -280, 58], 200, 30, 60),
    O("grid_street", "Street level in the grid", [70, -445, 59], 30, 8, 40),
    O("water_tower", "Water tower and the east rail line", [330, -290, 63], 250, 25, 40)]
I = lambda n, c, b, e, l, f=12: {"name": n, "caption": c, "in": b, "eye": e, "look": l, "focal": f}
inside = [
    I("b01_ground_floor", "B01 ground floor, through to the doorway onto the frame", "B01", [1.3, 4.2, 1.7], [12.2, 9.2, 1.0]),
    I("b01_first_floor", "B01 first floor: survivor V07 sitting under the west window (A03)", "B01", [11.8, 4.1, 5.4], [1.2, 7.0, 4.0]),
    I("b01_frame_deck", "B01 frame deck: survivors V02, V03, V10 on the plywood (B08, A02)", "B01", [13.4, 3.9, 6.1], [21.5, 8.5, 4.4], 14),
    I("b01_roof", "B01 roof: the cream jib crane and survivor V04 (B09)", "B01", [1.6, 4.0, 9.6], [10.5, 8.5, 7.8], 14),
    I("b01_wing_under", "Under B01's collapsed wing: the tilted roof slab and the pancaked floors", "B01", [2.0, 15.4, 1.5], [18.0, 11.5, 1.8]),
    I("b01_stair_133", "The stair up to the 133 landing", "B01", [13.0, -1.5, 1.8], [22.4, 2.0, 4.5], 14),
    I("s01_top_floor", "S01 drill tower, the open top floor", "S01", [1.0, 1.0, 18.3], [5.7, 7.2, 17.3]),
    I("b35_ground_floor", "B35 tan house, ground floor", "B35", [1.0, 1.0, 1.6], [11.0, 5.8, 1.0]),
    I("b35_first_floor", "B35 tan house, first floor", "B35", [11.0, 1.0, 4.4], [1.0, 5.8, 3.4]),
    I("b03_unit3", "B03 strip mall, the intact unit U3", "B03", [1.0, 9.2, 1.7], [14.0, 14.2, 1.2]),
    I("b06_hall", "B06 warehouse hall", "B06", [2.0, 2.0, 2.5], [38.0, 28.0, 3.0])]

b = yaml.safe_load(open(SPECS / "B01.yaml")); o = np.array(b["origin"]); t = math.radians(b["yaw_deg"])
to_local = lambda p: np.r_[np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]]) @ (np.array(p[:2]) - o[:2]), p[2] - o[2]]
LEVELS = [(4.0, 5.0, (12.9, 3.6, 24.6, 9.8)), (7.4, 8.2, (0.6, 3.6, 12.5, 9.6)), (3.5, 4.0, (0.7, 3.7, 12.4, 9.5))]   # frame deck, roof, first floor
survivors = []
for v in yaml.safe_load(open(SPECS / "survivors.yaml")):
    s = {"name": f"survivor_{v['id']}", "caption": f"{v['id']} {v['pose']}: {v['note']}", "video": [f"video/{v['frame'].split('/', 1)[1].replace('/', '_')[:-4]}.jpg"]}
    L = to_local(v["at"])
    lvl = next((box for z0, z1, box in LEVELS if z0 - 0.3 <= L[2] <= z1 and box[0] - 0.5 <= L[0] <= box[2] + 0.5 and box[1] - 0.5 <= L[1] <= box[3] + 0.5), None)
    if lvl is None:
        s.update(target=[v["at"][0], v["at"][1], v["at"][2] + 0.4], az=(v["yaw"] + 120) % 360, el=28, dist=4.5, focal=20)
    else:
        e = next(e for a in np.radians(np.arange(0, 360, 30)) for e in [L[:2] + 3.2 * np.array([math.cos(a), math.sin(a)])]
                 if lvl[0] <= e[0] <= lvl[2] and lvl[1] <= e[1] <= lvl[3])
        s.update({"in": "B01", "eye": [round(float(x), 2) for x in (*e, L[2] + 1.7)], "look": [round(float(x), 2) for x in (*L[:2], L[2] + 0.2)], "focal": 14})
    survivors.append(s)

G = {"title": "Disaster City (TEEX) in Isaac Sim",
     "intro": "Rebuilt from Google 3D tiles + two drone videos: hero buildings measured off drone reconstructions and textured from the video, "
              "rubble piles from rubble assets, vehicles and props from Nucleus/Objaverse, survivors where the video shows them.",
     "sections": [
         {"title": "Outside", "text": "Orbit views of the assembled scene (host Isaac Sim 6.0.1, RTX real-time).", "shots": outside},
         {"title": "Inside the buildings", "text": "The hero buildings are enterable: real openings, floors, stairs and interior lights.", "shots": inside},
         {"title": "Survivors", "text": "Casualty actors found in the drone video (keypoint R-CNN on the georeferenced frames, rays cast onto the model, "
                                        "clustered, checked by eye) and placed as posed RenderPeople rigs. Each links the scene rendered from the drone's "
                                        "own camera beside that frame.", "shots": survivors},
         {"title": "Survivors from above", "text": "", "shots": [O("survivors_overview", "All 11 survivors round B01: on the decks, the roof, the road and the kerbs", [38, -418, 58], 225, 55, 60)]}]}
yaml.safe_dump(G, open(SPECS / "gallery.yaml", "w"), sort_keys=False, width=200, default_flow_style=None)
print(sum(len(s["shots"]) for s in G["sections"]), "shots ->", SPECS / "gallery.yaml")
