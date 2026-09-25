# /// script
# requires-python = ">=3.10"
# dependencies = ["pyyaml", "opencv-python-headless", "numpy"]
# ///
"""Apply LABELS.yaml to the Disaster City blend and draw the labelled map.

    uv run --script label_map.py

1. Blender (on PATH) measures each label's height off "Google 3D Tiles.001",
   rebuilds the `Labels` collection in blender_data/disaster_city.blend (one
   empty + flat text per label, grouped by kind) and saves the blend.
2. It also renders the site top-down; this script draws the labels on it ->
   LABELS_map.png. `height_m` is written back into LABELS.yaml.
"""
import json, re, subprocess, sys, tempfile
from pathlib import Path
import cv2, numpy as np, yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _paths import DATA, LABELS
BLEND = DATA / "blender_data/disaster_city.blend"
CX, CY, EXT, PX = 75.0, -300.0, 560.0, 4480          # render window, 0.125 m/px
COLORS = {  # BGR for the map, RGB(0-1) in Blender
    "building": (60, 60, 255), "rubble": (0, 165, 255), "vehicle": (255, 80, 200),
    "structure": (255, 255, 0), "zone": (80, 255, 80), "road": (200, 200, 200),
    "water": (255, 160, 60), "offsite": (150, 150, 150)}
MEASURED = {"building", "rubble", "structure", "vehicle"}

BLENDER_SCRIPT = r'''
import bpy, json, sys, math
from mathutils import Vector
a = sys.argv[sys.argv.index("--") + 1:]
labels, colors, out_json, out_png = json.load(open(a[0])), json.load(open(a[1])), a[2], a[3]
cx, cy, ext, px = map(float, a[4:8])
sc = bpy.context.scene
tiles = bpy.data.objects["Google 3D Tiles.001"]
dg = bpy.context.evaluated_depsgraph_get()

def z_at(x, y):
    hit, loc, *_ = sc.ray_cast(dg, Vector((x, y, 1000)), Vector((0, 0, -1)))
    return loc.z if hit else None

def measure(l):
    x, y = l["at"]
    tops = [z for dx in range(-3, 4) for dy in range(-3, 4) if (z := z_at(x + dx, y + dy)) is not None]
    radii = [l["size_m"] / 2 + 5] if "size_m" in l else [12, 20, 30]   # unknown footprint: reach past it
    ring = sorted(z for r in radii for i in range(24)
                  if (z := z_at(x + r * math.cos(i * math.pi / 12), y + r * math.sin(i * math.pi / 12))) is not None)
    if not tops or not ring: return None, None
    ground = ring[len(ring) // 10]
    return max(tops), ground

# rebuild the Labels collection
old = bpy.data.collections.get("Labels")
if old:
    for o in list(old.all_objects):
        d = o.data
        bpy.data.objects.remove(o)
        if d is not None and d.users == 0 and isinstance(d, bpy.types.Curve): bpy.data.curves.remove(d)
    for c in list(old.children_recursive): bpy.data.collections.remove(c)
    bpy.data.collections.remove(old)
root = bpy.data.collections.new("Labels"); sc.collection.children.link(root)
subs, mats, out = {}, {}, {}
for l in labels:
    k = l["kind"]
    if k not in subs:
        subs[k] = bpy.data.collections.new(f"Labels_{k}"); root.children.link(subs[k])
        m = bpy.data.materials.get(f"label_{k}") or bpy.data.materials.new(f"label_{k}")
        m.diffuse_color = (*colors[k], 1.0); mats[k] = m
    top, ground = measure(l)
    h = round(top - ground, 1) if top is not None else None
    out[l["id"]] = h
    name = f'{l["id"]}_{l["name"]}'
    e = bpy.data.objects.new(name, None)
    e.empty_display_type = "SINGLE_ARROW"; e.empty_display_size = 4
    e.location = (l["at"][0], l["at"][1], top if top is not None else 50)
    for key in ("id", "name", "kind", "note"):
        if key in l: e[key] = l[key]
    if h is not None: e["height_m"] = h
    subs[k].objects.link(e)
    cu = bpy.data.curves.new(name + "_text", "FONT")
    cu.body = f'{l["id"]} {l["name"]}'; cu.size = 6 if k == "zone" else 3; cu.align_x = "CENTER"
    t = bpy.data.objects.new(name + "_text", cu); t.data.materials.append(mats[k])
    t.parent = e; t.location = (0, 0, 3); t.hide_render = True
    subs[k].objects.link(t)
json.dump(out, open(out_json, "w"))
bpy.ops.wm.save_mainfile()

# top-down render of the tiles only
hidden = {o.name: o.hide_render for o in bpy.data.objects}
for o in bpy.data.objects: o.hide_render = o.name != tiles.name
cam = bpy.data.objects["Camera"]; cam.location = (cx, cy, 500); cam.rotation_euler = (0, 0, 0)
cam.data.type = "ORTHO"; cam.data.ortho_scale = ext; cam.data.clip_end = 2000
sc.camera = cam; sc.render.engine = "BLENDER_WORKBENCH"
sc.display.shading.light = "FLAT"; sc.display.shading.color_type = "TEXTURE"
sc.render.resolution_x = sc.render.resolution_y = int(px); sc.render.resolution_percentage = 100
sc.render.filepath = out_png; bpy.ops.render.render(write_still=True)
'''


def main():
    labels = yaml.safe_load(open(LABELS))["labels"]
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        (tmp / "labels.json").write_text(json.dumps(labels))
        (tmp / "colors.json").write_text(json.dumps({k: [c[2] / 255, c[1] / 255, c[0] / 255] for k, c in COLORS.items()}))
        (tmp / "run.py").write_text(BLENDER_SCRIPT)
        subprocess.run(["blender", "-b", str(BLEND), "--python", str(tmp / "run.py"), "--",
                        str(tmp / "labels.json"), str(tmp / "colors.json"), str(tmp / "h.json"), str(tmp / "top.png"),
                        str(CX), str(CY), str(EXT), str(PX)], check=True, stdout=subprocess.DEVNULL)
        heights = json.loads((tmp / "h.json").read_text())
        img = cv2.imread(str(tmp / "top.png"))

    # write height_m back into LABELS.yaml, keeping its comments and layout
    src = LABELS.read_text().splitlines()
    kinds = {l["id"]: l["kind"] for l in labels}
    for i, line in enumerate(src):
        m = re.match(r"^- \{id: (\w+),", line)
        if not m or kinds[m[1]] not in MEASURED: continue
        line = re.sub(r", height_m: [-\d.]+", "", line)
        if heights.get(m[1]) is not None:
            line = re.sub(r"(at: \[[^\]]*\])", rf"\1, height_m: {heights[m[1]]}", line, count=1)
        src[i] = line
    LABELS.write_text("\n".join(src) + "\n")

    # draw the map
    r = EXT / PX
    to_px = lambda x, y: (int((x - (CX - EXT / 2)) / r), int(((CY + EXT / 2) - y) / r))
    img = (img * 0.8).astype(np.uint8)
    for l in sorted(labels, key=lambda l: l["kind"] != "zone"):   # zones underneath
        u, v = to_px(*l["at"]); c = COLORS[l["kind"]]
        big = l["kind"] == "zone"
        if "size_m" in l: cv2.circle(img, (u, v), int(l["size_m"] / 2 / r), c, 3)
        if not big: cv2.circle(img, (u, v), 9, c, -1); cv2.circle(img, (u, v), 9, (0, 0, 0), 2)
        txt = f'{l["id"]} {l["name"]}'
        s, th = (1.3, 3) if big else (0.9, 2)
        (tw, tht), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, s, th)
        org = (u - tw // 2, v + tht // 2) if big else (u - tw - 14 if l.get("side") == "left" else u + 14, v + tht // 2)
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, s, (0, 0, 0), th + 5, cv2.LINE_AA)
        cv2.putText(img, txt, org, cv2.FONT_HERSHEY_SIMPLEX, s, c, th, cv2.LINE_AA)
    # legend + scale bar
    y0 = 60
    for k, c in COLORS.items():
        cv2.circle(img, (60, y0), 14, c, -1)
        cv2.putText(img, k, (90, y0 + 12), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 3, cv2.LINE_AA); y0 += 50
    cv2.line(img, (60, PX - 60), (60 + int(50 / r), PX - 60), (255, 255, 255), 6)
    cv2.putText(img, "50 m   (N up; world X/Y of disaster_city.blend)", (60, PX - 80), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 3, cv2.LINE_AA)
    cv2.imwrite(str(DATA / "LABELS_map.png"), img)
    print(f"{len(labels)} labels; heights for {sum(h is not None for h in heights.values())}; wrote LABELS_map.png, LABELS.yaml, {BLEND.name}")


if __name__ == "__main__":
    sys.exit(main())
