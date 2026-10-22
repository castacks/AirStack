"""Survivor variants drawn from the placement candidates -> data/recon/survivors/v<k>/.

    ~/.venvs/recon/bin/python sample_survivors.py 1 2 3 4 5        # after survivor_candidates.py

Variant k is drawn with numpy's generator seeded k: the video's class mix (ground / elevated /
covered, from placement_candidates.json), each class without replacement, survivors >= 4 m
apart; a pose the candidate has room for, a rig and a yaw. Candidates listed in
v<k>/rejected.json (verify_people.py: the posed person hit something) are left out, so a redraw
is reproducible too. v0 is the video's own survivors (specs/survivors.yaml).

Writes survivors.yaml (specs/survivors.yaml's schema, for place_people.py) and
ground_truth.json (DisasterWorld's: world frame, survivors [{id, position}]).
"""
import json
import math
import sys

import numpy as np
import yaml

from _paths import R, SPECS

OUT = R / "survivors"
RIGS = ["carla", "claudia", "eric", "manuel", "nathan", "sophia"]
LYING = ["lying_supine", "lying_prone", "lying_side_l", "lying_prone_reach"]
MIN_APART_M = 4.0


def write(k: int, people: list[dict]) -> None:
    d = OUT / f"v{k}"
    d.mkdir(parents=True, exist_ok=True)
    (d / "survivors.yaml").write_text(yaml.safe_dump(people, sort_keys=False, default_flow_style=None, width=200))
    (d / "ground_truth.json").write_text(json.dumps({"world_frame": "world", "survivors": [
        {"id": p["id"], "position": p["at"]} for p in people]}, indent=1))


def sample(k: int, cands: list[dict], mix: dict) -> list[dict]:
    rng = np.random.default_rng(k)
    rejected = set(json.loads((OUT / f"v{k}" / "rejected.json").read_text())) \
        if (OUT / f"v{k}" / "rejected.json").exists() else set()
    chosen: list[dict] = []
    for cls in ("covered", "canopy", "elevated", "ground"):     # scarcest first
        pool = [c for c in cands if c["class"] == cls and c["id"] not in rejected]
        for _ in range(mix[cls]):
            ok = [c for c in pool if all(math.dist(c["position"][:2], q["position"][:2]) >= MIN_APART_M for q in chosen)]
            if not ok:
                raise SystemExit(f"v{k}: no {cls} candidate left {MIN_APART_M} m from the others")
            c = ok[rng.integers(len(ok))]
            chosen.append(c)
            pool.remove(c)
    people = []
    for i, c in enumerate(chosen, start=1):
        lie = bool(c["lying_yaws_deg"]) and (not c["sit"] or rng.random() < 0.7)
        pose = str(rng.choice(LYING)) if lie else "sit_ground"
        yaw = float(rng.choice(c["lying_yaws_deg"])) + float(rng.uniform(-15, 15)) if lie else float(rng.uniform(-180, 180))
        people.append({"id": f"S{i:02d}", "at": c["position"], "pose": pose, "yaw": round(yaw, 1),
                       "rig": str(rng.choice(RIGS)), "frame": "", "candidate": c["id"],
                       "note": f"{c['class']} on {c['surface']}"})
    return people


if __name__ == "__main__":
    data = json.loads((OUT / "placement_candidates.json").read_text())
    write(0, [{**s, "at": [float(v) for v in s["at"]]} for s in yaml.safe_load(open(SPECS / "survivors.yaml"))])
    for k in (int(a) for a in sys.argv[1:]):
        people = sample(k, data["candidates"], data["video_class_mix"])
        write(k, people)
        print(f"v{k}:", " ".join(f"{p['id']}={p['note'].split()[0]}" for p in people))
