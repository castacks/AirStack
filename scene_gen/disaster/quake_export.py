"""Dataset sidecars for an assembled urban-earthquake stage.

The earthquake launcher predates ``scene_api.build_scene``'s frozen-dataset
path, so its damage records are not shaped like the suburban ``info`` object.
This module is the small, deterministic adapter: it never changes the stage;
it only converts the already-selected buildings, casualties and vehicles to
the common ``GT_people.json`` / ``GT_hints.json`` / ``build_stats.json``
contract.
"""

import json
import os


CASUALTY_STATES = frozenset((
    "prone_casualty", "interior_casualty", "rubble_casualty",
))


def base_grade(value):
    """Canonical damage grade from labels such as ``AEC_DG4+tilt``."""
    value = str(value or "DG0")
    if value.startswith("AEC_"):
        value = value[4:]
    return value.split("+", 1)[0]


def people_document(records, report):
    """Return the casualty-only ground-truth document used by the launcher."""
    records = list(records or ())
    active = [r for r in records if r.get("active", True)]
    ordinary = [r for r in active if r.get("state") not in CASUALTY_STATES]
    if ordinary:
        raise ValueError("earthquake dataset contains standing/walking people")
    report = dict(report or {})
    return {
        "schema": "airstack.earthquake_people/2",
        "count": len(active),
        "prone_casualties": len(active),
        "interior_casualties": int(report.get("interior_casualties", 0)),
        "rubble_casualties": int(report.get("rubble_casualties", 0)),
        "standing_or_walking": 0,
        "generic_humans_deactivated": int(
            report.get("generic_humans_deactivated", 0)),
        "cover_pieces": int(report.get("cover_pieces", 0)),
        "underfilled": int(report.get("underfilled", 0)),
        "people": active,
    }


def house_objects(records):
    """Adapt ``quake.assemble`` records to ``gt_hints`` building records."""
    out = []
    for rec in records or ():
        path = rec.get("prim")
        if not path:
            continue
        grade = base_grade(rec.get("grade"))
        out.append({
            "prim_path": str(path),
            "style": str(rec.get("style") or "unknown"),
            "level": "pristine" if grade == "DG0" else str(rec.get("grade")),
            "yaw_deg": float(rec.get("yaw_deg", 0.0)),
            "row": False,
        })
    return out


def cars_from_placements(placements):
    """Best-effort common vehicle records from the city placement list."""
    out = []
    for row in placements or ():
        category = str(row.get("category") or "").lower()
        if category not in ("car", "vehicle", "truck", "van"):
            continue
        path = row.get("prim_path")
        if not path:
            continue
        out.append({
            "prim_path": str(path),
            "usd": str(row.get("usd") or ""),
            "roll_deg": float(row.get("roll_deg", 0.0)),
            "pitch_deg": float(row.get("pitch_deg", 0.0)),
            "yaw_deg": float(row.get("yaw_deg", 0.0)),
            "heading_deg": row.get("heading_deg"),
            "axis_up": str(row.get("axis_up") or "Z"),
            "occupied": False,
            "role": row.get("role"),
        })
    return out


def write_sidecars(stage, out_dir, stats, placements, people_doc, ssf,
                   scene_config, quake_seed, arch_dir, gac_arch_dir,
                   region_m, magnitude=None, people_variant=1):
    """Write all non-image files required beside a frozen quake scene."""
    from disaster import gt_hints

    os.makedirs(out_dir, exist_ok=True)
    houses = house_objects((stats or {}).get("records", ()))
    cars = cars_from_placements(placements)
    info = {
        "parent": "/World/stage/generated",
        "binfo": {"cars": cars},
        "house_objects": houses,
        "tree_objects": [],
        "cars": [],
        "blockers": [],
    }
    hints = gt_hints.build(stage, info, ssf, disaster="earthquake")
    meta = {
        "scene_config": scene_config,
        "seed": int(quake_seed),
        "people_variant": int(people_variant),
        "disaster": "earthquake",
        "region_m": [float(v) for v in region_m],
        "magnitude": None if magnitude is None else float(magnitude),
        "arch_dir": arch_dir,
        "gac_arch_dir": gac_arch_dir,
        "units": "metres, world frame, plate centred on the origin",
    }
    gt_hints.write(os.path.join(out_dir, "GT_hints.json"), hints, meta=meta)
    with open(os.path.join(out_dir, "GT_people.json"), "w") as fh:
        json.dump(people_doc, fh, indent=1)
    build = dict(meta)
    build.update({
        "buildings": int((stats or {}).get("buildings", len(houses))),
        "building_tally": dict((stats or {}).get("tally", {})),
        "tilted": int((stats or {}).get("tilted", 0)),
        "missing_archetypes": int((stats or {}).get("missing", 0)),
        "people": int(people_doc.get("count", 0)),
        "standing_or_walking": int(people_doc.get("standing_or_walking", 0)),
        "cars": len(cars),
        "hint_counts": gt_hints.summarise(hints),
    })
    with open(os.path.join(out_dir, "build_stats.json"), "w") as fh:
        json.dump(build, fh, indent=1)
    return {"hints": hints, "build_stats": build}
