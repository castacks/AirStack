"""GT_hints.json — the object-level ground truth a frozen dataset scene ships.

    from disaster import gt_hints
    recs = gt_hints.build(stage, info, ssf, disaster="wildfire")
    gt_hints.write(path, recs, meta={...})

WHAT THIS IS FOR. `GT_people.json` says where the targets are. This says what
ELSE is in the scene and where — the confusers a detector has to reject, and
the structures a searcher can use as a prior. It is a HINT file, not an answer
key: every class in it is something a camera can see from the air and a human
labeller would draw a box around.

THE CLASS NAMES ARE FIXED AND ARE THE CALLER'S, VERBATIM. `CLASSES` is the
vocabulary; a disaster adds to it through `EXTRA_CLASSES` and never renames one.
Downstream evaluation keys off these strings, so a rename is a dataset break.

WHERE THE FACTS COME FROM, AND WHY NOT THE STAGE
------------------------------------------------
A house's damage level lives in the FILENAME of the archetype it references
(`house_ranch_rubble.usd`) and a USD reference does not publish that anywhere a
stage walk can read it. Same for a tree's species and level. So the levels are
threaded out of `scene_api.build_scene` in `info["house_objects"]` /
`info["tree_objects"]` and this module joins them to the stage only for the
BOX. Anything derivable from the plan is taken from the plan.

THE BBOX CACHE MUST INCLUDE BOTH PURPOSES. `[default_]` alone silently
declines to measure any prim authored under `render` — the same disagreement
that made `bake.export_object` report a clean file that `audit_archetype` then
found floating (see build-wildfire-scenes / build-tornado-scenes). Both, always.

AABB, NOT OBB — a known limitation, recorded rather than hidden. Every record
carries `yaw_deg`, so a consumer that wants the oriented box can rebuild it;
what is written is the world axis-aligned bound, which for a house at 45
degrees is up to 41% larger than the building. `UsdGeom.BBoxCache.
ComputeUntransformedBound` is NOT the way out: it does not exclude the prim's
own `rotateZ` (measured — see the tornado skill's `toss_prim` entry), so it
returns exactly the inflated box it looks like it avoids.
"""

import json
import math
import os

# ---------------------------------------------------------------------------
# the vocabulary
# ---------------------------------------------------------------------------

#: Classes every disaster emits. Order is the reporting order.
CLASSES = (
    "Building",
    "Damaged building",
    "Tree",
    "Burnt Tree",
    "Fallen Tree",
    "Debris",
    "Car",
    "Van",
    "Truck",
    "Toppled",
)

#: Scene-specific classes, by disaster. These are PLACES a searcher would
#: prioritise — the wildfire pool is the canonical one (it survives the fire
#: intact, it is the highest-contrast feature left in the black, and people
#: shelter at it). Kept separate from CLASSES so a consumer can tell the fixed
#: vocabulary from the per-disaster extension.
EXTRA_CLASSES = {
    "wildfire": ("Pool", "Parking Lot"),
    "urban_fire": ("Pool", "Parking Lot"),
    "tornado": ("Parking Lot",),
    "earthquake": ("Parking Lot",),
    "hurricane": ("Pool", "Parking Lot"),
}

# ---------------------------------------------------------------------------
# tree damage ladders -> classes
# ---------------------------------------------------------------------------
#
# EACH DISASTER HAS ITS OWN LADDER AND THEY DO NOT LINE UP. The wildfire ladder
# is scorched/torched/snag/fallen/stump; the tornado one is
# limbed/snapped/leaning/fallen. `snag` is a standing dead trunk and reads as a
# burnt tree; `snapped` is a wind-broken bole with its bark on and does NOT —
# it is a damaged tree, and calling it burnt in a tornado scene is a label
# error a detector would learn.
_TREE_CLASS = {
    "wildfire": {
        "pristine": "Tree",
        "scorched": "Burnt Tree",
        "torched": "Burnt Tree",
        "snag": "Burnt Tree",
        "stump": "Burnt Tree",
        "fallen": "Fallen Tree",
    },
    "tornado": {
        "pristine": "Tree",
        "limbed": "Tree",
        "snapped": "Fallen Tree",
        "leaning": "Fallen Tree",
        "fallen": "Fallen Tree",
    },
}
_TREE_CLASS["urban_fire"] = _TREE_CLASS["wildfire"]
_TREE_CLASS["earthquake"] = dict(_TREE_CLASS["tornado"])
_TREE_CLASS["hurricane"] = dict(_TREE_CLASS["tornado"])

#: Every house level that is NOT this one is "Damaged building".
_HOUSE_INTACT = "pristine"

# ---------------------------------------------------------------------------
# vehicles
# ---------------------------------------------------------------------------
#
# BY ASSET STEM, because that is the only thing that says what the vehicle IS.
# The pool's `tags` are about where a car may be PARKED (`residential`,
# `street`, `parked_only`), not about its body style, so a tag lookup puts the
# delivery van and the saloon in the same class.
_VAN_STEMS = ("delivery_van", "van", "gmc_motorhome", "motorhome", "rv")
_TRUCK_STEMS = ("pickup", "truck", "citybus", "bus", "lorry")

#: Beyond this the object is not standing on its wheels. 25 degrees is well
#: past what a kerb or a driveway crossfall produces and well short of the
#: 60-100 degrees a rolled car sits at.
TOPPLE_DEG = 25.0


def art_roll(axis_up):
    """The roll a placement carries just to STAND THE ASSET UP.

    `AssetPools.roll_of` gives every Y-up asset +90 degrees so its own up axis
    lands on world Z. That is an ART CORRECTION, not an attitude — the same
    class of trap as "`yaw_deg` IS NOT THE HEADING" in the people skill, one
    axis over. Testing the raw `roll_deg` against a topple threshold called
    **124 ordinary driveway cars "Toppled"** on the first 1 km wildfire plate,
    in a disaster where nothing tips a car at all. Measure the DEVIATION from
    this baseline instead.
    """
    return 90.0 if str(axis_up or "Z").upper() == "Y" else 0.0


def vehicle_class(usd):
    """"Car" / "Van" / "Truck" from an asset URL."""
    stem = os.path.basename(str(usd or "")).lower()
    stem = stem.rsplit(".", 1)[0]
    for s in _TRUCK_STEMS:
        if s in stem:
            return "Truck"
    for s in _VAN_STEMS:
        if s in stem:
            return "Van"
    return "Car"


def house_class(level):
    return "Building" if level == _HOUSE_INTACT else "Damaged building"


def tree_class(level, disaster="wildfire"):
    ladder = _TREE_CLASS.get(disaster) or _TREE_CLASS["wildfire"]
    return ladder.get(level, "Tree")


def is_toppled(roll_deg, pitch_deg, axis_up="Z"):
    """True when the object is not resting the way its art intends.

    Measured against `art_roll`, never against zero — see that function.
    """
    return (abs(float(roll_deg or 0.0) - art_roll(axis_up)) > TOPPLE_DEG
            or abs(float(pitch_deg or 0.0)) > TOPPLE_DEG)


# ---------------------------------------------------------------------------
# measuring
# ---------------------------------------------------------------------------

def _bbox_cache():
    from pxr import Usd, UsdGeom
    return UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                             useExtentsHint=False)


def _world_box(bc, stage, path, ssf):
    """`([x0,y0,z0], [x1,y1,z1])` in METRES, or None.

    A fresh range per call: `BBoxCache` is a CACHE, and one carried across a
    transform change hands back the pre-change bound (the lesson `tip_tree`'s
    seating bisection is built on). Nothing here moves prims, so one cache is
    safe for a whole pass — but never reuse one across an authoring step.
    """
    prim = stage.GetPrimAtPath(path)
    if not (prim and prim.IsValid()):
        return None
    try:
        rng = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    except Exception:
        return None
    if rng.IsEmpty():
        return None
    lo, hi = rng.GetMin(), rng.GetMax()
    s = 1.0 / float(ssf or 1.0)
    return ([lo[0] * s, lo[1] * s, lo[2] * s],
            [hi[0] * s, hi[1] * s, hi[2] * s])


def _rec(cls, box, **kw):
    d = {"class": cls}
    if box is not None:
        lo, hi = box
        d["bbox_min"] = [round(v, 3) for v in lo]
        d["bbox_max"] = [round(v, 3) for v in hi]
        d["centre"] = [round(0.5 * (lo[i] + hi[i]), 3) for i in range(3)]
        d["size_m"] = [round(hi[i] - lo[i], 3) for i in range(3)]
    d.update(kw)
    return d


# ---------------------------------------------------------------------------
# the pass
# ---------------------------------------------------------------------------

def build(stage, info, ssf, disaster="wildfire", verbose=True):
    """Every hint record for an assembled scene.

    *info* is `scene_api.build_scene`'s `info_out`. *ssf* is stage units per
    metre — every coordinate written out is METRES.
    """
    bc = _bbox_cache()
    parent = (info.get("parent") or "/World/stage/generated").rstrip("/")
    binfo = info.get("binfo") or {}
    out = []
    n_unmeasured = 0

    def add(cls, path, **kw):
        nonlocal n_unmeasured
        box = _world_box(bc, stage, path, ssf) if path else None
        if path and box is None:
            n_unmeasured += 1
        out.append(_rec(cls, box, prim_path=path, **kw))

    # ---- buildings --------------------------------------------------------
    for h in info.get("house_objects") or ():
        add(house_class(h["level"]), h["prim_path"],
            damage_level=h["level"], style=h["style"],
            yaw_deg=round(h["yaw_deg"], 2),
            row_home=bool(h.get("row")),
            burn_age_s=h.get("burn_age_s"))

    # ---- trees ------------------------------------------------------------
    for t in info.get("tree_objects") or ():
        add(tree_class(t["level"], disaster), t["prim_path"],
            damage_level=t["level"], species=t["species"],
            yaw_deg=round(t["yaw_deg"], 2),
            burn_age_s=t.get("burn_age_s"))

    # ---- vehicles ---------------------------------------------------------
    #
    # ONE INDEX, TWO SOURCES. `binfo["cars"]` is the plat's parked fleet
    # (driveways, kerbs, row-home courts) and `info["cars"]` is the survivor
    # pass's own (refuge lot, evacuation queue, cul-de-sac). They are separate
    # lists and a car can be in both — the refuge scenario ADOPTS parked cars
    # rather than adding more — so they are merged on prim path.
    seen = set()
    for src, origin in ((binfo.get("cars") or (), "plat"),
                        (info.get("cars") or (), "survivor")):
        for c in src:
            path = c.get("prim_path")
            if not path or path in seen:
                continue
            seen.add(path)
            toppled = is_toppled(c.get("roll_deg"), c.get("pitch_deg"),
                                 c.get("axis_up"))
            base = vehicle_class(c.get("usd"))
            add("Toppled" if toppled else base, path,
                subclass=base, origin=origin,
                # THE STEM, KEPT. `Car`/`Van`/`Truck` is decided from it and
                # nothing else, so a mis-class is only auditable if the record
                # says what it was decided from.
                asset=os.path.basename(str(c.get("usd") or "")),
                role=c.get("role"),
                yaw_deg=round(float(c.get("yaw_deg") or 0.0), 2),
                heading_deg=(round(float(c["heading_deg"]), 2)
                             if c.get("heading_deg") is not None else None),
                occupied=bool(c.get("occupied")))

    # ---- toppled street furniture, and the blockages ----------------------
    #
    # A blockage is TWO different things on the stage and they get different
    # classes: a `fallen_tree` blocker is a referenced tree archetype lying
    # across the carriageway (Fallen Tree), a `streetlight` blocker is a prim
    # already on the stage re-authored on its side (Toppled). The litter field
    # each one carries is one Debris record per BLOCKAGE, not per stick — a
    # 0.4 m limb is not a target and a hint file full of them is noise.
    for i, b in enumerate(info.get("blockers") or ()):
        kind = b.get("kind")
        if kind == "fallen_tree":
            add("Fallen Tree", "{0}/inst/blocker_{1}".format(parent, i),
                damage_level="fallen", blocks_road=True,
                yaw_deg=round(float(b.get("yaw_deg") or 0.0), 2))
        elif kind == "streetlight":
            add("Toppled", b.get("prim_path"), subclass="streetlight",
                blocks_road=True,
                yaw_deg=round(float(b.get("yaw_deg") or 0.0), 2))
        pieces = b.get("debris") or ()
        if pieces:
            lo = [1e18, 1e18, 1e18]
            hi = [-1e18, -1e18, -1e18]
            got = 0
            for j in range(len(pieces)):
                pb = _world_box(bc, stage,
                                "{0}/inst/blockdeb_{1}_{2}".format(parent, i, j),
                                ssf)
                if pb is None:
                    continue
                got += 1
                for k in range(3):
                    lo[k] = min(lo[k], pb[0][k])
                    hi[k] = max(hi[k], pb[1][k])
            if got:
                out.append(_rec("Debris", (lo, hi), pieces=got,
                                source="road_blockage", blocks_road=True,
                                prim_path="{0}/inst/blockdeb_{1}_*"
                                          .format(parent, i)))

    # ---- scene-specific: pools -------------------------------------------
    #
    # NOT MEASURED OFF THE STAGE. A pool is a HOLE cut in the ground sheet with
    # a water plane in it; there is no single prim whose bound is the pool, and
    # the ring is exact in the plan. Same reason `_record_pool` exists at all.
    if "Pool" in (EXTRA_CLASSES.get(disaster) or ()):
        for p in binfo.get("pools") or ():
            ring = p.get("water_ring") or []
            if len(ring) < 4:
                continue
            xs = [float(q[0]) for q in ring]
            ys = [float(q[1]) for q in ring]
            out.append(_rec("Pool", ([min(xs), min(ys), 0.0],
                                     [max(xs), max(ys), 0.0]),
                            yaw_deg=round(float(p.get("yaw_deg") or 0.0), 2),
                            house_index=p.get("house_index"),
                            water_ring=[[round(float(q[0]), 3),
                                         round(float(q[1]), 3)] for q in ring]))

    # ---- scene-specific: hard standing -----------------------------------
    #
    # The park's refuge lot and every row-home court. Both are the same object
    # in `suburb_park.parking_info`'s schema — bare asphalt with a bay
    # schedule — and both are where a wildfire evacuation collects (14 of the
    # Camp Fire's 31 temporary refuge areas were parking lots).
    if "Parking Lot" in (EXTRA_CLASSES.get(disaster) or ()):
        lots = []
        park_lot = ((binfo.get("park") or {}).get("parking") or {})
        if park_lot.get("centre"):
            lots.append(("park", park_lot))
        for cl in binfo.get("clusters") or ():
            lot = cl.get("parking") or {}
            if lot.get("centre"):
                lots.append(("row_home_court", lot))
        for kind, lot in lots:
            bays = lot.get("bays") or []
            cx, cy = lot["centre"][0], lot["centre"][1]
            if bays:
                # `suburb_park.parking_info` bays are `{"centre": (x, y),
                # "yaw_deg": ...}` — there is no flat x/y on them.
                xs = [float(b["centre"][0]) for b in bays]
                ys = [float(b["centre"][1]) for b in bays]
                box = ([min(xs) - 2.5, min(ys) - 2.5, 0.0],
                       [max(xs) + 2.5, max(ys) + 2.5, 0.0])
            else:
                box = ([cx - 10.0, cy - 10.0, 0.0], [cx + 10.0, cy + 10.0, 0.0])
            out.append(_rec("Parking Lot", box, subclass=kind,
                            bays=len(bays),
                            centre_xy=[round(float(cx), 3), round(float(cy), 3)]))

    for i, r in enumerate(out):
        r["id"] = i

    if verbose:
        print("[gt_hints] {0} record(s), {1} class(es){2}".format(
            len(out), len({r["class"] for r in out}),
            "" if not n_unmeasured
            else "; {0} prim(s) could not be measured".format(n_unmeasured)))
        for cls in list(CLASSES) + list(EXTRA_CLASSES.get(disaster) or ()):
            n = sum(1 for r in out if r["class"] == cls)
            if n:
                print("[gt_hints]   {0:<20} {1:>6}".format(cls, n))
    return out


def summarise(recs):
    """`{class: count}`, in `CLASSES` order then whatever else turned up."""
    tally = {}
    for r in recs:
        tally[r["class"]] = tally.get(r["class"], 0) + 1
    ordered = {}
    for c in CLASSES:
        if c in tally:
            ordered[c] = tally.pop(c)
    for c in sorted(tally):
        ordered[c] = tally[c]
    return ordered


def write(path, recs, meta=None):
    d = os.path.dirname(os.path.abspath(path))
    if d:
        os.makedirs(d, exist_ok=True)
    payload = {"meta": dict(meta or {}), "counts": summarise(recs),
               "classes": list(CLASSES), "hints": recs}
    with open(path, "w") as fh:
        json.dump(payload, fh, indent=1)
    print("[gt_hints] wrote {0} ({1} record(s))".format(path, len(recs)))
    return path
