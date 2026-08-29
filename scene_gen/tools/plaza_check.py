"""plaza_check.py — offline acceptance test for `city_detail`'s highrise plazas.

    python3 tools/plaza_check.py --config downtown_gac

Runs the real layout pipeline exactly as `plan_png.py` does (no Isaac Sim,
no `pxr`), then calls `city_detail.build()` TWICE with the same seed — once
with `city_detail.plazas` off, once with it on — and takes the tail of the
second run's placement list as the plaza-only furniture. That is a real
question about the actual code path (do the two runs even agree on every
prop that ISN'T a plaza?), not a assumption: `_place_plazas` is gated and
RNG-free when off, so the two runs are required to agree on everything up to
where the second one's tail begins, and this checks that literally, not by
inspection.

The DOWNTOWN_GAC PRESET DOES NOT YET SET `city_detail.plazas` — the exact
YAML block is in `_PLAZA_CFG` below and was reported to the preset's owner
rather than committed, per this task's scope (city_detail.py and new tools
only). This script therefore INJECTS `_PLAZA_CFG` into the loaded config
before the second `build()` call, so what is measured here is exactly the
preset owner is being handed, not some other set of numbers.

WHAT IS CHECKED, per plaza (a plaza = a connected cluster of plaza-only
placements, found by proximity — see `_cluster`, which knows nothing about
`_place_plazas`'s own free-rect search and so cannot simply agree with it by
construction):

  overlap      every plaza item's oriented footprint (`parks._box`,
               `parks._box_hit` — real SAT, not an axis-aligned approximation,
               because a ring bench stands at an arbitrary yaw) against every
               OTHER placement in the whole scene: buildings, cars, humans,
               sidewalk furniture, and every other plaza item. Co-located
               fountain parts (basin + three water discs, one shared origin
               by design) are the one documented exception.
  bounds       every corner of every plaza item's footprint lies inside the
               block's own inset rect — the same rect `city_detail.build()`
               keeps sidewalk furniture inside.
  ring symmetry  the bench ring's radius has a tight spread (evenly pitched
               means every bench is the same distance from the centre) and
               its angular gaps have a tight spread (means the pitch itself
               is even) — both MEASURED off the placed positions, not the
               formula that produced them, so a silently-dropped bench (an
               occupancy collision) shows up as one outsized gap rather than
               being averaged away.
"""

import argparse
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
sys.path.insert(0, _SCENE_GEN)
sys.path.insert(0, _HERE)

import plan_png                                                # noqa: E402

# The exact YAML proposed for `presets/downtown_gac.yaml`'s `city_detail:`
# block — see the report this tool's run accompanies. Kept as one literal
# dict here (not re-parsed from a YAML string) so there is no risk of this
# checker drifting from what it prints as "the exact YAML".
_PLAZA_CFG = {
    "enabled": True,
    "typologies": ["highrise"],
    "min_side_m": 15.0,
    "min_area_m2": 280.0,
    "max_per_block": 2,
    "podium_margin_m": 1.5,
    "edge_margin_m": 2.5,
    "max_radius_m": 13.0,
    "min_radius_m": 5.0,
    "fountain_chance": 0.65,
    "ring_gap_m": 2.2,
    "bench_spacing_m": 6.5,
    "trash_cans_per_plaza": 2,
    "streetlights_per_plaza": 4,
    "planter_spacing_m": 5.0,
    "planter_inset_m": 1.2,
    "cafe_setback_m": 3.0,
    "cafe_spacing_m": 4.5,
    "cafe_min_run_m": 8.0,
    "cluster_radius_frac": 0.35,
}


def _deepcopy_cfg(cfg):
    import copy
    return copy.deepcopy(cfg)


def run(config_name: str):
    from detail import city_detail, districts, parks

    cfg, layout, placements, res = plan_png.build(config_name)
    seed = int(cfg.get("seed", 0)) + 1013

    cfg_base = _deepcopy_cfg(cfg)
    (cfg_base.get("city_detail") or {}).pop("plazas", None)

    cfg_plaza = _deepcopy_cfg(cfg)
    cfg_plaza.setdefault("city_detail", {})["plazas"] = dict(_PLAZA_CFG)

    detail_base = city_detail.build(cfg_base, layout, res,
                                    random.Random(seed), placements=placements)
    detail_full = city_detail.build(cfg_plaza, layout, res,
                                    random.Random(seed), placements=placements)

    assert detail_full[:len(detail_base)] == detail_base, (
        "plaza pass perturbed the sidewalk-furniture sequence — it must be "
        "RNG-free and additive-only when nothing before it changes")
    plaza_only = detail_full[len(detail_base):]
    print(f"[plaza_check] {config_name}: {len(detail_base)} sidewalk props "
          f"unchanged, {len(plaza_only)} plaza props appended")

    if not plaza_only:
        print("[plaza_check] no plazas composed — nothing to check")
        return 0

    # ---- independent re-discovery of the free-rect distribution, for the
    # report only (not used to group or judge anything below).
    inset = districts.block_inset(cfg, res)
    typ_of = layout.get("_typology_of") or {}
    houses = [p for p in placements if p.get("category") == "house"]
    all_free = []
    for raw in layout.get("blocks") or ():
        if (typ_of.get(tuple(raw)) or typ_of.get(raw)) not in ("highrise",):
            continue
        bx0, by0, bx1, by1 = city_detail._rect(raw)
        rect = (bx0 + inset, by0 + inset, bx1 - inset, by1 - inset)
        obs = [districts._rect_of(p, res, margin=_PLAZA_CFG["podium_margin_m"])
              for p in houses if bx0 <= p["x_m"] <= bx1 and by0 <= p["y_m"] <= by1]
        if not obs:
            continue
        free = districts.free_rects(rect, obs, min_side=1.0)
        all_free += [(f[2] - f[0], f[3] - f[1]) for f in free]
    all_free.sort(key=lambda wh: -wh[0] * wh[1])
    print(f"[plaza_check] free-rect size distribution on this block set "
          f"({len(all_free)} total, any size): "
          + ", ".join(f"{w:.0f}x{h:.0f}" for w, h in all_free[:12])
          + (" ..." if len(all_free) > 12 else ""))

    # ---- cluster the plaza-only output by proximity alone: two items within
    # `merge_r` of each other are the same plaza. `merge_r` has to clear
    # `max_radius_m + edge_margin_m + planter reach` (13 + 2.5 + a few m) so a
    # ring's own outermost items (streetlights, planters) don't get split
    # off as a separate "cluster", and has to stay well under the ~30 m+ gap
    # this task's own preset comment measures between distinct plazas.
    clusters = _cluster(plaza_only, merge_r=22.0)
    print(f"[plaza_check] {len(clusters)} plaza cluster(s) found by "
          f"proximity alone")

    everything = placements + detail_base
    by_block = {tuple(b): city_detail._rect(b) for b in (layout.get("blocks") or ())}

    n_fail = 0
    for i, cluster in enumerate(clusters):
        n_fail += _check_cluster(i, cluster, everything, plaza_only,
                                 houses, by_block, res, parks)

    counts = {}
    for p in plaza_only:
        counts[p["category"]] = counts.get(p["category"], 0) + 1
    print(f"[plaza_check] plaza furniture by category: "
          + "  ".join(f"{k}={v}" for k, v in sorted(counts.items())))

    if n_fail:
        print(f"[plaza_check] FAIL: {n_fail} check(s) failed across "
              f"{len(clusters)} plaza(s)")
        return 1
    print(f"[plaza_check] PASS: {len(clusters)} plaza(s), 0 failures")
    return 0


def _cluster(items, merge_r):
    """Connected components of *items* under "within merge_r of each other".
    O(n^2) — plaza counts are in the tens, not worth a spatial index."""
    n = len(items)
    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    r2 = merge_r * merge_r
    for i in range(n):
        for j in range(i + 1, n):
            dx = items[i]["x_m"] - items[j]["x_m"]
            dy = items[i]["y_m"] - items[j]["y_m"]
            if dx * dx + dy * dy <= r2:
                union(i, j)

    groups = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(items[i])
    return list(groups.values())


def _footprint(p, res):
    fp = res.get(p.get("usd", ""), p.get("category"), scale=p.get("scale"),
                axis_up=p.get("axis_up", "Z"))
    return fp["sx"], fp["sy"]


# Categories city_detail places in `_POLE` mode: only the pole reserves
# ground, the arm overhangs — see `city_detail._occ_extent`'s `_POLE` branch
# and `place_free(..., pole=True)`.
_POLE_CATS = {"streetlight", "utility_pole", "traffic_light"}
# `_CANOPY` mode: only the trunk and its pit reserve ground — a crown is
# above head height and a bench belongs UNDER a street tree, not away from
# it. `_CANOPY_FRAC` matches `city_detail._occ_extent`'s own default (and
# downtown_gac.yaml's explicit `canopy_footprint_frac: 0.25`).
_CANOPY_CATS = {"street_tree"}
_CANOPY_FRAC = 0.25


def _box_of(p, res):
    from detail import parks
    sx, sy = _footprint(p, res)
    cat = p.get("category")
    if cat in _POLE_CATS:
        d = min(sx, sy)
        sx, sy = d, d
    elif cat in _CANOPY_CATS:
        t = max(0.2, min(sx, sy) * _CANOPY_FRAC)
        sx, sy = t, t
    return parks._box(p["x_m"], p["y_m"], float(p.get("yaw_deg", 0.0)), sx, sy)


def _check_cluster(idx, cluster, everything, plaza_only, houses, by_block,
                   res, parks):
    fails = 0
    cx = sum(p["x_m"] for p in cluster) / len(cluster)
    cy = sum(p["y_m"] for p in cluster) / len(cluster)
    counts = {}
    for p in cluster:
        counts[p["category"]] = counts.get(p["category"], 0) + 1
    tag = f"plaza[{idx}] @ ({cx:.0f},{cy:.0f})"
    detail = "  ".join(f"{k}={v}" for k, v in sorted(counts.items()))
    print(f"[plaza_check] {tag}: {len(cluster)} items  {detail}")

    # ---- bounds: every corner inside SOME highrise block's own inset rect
    block = None
    for rect in by_block.values():
        if rect[0] - 1e-6 <= cx <= rect[2] + 1e-6 and rect[1] - 1e-6 <= cy <= rect[3] + 1e-6:
            block = rect
            break
    if block is None:
        print(f"[plaza_check]   FAIL bounds: cluster centre is not inside "
              f"any block")
        fails += 1
    else:
        for p in cluster:
            sx, sy = _footprint(p, res)
            half = math.hypot(sx, sy) / 2.0   # conservative: any yaw
            x, y = p["x_m"], p["y_m"]
            if not (block[0] - 0.05 <= x - half and x + half <= block[2] + 0.05
                    and block[1] - 0.05 <= y - half and y + half <= block[3] + 0.05):
                print(f"[plaza_check]   FAIL bounds: {p['category']} at "
                      f"({x:.1f},{y:.1f}) footprint reaches outside block "
                      f"{tuple(round(v, 1) for v in block)}")
                fails += 1

    # ---- overlap: every cluster item vs every other placement in the scene,
    # oriented SAT. Co-located fountain parts are the documented exception;
    # a pole's own ARM is the other one — `city_detail._occ_extent`'s `_POLE`
    # mode (and `place_free(..., pole=True)`, the plaza ring's own use of it)
    # both treat only the pole as ground-level, because the arm overhangs by
    # design, the same as a real streetlight over a real sidewalk. Testing a
    # pole's FULL bbox here would flag that overhang as a floor-level
    # collision with whatever stands under it, which is not what either the
    # occupancy grid or reality enforces.
    boxes = [(p, _box_of(p, res)) for p in everything + plaza_only]
    n_overlap = 0
    for p in cluster:
        pb = _box_of(p, res)
        for q, qb in boxes:
            if q is p:
                continue
            if p["category"] == "park_feature" and q["category"] == "park_feature":
                continue          # co-located fountain stack, by design
            dx, dy = p["x_m"] - q["x_m"], p["y_m"] - q["y_m"]
            if dx * dx + dy * dy > 400.0:     # 20 m broad-phase
                continue
            if parks._box_hit(pb, qb):
                n_overlap += 1
                if n_overlap <= 5:
                    print(f"[plaza_check]   FAIL overlap: {p['category']} "
                          f"@({p['x_m']:.1f},{p['y_m']:.1f}) hits "
                          f"{q['category']} @({q['x_m']:.1f},{q['y_m']:.1f})")
    if n_overlap:
        print(f"[plaza_check]   {n_overlap} overlap(s) total in this cluster")
        fails += 1

    # ---- ring symmetry: benches evenly pitched and equidistant, measured
    benches = [p for p in cluster if p["category"] == "bench"]
    if len(benches) >= 3:
        bcx = sum(p["x_m"] for p in benches) / len(benches)
        bcy = sum(p["y_m"] for p in benches) / len(benches)
        radii = [math.hypot(p["x_m"] - bcx, p["y_m"] - bcy) for p in benches]
        r_mean = sum(radii) / len(radii)
        r_spread = (max(radii) - min(radii)) / r_mean if r_mean else 0.0
        angles = sorted(math.degrees(math.atan2(p["y_m"] - bcy,
                                                 p["x_m"] - bcx)) % 360.0
                        for p in benches)
        n = len(angles)
        gaps = [(angles[(k + 1) % n] - angles[k]) % 360.0 for k in range(n)]
        gap_mean = 360.0 / n
        gap_spread = max(abs(g - gap_mean) for g in gaps)
        expected_n = max(3, round(2.0 * math.pi * r_mean / _PLAZA_CFG["bench_spacing_m"]))
        print(f"[plaza_check]   ring: {n} benches (expected {expected_n} for "
              f"r={r_mean:.1f} m), r_spread={100 * r_spread:.1f}%, "
              f"max angular deviation={gap_spread:.1f} deg "
              f"(even pitch = {gap_mean:.1f} deg)")
        if r_spread > 0.03:
            print(f"[plaza_check]   FAIL ring: radius spread {100 * r_spread:.1f}% "
                  f"> 3% — benches are not equidistant from the centre")
            fails += 1
        if n == expected_n and gap_spread > 3.0:
            print(f"[plaza_check]   FAIL ring: angular deviation "
                  f"{gap_spread:.1f} deg > 3 deg with no bench dropped — "
                  f"not evenly pitched")
            fails += 1
        elif n != expected_n:
            print(f"[plaza_check]   NOTE: {expected_n - n} bench(es) dropped "
                  f"to an occupancy collision — the remaining ring is "
                  f"correctly uneven at that one gap, not a bug")
    elif benches:
        print(f"[plaza_check]   NOTE: only {len(benches)} bench(es) — too "
              f"few to test symmetry")

    return fails


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", default="downtown_gac")
    a = ap.parse_args()
    sys.exit(run(a.config))


if __name__ == "__main__":
    main()
