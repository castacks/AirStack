"""damage_flow — fracture one building to a structural level.

The per-building break logic, lifted verbatim from
`suburb_mini_wildfire_launch_script.py` so the archetype bake harness produces
IDENTICAL wreckage to the live scene. `BREAK_PLAN` and `cascade_supports` live
here too. The mini launcher keeps its own inline copy for now; this module is
the reusable one the archetype pipeline calls.
"""

import math
import random

import numpy as np

# n_walls, partial_p, seeds, cut_range, n_floors — per structural level.
BREAK_PLAN = {
    "pristine":         (0, 0.00, 0,  (0.00, 0.00), 0),
    "scorched":         (0, 0.00, 0,  (0.00, 0.00), 0),
    "roof_collapsed":   (1, 0.90, 9,  (0.58, 0.80), 0),
    "partial_collapse": (3, 0.75, 11, (0.38, 0.62), 1),
    "burned_out":       (5, 0.55, 12, (0.20, 0.44), 2),
    "rubble":           (8, 0.30, 14, (0.06, 0.26), 4),
}

DAMAGED_LEVELS = ("roof_collapsed", "partial_collapse", "burned_out", "rubble")


def cascade_supports(items, broken, radius_m=4.0, z_eps=0.5):
    """Close a break set under support: anything ABOVE a broken module within
    `radius_m` also falls, until nothing new is added."""
    by_id = {id(q): q for q in items}
    changed = True
    while changed:
        changed = False
        low = [by_id[i] for i in broken if i in by_id]
        for q in items:
            if id(q) in broken:
                continue
            zq = float(q.get("z_m", 0.0))
            for b in low:
                if float(b.get("z_m", 0.0)) >= zq - z_eps:
                    continue
                if math.hypot(q["x_m"] - b["x_m"],
                              q["y_m"] - b["y_m"]) <= radius_m:
                    broken.add(id(q))
                    changed = True
                    break
    return broken


def damage_building(stage, parent, items, tag, level, finish, mats, rng, nrng):
    """Fracture the modules of one building to `level`. Returns fragment paths.

    `items` is the building's module placements (each with `prim_path`,
    `category`, `x_m`/`y_m`/`z_m`). `mats` is `damage.char_materials(...)`.
    Faithful copy of the mini launcher's per-building loop.
    """
    from pxr import UsdShade

    from . import damage
    from . import vtk_fracture as fracture

    n_walls, partial_p, seeds, cut_range, n_floors = BREAK_PLAN[level]
    frags = []
    if not seeds:
        return frags

    roofs = [q for q in items
             if damage._sub_of(q.get("category")) in ("roof", "bay_roof")]
    walls = [q for q in items if damage._sub_of(q.get("category")) == "wall"]
    floors = [q for q in items if damage._sub_of(q.get("category")) == "floor"]

    walls_sorted = sorted(walls, key=lambda q: float(q.get("z_m", 0.0)))
    take = min(n_walls, len(walls_sorted))
    pick = walls_sorted[:take] if take else []
    if take and len(walls_sorted) > take:
        pool = walls_sorted[:min(len(walls_sorted), take + 3)]
        pick = rng.sample(pool, take)

    floors_sorted = sorted(floors, key=lambda q: -float(q.get("z_m", 0.0)))
    pick_f = floors_sorted[:min(n_floors, len(floors_sorted))]

    light = []
    if level in ("partial_collapse", "burned_out", "rubble"):
        light = [q for q in walls if q not in pick]

    broken_ids = cascade_supports(
        items, {id(q) for q in pick + pick_f + roofs + light})
    light_ids = {id(q) for q in light}
    targets = [q for q in items if id(q) in broken_ids]
    for q in targets:
        path = q.get("prim_path")
        if not path:
            continue
        if damage.is_incombustible(q.get("category")):
            continue
        out = "{0}/brk_{1}_{2}".format(parent, tag, path.rsplit("/", 1)[-1])
        src_tex = damage.bound_texture(stage, path)
        is_wall = damage._sub_of(q.get("category")) == "wall"
        cut = ((0.74, 0.93) if id(q) in light_ids else cut_range)
        if is_wall and (id(q) in light_ids
                        or (q in pick and rng.random() < partial_p)):
            st, lo = fracture.fracture_partial(
                stage, path, out, n_pieces=seeds, rng=nrng,
                cut_frac=rng.uniform(*cut), mode="char")
            frags.extend(st)
            frags.extend(lo)
            if src_tex:
                heavy = damage.scorched_material(
                    stage, parent, None, len(damage.SOOT_LEVELS) - 1,
                    texture=src_tex, triplanar=True)
                for pth in st + lo:
                    pr = stage.GetPrimAtPath(pth)
                    if pr and pr.IsValid():
                        UsdShade.MaterialBindingAPI(pr).Bind(heavy)
        else:
            hi_floor = (damage._sub_of(q.get("category")) == "floor"
                        and float(q.get("z_m", 0.0)) > 1.5)
            made = fracture.fracture_prim(
                stage, path, out,
                n_pieces=seeds + (3 if hi_floor else 0), rng=nrng,
                mode="char", rough=0.045, verbose=False,
                consume=0.55, consume_pool=1.02)
            frags.extend(made)
            heavy = (damage.scorched_material(
                stage, parent, None, len(damage.SOOT_LEVELS) - 1,
                texture=src_tex, triplanar=True) if src_tex else None)
            for pth in made:
                pr = stage.GetPrimAtPath(pth)
                if not pr or not pr.IsValid():
                    continue
                if heavy is not None and rng.random() < 0.25:
                    UsdShade.MaterialBindingAPI(pr).Bind(heavy)
                else:
                    UsdShade.MaterialBindingAPI(pr).Bind(
                        damage._pick(rng, finish or "char", mats))
    return frags
