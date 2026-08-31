#!/usr/bin/env python
"""dtc_grid_probe — what `gac_fire.prepare` MEASURES on a downtowncity block,
without slicing it or burning it.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/dtc_grid_probe.py Amar_Tower Building_12 ..."

With no arguments it runs the six picks the `dtc:` fire row is built from.
`--all` runs every asset in `_plans/dtc_buildings.json`.

Bare `pxr` (`tools/usd_python.sh`), no Kit, no GPU, no slicer lock — safe
beside a live session and beside another agent's bake.

WHAT IT PRINTS, AND WHY EACH NUMBER IS HERE
-------------------------------------------
* **scale** — the asset's own `metersPerUnit` (`gac_fire.asset_scale`), so a
  pack whose units were assumed rather than measured shows up immediately.
  downtowncity is `mpu = 1`; GreatAmericanCity is 0.01.
* **bbox trim** — whether `gac_fire.trim_bbox` had to take baked-in
  landscaping out of the measured box (Amar_Tower's roof garden).
* **storey grid** — `gac_storey_slice.grid_for`'s own result: the period, the
  CONFIDENCE, and whether it came back `measured` or fell back to
  `regular_grid`. `gac_slice.MIN_CONFIDENCE` (0.55) is the cut; below it a
  regular grid is a legitimate result, not a failure, but the two must not be
  confused when judging a slice (`grid_for`'s docstring).
* **window islands per side** — `gac_fire.window_rects`, the openings the
  fire vents through and the thing `prepare` ranks elevations by. A side with
  no islands never gets a fire, however hot the level.
* **construction type** — `quake_sliced.construction_type` on the measured
  height, i.e. which `urban_fire.LADDER` the level will be looked up in.
* **planned band** — `urban_fire.plan_fire` at the level given, so the row's
  level assignment can be sanity-checked before any bake runs.
"""
import json
import os
import random
import resource
import sys
import time

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom                                   # noqa: E402
from detail import gac_slice as gsl                            # noqa: E402
from detail import gac_storey_slice as gss                     # noqa: E402
from disaster import gac_fire as gf                            # noqa: E402
from disaster import quake_sliced as qs                        # noqa: E402
from disaster import urban_fire as uf                          # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PLANS = os.path.join(os.path.dirname(HERE), "_plans", "dtc_buildings.json")

#: the six the row is built from, with the level each is planned at
PICKS = [("Building_11", "F1"), ("Building_12", "F2"), ("Carved_18", "F3"),
         ("Carved_14", "F4"), ("Carved_13", "F5"), ("Amar_Tower", "F3")]


def probe(name, level, kind="dtc", seed=7, verbose=True):
    t0 = time.time()
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    cell = "/W/g0"
    UsdGeom.Xform.Define(st, cell)

    k, asset = gf.split_kind(name, kind)
    pack = gf.PACKS[k]
    url = gf.asset_url(asset, k)
    scale = gf.asset_scale(url, pack["scale"], verbose=False)
    src = gf.place_source(st, cell, url, scale)
    if not src:
        return {"name": asset, "error": "nothing composed"}
    wins, bbox0 = gsl.window_centres(st, src)
    bbox, trim = gf.trim_bbox(st, src, bbox0, pack["bbox_exclude"],
                              verbose=False)
    # `grid_for` REPORTS ITS FALLBACK'S confidence (0.0), not the measured
    # one it rejected, so ask `measure_grid` directly as well — the number
    # that says HOW FAR under `gsl.MIN_CONFIDENCE` an asset fell is the whole
    # point of this probe.
    n_win = sum(len(v) for v in (wins or {}).values())
    gm = gsl.measure_grid(wins, bbox, verbose=False, name=asset) if wins else {}
    g, measured = gss.grid_for(st, src, bbox, wins, name=asset, verbose=False)
    rects = gf.window_rects(st, src)
    mesh = gss.read_mesh(st, src, verbose=False)
    m = gf.mass_from_grid(g, bbox, mesh=gf.mesh_without_props(
        mesh, pack["bbox_exclude"]))
    H = m["top"] - m["z0"]
    btype = (qs.construction_type(url, H=H) if pack["construction_table"]
             else ("urm" if H <= 25.0 else "rc"))
    n_st = len(m["levels"])
    info = {"style": pack["style_prefix"] + asset, "family": "01",
            "type": btype, "x": 0.0, "y": 0.0, "yaw": 0.0,
            "masses": {"main": m}, "elements": [], "H": H}
    rng = random.Random(seed)
    origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    ranked = sorted(rects.keys(), key=lambda sd: -len(rects[sd]))
    ranked = [sd for sd in ranked if rects[sd]] or ["S"]
    n_side = 1 if level in ("F1", "F2") else (
        2 if level == "F3" else min(len(ranked), rng.randint(2, 4)))
    sides = tuple(ranked[:max(1, n_side)])
    fire = uf.plan_fire(info, level, rng, origin=origin, sides=sides)
    rec = {
        "name": asset, "kind": k, "level": level, "scale": scale,
        "W": round(m["W"], 1), "D": round(m["D"], 1), "H": round(H, 1),
        "top": round(m["top"], 1), "deck_z": round(float(m.get("deck_z") or 0), 1),
        "deck_note": m.get("deck_note"),
        "trim": trim,
        "storey_h": round(float(g.get("storey_h") or 0), 2),
        "confidence": round(float(g.get("confidence") or 0.0), 3),
        "win_centres": n_win,
        "measured_conf": round(float(gm.get("confidence") or 0.0), 3),
        "measured_rise": round(float(gm.get("storey_h") or 0.0), 2),
        "measured_why": gm.get("why"),
        "min_confidence": gsl.MIN_CONFIDENCE,
        "measured": bool(measured), "n_storeys": n_st,
        "islands": {sd: len(rects.get(sd) or []) for sd in ("S", "E", "N", "W")},
        "btype": btype,
        "ladder": [r for r, _kw in uf.LADDER.get(btype, {}).get(level, [])],
        "origin": fire["origin"], "band": (fire["storeys"][0], fire["top"]),
        "sides": "/".join(fire["sides"]), "roof": bool(fire.get("roof")),
        "sec": round(time.time() - t0, 1),
        "rss_mb": round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0),
    }
    if verbose:
        print("%-13s %-4s mpu=%-5g %6.1f x %6.1f x %6.1f m  storey %5.2f m "
              "conf %5.3f %-8s n=%-3d islands S/E/N/W %4d/%4d/%4d/%4d  %-8s "
              "origin %2d band %2d-%-2d on %-7s roof=%-5s %5.1fs %5d MB"
              % (rec["name"], level, scale, rec["W"], rec["D"], rec["H"],
                 rec["storey_h"], rec["confidence"],
                 "MEASURED" if measured else "regular",
                 rec["n_storeys"], rec["islands"]["S"], rec["islands"]["E"],
                 rec["islands"]["N"], rec["islands"]["W"], btype,
                 rec["origin"], rec["band"][0], rec["band"][1], rec["sides"],
                 rec["roof"], rec["sec"], rec["rss_mb"]), flush=True)
        print("      window-centre sample %d face(s); measure_grid: rise "
              "%.2f m confidence %.3f (cut %.2f) %s" % (
                  n_win, rec["measured_rise"], rec["measured_conf"],
                  gsl.MIN_CONFIDENCE,
                  "-> USED" if measured else "-> REJECTED, regular grid "
                  "%.2f m x%d%s" % (rec["storey_h"], rec["n_storeys"],
                                    "" if not gm.get("why")
                                    else " (%s)" % gm["why"])), flush=True)
        if trim:
            print("      trim: %s" % trim, flush=True)
        print("      deck z=%.1f (top %.1f) — %s" % (
            rec["deck_z"], rec["top"], rec["deck_note"]), flush=True)
        print("      ladder %s -> %s" % (btype, ", ".join(rec["ladder"]) or "(none)"),
              flush=True)
    return rec


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if "--all" in sys.argv[1:]:
        with open(PLANS) as fh:
            todo = [(r["name"], "F3") for r in json.load(fh)]
    elif args:
        todo = []
        for a in args:
            nm, _s, lv = a.partition(":")
            todo.append((nm, lv or "F3"))
    else:
        todo = list(PICKS)
    out = []
    for nm, lv in todo:
        try:
            out.append(probe(nm, lv))
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("%-13s FAILED %s" % (nm, exc), flush=True)
    print()
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
