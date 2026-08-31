"""_dtc_open_probe — where are a merged asset's window ISLANDS, by side and
storey, and how many fire EVENTS does each candidate level get?

`gac_fire.prepare` plans the fire on a band of storeys and a set of sides;
`soot_plume.plan_events` then asks `openings_provider` for the openings in
each (side, storey) cell. An asset whose glazing does not reach the band gets
ZERO events — no soot, no flames, an untouched building with a fire plan
attached. This prints the island map so a level can be chosen off evidence.
"""
import random, sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
import numpy as np
from pxr import Usd, UsdGeom
from detail import gac_slice as gsl, gac_storey_slice as gss
from disaster import gac_fire as gf, quake_sliced as qs, soot_plume as spl, urban_fire as uf

NAMES = sys.argv[1:] or ["dtc:Building_12"]
for NAME in NAMES:
    NAME, _at, PIN = NAME.partition("@")
    PIN = int(PIN) if PIN else None
    kind, asset = gf.split_kind(NAME)
    pack = gf.PACKS[kind]
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0); UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W"); st.SetDefaultPrim(st.GetPrimAtPath("/W"))
    UsdGeom.Xform.Define(st, "/W/g0")
    url = gf.asset_url(asset, kind)
    src = gf.place_source(st, "/W/g0", url, gf.asset_scale(url, pack["scale"], verbose=False))
    wins, bb = gsl.window_centres(st, src)
    bb, _t = gf.trim_bbox(st, src, bb, pack["bbox_exclude"], verbose=False)
    g, meas = gss.grid_for(st, src, bb, wins, name=asset, verbose=False)
    planes = {}
    rects = gf.window_rects(st, src, planes=planes)
    mesh = gss.read_mesh(st, src, verbose=False)
    m = gf.mass_from_grid(g, bb, mesh=gf.mesh_without_props(mesh, pack["bbox_exclude"]))
    H = m["top"] - m["z0"]
    btype = qs.construction_type(url, H=H) if pack["construction_table"] else (
        "urm" if H <= 25.0 else "rc")
    prov = gf.openings_provider(rects, m, planes=planes)
    n_st = len(m["levels"])
    print("\n=== %s  %d storey(s) of %.2f m, %s, %d island(s)"
          % (asset, n_st, g["storey_h"], btype, prov.count))
    print("    storey:  " + " ".join("%4d" % s for s in range(n_st)))
    for side in ("S", "E", "N", "W"):
        row = [len(prov(None, "main", side, s)) for s in range(n_st)]
        print("    %-8s %s   total %d" % (side, " ".join("%4d" % v for v in row), sum(row)))
    for level in ("F1", "F2", "F3", "F4", "F5", "F5c", "F6"):
        rng = random.Random(7)
        origin = PIN if PIN is not None else max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
        ranked = [s for s in sorted(rects, key=lambda q: -len(rects[q])) if rects[s]] or ["S"]
        n_side = 1 if level in ("F1", "F2") else (2 if level == "F3"
                                                  else min(len(ranked), rng.randint(2, 4)))
        sides = tuple(ranked[:max(1, n_side)])
        info = {"style": "x", "family": "01", "type": btype, "x": 0.0, "y": 0.0,
                "yaw": 0.0, "masses": {"main": m}, "elements": [], "H": H}
        fire = uf.plan_fire(info, level, rng, origin=origin, sides=sides)
        ctx0 = {"info": info, "fire": fire, "rng": rng, "tag": "d0",
                "soot_openings": prov}
        ev = spl.plan_events(ctx0, uf._severity)
        print("    %-4s origin %2d band %2d-%-2d on %-8s -> %s"
              % (level, fire["origin"], fire["storeys"][0], fire["top"],
                 "/".join(fire["sides"]), spl.summarise(ev) or "0 event(s)"))
