"""_gac_starved_probe -- verify `gac_fire`'s starved-events fix AND its
2026-08-31 sides-reconciliation policy change offline, bare USD, up to (not
including) the slice -- the same measurement half `tools/_dtc_open_probe.py`
exercises (`window_rects` / `mass_from_grid` / `openings_provider` /
`urban_fire.plan_fire` / `soot_plume.plan_events`), calling `gac_fire`'s OWN
`_reconcile_sides` / `_nudge_origin_to_real` / `_real_storeys_by_side`
directly (never reimplemented here) so this probe cannot drift out of
lockstep with `prepare`'s real logic.

RECAP. The starved-events trap (fire_dtc4, 2026-08-31): a GAC building's
real window islands can be entirely missing on the elevation(s) a fire plan
asks to vent through, and `openings_provider` used to synthesize windows
straight onto whatever side it was told, blank party wall included --
flames/smoke/soot on a wall with no glass while the real windowed elevation
stayed dark (bench review, `gac_SM_Building_26_F5_o3_NW_s684`: "smoke coming
out of its sides that are blank rather than the windows... windows are on
the long side"). The fix (fire_dtc5, 2026-08-31) is two-layered:

  1. `_reconcile_sides` -- if the building has real glazing ANYWHERE, the
     venting side set is corrected to real-glazed elevations BEFORE any
     synthesis is even considered; synthesis is left to buildings with
     ZERO real glazing at all (`SM_Building_11`/`SM_Building_27`, painted
     windows this pack's texture tokens miss).
  2. `_nudge_origin_to_real` -- when the reconciled side IS real but the
     requested origin storey sits above where the real glazing reaches
     (`SM_Building_02`), the origin (never the band WIDTH) slides down onto
     it, per level's own band rules, before falling back to synthesis.

Two modes:

  --manifest scene_gen/_plans/fire_city_500m_39.json
      Census every `kind == "gac"` record in the plan -- the real
      (asset, level, origin, sides) the city solver actually produced --
      and print requested vs. RECONCILED origin/sides, the real-vs-
      synthetic opening share, and the event tally. This is the primary
      verification the 2026-08-31 policy asks for.

  Name@origin:SIDES:LEVEL[,LEVEL...] [Name@origin:SIDES:LEVEL ...]
      One-off spec mode (unchanged from the original starved-events
      probe) for a specific combination not in the current manifest.
"""
import json
import random
import sys

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom
from detail import gac_slice as gsl, gac_storey_slice as gss
from disaster import gac_fire as gf, quake_sliced as qs, soot_plume as spl, urban_fire as uf

_CACHE = {}


def measure(asset, kind="gac"):
    """(rects, m, btype, n_st) for one GAC asset, cached across records --
    the manifest repeats the same asset at several (level, origin, sides),
    and re-opening the Nucleus USD each time would be needless."""
    key = (kind, asset)
    if key in _CACHE:
        return _CACHE[key]
    pack = gf.PACKS[kind]
    st = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(st, "/W")
    st.SetDefaultPrim(st.GetPrimAtPath("/W"))
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
    n_st = len(m["levels"])
    out = (rects, m, btype, n_st, planes)
    _CACHE[key] = out
    return out


def run_record(asset, level, req_origin, req_sides, kind="gac", seed=7):
    """Replays `gac_fire.prepare`'s post-measurement pipeline exactly,
    calling its real `_reconcile_sides` / `_nudge_origin_to_real` /
    `openings_provider(band=...)` -- prints requested vs. effective
    (origin, sides), real vs. synthetic opening counts, and the events
    that result. Returns a dict row for the caller's own summary table."""
    rects, m, btype, n_st, planes = measure(asset, kind)
    origin = max(0, min(n_st - 1, int(req_origin)))
    sides = tuple(req_sides)

    venting_sides, side_note = gf._reconcile_sides(rects, sides, name=asset)
    real_by_side = gf._real_storeys_by_side(rects, m)
    eff_origin, origin_note = gf._nudge_origin_to_real(origin, venting_sides, real_by_side)

    info = {"style": "x", "family": "01", "type": btype, "x": 0.0, "y": 0.0,
            "yaw": 0.0, "masses": {"main": m}, "elements": [], "H": m["top"] - m["z0"]}
    rng = random.Random(seed)
    fire = uf.plan_fire(info, level, rng, origin=eff_origin, sides=venting_sides)
    provider = gf.openings_provider(rects, m, planes=planes,
                                    band=(fire["sides"], fire["storeys"]))
    ctx = {"info": info, "fire": fire, "rng": random.Random(seed),
          "tag": "d0", "soot_openings": provider}
    events = spl.plan_events(ctx, uf._severity)

    # Only the openings actually IN the fire's own (sides x storeys) band --
    # `provider.count` also carries real islands on sides/storeys outside
    # the band (e.g. a real S-side island on a building venting E/N), which
    # would inflate `real_n` here if used directly.
    band_ops = [op for st in fire["storeys"] for sd in fire["sides"]
               for op in provider(None, "main", sd, st)]
    synth_n = sum(1 for op in band_ops if op["e"].get("synthetic"))
    real_n = len(band_ops) - synth_n

    print("\n=== %-16s %-4s  requested: o%-3d sides=%-8s  ->  effective: "
          "o%-3d sides=%-8s%s%s"
          % (asset, level, origin, "/".join(sides), fire["origin"],
             "/".join(fire["sides"]),
             ("  [" + side_note.split(": ", 1)[-1] + "]") if side_note else "",
             ("  [" + origin_note + "]") if origin_note else ""))
    print("    openings: %d real, %d synthetic (synthetic_sides=%s)  "
          "events: %s"
          % (real_n, synth_n, "/".join(provider.synthetic_sides) or "none",
             spl.summarise(events) or "0 event(s)"))
    return {"asset": asset, "level": level, "req_origin": origin,
           "req_sides": sides, "eff_origin": fire["origin"],
           "eff_sides": fire["sides"], "real_n": real_n, "synth_n": synth_n,
           "n_events": len(events), "reconciled": bool(side_note),
           "origin_nudged": bool(origin_note)}


def _spec_mode(specs):
    for spec in specs:
        name, _at, rest = spec.partition("@")
        origin_s, _c1, rest2 = rest.partition(":")
        sides_s, _c2, levels_s = rest2.partition(":")
        kind, asset = gf.split_kind(name)
        origin = int(origin_s)
        sides = tuple(sides_s) if sides_s else ("S", "E", "N", "W")
        levels = levels_s.split(",") if levels_s else ["F5"]
        for level in levels:
            run_record(asset, level, origin, sides, kind=kind)


def _manifest_mode(path):
    with open(path) as f:
        plan = json.load(f)
    recs = [r for r in plan.get("records", []) if r.get("kind") == "gac"]
    print("=== %d GAC record(s) in %s ===" % (len(recs), path))
    rows = []
    for r in recs:
        rows.append(run_record(r["asset"], r["level"], r["origin"], r["sides"]))
    n_zero = sum(1 for row in rows if row["n_events"] == 0)
    n_reconciled = sum(1 for row in rows if row["reconciled"])
    n_nudged = sum(1 for row in rows if row["origin_nudged"])
    n_all_synth = sum(1 for row in rows if row["real_n"] == 0 and row["synth_n"] > 0)
    print("\n=== summary: %d record(s), %d STILL ZERO events, "
          "%d sides-reconciled, %d origin-nudged, %d fully-synthetic ==="
          % (len(rows), n_zero, n_reconciled, n_nudged, n_all_synth))
    for row in rows:
        if row["n_events"] == 0:
            print("    STILL ZERO: %s %s req o%d/%s -> eff o%d/%s"
                  % (row["asset"], row["level"], row["req_origin"],
                     "/".join(row["req_sides"]), row["eff_origin"],
                     "/".join(row["eff_sides"])))


if __name__ == "__main__":
    args = sys.argv[1:]
    if args and args[0] == "--manifest":
        _manifest_mode(args[1])
    elif args:
        _spec_mode(args)
    else:
        _spec_mode(["SM_Building_11@18:ENW:F5,F4,F3"])
