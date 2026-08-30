#!/usr/bin/env python
"""collapse_edge_probe — did the PARTIAL COLLAPSE tear every edge of its hole,
and did the burn zone soot everything round it?

The offline answer to the two halves of the user's round-4 review of the
ModernCityEnvironment F5c building (2026-08-30):

    "The breaks in the partial collapse of MCE look great but some parts of it
     seem like they were directly cut off from the actual prims and therefore
     look like sharp straight or rectangular cuts ... Also there are parts of
     the surface that look pristine. Any parts directly near where the
     building collapsed (up, left, down, right, anything) would have been
     flamed and scorched."

Runs `disaster.urban_fire.burn_building` at F5c on a bare USD stage — no Kit,
no Flow, no physics, exactly `tools/kit_burn_probe.py`'s path — and reports:

  * how many modules came away;
  * PER EDGE OF THE HOLE (above / below / left / right / return) how many
    surviving modules touch it and how many of them were torn. **Anything but
    100 % is a straight kit seam left on the edge of the hole**, which is the
    complaint;
  * the fragments each tear produced, and any tear that left NO static (a
    module that was effectively killed rather than torn — the far portion is
    supposed to stay standing);
  * the burn-zone rectangles, and the soot skin's alpha INSIDE the zone
    against the alpha in the 8 m of wall just outside it. Mean alpha inside
    must be >= 0.80 and the outside band must be lower, or the collapse still
    has pristine cladding round it;
  * the skin as a PNG, so the elevation can actually be looked at.

    docker exec isaac-sim bash -c "/isaac-sim/AirStack/scene_gen/tools/usd_python.sh \\
        /isaac-sim/AirStack/scene_gen/tools/collapse_edge_probe.py"

With NO arguments it runs the two buildings of the fire row: `commercial_mid`
(family 04, urm -> `mode="elevation"`) and `office_wide` (family 02, rc ->
`mode="corner"`), both venting on S,E — the same `UF_SIDES` the bench pins so
its `face` camera looks at the collapse.

    collapse_edge_probe.py STYLE [SIDES]        one building, e.g. `apartment S,E`

PNG output goes to `COLLAPSE_PNG_DIR`, else
`/isaac-sim/.nvidia-omniverse/logs/collapse_skin/` — which is
`~/docker/isaac-sim/logs/collapse_skin/` on the host. (`~/scorch_previews` is
NOT bind-mounted into the isaac-sim container; `docker inspect` its mounts
before assuming otherwise.)
"""
import os
import random
import resource
import sys
import time
import traceback

import numpy as np

sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from pxr import Usd, UsdGeom                                  # noqa: E402
import scene_generator as sg                                  # noqa: E402
from detail import urban_building as ub                       # noqa: E402
from disaster import (fire_collapse as fc, fracture,          # noqa: E402
                      quake_flow as qf, soot_plume as spl,
                      urban_fire as uf)

LEVEL = fc.FIRE_LEVEL                                          # "F5c"
DEFAULTS = [("commercial_mid", ("S", "E")), ("office_wide", ("S", "E"))]
PNG_DIR = (os.environ.get("COLLAPSE_PNG_DIR")
           or "/isaac-sim/.nvidia-omniverse/logs/collapse_skin")
# "just outside" the zone, in metres of wall — the band the review calls
# pristine if the zone did not reach it.
OUTSIDE_BAND_M = 8.0


def _sides(spec):
    if not spec:
        return None
    parts = spec.replace("/", ",").split(",")
    out = tuple(q.strip().upper()[:1] for q in parts if q.strip())
    return out or None


# ---------------------------------------------------------------------------
def _edge_table(plan):
    """The deliverable: every edge of the hole, and what share of the modules
    against it were torn."""
    jobs = plan.get("edges") or []
    cen = plan.get("edge_census") or fc.edge_census(jobs)
    print("[collapse] EDGES OF THE HOLE  (neighbours / torn / %):")
    bad = 0
    for cls in fc.EDGE_CLASSES:
        n, t = cen[cls]
        pct = (100.0 * t / n) if n else 100.0
        if n and t < n:
            bad += n - t
        print("    {0:<8} {1:3d} neighbour(s)  {2:3d} torn  {3:5.1f} %{4}"
              .format(cls, n, t, pct, "   <== NOT 100 %" if n and t < n else ""))
    n_all = len(jobs)
    n_torn = len([j for j in jobs if j.get("torn")])
    n_drop = len([j for j in jobs if j.get("dropped")])
    n_fail = len([j for j in jobs if j.get("failed")])
    n_nostat = len([j for j in jobs if j.get("no_static")])
    st = sum(int(j.get("n_static") or 0) for j in jobs)
    lo = sum(int(j.get("n_loose") or 0) for j in jobs)
    print("    modules at the edge: {0}, torn {1}, over budget {2}, "
          "fracture returned nothing {3}".format(n_all, n_torn, n_drop, n_fail))
    print("    edge fragments: {0} static (still standing) + {1} loose"
          .format(st, lo))
    print("FLAG untorn-edge: {0} surviving module(s) against the hole that "
          "kept a factory kit seam — expected 0.".format(bad + n_drop + n_fail))
    print("FLAG torn-away: {0} torn module(s) that left NO static behind (the "
          "far portion is supposed to stay standing) — expected 0."
          .format(n_nostat))
    return bad + n_drop + n_fail, n_nostat


def _zone_stats(ctx, plan, tag):
    """Alpha inside the burn zone against the 8 m of wall just outside it."""
    sk = ctx.get("soot_skin")
    if sk is None:
        print("[collapse] NO SOOT SKIN — `plan_events` produced nothing, so "
              "the building has no soot at all (the F5c/`DURATION_S` trap)")
        return None
    rects = plan.get("burn_zone") or []
    print("[collapse] BURN ZONE: {0} rect(s) in soot_plume.side_u metres"
          .format(len(rects)))
    for r in rects:
        print("    {0}  u {1:6.1f} .. {2:6.1f} m   z {3:6.1f} .. {4:6.1f} m"
              .format(r[0], r[1], r[2], r[3], r[4]))
    if sk.get("zone") is None:
        print("FLAG zone-missing: the skin carries NO zone — `burn_zone` did "
              "not reach `soot_plume.skin`. Expected a zone.")
        return None
    z = sk["zone"]
    alpha = sk["rgba"][..., 3]
    d = spl.zone_field(rects, alpha.shape[0], alpha.shape[1], sk["ppm"],
                       sk["per"], sk["H"], sk["z0"], sk["offsets"])
    inside = z > 0.999
    near = (d < 0.0) & (d > -OUTSIDE_BAND_M)
    far = d <= -OUTSIDE_BAND_M
    # the same skin with NO zone, from the same generator: what the wall
    # round the hole looked like before this change
    base = spl.skin(ctx, sk["events"], np.random.default_rng(
        spl.event_seed(ctx) ^ 0x5EED), finish=ctx["fire"].get("finish") or "char",
        glass=(ctx["info"]["type"] == "rc_glass"), duration_scale=1.4)
    b = base["rgba"][..., 3]
    a_in = float(alpha[inside].mean()) if inside.any() else float("nan")
    a_near = float(alpha[near].mean()) if near.any() else float("nan")
    a_far = float(alpha[far].mean()) if far.any() else float("nan")
    print("    canvas {0} x {1} px at {2:.1f} px/m; zone covers {3:.1f} % of it"
          .format(alpha.shape[1], alpha.shape[0], sk["ppm"],
                  100.0 * float(inside.mean())))
    print("    mean ALPHA  inside {0:.3f}   outside within {1:.0f} m {2:.3f}"
          "   beyond that {3:.3f}".format(a_in, OUTSIDE_BAND_M, a_near, a_far))
    print("    ...same skin with NO burn zone: inside {0:.3f}, outside within "
          "{1:.0f} m {2:.3f}".format(
              float(b[inside].mean()) if inside.any() else float("nan"),
              OUTSIDE_BAND_M,
              float(b[near].mean()) if near.any() else float("nan")))
    was_clean = inside & (b < 0.4)
    if was_clean.any():
        print("    {0} px inside the zone were effectively PRISTINE "
              "(alpha < 0.4, mean {1:.3f}); they are now mean {2:.3f}, "
              "min {3:.3f}".format(int(was_clean.sum()),
                                   float(b[was_clean].mean()),
                                   float(alpha[was_clean].mean()),
                                   float(alpha[was_clean].min())))
    # WHERE THE COVERAGE FIELD IS EXACTLY ZERO nothing may have moved. (Not
    # `d <= 0`: the ramp deliberately reaches 1.5-2.5 m OUTSIDE the rectangle,
    # wandered by the streak noise, which is what stops the zone having an
    # edge. `zone == 0` is the true no-op region.)
    off = z <= 0.0
    print("    where the zone is exactly 0 the skin is untouched: {0} "
          "({1:.1f} % of the canvas)".format(
              bool(np.array_equal(b[off], alpha[off])),
              100.0 * float(off.mean())))
    print("FLAG zone-alpha: mean alpha inside the zone is {0:.3f}; must be "
          ">= 0.80.".format(a_in))
    print("FLAG zone-contrast: outside-within-{0:.0f}m alpha {1:.3f} must be "
          "LOWER than inside {2:.3f}: {3}".format(
              OUTSIDE_BAND_M, a_near, a_in, "ok" if a_near < a_in else "NO"))
    try:
        os.makedirs(PNG_DIR, exist_ok=True)
        p1 = os.path.join(PNG_DIR, "skin_{0}_{1}.png".format(tag, LEVEL))
        spl.save_skin_png(sk, p1)
        p0 = os.path.join(PNG_DIR, "skin_{0}_{1}_nozone.png".format(tag, LEVEL))
        spl.save_skin_png(base, p0)
        print("    PNG (S|E|N|W unwrapped, row 0 = top): {0}".format(p1))
        print("    PNG without the zone, same seed:      {0}".format(p0))
    except Exception as exc:                                  # pragma: no cover
        print("    PNG write FAILED: {0}".format(exc))
    return a_in


# ---------------------------------------------------------------------------
def run_one(style, sides, seed):
    t0 = time.time()
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/W")
    stage.SetDefaultPrim(stage.GetPrimAtPath("/W"))
    UsdGeom.Scope.Define(stage, "/W/bench")
    cell = "/W/bench/k0"
    UsdGeom.Xform.Define(stage, cell)
    mats = uf.materials(stage, "/W/bench")
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed)

    pls = ub.build_building(style, 0.0, 0.0, 0.0, random.Random(seed + 7))
    sg.apply_placements(stage, pls, cell + "/parts", 1.0)
    ub.apply_glass_tint(stage, pls)
    specs = qf._mass_specs(style, 0.0, 0.0, 0.0)
    main_spec = max(specs, key=lambda q: len(q["levels"]))
    n_st = max(1, len(main_spec["levels"]))
    origin = max(0, min(n_st - 1, int(round(0.25 * (n_st - 1)))))
    ctx = uf.burn_building(stage, cell, style, pls, 0.0, 0.0, 0.0, LEVEL,
                           rng, nrng, mats, "k0", flow_root=None,
                           origin=origin, sides=sides, mat_cache={})

    for n in ctx["notes"]:
        if "partial collapse" in n:
            print("   note:", n[:400])
    plan = ctx.get("partial_collapse")
    if plan is None:
        print("[collapse] {0}: NO PARTIAL COLLAPSE RAN — `{1}` is not in this "
              "construction type's ladder (rc_glass has no collapse by "
              "design)".format(style, LEVEL))
        return ctx
    m = ctx["info"]["masses"][plan["mass"]]
    print("[collapse] {0} {1} type={2} mode={3} sides={4} origin={5}: mass {6} "
          "{7:.0f} x {8:.0f} m, {9} storey(s)".format(
              style, LEVEL, ctx["info"]["type"], plan["mode"],
              ",".join(plan["sides"]), ctx["fire"]["origin"], plan["mass"],
              m["W"], m["D"], plan["n_levels"]))
    print("    DEAD: {0} module(s) from storey {1} up ({2} storey(s)); "
          "{3} survive on the lost elevation(s)".format(
              len(plan["kill"]), plan["s0"], len(plan["storeys"]),
              len([e for e in ctx["info"]["elements"]
                   if e["mass"] == plan["mass"] and e["side"] in plan["sides"]
                   and e["role"] in fc.SHELL_ROLES and not e.get("dead")])))
    n_bad, n_nostat = _edge_table(plan)
    a_in = _zone_stats(ctx, plan, "{0}".format(style))
    print("[collapse] {0}: loose {1}, static_extra {2}, authored {3}; "
          "{4:.0f}s".format(style, len(ctx["loose"]), len(ctx["static_extra"]),
                            len(ctx["authored"]), time.time() - t0))
    return ctx


def main():
    args = sys.argv[1:]
    seed = 7
    if "--seed" in args:
        i = args.index("--seed")
        seed = int(args[i + 1])
        del args[i:i + 2]
    if not args:
        jobs = [(s, sd, seed + 31 * i) for i, (s, sd) in enumerate(DEFAULTS)]
    else:
        jobs = [(args[0], _sides(args[1]) if len(args) > 1 else ("S", "E"),
                 seed)]

    fracture.ensure_deps(verbose=False)
    fracture.ensure_vtk(verbose=False)
    t0 = time.time()
    for style, sides, sd in jobs:
        print("=" * 78)
        print("[collapse] {0} {1} sides={2} seed={3}".format(
            style, LEVEL, ",".join(sides or ()), sd))
        run_one(style, sides, sd)
    print("=" * 78)
    rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    print("PEAK_RSS_MB {0:.0f} WALL_S {1:.0f}".format(rss, time.time() - t0))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
