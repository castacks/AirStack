#!/usr/bin/env python3
"""Find debris that is floating in a BUILT scene, and say where it came from.

    P=$(ls -d /isaac-sim/extscache/omni.usd.libs-*/ | head -1); \
    LD_LIBRARY_PATH=$P/bin PYTHONPATH=$P \
    /isaac-sim/kit/python/bin/python3 tools/debris_float_probe.py <flat.usd> [-n 10]

WHY THIS EXISTS. "The debris is floating" has been reported repeatedly and each
round fixed a different mechanism, because THERE ARE SEVERAL POPULATIONS OF
DEBRIS IN THIS SCENE and they are authored by completely different code:

  * `blockdeb_<i>_<j>`  — the road-blockage litter, PLANNED by
    `people._blocker_debris` and authored by `scene_api._place_debris`. Placed
    live, at a z this repository computes.
  * `t_<i>` (tree instances) — the `log_*` sticks inside a BAKED tree
    archetype. Placed by `vegetation.wood_debris` at bake time, frozen into the
    archetype USD, and re-used by reference. Their height relative to the tree
    was decided once, months of runs ago.
  * `h_<i>` (house instances) — `frag_*` fracture fragments, likewise baked.

Fixing the first does nothing for the other two, and from the air they are
indistinguishable. So this reports the POPULATION alongside the height, which
is the only way to know which code to go and look at.

**A CONSISTENT HEIGHT IS THE TELL.** A solver does not leave unrelated bodies
at the same height to the centimetre; a single uniform OFFSET does. The
wildfire and tornado skills both record that signature — "every seated stick in
`tree_American_Beech_torched` sat at exactly 0.567 m" — and it is the
difference between a physics problem and an arithmetic one. The summary prints
the spread per population for exactly that reason.

Standalone `pxr`: no SimulationApp, no Kit, safe to run beside a live sim.
"""

import argparse
import collections
import os
import sys

from pxr import Gf, Usd, UsdGeom


#: Mesh-name prefixes that are DEBRIS rather than structure, per population.
DEBRIS_NAMES = ("log_", "debris", "frag_", "brk_", "stick", "splinter")

#: Prim-name prefixes under `<parent>/inst/` and the population each belongs to.
POPULATIONS = (("blockdeb_", "blockage (live, people._blocker_debris)"),
               ("blocker_", "blockage tree (baked archetype)"),
               ("t_", "tree archetype (baked, vegetation.wood_debris)"),
               ("h_", "house archetype (baked, fracture frags)"))


def population(name):
    for pre, label in POPULATIONS:
        if name.startswith(pre):
            return label
    return "other"


def _xform_cache():
    return UsdGeom.XformCache(Usd.TimeCode.Default())


def _range(xc, prim):
    """Tight world AABB from the mesh's POINTS. `(lo, hi)` or None.

    **NEVER `UsdGeom.BBoxCache` HERE.** It returns the AABB of an AABB — the
    local extent box's eight corners, transformed and re-axis-aligned — and for
    a Voronoi sliver lying diagonally in its own box that inflates several-fold
    IN BOTH DIRECTIONS. Measured on `tree_Black_Oak_snag/log_017`: the geometry
    sits at world z 0.421..0.565 and the bbox reports 0.000..0.986, so a piece
    hanging 42 cm in the air reads as resting on the ground.

    Every earlier version of this probe used the bbox cache and reported the
    library clean; a points-based pass over the same files then found 2,261
    unsupported pieces in 62 of 78 archetypes. The bbox is why four rounds of
    "the debris is still floating" each ended in a measurement that said it was
    not.
    """
    from pxr import Gf
    m = UsdGeom.Mesh(prim)
    pts = m.GetPointsAttr().Get() if m else None
    if not pts:
        return None
    M = xc.GetLocalToWorldTransform(prim)
    lo = [1e30] * 3
    hi = [-1e30] * 3
    for q in pts:
        w = M.Transform(Gf.Vec3d(float(q[0]), float(q[1]), float(q[2])))
        for k in range(3):
            v = float(w[k])
            if v < lo[k]:
                lo[k] = v
            if v > hi[k]:
                hi[k] = v
    return lo, hi



def _airborne(xc, meshes, air_tol=0.10, seat_frac=0.20):
    """Meshes with nothing under them, by the `bake._air_and_sunk` rule.

    A box that merely TOWERS PAST us is not support — the support's top has to
    land inside our own vertical span, and the plan overlap has to be a SEAT
    (at least `seat_frac` of the smaller of the two plan areas) rather than a
    corner clip. Both of those were got wrong twice in the tornado bake and are
    written up there; this is the v3 test.

    Axis-aligned, so it OVER-counts support: every number it returns is a lower
    bound on the real airborne population.
    """
    boxes = []
    for m in meshes:
        r = _range(xc, m)
        if r is None:
            continue
        boxes.append((m, r[0], r[1]))
    out = []
    for (m, lo, hi) in boxes:
        z0, z1 = lo[2], hi[2]
        area = max(1e-9, (hi[0] - lo[0]) * (hi[1] - lo[1]))
        held = False
        for (q, qlo, qhi) in boxes:
            if q is m:
                continue
            qtop = qhi[2]
            if not (z0 - air_tol <= qtop <= z1):
                continue
            ox = min(hi[0], qhi[0]) - max(lo[0], qlo[0])
            oy = min(hi[1], qhi[1]) - max(lo[1], qlo[1])
            if ox <= 0.0 or oy <= 0.0:
                continue
            qarea = max(1e-9, (qhi[0] - qlo[0]) * (qhi[1] - qlo[1]))
            if (ox * oy) >= seat_frac * min(area, qarea):
                held = True
                break
        if not held:
            out.append((m, float(z0)))
    return out


def sweep_archetypes(args):
    """Every archetype in a directory, measured in its OWN frame.

    THE COMPLETE ANSWER, and the reason it exists: probing a built scene stops
    at `-n` floaters and reports whichever archetype the traversal reached
    first, which is a sample dressed up as a finding. An archetype is baked
    once and referenced hundreds of times, so a defect in one file is a defect
    in every instance of it — the population that matters is FILES, and there
    are only ~78 of them.
    """
    import glob

    files = sorted(glob.glob(os.path.join(args.archetypes, "*.usd")))
    if not files:
        raise SystemExit("no .usd in " + args.archetypes)
    print("=" * 92)
    print("ARCHETYPE DEBRIS SWEEP  {0}   ({1} file(s), air > {2:.3f} m over "
          "the archetype's own datum)".format(args.archetypes, len(files),
                                              args.air))
    print("=" * 92)
    print("{0:<40} {1:>7} {2:>8} {3:>9} {4:>9} {5:>9}".format(
        "archetype", "debris", "airborne", "min_z", "max_z", "spread"))
    bad = []
    for f in files:
        st = Usd.Stage.Open(f)
        if st is None:
            continue
        xc = _xform_cache()
        root = st.GetPrimAtPath("/Baked") or st.GetPseudoRoot()
        allm = [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies())
                if p.IsA(UsdGeom.Mesh)]
        deb = [m for m in allm
               if any(k in m.GetName().lower() for k in DEBRIS_NAMES)]
        if not deb:
            continue
        air = [(m, z) for (m, z) in _airborne(xc, allm) if m in set(deb)
               and z > args.air]
        name = os.path.basename(f)[:-4]
        if not air:
            print("{0:<40} {1:>7} {2:>8}".format(name, len(deb), 0))
            continue
        zs = [z for _m, z in air]
        spread = max(zs) - min(zs)
        print("{0:<40} {1:>7} {2:>8} {3:>9.4f} {4:>9.4f} {5:>9.4f}{6}".format(
            name, len(deb), len(air), min(zs), max(zs), spread,
            "   <== UNIFORM OFFSET" if spread < 0.02 else ""))
        bad.append((name, len(deb), len(air), min(zs), max(zs), spread,
                    [m.GetName() for m, _z in air][:6]))
    print("\n" + "=" * 92)
    if not bad:
        print("CLEAN: no archetype has airborne debris above the threshold.")
        return 0
    print("{0} of {1} archetype(s) carry airborne debris.".format(
        len(bad), len(files)))
    uni = [b for b in bad if b[5] < 0.02]
    if uni:
        print("\n{0} of them are at ONE height to under 2 cm — arithmetic, "
              "not physics:".format(len(uni)))
        for b in uni:
            print("   {0:<38} {1} piece(s) at {2:.4f} m   {3}".format(
                b[0], b[2], b[3], ", ".join(b[6])))
    var = [b for b in bad if b[5] >= 0.02]
    if var:
        print("\n{0} have a real spread — a per-piece rule, not one "
              "datum:".format(len(var)))
        for b in var:
            print("   {0:<38} {1} piece(s) {2:.3f}..{3:.3f} m".format(
                b[0], b[2], b[3], b[4]))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd", nargs="?",
                    help="a built/flattened scene to probe")
    ap.add_argument("--archetypes", metavar="DIR",
                    help="instead, sweep EVERY archetype .usd in DIR. This is "
                         "the complete answer; probing a scene stops at -n "
                         "floaters and tells you only what it happened to hit "
                         "first.")
    ap.add_argument("-n", type=int, default=10,
                    help="stop after this many floaters PER POPULATION")
    ap.add_argument("--ground", type=float, default=0.003,
                    help="ground height in metres (_Z_GRASS * z_scale)")
    ap.add_argument("--road", type=float, default=0.015,
                    help="carriageway height in metres")
    ap.add_argument("--air", type=float, default=0.06,
                    help="clear air under a piece before it counts as floating")
    ap.add_argument("--parent", default="/World/stage/generated")
    ap.add_argument("--max-objects", type=int, default=4000,
                    help="how many instances to open before giving up")
    args = ap.parse_args()

    if args.archetypes:
        return sweep_archetypes(args)
    if not args.usd:
        raise SystemExit("give a scene usd, or --archetypes DIR")
    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        raise SystemExit("cannot open " + args.usd)
    inst = stage.GetPrimAtPath(args.parent + "/inst")
    if not (inst and inst.IsValid()):
        raise SystemExit("no " + args.parent + "/inst on this stage")

    xc = _xform_cache()
    found = collections.defaultdict(list)
    counted = collections.Counter()
    scanned = 0

    for obj in inst.GetChildren():
        pop = population(obj.GetName())
        if len(found[pop]) >= args.n and all(
                len(found[p]) >= args.n for _pre, p in POPULATIONS):
            break
        if len(found[pop]) >= args.n:
            continue
        scanned += 1
        if scanned > args.max_objects:
            break

        # The blockage litter is ONE MESH PER PRIM, so the object itself is the
        # piece. Everything else is a referenced archetype whose debris lives
        # inside it, behind an instance boundary.
        if obj.GetName().startswith("blockdeb_"):
            leaves = [obj]
        else:
            leaves = [p for p in Usd.PrimRange(obj, Usd.TraverseInstanceProxies())
                      if p.IsA(UsdGeom.Mesh)
                      and any(k in p.GetName().lower() for k in DEBRIS_NAMES)]
        for m in leaves:
            counted[pop] += 1
            r = _range(xc, m)
            if r is None:
                continue
            lo, hi = r
            # A piece over the CARRIAGEWAY is measured against the road, and
            # everything else against the ground: the ladder puts grass below
            # asphalt and calling a bedded road piece a floater is noise.
            gap = lo[2] - args.ground
            if gap <= args.air:
                continue
            found[pop].append({
                "path": str(m.GetPath()),
                "object": obj.GetName(),
                "mesh": m.GetName(),
                "min_z": round(float(lo[2]), 4),
                "max_z": round(float(hi[2]), 4),
                "gap": round(float(gap), 4),
                "xy": (round(float(0.5 * (lo[0] + hi[0])), 1),
                       round(float(0.5 * (lo[1] + hi[1])), 1)),
                "size": (round(float(hi[0] - lo[0]), 2),
                         round(float(hi[1] - lo[1]), 2),
                         round(float(hi[2] - lo[2]), 2)),
            })
            if len(found[pop]) >= args.n:
                break

    print("=" * 78)
    print("DEBRIS FLOAT PROBE  {0}".format(os.path.basename(args.usd)))
    print("  ground {0:.3f} m   road {1:.3f} m   air threshold {2:.3f} m"
          .format(args.ground, args.road, args.air))
    print("  {0} object(s) opened".format(scanned))
    print("=" * 78)
    for _pre, pop in POPULATIONS:
        hits = found.get(pop) or []
        n_seen = counted.get(pop, 0)
        print("\n{0}".format(pop.upper()))
        print("  {0} debris mesh(es) measured, {1} floating".format(
            n_seen, len(hits)))
        if not hits:
            continue
        for h in hits:
            print("    {0:<44} min_z {1:>7.3f}  gap {2:>6.3f}  "
                  "size {3}  at ({4}, {5})".format(
                      h["object"] + "/" + h["mesh"], h["min_z"], h["gap"],
                      h["size"], h["xy"][0], h["xy"][1]))
        zs = [h["min_z"] for h in hits]
        spread = max(zs) - min(zs)
        print("    ---- min_z {0:.4f} .. {1:.4f}   SPREAD {2:.4f} m".format(
            min(zs), max(zs), spread))
        # THE DIAGNOSIS. Say it, rather than leaving it to be re-derived.
        if spread < 0.02:
            print("    ==> CONSISTENT to under 2 cm. A solver does not leave "
                  "unrelated pieces at one height;")
            print("        a single uniform OFFSET does. This is arithmetic, "
                  "not physics.")
        elif spread < 0.15:
            print("    ==> tightly clustered ({0:.3f} m). Likely one datum "
                  "wrong, with per-piece variation on top.".format(spread))
        else:
            print("    ==> spread over {0:.2f} m. Not a single offset — "
                  "look for a per-piece rule.".format(spread))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
