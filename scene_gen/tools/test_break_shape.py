"""Acceptance test for round-3 break SHAPE (host-side, no pxr, no Isaac).

Two halves, both runnable from the repo root:

  python3 scene_gen/tools/test_break_shape.py
      Slices the pure-python / numpy-only helpers out of `quake_flow.py` and
      `fracture.py` by AST and exercises them:
        * `_p_staircase`   every riser is an INTEGER multiple of the course
                           pitch and every run an integer multiple of the half
                           stretcher — the thing `_a_stepped` gets wrong
        * `_p_brick_seeds` the lattice is course-aligned, running-bond, and
                           its jitter never moves a seed off its own unit
        * `_p_prism_seeds` every seed sits on ONE plane through the member, so
                           every cut face comes out normal to the surface
        * `_p_is_sliver`   blades and wafers rejected, rafts kept

  python3 scene_gen/tools/test_break_shape.py <frags.jsonl> [more.jsonl ...]
      Scores a bench run's fragment set against agent R's acceptance table
      (_plans/eq_round3_R.md §5). Produce the file by adding EQ_DUMP_FRAGS=1
      to the eq_bench.sh line; it lands in the capture directory as
      `frags.jsonl` (see `fracture._p_dump`).

SHAPE DEFINITIONS, stated because there is more than one convention.
Sorted axes a >= b >= c of the fragment's axis-aligned extents.
  flaky           c / b < 0.6      (BS 812 Flakiness Index, thickness against
                                    the intermediate dimension)
  elongated       a / b > 1.8      (BS 812 Elongation Index)
  equidimensional b / a > 2/3 AND c / b > 2/3      (R's own definition)
  blade           elongated AND flaky              (R: this must be ~0 %)
`perp` / `face` come from the dump: the area-weighted share of a fragment's
surface within 20 deg of perpendicular to, and of parallel to, its own
thinnest axis. For a brick cell or a prism those two account for nearly all of
it; whatever is left is the OBLIQUE face that reads as a shard.
"""
import ast
import bisect
import json
import math
import os
import random
import sys
import textwrap

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QF = os.path.join(ROOT, "disaster", "quake_flow.py")
FR = os.path.join(ROOT, "disaster", "fracture.py")

QF_WANT = {"_p_staircase", "_p_wobble", "_a_torn"}
FR_WANT = {"_p_axes", "_p_brick_seeds", "_p_prism_seeds", "_p_is_sliver"}


class _Box(object):
    """The only thing `_p_*_seeds` asks of a mesh is `.bounds`."""

    def __init__(self, lo, hi):
        self.bounds = (np.asarray(lo, dtype=float), np.asarray(hi, dtype=float))


def _slice(path, want, ns):
    src = open(path).read()
    lines = src.splitlines()
    got = set()
    for node in ast.parse(src).body:
        if isinstance(node, ast.FunctionDef) and node.name in want:
            exec(compile(textwrap.dedent(
                "\n".join(lines[node.lineno - 1:node.end_lineno])),
                "<sliced>", "exec"), ns)
            got.add(node.name)
        elif isinstance(node, ast.Assign):
            # module constants the sliced functions close over
            for t in node.targets:
                if isinstance(t, ast.Name) and t.id.isupper():
                    try:
                        ns[t.id] = ast.literal_eval(node.value)
                    except Exception:
                        pass
    missing = want - got
    assert not missing, "not found in %s: %s" % (path, missing)
    return ns


def load():
    ns = {"math": math, "bisect": bisect, "np": np, "random": random}
    _slice(QF, QF_WANT, ns)
    _slice(FR, FR_WANT, ns)
    return ns


# ---------------------------------------------------------------------------
# HALF 0 — the keyword arguments actually exist
# ---------------------------------------------------------------------------
def check_calls(paths=(QF, FR)):
    """Every keyword passed at an intra-module call site is one the callee
    accepts.

    This exists because a `_refine_cell(..., flat_axis=...)` call went out to
    the GPU with the matching signature change missing, and the only way that
    surfaced was a Traceback 17 s into a bench run — which `eq_bench.sh` then
    reported as DONE, because Kit still exits 0 after a python error and the
    runner matches `^EXIT 0`. Two runs and twenty minutes. It is a five-line
    AST walk to catch offline."""
    bad = []
    for path in paths:
        tree = ast.parse(open(path).read())
        sigs = {}
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                a = node.args
                names = set(q.arg for q in list(a.posonlyargs) + list(a.args)
                            + list(a.kwonlyargs))
                sigs[node.name] = (names, a.kwarg is not None)
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            f = node.func
            name = (f.id if isinstance(f, ast.Name)
                    else f.attr if isinstance(f, ast.Attribute) else None)
            if name not in sigs:
                continue
            names, has_kw = sigs[name]
            if has_kw:
                continue
            for kw in node.keywords:
                if kw.arg is None:          # **kwargs unpacking: unknowable
                    continue
                if kw.arg not in names:
                    bad.append("%s:%d  %s() has no keyword %r"
                               % (os.path.basename(path), node.lineno, name,
                                  kw.arg))
    return bad


# ---------------------------------------------------------------------------
# HALF 1 — the math, offline
# ---------------------------------------------------------------------------
def check_math():
    ns = load()
    bad = []
    P_BOND = ns.get("P_BOND") or {"modular": (0.203, 0.0677, 0.092),
                                  "heritage": (0.240, 0.086, 0.110)}

    # --- the staircase is QUANTISED --------------------------------------
    for name, (p_l, p_c, _p_t) in P_BOND.items():
        for seed in range(10):
            rng = random.Random(seed)
            f = ns["_p_staircase"](rng, 0.60, 22.0, p_l * 0.5, p_c)
            v = [f(i * 0.01) for i in range(2201)]
            if max(abs(q) for q in v) > 0.60 + p_c + 1e-9:
                bad.append("%s/%d: staircase exceeds amp+one course" % (name, seed))
            # every LEVEL is a whole number of courses off the datum
            for q in set(round(x, 9) for x in v):
                k = q / p_c
                if abs(k - round(k)) > 1e-6:
                    bad.append("%s/%d: level %.4f m is %.3f courses" %
                               (name, seed, q, k))
                    break
            risers = [abs(b - a) for a, b in zip(v, v[1:]) if abs(b - a) > 1e-12]
            if not risers:
                bad.append("%s/%d: no risers over 22 m" % (name, seed))
                continue
            if min(risers) < p_c - 1e-9:
                bad.append("%s/%d: riser %.4f m under one course (%.4f)" %
                           (name, seed, min(risers), p_c))
            if max(risers) > 4 * p_c + 1e-6:
                bad.append("%s/%d: riser %.4f m over four courses" %
                           (name, seed, max(risers)))
            if not (10 <= len(risers) <= 90):
                bad.append("%s/%d: %d risers over 22 m" % (name, seed, len(risers)))
            # ...and every STEP happens at a whole number of half
            # stretchers. Sampled at 2 mm, so a detected step time is up to
            # one sample EARLY; the tolerance is that sample, nothing more.
            w = [f(i * 0.002) for i in range(11001)]
            ts = [(i + 1) * 0.002 for i, (x, y) in enumerate(zip(w, w[1:]))
                  if abs(y - x) > 1e-12]
            tol = 0.002 / (p_l * 0.5) + 1e-6
            for t in ts[:40]:
                k = t / (p_l * 0.5)
                if abs(k - round(k)) > tol:
                    bad.append("%s/%d: step at %.4f m = %.3f half-stretchers" %
                               (name, seed, t, k))
                    break

    # `_p_wobble` must return the staircase for urm and the tear for rc
    rng = random.Random(3)
    fu = ns["_p_wobble"](rng, 0.5, 20.0, "urm", pitch=P_BOND["modular"])
    lv = set(round(fu(i * 0.05), 9) for i in range(400))
    if any(abs(q / P_BOND["modular"][1] - round(q / P_BOND["modular"][1])) > 1e-6
           for q in lv):
        bad.append("_p_wobble(urm) is not on the courses")
    fr = ns["_p_wobble"](random.Random(3), 0.5, 20.0, "rc")
    vr = [fr(i * 0.05) for i in range(400)]
    if len(set(round(q, 6) for q in vr)) < 50:
        bad.append("_p_wobble(rc) is quantised — it should tear, not step")

    # --- the brick lattice ------------------------------------------------
    for name, pitch in P_BOND.items():
        p_l, p_c, p_t = pitch
        wall = _Box((0, 0, 0), (4.0, 0.38, 3.0))
        for seed in range(6):
            rng = np.random.default_rng(seed)
            P = ns["_p_brick_seeds"](wall, 40, rng, pitch=pitch, keep=1.0)
            if len(P) < 8:
                bad.append("%s/%d: brick lattice made only %d seeds" %
                           (name, seed, len(P)))
                continue
            # thin axis = Y here; every seed must sit on the wall's mid-plane
            if float(np.ptp(P[:, 1])) > 1e-6:
                bad.append("%s/%d: brick seeds are not coplanar through the "
                           "wall (spread %.4f m)" % (name, seed,
                                                     float(np.ptp(P[:, 1]))))
            # rows are course-aligned: the distinct row heights must be a
            # constant pitch that is a whole number of courses. (Rows are
            # clustered, not rounded — the per-row course jitter is up to a
            # tenth of a course.)
            zs = sorted(set(round(q, 4) for q in P[:, 2]))
            dz = [b - a for a, b in zip(zs, zs[1:]) if b - a > 1e-3]
            if dz:
                k = np.median(dz) / p_c
                # tolerance = the per-row course jitter (+-0.1 course on each
                # of the two rows), not a fudge factor
                if abs(k - round(k)) > 0.25:
                    bad.append("%s/%d: row pitch %.4f m = %.2f courses" %
                               (name, seed, float(np.median(dz)), k))
            # running bond: adjacent rows are offset by ~half a stretcher
            rows = {}
            for q in P:
                rows.setdefault(round(float(q[2]), 3), []).append(float(q[0]))
            ks = sorted(rows)
            offs = []
            for a, b in zip(ks, ks[1:]):
                if len(rows[a]) < 2 or len(rows[b]) < 2:
                    continue
                offs.append(abs((min(rows[a]) - min(rows[b]))) % (p_l))
            if len(ks) >= 3 and offs and max(offs) < p_l * 0.25:
                bad.append("%s/%d: no running-bond offset between rows" %
                           (name, seed))
            # jitter never moves a seed a whole unit
            if float(np.ptp(P[:, 0])) > 4.0 + p_l:
                bad.append("%s/%d: brick seeds escape the module" % (name, seed))

    # a request the lattice cannot honour must be CAPPED, not exploded
    rng = np.random.default_rng(1)
    P = ns["_p_brick_seeds"](_Box((0, 0, 0), (4.0, 0.38, 3.0)), 20, rng,
                             pitch=P_BOND["modular"], keep=1.0)
    # `BRICK_NMAX` is the ceiling the blocky clamp is allowed to spend, and it
    # is the only thing between a course-pitch lattice and 3500 rigid bodies.
    nmax = float(ns.get("BRICK_NMAX", 2.4))
    if len(P) > 20 * nmax + 2:
        bad.append("brick lattice ignored the cap: %d seeds for a budget of "
                   "20 (ceiling %.0f)" % (len(P), 20 * nmax))

    # --- the prism lattice ------------------------------------------------
    for seed in range(6):
        rng = np.random.default_rng(seed)
        slab = _Box((0, 0, 0), (18.0, 14.0, 0.22))
        P = ns["_p_prism_seeds"](slab, 60, rng)
        if len(P) < 20:
            bad.append("prism %d: only %d seeds" % (seed, len(P)))
            continue
        if float(np.ptp(P[:, 2])) > 1e-9:
            bad.append("prism %d: seeds are NOT one layer (spread %.4f m) — "
                       "that is the shard bug" % (seed, float(np.ptp(P[:, 2]))))
        # nearest-neighbour spacing must be comfortably over the thickness,
        # or the cells are cubes of a plate rather than prisms through it
        d = np.linalg.norm(P[:, None, :2] - P[None, :, :2], axis=2)
        np.fill_diagonal(d, 1e9)
        if float(d.min(axis=1).mean()) < 0.22:
            bad.append("prism %d: mean spacing %.3f m under the 0.22 m "
                       "thickness" % (seed, float(d.min(axis=1).mean())))

    # --- sliver rejection --------------------------------------------------
    sv = ns["_p_is_sliver"]
    cases = [((0.40, 0.38, 0.34), False, "a brick cluster"),
             ((1.20, 0.20, 0.18), True, "a needle"),
             ((0.60, 0.55, 0.06), True, "a wafer"),
             ((5.00, 4.00, 0.20), False, "a slab raft (exempt over 1.2 m)"),
             ((0.90, 0.30, 0.05), True, "a blade")]
    for ext, want, why in cases:
        if bool(sv(ext)) != want:
            bad.append("_p_is_sliver%s -> %s, wanted %s (%s)"
                       % (ext, not want, want, why))
    return bad


# ---------------------------------------------------------------------------
# HALF 2 — score a bench run's fragment set
# ---------------------------------------------------------------------------
def _abc(r):
    return float(r["a"]), float(r["b"]), float(r["c"])


def score(rows, label=""):
    """Print the acceptance table for one fragment set. Returns n_fail."""
    if not rows:
        print("  (no fragments)")
        return 1
    A = np.array([_abc(r) for r in rows], dtype=float)
    a, b, c = A[:, 0], A[:, 1], A[:, 2]
    b = np.maximum(b, 1e-9)
    flaky = (c / b) < 0.6
    elong = (a / np.maximum(b, 1e-9)) > 1.8
    equi = ((b / np.maximum(a, 1e-9)) > 2.0 / 3.0) & ((c / b) > 2.0 / 3.0)
    blade = flaky & elong
    perp = np.array([float(r.get("perp", 0.0)) for r in rows])
    face = np.array([float(r.get("face", 0.0)) for r in rows])
    obl = np.clip(1.0 - perp - face, 0.0, 1.0)
    urm = np.array([("urm" in str(r.get("tag", ""))) for r in rows])
    n = len(rows)
    fi, ei = 100.0 * flaky.mean(), 100.0 * elong.mean()
    eq, bl = 100.0 * equi.mean(), 100.0 * blade.mean()

    checks = []
    checks.append(("mean Flakiness Index", "%.1f %%" % fi, fi < 15.0, "< 15 %"))
    checks.append(("mean Elongation Index", "%.1f %%" % ei, ei < 25.0, "< 25 %"))
    checks.append(("equidimensional", "%.1f %%" % eq, 55.0 <= eq <= 75.0,
                   "55-75 %"))
    checks.append(("blades", "%.1f %%" % bl, bl <= 2.0, "~0 %"))
    checks.append(("oblique face area (median)", "%.1f %%" % (100 * np.median(obl)),
                   float(np.median(obl)) < 0.20,
                   "< 20 % (faces normal to the surface)"))
    if urm.any():
        u = A[urm]
        small = ((u[:, 0] >= 0.06) & (u[:, 0] <= 0.25)).mean() * 100.0
        checks.append(("URM 1-2 units (0.06-0.25 m)", "%.1f %%" % small,
                       small >= 30.0,
                       ">= 30 % here (60-80 % counting the authored heap)"))
        checks.append(("URM largest piece", "%.2f m" % float(u[:, 0].max()),
                       float(u[:, 0].max()) <= 2.5,
                       "<= 2.5 m (a lintel, not a wall shard)"))
    if (~urm).any():
        rc = A[~urm]
        big = int((rc[:, 0] > 2.0).sum())
        checks.append(("RC pieces > 2 m", "%d" % big, big >= 1,
                       ">= 3 per collapsed building (rafts)"))

    print("  %s  n=%d" % (label, n))
    n_fail = 0
    for name, got, ok, want in checks:
        n_fail += 0 if ok else 1
        print("    %-30s %-10s %-4s  want %s"
              % (name, got, "PASS" if ok else "FAIL", want))
    # size histogram, for reading rather than passing
    edges = [0.0, 0.06, 0.25, 0.55, 1.0, 2.0, 1e9]
    hist = np.histogram(a, bins=edges)[0]
    print("    size by count (longest axis): " + "  ".join(
        "%s-%s %.0f%%" % (("%.2f" % edges[i]) if edges[i] else "0",
                          ("%.2f" % edges[i + 1]) if edges[i + 1] < 1e8 else "+",
                          100.0 * hist[i] / n) for i in range(len(hist))))
    return n_fail


def main(argv):
    bad = check_calls() + check_math()
    print("\n".join("  " + q for q in bad) if bad else "break-shape math OK")
    rc = 1 if bad else 0
    for path in argv:
        if not os.path.exists(path):
            print("missing: %s" % path)
            rc = 1
            continue
        rows = []
        for line in open(path):
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except Exception:
                    pass
        print("\n%s" % path)
        loose = [r for r in rows if r.get("loose")]
        rc += score(rows, "ALL")
        if loose and len(loose) != len(rows):
            rc += score(loose, "LOOSE only")
        for md in sorted(set(str(r.get("mode", "")) for r in rows)):
            sub = [r for r in rows if str(r.get("mode", "")) == md]
            if len(sub) >= 20 and len(sub) != len(rows):
                rc += score(sub, "mode=%s" % (md or "?"))
    return 1 if rc else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
