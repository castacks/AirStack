"""Offline check of the round-2 break-line math (no pxr, no Isaac).

Slices the pure-python helpers out of quake_flow.py by AST and exercises them:
  * `_a_stepped`  is a bounded staircase whose risers are all >= one course
  * `_a_torn`     has metre-scale structure, not one lazy arc
  * `_a_hole_outline` is a closed, strictly positive, NON-circular outline
Run from the repo root: python3 scene_gen/tools/test_break_lines.py
"""
import ast, math, bisect, random, textwrap, sys

SRC = "scene_gen/disaster/quake_flow.py"
WANT = {"_a_stepped", "_a_torn", "_a_wobble", "_a_hole_outline"}


def load():
    src = open(SRC).read()
    lines = src.splitlines()
    ns = {"math": math, "bisect": bisect,
          "STEP_H": ((0.30, 1.20), (0.10, 0.40)),
          "STEP_V": ((0.20, 0.60), (0.10, 0.30))}
    for node in ast.parse(src).body:
        if isinstance(node, ast.FunctionDef) and node.name in WANT:
            exec(compile(textwrap.dedent(
                "\n".join(lines[node.lineno - 1:node.end_lineno])),
                "<sliced>", "exec"), ns)
    missing = WANT - set(ns)
    assert not missing, "not found in %s: %s" % (SRC, missing)
    return ns


def turning_points(v):
    return sum(1 for a, b, c in zip(v, v[1:], v[2:])
               if (b - a) * (c - b) < 0)


def main():
    ns = load()
    bad = []
    # --- stepped (masonry courses) -------------------------------------
    for seed in range(8):
        rng = random.Random(seed)
        f = ns["_a_stepped"](rng, 0.6, 22.0, (0.30, 1.20), (0.10, 0.40))
        v = [f(i * 0.02) for i in range(1101)]
        risers = [abs(b - a) for a, b in zip(v, v[1:]) if abs(b - a) > 1e-12]
        if max(abs(q) for q in v) > 0.6 + 1e-9:
            bad.append("stepped %d: exceeds amp" % seed)
        if not (12 <= len(risers) <= 80):
            bad.append("stepped %d: %d risers over 22 m" % (seed, len(risers)))
        if risers and min(risers) < 0.099:
            bad.append("stepped %d: riser %.3f m < one course" % (seed, min(risers)))
    # --- torn (concrete) ------------------------------------------------
    for seed in range(8):
        rng = random.Random(seed)
        g = ns["_a_torn"](rng, 0.5, 22.0)
        v = [g(i * 0.02) for i in range(1101)]
        if max(abs(q) for q in v) > 0.5 * 1.001:
            bad.append("torn %d: exceeds amp" % seed)
        tp = turning_points(v)
        if tp < 10:
            bad.append("torn %d: only %d turning points over 22 m (an arc)" % (seed, tp))
    # --- hole outline ---------------------------------------------------
    for seed in range(10):
        rng = random.Random(seed)
        rad, rmax = ns["_a_hole_outline"](rng)
        v = [rad(i * 0.01) for i in range(629)]
        if min(v) <= 0.15:
            bad.append("hole %d: radius collapses to %.2f" % (seed, min(v)))
        if abs(rad(0.3) - rad(0.3 + 2 * math.pi)) > 1e-9:
            bad.append("hole %d: not closed" % seed)
        if max(v) - min(v) < 0.25:
            bad.append("hole %d: spread %.2f — too round to read as a tear"
                       % (seed, max(v) - min(v)))
        if rmax < max(v) - 1e-6:
            bad.append("hole %d: rmax %.2f under-reports %.2f" % (seed, rmax, max(v)))
    print("\n".join(bad) if bad else "break-line math OK")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
