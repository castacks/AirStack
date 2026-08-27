#!/usr/bin/env python
"""_g2_check_table.py -- offline sanity check of `quake_flow._G2_WIN_FACES`.

No pxr, no Isaac, no GPU: it parses the table out of the source with `ast` and
measures it against `urban_building.PIECES`, which is the same table
`_piece_frame` reads. Catches the paste errors and the frame mistakes that
would otherwise only show up as glass floating a metre off a wall in a bench
capture:

  * u outside the piece's own width -- e.g. the Downtown_West "dw" frame,
    where `_b_face_pt` measures u from the piece's LEFT end and the pieces are
    centred on their pivot, so the table's u must be `local_y - ymin`.
  * v outside the piece's own height.
  * a glass rectangle bigger than the hole it sits in.
  * a quad pushed so far back it is behind the module and hanging in the room.

    python3 scene_gen/tools/_g2_check_table.py
"""
import ast
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "scene_gen"))

MAX_DEPTH = 1.80


def main():
    from detail import urban_building as ub
    src = open(os.path.join(REPO, "scene_gen", "disaster", "quake_flow.py")).read()
    tbl = None
    for n in ast.parse(src).body:
        if isinstance(n, ast.Assign) and getattr(n.targets[0], "id", "") == "_G2_WIN_FACES":
            tbl = ast.literal_eval(n.value)
    if not tbl:
        print("_G2_WIN_FACES not found"); return 1
    bad, n_op = [], 0
    for nm in sorted(tbl):
        meas = ub.PIECES.get(nm)
        if not meas:
            bad.append((nm, "not in urban_building.PIECES")); continue
        sx, sy, sz, xmin, ymin, zmin = meas
        dw = ub._kit(nm)[1] == "dw"
        width = sy if dw else sx
        thick = sx if dw else sy
        for r in tbl[nm]:
            n_op += 1
            u0, u1, v0, v1, o, hu0, hu1, hv0, hv1 = r
            if not (-0.05 <= u0 < u1 <= width + 0.05):
                bad.append((nm, "u {0:.2f}..{1:.2f} outside 0..{2:.2f}".format(u0, u1, width)))
            if not (zmin - 0.05 <= v0 < v1 <= zmin + sz + 0.05):
                bad.append((nm, "v {0:.2f}..{1:.2f} outside {2:.2f}..{3:.2f}".format(
                    v0, v1, zmin, zmin + sz)))
            if u0 < hu0 - 0.02 or u1 > hu1 + 0.02 or v0 < hv0 - 0.02 or v1 > hv1 + 0.02:
                bad.append((nm, "glass rect larger than its hole"))
            if o > 0.0 or o < -MAX_DEPTH:
                bad.append((nm, "out {0:.2f} outside -{1:.2f}..0".format(o, MAX_DEPTH)))
            if o < -(thick + 1.3):
                bad.append((nm, "out {0:.2f} is behind a {1:.2f} m module".format(o, thick)))
    print("_G2_WIN_FACES: {0} modules, {1} openings, {2} problem(s)".format(
        len(tbl), n_op, len(bad)))
    for b in bad:
        print("  {0:<44s} {1}".format(*b))
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
