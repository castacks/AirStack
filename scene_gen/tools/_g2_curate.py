#!/usr/bin/env python
"""_g2_curate.py -- turn the raw `_g2_win_rects.py` sweep into the checked-in
`_G2_WIN_FACES` table.

Round 3, agent G2. The probe measures everything it can find; not everything
it finds is a WINDOW. A 5 x 4.9 m recess 2.1 m deep is a porte-cochere, a 45 %-
filled slot on a rooftop module is a plant screen, a 0.39 m band of
`MI_Building_Green_Trim_A` over a shopfront is a cornice reveal. Rejecting
those by hand in the notes would be unreproducible, so the rules live here and
run over the checked-in JSON.

    python3 scene_gen/tools/_g2_curate.py [scene_gen/_plans/glazing_probe/g2_win_rects.json]
"""
import json
import sys

# Modules whose openings are GLAZED. Everything else the probe found is a
# door recess, an arcade, a plant screen or a cornice reveal.
KEEP = [
    # family 01 -- stone apartment. Upper-storey punched windows only: the
    # 6 m ground band is a stone arcade with a 1.1-2.1 m deep recess, not glass.
    "SM_MBuilding01_Facade_A", "SM_MBuilding01_Facade_B",
    "SM_MBuilding01_Facade_C", "SM_MBuilding01_Facade_D",
    # family 02 -- office. Facades plus the glazed ground-floor bays.
    "SM_MBuilding02_Facade_A", "SM_MBuilding02_Facade_B",
    "SM_MBuilding02_Facade_C", "SM_MBuilding02_FirstFloor_B",
    "SM_MBuilding02_FirstFloor_C", "SM_MBuilding02_FirstFloor_E",
    # family 03 -- brownstone. Facade_B_* are the FRENCH windows that run to
    # the module's own bottom edge (the flood-fill trap).
    "SM_MBuilding03_Facade_A", "SM_MBuilding03_Facade_B_Bottom",
    "SM_MBuilding03_Facade_B_Middle", "SM_MBuilding03_Facade_B_Upper",
    "SM_MBuilding03_Facade_C", "SM_MBuilding03_FirstFloor_A",
    # family 04 -- brick commercial. The UPPER facade windows, which agent G's
    # material probe missed entirely, and the top floor as two windows of two
    # lights each instead of one union rectangle over both.
    "SM_MBuilding04_Facade_A", "SM_MBuilding04_TopFloor_A",
    # Downtown_West terrace: the lvl1 shopfronts (full-width glass, so they
    # punch no hole at all) and the lvl2/lvl3 window bands.
    # NOT `SM_build_b_mod_lvl1_storefront_a_3_5m`: its mesh is authored from
    # local y = 0.00 to 3.00, while `ub.PIECES` records it CENTRED at
    # -1.50..1.54 — a 1.5 m disagreement between the asset and the table
    # `_piece_frame` reads, so anything keyed off `_piece_frame` lands half a
    # module along the wall. The other two lvl1 storefronts are consistent.
    # Flagged for the reviewer; not fixed here (PIECES is not mine).
    "SM_build_b_mod_lvl1_storefront_b_wall3m",
    "SM_build_b_mod_lvl1_storefront_b_wall5m",
    "SM_build_b_mod_lvl2_singlewindow", "SM_build_b_mod_lvl2_doublewindow",
    "SM_build_b_mod_lvl2_widewindow",
    "SM_build_b_mod_lvl3_singlewindow", "SM_build_b_mod_lvl3_doublewindow",
]

MIN_FILL = 0.85     # a window opening is a RECTANGLE; 45-75 % is a louvre
MIN_W = 0.35        # m
MIN_H = 0.50        # m -- kills the 0.39 m cornice reveal over a shopfront
MAX_AR = 6.0        # width/height -- a 10:1 band is a trim recess
MAX_DEEP = 1.80     # m behind the module face: deeper is an arcade or a stoop
MAX_AREA = 22.0     # m^2 -- a 5 x 3.3 m shopfront bay is real; 25 m^2 is a
#                     porte-cochere
GLASSY = ("glass", "window", "glazing")


def main(path):
    with open(path) as f:
        raw = json.load(f)
    out, dropped = {}, []
    for nm in sorted(raw):
        if nm not in KEEP:
            if raw[nm]:
                dropped.append((nm, "not a glazed module", len(raw[nm])))
            continue
        rows = []
        for r in raw[nm]:
            w = r["u1"] - r["u0"]
            h = r["v1"] - r["v0"]
            why = None
            if r["fill"] < MIN_FILL:
                why = "fill {0:.0%}".format(r["fill"])
            elif w < MIN_W or h < MIN_H:
                why = "{0:.2f}x{1:.2f} m".format(w, h)
            elif w / max(1e-6, h) > MAX_AR:
                why = "aspect {0:.1f}".format(w / h)
            elif abs(r["out"]) > MAX_DEEP:
                why = "out {0:.2f} m".format(r["out"])
            elif w * h > MAX_AREA:
                why = "area {0:.1f} m2".format(w * h)
            elif (r["src"] == "hole" and r["mats"]
                  and not any(k in q.lower() for q in r["mats"] for k in GLASSY)):
                # a hole whose lining is bound to a NAMED, non-glass material
                # is a trim recess, not a window (the 0.68 m
                # MI_Building_Green_Trim_A band over a Downtown_West shopfront).
                # Families 01/02/03 report no material at all, so they pass.
                why = "no glass material {0}".format(r["mats"])
            if why:
                dropped.append((nm, why, 1))
                continue
            rows.append((round(r["u0"], 3), round(r["u1"], 3),
                         round(r["v0"], 3), round(r["v1"], 3),
                         round(r["out"], 3), round(r["hu0"], 3),
                         round(r["hu1"], 3), round(r["hv0"], 3),
                         round(r["hv1"], 3)))
        if rows:
            out[nm] = rows
    n = sum(len(v) for v in out.values())
    print("# _G2_WIN_FACES -- {0} modules, {1} openings, measured by".format(
        len(out), n))
    print("# scene_gen/tools/_g2_win_rects.py; curated by "
          "scene_gen/tools/_g2_curate.py.")
    print("# (u0, u1, v0, v1, out) is the GLASS rectangle and the depth the dark")
    print("# quad goes at; (hu0, hu1, hv0, hv1) is the hole in the wall -- the")
    print("# reveal, which the racked frame and the sill litter key off.")
    print("_G2_WIN_FACES = {")
    for nm in sorted(out):
        print('    "{0}": ['.format(nm))
        for r in out[nm]:
            print("        ({0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}),".format(*r))
        print("    ],")
    print("}")
    print()
    for d in dropped:
        print("# dropped {0:<44s} {1} x{2}".format(d[0], d[1], d[2]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1
                          else "scene_gen/_plans/glazing_probe/g2_win_rects.json"))
