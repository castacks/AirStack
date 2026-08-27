"""quake — assemble an earthquake-damaged district from baked archetypes.

The scene-level half of the earthquake pipeline (`quake_flow` is the
per-building half). The downtown layout is generated with the PRISTINE kit
archetypes in its building pools, so the packer lays out real footprints;
this pass then reads the disaster field at every placed building, picks an
EMS-98 grade for its construction type, and re-points the reference at the
baked damaged archetype of that grade. Same origin, same pose — the bake
re-centred every archetype on the building's own centre — so nothing moves.

Two things are done here rather than in the bake:

* **tilt_sink** is a rigid transform on the pristine archetype (rotate about
  a base edge, sink). Baking it would multiply the library for no gain.
* **which building gets what** is a scene decision: the field, the
  construction type's vulnerability, a per-building jitter so the grades are
  not contour rings, and a slenderness gate on tilting (Adapazarı: only
  H/B > 2 blocks free on a side tilted significantly).

Module level is stdlib + numpy + pxr, the same bar the rest of `scene_gen`
holds.
"""

import json
import math
import os
import random

from pxr import Gf, Sdf, Usd, UsdGeom

from . import quake_flow as qf

ARCH_PREFIX = "bld_"


def load_manifest(arch_dir):
    """{(style, level): record} from the bake's archetypes.json."""
    path = os.path.join(arch_dir, "archetypes.json")
    with open(path) as fh:
        recs = json.load(fh)
    out = {}
    for r in recs:
        out[(r["style"], r["level"])] = r
    return out


def style_of(usd):
    """`.../bld_office_wide_DG0.usd` -> ("office_wide", "DG0")."""
    base = os.path.basename(str(usd))
    if not base.startswith(ARCH_PREFIX) or not base.endswith(".usd"):
        return None, None
    stem = base[len(ARCH_PREFIX):-4]
    style, _, level = stem.rpartition("_DG")
    if not style:
        return None, None
    level = "DG" + level
    # strip a `_vN` variant suffix from the level
    if "_v" in level:
        level = level.split("_v")[0]
    return style, level


def _variants(manifest, style, grade):
    """Every baked level for (style, grade): `DG3`, `DG3_v1`, `DG3_v2`..."""
    out = []
    for (st, lv), r in manifest.items():
        if st == style and (lv == grade or lv.startswith(grade + "_v")):
            out.append(r)
    return out


def _region(config):
    w, h = config["layout"]["region_m"]
    return (-float(w) / 2.0, -float(h) / 2.0, float(w) / 2.0, float(h) / 2.0)


def _soft_soil(config, rng):
    """The liquefaction patch: one ellipse on the plate where foundation
    failures concentrate (Adapazari, Christchurch, Niigata: whole districts
    at once, not a scatter). Returns `inside(x, y) -> 0..1` (1 at the centre,
    0 outside), or None when `disaster.soft_soil: false`."""
    dis = config.get("disaster") or {}
    soil = dis.get("soft_soil")
    if soil is False:
        return None
    soil = soil if isinstance(soil, dict) else {}
    x0, y0, x1, y1 = _region(config)
    w, h = x1 - x0, y1 - y0
    c = soil.get("center") or [rng.uniform(x0 + 0.25 * w, x1 - 0.25 * w),
                               rng.uniform(y0 + 0.25 * h, y1 - 0.25 * h)]
    cx, cy = float(c[0]), float(c[1])
    rx = float(soil.get("rx_m", 0.28 * w))
    ry = float(soil.get("ry_m", 0.2 * h))
    ang = math.radians(float(soil.get("angle_deg", rng.uniform(0, 180))))

    def inside(x, y):
        dx, dy = x - cx, y - cy
        u = (dx * math.cos(ang) + dy * math.sin(ang)) / rx
        v = (-dx * math.sin(ang) + dy * math.cos(ang)) / ry
        r = math.hypot(u, v)
        return max(0.0, 1.0 - r * r)
    inside.centre = (cx, cy)
    inside.radii = (rx, ry)
    return inside


def _blocked(p, rec, fall_yaw_deg, H, placements):
    """Would a building falling toward `fall_yaw_deg` land on another
    building? Sweeps a rectangle H long past the wall line and tests other
    buildings' centres against it."""
    x, y = float(p["x_m"]), float(p["y_m"])
    W, D = float(rec.get("W", 20.0)), float(rec.get("D", 20.0))
    a = math.radians(fall_yaw_deg)
    ux, uy = math.cos(a), math.sin(a)
    half = max(W, D) / 2.0
    for q in placements:
        if q is p or q.get("category") != "house":
            continue
        dx, dy = float(q["x_m"]) - x, float(q["y_m"]) - y
        along = dx * ux + dy * uy
        across = abs(-dx * uy + dy * ux)
        if half < along < half + H + 4.0 and across < half + 6.0:
            return True
    return False


def assemble(stage, config, placements, arch_dir, seed=11, ssf=1.0,
             tilt_chance=None, verbose=True, foundation_rate=None):
    """Swap placed pristine archetypes for damaged ones by the field.

    Then the FOUNDATION pass: on the soft-soil patch a share of the
    buildings that did not collapse (DG0-DG3) get a baked foundation
    archetype instead — `SETTLE` (level sink), `TILT` (10-30 deg on a raft)
    or `OV` (overturned, one per scene, only where the fall is clear) —
    and a few elsewhere get the mild lean (`_tilt_prim`).

    Returns a stats dict: `buildings`, `tally` (grade -> count),
    `tilted`, `missing`, `foundation`, `soft_soil`.
    """
    import scene_generator as sg

    dis = config.get("disaster") or {}
    field = sg.make_damage_field(dis.get("field") or {"kind": "uniform", "inside": 0.0},
                                 _region(config))
    debris = dis.get("debris") or {}
    if tilt_chance is None:
        tilt_chance = float(debris.get("tilt_chance", 0.25))
    tilt_deg = debris.get("tilt_deg", [3.0, 9.0])
    sink_m = debris.get("sink_m", [0.4, 1.4])
    manifest = load_manifest(arch_dir)
    rng = random.Random(seed + 4242)
    soft = _soft_soil(config, random.Random(seed + 77))
    grade_scale = float(dis.get("grade_scale", 1.0))
    if foundation_rate is None:
        soil_cfg = dis.get("soft_soil") if isinstance(dis.get("soft_soil"), dict) else {}
        foundation_rate = float(soil_cfg.get("rate", 0.55))
    tally, n, tilted, missing = {}, 0, 0, 0
    n_found = {"SETTLE": 0, "TILT": 0, "OV": 0}
    ov_used = False
    records = []
    for p in placements:
        style, level = style_of(p.get("usd"))
        if not style or p.get("category") != "house":
            continue
        path = p.get("prim_path")
        prim = stage.GetPrimAtPath(path) if path else None
        if not prim or not prim.IsValid():
            continue
        rec = manifest.get((style, "DG0")) or {}
        btype = rec.get("type") or qf.FAMILY_TYPE.get(rec.get("family", ""), "urm")
        x, y = float(p["x_m"]), float(p["y_m"])
        inten = float(field(x, y))
        grade = qf.level_for_intensity(inten * grade_scale, btype, rng)
        n += 1
        chosen = None
        if grade != "DG0":
            vs = _variants(manifest, style, grade)
            if not vs:
                # step down until something is baked, so a missing DG4
                # becomes a DG3 rather than a pristine building in the core
                g = int(grade[2])
                while g > 0 and not vs:
                    g -= 1
                    vs = _variants(manifest, style, "DG{0}".format(g))
                missing += 1
            if vs:
                chosen = rng.choice(vs)
                grade = chosen["level"].split("_v")[0]
                refs = prim.GetReferences()
                refs.ClearReferences()
                refs.AddReference(chosen["usd"])
                prim.Load()
                p["usd"] = chosen["usd"]
        # FOUNDATION FAILURE: intact (or lightly damaged) buildings on the
        # strong-shaking side lean and sink. Gated on slenderness the way
        # Adapazarı was, and on the grade — a pancaked building cannot also
        # tilt, and a DG4 archetype was settled flat by PhysX so must not be
        # rolled (the wildfire skill's rule).
        H = float(rec.get("H", 12.0))
        B = max(1.0, min(float(rec.get("W", 20.0)), float(rec.get("D", 20.0))))
        slender = H / B
        # FOUNDATION FAILURE ON THE SOFT-SOIL PATCH. Only buildings still
        # standing (a pancake cannot also tilt, and a baked collapse was
        # settled flat — never roll it). Slender ones tilt or go over, squat
        # ones settle. One overturn per scene, only where the fall lands on
        # nothing (the OV archetype falls toward its own front, -Y).
        soil_w = soft(x, y) if soft else 0.0
        # inside the patch the weight is 0.35-1.0 (not (1-r^2), which fell to
        # nothing by mid-radius and left one settled building in a 250 m
        # scene); a small chance anywhere on the plate as well
        p_found = (foundation_rate * (0.35 + 0.65 * soil_w)) if soil_w > 0.0 else 0.05
        if grade in ("DG0", "DG1", "DG2", "DG3") and rng.random() < p_found:
            r = rng.random()
            if slender > 1.5 and not ov_used and r < 0.3 and inten > 0.4 \
                    and btype != "rc_glass" and H <= 36.0:
                lvl = "OV"
            elif slender > 1.0 and r < 0.65:
                lvl = "TILT"
            else:
                lvl = "SETTLE"
            if lvl == "OV" and _blocked(p, rec, float(p.get("yaw_deg", 0.0)) - 90.0,
                                        H, placements):
                lvl = "TILT"
            vs = _variants(manifest, style, lvl)
            if not vs and lvl != "SETTLE":
                lvl = "SETTLE"
                vs = _variants(manifest, style, lvl)
            if vs:
                chosen = rng.choice(vs)
                refs = prim.GetReferences()
                refs.ClearReferences()
                refs.AddReference(chosen["usd"])
                prim.Load()
                p["usd"] = chosen["usd"]
                if lvl == "OV":
                    ov_used = True
                n_found[lvl] += 1
                tally[lvl] = tally.get(lvl, 0) + 1
                records.append(dict(style=style, x=x, y=y, intensity=round(inten, 3),
                                    grade=lvl, prim=path))
                continue
        p_tilt = tilt_chance * inten * (1.3 if slender > 1.6 else 0.6)
        if grade in ("DG0", "DG1", "DG2", "DG3") and rng.random() < p_tilt:
            deg = rng.uniform(float(tilt_deg[0]), float(tilt_deg[1]))
            snk = rng.uniform(float(sink_m[0]), float(sink_m[1]))
            _tilt_prim(stage, prim, p, rec, deg, snk, rng, ssf)
            tilted += 1
            grade = grade + "+tilt"
        tally[grade] = tally.get(grade, 0) + 1
        records.append(dict(style=style, x=x, y=y, intensity=round(inten, 3),
                            grade=grade, prim=path))
    if verbose:
        print("[quake] {0} buildings: {1}; {2} mild-tilted, {3} grade(s) stepped down "
              "for a missing archetype".format(
                  n, ", ".join("{0}={1}".format(k, v) for k, v in sorted(tally.items())),
                  tilted, missing))
        if soft:
            print("[quake] soft-soil patch at ({0:.0f}, {1:.0f}) radii {2:.0f} x {3:.0f} m: "
                  "SETTLE {4}, TILT {5}, OV {6}".format(
                      soft.centre[0], soft.centre[1], soft.radii[0], soft.radii[1],
                      n_found["SETTLE"], n_found["TILT"], n_found["OV"]))
    return {"buildings": n, "tally": tally, "tilted": tilted, "missing": missing,
            "records": records, "foundation": n_found,
            "soft_soil": (soft.centre + soft.radii) if soft else None}


def _tilt_prim(stage, prim, p, rec, deg, sink, rng, ssf):
    """Rotate a placed archetype about one of its base edges and sink it."""
    W, D = float(rec.get("W", 20.0)), float(rec.get("D", 20.0))
    yaw = float(p.get("yaw_deg", 0.0))
    side = rng.choice(["S", "E", "N", "W"])
    nx, ny = {"S": (0.0, -1.0), "N": (0.0, 1.0), "E": (1.0, 0.0), "W": (-1.0, 0.0)}[side]
    a = math.radians(yaw)
    ox, oy = nx * math.cos(a) - ny * math.sin(a), nx * math.sin(a) + ny * math.cos(a)
    edge = (D if side in ("S", "N") else W) / 2.0
    cx, cy = float(p["x_m"]) * ssf, float(p["y_m"]) * ssf
    px, py = cx + ox * edge * ssf, cy + oy * edge * ssf
    ax, ay = -oy, ox
    P = Gf.Vec3d(px, py, float(p.get("z_m", 0.0)) * ssf)
    R = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(ax, ay, 0.0), -abs(deg)))
    M = (Gf.Matrix4d().SetTranslate(-P) * R * Gf.Matrix4d().SetTranslate(P)
         * Gf.Matrix4d().SetTranslate(Gf.Vec3d(0.0, 0.0, -abs(sink) * ssf)))
    xf = UsdGeom.Xformable(prim)
    local = UsdGeom.XformCache().GetLocalTransformation(prim)[0]
    m = local * M
    tr = Gf.Transform(m)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(tr.GetTranslation()))
    q = tr.GetRotation().GetQuat()
    xf.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(q.GetImaginary())))
    xf.AddScaleOp().Set(Gf.Vec3f(tr.GetScale()))


# ---------------------------------------------------------------------------
# BATCH D — the ground and the gaps: dust halo, fissures, boils, pounding
# ---------------------------------------------------------------------------
# A FLAT pale grey, not a photographed surface: the halo is a film of
# concrete dust over whatever is there, and a pavement map (with its moss)
# projected through the overlay read as a second pavement.
DUST_TEXTURE = "airstack://scene_gen/assets/materials/quake/Dust_Grey.png"


def _bld_masses(records, manifest, placements):
    """[(x, y, yaw, W, D, H, grade)] for every assembled building."""
    out = []
    by_prim = {p.get("prim_path"): p for p in placements}
    for r in records:
        p = by_prim.get(r["prim"]) or {}
        rec = manifest.get((r["style"], "DG0")) or {}
        out.append((float(r["x"]), float(r["y"]), float(p.get("yaw_deg", 0.0)),
                    float(rec.get("W", 20.0)), float(rec.get("D", 20.0)),
                    float(rec.get("H", 12.0)), r["grade"]))
    return out


def ground_effects(stage, config, stats, placements, arch_dir, parent, ssf,
                   seed=11, dust=True, fissures=True, boils=True, pounding=True,
                   verbose=True):
    """Author what the quake did to the GROUND and the GAPS, after `assemble`.

    * DUST HALO: every DG4/DG5 building throws a grey concrete-dust film over
      the ground within 1-2 H (the research's halo), as a translucent overlay
      through `ground.build_overlay` — so it needs BOTH fractional-cutout
      flags on the app command line AND re-asserted after the stage loads,
      exactly as the burn scar does (wildfire skill). Roads, walks and roofs
      under it stay legible; the whiteness is the read.
    * FISSURES: on the soft-soil patch, meandering tension cracks 0.3-0.9 m
      wide as dark strips, roughly parallel to the patch's long axis.
    * BOILS: sand-boil fans across the patch on open ground.
    * POUNDING: where two buildings stand within 1.5 m, a vertical scar on the
      taller one at the shorter one's roof line and a wedge of rubble in the
      gap (Mexico City 1985: 40 % of buildings collided).
    """
    from pxr import Sdf, UsdGeom
    import scene_generator as sg
    from . import ground, quake_flow as qf

    rng = random.Random(seed + 313)
    manifest = load_manifest(arch_dir)
    blds = _bld_masses(stats.get("records", []), manifest, placements)
    x0, y0, x1, y1 = _region(config)
    mats = qf.materials(stage, parent)
    scope = parent + "/quake_ground"
    UsdGeom.Scope.Define(stage, Sdf.Path(scope))
    n_dust = n_fis = n_boil = n_pound = 0

    # a minimal ctx so the per-building helpers can author here
    def _ctx(tag):
        return {"stage": stage, "parent": scope, "rng": rng, "mats": mats, "tag": tag,
                "authored": [], "static_extra": [], "loose": [], "velocity": {},
                "notes": [], "n_uid": 0, "info": {"type": "rc"}}

    dcfg = (config.get("disaster") or {}).get("dust") or {}
    reach5 = float(dcfg.get("reach_h5", 1.0))
    reach4 = float(dcfg.get("reach_h4", 0.5))
    op_max = float(dcfg.get("opacity_max", 0.42))
    if dust:
        heavy = [b for b in blds if b[6] in ("DG4", "DG5", "OV")]

        def coverage_at(x, y):
            c = 0.0
            for bx, by, _yaw, W, D, H, g in heavy:
                r = math.hypot(x - bx, y - by) - max(W, D) * 0.5
                # ~1.0 H / 0.5 H, not the research's outer 2 H: at 2 H eleven
                # collapses whitened the whole 250 m plate and buried the
                # road markings. The halo has to leave clean ground between
                # piles for the piles to read as piles.
                reach = (reach5 if g in ("DG5", "OV") else reach4) * H
                if r < reach:
                    c += (0.8 if g in ("DG5", "OV") else 0.3) * max(0.0, 1.0 - max(0.0, r) / reach) ** 1.8
            return min(1.0, c)
        if heavy:
            made = ground.build_overlay(
                stage, coverage_at, (x0, y0, x1, y1), ssf, 0.035,
                material_parent=scope, root=scope + "/dust", cell_m=3.0, bands=10,
                tile_m=18.0, op_range=(0.06, op_max),
                texture=sg._join_asset_root(DUST_TEXTURE, ""), verbose=verbose)
            n_dust = len(made)

    soft = stats.get("soft_soil")
    if soft and (fissures or boils):
        cx, cy, rx, ry = soft
        ctx = _ctx("soil")
        if fissures:
            # 2-4 cracks, each a chain of thin dark boxes following a noisy
            # line roughly along the patch's long axis
            n_f = rng.randrange(2, 5)
            ang0 = 0.0 if rx >= ry else 90.0
            for k in range(n_f):
                off = rng.uniform(-0.6, 0.6) * min(rx, ry)
                length = rng.uniform(0.8, 1.6) * max(rx, ry)
                a = math.radians(ang0 + rng.uniform(-25, 25))
                px = cx - math.cos(a) * length / 2.0 - math.sin(a) * off
                py = cy - math.sin(a) * length / 2.0 + math.cos(a) * off
                step = 3.0
                w = rng.uniform(0.3, 0.9)
                heading = a
                for i in range(int(length / step)):
                    heading += math.radians(rng.uniform(-14, 14))
                    nx, ny = px + math.cos(heading) * step, py + math.sin(heading) * step
                    mx, my = (px + nx) / 2.0, (py + ny) / 2.0
                    if not (x0 < mx < x1 and y0 < my < y1):
                        break
                    path = "{0}/fissure_{1}".format(scope, qf._uid(ctx))
                    qf._box(stage, path, mx, my, 0.02, step * 1.15, w * rng.uniform(0.7, 1.3),
                            0.06, math.degrees(heading), mats["rebar"])
                    # an upthrown lip on one side
                    if rng.random() < 0.5:
                        lp = "{0}/lip_{1}".format(scope, qf._uid(ctx))
                        sgn = rng.choice((-1, 1))
                        qf._box(stage, lp, mx - math.sin(heading) * sgn * w, my + math.cos(heading) * sgn * w,
                                0.06, step * 1.1, 0.5, 0.12, math.degrees(heading), mats["concrete"])
                    px, py = nx, ny
                    n_fis += 1
        if boils:
            n_b = int(rx * ry / 1400.0) + 4
            m = {"cx": cx, "cy": cy, "yaw": 0.0, "W": 1.0, "D": 1.0, "z0": 0.0, "top": 6.0}
            # scatter over the ellipse, avoiding building footprints
            for k in range(n_b):
                for _try in range(8):
                    u, v = rng.uniform(-1, 1), rng.uniform(-1, 1)
                    if u * u + v * v > 1.0:
                        continue
                    bx, by = cx + u * rx, cy + v * ry
                    if any(abs(bx - b[0]) < b[3] / 2.0 + 2.0 and abs(by - b[1]) < b[4] / 2.0 + 2.0
                           for b in blds):
                        continue
                    mm = dict(m, cx=bx, cy=by)
                    qf._ejecta(ctx, mm, 1, reach_frac=0.3)
                    n_boil += 1
                    break

    if pounding:
        ctx = _ctx("pound")
        done = set()
        for i, a in enumerate(blds):
            for j, b in enumerate(blds):
                if j <= i or (i, j) in done:
                    continue
                if a[6] in ("DG5", "OV") or b[6] in ("DG5", "OV"):
                    continue
                gx = abs(a[0] - b[0]) - (a[3] + b[3]) / 2.0
                gy = abs(a[1] - b[1]) - (a[4] + b[4]) / 2.0
                # side by side along x, or along y, with a small gap
                if -2.0 < gx < 1.5 and gy < -3.0:
                    axis, gap = "x", gx
                elif -2.0 < gy < 1.5 and gx < -3.0:
                    axis, gap = "y", gy
                else:
                    continue
                if abs(a[5] - b[5]) < 2.5:
                    continue                      # same height: no hammering
                if rng.random() > 0.6:
                    continue
                done.add((i, j))
                tall, short = (a, b) if a[5] > b[5] else (b, a)
                # the scar: a dark strip on the tall one's face at the short
                # one's roof height, plus a wedge of rubble in the gap
                if axis == "x":
                    sx = 1.0 if short[0] > tall[0] else -1.0
                    fx = tall[0] + sx * tall[3] / 2.0
                    fy = tall[1]
                    yaw = 90.0
                    L = min(tall[4], short[4]) * 0.8
                else:
                    sy = 1.0 if short[1] > tall[1] else -1.0
                    fx = tall[0]
                    fy = tall[1] + sy * tall[4] / 2.0
                    yaw = 0.0
                    L = min(tall[3], short[3]) * 0.8
                zc = short[5]
                path = "{0}/scar_{1}".format(scope, qf._uid(ctx))
                qf._box(stage, path, fx, fy, zc, L, 0.35, 1.2, yaw, mats["dark_concrete"])
                for k in range(int(L / 1.5)):
                    t = rng.uniform(-0.5, 0.5) * L
                    ex = fx + (t if yaw == 0.0 else 0.0)
                    ey = fy + (t if yaw == 90.0 else 0.0)
                    ch = "{0}/pound_{1}".format(scope, qf._uid(ctx))
                    sz = rng.uniform(0.3, 0.9)
                    qf._box(stage, ch, ex, ey, rng.uniform(0.1, 0.8), sz, sz * 0.7, sz * 0.5,
                            rng.uniform(0, 180), mats["dark_concrete"] if rng.random() < 0.5 else mats["brick"])
                n_pound += 1
    if verbose:
        print("[quake] ground: {0} dust band(s), {1} fissure segment(s), {2} boil(s), "
              "{3} pounding scar(s)".format(n_dust, n_fis, n_boil, n_pound))
    return {"dust_bands": n_dust, "fissures": n_fis, "boils": n_boil, "pounding": n_pound}
