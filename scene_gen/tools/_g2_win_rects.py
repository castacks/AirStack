#!/usr/bin/env python
"""_g2_win_rects.py -- the PUNCHED WINDOW openings of every kit wall module.

Round 3, agent G2. Agent G's `_g_glass_rects.py` keys on the bound MATERIAL,
so it found families 04/05 (separate glass geometry) and nothing at all on
families 01/02/03 -- the apartment, office and brownstone stock, i.e. most of
the city -- whose glass is painted into the facade map. `r_storefront_glass`
then fell back to a windrow at the sill: an office at DG3 with 82 pieces of
glass on the pavement and not one broken window.

The REVEALS are real geometry, so the opening does not have to be guessed.

Two passes, because the kit does it two ways:

  A. HOLE PASS -- the window is a hole punched through the wall.
     1. Every face into `_piece_frame` coordinates: u along the piece, v up,
        o OUTWARD (canonical kit o = -y; Downtown_West "dw" kit o = +x).
     2. WALL PLANE = the outward-facing plane carrying the most area.
     3. Rasterise (2 cm) every face at or in FRONT of that plane. Anything
        deeper is reveal or interior and must not fill the hole.
     4. Flood the OUTSIDE in from the raster border -- but only from border
        cells where the module has NO geometry at any depth. Without that
        guard a French window or a shopfront that runs to the bottom edge of
        its module (SM_MBuilding03_Facade_B_Upper, x 0.79..3.21, z 0.00..2.61)
        leaks and is missed.
     5. Unflooded empty components are the openings.
     6. GLASS PLANE inside an opening = the SHALLOWEST recessed outward-facing
        plane holding >= `G_LIGHT_FRAC` of the opening's area. Shallowest, not
        deepest: a kit window is a stepped reveal with two lights at different
        depths (fam01 Facade_A: lower light y = 0.25, upper light y = 0.20),
        and a dark quad behind the deepest one leaves the upper light still
        showing its painted glass. 2 cm in FRONT of the shallowest light
        blanks every light and keeps every reveal step.

  B. GLASS PASS -- the window is a glass MESH (fam 04/05, Downtown_West
     storefronts, CivilianArea sashes). Faces bound to a glass material,
     clustered by plane and then by connectivity, one rectangle per pane
     group. This is how a full-width shopfront -- which punches no hole at
     all, because there is no wall left to punch -- gets measured.

    scene_gen/tools/_t_pxr.sh scene_gen/tools/_g2_win_rects.py [G_NAMES=a,b]
      [G_JSON=/abs/path.json] [G_ALL=1]

Prints a paste-ready `_G2_WIN_FACES` dict plus a per-module audit.
"""
import json
import os
import sys

REPO = "/isaac-sim/AirStack"
sys.path.insert(0, os.path.join(REPO, "scene_gen"))

NAMES = [n for n in os.environ.get("G_NAMES", "").split(",") if n]
CELL = float(os.environ.get("G_CELL", "0.02"))
MIN_AREA = float(os.environ.get("G_MIN_AREA", "0.25"))   # m^2: below this it is a vent
MIN_SIDE = float(os.environ.get("G_MIN_SIDE", "0.30"))   # m
LIGHT_FRAC = float(os.environ.get("G_LIGHT_FRAC", "0.08"))
LIGHT_GAP = float(os.environ.get("G_LIGHT_GAP", "0.12"))   # m: one glazing cluster
JSON_OUT = os.environ.get("G_JSON", "")
ALL_PIECES = os.environ.get("G_ALL", "") == "1"


def _faces(stage, scale):
    from pxr import Gf, Usd, UsdGeom, UsdShade
    out = []
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts:
            continue
        xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        wp = []
        for q in pts:
            g = xf.Transform(Gf.Vec3d(q[0], q[1], q[2]))
            wp.append((g[0] * scale, g[1] * scale, g[2] * scale))
        fmat = {}
        for s in UsdGeom.Subset.GetAllGeomSubsets(m):
            try:
                bm = UsdShade.MaterialBindingAPI(s.GetPrim()).ComputeBoundMaterial()[0]
            except Exception:
                bm = None
            nm = ""
            if bm and bm.GetPrim().IsValid():
                nm = bm.GetPrim().GetName()
                for st_ in (bm.GetPrim().GetPrimStack() or []):
                    lp = getattr(st_.layer, "identifier", "")
                    if "Materials/" in lp:
                        nm = lp.rsplit("/", 1)[-1].rsplit(".", 1)[0]
                        break
            for fi in (s.GetIndicesAttr().Get() or []):
                fmat[int(fi)] = nm
        k = 0
        for fi, c in enumerate(counts):
            c = int(c)
            poly = [wp[idx[k + j]] for j in range(c)]
            k += c
            out.append((poly, fmat.get(fi, "")))
    return out


def _uvo(poly, dw, u_off=0.0):
    """(u along the piece, v = world-local height, o = OUTWARD).

    `_b_face_pt` measures u from 0 at the piece's left end. Downtown_West
    pieces are CENTRED on their pivot and run along +Y, and `_piece_frame`
    hands `_b_face_pt` a width of `sy` with `u - width/2` applied inside -- so
    the table's u is `local_y - ymin`, not local_y. Getting that wrong puts
    every dw storefront half a module along the wall."""
    if dw:
        return [(q[1] - u_off, q[2], q[0]) for q in poly]
    return [(q[0], q[2], -q[1]) for q in poly]


def _newell(poly):
    nx = ny = nz = 0.0
    n = len(poly)
    for j in range(n):
        a, b = poly[j], poly[(j + 1) % n]
        nx += (a[1] - b[1]) * (a[2] + b[2])
        ny += (a[2] - b[2]) * (a[0] + b[0])
        nz += (a[0] - b[0]) * (a[1] + b[1])
    return (nx, ny, nz)


def _area_uv(uv):
    a = 0.0
    for k in range(len(uv)):
        p, q = uv[k], uv[(k + 1) % len(uv)]
        a += p[0] * q[1] - q[0] * p[1]
    return abs(a) / 2.0


class Ras(object):
    def __init__(self, u0, u1, v0, v1):
        self.u0, self.v0 = u0, v0
        self.nu = max(4, int((u1 - u0) / CELL) + 1)
        self.nv = max(4, int((v1 - v0) / CELL) + 1)
        self.g = bytearray(self.nu * self.nv)

    def fill(self, poly_uv):
        g, nu, nv, u0, v0 = self.g, self.nu, self.nv, self.u0, self.v0
        umin = min(q[0] for q in poly_uv); umax = max(q[0] for q in poly_uv)
        vmin = min(q[1] for q in poly_uv); vmax = max(q[1] for q in poly_uv)
        if umax - umin < 1e-6 or vmax - vmin < 1e-6:
            ja = max(0, int((vmin - v0) / CELL)); jb = min(nv - 1, int((vmax - v0) / CELL))
            ia = max(0, int((umin - u0) / CELL)); ib = min(nu - 1, int((umax - u0) / CELL))
            for j in range(ja, jb + 1):
                for i in range(ia, ib + 1):
                    g[j * nu + i] = 1
            return
        ja = max(0, int((vmin - v0) / CELL)); jb = min(nv - 1, int((vmax - v0) / CELL))
        n = len(poly_uv)
        for j in range(ja, jb + 1):
            vy = v0 + (j + 0.5) * CELL
            xs = []
            for k in range(n):
                a, b = poly_uv[k], poly_uv[(k + 1) % n]
                if (a[1] > vy) == (b[1] > vy):
                    continue
                t = (vy - a[1]) / (b[1] - a[1])
                xs.append(a[0] + t * (b[0] - a[0]))
            xs.sort()
            row = j * nu
            for k in range(0, len(xs) - 1, 2):
                ia = max(0, int((xs[k] - u0) / CELL))
                ib = min(nu - 1, int((xs[k + 1] - u0) / CELL))
                for i in range(ia, ib + 1):
                    g[row + i] = 1

    def reachable_from_border(self):
        """Empty cells connected to the raster border: the OUTSIDE."""
        nu, nv, g = self.nu, self.nv, self.g
        seen = bytearray(nu * nv)
        stack = []
        for i in range(nu):
            for j in (0, nv - 1):
                k = j * nu + i
                if not g[k] and not seen[k]:
                    seen[k] = 1; stack.append((i, j))
        for j in range(nv):
            for i in (0, nu - 1):
                k = j * nu + i
                if not g[k] and not seen[k]:
                    seen[k] = 1; stack.append((i, j))
        while stack:
            i, j = stack.pop()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, j + dj
                if 0 <= a < nu and 0 <= b < nv:
                    k = b * nu + a
                    if not g[k] and not seen[k]:
                        seen[k] = 1; stack.append((a, b))
        return seen

    def components_excluding(self, outside):
        """Connected components of empty cells, CLIPPED at `outside`.

        Not "discard any component that touches the outside": a French window
        opens onto the module's own bottom edge, so its component always
        touches the raster's padding ring, and discarding on contact threw
        away every one of them (SM_MBuilding03_Facade_B_Upper). The flood has
        already stopped at the reveal behind that edge, so the padding ring is
        the only outside cell the component can reach -- clip there."""
        nu, nv, g = self.nu, self.nv, self.g
        seen = bytearray(nu * nv)
        comps = []
        for j0 in range(nv):
            for i0 in range(nu):
                k0 = j0 * nu + i0
                if g[k0] or seen[k0] or outside[k0]:
                    continue
                seen[k0] = 1
                cells = [(i0, j0)]; stack = [(i0, j0)]
                while stack:
                    i, j = stack.pop()
                    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        a, b = i + di, j + dj
                        if 0 <= a < nu and 0 <= b < nv:
                            k = b * nu + a
                            if not g[k] and not seen[k] and not outside[k]:
                                seen[k] = 1
                                cells.append((a, b)); stack.append((a, b))
                iu = [c[0] for c in cells]; iv = [c[1] for c in cells]
                comps.append((self.u0 + min(iu) * CELL,
                              self.u0 + (max(iu) + 1) * CELL,
                              self.v0 + min(iv) * CELL,
                              self.v0 + (max(iv) + 1) * CELL,
                              len(cells) * CELL * CELL))
        return comps


def _blob_components(polys, u0, u1, v0, v1):
    """Connected components of a set of coplanar polygons: their bboxes."""
    r = Ras(u0 - CELL, u1 + CELL, v0 - CELL, v1 + CELL)
    for p in polys:
        r.fill(p)
    # invert: components of the FILLED cells
    nu, nv, g = r.nu, r.nv, r.g
    seen = bytearray(nu * nv)
    out = []
    for j0 in range(nv):
        for i0 in range(nu):
            k0 = j0 * nu + i0
            if not g[k0] or seen[k0]:
                continue
            seen[k0] = 1
            cells = [(i0, j0)]; stack = [(i0, j0)]
            while stack:
                i, j = stack.pop()
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    a, b = i + di, j + dj
                    if 0 <= a < nu and 0 <= b < nv:
                        k = b * nu + a
                        if g[k] and not seen[k]:
                            seen[k] = 1; cells.append((a, b)); stack.append((a, b))
            iu = [c[0] for c in cells]; iv = [c[1] for c in cells]
            out.append((r.u0 + min(iu) * CELL, r.u0 + (max(iu) + 1) * CELL,
                        r.v0 + min(iv) * CELL, r.v0 + (max(iv) + 1) * CELL,
                        len(cells) * CELL * CELL))
    return out


GLASS_RE = ("glass", "window", "glazing")


def measure(nm, ub, verbose=True):
    from pxr import Usd
    meas = ub.PIECES.get(nm)
    if not meas:
        return None
    url = ub._usd(nm)
    _kit_url, frame, scale = ub._kit(nm)
    dw = (frame == "dw")
    st = Usd.Stage.Open(url)
    if not st:
        print("# CANNOT OPEN", nm)
        return None
    fs = _faces(st, scale)
    if not fs:
        return None
    sx, sy, sz, xmin, ymin, zmin = meas
    o_face = (-ymin + 0.02) if not dw else (abs(xmin) + 0.02)
    tri = []
    for p, mt in fs:
        uv = _uvo(p, dw, ymin if dw else 0.0)
        n = _newell(p)
        no = (-n[1]) if not dw else n[0]
        tri.append((uv, no, mt))
    u0 = min(q[0] for uv, _n, _m in tri for q in uv) - CELL
    u1 = max(q[0] for uv, _n, _m in tri for q in uv) + CELL
    v0 = min(q[1] for uv, _n, _m in tri for q in uv) - CELL
    v1 = max(q[1] for uv, _n, _m in tri for q in uv) + CELL
    if (u1 - u0) * (v1 - v0) / (CELL * CELL) > 4e6:
        print("# TOO BIG", nm)
        return None
    planes = {}
    for uv, no, _m in tri:
        if no <= 0:
            continue
        key = round(sum(q[2] for q in uv) / len(uv), 2)
        planes[key] = planes.get(key, 0.0) + _area_uv(uv)
    if not planes:
        return None
    wall_o = max(planes, key=lambda k: planes[k])

    # --- pass A: holes -----------------------------------------------------
    solid = Ras(u0, u1, v0, v1)
    anygeo = Ras(u0, u1, v0, v1)
    for uv, _no, _m in tri:
        flat = [(q[0], q[1]) for q in uv]
        anygeo.fill(flat)
        # HALF A CENTIMETRE, not two. `SM_MBuilding03_Facade_B_Upper`'s first
        # reveal step is 2 cm behind the wall plane, so a 2 cm tolerance made
        # the reveal itself a blocker and filled the very hole it lines.
        if max(q[2] for q in uv) >= wall_o - 0.005:
            solid.fill(flat)
    # THE FLOOD IS BLOCKED BY GEOMETRY AT ANY DEPTH. A French window or a
    # shopfront that runs to the bottom edge of its own module
    # (SM_MBuilding03_Facade_B_Upper, u 0.79..3.21, v 0.00..2.61) opens onto
    # the raster border, so a flood seeded there walks straight into it and
    # the opening is never found. The reveal behind it, though, does cover
    # those cells -- so flooding a `solid | anygeo` raster stops at the module
    # edge, while a region that is genuinely outside the silhouette (above a
    # cornice, beside a corner leg) has no geometry at any depth and floods.
    flood = Ras(u0, u1, v0, v1)
    for k in range(len(flood.g)):
        flood.g[k] = solid.g[k] | anygeo.g[k]
    outside = flood.reachable_from_border()
    comps = solid.components_excluding(outside)
    if os.environ.get("G_DEBUG"):
        print("#   DEBUG wall_o", wall_o, "raw comps", len(comps))
        for c in comps:
            print("#     comp u {0:.2f}..{1:.2f} v {2:.2f}..{3:.2f} a {4:.2f}".format(*c))
    rows = []
    for (ha, hb, hva, hvb, area) in comps:
        if area < MIN_AREA or (hb - ha) < MIN_SIDE or (hvb - hva) < MIN_SIDE:
            continue
        inside = []
        mats = set()
        for uv, no, mt in tri:
            cu = sum(q[0] for q in uv) / len(uv)
            cv = sum(q[1] for q in uv) / len(uv)
            if not (ha - 0.02 <= cu <= hb + 0.02 and hva - 0.02 <= cv <= hvb + 0.02):
                continue
            inside.append((uv, no, mt))
            if mt:
                mats.add(mt)
        if not inside:
            continue
        # recessed outward-facing planes, by area
        pl = {}
        for uv, no, _m in inside:
            if no <= 0:
                continue
            co = sum(q[2] for q in uv) / len(uv)
            if co > wall_o - 0.02:
                continue
            pl.setdefault(round(co, 2), []).append(uv)
        hole_a = (hb - ha) * (hvb - hva)
        if os.environ.get("G_DEBUG"):
            print("#   PLANES for hole u {0:.2f}..{1:.2f} v {2:.2f}..{3:.2f} "
                  "(a {4:.2f}):".format(ha, hb, hva, hvb, hole_a))
            for o_ in sorted(pl, reverse=True):
                aa = sum(_area_uv(q) for q in pl[o_])
                pu = [q[0] for q in pl[o_] for q in q]
                pv = [q[1] for q in pl[o_] for q in q]
                print("#     o {0:7.3f}  area {1:6.2f} ({2:4.0f}%)  u {3:5.2f}"
                      "..{4:5.2f} v {5:5.2f}..{6:5.2f}".format(
                          o_, aa, 100 * aa / hole_a, min(pu), max(pu),
                          min(pv), max(pv)))
        # THE GLAZING CLUSTER, not the first reveal step. A kit window is a
        # STEPPED reveal: fam01 Facade_A has rings at y = 0.10 and 0.15 and
        # then its two lights at y = 0.20 (upper) and y = 0.25 (lower). Taking
        # the shallowest plane over a fixed area share picks the outer ring and
        # the dark quad ends up 8 cm behind the wall, flat, with the reveal
        # hidden; taking the deepest leaves the upper light still showing its
        # painted glass. So: cluster every significant plane within
        # LIGHT_GAP of the DEEPEST one -- those are the lights -- and put the
        # quad 2 cm in front of the SHALLOWEST member of that cluster.
        sig = [(o, ps) for o, ps in pl.items()
               if sum(_area_uv(p) for p in ps) >= LIGHT_FRAC * hole_a]
        big = []
        if sig:
            o_deep = min(o for o, _ps in sig)
            big = [(o, ps) for o, ps in sig if o <= o_deep + LIGHT_GAP]
        if big:
            o_glass = max(o for o, _ps in big)      # shallowest light of the cluster
            gu0 = min(q[0] for o, ps in big for p in ps for q in p)
            gu1 = max(q[0] for o, ps in big for p in ps for q in p)
            gv0 = min(q[1] for o, ps in big for p in ps for q in p)
            gv1 = max(q[1] for o, ps in big for p in ps for q in p)
        else:
            o_glass = min(sum(q[2] for q in uv) / len(uv) for uv, _n, _m in inside)
            gu0, gu1, gv0, gv1 = ha, hb, hva, hvb
        gu0 = max(gu0, ha); gu1 = min(gu1, hb)
        gv0 = max(gv0, hva); gv1 = min(gv1, hvb)
        if gu1 - gu0 < MIN_SIDE or gv1 - gv0 < MIN_SIDE:
            gu0, gu1, gv0, gv1 = ha, hb, hva, hvb
        rows.append(dict(
            src="hole",
            u0=round(gu0, 3), u1=round(gu1, 3),
            v0=round(gv0, 3), v1=round(gv1, 3),
            out=round(o_glass + 0.02 - o_face, 3),
            hu0=round(ha, 3), hu1=round(hb, 3),
            hv0=round(hva, 3), hv1=round(hvb, 3),
            deep=round(min(sum(q[2] for q in uv) / len(uv)
                           for uv, _n, _m in inside) - o_face, 3),
            fill=round(area / max(1e-6, hole_a), 3),
            mats=sorted(mats)[:4]))

    # --- pass B: glass-material subsets ------------------------------------
    gp = {}
    for uv, no, mt in tri:
        if not mt or not any(k in mt.lower() for k in GLASS_RE):
            continue
        if no <= 0:
            continue
        gp.setdefault(round(sum(q[2] for q in uv) / len(uv), 2), []).append(
            [(q[0], q[1]) for q in uv])
    for o, polys in sorted(gp.items()):
        pu0 = min(q[0] for p in polys for q in p); pu1 = max(q[0] for p in polys for q in p)
        pv0 = min(q[1] for p in polys for q in p); pv1 = max(q[1] for p in polys for q in p)
        for (a, b, c, d, ar) in _blob_components(polys, pu0, pu1, pv0, pv1):
            if ar < MIN_AREA or (b - a) < MIN_SIDE or (d - c) < MIN_SIDE:
                continue
            rows.append(dict(
                src="glass", u0=round(a, 3), u1=round(b, 3),
                v0=round(c, 3), v1=round(d, 3),
                out=round(o + 0.02 - o_face, 3),
                hu0=round(a, 3), hu1=round(b, 3),
                hv0=round(c, 3), hv1=round(d, 3),
                deep=round(o - o_face, 3), fill=round(ar / max(1e-6, (b - a) * (d - c)), 3),
                mats=["<glass subset>"]))
    # a glass rect that sits inside a hole rect replaces it: it is the real
    # pane, the hole is only the reveal
    keep = []
    for r in rows:
        if r["src"] != "hole":
            keep.append(r); continue
        inner = [q for q in rows if q["src"] == "glass"
                 and q["u0"] >= r["hu0"] - 0.05 and q["u1"] <= r["hu1"] + 0.05
                 and q["v0"] >= r["hv0"] - 0.05 and q["v1"] <= r["hv1"] + 0.05]
        if inner:
            for q in inner:
                q["hu0"], q["hu1"] = r["hu0"], r["hu1"]
                q["hv0"], q["hv1"] = r["hv0"], r["hv1"]
            continue
        keep.append(r)
    rows = sorted(keep, key=lambda r: (round(r["v0"], 2), round(r["u0"], 2)))
    if verbose:
        print("# {0:<42s} wall_o {1:6.2f}  o_face {2:6.2f}  faces {3:5d}  "
              "openings {4}".format(nm, wall_o, o_face, len(fs), len(rows)))
        for r in rows:
            print("#   {0:5s} hole u {1:6.2f}..{2:6.2f} v {3:6.2f}..{4:6.2f} | "
                  "glass u {5:6.2f}..{6:6.2f} v {7:6.2f}..{8:6.2f} out {9:7.3f} "
                  "deep {10:7.3f} fill {11:5.0f}%  {12}".format(
                      r["src"], r["hu0"], r["hu1"], r["hv0"], r["hv1"],
                      r["u0"], r["u1"], r["v0"], r["v1"], r["out"], r["deep"],
                      100.0 * r["fill"], r["mats"]))
    return rows


def collect_names():
    import random
    from detail import urban_building as ub
    out = {}
    for stl in ub.STYLES:
        try:
            pls = ub.build_building(stl, 0, 0, 0, rng=random.Random(4))
        except Exception:
            continue
        for p in pls:
            n = str(p.get("usd", "")).rsplit("/", 1)[-1].rsplit(".", 1)[0]
            out.setdefault(n, set()).add(stl)
    return out


def main():
    from detail import urban_building as ub
    if NAMES:
        names = {n: [] for n in NAMES}
    elif ALL_PIECES:
        names = {n: [] for n in ub.PIECES}
    else:
        names = {n: sorted(s) for n, s in collect_names().items()}
    allrows = {}
    for nm in sorted(names):
        try:
            rows = measure(nm, ub)
        except Exception as exc:
            print("# FAILED", nm, type(exc).__name__, exc)
            continue
        if rows:
            allrows[nm] = rows
    print()
    print("_G2_WIN_FACES = {")
    for nm in sorted(allrows):
        cells = ["({0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8})".format(
            r["u0"], r["u1"], r["v0"], r["v1"], r["out"],
            r["hu0"], r["hu1"], r["hv0"], r["hv1"]) for r in allrows[nm]]
        print('    "{0}": [{1}],'.format(nm, ", ".join(cells)))
    print("}")
    if JSON_OUT:
        with open(JSON_OUT, "w") as f:
            json.dump(allrows, f, indent=1, sort_keys=True)
        print("# wrote", JSON_OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
