#!/usr/bin/env python3
"""edit_usds — pose SEVERAL archetypes in one window, then write each back.

    # lay a session out and open it
    tools/edit_usds.py assets/archetypes/earthquake/*_soft_storey.usd

    # ...edit, Ctrl+S, close the window. The split runs on exit.
    # If anything goes wrong, or you want to re-run it:
    tools/edit_usds.py --split --session <path printed at startup>

WHY THIS IS TWO LAYERS AND NOT ONE
----------------------------------
`edit_usd.py` opens ONE file as the root layer, so Ctrl+S writes straight back
to it. That cannot be done for several files at once — a stage has exactly one
root layer, and the whole point here is to see a dozen wrecks side by side.

So each source is REFERENCED onto its own prim in a session stage. USD does not
author across a reference arc, which is exactly what makes this safe: every
edit lands as an `over` in the session layer, the sources are untouched while
you work, and the split is an explicit, reviewable step rather than a dozen
files being rewritten under you.

    /World/<slug>_at        <- the tool's grid offset. NOT an edit; never
        /<slug>                copied back. Edit this prim or below.
            ...                <- reference to <slug>.usd (its defaultPrim)

WHAT THE SPLIT COPIES BACK, AND WHAT IT REFUSES TO
--------------------------------------------------
TRANSFORMS AND VISIBILITY ONLY (`xformOp:*`, `xformOpOrder`, `visibility`).

That is the deliberate part. The obvious implementation is `Sdf.CopySpec` of
each edited subtree onto the source, and it is a data-loss bug: `CopySpec`
REPLACES the destination spec, so copying an `over` that holds one rotate op
onto a `def Mesh` that holds points, faces, UVs and material bindings replaces
the mesh with the rotate op. The building would vanish and the file would still
open cleanly.

So the split transfers named fields it understands and REPORTS every other
authored field it saw and skipped, rather than guessing. Posing a wreck — move,
rotate, hide a piece — is fully covered; modelling new geometry is not, and the
tool tells you when you have done that instead of quietly dropping it.
"""

from __future__ import annotations

import argparse
import datetime
import glob
import json
import math
import os
import shutil
import sys

_SCENE_GEN = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SCENE_GEN not in sys.path:
    sys.path.insert(0, _SCENE_GEN)

#: Fields the split understands. Everything else is reported, not copied.
_XFORM_PREFIX = "xformOp:"
_KEEP = ("xformOpOrder", "visibility")

SIDECAR_SUFFIX = ".map.json"


def _parse(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("usds", nargs="*", help="USD files, or directories of them")
    p.add_argument("--session", default="",
                   help="session stage path (default: alongside the first "
                        "input, as _edit_session.usda)")
    p.add_argument("--spacing", type=float, default=0.0,
                   help="metres between items (0 = 1.6x the widest)")
    p.add_argument("--no-explode", action="store_true",
                   help="leave merged meshes as one prim; by default every "
                        "mesh is split into its disconnected pieces so they "
                        "can be posed individually, and rejoined on save")
    p.add_argument("--no-backup", action="store_true",
                   help="skip the <name>.orig.usd snapshots")
    p.add_argument("--split", action="store_true",
                   help="skip the window; just write an existing session back. "
                        "Run this one under a python that HAS pxr "
                        "(AirStack/.venv), not Kit's python.sh, which only "
                        "gains pxr once SimulationApp has started.")
    p.add_argument("--headless", action="store_true")
    return p.parse_args(argv)


def _expand(paths):
    out = []
    for p in paths:
        if os.path.isdir(p):
            out += sorted(glob.glob(os.path.join(p, "*.usd"))
                          + glob.glob(os.path.join(p, "*.usda")))
        else:
            out += sorted(glob.glob(p)) or [p]
    # A backup is not a subject.
    return [p for p in out if os.path.isfile(p) and ".orig." not in p]


def _slug(path):
    return os.path.splitext(os.path.basename(path))[0]


# ---------------------------------------------------------------------------
# split
# ---------------------------------------------------------------------------

#: Pieces live UNDER the mesh they came from, named ``piece_NNNN``, and the
#: parent is emptied rather than deactivated. Making them children is what
#: makes the transform and the material binding come for free: USD inherits
#: both down the namespace, so a piece is posed in its parent's space and
#: shaded by its parent's material with nothing copied. Authoring them as a
#: sibling scope instead cost two bugs on 2026-08-30 -- pieces at raw authored
#: scale because the mesh's own `AddTransformOp` was left behind, and pieces
#: rendering default white because the binding was -- plus a deactivated prim
#: per exploded mesh cluttering the outliner.
PIECE_PREFIX = "piece_"

#: Marks a mesh whose geometry currently lives in its pieces. Rejoin keys off
#: this rather than off the child names, so a prim that legitimately has mesh
#: children is never mistaken for an exploded one.
EXPLODED_ATTR = "airstack:explodedPieces"

#: Ceiling on pieces per SESSION. A rubble mesh is every fragment of one
#: material merged, so a wrecked tower can hold several hundred; past a few
#: thousand prims the viewport, not the maths, is what stops being usable.
MAX_PARTS = int(os.environ.get("SCENE_EDIT_MAX_PARTS", "4000") or 4000)


def _connected(faces, n_verts):
    """Component id per vertex, for a face array indexing *n_verts* points."""
    import numpy as np
    try:
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
        e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]],
                            faces[:, [2, 0]]])
        g = coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])),
                       shape=(n_verts, n_verts))
        return connected_components(g, directed=False)
    except Exception:                                            # noqa: BLE001
        parent = np.arange(n_verts)

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        for tri in faces:
            a = find(tri[0])
            for v in tri[1:]:
                b = find(v)
                if a != b:
                    parent[b] = a
        roots = np.array([find(i) for i in range(n_verts)])
        uniq, lab = np.unique(roots, return_inverse=True)
        return len(uniq), lab


def _tri_faces(counts, idx):
    """Fan-triangulate, returning (T, 3). Editing only needs the topology."""
    import numpy as np
    counts = np.asarray(counts, dtype=np.int64)
    idx = np.asarray(idx, dtype=np.int64)
    starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
    out = []
    for c, s in zip(counts, starts):
        f = idx[s:s + c]
        for k in range(1, c - 1):
            out.append((f[0], f[k], f[k + 1]))
    return np.asarray(out, dtype=np.int64) if out else np.zeros((0, 3), np.int64)


def explode_session(session_path: str) -> int:
    """`explode_stage` on a session FILE (the CLI path). Saves in place."""
    from pxr import Usd
    stage = Usd.Stage.Open(session_path)
    n = explode_stage(stage)
    stage.GetRootLayer().Save()
    return n


def explode_stage(stage) -> int:
    """Split every multi-piece mesh into one child prim per disconnected piece.

    The baker MERGES fragments by material -- `rubble_<material>` is every
    fragment sharing one material in a single prim -- which is right for load
    time and wrong for posing: two slabs that never touch are one selectable
    thing, so nudging one drags the other. This separates them for the edit and
    `rejoin_session` puts them back, so the saved archetype keeps the merged
    form and its file size.

    Meshes carrying GeomSubsets are left alone: a subset indexes faces of the
    prim it lives on, and splitting the prim invalidates those indices.
    """
    from pxr import Sdf, Usd, UsdGeom, Vt
    import numpy as np

    made = total = skipped_subsets = capped = 0
    for prim in list(stage.Traverse()):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if prim.GetName().startswith(PIECE_PREFIX):
            continue
        if UsdGeom.Subset.GetAllGeomSubsets(UsdGeom.Imageable(prim)):
            skipped_subsets += 1
            continue
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        P = np.asarray(pts, dtype=np.float64)
        F = _tri_faces(counts, idx)
        if not len(F):
            continue
        n, lab = _connected(F, len(P))
        total += 1
        if n < 2:
            continue
        # ALL OR NOTHING per mesh. Authoring some pieces and then emptying the
        # parent would DROP whatever did not fit -- measured 2026-08-30 on
        # `SM_Building_03_pancaked`: 1,511,062 verts in, 1,183,564 out. A mesh
        # over budget stays merged, which is merely less convenient.
        if made + n > MAX_PARTS:
            capped += 1
            continue

        pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
        st = pv.Get() if pv else None
        st_vertex = (st is not None and pv.GetInterpolation()
                     == UsdGeom.Tokens.vertex and len(st) == len(P))
        ST = np.asarray(st, dtype=np.float64) if st_vertex else None

        face_lab = lab[F[:, 0]]
        for c in range(n):
            sel = F[face_lab == c]
            if not len(sel):
                continue
            used = np.unique(sel)
            remap = np.zeros(len(P), dtype=np.int64)
            remap[used] = np.arange(len(used))
            part = UsdGeom.Mesh.Define(
                stage, prim.GetPath().AppendChild(f"{PIECE_PREFIX}{c:04d}"))
            part.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(
                P[used].astype(np.float32)))
            part.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
                np.full(len(sel), 3, dtype=np.int32)))
            part.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
                remap[sel].reshape(-1).astype(np.int32)))
            part.CreateDoubleSidedAttr(True)
            part.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            if ST is not None:
                UsdGeom.PrimvarsAPI(part).CreatePrimvar(
                    "st", Sdf.ValueTypeNames.TexCoord2fArray,
                    UsdGeom.Tokens.vertex).Set(
                        Vt.Vec2fArray.FromNumpy(ST[used].astype(np.float32)))
            made += 1

        # EMPTY THE PARENT, do not deactivate it. Its surfaces now exist in the
        # pieces, so leaving them would double every face; deactivating would
        # take the pieces with it (a deactivated prim prunes its subtree) and
        # fill the outliner with dead entries.
        m.GetPointsAttr().Set(Vt.Vec3fArray())
        m.GetFaceVertexCountsAttr().Set(Vt.IntArray())
        m.GetFaceVertexIndicesAttr().Set(Vt.IntArray())
        prim.CreateAttribute(EXPLODED_ATTR, Sdf.ValueTypeNames.Int,
                             custom=True).Set(int(n))

    print(f"[edit] exploded {made} piece(s) out of {total} mesh(es)"
          + (f"; {skipped_subsets} skipped (GeomSubsets)" if skipped_subsets
             else "")
          + (f"; {capped} left merged to stay under SCENE_EDIT_MAX_PARTS="
             f"{MAX_PARTS}" if capped else ""))
    return made


def rejoin_session(session_path: str) -> dict:
    """`rejoin_stage` on a session FILE (the split path)."""
    from pxr import Usd
    return rejoin_stage(Usd.Stage.Open(session_path))


def flatten_pieces(stage) -> int:
    """Move pieces up beside the mesh they came from, and deactivate it.

    Pieces started as CHILDREN so that transform and material came for free by
    inheritance. Nested under a prim that still exists, though, they read as
    "the original is still there with extra stuff inside it" -- so this lifts
    them to SIBLINGS and switches the original off, which is what the outliner
    should have shown all along.

    Two things have to be carried by hand once they are siblings, and they are
    exactly the two that broke when pieces last lived outside their mesh:

    * THE TRANSFORM is baked into the piece's POINTS rather than re-authored as
      an op. A sibling has no claim on the original's transform, and a piece
      carrying its own copy would then be moved by the user on top of it, so
      `rejoin` would have to unpick which part of the matrix was whose.
    * THE MATERIAL is bound directly on each piece.

    Naming is `<mesh>_piece_NNNN`, so `rejoin_stage` can still find a mesh's
    pieces without a sidecar.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
    import numpy as np

    # PATHS FIRST, prims second. Removing the old pieces as we go expires
    # every prim handle captured after them, so a list of prims taken up front
    # dies partway through -- measured as "Accessed invalid expired 'Mesh'
    # prim" on the first mesh whose pieces were removed.
    marked = [p.GetPath() for p in stage.Traverse()
              if p.GetAttribute(EXPLODED_ATTR)
              and p.GetAttribute(EXPLODED_ATTR).IsValid()]
    moved = 0
    for path in marked:
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            continue
        kids = [c for c in prim.GetChildren()
                if c.GetName().startswith(PIECE_PREFIX)]
        if not kids:
            continue
        T = np.array(Gf.Matrix4d(
            UsdGeom.Xformable(prim).GetLocalTransformation())).reshape(4, 4)
        mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
        dc = UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()
        parent = prim.GetPath().GetParentPath()
        stem = prim.GetName()
        for k, kid in enumerate(kids):
            km = UsdGeom.Mesh(kid)
            pts = km.GetPointsAttr().Get()
            cnt = km.GetFaceVertexCountsAttr().Get()
            idx = km.GetFaceVertexIndicesAttr().Get()
            if not pts or not cnt or not idx:
                continue
            X = np.array(Gf.Matrix4d(
                UsdGeom.Xformable(kid).GetLocalTransformation())).reshape(4, 4)
            v = np.asarray(pts, dtype=np.float64)
            v = (np.c_[v, np.ones(len(v))] @ (X @ T))[:, :3]
            dst = UsdGeom.Mesh.Define(
                stage, parent.AppendChild(f"{stem}_{PIECE_PREFIX}{k:04d}"))
            dst.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype(np.float32)))
            dst.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(
                np.asarray(cnt, dtype=np.int32)))
            dst.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(
                np.asarray(idx, dtype=np.int32)))
            dst.CreateDoubleSidedAttr(True)
            dst.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            pv = UsdGeom.PrimvarsAPI(km).GetPrimvar("st")
            sv = pv.Get() if pv else None
            if sv is not None and len(sv) == len(v):
                UsdGeom.PrimvarsAPI(dst).CreatePrimvar(
                    "st", Sdf.ValueTypeNames.TexCoord2fArray,
                    UsdGeom.Tokens.vertex).Set(sv)
            if mat and mat.GetPrim().IsValid():
                UsdShade.MaterialBindingAPI.Apply(dst.GetPrim()).Bind(mat)
            if dc:
                dst.CreateDisplayColorAttr(dc)
            moved += 1
        for kid in kids:
            stage.RemovePrim(kid.GetPath())
        prim.SetActive(False)
    print(f"[edit] flattened {moved} piece(s) beside their meshes")
    return moved


def rejoin_stage(stage) -> dict:
    """Merge exploded pieces back, in the parent mesh's own space.

    Returns ``{<mesh path>: (points, counts, indices, st)}`` so `split` can
    write geometry rather than only a transform. Each piece contributes its
    points through its OWN local transform -- which is relative to the parent,
    and is where the posing the user did actually lives. The parent's transform
    is deliberately not applied: the source mesh still carries it.
    """
    from pxr import Usd, UsdGeom, Gf
    import numpy as np

    out = {}
    # ALL PRIMS, not just active ones. `flatten_pieces` DEACTIVATES the mesh it
    # lifted the pieces out of, and `Stage.Traverse()` skips inactive prims --
    # so the markers this loop keys on became invisible and rejoin returned
    # nothing at all. Measured 2026-08-30: 1,511,062 verts of pieces on the
    # stage, 0 seen, which on save would have written EMPTY geometry over the
    # sources rather than the user's edit.
    for prim in Usd.PrimRange.Stage(stage, Usd.PrimAllPrimsPredicate):
        a = prim.GetAttribute(EXPLODED_ATTR)
        if not a or not a.IsValid():
            continue
        pieces = [c for c in prim.GetChildren()
                  if c.GetName().startswith(PIECE_PREFIX)]
        sibling_stem = prim.GetName() + "_" + PIECE_PREFIX
        parent_prim = prim.GetParent()
        siblings = ([c for c in parent_prim.GetChildren()
                     if c.GetName().startswith(sibling_stem)]
                    if parent_prim else [])
        if siblings:
            # FLATTENED layout: the pieces sit beside the mesh with its
            # transform BAKED INTO THEIR POINTS, so undo that transform to get
            # back into the source mesh's own space -- the source still carries
            # it, and applying it twice would fling the geometry.
            T = np.array(Gf.Matrix4d(UsdGeom.Xformable(prim)
                                     .GetLocalTransformation())).reshape(4, 4)
            Tin = np.linalg.inv(T)
            P, C, I, S, base = [], [], [], [], 0
            any_st = True
            for piece in siblings:
                pm = UsdGeom.Mesh(piece)
                pts = pm.GetPointsAttr().Get()
                cnt = pm.GetFaceVertexCountsAttr().Get()
                idx = pm.GetFaceVertexIndicesAttr().Get()
                if not pts or not cnt or not idx:
                    continue
                X = np.array(Gf.Matrix4d(UsdGeom.Xformable(piece)
                                         .GetLocalTransformation())).reshape(4, 4)
                v = np.asarray(pts, dtype=np.float64)
                v = (np.c_[v, np.ones(len(v))] @ (X @ Tin))[:, :3]
                pv = UsdGeom.PrimvarsAPI(pm).GetPrimvar("st")
                sv = pv.Get() if pv else None
                if sv is None or len(sv) != len(v):
                    any_st = False
                else:
                    S.append(np.asarray(sv, dtype=np.float64))
                P.append(v)
                C.append(np.asarray(cnt, dtype=np.int64))
                I.append(np.asarray(idx, dtype=np.int64) + base)
                base += len(v)
            if P:
                out[str(prim.GetPath())] = (
                    np.vstack(P), np.concatenate(C), np.concatenate(I),
                    np.vstack(S) if (any_st and S) else None)
            continue

        if not pieces:
            # ALREADY MERGED, in the GUI. The marker stays behind so `split`
            # still writes this prim's geometry: the edit lives in its
            # vertices now, which a pose copy cannot carry.
            m = UsdGeom.Mesh(prim)
            pts = m.GetPointsAttr().Get()
            cnt = m.GetFaceVertexCountsAttr().Get()
            idx = m.GetFaceVertexIndicesAttr().Get()
            if pts and cnt and idx:
                pv = UsdGeom.PrimvarsAPI(m).GetPrimvar("st")
                sv = pv.Get() if pv else None
                out[str(prim.GetPath())] = (
                    np.asarray(pts, dtype=np.float64),
                    np.asarray(cnt, dtype=np.int64),
                    np.asarray(idx, dtype=np.int64),
                    np.asarray(sv, dtype=np.float64)
                    if sv is not None and len(sv) == len(pts) else None)
            continue
        P, C, I, S, base = [], [], [], [], 0
        any_st = True
        for piece in prim.GetChildren():
            if not piece.GetName().startswith(PIECE_PREFIX):
                continue
            pm = UsdGeom.Mesh(piece)
            if not pm:
                continue
            pts = pm.GetPointsAttr().Get()
            cnt = pm.GetFaceVertexCountsAttr().Get()
            idx = pm.GetFaceVertexIndicesAttr().Get()
            if not pts or not cnt or not idx:
                continue
            v = np.asarray(pts, dtype=np.float64)
            xf = UsdGeom.Xformable(piece).GetLocalTransformation()
            M = np.array(Gf.Matrix4d(xf)).reshape(4, 4)
            v = (np.c_[v, np.ones(len(v))] @ M)[:, :3]
            pv = UsdGeom.PrimvarsAPI(pm).GetPrimvar("st")
            sv = pv.Get() if pv else None
            if sv is None or len(sv) != len(v):
                any_st = False
            else:
                S.append(np.asarray(sv, dtype=np.float64))
            P.append(v)
            C.append(np.asarray(cnt, dtype=np.int64))
            I.append(np.asarray(idx, dtype=np.int64) + base)
            base += len(v)
        if not P:
            continue
        out[str(prim.GetPath())] = (
            np.vstack(P), np.concatenate(C), np.concatenate(I),
            np.vstack(S) if (any_st and S) else None)
    if out:
        print(f"[split] rejoining {len(out)} exploded mesh(es)")
    return out


def _set_attr(tspec, name, typename, value):
    """Author *value* on *tspec*, creating the attribute spec if needed."""
    from pxr import Sdf
    a = tspec.attributes.get(name)
    if a is None:
        a = Sdf.AttributeSpec(tspec, name, typename)
    a.default = value


def _save_layer(dst, path: str) -> bool:
    """Save *dst* over *path*, and actually CHECK that it happened.

    `Sdf.Layer.Save()` returns False on failure, it does not raise, so an
    unchecked call reports success for a write that never landed -- the worst
    failure mode for an editing tool, because the user believes their posing is
    saved and it is gone.

    The export-and-move fallback covers a target this process cannot open in
    place: a rung the baker just wrote is owned by root inside the container,
    while a host-side split runs as the user. The library DIRECTORY is ours
    either way, so writing beside it and moving over succeeds where an in-place
    save cannot. (Measured 2026-08-30: `substitute_rung.py` hit exactly this as
    a PermissionError. The six-file session that prompted this check did NOT --
    see `split` for what was actually wrong there.)
    """
    try:
        if dst.Save():
            return True
    except Exception:                                            # noqa: BLE001
        pass
    tmp = path + ".split.tmp.usd"
    try:
        if not dst.Export(tmp):
            return False
        os.unlink(path)
        shutil.move(tmp, path)
        return True
    except OSError:
        if os.path.exists(tmp):
            try:
                os.unlink(tmp)
            except OSError:
                pass
        return False


def _gizmo_at_mesh_centre():
    """Put the move/rotate/scale gizmo on each object's own geometry.

    Kit defaults this to "Authored Pivot", and a baked fragment's authored
    pivot sits at the object origin -- which for a wreck recentred on the
    world is nowhere near the piece you clicked. The gizmo then appears
    detached from the mesh, and dragging it feels like moving something else.

    "Bounding Box Center" places it at the centre of the selected object's own
    bounds instead. Set explicitly on every launch rather than trusting the
    persisted value, so an edit session behaves the same on any machine.
    """
    try:
        import carb
        carb.settings.get_settings().set(
            "/persistent/exts/omni.kit.manipulator.prim.core"
            "/manipulator/placement", "Bounding Box Center")
    except Exception as exc:                                     # noqa: BLE001
        print(f"[edit] could not set gizmo placement: {exc}")


def split(session_path: str) -> int:
    """Write each wrapper's pose edits back into its source USD."""
    from pxr import Sdf, Vt

    side = session_path + SIDECAR_SUFFIX
    if not os.path.isfile(side):
        print(f"[split] no sidecar at {side}", file=sys.stderr)
        return 2
    with open(side) as fh:
        entries = json.load(fh)["items"]
    sess = Sdf.Layer.FindOrOpen(session_path)
    if sess is None:
        print(f"[split] cannot open {session_path}", file=sys.stderr)
        return 2

    here = os.path.dirname(os.path.abspath(session_path))
    total_written = 0
    touched = []
    merged = rejoin_session(session_path)
    for e in entries:
        # Tolerate both: sessions written before the paths became relative
        # still carry an absolute one.
        src = e["source"]
        src = src if os.path.isabs(src) else os.path.join(here, src)
        dst = Sdf.Layer.FindOrOpen(src)
        if dst is None:
            print(f"[split] SKIP (cannot open) {src}")
            continue
        wrapper = Sdf.Path(e["wrapper"])
        base = Sdf.Path(e["default_prim"])
        moved, skipped = [], []

        def walk(spec_path: Sdf.Path):
            spec = sess.GetPrimAtPath(spec_path)
            if spec is None:
                return
            # The pieces are not prims the source has; their geometry goes
            # back through `merged` below, not through the pose walk, which
            # would otherwise report every piece as "no such prim".
            if spec_path.name.startswith(PIECE_PREFIX):
                return
            # THE WRAPPER ITSELF IS AN EDITABLE PRIM, and it is the one a
            # user reaches for first: click the building, move it. Its
            # transform maps onto the source's defaultPrim. Skipping it (the
            # first version did) made "select and drag" silently do nothing,
            # which is the worst possible failure for an editing tool.
            # `/World/<slug>_at` above it carries the grid offset and is never
            # walked, so the offset can never leak into a source.
            if True:
                if spec_path == wrapper:
                    rel, target = Sdf.Path("."), base
                else:
                    rel = spec_path.MakeRelativePath(wrapper)
                    target = base.AppendPath(rel)
                fields = {}
                for attr in spec.attributes:
                    n = attr.name
                    if n.startswith(_XFORM_PREFIX) or n in _KEEP:
                        fields[n] = attr.default
                    else:
                        skipped.append(f"{rel}.{n}")
                if fields:
                    tspec = dst.GetPrimAtPath(target)
                    if tspec is None:
                        skipped.append(f"{rel} (no such prim in the source)")
                    else:
                        for n, v in fields.items():
                            a = tspec.attributes.get(n)
                            if a is None:
                                a = Sdf.AttributeSpec(
                                    tspec, n,
                                    Sdf.ValueTypeNames.Find(
                                        str(spec.attributes[n].typeName)))
                            a.default = v
                        moved.append(str(rel))
            for child in spec.nameChildren:
                walk(spec_path.AppendChild(child.name))

        walk(wrapper)

        # GEOMETRY for anything that was exploded on open. A pose copy cannot
        # express this: the user moved PIECES that the source holds as one
        # merged mesh, so the edit only exists as vertices.
        for sess_path, (P, C, I, S) in merged.items():
            sp = Sdf.Path(sess_path)
            if not sp.HasPrefix(wrapper):
                continue
            rel = (Sdf.Path(".") if sp == wrapper
                   else sp.MakeRelativePath(wrapper))
            target = base if sp == wrapper else base.AppendPath(rel)
            tspec = dst.GetPrimAtPath(target)
            if tspec is None:
                skipped.append(f"{rel} (no such prim in the source)")
                continue
            _set_attr(tspec, "points", Sdf.ValueTypeNames.Point3fArray,
                      Vt.Vec3fArray.FromNumpy(P.astype("float32")))
            _set_attr(tspec, "faceVertexCounts", Sdf.ValueTypeNames.IntArray,
                      Vt.IntArray.FromNumpy(C.astype("int32")))
            _set_attr(tspec, "faceVertexIndices", Sdf.ValueTypeNames.IntArray,
                      Vt.IntArray.FromNumpy(I.astype("int32")))
            if S is not None:
                _set_attr(tspec, "primvars:st",
                          Sdf.ValueTypeNames.TexCoord2fArray,
                          Vt.Vec2fArray.FromNumpy(S.astype("float32")))
            moved.append(f"{rel} (geometry)")

        if moved and not dst.dirty:
            # AUTHORED BUT UNCHANGED. Kit writes an `xformOp:transform` onto
            # every prim the user so much as selects, so `moved` being
            # non-empty does NOT mean anything was posed -- the value can
            # equal what the source already held, leaving the layer clean and
            # `Save()` a no-op that correctly returns True. Reporting "wrote N
            # prims" here is how this tool claimed 3 files and changed 1.
            print(f"[split] {os.path.basename(src)}: pose identical to "
                  f"source, nothing to write")
            continue
        if moved:
            if not _save_layer(dst, src):
                print(f"[split] {os.path.basename(src)}: FAILED TO WRITE "
                      f"({len(moved)} prim(s) lost)")
                continue
            total_written += 1
            touched.append(src)
            print(f"[split] {os.path.basename(src)}: wrote {len(moved)} "
                  f"prim(s) -> {', '.join(moved[:4])}"
                  f"{' ...' if len(moved) > 4 else ''}")
        else:
            print(f"[split] {os.path.basename(src)}: no pose edits")
        if skipped:
            uniq = sorted(set(skipped))
            print(f"[split]   NOT copied ({len(uniq)}): {', '.join(uniq[:6])}"
                  f"{' ...' if len(uniq) > 6 else ''}")

    print(f"\n[split] {total_written} file(s) updated")
    # Provenance, same as edit_usd.
    try:
        from tools.edit_usd import _mark_hand_edited
        from archetypes import version as V
        fp = V.source_fingerprint()
        # THE RESOLVED PATHS, not `e["source"]` — that is relative to the
        # session now, so stat-ing it against the cwd silently marked nothing.
        for src in touched:
            if os.path.isfile(src):
                _mark_hand_edited(src, fp)
    except Exception as exc:                                     # noqa: BLE001
        print(f"[split] could not mark provenance: {exc}")
    return 0


# ---------------------------------------------------------------------------
# build the session
# ---------------------------------------------------------------------------

def build_session(files, session_path, spacing):
    """Reference every source onto a grid and write the session + sidecar."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux

    stage = Usd.Stage.CreateNew(session_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

    # Measure first so the grid pitch fits the widest subject.
    widths = []
    for f in files:
        s = Usd.Stage.Open(f)
        c = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_,
                                  UsdGeom.Tokens.render])
        r = c.ComputeWorldBound(s.GetPseudoRoot()).ComputeAlignedRange()
        widths.append(0.0 if r.IsEmpty()
                      else max(r.GetMax()[0] - r.GetMin()[0],
                               r.GetMax()[1] - r.GetMin()[1]))
    pitch = spacing or (1.6 * max(widths + [20.0]))
    cols = max(1, int(math.ceil(math.sqrt(len(files)))))

    items = []
    for i, f in enumerate(files):
        slug = _slug(f)
        src = Usd.Stage.Open(f)
        dp = src.GetDefaultPrim()
        if not dp:
            print(f"[edit] SKIP {slug}: no defaultPrim to reference")
            continue
        at = UsdGeom.Xform.Define(
            stage, Sdf.Path(f"/World/{slug}_at"))
        at.AddTranslateOp().Set(Gf.Vec3d((i % cols) * pitch,
                                         (i // cols) * pitch, 0.0))
        holder = UsdGeom.Xform.Define(
            stage, Sdf.Path(f"/World/{slug}_at/{slug}"))
        holder.GetPrim().GetReferences().AddReference(os.path.abspath(f))
        # RELATIVE TO THE SESSION, not absolute. The repo is bind-mounted at
        # `/isaac-sim/AirStack` in the container and `~/coasei/AirStack` on the
        # host, so an absolute path written by one side cannot be opened by the
        # other — the split ran on the host against a session built in the
        # container and skipped every file as "cannot open". The manifest's
        # `usd` field is relative for exactly this reason.
        items.append({"source": os.path.relpath(os.path.abspath(f),
                                                os.path.dirname(
                                                    os.path.abspath(
                                                        session_path))),
                      "wrapper": str(holder.GetPath()),
                      "default_prim": str(dp.GetPath()),
                      "grid": [(i % cols) * pitch, (i // cols) * pitch]})

    # Review lighting lives in the SESSION stage, which is never split back —
    # only the per-item wrappers are read, so this cannot reach a source.
    rig = Sdf.Path("/World/_review")
    UsdGeom.Scope.Define(stage, rig)
    UsdLux.DomeLight.Define(
        stage, rig.AppendChild("dome")).CreateIntensityAttr(900.0)
    key = UsdLux.DistantLight.Define(stage, rig.AppendChild("key"))
    key.CreateIntensityAttr(2200.0)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(
        Gf.Vec3f(-45.0, 0.0, 30.0))

    stage.GetRootLayer().Save()
    with open(session_path + SIDECAR_SUFFIX, "w") as fh:
        json.dump({"created": datetime.datetime.now().astimezone().isoformat(
            timespec="seconds"), "items": items}, fh, indent=2)
    return items, pitch


def main(argv=None) -> int:
    args = _parse(argv)

    if args.split:
        if not args.session:
            print("[edit] --split needs --session", file=sys.stderr)
            return 2
        return split(args.session)

    files = _expand(args.usds)
    if not files:
        print("[edit] nothing to open", file=sys.stderr)
        return 2
    session = args.session or os.path.join(
        os.path.dirname(os.path.abspath(files[0])), "_edit_session.usda")

    # SIMULATIONAPP FIRST. `build_session` needs `pxr`, and under Kit's
    # `python.sh` there is no `pxr` until the app has built its environment —
    # the mirror image of the hazard `bake_cli.py` documents (there, importing
    # `pxr` too EARLY makes Kit segfault; here, using it too early is a bare
    # ModuleNotFoundError). Either way the app bracket is what decides.
    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": args.headless})
    _gizmo_at_mesh_centre()
    import omni.kit.app
    import omni.usd

    # BACK EVERY SOURCE UP FIRST. `edit_usd.py` has always done this and this
    # one did not — an asymmetry worth closing, because the split writes to
    # several files at once and "undo" for a pose you have already saved is
    # otherwise a re-bake of that cell.
    if not args.no_backup:
        import shutil
        for f in files:
            orig = os.path.splitext(f)[0] + ".orig" + os.path.splitext(f)[1]
            if not os.path.exists(orig):
                shutil.copy2(f, orig)
                print(f"[edit] backup -> {os.path.basename(orig)}")

    items, pitch = build_session(files, session, args.spacing)
    print(f"[edit] session {session}  ({len(items)} items, pitch {pitch:.0f} m)")
    if not args.no_explode:
        explode_session(session)

    omni.usd.get_context().open_stage(session)
    for _ in range(60):
        omni.kit.app.get_app().update()

    print("\n" + "=" * 72)
    print(f"EDITING {len(items)} archetype(s) — session {session}")
    print("  edit    : move/rotate/hide prims INSIDE each <slug> wrapper")
    print("  save    : Ctrl+S  (writes the SESSION, not the sources)")
    print("  exit    : close the window; the split runs automatically")
    print("  redo    : tools/edit_usds.py --split --session " + session)
    print("=" * 72 + "\n")
    sys.stdout.flush()

    while app.is_running():
        omni.kit.app.get_app().update()

    # SPLIT BEFORE CLOSING KIT, and exit without falling through.
    #
    # `SimulationApp.close()` HARD-EXITS the process — the app is launched with
    # `/app/fastShutdown`, so nothing after it runs. `bake_cli.py` documents
    # the same trap for its exit code; here it silently skipped the entire
    # write-back, so a session of edits looked saved, reported nothing, and
    # left every source file untouched. Measured 2026-08-30 on `block_01` and
    # `federal_bureau`: both still carried their original bake mtimes.
    print("\n[edit] window closed — splitting back\n")
    sys.stdout.flush()
    code = split(session)
    sys.stdout.flush()
    os._exit(code)


if __name__ == "__main__":
    sys.exit(main())
