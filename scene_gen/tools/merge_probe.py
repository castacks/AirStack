#!/usr/bin/env python3
"""merge_probe — does WELDING an asset's separate meshes let VTK cut it?

    /isaac-sim/python.sh tools/merge_probe.py <pristine.usd> [--weld-tol M]

`Reference_Brownstone6Row` hangs BOTH cutters on its `cracked` rung -- 66 min
on VTK, 20+ on numpy -- while far larger single-mesh assets cut in ~2 min. The
one structural difference: it ships 90 separate mesh prims, so the triangle
soup handed to the cutter is 90 DISCONNECTED shells rather than one solid.

`_vtk_prepare` welds exactly-coincident points, which does not join shells that
merely touch. This probe welds on a TOLERANCE first, reports how many connected
components that leaves, and then times the clip -- so "merge the meshes" can be
answered with a number instead of a guess.

Run it under `timeout`: a spin inside native VTK ignores SIGINT and SIGTERM, so
the only way to end one is from outside the process.
"""
from __future__ import annotations
import argparse, os, sys, time
import numpy as np

_SG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SG not in sys.path:
    sys.path.insert(0, _SG)


def _components(verts, faces):
    try:
        from scipy.sparse import coo_matrix
        from scipy.sparse.csgraph import connected_components
    except Exception:
        return -1
    n = len(verts)
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    g = coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n))
    return connected_components(g, directed=False)[0]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("usd")
    ap.add_argument("--weld-tol", type=float, default=0.0,
                    help="metres; 0 leaves the soup unwelded (the baseline)")
    ap.add_argument("--mech", default="crack")
    ap.add_argument("--backend", default="vtk")
    ap.add_argument("--max-cells", type=int, default=800)
    ap.add_argument("--to-stage", action="store_true",
                    help="time `fracture_to_stage` -- the cut PLUS authoring "
                         "every fragment prim and its physics. `_fracture_soup` "
                         "alone is pure numpy and cleared this asset in 6.1s, "
                         "so the stage/PhysX half is what is left untested")
    ap.add_argument("--gates", action="store_true",
                    help="time the PRE-CUT gates (survey + interior_fill), "
                         "which is what is left once thicken and the fracture "
                         "are both cleared")
    ap.add_argument("--thicken", type=float, default=0.0,
                    help="metres; time `solidify_prims` (the THICKEN step that "
                         "runs BEFORE the cut) instead of the fracture")
    a = ap.parse_args(argv)

    os.environ["SCENE_FRACTURE_BACKEND"] = a.backend
    from isaacsim import SimulationApp
    app = SimulationApp(launch_config={"headless": True})
    try:
        from pxr import Usd, UsdGeom, Gf
        import disaster.mesh_damage as md
        from disaster import quake as Q

        stage = Usd.Stage.Open(a.usd)
        cache = UsdGeom.XformCache()
        V, F, off, nprim = [], [], 0, 0
        for p in stage.Traverse():
            if not p.IsA(UsdGeom.Mesh):
                continue
            m = UsdGeom.Mesh(p)
            pts, cnt, idx = (m.GetPointsAttr().Get(),
                            m.GetFaceVertexCountsAttr().Get(),
                            m.GetFaceVertexIndicesAttr().Get())
            if not pts or not cnt or not idx:
                continue
            nprim += 1
            P = np.asarray(pts, dtype=np.float64)
            M = np.array(Gf.Matrix4d(cache.GetLocalToWorldTransform(p))).reshape(4, 4)
            P = (np.c_[P, np.ones(len(P))] @ M)[:, :3]
            cnt = np.asarray(cnt); idx = np.asarray(idx)
            st = np.concatenate([[0], np.cumsum(cnt)[:-1]])
            for c, s0 in zip(cnt, st):
                f = idx[s0:s0 + c]
                for k in range(1, c - 1):
                    F.append([f[0] + off, f[k] + off, f[k + 1] + off])
            V.append(P); off += len(P)
        V = np.vstack(V); F = np.asarray(F, dtype=np.int64)
        print(f"[probe] {nprim} mesh prim(s)  {len(V):,} verts  {len(F):,} tris",
              flush=True)

        if a.weld_tol > 0:
            q = np.round(V / a.weld_tol).astype(np.int64)
            _, first, inv = np.unique(q, axis=0, return_index=True,
                                      return_inverse=True)
            V2, F2 = V[first], inv[F]
            F2 = F2[(F2[:, 0] != F2[:, 1]) & (F2[:, 1] != F2[:, 2])
                    & (F2[:, 0] != F2[:, 2])]
            print(f"[probe] welded @ {a.weld_tol} m -> {len(V2):,} verts "
                  f"{len(F2):,} tris", flush=True)
            V, F = V2, F2
        print(f"[probe] connected components: {_components(V, F)}", flush=True)

        if a.to_stage:
            root = stage.GetDefaultPrim()
            prims = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
            b = md.bounds_of(prims)
            mech = Q.MECHANISMS[a.mech]
            fail = md.Failure(a.mech, lambda p, m=mech: np.full(
                len(np.atleast_2d(p)), float(m.intensity)))
            t0 = time.time()
            cut = md.fracture_to_stage(stage, root, b, fail, seed=7)
            print(f"[probe] TO_STAGE DONE {time.time() - t0:.1f}s  "
                  f"paths={len(cut.get('paths', []))} cells={cut.get('cells')}",
                  flush=True)
            return 0

        if a.gates:
            prims = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
            b = md.bounds_of(prims)
            t0 = time.time()
            fill = md.interior_fill(prims, b, 3.5, 3.0)
            print(f"[probe] interior_fill {time.time() - t0:.1f}s -> {fill}",
                  flush=True)
            from disaster import survey as SV
            t0 = time.time()
            cl = SV.classify_mesh(V, F)
            print(f"[probe] classify_mesh {time.time() - t0:.1f}s -> {cl}",
                  flush=True)
            return 0

        if a.thicken > 0:
            # THE STEP BEFORE THE CUT. The fracture probe cleared this asset in
            # 6.1s, so a bake that sits for 20-66 min on it is not stuck in the
            # cutter; `solidify` is step one and is voxel-based.
            prims = [p for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
            b = md.bounds_of(prims)
            t0 = time.time()
            got = md.solidify_prims(prims, a.thicken, bounds=b)
            print(f"[probe] THICKEN DONE {time.time() - t0:.1f}s  {got}",
                  flush=True)
            return 0

        # The cutter wants an UNWELDED soup (3 verts per tri).
        flat = F.reshape(-1)
        soup = md.Soup(V[flat], None,
                       np.arange(len(flat), dtype=np.int64).reshape(-1, 3),
                       None, [], [])
        mech = Q.MECHANISMS[a.mech]
        fail = md.Failure(a.mech, lambda p, m=mech: np.full(
            len(np.atleast_2d(p)), float(m.intensity)))
        seeds = md.fracture_seeds(soup, fail, float(mech.fragment_m), 7,
                                  a.max_cells)
        print(f"[probe] backend={md.active_backend()} seeds={len(seeds)}",
              flush=True)
        t0 = time.time()
        frags = md._fracture_soup(soup, seeds, cap=True)
        print(f"[probe] DONE {time.time() - t0:.1f}s  {len(frags)} fragment(s)",
              flush=True)
    finally:
        sys.stdout.flush()
        os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
