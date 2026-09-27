"""Shared USD edits for third-party assets."""
import numpy as np
from pxr import Usd, UsdGeom, Vt


def bake_instancers(ts):
    """Replace each PointInstancer in a tree asset by one plain mesh of all its instances: Isaac's segmentation
    leaves point-instanced geometry UNLABELLED (White Ash and Honey Locust crowns came back unlabelled)."""
    from pxr import UsdShade
    done = 0
    for q in [q for q in ts.Traverse() if q.IsA(UsdGeom.PointInstancer) and q.IsActive()]:
        pi = UsdGeom.PointInstancer(q); Ms = [np.array(m) for m in pi.ComputeInstanceTransformsAtTime(Usd.TimeCode.Default(), Usd.TimeCode.Default())]
        idx = np.array(pi.GetProtoIndicesAttr().Get()); protos = [ts.GetPrimAtPath(t) for t in pi.GetPrototypesRel().GetTargets()]
        for k_, pr in enumerate(protos):
            for mp in [x for x in Usd.PrimRange(pr) if x.IsA(UsdGeom.Mesh)]:
                m = UsdGeom.Mesh(mp); P = np.array(m.GetPointsAttr().Get(), float)
                L = np.array(UsdGeom.Xformable(mp).ComputeLocalToWorldTransform(0)) @ np.linalg.inv(np.array(UsdGeom.Xformable(pr).ComputeLocalToWorldTransform(0)))
                P = np.c_[P, np.ones(len(P))] @ L; cnt = np.array(m.GetFaceVertexCountsAttr().Get()); fi = np.array(m.GetFaceVertexIndicesAttr().Get())
                sel = [M for M, j in zip(Ms, idx) if j == k_]
                V = np.concatenate([(P @ M)[:, :3] for M in sel]); F = np.concatenate([fi + i * len(P) for i in range(len(sel))])
                out = UsdGeom.Mesh.Define(ts, q.GetParent().GetPath().AppendChild(f"{q.GetName()}_{mp.GetName()}_baked"))
                xf = UsdGeom.Xformable(out); xf.AddTransformOp().Set(UsdGeom.Xformable(q).GetLocalTransformation())
                out.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(V.astype(np.float32))); out.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.tile(cnt, len(sel)).astype(np.int32)))
                out.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(F.astype(np.int32))); out.CreateSubdivisionSchemeAttr(m.GetSubdivisionSchemeAttr().Get() or "none")
                out.CreateDoubleSidedAttr(True)
                for pv in UsdGeom.PrimvarsAPI(m).GetPrimvars():
                    v_ = pv.Get()
                    if v_ is None or pv.IsIndexed(): continue
                    npv = UsdGeom.PrimvarsAPI(out).CreatePrimvar(pv.GetPrimvarName(), pv.GetTypeName(), pv.GetInterpolation())
                    npv.Set(v_ if pv.GetInterpolation() == UsdGeom.Tokens.constant else type(v_).FromNumpy(np.concatenate([np.asarray(v_)] * len(sel))))
                n_ = m.GetNormalsAttr().Get()
                if n_ is not None and len(n_):
                    N = np.asarray(n_, float) @ L[:3, :3]; N = np.concatenate([N @ M[:3, :3] for M in sel]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-9
                    out.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(N.astype(np.float32))); out.SetNormalsInterpolation(m.GetNormalsInterpolation())
                mat = UsdShade.MaterialBindingAPI(mp).ComputeBoundMaterial()[0]
                if mat: UsdShade.MaterialBindingAPI.Apply(out.GetPrim()).Bind(mat)
                done += 1
        q.SetActive(False)
    return done
