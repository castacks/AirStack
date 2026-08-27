"""A pxr stand-in good enough to exercise quake_flow's authoring helpers on
the host (no Isaac). Row-vector matrices, exactly like Gf."""
import math
import sys
import types

import numpy as np


class Vec3(tuple):
    def __new__(cls, *a):
        if len(a) == 1:
            a = tuple(a[0])
        return super().__new__(cls, (float(a[0]), float(a[1]), float(a[2])))


    def __neg__(self):
        return Vec3(-self[0], -self[1], -self[2])

    def __add__(self, o):
        return Vec3(self[0] + o[0], self[1] + o[1], self[2] + o[2])

    def __sub__(self, o):
        return Vec3(self[0] - o[0], self[1] - o[1], self[2] - o[2])


Vec3f = Vec3
Vec3d = Vec3


class Vec2f(tuple):
    def __new__(cls, a, b):
        return super().__new__(cls, (float(a), float(b)))


class Rotation:
    def __init__(self, axis, deg):
        self.axis = np.asarray(tuple(axis), float)
        self.deg = float(deg)

    def GetQuat(self):
        class Q:
            def GetReal(self_):
                return 1.0

            def GetImaginary(self_):
                return Vec3(0, 0, 0)
        return Q()


class Matrix4d:
    def __init__(self, m=None):
        self.m = np.eye(4) if m is None else np.asarray(m, float).copy()

    def SetRotate(self, rot):
        ax = rot.axis / (np.linalg.norm(rot.axis) or 1.0)
        th = math.radians(rot.deg)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        Rcol = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)
        self.m = np.eye(4)
        self.m[:3, :3] = Rcol.T          # row-vector convention
        return self

    def SetTranslate(self, t):
        self.m = np.eye(4)
        self.m[3, :3] = np.asarray(tuple(t), float)
        return self

    def __mul__(self, other):
        return Matrix4d(self.m @ other.m)

    def Transform(self, p):
        v = np.array([p[0], p[1], p[2], 1.0]) @ self.m
        return Vec3(v[0], v[1], v[2])

    def ExtractTranslation(self):
        return Vec3(*self.m[3, :3])


class Transform:
    def __init__(self, m):
        self.m = m

    def GetTranslation(self):
        return Vec3(*self.m.m[3, :3])

    def GetRotation(self):
        return Rotation((0, 0, 1), 0.0)

    def GetScale(self):
        return Vec3(1, 1, 1)


class Quatf:
    def __init__(self, r, i):
        self.r, self.i = r, i


Gf = types.SimpleNamespace(Vec3f=Vec3f, Vec3d=Vec3d, Vec2f=Vec2f,
                           Matrix4d=Matrix4d, Rotation=Rotation,
                           Transform=Transform, Quatf=Quatf)


class Path(str):
    def GetParentPath(self):
        return Path(self.rsplit("/", 1)[0])

    def AppendChild(self, name):
        return Path(self + "/" + str(name))

    def pathString(self):
        return str(self)


Sdf = types.SimpleNamespace(Path=Path, AssetPath=lambda *a: (a[0] if a else ""),
                            ValueTypeNames=types.SimpleNamespace(
                                **{k: k for k in (
                                    "Color3f", "Float", "Asset", "Token",
                                    "Float2", "Float3", "Bool", "Int",
                                    "String", "Normal3f", "TexCoord2f")}))
Vt = types.SimpleNamespace(Vec3fArray=list, IntArray=list, FloatArray=list)


class _Op:
    def __init__(self, prim, kind):
        self.prim, self.kind = prim, kind

    def Set(self, v):
        self.prim.ops.append((self.kind, v))
        if self.kind == "translate":
            self.prim.world = Matrix4d().SetTranslate(v)


class Prim:
    def __init__(self, stage, path):
        self.stage, self.path = stage, str(path)
        self.ops = []
        self.attrs = {}
        self.world = Matrix4d()

    def IsValid(self):
        return True

    def GetPrim(self):
        return self

    def GetPath(self):
        return Path(self.path)

    def __getattr__(self, name):
        if name.startswith("Create") or name.startswith("Set") or name.startswith("Get"):
            def f(*a, **k):
                if name == "GetPointsAttr":
                    class A:
                        def Get(self_):
                            return self.attrs.get("points", [])
                    return A()
                if name.startswith("Create") and a:
                    self.attrs[name[6:]] = a[0]
                return _Attr()
            return f
        raise AttributeError(name)


class _Attr:
    def Set(self, *a, **k):
        return None

    def Get(self):
        return None

    def __getattr__(self, name):
        return lambda *a, **k: None


class Xformable:
    def __init__(self, prim):
        self.prim = prim

    def AddTranslateOp(self):
        return _Op(self.prim, "translate")

    def AddRotateZOp(self):
        return _Op(self.prim, "rotateZ")

    def AddRotateXYZOp(self):
        return _Op(self.prim, "rotateXYZ")

    def AddOrientOp(self):
        return _Op(self.prim, "orient")

    def AddScaleOp(self):
        return _Op(self.prim, "scale")

    def ClearXformOpOrder(self):
        self.prim.ops = []

    def GetOrderedXformOps(self):
        return []


class XformCache:
    def GetLocalToWorldTransform(self, prim):
        return prim.world

    def GetLocalTransformation(self, prim):
        return (prim.world, None)


class _Typed:
    @staticmethod
    def Define(stage, path):
        return stage.define(path)

    def __init__(self, prim):
        self.prim = prim


class Mesh(_Typed):
    pass


UsdGeom = types.SimpleNamespace(
    Mesh=Mesh, Xform=_Typed, Scope=_Typed, Camera=_Typed,
    Xformable=Xformable, XformCache=XformCache,
    Tokens=types.SimpleNamespace(none="none", faceVarying="faceVarying"),
    XformOp=types.SimpleNamespace(TypeTranslate="t", TypeRotateZ="rz"),
    SetStageMetersPerUnit=lambda *a: None, SetStageUpAxis=lambda *a: None)


class MaterialBindingAPI:
    def __init__(self, prim):
        pass

    def Bind(self, mat):
        return None

    def ComputeBoundMaterial(self):
        return (None, None)


UsdShade = types.SimpleNamespace(
    MaterialBindingAPI=MaterialBindingAPI,
    Material=types.SimpleNamespace(Get=lambda *a: None,
                                   Define=lambda st, path: st.define(path)),
    Shader=types.SimpleNamespace(Get=lambda *a: None,
                                 Define=lambda st, path: st.define(path)),
    Tokens=types.SimpleNamespace(surface="surface", mdl="mdl"))
Usd = types.SimpleNamespace(PrimRange=lambda root: [],
                            TimeCode=types.SimpleNamespace(Default=lambda: 0.0))
UsdLux = types.SimpleNamespace()

pxr = types.ModuleType("pxr")
pxr.Gf, pxr.Sdf, pxr.Vt, pxr.UsdGeom, pxr.UsdShade = Gf, Sdf, Vt, UsdGeom, UsdShade
pxr.Usd, pxr.UsdLux = Usd, UsdLux


class Stage:
    def __init__(self):
        self.prims = {}

    def define(self, path):
        p = str(path)
        if p in self.prims:
            raise RuntimeError("duplicate Define: " + p)
        pr = Prim(self, p)
        self.prims[p] = pr
        return pr

    def GetPrimAtPath(self, path):
        return self.prims.get(str(path))

    def DefinePrim(self, path):
        return self.define(path)


def install():
    sys.modules["pxr"] = pxr
    return pxr
