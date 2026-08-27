#!/usr/bin/env python
import os, sys
REPO = "/isaac-sim/AirStack"; sys.path.insert(0, os.path.join(REPO, "scene_gen"))
OUT = "/isaac-sim/.nvidia-omniverse/logs/_g_tex"
ROOT = ("omniverse://airlab-nucleus.andrew.cmu.edu:443/Projects/SEI-COA/"
        "ModernCityEnvironment01/")


def main():
    import omni.client
    from pxr import Usd, UsdShade, Sdf, Ar
    try:
        os.makedirs(OUT)
    except OSError:
        pass
    for folder in ("Materials/", "Textures/", ""):
        r = omni.client.list(ROOT + folder)
        print("LIST", folder, str(r[0]), len(r[1]) if len(r) > 1 else 0)
        if len(r) > 1:
            for e in r[1][:400]:
                if "Skyscraper" in e.relative_path or folder == "":
                    print("   ", e.relative_path)
    # resolve through USD instead of guessing
    st = Usd.Stage.Open(ROOT + "Meshes/SM_MBuilding05_SkyscraperFacade_B.usd")
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        sh = UsdShade.Shader(p)
        if not sh or sh.GetIdAttr().Get() != "UsdUVTexture":
            continue
        f = sh.GetInput("file")
        v = f.Get() if f else None
        if v is None:
            continue
        res = v.resolvedPath or ""
        print("TEX", v.path, "->", res)
        if res:
            rr = omni.client.read_file(res)
            print("   read", str(rr[0]))
            if "OK" in str(rr[0]):
                nm = res.rsplit("/", 1)[-1]
                with open(os.path.join(OUT, nm), "wb") as fh:
                    fh.write(memoryview(rr[-1]).tobytes())
                print("   wrote", nm, os.path.getsize(os.path.join(OUT, nm)))
    return 0


raise SystemExit(main())
