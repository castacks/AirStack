import sys, os, collections
from pxr import Usd, UsdGeom
for f in sys.argv[1:]:
    st = Usd.Stage.Open(f)
    dp = st.GetDefaultPrim()
    c = collections.Counter()
    for p in Usd.PrimRange(dp):
        if not p.IsA(UsdGeom.Mesh):
            continue
        n = p.GetName()
        key = ("frag" if n.startswith("frag_") else
               "heap" if n.startswith(("heap_", "windrow_", "collar_")) else
               "litter" if n.startswith("litter_") else
               "slab/col/part" if n.startswith(("slab_", "col_", "part_", "roofslab")) else
               "prop" if n.startswith("prop_") else
               "kit" if n.startswith("SM_") else "other:" + n.split("_")[0])
        c[key] += 1
    print(os.path.basename(f), dict(c))
