"""_dtc_export_neg — the NEGATIVE control for `dtc_export_probe.py`.

"A verifier that cannot fail on the bad file is decoration"
(`test_fire_bake.py`'s own rule). This runs the identical chain with
`gac_slice._material_source` reverted to its pre-2026-08-30 behaviour — the
`Materials/*_Inst.usd` sidecar only, `(None, None)` for anything else — which
is what a downtowncity block hit before the inline-layer fallback existed.
Expected: every material FAILS to rehome, the source subtree has to be KEPT,
and the export says `src_kept=True`.
"""
import sys
sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
from detail import gac_slice as gsl


def _old_material_source(mat_prim):
    for spec in mat_prim.GetPrimStack():
        ident = str(spec.layer.identifier)
        if "/Materials/" in ident or ident.endswith("_Inst.usd"):
            return ident, spec.path
    return None, None


gsl._material_source = _old_material_source
print("[neg] gac_slice._material_source reverted to the sidecar-only form")
import runpy
sys.argv = ["dtc_export_probe.py"] + sys.argv[1:]
runpy.run_path("/isaac-sim/AirStack/scene_gen/tools/dtc_export_probe.py",
               run_name="__main__")
