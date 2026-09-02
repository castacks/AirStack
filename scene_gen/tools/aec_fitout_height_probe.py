"""aec_fitout_height_probe — isolate the fit-out Z-overshoot hypothesis for
AEC brownstones: `register_style`'s synthetic spec models every storey as
ONE uniform height (`g["storey_h"]`) repeated `n` times, and `fit_interior`
(called from `urban_fire.burn_building` via `quake_flow.describe` ->
`_mass_specs`) builds its slab/column grid off THAT rebuilt mass box, not
`gac_fire.prepare()`'s own REAL measured one (`mass_from_grid`, off the
merged mesh's actual bbox/levels).

This runs `gac_fire.prepare()` (place + measure + plan, no slice, no bake —
`level="F1"` so no atlas bake is even attempted) then reproduces
`gac_storey_slice.slice_to_kit`'s own `register_style` call and
`quake_flow._mass_specs`'s rebuild, and prints both height models side by
side, per building.

Read-only / offline. No Kit, no GPU, no bake written anywhere real (fresh
throwaway stage). Run via usd_python.sh:
  docker exec isaac-sim bash -c "cd /isaac-sim/AirStack && \
    ./scene_gen/tools/usd_python.sh scene_gen/tools/aec_fitout_height_probe.py \
    Reference_Brownstone5Row Reference_Brownstone8Row Reference_Brownstone10Row"
"""
import random
import sys


def probe_one(asset):
    sys.path.insert(0, "/isaac-sim/AirStack/scene_gen")
    from pxr import Usd, UsdGeom

    from disaster import gac_fire as gf
    from detail import gac_slice as gsl
    from detail import gac_storey_slice as gss
    from detail import urban_building as ub
    from disaster import quake_flow as qf

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    cell = "/World/cell0"
    UsdGeom.Xform.Define(stage, cell)

    rng = random.Random(7)
    name = "aec:" + asset
    pre = gf.prepare(stage, cell, name, "F1", rng, tag="probe", verbose=False)
    g = pre["grid"]
    m = pre["mass"]
    real_h = float(m["top"] - m["z0"])
    real_n = len(m["levels"])

    style = "probe_" + asset
    spec = gsl.register_style(g, style, pieces_of=[], family="01")
    rebuilt = qf._mass_specs(style, 0.0, 0.0, 0.0)[0]
    rebuilt_h = float(rebuilt["top"] - 0.0)   # z0=0 in this call
    rebuilt_n = len(rebuilt["levels"])

    storey_band = next(b for b in spec["bands"] if not b.get("parapet"))
    print("=== {0} ===".format(asset))
    print("  grid: storey_h={0:.3f}  raw storeys={1}  bbox_z=({2:.3f},{3:.3f})"
          .format(g["storey_h"], len(g.get("storeys") or []),
                  g["bbox"][0][2], g["bbox"][1][2]))
    print("  REAL measured mass (mass_from_grid): n_storeys={0}  H={1:.3f} m  "
          "levels={2}".format(real_n, real_h,
                              ["{0:.2f}".format(z) for z in m["levels"]]))
    print("  REBUILT synthetic mass (register_style -> _mass_specs): "
          "band h={0:.3f} x repeat={1} = {2:.3f} m; n_levels={3}  H={4:.3f} m"
          .format(storey_band["h"], storey_band["repeat"],
                  storey_band["h"] * storey_band["repeat"], rebuilt_n, rebuilt_h))
    print("  DELTA (rebuilt - real): {0:+.3f} m over {1} storey(s) -- this is "
          "what fit_interior's slab/column grid extends past the real roof/"
          "walls by (Z) once storeys() ships levels this dense".format(
              rebuilt_h - real_h, real_n))
    print("  mass W x D: real={0:.3f} x {1:.3f}   (fit_interior's plan box is "
          "always this rectangle, real footprint or not)".format(m["W"], m["D"]))


if __name__ == "__main__":
    for a in sys.argv[1:]:
        try:
            probe_one(a)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print("FAILED", a, exc)
