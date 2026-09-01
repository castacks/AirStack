"""sky_presets.py — `mid_day` / `sunset` lighting presets shared by the fire
launch scripts (the people bench and, behind a knob, the city).

WHY THIS EXISTS (2026-08-31, user: "the current one looks like sunset ... I
want more of a mid day vibe. Let's try this out in the empty scene for human
placements" — i.e. `fire_people_bench_launch_script.py`, the 3-building
bench).

THE DIAGNOSIS, TRACED END TO END — there are actually TWO different
"current" looks in this repo, not one:

  1. THE BENCH. `fire_people_bench_launch_script.py` lights through
     `disaster/fire_assembly_lib.build_ground_and_light`, which authors a
     `DistantLight` key via `AddRotateXYZOp((-25.0, 0.0, 28.0))` — 25 degrees
     of elevation — colored warm `(1.0, 0.94, 0.86)`. That function's own
     docstring explains why: char reads at 0.15 and a spall scar at 0.44, so
     a flat overhead key crushes the whole elevation to black; "a raking sun
     separates the tongues and the scars." It is a genuine low
     late-afternoon/golden-hour sun, chosen for DAMAGE readability, not for a
     people bench.
  2. Independently, `disaster/planks.py`'s `_lay` docstring measured the
     RetroNeighborhood stage's own borrowed sun (`asset_sets/shared.yaml`'s
     `sky: RetroNeighborhood/RetroNeighborhood.stage.usd`, composed in by
     `scene_prep.add_sky`) at 16.36 h, **24.4 degrees of elevation** —
     independently arrived at, same story, ~25 degrees.
  3. THE CITY. `urban_fire_city_launch_script.py` does not use either of the
     above. It goes through `resolve_sky`/`add_sky` against
     `downtown_fire_500.yaml`, which deliberately carries NO `sky:` key
     (reverted 2026-08-29, user: "revert the sky back to what it was
     before" — a `mealie_road.hdr` dome, a Clear-category HDRI with its OWN
     baked-in low sun, made the city read as evening; see that preset's own
     "---- LIGHT ----" comment block). Today the city is dome-only, flat and
     directionless (`sky_intensity: 2000.0`, `sky_exposure: 0.0`, no
     shadow-casting sun at all — `_disable_sky_sun` vetoes any a borrowed
     sky would have brought).

So `sunset` below reproduces the BENCH's literal numbers exactly (kept for
an A/B, not "improved" in any way); the CITY launcher's own default path
leaves its existing dome-only code untouched rather than calling into this
module at all — see that launcher's lighting block and its `FC_SKY` knob.

THE FIX. `mid_day` raises the sun to a real overhead angle (60-70 degrees),
switches its color from warm amber to a neutral-to-cool ~6500 K white, and
lights the dome with a procedural blue-sky color. No HDRI is required for
any of that — every number below is a flat color or a distant-light value —
so lighting improves even with Nucleus unreachable or a sky asset path that
404s. An explicit HDRI/stage path can still be layered on top (pass it as
`name` instead of a preset key — see `apply_sky_preset`) once one is
confirmed to actually resolve: see `scene_gen/tools/sky_probe.py` and
`scene_gen/_plans/isaac_overcast_sky.md` for the candidate list and the
material-binding trap (`UsdShade.MaterialBindingAPI(...)
.UnbindAllBindings()` must run before a new texture is set, or a bound MDL
silently keeps painting the old sky over the new texture).

Env convention (not read by this module — each launcher owns its own knob):
callers read `PB_SKY` / `FC_SKY` and pass the value straight through to
`apply_sky_preset` as `name`. A value that is not a known preset key is
treated as an explicit HDRI/.hdr/.exr/stage-USD path or URL: the `mid_day`
sun/dome NUMBERS are kept, only the dome's color is replaced by that
texture.
"""

from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------
# Each entry:
#   dome_intensity, dome_exposure, dome_color   -- ambient fill (DomeLight)
#   sun_intensity, sun_angle, sun_color         -- key light (DistantLight)
#   sun_elev_deg, sun_az_deg                    -- key light direction,
#     applied as `AddRotateXYZOp((-elev, 0, az))` -- the same convention
#     `fire_assembly_lib.build_ground_and_light` already uses, reproduced
#     here rather than reinvented so `sunset` can be numerically identical.
PRESETS = {
    # `fire_assembly_lib.build_ground_and_light`'s key numbers, verbatim:
    # dome 700 @ (0.72, 0.76, 0.86), key 3200 @ angle 0.9, warm
    # (1.00, 0.94, 0.86), 25 deg elevation / 28 deg azimuth. This is the
    # bench's literal CURRENT look, kept only so mid_day has something to be
    # compared against.
    "sunset": dict(
        dome_intensity=700.0, dome_exposure=0.0,
        dome_color=(0.72, 0.76, 0.86),
        sun_intensity=3200.0, sun_angle=0.9,
        sun_color=(1.00, 0.94, 0.86),
        sun_elev_deg=25.0, sun_az_deg=28.0,
    ),
    # High overhead sun, neutral-cool ~6500 K white (D65-ish; not a literal
    # blackbody conversion, just close enough to read as "neutral daylight"
    # rather than tungsten-warm), blue-sky dome fill. Purely procedural --
    # no HDRI dependency, so this is what renders even offline.
    "mid_day": dict(
        dome_intensity=1200.0, dome_exposure=0.0,
        dome_color=(0.55, 0.72, 0.93),          # clear midday sky blue
        sun_intensity=3500.0, sun_angle=0.53,   # real solar angular diameter
        sun_color=(0.97, 0.98, 1.00),           # ~6500 K neutral-cool white
        sun_elev_deg=65.0, sun_az_deg=135.0,
    ),
}

DEFAULT_DOME_PATH = "/World/DomeLight"
DEFAULT_SUN_PATH = "/World/KeySun"


def _looks_like_a_path(name):
    """True if *name* reads as an asset path/URL rather than a preset key."""
    if not name or name in PRESETS:
        return False
    lower = name.lower()
    return ("://" in name or "/" in name or "\\" in name
            or lower.endswith((".hdr", ".exr", ".usd", ".usda", ".usdc")))


def apply_sky_preset(stage, name="mid_day", *, dome_path=DEFAULT_DOME_PATH,
                     sun_path=DEFAULT_SUN_PATH, prefix="sky"):
    """Author (or update) one dome + one sun light for preset *name*.

    *name* is a `PRESETS` key ("mid_day", "sunset") or an explicit HDRI /
    stage-USD path or URL — in that case the `mid_day` sun/dome NUMBERS are
    used and only the dome gets textured with *name* instead of a flat
    color. Anything else unrecognized falls back to "mid_day" (never
    silently to nothing — an unset/typo'd knob should still look better than
    the pre-existing sunset default, not identical to it by accident).

    Idempotent: both lights are `Get`-or-`Define`d at their given paths, so
    calling this again (e.g. to switch presets on a live stage between
    iterations) updates the same two prims instead of leaving a stray light
    behind — the same pattern `scene_prep.add_dome_light` already uses.

    Returns the resolved preset key actually applied, for logging.
    """
    texture_file = None
    key = name if name in PRESETS else None
    if key is None:
        if _looks_like_a_path(name):
            texture_file = name
        key = "mid_day"
    p = PRESETS[key]

    if stage.GetPrimAtPath(dome_path).IsValid():
        dome = UsdLux.DomeLight.Get(stage, dome_path)
    else:
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path(dome_path))
    dome.CreateIntensityAttr(p["dome_intensity"])
    dome.CreateExposureAttr(p["dome_exposure"])
    dome.CreateColorAttr(Gf.Vec3f(*p["dome_color"]))
    if texture_file:
        # A material BOUND to the dome ignores the light prim's own
        # texture:file attribute -- unbind first or the swap silently does
        # nothing (see scene_gen/_plans/isaac_overcast_sky.md, trap #2).
        UsdShade.MaterialBindingAPI(dome.GetPrim()).UnbindAllBindings()
        dome.CreateTextureFileAttr(Sdf.AssetPath(texture_file))
        dome.CreateTextureFormatAttr(UsdLux.Tokens.latlong)

    if stage.GetPrimAtPath(sun_path).IsValid():
        sun = UsdLux.DistantLight.Get(stage, sun_path)
    else:
        sun = UsdLux.DistantLight.Define(stage, Sdf.Path(sun_path))
    sun.CreateIntensityAttr(p["sun_intensity"])
    sun.CreateAngleAttr(p["sun_angle"])
    sun.CreateColorAttr(Gf.Vec3f(*p["sun_color"]))
    xf = UsdGeom.Xformable(sun.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddRotateXYZOp().Set(
        Gf.Vec3f(-float(p["sun_elev_deg"]), 0.0, float(p["sun_az_deg"])))

    print("[{0}] sky preset '{1}'{2}: dome '{3}' (I={4}, exp={5}, "
          "color={6}), sun '{7}' (I={8}, elev={9} deg, az={10} deg){11}"
          .format(prefix, name,
                  "" if key == name else " -> '{0}'".format(key),
                  dome_path, p["dome_intensity"], p["dome_exposure"],
                  tuple(p["dome_color"]), sun_path, p["sun_intensity"],
                  p["sun_elev_deg"], p["sun_az_deg"],
                  " texture={0}".format(texture_file) if texture_file
                  else ""))
    return key
