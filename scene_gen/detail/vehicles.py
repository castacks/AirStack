"""vehicles.py — cars as things the camera looks INTO, not just at.

Two jobs, both about a parked car being a container rather than a solid:

    strip_glass         deactivate the glass meshes so an occupant is visible
    car_pose_for_lane   where a car sits when it is DRIVING, not parked

WHY THE GLASS HAS TO GO RATHER THAN BE MADE TRANSPARENT. Every window in this
library is authored as fractional opacity — a `UsdPreviewSurface` with
`inputs:opacity` at 0.2-0.8, or an OmniPBR with `opacity_constant` — and this
renderer throws that away. `kit/mdl/rtx/UsdPreviewSurface.mdl` blends diffuse
against TRANSMISSION by `opacity`, gated by a hidden
`enable_specular_transmission = false`, so fractional opacity is forced to 1.0
and only `opacityThreshold > 0` (a hard cutout) does anything; OmniPBR's
`opacity_constant` becomes a fractional CUTOUT, which RTX Real-Time discards
unless `/rtx/raytracing/fractionalCutoutOpacity` is on. The whole history is in
the `build-wildfire-scenes` skill's ground-scar section, which lost four
attempts to exactly this. Net effect: car glass renders OPAQUE today, so the
only way to see a driver is for the glass not to be there.

WHICH ASSETS THIS CAN WORK ON AT ALL. Measured in Isaac (`shared.yaml` records
the same list under the `glass_separable` tag):

    RetroNeighborhood/130.usdz                    glass is its own Mesh, bound
    RetroNeighborhood/Nissan_Fairlady_Z_...usdz   to a material whose shader
    RetroNeighborhood/FREE_GMC_Motorhome_...usdz  carries a fractional opacity
    RetroNeighborhood/ZIS-101A_Sport_1938.usdz    -> strippable

    Muyang/DownTown/Assets/Vehicle_{A,Taxi,Police}.usd
        ONE mesh (`LOD0`, ~2.3k pts), NO material binding at all, windows
        painted into the base-colour texture -> nothing to strip, and no
        amount of shader work will make one transparent.

So this returns 0 on the Muyang cars by construction, and the caller decides
whether that matters (`suburb_scene.build_cars` records `glass_separable` on
every placement so a later occupant pass can pick cars it can see into).

TWO SIGNALS, OR'd, because neither alone covers the pack. OPACITY is the
correct one and is what the four Retro assets actually declare; NAMES catch a
mesh whose material could not be resolved — a common enough state on kit art,
where binding is per GeomSubset and a per-Mesh `ComputeBoundMaterial` reports
much of this library as unbound (see `tools/material_binding.py`).

THE SUBSET RULE IS DELIBERATELY CONSERVATIVE. Where a Mesh is unbound but its
GeomSubsets are bound, the mesh is removed only if EVERY bound subset is
glassy. One glass subset on a mesh that is otherwise the body means the body
and the glass are the same geometry, and deactivating it deletes the car.
"""

import math
import os

# `pxr` IS IMPORTED PER FUNCTION, not at module scope — the same reason
# `modular_house` does it ("so the rest of this file stays pure Python: the
# layout is built and tested host-side, where the project venv has no `pxr`").
# `CABIN_RULES` and `can_open_cabin` are policy, not USD, and the host-side
# planner and its 2D harness have to be able to read them; a module-level
# import made the whole file unimportable there and quietly sent every
# occupant decision down the permissive fallback.


# Substrings that name glass, on a Mesh or on the material bound to it. Matched
# case-insensitively against the prim NAME and against its full path, because
# kit art often carries the meaning in a parent scope (".../glass/Mesh_003")
# rather than in the leaf.
GLASS_TOKENS = ("glass", "window", "windshield", "windscreen", "glazing")

# Shader inputs that mean "not fully opaque". `opacity` is UsdPreviewSurface;
# `opacity_constant` / `cutout_opacity` are OmniPBR (`enable_opacity` gates the
# first — see OmniPBRBase.mdl `cutout_opacity = enable_opacity ? opacity_value
# : 1.0` — so an explicit False there means opaque no matter what the constant
# says, and is honoured below).
OPACITY_INPUTS = ("opacity", "opacity_constant", "opacityConstant",
                  "cutout_opacity")

# Below this an input counts as translucent. Not 1.0: authored values land on
# 0.9999-ish often enough that an exact test would miss them, and nothing in
# this library means 0.9995 as "slightly frosted".
OPAQUE_EPS = 0.999


def _lower_path(prim):
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    return str(prim.GetPath()).lower()


def _name_says_glass(prim):
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    if prim is None or not prim:
        return False
    return any(tok in _lower_path(prim) for tok in GLASS_TOKENS)


def _shaders_of(material):
    """Every shader worth reading for *material*, most authoritative first.

    The connected surface output for each render context comes first — that is
    the shader that actually runs. The material's whole subtree follows as a
    fallback, because a Material re-authored by our own passes (or exported by
    a DCC) can leave the terminal unconnected while still carrying the shader
    prim underneath it.
    """
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    out, seen = [], set()

    def _add(p):
        if p and p.IsValid() and p.GetPath() not in seen:
            seen.add(p.GetPath())
            out.append(UsdShade.Shader(p))

    if not material:
        return out
    for ctx in ("", "mdl", "glslfx"):
        try:
            src = (material.ComputeSurfaceOutput(ctx) if ctx
                   else material.ComputeSurfaceOutput())
        except Exception:
            continue
        if not src:
            continue
        try:
            conn = src.GetConnectedSource()
        except Exception:
            conn = None
        if conn and conn[0]:
            _add(conn[0].GetPrim())
    try:
        for p in Usd.PrimRange(material.GetPrim()):
            if p.IsA(UsdShade.Shader):
                _add(p)
    except Exception:
        pass
    return out


def _min_opacity(material):
    """The lowest constant opacity any shader under *material* declares.

    ``1.0`` when nothing says otherwise — including when an input exists but is
    CONNECTED to a texture rather than authored as a constant. A connected
    opacity map is not read here on purpose: it cannot be evaluated without
    sampling the image, and guessing "textured therefore transparent" would
    delete a car body carrying an alpha-masked decal.
    """
    best = 1.0
    for sh in _shaders_of(material):
        if not sh:
            continue
        enable = None
        try:
            inp = sh.GetInput("enable_opacity")
            if inp:
                v = inp.Get()
                if isinstance(v, bool):
                    enable = v
        except Exception:
            pass
        for name in OPACITY_INPUTS:
            try:
                inp = sh.GetInput(name)
            except Exception:
                continue
            if not inp:
                continue
            try:
                if inp.HasConnectedSource():
                    continue
                v = inp.Get()
            except Exception:
                continue
            if v is None or isinstance(v, bool):
                continue
            try:
                f = float(v)
            except (TypeError, ValueError):
                continue
            # OmniPBR only applies its constant when the switch is on. An
            # UNSET switch is left alone: the MDL default is on, and this is
            # exactly the case where the four Retro assets live.
            if enable is False and name in ("opacity_constant",
                                            "opacityConstant",
                                            "cutout_opacity"):
                continue
            best = min(best, f)
    return best


def _bound_material(prim):
    """The material bound to *prim*, or None. Never raises."""
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    try:
        mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
    except Exception:
        return None
    if mat and mat.GetPrim() and mat.GetPrim().IsValid():
        return mat
    return None


def _material_is_glass(material):
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    if material is None:
        return False
    return (_name_says_glass(material.GetPrim())
            or _min_opacity(material) < OPAQUE_EPS)


def _mesh_is_glass(mesh_prim):
    """Is this whole Mesh glass? See the module docstring's subset rule."""
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    if _name_says_glass(mesh_prim):
        return True
    mat = _bound_material(mesh_prim)
    if mat is not None:
        return _material_is_glass(mat)
    # Unbound at the Mesh: try the GeomSubsets, and require ALL of them.
    subs = []
    for child in mesh_prim.GetChildren():
        if child.IsA(UsdGeom.Subset):
            m = _bound_material(child)
            if m is not None:
                subs.append(_material_is_glass(m) or _name_says_glass(child))
    return bool(subs) and all(subs)


# WHAT IT TAKES TO SEE INSIDE EACH CAR, per asset, because it is per asset.
#
# The tag `glass_separable` says only "some mesh binds a transparent material".
# It is not a recipe, and treating it as one gets two assets wrong in opposite
# directions:
#
#   `Nissan_Fairlady_Z` — its transparent materials (`Material`,
#       `Material_009`) are bound to the ROOF AND BODY PANELS as well as the
#       windows, and every mesh is named `Object_N`, so `strip_glass` is
#       correct and still removes the roof and the pillars: the car comes out a
#       convertible. What it actually needs is ONE named mesh removed.
#   `Vehicle_A` / `Vehicle_Taxi` / `Vehicle_Police` — one `LOD0` mesh with the
#       windows painted into the texture. There is nothing to remove and no
#       rule will make an occupant visible; they are recorded here so nobody
#       tries again.
#
# The values were established on `car_occupants_launch_script.py`, which stands
# one occupant in each car and photographs it from the side. Re-measure there
# after adding a vehicle; a car that is not in this table falls back to
# `strip_glass`, which is the safe generic answer.
CABIN_RULES = {
    "130.usdz":                                    {"strip": True,  "extra": ()},
    "FREE_GMC_Motorhome_reimagined_low_poly.usdz": {"strip": True,  "extra": ()},
    "Nissan_Fairlady_Z_S30240Z_1978.usdz":         {"strip": False,
                                                    "extra": ("Object_16",)},
    "ZIS-101A_Sport_1938.usdz":                    {"strip": True,  "extra": ()},
    # Painted-on glass: nothing to open.
    "Vehicle_A.usd":                               {"strip": False, "extra": ()},
    "Vehicle_Taxi.usd":                            {"strip": False, "extra": ()},
    "Vehicle_Police.usd":                          {"strip": False, "extra": ()},
    "Car_01_0.usd":                                {"strip": False, "extra": ()},
    # ---- the 2026-08-26 standalone drop: ALL painted-on glass ---------------
    # Screened structurally before they went into any pool (open the stage,
    # walk every Mesh, look for one whose name says glass or whose bound
    # material carries a fractional opacity): every one of the eleven is a
    # SINGLE merged mesh with no separable glass and no transparent material
    # anywhere — the `Vehicle_A` case exactly, not the `Nissan` one. There is
    # nothing for `strip_glass` to remove and no mesh worth naming in `extra`.
    #
    # THEY ARE HERE PRECISELY SO THEY ARE NOT UNKNOWN. `can_open_cabin` returns
    # True for a car it has never heard of, on the reasoning that ordinary
    # glass is the commoner case — so leaving them out would let the occupant
    # planner seat people in them, and every one of those people would be a
    # labelled target invisible from every angle. They stay in the pool as
    # parked cars, which is all they can do.
    "burned_car_01.usdc":                          {"strip": False, "extra": ()},
    "burned_car_02.usdc":                          {"strip": False, "extra": ()},
    "citybus.usdc":                                {"strip": False, "extra": ()},
    "delivery_van.usdc":                           {"strip": False, "extra": ()},
    "generic_sedan.usdc":                          {"strip": False, "extra": ()},
    "lowpoly_coupe.usdc":                          {"strip": False, "extra": ()},
    "lowpoly_pickup.usdc":                         {"strip": False, "extra": ()},
    "lowpoly_sedan.usdc":                          {"strip": False, "extra": ()},
    "lowpoly_wagon.usdc":                          {"strip": False, "extra": ()},
    "police_car.usdc":                             {"strip": False, "extra": ()},
    "red_car.usdc":                                {"strip": False, "extra": ()},
}


def can_open_cabin(usd):
    """Can an occupant in this car be SEEN from outside?

    The gate for putting somebody in a car, and it is not the same question as
    `glass_separable`. That tag says a mesh binds a transparent material;
    this asks whether `open_cabin` has a way to get the glass off — which for
    `Nissan_Fairlady_Z` is a named mesh precisely BECAUSE it is not separable
    by material, and for the Muyang cars is nothing at all. An unknown car
    falls back to True on the same reasoning `open_cabin` does: the generic
    strip is the safe default, and a car nobody has measured is more likely to
    have ordinary glass than painted-on windows.
    """
    rule = CABIN_RULES.get(os.path.basename(str(usd)))
    if rule is None:
        return True
    return bool(rule.get("strip")) or bool(rule.get("extra"))


def open_cabin(stage, prim, usd, verbose=False):
    """Make the inside of one car visible. Returns how many meshes went off.

    The single entry point for "deactivate the windows", so the assembled plat
    and the occupant bench cannot drift apart on it. Applies this asset's rule
    from `CABIN_RULES` — a generic `strip_glass` for the cars where that is
    right, plus any mesh that has to be named because no rule can tell it from
    bodywork.
    """
    from pxr import Usd, UsdGeom
    if isinstance(prim, str):
        path = prim
        prim = stage.GetPrimAtPath(path)
    if not (prim and prim.IsValid()):
        return 0
    rule = CABIN_RULES.get(os.path.basename(str(usd)))
    n = 0
    if rule is None or rule.get("strip"):
        n += strip_glass(stage, prim, verbose=verbose)
    for name in ((rule or {}).get("extra") or ()):
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Mesh) and p.GetName() == name:
                if p.SetActive(False):
                    n += 1
    return n


def strip_glass(stage, prim, verbose=False):
    """Deactivate every glass Mesh under *prim*. Returns how many.

    *prim* is a ``Usd.Prim`` or a path string — the placement's ``prim_path``,
    which `scene_generator.apply_placements` writes back onto every placement
    dict once it has authored it.

    READ THROUGH INSTANCE PROXIES, AUTHOR ONLY OUTSIDE THEM. A referenced car
    composes as a normal (non-instanced) subtree, so the meshes are editable —
    but the traversal has to be able to see into a prototype if one ever
    appears, and a proxy prim cannot be edited. Proxies are therefore visited
    and skipped, loudly under *verbose*, rather than silently mis-counted.

    COLLECT PATHS, THEN AUTHOR. Deactivating a prim expires every handle into
    its subtree, including the range being iterated — the same failure the
    fracture pass hit ("Invalid range starting with expired 'Material' prim").
    """
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    if prim is None:
        return 0
    if isinstance(prim, str):
        prim = stage.GetPrimAtPath(prim)
    if not (prim and prim.IsValid()):
        return 0

    doomed, proxies = [], 0
    try:
        rng = Usd.PrimRange(prim, Usd.TraverseInstanceProxies(
            Usd.PrimDefaultPredicate))
    except Exception:
        rng = Usd.PrimRange(prim)
    for p in rng:
        if not p.IsA(UsdGeom.Mesh):
            continue
        try:
            if not _mesh_is_glass(p):
                continue
        except Exception:
            continue
        if p.IsInstanceProxy():
            proxies += 1
            continue
        doomed.append(p.GetPath())

    n = 0
    for path in doomed:
        q = stage.GetPrimAtPath(path)
        if q and q.IsValid():
            try:
                q.SetActive(False)
                n += 1
            except Exception:
                pass
    if verbose:
        print(f"[vehicles] strip_glass {prim.GetPath()}: {n} meshes off"
              + (f", {proxies} skipped as instance proxies" if proxies else ""))
    return n


def car_pose_for_lane(net, edge, s, side):
    """Where a car sits IN a traffic lane, and which way it is going.

    For the evacuation-queue pass: a car on the road rather than at the kerb.
    Parking lives in `suburb_scene.build_cars`; this is the moving-traffic
    geometry, and the two must not re-derive "which side is which" separately.

    RIGHT-HAND TRAFFIC. `suburb_net._perp` is the LEFT normal, so the RIGHT of
    the direction of travel is ``(t.y, -t.x)``. Traffic running along +t (from
    ``edge.pts[0]`` toward ``edge.pts[-1]``) therefore keeps to ``(t.y, -t.x)``
    and traffic running along -t keeps to ``(-t.y, t.x)`` — which is the same
    physical half of the road seen from the other end, so the two lanes come out
    on opposite sides of the centreline, as they should.

    Args:
        net:   the `suburb_net.Network` (used only to resolve an edge id).
        edge:  a `suburb_net.Edge`, or its integer id.
        s:     arclength along ``edge.pts``, metres from ``pts[0]``.
        side:  +1 for the lane carrying traffic along +t, -1 for the other one.

    Returns:
        ``(x, y, heading_deg)``. *heading_deg* is the DIRECTION OF TRAVEL, i.e.
        exactly what `pools.place` wants as its ``yaw`` argument — it adds the
        asset's own ``yaw-offset`` on top. Do not add
        `modular_house.CAR_YAW_EXTRA` to it; see `build_cars` for why that
        constant is a lot-frame bearing and not an art correction.
    """
    from pxr import Usd, UsdGeom, UsdShade  # noqa: F401
    from layout import suburb_net as sn

    if not hasattr(edge, "pts"):
        edge = net.edges[int(edge)]
    pts = edge.pts
    s = max(0.0, min(float(s), sn.polyline_length(pts)))
    c = sn.point_at(pts, s)
    t = sn.tangent_at(pts, s)
    sgn = 1.0 if float(side) >= 0.0 else -1.0
    # Lane centre: half the carriageway is one direction's, and the car sits in
    # the middle of that half — a quarter of the kerb-to-kerb width off the
    # centreline. On the two-lane local streets this suburb is made of, that is
    # the lane centre exactly.
    off = float(getattr(edge, "half_w", 4.0)) * 0.5
    nx, ny = t[1] * sgn, -t[0] * sgn          # right of the direction of travel
    return (c[0] + nx * off, c[1] + ny * off,
            math.degrees(math.atan2(t[1] * sgn, t[0] * sgn)))
