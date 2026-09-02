"""fire_bake — the pure-python half of the PER-BUILDING fire bake.

WHY A PER-BUILDING BAKE EXISTS
------------------------------
`gac_fire_bench_launch_script.py` builds a whole row in ONE Kit process: six
sliced GreatAmericanCity buildings plus two ModernCityEnvironment kit
buildings, burned, then settled together. Measured on the F1..F6 + 2x F5c
row (2026-08-30): **25 GB RSS**, one combined settle of ~2,350 loose bodies
that ended with **688 bodies STILL MOVING at bake time** — i.e. frozen
mid-flight — and a scene that is slow to open because every building's
merged source, slicer state and physics scene is still in it.

The user's instruction (2026-08-30): *"you can do 1 building at a time, bake
it then launch them together as static. That might be faster. I don't need
these bakes to be saved on host since we will need to work on them, they can
be container only."*

So the pipeline is now two stages:

  1. `simulation/isaac-sim/launch_scripts/fire_bake_launch_script.py` — ONE
     building per headless Kit process. It builds it at the origin, burns it,
     settles it *alone* (so the settle can actually converge — the combined
     run's 688 movers were a step budget shared between eight buildings), and
     exports a self-contained USD plus a `.json` sidecar.
  2. `simulation/isaac-sim/launch_scripts/fire_assembly_launch_script.py` —
     references the bakes side by side as STATIC geometry, then re-places the
     Flow emitters from the sidecars.

**NO FLOW IN A BAKE.** Smoke and flame are authored in the ASSEMBLY (the
user's stated plan), so a bake carries no `FlowEmitter*` prims at all — but
it DOES carry the fire EVENTS (`soot_plume.plan_events` records reduced to
plain data by `events_to_json` below), which is everything
`urban_fire._flame_sources` needs to put the emitters back on the right
window heads of the right storeys of the right elevations.

THREE TRAPS THIS MODULE EXISTS TO HANDLE
-----------------------------------------

**1. The material trap (`tools/bake_gac_kits.py`, `gac_slice.rehome_materials`).**
A sliced GAC piece binds a Material prim living *inside the merged source's
own subtree* (`<cell>/src`). So does every SOOTED copy of one:
`soot_plume.piece_material_like` composes the source material with an
`AddInternalReference` and overrides only its diffuse map. Export the pieces
with `<cell>/src` stripped and every one of them renders WHITE with its
geometry and UVs perfectly intact — a silent failure that looks like a
texture bug ("this turned every sliced building white"). `rehome_for_export`
re-anchors each such material onto the GAC material's OWN Nucleus file
(`gac_slice._material_source`) and re-applies the local PNG override, so the
source subtree can then be dropped. When it CANNOT (no resolvable source
file), the caller is told to keep `<cell>/src` composed rather than ship a
white building.

**2. `stage.Flatten()` is the wrong tool** (`freeze-disaster-dataset`
skill, `disaster/freeze.py`). Kit meshes/subsets/materials carry an
`assetInfo` dict core USD cannot unpack (`Usd_CrateFile::_UnpackValue:
unsupported type enum value 0`). The baker therefore exports the **ROOT LAYER
ONLY** (`stage.GetRootLayer().Export`), exactly as `bake_gac_kits.py` does:
everything it wrote is either a freshly authored prim or a fresh reference,
and a root-layer export preserves those without composing a single spec out
of the poisoned source.

**3. `settle.bake(bake_result=True)` freezes positions but leaves the physics
APIs applied** (`PhysicsRigidBodyAPI` with `rigidBodyEnabled=false`,
`PhysicsCollisionAPI`, `PhysicsMeshCollisionAPI`,
`PhysxConvexDecompositionCollisionAPI`, ~22 `physics:`/`physx*` attributes per
body — `disaster/bake.py` measured 994 of 4,824 meshes carrying them). They
are inert but they cost composition time and crate bytes in every assembly
that references the file, and the assembly is meant to be *static*.
`strip_physics` removes them.

WHAT IS HERE AND WHAT IS NOT. Everything in this file is importable with no
Kit, and the half that needs `pxr` imports it inside the function, so
`scene_gen/tests/test_fire_bake.py` runs host-side for the schema and
in-container (`tools/usd_python.sh`) for the USD half.
"""

import hashlib
import json
import math
import os

#: sidecar schema version — bump when a field changes meaning, not when one
#: is added (readers use `.get`).
SCHEMA = 1

#: Container-side default. NOT under the repo: the bakes are working files
#: ("I don't need these bakes to be saved on host", user 2026-08-30), and the
#: repo is bind-mounted, so writing them into `scene_gen/` would put tens of
#: GB on the host and into everybody else's checkout.
DEFAULT_OUT_DIR = "/isaac-sim/.cache/fire_bakes"

#: The bake's own hierarchy. `/World` is the exported file's `defaultPrim`
#: (USD requires a ROOT prim there), everything the baker authors is under
#: `/World/bake`, and the settle's own `/World/physicsScene` +
#: `/World/settleGroundPlane` are removed before the export.
DEFAULT_PRIM = "/World"
BAKE_ROOT = "/World/bake"
STRIP_PRIMS = ("/World/physicsScene", "/World/settleGroundPlane",
               "/World/ReviewCamera", "/World/flow")

#: `gac` a GreatAmericanCity merged asset, `dtc` a downtowncity merged asset,
#: `aec` an AEC "small brownstone" rowhouse (`Reference_Brownstone*Row.usd`,
#: already a bag of instanced parts — see `gac_fire.PACKS["aec"]`) — all
#: three sliced and burnt by `gac_fire.burn_gac` (which resolves the pack
#: from the kind, see `gac_fire.PACKS`); `kit` a ModernCityEnvironment kit
#: style built by `urban_building.build_building` + `urban_fire.burn_building`.
KINDS = ("gac", "dtc", "aec", "kit")
#: the kinds that route to the sliced merged-asset path
SLICED_KINDS = ("gac", "dtc", "aec")

#: applied API schemas and attribute prefixes that must not ship
_PHYS_SCHEMA_PREFIX = ("Physics", "Physx")
_PHYS_ATTR_PREFIX = ("physics:", "physx")

#: Flow prim type names — a bake must contain none of these.
_FLOW_TYPES = ("FlowEmitter", "FlowSimulate", "FlowOffscreen", "FlowRender")


# ---------------------------------------------------------------------------
# The manifest: `kind:name:level[:origin[:sides[:seed]]]`
# ---------------------------------------------------------------------------
def parse_entry(text, index=0, base_seed=7):
    """One manifest entry -> a dict the launcher's env knobs map onto 1:1.

    `kind:name:level[:origin[:sides[:seed]]]`, e.g.

        gac:SM_Building_02:F1
        gac:SM_Building_09:F6:3
        dtc:Amar_Tower:F3
        dtc:Building_12:F2:4
        kit:commercial_mid:F5c::S,E
        kit:office_wide:F5c:1:S/E:12

    Empty fields are absent, not defaults — the same rule
    `urban_fire_bench_launch_script` uses for its `UF_BUILDINGS` rows, and
    the same rule the launchers' `_env` applies to the container's habit of
    exporting every knob as an EMPTY STRING.

    `seed` defaults to `base_seed + 31 * index`, which is exactly the
    per-column seed `gac_fire_bench_launch_script` gives building `index`
    (`random.Random(SEED + 31*i)`), so a per-building bake of column i
    reproduces the row's building i rather than a different draw of it.
    """
    bits = [q.strip() for q in str(text).split(":")]
    while len(bits) < 6:
        bits.append("")
    kind, name, level, origin, sides, seed = bits[:6]
    kind = (kind or "").lower()
    if kind not in KINDS:
        raise ValueError(
            "unknown bake kind {0!r} in {1!r} (expected one of {2})".format(
                kind, text, "/".join(KINDS)))
    if not name:
        raise ValueError("no asset/style name in {0!r}".format(text))
    return {"kind": kind, "name": name, "level": (level or "F3"),
            "origin": (int(origin) if origin else None),
            "sides": parse_sides(sides),
            "seed": int(seed) if seed else int(base_seed) + 31 * int(index),
            "index": int(index)}


def parse_sides(spec):
    """`"S,E"` / `"S/E"` / `("S","E")` / `""` -> a tuple of letters or None."""
    if not spec:
        return None
    if isinstance(spec, str):
        spec = spec.replace("/", ",").split(",")
    out = tuple(str(q).strip().upper()[:1] for q in spec if str(q).strip())
    return out or None


def parse_manifest(text, base_seed=7):
    """A whitespace- (or `;`-) separated manifest -> [entry].

    NOT comma-separated, and that is deliberate: the SIDES field is itself a
    comma list (`kit:commercial_mid:F5c::S,E`), so splitting the manifest on
    commas turns one entry into two and the second one raises `unknown bake
    kind 'e'`. Entries come from argv in `tools/fire_bake.sh`, where
    whitespace is the natural separator anyway.

    Blank items and `#` comments are dropped; the INDEX used for the default
    seed counts only the entries that survive, so a comment cannot silently
    re-seed everything after it.
    """
    items = []
    for line in str(text).replace(";", "\n").splitlines():
        # A COMMENT IS TO END OF LINE, not a token. `"# a comment"` split on
        # whitespace leaves `a` and `comment` behind, which then parse as
        # bake kinds — the failure is a ValueError two entries later, with a
        # message about the wrong thing entirely.
        for raw in line.split("#", 1)[0].split():
            if raw.strip():
                items.append(raw.strip())
    return [parse_entry(t, i, base_seed) for i, t in enumerate(items)]


def _safe(s):
    return "".join(c if (c.isalnum() or c in "._-") else "_" for c in str(s))


#: canonical ring order for a stem's SIDES segment — S,E,N,W, not manifest
#: order, so `S,E` and `E,S` (the same building, on fire from the same two
#: sides) collide on one stem instead of caching twice.
_SIDE_RING = ("S", "E", "N", "W")


def out_stem(entry):
    """`<kind>_<name>_<level>[_o<origin>][_<sides>]_s<seed>` — the .usd and
    .json share it.

    `origin` (the storey the fire started on) and `sides` (which elevations
    are alight) come from a CITY-CELL entry — the per-building record
    `urban_fire_spread.solve` writes to `fire_city_<seed>.json`
    (`urban_fire_city_plan.md` sec 2-3) — and are folded into the stem so two
    ignitions of the same asset/level/seed cache as two different bakes
    rather than colliding. Both are OMITTED, not zero-filled, when absent:
    a plain row entry (`origin`/`sides` both `None` — every
    `fire_bake.sh`/`gac_fire_bench_launch_script` row today) keeps the exact
    stem it has always had, `<kind>_<name>_<level>_s<seed>`, so no bake
    already on disk goes stale. `sides` are joined in RING order
    (`_SIDE_RING`), not the order they appear in the entry.
    """
    bits = [_safe(entry["kind"]), _safe(entry["name"]), _safe(entry["level"])]
    origin = entry.get("origin")
    if origin is not None:
        bits.append("o{0}".format(int(origin)))
    sides = entry.get("sides")
    if sides:
        ring = "".join(c for c in _SIDE_RING if c in sides)
        if ring:
            bits.append(ring)
    return "_".join(bits) + "_s{0}".format(int(entry["seed"]))


def out_paths(entry, out_dir=DEFAULT_OUT_DIR):
    stem = out_stem(entry)
    return (os.path.join(out_dir, stem + ".usd"),
            os.path.join(out_dir, stem + ".json"))


def bake_tag(entry):
    """The short per-building tag `urban_fire` puts in every authored prim
    name. Short on purpose (prim paths get long on a sliced building) and
    stable, so a re-bake overwrites rather than accumulating."""
    return "{0}{1}".format(entry["kind"][0], entry["index"])


# ---------------------------------------------------------------------------
# The fire events, as plain data
# ---------------------------------------------------------------------------
# WHAT HAS TO SURVIVE, AND WHY EXACTLY THESE FIELDS.
# `urban_fire._flame_sources(ctx, root, op, state, scale, tag, per_opening)`
# — the one function that puts a Flow emitter on a window — reads:
#     op["hua"/"hub"/"hva"/"hvb"] (falling back to ua/ub/va/vb)  the reveal
#     op["fr"], op["out"]        -> quake_flow._b_face_pt: the façade plane
#     op["m"], op["side"]        -> quake_flow._outward: which way it vents
#     op["storey"], op["e"]      -> urban_fire._severity: how hard it burned
# and `_severity` in turn reads `ctx["fire"]` plus `_el_jitter(e)`, which
# hashes `e["x"]/["y"]/["z"]`. `r_flames` additionally needs each event's
# `state`, `storey`, `id` and `ops`, and skips any event one of whose
# openings belongs to a module a collapse has since killed (`e["dead"]`) —
# so the dead flag is serialised AS IT STANDS AFTER THE LADDER, which is the
# whole point of recording the events at the end of a bake rather than at
# the start.
#
# `"synthetic"` (2026-08-31, ADDITIVE — "avoid fires at the extreme top of
# buildings... unless we're 100% sure about windows on the top floor"):
# `gac_fire.openings_provider` sets `e["synthetic"] = True` on every opening
# its bay-window fallback invented (never on a real, measured one), so
# `fire_assembly_lib`'s top-storey filter can tell "sure" from "not sure"
# straight off the sidecar. ABSENT means real — every bake baked before this
# field existed round-trips exactly as it always has (`op_from_json` never
# invents the key), and `fire_assembly_lib.is_synthetic_op` falls back to
# the pre-existing `"gac_window_synth"` vs `"gac_window"` `name` convention
# for exactly those older bakes.
_OP_SCALARS = ("ua", "ub", "va", "vb", "hua", "hub", "hva", "hvb", "out")
_EV_SCALARS = ("u0", "u1", "z_sill", "z_head", "w_t", "h_eq", "A_v",
               "tau", "intensity", "drift")
_E_KEYS = ("mass", "x", "y", "z", "side", "storey", "name", "role", "dead",
          "synthetic")


def frame_to_json(fr):
    """`quake_flow._piece_frame` / `gac_fire.side_frame` 7-tuple -> a list.

    `(ox, oy, yaw, width, height, depth, dw)`; `dw` is the bool that says the
    frame pivots at the piece's CENTRE rather than its left end, and dropping
    it puts every emitter half a module off.
    """
    fr = list(fr)
    while len(fr) < 7:
        fr.append(0.0)
    return [float(v) for v in fr[:6]] + [bool(fr[6])]


def frame_from_json(v):
    return tuple(float(q) for q in v[:6]) + (bool(v[6]),)


def mass_to_json(m):
    """A `quake_flow` mass box -> plain data.

    LOSSY ON `spec`, DELIBERATELY. A kit mass carries `ub.STYLES[style]`'s
    whole nested band/wing spec; the only thing anything downstream of a bake
    reads out of it is `soot_plume.parapet_height`, which sums the `h` of the
    bands flagged `parapet`. Keeping that and dropping the rest means the
    sidecar cannot drift out of step with `urban_building`'s style table.

    NOT LOSSY ON `deck_z`, AS OF 2026-08-31. `gac_fire.mass_from_grid`
    measures the real roof SURFACE and its own docstring says the bbox `top`
    "is the parapet coping, not the deck" — and then this function dropped
    the measurement on the floor, so every consumer that asked a real bake
    sidecar for a deck height got nothing. `fire_people.deck_z()` reads
    `doc["fire"]["deck_z"]` and then `masses[*]["deck_z"]`; both were dead,
    so every roof group it has ever placed fell through to its third
    fallback (`H - PARAPET_EST_M`, `deck_source="estimated"`) and its own
    `SIDECAR_FIELD_USE` table says so in writing. `None` for a kit mass,
    which never carries one — the consumers all test `is not None`.
    """
    spec = m.get("spec") or {}
    bands = [{"h": float(b.get("h", 0.0)), "parapet": bool(b.get("parapet"))}
             for b in (spec.get("bands") or [])]
    deck = m.get("deck_z")
    return {"tag": m.get("tag", "main"), "cx": float(m["cx"]),
            "cy": float(m["cy"]), "yaw": float(m.get("yaw", 0.0)),
            "W": float(m["W"]), "D": float(m["D"]), "z0": float(m["z0"]),
            "top": float(m["top"]),
            "deck_z": (None if deck is None else float(deck)),
            "levels": [float(z) for z in (m.get("levels") or [])],
            "module": float(m.get("module", 4.0)),
            "spec": {"bands": bands}}


def mass_from_json(d):
    m = dict(d)
    m["levels"] = [float(z) for z in (d.get("levels") or [])]
    m["spec"] = {"bands": list((d.get("spec") or {}).get("bands") or [])}
    # `deck_z` round-trips as a float or stays absent; never as a string, and
    # never coerced to 0.0 — every consumer tests `is not None`, and a 0.0
    # deck would seat a roof group at grade.
    if d.get("deck_z") is not None:
        m["deck_z"] = float(d["deck_z"])
    return m


def op_to_json(op):
    e = op.get("e") or {}
    span = op.get("span") or (0.0, 0.0, 0.0, 0.0)
    d = {"fr": frame_to_json(op["fr"]),
         "side": op.get("side") or e.get("side") or "S",
         "storey": int(op.get("storey", e.get("storey", 0))),
         "mass": op.get("mass") or e.get("mass") or "main",
         "span": [float(v) for v in span],
         "e": {k: e.get(k) for k in _E_KEYS if k in e}}
    d["e"]["dead"] = bool(e.get("dead"))
    for k in _OP_SCALARS:
        if op.get(k) is not None:
            d[k] = float(op[k])
    return d


def op_from_json(d, masses):
    """Rebuild one opening record. `masses` is `{tag: mass dict}` — the mass
    is shared by every opening on it, so it is stored ONCE per bake rather
    than copied into each of a few hundred records."""
    m = masses.get(d.get("mass")) or (list(masses.values())[0] if masses
                                      else None)
    span = tuple(float(v) for v in (d.get("span") or (0.0, 0.0, 0.0, 0.0)))
    op = {"fr": frame_from_json(d["fr"]), "side": d.get("side", "S"),
          "storey": int(d.get("storey", 0)),
          "mass": d.get("mass", "main"), "span": span,
          "e": dict(d.get("e") or {}), "m": m}
    for k in _OP_SCALARS:
        if d.get(k) is not None:
            op[k] = float(d[k])
    # `_flame_sources` reads `ua/ub/va/vb` unconditionally (the `h*` reveal
    # fields fall back to them), so a record that somehow lost them is
    # rebuilt from its own span rather than raising in the middle of a build.
    op.setdefault("ua", span[0])
    op.setdefault("ub", span[1])
    op.setdefault("va", span[2])
    op.setdefault("vb", span[3])
    op.setdefault("out", -0.05)
    return op


def events_to_json(events):
    return [dict({k: (float(ev[k]) if k in ev else 0.0) for k in _EV_SCALARS},
                 id=int(ev.get("id", i)),
                 mass=ev.get("mass", "main"),
                 side=ev.get("side", "S"),
                 storey=int(ev.get("storey", 0)),
                 state=ev.get("state", "out"),
                 top=bool(ev.get("top")),
                 ops=[op_to_json(o) for o in (ev.get("ops") or [])])
            for i, ev in enumerate(events or [])]


def events_from_json(data, masses):
    out = []
    for d in (data or []):
        ev = {k: float(d.get(k, 0.0)) for k in _EV_SCALARS}
        ev.update(id=int(d.get("id", 0)), mass=d.get("mass", "main"),
                  side=d.get("side", "S"), storey=int(d.get("storey", 0)),
                  state=d.get("state", "out"), top=bool(d.get("top")),
                  ops=[op_from_json(o, masses) for o in (d.get("ops") or [])])
        out.append(ev)
    return out


def _rotate_xy(x, y, cos_t, sin_t):
    """`(x, y)` rotated CCW by the angle whose cos/sin are given, about the
    ORIGIN — the same convention `quake_flow._b_face_pt`'s own basis
    (`ca, sa = cos(yaw), sin(yaw)`) and `quake_flow._outward`
    (`math.radians(m["yaw"])`) already use, which is exactly why rotating a
    frame's origin this way and adding the same angle to its yaw reproduces
    rotating every point the frame ever generates."""
    return x * cos_t - y * sin_t, x * sin_t + y * cos_t


def place(masses, events, seats, dx, dy, yaw_deg=0.0):
    """Rotate a bake's fire geometry (and its seats) about the origin by
    `yaw_deg`, THEN move it to its cell in the assembly — the general form of
    `translate`, needed once a city assembly references a bake under a
    `rotateXYZ(0, 0, yaw_deg)` holder instead of a row's plain `x=column`
    (`urban_fire_city_plan.md` sec 4b).

    A bake is built at the ORIGIN facing its own local +Y-front convention;
    the assembly then rotates it in place to face however the city cell it
    replaces actually sits, and translates it there. A Flow emitter is
    authored at `<flow_root>/emitters/...`, which is at world origin and
    inherits NEITHER the column's rotation nor its translation, so both have
    to be baked into the FRAME ORIGINS (`op["fr"]`), the MASS CENTRES
    (`m["cx"]/["cy"]`, which the interior/roof plume seats derive from), and
    the recorded SEATS themselves — exactly what `_flame_sources`,
    `_interior_seats`/`_roof_seats` and `quake_flow._outward` each read.

    Concretely, for `θ = radians(yaw_deg)`:
      * each opening's `fr = (ox, oy, yaw, ...)`: `(ox, oy)` rotates about the
        origin and `yaw` (already RADIANS — `quake_flow._piece_frame`/
        `gac_fire.side_frame`) gets `+= θ`, so `_b_face_pt(fr, u, v, out)`
        moves exactly as if the whole façade plane had been rotated then
        translated (proved algebraically: `_b_face_pt`'s local basis vectors
        `(cos yaw, sin yaw)` / `(sin yaw, -cos yaw)` rotate to
        `(cos(yaw+θ), sin(yaw+θ))` / `(sin(yaw+θ), -cos(yaw+θ))` under the
        same rotation, so the whole point transforms consistently).
      * each mass's `(cx, cy)` rotates the same way and `m["yaw"]` (DEGREES —
        `quake_flow._outward` reads `math.radians(m["yaw"])`) gets
        `+= yaw_deg`, so the outward normal for every side rotates with it.
      * each seat's `(x, y)` rotates the same way; `z` is untouched — a yaw
        about the vertical axis never moves height.
      * `e["x"]/["y"]` are deliberately LEFT ALONE, same reason as always:
        `urban_fire._el_jitter` hashes them for each module's stable
        severity wobble, and shifting them would give the assembled building
        a different soot-to-flame relationship than the one baked into its
        own textures.

    `seats` is the sidecar's own `{"interior": [...], "roof": [...]}` (each
    entry a dict with at least `x`/`y`/`z`) or `None` — a caller with no
    seats to move (or none yet loaded) passes `None` and gets `None` back.
    Returns `(masses, events, seats)`, mutated in place.
    """
    theta = math.radians(yaw_deg)
    ca, sa = math.cos(theta), math.sin(theta)
    for m in masses.values():
        cx, cy = _rotate_xy(float(m["cx"]), float(m["cy"]), ca, sa)
        m["cx"] = cx + dx
        m["cy"] = cy + dy
        m["yaw"] = float(m.get("yaw", 0.0)) + yaw_deg
    for ev in events:
        for op in ev.get("ops") or []:
            fr = list(op["fr"])
            ox, oy = _rotate_xy(float(fr[0]), float(fr[1]), ca, sa)
            fr[0] = ox + dx
            fr[1] = oy + dy
            fr[2] = float(fr[2]) + theta
            op["fr"] = tuple(fr)
    if seats:
        for group in ("interior", "roof"):
            for seat in seats.get(group) or []:
                x, y = _rotate_xy(float(seat["x"]), float(seat["y"]), ca, sa)
                seat["x"] = x + dx
                seat["y"] = y + dy
    return masses, events, seats


def translate(masses, events, dx, dy):
    """Move a bake's fire geometry to its column in the assembly.

    A bake is built at the ORIGIN and referenced at `x = column` — but a Flow
    emitter is authored at `<flow_root>/emitters/...`, which is at world
    origin and does not inherit the column's transform. `_flame_sources`
    computes its position from `op["fr"]`, so translating the FRAME ORIGINS
    (and the mass centres, which the interior/roof plume seats use) moves
    every emitter with the building and needs no change to `urban_fire`.

    `e["x"]/["y"]` are deliberately LEFT ALONE: `urban_fire._el_jitter`
    hashes them to give each module its stable severity wobble, so shifting
    them would give the assembled building a different soot-to-flame
    relationship than the one baked into its textures.

    Translation-only special case of `place(..., yaw_deg=0.0)` — a straight
    row (`gac_fire_bench_launch_script`, `fire_assembly_launch_script`'s
    plain `x=column` layout) never rotates a bake, only moves it, and this
    keeps that call site untouched.
    """
    masses, events, _seats = place(masses, events, None, dx, dy, 0.0)
    return masses, events


# ---------------------------------------------------------------------------
# The sidecar
# ---------------------------------------------------------------------------
def sidecar(entry, fire, masses, events, bbox, top_z, seats, notes,
            timings, counts, settle_info=None, usd="", textures_dir="",
            src_kept=False, extra=None):
    """The `.json` that travels beside a baked `.usd`.

    Everything the ASSEMBLY needs and nothing it does not: the fire plan, the
    events (so the emitters go back on the right openings), the mass boxes
    (so `_outward`/`_to_world` still work), the SETTLED bbox and `top_z` (so
    the roof plume is seated on what is actually left of the building rather
    than on the pre-collapse `m["top"]` — "smoke floating over a collapsed
    building"), the interior/roof plume seats, and enough provenance to tell
    two bakes of the same asset apart.

    `extra`, if given, is a dict merged directly onto the document's TOP
    LEVEL (`doc.update(extra)`) — already how `fire_bake_launch_script.py`
    records its own `rehome`/`build_seed`/`baked_kit` info, and the same
    mechanism the city driver uses for `extra={"city": {"cell", "x", "y",
    "yaw_deg", "z", "typology", "orig_usd"}}` (`urban_fire_city_plan.md` sec
    3) — the record of which generated-city cell this bake replaces, so the
    city launcher can find its cell transform and hide the intact prim
    without re-deriving either from the manifest. Because it is a plain
    `update`, `extra` must be NAMESPACED (`{"city": {...}}`, not the city
    fields spliced in loose) or it will silently overwrite one of this
    function's own keys (`kind`, `name`, `fire`, `masses`, ...). Whatever
    lands there round-trips unchanged through `write_sidecar`/`read_sidecar`/
    `load_for_assembly` — plain JSON, no schema of its own.
    """
    doc = {
        "schema": SCHEMA,
        "kind": entry["kind"], "name": entry["name"], "level": entry["level"],
        "seed": int(entry["seed"]), "index": int(entry.get("index", 0)),
        "tag": bake_tag(entry),
        "usd": usd,
        "default_prim": DEFAULT_PRIM,
        "root": BAKE_ROOT,
        "cell": "{0}/{1}".format(BAKE_ROOT, bake_tag(entry)),
        "textures_dir": textures_dir,
        "src_kept": bool(src_kept),
        # `deck_z` IS COPIED, NOT DERIVED. `gac_fire.prepare` stashes the
        # MEASURED roof-deck height on this same `fire` dict
        # (`fire["deck_z"] = m.get("deck_z")`, and its comment explains why
        # it has to ride here rather than on the mass), and this fixed key
        # list used to drop it — so the one channel `gac_fire` built to
        # carry it ended at the sidecar and `fire_people.deck_z()` fell
        # through to `H - PARAPET_EST_M` for every roof group ever placed.
        # `None` on the kit path, whose `plan_fire` never sets the key.
        "fire": {"origin": int(fire["origin"]),
                 "storeys": [int(s) for s in fire["storeys"]],
                 "top": int(fire["top"]),
                 "sides": list(fire["sides"]),
                 "n_storeys": int(fire["n_storeys"]),
                 "mass": fire["mass"],
                 "roof": bool(fire["roof"]),
                 "level": fire["level"],
                 "state": fire.get("state"),
                 "deck_z": (None if fire.get("deck_z") is None
                            else float(fire["deck_z"])),
                 "finish": fire.get("finish")},
        "masses": {k: mass_to_json(v) for k, v in masses.items()},
        "storeys": int(fire["n_storeys"]),
        "events": events_to_json(events),
        "bbox": [float(v) for v in bbox] if bbox else None,
        "top_z": (float(top_z) if top_z is not None else None),
        "seats": seats or {"interior": [], "roof": []},
        "notes": list(notes or []),
        "timings": dict(timings or {}),
        "counts": dict(counts or {}),
        "settle": dict(settle_info or {}),
    }
    # BOTH CHANNELS, NOT ONE. `fire_people.deck_z()` looks at
    # `doc["fire"]["deck_z"]` FIRST and `masses[*]["deck_z"]` second, and
    # the second is the one a consumer reaching for a specific mass will
    # use. On the gac/dtc path the measurement only ever reaches this
    # function on `fire` — `burn_building` rebuilds `ctx["info"]["masses"]`
    # from `quake_flow.describe`, which measures a fresh box off the sliced
    # pieces and knows nothing about `deck_z` (gac_fire's own comment where
    # it sets `fire["deck_z"]` spells this out) — so back-fill it onto the
    # mass the fire is on. Never overwrite a mass that measured its own.
    _dz = doc["fire"].get("deck_z")
    if _dz is not None:
        _mt = doc["fire"].get("mass") or "main"
        _mm = doc["masses"].get(_mt) or doc["masses"].get("main")
        if isinstance(_mm, dict) and _mm.get("deck_z") is None:
            _mm["deck_z"] = float(_dz)
    if extra:
        doc.update(extra)
    return doc


def write_sidecar(path, doc):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(doc, fh, indent=1, sort_keys=True)
    os.replace(tmp, path)
    return path


def read_sidecar(path):
    with open(path) as fh:
        doc = json.load(fh)
    if int(doc.get("schema", 0)) != SCHEMA:
        print("[fire_bake] WARNING: {0} is schema {1}, this code is schema "
              "{2}".format(path, doc.get("schema"), SCHEMA))
    return doc


def load_for_assembly(json_path):
    """`(doc, masses, events)` with the events already rehydrated against the
    bake's own mass boxes — one call, so the assembler cannot forget the
    `masses` argument and silently give every opening the wrong mass."""
    doc = read_sidecar(json_path)
    masses = {k: mass_from_json(v) for k, v in (doc.get("masses") or {}).items()}
    events = events_from_json(doc.get("events") or [], masses)
    return doc, masses, events


# ---------------------------------------------------------------------------
# USD surgery (pxr only — no Kit, so the offline harness can run it too)
# ---------------------------------------------------------------------------
def strip_physics(stage, root=None, remove_prims=STRIP_PRIMS, verbose=True):
    """Take the physics off a settled bake so the assembly is static.

    `settle.bake` freezes each body's transform and sets
    `physics:rigidBodyEnabled = false` — "off, not removed: dropping the API
    would also drop the collider". That is right for the settling process and
    wrong for a shipped file: the assembly never simulates, and every applied
    `Physics*`/`Physx*` schema plus its ~22 `physics:`/`physx*` attributes is
    composition time and crate bytes for nothing (`disaster/bake.py` measured
    994 of 4,824 meshes carrying exactly this dead weight).

    Also removes the settle's own scene prims — `/World/physicsScene` and
    `/World/settleGroundPlane` — which would otherwise land in every
    reference of the bake, eight identical ground planes deep.

    Returns `{"prims", "schemas", "attrs", "removed"}`.
    """
    from pxr import Sdf, Usd

    st = {"prims": 0, "schemas": 0, "attrs": 0, "removed": 0}
    for p in (remove_prims or ()):
        prim = stage.GetPrimAtPath(Sdf.Path(p))
        if prim and prim.IsValid():
            stage.RemovePrim(Sdf.Path(p))
            st["removed"] += 1
    rp = (stage.GetPrimAtPath(Sdf.Path(root)) if root
          else stage.GetPseudoRoot())
    if not rp or not rp.IsValid():
        return st
    # ALL prims, inactive ones included: `r_gut_interior` deactivates
    # partitions and props rather than deleting them, and a deactivated prim
    # still carries its specs into the exported layer.
    for prim in Usd.PrimRange(rp, Usd.PrimAllPrimsPredicate):
        if prim.IsInstanceProxy():
            continue
        hit = False
        for name in list(prim.GetAppliedSchemas()):
            if not str(name).startswith(_PHYS_SCHEMA_PREFIX):
                continue
            try:
                prim.RemoveAppliedSchema(str(name))
                st["schemas"] += 1
                hit = True
            except Exception:
                pass
        # `GetPropertyNames()`, NOT `GetAttributes()`. A sliced GAC building
        # is tens of thousands of prims and the second builds an attribute
        # object for every property on every one of them; the names are all
        # this needs to decide, and only the matches are then materialised.
        for n in prim.GetPropertyNames():
            if not n.startswith(_PHYS_ATTR_PREFIX):
                continue
            try:
                prim.RemoveProperty(n)
                st["attrs"] += 1
                hit = True
            except Exception:
                pass
        if hit:
            st["prims"] += 1
    if verbose:
        print("[fire_bake] physics stripped: {0} prim(s), {1} API schema(s), "
              "{2} attribute(s), {3} scene prim(s) removed".format(
                  st["prims"], st["schemas"], st["attrs"], st["removed"]))
    return st


#: leaf-name prefixes that mark a prim (or its NEAREST MATCHING ANCESTOR --
#: see `_candidate_key`) as CANDIDATE debris for `deactivate_airborne`. Never
#: kit/slice STRUCTURE (wall, pier, corner, core, parapet, roof, slab_, col_,
#: beam_, deck_, roofslab_, part_, kit modules) -- structure is simply never
#: in this tuple, so it is never a candidate no matter how it sits. Order
#: matters ONLY where one entry is a prefix of another (`spallhalo` before
#: `spall`) so the more specific name wins the by-prefix summary bucket.
#: Confirmed against what `urban_fire.py`/`fire_collapse.py`/`quake_flow.py`
#: actually author as LOOSE or STATIC_EXTRA for a fire ladder, cross-checked
#: with `tools/debris_float_probe.py`'s own population table:
_CANDIDATE_PREFIXES = (
    # `fracture.fracture_prim`/`fracture_partial` fragments, made by
    # `quake_flow._break`/`_break_split`/`_break_box_like` (a broken wall
    # cell, a burnt roof lid) and directly by `urban_fire.r_fire_collapse`'s
    # `lidbrk_*` shattered lid. `fracture._write_mesh`/`fracture_prim`
    # `UsdGeom.Mesh.Define`s each fragment DIRECTLY at `frag_NNN` (no
    # wrapper Xform), so the walk below matches it at distance zero and
    # every fragment is judged on its own -- right, because `_break_split`
    # and `fracture_prim` settle loose fragments INDEPENDENTLY, not as one
    # rigid group.
    "frag", "frub",
    # `quake_flow._heap(tag="fireheap")` -- the collapse mass a burnt
    # storey drops into. Direct meshes, dense enough that a legitimate one
    # almost always finds another chunk (or the floor) right under it.
    "fireheap", "heap",
    # `_scar`/`_face_polygon(kind="spall"/"spallhalo")` -- the pale scar
    # and its dark halo on a sooted wall. THIS IS THE OLD DOCSTRING'S OWN
    # MOTIVATING CASE: "spall halos stamped on a wall that a collapse then
    # took away". `spallhalo_x` also starts with `spall`, hence the order.
    "spallhalo", "spall",
    # `r_render_peel`'s exposed-substrate patches and their scorch lips —
    # same wall-stamp class as spall since the 2026-08-31 fix anchored them
    # on measured wall relief; before this line the judge never LOOKED at a
    # peel, which is how 0.44 m floaters shipped (peel agent's flag).
    "peelhalo", "peel",
    # bearing joist stubs / roof rafter teeth left in a wall pocket after a
    # slab or roof failure (`urban_fire._joist_stubs`, the rafter-teeth
    # recipe), exposed reinforcement in a spall (`r_spall`'s `sbar_`) and
    # the earthquake-shared `rebar_` the fire palette rebinds to steel.
    "rafter", "joist", "rebar", "sbar",
    # roof + wall vent stacks (`dress_roof_urban`, `_wall_vents`).
    "vent",
    # generic quake_flow/fracture/rubble families this fire ladder does not
    # reach TODAY but shares the authoring toolkit with -- kept so a future
    # recipe that does use them is covered without another audit.
    "crack", "chunk", "debris", "rubble", "glass", "shard", "plank",
    "stick", "brick",
    # `urban_fire`-specific debris populations, confirmed by name AND by
    # `tools/debris_float_probe.py`'s own `POPULATIONS` table:
    "rdeb",       # roof-deck debris (`r_roof_scorch`)
    "sdeb",       # street debris (`r_street_debris`)
    "cwglass",    # curtain-wall glass in the fall zone (`r_curtain_burn`)
    "glit",       # sill / ground glass litter (`_sill_litter`)
    "lidbrk",     # the shattered-lid SCOPE (its children are `frag_NNN`
                  # and already match above; kept for a stray empty scope)
    "pcin",       # partial-collapse interior lumps (`fire_collapse`)
    # ROOF PLANT (`dress_roof_urban`): `ac_` is a REFERENCED prop
    # (`quake_flow._prop` composes `_AC_UNIT`'s own mesh hierarchy under
    # it, so the meshes inside carry the SOURCE asset's names, never
    # `ac_...` themselves -- exactly the "nearest ancestor Xform" case).
    # `tank`/`bulkhead`/`bulkcap`/`acpad` are single authored meshes. All
    # five are `roof_plant` in the settle -- STATIC until a breached roof
    # hands them to the solver (`_b_settle_roof_plant`, "IF THE DECK WENT,
    # SO DOES WHAT WAS BOLTED TO IT") -- so a floating one is evidence the
    # roof under it went away without taking it along. `ac_` carries the
    # trailing underscore ON PURPOSE: the housekeeping pad under the row is
    # `acpad_...`, its own separate entry below -- an unqualified "ac"
    # would have matched it too, for the wrong reason.
    "ac_", "acpad", "tank", "bulkhead", "bulkcap",
)

#: how far down/out a ray reaches before giving up. 1000 m clears any single
#: baked building in this dataset with room to spare; a miss means "ground".
_DOWN_REACH_M = 1000.0
#: a downward hit within this of the candidate's own base is "seated" --
#: resting flush on its support, not merely found one at long range.
_SEAT_TOL_M = 0.15
#: how far past its own half-extent a horizontal ray reaches to find a
#: neighbour it is leaning on.
_LEAN_MARGIN_M = 0.15
_LEAN_MIN_DIAG_M = 1.2      # smaller than this cannot lean, only hang
# WALL-ATTACHED STAMPS split into the two shapes they actually are, because
# only one of those shapes can be tested for "is the wall still there":
#
#   * `_WALL_DECAL_FAMILIES` -- FLAT: `spall`/`spallhalo` (`urban_fire.
#     _scar`/`_face_polygon`) and `crack` (`quake_flow._b_crack`) are all a
#     fan or strip of triangles whose points are EVERY ONE pushed the same
#     `out`/`proud` distance along the same face normal
#     (`urban_fire._stamp_pt`, `quake_flow._b_face_pt`) -- exactly coplanar
#     by construction, so the mesh's own points determine a real face
#     normal and a real "behind the face" direction to test.
#   * `_WALL_BAR_FAMILIES` -- ROUND: `sbar` (`r_spall`'s exposed
#     reinforcement) and `rebar` (`quake_flow._rebar_tuft`) are `_cyl`
#     cylinders. A cylinder has no single flat face to be "behind" --
#     PCA/SVD on a ring of surface points returns two comparably-sized
#     radial axes and treats its LENGTH as the thin one, which is exactly
#     the wrong axis to ray-cast along for backing. The backing test below
#     does not apply to them cleanly, so they keep the old blanket
#     exemption into the general lean check instead (BUG, 2026-08-30,
#     `sbar`/`rebar` NOT gap-tested the way `spall`/`spallhalo`/`crack` now
#     are -- accepted, they are few, small, and embedded flush in a spall
#     or roof edge their own family already backing-tests or heap-cap
#     seats).
#
# BACKING TEST (2026-08-30, fire_dtc3 bench, building b5): the blanket
# exemption this replaces let ANY spall/halo/crack through the general lean
# check no matter what -- if anything -- was actually behind it, which is
# how patches from a wall a collapse tore away rode into the export still
# ACTIVE and hanging 10-38 m in the air with nothing under or beside them
# (`spall_g5_70` measured 37.7 m up with `hit=ground`, `contact=False` --
# the judge already said "not contact", the exemption just never asked).
# See `_flat_normal`/`_wall_backing_contact` and their use in
# `_judge_candidates` below.
_WALL_DECAL_FAMILIES = {"spall", "spallhalo", "crack", "peel", "peelhalo"}
_WALL_BAR_FAMILIES = {"sbar", "rebar"}
_WALL_ATTACHED = _WALL_DECAL_FAMILIES | _WALL_BAR_FAMILIES
#: how close real (non-stamp) geometry must sit behind a flat wall stamp,
#: along the stamp's OWN normal, to count as "the wall is still there".
_BACKING_MAX_M = 0.35
_RAY_TOL = 1e-6


def _match_prefix(name):
    """The `_CANDIDATE_PREFIXES` entry `name` starts with, or None."""
    for pre in _CANDIDATE_PREFIXES:
        if name.startswith(pre):
            return pre
    return None


def _candidate_key(prim, root_path):
    """Is `prim` (a Mesh) part of a candidate, and if so which ONE?

    Walks from the mesh itself upward. A mesh whose OWN name matches (every
    directly-authored debris mesh in this codebase: `_box`/`_cyl`/`_a_lump`/
    `fracture._write_mesh` all `Define` the geometry straight at the named
    path) is its own candidate. A mesh that does not match but sits under an
    ancestor that does -- a referenced prop's internal geometry, e.g. an
    `ac_*` condenser's own mesh hierarchy, named by the SOURCE asset and
    never by the debris convention -- is grouped under that ancestor, so the
    whole referenced unit is judged (and, if it fails, deactivated) as the
    one rigid body it actually is. Returns the matched prim's path as a
    string, or None (never a candidate -- includes `root` itself and every
    kit/slice structure prim, which by construction match nothing here).
    """
    p = prim
    while p and p.IsValid():
        path = p.GetPath()
        if path == root_path:
            return None
        if _match_prefix(path.name) is not None:
            return str(path)
        p = p.GetParent()
    return None


def _ray_hit_points(locator, ipts, icells, p0, p1, owner, own_of_cell):
    """`[(world point, owner id), ...]` for every hit on `p0->p1` NOT owned
    by `owner` -- the owner id lets a caller say WHOSE triangle it hit
    (`_judge_candidates` resolves it back to a prim path for `support_path`,
    which is what makes `tools/airborne_probe.py`'s spot-check possible).

    `ipts`/`icells` are scratch `vtkPoints`/`vtkIdList` the caller reuses
    across thousands of rays -- allocating a fresh pair per ray is the kind
    of per-call VTK object overhead that turns a sub-second sweep into a
    slow one. `IntersectWithLine(p0, p1, tol, points, cellIds)` collects
    EVERY intersection along the segment (not just the nearest), which is
    what makes self-exclusion possible in one call: a single-hit query has
    no way to say "skip that one, keep looking" without re-casting from
    just past it.
    """
    ipts.Reset()
    icells.Reset()
    locator.IntersectWithLine(p0, p1, _RAY_TOL, ipts, icells)
    out = []
    for i in range(icells.GetNumberOfIds()):
        cid = icells.GetId(i)
        o = int(own_of_cell[cid])
        if o == owner:
            continue
        out.append((ipts.GetPoint(i), o))
    return out


def _owner_prefix(owner_path, o):
    """The `_CANDIDATE_PREFIXES` entry the prim behind ownership id `o`
    matches, or `"?"` if it is not a candidate at all (a structure mesh) or
    `o` is `None`. Shared by the lean check and the backing test below so
    "what family is on the other end of this hit" is answered one way."""
    from pxr import Sdf

    opath = owner_path.get(o) if o is not None else None
    if not opath:
        return "?"
    return _match_prefix(Sdf.Path(opath).name) or "?"


def _flat_normal(pts):
    """Unit normal of a (near-)planar point cloud, or `None` if `pts` is
    too small or too degenerate (e.g. every point coincident) to fit one.

    Every point a wall stamp authors (`urban_fire._stamp_pt`, `quake_flow.
    _b_face_pt`) is pushed the SAME distance along the SAME face normal, so
    the points are exactly coplanar and the LEAST-varying axis of the
    centred cloud -- the smallest-singular-vector of its SVD -- IS that
    normal. This depends on nothing but the points themselves: not
    triangle winding, not world axes, not the wall's yaw, so it is exactly
    as good on a diagonal or corner wall as on an axis-aligned one.
    """
    import numpy as np

    if pts is None or len(pts) < 3:
        return None
    centered = pts - pts.mean(axis=0)
    try:
        _u, _s, vt = np.linalg.svd(centered, full_matrices=False)
    except Exception:
        return None
    n = vt[-1]
    norm = float(np.linalg.norm(n))
    if norm < 1e-9:
        return None
    return n / norm


def _wall_backing_contact(locator, ipts, icells, gid, own_of_cell,
                          owner_path, pts, reach_m=_BACKING_MAX_M):
    """Is there real (non-stamp) geometry within `reach_m` of the point
    cloud `pts`, along `pts`'s OWN flat normal, on either side of it?

    `pts` is the candidate's own world-space points (a list of per-mesh
    arrays, or `None` if none were kept for it -- see `_judge_candidates`'
    mesh-load loop). One ray each way along the fitted normal, starting at
    the cloud's centroid (a point ON the plane): a hit is only backing if
    it is NOT another wall stamp (`_owner_prefix(...) in _WALL_ATTACHED` --
    a decal cannot back a decal, same rule the lean check applies to
    "leaning" support) -- the candidate's OWN triangles are already
    excluded by `_ray_hit_points`'s owner check regardless of where on the
    ray they'd fall.

    Returns `(backed, distance, path)` -- `path` (mirrors `_judge_
    candidates`' own `support_path`) is `owner_path` resolved for whatever
    the WINNING ray actually hit, so a caller can tell a real kit wall
    (`.../wall_x_0_00_0000`) from a hit that should never have been able to
    back anything, like a merged source mesh still composed (invisible)
    under `<cell>/src` at judge time. `distance`/`path` are both `None`
    when nothing qualified within `reach_m`, including when `pts` was too
    degenerate to fit a normal to at all -- always treated as UNbacked,
    same as "nothing found".
    """
    import numpy as np

    if pts is None:
        return False, None, None
    P = np.vstack(pts) if isinstance(pts, list) else np.asarray(pts)
    n = _flat_normal(P)
    if n is None:
        return False, None, None
    c = P.mean(axis=0)
    best, best_owner = None, None
    for sign in (1.0, -1.0):
        d = n * sign
        p0 = (float(c[0]), float(c[1]), float(c[2]))
        p1 = (float(c[0] + d[0] * reach_m), float(c[1] + d[1] * reach_m),
              float(c[2] + d[2] * reach_m))
        hits = _ray_hit_points(locator, ipts, icells, p0, p1, gid,
                               own_of_cell)
        for pt, o in hits:
            if _owner_prefix(owner_path, o) in _WALL_ATTACHED:
                continue                    # a decal cannot back a decal
            dist = float(np.linalg.norm(np.array(pt) - c))
            if best is None or dist < best:
                best, best_owner = dist, o
    backed = best is not None and best <= reach_m + 1e-9
    path = owner_path.get(best_owner) if backed and best_owner is not None else None
    return backed, best, path


def _judge_candidates(stage, root, gap_m=1.0, verbose=False):
    """The real-geometry analysis behind `deactivate_airborne` -- everything
    short of actually flipping `active`, so a caller (the function itself,
    or `tools/airborne_probe.py`'s dry run) can inspect the judgement before
    anything is mutated.

    REWRITTEN (2026-08-30) off real geometry, because the axis-aligned-box
    version had two failures the user's GAC review actually hit:

      1. "Seated" meant SOME OTHER PRIM'S BOX overlaps the candidate's
         footprint and contains its bottom z. A sliced building piece's box
         is not the piece -- an L-shaped corner's box covers the whole
         square it turns, and a `core` piece's box spans the entire plan --
         so any debris floating inside the building's envelope passed as
         seated on top of a piece it was nowhere near.
      2. Only prims with a bounding diagonal under 3 m were even looked
         at, so a hanging slab fragment bigger than that was invisible to
         the test regardless of how far it floated.

    What replaces both: candidates are chosen by NAME (`_candidate_key`,
    never kit/slice structure), and every ACTIVE mesh under `root` --
    candidate or not -- is triangulated into one world-space `vtkPolyData`
    (`soot_bake.triangles` fan-triangulates; each triangle is tagged with an
    OWNER id: the candidate's group id for a candidate's own triangles, a
    unique id for everything else, so "ignore hits on the candidate's own
    triangles" is a single array lookup per hit) and wrapped in a
    `vtkStaticCellLocator` (measured on the largest bake, 7,396 active
    meshes / 414,538 triangles: 0.05 s to build vs. 0.07 s for
    `vtkCellLocator` and 0.92 s for `vtkOBBTree` -- `vtkStaticCellLocator`
    chosen for being faster on both counts). For each candidate: five
    downward rays (footprint centroid + the 4 points at 25/75% of its xy
    extent) find the highest non-self hit below it (`support`, or the
    ground at z=0 if none); `gap = bottom_z - support`. Four horizontal
    rays from the centroid, reaching `half_extent + _LEAN_MARGIN_M` past
    each side, catch a piece LEANING on a neighbour rather than resting
    under one -- except a flat wall stamp (`_WALL_DECAL_FAMILIES`:
    `spall`/`spallhalo`/`crack`), which instead gets `_wall_backing_contact`
    (two rays along its OWN fitted face normal, not the world's cardinal
    directions) since "leaning" was never the right question for something
    stamped flush on a wall in the first place -- and a wall bar
    (`_WALL_BAR_FAMILIES`: `sbar`/`rebar`, round, no single face to fit a
    normal to) which keeps the plain size exemption into the horizontal-ray
    check instead. A candidate is judged for deactivation when `gap > gap_m`
    and none of the downward gap itself (`<= _SEAT_TOL_M`, i.e. already
    flush), the horizontal lean rays, or the backing test found contact.

    Returns `{"n_meshes", "n_triangles", "load_s", "build_s", "ray_s",
    "locator", "judged"}` -- `judged` is `[{"path", "prefix", "bottom_z",
    "support_z", "support_path", "gap", "contact", "backing_m",
    "backing_path", "deactivate"}, ...]`, one entry per candidate GROUP (see
    `_candidate_key`). `support_path` is the path of whatever prim the
    WINNING downward ray actually hit (None if no ray hit anything -- the
    candidate is falling all the way to the assumed ground at z=0).
    `backing_m`/`backing_path` are the distance and prim path
    `_wall_backing_contact` found real backing at (both None for anything
    that never ran that test, including a `_WALL_DECAL_FAMILIES` candidate
    already `contact` from its downward ray alone, and for a genuinely
    unbacked one). `{"n_meshes": 0, "judged": []}` (no `locator`) when `root`
    is missing, vtk will not import, or nothing in the subtree matches a
    candidate name.
    """
    from pxr import Sdf, Usd, UsdGeom

    from . import fracture

    empty = {"n_meshes": 0, "n_triangles": 0, "load_s": 0.0, "build_s": 0.0,
             "ray_s": 0.0, "locator": None, "judged": []}
    rp = stage.GetPrimAtPath(Sdf.Path(root))
    if not rp or not rp.IsValid():
        return dict(empty)
    if not fracture.ensure_vtk(verbose=verbose):
        if verbose:
            print("[fire_bake] deactivate_airborne: vtk unavailable -- "
                  "skipping (0 deactivated)")
        return dict(empty)

    import time

    import numpy as np
    import vtk
    from vtk.util import numpy_support as ns

    from . import soot_bake as sb

    t0 = time.time()
    root_path = rp.GetPath()
    xf = UsdGeom.XformCache()

    all_pts, all_tris, tri_owner = [], [], []
    groups = {}                 # key(str) -> {"gid", "lo", "hi"}
    group_id_of_key = {}
    owner_path = {}              # owner id -> the prim path RESPONSIBLE for
                                 # it (the candidate group's own key, or a
                                 # structure mesh's own path) -- lets a hit
                                 # be resolved back to a name for reporting.
    npts_total, n_meshes, n_tris = 0, 0, 0

    for p in Usd.PrimRange(rp):
        if not p.IsA(UsdGeom.Mesh) or not p.IsActive():
            continue
        # INVISIBLE GEOMETRY IS NOT SUPPORT. At bake time the merged source
        # subtree (`<cell>/src`, hidden by `slice_to_kit`) is still on the
        # stage, so every ray inside the building envelope hit the intact
        # original and the sweep under-flagged by 33-114 prims per building
        # (fire_row3, 2026-08-30) — the export drops the source and the same
        # prims then hung in the air.
        try:
            if UsdGeom.Imageable(p).ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
        except Exception:
            pass
        mesh = UsdGeom.Mesh(p)
        pts = mesh.GetPointsAttr().Get()
        if not pts:
            continue
        counts = mesh.GetFaceVertexCountsAttr().Get()
        idx = mesh.GetFaceVertexIndicesAttr().Get()
        if not counts or not idx:
            continue
        n_meshes += 1
        M = np.array(xf.GetLocalToWorldTransform(p), dtype=float)
        W = np.asarray(pts, dtype=float) @ M[:3, :3] + M[3, :3]

        key = _candidate_key(p, root_path)
        if key is not None:
            gid = group_id_of_key.get(key)
            if gid is None:
                gid = len(group_id_of_key)
                group_id_of_key[key] = gid
                # Only the flat-decal families need their own point cloud
                # kept around for `_wall_backing_contact`'s normal fit --
                # every other candidate (a sliced building piece among
                # them, thousands of triangles) would make this a real
                # memory cost for a group that will never look at it.
                pfx0 = _match_prefix(Sdf.Path(key).name)
                groups[key] = {"gid": gid, "lo": None, "hi": None,
                              "pts": [] if pfx0 in _WALL_DECAL_FAMILIES
                                     else None}
            g = groups[key]
            lo, hi = W.min(0), W.max(0)
            g["lo"] = lo if g["lo"] is None else np.minimum(g["lo"], lo)
            g["hi"] = hi if g["hi"] is None else np.maximum(g["hi"], hi)
            if g["pts"] is not None:
                g["pts"].append(W)
            owner = gid
            owner_path[owner] = key
        else:
            owner = -n_meshes         # unique, never equals a group id (>= 0)
            owner_path[owner] = str(p.GetPath())

        tri, _tf, _ts = sb.triangles(counts, idx)
        if len(tri):
            all_tris.append(tri + npts_total)
            tri_owner.append(np.full(len(tri), owner, dtype=np.int64))
            n_tris += len(tri)
        all_pts.append(W)
        npts_total += len(W)

    if not groups:
        out = dict(empty)
        out["n_meshes"], out["n_triangles"] = n_meshes, n_tris
        out["load_s"] = time.time() - t0
        return out

    V = np.vstack(all_pts) if all_pts else np.zeros((0, 3))
    F = (np.vstack(all_tris) if all_tris else
         np.zeros((0, 3), dtype=np.int64))
    own_of_cell = (np.concatenate(tri_owner) if tri_owner else
                  np.zeros((0,), dtype=np.int64))
    t1 = time.time()

    vpts = vtk.vtkPoints()
    vpts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(V), deep=True))
    ca = vtk.vtkCellArray()
    if len(F):
        cells = np.hstack(
            [np.full((len(F), 1), 3, dtype=np.int64), F]).ravel()
        ca.ImportLegacyFormat(ns.numpy_to_vtkIdTypeArray(cells, deep=True))
    pd = vtk.vtkPolyData()
    pd.SetPoints(vpts)
    pd.SetPolys(ca)

    locator = vtk.vtkStaticCellLocator()
    locator.SetDataSet(pd)
    locator.BuildLocator()
    t2 = time.time()

    ipts, icells = vtk.vtkPoints(), vtk.vtkIdList()
    judged = []
    for key, g in groups.items():
        gid, lo, hi = g["gid"], g["lo"], g["hi"]
        cx, cy, cz = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])
        dx, dy = hi[0] - lo[0], hi[1] - lo[1]
        bottom_z = float(lo[2])

        support, support_owner = None, None
        # A PLATE IS HELD AT ITS EDGES. Five interior points are right for a
        # chip or a prop; a roof-deck rim fragment spanning the plan is held
        # by the walls along its PERIMETER, and its interior rays see the
        # open floor twenty metres down — the sweep deleted the whole roof
        # deck of every collapse-level GAC building as "unsupported"
        # (fire_row3, 2026-08-30: parapet ring left, floor plate exposed).
        # Edge midpoints and corners, inset a little, join the sample set.
        ex = min(0.15, 0.1 * dx) / max(dx, 1e-6)
        ey = min(0.15, 0.1 * dy) / max(dy, 1e-6)
        samples = [(0.5, 0.5), (0.25, 0.25), (0.25, 0.75), (0.75, 0.25),
                   (0.75, 0.75), (0.5, ey), (0.5, 1.0 - ey), (ex, 0.5),
                   (1.0 - ex, 0.5), (ex, ey), (ex, 1.0 - ey), (1.0 - ex, ey),
                   (1.0 - ex, 1.0 - ey)]
        for fx, fy in samples:
            x, y = lo[0] + fx * dx, lo[1] + fy * dy
            # START ABOVE THE CANDIDATE'S OWN BOTTOM, not below it: a prop
            # sitting exactly flush on its neighbour (bulkcap on bulkhead,
            # a condenser on its pad) has the neighbour's top face AT
            # `bottom_z`, and a ray that starts 2 cm under it never hits
            # it — "floating" by 0.00 m (roof-seat probe, 2026-08-30). Own
            # triangles are excluded by owner id, so starting 5 cm up costs
            # nothing.
            p0 = (float(x), float(y), bottom_z + 0.05)
            p1 = (float(x), float(y), bottom_z - _DOWN_REACH_M)
            hits = _ray_hit_points(locator, ipts, icells, p0, p1, gid,
                                   own_of_cell)
            for pt, o in hits:
                z = float(pt[2])
                if support is None or z > support:
                    support, support_owner = z, o
        support_z = 0.0 if support is None else support
        support_path = (owner_path.get(support_owner)
                        if support_owner is not None else None)
        gap = bottom_z - support_z
        contact = gap <= _SEAT_TOL_M

        # ONLY SOMETHING BIG ENOUGH TO LEAN GETS TO LEAN. A torn slab or a
        # wall panel can rest against a wall; a 0.3 m chip touching a façade
        # is hanging on it — the trail of black chips down the side of
        # SM_Building_23 F5c beside the lost corner (fire_row3d, 2026-08-30).
        diag = float(((hi[0] - lo[0]) ** 2 + (hi[1] - lo[1]) ** 2
                      + (hi[2] - lo[2]) ** 2) ** 0.5)
        pfx = _match_prefix(Sdf.Path(key).name) or "?"
        backing_m = backing_path = None
        if not contact and pfx in _WALL_DECAL_FAMILIES:
            # A FLAT WALL STAMP IS ATTACHED, NOT LEANING -- but only when
            # the wall it was stamped on is still actually there. Replaces
            # the old blanket exemption (see `_WALL_DECAL_FAMILIES`'s own
            # comment for the bug it let through) with a real backing test:
            # cast along the stamp's OWN fitted normal instead of the
            # world's 4 cardinal directions, which is what makes this right
            # on a diagonal or corner wall too.
            contact, backing_m, backing_path = _wall_backing_contact(
                locator, ipts, icells, gid, own_of_cell, owner_path,
                g.get("pts"))
        elif not contact and (diag >= _LEAN_MIN_DIAG_M
                              or pfx in _WALL_BAR_FAMILIES):
            # `sbar`/`rebar` are exempted from the size floor above because
            # they are round bars stamped/tufted flush and tiny — the size
            # rule threw 100+ of them off every damaged wall (row 3e,
            # 2026-08-30) — but (unlike the flat decals above) they get no
            # dedicated backing test of their own; see `_WALL_BAR_FAMILIES`'s
            # comment for why.
            hx, hy = dx / 2.0 + _LEAN_MARGIN_M, dy / 2.0 + _LEAN_MARGIN_M
            for ddx, ddy, reach in ((1.0, 0.0, hx), (-1.0, 0.0, hx),
                                    (0.0, 1.0, hy), (0.0, -1.0, hy)):
                p0 = (float(cx), float(cy), float(cz))
                p1 = (float(cx + ddx * reach), float(cy + ddy * reach),
                      float(cz))
                # A DECAL DOES NOT HOLD UP A DECAL. A spall patch and its halo
                # are co-located, so each "leaned" on the other and both hung
                # 0.6-15 m outside the wall their module lost (SM_Building_23
                # F5c, fire_row3f). Lean support must come from something
                # that is not itself a wall stamp.
                hits = _ray_hit_points(locator, ipts, icells, p0, p1, gid,
                                       own_of_cell)
                for _pt, o in hits:
                    if _owner_prefix(owner_path, o) not in _WALL_ATTACHED:
                        contact = True
                        break
                if contact:
                    break

        judged.append({
            "path": key, "prefix": pfx,
            "bottom_z": bottom_z, "support_z": float(support_z),
            "support_path": support_path, "gap": float(gap),
            "contact": bool(contact), "backing_m": backing_m,
            "backing_path": backing_path,
            "deactivate": bool(gap > gap_m and not contact),
        })
    t3 = time.time()

    return {"n_meshes": n_meshes, "n_triangles": n_tris,
            "load_s": t1 - t0, "build_s": t2 - t1, "ray_s": t3 - t2,
            "locator": locator, "judged": judged}


#: `deactivate_airborne` re-judges until a pass removes nothing new, or
#: gives up and ships whatever is left rather than loop forever on some
#: pathological mutual-support cycle that should not exist but should never
#: be allowed to hang a bake either.
_AIRBORNE_MAX_PASSES = 12


def deactivate_airborne(stage, root, gap_m=1.0, verbose=True, **_ignored):
    """Deactivate every candidate mesh under `root` that hangs in the air.

    A thin wrapper over `_judge_candidates` (read ITS docstring for the
    method): this flips `active` on what it judged airborne, prints the
    by-prefix summary the old box-based version also printed, and returns
    the count -- the only part of the contract the caller
    (`fire_bake_launch_script.py`, `settle_info["airborne_off"] =
    fb.deactivate_airborne(stage, fb.BAKE_ROOT)`, no extra args) depends on.

    ITERATES TO A FIXED POINT (2026-08-31; fire_dtc3 bench, `gac_
    SM_Building_26_F4_s162`, 14 `spall`/`spallhalo` still ACTIVE in the
    export that `tools/airborne_probe.py` immediately re-flagged on the
    same file, cold). `_judge_candidates` judges every candidate from ONE
    snapshot of "what's currently active", then this function used to flip
    `SetActive(False)` on the losers in a single batch -- so a candidate
    whose only "seat" (the plain downward ray has NO family exclusion, only
    self-exclusion) or "backing" was ANOTHER candidate that this SAME batch
    was about to turn off got to keep a contact that would not survive the
    save. MEASURED: `sbar_g5_20` (exposed rebar, authored INSIDE the very
    spall region it belongs to -- `urban_fire.r_spall`, so a spall and its
    exposed bar are meant to be centimetres apart) sat 4.2 cm under
    `spallhalo_g5_16` and 4.5 cm under `spall_g5_17` -- close enough that
    the downward ray called them "seated", `contact=True`, before either
    branch below ever got a look. `sbar_g5_20` had no support of its own
    (same wall gone), so the SAME pass judged it airborne too and switched
    it off -- correctly, but a batch too late for the two stamps that had
    just leaned on it. The fix does not touch a single per-candidate rule:
    it rebuilds the WHOLE locator from the now-smaller active set and
    re-judges everyone still standing, exactly what a cold re-open already
    does, so a live bake converges to the same answer instead of needing a
    second pass from `tools/airborne_probe.py --fix` after the fact.
    Removals only ever shrink the active set, so each pass can only find
    the same or MORE floaters than the last -- monotone, so this cannot
    oscillate, only stop improving (`_AIRBORNE_MAX_PASSES` is a hang guard,
    not a real limit: measured convergence on every bake so far is 2 passes,
    the second one finding zero).
    """
    from pxr import Sdf

    # A FAMILY THAT IS MOSTLY "AIRBORNE" IS NOT AIRBORNE — its floor is
    # missing. 6,186 `fireheap` chips of SM_Building_09 F6 sat over nothing
    # because the storey they landed on had no slab (fire_row1, 2026-08-30);
    # the answer was to author the floor (`r_fire_collapse`), never to
    # delete the pile ("we want the debris to be there", user). So if more
    # than FAMILY_CAP of a family's judged members would go, none of them
    # does and the bake says so loudly — that is a pipeline bug to fix
    # upstream, not stray debris.
    #
    # THE CAP PROTECTS PILES, NOT PROPS. A pile family legitimately needs a
    # floor under it -- `fireheap`/`heap` (the collapse mass), `frub` (floor
    # rubble), `sdeb` (street debris), `glit` (sill/ground glass litter) --
    # and "most of them are airborne" there really is evidence of a missing
    # slab upstream. A `frag`/`rafter`/`joist`/`bulkhead`/`bulkcap`/`acpad`/
    # `vent`/`tank`/`spall`/`spallhalo` family is never that: each member is
    # either its own rigid fragment (a shattered roof lid, a spall) or one
    # individually-placed roof item, so a large share of one floating IS
    # stray debris and capping there hid it -- 35/121 unsupported `frag`
    # roof-lid pieces at deck height over SM_Building_23's F4 burn-through
    # hole, silently "left alone" by this same cap (row-2 review,
    # 2026-08-30). Restricting the cap to the pile allowlist means every
    # other family always deactivates when judged, cap or no cap. The cap
    # is re-evaluated FRESH every pass, from that pass's own judged counts
    # -- correct, since a pass that just removed a pile's neighbours can
    # change whether the pile itself still looks mostly-unsupported.
    PILE_FAMILIES = {"fireheap", "heap", "frub", "sdeb", "glit"}
    FAMILY_CAP = 0.25

    total_gone, total_by_kind = 0, {}
    for pass_n in range(1, _AIRBORNE_MAX_PASSES + 1):
        info = _judge_candidates(stage, root, gap_m=gap_m,
                                 verbose=verbose and pass_n == 1)
        judged_by = {}
        for j in info["judged"]:
            if j["prefix"] not in PILE_FAMILIES:
                continue
            judged_by.setdefault(j["prefix"], [0, 0])
            judged_by[j["prefix"]][0] += 1
            judged_by[j["prefix"]][1] += 1 if j["deactivate"] else 0
        skip = {k for k, (n, g) in judged_by.items()
                if n >= 20 and g > FAMILY_CAP * n}
        for k in sorted(skip):
            n, g = judged_by[k]
            print("[fire_bake] WARNING: {0}/{1} `{2}` prim(s) have no "
                  "support — that is a missing floor, not stray debris; "
                  "leaving the family alone".format(g, n, k))
        gone, by_kind = [], {}
        for j in info["judged"]:
            if not j["deactivate"] or j["prefix"] in skip:
                continue
            gone.append(j["path"])
            by_kind[j["prefix"]] = by_kind.get(j["prefix"], 0) + 1

        for key in gone:
            stage.GetPrimAtPath(Sdf.Path(key)).SetActive(False)
        total_gone += len(gone)
        for k, v in by_kind.items():
            total_by_kind[k] = total_by_kind.get(k, 0) + v
        if verbose:
            print("[fire_bake] airborne check pass {0}: {1} candidate(s) "
                  "judged, {2} deactivated{3}".format(
                      pass_n, len(info["judged"]), len(gone),
                      ": " + ", ".join(
                          "{0} {1}".format(v, k) for k, v in
                          sorted(by_kind.items(), key=lambda kv: -kv[1]))
                      if gone else ""))
        if not gone:
            break
    else:
        print("[fire_bake] WARNING: airborne check still finding new "
              "floaters after {0} passes — stopping anyway; the export is "
              "not guaranteed floater-free".format(_AIRBORNE_MAX_PASSES))

    if verbose:
        print("[fire_bake] airborne check: {0} pass(es), {1} total "
              "deactivated{2}".format(
                  pass_n, total_gone,
                  ": " + ", ".join(
                      "{0} {1}".format(v, k) for k, v in
                      sorted(total_by_kind.items(), key=lambda kv: -kv[1]))
                  if total_gone else ""))
    return total_gone


def _under(path, prefixes):
    s = str(path)
    return any(s == p or s.startswith(p.rstrip("/") + "/") for p in prefixes)


def bound_materials(stage, root):
    """`{material path: (material, [bindable prims])}` under `root`.

    Asks the SUBSETS and MESHES what they are bound to
    (`ComputeBoundMaterial`) rather than reaching into whoever bound them —
    the same decoupling `tools/bake_gac_kits.py` chose, and the reason this
    works for a live slice, a baked kit and a kit building alike.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    out = {}
    rp = stage.GetPrimAtPath(Sdf.Path(root))
    if not rp or not rp.IsValid():
        return out
    for prim in Usd.PrimRange(rp, Usd.PrimAllPrimsPredicate):
        if not (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Subset)):
            continue
        mat = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
        if not mat or not mat.GetPrim().IsValid():
            continue
        key = str(mat.GetPrim().GetPath())
        rec = out.setdefault(key, (mat, []))
        rec[1].append(prim)
    return out


def depends_on(mat_prim, doomed):
    """Does this material's COMPOSITION reach into a doomed subtree?

    Two ways it can, and only the first is visible from the prim's own path:

      * the material prim itself lives under `<cell>/src` (a live
        `slice_to_kit` binds the source's own material prims), or
      * it is a `soot_plume.piece_material_like` copy — a prim of its own
        under `<cell>/SootLooks` or `<cell>/FireLooks` whose whole network is
        an `AddInternalReference` INTO the source subtree, with only the
        diffuse map overridden on top. Its path says nothing; its
        composition arcs say everything.
    """
    from pxr import Usd

    if _under(mat_prim.GetPath(), doomed):
        return True
    try:
        q = Usd.PrimCompositionQuery(mat_prim)
        for arc in q.GetCompositionArcs():
            node = arc.GetTargetNode()
            if node and _under(node.path, doomed):
                return True
    except Exception:
        pass
    # A reference whose target is ALREADY GONE (a bake reopened after its
    # source subtree was dropped) composes to nothing, so the query above
    # never lists it — the authored list-op still names the path.
    try:
        refs = mat_prim.GetMetadata("references")
        for r in (refs.GetAddedOrExplicitItems() if refs else []):
            if not r.assetPath and _under(r.primPath, doomed):
                return True
    except Exception:
        pass
    return False


def prune_orphan_materials(stage, root, doomed, verbose=True):
    """Remove every Material under `root` that reaches into `doomed` and is
    bound by NOTHING.

    `_bind_soot` makes a `piece_material_like` copy per subset it bakes; a
    later pass can then rebind that subset elsewhere (`gac_fire.darken_glass`
    blacks out band glass AFTER the per-piece soot bake) and the copy is left
    behind unbound. `rehome_for_export` only sees BOUND materials, so twelve
    such orphans rode into `gac_SM_Building_24_F2`'s export (2026-08-30) with
    an internal reference into the dropped `<cell>/src` — nothing rendered
    wrong, but every open of the file logged twelve unresolved-reference
    errors and `verify_export` (rightly) refused it. Returns the count.
    """
    from pxr import Sdf, Usd, UsdShade

    doomed = tuple(doomed or ())
    rp = stage.GetPrimAtPath(Sdf.Path(root))
    if not doomed or not rp or not rp.IsValid():
        return 0
    bound = set(bound_materials(stage, root).keys())
    gone = []
    for prim in Usd.PrimRange(rp, Usd.PrimAllPrimsPredicate):
        if not prim.IsA(UsdShade.Material):
            continue
        key = str(prim.GetPath())
        if key in bound or _under(prim.GetPath(), doomed):
            continue
        if depends_on(prim, doomed):
            gone.append(prim.GetPath())
    for pth in gone:
        stage.RemovePrim(pth)
    if verbose and gone:
        print("[fire_bake] pruned {0} orphan material(s) that still pointed "
              "into {1} (unbound sooted copies)".format(len(gone), "/".join(doomed)))
    return len(gone)


def local_texture_override(mat_prim):
    """`(relative shader path, input name, texture path)` of the LOCAL file a
    material overrides its base map with, or `(None, None, None)`.

    That override is the soot: `piece_material_like` composes the kit's own
    material and swaps exactly one input for a PNG on local disk. Everything
    else in the network (normal, roughness, AO) comes from the source, which
    is what keeps a sooted module lit like its untouched neighbour rather
    than like a flat OmniPBR rectangle.
    """
    from pxr import Sdf, Usd, UsdShade

    if not mat_prim or not mat_prim.IsValid():
        return None, None, None
    root = mat_prim.GetPath()
    for p in Usd.PrimRange(mat_prim):
        sh = UsdShade.Shader(p)
        if not sh:
            continue
        for inp in sh.GetInputs():
            try:
                v = inp.Get()
            except Exception:
                continue
            if not isinstance(v, Sdf.AssetPath):
                continue
            s = v.path or v.resolvedPath or ""
            if not s or "://" in s:
                continue
            if not os.path.isabs(s):
                continue
            return p.GetPath().MakeRelativePath(root), inp.GetBaseName(), s
    return None, None, None


def rehome_for_export(stage, root, doomed, looks_scope, verbose=True):
    """Re-anchor every material bound under `root` that depends on `doomed`.

    THE FAILURE THIS PREVENTS IS SILENT. Export a sliced GAC building with
    `<cell>/src` gone and every piece renders WHITE with its geometry and its
    UVs perfectly intact — `gac_slice.rehome_materials`' own docstring, and
    this repo has been burned there once already.

    For each such material: find the material's OWN usd file
    (`gac_slice._material_source` — measured: every GreatAmericanCity
    material is a separate `Materials/*_Inst.usd`), define a fresh prim under
    `looks_scope` referencing it, re-apply the local PNG override if there
    was one, and rebind every subset that was on the old prim.

    Returns `{"needed", "rehomed", "failed", "rebound", "failed_paths"}`.
    A non-zero `failed` means the caller MUST NOT strip `doomed`: shipping a
    white building is worse than shipping an invisible source subtree.
    """
    from pxr import Sdf, UsdGeom, UsdShade
    from detail import gac_slice as gsl

    doomed = tuple(doomed or ())
    st = {"needed": 0, "rehomed": 0, "failed": 0, "rebound": 0,
          "failed_paths": [], "pruned": 0}
    if not doomed:
        return st
    binds = bound_materials(stage, root)
    todo = [(k, v) for k, v in binds.items() if depends_on(v[0].GetPrim(), doomed)]
    st["needed"] = len(todo)
    if not todo:
        st["pruned"] = prune_orphan_materials(stage, root, doomed, verbose=verbose)
        if verbose:
            print("[fire_bake] no material under {0} depends on {1} — nothing "
                  "to rehome".format(root, "/".join(doomed)))
        return st
    UsdGeom.Scope.Define(stage, Sdf.Path(looks_scope))
    made = {}
    for key, (mat, targets) in todo:
        mp = mat.GetPrim()
        ident, pp = gsl._material_source(mp)
        rel, inp_name, tex = local_texture_override(mp)
        if not ident:
            st["failed"] += 1
            st["failed_paths"].append(key)
            continue
        # NAME BY SOURCE, NOT BY THE OLD PRIM'S NAME. Every GAC section calls
        # its material `UnrealMaterial`, and the sooted copies are `m0, m1,
        # ...` — either way two different materials collapse onto one prim.
        # The file stem plus a hash of the source path is unique and stable.
        stem = _safe(os.path.basename(ident).rsplit(".", 1)[0]) or "mat"
        h = hashlib.md5(("{0}|{1}|{2}".format(key, ident, tex or ""))
                        .encode("utf-8")).hexdigest()[:8]
        dst = Sdf.Path("{0}/{1}_{2}".format(looks_scope, stem, h))
        new = made.get(str(dst))
        if new is None:
            prim = stage.DefinePrim(dst)
            ok = (prim.GetReferences().AddReference(ident, Sdf.Path(pp))
                  if pp is not None else
                  prim.GetReferences().AddReference(ident))
            if not ok:
                st["failed"] += 1
                st["failed_paths"].append(key)
                continue
            if rel is not None and inp_name:
                # THE SOOT IS ONE INPUT. Re-apply it at the same RELATIVE
                # shader path inside the freshly referenced network — the
                # network is identical, only its anchor moved.
                target = dst.AppendPath(rel) if str(rel) not in (".", "") else dst
                sp = stage.GetPrimAtPath(target)
                sh = UsdShade.Shader(sp) if sp and sp.IsValid() else None
                if sh:
                    i = sh.GetInput(inp_name) or sh.CreateInput(
                        inp_name, Sdf.ValueTypeNames.Asset)
                    i.Set(Sdf.AssetPath(str(tex)))
                else:
                    print("[fire_bake] WARNING: {0}: could not re-apply the "
                          "soot map at {1} — the piece keeps its clean "
                          "texture".format(key, target))
            new = UsdShade.Material.Get(stage, dst)
            if not new:
                st["failed"] += 1
                st["failed_paths"].append(key)
                continue
            made[str(dst)] = new
            st["rehomed"] += 1
        for t in targets:
            UsdShade.MaterialBindingAPI(t).Bind(new)
            st["rebound"] += 1
    # The old prims are now unbound and still point into the doomed subtree;
    # leave one behind and reopening the file logs a composition error per
    # piece. Drop them.
    if not st["failed"]:
        for key, (mat, _t) in todo:
            p = Sdf.Path(key)
            if not _under(p, doomed) and stage.GetPrimAtPath(p).IsValid():
                stage.RemovePrim(p)
    # ...and the copies nothing binds any more (see `prune_orphan_materials`)
    st["pruned"] = prune_orphan_materials(stage, root, doomed, verbose=verbose)
    if verbose:
        print("[fire_bake] rehomed {0}/{1} material(s) onto {2}, rebound {3} "
              "binding(s){4}".format(st["rehomed"], st["needed"], looks_scope,
                                     st["rebound"],
                                     "" if not st["failed"] else
                                     "  *** {0} FAILED — keeping the source "
                                     "subtree ***".format(st["failed"])))
    return st


# ---------------------------------------------------------------------------
# Cold verification: does the exported file stand on its own?
# ---------------------------------------------------------------------------
def _diffuse_textures(stage):
    """`[(material path, texture)]` for every material on the stage."""
    from pxr import Usd, UsdShade
    from . import soot_plume as spl

    out = []
    for prim in Usd.PrimRange.Stage(stage, Usd.PrimAllPrimsPredicate):
        if not prim.IsA(UsdShade.Material):
            continue
        _sp, _inp, tex = spl.find_basecolor(prim)
        out.append((str(prim.GetPath()), tex or ""))
    return out


def verify_export(usd_path, doomed=("/src",), expect_root=BAKE_ROOT,
                  check_remote=True, verbose=True):
    """Open a bake COLD and check the four things that fail silently.

      1. **Every material's base map resolves.** A local PNG that is not on
         disk and a Nucleus URL that 404s both render as an untextured
         building, which is indistinguishable from an art bug.
      2. **Nothing still points into the doomed subtree.** The material trap
         again: a binding or a composition arc into `<cell>/src` is a piece
         that will be white the moment the source is not composed.
      3. **No physics.** An applied `Physics*`/`Physx*` schema means the
         assembly is not static.
      4. **No Flow.** Smoke belongs to the assembly, not to a bake.

    Runs on the bare-USD harness (`tools/usd_python.sh`) — no Kit, no
    `SimulationApp`, safe beside a live session.
    """
    from pxr import Sdf, Usd, UsdGeom, UsdShade

    info = {"path": usd_path, "ok": False,
            "size_mb": round(os.path.getsize(usd_path) / 1e6, 2)
            if os.path.exists(usd_path) else 0.0}
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        info["error"] = "stage would not open"
        return info
    info["default_prim"] = (str(stage.GetDefaultPrim().GetPath())
                            if stage.GetDefaultPrim() else "")
    prims = list(Usd.PrimRange.Stage(stage, Usd.PrimAllPrimsPredicate))
    info["prims"] = len(prims)
    info["meshes"] = sum(1 for p in prims if p.IsA(UsdGeom.Mesh))
    info["materials"] = sum(1 for p in prims if p.IsA(UsdShade.Material))
    info["has_root"] = bool(expect_root and
                            stage.GetPrimAtPath(Sdf.Path(expect_root)).IsValid())

    # 1) textures
    missing, remote, checked = [], [], 0
    for mpath, tex in _diffuse_textures(stage):
        if not tex:
            continue
        checked += 1
        if "://" in tex:
            remote.append(tex)
            continue
        if not os.path.exists(tex):
            missing.append((mpath, tex))
    info["textures_checked"] = checked
    info["textures_missing"] = missing[:12]
    info["n_textures_missing"] = len(missing)
    info["n_textures_remote"] = len(remote)
    info["remote_unreachable"] = []
    if check_remote and remote:
        try:
            import omni.client
            seen = set()
            for u in remote:
                if u in seen:
                    continue
                seen.add(u)
                res = omni.client.stat(u)[0]
                if res != omni.client.Result.OK:
                    info["remote_unreachable"].append(u)
        except Exception as exc:
            info["remote_check_error"] = str(exc)

    # 2) the doomed subtree
    stray = []
    for p in prims:
        if _under(p.GetPath(), doomed) or any(
                str(p.GetPath()).endswith(d) for d in doomed):
            stray.append(str(p.GetPath()))
    info["doomed_prims"] = stray[:12]
    info["n_doomed_prims"] = len(stray)
    # A COMPOSITION QUERY PER PRIM IS TOO EXPENSIVE on a sliced building
    # (tens of thousands of prims), and it is not needed: the arcs that can
    # point into a dropped subtree are the MATERIALS' internal references
    # (`piece_material_like`) and anything else that authored a reference of
    # its own. Everything else composes from its parent.
    unresolved = []
    for p in prims:
        if not (p.IsA(UsdShade.Material) or p.HasAuthoredReferences()
                or p.HasAuthoredPayloads()):
            continue
        try:
            q = Usd.PrimCompositionQuery(p)
            for arc in q.GetCompositionArcs():
                node = arc.GetTargetNode()
                if node and _under(node.path, doomed):
                    unresolved.append("{0} -> {1}".format(p.GetPath(), node.path))
        except Exception:
            continue
    info["doomed_arcs"] = unresolved[:12]
    info["n_doomed_arcs"] = len(unresolved)

    # 3) physics, 4) flow
    phys = [str(p.GetPath()) for p in prims
            if any(str(s).startswith(_PHYS_SCHEMA_PREFIX)
                   for s in p.GetAppliedSchemas())]
    info["n_physics_prims"] = len(phys)
    info["physics_prims"] = phys[:8]
    flow = [str(p.GetPath()) for p in prims
            if str(p.GetTypeName()).startswith(_FLOW_TYPES)]
    info["n_flow_prims"] = len(flow)
    info["flow_prims"] = flow[:8]

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                           useExtentsHint=False)
    try:
        r = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
        if not r.IsEmpty():
            lo, hi = r.GetMin(), r.GetMax()
            info["bbox"] = [round(float(v), 2) for v in
                            (lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])]
    except Exception as exc:
        info["bbox_error"] = str(exc)

    # 5) THE SETTLE ACTUALLY CAME TO REST. Not a property of the exported
    # USD — a body frozen mid-flight is ordinary geometry by the time it
    # gets here, which is exactly why this check has to read the sidecar and
    # why `BAKE VERIFY OK` printed over a bake that had just deleted 80
    # bodies for still moving (`city_smoke43`, 2026-08-31). Only
    # `still_moving` gates the verdict: the box-based `below_grade` count is
    # a known over-report (`settle._z_min`) and the frozen kit bake carries
    # 59 of them by design, so failing on that would fail every kit record.
    side = os.path.splitext(usd_path)[0] + ".json"
    st_info = {}
    if os.path.exists(side):
        try:
            with open(side) as fh:
                st_info = (json.load(fh) or {}).get("settle") or {}
        except Exception as exc:                   # pragma: no cover
            info["settle_read_error"] = str(exc)
    info["settle"] = st_info
    info["n_still_moving"] = int(st_info.get("still_moving") or 0)
    info["settle_converged"] = st_info.get("converged")

    info["ok"] = (info["meshes"] > 0 and not missing
                  and not info["remote_unreachable"]
                  and not stray and not unresolved
                  and not phys and not flow
                  and not info["n_still_moving"])
    if verbose:
        report_verify(info)
    return info


def report_verify(info):
    print("=" * 72)
    print("BAKE VERIFY {0}  {1}".format(
        "OK " if info.get("ok") else "*** PROBLEM ***", info.get("path")))
    print("  {0} MB, {1} prim(s), {2} mesh(es), {3} material(s), defaultPrim "
          "{4}".format(info.get("size_mb"), info.get("prims"),
                       info.get("meshes"), info.get("materials"),
                       info.get("default_prim")))
    if info.get("bbox"):
        b = info["bbox"]
        print("  bbox        {0:.1f} x {1:.1f} x {2:.1f} m".format(
            b[3] - b[0], b[4] - b[1], b[5] - b[2]))
    print("  textures    {0} checked, {1} missing, {2} remote, {3} "
          "unreachable".format(info.get("textures_checked"),
                               info.get("n_textures_missing"),
                               info.get("n_textures_remote"),
                               len(info.get("remote_unreachable") or [])))
    for mp, t in (info.get("textures_missing") or []):
        print("    MISSING   {0}  <- {1}".format(t, mp))
    for u in (info.get("remote_unreachable") or [])[:6]:
        print("    UNREACHED {0}".format(u))
    print("  source      {0} prim(s), {1} arc(s) still in the doomed "
          "subtree".format(info.get("n_doomed_prims"),
                           info.get("n_doomed_arcs")))
    for p in (info.get("doomed_prims") or [])[:6]:
        print("    STRAY     {0}".format(p))
    for a in (info.get("doomed_arcs") or [])[:6]:
        print("    ARC       {0}".format(a))
    if info.get("settle"):
        st = info["settle"]
        print("  rest        {0} still moving, {1} below grade (boxes), {2} "
              "clamped, converged={3}, stop={4}".format(
                  st.get("still_moving"), st.get("below_grade"),
                  st.get("clamped"), st.get("converged"),
                  st.get("stop_reason")))
        if info.get("n_still_moving"):
            print("    NOT AT REST  {0} body(s) were frozen mid-flight and "
                  "DELETED from this export -- the settle did not converge"
                  .format(info["n_still_moving"]))
    print("  physics     {0} prim(s) still carry a Physics*/Physx* schema"
          .format(info.get("n_physics_prims")))
    for p in (info.get("physics_prims") or [])[:4]:
        print("    PHYSICS   {0}".format(p))
    print("  flow        {0} Flow prim(s) (a bake must have none)".format(
        info.get("n_flow_prims")))
    print("=" * 72)
    return info


# ---------------------------------------------------------------------------
# Host-side self-check
# ---------------------------------------------------------------------------
def check(verbose=True):
    """The schema round-trips and the manifest parses. No USD, no Kit."""
    problems = []
    try:
        ents = parse_manifest(
            "gac:SM_Building_02:F1 kit:commercial_mid:F5c::S,E", base_seed=7)
    except Exception as exc:
        return ["fire_bake: manifest parse raised {0}".format(exc)]
    if len(ents) != 2:
        problems.append("fire_bake: manifest parsed to {0} entries, not 2"
                        .format(len(ents)))
    if ents[1]["seed"] != 7 + 31:
        problems.append("fire_bake: default seed for entry 1 is {0}, not {1}"
                        .format(ents[1]["seed"], 7 + 31))
    if ents[1]["sides"] != ("S", "E"):
        problems.append("fire_bake: sides parsed as {0!r}".format(
            ents[1]["sides"]))
    if verbose and not problems:
        print("[fire_bake] check OK")
    return problems
