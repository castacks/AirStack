"""kinds — one class per disaster type, and the stage hooks each one fills in.

WHAT THIS IS FOR
----------------
Everything a disaster type decides used to be a branch somewhere else. The
archetype baker asked `if self.disaster == "earthquake"` to pick a damage
script and `fire=(self.disaster == "fire")` to decide whether a tree comes out
charred; the fire's emitters were authored by a launch script and by nothing
else, so a fire scene could not be baked at all while an earthquake scene
could. The pipeline was staged for one disaster and improvised for the other.

A `Disaster` is that decision set, in one place, with a subclass per type:

    field()             the SHAPE of the damage — which `disaster.field`
                        class, built from the compiled config.
    ladder()            the rungs an asset can land on (`levels.LADDERS`).
    damage_archetype()  STAGE A. How one clean instance is wrecked before it
                        is exported — by rung where the type has a damage
                        script, by intensity where it does not.
    chars_vegetation    STAGE A. Whether a felled tree comes out charred.
    bake_stage_b()      STAGE B. Anything the type authors onto the finished
                        scene that is not a placement. Fire's emitters are the
                        only one today, and they are why this hook exists.
    attach_runtime()    STAGE C. Anything a loaded scene has to be told that
                        the USD could not carry. Fire's carb settings, again
                        the only one.
    place_targets()     STAGE C. Who is in the scene to be found, and where.
                        Config-driven for every type (`targets.py`), so no
                        subclass overrides it — the hook exists so a type that
                        needs a different population model has somewhere to
                        put it.

Stage A and Stage B themselves stay type-agnostic: `archetypes/bake.py` and
`generate_scene.py` call the hooks and never name a disaster.

WHAT IS DELIBERATELY NOT HERE
-----------------------------
`compile_disaster.py` still owns what a severity means for each type — the
knob values, and which field kind and parameters the preset gets. That is
config compilation and it runs on the host with no stage; this is behaviour at
stage time. Splitting them the other way would put a table of numbers behind an
import of `pxr`.

WHICH SCRIPT WRECKS A BUILDING is not restated here either. That binding lives
in `mesh_damage.DAMAGE_SCRIPTS`, because the live damage path dispatches off
it too, and a disaster that wrecked buildings one way at bake time and another
way at scene time would produce a library that does not match the scenes it is
placed into. `Disaster.damage_script` reads that registry; it does not shadow
it.
"""

import os

from disaster import levels
from disaster.field import make_damage_field


def _pack_materials(config):
    """`scene_generator.asset_materials`, memoised on the config it came from.

    Stage A asks per archetype and the walk covers the whole `usds` tree.
    """
    got = getattr(_pack_materials, "_cache", None)
    if got is not None and got[0] is config:
        return got[1]
    from scene_generator import asset_materials
    table = asset_materials(config)
    _pack_materials._cache = (config, table)
    return table


class Disaster:
    """The general form. Subclasses override only what differs.

    The base is not abstract: an unknown type degrades to "generic damage, no
    extra stages", which is the same forgiving default `levels.NONE_LADDER`
    gives a typo'd ladder — a bake that has already cost an hour should not die
    on a name.
    """

    name = "none"

    #: The `disaster.field.kind` this type uses when a hand-written config does
    #: not name one. Compiled presets always name one, so this is a fallback
    #: rather than the authority — see `field`.
    default_field_kind = "uniform"

    #: Whether this event chars what it fells. Only a fire does; every other
    #: type uses the same felling geometry with no soot on it.
    chars_vegetation = False

    # -- where it hit ------------------------------------------------------
    def field(self, dis_cfg: dict, region: tuple):
        """The :class:`~disaster.field.DamageField` for a compiled config.

        The config's `field.kind` wins. This is not a formality: fire's
        compiled field is a `radial` scar even though `EllipseField` exists and
        is what the front actually is, and quietly substituting the ellipse
        here would move every building's damage level.
        """
        cfg = dict((dis_cfg or {}).get("field") or {})
        cfg.setdefault("kind", self.default_field_kind)
        return make_damage_field(cfg, region)

    # -- how hard ----------------------------------------------------------
    def ladder(self, kind: str = levels.STRUCTURE):
        return levels.ladder(self.name, kind)

    def level_at(self, intensity: float, kind: str = levels.STRUCTURE):
        return levels.level_at(self.name, intensity, kind)

    # -- Stage A -----------------------------------------------------------
    @property
    def damage_script(self):
        """``(module, function)`` for this type's own damage script, or None.

        Read from `mesh_damage.DAMAGE_SCRIPTS` rather than restated per
        subclass: the live path (`mesh_damage.damage_one`) dispatches off that
        registry, and a second copy here is exactly how a disaster could end up
        wrecking buildings one way at bake time and another way at scene time.
        """
        from disaster import mesh_damage as md

        return md.DAMAGE_SCRIPTS.get(self.name)

    def damage_archetype(self, stage, prim, level, *, seed, config, intensity,
                         source=""):
        """Wreck one clean instance for the archetype library. Stage A, step 3b.

        BY RUNG WHERE A SCRIPT EXISTS, by intensity where one does not. A
        script is written in terms of kinds of failure over regions of the plan
        (its ``RUNG_PLAN``), so it takes the level name and composes the
        failure the name means — one wing sheared off, the ground floor gone
        under another, the rest cracked. Everything else goes through the
        asset-generic operators (subdivide, solidify, delete_faces,
        value_noise), which is what an arbitrary Nucleus or Objaverse building
        needs, at *intensity*.

        *intensity* is the MIDPOINT of the rung's band, not its threshold —
        the threshold is where the rung begins, so damaging at it would render
        every archetype at the gentlest damage its name permits. See
        `archetypes.bake.Baker._intensity`.

        The twin of `mesh_damage.damage_one`, which does the same dispatch for
        the LIVE path where only an intensity is known and the rung has to be
        quantised out of it.
        """
        import importlib

        from disaster import mesh_damage as md

        dis = config.get("disaster") or {}
        # `heading_deg` is the scene's, for the storm types: every archetype in
        # the library has to fail on the same face, or a street assembled from
        # it points its wreckage in four directions. Scripts and operators with
        # no bearing accept and ignore it.
        heading = md._heading_of(dis)
        # WHAT THE ARCHETYPE IS MADE OF — THE ASSET'S OWN CONSTRUCTION FIRST.
        # The locale material (`compile_disaster.LOCALE_MATERIAL`) is the right
        # answer for a suburb, where every house is timber frame, and the wrong
        # one for a downtown block, where no two buildings are built alike. It
        # used to be the only answer here, so every archetype in the library
        # was cut as masonry however the pack described it: a glass-and-steel
        # tower shed brick at every break, and the floors `fill_interior`
        # authors inside it came out clad in brick too. The LIVE path has read
        # the pack per asset since the material table landed; Stage A has to do
        # the same lookup, because the wall thickness and the fragment shape
        # are baked into the geometry and cannot be corrected at placement.
        mcfg = dis.get("mesh_damage") or {}
        kind = md.material_for_asset(source, mcfg.get("material"),
                                     _pack_materials(config))
        mat = md.material(kind)
        # SAY IT, so a wrong material is visible in the bake log rather than
        # only in the finished scene. The live path prints the same thing on
        # its per-building line.
        print(f"[stage-a]     material: {kind}"
              + (f" (pack says {os.path.basename(str(source))})" if source else ""),
              flush=True)
        # THE SAME TESSELLATION LEVER THE LIVE PATH HAS. Stage A is where it
        # bites hardest — an archetype carries its subdivision to disk and a
        # scene references it dozens of times — and until this it reached only
        # `mesh_damage.apply_to_stage`, so `SCENE_SUBDIVIDE_M` on a bake was a
        # no-op. `shatter`'s own default is 4.0; the config may lower it and
        # the env var may only coarsen it.
        sub = mcfg.get("subdivide") or {}
        edge = md.subdivide_edge_m(float(sub.get("max_edge_m", 4.0))
                                   if sub.get("enabled", True) else 0.0)

        script = self.damage_script
        if script is not None:
            # `settle_it=False`: Stage A settles the WHOLE grid in one PhysX
            # pass at the end (`Baker.settle`), so a per-cell settle here would
            # run the solver once per archetype and then again over everything.
            rep = getattr(importlib.import_module(script[0]), script[1])(
                stage, prim, level, seed=seed, settle_it=False,
                material=kind, heading_deg=heading, max_edge_m=edge)
        else:
            thick = mcfg.get("thickness") or {}
            rep = md.damage_building(
                stage, prim, self.name, intensity, seed=seed,
                wall_m=float(thick.get("wall_m", mat.wall_m)), grain=mat.grain,
                core_material_kind=kind, heading_deg=heading,
                max_edge_m=edge)
        # ON THE REPORT, so the caller does not repeat the lookup. Stage A
        # needs it again to decide what the wreck sheds — a break exposes the
        # structure, and the rubble around the base has to be the same
        # substance as the cut faces above it (`disaster/debris.py`).
        rep["material"] = kind
        return rep

    # -- Stage B -----------------------------------------------------------
    def bake_stage_b(self, stage, config, placements, scene_scale_factor=1.0):
        """Author what this type adds to a finished scene. Returns a report.

        Runs after `apply_placements`, so every placement has a `prim_path` and
        a real prim to measure. Most types add nothing: their whole effect is
        in the placement list and in the archetypes it references.
        """
        return {}

    # -- Stage C -----------------------------------------------------------
    def attach_runtime(self, stage):
        """Tell a loaded scene what the USD could not carry. Returns a count."""
        return 0

    def place_targets(self, stage, config, placements=None, layout=None,
                      resolver=None, parent_path="", scene_scale_factor=1.0):
        """Populate the loaded scene with targets. Returns the victim list.

        SPEC's Stage C step 2 — "place targets (usually human victims), noting
        their ground truth locations". The model lives in `targets.py` and is
        driven entirely by the compiled `targets:` block, so a type with no
        cohort weights (everything but earthquake, today) places nobody and
        this costs it nothing.

        Deliberately NOT a Stage B hook: victims must not be baked into the
        scene USD, or the same city could not be re-populated for a second
        search trial.
        """
        import targets

        return targets.place(stage, config, placements=placements,
                             layout=layout, resolver=resolver,
                             parent_path=parent_path,
                             scene_scale_factor=scene_scale_factor)


class Earthquake(Disaster):
    """Shaking. Fails a building a storey at a time.

    Its damage script is `disaster.quake`, bound through
    `mesh_damage.DAMAGE_SCRIPTS` — see `Disaster.damage_script`.
    """

    name = "earthquake"
    default_field_kind = "radial"


class Fire(Disaster):
    """A conflagration: chars what it does not consume, and keeps burning.

    The only type with work in every stage. Stage A bakes charred structures
    and torched trees; Stage B authors the Flow emitters and their schedule;
    Stage C switches the renderer on.
    """

    name = "fire"
    default_field_kind = "radial"
    chars_vegetation = True

    def bake_stage_b(self, stage, config, placements, scene_scale_factor=1.0):
        """The Flow rig, its emitters, and the burn as timeSamples.

        WHY THIS IS STAGE B AND NOT STAGE C. The emitters are geometry-derived
        — one sphere fitted to each fuel prim, on a seeded schedule solved in
        closed form from the same elliptical front the damage field uses. None
        of that needs a running sim, so none of it belongs at load time; what
        genuinely cannot bake is the voxel solve, and that is Flow's business
        at render time whether the rig came from a bake or from a script.
        """
        from disaster import fire

        cfg = (config.get("disaster") or {}).get("fire") or {}
        if not cfg.get("enabled", True):
            return {}
        return fire.bake_emitters(stage, placements, cfg,
                                  scene_scale_factor=scene_scale_factor)

    def attach_runtime(self, stage):
        from disaster import fire

        return fire.attach_runtime(stage)


class Tornado(Disaster):
    """Wind along a corridor. Strips before it breaks.

    Its damage script is `disaster.tornado`, bound the same way.
    """

    name = "tornado"
    default_field_kind = "path"


class Hurricane(Disaster):
    """Broad and even — no edge to speak of."""

    name = "hurricane"
    default_field_kind = "uniform"


class Flood(Disaster):
    """Water displaces and stains; it rarely takes a structure down."""

    name = "flood"
    default_field_kind = "radial"


#: One instance per type. They are stateless, so sharing them is free and the
#: identity is useful — `get("fire") is get("fire")`.
_REGISTRY = {d.name: d for d in (Disaster(), Earthquake(), Fire(), Tornado(),
                                 Hurricane(), Flood())}


def get(disaster_type) -> Disaster:
    """The :class:`Disaster` for a type name. Unknown names get the base.

    Accepts a compiled config dict as well as a name, because most callers have
    one and reaching for `config["disaster"]["type"]` at every site is how a
    `None` disaster block turns into a KeyError halfway through a bake.
    """
    if isinstance(disaster_type, dict):
        dis = disaster_type.get("disaster")
        disaster_type = (dis or {}).get("type") if isinstance(dis, dict) \
            else disaster_type.get("type")
    return _REGISTRY.get(str(disaster_type or "none").lower(),
                         _REGISTRY["none"])


def names() -> list:
    """Every registered type, `none` last — it is the absence of one."""
    return sorted(n for n in _REGISTRY if n != "none") + ["none"]
