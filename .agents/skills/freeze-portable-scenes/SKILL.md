---
name: freeze-portable-scenes
description: >-
  Freeze or bake a scene so it still WORKS on another machine. The three defects
  that shipped silently in all 18 cells of the first frozen dataset and cost a
  full 8-drone benchmark sweep — no sky (the cell renders black), build-machine
  absolute asset paths (which a Nucleus-anchored layer resolves AGAINST THE
  SERVER, so copying files to local disk can never fix it), and cross-scope
  material bindings (which a prim-by-prim consumer drops, rendering geometry
  untextured grey). Why none of them is visible from the build machine, why a
  finished cell CANNOT be repaired afterwards (no USD build can re-serialise a
  kit-written crate), the `make_portable` pre-flatten pass and the `verify()`
  gate that now blocks all three, and the load-side repairs for cells already
  shipped. Read before touching `disaster/freeze.py`, `disaster/bake.py`,
  `fire_bake.py`, any `*_bake_*_launch_script.py`, or the frozen-cell branch of
  `example_multi_drone_scene_import.py`.
---

# Freezing a scene that works somewhere else

## The one-sentence version

A frozen cell is not a picture of your stage — it is a **contract with a machine
you have never seen**, and everything the build machine supplies implicitly (its
files, its sky, its single composed stage) has to be made explicit before the
flatten, because after the flatten it is too late forever.

## Status

`disaster/freeze.py` gained `make_portable()` (pre-flatten repair) and three new
`verify()` gates on 2026-08-30, after all 18 cells of `final_disaster_dataset`
were found to have all three defects. The shipped cells are NOT fixed — they
cannot be (see "Why you cannot repair a finished cell"); they are worked around
on the load side and by staging assets on Nucleus.

---

## The three defects

Every one of them shipped in all 18 cells. Every one of them reports success at
build time. Every one is invisible in the `snaps/` you review.

### 1. The cell renders black — no sky travelled

**Measured.** `Sdf` over a shipped cell, 83,215 prims:

```
every Fire cell     ONE light: SphereLight r=0.25 m at (0,0,2.5), intensity 1e5
every Tornado cell  ZERO lights
```

A 25 cm bulb 2.5 m off the ground, over a 1 km plate. Downstream, in the
2026-08-30 benchmark: the 2048² overhead frame was **99.93 % pure RGB(0,0,0)** —
261,970 of 262,144 marker cells exactly black — and the only non-black pixels
were the Flow flames, the one emissive thing left.

**Cause, and it is a two-part trap.** `DEACTIVATE_DEFAULT` turns off
`/World/Environment` and `/World/stage/Environment` for good reasons (the
collector would drag the default env's assets along; the default ground plane
would z-fight the generated plat). But:

* the sky and sun LIVE in that prim, so deactivating it removes the lighting;
* **Kit's flatten DROPS deactivated prims entirely** rather than writing them
  inactive. Measured: **0 of 83,215 prims in a shipped cell carry
  `active=False`.** So the comment in `_reference_frozen_scene` claiming the
  flatten "leaves them in the file as DEACTIVATED prims" is wrong, and the
  `ReviewCamera` probe just below it is dead code — neither path exists.

**The same trap, inverted.** `DEACTIVATE_DEFAULT` listed `/World/GroundPlane`,
but Pegasus' default env composes under `/World/stage`, so the real paths are
`/World/stage/GroundPlane` and `/World/stage/SphereLight`. Neither matched. The
freeze therefore **removed the one thing worth keeping and kept both things it
was trying to drop.** Both paths are now in the list.

**"Has a light" is the wrong check.** A cell with exactly one light still
rendered black. Only a **DomeLight** (ambient everywhere) or a **DistantLight**
(parallel rays, no falloff) lights a plate; a SphereLight's 1/r² is nothing at
100 m, let alone at the 500 m plate edge. `verify()` gates on `sky_lights`, not
`lights`.

### 2. Build-machine asset paths — and why staging files CANNOT fix it

**Measured.** Across all 18 cells: **332 distinct absolute paths** under
`/isaac-sim/AirStack/scene_gen/assets/` (272 png / 34 jpg / 26 mdl); 248 in a
single Level-1 cell. All three trees are git-ignored — `aec/*` (.gitignore:112),
`objaverse/*` (107), `materials/scorched/` (143) — so a fresh clone has 1 tracked
file under `aec`, 1 under `objaverse`, 380 of 13,164 under `materials`.

**The part that is genuinely counter-intuitive.** The obvious fix is "copy the
files onto the consumer's disk". It does not work, and here is the proof: 659
files were staged onto a pod at 21:23, a run started at 21:31, and it still
logged **505 not-found lines between 21:33 and 21:35** for files that were
present, non-zero and root-readable the entire time.

The reason, measured with `omni.client.combine_urls` — the same combination the
USD resolver performs:

```
anchor:      omniverse://<host>:443/Projects/SEI-COA/final_disaster_dataset/.../cell.usd
asset path:  /isaac-sim/AirStack/scene_gen/assets/materials/burn/Burn_Scorch.png
COMBINES TO: omniverse://<host>:443/isaac-sim/AirStack/scene_gen/assets/materials/burn/Burn_Scorch.png
```

**An absolute path inside a layer anchored on `omniverse://` resolves against
that SERVER.** Local disk is never consulted. This is also exactly why it always
works on the build box — there the cell is opened as a local file, so the same
path anchors locally, and every test passes.

An explicit `omniverse://` URL is the only form that resolves the same way from
every machine and every anchor. That is what `make_portable` writes.

### 3. Cross-scope material bindings — the grey burn scar

**Measured.** All 12 `/World/burnGround/band_*` bind to
`/World/stage/generated/BurnLooks/band_*`; `/World/burnGround/BurnLooks` does not
exist. Tornado cells: 11 `scourGround` bands, same shape.

That is fine on one stage. But a consumer that references `/World/<name>`
prim-by-prim — which `_reference_frozen_scene` must do, to keep the cell's
`PhysicsScene` out — puts them in **different composition arcs**, and USD drops a
relationship whose target is outside its own arc:

```
The relationship target </World/stage/generated/BurnLooks/band_0> from
</World/burnGround/band_0.material:binding> ... refers to a path outside the
scope of the reference from </World/burnGround>. Ignoring.
```

Binding dropped → no material → **default grey**. The user's report was "the
burnt floor looks grey, something is wrong", and this was it.

---

## Why you cannot repair a finished cell

Do not plan on fixing a shipped cell by rewriting it. **No USD build available
can re-serialise a kit-written crate.** Both Isaac 5.1's `omni.usd.libs` pxr and
a clean `usd-core` 26.08 raise on `Sdf.Layer.Export`:

```
Sdf_CrateFile::CrateFile::_UnpackValue :
  'Attempted to unpack unsupported type enum value 0'
```

READING is fine — 83,215 prims, 810,233 properties, 14,307 asset attributes and
534 unique asset paths all enumerate cleanly, and that is how every measurement
in this document was taken. It is writing that fails. `UsdUtils.ModifyAssetPaths`
fails for the same reason (it walks every field), and so does
`UsdUtils.ComputeAllDependencies` — which is why `verify()` now wraps it in
try/except and computes the portability verdict from a plain shader-attribute
walk instead.

This is the same poisoned `assetInfo` that stalls `omni.kit.usd.collect` and that
forces `disaster.bake` to flatten with Kit and slice with USD.

**Therefore every repair must happen on the LIVE STAGE, BEFORE the flatten.**
There the shader prims are ordinary prims and setting an attribute is cheap.

---

## What the freeze now does

`freeze.make_portable(stage)` runs in `export_scene` immediately after
`deactivate_prims` and before `export_as_stage_async`:

1. rewrites every shader asset path under `ASSET_LOCAL_PREFIX` to `ASSET_MIRROR`;
2. copies any material bound from outside a `/World/<scope>` INTO
   `<scope>/Looks` and rebinds, so the binding is valid under any composition;
3. authors `/World/FrozenDome` + `/World/FrozenSun` so the cell is lit by
   construction, without bringing back the default env's flat ground.

Disable with `export_scene(..., portable=False)` only to reproduce an old cell.

## What the gate now blocks

`verify()` returns three new fields and `ok` is False if any fails:

| field | fails when | renders as |
|---|---|---|
| `sky_lights` | no DomeLight or DistantLight | black |
| `build_local` | any absolute non-`omniverse://` path outside the cell dir | untextured |
| `cross_scope_bindings` | a `/World/<scope>` binding points outside its scope | grey |

`report()` prints all three every time, pass or fail.

**Run against a shipped cell it correctly refuses:**

```
ok: False | lights: 1 | sky_lights: 0 | build_local: 102 | cross_scope: 12
```

### The hole this closes

`ok` used to ignore `external` entirely whenever `expect_self_contained=False`,
which is the normal `collect=False` case. That lumped two very different things
together:

* `omniverse://...` — **portable.** Resolves identically from any machine and
  any anchor. Fine in a non-collected cell.
* `/abs/local/path` — **fatal, always.** Only the build machine has it, and
  because of the anchoring rule above it cannot even be rescued by copying files
  to the consumer.

That single missing distinction is how 248 build-machine paths per cell shipped
unnoticed through a verification step whose whole job was to catch them.

---

## For cells already shipped

Three load-side repairs, all in the frozen branch of
`example_multi_drone_scene_import.py`. They are workarounds; a re-freeze is the
real fix.

* `_light_frozen_scene` — deactivates stray point lights (radius-gated, so a real
  fill light survives), borrows the same sky the generated path uses
  (`shared.yaml`'s `RetroNeighborhood.stage.usd`), and adds a sun ONLY if the sky
  did not bring a DistantLight. Logs a census; says `<NONE — the scene will
  render BLACK>` if it ever regresses.
* `_rebind_frozen_overlays` — re-authors cross-scope bindings after composition,
  where both prims are ordinary stage paths. Verified 12 on fire, 11 on tornado.
* `_rebase_local_assets` — rewrites build-machine paths to the mirror. **Always
  rewrites when the cell came from a URL** (`_FROZEN_URL`), because a local file
  existing proves nothing when the resolver is asking a server.

**Know its limit.** It runs in `_finish_frozen_scene`, i.e. AFTER composition —
measured at 21:46:26 against errors spanning 21:44:14–21:46:11. Zero errors occur
after it and the count stays static, but it **cannot prevent** the load-time
failures. The durable stopgap for existing cells is to put the closure on Nucleus
at the path the resolver actually computes:

```
omniverse://<host>:443/isaac-sim/AirStack/scene_gen/assets/...
```

659 files (332 referenced + each MDL's own folder, since an `.mdl` resolves its
textures relative to itself), copied server-to-server. Tools:
`scene_gen/tools/stage_frozen_assets.py`, and `rebase_frozen_assets.py` for the
rewrite that only works pre-flatten.

---

## The checklist for any new bake or freeze

1. **Never let a build-machine absolute path into a shipped file.** Rewrite to a
   URL before the flatten, or run the collect.
2. **Author the light explicitly.** Do not rely on a default environment
   surviving — deactivating it deletes it.
3. **Keep a material in the same `/World/<scope>` as the geometry that binds it**,
   or expect any prim-by-prim consumer to drop the binding.
4. **Deactivate by MEASURED path, not by assumed path.** Print what actually
   matched. Two of the four entries in `DEACTIVATE_DEFAULT` silently matched
   nothing for the entire life of the dataset.
5. **Verify on the artefact, not the stage.** Open the written file cold and ask
   what a stranger's machine would see.
6. **Test the failure, not just the success.** The check that would have caught
   all of this is one line: does `ok` go False on a cell you know is broken?

## Related

`freeze-dataset-state` (what is in the dataset today), `freeze-disaster-dataset`
(the matrix and directory contract), `benchmark-disaster-dataset` (what a run
does with a cell), `slice-buildings-into-kits` (the same Kit-flatten / USD-slice
split and the same poisoned `assetInfo`).

## 2026-09-02 — the fourth defect: sliced pieces lose their source materials

`verify()` catches build-machine paths, missing sky and cross-scope bindings.
It does NOT catch this one, because the binding is not broken in a way a path
check can see: a sliced building's piece is bound to a Material prim that is
itself a REFERENCE out to a per-material asset, and in the exported file that
prim composes to a bare TYPELESS placeholder. `GetPrimAtPath(target).IsValid()`
returns True; `ComputeBoundMaterial()` returns nothing; the piece renders flat
fallback grey. Measured: 0 of 12-14 GAC facade materials survived in 33 of 33
per-building bakes — for months, unnoticed, because the fallback greys pass
for concrete. On brick (AEC brownstones) the same defect is glaring.

Fixed at the source in `gac_storey_slice._selfcontained_like` (re-author the
material into the bake with its MDL module + subIdentifier + inputs, never a
reference). The portable-export lesson generalises: **an exported cell must
contain no binding whose resolution depends on a prim that lived only in the
build-time composition.** If you add a check for this, the test is a COLD
REOPEN asserting every bound material `IsA(UsdShade.Material)` — not a path
scan, and not `IsValid()`.
