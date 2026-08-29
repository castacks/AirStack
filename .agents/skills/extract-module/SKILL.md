---
name: extract-module
description: Extract an in-tree AirStack capability into a standalone module repo — choose history extraction vs plain copy, author and validate module.yaml, build test_stack/ from a reference stack, handle submodules and host-setup hooks, write a TRUNK_REMOVAL.md checklist, sequence the trunk-removal PR against the module overlay, and wire module CI. Use when graduating a trunk package (or fork research) into an asm_* module per RFC #379.
license: BSD-3-Clause-Clear
metadata:
  author: AirLab CMU
  repository: AirStack
---

# Skill: Extract an In-Tree Capability into a Module Repo

Distilled from three completed extractions: `asm_macvo` (heavy Docker deps +
submodule), `asm_dfm2_disturbances` (Isaac extension from a fork), and
`asm_optitrack` (proprietary SDK via hooks). This skill covers the **extraction
procedure**; for authoring the module repo itself (manifest fields,
canonical-defaults launch rule, repo anatomy) defer to
[create-module](../create-module/SKILL.md) — don't duplicate it here.

> `airstack module extract` automation is **future work** (RFC #379 §11 names it
> as the graduation step for `module create --in-tree` research). Today the
> extraction is manual; this skill is the manual.

## 0. Scope the extraction

- [ ] List every trunk artifact the capability touches — not just the package:
  Dockerfile blocks and build args, compose args, bringup launch wiring,
  keepalive/foxglove rows, wiring snapshots, docs pages, mkdocs excludes,
  `.devcontainer` launch entries, rviz layouts. (The macvo extraction touched
  all of these.) This list becomes `TRUNK_REMOVAL.md` (§6).
- [ ] Record the exact trunk ref and `VERSION` you extract from — `module.yaml`
  `airstack_compat` and the checklist's line references pin against it.
- [ ] If the source lives in a **fork**, run `airstack module doctor --drift`
  there first: module-contained changes move with you; trunk edits are
  extraction debt to upstream, carry, or turn into a convention.

## 1. History: `git filter-repo` vs plain copy

- **`git filter-repo`** when the capability lives in one or two clean trunk
  paths, its history is worth keeping, and the code moves mostly as-is:

  ```bash
  git clone --no-local /path/to/AirStack asm_<name> && cd asm_<name>
  git filter-repo --path robot/ros_ws/src/<layer>/<pkg> \
                  --path-rename robot/ros_ws/src/<layer>/<pkg>:<pkg>
  ```

- **Plain copy into a fresh repo** when sources are scattered (the dfm2 case:
  files spread across multiple fork branches, some only in dangling commits),
  contain unresolved conflict markers, or need heavy rewriting to trunk
  conventions anyway. Cite the source commits in the README / a friction log
  instead of carrying history.
- Either way, keep a **FRICTION_LOG.md** (or notebook entry) of every step that
  needed manual invention — it is the requirements list for the future tooling.

## 2. Author + validate the manifest

- [ ] Follow [create-module](../create-module/SKILL.md) for `module.yaml`. Key
  extraction-specific choices seen in practice:
  - Heavy image deps → put **everything** in `Dockerfile.module` (tier 2) and
    keep `deps: {apt: [], pip: []}` empty so the layer story is unambiguous
    (asm_macvo).
  - Large pinned files (model weights) → `assets:` with `url` + `sha256` +
    `dest`, no Git LFS.
  - `airstack_compat`: a real semver range against the trunk `VERSION` you
    tested (e.g. `">=0.19.0-alpha.18 <0.20.0"`), never a branch.
- [ ] Validate:

  ```bash
  python3 tools/validate_module.py /path/to/asm_<name>   # exit 0, {"valid": true}
  airstack module add /path/to/asm_<name>                # local-path add + sync
  airstack module doctor                                 # manifests + overlay integrity
  ```

## 3. Host-side SDK installs → `hooks.host_setup`

Proprietary or license-gated SDKs (asm_optitrack's NatNet SDK) never go in the
image or in git:

- [ ] Ship a download script and declare it:

  ```yaml
  hooks:
    host_setup: <pkg>/scripts/download-<sdk>.sh
  ```

- [ ] The hook contract: **idempotent, no sudo, writes only inside the module
  checkout** (gitignore the landed files). `airstack module sync` runs it;
  `--no-hooks` skips it.

## 4. Git submodules

If the trunk package embeds a submodule (macvo's MAC-VO network):

- [ ] The **module repo** carries the submodule now; `airstack module sync` uses
  `vcs import --recursive`, so it clones automatically.
- [ ] Put the trunk-side removal in the checklist: `git submodule deinit -f
  <path>` **before** `git rm -r`, plus deleting the `.gitmodules` entry.

## 5. `test_stack/` from the reference stack

- [ ] Copy the closest trunk reference stack (`airstack stack new <src> <tmp>`
  or copy by hand) into the module's `test_stack/` and wire the module in its
  `launch/stack.launch.xml` — the **one** place the module is wired
  (single-locus rule; the module's own launch file keeps canonical defaults
  and zero remaps).
- [ ] Pin the module in `test_stack/modules.repos`. Before the repo has a
  remote/tag the pin is a **placeholder** — track updating it on first
  push/tag as a precondition in the checklist.
- [ ] Smoke it end-to-end from a trunk checkout:

  ```bash
  airstack module add /path/to/asm_<name>
  airstack module lock --build          # only if the module has a Dockerfile.module
  airstack up --sim isaac --robots 1 --headless --play --wait
  airstack test -m liveliness --sim isaacsim --num-robots 1 -v
  ```

## 6. Write `TRUNK_REMOVAL.md` in the module repo

The pattern that made the macvo extraction reviewable: a checklist **in the
module repo** enumerating every trunk file/block the trunk-removal PR must
delete, written for the orchestrator of that PR. Include:

- [ ] The package `git rm` (+ submodule/.gitmodules steps).
- [ ] Dockerfile blocks and build args to delete, **with a pip/apt consumer
  audit**: for every dep you remove, `git grep -Iil <pkg>` outside the module
  proves nothing else in the image uses it. Mark shared deps (numpy, matplotlib)
  **keep** explicitly, with reasoning.
- [ ] Compose build args, bringup launch gates/remaps (the wiring moves into
  stacks/module launch files), keepalive/foxglove rows, wiring snapshots to
  regenerate (`airstack test -m wiring --stack <name>`), docs/skills/mkdocs
  references.
- [ ] Verification: clean-cache image build with before/after size, a
  `git grep -in <name>` residue check, the trunk test suite without the module,
  and a module dogfood run (add → sync → lock --build → up → liveliness).

## 7. Sequencing rules (the ones that bite)

- **Never have the same package in trunk and the module overlay at once.** A
  duplicate colcon package makes builds nondeterministic. Remove the trunk copy
  in the same PR that consumers start pinning the module — or gate the overlay
  (don't `module add` until the trunk-removal PR merges). The macvo order:
  trunk-removal PR merges → module CI can go green → trunk's stack pins the
  module tag.
- **Stale colcon cache:** removed packages linger in the container's `install/`
  (and `build/`) until a clean rebuild — a "deleted" package that still launches
  is cache, not magic. `airstack clean` (host) or remove `build/ install/ log/`
  in the container, then `bws`.
- **Placeholder pins rot silently:** every `modules.repos` pin written before
  the first push/tag must be updated and is a checklist precondition, not a
  footnote.

## 8. CI for the module repo

- [ ] Add `.github/workflows/ci.yml` calling trunk's reusable workflow — see
  [docs/development/module_ci.md](../../../docs/development/module_ci.md):

  ```yaml
  jobs:
    system-tests:
      uses: castacks/AirStack/.github/workflows/module-system-tests.yml@v0.19.0
      with:
        airstack_ref: v0.19.0
        marks: liveliness            # per module category — see module_ci.md
        sim: msairsim                # the cheap bring-up
  ```

  Pin the workflow ref and `airstack_ref` **together**. First-party policy: the
  workflow hard-fails outside the `castacks` org.
- [ ] Known gap (tracked): the reusable workflow runs `module add`/`sync`
  (layer *plan* only) but not `airstack module lock --build`, so a
  `Dockerfile.module` is not built in CI — verify tier-2 builds manually until
  that lands.
- [ ] Register the module in the index repo
  (`castacks/airstack-modules-index`) once CI is green. **Registration is
  TWO merges, and the registry one is the one that gets missed:**
  1. **Registry repo:** PR adding `modules/<name>.yaml` (+ `stacks/<name>.yaml`
     for a consuming reference stack) to `castacks/airstack-modules-index`;
     `tools/validate_entry.py` / its CI must pass. **This PR must be MERGED,
     not just opened.**
  2. **Trunk repo:** copy the same entries into
     `tests/meta/fixtures/modules_index/`, regenerate the committed catalog
     (`python3 tools/gen_docs_catalog.py --index tests/meta/fixtures/modules_index
     --modules-dir <empty-dir>`), and add the module page + stack README to
     the `mkdocs.yml` nav.

  Why both: the docs deploy workflows regenerate `docs/modules/` against the
  **live registry** at build time — the committed pages are only the fallback
  for an *unreachable* registry. If the trunk PR merges while the registry PR
  sits unmerged, the deploy silently drops the module from the published
  catalog even though `docs/modules/index.md` in git looks right (this
  happened with `mighty`, 2026-08-29). Merge the registry PR **before or
  with** the trunk PR; if it lands late, re-run the deploy:
  `gh workflow run deploy_docs_from_develop.yaml --repo castacks/AirStack --ref develop`.

  **Automation (2026-08-29):** the trunk half no longer needs to be
  hand-built — after the registry PR merges, dispatch the
  `sync-modules-index` workflow (Actions tab; also runs daily) and it opens
  the trunk sync PR (fixture mirror + regenerated pages + VERSION bump).
  The develop docs deploy independently raises a `docs-catalog-drift` issue
  whenever the committed catalog and the live registry disagree, so a missed
  sync can no longer stay silent. Caveat: the bot PR is opened with the
  workflow token, which does not trigger CI — close and reopen it to run
  the checks.

## References

- [create-module](../create-module/SKILL.md) — manifest, anatomy, canonical-defaults rule
- [create-stack](../create-stack/SKILL.md) — stack folders and wiring.md
- [docs/development/modules.md](../../../docs/development/modules.md) — overlay, dep tiers, `module lock`
- [docs/development/module_ci.md](../../../docs/development/module_ci.md) — the reusable CI caller
- Worked examples: `asm_macvo/TRUNK_REMOVAL.md` (trunk-removal checklist),
  `asm_dfm2_disturbances/FRICTION_LOG.md` (fork archaeology + port decisions),
  `asm_optitrack` (hooks.host_setup)
