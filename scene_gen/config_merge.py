"""config_merge — the one deep merge, in a module nothing else imports.

There were three copies of this function: `scene_generator._deep_merge`,
`compile_disaster.deep_merge` and `compile_locale._deep_merge`. The third
carried a note explaining why it was a copy —

    "duplicated rather than imported to keep this module free of that import
     cycle (compile_disaster imports this one)"

— and that reason is real: `compile_disaster` imports `compile_locale` at
module scope, so `compile_locale` cannot import back. The fix is not to import
either one from the other but to put the function somewhere neither owns.

Semantics, which every copy agreed on and which callers depend on: nested
dicts merge KEY BY KEY, everything else — scalars and **lists** — is replaced
outright. Lists are wholesale on purpose: an asset pool that appends rather
than replaces is unpredictable to override, which is why the pack format has
an explicit `<key>+` spelling for the append case (`_merge_asset_pack`).

MERGING PRESERVES INSERTION ORDER, and that is load-bearing rather than
incidental. `city_detail` walks its `categories` dict in order, placing
furniture into a shared occupancy grid, so whichever category comes first wins
the contested kerb — which means re-ordering keys silently changes the scene.
"""


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge *override* into *base*, in place. Returns *base*."""
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base
