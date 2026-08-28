# Results — 004 CLI command groups (osmo, config, images)

## a-cli-paths — verdict: PASS

Verified on 2026-08-24:

| Check | Result |
|-------|--------|
| `bash -n` airstack.sh, osmo.sh, config.sh | clean |
| `airstack lint` | 388 passed, 7 skipped (contract suite) + bash -n + py_compile |
| workflow YAMLs (`system-tests`, `module-system-tests`) | parse OK |
| `airstack help` / `commands` | one row each for `osmo`, `config`, `images`; no legacy rows |
| `airstack help osmo` / `config` / `images` | full subcommand reference printed |
| `airstack osmo/config/images bad-sub` | error + group help, exit 1 |
| bare `airstack images` | lists project images (old behavior kept) |
| `airstack images rm` (no term) | "Search term required" + group help |
| `--progress=quiet images pull --ignore-pull-failures` | flags-first parse works (sub = pull) |
| legacy `osmo:logs`, `config:nucleus`, `image-pull`, `image-delete`, `rmi` | deprecation warn, then forward to the real subcommand |
| `echo n \| airstack image-delete` | forwards, lists images, aborts on "n" (nothing deleted) |
| `tests/system/test_build_docker.py` | py_compile clean with `airstack_cmd("images", "build", ...)` |
