# M00 baseline register

Assessment date: 2026-11-27 (America/New_York). Machine evidence uses UTC timestamps.

| Baseline | Identity / status |
|---|---|
| Repository | `https://github.com/castacks/AirStack.git`, local `/root/AirStack` |
| Authorized branch | `ore_proj` |
| Starting HEAD | `169c83a58bbb387e9a9086ef49ca2dd5887a7eb7` |
| Initial working tree / index | Clean; recorded before creating RRM |
| Protected reference | `RRM-transfer/rrm-9d26a58/`; 648 file/symlink entries inventoried |
| Other protected tracked paths | 1,871 tracked entries including gitlinks, before RRM creation |
| Repository instructions | `AGENTS.md` read; explicit user restriction confines deliverables to RRM, including notebook/report equivalents; no root notebook or MkDocs edits |
| Requested controlled input location | `/root/Downloads/RRM-C001-F00-EXT/` absent initially |
| User-supplied controlled input | `/root/AirStack/RRM/RRM-C001-F00-EXT/`, read-only; 48 files / 238,449 bytes |
| Input integrity | Bundled SHA256SUMS: 26/26; candidate RELEASE_MANIFEST.sha256: 20/20. Inventory includes both manifests |
| Authority limitation | Original frozen F00/SRS/BOUNDARY records are referenced by EXT but not supplied; no independent authentication of those originals |
| Frozen F00 disposition | CLOSED / FROZEN / QUALIFICATION BLOCKED; unchanged |
| Candidate provider release | F00X-REL-001, not released; scripts and executable images missing from supplied package |
| M00 commit | Identified by Git commit `RRM-F00 M00: establish baseline and requirements assessment`; resulting SHA recorded in ignored local receipt and user handoff |

Evidence: `RRM/evidence/M00/repository-before.json`, `source-manifest.json`, `controlled-package-manifest.json`, `runtime-probes.json`, `passive-runtime.json`. SHA-256 manifests establish content identity, not correctness or qualification. The pre-change inventory excludes ignored runtime artifacts, container filesystems and submodule internal working trees; gitlink HEADs are recorded.
