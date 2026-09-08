#!/usr/bin/env bash
# Stage 1 of the defect-mining study for every project (ICRA 2027 AirStack paper, Sec. VI-B).
#   ./run_defect_mining.sh [REPOS_DIR] [OUT_DIR]
# Uses the clones made by ../reuse/run_reuse_study.sh (run that first) plus castacks/DontFoolMeTwice.
# Needs git, python3, and `gh` authenticated (for the AirStack-core and airstack-dfm2 PR/CI history).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPOS=${1:-$HERE/../reuse/repos}
OUT=${2:-$HERE/results}
M="python3 $HERE/mine_defects.py"

[ -d "$REPOS/DontFoolMeTwice/.git" ] || git clone -q https://github.com/castacks/DontFoolMeTwice.git "$REPOS/DontFoolMeTwice"
AS=$REPOS/AirStack
git -C "$AS" fetch -q origin refs/heads/develop refs/heads/raven refs/heads/junbin/planning_demo refs/heads/yikuan/SVG_ground_control

# AirStack core: from the develop commit preceding 2026-04-28 (system tests went live) to the 2026-09-08 develop tip.
$M --project AirStack-core --repo "$AS" --base 2624ffd7b43e8e6e3222ffe39f92e96205116c40 --head 3cddcb24d1f190fe936aae91f208935cea5d8e83 \
   --github castacks/AirStack --since 2026-04-28 --until 2026-09-08 --pr-base develop --pr-base main --out "$OUT/airstack-core"

$M --project Shimizu  --repo "$AS" --base 39e5e698cf3404abe70c208b99e7667db5a28ab6 --head a50c19e9edd092f2606432be3e046a090a10da10 --links castacks/AirStack --all-messages --out "$OUT/shimizu"
$M --project SwarmCBF --repo "$AS" --base e4b499d120ef5157232c6ef6b1109488a94c9641 --head 564d43e4542c1e2dd4cd24cebeb21b3fb3843ad2 --links castacks/AirStack --all-messages --out "$OUT/swarmcbf"

$M --project RAVEN --repo "$AS"              --base 2d4f4be313fdb2221b23cf728fd906ae30c6d62a --head 278acbffaf748cd6e0102b3a25cfea544e031c83 --links castacks/AirStack --all-messages --out "$OUT/raven/airstack"
$M --project RAVEN --repo "$REPOS/RayFronts" --base cded3eee90ad82fbff44c58e7dff31523c7fe529 --head 8d838d79e444a035b71865034e4c33020b981fc8 --links seungchan-kim/RayFronts --all-messages --out "$OUT/raven/rayfronts"
$M --project RAVEN --repo "$REPOS/RAVEN"     --head 91ef2e7aacb9d2cbb6bbaf72bdcac7754638a778 --links castacks/RAVEN --all-messages --out "$OUT/raven/raven-top"

if [ -d "$REPOS/airstack-dfm2/.git" ]; then
  $M --project DFM2 --repo "$REPOS/airstack-dfm2" --base 19bf91d82380bc5a5cdd44bce06b265a23dd7524 --head 3af8f8e0797207ac9d0c8232012ec1362101d04a \
     --github castacks/airstack-dfm2 --since 2026-02-01 --until 2026-09-08 --pr-base main --all-messages --out "$OUT/dfm2/airstack-dfm2"
else
  echo "[skip] castacks/airstack-dfm2 is private; DFM2 AirStack-side enumeration skipped"
fi
$M --project DFM2 --repo "$REPOS/DontFoolMeTwice" --head 4ec60f84a2416b17ff45e0aef4c613355c49cc62 --links castacks/DontFoolMeTwice --all-messages --out "$OUT/dfm2/dontfoolmetwice"
echo "Stage 1 written to $OUT; now classify per ../defect_mining_prompt.md, then run summarize_defects.py"
