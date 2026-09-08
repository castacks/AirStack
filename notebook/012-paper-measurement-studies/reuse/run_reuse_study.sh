#!/usr/bin/env bash
# Reproduce the lines-of-code reuse study (ICRA 2027 AirStack paper, Sec. V / Table I).
#
#   ./run_reuse_study.sh [REPOS_DIR] [OUT_DIR]
#
# Clones (or fetches into) the repositories named in projects.yaml, checks that
# every pinned SHA is present, then runs loc_reuse.py.  Private repositories
# (castacks/airstack-dfm2) are skipped with a message if you lack access; the
# public projects still run.  Needs: git, python3, PyYAML (pip install pyyaml).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPOS=${1:-$HERE/repos}
OUT=${2:-$HERE/results}
mkdir -p "$REPOS"

fetch() {  # fetch <dir> <url> <ref-or-sha>...
  local dir=$1 url=$2; shift 2
  if [ ! -d "$REPOS/$dir/.git" ]; then
    echo "[clone] $url -> $dir"
    git clone -q --no-checkout "$url" "$REPOS/$dir" || { echo "[skip] cannot clone $url (private?)"; return 0; }
  fi
  for ref in "$@"; do
    git -C "$REPOS/$dir" fetch -q origin "$ref" 2>/dev/null || true
    git -C "$REPOS/$dir" cat-file -e "${ref}^{commit}" 2>/dev/null \
      || { echo "[warn] $dir: $ref not reachable"; }
  done
}

# castacks/AirStack: the three team branches + every fork point used as a base
fetch AirStack https://github.com/castacks/AirStack.git \
  refs/heads/raven refs/heads/junbin/planning_demo refs/heads/yikuan/SVG_ground_control \
  refs/heads/develop refs/heads/main
fetch RAVEN     https://github.com/castacks/RAVEN.git            refs/heads/main
fetch RayFronts https://github.com/seungchan-kim/RayFronts.git   refs/heads/raven refs/heads/main
fetch LVLM      https://github.com/seungchan-kim/LVLM.git        refs/heads/main
fetch airstack-dfm2 git@github.com:castacks/airstack-dfm2.git    refs/heads/main   # private

python3 "$HERE/loc_reuse.py" "$HERE/projects.yaml" --repos-dir "$REPOS" --out "$OUT"
echo "results in $OUT (summary.md, table_rows.tex, <project>.md/json)"
