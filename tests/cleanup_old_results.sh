#!/bin/bash
# Delete test result runs older than the given cutoff directory name.
# Usage: ./cleanup_old_results.sh 2026-06-05_17-46-02

set -euo pipefail

KEEP="${1:?Usage: $0 <cutoff-dir-name>  # keeps this dir and newer}"
RESULTS="$(cd "$(dirname "$0")/results" && pwd)"

deleted=0
for d in "$RESULTS"/*/; do
  name=$(basename "$d")
  if [[ "$name" < "$KEEP" ]]; then
    rm -rf "$d"
    echo "deleted: $name"
    deleted=$((deleted + 1))
  fi
done

echo "Done. Deleted $deleted run(s). Remaining:"
ls -1 "$RESULTS" | sort
