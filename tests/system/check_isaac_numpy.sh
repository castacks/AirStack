#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    printf 'Usage: bash %s <isaac-image-reference>\n' "${0##*/}" >&2
    exit 2
fi

docker run --rm --pull=never --entrypoint /isaac-sim/python.sh "$1" -c '
import numpy
import sys

print(f"Isaac Python {sys.version.split()[0]}, NumPy {numpy.__version__}")
if numpy.__version__ != "1.26.4":
    raise SystemExit("Expected Isaac Sim NumPy 1.26.4")
'
