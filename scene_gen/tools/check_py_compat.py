"""check_py_compat.py — do these files parse on the Isaac Sim container's Python?

    python3 tools/check_py_compat.py            # scene_gen + launch scripts
    python3 tools/check_py_compat.py --min 3.10 path/to/file.py ...

WHY THIS EXISTS
---------------
The host and the container do not run the same Python, and the gap is a
SYNTAX gap, not a library one — so nothing catches it until the container
imports the file and dies at load. It has already cost one launch:

    print(f"\\n{'ALL PASS' if not fails else str(len(fails)) + ' FAILURE(S): '
          + repr(fails)}")

An expression spanning two lines INSIDE an f-string's braces is PEP 701, i.e.
Python 3.12 and up. It parsed fine on the 3.12 host, and the container raised
`SyntaxError: unterminated string literal` before a single line of the scene
ran. `suburb_yards.py` had carried it for a while without anyone noticing,
because nothing imported that module — it is normally run as a script, and a
script that is never run is never parsed.

`ast.parse(..., feature_version=...)` is the whole trick: it asks CPython to
parse with an older grammar, which is exactly the question worth asking. It
catches syntax, not semantics — a 3.11-only stdlib call still gets through, so
this is a floor rather than a guarantee.
"""

import argparse
import ast
import glob
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_REPO = os.path.dirname(_SCENE_GEN)

# Isaac Sim 4.x ships Python 3.10; 3.11 is checked too so the answer does not
# quietly depend on which minor the image happens to carry.
DEFAULT_TARGETS = ((3, 10), (3, 11))

DEFAULT_GLOBS = (
    os.path.join(_SCENE_GEN, "**", "*.py"),
    os.path.join(_REPO, "simulation", "isaac-sim", "launch_scripts", "*.py"),
    os.path.join(_REPO, "simulation", "isaac-sim", "utils", "*.py"),
)


def files_to_check(patterns):
    out = []
    for pat in patterns:
        for f in glob.glob(pat, recursive=True):
            if "__pycache__" in f or os.sep + ".venv" + os.sep in f:
                continue
            out.append(f)
    return sorted(set(out))


def check(path, targets):
    """Return a list of ``(target, lineno, message)`` failures for *path*."""
    try:
        src = open(path, encoding="utf-8").read()
    except (OSError, UnicodeDecodeError) as e:
        return [((0, 0), 0, f"unreadable: {e}")]
    fails = []
    for ver in targets:
        try:
            ast.parse(src, filename=path, feature_version=ver)
        except SyntaxError as e:
            fails.append((ver, e.lineno or 0, (e.msg or str(e))))
            break            # the lowest failing target is the useful one
        except ValueError as e:
            fails.append((ver, 0, str(e)))
            break
    return fails


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="*", help="files to check (default: all)")
    ap.add_argument("--min", default="3.10",
                    help="lowest Python to check against (default 3.10)")
    args = ap.parse_args()

    major, minor = (int(x) for x in args.min.split(".")[:2])
    targets = tuple(t for t in DEFAULT_TARGETS if t >= (major, minor)) \
        or ((major, minor),)

    paths = args.paths or files_to_check(DEFAULT_GLOBS)
    bad = 0
    for p in paths:
        for ver, line, msg in check(p, targets):
            bad += 1
            rel = os.path.relpath(p, _REPO)
            print(f"FAIL  py{ver[0]}.{ver[1]}  {rel}:{line}  {msg}")
    tgt = ", ".join(f"{a}.{b}" for a, b in targets)
    if bad:
        print(f"\n{bad} file(s) will not parse on Python {tgt} "
              f"(host is {sys.version_info.major}.{sys.version_info.minor})")
        return 1
    print(f"{len(paths)} file(s) parse on Python {tgt}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
