#!/usr/bin/env python3
"""nucleus.py — read Nucleus from the HOST, without booting Isaac Sim.

    python3 tools/nucleus.py --setup                     # one-time, ~44 MB
    python3 tools/nucleus.py --ls   omniverse://…/Library/Stages/
    python3 tools/nucleus.py --stat omniverse://…/debris_1.usd
    python3 tools/nucleus.py --get  omniverse://…/debris_1.usd  /tmp/d.usd

    from nucleus import connect, ls, read, exists      # as a library
    client = connect()

WHY THIS EXISTS
---------------
Every Nucleus tool in this repo says the same thing: `omni.client` only works
inside the isaac-sim container, after `SimulationApp` has started, so a listing
costs an ~80 s headless Kit boot and has to be driven through `docker exec`
with the output written to a file because Kit swallows stdout. See the
docstrings of `nucleus_browse.py` and `localize_nucleus_assets.py`.

That turns out to be false, and expensively so. `omni.client` is a plain shared
library with a pybind layer; it does not need Kit, an app, a GPU, or a USD
build. What it needs is `libcarb.so` on the loader path, which lives one
directory up from the extension (`/isaac-sim/kit/libcarb.so`, not
`…/omni.client.lib/bin/`). Miss that and the import fails with

    ImportError: libcarb.so: cannot open shared object file

or — if a stale copy is found — connects and returns `ERROR_CONNECTION`, which
reads like a network or credential problem and is not one. That single missing
path is the whole reason for the "must run inside the container" rule.

Measured, host, `AirStack/.venv` (CPython 3.10):

    import omni.client        0.08 s      (against ~80 s for a headless Kit boot)
    list  Library/Stages/     0.71 s      32 entries
    stat  debris_1.usd        0.67 s
    read  debris_1.usd        0.02 s      79,326 bytes

WHAT THIS DOES AND DOES NOT GIVE YOU
------------------------------------
It gives the `omni.client` API: `list`, `stat`, `read_file`, `copy`. That is
enough to browse, audit and MIRROR Nucleus from the host.

It does **not** make `Usd.Stage.Open("omniverse://…")` work here. That needs
`omni.usd_resolver`, which is a USD plugin: it is compiled against Kit's own
USD build and shipped `cp311`-only, while this venv is 3.10 and the host
`usd-core` is a different USD version. Dropping the plugin in would be an ABI
coin-flip, so it is deliberately not attempted.

The supported way to open a Nucleus asset on the host is therefore unchanged
and is the one already in the tree: mirror it locally with
`localize_nucleus_assets.py` and open the copy under `assets/nucleus/`. What
changes is that filling that mirror no longer needs a container — `download()`
below does it from here.

CREDENTIALS
-----------
Read from `simulation/isaac-sim/docker/omni_pass.env`, the same file the
container uses, and never printed. They are required: the server refuses
anonymous access (`ERROR_CONNECTION` with the variables unset), so a failure
with no credentials looks identical to the missing-`libcarb` failure above.

Values are used exactly as written in the file. The `$$` doubling the template
warns about is a **docker-compose** escape; this file quotes the value instead
(`OMNI_USER='$omni-api-token'`), so it is already literal and must not be
un-escaped again.

THE RUNTIME IS EXTRACTED, NOT VENDORED
--------------------------------------
`--setup` copies four files out of the isaac-sim image into a gitignored
directory. They are NVIDIA binaries that already exist on this machine inside
the image; copying beats vendoring them into the repo, and it keeps them
pinned to the same Isaac Sim build everything else here runs against.

It works whether or not a container is up: a running one is used directly, and
otherwise a stopped throwaway is created from the image named in `.env` and
removed again. Nothing is downloaded.
"""

from __future__ import annotations

import argparse
import ctypes
import os
import subprocess
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_SCENE_GEN = os.path.dirname(_HERE)
_REPO = os.path.dirname(_SCENE_GEN)

#: Where the extracted runtime lives. Gitignored; `--setup` fills it.
RUNTIME = os.path.join(_SCENE_GEN, "assets", "omniclient")

#: The container paths of what has to come out. `libcarb.so` is the one that is
#: NOT under the extension's own `bin/` — see the module docstring.
_PARTS = {
    "omni": "/isaac-sim/kit/extscore/omni.client.lib/omni",
    "bin/libomniclient.so":
        "/isaac-sim/kit/extscore/omni.client.lib/bin/libomniclient.so",
    "bin/libomniverse_connection.so":
        "/isaac-sim/kit/extscore/omni.client.lib/bin/libomniverse_connection.so",
    "bin/libcarb.so": "/isaac-sim/kit/libcarb.so",
}

#: Preload order matters: carb first, then the client that needs it.
_PRELOAD = ("libcarb.so", "libomniclient.so")

CREDS = os.path.join(_REPO, "simulation", "isaac-sim", "docker",
                     "omni_pass.env")

_client = None


# ---------------------------------------------------------------------------
# setup
# ---------------------------------------------------------------------------

def _image() -> str:
    """The isaac-sim image tag, from `.env`."""
    env = {}
    path = os.path.join(_REPO, ".env")
    if os.path.exists(path):
        for line in open(path):
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                env[k.strip()] = v.strip().strip('"').strip("'")
    reg = env.get("PROJECT_DOCKER_REGISTRY", "airlab-docker.andrew.cmu.edu/airstack")
    proj = env.get("PROJECT_NAME", "airstack")
    ver = env.get("VERSION", "latest")
    return f"{reg}/{proj}:{ver}_isaac-sim"


def _source_container() -> tuple:
    """``(container, cleanup)`` to copy from. Prefers one already running."""
    out = subprocess.run(
        ["docker", "ps", "--filter", "ancestor=" + _image(),
         "--format", "{{.Names}}"], capture_output=True, text=True)
    name = (out.stdout or "").split("\n")[0].strip()
    if not name:
        # Any container off that image, running or not.
        out = subprocess.run(
            ["docker", "ps", "-a", "--filter", "name=isaac-sim",
             "--format", "{{.Names}}"], capture_output=True, text=True)
        name = (out.stdout or "").split("\n")[0].strip()
    if name:
        return name, False

    print(f"[nucleus] no isaac-sim container; creating a throwaway from "
          f"{_image()}")
    out = subprocess.run(["docker", "create", _image()],
                         capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError(
            "could not create a container from the isaac-sim image "
            "(is it pulled?):\n" + (out.stderr or "").strip())
    return out.stdout.strip(), True


def setup(force: bool = False) -> str:
    """Extract the runtime from the isaac-sim image. Idempotent."""
    if not force and is_ready():
        print(f"[nucleus] runtime already present: {RUNTIME}")
        return RUNTIME

    container, throwaway = _source_container()
    try:
        os.makedirs(os.path.join(RUNTIME, "bin"), exist_ok=True)
        for rel, src in _PARTS.items():
            dst = os.path.join(RUNTIME, rel)
            if rel == "omni":
                # Copy the package dir itself, not into an existing one.
                if os.path.isdir(dst):
                    import shutil
                    shutil.rmtree(dst)
                dst = RUNTIME
            r = subprocess.run(["docker", "cp", f"{container}:{src}", dst],
                               capture_output=True, text=True)
            # `docker cp` warns about a dangling symlink inside omni/client/utils
            # (it points at a sibling extension that is not being copied). The
            # package does not import it, so a non-zero rc with files present is
            # not a failure.
            if r.returncode != 0 and not os.path.exists(
                    os.path.join(RUNTIME, rel if rel != "omni" else "omni")):
                raise RuntimeError(f"docker cp failed for {src}:\n{r.stderr}")
    finally:
        if throwaway:
            subprocess.run(["docker", "rm", "-f", container],
                           capture_output=True)

    # Bindings for every CPython the image ships; keep only ours.
    impl = os.path.join(RUNTIME, "omni", "client", "impl")
    tag = f"cpython-{sys.version_info.major}{sys.version_info.minor}"
    kept = None
    if os.path.isdir(impl):
        for f in sorted(os.listdir(impl)):
            if not f.startswith("_omniclient.cpython-"):
                continue
            if tag in f:
                kept = f
            else:
                os.remove(os.path.join(impl, f))
    if kept is None:
        raise RuntimeError(
            f"the image ships no {tag} binding for omni.client — this "
            f"interpreter is {sys.version.split()[0]}; use one the image "
            f"supports, or run the container-side tools instead.")

    print(f"[nucleus] runtime -> {RUNTIME}  ({kept})")
    if not os.path.exists(CREDS):
        print(f"[nucleus] WARNING: no credentials at {CREDS} — the server "
              f"refuses anonymous access, so calls will fail with "
              f"ERROR_CONNECTION.")
    return RUNTIME


def is_ready() -> bool:
    """True when the runtime is extracted and matches this interpreter."""
    impl = os.path.join(RUNTIME, "omni", "client", "impl")
    tag = f"cpython-{sys.version_info.major}{sys.version_info.minor}"
    return (all(os.path.exists(os.path.join(RUNTIME, r)) for r in _PARTS)
            and os.path.isdir(impl)
            and any(tag in f for f in os.listdir(impl)))


# ---------------------------------------------------------------------------
# connect
# ---------------------------------------------------------------------------

def load_credentials(path: str = None) -> dict:
    """`OMNI_*` from the container's env file, into `os.environ`. Not printed.

    Values are taken literally, quotes stripped — see the docstring on the
    `$$` compose escape, which must NOT be undone here.
    """
    path = path or CREDS
    got = {}
    if not os.path.exists(path):
        return got
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, v = line.split("=", 1)
        k = k.strip()
        if not k.startswith(("OMNI_", "ACCEPT_")):
            continue
        v = v.strip()
        if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
            v = v[1:-1]
        os.environ.setdefault(k, v)
        got[k] = len(v)
    return got


def connect(auto_setup: bool = True):
    """Return the `omni.client` module, ready to use. Cached.

    Preloads the shared libraries `RTLD_GLOBAL` so the extension's `DT_NEEDED`
    entries resolve against what is already in the process — which is why no
    `LD_LIBRARY_PATH` is needed and why this works from a plain `uv run`.
    """
    global _client
    if _client is not None:
        return _client

    if not is_ready():
        if not auto_setup:
            raise RuntimeError(
                "omni.client runtime missing — run: "
                "python3 tools/nucleus.py --setup")
        setup()

    load_credentials()
    for so in _PRELOAD:
        ctypes.CDLL(os.path.join(RUNTIME, "bin", so), mode=ctypes.RTLD_GLOBAL)
    if RUNTIME not in sys.path:
        sys.path.insert(0, RUNTIME)

    import omni.client as _c                                    # noqa: E402
    _client = _c
    return _client


# ---------------------------------------------------------------------------
# a thin, honest API over the four calls that matter
# ---------------------------------------------------------------------------

def server_root() -> str:
    """``omniverse://host`` — the SERVER, not the asset root.

    `OMNI_SERVER` points at a subtree (`…/NVIDIA/Assets/Isaac/5.1`) because
    that is what Isaac Sim wants it to mean, so appending a path to it lands
    nowhere. Everything the scene configs reference lives under
    `/Library/Stages/` on the same host, so the host is what gets reused.
    """
    load_credentials()
    srv = os.environ.get("OMNI_SERVER", "")
    if "://" in srv:
        scheme, rest = srv.split("://", 1)
        return f"{scheme}://{rest.split('/', 1)[0]}"
    return "omniverse://airlab-nucleus.andrew.cmu.edu"


def ls(url: str) -> list:
    """``[(name, size, is_dir), …]``. Raises on a non-OK result."""
    c = connect()
    res, entries = c.list(url)
    if res != c.Result.OK:
        raise RuntimeError(f"list {url}: {res}")
    out = []
    for e in entries or []:
        is_dir = bool(int(e.flags) & int(c.ItemFlags.CAN_HAVE_CHILDREN))
        out.append((e.relative_path, int(getattr(e, "size", 0) or 0), is_dir))
    return out


def exists(url: str) -> bool:
    c = connect()
    return c.stat(url)[0] == c.Result.OK


def stat(url: str):
    """The entry, or None when it does not resolve."""
    c = connect()
    res, entry = c.stat(url)
    return entry if res == c.Result.OK else None


def read(url: str) -> bytes:
    c = connect()
    res, _ver, content = c.read_file(url)
    if res != c.Result.OK:
        raise RuntimeError(f"read {url}: {res}")
    return bytes(memoryview(content))


def download(url: str, dest: str) -> int:
    """Write *url* to *dest*, making parents. Returns bytes written."""
    data = read(url)
    os.makedirs(os.path.dirname(os.path.abspath(dest)) or ".", exist_ok=True)
    with open(dest, "wb") as fh:
        fh.write(data)
    return len(data)


def walk(url: str, max_depth: int = 3, _depth: int = 0):
    """Yield ``(url, size, is_dir)`` recursively, depth-limited."""
    base = url if url.endswith("/") else url + "/"
    for name, size, is_dir in ls(base):
        full = base + name
        yield full, size, is_dir
        if is_dir and _depth < max_depth:
            yield from walk(full, max_depth, _depth + 1)


# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--setup", action="store_true",
                    help="extract the runtime from the isaac-sim image")
    ap.add_argument("--force", action="store_true", help="re-extract")
    ap.add_argument("--check", action="store_true",
                    help="report runtime + credentials + a live call")
    ap.add_argument("--ls", metavar="URL")
    ap.add_argument("--walk", metavar="URL")
    ap.add_argument("--depth", type=int, default=2)
    ap.add_argument("--stat", metavar="URL")
    ap.add_argument("--get", nargs=2, metavar=("URL", "DEST"))
    args = ap.parse_args()

    if args.setup or args.force:
        setup(force=args.force)
        if not (args.check or args.ls or args.walk or args.stat or args.get):
            return 0

    if args.check:
        print(f"runtime      {'ready' if is_ready() else 'MISSING'}  {RUNTIME}")
        got = load_credentials()
        print(f"credentials  {'ok' if got else 'MISSING'}  {CREDS}"
              + (f"  ({', '.join(sorted(got))})" if got else ""))
        probe = server_root() + "/Library/Stages/"
        try:
            n = len(ls(probe))
            print(f"live call    ok — {n} entries under {probe}")
        except Exception as exc:                                # noqa: BLE001
            print(f"live call    FAILED — {type(exc).__name__}: {exc}")
            return 1
        return 0

    if args.ls:
        for name, size, is_dir in ls(args.ls):
            print(f"{'d' if is_dir else '-'} {size:>12,}  {name}")
        return 0

    if args.walk:
        for full, size, is_dir in walk(args.walk, args.depth):
            print(f"{'d' if is_dir else '-'} {size:>12,}  {full}")
        return 0

    if args.stat:
        e = stat(args.stat)
        print(f"{args.stat}: "
              + (f"{e.size:,} bytes" if e else "DOES NOT RESOLVE"))
        return 0 if e else 1

    if args.get:
        n = download(*args.get)
        print(f"{args.get[1]}  {n:,} bytes")
        return 0

    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
