#!/usr/bin/env python3
"""Plan retag-vs-rebuild for AirStack docker-build.yml publishes.

For each compose service with a ``build:`` section under the selected profiles,
compute a content fingerprint of its image inputs. If the previous versioned
image carries the same ``org.airstack.content-fingerprint`` label, the service
is marked ``retag``; otherwise ``build``.

Also writes an ephemeral compose override that applies the fingerprint as a
build label on services that will be rebuilt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

FINGERPRINT_LABEL = "org.airstack.content-fingerprint"

# Explicit roots so broad compose ``context:`` dirs do not hash the whole tree.
# Keys are compose service names after ``docker compose config`` resolution.
SERVICE_FINGERPRINT_ROOTS: dict[str, list[str]] = {
    "robot-desktop": [
        "robot/docker/Dockerfile.robot",
        "robot/docker/docker-compose.yaml",
        "robot/docker/robot-base-docker-compose.yaml",
        "robot/docker/custom_rosdep.yaml",
        "robot/docker/wait_for_px4.py",
        "robot/docker/.bashrc",
        "robot/docker/robot_name_map",
    ],
    "gcs": [
        "gcs/docker/Dockerfile.gcs",
        "gcs/docker/docker-compose.yaml",
        "gcs/docker/gcs-base-docker-compose.yaml",
        "gcs/docker/.bashrc",
        "gcs/docker/resources",
        "gcs/docker/Foxglove",
    ],
    "isaac-sim": [
        "simulation/isaac-sim/docker/Dockerfile.isaac-ros",
        "simulation/isaac-sim/docker/docker-compose.yaml",
        "simulation/isaac-sim/docker/fastdds.xml",
        "simulation/isaac-sim/docker/.bashrc",
        "simulation/isaac-sim/docker/omniverse.toml",
    ],
    "ms-airsim": [
        "simulation/ms-airsim/docker/Dockerfile",
        "simulation/ms-airsim/docker/docker-compose.yaml",
        "simulation/ms-airsim/docker/entrypoint.sh",
    ],
}

# Extra roots when DOCKER_IMAGE_BUILD_MODE=prebuilt (workspace baked into image).
PREBUILT_EXTRA_ROOTS: dict[str, list[str]] = {
    "robot-desktop": [
        "robot/ros_ws/src",
        "common/ros_packages",
        "common/fastdds.xml",
    ],
    "gcs": [
        "gcs/ros_ws",
        "common/ros_packages",
    ],
}

# .env keys whose values affect image tags or layers (included in fingerprint).
ENV_FINGERPRINT_KEYS = (
    "DOCKER_IMAGE_BUILD_MODE",
    "PROJECT_DOCKER_REGISTRY",
    "PROJECT_NAME",
    "CACHE_TAG",
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def load_dotenv(path: Path) -> dict[str, str]:
    env: dict[str, str] = {}
    if not path.is_file():
        return env
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        value = value.strip().strip('"').strip("'")
        env[key.strip()] = value
    return env


def run(cmd: list[str], *, cwd: Path, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=cwd,
        check=check,
        text=True,
        capture_output=True,
    )


def compose_config(root: Path, profiles: str, env: dict[str, str]) -> dict[str, Any]:
    cmd_env = os.environ.copy()
    cmd_env.update(env)
    cmd_env["COMPOSE_PROFILES"] = profiles
    proc = subprocess.run(
        ["docker", "compose", "-f", "docker-compose.yaml", "config", "--format", "json"],
        cwd=root,
        env=cmd_env,
        text=True,
        capture_output=True,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError(
            "docker compose config failed:\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
    return json.loads(proc.stdout)


def git_ls_files(root: Path, pathspec: str) -> list[str]:
    proc = run(
        ["git", "ls-files", "-z", "--", pathspec],
        cwd=root,
        check=False,
    )
    if proc.returncode != 0:
        return []
    return [p for p in proc.stdout.split("\0") if p]


def collect_tracked_files(root: Path, roots: list[str]) -> list[str]:
    files: set[str] = set()
    for rel in roots:
        path = root / rel
        if path.is_file():
            files.add(rel)
            continue
        if path.is_dir():
            for tracked in git_ls_files(root, rel):
                # Skip local secrets / generated pass files
                if tracked.endswith("omni_pass.env"):
                    continue
                if "/.dev/" in f"/{tracked}/" or tracked.endswith("/.dev"):
                    continue
                files.add(tracked)
    return sorted(files)


def file_sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def resolve_dockerfile(root: Path, service_cfg: dict[str, Any]) -> Path | None:
    build = service_cfg.get("build") or {}
    if not isinstance(build, dict):
        return None
    dockerfile = build.get("dockerfile")
    context = build.get("context") or "."
    if not dockerfile:
        return None
    # compose config usually resolves dockerfile to an absolute path
    df = Path(dockerfile)
    if df.is_absolute():
        return df
    return (Path(context) / df).resolve() if Path(context).is_absolute() else (root / context / df).resolve()


def find_dockerignore(dockerfile: Path, context: Path) -> Path | None:
    for candidate in (
        context / ".dockerignore",
        dockerfile.parent / ".dockerignore",
    ):
        if candidate.is_file():
            return candidate
    return None


def previous_image_ref(image: str, version: str, previous_version: str) -> str | None:
    if not previous_version or not version or version == previous_version:
        return None
    # Tags look like ...:v{VERSION}_suffix — replace only the version segment.
    needle = f":v{version}_"
    if needle not in image:
        # Fallback: replace first occurrence of the bare version in the tag.
        tag_part = image.rsplit(":", 1)
        if len(tag_part) != 2 or version not in tag_part[1]:
            return None
        return f"{tag_part[0]}:{tag_part[1].replace(version, previous_version, 1)}"
    return image.replace(needle, f":v{previous_version}_", 1)


def cache_tag_from_build(build: dict[str, Any], cache_tag: str) -> str | None:
    tags = build.get("tags") or []
    pfx = f":{cache_tag}_"
    for tag in tags:
        if isinstance(tag, str) and pfx in tag:
            return tag
    return None


def inspect_fingerprint_label(image: str) -> str | None:
    """Return the fingerprint label from a registry image, or None if unavailable."""
    proc = subprocess.run(
        [
            "docker",
            "buildx",
            "imagetools",
            "inspect",
            image,
            "--format",
            "{{json .}}",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    if proc.returncode != 0:
        return None
    try:
        data = json.loads(proc.stdout)
    except json.JSONDecodeError:
        return None

    # buildx JSON shape varies by version; hunt for Labels in common places.
    def walk(obj: Any) -> str | None:
        if isinstance(obj, dict):
            labels = obj.get("Labels") or obj.get("labels")
            if isinstance(labels, dict) and FINGERPRINT_LABEL in labels:
                return labels[FINGERPRINT_LABEL]
            for v in obj.values():
                found = walk(v)
                if found:
                    return found
        elif isinstance(obj, list):
            for item in obj:
                found = walk(item)
                if found:
                    return found
        return None

    return walk(data)


def compute_fingerprint(
    root: Path,
    service_name: str,
    service_cfg: dict[str, Any],
    env: dict[str, str],
) -> str:
    build = service_cfg.get("build") or {}
    roots = list(SERVICE_FINGERPRINT_ROOTS.get(service_name, []))

    dockerfile = resolve_dockerfile(root, service_cfg)
    if dockerfile and dockerfile.is_file():
        try:
            rel = str(dockerfile.relative_to(root))
        except ValueError:
            rel = str(dockerfile)
        if rel not in roots:
            roots.insert(0, rel)

    mode = env.get("DOCKER_IMAGE_BUILD_MODE", "dev")
    if mode == "prebuilt":
        roots.extend(PREBUILT_EXTRA_ROOTS.get(service_name, []))

    # If service has no map entry, fall back to dockerfile directory.
    if not roots and dockerfile is not None:
        try:
            roots = [str(dockerfile.parent.relative_to(root))]
        except ValueError:
            roots = []

    tracked = collect_tracked_files(root, roots)

    h = hashlib.sha256()
    h.update(f"service:{service_name}\n".encode())
    h.update(f"DOCKER_IMAGE_BUILD_MODE:{mode}\n".encode())

    for key in ENV_FINGERPRINT_KEYS:
        h.update(f"env:{key}={env.get(key, '')}\n".encode())

    args = build.get("args") or {}
    if isinstance(args, dict):
        for k in sorted(args):
            h.update(f"arg:{k}={args[k]}\n".encode())

    context = build.get("context")
    if context:
        h.update(f"context:{context}\n".encode())
        ctx_path = Path(context) if Path(context).is_absolute() else root / context
        dockerignore = find_dockerignore(dockerfile, ctx_path) if dockerfile else None
        if dockerignore and dockerignore.is_file():
            h.update(f"dockerignore:{dockerignore.name}\n".encode())
            h.update(dockerignore.read_bytes())
            h.update(b"\n")

    for rel in tracked:
        path = root / rel
        if not path.is_file():
            continue
        h.update(f"file:{rel}\n".encode())
        h.update(file_sha256(path).encode())
        h.update(b"\n")

    return h.hexdigest()


def write_override(path: Path, build_services: dict[str, str]) -> None:
    """Write compose override that sets build.labels fingerprint for rebuilds."""
    lines = [
        "# Generated by docker_image_plan.py — do not commit.",
        "services:",
    ]
    if not build_services:
        # Valid empty mapping; compose merge ignores it when nothing rebuilds.
        lines[-1] = "services: {}"
    else:
        for name, fingerprint in sorted(build_services.items()):
            lines.append(f"  {name}:")
            lines.append("    build:")
            lines.append("      labels:")
            lines.append(f"        {FINGERPRINT_LABEL}: \"{fingerprint}\"")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_plan(
    root: Path,
    *,
    version: str,
    previous_version: str,
    profiles: str,
    force_rebuild: bool,
    env: dict[str, str],
) -> dict[str, Any]:
    config = compose_config(root, profiles, env)
    services_out: dict[str, Any] = {}
    cache_tag = env.get("CACHE_TAG") or "cache"

    for name, cfg in (config.get("services") or {}).items():
        if not isinstance(cfg, dict):
            continue
        build = cfg.get("build")
        if not isinstance(build, dict) or not build:
            continue

        image = cfg.get("image")
        if not isinstance(image, str) or not image:
            continue

        fingerprint = compute_fingerprint(root, name, cfg, env)
        prev_image = previous_image_ref(image, version, previous_version)
        cache_image = cache_tag_from_build(build, cache_tag)

        action = "build"
        reason = "force_rebuild" if force_rebuild else "default_build"
        prev_fp = None
        if force_rebuild:
            action = "build"
            reason = "force_rebuild"
        elif not prev_image:
            action = "build"
            reason = "no_previous_image"
        else:
            prev_fp = inspect_fingerprint_label(prev_image)
            if prev_fp is None:
                action = "build"
                reason = "previous_missing_or_unlabeled"
            elif prev_fp == fingerprint:
                action = "retag"
                reason = "fingerprint_match"
            else:
                action = "build"
                reason = "fingerprint_mismatch"

        services_out[name] = {
            "image": image,
            "previous_image": prev_image,
            "cache_tag": cache_image,
            "fingerprint": fingerprint,
            "previous_fingerprint": prev_fp,
            "action": action,
            "reason": reason,
        }

    return {
        "previous_version": previous_version,
        "version": version,
        "profiles": profiles,
        "force_rebuild": force_rebuild,
        "label": FINGERPRINT_LABEL,
        "services": services_out,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=None, help="Repo root (default: AirStack/)")
    parser.add_argument("--version", default="", help="Current VERSION (default: from .env)")
    parser.add_argument("--previous-version", default="", help="Previous VERSION for retag source")
    parser.add_argument(
        "--profiles",
        default="",
        help="Compose profiles (default: COMPOSE_PROFILES or desktop,isaac-sim,ms-airsim)",
    )
    parser.add_argument(
        "--force-rebuild",
        action="store_true",
        help="Mark every service as build",
    )
    parser.add_argument(
        "--plan-out",
        type=Path,
        default=Path("docker-image-plan.json"),
        help="Where to write the plan JSON",
    )
    parser.add_argument(
        "--override-out",
        type=Path,
        default=Path("docker-compose.fingerprint.yaml"),
        help="Compose override with build labels for rebuild services",
    )
    parser.add_argument(
        "--github-output",
        type=Path,
        default=None,
        help="Optional path to append GITHUB_OUTPUT keys",
    )
    args = parser.parse_args(argv)

    root = (args.root or repo_root()).resolve()
    env = load_dotenv(root / ".env")
    # Prefer process env overlays (CI exports .env via set -a).
    for key in (
        "VERSION",
        "DOCKER_IMAGE_BUILD_MODE",
        "PROJECT_DOCKER_REGISTRY",
        "PROJECT_NAME",
        "CACHE_TAG",
        "COMPOSE_PROFILES",
    ):
        if os.environ.get(key):
            env[key] = os.environ[key]

    version = args.version or env.get("VERSION") or ""
    if not version:
        print("ERROR: VERSION is empty", file=sys.stderr)
        return 1

    previous_version = args.previous_version
    profiles = (
        args.profiles
        or os.environ.get("COMPOSE_PROFILES")
        or env.get("COMPOSE_PROFILES")
        or "desktop,isaac-sim,ms-airsim"
    )

    plan = build_plan(
        root,
        version=version,
        previous_version=previous_version,
        profiles=profiles,
        force_rebuild=args.force_rebuild,
        env=env,
    )

    build_services = {
        name: svc["fingerprint"]
        for name, svc in plan["services"].items()
        if svc["action"] == "build"
    }
    retag_services = [name for name, svc in plan["services"].items() if svc["action"] == "retag"]

    args.plan_out = args.plan_out if args.plan_out.is_absolute() else root / args.plan_out
    args.override_out = (
        args.override_out if args.override_out.is_absolute() else root / args.override_out
    )
    args.plan_out.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    write_override(args.override_out, build_services)

    print(f"Wrote plan → {args.plan_out}")
    print(f"Wrote override → {args.override_out}")
    for name, svc in sorted(plan["services"].items()):
        print(
            f"  {name}: action={svc['action']} reason={svc['reason']} "
            f"fp={svc['fingerprint'][:12]}…"
        )

    build_list = " ".join(sorted(build_services))
    retag_list = " ".join(sorted(retag_services))
    if args.github_output:
        with args.github_output.open("a", encoding="utf-8") as fh:
            fh.write(f"plan_path={args.plan_out}\n")
            fh.write(f"override_path={args.override_out}\n")
            fh.write(f"build_services={build_list}\n")
            fh.write(f"retag_services={retag_list}\n")
            fh.write(f"build_count={len(build_services)}\n")
            fh.write(f"retag_count={len(retag_services)}\n")
    else:
        # Also support GITHUB_OUTPUT env when set by Actions.
        gh_out = os.environ.get("GITHUB_OUTPUT")
        if gh_out:
            with open(gh_out, "a", encoding="utf-8") as fh:
                fh.write(f"plan_path={args.plan_out}\n")
                fh.write(f"override_path={args.override_out}\n")
                fh.write(f"build_services={build_list}\n")
                fh.write(f"retag_services={retag_list}\n")
                fh.write(f"build_count={len(build_services)}\n")
                fh.write(f"retag_count={len(retag_services)}\n")

    return 0


if __name__ == "__main__":
    sys.exit(main())
