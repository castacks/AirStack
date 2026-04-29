#!/usr/bin/env python3
"""AirStack CI orchestrator.

Polls the GitHub API for queued workflow_jobs whose labels match this
orchestrator's runner_labels, and spawns truly ephemeral OpenStack instances
to execute them. Each ephemeral instance receives a single-use GitHub JIT
runner config via cloud-init; the GitHub PAT never leaves this orchestrator.

Two cooperating loops:
  - spawn loop: discover queued jobs, spawn one Nova server per job
  - reap loop:  delete servers whose jobs have completed, plus stragglers
                older than max_job_minutes and orphans not in state.json

State persists in /var/lib/airstack-orchestrator/state.json so the
orchestrator can survive restarts without leaking instances.
"""

from __future__ import annotations

import argparse
import base64
import json
import logging
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import openstack
import requests
import yaml
from jinja2 import Template

DEFAULT_CONFIG_PATH = "/etc/airstack-orchestrator/config.yaml"
DEFAULT_PAT_PATH = "/etc/airstack-orchestrator/github-pat"
DEFAULT_STATE_PATH = "/var/lib/airstack-orchestrator/state.json"
DEFAULT_TEMPLATE_PATH = "/opt/airstack-orchestrator/cloud-init.yaml.j2"

# Metadata key/value applied to every Nova server we spawn. Used by the
# orphan reaper to identify servers we own even when state.json is missing.
ROLE_META_KEY = "airstack-role"
ROLE_META_VAL = "ephemeral-runner"
JOB_META_KEY = "airstack-job-id"

GITHUB_API = "https://api.github.com"

log = logging.getLogger("orchestrator")


def load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def load_pat(path: str) -> str:
    with open(path) as f:
        return f.read().strip()


def load_state(path: str) -> dict:
    if not os.path.exists(path):
        return {"jobs": {}}
    with open(path) as f:
        return json.load(f)


def save_state(path: str, state: dict) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f, indent=2, sort_keys=True)
    os.replace(tmp, path)


def gh_request(method: str, path: str, pat: str, **kwargs: Any) -> Any:
    url = f"{GITHUB_API}{path}"
    headers = kwargs.pop("headers", {})
    headers.update(
        {
            "Authorization": f"Bearer {pat}",
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
        }
    )
    r = requests.request(method, url, headers=headers, timeout=30, **kwargs)
    r.raise_for_status()
    if not r.text:
        return None
    return r.json()


def find_queued_jobs(repo: str, runner_labels: list[str], pat: str) -> list[dict]:
    """Return queued workflow_jobs whose labels include all runner_labels."""
    runs = gh_request(
        "GET", f"/repos/{repo}/actions/runs?status=queued&per_page=20", pat
    )
    label_set = set(runner_labels)
    matches: list[dict] = []
    for run in runs.get("workflow_runs", []):
        jobs = gh_request("GET", f"/repos/{repo}/actions/runs/{run['id']}/jobs", pat)
        for job in jobs.get("jobs", []):
            if job.get("status") != "queued":
                continue
            if not label_set.issubset(set(job.get("labels", []))):
                continue
            if job.get("runner_id"):
                continue
            matches.append(
                {
                    "job_id": str(job["id"]),
                    "run_id": run["id"],
                    "name": job["name"],
                    "labels": job["labels"],
                }
            )
    return matches


def mint_jit_config(
    repo: str, runner_name: str, runner_labels: list[str], pat: str,
    runner_group_id: int = 1,
) -> str:
    body = {
        "name": runner_name,
        "runner_group_id": runner_group_id,
        "labels": runner_labels,
    }
    resp = gh_request(
        "POST",
        f"/repos/{repo}/actions/runners/generate-jitconfig",
        pat,
        json=body,
    )
    return resp["encoded_jit_config"]


def get_job_status(repo: str, job_id: str, pat: str) -> dict | None:
    """Return the job dict, or None if 404 (job purged)."""
    url = f"{GITHUB_API}/repos/{repo}/actions/jobs/{job_id}"
    r = requests.get(
        url,
        headers={
            "Authorization": f"Bearer {pat}",
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
        },
        timeout=30,
    )
    if r.status_code == 404:
        return None
    r.raise_for_status()
    return r.json()


def render_cloud_init(template_path: str, encoded_jit_config: str,
                      runner_version: str) -> str:
    with open(template_path) as f:
        tmpl = Template(f.read())
    return tmpl.render(
        encoded_jit_config=encoded_jit_config,
        runner_version=runner_version,
    )


def spawn_server(
    conn: openstack.connection.Connection,
    config: dict,
    name: str,
    job_id: str,
    user_data: str,
) -> str:
    flavor = conn.compute.find_flavor(config["flavor_name"], ignore_missing=False)
    network = conn.network.find_network(config["network_name"], ignore_missing=False)
    create_kwargs = dict(
        name=name,
        flavor_id=flavor.id,
        networks=[{"uuid": network.id}],
        key_name=config["keypair_name"],
        security_groups=[{"name": config["security_group"]}],
        user_data=base64.b64encode(user_data.encode()).decode(),
        metadata={
            ROLE_META_KEY: ROLE_META_VAL,
            JOB_META_KEY: job_id,
        },
    )

    # Flavors with disk=0 (typical for GPU flavors on this cloud) cannot boot
    # directly from an image — Nova rejects with "Block Device Mapping is
    # Invalid: You specified more local devices than the limit allows". When
    # boot_volume_size_gb is set, boot from a Cinder volume sourced from the
    # image and delete it on termination. Otherwise fall back to direct image
    # boot (works only when the flavor has a non-zero root disk).
    boot_volume_size_gb = int(config.get("boot_volume_size_gb") or 0)
    if boot_volume_size_gb > 0:
        create_kwargs["block_device_mapping"] = [
            {
                "uuid": config["image_id"],
                "source_type": "image",
                "destination_type": "volume",
                "boot_index": 0,
                "volume_size": boot_volume_size_gb,
                "delete_on_termination": True,
            }
        ]
    else:
        create_kwargs["image_id"] = config["image_id"]

    az = config.get("availability_zone")
    if az:
        create_kwargs["availability_zone"] = az
    server = conn.compute.create_server(**create_kwargs)
    return server.id


def delete_server(conn: openstack.connection.Connection, server_id: str) -> None:
    try:
        conn.compute.delete_server(server_id, ignore_missing=True, force=True)
    except Exception as e:
        log.warning("delete_server(%s) failed: %s", server_id, e)


def list_owned_servers(conn: openstack.connection.Connection) -> list[Any]:
    """List all Nova servers that carry our role metadata."""
    owned = []
    for s in conn.compute.servers(details=True):
        meta = getattr(s, "metadata", None) or {}
        if meta.get(ROLE_META_KEY) == ROLE_META_VAL:
            owned.append(s)
    return owned


def find_free_floating_ip(
    conn: openstack.connection.Connection, pool: list[str]
) -> Any:
    """Return the FloatingIP resource for the first address in `pool` that is
    not currently associated with any port. Returns None if all are in use.

    Iterates `pool` in order so attachments rotate through it sequentially.
    Logs a warning for any pool member that doesn't exist in this project.
    """
    if not pool:
        return None
    pool_set = set(pool)
    fips_by_addr: dict[str, Any] = {}
    for fip in conn.network.ips():
        if fip.floating_ip_address in pool_set:
            fips_by_addr[fip.floating_ip_address] = fip
    missing = pool_set - fips_by_addr.keys()
    if missing:
        log.warning(
            "floating_ips configured but not found in this project: %s",
            sorted(missing),
        )
    for addr in pool:
        fip = fips_by_addr.get(addr)
        if fip is not None and not fip.port_id:
            return fip
    return None


def attach_floating_ip(
    conn: openstack.connection.Connection, server_id: str, fip: Any
) -> str:
    """Wait for the server to have a network port, then associate `fip`.
    Returns the floating IP address."""
    for _ in range(60):  # ~120s
        ports = list(conn.network.ports(device_id=server_id))
        if ports:
            break
        time.sleep(2)
    else:
        raise RuntimeError(f"server {server_id} got no network port within 120s")
    conn.network.update_ip(fip, port_id=ports[0].id)
    return fip.floating_ip_address


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_iso(s: str) -> datetime:
    return datetime.fromisoformat(s)


class Orchestrator:
    def __init__(self, config: dict, pat: str, state_path: str, template_path: str):
        self.config = config
        self.pat = pat
        self.state_path = state_path
        self.template_path = template_path
        self.conn = openstack.connect(cloud=config.get("openstack_cloud", "airstack"))
        self.repo = config["repo"]
        self.runner_labels = config["runner_labels"]
        self.runner_version = config["runner_version"]
        self.max_concurrent = int(config.get("max_concurrent", 3))
        self.floating_ips: list[str] = list(config.get("floating_ips") or [])
        # Cap spawns to FIP pool size so we never queue jobs we can't address.
        self.effective_max_concurrent = self.max_concurrent
        if self.floating_ips:
            self.effective_max_concurrent = min(
                self.max_concurrent, len(self.floating_ips)
            )
        self.max_job_minutes = int(config.get("max_job_minutes", 90))
        self.spawn_interval = int(config.get("spawn_poll_interval_s", 15))
        self.reap_interval = int(config.get("reap_poll_interval_s", 30))
        self.stop_evt = threading.Event()

    def stop(self, *_: Any) -> None:
        log.info("stop signal received; draining loops")
        self.stop_evt.set()

    def spawn_once(self) -> None:
        state = load_state(self.state_path)
        active = len(state["jobs"])
        if active >= self.effective_max_concurrent:
            return
        try:
            queued = find_queued_jobs(self.repo, self.runner_labels, self.pat)
        except Exception as e:
            log.warning("find_queued_jobs failed: %s", e)
            return

        for job in queued:
            if active >= self.effective_max_concurrent:
                break
            job_id = job["job_id"]
            if job_id in state["jobs"]:
                continue

            # Pre-check FIP availability before minting a JIT token so we
            # don't burn one when there's nowhere to attach the worker.
            reserved_fip = None
            if self.floating_ips:
                reserved_fip = find_free_floating_ip(self.conn, self.floating_ips)
                if reserved_fip is None:
                    log.warning(
                        "no free floating IP in pool (%d configured); "
                        "deferring spawns until one frees up",
                        len(self.floating_ips),
                    )
                    break

            ts = int(time.time())
            runner_name = f"ephemeral-{job_id}-{ts}"
            try:
                jit = mint_jit_config(
                    self.repo, runner_name, self.runner_labels, self.pat
                )
                user_data = render_cloud_init(
                    self.template_path, jit, self.runner_version
                )
                server_id = spawn_server(
                    self.conn, self.config, runner_name, job_id, user_data
                )
            except Exception as e:
                log.exception("spawn failed for job %s: %s", job_id, e)
                continue

            floating_ip_addr: str | None = None
            if reserved_fip is not None:
                try:
                    floating_ip_addr = attach_floating_ip(
                        self.conn, server_id, reserved_fip
                    )
                    log.info(
                        "attached floating IP %s to server %s (job %s)",
                        floating_ip_addr, server_id, job_id,
                    )
                except Exception as e:
                    log.exception(
                        "FIP attach failed for server %s; deleting to avoid "
                        "leaking a worker without external access: %s",
                        server_id, e,
                    )
                    delete_server(self.conn, server_id)
                    continue

            state["jobs"][job_id] = {
                "run_id": job["run_id"],
                "server_id": server_id,
                "runner_name": runner_name,
                "spawned_at": now_utc_iso(),
                "name": job["name"],
                "floating_ip": floating_ip_addr,
            }
            save_state(self.state_path, state)
            active += 1
            log.info(
                "spawned server %s for job %s (%s)", server_id, job_id, job["name"]
            )

    def reap_once(self) -> None:
        state = load_state(self.state_path)
        now = datetime.now(timezone.utc)

        # 1. Delete servers for completed jobs.
        for job_id in list(state["jobs"].keys()):
            entry = state["jobs"][job_id]
            try:
                job = get_job_status(self.repo, job_id, self.pat)
            except Exception as e:
                log.warning("get_job_status(%s) failed: %s", job_id, e)
                continue
            if job is None or job.get("status") == "completed":
                log.info("reaping server %s (job %s done)", entry["server_id"], job_id)
                delete_server(self.conn, entry["server_id"])
                del state["jobs"][job_id]
                continue

            # 2. Force-reap stragglers older than max_job_minutes.
            spawned = parse_iso(entry["spawned_at"])
            age_min = (now - spawned).total_seconds() / 60.0
            if age_min > self.max_job_minutes:
                log.warning(
                    "force-reaping server %s (job %s age %.1fm > %dm)",
                    entry["server_id"], job_id, age_min, self.max_job_minutes,
                )
                delete_server(self.conn, entry["server_id"])
                del state["jobs"][job_id]

        save_state(self.state_path, state)

        # 3. Orphan sweep: any server we own that isn't in state and isn't
        #    in the brief just-spawned window. Catches state.json wipes and
        #    crashes between spawn and save_state.
        try:
            owned = list_owned_servers(self.conn)
        except Exception as e:
            log.warning("list_owned_servers failed: %s", e)
            return
        tracked_ids = {e["server_id"] for e in state["jobs"].values()}
        for s in owned:
            if s.id in tracked_ids:
                continue
            created = getattr(s, "created_at", None)
            if created:
                try:
                    age_min = (now - parse_iso(created.replace("Z", "+00:00"))).total_seconds() / 60.0
                except Exception:
                    age_min = self.max_job_minutes + 1
            else:
                age_min = self.max_job_minutes + 1
            # Only reap orphans that have lived past one spawn interval
            # (to avoid racing our own freshly-created server).
            if age_min < 2:
                continue
            log.warning(
                "orphan-reaping server %s (not in state, age %.1fm)", s.id, age_min
            )
            delete_server(self.conn, s.id)

    def run(self) -> None:
        log.info(
            "orchestrator started: repo=%s labels=%s max_concurrent=%d "
            "(effective=%d, floating_ip_pool=%d)",
            self.repo, self.runner_labels, self.max_concurrent,
            self.effective_max_concurrent, len(self.floating_ips),
        )
        last_spawn = 0.0
        last_reap = 0.0
        while not self.stop_evt.is_set():
            now = time.monotonic()
            if now - last_spawn >= self.spawn_interval:
                try:
                    self.spawn_once()
                except Exception:
                    log.exception("spawn loop iteration failed")
                last_spawn = now
            if now - last_reap >= self.reap_interval:
                try:
                    self.reap_once()
                except Exception:
                    log.exception("reap loop iteration failed")
                last_reap = now
            self.stop_evt.wait(timeout=1.0)
        log.info("orchestrator stopped")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--pat", default=DEFAULT_PAT_PATH)
    parser.add_argument("--state", default=DEFAULT_STATE_PATH)
    parser.add_argument("--template", default=DEFAULT_TEMPLATE_PATH)
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper()),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        stream=sys.stdout,
    )

    config = load_yaml(args.config)
    pat = load_pat(args.pat)

    orch = Orchestrator(config, pat, args.state, args.template)
    signal.signal(signal.SIGINT, orch.stop)
    signal.signal(signal.SIGTERM, orch.stop)
    orch.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
