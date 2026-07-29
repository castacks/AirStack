#!/usr/bin/env python3
"""AirStack CI orchestrator (OSMO backend).

Polls the GitHub API for queued workflow_jobs whose labels match this
orchestrator's runner_labels, and submits truly ephemeral OSMO workflows to
execute them. Each workflow runs a single-job GitHub Actions runner in a
privileged, GPU-enabled container on an OSMO compute pool; the GitHub PAT never
leaves this orchestrator, and an OSMO service-account token (not a personal
account) is used only to submit / query / cancel workflows.

This is a drop-in replacement for the previous OpenStack-Nova backend: the
GitHub side is unchanged (`runs-on: [self-hosted, airstack-ephemeral]`, the
single-use JIT runner config, the same-repo fork guard). Only the *spawn*
target changed from "create a Nova VM" to "submit an OSMO workflow". The
one-job-per-worker, destroy-after semantics are preserved — when the runner's
`run.sh` exits after a single job, the OSMO task completes and the pod is torn
down.

Two cooperating loops:
  - spawn loop: discover queued jobs, submit one OSMO workflow per job
  - reap loop:  cancel workflows whose jobs have completed, plus stragglers
                older than max_job_minutes and orphans not in state.json

State persists in /var/lib/airstack-orchestrator/state.json so the
orchestrator can survive restarts without leaking workflows.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import signal
import subprocess
import sys
import tempfile
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import requests
import yaml
from jinja2 import Template

DEFAULT_CONFIG_PATH = "/etc/airstack-orchestrator/config.yaml"
DEFAULT_PAT_PATH = "/etc/airstack-orchestrator/github-pat"
DEFAULT_OSMO_TOKEN_PATH = "/etc/airstack-orchestrator/osmo-token"
DEFAULT_STATE_PATH = "/var/lib/airstack-orchestrator/state.json"
DEFAULT_TEMPLATE_PATH = "/opt/airstack-orchestrator/runner-workflow.yaml.j2"

GITHUB_API = "https://api.github.com"

log = logging.getLogger("orchestrator")


# ── file / state helpers ────────────────────────────────────────────────────

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


def now_utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def parse_iso(s: str) -> datetime:
    return datetime.fromisoformat(s)


# ── GitHub API (unchanged from the OpenStack backend) ─────────────────────────

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


# ── OSMO CLI output parsing ───────────────────────────────────────────────────
#
# The exact JSON keys returned by `osmo workflow {submit,query,list}` can vary
# slightly by OSMO release, so these parsers try a set of likely keys and fall
# back to scraping the human-readable text output. Verify the keys against your
# deployed version with `osmo workflow submit --dry-run` / `--format-type json`
# once and simplify if desired.

# Live AirLab OSMO 6.2.x returns workflow_uuid on list/query; submit may use
# workflow_id / id / name. Prefer uuid-like keys before "name" so we don't
# accidentally treat the human workflow name as the id when both are present.
_WF_ID_KEYS = (
    "workflow_uuid", "workflow_id", "workflowId", "id", "uuid", "name", "workflow",
)
_STATUS_KEYS = ("status", "state", "workflow_status", "phase")
_KNOWN_STATUSES = {
    "RUNNING", "PENDING", "WAITING", "COMPLETED", "FAILED",
    "FAILED_EXEC_TIMEOUT", "FAILED_SERVER_ERROR", "FAILED_QUEUE_TIMEOUT",
    "FAILED_SUBMISSION", "FAILED_CANCELED", "FAILED_BACKEND_ERROR",
    "FAILED_IMAGE_PULL", "FAILED_EVICTED", "FAILED_START_ERROR",
    "FAILED_START_TIMEOUT", "FAILED_PREEMPTED",
}
# Non-terminal statuses the orphan sweep considers "still alive".
_ACTIVE_STATUSES = ("RUNNING", "PENDING", "WAITING")


def _loads_or_none(text: str | None) -> Any:
    try:
        return json.loads(text)  # type: ignore[arg-type]
    except (json.JSONDecodeError, TypeError):
        return None


def _first_str(d: dict, keys: tuple[str, ...]) -> str | None:
    for k in keys:
        v = d.get(k)
        if isinstance(v, str) and v:
            return v
    return None


def _extract_workflow_id(stdout: str | None) -> str | None:
    data = _loads_or_none(stdout)
    if isinstance(data, dict):
        wid = _first_str(data, _WF_ID_KEYS)
        if wid:
            return wid
        wf = data.get("workflow")
        if isinstance(wf, dict):
            wid = _first_str(wf, _WF_ID_KEYS)
            if wid:
                return wid
    m = re.search(r"Workflow\s*ID\s*[-:]\s*(\S+)", stdout or "", re.IGNORECASE)
    return m.group(1) if m else None


def _extract_status(stdout: str | None) -> str | None:
    data = _loads_or_none(stdout)
    if isinstance(data, dict):
        st = _first_str(data, _STATUS_KEYS)
        if st:
            return st.upper()
        wf = data.get("workflow")
        if isinstance(wf, dict):
            st = _first_str(wf, _STATUS_KEYS)
            if st:
                return st.upper()
    up = (stdout or "").upper()
    for s in sorted(_KNOWN_STATUSES, key=len, reverse=True):
        if s in up:
            return s
    return None


def _extract_workflow_list(stdout: str | None) -> list[dict]:
    data = _loads_or_none(stdout)
    if isinstance(data, dict):
        for key in ("workflows", "items", "results", "data"):
            if isinstance(data.get(key), list):
                data = data[key]
                break
    items: list[dict] = []
    if isinstance(data, list):
        for entry in data:
            if not isinstance(entry, dict):
                continue
            wid = _first_str(entry, _WF_ID_KEYS)
            name = entry.get("name") if isinstance(entry.get("name"), str) else None
            status = _first_str(entry, _STATUS_KEYS)
            if wid or name:
                items.append(
                    {"id": wid, "name": name,
                     "status": status.upper() if status else None}
                )
    return items


def _is_terminal(status: str | None) -> bool:
    if not status:
        return False
    return status == "COMPLETED" or status.startswith("FAILED")


def _looks_like_auth_error(r: subprocess.CompletedProcess) -> bool:
    blob = f"{r.stdout or ''}\n{r.stderr or ''}".lower()
    markers = ("401", "403", "unauthorized", "forbidden", "expired",
               "not logged in", "please login", "authentication",
               "invalid token", "token is invalid")
    return any(m in blob for m in markers)


def _name_age_minutes(name: str | None) -> float | None:
    """Age in minutes parsed from our `...-<unix_ts>` name suffix, or None.

    OSMO may append its own suffix after the name we submit, so we match the
    first 10+ digit run (the unix timestamp) even when trailing chars follow.
    """
    m = re.search(r"-(\d{10,})(?:\D.*)?$", name or "")
    if not m:
        return None
    try:
        ts = int(m.group(1))
    except ValueError:
        return None
    return (time.time() - ts) / 60.0


# ── orchestrator ──────────────────────────────────────────────────────────────

class Orchestrator:
    def __init__(self, config: dict, pat: str, state_path: str, template_path: str):
        self.config = config
        self.pat = pat
        self.state_path = state_path
        self.template_path = template_path

        # OSMO target.
        self.osmo_bin = config.get("osmo_bin", "osmo")
        self.osmo_url = config["osmo_url"]
        self.token_file = config.get("osmo_token_file", DEFAULT_OSMO_TOKEN_PATH)
        self.pool = config["pool"]
        self.platform = config.get("platform", "") or ""
        self.priority = str(config.get("priority", "NORMAL")).upper()

        # Runner task shape.
        self.runner_image = config["runner_image"]
        self.cpu = config.get("cpu", 8)
        self.gpu = config.get("gpu", 1)
        self.memory = config.get("memory", "32Gi")
        self.storage = config.get("storage", "300Gi")
        self.privileged = bool(config.get("privileged", True))
        self.host_network = bool(config.get("host_network", False))

        # GitHub.
        self.repo = config["repo"]
        self.runner_labels = config["runner_labels"]

        # Limits / timing.
        self.max_concurrent = int(config.get("max_concurrent", 3))
        self.max_job_minutes = int(config.get("max_job_minutes", 2880))
        self.spawn_interval = int(config.get("spawn_poll_interval_s", 15))
        self.reap_interval = int(config.get("reap_poll_interval_s", 30))
        self.submit_timeout = int(config.get("submit_timeout_s", 180))
        self.workflow_prefix = config.get("workflow_name_prefix", "gha-runner-")

        self.stop_evt = threading.Event()

        # Establish the OSMO session up-front for early feedback; individual
        # commands re-login on demand if the session lapses.
        self._login()

    def stop(self, *_: Any) -> None:
        log.info("stop signal received; draining loops")
        self.stop_evt.set()

    # ── OSMO CLI plumbing ────────────────────────────────────────────────────

    def _run_osmo(self, args: list[str], timeout: int) -> subprocess.CompletedProcess:
        return subprocess.run(
            [self.osmo_bin, *args],
            capture_output=True, text=True, timeout=timeout,
        )

    def _login(self) -> bool:
        try:
            r = self._run_osmo(
                ["login", self.osmo_url, "--method", "token",
                 "--token-file", self.token_file],
                timeout=60,
            )
        except Exception as e:  # noqa: BLE001 - startup best-effort
            log.warning("osmo login raised: %s", e)
            return False
        if r.returncode != 0:
            log.warning(
                "osmo login failed (rc=%d): %s",
                r.returncode, (r.stderr or r.stdout).strip(),
            )
            return False
        log.info("osmo login succeeded (url=%s, token_file=%s)",
                 self.osmo_url, self.token_file)
        return True

    def _osmo(self, args: list[str], timeout: int,
              relogin: bool = True) -> subprocess.CompletedProcess:
        """Run an osmo CLI command, re-logging-in once on an auth failure."""
        r = self._run_osmo(args, timeout=timeout)
        if r.returncode != 0 and relogin and _looks_like_auth_error(r):
            log.info("osmo command hit an auth error; re-logging in and retrying")
            if self._login():
                r = self._run_osmo(args, timeout=timeout)
        return r

    def submit_workflow(self, workflow_file: str) -> tuple[str, str]:
        """Submit a workflow. Returns (workflow_id, live_name).

        AirLab OSMO 6.2 submit JSON is typically only {name, overview, logs}
        (no uuid), and the service may append a numeric suffix to the name
        (e.g. ``...-1``). We immediately query to resolve uuid + live name so
        state/reap stay consistent with ``workflow list`` (``workflow_uuid``).
        """
        args = ["workflow", "submit", workflow_file, "--pool", self.pool,
                "--priority", self.priority, "--format-type", "json"]
        r = self._osmo(args, timeout=self.submit_timeout)
        if r.returncode != 0:
            raise RuntimeError(
                f"osmo workflow submit failed (rc={r.returncode}): "
                f"{(r.stderr or r.stdout).strip()}"
            )
        submitted_name = _extract_workflow_id(r.stdout) or _extract_workflow_id(r.stderr)
        if not submitted_name:
            raise RuntimeError(
                "could not parse workflow id/name from submit output: "
                f"{(r.stdout or '').strip()[:500]}"
            )
        live_name, uuid = submitted_name, None
        q = self._osmo(
            ["workflow", "query", submitted_name, "--format-type", "json"],
            timeout=60,
        )
        if q.returncode == 0:
            data = _loads_or_none(q.stdout) or _loads_or_none(q.stderr)
            if isinstance(data, dict):
                if isinstance(data.get("name"), str) and data["name"]:
                    live_name = data["name"]
                for k in ("uuid", "workflow_uuid", "workflow_id", "id"):
                    v = data.get(k)
                    if isinstance(v, str) and v:
                        uuid = v
                        break
        return (uuid or live_name), live_name

    def query_status(self, workflow_id: str) -> str | None:
        r = self._osmo(
            ["workflow", "query", workflow_id, "--format-type", "json"],
            timeout=60,
        )
        if r.returncode != 0:
            log.debug("osmo workflow query %s failed: %s",
                      workflow_id, (r.stderr or r.stdout).strip())
            return None
        return _extract_status(r.stdout) or _extract_status(r.stderr)

    def cancel_workflow(self, workflow_id: str) -> None:
        r = self._osmo(
            ["workflow", "cancel", workflow_id, "--force",
             "--message", "orchestrator reap", "--format-type", "json"],
            timeout=60,
        )
        if r.returncode != 0:
            log.warning("osmo workflow cancel %s failed (rc=%d): %s",
                        workflow_id, r.returncode, (r.stderr or r.stdout).strip())

    def list_runner_workflows(self) -> list[dict]:
        """Active workflows (RUNNING/PENDING/WAITING) named with our prefix."""
        r = self._osmo(
            ["workflow", "list", "--name", self.workflow_prefix,
             "--pool", self.pool, "--count", "100",
             "--status", *_ACTIVE_STATUSES, "--format-type", "json"],
            timeout=60,
        )
        if r.returncode != 0:
            log.warning("osmo workflow list failed: %s",
                        (r.stderr or r.stdout).strip())
            return []
        return _extract_workflow_list(r.stdout)

    # ── workflow rendering ───────────────────────────────────────────────────

    def render_workflow(self, workflow_name: str, encoded_jit_config: str) -> str:
        with open(self.template_path) as f:
            tmpl = Template(f.read())
        return tmpl.render(
            workflow_name=workflow_name,
            runner_image=self.runner_image,
            cpu=self.cpu,
            gpu=self.gpu,
            memory=self.memory,
            storage=self.storage,
            platform=self.platform,
            privileged="true" if self.privileged else "false",
            host_network="true" if self.host_network else "false",
            encoded_jit_config=encoded_jit_config,
            runner_labels=self.runner_labels,
        )

    def _write_temp_workflow(self, name: str, content: str) -> str:
        fd, path = tempfile.mkstemp(prefix=f"{name}-", suffix=".yaml")
        with os.fdopen(fd, "w") as f:
            f.write(content)
        return path

    # ── loops ────────────────────────────────────────────────────────────────

    def spawn_once(self) -> None:
        state = load_state(self.state_path)
        active = len(state["jobs"])
        if active >= self.max_concurrent:
            return
        try:
            queued = find_queued_jobs(self.repo, self.runner_labels, self.pat)
        except Exception as e:  # noqa: BLE001
            log.warning("find_queued_jobs failed: %s", e)
            return

        for job in queued:
            if active >= self.max_concurrent:
                break
            job_id = job["job_id"]
            if job_id in state["jobs"]:
                continue

            ts = int(time.time())
            # OSMO workflow name doubles as the JIT runner registration name.
            workflow_name = f"{self.workflow_prefix}{job_id}-{ts}"
            tmp_path: str | None = None
            try:
                jit = mint_jit_config(
                    self.repo, workflow_name, self.runner_labels, self.pat
                )
                workflow_yaml = self.render_workflow(workflow_name, jit)
                tmp_path = self._write_temp_workflow(workflow_name, workflow_yaml)
                workflow_id, live_name = self.submit_workflow(tmp_path)
            except Exception as e:  # noqa: BLE001
                log.exception("submit failed for job %s: %s", job_id, e)
                continue
            finally:
                if tmp_path:
                    try:
                        os.remove(tmp_path)
                    except OSError:
                        pass

            state["jobs"][job_id] = {
                "run_id": job["run_id"],
                "workflow_id": workflow_id,
                "workflow_name": live_name,
                "runner_name": workflow_name,
                "submitted_at": now_utc_iso(),
                "name": job["name"],
            }
            save_state(self.state_path, state)
            active += 1
            log.info(
                "submitted workflow %s for job %s (%s)",
                workflow_id, job_id, job["name"],
            )

    def reap_once(self) -> None:
        state = load_state(self.state_path)
        now = datetime.now(timezone.utc)

        # 1. Cancel workflows for completed / purged jobs.
        for job_id in list(state["jobs"].keys()):
            entry = state["jobs"][job_id]
            wid = entry["workflow_id"]
            try:
                job = get_job_status(self.repo, job_id, self.pat)
            except Exception as e:  # noqa: BLE001
                log.warning("get_job_status(%s) failed: %s", job_id, e)
                continue

            if job is None or job.get("status") == "completed":
                # The runner usually exits on its own (task self-completes and
                # the pod is torn down); only cancel if it's somehow still live.
                status = self.query_status(wid)
                if not _is_terminal(status):
                    log.info("reaping workflow %s (job %s done, wf status=%s)",
                             wid, job_id, status)
                    self.cancel_workflow(wid)
                else:
                    log.info("workflow %s already terminal (%s) for job %s",
                             wid, status, job_id)
                del state["jobs"][job_id]
                continue

            # 2. Force-reap stragglers older than max_job_minutes.
            age_min = (now - parse_iso(entry["submitted_at"])).total_seconds() / 60.0
            if age_min > self.max_job_minutes:
                log.warning(
                    "force-reaping workflow %s (job %s age %.1fm > %dm)",
                    wid, job_id, age_min, self.max_job_minutes,
                )
                self.cancel_workflow(wid)
                del state["jobs"][job_id]

        save_state(self.state_path, state)

        # 3. Orphan sweep: our-named workflows still active but absent from
        #    state (catches state.json wipes and crashes between submit and
        #    save_state). Skip very fresh ones so we don't race our own submit.
        try:
            listed = self.list_runner_workflows()
        except Exception as e:  # noqa: BLE001
            log.warning("list_runner_workflows failed: %s", e)
            return
        tracked_ids = {e["workflow_id"] for e in state["jobs"].values()}
        tracked_names = {e["workflow_name"] for e in state["jobs"].values()}
        for wf in listed:
            wid, wname = wf.get("id"), wf.get("name")
            if (wid and wid in tracked_ids) or (wname and wname in tracked_names):
                continue
            age = _name_age_minutes(wname)
            if age is not None and age < 2:
                continue
            target = wid or wname
            if not target:
                continue
            log.warning("orphan-reaping workflow %s (not in state)", target)
            self.cancel_workflow(target)

    def run(self) -> None:
        log.info(
            "orchestrator started (OSMO backend): repo=%s labels=%s pool=%s "
            "platform=%s max_concurrent=%d",
            self.repo, self.runner_labels, self.pool,
            self.platform or "(pool default)", self.max_concurrent,
        )
        last_spawn = 0.0
        last_reap = 0.0
        while not self.stop_evt.is_set():
            now = time.monotonic()
            if now - last_spawn >= self.spawn_interval:
                try:
                    self.spawn_once()
                except Exception:  # noqa: BLE001
                    log.exception("spawn loop iteration failed")
                last_spawn = now
            if now - last_reap >= self.reap_interval:
                try:
                    self.reap_once()
                except Exception:  # noqa: BLE001
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
