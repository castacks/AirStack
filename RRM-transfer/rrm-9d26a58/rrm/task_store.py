"""Persistent immutable goal/run index and allow-listed evidence links.

This module owns storage state only. It has no ROS, model, subprocess, or execution
authority. Artifact files remain the source of truth; the database indexes their
checksums and lifecycle so the console can render one attempt at a time.
"""
from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import uuid


RUN_STATUSES = frozenset({
    "SAVED_NOT_SUBMITTED", "INFERENCE_QUEUED", "INFERENCE_RUNNING",
    "CANDIDATE_ACCEPTED", "CANDIDATE_REJECTED", "INFERENCE_FAILED",
})
EXECUTION_STATES = frozenset({
    "NOT_DISPATCHED", "REVIEW_REQUIRED", "APPROVED", "DISPATCHING", "FINISHED",
})


class TaskStore:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as db:
            self._migrate_runs(db)
            db.executescript("""
                CREATE TABLE IF NOT EXISTS goals (
                    goal_id TEXT PRIMARY KEY,
                    objective TEXT NOT NULL,
                    constraints_revision TEXT NOT NULL,
                    embodiment_id TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS run_events (
                    event_id TEXT PRIMARY KEY,
                    run_id TEXT NOT NULL REFERENCES runs(run_id),
                    kind TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    artifact_path TEXT NOT NULL,
                    sha256 TEXT NOT NULL,
                    summary_json TEXT NOT NULL,
                    UNIQUE(run_id, kind, artifact_path)
                );
                CREATE INDEX IF NOT EXISTS runs_by_goal ON runs(goal_id, created_at);
                CREATE INDEX IF NOT EXISTS events_by_run ON run_events(run_id, created_at, event_id);
            """)

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=10)
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA foreign_keys=ON")
        try:
            with db:
                yield db
        finally:
            db.close()

    @staticmethod
    def _create_runs_table(db: sqlite3.Connection) -> None:
        db.execute("""
            CREATE TABLE runs (
                run_id TEXT PRIMARY KEY,
                goal_id TEXT NOT NULL REFERENCES goals(goal_id),
                task_id TEXT NOT NULL,
                created_at TEXT NOT NULL,
                status TEXT NOT NULL CHECK(status IN
                    ('SAVED_NOT_SUBMITTED','INFERENCE_QUEUED','INFERENCE_RUNNING',
                     'CANDIDATE_ACCEPTED','CANDIDATE_REJECTED','INFERENCE_FAILED')),
                artifact_dir TEXT NOT NULL,
                execution_state TEXT NOT NULL DEFAULT 'NOT_DISPATCHED'
                    CHECK(execution_state IN
                        ('NOT_DISPATCHED','REVIEW_REQUIRED','APPROVED','DISPATCHING','FINISHED')),
                psc_job_id TEXT,
                candidate_sha256 TEXT,
                proposal_sha256 TEXT
            )
        """)

    def _migrate_runs(self, db: sqlite3.Connection) -> None:
        """Add lifecycle columns without losing the console's existing local history."""
        row = db.execute("SELECT sql FROM sqlite_master WHERE type='table' AND name='runs'").fetchone()
        if row is None:
            self._create_runs_table(db)
            return
        sql = row["sql"] or ""
        needed = ("INFERENCE_QUEUED", "psc_job_id", "candidate_sha256", "proposal_sha256")
        if all(item in sql for item in needed):
            return
        db.execute("ALTER TABLE runs RENAME TO runs_legacy")
        self._create_runs_table(db)
        db.execute("""
            INSERT INTO runs (run_id, goal_id, task_id, created_at, status, artifact_dir, execution_state)
            SELECT run_id, goal_id, task_id, created_at, status, artifact_dir, execution_state
            FROM runs_legacy
        """)
        db.execute("DROP TABLE runs_legacy")

    def record(self, *, goal_id, run_id, task, created_at, status, artifact_dir):
        if status not in RUN_STATUSES:
            raise ValueError("Invalid run status.")
        if not all(isinstance(key, str) and re.fullmatch(r"[0-9a-f]{32}", key)
                   for key in (goal_id, run_id)):
            raise ValueError("Invalid goal or run ID.")
        goal = (goal_id, task["objective"], task["constraints_revision"],
                task["requested_embodiment_id"], created_at)
        run = (run_id, goal_id, task["task_id"], created_at, status, str(artifact_dir.resolve()))
        with self.connect() as db:
            db.execute("INSERT OR IGNORE INTO goals VALUES (?, ?, ?, ?, ?)", goal)
            existing = db.execute("SELECT * FROM goals WHERE goal_id=?", (goal_id,)).fetchone()
            if tuple(existing)[:4] != goal[:4]:
                raise ValueError("Saved goals are immutable; save edited text as a new goal.")
            db.execute("""INSERT OR IGNORE INTO runs
                (run_id,goal_id,task_id,created_at,status,artifact_dir) VALUES (?,?,?,?,?,?)""", run)
            existing_run = db.execute("SELECT * FROM runs WHERE run_id=?", (run_id,)).fetchone()
            if tuple(existing_run)[:6] != run:
                raise ValueError("Run ID already refers to different evidence.")

    def record_request(self, manifest: dict, payload: dict, directory: Path):
        if manifest["status"] != "SAVED_NOT_SUBMITTED" or manifest["execution_dispatch"] is not False:
            raise ValueError("Only unsent request artifacts may be indexed as saved requests.")
        if manifest["task_id"] != payload["task"]["task_id"]:
            raise ValueError("Request manifest/task mismatch.")
        self.record(goal_id=manifest.get("goal_id", manifest["request_id"]),
                    run_id=manifest["request_id"], task=payload["task"],
                    created_at=manifest["created_at"], status=manifest["status"], artifact_dir=directory)

    def record_reference(self, bundle: Path) -> str:
        raw = (bundle / "result.json").read_bytes()
        record = json.loads(raw)
        if record.get("execution_dispatch") is not False or record["candidate"]["status"] != "ACCEPTED":
            raise ValueError("Reference must be a verified, unexecuted accepted candidate.")
        task = json.loads((bundle / "input.json").read_text())["task"]
        run_id = uuid.uuid5(uuid.NAMESPACE_URL, hashlib.sha256(raw).hexdigest()).hex
        goal_id = uuid.uuid5(uuid.NAMESPACE_URL, "rrm-reference:" + run_id).hex
        with self.connect() as db:
            existing = db.execute("SELECT created_at FROM runs WHERE run_id=?", (run_id,)).fetchone()
        created_at = existing[0] if existing else datetime.now(timezone.utc).isoformat()
        self.record(goal_id=goal_id, run_id=run_id, task=task, created_at=created_at,
                    status="CANDIDATE_ACCEPTED", artifact_dir=bundle)
        # The imported historical proposal has its separate legacy execution panel;
        # it must never appear as a newly reviewable candidate for a different goal.
        self.set_lifecycle(run_id, status="CANDIDATE_ACCEPTED", execution_state="NOT_DISPATCHED",
                           candidate_sha256=hashlib.sha256(raw).hexdigest())
        return run_id

    def set_lifecycle(self, run_id: str, *, status: str | None = None,
                      execution_state: str | None = None, psc_job_id: str | None = None,
                      candidate_sha256: str | None = None, proposal_sha256: str | None = None) -> None:
        if status is not None and status not in RUN_STATUSES:
            raise ValueError("Invalid run status.")
        if execution_state is not None and execution_state not in EXECUTION_STATES:
            raise ValueError("Invalid execution state.")
        fields, values = [], []
        for column, value in (("status", status), ("execution_state", execution_state),
                              ("psc_job_id", psc_job_id), ("candidate_sha256", candidate_sha256),
                              ("proposal_sha256", proposal_sha256)):
            if value is not None:
                fields.append(f"{column}=?")
                values.append(value)
        if not fields:
            return
        values.append(run_id)
        with self.connect() as db:
            if db.execute(f"UPDATE runs SET {', '.join(fields)} WHERE run_id=?", values).rowcount != 1:
                raise ValueError("Saved run not found.")

    def record_event(self, run_id: str, *, kind: str, artifact_path: Path, summary: dict) -> str:
        if not isinstance(kind, str) or not re.fullmatch(r"[a-z0-9][a-z0-9_-]{1,80}", kind):
            raise ValueError("Invalid evidence kind.")
        path = artifact_path.resolve()
        if not path.is_file():
            raise ValueError("Evidence artifact is missing.")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        event_id = uuid.uuid5(uuid.NAMESPACE_URL, f"rrm-event:{run_id}:{kind}:{path}:{digest}").hex
        created_at = datetime.now(timezone.utc).isoformat()
        with self.connect() as db:
            if db.execute("SELECT 1 FROM runs WHERE run_id=?", (run_id,)).fetchone() is None:
                raise ValueError("Saved run not found.")
            db.execute("""INSERT OR IGNORE INTO run_events
                (event_id,run_id,kind,created_at,artifact_path,sha256,summary_json)
                VALUES (?,?,?,?,?,?,?)""",
                (event_id, run_id, kind, created_at, str(path), digest,
                 json.dumps(summary, sort_keys=True, separators=(",", ":"))))
        return event_id

    def get_event(self, run_id: str, event_id: str):
        with self.connect() as db:
            row = db.execute("SELECT * FROM run_events WHERE run_id=? AND event_id=?",
                             (run_id, event_id)).fetchone()
        if row is None:
            return None
        value = dict(row)
        value["summary"] = json.loads(value.pop("summary_json"))
        return value

    def get_goal(self, goal_id):
        if not isinstance(goal_id, str):
            raise ValueError("Invalid goal ID.")
        with self.connect() as db:
            row = db.execute("SELECT * FROM goals WHERE goal_id=?", (goal_id,)).fetchone()
        if row is None:
            raise ValueError("Saved goal not found.")
        return dict(row)

    def get_run(self, run_id):
        with self.connect() as db:
            row = db.execute("SELECT * FROM runs WHERE run_id=?", (run_id,)).fetchone()
        return dict(row) if row else None

    def history(self):
        with self.connect() as db:
            goals = [dict(row) for row in db.execute("SELECT * FROM goals ORDER BY created_at DESC, goal_id")]
            for goal in goals:
                runs = [dict(row) for row in db.execute(
                    "SELECT * FROM runs WHERE goal_id=? ORDER BY created_at DESC,run_id", (goal["goal_id"],))]
                for run in runs:
                    events = [dict(row) for row in db.execute(
                        "SELECT * FROM run_events WHERE run_id=? ORDER BY created_at,event_id", (run["run_id"],))]
                    for event in events:
                        event["summary"] = json.loads(event.pop("summary_json"))
                        event.pop("artifact_path", None)
                        event.pop("sha256", None)
                    run["events"] = events
                goal["runs"] = runs
        return goals
