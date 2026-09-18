"""Persistent goal/run index. Artifacts stay in files; no execution authority."""
from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import uuid


class TaskStore:
    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS goals (
                    goal_id TEXT PRIMARY KEY,
                    objective TEXT NOT NULL,
                    constraints_revision TEXT NOT NULL,
                    embodiment_id TEXT NOT NULL,
                    created_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS runs (
                    run_id TEXT PRIMARY KEY,
                    goal_id TEXT NOT NULL REFERENCES goals(goal_id),
                    task_id TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    status TEXT NOT NULL CHECK(status IN
                        ('SAVED_NOT_SUBMITTED', 'CANDIDATE_ACCEPTED')),
                    artifact_dir TEXT NOT NULL,
                    execution_state TEXT NOT NULL DEFAULT 'NOT_DISPATCHED'
                        CHECK(execution_state = 'NOT_DISPATCHED')
                );
                CREATE INDEX IF NOT EXISTS runs_by_goal ON runs(goal_id, created_at);
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

    def record(self, *, goal_id, run_id, task, created_at, status, artifact_dir):
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
        # Legacy folders lacked goal_id; use their stable request ID without rewriting them.
        self.record(goal_id=manifest.get("goal_id", manifest["request_id"]),
                    run_id=manifest["request_id"], task=payload["task"],
                    created_at=manifest["created_at"], status=manifest["status"], artifact_dir=directory)

    def record_reference(self, bundle: Path):
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
                goal["runs"] = [dict(row) for row in db.execute(
                    "SELECT * FROM runs WHERE goal_id=? ORDER BY created_at DESC,run_id", (goal["goal_id"],))]
        return goals
