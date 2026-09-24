import json
import sys
from pathlib import Path

SOURCE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(SOURCE_ROOT))
sys.path.insert(0, str(SOURCE_ROOT / ".rrm-deps"))

from rrm.visual_world_builder import score_visual_snapshot
from rrm.state_contracts import StateSnapshot, FactEvidence, FactKey, FactProvenance, Truth

# Load the teacher snapshot we generated earlier
teacher_file = "/root/AirStack/.rrm-artifacts/hand-tabletop-probe-20260924-new/c02_facts.json"
with open(teacher_file, 'r') as f:
    teacher_data = json.load(f)["snapshot"]
teacher_snapshot = StateSnapshot.model_validate(teacher_data)

observed_s = teacher_data["evidence"][0]["observed_monotonic_s"]

candidate_evidence = [
    # Saw red block
    FactEvidence(key=FactKey(subject="red_block", predicate="exists"), truth=Truth.TRUE, provenance=FactProvenance.INFERRED, source_ref="model", observed_monotonic_s=observed_s, received_monotonic_s=observed_s, max_age_s=10),
    FactEvidence(key=FactKey(subject="red_block", predicate="kind", obj="block"), truth=Truth.TRUE, provenance=FactProvenance.INFERRED, source_ref="model", observed_monotonic_s=observed_s, received_monotonic_s=observed_s, max_age_s=10),
    FactEvidence(key=FactKey(subject="red_block", predicate="localized"), truth=Truth.TRUE, provenance=FactProvenance.INFERRED, source_ref="model", observed_monotonic_s=observed_s, received_monotonic_s=observed_s, max_age_s=10),
    # Saw tray
    FactEvidence(key=FactKey(subject="tray_1", predicate="exists"), truth=Truth.TRUE, provenance=FactProvenance.INFERRED, source_ref="model", observed_monotonic_s=observed_s, received_monotonic_s=observed_s, max_age_s=10),
    FactEvidence(key=FactKey(subject="tray_1", predicate="kind", obj="tray"), truth=Truth.TRUE, provenance=FactProvenance.INFERRED, source_ref="model", observed_monotonic_s=observed_s, received_monotonic_s=observed_s, max_age_s=10),
    # Missed blue block and table (maybe partial occlusion or bad model)
]

candidate_snapshot = StateSnapshot(
    snapshot_id="candidate-001",
    revision="candidate-v1",
    task_id="hand-task-001",
    episode_id=teacher_data["episode_id"],
    evidence=candidate_evidence,
    complete_domains=[]
)

catalog = {
    "red_block": "block",
    "blue_block": "block",
    "tray_1": "tray",
    "table": "table"
}

score = score_visual_snapshot(
    candidate_snapshot,
    teacher_snapshot,
    now_monotonic_s=observed_s,
    entity_catalog=catalog
)

print(score.model_dump_json(indent=2))
