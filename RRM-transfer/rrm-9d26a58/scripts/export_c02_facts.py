import json

probe_file = "/root/AirStack/.rrm-artifacts/hand-tabletop-probe-20260924-e/probe.json"
with open(probe_file, 'r') as f:
    probe = json.load(f)

# Use index 0 for the export (could be any since it's verified same)
observed_s = probe["reset_samples"][0]["camera_capture"]["observed_monotonic_s"]
episode_id = probe["reset_samples"][0]["camera_capture"]["episode_id"]

evidence = []
for entity_id, prim_path in probe["scene_entities"].items():
    evidence.append({
        "key": {"subject": entity_id, "predicate": "exists"},
        "truth": "TRUE",
        "provenance": "SIMULATOR",
        "source_ref": f"probe:{episode_id}",
        "observed_monotonic_s": observed_s,
        "received_monotonic_s": observed_s,
        "max_age_s": 10
    })
    evidence.append({
        "key": {"subject": entity_id, "predicate": "kind", "obj": entity_id.replace("_", " ")},
        "truth": "TRUE",
        "provenance": "SIMULATOR",
        "source_ref": f"probe:{episode_id}",
        "observed_monotonic_s": observed_s,
        "received_monotonic_s": observed_s,
        "max_age_s": 10
    })
    evidence.append({
        "key": {"subject": entity_id, "predicate": "localized"},
        "truth": "TRUE",
        "provenance": "SIMULATOR",
        "source_ref": f"probe:{episode_id}",
        "observed_monotonic_s": observed_s,
        "received_monotonic_s": observed_s,
        "max_age_s": 10
    })

out_data = {
    "snapshot": {
        "snapshot_id": "hand-tabletop-001",
        "revision": "hand-tabletop-v1",
        "task_id": "hand-task-001",
        "episode_id": episode_id,
        "evidence": evidence,
        "complete_domains": []
    }
}

out_file = "/root/AirStack/.rrm-artifacts/hand-tabletop-probe-20260924-e/c02_facts.json"
with open(out_file, 'w') as f:
    json.dump(out_data, f, indent=2)

print(f"Exported to {out_file}")
