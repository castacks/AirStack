# (b) Runner smoke — 2026-09-11 05:06 EDT, agent_study `93356dd` (pre-commit tree), branch airstack-paper `4556d8233`

Commands (no tokens, no GPU; `--smoke` skips provisioning and dry-runs judges):

    python3 runner/run_trial.py --config config/study_config_v7_external.yaml --arm E2 --model none --trial-index 1 --agent mock:runner/mock_agents/noop_agent.sh --smoke --force
    python3 runner/run_trial.py --config config/study_config_v7_external.yaml --arm E1 --model none --trial-index 1 --agent mock:runner/mock_agents/noop_agent.sh --smoke --force
    python3 runner/run_trial.py --arm A3 --model none --trial-index 9 --agent mock:runner/mock_agents/noop_agent.sh --smoke --force   # v6 regression

| Check | Result |
|---|---|
| py_compile / bash -n of all edited + new scripts | OK |
| v7 `judge:` block vs v6 (`diff`) | identical |
| E2 workspace | `aerostack2/`, `project_gazebo/`, `ENVIRONMENT.md`, `provided/`, `judge`, `.study_host_mode` |
| E2 shim exports | `STUDY_ARM=E2 STUDY_HOST_MODE=1` + v6 judge params |
| E2 provisioned manifest | 1238 files; solution snapshot 1 new file (AGENT_REPORT.md) |
| E1 workspace | `unified_autonomy_stack/`, `ENVIRONMENT.md`, `provided/`, `judge` |
| prompt_sha256 (E1/E2) | `afad954d8087…` = v6 frozen prompt |
| config_sha256 v7 | `bb4f5ca21dd3…` (v6: `3b2402f61245…`) |
| A3 v6 smoke | unchanged config hash `3b2402f6…`; shim gains `STUDY_HOST_MODE=1` only |
| UAS `import_all_repos.sh --exact` on this box | 1:47 wall, 5.1 GB; aborts at ws_sim (`ntnu-arl/helicopter_bridge` is private) → provisioning loop tolerates per-workspace failures |
