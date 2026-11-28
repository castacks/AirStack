# Offline validation

Design review only; no construction, deployment or execution authorization.

Run from repository root:

```
python3 -B RRM/engineering/M00-F/test_validate_design.py
python3 -B RRM/engineering/M00-F/validate_design.py RRM/engineering/M00-F/M00F_NONEXECUTABLE_POLICY.json
python3 -B RRM/engineering/M00-F/validate_milestone.py
```

Only Python standard library and read-only Git metadata/file inspection are used. No Docker, Compose, ROS, simulator, network or hardware command is invoked. The validator accepts a deliberately closed non-executable JSON policy schema. It rejects unknown/missing keys, device/host mounts, privilege, host namespaces, extra capabilities, external ports/routes, forwarding, unknown/hardware env and entrypoint or overlay changes. It does not parse arbitrary Compose, inspect image entrypoints, validate effective policy or establish GPU support. Those are future engineering tasks.

One positive fixture represents policy intent with runtime_verified false and GPU policy unapproved.27 negative fixtures cover independent exposure/input classes; additional missing-field and malformed-root subcases ensure fail-closed input handling. The expected positive result is PASS_STATIC_INTENT_ONLY, with runtime NOT YET VERIFIED. These are software validator tests, not startup or isolation trials. No YAML/launch/start automation is supplied.

Milestone validation also checks all protected tracked paths/modes/symlinks/submodules, controlled input inventory, local source hashes, pinned historical checksums, prerequisite/mapping continuity, unpassed runtime gates and exact new-file/checksum scope. The saved result excludes its own digest to avoid recursion and is bound by Git. Postcommit results must reproduce. Existing milestone validators have sealed file horizons and are run before successor writes, rather than altered to accept new files.
