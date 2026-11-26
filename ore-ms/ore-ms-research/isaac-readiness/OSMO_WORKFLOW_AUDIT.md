# OSMO workflow audit

The checked-in workflow requests 16 CPU, one GPU, 64 GiB memory and 500 GiB storage. It enables autolaunch and livestream, defaults the source branch to `main`, and uses the floating workspace image tag `latest`. Submit with an explicit research branch override. The entrypoint defaults `OSMO_AIRSTACK_UP` to true and deliberately keeps SSH alive after a launch failure. The user-supplied workflow specified the research branch and requested 10 CPU and 48 GiB, so its resource requests cannot be attributed to the checked-in YAML.

On the accessible pod, SSH is listening on TCP 22, Docker 29.5.0 responds using overlay2, NVIDIA runtime is registered, and the Isaac container sees the allocated GPU. Prior entrypoint logs directly establish an attempted full startup, so the standalone-only path was not used for that attempt. Scheduler events, granted CPU/RAM/storage, outer image digest, and a second external SSH authentication were not available from this session.

Fresh submission was blocked by the absence of the OSMO CLI and an authenticated submission context in this workspace. No replacement pool or workflow was launched. See [reproduction](REPRODUCIBILITY.md).
