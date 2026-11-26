# Mac deployment handoff

## Check the Mac-local file

Within `workflow.groups[].tasks[]`, find the task whose `name` is `workspace`. Confirm the following entries are aligned under that task's existing `environment:` mapping. Keep the outer `image:` and existing rclone `command:`, `args:` and `credentials:` in place.

```yaml
        AUTOLAUNCH: "true"
        ISAAC_SIM_LIVESTREAM: "true"
        NUM_ROBOTS: "1"
        AIRSTACK_BRANCH: "ore-ms-orb-development"
        AIRSTACK_REPO_URL: "https://github.com/castacks/AirStack.git"
        ISAAC_SIM_IMAGE: "airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697"
```

If `OSMO_AIRSTACK_UP` exists and is false, set it to `"true"`; otherwise the default already enables startup. If `COMPOSE_PROFILES` is explicitly set for this task, remove that override or set it to `"desktop,isaac-sim-livestream"`. The branch CLI automatically selects no-build for the digest. For an explicit workflow-level safeguard, optionally add `AIRSTACK_NO_IMAGE_BUILD: "1"` beside `ISAAC_SIM_IMAGE`. The planned resource block is 10 CPU, one GPU, 48 GiB memory and 500 GiB storage.

**STOP before submission** if the exact digest, branch, livestream profile, or no-build path cannot be established, or if the local workflow validator reports an error. If the candidate is unavailable on a fresh pod, stop at the launch error and retain the sanitized logs. Avoid removing the pin or rebuilding as a fallback.

## Validate and submit from your Mac

Use your existing pool name for `$POOL`. The commands below assume the existing public key is `~/.ssh/id_ed25519.pub`; substitute your usual public-key path if different. Check which validation subcommand is supported by your installed CLI. The current OSMO documentation provides a non-submitting `validate` command. On clients without it, use `submit --dry-run` only if local help explicitly lists that option. Do not share the rendered spec. [NVIDIA OSMO CLI reference](https://nvidia.github.io/OSMO/main/user_guide/appendix/cli/cli_workflow.html)

```bash
osmo workflow validate --help
osmo workflow validate osmo/workflows/airstack-dev.yaml --pool "$POOL"
```

If validation passes and the settings above match, run the following when you choose to submit.

```bash
osmo workflow submit osmo/workflows/airstack-dev.yaml \
  --pool "$POOL" \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)"
```

Save the workflow ID printed by submission in `WF_ID`. To recover it, use `osmo workflow list --name airstack-dev` and match the submission time and pool. Confirm it with `osmo workflow query "$WF_ID"`.

## Forward and capture startup evidence

Run the forwards in separate terminals using the same workflow ID.

```bash
osmo workflow port-forward "$WF_ID" workspace --port 2200:22 --connect-timeout 86400
osmo workflow port-forward "$WF_ID" workspace --port 49100:49100 --connect-timeout 86400
osmo workflow port-forward "$WF_ID" workspace --port 49099 --udp --connect-timeout 86400
osmo workflow port-forward "$WF_ID" workspace --port 8766:8766 --connect-timeout 86400
```

Use SSH at `root@localhost` port 2200, Isaac WebRTC at localhost, and the existing `airstack_default` layout with Foxglove WebSocket `ws://localhost:8766`. Use a workflow-specific SSH host-key alias so a previous workflow's host key does not collide.

```bash
ssh -p 2200 -o HostKeyAlias="$WF_ID" root@localhost
cd /root/AirStack
git rev-parse --abbrev-ref HEAD
git rev-parse HEAD
docker ps -a --format '{{.Names}} {{.Status}}'
docker inspect --format '{{.Config.Image}} {{.Image}} {{.State.StartedAt}}' isaac-sim-livestream
docker exec isaac-sim-livestream /isaac-sim/python.sh -c 'import numpy; print(numpy.__version__)'
```

Return the workflow ID, the output above, the sanitized `compose up --no-build` or failure line from the workspace log, and whether Isaac WebRTC and Foxglove displayed data. Include the Kit startup and ROS bridge milestones if available. Send no environment dumps or credential values. Expected NumPy is 1.26.4. If the digest or source branch differs, stop before further runtime testing.
