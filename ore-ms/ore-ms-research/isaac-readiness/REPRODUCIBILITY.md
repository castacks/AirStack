# Reproducibility

The observed checkout was `a91d61e6fa7b7489b6ada263151da4ede6400428`. Verify the remote branch still points there before submission and verify HEAD again after cloning. [Source hashes](evidence/source-sha256.txt) were computed from the observed checkout. [Image provenance](IMAGE_PROVENANCE.md) distinguishes registry digest from local configuration ID. The runtime overlay on Isaac means the image alone will not replay the repaired pod.

Use the checked-in workflow from an authenticated workstation after publishing the compatible image. Capture the submitted YAML, outer and inner digests, resource grant, checkout and entrypoint status. The checked-in workflow floats the workspace tag and clones a branch, so this invocation alone only verifies a branch launch.

```bash
osmo workflow submit osmo/workflows/airstack-dev.yaml --pool <gpu-pool> \
  --set-env "SSH_PUB_KEY=$(cat ~/.ssh/id_ed25519.pub)" \
  --set-env AIRSTACK_BRANCH=ore-ms-orb-development \
  --set-env OSMO_AIRSTACK_UP=true
osmo workflow port-forward <workflow-id> workspace --port 2200:22 --connect-timeout 86400
```

On the pod, inspect the startup result and container states. Verify NumPy in the untouched image and live container, confirm the robot's domain with the configured environment, and run the probes from the repository root.

```bash
evidence=ore-ms/ore-ms-research/isaac-readiness/evidence
docker ps
docker exec isaac-sim-livestream /isaac-sim/python.sh -c 'import numpy; print(numpy.__version__)'
for domain in 0 1; do
  docker exec -i airstack-robot-desktop-1 bash -c \
    "source /opt/ros/jazzy/setup.bash; ROS_DOMAIN_ID=$domain python3 -" \
    < "$evidence/observe_ros.py"
done
timeout 25 python3 "$evidence/observe_foxglove.py"
```

For the external test, run the Foxglove helper locally for the explicitly selected workflow ID and use `ws://localhost:8766`. Record the same topic set and duration with and without the full layout, using controlled paired publisher/subscriber timestamps to measure latency. Preserve the Foxglove version, layout hash, topic choices and tunnel disconnect logs. Keep raw images and credential-bearing outputs outside Git.

Exact replay applies to the frozen numerical reference JSON. It was checked against historical M11 target counts. Local bridge re-observation shows tolerance-worthy variability, but no tolerance-bound runtime replay was established. The ambient stage asset digest, physics time step, camera exposure configuration and seed were not recovered as a complete runtime manifest. Rendering and timing remain unreproduced. The fixture protocol lists the pins needed for a comparable capture.
