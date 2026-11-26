# Published image manifest

- Candidate tag: `airlab-docker.andrew.cmu.edu/airstack/airstack:ore-ms-numpy126-r1-e5436fc3_isaac-sim`.
- Deployment reference: `airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697`.
- Registry manifest: Docker schema 2, Linux amd64, verified by registry lookup. Pull by digest returned the same manifest and local configuration ID `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`.
- The candidate uses a new tag. The branch's default simulator, robot and GCS tags were left intact. Select the digest explicitly for a deployment.

```bash
export ISAAC_SIM_IMAGE='airlab-docker.andrew.cmu.edu/airstack/airstack@sha256:ba2402ec4f389fad6552d79e87871131c587d090963d1d894539016904f3e697'
docker pull --platform linux/amd64 "$ISAAC_SIM_IMAGE"
bash tests/system/check_isaac_numpy.sh "$ISAAC_SIM_IMAGE"
COMPOSE_PROFILES=desktop,isaac-sim-livestream docker compose up -d \
  --no-build --no-deps --pull never --force-recreate isaac-sim-livestream
```

The recreation command is for an existing grounded simulation session with the robot and GCS already running. See [registry inspection](evidence/manifest.txt), [platform](evidence/platform.json) and [pulled image ID](evidence/pulled-image.txt).
