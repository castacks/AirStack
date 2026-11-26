# Build provenance

The candidate tag is `airlab-docker.andrew.cmu.edu/airstack/airstack:ore-ms-numpy126-r1-e5436fc3_isaac-sim`. Registry lookup before the build returned tag not found. The build used the existing x86_64 pod and requested Linux amd64. BuildKit resolved the Isaac 5.1.0 base to `sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54`. PX4 uses the existing v1.16.1 build argument. The Dockerfile and copied Fast DDS configuration are hashed in [source provenance](evidence/source-provenance.txt). Upstream apt and pip indexes and the Pegasus branch source can change in a later rebuild. The immutable published artifact is the runtime replay reference.

From the repository root, choose a new unused versioned tag for a new build.

```bash
docker buildx build --platform linux/amd64 --progress=plain --load \
  -t "$candidate" -f simulation/isaac-sim/docker/Dockerfile.isaac-ros \
  simulation/isaac-sim >build.log 2>&1
bash tests/system/check_isaac_numpy.sh "$candidate"
docker push "$candidate"
docker buildx imagetools inspect "$candidate"
```

BuildKit completed all 19 image steps, including the late NumPy check. Local configuration ID was `sha256:b45a696338c10c984487b9b91a215b20c579afabc1f00a216dc43dacfb76bacd`. The [complete build log](evidence/build.log.gz) is gzip-compressed. BuildKit reported the existing undefined `LD_LIBRARY_PATH` warning. pip reported pre-existing NVIDIA package metadata conflicts involving optional dependencies. The NumPy-constrained install selected OpenCV 4.11.0.86 and quantized-mesh-tile 0.6.1, and the final NumPy assertion succeeded.
