# Persistent RRM Cosmos worker image. Build from the AirStack repository root:
# docker buildx build --platform linux/amd64 \
#   -f osmo/cosmos-worker/Dockerfile \
#   -t airlab-docker.andrew.cmu.edu/airstack/airstack-rrm-cosmos-worker:latest --push .
#
# The licensed Cosmos snapshot is intentionally NOT downloaded into this image. The
# OSMO workflow retrieves the approved, pinned snapshot into task-local storage using
# a user-owned generic credential at boot time.
FROM nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/opt/rrm:/opt/rrm/scripts

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Match the PSC inference environment that successfully ran Cosmos Reason2. Torch is
# installed with CUDA 12.8 support; the OSMO pool must still provide a compatible host
# NVIDIA driver and one allocated GPU.
RUN python3 -m pip install --break-system-packages --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cu128 \
    torch==2.9.0 torchvision==0.24.0 \
    && python3 -m pip install --break-system-packages --no-cache-dir \
    accelerate==1.11.0 pillow==11.3.0 pydantic==2.13.5 transformers==4.57.3

COPY RRM-transfer/rrm-9d26a58 /opt/rrm

ENTRYPOINT ["python3", "/opt/rrm/scripts/rrm_cosmos_worker.py"]
