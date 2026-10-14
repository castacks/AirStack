# MS project: RGB–thermal geometric uncertainty

Study whether RGB and thermal uncertainty estimates are comparable and predict
which modality supplies more reliable geometric measurements. The first stage
is baseline reproduction and uncertainty/error characterization; adaptive
fusion follows only if the measurements support it.

Start with [MS_RESEARCH_HANDOFF.md](MS_RESEARCH_HANDOFF.md) for the approved
research scope, decisions, resource pointers, and verified progress.

## Repository layout

- `MAC-VO-main/`: extracted MAC-VO source, recovered FlowFormer/DPVO dependencies,
  and environment/verification scripts. See its upstream README for the pipeline.
- `macslam/`: downloaded checkpoint bundle, excluded from Git.
- `MAC-VO-main/Scripts/Setup/README.md`: environment and Box staging instructions.
- The research proposal PDF is retained at the project root.

## Verified starting point

`MACT_Offroad.pth` strictly loads all 428 checkpoint entries into FlowFormerCov.
The MACT frontend runs on the RTX PRO 5000 Blackwell GPU using Torch 2.7.1 and
CUDA 12.8. Synthetic inference at 128x128 and 480x640 produces finite flow and
positive covariance. Full thermal VO and dataset accuracy remain unverified.

The older `macvo:box` image cannot execute on Blackwell. Use `macvo:blackwell`:

```bash
cd /root/AirStack/ore-ms/MAC-VO-main
docker build --progress=plain --network=host -t macvo:blackwell \
  -f Docker/Dockerfile.blackwell .
docker run --rm --gpus all \
  -v /root/AirStack/ore-ms:/workspace -w /workspace/MAC-VO-main \
  macvo:blackwell python Scripts/Setup/verify_mact_checkpoint.py \
  --forward --height 480 --width 640
```

Replace the host mount path if the project is restored elsewhere. Avoid
installing the legacy `requirements.txt` into this image: its xFormers pin
would downgrade the framework. The recovered frontend uses native attention.

## Checkpoints and data

The shared CMU Box bundle is `macslam.zip`. Extract it at the project root to
produce `macslam/MACT_Offroad.pth`, `MACVO_FrontendCov.pth`, and
`MACVO_posenet.pkl`. Archive SHA-256:

```text
77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c
```

AirLab processed data: `/offroad/datasets/tartandrive3`; raw bags:
`/offroad/rosbags`. Prioritize `20251027` or `20251106` for RGB-stereo and
thermal-stereo. These paths are not mounted in the current workspace.

Next obtain about 10 synchronized stereo frames per modality, timestamps,
intrinsics, extrinsics/baseline, and rectification metadata. Preserve the raw
thermal files. Thermal normalization and the checkpoint's training split are
still unknown; three-channel input compatibility alone does not establish the
original preprocessing recipe.

## Storage and shutdown

OSMO workspace files and local Docker images are temporary. Keep model/data
archives and outputs in persistent storage. Box remote `ore_ms` is configured;
the user has saved its config in the OSMO generic credential `ore-ms-box`.
Future workflows must map that credential and restore the private rclone
config; see the setup guide. Tokens and checkpoint/data files are excluded
from Git. Base64 encoding is not encryption.

Commit code and notes before ending a session, then push to an authorized
repository or download a Git bundle to persistent storage. A local commit
does not survive deletion of the workspace.

This project is tracked inside AirStack on the `ore_ms` branch. From the
AirStack root, recover its pinned dependencies with:

```bash
git submodule update --init --recursive -- \
  ore-ms/MAC-VO-main/Module/Network/FlowFormer \
  ore-ms/MAC-VO-main/Baseline/DPVO
```

To restore a downloaded `ore-ms.bundle`:

```bash
git clone ore-ms.bundle ore-ms
cd ore-ms
git submodule update --init --recursive
```

Then restore the checkpoint bundle and rebuild the GPU image. DPVO source is
present, but its compiled extensions and execution have not been verified.
