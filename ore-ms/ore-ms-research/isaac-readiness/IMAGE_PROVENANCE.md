# Image provenance

| Running service | Local image configuration ID | Registry manifest digest |
|---|---|---|
| Isaac livestream | `sha256:306993a1d84df39572c0f7c049ba9ce90934d091534c43ae4b88e57faa17c808` | Unavailable |
| Robot desktop | `sha256:191cbf60c589dac0167518e4d3c0fa3b07807eac4d3d4df0b5c5d25bc003a9e1` | Unavailable |
| GCS | `sha256:9e44be1c0c38e297ba5b9fc463f472ca5c224091969f50e417825a412e61aa7e` | `sha256:d25c4ae77de5fd6da43c334220e833dacedd61bceafce389e882b735ae41af9a` |

The local Isaac image was built with Isaac Sim 5.1.0, Jazzy, and the PX4 v1.16.1 Dockerfile default. The independent image-only import printed NumPy 2.4.6. The live Isaac import printed Python 3.11.13 and NumPy 1.26.4. PX4 SITL and the separate Python 3.12.3 venv are present. The GCS package is `ros-jazzy-foxglove-bridge 3.4.1-1noble.20260615.172953`. Local configuration IDs identify the inspected images. Only the GCS entry has a resolved registry digest. The outer workspace and NVIDIA base registry digests were unavailable.
