# Existing capability audit

Inspection covered the active AirStack checkout and host Python. The evidence log records revision, tree, system, GPU, tool probes and locally cached images. The checkout includes ROS 2 Jazzy integration, Isaac Sim/Pegasus assets and existing MAC-VO work. Existing code and data serve as candidates for later integration. Host Python 3.12.3 suffices for M01 with its standard library. No packages were installed or images pulled.

| Candidate | Host detection | Upstream license and primary source | Maturity, compute and reuse boundary |
|---|---|---|---|
| NumPy | No host distribution | [BSD-3-Clause](https://numpy.org/doc/stable/license.html) | Mature CPU arrays and Monte Carlo vectorization. No need for 442 observations. |
| SciPy | No host distribution | [BSD-3-Clause](https://github.com/scipy/scipy) | Mature CPU integration and estimation utilities. Candidate alternate quadrature. |
| SymPy | No host distribution | [BSD-3-Clause](https://github.com/sympy/sympy/blob/master/LICENSE) | Mature CPU symbolic algebra. Candidate Jacobian derivation with numerical cross-check. |
| OpenCV | Python wheels and cv2 unobserved on host | [Apache-2.0 for current upstream](https://github.com/opencv/opencv/blob/4.x/LICENSE) | Mature CPU vision with optional accelerators. Candidate registration and PSF tooling. Wheel dependencies need separate review. |
| PyTorch | No host distribution | [BSD-style](https://github.com/pytorch/pytorch) | Mature CPU/GPU autodiff. GPU and memory sized to data and future models. |
| Isaac Sim | Host distribution unobserved. AirStack assets present | [Apache-2.0 source and additional terms](https://docs.isaacsim.omniverse.nvidia.com/latest/common/licenses.html) | RTX-class GPU for typical rendering workflows. Scene and poses reusable once renderer compatibility is validated. RGB rendering is insufficient evidence for calibrated LWIR. |
| Gazebo | No gz/gazebo on PATH | [Apache-2.0 for Gazebo Sim](https://github.com/gazebosim/gz-sim) | Established simulation stack, rendering GPU optional by workload. Candidate ROS sensor plumbing. |
| MuJoCo | No host distribution | [Apache-2.0](https://github.com/google-deepmind/mujoco) | Mature CPU dynamics, optional offscreen rendering. Candidate contact and robot pose simulation. |
| Mitsuba 3 | No host distribution | [BSD-3-Clause](https://github.com/mitsuba-renderer/mitsuba3/blob/master/LICENSE) | Research-grade CPU/GPU spectral renderer. Qualify wavelength ranges, emitters, media and sensor response for LWIR. |
| libRadtran | No uvspec on PATH | [GPL, see project manual](https://www.libradtran.org/doc/libRadtran.pdf) | Established CPU atmospheric transfer. Coordinate conventions, input profiles and short-path applicability need verification. |
| PyRadi | No host distribution | [MIT repository license](https://github.com/NelisW/pyradi/blob/master/LICENSE), [BSD-3-Clause in ryplanck.py](https://raw.githubusercontent.com/NelisW/pyradi/master/pyradi/ryplanck.py) | Research CPU radiometry utilities. The Planck module produces exitance. Check the radiance conversion, license headers and version before reuse. |
| NVIDIA Warp | No host distribution | [Apache-2.0](https://github.com/NVIDIA/warp) | CPU/CUDA kernels for batched rays. Profiling and numerical parity should precede adoption. |
| ROS 2 | No ros2 on host PATH. Jazzy declared by checkout | [Apache-2.0 core package example](https://github.com/ros2/rclcpp) | Mature CPU middleware with package-specific terms. Candidate synchronized replay and calibrated transform transport. |

Cached Docker tags do not establish installed component versions inside images. No in-container packages or external private baseline inventories were inferred. GPU is available on the host but M01 runs on CPU.
