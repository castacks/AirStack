# Working paper references

Updated 12 November 2026. This is the living entry point; dated reviews remain historical evidence.

Use `papers.json` for reading levels, source/code pins, local status and decision-note links. `papers.bib` is a minimal citation export; missing authors/venues are intentionally not invented. `citation-index.json` retains historical primary-source links, including papers awaiting metadata review. Citation presence is not full-paper reading.

| Key | Paper | Current use / status |
|---|---|---|
| `pnec` | [The Probabilistic Normal Epipolar Constraint for Frame-To-Frame Rotation Optimization under Uncertain Feature Positions](https://arxiv.org/html/2204.02256v1) | Equation-level local prototype only; fixed covariance/rotation routes closed without promotion. |
| `dnls` | [Learning Correspondence Uncertainty via Differentiable Nonlinear Least Squares](https://arxiv.org/html/2305.09527v2) | No official optimizer or learning-system reproduction; conditional flow covariance is not symmetric endpoint covariance. |
| `self-tio` | [Self-TIO: Thermal-Inertial Odometry via Self-Supervised 16-Bit Feature Extractor and Tracker](https://ieeexplore.ieee.org/document/10803066/) | Authors do not release code/networks per repository; local prototype is not Self-TIO. |
| `tp-tio` | [TP-TIO: A Robust Thermal-Inertial Odometry with Deep ThermalPoint](https://arxiv.org/abs/2012.03455) | Inspired translation-only patch prototype; exact local raw-refinement configuration failed; this does not reject TP-TIO. |
| `thermal-sfm` | [Self-Supervised Depth and Ego-Motion Estimation for Monocular Thermal Video Using Multi-Spectral Consistency Loss](https://arxiv.org/abs/2103.00760) | Reproduction pending author checkpoint and processed ViViD data; not executed. |
| `d3vo` | [D3VO: Deep Depth, Deep Pose and Deep Uncertainty for Monocular Visual Odometry](https://arxiv.org/abs/2003.01060) | See historical notes; no new reproduction asserted |
| `droid` | [DROID-SLAM: Deep Visual SLAM for Monocular, Stereo, and RGB-D Cameras](https://arxiv.org/abs/2108.10869) | See historical notes; no new reproduction asserted |
| `maci2` | [MAC-I²: Learned Metrics-Aware Covariance for Robust Visual-Inertial Fusion in Initialization and Calibration](https://mac-i2.github.io/) | See historical notes; no new reproduction asserted |
| `cmrw` | [Self-Supervised Spatial Correspondence Across Modalities](https://arxiv.org/abs/2506.03148) | See historical notes; no new reproduction asserted |
| `tartanmatch` | [TartanMatch: Towards Universal Dense Correspondence Across Modalities](https://arxiv.org/abs/2610.14513) | Pretrained matcher executed; fixed replacement/native and longer rotation configurations failed their gates. |
| `macthermal` | [3D Thermal Perception for Autonomous Navigation in Visually Degraded, Unstructured Environments](https://publications.ri.cmu.edu/3d-thermal-perception-for-autonomous-navigation-in-visually-degraded-unstructured-environments) | See historical notes; no new reproduction asserted |
| `kltnet` | [KLTNet: Learning Sparse Feature Tracking for Robust and Accurate Monocular Visual-Inertial Odometry](https://arxiv.org/abs/2608.24544) | Mechanism inspiration only; no learned triplet tracker reproduction. |
| `macvo` | [MAC-VO: Metrics-aware Covariance for Learning-based Stereo Visual Odometry](https://arxiv.org/html/2409.09479v2) | See historical notes; no new reproduction asserted |
| `imagebind` | [ImageBind: One Embedding Space To Bind Them All](https://openaccess.thecvf.com/content/CVPR2023/html/Girdhar_ImageBind_One_Embedding_Space_To_Bind_Them_All_CVPR_2023_paper.html) | See historical notes; no new reproduction asserted |
| `cmc` | [Contrastive Multiview Coding](https://www.ecva.net/papers/eccv_2020/papers_ECCV/html/1453_ECCV_2020_paper.php) | See historical notes; no new reproduction asserted |
| `dsn` | [Domain Separation Networks](https://proceedings.neurips.cc/paper/2016/hash/45fbc6d3e05ebd93369ce542e8f2322d-Abstract.html) | See historical notes; no new reproduction asserted |

For every new paper used: record its stable ID/URL/version, reading depth and date, code commit/checkpoint hash when used, assumptions and limitations, and links to the proposal/experiment it informed. Update the local result/status after execution; distinguish author claims, our hypothesis, our prototype and an actual reproduction. Add a new dated reading event rather than erasing prior scope. Do not archive full copyrighted paper text in Git or Box.

Current baseline feasibility: [ThermalSfMLearner-MS](../thermal-sfm-reproduction-2026-11-12.md).
