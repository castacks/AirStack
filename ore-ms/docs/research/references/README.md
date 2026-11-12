# Working paper references

Latest source use: [bounded real-input contract](../shared-motion-input-contract-findings-2026-11-12.md). Official MS² data-format/setup pages rechecked; no new full-paper reading, model reproduction or physical synchronization claim.


Latest numerical finding: [shared-motion stationarity audit](../shared-motion-stationarity-findings-2026-11-12.md) resolves the saved gradient discrepancy without prototype/tolerance changes. DNLS remains inspiration only; no new paper or official-system reproduction.


Latest implementation/result: [shared-motion feasibility](../shared-motion-feasibility-findings-2026-11-12.md). DNLS-inspired synthetic prototype only; independent gradient audit fails one check. No official-system reproduction, training or calibration claim.


Latest decision: [multi-sensor research reset](../multisensor-research-reset-2026-11-12.md). MAC-VO, MAC-I² and DNLS reading/source events updated; no new paper added. Shared motion utility and covariance calibration remain separate claims.


Updated 12 November 2026. Living entry point; dated reviews remain historical.

Reading depth, code/checkpoint pins and linked local decisions are in `papers.json`. `papers.bib` is a minimal citation export with no invented metadata. `citation-index.json` retains primary-source links pending review; citation presence does not mean full-paper reading.

| Key | Paper | Current use / status |
|---|---|---|
| `pnec` | [The Probabilistic Normal Epipolar Constraint for Frame-To-Frame Rotation Optimization under Uncertain Feature Positions](https://arxiv.org/html/2204.02256v1) | Equation-level local prototype only; fixed covariance/rotation routes closed without promotion. |
| `dnls` | [Learning Correspondence Uncertainty via Differentiable Nonlinear Least Squares](https://arxiv.org/html/2305.09527v2) | No official optimizer or learning-system reproduction; conditional flow covariance is not symmetric endpoint covariance. |
| `self-tio` | [Self-TIO: Thermal-Inertial Odometry via Self-Supervised 16-Bit Feature Extractor and Tracker](https://ieeexplore.ieee.org/document/10803066/) | Authors do not release code/networks per repository; local prototype is not Self-TIO. |
| `tp-tio` | [TP-TIO: A Robust Thermal-Inertial Odometry with Deep ThermalPoint](https://arxiv.org/abs/2012.03455) | Inspired translation-only patch prototype; exact local raw-refinement configuration failed; this does not reject TP-TIO. |
| `thermal-sfm` | [Self-Supervised Depth and Ego-Motion Estimation for Monocular Thermal Video Using Multi-Spectral Consistency Loss](https://arxiv.org/abs/2103.00760) | ViViD source-protocol reproduction has close headline agreement; fixed original-input MS2 transfer fails all three gates with severe known clipping. No model promotion. |
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
| `vivid-2019` | [ViViD : Vision for Visibility Dataset](https://arxiv.org/html/2103.00760v3) | Data source used for author-protocol reproduction. Do not identify it as the later ViViD++ driving cohort. |
| `vivid-plusplus` | [ViViD++: Vision for Visibility Dataset](https://arxiv.org/abs/2204.06183) | Dataset-version distinction only. No ViViD++ payload used; author project states it extends the2019 predecessor. |
| `resnet` | [Deep Residual Learning for Image Recognition](https://arxiv.org/abs/1512.03385) | Architecture provenance: ResNet18 in author thermal depth/pose checkpoints; no original ResNet benchmark reproduction. |
| `monodepth2` | [Digging Into Self-Supervised Monocular Depth Estimation](https://arxiv.org/abs/1806.01260v4) | Code provenance: author encoder/pose modules credit Monodepth2; no Monodepth2 baseline run. |
| `ms2-2023` | [Deep Depth Estimation From Thermal Image](https://openaccess.thecvf.com/content/CVPR2023/papers/Shin_Deep_Depth_Estimation_From_Thermal_Image_CVPR_2023_paper.pdf) | Source default-code/depth convention used in fixed MS2 transfer gate; no full dataset/model benchmark reproduction. |
| `ms2-extended` | [Deep Depth Estimation from Thermal Image: Dataset, Benchmark, and Challenges](https://arxiv.org/abs/2503.22060v1) | Corroborating nominal camera formula and warning that parameters are camera-dependent. New benchmark preprocessing/weights not adopted. Repository citation uses Analysis in its title; primary preprint uses Challenges. |

For every paper used, record stable ID/URL/version, reading date/depth, author claims versus local interpretation, code/checkpoint pins, limitations and linked decisions/results. Preserve prior reading events. Full copyrighted paper text is excluded from Git and Box.

Latest: [fixed MS² transfer findings](../ms2-depth-transfer-findings-2026-11-12.md); [ViViD reproduction](../thermal-sfm-reproduction-findings-2026-11-12.md).
