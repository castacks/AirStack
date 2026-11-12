# Working paper references

Updated 12 November 2026. Living entry point; dated reviews remain historical.

Reading depth, code/checkpoint pins and linked experiment decisions are in `papers.json`. `papers.bib` is a minimal citation export with no invented metadata. `citation-index.json` retains historical primary-source links pending review. Citation presence does not mean full-paper reading.

| Key | Paper | Current use / status |
|---|---|---|
| `pnec` | [The Probabilistic Normal Epipolar Constraint for Frame-To-Frame Rotation Optimization under Uncertain Feature Positions](https://arxiv.org/html/2204.02256v1) | Equation-level local prototype only; fixed covariance/rotation routes closed without promotion. |
| `dnls` | [Learning Correspondence Uncertainty via Differentiable Nonlinear Least Squares](https://arxiv.org/html/2305.09527v2) | No official optimizer or learning-system reproduction; conditional flow covariance is not symmetric endpoint covariance. |
| `self-tio` | [Self-TIO: Thermal-Inertial Odometry via Self-Supervised 16-Bit Feature Extractor and Tracker](https://ieeexplore.ieee.org/document/10803066/) | Authors do not release code/networks per repository; local prototype is not Self-TIO. |
| `tp-tio` | [TP-TIO: A Robust Thermal-Inertial Odometry with Deep ThermalPoint](https://arxiv.org/abs/2012.03455) | Inspired translation-only patch prototype; exact local raw-refinement configuration failed; this does not reject TP-TIO. |
| `thermal-sfm` | [Self-Supervised Depth and Ego-Motion Estimation for Monocular Thermal Video Using Multi-Spectral Consistency Loss](https://arxiv.org/abs/2103.00760) | Executed full outdoor source protocol on modern runtime: close depth headline agreement, strict all-column/pose table match fails; no MS2 transfer claim. |
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

For every paper used, record its stable ID/URL/version, reading date/depth, author claims versus local interpretation, code/checkpoint pins, limitations and linked decisions/results. Preserve historical reading events. No full copyrighted paper text goes into Git or Box.

Completed reproduction: [findings and limits](../thermal-sfm-reproduction-findings-2026-11-12.md); [frozen plan](../thermal-sfm-execution-plan-2026-11-12.md). ViViD 2019 and ViViD++ are separate records.
