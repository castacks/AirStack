# Published covariance claims and local experiment boundaries

12 November2026, America/New_York. Targeted source reading to answer the user's
question about MACThermal and MAC-I². No new experiment, training or method choice.

## MACThermal

Yifei Liu, CMU-RI-TR-25-103, thesis PDF sections3.3–3.5, printed pages26–28
(PDF pages39–41) directly inspected as extracted text. Section2.6's autonomy
comparison setup also inspected. This is not a full thesis/figure reanalysis.

Section3.3 adapts FlowFormer matching and the covariance head on combined
TartanAirV2 forest data and geometrically augmented TartanDrive2.5T. Section3.4
uses uncertainty both for feature selection and covariance-weighted two-frame
3D pose factors. Table3.1 compares original MACVO and the full MACThermal
adaptation on held-out TartanDrive2.5T thermal sequences:1px flow accuracy
0.4022→0.8600,3px0.6343→0.9677,reported within1sigma coverage0.6563→0.9390.
These are reported flow/coverage results. Coverage alone is not an isolated pose
accuracy or calibrated-probability guarantee. The table changes matching and
uncertainty together; it does not isolate covariance weighting alone at night.
Section3.5 reports qualitative trajectory/map consistency against SuperOdometry.
The separate autonomy comparison explicitly uses common SuperOdometry odometry
to isolate sensor/mapping effects, so those traversal results cannot establish
thermal covariance's isolated VO accuracy benefit.

Source: https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf
Institutional record: https://publications.ri.cmu.edu/3d-thermal-perception-for-autonomous-navigation-in-visually-degraded-unstructured-environments
No copyrighted full thesis is added to the repository/recovery artifacts.
The private released checkpoint's exact training membership/recipe remains unverified.

## MAC-I²

arXiv2609.07116v1, sectionsIV-A,VI and TableIII rechecked directly. It derives
visual pose covariance from MAC-VO feature uncertainty and combines visual and
inertial residuals with learned covariances in VI initialization/calibration.
Its explicit TableIII ablation reports mean body-rotation initialization error
0.362→0.215deg and body-translation0.024→0.021m for identity visual+IMU covariance
versus learned visual+IMU covariance on the reported EuRoC evaluation. These are
joint covariance changes, not an isolated thermal or visual-only gain.
TableIII includes single-covariance ablations too; benefits depend on the paired
fusion model. The paper evaluates vision+IMU, not RGB+thermal nighttime fusion.

Source: https://arxiv.org/html/2609.07116

## Relationship to local results

The papers support uncertainty-aware estimation within their evaluated systems.
Our negative RGB+thermal fusion extensions do not refute those results. Our local
conditional shared-geometry and small learned-weight recipes are not
faithful reproductions of the papers or controlled thermal-only covariance
ablations. Published benefit, reproduction of that benefit and benefit of adding
thermal to an RGB estimator are separate empirical questions.
No new experiment is implicitly selected by this clarification.
