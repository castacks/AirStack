# Next on resume: separate rotation constraint from extra optimization

Recorded15November2026 UTC on `ore-ms-orb-development`, starting commit
`53afa19d1`. **Planned only; this comparison has not been run.** User requested
this checkpoint before ending the session.

The qualified longer-night continuation gives positionRMSE33.482921m for native
RGB,31.131610m for RGB-only translation refinement and31.276695m with thermal.
RGB-only refinement improves about7.02%; thermal then regresses0.466%against
that stronger control. We must determine whether the RGB gain comes from the
fixed-rotation restriction or simply an additional optimization pass before
claiming an architectural or multisensor improvement.

The next finite control comparison is:

1. Native RGB, unchanged first pass.
2. The same RGB observations with an additional full6DoF pose solve.
3. The same RGB observations with an additional3DoF translation solve while
   retaining the first pass's RGB rotation.

Start with identical accepted factors, source coordinates/covariances, first-pass
solution and second-stage initialization for the paired refinements. Isolate
this solver mechanism on fixed per-pair inputs before adding different feedback
histories. Keep native robust loss, precision model, solver and stopping settings;
freeze the actual protocol and source pins before predictions. Do not silently
change the published3D covariance objective into our captured disparity path:
this compares the verified local native disparity implementation.

Use an equal declared maximum second-pass iteration budget and record actual
iterations and CPU solver time. Equal iteration counts do not establish equal
computation, and cached solver timing is not end-to-end VO timing. Report
translation/rotation mean,RMSE,p95 and output availability on identical support.
Any trajectory composition from fixed-pair inputs must retain its replay scope.
No new weights, training, threshold search or selected successful windows.

Use the qualified night continuation source599–1199 as the first fixed population;
its600increments all passed reproduction. The day continuation remains diagnostic:
pair1034differs2.944mmfrom archived native against a frozen1mmbound. Do not waive
that failure to obtain a broader claim. A subsequent different recording check
is warranted only after the mechanism comparison adds useful evidence.

If extra full-pose optimization matches the refinement gain, credit the extra
optimization budget. If the translation constraint provides added accuracy at
comparable measured cost, retain it as a scoped RGB backend result and assess
existing decoupled-optimization prior art before claiming novelty. Keep thermal
as a separate incremental ablation against the strongest RGB control. Do not
train another fusion gate now.

Preserve the positive short-transfer results: daytime8.507%/6.401%lower position
RMSE versus native/RGB-refinement, nighttime1.031%/2.857%, with exact RGB rotation.
Both100-frame prefixes were historically exposed and roughly10seconds long.
These gains and the longer negative incremental result belong together.

Authoritative state: [continuation findings](fixed-rotation-continuation-findings-2026-11-15.md),
[short recording transfer](fixed-rotation-recording-transfer-findings-2026-11-15.md),
and [component-constrained baseline](component-constrained-native-findings-2026-11-15.md).
Generic rotation/translation separation is established prior art (MAC-VO/SFVO/
Buczko/MSO reading histories retained). This checkpoint adds no new source reading,
experiment, protected-data access, commit publication or model inference.

Results are ignored in Git and preserved on Box. Latest complete evidence packet:
`ore_ms:ore-ms/backups/2026-11-15-fixed-rotation-continuation`, containing
`continuation-artifacts.tar.gz` and its archive/member SHA manifest. Earlier
transfer evidence is in `2026-11-15-fixed-rotation-recording-transfer`.
Runtime used `PYTHONPATH=/tmp/thermal-intake/packages`, Torch2.7.1+cpu/PyPose0.6.8,
`OPENBLAS_NUM_THREADS=1`; no fresh neural runtime is needed for this cached control.
