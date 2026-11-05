# EXP143: complete-record ORB support and matched-error comparison

2026-11-05 America/New_York. Native freeze 37932ff0e, parent EXP142/bf0e1493e;
matched-support supplement 102cc0346 and rigid-inverse audit refinement 97bddeb1f
precede all ORB reference scores.
[Full findings](../docs/research/orb-full-record-findings-2026-11-05.md).

Six sequential one-attempt runs process 55,618/55,618 source calls with native
calibration and terminal success. Tracking-supported outputs54,432; all six
trajectories break, so none passes the continuous-trajectory engineering flag.
Thermal pose coverage day/night/rain99.466%/99.141%/99.739% versus
RGB97.831%/93.150%/98.172%; map-continuity segments2/4/2 versus5/23/8.
Thermal translation RPE is higher on all three recordings. Thermal rotation RPE
is lower in day/night, higher in rain. Preserve all failures/nulls and source axes.

On identical ORB-supported fragments, DROID has lower translation and rotation
RPE in all six cases. ORB day RGB has lower fragment ATE; other five favor DROID.
These are CPU received ORB versus GPU offline DROID outputs, not matched hardware
or latency. Thermal ORB wall is lower across three runs, but peak CPU RSS is higher
in night/rain. Image size/map history differ; no intrinsic sensor-cost claim.

Forty tests before inference, five matched-support and three inverse controls
before scoring pass. V1 general-inverse check failure is retained; V2 checks the
original rigid-transpose contract without tolerance/output changes. Independent
818,793 checks/36 mutations plus153,030 matched metric checks pass. No inference
retry, fitting, reserve access, default change or physical/adaptive benchmark claim.

Next control: distinguish received from finalized ORB poses on a predeclared
existing interval; retain map breaks and supplied timing/geometry qualifications.
Backup: backups/2026-11-05-orb-full-record-exp143.
