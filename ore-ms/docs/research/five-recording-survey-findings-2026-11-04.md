# EXP128: five-recording sampling and retained input-control failure

2026-11-04 / ore_ms. Sampling freeze92fe7d815; conditional native-input freeze
a221d0b81. Parentd8de5b4b2. Development study, no blind or training-held-out claim.

Four of15fixed chronological thirds meet the frozen100group input-association
rule. The subsequent strict original-producer replay control fails on all800
thermal inputs. **No native MSA inference is run in EXP128.** Keep both facts;
metadata eligibility is not native tracking, and this input-contract failure
is not a VO failure or proof that the recordings lack complementary modalities.

## Metadata selection, unchanged after outcomes

| Existing recording | Strict paired groups | Early third | Middle third | Late third |
|---|---:|---|---|---|
| night_macthermal_salon_redcourse_waypointskip1 | 510 | no100-group window | no100-group window | no100-group window |
| night_so_waypoint_skip2 | 184 | no100-group window | no100-group window | no100-group window |
| night_teleop_so_salon_fig8 | 515 | no100-group window | no100-group window | no100-group window |
| night_thermal_so_salon | 1009 | qualified | no100-group window | qualified |
| night_thermal_so_salon3 | 3237 | no100-group window | qualified | qualified |

All five records already had image/LK exposure inEXP035/036; most prior sampled
RGB was dark, and pairing holes were known. Nothing here creates an untouched
recording. Each source timestamp is parsed with Decimal into integer nanoseconds;
missing/nonfinite/unparseable/duplicate rows retain counts and original image IDs.
Chronological-nearest without reuse uses inherited2msstereo/30mscross bounds.
Within each RGB elapsed-time third, first100consecutive paired groups must have
positive<=150ms gaps in allfourstreams and nonoverlapping controller groups.
No thresholds are widened, shorter windows substituted, or scenes chosen from
brightness/support/error. All11ineligible thirds remain, and no native conclusion
is drawn for their three recordings. These are engineering association bounds,
not proof of physical exposure synchronization or sensor failure.

The qualified windows are salon early/late (RGB250–349 and856–955) andsalon3
middle/late (RGB1912–2011 and4034–4133). Source rows/rawthermal associations are
frozen before selected-image inspection. All rawthermal bindings use exact
integer-nanosecond equality with no fallback. Selected2,400PNGmembers from the two
records pass ZIPCRC and SHA registration; each window keeps all100source groups.
No pose/reference numeric stream or protectedMS2record is opened.

## Input preparation and failure

The old EXP023 archive SHA is verified, then11calibration/producer members are
verified against its member manifest and restored under a new evidence path.
The planned route uses the same originalproducer replay followed by the existing
raw-first MSArectification/intensity recipe, unchanged original solver/weights.
The first exact provided-image replay fails before the MSA image is written;
its log and partial prepared directory are retained, and it is never retried.

A separately declared retrospective diagnostic then compares **all four fixed
windows** with only the two already specified recipes. It does not authorize
native inference or choose a replacement recipe:

| Fixed window | Original-producer exact / 200 | Frozen MSA target exact / 200 | Original gray MAE median |
|---|---:|---:|---:|
| night_thermal_so_salon/early | 0 | 0 | 0.057033 |
| night_thermal_so_salon/late | 0 | 0 | 0.064130 |
| night_thermal_so_salon3/middle | 0 | 0 | 0.058171 |
| night_thermal_so_salon3/late | 0 | 0 | 0.062022 |

MAE uses8-bit gray intensity, not pose error. The originalproducer is close but
not exact (per-image MAE roughly0.04–0.19); all channels in the provided images
are replicated grayscale. The frozenMSAtarget also fails exact source parity.
This does not identify whether configuration, implementation/version, source
association or another export detail explains the differences. No cause, new
preprocessing recipe, tolerance or physical calibration is inferred or fitted.
The source_P differs from the frozenMSAprojection; no overwrite is used to claim
an original source-export calibration match.

The originalcontrol criterion stays failed. Prepared-lock, input-audit and native
capture artifacts for successful MSAexecution are absent. The user-authorized
broader survey continues only as a **separate** [EXP129 archive-export baseline](provided-export-survey-findings-2026-11-04.md),
frozen before native outcomes. It bytecopies providedimages/camera-info and makes
no MSAtransfer claim; it cannot satisfy or erase this failed control.

## Validation and implications

Eight metadata semantic tests pass. Independent association/selection audit passes
5,723checks over all5records/15thirds and sourceCRC/SHA;45initial and562stage2
source/metadata pins remain unchanged. Four-window diagnostic covers800thermal
images without changing source IDs or reading labels. Earlier source outcomes,
reserve status and failure logs remain intact.

Additional imageuploads were not needed to reach this boundary. The useful source
question is the applicable processed-thermal producer/configuration/library
version and raw-to-processed provenance for these exact recorded source IDs.
That is separate from the remaining GNSS/frame/time/quality evidence needed for
physical accuracy. The candidate and production defaults remain unchanged.

Artifacts:results/five-recording-survey/run-001. Combined recovery packet:
backups/2026-11-04-five-recording-survey. Model/runtime prerequisite EXP112/106;
calibration source EXP023. Original sourceZIPs and frozen metadata indexes EXP127.
