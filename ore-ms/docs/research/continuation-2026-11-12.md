# Continuation decision — 12 November 2026

Resume point: `cc6752562` on `ore-ms-orb-development`. The latest completed
work is the final-output residual fit, not EXP144 ORB finalization. This note
records scope and a proposed next step; it contains no new experimental result.

## Evidence restored

Box access works through the private rclone recovery configuration. Only the
manifest and two reports from `backups/2026-11-10-final-residual-fit` were copied
to `results/continuation-2026-11-12/recovery/`. Both reports match the checked-in
versions byte for byte. No dataset, checkpoint, runtime or artifact archive was
restored. The manifest was retrieved, but its full artifact inventory was not
verified against restored payloads.

## What stays closed

- [The final residual recipe](final-residual-fit-findings-2026-11-10.md):
  training loss decreased 17.65%, but rain depth AbsRel worsened 15.47%;
  downstream motion remains worse than RGB. No budget, learning-rate or
  checkpoint search to rescue it.
- [Repeated confidence/fusion variants](fusion-mechanism-review-2026-11-09.md):
  another renamed residual, covariance or geometry score needs a new causal
  hypothesis before implementation.
- ORB finalization and baseline initialization have already been investigated
  in EXP144–152. Restoring EXP144 as the current starting point would skip
  later evidence.

## One proposed next step

Follow the [latest brief](progress-and-next-steps-2026-11-10.md): evaluate a
materially different correspondence intervention. My proposed candidate is a
frozen pretrained matcher on **within-thermal temporal pairs**, compared with
the existing thermal temporal flow. This tests correspondence means directly,
rather than another stereo output correction or camera-selection score.
TartanMatch is a candidate from the earlier source review, not a selected or
currently verified runnable dependency.

Before downloading a model or restoring image archives, answer one finite
question from existing metadata/source: can this comparison use an existing
exposed packet with a defensible correspondence-error target and an unchanged
downstream motion evaluation? Check the native flow coordinate/direction,
occlusion/eligibility rules, and how replacement flow would enter the frozen
solver. Supplied projected depth alone is not certified cross-frame visibility.
Predicted cycle consistency must not become its own accuracy label.

The [earlier representation review](shared-representation-feasibility-2026-11-09.md)
did not establish this target. Its warning against an open-ended matcher
integration remains relevant; the November 10 brief motivates reconsidering
one bounded intervention, not starting an encoder-training project.

**Stop rule:** if the existing packet cannot support that comparison, record
the exact missing target or interface and stop this candidate. Do not expand
into dataset acquisition, reference reconstruction or a new backend to make
the pilot possible. If it can, freeze one small comparison and its advancement
rule before execution. Advancement requires correspondence accuracy and
downstream translation improvement without rotation or coverage regression;
matching counts or attractive visualizations alone are insufficient. Numerical
thresholds and input membership remain to be specified before any run.

No new training, inference, reserve access or model/default change occurred
during this continuation. Historical results were reviewed, not rerun.
