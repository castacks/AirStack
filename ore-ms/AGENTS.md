# Research operating rules

Read [the current handoff](MS_RESEARCH_HANDOFF.md) before research work. Use the
[research index](docs/research/INDEX.md) to find dated protocols and findings.
The [history](MS_RESEARCH_HISTORY.md) preserves earlier checkpoints; its proposed
next steps do not override the current handoff.

## Research decisions

- Deployment characterization is a valid research objective: quantify RGB-only,
  thermal-only and fusion benefits, failure/recovery behavior and cost across
  conditions. Do not require a new algorithm or an unsolved-baseline failure as
  an admission gate when the user has selected characterization as the goal.
- Consult relevant primary papers and released implementations before recommending
  a method. Compare it with the experiment inventory and name the measured failure
  it addresses. Keep author claims, our interpretation, feasibility and local results separate.
- Review gaps in the foundational papers broadly. The original uncertainty-comparison
  question does not exclude other evidence-backed directions. Accuracy or measured
  matched-accuracy computation is the application goal; scoped gains are useful.
- Keep each proposed experiment finite: one hypothesis, fixed controls, explicit
  endpoints and a stopping decision. A failed configuration closes that configuration,
  not a whole research direction. Do not repeat closed diagnostic or tuning loops.
- Update the [paper registry](docs/research/references/README.md), `papers.json`
  and `papers.bib` when a source informs work. Preserve reading events, stable IDs,
  source/code/checkpoint pins and limitations. Abstract-only reading and unchecked
  code must be explicit; a mechanism prototype is not an author-system reproduction.
- Distinguish uncertainty ranking, statistical calibration, selection, pose accuracy
  and computation. Gains in one do not establish gains in the others or novelty.

## Evidence and execution

- Preserve frozen protocols, source hashes, original failed attempts, independent
  auditors, missing outputs and qualification failures. Do not relax gates or select
  successful windows after seeing outcomes. Keep positive and negative cases together.
- Treat exposed development data, synthetic targets, supplied-reference benchmarks
  and independently qualified physical sensor evidence as distinct populations.
  Physical exposure/clock semantics and training overlap remain qualified limitations.
- Frontend match/essential support is not pose accuracy or stateful VO recovery.
  Check bright-control reference sanity before interpreting error rankings. Dataset
  condition names do not make every interval smoky or zero-lux; inspect fixed-time
  samples and retain clear/static segments with explicit population labels.
- Keep tracked-body, IMU and camera frames and clock shifts distinct. A Kalibr
  camera–IMU transform/shift does not establish the mocap chain. Fixed world axes
  cancel from relative motion; do not fit axis flips to rescue exposed errors.
- Keep MAC-VO visible stereo, MACThermal thermal stereo, MAC-I²/MAC-VIO visual-inertial
  estimation and our RGB–thermal extensions distinct. Generic weighting and rotation/
  translation decoupling have established prior art.
- No automatic training, weight/head/window/threshold rescue, clock fitting or protected
  dataset access follows from a failed study. Follow the applicable frozen input contract.
- Keep raw results, datasets, checkpoints, credentials and local notebooks outside Git.
  Verify Box archive/member hashes when restoring evidence. Historical runtime success
  does not establish the current machine's environment.
- Check existing local/Box coverage before requesting data. When additional data would
  materially strengthen a study, name the recording or conditions and explain why.

## Documentation ownership

- README: stable project overview and navigation.
- Current handoff: present question, decisive evidence, unresolved issues and next task.
- Dated research notes and experiment records: detailed protocols, outcomes and decisions.
- Research history: preserved checkpoints, not a second current action list.
- This file: operating rules, without an appended experiment progress log.

Add a link to the index for a new study. Keep frozen script paths and evidence pins
intact unless an explicit migration preserves their reproducibility contracts.

## Enforced maintenance

Run `python3 ore-ms/maintenance/check_docs.py` from the repository root after
documentation edits. CI checks entry-page size, local file links, numbered
experiment coverage and the frozen history checksum. Entry-page limits are
100 lines / 6 KB for README, 120 lines / 8 KB for these rules, and 150 lines /
12 KB for the current handoff. Put detailed progress in dated findings and link it.
Replace superseded current-status paragraphs rather than appending another log.

Before adding a script, search both `tools/` and `MAC-VO-main/Scripts/Setup/`
for an existing runner or helper. Extend reusable code when its contract permits;
keep independent auditors independent. A necessary frozen variant must explain
its difference and superseded version in its experiment record. Do not copy a
runner merely to change paths or settings. Do not move frozen scripts during
routine cleanup. Review the affected topic for superseded summaries when closing
a study. Do not raise budgets or rewrite archive checksums just to pass CI.

Before freezing or launching a capture, preflight every declared interval for every
arm: required events, native support flags, factor binding and input hashes. Record
missing support explicitly; define any fallback before predictions. Do not discover
whole-span input gaps by launching a prefix, or change the population after failure.
