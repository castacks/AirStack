# EXP-034: Localize thermal/GNSS displacement disagreement

Recorded: 2026-10-22T18:26:05.343848+00:00; branch ore_ms; starting commit e83f9d3d17ddb5a57a2b71275ddfdde731f53c57.

## Question and method

While GNSS frame/quality and checkpoint training replies are pending, determine whether the longer thermal/GNSS displacement disagreement is temporally localized. Reuse all three fixed EXP032 thermal windows. No inference, new data or changes to poses/calibration/weights/defaults. Before analysis, fix frame boundaries [0,20,40,60,80,99], partitioning all 99 transitions exactly once. Blocks share boundary frames; durations vary because original acquisition gaps are preserved. No block selection based on agreement.

For every block compare Euclidean endpoint displacement directly from saved VO translations and EXP033 interpolated GNSS XYZ positions. Preserve all three predefined GNSS offsets −0.1/0/+0.1s (45 block/offset cases); adopt none. Check whole-window norms against EXP033. GNSS and camera origins differ; magnitude disagreement is not vector pose error, ATE/RPE, or calibrated uncertainty. Do not add block magnitudes/differences to explain whole-window vector displacement.

## Zero-offset results

| Window | Frames | Duration s | Gaps >150ms | GNSS m | VO m | VO−GNSS m |
|---|---|---:|---:|---:|---:|---:|
| early | 0–20 | 2.000 | 0 | 0.8807 | 0.9218 | +0.0411 |
| early | 20–40 | 2.000 | 0 | 0.0473 | 0.0219 | -0.0253 |
| early | 40–60 | 2.000 | 0 | 0.6544 | 0.3807 | -0.2737 |
| early | 60–80 | 4.800 | 15 | 15.5087 | 11.4652 | -4.0435 |
| early | 80–99 | 4.101 | 15 | 14.6594 | 13.6863 | -0.9731 |
| middle | 0–20 | 2.000 | 0 | 6.7661 | 6.5465 | -0.2196 |
| middle | 20–40 | 2.000 | 0 | 6.9514 | 6.6440 | -0.3074 |
| middle | 40–60 | 2.000 | 0 | 6.5148 | 6.3495 | -0.1653 |
| middle | 60–80 | 2.000 | 0 | 6.7899 | 6.5947 | -0.1951 |
| middle | 80–99 | 1.900 | 0 | 5.5668 | 5.6579 | +0.0911 |
| late | 0–20 | 2.000 | 0 | 5.2609 | 5.2864 | +0.0255 |
| late | 20–40 | 2.000 | 0 | 4.5116 | 4.1817 | -0.3299 |
| late | 40–60 | 3.100 | 1 | 10.2229 | 5.6984 | -4.5245 |
| late | 60–80 | 2.000 | 0 | 6.7694 | 6.6434 | -0.1260 |
| late | 80–99 | 1.900 | 0 | 6.0607 | 5.9799 | -0.0809 |

![Fixed block displacement comparison](../results/gnss-block-localization/block-displacements.png)

Early: the two gap-heavy blocks have VO−GNSS differences −4.04m and −0.97m; the three regular-cadence blocks range −0.27m to +0.04m, with substantial differences in motion amount and duration. Late: the block containing the 1.2s gap has GNSS10.22m versus VO5.70m (−4.52m), while the four other blocks differ −0.33m to +0.03m. Middle: no gaps>150ms, block differences range −0.31m to +0.09m. This localizes the strongest scalar disagreement near gap-containing sections. It does not establish a causal effect of gaps: motion, scene, turning, depth and physical lever arms also vary. Independent LK support at long gaps was already weak in EXP032. A uniform scale correction is not justified by these results.

The plot shows GNSS range across the three fixed clock offsets; it is a sensitivity range, not a confidence interval. Original low-motion blocks remain included without relative percentage scores that could amplify jitter. GNSS jitter and unknown receiver quality limit their interpretation.

## Verification and reproduction

Independent scalar interpolation from original GNSS text and integer timestamps plus direct raw VO translation distances reproduce all90 block magnitudes: GNSS absolute tolerance1e-8m, VO1e-10m. All45 cases pass gap counts and exact transition partition checks. Whole-window controls match EXP033. Prior828/3041/1229 scientific hashes remain unchanged;15 immediate analysis input/source hashes are recorded. Figure visually checked. This is a saved-output diagnostic rather than an inference/test-suite change.

With ore-ms mounted at /workspace in the existing macvo:geometry image and working directory /workspace/MAC-VO-main:

```bash
python -m Scripts.Setup.td3_gnss_blocks
python /workspace/results/gnss-block-localization/staging/validate.py
```

Protocol, arrays, CSV, validation and figure are under results/gnss-block-localization. Runtime image matches EXP033 CPU image sha256:101a3943f3d10fbd85611e0401364e94e343f0936d7cae535a396138987cdbb1.

## Owner update and next evidence

Latest user-pasted Slack replies (displayed1:38/1:39PM, exact message date/timezone/permalink unverified): Geethika expressed surprise that targetless calibration worked better than target-based and supported following up with Yifei. This acknowledges the reported comparison, not general superiority of a calibration method. Matthew recalled the runs probably lacked RTK and suggested GNSS was suitable for satellite-map trajectories; he asked the desired conversion frame. This is tentative recollection, not verified fix status or accuracy. Exact XYZ mapping, physical origin/lever arm, timing, quality and training membership remain unresolved. Existing historical source notes stay frozen.

No additional data needed for this diagnostic. For the next numerical RGB/thermal comparison, another night recording with visible RGB scene detail would materially help; request candidate20251027 folder names before another upload. To test a gap-specific cause would require a suitable contiguous sequence/control at comparable motion and texture, not merely treating these observational blocks as a causal experiment.
