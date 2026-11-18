# IRS RTVI continuous-frame photometric intervention

## Candidate natural transitions

The [IRS release](https://christopherdoer.github.io/datasets/irs_rtvi_datasets_iros2021)
describes the dark recordings as lights switched off. Whole-span inspection
of the three staged recordings confirms that the observed darkness is
already present at the start. A frozen one-second median of visible-image
p95 intensity, with at least 15 frames per bin, classifies all 95 eligible
easy bins as bright and all 135 dark plus 87 fast-dark bins as dark. The
dark bin medians are 10–12 on an 8-bit scale. The easy medians are 225–252.5.
No recording has two consecutive bright bins followed by two dark bins.

The [current TartanRGBT release](https://huggingface.co/datasets/theairlabcmu/TartanRGBT)
has updated extrinsics and raw ZED IMU. Its supplied odometry is MAC-VO
output, and the [raw topic list](https://huggingface.co/datasets/theairlabcmu/TartanRGBT/blob/main/RAW_BAGS.md#topics-in-each-bag)
does not list a separate mocap or positioning reference. It cannot give
independent metric truth for this comparison. The already reviewed
[SubT-MRS FlashLight sequence](flashlight-qualification-findings-2026-11-16.md)
retains its nominal supplied-reference tier.

## Paced replay with continuous visual arrivals

Use the first 46 s of the already qualified `mocap_easy` bag and the
[existing one-slot handoff](irs-rtvi-single-slot-handoff-2026-11-17.md).
Dimming divides native 8-bit visual pixel values by 16 using integer
division during the same fixed 40–42 s interval as the earlier outage.
Visual p95 in the affected frames changes from 229–253 to 14–15.
All arms receive exactly the same stream of input timestamps. The
scheduled arm selects thermal during the interval and returns to visual
immediately. Discarded visual images in the scheduled arm do not need
to be rendered. The outage schedule is supplied externally.

| 1× replay arm | Median reported tracks during window | Largest wall-clock publication gap | Position RMSE during window | RMSE in first 2 s after return |
|---|---:|---:|---:|---:|
| Unmodified visual | 10 | 0.061 s | 0.045 m | 0.041 m |
| Dimmed visual | 0 | 0.056 s | 0.058 m | 0.066 m |
| Scheduled switch to thermal during dimming | 10 | 0.065 s | 0.066 m | 0.066 m |

The longest visual input interval during the intervention and its
boundary is 50.02 ms in every arm, well below the earlier 125 ms
frame-loss timeout. All 40 scheduled thermal frames produced poses.
All three arms score 40 positions during the intervention and 40
after return. The unmodified replay matches 917 shared native visual
pose rows within the frozen tolerance. The independent checker also
matches all 799 shared pre-intervention pose rows across the three arms.

All arms are within the earlier 0.20 s publication and 0.15 m nominal
position bounds. The handoff restores reported features, while staying
on visual yields slightly lower nominal error during this short
intervention. The return-window errors are effectively the same between
the two intervention arms. A two-second tracking loss can therefore
leave pose error within the existing bound in this easy replay.
The test establishes neither an accuracy advantage for switching nor
the behavior of a real low-light sensor or a causal quality detector.

The extraction and replay artifacts are under `illumination-intake/`
and `photometric-transition/` in
`/root/ore-ms-artifacts/irs-rtvi-intake/stateful/`. The preflight first
used camera-header origins for the two dark bags. Repeating with the
earliest exact bag record timestamp preserved the bin classification
and event result. Both inventories are retained. No new bags or Box
storage were needed.
