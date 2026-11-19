# IRS RTVI withholding dim-flagged visible updates

This [Super Odometry-inspired control](super-odometry-handoff-relevance-2026-11-18.md)
tests measurement admission in the existing RRxIO handoff.
Use the [same causal brightness rule](irs-rtvi-causal-dimming-2026-11-18.md)
on Medium and Difficult. The detector reads the same three consecutive
dimmed frames and triggers at the same stamp, and thermal starts at
the same sensor timestamp. Withhold those three visible updates from
the filter and retain all IMU messages. Keep thermal active afterward.
This isolates admission of the three flagged visible measurements
within the current filter and publication implementation.

The paired position comparison uses the existing pre-intervention
yaw and translation fit on 199 medium and 200 difficult reference
timestamps, with unit scale. The maximum interpolation bracket is
100 ms. Withholding loses the first three reference positions in each
intervention, reducing the shared cohort from 200 to 197. All arms
are rescored on that cohort. The first two recovery seconds (40
positions on each recording) and seconds 2–10 (160 medium and 159
difficult) keep their full reference support.

| Recording and arm | Dimming RMSE | First 2 s after light returns | 2–10 s after | Longest onset output gap |
| --- | ---: | ---: | ---: | ---: |
| Medium, apply flagged updates | 0.106 m | 0.183 m | 0.135 m | 50.01 ms |
| Medium, withhold flagged updates | 0.105 m | 0.181 m | 0.134 m | 195.50 ms |
| Difficult, apply flagged updates | 0.114 m | 0.254 m | 0.324 m | 50.09 ms |
| Difficult, withhold flagged updates | 0.098 m | 0.177 m | 0.209 m | 195.50 ms |

On Difficult, withholding reduces early recovery RMSE by 7.7 cm and
later recovery RMSE by 11.6 cm. On Medium it changes early RMSE by
2.2 mm and later RMSE by 0.8 mm. Both still exceed the 0.15 m early
accuracy bound. Both exceed the 0.15 s sensor-time publication-gap
bound when the flagged frames are withheld. The paired intervention
RMSE omits three unsupported timestamps per bag. It does not
characterize error inside the output gap.

Difficult's scheduled switch still scores 0.122 m in the first two
seconds after restored light, compared with 0.177 m for the withholding
control. Thus admission of degraded images contributes to its recovery
error with this detector, and withholding them alone does not reproduce
the scheduled result. The comparison does not identify a particular
bias or direction as the mechanism.

The initial veto captures matched all shared pre-intervention output
positions exactly but missed 13 Medium and 21 Difficult startup rows,
failing the 99% parity gate. These attempts are retained and unscored.
The retry waits for the odometry subscriber before starting bag
processing. It preserves the sensor times, filter, and measurement
schedule. The valid arms match all 800 Medium and 784 Difficult
native pre-intervention poses exactly. Logs verify identical trigger
and thermal switch times, exactly three withheld visible updates,
and unchanged thermal input counts. Independent arithmetic reproduces
all 30 RMSEs within 1e-10 m.

Frozen inputs, patches, captures, error grids and verification hashes
are under `/root/ore-ms-artifacts/irs-author-rescore/veto/`.
