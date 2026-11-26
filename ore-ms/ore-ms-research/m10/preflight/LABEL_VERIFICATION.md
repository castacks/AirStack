# Independent label verification

The renderer sends M04 world-space rays through finite rectangles and records projected target support and the nearest intersected object. The audit reverses this calculation in image space. For each declared plane, the independent oracle enumerates its integer-center footprint, applies frame clipping, and paints surfaces in decreasing depth order. Set intersection and exact rational sums give the visibility fraction and centroid. The oracle imports neither the renderer nor the M04 visibility function for label verification. Its separate radiometry check compares the rendering kernel to the accepted M06 scalar adapter.

| Control | Projected | First-hit target | Visible fraction | Label |
|---|---:|---:|---:|---|
| Reference and nominal support interventions | 48 | 48 | 1 | Feasible |
| Partial occlusion | 48 | 24 | 1/2 | Feasible |
| Both boundaries exactly | 16 | 4 | 1/4 | Feasible |
| Exactly 25% with one pixel too few | 12 | 3 | 1/4 | Insufficient partial |
| Exactly four pixels below 25% | 20 | 4 | 1/5 | Insufficient partial |
| Full occlusion | 48 | 0 | 0 | Hidden |
| Target outside the frame | 0 | 0 | Undefined | Empty support |
| No target or matched-looking distractor | 0 | 0 | Undefined | Absent |
| Distractor or contrast without target support | 0 | 0 | Undefined | Distractor |

All 13,824 opportunities and both geometry planes agreed. The maximum centroid discrepancy was 0 px against the 1e−12 px tolerance, and every threshold fixture reproduced across all 512 realizations. The independent test reports the opaque ID and expected and observed values on disagreement. On 36 scalar M06 transfer probes covering four materials, three media and three slant ranges, the largest absolute band-integral difference was 1.42e−14 against a 1e−10 tolerance. See [independent verification](evidence/independent_verification.json).
