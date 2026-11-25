# M04 acceptance report

- **Experiment** ORE-MS-EXP-PER-001
- **Directive** M04-R1
- **Recommendation** ACCEPT WITH CONDITIONS. All eight frozen verification criteria passed. Results support deterministic, synthetic, binary visibility for the declared primitives and numerical rules. Project Director approval, visually complex geometry, physical camera validation and external review remain open.

The implementation supports configured pinhole transforms, depth-ordered world-plane rectangles, deterministic translating target and foreground, and independently reported target-only projection, first hit and visible mask. The [geometry specification](SCIENTIFIC_MODEL.md) and [requirements matrix](REQUIREMENTS_TRACEABILITY.csv) describe frames, units, Jacobian, tie policy, and case scope.

| Experiment | Result |
|---|---|
| Static unobstructed and partial cases | Target has 35 in-frame pixel centers. Blocking 1, 3, 5 and 7 columns left 30, 20, 10 and 0 visible pixels. |
| Near, far, overlap and order | A farther overlapping patch left all 35 target pixels visible. The nearer overlap case left 10. Enumeration had no effect. |
| Moving foreground | Entered the image at the 1 s sample and blocked 15 target pixels at 4 s. All 35 were visible again at 6 s, with the foreground out of frame at 8 s. |
| Moving target | In-frame support changed from 30 to 35, then down to 0. At the 6 s sample the visibility fraction was undefined. |
| Distinct foreground and complete occlusion | At the center the red direct band rose from 0.17189 to 0.33088 W·m⁻²·sr⁻¹ and the LWIR band from 56.7010 to 80.1190 W·m⁻²·sr⁻¹. Changing every hidden target material property left the entire observed table byte-equivalent after disregarding case labels. |
| Matched, attenuated and saturated controls | An open target pixel and an occluded pixel with matched foreground had identical codes in all four bands. Attenuation and detector clipping changed observations without changing any geometric labels. |

The 16 configurations yielded 30 frames, 5,610 ray records and 1,148 first-hit radiometric records. The maximum range/depth difference was 3.55e−15 m and there were zero oracle label disagreements. All serialized results reproduced byte for byte. See the [visibility plot](evidence/visibility.png), [frame metrics](evidence/frames.csv), [verification detail](VERIFICATION_REPORT.md) and [reproduction instructions](REPRODUCTION.md).

The [roadmap mandate](../RESEARCH_ROADMAP_MANDATE.md) records M01–M03 conditional acceptance and M04 in verification. M05–M13 await authorization. No stochastic evolution, combined multispectral simulator, inference or fusion was implemented.
