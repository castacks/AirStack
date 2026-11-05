# EXP148: fixed local projection/depth sensitivity

Freeze aa0b84505; parent 229eeb87f.

2026-11-05 — [EXP148 local projection/depth sensitivity](../docs/research/depth-projection-sensitivity-findings-2026-11-05.md): freeze aa0b84505; same 40maps/1398primary points, fixed 3×3/5×5reference windows, no new inference/acquisition/fit. Thermal5×5locally consistent subset 672 points/20 images retains original median depth ratio 0.874161 (all 0.826784);663/1089shorter than every eligible local label, including 99insufficient-support points. RGB consistency support 0 points at 3×3/one point at 5×5, no comparable camera subset. All primary errors unchanged; all strata/empties retained.6 tests,84738independent checks/4corruptions pass. Engineering window/consistency rules do not qualify visibility or physical timing; no correction/reserve/default/EXP133 promotion. Next native stereo-match ambiguity at retained points. Backup backups/2026-11-05-depth-projection-sensitivity-exp148.

