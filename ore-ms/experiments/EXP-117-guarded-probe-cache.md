# EXP117: guarded evaluation and stale-state/probe cache

Client date2026-11-03. Freeze8ac4c9ede, [findings](../docs/research/guarded-probe-cache-findings-2026-11-03.md). Eighteen retained dense native windows/seven exposed records;two trailing guardframes andtwo bootstrap pairs;1150corepairs/108policywindows. No new inference/training/reserves or pose-label reads.

Boundedprobe TTL1s/period1s/two-requestburst passes fixed masked contract versusTTLfullread:TD3eligible292/unavailable0/uncommitted3,requests590→492(16.6%core reduction);MS²851/4/0,1710→1290(24.6%). Countsbootstrap/guards separate. Requests are not saved native inference; full dense graph/timing remains. FixedT onTD3/fixedR onMS² andunboundedhold outperform adaptive controls here; no adaptive superiority or adoption.

Recovery evidence remains weak:bounded fresh return5/16MS²runs,known failure+return2/16;TD3oneone-pairreturn missed. Motion delivery needs originating pair AND later writeback invocation requested. Native sparse gap/warm-start correctness untested. Next actual native gap/probe replay with fresh densecontrols and matched-interval errors/costs.

9tests;145674cache/mask/score checks,9776nativeguard/truth checks,108hiddenoutcomemutations invariant,2150processingwritebindings.35pins/weights unchanged. Retained run001failedfixture/firstlock;fixedtest timestamp before run002/outcomes,no policy changes. Backup backups/2026-11-03-guarded-probe-cache;parentEXP116/runtimeEXP112.

| Domain / policy | Eligible preselection | Unavailable preselection | No trusted choice while available | Core pair requests | Fraction of full read |
|---|---:|---:|---:|---:|---:|
| td3 / full_read_cache | 292 | 0 | 3 | 590 | 100.00% |
| td3 / bounded_probe | 292 | 0 | 3 | 492 | 83.39% |
| td3 / bounded_no_probe | 292 | 0 | 3 | 298 | 50.51% |
| td3 / unbounded_hold | 295 | 0 | 0 | 295 | 50.00% |
| td3 / fixed_rgb | 171 | 124 | 0 | 295 | 50.00% |
| td3 / fixed_thermal | 295 | 0 | 0 | 295 | 50.00% |
| ms2 / full_read_cache | 851 | 4 | 0 | 1710 | 100.00% |
| ms2 / bounded_probe | 851 | 4 | 0 | 1290 | 75.44% |
| ms2 / bounded_no_probe | 849 | 4 | 2 | 857 | 50.12% |
| ms2 / unbounded_hold | 855 | 0 | 0 | 855 | 50.00% |
| ms2 / fixed_rgb | 855 | 0 | 0 | 855 | 50.00% |
| ms2 / fixed_thermal | 829 | 26 | 0 | 855 | 50.00% |
