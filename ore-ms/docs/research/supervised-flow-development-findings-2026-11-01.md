# EXP097 — source-backed RGB flow development data

2026-11-01 America/New_York. Acquisition/preparation protocol e05d7e061,
[exact before-payload declaration](supervised-flow-development-protocol-2026-11-01.json).
The EXP096 runner now has an executable, source-backed development manifest:
16training pairs and4validation pairs from two TartanAirV1 environments.
No fitting, model inference, optimizer step or quality comparison occurs here.

## Source and partition

The [official V1 format documentation](https://github.com/castacks/tartanair_tools/blob/841584a8a1ebbdcc832430212609d48b6b624443/data_type.md)
describes left-camera RGB, provided forward optical flow and separate invalidity
masks. The [official downloader](https://github.com/castacks/tartanair_tools/blob/841584a8a1ebbdcc832430212609d48b6b624443/download_training.py)
selects `theairlabcmu/tartanair` as its Hugging Face source. That dataset's captured
metadata declares BSD3-Clause. Its immutable revision is
`65e00180ea952475878a748543e11e9ec20beaac`; file sizes and asserted LFS hashes
are pinned. Captured source/license notices and attribution travel in recovery.

Training: `tartanair-v1/abandonedfactory/Easy/P000`,first16adjacent pairs
000000→000001 through000015→000016. Validation:
`tartanair-v1/amusement/Easy/P001`,first4adjacent pairs000000→000001 through
000003→000004. Trajectories were selected from archive metadata before pixels;
P001 is the first available amusement Easy trajectory. Entire trajectories,
including camera aliases, remain in their assigned partition. No filtering by
image appearance, label statistics or model errors occurred. All20pairs retained.

These are synthetic RGB provided-reference development labels, not thermal
radiometry or independent physical VO truth. Base checkpoint training membership
is still unknown. This establishes recording separation for our new finetuning
stage, not network-pretraining-held-out validation. The20prior project ledger
entries and two conditional MS² reserves remain unchanged.

## Acquisition and exact arrays

Six full public ZIPs total about64GB. Instead, bounded HTTPS range reads selected
62members:22leftimages,20flowNPYs,20maskNPYs. Acquisition transferred65,498,766bytes;
metadata indexing separately used8,801,558bytes. Every response required exact206
Content-Range/body length; no full-response fallback. Pinned central entries,
ZIPCRC,extracted member sizes and SHA256 were checked. Full-archive LFS hashes
are source assertions, not independently recomputed hashes of entire downloads.
No pose/depth/right-image member was extracted. Signed redirect URLs are excluded
from logs; public revision URLs/ranges/hashes are retained.

`Scripts/Setup/remote_zip.py` implements bounded seekable range access.
`Scripts/Setup/prepare_tartanair_flow.py` packs unchanged native480×640 arrays:
OpenCV uint8BGR→RGB→FP32/255→CHW; provided FP32(u,v)flow→CHW without scaling;
valid mask is exactly source uint8mask==0. Positive invalid values are excluded,
consistent with the [official Microsoft sample](https://learn.microsoft.com/en-us/azure/open-datasets/dataset-tartanair-simulation).
The original mask contains0,1,10,100. Loss-level finite-target/max_flow checks
remain separate. No crop, resize, augmentation, padding or photometric fitting.
Separate V1 flow/mask files are explicit; the legacy NPY loader is not substituted.

## Validation and artifacts

Independent Pillow RGB decoding and NumPy label loading passed320checks across
all20pairs/80prepared array fields. Shapes,dtypes,flow bytes,mask polarity and
image tensor bytes agree exactly. Source masks mark5,783,582pixels valid across
6,144,000total;all20samples have valid supervised targets under max_flow400.
The source-aware existing ledger validator verifies all snapshot hashes/literal
quotes, and the runner's manifest inspector admits the data.

The first preparation report is retained. Before publication, a metadata amendment
normalizes the new exposure observations to the existing enum and adds a literal
source snapshot for its evidence quotes. Original manifest/ledger bytes and hashes
are preserved; arrays, selection, recipe and optimization settings are unchanged.

CPU suite241pass/1CUDA-onlyskip (9new+232regressions). All594prior frozen source
files,original ledger and both supplied checkpoint SHA identities remain unchanged.
Only new range/preparation/test source files are added. No old model,trainer,
selector,loss,adapter or inference defaults change.

The ready [manifest](../../experiments/covariance-retraining-tartanair-development-2026-11-01.json)
pins the [recipe](../../experiments/covariance-retraining-tartanair-recipe-2026-11-01.json),
[new versioned exposure ledger](../../experiments/recording-exposure-ledger-2026-11-01-tartanair-development.json),
source label evidence,all20prepared files and exact RGB initialization. Raw and
prepared data live at `data/tartanair-v1-development/run-001/`. Full range logs,
ZIP indices/source snapshots,acquisition/preparation reports,original metadata
copies/amendment and independent validation live at
`results/supervised-flow-development/run-001/`.

## Next bounded fit

The before-payload protocol fixes seed0,16steps,checkpointinterval8,AdamWlr1e-5,
betas[.9,.999],eps1e-8,decay0,clip1,gamma.85,max_flow400,constantLR,batch1cyclic.
The released RGB initialization remains
`bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a`.
Architecture/preprocessing/runtime/loss use EXP095/096 contracts. These parameters
were not selected from this data's model errors. This is a sanity-fit budget,
not an adequate training schedule or a promise of better calibration.

Run from MAC-VO-main in the pinned GPU runtime with a fresh output directory:

```sh
python -m Train.MatchingNet.retraining_runner --root /root/AirStack/ore-ms --manifest /root/AirStack/ore-ms/experiments/covariance-retraining-tartanair-development-2026-11-01.json --output /root/AirStack/ore-ms/results/tartanair-covariance-development/run-001 --device cuda
```

No model run has started. Evaluate the declared full validation prefix without
checkpoint/window selection and record frozen-mean/source/state/resume integrity
before discussing any quality effect. This data preparation does not qualify the
separate MS² temporal reference/test contract; its previous7requirements remain.
Recovery: `backups/2026-11-01-supervised-flow-development` on ore-ms Box root.
