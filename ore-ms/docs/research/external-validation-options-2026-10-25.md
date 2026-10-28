# External validation options — 2026-10-25

**Access update, 2026-10-28:** CATS approval is now received from the user and bounded v2 payload access is verified. See [the intake and sample schema](cats-intake-2026-10-28.md). The access statements below describe the original 2026-10-25 shortlist; they no longer describe current CATS availability.

User update: current students may not know the original acquisition/reference details. Responses from Yifei and Matthew remain pending. Treat those replies as useful source clarification; pursue independent research datasets in parallel rather than making all validation depend on the replies. This note is a source-based shortlist, not a new experiment or verified dataset readiness result.

## Priority and scope

Prioritize a stronger external reference for each measurement target. A new dataset can validate the uncertainty/error relationship on its own documented protocol; it cannot retroactively establish TD3/MS² exposure timing, calibration validity or checkpoint training membership. Retain existing results as labelled projected-reference diagnostics.

| Candidate | What the primary sources document | Potential role and limits | Current access evidence |
|---|---|---|---|
| CATS | RGB stereo, thermal stereo, Trimble laser-scanned static scenes; software-triggered cameras. Long scan duration motivates static geometry. | Best first lead for independent stereo correspondence/depth-error validation. Static geometry reduces motion/timing sensitivity; projection/calibration/occlusion and actual sample format still require inspection. No temporal/VO validation follows. Scanner accuracy is not the total projected-label error. | Official download page currently requires an access request. No request submitted or new data acquired. |
| STheReO | Sequential RGB/thermal stereo, LiDAR/IMU and baseline poses from LIO-SAM with RTK-GPS. Paper explicitly discusses NUC-related thermal frame loss/asynchrony. | Candidate for later sequential evaluation with a reference independent of MAC. Not an automatic synchronization fix or surveyed pose truth. Previously measured calibration/pose-gap issues and EXP040 download limits remain unresolved, not silently waived. | Official page lists sequence links. Prior EXP040 sample-image download hit Drive quota; image availability was not re-probed in this shortlist. |
| NGCLAB Multi-Spectral Dataset | One standard camera and one LWIR camera with hardware trigger; Kinect2 depth; motion-capture poses documented. | Potential component/motion-reference study after checking per-sequence labels. Documented rig lacks separate RGB and thermal stereo pairs, so it is not a drop-in for the current MAC stereo experiment. No pipeline/method pivot approved or implemented here. | Public repository links sequences and calibration; actual payload access/labels not inspected here. |

CATS is a complementary static stereo check, not a replacement for the temporal research question. Request/access and bounded file-level inspection should precede any claim of readiness or large download. If available, freeze a scene subset before model scores, verify raw thermal encoding, original calibration and label masks, and retain reference uncertainty. Do not assume that a public benchmark is error-free or checkpoint-training-held-out.

## Primary sources checked

- [CATS setup](https://bigdatavision.org/CAT/setup.html): camera configuration, software trigger, slow laser scanner and static-scene procedure.
- [CATS CVPR2017 paper](https://openaccess.thecvf.com/content_cvpr_2017/papers/Treible_CATS_A_Color_CVPR_2017_paper.pdf): benchmark construction and scope.
- [CATS download](https://bigdatavision.org/CAT/download.html): current request-access form.
- [STheReO paper](https://sangwoojung98.github.io/pdf/inter_conf/yun2022sthereo.pdf): thermal NUC/asynchrony, stereo rig and LIO-SAM/RTK-GPS baseline construction.
- [STheReO download](https://sites.google.com/view/rpmsthereo/download): official sequence listing.
- [NGCLAB repository](https://github.com/NGCLAB/multi-spectral-dataset): hardware synchronization, Kinect2 depth, motion-capture reference and documented camera configuration.

No emails, author questions or access forms were submitted. No dataset was downloaded, adapted or evaluated in this source-only shortlist. No new physical accuracy result is claimed. Additional ordinary day images are not the immediate requirement; independent reference construction and usable stereo data are the selection criteria.
