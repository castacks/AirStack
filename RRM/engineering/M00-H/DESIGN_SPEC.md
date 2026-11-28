# M00-H design
Date: 2026-11-28T02:21:18.705045+00:00; ore_proj; starting45d026e2b8c46b0e1c44013d8dd9d7ed49e3d5ac; Status DONE.

(a) Build — DONE: deterministic local dependency checker and conditional original-host CMake build; do not install or stub. Python dependency-free components compile/test separately. Full build BLOCKED if compiler/CMake/ROS/libs absent.
(b) Semantics/adapters — DONE: pure source-pinned guard reference with failed-attempt/unsafe/readback semantics; lifecycle, MAVROS, graph and recorder adapters preserve source clocks and never control startup/guard. Explicit unverified ROS binding. Correct G reference omission without modifying G history.
(c) Recorder — DONE: exclusive binary append journal, complete partial-write handling, per-record fsync and directory sync, hash/schema/identity/sequence checks, failed-writer latch, immutable interrupted originals and linked new-segment recovery. No silent repair/truncation. Synthetic memory/temp-file/process-failure tests only; all writes RRM.
(d) Integration/preflight — DONE: deterministic fixtures exercise20 required cases; incorporate G static policy read-only, build/source/reference and test results with runtime always NOT VERIFIED/BLOCKED. Full protected/input/history/20CHK/29MAP validation and local commit; no active services.
