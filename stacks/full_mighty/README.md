# full_mighty

The disaster-dataset eval stack: full autonomy with the **MIGHTY**
Hermite-spline local planner (MIT ACL, RA-L 2026; `castacks/asm_mighty`)
behind `/tasks/navigate` and the trajectory controller, instead of droan_gl —
which spent its time spinning in place on the plats.

```bash
./airstack.sh up --stack full_mighty            # or AIRSTACK_STACK=full_mighty in .env
```

`launch/stack.launch.xml` is the entry `robot.launch.xml` dispatches to
(`AIRSTACK_STACK_DIR`, RFC #379 S3 as on upstream develop). The local layer
is composed flat from canonical module launch files; MIGHTY replaces the
`droan_gl` include with `mighty_module.launch.xml`, fed the STEREO cloud
(`perception/stereo_image_proc/point_cloud`, frame `camera_left`) and the
stack-tuned configs in `local_bringup/config/`.

What the module is and what this branch changed in it:
`robot/ros_ws/src/modules/asm_mighty/ORIGIN.md` (pin: `modules.repos`).
Log lines that prove the stack took, and the first-run watch list:
`.agents/skills/benchmark-disaster-dataset/SKILL.md` §4c.

Remaining part of the develop port: the interface / sensors / perception /
global / behavior layers are still included through this branch's layer
bringup files, not flattened; `airstack module add/sync/lock` do not exist
here (the module is in-tree).
