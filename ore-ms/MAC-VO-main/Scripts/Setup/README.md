# OSMO + Box staging

## Blackwell GPU environment

Build and run from the MAC-VO repository root:

```bash
docker build --progress=plain --network=host -t macvo:blackwell \
  -f Docker/Dockerfile.blackwell .
docker run --rm --gpus all \
  -v /root/AirStack/ore-ms:/workspace -w /workspace/MAC-VO-main \
  macvo:blackwell python Scripts/Setup/verify_mact_checkpoint.py --forward
```

The image uses Torch 2.7.1 / torchvision 0.22.1 with CUDA 12.8 and native
PyTorch attention. Do not install the legacy `requirements.txt` in this image:
its xFormers pin would replace this framework with a Blackwell-incompatible
version. The local Docker image must be rebuilt or published to an accessible
registry to use it in a future OSMO workflow; its tag alone does not persist.

Verified on RTX PRO 5000 Blackwell: strict MACT checkpoint loading and GPU
forward passes at 128x128 and 480x640, with finite flow and positive covariance.
The 480x640 synthetic forward took 0.404 seconds including first-use overhead;
this is a smoke test, not a dataset accuracy result or a throughput benchmark.
`python MACVO.py --help` also passes in this image. Full odometry and DPVO
remain untested.

## Verify the recovered MACT frontend

From the MAC-VO repository root, in an environment with the model dependencies:

```bash
python3 Scripts/Setup/verify_mact_checkpoint.py
python3 Scripts/Setup/verify_mact_checkpoint.py --forward --device cpu
```

Both passed for the extracted `../macslam/MACT_Offroad.pth`: a strict load of
428 entries and finite synthetic flow/covariance outputs. Use `--forward` for
the GPU test. The current `macvo:box` image uses Torch 2.4 and cannot execute
GPU kernels on this session's Blackwell GPU; a CUDA 12.8 / Torch 2.7+ build
is required. Device discovery alone does not validate GPU execution.

FlowFormer was recovered at `c836f66e597620a2ffaaffb60693201050b93b7a`;
DPVO at `f7266f7b0b3c212001afe260d391539376df8283`. DPVO compiled extensions
have not been built. The archive lacks the parent Git metadata needed to
verify the original submodule revision pins.

OSMO workflow storage is disposable. This setup installs the `rclone` command
in the project Docker image and configures it at workflow startup from an
OSMO-managed secret. The Box token is never committed to this repository.

## One-time Box authorization

1. Start an OSMO workflow with the project image, then run `rclone config`.
2. Create a remote named `ore_ms` with storage type `box`; leave the client ID,
   client secret, app config, and access-token prompts blank; select the
   `user` account type.
3. Because OSMO is headless, select `n` when asked to use a browser. Run the
   printed `rclone authorize ...` command on your own browser-capable computer,
   complete the CMU Box sign-in, then paste the resulting token back into the
   OSMO terminal. Do not send the token in chat.
4. Still in that OSMO workflow, encode its newly created config:

   ```bash
   base64 -w 0 ~/.config/rclone/rclone.conf
   ```

5. On your **laptop** (where `osmo login` is configured), save the single-line
   output as an OSMO `GENERIC` credential. This avoids placing it in shell
   history:

   ```bash
   IFS= read -rs BOX_RCLONE_CONFIG_B64  # paste the base64 config, then press Enter
   osmo credential set ore-ms-box --type GENERIC \
     --payload "rclone_config_b64=${BOX_RCLONE_CONFIG_B64}"
   unset BOX_RCLONE_CONFIG_B64
   ```

   Do not save the value in this repository, Box, or a shell-history file.
   CMU Box SSO may require a Box password configured under Settings → Account
   → Authentication before OAuth succeeds.

6. Add this reference to the `credentials:` section of the OSMO task that
   runs MAC-VO (for AirStack's `osmo/workflows/airstack-dev.yaml`, it sits next
   to `airlab-nucleus` and `airlab-docker-login`):

   ```yaml
   ore-ms-box:
     RCLONE_CONFIG_B64: rclone_config_b64
   ```

   The left side is the environment variable available in the task; the right
   side is the field in the OSMO credential. Use `RCLONE_REMOTE: ore_ms` in
   the task's `environment:` section if the default is overridden.

Only use this remote from OSMO. Copying the same Box refresh token to multiple
machines can invalidate it.

## Every workflow

Expose `RCLONE_CONFIG_B64` and `RCLONE_REMOTE=ore_ms` to the workflow, then run:

```bash
bash Scripts/Setup/bootstrap_osmo.sh
```

Stage an active input subset with `copy`, not `sync`:

```bash
rclone copy ore_ms:ore-ms/macslam.zip ./Model --progress
rclone copy ore_ms:ore-ms/tartandrive3/20251027 ./data/tartandrive3/20251027 \
  --progress --transfers 8 --checkers 16 --retries 10
```

Before ending the workflow, copy experiment outputs back to Box. Keep the
AirLab NAS as the canonical source of raw data and use Box only for selected
inputs, model weights, manifests, and experiment artifacts.
