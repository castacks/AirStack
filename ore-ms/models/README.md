# Model inputs

Extract the original `data/archives/macslam.zip` into this directory:

```bash
python -m zipfile -e data/archives/macslam.zip models
```

Run that command from `ore-ms/`. The resulting local, ignored directory is:

```text
models/macslam/
  MACT_Offroad.pth
  MACVO_FrontendCov.pth
  MACVO_posenet.pkl
```

The archive SHA-256 is
`77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c`.
Use the setup verifier to check the actual frontend state and execution; merely
finding a file is not checkpoint validation. Source inputs and generated outputs
stay out of version control. See `MAC-VO-main/Scripts/Setup/README.md` for runtime
commands and the audited RGB compatibility migration.
