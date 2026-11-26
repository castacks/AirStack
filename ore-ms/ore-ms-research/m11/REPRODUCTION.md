# Reproduction

From the M11 directory, use Linux x86-64, CPython 3.12.3, NumPy 2.2.6 and SciPy 1.15.3. The comparison uses one OpenBLAS thread. A scratch environment avoids changes to AirStack or the simulator.

```sh
python3 -m venv /tmp/ore-m11
/tmp/ore-m11/bin/python -m pip install -r requirements.txt
M11_PYTHON=/tmp/ore-m11/bin/python ./reproduce.sh
```

The script verifies the package and imported input hashes, executes evidence and reference-boundary tests, regenerates outputs in a fresh directory, and checks all 11 science-table hashes. Timing observations are packaged but excluded from byte-for-byte replay. It prints the scratch directory. No downloads or Box access are needed once the two dependencies are installed. The original checkpoint, calibration and milestone sources come from the checkout. [Package hashes](PACKAGE_SHA256SUMS.txt) · [Input hashes](INPUT_SHA256SUMS.txt) · [Science output hashes](EVIDENCE_SHA256SUMS.txt)
