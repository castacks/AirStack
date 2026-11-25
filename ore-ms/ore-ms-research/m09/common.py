"""Frozen M09 IO and issue-time feature definitions."""
import csv
from functools import lru_cache
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CFG = json.loads((HERE/"config.json").read_text())
HORIZONS = CFG["horizons_s"]
IDS = ("case", "realization", "split")
CODES = ("R_code", "G_code", "B_code", "lwir_code")
ISSUE = (*IDS, *CODES)
LABELS = (*IDS, "horizon_s", "burden", "rgb_t550", "lwir_t11")
TEACHER = ("realization", "split", "mode", "horizon_s", "mean", "scale", "status")
ISSUED = (*IDS, "model", "size", "seed", "mode", "status", "flags", "novelty", "novel",
          "location", "scale")

DETECTOR_MODES = {"RGB": (0, 1, 2), "LWIR": (3,), "joint": (0, 1, 2, 3)}
THREE_MODE_CASES = {"matched", "air_mismatch", "dynamics_mismatch"}


def parse(path, fields):
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if tuple(reader.fieldnames or ()) != fields:
            raise ValueError("unexpected input columns")
        for row in reader:
            if set(row) != set(fields) or any(value is None for value in row.values()):
                raise ValueError("malformed record")
            yield row


def writer(file, fields):
    result = csv.writer(file,lineterminator="\n")
    result.writerow(fields)
    return result


def dump(path, content):
    path.write_text(json.dumps(content,sort_keys=True,separators=(",",":"),allow_nan=False)+"\n")


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def logit(values):
    epsilon = CFG["logit_epsilon"]
    bounded = np.clip(values,epsilon,1-epsilon)
    return np.log(bounded)-np.log1p(-bounded)


def sigmoid(value):
    return np.exp(-np.logaddexp(0,-value))


def raw_codes(row, mode):
    output = np.full(4,np.nan)
    permitted = DETECTOR_MODES[mode]
    for i in permitted:
        if row["case"] == "thermal_unknown" and i == 3:
            continue
        try:
            value = int(row[CODES[i]])
        except ValueError:
            continue
        if 0 < value < CFG["max_code"]:
            output[i] = value/CFG["max_code"]
    return output


def features(row, mode, scaling):
    codes = raw_codes(row,mode)
    mask = np.isfinite(codes)
    values = np.where(mask,(codes - scaling["center"])/scaling["scale"],0)
    return np.r_[values,mask]


@lru_cache(maxsize=1)
def optical_contract():
    m08 = json.loads((ROOT/"m08"/"config.json").read_text())
    m07 = json.loads((ROOT/"m07"/"config.json").read_text())["optical"]
    return m08["length_m"],m07


def target_mapping(burden):
    length,m07 = optical_contract()
    return np.stack((burden,np.exp(-length*(m07["rgb_floor"]+m07["rgb_span"]*burden)),
                     np.exp(-length*(m07["lwir_floor"][1]+m07["lwir_span"][1]*burden))),axis=-2)
