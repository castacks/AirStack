"""Strict estimator-facing schema and M06 code projection."""
import csv

KEYS = ("corpus", "case", "realization", "split", "time_s")
CODES = ("R_code", "G_code", "B_code", "lwir_code")
FEATURES = (*KEYS, *CODES)


def m06_keys(pair, frame):
    return ("M06", pair["case"], int(pair["realization"]), frame["split"], int(pair["time_s"]))


def project_m06(pair, frame):
    return (*m06_keys(pair, frame), *(pair[key] for key in CODES))


def rows(path, fields):
    with path.open(newline="") as source:
        reader = csv.DictReader(source)
        if tuple(reader.fieldnames or ()) != fields:
            raise ValueError("unexpected input columns")
        for row in reader:
            if set(row) != set(fields) or any(value is None for value in row.values()):
                raise ValueError("malformed record")
            yield row
