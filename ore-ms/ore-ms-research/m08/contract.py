"""Strict serialized forecast boundaries shared by generation and scoring."""
import csv

IDS = ("case","realization","split")
FEATURES = (*IDS,"R_code","G_code","B_code","lwir_code")
LABELS = (*IDS,"horizon_s","burden","rgb_t550","lwir_t11")
ORACLE = (*IDS,"coefficients")
FORECASTS = (*IDS,"method","horizon_s","status","flags","ess","burden_samples")
TARGETS = LABELS[4:]


def rows(filename, columns):
    with filename.open(newline="") as file:
        reader = csv.DictReader(file)
        if tuple(reader.fieldnames or ()) != columns:
            raise ValueError("unexpected input columns")
        for row in reader:
            if set(row) != set(columns) or any(value is None for value in row.values()):
                raise ValueError("invalid data row")
            yield row


def output(file, columns):
    writer = csv.writer(file,lineterminator="\n")
    writer.writerow(columns)
    return writer
