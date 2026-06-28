#!/usr/bin/env python3
"""Validate Topo-MPPI per-cycle planning timing CSV files."""

import argparse
import csv
import math
import sys
from pathlib import Path


EXPECTED_COLUMNS = [
    "stamp",
    "cycle",
    "success",
    "fail_reason",
    "total_ms",
    "init_ms",
    "topo_ms",
    "mppi_ms",
    "bspline_ms",
    "refine_ms",
    "validator_ms",
    "topo_paths",
    "mppi_candidates",
    "mppi_successes",
    "continuous_failures",
    "bspline_failures",
    "start_goal_dist",
    "final_static_clearance",
    "final_dynamic_clearance",
    "used_mppi_fallback",
    "used_static_topo_seed",
]

INT_COLUMNS = [
    "cycle",
    "success",
    "topo_paths",
    "mppi_candidates",
    "mppi_successes",
    "continuous_failures",
    "bspline_failures",
    "used_mppi_fallback",
    "used_static_topo_seed",
]

NUMERIC_COLUMNS = [
    "stamp",
    "total_ms",
    "init_ms",
    "topo_ms",
    "mppi_ms",
    "bspline_ms",
    "refine_ms",
    "validator_ms",
    "start_goal_dist",
    "final_static_clearance",
    "final_dynamic_clearance",
]

NONNEG_COLUMNS = [
    "total_ms",
    "init_ms",
    "topo_ms",
    "mppi_ms",
    "bspline_ms",
    "refine_ms",
    "validator_ms",
    "topo_paths",
    "mppi_candidates",
    "mppi_successes",
    "continuous_failures",
    "bspline_failures",
    "start_goal_dist",
]


def parse_float(row, col, row_num, errors):
    try:
        value = float(row[col])
    except (KeyError, TypeError, ValueError):
        errors.append(f"row {row_num}: {col} is not numeric: {row.get(col)!r}")
        return math.nan
    if not math.isfinite(value):
        errors.append(f"row {row_num}: {col} is not finite: {row.get(col)!r}")
    return value


def parse_int(row, col, row_num, errors):
    value = parse_float(row, col, row_num, errors)
    if math.isfinite(value) and abs(value - round(value)) > 1e-6:
        errors.append(f"row {row_num}: {col} should be integer-like, got {value}")
    return int(round(value)) if math.isfinite(value) else 0


def row_is_repeated_header(row):
    return row.get("stamp") == "stamp" and row.get("cycle") == "cycle"


def validate(path, require_data):
    errors = []
    warnings = []
    rows_checked = 0
    last_cycle = None

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != EXPECTED_COLUMNS:
            errors.append(
                "CSV schema mismatch.\n"
                f"expected: {EXPECTED_COLUMNS}\n"
                f"actual:   {reader.fieldnames}"
            )
            return rows_checked, errors, warnings

        for row_num, row in enumerate(reader, start=2):
            if row_is_repeated_header(row):
                errors.append(f"row {row_num}: repeated header row inside CSV")
                continue
            rows_checked += 1

            ints = {col: parse_int(row, col, row_num, errors) for col in INT_COLUMNS}
            values = {col: parse_float(row, col, row_num, errors) for col in NUMERIC_COLUMNS}

            for col in NONNEG_COLUMNS:
                value = ints[col] if col in ints else values[col]
                if math.isfinite(value) and value < 0:
                    errors.append(f"row {row_num}: {col} must be non-negative, got {value}")

            for col in ("success", "used_mppi_fallback", "used_static_topo_seed"):
                if ints[col] not in (0, 1):
                    errors.append(f"row {row_num}: {col} must be 0 or 1, got {ints[col]}")

            cycle = ints["cycle"]
            if cycle <= 0:
                errors.append(f"row {row_num}: cycle must be positive, got {cycle}")
            if last_cycle is not None and cycle < last_cycle:
                warnings.append(
                    f"row {row_num}: cycle decreased from {last_cycle} to {cycle}; "
                    "file may contain appended data from multiple runs"
                )
            last_cycle = cycle

            if ints["mppi_successes"] > ints["mppi_candidates"] and ints["mppi_candidates"] > 0:
                warnings.append(
                    f"row {row_num}: mppi_successes={ints['mppi_successes']} "
                    f"> mppi_candidates={ints['mppi_candidates']}"
                )

    if require_data and rows_checked == 0:
        errors.append("planning timing CSV has no data rows")

    return rows_checked, errors, warnings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", help="Path to *_planning_timing.csv")
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="Allow header-only files from runs where no planning cycles were logged",
    )
    args = parser.parse_args()

    path = Path(args.csv_path)
    if not path.exists():
        print(f"ERROR: planning timing CSV does not exist: {path}", file=sys.stderr)
        return 1

    rows_checked, errors, warnings = validate(path, require_data=not args.allow_empty)
    for warning in warnings:
        print(f"WARNING: {warning}", file=sys.stderr)
    if errors:
        print(f"ERROR: checked {rows_checked} rows in {path}", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(f"OK: checked {rows_checked} rows in {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
