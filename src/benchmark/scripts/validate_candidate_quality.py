#!/usr/bin/env python3
"""Validate Topo-MPPI per-candidate quality CSV files."""

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


EXPECTED_COLUMNS = [
    "stamp",
    "cycle",
    "dynamic_scene",
    "candidate_idx",
    "selected",
    "success",
    "reject_reason",
    "raw_cost",
    "normalized_cost",
    "selection_score",
    "dynamic_scene_score",
    "risk_score",
    "static_tail_risk",
    "dynamic_tail_risk",
    "guide_consistency_risk",
    "best_score",
    "path_length",
    "topo_length",
    "final_goal_dist",
    "progress_ratio",
    "early_progress_ratio",
    "direction_cos",
    "max_goal_overshoot",
    "min_static_clearance",
    "min_dynamic_clearance",
    "topo_min_static_clearance",
    "topo_min_dynamic_clearance",
    "topo_cost",
    "waypoints",
]

INT_COLUMNS = [
    "cycle",
    "dynamic_scene",
    "candidate_idx",
    "selected",
    "success",
    "waypoints",
]

NUMERIC_COLUMNS = [
    "stamp",
    "raw_cost",
    "normalized_cost",
    "selection_score",
    "dynamic_scene_score",
    "risk_score",
    "static_tail_risk",
    "dynamic_tail_risk",
    "guide_consistency_risk",
    "best_score",
    "path_length",
    "topo_length",
    "final_goal_dist",
    "progress_ratio",
    "early_progress_ratio",
    "direction_cos",
    "max_goal_overshoot",
    "min_static_clearance",
    "min_dynamic_clearance",
    "topo_min_static_clearance",
    "topo_min_dynamic_clearance",
    "topo_cost",
]

VALID_REJECT_REASONS = {
    "mppi_failed_or_z_rejected",
    "selected_dynamic_score_or_gate",
    "selected_selection_score_or_gate",
    "no_valid_best_candidate",
    "not_selected_dynamic_score_or_gate",
    "not_selected_selection_score_or_gate",
}


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
    rows_by_cycle = defaultdict(list)

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
            for col in NUMERIC_COLUMNS:
                parse_float(row, col, row_num, errors)

            for col in ("dynamic_scene", "selected", "success"):
                if ints[col] not in (0, 1):
                    errors.append(f"row {row_num}: {col} must be 0 or 1, got {ints[col]}")
            if ints["cycle"] <= 0:
                errors.append(f"row {row_num}: cycle must be positive, got {ints['cycle']}")
            if ints["candidate_idx"] < 0:
                errors.append(
                    f"row {row_num}: candidate_idx must be non-negative, got {ints['candidate_idx']}"
                )
            if ints["waypoints"] < 0:
                errors.append(f"row {row_num}: waypoints must be non-negative, got {ints['waypoints']}")

            reason = row.get("reject_reason", "")
            if reason not in VALID_REJECT_REASONS:
                errors.append(f"row {row_num}: unknown reject_reason={reason!r}")

            if ints["selected"] == 1 and ints["success"] != 1:
                errors.append(f"row {row_num}: selected candidate must have success=1")
            if ints["selected"] == 1 and not reason.startswith("selected_"):
                errors.append(
                    f"row {row_num}: selected candidate has non-selected reject_reason={reason!r}"
                )
            if ints["selected"] == 0 and reason.startswith("selected_"):
                errors.append(
                    f"row {row_num}: non-selected candidate has selected reject_reason={reason!r}"
                )

            rows_by_cycle[ints["cycle"]].append((row_num, row, ints))

    if require_data and rows_checked == 0:
        errors.append("candidate CSV has no data rows")

    for cycle, entries in rows_by_cycle.items():
        selected = [entry for entry in entries if entry[2]["selected"] == 1]
        successes = [entry for entry in entries if entry[2]["success"] == 1]
        if len(selected) > 1:
            row_nums = [str(entry[0]) for entry in selected]
            errors.append(f"cycle {cycle}: more than one selected candidate at rows {row_nums}")
        if successes and not selected:
            errors.append(f"cycle {cycle}: has successful candidates but no selected candidate")

        candidate_indices = [entry[2]["candidate_idx"] for entry in entries]
        if len(candidate_indices) != len(set(candidate_indices)):
            errors.append(f"cycle {cycle}: duplicate candidate_idx values")
        if candidate_indices and sorted(candidate_indices) != list(range(max(candidate_indices) + 1)):
            warnings.append(
                f"cycle {cycle}: candidate_idx values are not contiguous from zero: "
                f"{sorted(candidate_indices)}"
            )

    return rows_checked, errors, warnings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", help="Path to *_candidate_quality.csv")
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="Allow header-only files from runs where no MPPI candidates were generated",
    )
    args = parser.parse_args()

    path = Path(args.csv_path)
    if not path.exists():
        print(f"ERROR: candidate quality CSV does not exist: {path}", file=sys.stderr)
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
