#!/usr/bin/env python3
"""Validate benchmark_results_extended.csv.

This checker catches common data-quality failures before the metrics are used
for algorithm tuning or paper tables:
  - stale/incorrect CSV schema
  - non-numeric values in numeric columns
  - impossible negative times, path lengths, counts, or thresholds
  - distance sentinel misuse, e.g. min distance is -1 while samples exist
  - dynamic-scene rows that never received /dynamic_obstacles/state samples
  - optional paper-quality bounds for velocity, acceleration, and jerk
"""

import argparse
import csv
import math
import sys
from pathlib import Path


EXPECTED_COLUMNS = [
    "method",
    "scenario",
    "run_id",
    "success",
    "travel_time_s",
    "traj_length_m",
    "min_obs_dist_m",
    "collision_count",
    "collision_sample_count",
    "time_below_margin_s",
    "below_margin_sample_count",
    "cloud_samples",
    "min_static_clearance_m",
    "static_collision_count",
    "static_collision_sample_count",
    "static_time_below_margin_s",
    "static_below_margin_sample_count",
    "static_samples",
    "min_dynamic_clearance_m",
    "dynamic_collision_count",
    "dynamic_collision_sample_count",
    "dynamic_time_below_margin_s",
    "dynamic_below_margin_sample_count",
    "dynamic_samples",
    "avg_vel_ms",
    "max_vel_ms",
    "avg_acc_ms2",
    "max_acc_ms2",
    "jerk_integral",
    "smoothness_cost",
    "collision_threshold_m",
    "safety_margin_m",
    "dynamic_safety_margin_m",
]

QUALITY_COLUMNS = [
    "method",
    "scenario",
    "run_id",
    "success",
    "travel_time_s",
    "traj_length_m",
    "avg_vel_ms",
    "max_vel_ms",
    "p95_vel_ms",
    "p99_vel_ms",
    "avg_acc_ms2",
    "max_acc_ms2",
    "p95_acc_ms2",
    "p99_acc_ms2",
    "jerk_integral",
    "smoothness_cost",
    "min_z_m",
    "max_z_m",
    "low_altitude_sample_count",
    "odom_samples",
]

INT_COLUMNS = [
    "run_id",
    "success",
    "collision_count",
    "collision_sample_count",
    "below_margin_sample_count",
    "cloud_samples",
    "static_collision_count",
    "static_collision_sample_count",
    "static_below_margin_sample_count",
    "static_samples",
    "dynamic_collision_count",
    "dynamic_collision_sample_count",
    "dynamic_below_margin_sample_count",
    "dynamic_samples",
]

NONNEG_COLUMNS = [
    "travel_time_s",
    "traj_length_m",
    "collision_count",
    "collision_sample_count",
    "time_below_margin_s",
    "below_margin_sample_count",
    "cloud_samples",
    "static_collision_count",
    "static_collision_sample_count",
    "static_time_below_margin_s",
    "static_below_margin_sample_count",
    "static_samples",
    "dynamic_collision_count",
    "dynamic_collision_sample_count",
    "dynamic_time_below_margin_s",
    "dynamic_below_margin_sample_count",
    "dynamic_samples",
    "avg_vel_ms",
    "max_vel_ms",
    "avg_acc_ms2",
    "max_acc_ms2",
    "jerk_integral",
    "smoothness_cost",
    "collision_threshold_m",
    "safety_margin_m",
    "dynamic_safety_margin_m",
]

DEFAULT_MIN_SUCCESS_TRAJ_LENGTH = {
    "env2": 60.0,
    "env4": 60.0,
    "env5": 40.0,
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
    return row.get("method") == "method" and row.get("scenario") == "scenario"


def load_quality_rows(path, errors):
    quality_path = Path(path)
    if not quality_path.exists():
        errors.append(f"quality CSV does not exist: {quality_path}")
        return {}

    with quality_path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != QUALITY_COLUMNS:
            errors.append(
                "quality CSV schema mismatch.\n"
                f"expected: {QUALITY_COLUMNS}\n"
                f"actual:   {reader.fieldnames}"
            )
            return {}

        rows_by_key = {}
        for row in reader:
            if row_is_repeated_header(row):
                continue
            key = (row.get("method"), row.get("scenario"), row.get("run_id"))
            rows_by_key[key] = row
        return rows_by_key


def validate(path, dynamic_scenarios, args):
    errors = []
    warnings = []
    rows_checked = 0
    quality_rows = {}
    if args.quality_csv is not None:
        quality_rows = load_quality_rows(args.quality_csv, errors)

    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames != EXPECTED_COLUMNS:
            errors.append(
                "CSV schema mismatch.\n"
                f"expected: {EXPECTED_COLUMNS}\n"
                f"actual:   {reader.fieldnames}"
            )
            return rows_checked, errors, warnings

        all_rows = list(reader)
        if args.method is not None:
            all_rows = [row for row in all_rows if row.get("method") == args.method]
        if args.scenario is not None:
            all_rows = [row for row in all_rows if row.get("scenario") == args.scenario]
        if args.run_id is not None:
            all_rows = [row for row in all_rows if row.get("run_id") == str(args.run_id)]
        if args.last is not None:
            all_rows = all_rows[-args.last:]
        if not all_rows:
            errors.append("no rows matched the requested filters")
            return rows_checked, errors, warnings

        for offset, row in enumerate(all_rows):
            row_num = offset + 2
            if row_is_repeated_header(row):
                errors.append(f"row {row_num}: repeated header row inside CSV")
                continue
            rows_checked += 1

            for col in INT_COLUMNS:
                parse_int(row, col, row_num, errors)

            success = parse_int(row, "success", row_num, errors)
            if success not in (0, 1):
                errors.append(f"row {row_num}: success must be 0 or 1, got {success}")

            values = {col: parse_float(row, col, row_num, errors) for col in NONNEG_COLUMNS}
            for col, value in values.items():
                if math.isfinite(value) and value < 0:
                    errors.append(f"row {row_num}: {col} must be non-negative, got {value}")

            cloud_samples = parse_int(row, "cloud_samples", row_num, errors)
            min_obs = parse_float(row, "min_obs_dist_m", row_num, errors)
            static_samples = parse_int(row, "static_samples", row_num, errors)
            min_static = parse_float(row, "min_static_clearance_m", row_num, errors)
            if cloud_samples > 0 and min_obs < 0:
                errors.append(
                    f"row {row_num}: min_obs_dist_m is {min_obs} but cloud_samples={cloud_samples}"
                )
            if cloud_samples == 0 and min_obs != -1.0:
                warnings.append(
                    f"row {row_num}: cloud_samples=0 should normally use min_obs_dist_m=-1"
                )
            if static_samples != cloud_samples:
                errors.append(
                    f"row {row_num}: static_samples={static_samples} should match "
                    f"cloud_samples={cloud_samples}"
                )
            if abs(min_static - min_obs) > 1e-6:
                errors.append(
                    f"row {row_num}: min_static_clearance_m={min_static} should match "
                    f"min_obs_dist_m={min_obs} for the static point-cloud evaluator"
                )

            dynamic_samples = parse_int(row, "dynamic_samples", row_num, errors)
            min_dynamic = parse_float(row, "min_dynamic_clearance_m", row_num, errors)
            if dynamic_samples > 0 and min_dynamic == -1.0:
                errors.append(
                    f"row {row_num}: dynamic samples exist but min_dynamic_clearance_m is sentinel -1"
                )
            if dynamic_samples == 0 and min_dynamic != -1.0:
                warnings.append(
                    f"row {row_num}: dynamic_samples=0 should normally use min_dynamic_clearance_m=-1"
                )

            scenario = row.get("scenario", "")
            if scenario in dynamic_scenarios and dynamic_samples == 0:
                errors.append(
                    f"row {row_num}: scenario={scenario} expected dynamic samples but got 0"
                )

            collision_count = parse_int(row, "collision_count", row_num, errors)
            collision_samples = parse_int(row, "collision_sample_count", row_num, errors)
            static_collision_count = parse_int(row, "static_collision_count", row_num, errors)
            static_collision_samples = parse_int(
                row, "static_collision_sample_count", row_num, errors
            )
            if collision_count > collision_samples:
                errors.append(
                    f"row {row_num}: collision_count={collision_count} "
                    f"> collision_sample_count={collision_samples}"
                )
            if static_collision_count != collision_count:
                errors.append(
                    f"row {row_num}: static_collision_count={static_collision_count} "
                    f"should match collision_count={collision_count}"
                )
            if static_collision_samples != collision_samples:
                errors.append(
                    f"row {row_num}: static_collision_sample_count={static_collision_samples} "
                    f"should match collision_sample_count={collision_samples}"
                )

            dynamic_collision_count = parse_int(row, "dynamic_collision_count", row_num, errors)
            dynamic_collision_samples = parse_int(
                row, "dynamic_collision_sample_count", row_num, errors
            )
            if dynamic_collision_count > dynamic_collision_samples:
                errors.append(
                    f"row {row_num}: dynamic_collision_count={dynamic_collision_count} "
                    f"> dynamic_collision_sample_count={dynamic_collision_samples}"
                )

            if args.require_success and success != 1:
                errors.append(f"row {row_num}: success required but success={success}")
            min_success_traj_length = DEFAULT_MIN_SUCCESS_TRAJ_LENGTH.get(scenario, 0.0)
            if args.min_traj_length is not None:
                min_success_traj_length = max(min_success_traj_length, args.min_traj_length)
            traj_length = parse_float(row, "traj_length_m", row_num, errors)
            if success == 1 and traj_length < min_success_traj_length:
                errors.append(
                    f"row {row_num}: success row traj_length_m={traj_length:.4f} "
                    f"< required {min_success_traj_length:.4f} for scenario={scenario}"
                )
            if args.require_zero_collision and collision_count != 0:
                errors.append(
                    f"row {row_num}: zero combined collision required but collision_count={collision_count}"
                )
            if args.require_zero_dynamic_collision and dynamic_collision_count != 0:
                errors.append(
                    f"row {row_num}: zero dynamic collision required but "
                    f"dynamic_collision_count={dynamic_collision_count}"
                )
            if args.min_dynamic_clearance is not None and dynamic_samples > 0:
                if min_dynamic < args.min_dynamic_clearance:
                    errors.append(
                        f"row {row_num}: min_dynamic_clearance_m={min_dynamic:.4f} "
                        f"< required {args.min_dynamic_clearance:.4f}"
                    )
            if args.max_dynamic_below_margin is not None:
                dynamic_below_time = parse_float(
                    row, "dynamic_time_below_margin_s", row_num, errors
                )
                if dynamic_below_time > args.max_dynamic_below_margin:
                    errors.append(
                        f"row {row_num}: dynamic_time_below_margin_s={dynamic_below_time:.4f} "
                        f"> allowed {args.max_dynamic_below_margin:.4f}"
                    )

            if args.max_vel is not None:
                max_vel = parse_float(row, "max_vel_ms", row_num, errors)
                if max_vel > args.max_vel:
                    errors.append(
                        f"row {row_num}: max_vel_ms={max_vel:.4f} > allowed {args.max_vel:.4f}"
                    )
            if args.max_acc is not None:
                max_acc = parse_float(row, "max_acc_ms2", row_num, errors)
                if max_acc > args.max_acc:
                    errors.append(
                        f"row {row_num}: max_acc_ms2={max_acc:.4f} > allowed {args.max_acc:.4f}"
                    )
            if args.max_jerk_integral is not None:
                jerk_integral = parse_float(row, "jerk_integral", row_num, errors)
                if jerk_integral > args.max_jerk_integral:
                    errors.append(
                        f"row {row_num}: jerk_integral={jerk_integral:.4f} "
                        f"> allowed {args.max_jerk_integral:.4f}"
                    )

            if args.quality_csv is not None:
                key = (row.get("method"), row.get("scenario"), row.get("run_id"))
                quality_row = quality_rows.get(key)
                if quality_row is None:
                    errors.append(f"row {row_num}: no matching quality row for {key}")
                else:
                    if args.max_p99_vel is not None:
                        p99_vel = parse_float(quality_row, "p99_vel_ms", row_num, errors)
                        if p99_vel > args.max_p99_vel:
                            errors.append(
                                f"row {row_num}: p99_vel_ms={p99_vel:.4f} "
                                f"> allowed {args.max_p99_vel:.4f}"
                            )
                    if args.max_p99_acc is not None:
                        p99_acc = parse_float(quality_row, "p99_acc_ms2", row_num, errors)
                        if p99_acc > args.max_p99_acc:
                            errors.append(
                                f"row {row_num}: p99_acc_ms2={p99_acc:.4f} "
                                f"> allowed {args.max_p99_acc:.4f}"
                            )
                    if args.max_low_altitude_samples is not None:
                        low_altitude_samples = parse_int(
                            quality_row, "low_altitude_sample_count", row_num, errors
                        )
                        if low_altitude_samples > args.max_low_altitude_samples:
                            errors.append(
                                f"row {row_num}: low_altitude_sample_count="
                                f"{low_altitude_samples} > allowed "
                                f"{args.max_low_altitude_samples}"
                            )

    return rows_checked, errors, warnings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="src/benchmark/results/benchmark_results_extended.csv",
        help="Path to benchmark_results_extended.csv",
    )
    parser.add_argument(
        "--dynamic-scenarios",
        default="env3,env4",
        help="Comma-separated scenario names that must have dynamic samples",
    )
    parser.add_argument(
        "--require-success",
        action="store_true",
        help="Require all rows to be successful runs",
    )
    parser.add_argument(
        "--require-zero-collision",
        action="store_true",
        help="Require combined cloud-based collision_count to be zero",
    )
    parser.add_argument(
        "--require-zero-dynamic-collision",
        action="store_true",
        help="Require dynamic_collision_count to be zero",
    )
    parser.add_argument(
        "--min-dynamic-clearance",
        type=float,
        default=None,
        help="Require min_dynamic_clearance_m to be at least this value when dynamic samples exist",
    )
    parser.add_argument(
        "--max-dynamic-below-margin",
        type=float,
        default=None,
        help="Require dynamic_time_below_margin_s to be no larger than this value",
    )
    parser.add_argument(
        "--last",
        type=int,
        default=None,
        help="Validate only the last N data rows",
    )
    parser.add_argument(
        "--method",
        default=None,
        help="Validate only rows with this method name",
    )
    parser.add_argument(
        "--scenario",
        default=None,
        help="Validate only rows with this scenario name",
    )
    parser.add_argument(
        "--run-id",
        type=int,
        default=None,
        help="Validate only rows with this integer run_id",
    )
    parser.add_argument(
        "--max-vel",
        type=float,
        default=None,
        help="Require max_vel_ms to be no larger than this value",
    )
    parser.add_argument(
        "--max-acc",
        type=float,
        default=None,
        help="Require max_acc_ms2 to be no larger than this value",
    )
    parser.add_argument(
        "--max-jerk-integral",
        type=float,
        default=None,
        help="Require jerk_integral to be no larger than this value",
    )
    parser.add_argument(
        "--min-traj-length",
        type=float,
        default=None,
        help="Require successful rows to have at least this trajectory length in meters",
    )
    parser.add_argument(
        "--quality-csv",
        default=None,
        help="Optional benchmark_quality_metrics.csv for robust p95/p99 checks",
    )
    parser.add_argument(
        "--max-p99-vel",
        type=float,
        default=None,
        help="Require quality p99_vel_ms to be no larger than this value",
    )
    parser.add_argument(
        "--max-p99-acc",
        type=float,
        default=None,
        help="Require quality p99_acc_ms2 to be no larger than this value",
    )
    parser.add_argument(
        "--max-low-altitude-samples",
        type=int,
        default=None,
        help="Require low_altitude_sample_count to be no larger than this value",
    )
    args = parser.parse_args()

    path = Path(args.csv_path)
    if not path.exists():
        print(f"ERROR: CSV does not exist: {path}", file=sys.stderr)
        return 2

    dynamic_scenarios = {
        item.strip() for item in args.dynamic_scenarios.split(",") if item.strip()
    }
    rows_checked, errors, warnings = validate(path, dynamic_scenarios, args)

    for warning in warnings:
        print(f"WARNING: {warning}")
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        print(f"FAILED: checked {rows_checked} rows, {len(errors)} errors", file=sys.stderr)
        return 1

    print(f"OK: checked {rows_checked} rows in {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
