#!/usr/bin/env python3
"""Summarize Topo-MPPI per-candidate quality CSV files."""

import argparse
import csv
import math
import statistics
from collections import Counter
from pathlib import Path


NUMERIC_COLUMNS = [
    "raw_cost",
    "normalized_cost",
    "selection_score",
    "dynamic_scene_score",
    "risk_score",
    "static_tail_risk",
    "dynamic_tail_risk",
    "guide_consistency_risk",
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
]


def to_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return 0.0


def valid_metric_values(rows, col):
    values = []
    skipped = 0
    for row in rows:
        value = to_float(row, col)
        # Failed candidates use numeric sentinels such as DBL_MAX for fields
        # that have no geometric meaning. Keep them out of distribution stats.
        if not math.isfinite(value) or abs(value) > 1e9:
            skipped += 1
            continue
        values.append(value)
    return values, skipped


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * q))
    return ordered[max(0, min(len(ordered) - 1, idx))]


def summarize_rows(rows, label):
    print(f"[{label}] rows: {len(rows)}")
    if not rows:
        return
    cycles = {row.get("cycle", "") for row in rows}
    print(f"[{label}] cycles: {len(cycles)}")
    print(f"[{label}] selected rows: {sum(row.get('selected') == '1' for row in rows)}")
    print(f"[{label}] success rows: {sum(row.get('success') == '1' for row in rows)}")
    print(f"[{label}] reject_reason: {dict(Counter(row.get('reject_reason', '') for row in rows).most_common())}")

    for col in NUMERIC_COLUMNS:
        values, skipped = valid_metric_values(rows, col)
        if not values:
            print(f"[{label}] {col}: no valid numeric samples (skipped={skipped})")
            continue
        skipped_note = f" skipped={skipped}" if skipped else ""
        print(
            f"[{label}] {col}: mean={statistics.mean(values):.3f} "
            f"p50={statistics.median(values):.3f} "
            f"p95={percentile(values, 0.95):.3f} "
            f"min={min(values):.3f} max={max(values):.3f}"
            f"{skipped_note}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", help="Path to *_candidate_quality.csv")
    parser.add_argument("--last-cycles", type=int, default=None)
    args = parser.parse_args()

    path = Path(args.csv_path)
    rows = list(csv.DictReader(path.open()))
    if args.last_cycles is not None:
        cycles = []
        seen = set()
        for row in reversed(rows):
            cycle = row.get("cycle", "")
            if cycle not in seen:
                seen.add(cycle)
                cycles.append(cycle)
            if len(cycles) >= args.last_cycles:
                break
        keep = set(cycles)
        rows = [row for row in rows if row.get("cycle", "") in keep]

    print(f"file: {path}")
    summarize_rows(rows, "all")
    summarize_rows([row for row in rows if row.get("success") == "1"], "successful")
    summarize_rows([row for row in rows if row.get("selected") == "1"], "selected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
