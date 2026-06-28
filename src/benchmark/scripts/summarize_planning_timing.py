#!/usr/bin/env python3
"""Summarize Topo-MPPI per-cycle planning timing CSV files."""

import argparse
import csv
import statistics
from collections import Counter
from pathlib import Path


TIME_COLUMNS = [
    "total_ms",
    "init_ms",
    "topo_ms",
    "mppi_ms",
    "bspline_ms",
    "refine_ms",
    "validator_ms",
]


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * q))
    return ordered[max(0, min(len(ordered) - 1, idx))]


def to_float(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return 0.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", help="Path to *_planning_timing.csv")
    parser.add_argument(
        "--last",
        type=int,
        default=None,
        help="Summarize only the last N planning cycles",
    )
    args = parser.parse_args()

    path = Path(args.csv_path)
    rows = list(csv.DictReader(path.open()))
    if args.last is not None:
        rows = rows[-args.last :]

    print(f"file: {path}")
    print(f"cycles: {len(rows)}")
    if not rows:
        return 0

    for col in TIME_COLUMNS:
        values = [to_float(row, col) for row in rows]
        print(
            f"{col}: mean={statistics.mean(values):.2f} "
            f"p50={statistics.median(values):.2f} "
            f"p95={percentile(values, 0.95):.2f} "
            f"max={max(values):.2f}"
        )

    print("success:", dict(Counter(row.get("success", "") for row in rows)))
    print(
        "fail_reason:",
        dict(Counter(row.get("fail_reason", "") for row in rows).most_common()),
    )
    for col in ["topo_paths", "mppi_candidates", "mppi_successes"]:
        values = [to_float(row, col) for row in rows]
        print(f"{col}: mean={statistics.mean(values):.2f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
