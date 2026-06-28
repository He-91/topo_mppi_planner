#!/usr/bin/env python3
"""Aggregate paper-batch Topo-MPPI timing and candidate diagnostics.

This script does not rerun simulations. It summarizes existing
*_planning_timing.csv and *_candidate_quality.csv files for the current paper
batches into paper-facing CSV and Markdown tables.
"""

import csv
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


RESULTS_DIR = Path("src/benchmark/results")
OUT_TIMING = RESULTS_DIR / "paper_planning_timing_summary_20260627.csv"
OUT_CANDIDATE = RESULTS_DIR / "paper_candidate_quality_summary_20260627.csv"
OUT_MD = RESULTS_DIR / "paper_runtime_candidate_summary_20260627.md"

BATCHES = [
    "final5_20260626_135440",
    "papercomplete_main5_20260627_103027",
    "papercomplete_stress3_20260627_121213",
    "papercomplete_ablation3_20260627_123503",
]

TIMING_COLS = [
    "total_ms",
    "topo_ms",
    "mppi_ms",
    "bspline_ms",
    "refine_ms",
    "validator_ms",
]

CANDIDATE_NUMERIC_COLS = [
    "raw_cost",
    "normalized_cost",
    "selection_score",
    "path_length",
    "final_goal_dist",
    "progress_ratio",
    "min_static_clearance",
    "min_dynamic_clearance",
]


def percentile(values, q):
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = int(round((len(ordered) - 1) * q))
    return ordered[max(0, min(len(ordered) - 1, idx))]


def to_float(row, key):
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value) or abs(value) > 1e9:
        return None
    return value


def read_rows(path):
    try:
        with path.open(newline="") as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return []


def parse_case(path, suffix):
    name = path.name
    if not name.startswith("topo_mppi_") or not name.endswith(suffix):
        return None
    stem = name[: -len(suffix)]
    body = stem[len("topo_mppi_") :]
    for batch in BATCHES:
        prefix = batch + "_"
        if body.startswith(prefix):
            rest = body[len(prefix) :]
            marker = "_r"
            if marker not in rest:
                return None
            case = rest.rsplit(marker, 1)[0]
            return batch, case
    return None


def collect_files(suffix):
    grouped = defaultdict(list)
    for path in sorted(RESULTS_DIR.glob(f"topo_mppi_*{suffix}")):
        parsed = parse_case(path, suffix)
        if parsed is None:
            continue
        batch, case = parsed
        grouped[(batch, case)].append(path)
    return grouped


def summarize_timing():
    grouped_files = collect_files("_planning_timing.csv")
    rows_out = []
    for (batch, case), paths in sorted(grouped_files.items()):
        rows = []
        for path in paths:
            rows.extend(read_rows(path))

        out = {
            "batch": batch,
            "case": case,
            "files": len(paths),
            "cycles": len(rows),
            "success_cycle_rate": (
                sum(r.get("success") == "1" for r in rows) / len(rows) if rows else 0.0
            ),
            "fail_reasons": ";".join(
                f"{k}:{v}"
                for k, v in Counter(r.get("fail_reason", "") for r in rows).most_common(5)
                if k
            ),
        }
        for col in TIMING_COLS:
            values = [to_float(r, col) for r in rows]
            values = [v for v in values if v is not None]
            out[f"{col}_mean"] = statistics.mean(values) if values else 0.0
            out[f"{col}_p50"] = statistics.median(values) if values else 0.0
            out[f"{col}_p95"] = percentile(values, 0.95)
            out[f"{col}_max"] = max(values) if values else 0.0
        for col in ["topo_paths", "mppi_candidates", "mppi_successes"]:
            values = [to_float(r, col) for r in rows]
            values = [v for v in values if v is not None]
            out[f"{col}_mean"] = statistics.mean(values) if values else 0.0
        rows_out.append(out)
    return rows_out


def summarize_candidate():
    grouped_files = collect_files("_candidate_quality.csv")
    rows_out = []
    for (batch, case), paths in sorted(grouped_files.items()):
        rows = []
        for path in paths:
            rows.extend(read_rows(path))

        selected = [r for r in rows if r.get("selected") == "1"]
        successful = [r for r in rows if r.get("success") == "1"]
        cycles = {r.get("cycle", "") for r in rows if r.get("cycle", "")}
        out = {
            "batch": batch,
            "case": case,
            "files": len(paths),
            "rows": len(rows),
            "cycles": len(cycles),
            "selected_rows": len(selected),
            "success_rows": len(successful),
            "success_row_rate": len(successful) / len(rows) if rows else 0.0,
            "top_reject_reasons": ";".join(
                f"{k}:{v}"
                for k, v in Counter(r.get("reject_reason", "") for r in rows).most_common(5)
                if k
            ),
        }
        for prefix, subset in [("all", rows), ("selected", selected), ("successful", successful)]:
            for col in CANDIDATE_NUMERIC_COLS:
                values = [to_float(r, col) for r in subset]
                values = [v for v in values if v is not None]
                out[f"{prefix}_{col}_mean"] = statistics.mean(values) if values else 0.0
                out[f"{prefix}_{col}_p50"] = statistics.median(values) if values else 0.0
                out[f"{prefix}_{col}_p95"] = percentile(values, 0.95)
        rows_out.append(out)
    return rows_out


def write_csv(path, rows):
    if not rows:
        path.write_text("")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def fmt(value, digits=2):
    if isinstance(value, str):
        return value
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return str(value)


def write_md(timing_rows, candidate_rows):
    lines = [
        "# Paper Runtime And Candidate Diagnostics 2026-06-27",
        "",
        "This file is generated from existing Topo-MPPI diagnostic CSV files. It does not add new simulation runs.",
        "",
        "## Source Batches",
        "",
    ]
    lines.extend(f"- `{batch}`" for batch in BATCHES)
    lines.extend(
        [
            "",
            "## Planning Runtime Summary",
            "",
            "| Batch | Case | Files | Cycles | Total mean/p95/max ms | MPPI mean ms | B-spline mean ms | Validator mean ms | Candidate mean | Success cycle rate |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for row in timing_rows:
        lines.append(
            "| {batch} | {case} | {files} | {cycles} | {mean}/{p95}/{maxv} | {mppi} | {bspline} | {validator} | {cand} | {succ} |".format(
                batch=row["batch"],
                case=row["case"],
                files=row["files"],
                cycles=row["cycles"],
                mean=fmt(row["total_ms_mean"]),
                p95=fmt(row["total_ms_p95"]),
                maxv=fmt(row["total_ms_max"]),
                mppi=fmt(row["mppi_ms_mean"]),
                bspline=fmt(row["bspline_ms_mean"]),
                validator=fmt(row["validator_ms_mean"]),
                cand=fmt(row["mppi_candidates_mean"]),
                succ=fmt(row["success_cycle_rate"], 3),
            )
        )

    lines.extend(
        [
            "",
            "## Candidate Selection Summary",
            "",
            "| Batch | Case | Files | Candidate rows | Cycles | Selected rows | Success row rate | Selected static clearance mean | Selected dynamic clearance mean | Selected path length mean | Top reject reasons |",
            "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |",
        ]
    )
    for row in candidate_rows:
        lines.append(
            "| {batch} | {case} | {files} | {rows} | {cycles} | {selected} | {succ} | {sclr} | {dclr} | {plen} | {reasons} |".format(
                batch=row["batch"],
                case=row["case"],
                files=row["files"],
                rows=row["rows"],
                cycles=row["cycles"],
                selected=row["selected_rows"],
                succ=fmt(row["success_row_rate"], 3),
                sclr=fmt(row["selected_min_static_clearance_mean"], 3),
                dclr=fmt(row["selected_min_dynamic_clearance_mean"], 3),
                plen=fmt(row["selected_path_length_mean"], 2),
                reasons=row["top_reject_reasons"] or "none",
            )
        )

    lines.extend(
        [
            "",
            "## Reading",
            "",
            "- Use the runtime table to support real-time feasibility claims for Topo-MPPI. Report p95/max latency when discussing high-speed replanning.",
            "- Use the candidate table to show that multi-candidate selection is actually active, and to report selected-candidate clearance and path-length statistics.",
            "- Rows with `cycles=0` or `Candidate rows=0` mean the diagnostic CSV files exist but are empty; the zero-valued timing/candidate fields are missing-data placeholders, not measured zero cost.",
            "- These diagnostics are Topo-MPPI-only because EGO does not emit the same per-cycle Topo/MPPI/B-spline breakdown.",
            "",
        ]
    )
    OUT_MD.write_text("\n".join(lines))


def main():
    timing_rows = summarize_timing()
    candidate_rows = summarize_candidate()
    write_csv(OUT_TIMING, timing_rows)
    write_csv(OUT_CANDIDATE, candidate_rows)
    write_md(timing_rows, candidate_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
