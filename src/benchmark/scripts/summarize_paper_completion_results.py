#!/usr/bin/env python3
import argparse
import csv
import math
import re
from collections import defaultdict
from pathlib import Path


RESULT_RE = re.compile(
    r"^(?P<exp>.+?) \| (?P<status>SUCCESS|TIMEOUT|NO_ODOM|FAILED) \| "
    r"(?P<total_time>\d+)s .*? goal_dist=(?P<goal_dist>-?\d+(?:\.\d+)?) "
    r"\| replans=(?P<replans>\d+) \| col=(?P<col>\d+) "
    r"\| topo_fail=(?P<topo_fail>\d+) \| bs_fail=(?P<bs_fail>\d+)"
)


def mean(values):
    values = [v for v in values if v is not None and not math.isnan(v)]
    return sum(values) / len(values) if values else float("nan")


def std(values):
    values = [v for v in values if v is not None and not math.isnan(v)]
    if len(values) <= 1:
        return 0.0 if values else float("nan")
    m = mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def fmt(value, digits=2):
    if value is None or math.isnan(value):
        return "NA"
    return f"{value:.{digits}f}"


def read_result_rows(path):
    rows = []
    for line in Path(path).read_text(errors="replace").splitlines():
        m = RESULT_RE.search(line)
        if not m:
            continue
        d = m.groupdict()
        rows.append(
            {
                "exp_name": d["exp"],
                "status": d["status"],
                "total_time_s": float(d["total_time"]),
                "goal_dist": float(d["goal_dist"]),
                "replans": float(d["replans"]),
                "colwarn": float(d["col"]),
                "topo_fail": float(d["topo_fail"]),
                "bs_fail": float(d["bs_fail"]),
            }
        )
    return rows


def read_extended_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def f(row, key):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def valid_extended(row):
    if not row:
        return False
    if f(row, "cloud_samples") <= 0:
        return False
    if f(row, "travel_time_s") < 1.0:
        return False
    return True


def infer_case(exp_name, scenario, mode):
    if mode == "main":
        return scenario
    if "env3_hs" in exp_name:
        return "env3_high_speed"
    if "env4_dynstress" in exp_name:
        return "env4_dynamic_stress"
    for suffix in (
        "env5_no_topo",
        "env3_no_topo",
        "env4_no_mppi",
        "env3_no_mppi",
        "env2_single_topo",
        "env2_no_static_gate",
        "env4_dynamic_mode_score",
        "env4_no_bspline_fallback",
    ):
        if suffix in exp_name:
            return suffix
    return scenario


def join_snapshot(results_path, extended_path, mode, source_tag):
    results = read_result_rows(results_path)
    extended = read_extended_rows(extended_path)
    if len(results) != len(extended):
        raise RuntimeError(
            f"row count mismatch: {results_path} has {len(results)} result rows, "
            f"{extended_path} has {len(extended)} extended rows"
        )
    rows = []
    invalid = []
    for r, e in zip(results, extended):
        row = {**r}
        row["source_tag"] = source_tag
        row["method"] = e["method"]
        row["scenario"] = e["scenario"]
        row["case"] = infer_case(r["exp_name"], e["scenario"], mode)
        row["success"] = int(float(e["success"]))
        row["extended_valid"] = valid_extended(e)
        for key, value in e.items():
            if key not in ("method", "scenario"):
                row[key] = value
        if not row["extended_valid"]:
            invalid.append(row)
        rows.append(row)
    return rows, invalid


def summarize(rows, group_keys):
    groups = defaultdict(list)
    for row in rows:
        groups[tuple(row[k] for k in group_keys)].append(row)

    out = []
    for key, items in sorted(groups.items()):
        valid = [r for r in items if r["extended_valid"]]
        success = [r for r in valid if int(float(r["success"])) == 1 and r["status"] == "SUCCESS"]
        base = {k: v for k, v in zip(group_keys, key)}
        base["runs"] = len(valid)
        base["raw_runs"] = len(items)
        base["successes"] = len(success)
        base["invalid_rows"] = len(items) - len(valid)
        for metric, csv_key in (
            ("time_s", "travel_time_s"),
            ("path_m", "traj_length_m"),
            ("min_clearance_m", "min_obs_dist_m"),
            ("static_collision", "static_collision_count"),
            ("dynamic_collision", "dynamic_collision_count"),
            ("time_below_margin_s", "time_below_margin_s"),
            ("dynamic_below_margin_s", "dynamic_time_below_margin_s"),
            ("avg_vel", "avg_vel_ms"),
            ("max_vel", "max_vel_ms"),
            ("max_acc", "max_acc_ms2"),
        ):
            values = [f(r, csv_key) for r in success]
            base[f"{metric}_mean"] = mean(values)
            base[f"{metric}_std"] = std(values)
        for metric, key2 in (
            ("colwarn", "colwarn"),
            ("replans", "replans"),
            ("topo_fail", "topo_fail"),
            ("bs_fail", "bs_fail"),
        ):
            values = [float(r[key2]) for r in valid]
            base[f"{metric}_mean"] = mean(values)
            base[f"{metric}_std"] = std(values)
        out.append(base)
    return out


def write_csv(path, rows, columns):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns)
        w.writeheader()
        for row in rows:
            w.writerow({c: row.get(c, "") for c in columns})


def md_table(rows, columns, labels):
    lines = []
    lines.append("| " + " | ".join(labels) + " |")
    lines.append("| " + " | ".join(["---"] * len(columns)) + " |")
    for row in rows:
        vals = []
        for col in columns:
            val = row.get(col, "")
            if isinstance(val, float):
                vals.append(fmt(val))
            else:
                vals.append(str(val))
        lines.append("| " + " | ".join(vals) + " |")
    return "\n".join(lines)


def decorate_summary(rows):
    decorated = []
    for r in rows:
        d = dict(r)
        d["success"] = f"{r['successes']}/{r['runs']}"
        d["time"] = f"{fmt(r['time_s_mean'])} +/- {fmt(r['time_s_std'])}"
        d["colwarn"] = f"{fmt(r['colwarn_mean'])} +/- {fmt(r['colwarn_std'])}"
        d["replans"] = f"{fmt(r['replans_mean'])} +/- {fmt(r['replans_std'])}"
        d["static_collision"] = f"{fmt(r['static_collision_mean'])} +/- {fmt(r['static_collision_std'])}"
        d["dynamic_collision"] = f"{fmt(r['dynamic_collision_mean'])} +/- {fmt(r['dynamic_collision_std'])}"
        d["clearance"] = f"{fmt(r['min_clearance_m_mean'], 3)} +/- {fmt(r['min_clearance_m_std'], 3)}"
        d["diag"] = f"{fmt(r['topo_fail_mean'])} / {fmt(r['bs_fail_mean'])}"
        decorated.append(d)
    return decorated


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output-dir", default="src/benchmark/results")
    args = p.parse_args()
    out_dir = Path(args.output_dir)

    main_specs = [
        (
            "final5_20260626_135440",
            out_dir / "final5_20260626_135440_results_master_tail.txt",
            out_dir / "final5_20260626_135440_benchmark_results_extended.csv",
        ),
        (
            "papercomplete_main5_20260627_103027",
            out_dir / "papercomplete_main5_20260627_103027_results_master_tail.txt",
            out_dir / "papercomplete_main5_20260627_103027_benchmark_results_extended.csv",
        ),
    ]
    stress_specs = [
        (
            "papercomplete_stress3_20260627_121213",
            out_dir / "papercomplete_stress3_20260627_121213_results_master_tail.txt",
            out_dir / "papercomplete_stress3_20260627_121213_benchmark_results_extended.csv",
        )
    ]
    ablation_specs = [
        (
            "papercomplete_ablation3_20260627_123503",
            out_dir / "papercomplete_ablation3_20260627_123503_results_master_tail.txt",
            out_dir / "papercomplete_ablation3_20260627_123503_benchmark_results_extended.csv",
        )
    ]

    invalid = []
    main_rows = []
    for tag, result_path, extended_path in main_specs:
        rows, bad = join_snapshot(result_path, extended_path, "main", tag)
        main_rows.extend(rows)
        invalid.extend(bad)
    stress_rows = []
    for tag, result_path, extended_path in stress_specs:
        rows, bad = join_snapshot(result_path, extended_path, "stress", tag)
        stress_rows.extend(rows)
        invalid.extend(bad)
    ablation_rows = []
    for tag, result_path, extended_path in ablation_specs:
        rows, bad = join_snapshot(result_path, extended_path, "ablation", tag)
        ablation_rows.extend(rows)
        invalid.extend(bad)

    main_summary = summarize(main_rows, ["case", "method"])
    stress_summary = summarize(stress_rows, ["case", "method"])
    ablation_summary = summarize(ablation_rows, ["case"])

    columns = [
        "case",
        "method",
        "runs",
        "raw_runs",
        "successes",
        "invalid_rows",
        "time_s_mean",
        "time_s_std",
        "path_m_mean",
        "min_clearance_m_mean",
        "static_collision_mean",
        "dynamic_collision_mean",
        "colwarn_mean",
        "replans_mean",
        "topo_fail_mean",
        "bs_fail_mean",
    ]
    write_csv(out_dir / "paper_main10_summary_20260627.csv", main_summary, columns)
    write_csv(out_dir / "paper_stress3_summary_20260627.csv", stress_summary, columns)
    write_csv(
        out_dir / "paper_ablation3_summary_20260627.csv",
        ablation_summary,
        [c for c in columns if c != "method"],
    )
    write_csv(
        out_dir / "paper_invalid_rows_20260627.csv",
        invalid,
        [
            "source_tag",
            "exp_name",
            "status",
            "method",
            "scenario",
            "case",
            "success",
            "travel_time_s",
            "cloud_samples",
            "goal_dist",
        ],
    )

    md = []
    md.append("# Paper Completion Results 2026-06-27")
    md.append("")
    md.append("This file is generated from completed benchmark snapshots. It supersedes older paper-facing tables for the experiment batches listed below.")
    md.append("")
    md.append("## Source Batches")
    md.append("")
    for tag, _, _ in main_specs + stress_specs + ablation_specs:
        md.append(f"- `{tag}`")
    md.append("")
    md.append("## Validity Filter")
    md.append("")
    md.append("- Rows are joined by snapshot order from `*_results_master_tail.txt` and `*_benchmark_results_extended.csv`.")
    md.append("- Extended rows are considered valid only when `cloud_samples > 0` and `travel_time_s >= 1.0`.")
    md.append("- The filtered invalid rows are written to `src/benchmark/results/paper_invalid_rows_20260627.csv`.")
    md.append("- Success counts use valid rows only and require both benchmark success and `SUCCESS` status in the log summary.")
    md.append("")
    md.append("## 10-Run Main Comparison")
    md.append("")
    md.append(md_table(
        decorate_summary(main_summary),
        ["case", "method", "success", "time", "clearance", "static_collision", "dynamic_collision", "colwarn", "replans", "diag", "invalid_rows"],
        ["Env", "Method", "Success", "Time mean +/- std", "Min clearance", "Static collisions", "Dynamic collisions", "ColWarn", "Replans", "TopoFail / BsFail", "Invalid rows"],
    ))
    md.append("")
    md.append("## 3-Run Stress Tests")
    md.append("")
    md.append(md_table(
        decorate_summary(stress_summary),
        ["case", "method", "success", "time", "clearance", "static_collision", "dynamic_collision", "colwarn", "replans", "diag", "invalid_rows"],
        ["Case", "Method", "Success", "Time mean +/- std", "Min clearance", "Static collisions", "Dynamic collisions", "ColWarn", "Replans", "TopoFail / BsFail", "Invalid rows"],
    ))
    md.append("")
    md.append("## 3-Run Ablations")
    md.append("")
    md.append(md_table(
        decorate_summary(ablation_summary),
        ["case", "success", "time", "clearance", "static_collision", "dynamic_collision", "colwarn", "replans", "diag", "invalid_rows"],
        ["Case", "Success", "Time mean +/- std", "Min clearance", "Static collisions", "Dynamic collisions", "ColWarn", "Replans", "TopoFail / BsFail", "Invalid rows"],
    ))
    md.append("")
    md.append("## Current Reading")
    md.append("")
    md.append("- Topo-MPPI is strongest in the normal Env4 dynamic benchmark: it has higher success than EGO and much lower `ColWarn` and replanning pressure. The Env4 stress test is a limitation case because Topo-MPPI only reached `1/3` strict benchmark success, even though its successful run had low warning pressure.")
    md.append("- Topo-MPPI is not currently faster than EGO in Env2/Env5; those environments remain the main runtime and robustness limitations.")
    md.append("- Multi-topology and static safety gates matter in Env2: single-topo and no-static-gate ablations each produced one timeout in three runs.")
    md.append("- Removing MPPI did not destroy success rate in the tested Env3/Env4 ablations, but it increased B-spline failures and warning pressure, so the defensible claim is improved dynamic/local quality rather than absolute necessity for mission success.")
    md.append("- No-Topo ablations succeeded in this latest 3-run batch, so the claim should be phrased as topology guidance improving efficiency/robustness in complex scenes, not as a universal hard requirement.")
    md.append("- Stress Env4 exposes a strict route-validation issue for Topo-MPPI: two runs reached the goal region but were marked timeout by benchmark validation. This should be reported as a failure under the strict benchmark rule.")
    md.append("")
    (out_dir / "paper_completion_results_20260627.md").write_text("\n".join(md) + "\n")


if __name__ == "__main__":
    main()
