#!/usr/bin/env python3
"""Score benchmark rows with the temporary Topo-MPPI engineering score.

This is not a paper metric. It is an internal tuning helper for comparing
successful Topo-MPPI variants when true collision and continuous-clearance logs
are not available yet.
"""

import argparse
import re
from pathlib import Path


ROW_RE = re.compile(
    r"^\s*(?P<name>[^|]+?)\s*\|\s*(?P<status>\w+)\s*\|\s*"
    r"(?P<time>\d+(?:\.\d+)?)s\s*\|.*?"
    r"goal_dist=(?P<goal>[-+]?\d+(?:\.\d+)?)\s*\|\s*"
    r"replans=(?P<replans>\d+)\s*\|\s*"
    r"col=(?P<col>\d+)\s*\|\s*"
    r"topo_fail=(?P<topo>\d+)\s*\|\s*"
    r"bs_fail=(?P<bs>\d+)"
)


def score(row):
    return (
        1.00 * row["time"]
        + 0.03 * row["replans"]
        + 0.08 * row["bs"]
        + 0.03 * row["topo"]
        + 0.50 * row["col"]
        + 20.0 * max(0.0, row["goal"] - 0.15)
    )


def parse_rows(path):
    rows = []
    for line_no, line in enumerate(path.read_text(errors="replace").splitlines(), 1):
        match = ROW_RE.search(line)
        if not match:
            continue
        data = match.groupdict()
        if data["status"] != "SUCCESS":
            continue
        row = {
            "line": line_no,
            "name": data["name"].strip(),
            "time": float(data["time"]),
            "goal": float(data["goal"]),
            "replans": int(data["replans"]),
            "col": int(data["col"]),
            "topo": int(data["topo"]),
            "bs": int(data["bs"]),
        }
        row["score"] = score(row)
        rows.append(row)
    return rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "results",
        nargs="?",
        default="src/benchmark/results/results_master.txt",
        help="Path to results_master.txt",
    )
    parser.add_argument("--grep", default="", help="Only include rows whose name contains this text")
    parser.add_argument("--last", type=int, default=30, help="Score only the last N matching rows")
    args = parser.parse_args()

    path = Path(args.results)
    rows = parse_rows(path)
    if args.grep:
        rows = [row for row in rows if args.grep in row["name"]]
    if args.last > 0:
        rows = rows[-args.last :]

    rows = sorted(rows, key=lambda row: row["score"])
    print(
        "score,time,replans,col,topo_fail,bs_fail,goal_dist,line,name"
    )
    for row in rows:
        print(
            f"{row['score']:.2f},{row['time']:.1f},{row['replans']},"
            f"{row['col']},{row['topo']},{row['bs']},{row['goal']:.3f},"
            f"{row['line']},{row['name']}"
        )


if __name__ == "__main__":
    main()
