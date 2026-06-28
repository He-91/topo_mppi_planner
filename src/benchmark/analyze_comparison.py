#!/usr/bin/env python3
"""Compare Topo-MPPI vs EGO vs Fast-Planner benchmark results."""

import csv
import sys
from collections import defaultdict

CSV_PATH = "/home/he/ros_ws/test/ddo-topo-mppi/src/benchmark/results/benchmark_results.csv"

def load_data(csv_path):
    """Load and organize data by method and scenario."""
    data = defaultdict(lambda: defaultdict(list))
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            method = row['method']
            scenario = row['scenario']
            data[method][scenario].append(row)
    return data

def safe_float(val, default=float('inf')):
    try:
        return float(val)
    except:
        return default

def compute_stats(runs, key, lower_is_better=True):
    """Compute mean/std for a metric across runs."""
    vals = [safe_float(r[key]) for r in runs]
    if not vals:
        return None, None
    mean = sum(vals) / len(vals)
    if len(vals) > 1:
        std = (sum((v - mean)**2 for v in vals) / (len(vals) - 1)) ** 0.5
    else:
        std = 0
    return mean, std

def success_rate(runs):
    successes = sum(1 for r in runs if int(r['success']) == 1)
    return successes / len(runs) * 100 if runs else 0

def main():
    data = load_data(CSV_PATH)
    
    methods = ['ego', 'fast', 'ddo']
    scenarios = ['S1_sparse_static', 'S2_dense_static', 'S3_dynamic', 'S4_3d_vertical']
    
    # Key metrics
    metrics = [
        ('success_rate', 'Success %', None, True),
        ('travel_time_s', 'Travel Time (s)', True, True),
        ('traj_length_m', 'Traj Length (m)', True, True),
        ('collision_count', 'Collisions', True, True),
        ('min_obs_dist_m', 'Min Obs Dist (m)', False, True),
        ('avg_vel_ms', 'Avg Vel (m/s)', False, True),
        ('max_vel_ms', 'Max Vel (m/s)', None, True),
        ('max_acc_ms2', 'Max Accel (m/s²)', True, True),
        ('smoothness_cost', 'Smoothness', True, True),
    ]
    
    print("=" * 100)
    print("  Topo-MPPI vs EGO vs Fast-Planner Benchmark Comparison")
    print("  Bug Fixes Applied: MPPI dynamics sync + Feasibility check for all paths")
    print("=" * 100)
    
    # === PER-SCENARIO ANALYSIS ===
    for scenario in scenarios:
        print(f"\n{'='*80}")
        print(f"  Scenario: {scenario}")
        print(f"{'='*80}")
        
        for method in methods:
            runs = data[method].get(scenario, [])
            if not runs:
                continue
            
            sr = success_rate(runs)
            # Filter successful runs only for most metrics
            succ_runs = [r for r in runs if int(r['success']) == 1]
            
            print(f"\n  [{method.upper()}] ({len(runs)} runs, {sr:.0f}% success)")
            
            if succ_runs:
                for key, label, lower_better, _ in metrics:
                    if key == 'success_rate':
                        continue
                    mean, std = compute_stats(succ_runs, key)
                    if mean is not None:
                        print(f"    {label:25s}: {mean:10.4f} ± {std:10.4f}")
    
    # === OVERALL COMPARISON (across all scenarios, successful runs only) ===
    print(f"\n\n{'='*100}")
    print(f"  OVERALL COMPARISON (all scenarios combined, successful runs only)")
    print(f"{'='*100}")
    
    header = f"  {'Metric':25s}"
    for m in methods:
        header += f" | {m.upper():>25s}"
    header += " | Winner"
    print(header)
    print("  " + "-" * (25 + 3 * 28 + 10))
    
    # Aggregate all successful runs per method
    all_succ = {}
    for method in methods:
        all_runs = []
        for scenario in scenarios:
            all_runs.extend(data[method].get(scenario, []))
        all_succ[method] = [r for r in all_runs if int(r['success']) == 1]
    
    # Success rate (use ALL runs, not just successful)
    all_runs_cnt = {}
    for method in methods:
        all_runs_list = []
        for scenario in scenarios:
            all_runs_list.extend(data[method].get(scenario, []))
        all_runs_cnt[method] = all_runs_list
    
    sr_vals = {m: success_rate(all_runs_cnt[m]) for m in methods}
    line = f"  {'Success Rate (%)':25s}"
    for m in methods:
        line += f" | {sr_vals[m]:>25.1f}"
    winner = max(sr_vals, key=sr_vals.get)
    line += f" | {winner.upper()}"
    print(line)
    
    compare_metrics = [
        ('travel_time_s', 'Travel Time (s)', True),    # lower is better
        ('traj_length_m', 'Traj Length (m)', True),     # lower is better
        ('collision_count', 'Avg Collisions', True),    # lower is better
        ('min_obs_dist_m', 'Min Obs Dist (m)', False),  # higher is better
        ('avg_vel_ms', 'Avg Velocity (m/s)', False),    # higher is better (faster)
        ('max_vel_ms', 'Max Velocity (m/s)', None),     # informational
        ('avg_acc_ms2', 'Avg Accel (m/s²)', None),      # informational
        ('max_acc_ms2', 'Max Accel (m/s²)', True),      # lower is better
        ('smoothness_cost', 'Smoothness Cost', True),   # lower is better
    ]
    
    for key, label, lower_better in compare_metrics:
        means = {}
        line = f"  {label:25s}"
        for m in methods:
            mean, std = compute_stats(all_succ[m], key)
            means[m] = mean
            if mean is not None:
                line += f" | {mean:>15.4f} ± {std:>6.2f}"
            else:
                line += f" | {'N/A':>25s}"
        
        if lower_better is not None:
            if lower_better:
                winner = min(means, key=means.get)
            else:
                # For "higher is better" - but careful with inf
                valid_means = {k:v for k,v in means.items() if v != float('inf')}
                winner = max(valid_means, key=valid_means.get) if valid_means else "N/A"
            line += f" | {winner.upper()}" if isinstance(winner, str) else f" | {winner}"
        else:
            line += f" | -"
        print(line)
    
    # === PER-SCENARIO COMPARISON TABLE ===
    print(f"\n\n{'='*100}")
    print(f"  PER-SCENARIO COMPARISON (mean values, successful runs only)")  
    print(f"{'='*100}")
    
    for scenario in scenarios:
        print(f"\n  --- {scenario} ---")
        
        key_metrics = [
            ('travel_time_s', 'Time(s)', True),
            ('collision_count', 'Coll', True),
            ('min_obs_dist_m', 'MinDist', False),
            ('avg_vel_ms', 'AvgVel', False),
            ('smoothness_cost', 'Smooth', True),
        ]
        
        header = f"  {'Method':8s} {'Runs':>5s} {'SR%':>5s}"
        for _, lbl, _ in key_metrics:
            header += f" {lbl:>12s}"
        print(header)
        
        for method in methods:
            runs = data[method].get(scenario, [])
            if not runs:
                continue
            sr = success_rate(runs)
            succ = [r for r in runs if int(r['success']) == 1]
            
            line = f"  {method.upper():8s} {len(runs):>5d} {sr:>5.0f}"
            for key, _, _ in key_metrics:
                if succ:
                    mean, _ = compute_stats(succ, key)
                    line += f" {mean:>12.4f}"
                else:
                    line += f" {'N/A':>12s}"
            print(line)
    
    # === WINNER TALLY ===
    print(f"\n\n{'='*100}")
    print(f"  SCENARIO WINNERS (per metric, successful runs only)")
    print(f"{'='*100}")
    
    win_count = defaultdict(int)
    
    key_metrics_for_scoring = [
        ('travel_time_s', True),
        ('collision_count', True),
        ('min_obs_dist_m', False),
        ('avg_vel_ms', False),
        ('smoothness_cost', True),
    ]
    
    for scenario in scenarios:
        for key, lower_better in key_metrics_for_scoring:
            best_method = None
            best_val = None
            for method in methods:
                runs = data[method].get(scenario, [])
                succ = [r for r in runs if int(r['success']) == 1]
                if not succ:
                    continue
                mean, _ = compute_stats(succ, key)
                if best_val is None:
                    best_val = mean
                    best_method = method
                elif lower_better and mean < best_val:
                    best_val = mean
                    best_method = method
                elif not lower_better and mean > best_val:
                    best_val = mean
                    best_method = method
            
            if best_method:
                win_count[best_method] += 1
                print(f"  {scenario:25s} {key:20s} -> {best_method.upper()}")
    
    print(f"\n  TOTAL WINS:")
    for method in methods:
        print(f"    {method.upper():8s}: {win_count[method]} / {len(scenarios) * len(key_metrics_for_scoring)}")
    
    # === RAW DATA DUMP FOR DDO ===
    print(f"\n\n{'='*100}")
    print(f"  DDO RAW RESULTS")
    print(f"{'='*100}")
    print(f"  {'Scenario':25s} {'Run':>4s} {'OK':>3s} {'Time':>8s} {'Len':>8s} {'Coll':>5s} {'MinD':>8s} {'AvgV':>7s} {'MaxA':>8s} {'Smooth':>12s}")
    
    for scenario in scenarios:
        runs = data['ddo'].get(scenario, [])
        for r in runs:
            print(f"  {r['scenario']:25s} {r['run_id']:>4s} {r['success']:>3s} {float(r['travel_time_s']):>8.2f} {float(r['traj_length_m']):>8.2f} {r['collision_count']:>5s} {float(r['min_obs_dist_m']):>8.4f} {float(r['avg_vel_ms']):>7.4f} {float(r['max_acc_ms2']):>8.2f} {float(r['smoothness_cost']):>12.2f}")

if __name__ == '__main__':
    main()
