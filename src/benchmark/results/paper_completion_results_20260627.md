# Paper Completion Results 2026-06-27

This file is generated from completed benchmark snapshots. It supersedes older paper-facing tables for the experiment batches listed below.

## Source Batches

- `final5_20260626_135440`
- `papercomplete_main5_20260627_103027`
- `papercomplete_stress3_20260627_121213`
- `papercomplete_ablation3_20260627_123503`

## Validity Filter

- Rows are joined by snapshot order from `*_results_master_tail.txt` and `*_benchmark_results_extended.csv`.
- Extended rows are considered valid only when `cloud_samples > 0` and `travel_time_s >= 1.0`.
- The filtered invalid rows are written to `src/benchmark/results/paper_invalid_rows_20260627.csv`.
- Success counts use valid rows only and require both benchmark success and `SUCCESS` status in the log summary.

## 10-Run Main Comparison

| Env | Method | Success | Time mean +/- std | Min clearance | Static collisions | Dynamic collisions | ColWarn | Replans | TopoFail / BsFail | Invalid rows |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| env1 | ego | 10/10 | 50.71 +/- 1.97 | 0.196 +/- 0.116 | 2.40 +/- 2.01 | 0.00 +/- 0.00 | 8.90 +/- 3.11 | 95.20 +/- 8.94 | 0.00 / 0.00 | 0 |
| env1 | topo_mppi | 9/9 | 61.57 +/- 9.21 | 0.168 +/- 0.089 | 2.89 +/- 2.67 | 0.00 +/- 0.00 | 1.56 +/- 1.13 | 91.56 +/- 18.86 | 27.67 / 8.44 | 1 |
| env2 | ego | 9/10 | 55.98 +/- 2.07 | 0.189 +/- 0.084 | 2.33 +/- 1.80 | 0.00 +/- 0.00 | 41.70 +/- 120.66 | 254.30 +/- 212.08 | 0.00 / 0.00 | 0 |
| env2 | topo_mppi | 8/10 | 111.96 +/- 13.56 | 0.035 +/- 0.031 | 10.75 +/- 2.92 | 0.00 +/- 0.00 | 2.40 +/- 2.12 | 267.10 +/- 97.18 | 12.30 / 29.60 | 0 |
| env3 | ego | 10/10 | 50.82 +/- 1.81 | 0.173 +/- 0.107 | 2.10 +/- 1.52 | 0.90 +/- 0.88 | 8.80 +/- 8.11 | 90.20 +/- 17.02 | 0.00 / 0.00 | 0 |
| env3 | topo_mppi | 10/10 | 58.30 +/- 9.91 | 0.199 +/- 0.132 | 1.50 +/- 1.43 | 0.50 +/- 0.71 | 1.00 +/- 0.47 | 82.10 +/- 9.48 | 19.50 / 6.40 | 0 |
| env4 | ego | 8/10 | 53.61 +/- 5.68 | 0.026 +/- 0.022 | 6.50 +/- 2.93 | 7.88 +/- 2.10 | 162.50 +/- 90.24 | 471.10 +/- 580.88 | 0.00 / 0.00 | 0 |
| env4 | topo_mppi | 10/10 | 62.36 +/- 6.17 | 0.031 +/- 0.022 | 5.30 +/- 1.70 | 6.20 +/- 1.55 | 0.70 +/- 0.82 | 146.40 +/- 24.32 | 7.20 / 6.80 | 0 |
| env5 | ego | 9/10 | 40.93 +/- 4.51 | 0.084 +/- 0.078 | 7.11 +/- 3.59 | 0.00 +/- 0.00 | 1331.30 +/- 4029.98 | 2768.50 +/- 7915.44 | 0.00 / 0.00 | 0 |
| env5 | topo_mppi | 9/10 | 92.92 +/- 28.73 | 0.025 +/- 0.014 | 17.11 +/- 8.33 | 0.00 +/- 0.00 | 4.20 +/- 4.21 | 442.90 +/- 255.65 | 68.40 / 54.80 | 0 |

## 3-Run Stress Tests

| Case | Method | Success | Time mean +/- std | Min clearance | Static collisions | Dynamic collisions | ColWarn | Replans | TopoFail / BsFail | Invalid rows |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| env3_high_speed | ego | 3/3 | 25.52 +/- 3.40 | 0.136 +/- 0.153 | 2.67 +/- 2.31 | 0.67 +/- 0.58 | 2.67 +/- 0.58 | 77.00 +/- 7.55 | 0.00 / 0.00 | 0 |
| env3_high_speed | topo_mppi | 3/3 | 49.69 +/- 4.97 | 0.021 +/- 0.015 | 9.67 +/- 1.53 | 1.33 +/- 1.53 | 2.00 +/- 0.00 | 109.67 +/- 9.50 | 48.00 / 14.67 | 0 |
| env4_dynamic_stress | ego | 3/3 | 48.49 +/- 2.45 | 0.016 +/- 0.015 | 6.67 +/- 3.06 | 6.33 +/- 0.58 | 135.33 +/- 52.99 | 279.00 +/- 49.79 | 0.00 / 0.00 | 0 |
| env4_dynamic_stress | topo_mppi | 1/3 | 27.27 +/- 0.00 | 0.023 +/- 0.000 | 1.00 +/- 0.00 | 1.00 +/- 0.00 | 0.00 +/- 0.00 | 75.00 +/- 6.00 | 14.00 / 0.67 | 0 |

## 3-Run Ablations

| Case | Success | Time mean +/- std | Min clearance | Static collisions | Dynamic collisions | ColWarn | Replans | TopoFail / BsFail | Invalid rows |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| env2_no_static_gate | 2/3 | 101.76 +/- 13.24 | 0.019 +/- 0.015 | 12.50 +/- 0.71 | 0.00 +/- 0.00 | 1.67 +/- 2.08 | 432.67 +/- 287.51 | 24.00 / 21.33 | 0 |
| env2_single_topo | 2/3 | 85.77 +/- 7.89 | 0.107 +/- 0.027 | 6.50 +/- 2.12 | 0.00 +/- 0.00 | 2.33 +/- 3.21 | 335.00 +/- 221.38 | 2.00 / 39.33 | 0 |
| env3_no_mppi | 3/3 | 51.13 +/- 2.75 | 0.231 +/- 0.188 | 1.00 +/- 1.00 | 0.67 +/- 1.15 | 6.33 +/- 1.53 | 100.67 +/- 20.60 | 30.00 / 36.67 | 0 |
| env3_no_topo | 3/3 | 65.82 +/- 16.42 | 0.094 +/- 0.115 | 3.00 +/- 2.00 | 1.33 +/- 0.58 | 1.33 +/- 0.58 | 114.33 +/- 32.53 | 0.00 / 3.33 | 0 |
| env4_dynamic_mode_score | 3/3 | 59.40 +/- 2.99 | 0.017 +/- 0.013 | 5.00 +/- 1.73 | 5.67 +/- 0.58 | 1.00 +/- 1.00 | 143.67 +/- 6.66 | 8.00 / 6.67 | 0 |
| env4_no_bspline_fallback | 3/3 | 63.78 +/- 1.40 | 0.053 +/- 0.068 | 5.00 +/- 1.00 | 6.67 +/- 1.53 | 1.00 +/- 1.00 | 151.00 +/- 82.40 | 8.00 / 5.33 | 0 |
| env4_no_mppi | 3/3 | 49.60 +/- 3.98 | 0.051 +/- 0.036 | 3.33 +/- 1.53 | 5.67 +/- 1.15 | 4.67 +/- 1.53 | 130.00 +/- 17.32 | 6.00 / 59.67 | 0 |
| env5_no_topo | 3/3 | 67.76 +/- 3.23 | 0.122 +/- 0.107 | 6.33 +/- 5.13 | 0.00 +/- 0.00 | 1.67 +/- 1.53 | 585.67 +/- 284.74 | 0.00 / 80.00 | 0 |

## Current Reading

- Topo-MPPI is strongest in the normal Env4 dynamic benchmark: it has higher success than EGO and much lower `ColWarn` and replanning pressure. The Env4 stress test is a limitation case because Topo-MPPI only reached `1/3` strict benchmark success, even though its successful run had low warning pressure.
- Topo-MPPI is not currently faster than EGO in Env2/Env5; those environments remain the main runtime and robustness limitations.
- Multi-topology and static safety gates matter in Env2: single-topo and no-static-gate ablations each produced one timeout in three runs.
- Removing MPPI did not destroy success rate in the tested Env3/Env4 ablations, but it increased B-spline failures and warning pressure, so the defensible claim is improved dynamic/local quality rather than absolute necessity for mission success.
- No-Topo ablations succeeded in this latest 3-run batch, so the claim should be phrased as topology guidance improving efficiency/robustness in complex scenes, not as a universal hard requirement.
- Stress Env4 exposes a strict route-validation issue for Topo-MPPI: two runs reached the goal region but were marked timeout by benchmark validation. This should be reported as a failure under the strict benchmark rule.

