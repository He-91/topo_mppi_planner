# Paper Runtime And Candidate Diagnostics 2026-06-27

This file is generated from existing Topo-MPPI diagnostic CSV files. It does not add new simulation runs.

## Source Batches

- `final5_20260626_135440`
- `papercomplete_main5_20260627_103027`
- `papercomplete_stress3_20260627_121213`
- `papercomplete_ablation3_20260627_123503`

## Planning Runtime Summary

| Batch | Case | Files | Cycles | Total mean/p95/max ms | MPPI mean ms | B-spline mean ms | Validator mean ms | Candidate mean | Success cycle rate |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| final5_20260626_135440 | env1 | 5 | 404 | 25.45/83.31/202.71 | 4.49 | 9.72 | 0.09 | 3.47 | 0.889 |
| final5_20260626_135440 | env2 | 5 | 1218 | 44.83/122.65/275.47 | 6.88 | 14.74 | 0.25 | 3.84 | 0.469 |
| final5_20260626_135440 | env3 | 5 | 369 | 21.89/70.73/134.15 | 4.56 | 6.71 | 0.06 | 3.51 | 0.946 |
| final5_20260626_135440 | env4 | 5 | 630 | 23.40/56.34/127.14 | 5.03 | 5.19 | 0.12 | 3.44 | 0.695 |
| final5_20260626_135440 | env5 | 5 | 2146 | 48.02/135.05/436.45 | 3.04 | 14.86 | 0.21 | 3.71 | 0.342 |
| papercomplete_ablation3_20260627_123503 | env2_no_static_gate | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_ablation3_20260627_123503 | env2_single_topo | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_ablation3_20260627_123503 | env3_no_mppi | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_ablation3_20260627_123503 | env3_no_topo | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_ablation3_20260627_123503 | env4_dynamic_mode_score | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_ablation3_20260627_123503 | env4_no_bspline_fallback | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_ablation3_20260627_123503 | env4_no_mppi | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_ablation3_20260627_123503 | env5_no_topo | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_main5_20260627_103027 | env1 | 5 | 444 | 25.68/86.61/208.40 | 4.27 | 10.19 | 0.09 | 3.38 | 0.860 |
| papercomplete_main5_20260627_103027 | env2 | 5 | 1091 | 53.11/148.22/319.02 | 6.72 | 21.66 | 0.27 | 3.88 | 0.505 |
| papercomplete_main5_20260627_103027 | env3 | 5 | 392 | 24.18/77.16/202.86 | 4.35 | 8.97 | 0.05 | 3.42 | 0.974 |
| papercomplete_main5_20260627_103027 | env4 | 5 | 685 | 23.84/57.27/167.34 | 4.91 | 5.69 | 0.12 | 3.34 | 0.645 |
| papercomplete_main5_20260627_103027 | env5 | 5 | 1557 | 51.64/149.57/401.26 | 2.92 | 17.84 | 0.23 | 3.68 | 0.329 |
| papercomplete_stress3_20260627_121213 | env3_hs | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |
| papercomplete_stress3_20260627_121213 | env4_dynstress | 3 | 0 | 0.00/0.00/0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0.000 |

## Candidate Selection Summary

| Batch | Case | Files | Candidate rows | Cycles | Selected rows | Success row rate | Selected static clearance mean | Selected dynamic clearance mean | Selected path length mean | Top reject reasons |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| final5_20260626_135440 | env1 | 5 | 1369 | 95 | 369 | 0.997 | 2.680 | 5.000 | 5.20 | not_selected_selection_score_or_gate:996;selected_selection_score_or_gate:369;mppi_failed_or_z_rejected:4 |
| final5_20260626_135440 | env2 | 5 | 4659 | 346 | 1178 | 0.982 | 0.733 | 5.000 | 5.33 | not_selected_selection_score_or_gate:3396;selected_selection_score_or_gate:1178;mppi_failed_or_z_rejected:85 |
| final5_20260626_135440 | env3 | 5 | 1267 | 78 | 336 | 0.983 | 2.870 | 3.054 | 5.25 | not_selected_dynamic_score_or_gate:910;selected_dynamic_score_or_gate:336;mppi_failed_or_z_rejected:21 |
| final5_20260626_135440 | env4 | 5 | 2166 | 159 | 624 | 0.971 | 2.142 | 1.265 | 4.58 | not_selected_dynamic_score_or_gate:1479;selected_dynamic_score_or_gate:624;mppi_failed_or_z_rejected:63 |
| final5_20260626_135440 | env5 | 5 | 7860 | 706 | 2016 | 0.980 | 0.644 | 5.000 | 4.27 | not_selected_selection_score_or_gate:5686;selected_selection_score_or_gate:2016;mppi_failed_or_z_rejected:158 |
| papercomplete_ablation3_20260627_123503 | env2_no_static_gate | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_ablation3_20260627_123503 | env2_single_topo | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_ablation3_20260627_123503 | env3_no_mppi | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_ablation3_20260627_123503 | env3_no_topo | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_ablation3_20260627_123503 | env4_dynamic_mode_score | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_ablation3_20260627_123503 | env4_no_bspline_fallback | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_ablation3_20260627_123503 | env4_no_mppi | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_ablation3_20260627_123503 | env5_no_topo | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_main5_20260627_103027 | env1 | 5 | 1442 | 113 | 382 | 0.985 | 2.466 | 5.000 | 5.21 | not_selected_selection_score_or_gate:1039;selected_selection_score_or_gate:382;mppi_failed_or_z_rejected:21 |
| papercomplete_main5_20260627_103027 | env2 | 5 | 4209 | 255 | 1057 | 0.981 | 0.857 | 5.000 | 5.56 | not_selected_selection_score_or_gate:3074;selected_selection_score_or_gate:1057;mppi_failed_or_z_rejected:78 |
| papercomplete_main5_20260627_103027 | env3 | 5 | 1304 | 95 | 350 | 0.995 | 2.622 | 3.229 | 5.22 | not_selected_dynamic_score_or_gate:947;selected_dynamic_score_or_gate:350;mppi_failed_or_z_rejected:7 |
| papercomplete_main5_20260627_103027 | env4 | 5 | 2266 | 158 | 660 | 0.975 | 1.552 | 1.276 | 4.43 | not_selected_dynamic_score_or_gate:1549;selected_dynamic_score_or_gate:660;mppi_failed_or_z_rejected:57 |
| papercomplete_main5_20260627_103027 | env5 | 5 | 5615 | 555 | 1391 | 0.951 | 0.467 | 5.000 | 4.58 | not_selected_selection_score_or_gate:3951;selected_selection_score_or_gate:1391;mppi_failed_or_z_rejected:273 |
| papercomplete_stress3_20260627_121213 | env3_hs | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |
| papercomplete_stress3_20260627_121213 | env4_dynstress | 3 | 0 | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.00 | none |

## Reading

- Use the runtime table to support real-time feasibility claims for Topo-MPPI. Report p95/max latency when discussing high-speed replanning.
- Use the candidate table to show that multi-candidate selection is actually active, and to report selected-candidate clearance and path-length statistics.
- Rows with `cycles=0` or `Candidate rows=0` mean the diagnostic CSV files exist but are empty; the zero-valued timing/candidate fields are missing-data placeholders, not measured zero cost.
- These diagnostics are Topo-MPPI-only because EGO does not emit the same per-cycle Topo/MPPI/B-spline breakdown.
