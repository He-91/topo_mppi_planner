# Topo-MPPI

Topo-MPPI is a ROS Noetic UAV trajectory-planning workspace derived from
EGO-Planner. The active planner keeps the EGO-Planner FSM, B-spline trajectory
interface, and `traj_server` execution chain, then adds topology-guided
multi-path search and MPPI local optimization for dynamic-obstacle and
topology-constrained environments.

The active implementation is under `src/`. Reference planners are kept under
`packages/` for comparison and source-code reference.

## Planning Pipeline

```text
Waypoint / RViz goal
        |
        v
EGOReplanFSM
        |
        v
PlannerManager::reboundReplan()
        |
        +--> TopoPRM multi-path search
        +--> batched / multi-candidate MPPI optimization
        +--> B-spline parameterization, rebound, and fallback
        +--> final static / dynamic / z validators
        |
        v
/planning/bspline -> traj_server -> /planning/pos_cmd
```

Current baseline policy:

- B-spline remains the final executable trajectory layer.
- MPPI fallback is used when B-spline rebound/precheck fails.
- Multiple topology candidates are optimized before selecting the final local trajectory.
- Dynamic scenes use fast normalized-cost candidate selection by default.
- Static candidate safety gates and final validators remain enabled.

## Main Packages

| Path | ROS package | Role |
| --- | --- | --- |
| `src/planner/plan_manage` | `ddo_planner` | FSM, planner manager, launch files, trajectory server |
| `src/planner/path_searching` | `ddo_path_searching` | TopoPRM, CPU/GPU MPPI, CUDA kernels |
| `src/planner/bspline_opt` | `ddo_bspline_opt` | B-spline parameterization, rebound, refine optimization |
| `src/planner/plan_env` | `ddo_plan_env` | Grid map, occupancy map, EDT/ESDF queries |
| `src/planner/traj_utils` | `ddo_traj_utils` | Trajectory utilities and visualization |
| `src/uav_simulator` | multiple | Simulation, controller, map generator, waypoint tools |
| `src/benchmark` | `ddo_benchmark` | Benchmark runner and metric collection |

Reference packages:

| Path | Purpose |
| --- | --- |
| `packages/ego-planner` | Original EGO-Planner baseline/reference |
| `packages/Fast-Planner` | Fast-Planner reference |
| `packages/TGK-Planner` | TGK/kRRT topology reference |

## Quick Start

Enter the Docker container:

```bash
docker exec -it 65abafec5dc5 bash
```

Build:

```bash
cd /home/developer/ros_ws/ddo-topo-mppi
source /opt/ros/noetic/setup.bash
catkin build ddo_planner
source devel/setup.bash
```

Run the main simulation:

```bash
roslaunch ddo_planner topo_mppi_fastplanner_map.launch
```

Start RViz in another terminal:

```bash
cd /home/developer/ros_ws/ddo-topo-mppi
source devel/setup.bash
export DISPLAY=:0
rviz -d src/planner/plan_manage/launch/fastplanner_test.rviz
```

## Benchmark Entry Points

Single experiment:

```bash
bash src/benchmark/scripts/run_one_experiment.sh \
  topo_mppi_env4 \
  env4_pure_dynamic_sim.launch \
  planner_topo_mppi_env4.launch \
  /home/developer/ros_ws/ddo-topo-mppi \
  180
```

Paper-completion batches:

```bash
bash src/benchmark/scripts/run_paper_completion_experiments.sh main5
bash src/benchmark/scripts/run_paper_completion_experiments.sh stress3
bash src/benchmark/scripts/run_paper_completion_experiments.sh ablation3
python3 src/benchmark/scripts/summarize_paper_completion_results.py
python3 src/benchmark/scripts/summarize_paper_runtime_candidate_metrics.py
```

Current paper-facing generated summaries are under:

```text
src/benchmark/results/paper_completion_results_20260627.md
src/benchmark/results/paper_runtime_candidate_summary_20260627.md
```

## Current Status

This repository is an engineering validation baseline, not a final
all-environment superiority claim. Current results show Topo-MPPI is strongest
in dynamic scenes, especially Env4, while Env2 and Env5 remain the main
optimization targets for future work.
