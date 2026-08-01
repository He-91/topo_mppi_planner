# Topo-MPPI Planner

Topo-MPPI Planner is a ROS Noetic quadrotor planning workspace derived from
EGO-Planner. The active planner keeps the EGO-Planner finite-state replanning
framework, B-spline execution interface, and trajectory server, and adds a
topology-guided multi-path front end plus MPPI local optimization.

The main source code is under `src/`. Reference planners are kept under
`packages/` for comparison and source-code reference.

## Features

- Topology-guided multi-candidate path search based on TopoPRM.
- MPPI local trajectory optimization for static and dynamic obstacle scenes.
- B-spline trajectory parameterization and fallback execution interface.
- Static, dynamic, and altitude validators before trajectory publication.
- Benchmark launch files for Topo-MPPI, EGO-Planner, Fast-Planner, and TGK.

## Repository Layout

| Path | Description |
| --- | --- |
| `src/planner/plan_manage` | Planner FSM, planner manager, launch files, and trajectory server |
| `src/planner/path_searching` | TopoPRM, MPPI planner, CUDA helpers, and path search modules |
| `src/planner/bspline_opt` | B-spline optimization and fallback trajectory refinement |
| `src/planner/plan_env` | Occupancy grid, map, and distance-field utilities |
| `src/planner/traj_utils` | Trajectory messages, utilities, and visualization helpers |
| `src/uav_simulator` | Quadrotor simulator, map generator, odometry, and controller packages |
| `src/benchmark` | Benchmark launches, scripts, and metric collection code |
| `packages/ego-planner` | Original EGO-Planner reference implementation |
| `packages/Fast-Planner` | Fast-Planner reference implementation |
| `packages/TGK-Planner` | TGK/kRRT reference implementation |

## Planning Pipeline

```text
RViz goal / waypoint
        |
        v
EGOReplanFSM
        |
        v
PlannerManager::reboundReplan()
        |
        +--> TopoPRM multi-path search
        +--> MPPI candidate trajectory optimization
        +--> B-spline parameterization and fallback refinement
        +--> final safety validators
        |
        v
/planning/bspline -> traj_server -> /planning/pos_cmd
```

## Requirements

- Ubuntu 20.04
- ROS Noetic
- `catkin_tools`
- CUDA-capable NVIDIA GPU for GPU MPPI and depth rendering paths

Install common dependencies:

```bash
sudo apt-get update
sudo apt-get install -y libarmadillo-dev ros-noetic-catkin python3-catkin-tools
```

## Build

Inside the project workspace:

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

## Run Simulation（ENV3）

Start the main Topo-MPPI simulation:

```bash
cd ddo-topo-mppi
source devel/setup.bash
roslaunch ddo_planner topo_mppi_fastplanner_map.launch
```

Start RViz in another terminal:

```bash
cd ddo-topo-mppi
source devel/setup.bash
export DISPLAY=:0
rviz -d src/planner/plan_manage/launch/fastplanner_test.rviz
```

Use RViz `2D Nav Goal` to send a target.

## Benchmark Examples


### Environment 1

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/benchmark/scripts/run_one_experiment.sh \
  topo_mppi_env1 \
  env1_fast_sim.launch \
  planner_topo_mppi_env1.launch \
  /home/developer/ros_ws/ddo-topo-mppi \
  150
```

### Environment 2

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/benchmark/scripts/run_one_experiment.sh \
  topo_mppi_env2 \
  env2_ego_sim.launch \
  planner_topo_mppi_env2.launch \
  /home/developer/ros_ws/ddo-topo-mppi \
  150
```

### Environment 3

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/benchmark/scripts/run_one_experiment.sh \
  topo_mppi_env3 \
  env3_ddo_sim.launch \
  planner_topo_mppi_env3.launch \
  /home/developer/ros_ws/ddo-topo-mppi \
  150
```

### Environment 4

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/benchmark/scripts/run_one_experiment.sh \
  topo_mppi_env4 \
  env4_pure_dynamic_sim.launch \
  planner_topo_mppi_env4.launch \
  /home/developer/ros_ws/ddo-topo-mppi \
  180
```

### Environment 5

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/benchmark/scripts/run_one_experiment.sh \
  topo_mppi_env5 \
  env5_narrow_maze_sim.launch \
  planner_topo_mppi_env5.launch \
  /home/developer/ros_ws/ddo-topo-mppi \
  180
```

---

## Full Benchmark Scripts

Regenerate all benchmark results used in the paper:

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/benchmark/scripts/run_paper_completion_experiments.sh main5
```

Run the complete five-round comparison (Topo-MPPI vs. EGO):

```bash
cd ddo-topo-mppi
source /opt/ros/noetic/setup.bash
source devel/setup.bash

bash src/benchmark/scripts/run_final_5round_comparison.sh
```

## License

The source code follows the license terms included in this repository and in
the referenced third-party planner packages.
