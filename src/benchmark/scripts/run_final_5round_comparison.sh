#!/bin/bash
##############################################################################
# run_final_5round_comparison.sh
#
# Final paper-facing acceptance benchmark:
#   Topo-MPPI vs EGO, Env1~Env5, 5 rounds each = 50 experiments.
#
# Run inside the Docker container:
#   cd /home/developer/ros_ws/ddo-topo-mppi
#   source /opt/ros/noetic/setup.bash
#   source devel/setup.bash
#   bash src/benchmark/scripts/run_final_5round_comparison.sh
##############################################################################

set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_ONE="$SCRIPT_DIR/run_one_experiment.sh"
DDO_WS="/home/developer/ros_ws/ddo-topo-mppi"
EGO_WS="/home/developer/ros_ws/ddo-topo-mppi/packages/ego-planner"
RESULT_DIR="$DDO_WS/src/benchmark/results"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
TAG="final5_${TIMESTAMP}"
REPORT_FILE="$RESULT_DIR/${TAG}_results_master_tail.txt"
EXTENDED_ALL="$RESULT_DIR/benchmark_results_extended.csv"
EXTENDED_SNAPSHOT="$RESULT_DIR/${TAG}_benchmark_results_extended.csv"
TRAJ_ALL="$RESULT_DIR/trajectory_metrics.csv"
TRAJ_SNAPSHOT="$RESULT_DIR/${TAG}_trajectory_metrics.csv"

mkdir -p "$RESULT_DIR"

if [ ! -x "$RUN_ONE" ]; then
    echo "ERROR: missing executable run_one_experiment.sh: $RUN_ONE"
    exit 1
fi

if ! nvidia-smi >/dev/null 2>&1; then
    echo "ERROR: GPU/CUDA is not visible inside the container."
    echo "Restart the container, rerun nvidia-smi, then restart this benchmark."
    exit 2
fi

declare -a ENVS=(
    "env1 env1_fast_sim.launch planner_topo_mppi_env1.launch planner_ego_env1.launch 150"
    "env2 env2_ego_sim.launch planner_topo_mppi_env2.launch planner_ego_env2.launch 150"
    "env3 env3_ddo_sim.launch planner_topo_mppi_env3.launch planner_ego_env3.launch 180"
    "env4 env4_pure_dynamic_sim.launch planner_topo_mppi_env4.launch planner_ego_env4.launch 180"
    "env5 env5_narrow_maze_sim.launch planner_topo_mppi_env5.launch planner_ego_env5.launch 180"
)

TOTAL=$((5 * 5 * 2))
CURRENT=0
SUCCESS=0
FAIL=0
RESULT_START_LINE=0
EXTENDED_START_LINE=0
TRAJ_START_LINE=0
CURRENT_CHILD_PID=""

cleanup_children() {
    if [ -n "$CURRENT_CHILD_PID" ] && kill -0 "$CURRENT_CHILD_PID" 2>/dev/null; then
        kill -INT "$CURRENT_CHILD_PID" 2>/dev/null || true
        sleep 2
        kill -9 "$CURRENT_CHILD_PID" 2>/dev/null || true
        wait "$CURRENT_CHILD_PID" 2>/dev/null || true
    fi
    killall -9 rosmaster roscore roslaunch rosout 2>/dev/null || true
    for proc in simulator_node random_forest_sensing ego_planner_node \
        traj_server waypoint_generator ddo_planner_node \
        fast_planner_node so3_quadrotor_simulator_node quadrotor_simulator_so3 \
        odom_visualization mockamap_node dynamic_obstacle_generator \
        cloud_merger cloud_merger_node pcl_render_node nodelet random_forest \
        state_machine_node traj_server_node benchmark_node; do
        killall -9 "$proc" 2>/dev/null || true
    done
}

trap 'echo "Interrupted; cleaning active benchmark child..."; cleanup_children; exit 130' INT TERM

[ -f "$RESULT_DIR/results_master.txt" ] && RESULT_START_LINE=$(wc -l < "$RESULT_DIR/results_master.txt")
[ -f "$EXTENDED_ALL" ] && EXTENDED_START_LINE=$(wc -l < "$EXTENDED_ALL")
[ -f "$TRAJ_ALL" ] && TRAJ_START_LINE=$(wc -l < "$TRAJ_ALL")

echo "=================================================================="
echo " Final 5-round comparison: Topo-MPPI vs EGO"
echo " Tag: $TAG"
echo " Workspace: $DDO_WS"
echo " EGO workspace: $EGO_WS"
echo " Total experiments: $TOTAL"
echo " Started: $(date)"
echo "=================================================================="

run_case() {
    local exp_name="$1"
    local sim_launch="$2"
    local planner_launch="$3"
    local ws="$4"
    local timeout_s="$5"

    CURRENT=$((CURRENT + 1))
    echo ""
    echo "------------------------------------------------------------------"
    echo "[$CURRENT/$TOTAL] $exp_name"
    echo "sim=$sim_launch planner=$planner_launch timeout=${timeout_s}s"
    echo "------------------------------------------------------------------"

    bash "$RUN_ONE" "$exp_name" "$sim_launch" "$planner_launch" "$ws" "$timeout_s" &
    CURRENT_CHILD_PID=$!
    wait "$CURRENT_CHILD_PID"
    local exit_code=$?
    CURRENT_CHILD_PID=""

    local last_row=""
    if [ -f "$RESULT_DIR/results_master.txt" ]; then
        last_row="$(grep "^${exp_name} |" "$RESULT_DIR/results_master.txt" 2>/dev/null | tail -1)"
    fi
    if echo "$last_row" | grep -q "| SUCCESS |"; then
        SUCCESS=$((SUCCESS + 1))
    else
        FAIL=$((FAIL + 1))
    fi

    echo "[progress] $CURRENT/$TOTAL complete, success=$SUCCESS fail=$FAIL, exit=$exit_code"
    sleep 8
}

for round in 1 2 3 4 5; do
    echo ""
    echo "==================== ROUND $round / 5 ===================="
    for env_spec in "${ENVS[@]}"; do
        read -r env_name sim_launch topo_launch ego_launch timeout_s <<< "$env_spec"
        run_case "topo_mppi_${TAG}_${env_name}_r${round}" \
            "$sim_launch" "$topo_launch" "$DDO_WS" "$timeout_s"
        run_case "ego_${TAG}_${env_name}_r${round}" \
            "$sim_launch" "$ego_launch" "$EGO_WS" "$timeout_s"
    done
done

echo ""
echo "=================================================================="
echo " Final 5-round comparison finished: $(date)"
echo " success=$SUCCESS / $TOTAL, fail=$FAIL"
echo " Results: $RESULT_DIR/results_master.txt"
echo " Report: $REPORT_FILE"
echo "=================================================================="

{
    echo "# $TAG"
    echo "success=$SUCCESS / $TOTAL"
    echo "finished=$(date)"
    echo ""
    if [ -f "$RESULT_DIR/results_master.txt" ]; then
        tail -n +"$((RESULT_START_LINE + 1))" "$RESULT_DIR/results_master.txt"
    fi
} > "$REPORT_FILE"

if [ -f "$EXTENDED_ALL" ]; then
    head -1 "$EXTENDED_ALL" > "$EXTENDED_SNAPSHOT"
    tail -n +"$((EXTENDED_START_LINE + 1))" "$EXTENDED_ALL" >> "$EXTENDED_SNAPSHOT"
fi

if [ -f "$TRAJ_ALL" ]; then
    head -1 "$TRAJ_ALL" > "$TRAJ_SNAPSHOT"
    tail -n +"$((TRAJ_START_LINE + 1))" "$TRAJ_ALL" >> "$TRAJ_SNAPSHOT"
fi

cat "$REPORT_FILE"

if [ -f "$EXTENDED_SNAPSHOT" ]; then
    echo ""
    echo "Extended metrics snapshot: $EXTENDED_SNAPSHOT"
    python3 "$SCRIPT_DIR/summarize_final_5round_comparison.py" "$TAG" \
        --extended-csv "$EXTENDED_SNAPSHOT" || true
fi

if [ -f "$TRAJ_SNAPSHOT" ]; then
    echo "Trajectory metrics snapshot: $TRAJ_SNAPSHOT"
fi
