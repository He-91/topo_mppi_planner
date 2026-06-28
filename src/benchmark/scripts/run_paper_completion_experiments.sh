#!/bin/bash
##############################################################################
# run_paper_completion_experiments.sh
#
# Paper-data completion runner for the current Topo-MPPI workspace.
#
# Modes:
#   main5     Add five more main-comparison rounds for Env1~Env5.
#   stress3   Run the current three-run dynamic stress tests.
#   ablation3 Run the current three-run core/switch ablations.
#   all       Run main5, stress3, then ablation3.
#
# Run inside the Docker container:
#   cd /home/developer/ros_ws/ddo-topo-mppi
#   source /opt/ros/noetic/setup.bash
#   source devel/setup.bash
#   bash src/benchmark/scripts/run_paper_completion_experiments.sh main5
##############################################################################

set -u

MODE="${1:-all}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUN_ONE="$SCRIPT_DIR/run_one_experiment.sh"
DDO_WS="/home/developer/ros_ws/ddo-topo-mppi"
EGO_WS="/home/developer/ros_ws/ddo-topo-mppi/packages/ego-planner"
RESULT_DIR="$DDO_WS/src/benchmark/results"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
TAG="papercomplete_${MODE}_${TIMESTAMP}"
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
    echo "Restart container 65abafec5dc5, rerun nvidia-smi, then restart this benchmark."
    exit 2
fi

RESULT_START_LINE=0
EXTENDED_START_LINE=0
TRAJ_START_LINE=0
CURRENT=0
SUCCESS=0
FAIL=0
CURRENT_CHILD_PID=""

[ -f "$RESULT_DIR/results_master.txt" ] && RESULT_START_LINE=$(wc -l < "$RESULT_DIR/results_master.txt")
[ -f "$EXTENDED_ALL" ] && EXTENDED_START_LINE=$(wc -l < "$EXTENDED_ALL")
[ -f "$TRAJ_ALL" ] && TRAJ_START_LINE=$(wc -l < "$TRAJ_ALL")

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

run_case() {
    local exp_name="$1"
    local sim_launch="$2"
    local planner_launch="$3"
    local ws="$4"
    local timeout_s="$5"

    CURRENT=$((CURRENT + 1))
    echo ""
    echo "------------------------------------------------------------------"
    echo "[$CURRENT] $exp_name"
    echo "sim=$sim_launch planner=$planner_launch ws=$ws timeout=${timeout_s}s"
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

    echo "[progress] current=$CURRENT success=$SUCCESS fail=$FAIL exit=$exit_code"
    sleep 8
}

run_main5() {
    local rounds=5
    declare -a ENVS=(
        "env1 env1_fast_sim.launch planner_topo_mppi_env1.launch planner_ego_env1.launch 150"
        "env2 env2_ego_sim.launch planner_topo_mppi_env2.launch planner_ego_env2.launch 150"
        "env3 env3_ddo_sim.launch planner_topo_mppi_env3.launch planner_ego_env3.launch 180"
        "env4 env4_pure_dynamic_sim.launch planner_topo_mppi_env4.launch planner_ego_env4.launch 180"
        "env5 env5_narrow_maze_sim.launch planner_topo_mppi_env5.launch planner_ego_env5.launch 180"
    )

    for round in $(seq 1 "$rounds"); do
        echo ""
        echo "==================== PAPER MAIN EXTRA ROUND $round / $rounds ===================="
        for env_spec in "${ENVS[@]}"; do
            read -r env_name sim_launch topo_launch ego_launch timeout_s <<< "$env_spec"
            run_case "topo_mppi_${TAG}_${env_name}_r${round}" \
                "$sim_launch" "$topo_launch" "$DDO_WS" "$timeout_s"
            run_case "ego_${TAG}_${env_name}_r${round}" \
                "$sim_launch" "$ego_launch" "$EGO_WS" "$timeout_s"
        done
    done
}

run_stress3() {
    local rounds=3
    for round in $(seq 1 "$rounds"); do
        echo ""
        echo "==================== PAPER STRESS ROUND $round / $rounds ===================="
        run_case "topo_mppi_${TAG}_env3_hs_r${round}" \
            "env3_ddo_sim.launch" "planner_ddo_env3_hs.launch" "$DDO_WS" 180
        run_case "ego_${TAG}_env3_hs_r${round}" \
            "env3_ddo_sim.launch" "planner_ego_env3_hs.launch" "$EGO_WS" 180
        run_case "topo_mppi_${TAG}_env4_dynstress_r${round}" \
            "env4_pure_dynamic_stress_sim.launch" "planner_topo_mppi_env4_stress.launch" "$DDO_WS" 180
        run_case "ego_${TAG}_env4_dynstress_r${round}" \
            "env4_pure_dynamic_stress_sim.launch" "planner_ego_env4.launch" "$EGO_WS" 180
    done
}

run_ablation3() {
    local rounds=3
    for round in $(seq 1 "$rounds"); do
        echo ""
        echo "==================== PAPER ABLATION ROUND $round / $rounds ===================="
        run_case "topo_mppi_${TAG}_env5_no_topo_r${round}" \
            "env5_narrow_maze_sim.launch" "planner_ablation_env5_no_topo.launch" "$DDO_WS" 180
        run_case "topo_mppi_${TAG}_env3_no_topo_r${round}" \
            "env3_ddo_sim.launch" "planner_ablation_env3_no_topo.launch" "$DDO_WS" 180
        run_case "topo_mppi_${TAG}_env4_no_mppi_r${round}" \
            "env4_pure_dynamic_sim.launch" "planner_ablation_env4_no_mppi.launch" "$DDO_WS" 180
        run_case "topo_mppi_${TAG}_env3_no_mppi_r${round}" \
            "env3_ddo_sim.launch" "planner_ablation_env3_no_mppi.launch" "$DDO_WS" 180
        run_case "topo_mppi_${TAG}_env2_single_topo_r${round}" \
            "env2_ego_sim.launch" "planner_ablation_env2_single_topo.launch" "$DDO_WS" 150
        run_case "topo_mppi_${TAG}_env2_no_static_gate_r${round}" \
            "env2_ego_sim.launch" "planner_ablation_env2_no_static_gate.launch" "$DDO_WS" 150
        run_case "topo_mppi_${TAG}_env4_dynamic_mode_score_r${round}" \
            "env4_pure_dynamic_sim.launch" "planner_ablation_env4_dynamic_mode_score.launch" "$DDO_WS" 180
        run_case "topo_mppi_${TAG}_env4_no_bspline_fallback_r${round}" \
            "env4_pure_dynamic_sim.launch" "planner_ablation_env4_no_bspline_fallback.launch" "$DDO_WS" 180
    done
}

echo "=================================================================="
echo " Paper completion experiments"
echo " Mode: $MODE"
echo " Tag: $TAG"
echo " DDO workspace: $DDO_WS"
echo " EGO workspace: $EGO_WS"
echo " Started: $(date)"
echo "=================================================================="

case "$MODE" in
    main5)
        run_main5
        ;;
    stress3)
        run_stress3
        ;;
    ablation3)
        run_ablation3
        ;;
    all)
        run_main5
        run_stress3
        run_ablation3
        ;;
    *)
        echo "ERROR: unknown mode '$MODE'. Use main5, stress3, ablation3, or all."
        exit 3
        ;;
esac

{
    echo "# $TAG"
    echo "mode=$MODE"
    echo "success=$SUCCESS / $CURRENT"
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

echo ""
echo "=================================================================="
echo " Paper completion experiments finished: $(date)"
echo " success=$SUCCESS / $CURRENT, fail=$FAIL"
echo " Report: $REPORT_FILE"
echo " Extended snapshot: $EXTENDED_SNAPSHOT"
echo " Trajectory snapshot: $TRAJ_SNAPSHOT"
echo "=================================================================="

cat "$REPORT_FILE"
