#!/bin/bash
##############################################################################
# run_one_experiment.sh  —  Run ONE experiment (env sim + planner)
# Usage: run_one_experiment.sh <exp_name> <sim_launch> <planner_launch> <algo_ws> <timeout>
#
# Example: 
#   bash run_one_experiment.sh topo_mppi_env1 env1_fast_sim.launch planner_topo_mppi_env1.launch \
#     /home/developer/ros_ws/ddo-topo-mppi 150
##############################################################################

EXP_NAME="$1"
SIM_LAUNCH="$2"
PLANNER_LAUNCH="$3"
ALGO_WS="$4"
TIMEOUT="${5:-150}"

DDO_WS="/home/developer/ros_ws/ddo-topo-mppi"
LAUNCH_DIR="$DDO_WS/src/benchmark/launch/manual_test"
RESULT_DIR="$DDO_WS/src/benchmark/results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_FILE="$RESULT_DIR/${EXP_NAME}_${TIMESTAMP}.log"
BENCH_LOG="$RESULT_DIR/${EXP_NAME}_${TIMESTAMP}_benchmark.log"
RESULT_FILE="$RESULT_DIR/results_master.txt"
ODOM_CSV="$RESULT_DIR/${EXP_NAME}_${TIMESTAMP}_odom.csv"
TRAJ_METRIC_FILE="$RESULT_DIR/trajectory_metrics.csv"
PLANNING_TIMING_SRC=""
CANDIDATE_QUALITY_SRC=""

mkdir -p "$RESULT_DIR"
if [ ! -f "$TRAJ_METRIC_FILE" ]; then
    echo "exp_name,status,total_time_s,path_length_m,avg_speed_mps,max_speed_mps,max_acc_mps2,min_z_m,max_z_m,odom_samples,odom_csv" > "$TRAJ_METRIC_FILE"
fi

GOAL_THRESHOLD=1.0
ENV_NAME=$(echo "$EXP_NAME" | grep -oE 'env[0-9]' | head -1)
GOAL_X=0.0
GOAL_Y=0.0
GOAL_Z=1.0
MIN_SUCCESS_TRAJ_LENGTH=0.0
MIN_COMPLETION_ELAPSED=0
REQUIRE_BENCHMARK_COMPLETION=0
CHECKPOINT_COUNT=0
CHECKPOINT_THRESHOLD=1.5
CHECKPOINT0_X=0.0; CHECKPOINT0_Y=0.0; CHECKPOINT0_Z=1.0
CHECKPOINT1_X=0.0; CHECKPOINT1_Y=0.0; CHECKPOINT1_Z=1.0
CHECKPOINT2_X=0.0; CHECKPOINT2_Y=0.0; CHECKPOINT2_Z=1.0
case "$ENV_NAME" in
    env1|env3)
        GOAL_X=-19.0
        GOAL_Y=0.0
        ;;
    env4)
        GOAL_X=-19.0
        GOAL_Y=0.0
        MIN_SUCCESS_TRAJ_LENGTH=60.0
        REQUIRE_BENCHMARK_COMPLETION=1
        # Env4 is a single pass from (19,0,1) to (-19,0,1).
        # A start-point checkpoint can be missed if benchmark collection starts
        # after the UAV has already moved, incorrectly turning a valid arrival
        # into success=0. The long route-length gate already rejects fake
        # short successes.
        CHECKPOINT_COUNT=0
        ;;
    env2)
        GOAL_X=-15.0
        GOAL_Y=0.0
        MIN_SUCCESS_TRAJ_LENGTH=60.0
        MIN_COMPLETION_ELAPSED=55
        REQUIRE_BENCHMARK_COMPLETION=1
        CHECKPOINT_COUNT=3
        # Env2 is a 40m-scale diamond route. A 1.5m waypoint sphere rejects
        # normal local-planner corner cutting even when the UAV clearly visits
        # the intended quadrant. Keep ordered checkpoints, but use a route
        # tolerance that matches the map scale. A 5.0m sphere still produced
        # false failures when a valid 100m loop cut the (15,0,1) corner by
        # about 5.4m; the 60m minimum route-length gate still rejects short
        # fake successes.
        CHECKPOINT_THRESHOLD=6.0
        CHECKPOINT0_X=0.0
        CHECKPOINT0_Y=15.0
        CHECKPOINT0_Z=1.0
        CHECKPOINT1_X=15.0
        CHECKPOINT1_Y=0.0
        CHECKPOINT1_Z=1.0
        CHECKPOINT2_X=0.0
        CHECKPOINT2_Y=-15.0
        CHECKPOINT2_Z=1.0
        ;;
    env5)
        GOAL_X=-16.0
        GOAL_Y=0.0
        MIN_SUCCESS_TRAJ_LENGTH=40.0
        MIN_COMPLETION_ELAPSED=25
        REQUIRE_BENCHMARK_COMPLETION=1
        # Env5 starts near the positive-x side and returns to the negative-x
        # goal. A checkpoint near the start can be missed if metrics collection
        # begins after takeoff motion has already passed it; the 40m length gate
        # plus final-goal check is the robust route validator.
        CHECKPOINT_COUNT=0
        ;;
    *)
        echo "[${EXP_NAME}] WARNING: Unknown env in experiment name; completion will rely on FSM marker only."
        ;;
esac

METHOD_NAME="$EXP_NAME"
if [[ "$EXP_NAME" == ego_* ]]; then
    METHOD_NAME="ego"
elif [[ "$EXP_NAME" == topo_mppi_* ]] || [[ "$EXP_NAME" == ddo_* ]] || [[ "$EXP_NAME" == opt_* ]]; then
    METHOD_NAME="topo_mppi"
elif [[ "$EXP_NAME" == fast_* ]]; then
    METHOD_NAME="fast_planner"
elif [[ "$EXP_NAME" == tgk_* ]]; then
    METHOD_NAME="tgk"
fi

calc_goal_distance() {
    local x="$1" y="$2" z="$3"
    if [[ ! "$x" =~ ^-?[0-9]+(\.[0-9]+)?$ ]] || [[ ! "$y" =~ ^-?[0-9]+(\.[0-9]+)?$ ]] || [[ ! "$z" =~ ^-?[0-9]+(\.[0-9]+)?$ ]]; then
        echo "999999"
        return
    fi
    awk -v x="$x" -v y="$y" -v z="$z" -v gx="$GOAL_X" -v gy="$GOAL_Y" -v gz="$GOAL_Z" \
        'BEGIN { dx=x-gx; dy=y-gy; dz=z-gz; printf "%.3f", sqrt(dx*dx + dy*dy + dz*dz) }'
}

within_goal_threshold() {
    local dist="$1"
    awk -v d="$dist" -v th="$GOAL_THRESHOLD" 'BEGIN { exit !(d <= th) }'
}

echo "=================================================================="
echo " EXPERIMENT: $EXP_NAME"
echo " Sim: $SIM_LAUNCH  |  Planner: $PLANNER_LAUNCH"
echo " Algo WS: $ALGO_WS  |  Timeout: ${TIMEOUT}s"
echo " Final goal: (${GOAL_X}, ${GOAL_Y}, ${GOAL_Z}), threshold=${GOAL_THRESHOLD}m"
echo "=================================================================="

# ── Step 0: Pre-cleanup — kill any stale ROS processes from previous experiments ──
echo "[${EXP_NAME}] Pre-cleanup..."
killall -9 rosmaster roscore roslaunch rosout 2>/dev/null
for proc in simulator_node random_forest_sensing ego_planner_node \
    traj_server waypoint_generator ddo_planner_node \
    fast_planner_node so3_quadrotor_simulator_node quadrotor_simulator_so3 \
    odom_visualization mockamap_node dynamic_obstacle_generator \
    cloud_merger cloud_merger_node pcl_render_node nodelet random_forest \
    state_machine_node traj_server_node benchmark_node; do
    killall -9 "$proc" 2>/dev/null
done
# Kill python ROS nodes (e.g. u_shape_obstacle_publisher.py)
pkill -9 -f "u_shape_obstacle" 2>/dev/null
pkill -9 -f "__name:=" 2>/dev/null
# Nuclear: kill anything from /opt/ros or /devel
ps aux | grep -E '/opt/ros/|/devel/|rosmaster|roscore|roslaunch|__name:=' | grep -v grep | grep -v "run_one_experiment" | grep -v "run_remaining" | grep -v "run_full_bench" | awk '{print $2}' | xargs kill -9 2>/dev/null
sleep 3
# Verify no rosmaster is running
if pgrep -x rosmaster >/dev/null 2>&1; then
    echo "[${EXP_NAME}] WARNING: rosmaster still running, force killing..."
    pkill -9 -x rosmaster 2>/dev/null
    sleep 2
fi
echo "[${EXP_NAME}] Pre-cleanup done"

if [[ "$METHOD_NAME" == "topo_mppi" ]]; then
    if ! nvidia-smi >/dev/null 2>&1; then
        echo "[${EXP_NAME}] ERROR: CUDA/GPU unavailable; refusing to run Topo-MPPI benchmark."
        echo "[${EXP_NAME}]        Fix host/container NVIDIA runtime first, then rerun this experiment."
        echo "${EXP_NAME} | FAIL | gpu_unavailable | - | - | -" >> "$RESULT_FILE"
        exit 2
    fi
fi

# ── Step 1: Launch environment simulator ──
echo "[${EXP_NAME}] Launching simulator..."
source /opt/ros/noetic/setup.bash
source "$DDO_WS/devel/setup.bash"
if [ -f "$LAUNCH_DIR/$SIM_LAUNCH" ]; then
    roslaunch "$LAUNCH_DIR/$SIM_LAUNCH" &
else
    roslaunch ddo_benchmark "$SIM_LAUNCH" &
fi
SIM_PID=$!

# Env3/Env4 (dynamic obstacles) needs more startup time
if [[ "$SIM_LAUNCH" == *env3* ]] || [[ "$SIM_LAUNCH" == *env4* ]]; then
    sleep 18
else
    sleep 12
fi

# Check simulator health
echo "[${EXP_NAME}] Checking odom topic..."
if ! timeout 15 rostopic echo /visual_slam/odom -n 1 >/dev/null 2>&1; then
    echo "[${EXP_NAME}] ERROR: No odom topic! Aborting."
    echo "${EXP_NAME} | FAIL | no_odom | - | - | -" >> "$RESULT_FILE"
    kill -9 $SIM_PID 2>/dev/null
    wait $SIM_PID 2>/dev/null
    exit 1
fi
echo "[${EXP_NAME}] Simulator OK"

# ── Step 2: Launch planner ──
echo "[${EXP_NAME}] Launching planner..."
source /opt/ros/noetic/setup.bash
if [ -f "$ALGO_WS/devel/setup.bash" ]; then
    source "$ALGO_WS/devel/setup.bash"
else
    echo "[${EXP_NAME}] ERROR: Missing workspace setup: $ALGO_WS/devel/setup.bash"
    echo "${EXP_NAME} | FAIL | missing_setup | - | - | -" >> "$RESULT_FILE"
    kill -9 $SIM_PID 2>/dev/null
    wait $SIM_PID 2>/dev/null
    exit 1
fi

START_TIME=$(date +%s)
roslaunch "$LAUNCH_DIR/$PLANNER_LAUNCH" > "$LOG_FILE" 2>&1 &
PLAN_PID=$!
sleep 5  # Give planner time to initialize
PLANNER_DIED=0

# Record odometry for post-run trajectory metrics. This is intentionally
# written to a separate CSV so the legacy results_master.txt format stays stable.
rostopic echo -p /visual_slam/odom > "$ODOM_CSV" 2>/dev/null &
ODOM_PID=$!

# Collect paper-facing benchmark metrics in parallel with the legacy summary:
# completion time, executed path length, min obstacle distance, collision events,
# low-clearance exposure time, velocity/acceleration, and jerk smoothness.
if [[ "$METHOD_NAME" == "topo_mppi" ]] && [ -n "$ENV_NAME" ]; then
    PLANNING_TIMING_SRC="/tmp/topo_mppi_${ENV_NAME}_planning_timing.csv"
    CANDIDATE_QUALITY_SRC="/tmp/topo_mppi_${ENV_NAME}_candidate_quality.csv"
    : > "$PLANNING_TIMING_SRC"
    : > "$CANDIDATE_QUALITY_SRC"
fi
RUN_ID=$(date +%s)
rosparam delete /benchmark_node 2>/dev/null || true
rosparam set /benchmark_node/benchmark/method_name "$METHOD_NAME"
rosparam set /benchmark_node/benchmark/scenario_name "${ENV_NAME:-unknown}"
rosparam set /benchmark_node/benchmark/run_id "$RUN_ID"
rosparam set /benchmark_node/benchmark/timeout "$TIMEOUT"
rosparam set /benchmark_node/benchmark/goal_reach_threshold "$GOAL_THRESHOLD"
rosparam set /benchmark_node/benchmark/min_success_traj_length "$MIN_SUCCESS_TRAJ_LENGTH"
rosparam set /benchmark_node/benchmark/goal_x "$GOAL_X"
rosparam set /benchmark_node/benchmark/goal_y "$GOAL_Y"
rosparam set /benchmark_node/benchmark/goal_z "$GOAL_Z"
rosparam set /benchmark_node/benchmark/checkpoint_count "$CHECKPOINT_COUNT"
rosparam set /benchmark_node/benchmark/checkpoint_reach_threshold "$CHECKPOINT_THRESHOLD"
rosparam set /benchmark_node/benchmark/checkpoint0_x "$CHECKPOINT0_X"
rosparam set /benchmark_node/benchmark/checkpoint0_y "$CHECKPOINT0_Y"
rosparam set /benchmark_node/benchmark/checkpoint0_z "$CHECKPOINT0_Z"
rosparam set /benchmark_node/benchmark/checkpoint1_x "$CHECKPOINT1_X"
rosparam set /benchmark_node/benchmark/checkpoint1_y "$CHECKPOINT1_Y"
rosparam set /benchmark_node/benchmark/checkpoint1_z "$CHECKPOINT1_Z"
rosparam set /benchmark_node/benchmark/checkpoint2_x "$CHECKPOINT2_X"
rosparam set /benchmark_node/benchmark/checkpoint2_y "$CHECKPOINT2_Y"
rosparam set /benchmark_node/benchmark/checkpoint2_z "$CHECKPOINT2_Z"
rosparam set /benchmark_node/benchmark/collision_threshold "0.30"
rosparam set /benchmark_node/benchmark/collision_exit_threshold "0.40"
rosparam set /benchmark_node/benchmark/safety_margin "0.65"
rosparam set /benchmark_node/benchmark/dynamic_safety_margin "0.65"
rosparam set /benchmark_node/benchmark/auto_start "true"
rosparam set /benchmark_node/benchmark/output_dir "$RESULT_DIR"
rosparam set /benchmark_node/benchmark/output_file "benchmark_results_extended.csv"
rosparam set /benchmark_node/benchmark/odom_topic "/visual_slam/odom"
rosparam set /benchmark_node/benchmark/cloud_topic "/pcl_render_node/cloud"
rosparam set /benchmark_node/benchmark/trigger_topic "/traj_start_trigger"
rosparam set /benchmark_node/benchmark/dynamic_obstacle_topic "/dynamic_obstacles/state"
source /opt/ros/noetic/setup.bash
source "$DDO_WS/devel/setup.bash"
rosrun ddo_benchmark benchmark_node __name:=benchmark_node > "$BENCH_LOG" 2>&1 &
BENCH_PID=$!
sleep 1
if ! kill -0 $BENCH_PID 2>/dev/null; then
    echo "[${EXP_NAME}] WARNING: benchmark_node failed to start; see $BENCH_LOG"
fi

# ── Step 2b: Fast-Planner trigger ──
# Fast-Planner's FSM needs waypoint_generator to publish a waypoint
# before it transitions out of INIT. DDO/EGO handle PRESET_TARGET
# internally, but Fast-Planner requires an external trigger.
# We send a /move_base_simple/goal which triggers goal_callback in
# waypoint_generator, which in manual-lonely-waypoint mode publishes
# the goal to /waypoint_generator/waypoints, which triggers
# waypointCallback in topo_replan_fsm.
# Also, Fast-Planner processes one waypoint at a time in PRESET mode,
# so we need to re-trigger each time it reaches a waypoint (returns to
# WAIT_TARGET). We run a background trigger loop for this.
if [[ "$EXP_NAME" == fast_* ]]; then
    echo "[${EXP_NAME}] Fast-Planner detected: starting trigger loop"
    (
        # Initial delay for planner to fully initialize and enter WAIT_TARGET
        sleep 5
        PREV_WP_COUNT=-1
        while kill -0 $PLAN_PID 2>/dev/null; do
            # Check how many waypoint completions so far
            CUR_WP_COUNT=$(grep -c "from EXEC_TRAJ to WAIT_TARGET" "$LOG_FILE" 2>/dev/null | tail -1)
            CUR_WP_COUNT=${CUR_WP_COUNT:-0}
            
            # Only trigger if:
            # 1) Never triggered before (PREV_WP_COUNT == -1, initial trigger), OR
            # 2) A new WAIT_TARGET was reached (more than before)
            if [ "$CUR_WP_COUNT" -gt "$PREV_WP_COUNT" ] || [ "$PREV_WP_COUNT" -eq -1 ]; then
                # Check FSM is actually in WAIT_TARGET or INIT before triggering
                LAST_STATE=$(grep -oE "from [A-Z_]+ to [A-Z_]+" "$LOG_FILE" 2>/dev/null | tail -1 | grep -oE "to [A-Z_]+" | awk '{print $2}')
                FSM_STATE=$(grep -oE "state: [A-Z_]+" "$LOG_FILE" 2>/dev/null | tail -1 | awk '{print $2}')
                
                if [ "$PREV_WP_COUNT" -eq -1 ] || [ "$LAST_STATE" = "WAIT_TARGET" ] || [ "$FSM_STATE" = "WAIT_TARGET" ] || [ "$FSM_STATE" = "INIT" ]; then
                    echo "[${EXP_NAME}] Sending trigger (wp_done=$CUR_WP_COUNT)"
                    rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
                      "{ header: { frame_id: 'world' }, pose: { position: { x: 0, y: 0, z: 1.0 }, orientation: { w: 1.0 } } }" 2>/dev/null
                    PREV_WP_COUNT=$CUR_WP_COUNT
                    # Wait longer to let it start executing before checking again
                    sleep 8
                    continue
                fi
            fi
            sleep 3
        done
    ) &
    TRIGGER_PID=$!
    sleep 10  # Wait for first trigger to take effect
fi

# ── Step 3: Monitor ──
echo "[${EXP_NAME}] Monitoring..."
ELAPSED=0
REACHED=0

# Fast-Planner processes waypoints one-by-one in PRESET mode,
# so it needs N "EXEC_TRAJ to WAIT_TARGET" transitions for N waypoints.
# DDO/EGO plan global traj through all waypoints at once, needing only 1.
if [[ "$EXP_NAME" == fast_* ]]; then
    REQUIRED_WP_TRANSITIONS=3
else
    REQUIRED_WP_TRANSITIONS=1
fi

while [ $ELAPSED -lt $TIMEOUT ]; do
    sleep 5
    ELAPSED=$((ELAPSED + 5))
    
    # Check planner alive
    if ! kill -0 $PLAN_PID 2>/dev/null; then
        echo "[${EXP_NAME}] Planner died at ${ELAPSED}s"
        PLANNER_DIED=1
        break
    fi

    # For loop/long-route environments, benchmark_node is authoritative
    # because it checks path length and ordered checkpoints. FSM transition
    # logs can fire after the first waypoint in routes whose first and final
    # waypoint share the same position.
    if [ "$REQUIRE_BENCHMARK_COMPLETION" -eq 1 ] && [ -n "$BENCH_PID" ] &&
       ! kill -0 $BENCH_PID 2>/dev/null; then
        wait $BENCH_PID 2>/dev/null
        BENCH_PID=""
        LIVE_BENCH_ROW=""
        LIVE_BENCH_SUCCESS=""
        if [ -f "$RESULT_DIR/benchmark_results_extended.csv" ]; then
            LIVE_BENCH_ROW=$(awk -F, -v method="$METHOD_NAME" -v env="${ENV_NAME:-unknown}" -v run="$RUN_ID" \
                '$1 == method && $2 == env && $3 == run { row=$0 } END { print row }' \
                "$RESULT_DIR/benchmark_results_extended.csv")
            if [ -n "$LIVE_BENCH_ROW" ]; then
                LIVE_BENCH_SUCCESS=$(printf "%s\n" "$LIVE_BENCH_ROW" | awk -F, '{print $4}')
            fi
        fi
        if [ "$LIVE_BENCH_SUCCESS" = "1" ]; then
            echo "[${EXP_NAME}] Benchmark node reports SUCCESS at ${ELAPSED}s."
            REACHED=1
            break
        elif [ -n "$LIVE_BENCH_ROW" ]; then
            echo "[${EXP_NAME}] Benchmark node finished with success=0 at ${ELAPSED}s; stopping monitor."
            REACHED=0
            break
        else
            echo "[${EXP_NAME}] Benchmark node exited without a matching row; falling back to legacy monitor."
            REQUIRE_BENCHMARK_COMPLETION=0
        fi
    fi
    
    # Check for mission complete
    # DDO/EGO/Fast: "from EXEC_TRAJ to WAIT_TARGET"
    WP_COUNT=$(grep -c "from EXEC_TRAJ to WAIT_TARGET" "$LOG_FILE" 2>/dev/null | tail -1)
    WP_COUNT=${WP_COUNT:-0}
    # V15: Also count NEAR_GOAL arrivals (stuck detection declares arrival when near goal)
    NEAR_GOAL_COUNT=$(grep -c "Declaring arrival" "$LOG_FILE" 2>/dev/null | tail -1)
    NEAR_GOAL_COUNT=${NEAR_GOAL_COUNT:-0}
    TOTAL_WP=$((WP_COUNT + NEAR_GOAL_COUNT))
    if [ "$TOTAL_WP" -ge "$REQUIRED_WP_TRANSITIONS" ]; then
        CUR_X=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "x:" | awk '{printf "%.2f", $2}' || echo "?")
        CUR_Y=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "y:" | awk '{printf "%.2f", $2}' || echo "?")
        CUR_Z=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "z:" | awk '{printf "%.2f", $2}' || echo "?")
        GOAL_DIST=$(calc_goal_distance "$CUR_X" "$CUR_Y" "$CUR_Z")
        if [ "$REQUIRE_BENCHMARK_COMPLETION" -eq 1 ]; then
            echo "[${EXP_NAME}] Completion marker seen, waiting for benchmark route validation."
        elif within_goal_threshold "$GOAL_DIST" && [ "$ELAPSED" -lt "$MIN_COMPLETION_ELAPSED" ]; then
            echo "[${EXP_NAME}] Completion marker seen at ${ELAPSED}s but min route elapsed is ${MIN_COMPLETION_ELAPSED}s; continuing."
        elif within_goal_threshold "$GOAL_DIST"; then
            echo "[${EXP_NAME}] ★ COMPLETE at ${ELAPSED}s (dist=${GOAL_DIST}m, waypoint transitions: $WP_COUNT+${NEAR_GOAL_COUNT}near / $REQUIRED_WP_TRANSITIONS)"
            REACHED=1
            break
        else
            echo "[${EXP_NAME}] Completion marker seen but final goal is still ${GOAL_DIST}m away; continuing."
        fi
    fi
    
    # Print progress every 15s
    if [ $((ELAPSED % 15)) -eq 0 ]; then
        CUR_X=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "x:" | awk '{printf "%.1f", $2}' || echo "?")
        CUR_Y=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "y:" | awk '{printf "%.1f", $2}' || echo "?")
        if [[ "$EXP_NAME" == tgk_* ]]; then
            WP_SO_FAR=$(grep -c "Reached waypoint" "$LOG_FILE" 2>/dev/null | tail -1)
            WP_SO_FAR=${WP_SO_FAR:-0}
            # Count total waypoints from goal sender init message
            WP_TOTAL=$(grep -oP '\d+ waypoints' "$LOG_FILE" 2>/dev/null | head -1 | grep -oP '^\d+')
            WP_TOTAL=${WP_TOTAL:-"?"}
            NEAR_SO_FAR=0
            echo "[${EXP_NAME}] ${ELAPSED}s: pos=(${CUR_X}, ${CUR_Y}) wp=${WP_SO_FAR}/${WP_TOTAL}"
        else
            WP_SO_FAR=$(grep -c "from EXEC_TRAJ to WAIT_TARGET" "$LOG_FILE" 2>/dev/null | tail -1)
            WP_SO_FAR=${WP_SO_FAR:-0}
            NEAR_SO_FAR=$(grep -c "Declaring arrival" "$LOG_FILE" 2>/dev/null | tail -1)
            NEAR_SO_FAR=${NEAR_SO_FAR:-0}
            WP_TOTAL_SO_FAR=$((WP_SO_FAR + NEAR_SO_FAR))
            CUR_Z=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "z:" | awk '{printf "%.1f", $2}' || echo "?")
            GOAL_DIST=$(calc_goal_distance "$CUR_X" "$CUR_Y" "$CUR_Z")
            echo "[${EXP_NAME}] ${ELAPSED}s: pos=(${CUR_X}, ${CUR_Y}) dist=${GOAL_DIST}m wp=${WP_TOTAL_SO_FAR}/${REQUIRED_WP_TRANSITIONS}"
        fi
    fi
done

END_TIME=$(date +%s)
TOTAL_TIME=$((END_TIME - START_TIME))

# ── Step 4: Get final position ──
FINAL_X=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "x:" | awk '{printf "%.2f", $2}' || echo "?")
FINAL_Y=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "y:" | awk '{printf "%.2f", $2}' || echo "?")
FINAL_Z=$(timeout 2 rostopic echo /visual_slam/odom -n 1 2>/dev/null | grep -A3 "position:" | grep "z:" | awk '{printf "%.2f", $2}' || echo "?")
FINAL_GOAL_DIST=$(calc_goal_distance "$FINAL_X" "$FINAL_Y" "$FINAL_Z")

kill -9 $ODOM_PID 2>/dev/null
wait $ODOM_PID 2>/dev/null

if [ -n "$BENCH_PID" ] && kill -0 $BENCH_PID 2>/dev/null; then
    kill -2 $BENCH_PID 2>/dev/null
    sleep 2
fi
if [ -n "$BENCH_PID" ] && kill -0 $BENCH_PID 2>/dev/null; then
    kill -9 $BENCH_PID 2>/dev/null
fi
if [ -n "$BENCH_PID" ]; then
    wait $BENCH_PID 2>/dev/null
fi

# ── Step 5: Collect metrics ──
if [[ "$EXP_NAME" == tgk_* ]]; then
    # TGK-specific metrics
    REPLAN_COUNT=$(grep -cE "to REPLAN_TRAJ|to GENERATE_TRAJ" "$LOG_FILE" 2>/dev/null | tail -1)
    REPLAN_COUNT=${REPLAN_COUNT:-0}
    COL_WARN=$(grep -cE "about to collide|EMERGENCY" "$LOG_FILE" 2>/dev/null | tail -1)
    COL_WARN=${COL_WARN:-0}
    TOPO_FAIL=0  # TGK uses kRRT*, no separate topo phase
    BSPLINE_FAIL=0  # TGK uses polynomial traj, no B-spline
else
    REPLAN_COUNT=$(grep -cE "rebo replan|GEN_NEW_TRAJ|new_traj" "$LOG_FILE" 2>/dev/null | tail -1)
    REPLAN_COUNT=${REPLAN_COUNT:-0}
    COL_WARN=$(grep -cE "terminal point.*obstacle|in obstacle" "$LOG_FILE" 2>/dev/null | tail -1)
    COL_WARN=${COL_WARN:-0}
    TOPO_FAIL=$(grep -cE "No paths found|Topological planning failed" "$LOG_FILE" 2>/dev/null | tail -1)
    TOPO_FAIL=${TOPO_FAIL:-0}
    BSPLINE_FAIL=$(grep -cE "B-spline.*FAILED|B-spline failed" "$LOG_FILE" 2>/dev/null | tail -1)
	    BSPLINE_FAIL=${BSPLINE_FAIL:-0}
	fi

BENCH_ROW=""
BENCH_SUCCESS=""
if [ -f "$RESULT_DIR/benchmark_results_extended.csv" ]; then
    BENCH_ROW=$(awk -F, -v method="$METHOD_NAME" -v env="${ENV_NAME:-unknown}" -v run="$RUN_ID" \
        '$1 == method && $2 == env && $3 == run { row=$0 } END { print row }' \
        "$RESULT_DIR/benchmark_results_extended.csv")
    if [ -n "$BENCH_ROW" ]; then
        BENCH_SUCCESS=$(printf "%s\n" "$BENCH_ROW" | awk -F, '{print $4}')
    fi
fi

STATUS="TIMEOUT"
if [ "$PLANNER_DIED" -eq 1 ]; then
    STATUS="FAIL"
elif [ "$REACHED" -eq 1 ] && [ "$REPLAN_COUNT" -gt 0 ]; then
    STATUS="SUCCESS"
elif [ "$REACHED" -eq 1 ]; then
    STATUS="FAIL"
    echo "[${EXP_NAME}] Completion marker seen with zero replans; marking invalid instead of success."
else
    REACHED=0
fi
if [ -n "$BENCH_SUCCESS" ] && [ "$BENCH_SUCCESS" != "1" ] && [ "$STATUS" = "SUCCESS" ]; then
    STATUS="FAIL"
    echo "[${EXP_NAME}] Benchmark success=0; overriding legacy SUCCESS to FAIL."
fi

# ── Step 6: Print & save ──
echo ""
echo "═══════════ RESULT: $EXP_NAME ═══════════"
echo "  Status:   $STATUS"
echo "  Time:     ${TOTAL_TIME}s"
echo "  Position: ($FINAL_X, $FINAL_Y, $FINAL_Z)"
echo "  GoalDist: ${FINAL_GOAL_DIST}m"
echo "  Replans:  $REPLAN_COUNT | ColWarn: $COL_WARN | TopoFail: $TOPO_FAIL | BsplineFail: $BSPLINE_FAIL"
if [ -n "$BENCH_ROW" ]; then
    echo "  Benchmark: $BENCH_ROW"
elif [ -f "$RESULT_DIR/benchmark_results_extended.csv" ]; then
    echo "  Benchmark: unavailable for run_id=$RUN_ID (see $BENCH_LOG)"
fi
echo "═══════════════════════════════════════════"

echo "${EXP_NAME} | ${STATUS} | ${TOTAL_TIME}s | pos=(${FINAL_X},${FINAL_Y},${FINAL_Z}) | goal_dist=${FINAL_GOAL_DIST} | replans=${REPLAN_COUNT} | col=${COL_WARN} | topo_fail=${TOPO_FAIL} | bs_fail=${BSPLINE_FAIL}" >> "$RESULT_FILE"

python3 - "$EXP_NAME" "$STATUS" "$TOTAL_TIME" "$ODOM_CSV" "$TRAJ_METRIC_FILE" <<'PY'
import csv
import math
import sys

exp_name, status, total_time, odom_csv, metric_file = sys.argv[1:6]

def find_col(header, suffix):
    for i, name in enumerate(header):
        if name.endswith(suffix):
            return i
    return None

rows = []
try:
    with open(odom_csv, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if not header:
            raise RuntimeError("empty odom csv")
        t_i = 0
        x_i = find_col(header, "pose.pose.position.x")
        y_i = find_col(header, "pose.pose.position.y")
        z_i = find_col(header, "pose.pose.position.z")
        if x_i is None or y_i is None or z_i is None:
            raise RuntimeError("position columns not found")
        for row in reader:
            try:
                t = float(row[t_i]) * 1e-9
                x = float(row[x_i])
                y = float(row[y_i])
                z = float(row[z_i])
            except (ValueError, IndexError):
                continue
            rows.append((t, x, y, z))
except Exception:
    rows = []

path_length = 0.0
max_speed = 0.0
max_acc = 0.0
valid_speeds = []
valid_accs = []
min_z = None
max_z = None
prev = None
prev_speed = None
max_valid_speed = 8.0
max_valid_acc = 30.0
for sample in rows:
    t, x, y, z = sample
    min_z = z if min_z is None else min(min_z, z)
    max_z = z if max_z is None else max(max_z, z)
    if prev is not None:
        pt, px, py, pz = prev
        dt = t - pt
        dist = math.sqrt((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2)
        if 1e-4 < dt < 0.2:
            speed = dist / dt
            speed_valid = math.isfinite(speed) and speed <= max_valid_speed and dist < 2.0
            if speed_valid and dist > 0.002:
                path_length += dist
            if speed_valid:
                valid_speeds.append(speed)
                max_speed = max(max_speed, speed)
                if prev_speed is not None:
                    acc = abs(speed - prev_speed) / dt
                    if math.isfinite(acc) and acc <= max_valid_acc:
                        valid_accs.append(acc)
                        max_acc = max(max_acc, acc)
                prev_speed = speed
            else:
                prev_speed = None
    prev = sample

duration = 0.0
if len(rows) >= 2:
    duration = max(0.0, rows[-1][0] - rows[0][0])
avg_speed = path_length / duration if duration > 1e-4 else 0.0

def fmt(value):
    if value is None:
        return "nan"
    return f"{value:.3f}"

with open(metric_file, "a", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        exp_name,
        status,
        total_time,
        fmt(path_length),
        fmt(avg_speed),
        fmt(max_speed),
        fmt(max_acc),
        fmt(min_z),
        fmt(max_z),
        len(rows),
        odom_csv,
    ])

print(
    f"  TrajMetrics: path={path_length:.2f}m avg_v={avg_speed:.2f}m/s "
    f"max_v={max_speed:.2f}m/s max_acc={max_acc:.2f}m/s2 samples={len(rows)}"
)
PY

if [[ "$METHOD_NAME" == "topo_mppi" ]] && [ -n "$ENV_NAME" ]; then
    TIMING_SRC="$PLANNING_TIMING_SRC"
    TIMING_DST="$RESULT_DIR/${EXP_NAME}_${TIMESTAMP}_planning_timing.csv"
    if [ -f "$TIMING_SRC" ]; then
        cp "$TIMING_SRC" "$TIMING_DST"
        echo "  PlanningTiming: $TIMING_DST"
        if python3 "$DDO_WS/src/benchmark/scripts/validate_planning_timing.py" \
            "$TIMING_DST" --allow-empty >/tmp/topo_mppi_timing_validate.out 2>&1; then
            echo "  PlanningTimingValidation: OK"
        else
            echo "  PlanningTimingValidation: INVALID"
            sed 's/^/    /' /tmp/topo_mppi_timing_validate.out
        fi
    else
        echo "  PlanningTiming: unavailable ($TIMING_SRC not found)"
    fi
    CANDIDATE_SRC="$CANDIDATE_QUALITY_SRC"
    CANDIDATE_DST="$RESULT_DIR/${EXP_NAME}_${TIMESTAMP}_candidate_quality.csv"
    if [ -f "$CANDIDATE_SRC" ]; then
        cp "$CANDIDATE_SRC" "$CANDIDATE_DST"
        echo "  CandidateQuality: $CANDIDATE_DST"
        if python3 "$DDO_WS/src/benchmark/scripts/validate_candidate_quality.py" \
            "$CANDIDATE_DST" --allow-empty >/tmp/topo_mppi_candidate_validate.out 2>&1; then
            echo "  CandidateQualityValidation: OK"
        else
            echo "  CandidateQualityValidation: INVALID"
            sed 's/^/    /' /tmp/topo_mppi_candidate_validate.out
        fi
    else
        echo "  CandidateQuality: unavailable ($CANDIDATE_SRC not found)"
    fi
fi

# ── Step 7: Kill everything ──
kill -9 $PLAN_PID 2>/dev/null
kill -9 $SIM_PID 2>/dev/null
[ -n "$TRIGGER_PID" ] && kill -9 $TRIGGER_PID 2>/dev/null
[ -n "$ODOM_PID" ] && kill -9 $ODOM_PID 2>/dev/null
[ -n "$BENCH_PID" ] && kill -9 $BENCH_PID 2>/dev/null
wait $PLAN_PID 2>/dev/null
wait $SIM_PID 2>/dev/null
[ -n "$TRIGGER_PID" ] && wait $TRIGGER_PID 2>/dev/null
[ -n "$ODOM_PID" ] && wait $ODOM_PID 2>/dev/null
[ -n "$BENCH_PID" ] && wait $BENCH_PID 2>/dev/null

# Kill ALL ROS-related processes aggressively
killall -9 rosmaster roscore roslaunch rosout 2>/dev/null
sleep 1
# Kill by pattern: anything with 'ros' or known node names
for proc in simulator_node random_forest_sensing ego_planner_node \
    traj_server waypoint_generator ddo_planner_node \
    fast_planner_node so3_quadrotor_simulator_node quadrotor_simulator_so3 \
    odom_visualization mockamap_node dynamic_obstacle_generator \
    cloud_merger cloud_merger_node pcl_render_node nodelet random_forest \
    state_machine_node traj_server_node benchmark_node; do
    killall -9 "$proc" 2>/dev/null
done
# Kill python ROS nodes
pkill -9 -f "u_shape_obstacle" 2>/dev/null
pkill -9 -f "__name:=" 2>/dev/null
sleep 1
# Nuclear option: kill any process with 'ros' or ROS node markers (except our own shell)
ps aux | grep -E '/opt/ros/|/devel/|rosmaster|roscore|roslaunch|__name:=' | grep -v grep | grep -v "run_one_experiment" | grep -v "run_remaining" | grep -v "run_full_bench" | awk '{print $2}' | xargs kill -9 2>/dev/null
sleep 3

echo "[${EXP_NAME}] Done. Cleaned up."
