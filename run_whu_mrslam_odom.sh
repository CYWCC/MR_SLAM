#!/usr/bin/env bash
###############################################################################
# MR_SLAM 多机SLAM（直接播放前端bag，不启动FAST-LIO前端）
# 自动生成脚本，适用于已录制好每个机器人前端输出的bag
###############################################################################
set -e

# ====================== 路径配置 ======================
MAPPING_WS="/home/Mapping"
LOOPDETECTION_WS="/home/LoopDetection"
COSTMAP_WS="/home/Costmap"
TOOLS_DIR="/home/Tools"
VIS_DIR="/home/Visualization"

# ====================== bag路径（请根据实际情况修改） ======================
BAG0="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot0.bag"
BAG1="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot1.bag"
BAG2="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot2.bag"

NUM_ROBOTS=3
LOOP_DETECTION_METHOD="scancontext"
ENABLE_ELEVATION_MAPPING=false
ENABLE_COSTMAP=false

log_info() { echo -e "\033[0;32m[INFO]\033[0m $1"; }
log_step() { echo -e "\033[0;34m[STEP]\033[0m $1"; }

run_in_tmux() {
    local session_name=$1
    local window_name=$2
    local command=$3
    if ! tmux has-session -t "$session_name" 2>/dev/null; then
        tmux new-session -d -s "$session_name" -n "$window_name"
        tmux send-keys -t "${session_name}:${window_name}" "$command" C-m
    else
        tmux new-window -t "$session_name" -n "$window_name"
        tmux send-keys -t "${session_name}:${window_name}" "$command" C-m
    fi
}

cleanup() {
    pkill -f roscore 2>/dev/null || true
    pkill -f roslaunch 2>/dev/null || true
    pkill -f rosbag 2>/dev/null || true
    tmux kill-session -t jackal_mrslam 2>/dev/null || true
    sleep 1
}

start_roscore() {
    log_step "启动 roscore..."
    run_in_tmux "jackal_mrslam" "roscore" "roscore"
    sleep 3
}

start_rosbags() {
    log_step "播放3个前端bag并重映射topic..."
    run_in_tmux "jackal_mrslam" "bag0" "rosbag play $BAG0 --clock /robot_0/odom:=/robot_1/Odometry /robot_0/full_cloud:=/robot_1/cloud_registered"
    run_in_tmux "jackal_mrslam" "bag1" "rosbag play $BAG1 --clock /robot_1/odom:=/robot_2/Odometry /robot_1/full_cloud:=/robot_2/cloud_registered"
    run_in_tmux "jackal_mrslam" "bag2" "rosbag play $BAG2 --clock /robot_2/odom:=/robot_3/Odometry /robot_2/full_cloud:=/robot_3/cloud_registered"
    sleep 2
}

start_loop_detection() {
    log_step "启动循环检测..."
    source $LOOPDETECTION_WS/devel/setup.bash
    local WORK_DIR="/tmp/loop_detection"
    run_in_tmux "jackal_mrslam" "loop_detect" \
        "source $LOOPDETECTION_WS/devel/setup.bash && mkdir -p $WORK_DIR && cd $WORK_DIR && python3 $LOOPDETECTION_WS/src/RING_ros/main_SC.py"
    sleep 2
}

start_global_manager() {
    log_step "启动 global_manager..."
    source $MAPPING_WS/devel/setup.bash
    run_in_tmux "jackal_mrslam" "global_mgr" \
        "source $MAPPING_WS/devel/setup.bash && roslaunch global_manager global_manager.launch"
    sleep 2
}

start_visualization() {
    log_step "启动可视化..."
    local JACKAL_VIS="/home/cyw_local/MR_SLAM/Visualization/vis_jackal.rviz"
    if [ -f "$JACKAL_VIS" ]; then
        run_in_tmux "jackal_mrslam" "rviz" "rviz -d ${JACKAL_VIS}"
    else
        run_in_tmux "jackal_mrslam" "rviz" "rviz -d ${VIS_DIR}/vis.rviz"
    fi
}

main() {
    cleanup
    start_roscore
    start_rosbags
    start_loop_detection
    start_global_manager
    start_visualization
    log_info "系统启动完成！"
    log_info "使用 tmux attach -t jackal_mrslam 查看各终端"
}

main "$@"
