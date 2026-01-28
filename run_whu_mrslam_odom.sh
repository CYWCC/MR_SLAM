#!/usr/bin/env bash
###############################################################################
# MR_SLAM 多机SLAM（直接播放前端bag，不启动FAST-LIO前端）
# 自动生成脚本，适用于已录制好每个机器人前端输出的bag
###############################################################################
set -e

# ====================== 路径配置 ======================

MRSLAM_ROOT="/home"
MAPPING_WS="${MRSLAM_ROOT}/Mapping"
LOOPDETECTION_WS="${MRSLAM_ROOT}/LoopDetection"
COSTMAP_WS="${MRSLAM_ROOT}/Costmap"
TOOLS_DIR="${MRSLAM_ROOT}/Tools"
VIS_DIR="${MRSLAM_ROOT}/Visualization"

# ====================== bag路径（请根据实际情况修改） ======================
BAG0="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot0.bag"
BAG1="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot1.bag"
BAG2="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot2.bag"

NUM_ROBOTS=3
LOOP_DETECTION_METHOD="scancontext"
ENABLE_ELEVATION_MAPPING=false
ENABLE_COSTMAP=false

# ====================== 颜色定义 ======================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step() { echo -e "${BLUE}[STEP]${NC} $1"; }
log_config() { echo -e "${CYAN}[CONFIG]${NC} $1"; }

check_ros() {
    if [ -z "$ROS_DISTRO" ]; then
        log_info "Sourcing ROS setup..."
        source /opt/ros/noetic/setup.bash 2>/dev/null || \
        source /opt/ros/melodic/setup.bash 2>/dev/null || \
        source /opt/ros/kinetic/setup.bash
    fi
    log_info "ROS_DISTRO: $ROS_DISTRO"
}

source_workspace() {
    local ws_path=$1
    if [ -f "${ws_path}/devel/setup.bash" ]; then
        source "${ws_path}/devel/setup.bash"
    else
        log_warn "工作空间 ${ws_path}/devel/setup.bash 未找到"
    fi
}

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

# ====================== 启动函数 ======================
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

run_full_mode() {
    log_info "============================================"
    log_info "    WHU Multi-Robot SLAM 完整模式"
    log_info "============================================"
    log_config "Bag 文件: $BAG0, $BAG1, $BAG2"
    log_config "机器人数量: $NUM_ROBOTS"
    log_config "回环检测: $LOOP_DETECTION_METHOD"
    log_info "============================================"
    
    cleanup
    check_ros
    
    start_roscore
    start_rosbags
    start_loop_detection
    start_global_manager
    start_visualization
    
    log_info "============================================"
    log_info "         系统启动完成!"
    log_info "============================================"
    log_info ""
    log_info "rosbag 已自动开始播放"
    log_info "使用 tmux attach -t jackal_mrslam 查看各个终端"
    log_info ""
    log_info "停止运行: bash $0 stop"
}

show_help() {
    echo "WHU Multi-Robot SLAM 启动脚本"
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  run         运行完整的多机器人 SLAM 系统"
    echo "  stop        停止所有节点"
    echo "  help        显示此帮助信息"
    echo ""
    echo "配置说明:"
    echo "  请在脚本中修改以下变量:"
    echo "    BAG0/BAG1/BAG2             - 你的 rosbag 文件路径"
    echo "    LOOP_DETECTION_METHOD      - 回环检测方法"
    echo ""
    echo "Topic 映射:"
    echo "  /robot_0/odom -> /robot_1/Odometry"
    echo "  /robot_0/full_cloud -> /robot_1/cloud_registered"
    echo "  /robot_1/odom -> /robot_2/Odometry"
    echo "  /robot_1/full_cloud -> /robot_2/cloud_registered"
    echo "  /robot_2/odom -> /robot_3/Odometry"
    echo "  /robot_2/full_cloud -> /robot_3/cloud_registered"
    echo ""
    echo "示例:"
    echo "  $0 run"
}

main() {
    case "${1:-help}" in
        "run")
            run_full_mode
            ;;
        "stop")
            cleanup
            log_info "所有节点已停止"
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            log_error "未知命令: $1"
            show_help
            exit 1
            ;;
    esac
}

main "$@"
