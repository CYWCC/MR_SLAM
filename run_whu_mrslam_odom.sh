#!/usr/bin/env bash
###############################################################################
# MR_SLAM 多机SLAM（直接播放前端bag，不启动FAST-LIO前端）
# 目标：
#   1) 播放每个机器人前端输出bag（odom + cloud）
#   2) 启动 python relay：发布 /robot_i/Odometry（relay）、/robot_i/cloud_registered（世界系）、/robot_i/submap
#   3) 启动回环、global_manager、可视化
#
# 修复点（相对你贴的脚本）：
#   - WORLD_FRAME/BODY_FRAME 不再用全局固定值，改为按机器人设置，和原 C++ 一致：
#       world_frame = /robot_i/odom
#       body_frame  = /robot_i/${SENSOR_NAME}
#   - 默认 SENSOR_NAME="velodyne"（你按实际改），自动拼 frame：/robot_i/velodyne
#   - 增加基本检查：bag文件、relay脚本存在性
###############################################################################
set -e

# ====================== 路径配置 ======================

MRSLAM_ROOT="/home"
MAPPING_WS="${MRSLAM_ROOT}/Mapping"
LOOPDETECTION_WS="${MRSLAM_ROOT}/LoopDetection"
COSTMAP_WS="${MRSLAM_ROOT}/Costmap"
TOOLS_DIR="${MRSLAM_ROOT}/Tools"
VIS_DIR="${MRSLAM_ROOT}/Visualization"

# ====================== relay 脚本路径（请按实际修改） ======================
RELAY_PY="/home/cyw_local/MR_SLAM/Tools/relay_odom_pc2.py"

# ====================== bag路径（请根据实际情况修改） ======================
BAG0="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot0.bag"
BAG1="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot1.bag"
BAG2="/media/cyw/KESU/mapping_data/MRDR_Odom_data/group1/robot2.bag"

NUM_ROBOTS=3
LOOP_DETECTION_METHOD="scancontext"
ENABLE_ELEVATION_MAPPING=false
ENABLE_COSTMAP=false

# ====================== relay 参数（可按需修改） ======================
DIST_TH="1.5"
VOXEL_LEAF="0.2"

# 关键：这里请填你原 C++ 里 SensorName（去掉前导 / 也可以）
# 例如：velodyne / livox / ouster / base_link 等
SENSOR_NAME="velodyne"

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

    if ! command -v tmux >/dev/null 2>&1; then
        log_error "tmux 未安装，请先安装 tmux"
        exit 1
    fi

    if ! tmux has-session -t "$session_name" 2>/dev/null; then
        tmux new-session -d -s "$session_name" -n "$window_name"
        tmux send-keys -t "${session_name}:${window_name}" "$command" C-m
    else
        tmux new-window -t "$session_name" -n "$window_name"
        tmux send-keys -t "${session_name}:${window_name}" "$command" C-m
    fi
}

ensure_file() {
    local f=$1
    local name=$2
    if [ ! -f "$f" ]; then
        log_error "$name 不存在: $f"
        exit 1
    fi
}

# 拼出与 C++ 一致的 frame：
#   world_frame = /robot_i/odom
#   body_frame  = /robot_i/${SENSOR_NAME}  （C++里是 NameSpace + SensorName）
make_world_frame() {
    local rid=$1
    echo "/robot_${rid}/odom"
}
make_body_frame() {
    local rid=$1
    local sn="$SENSOR_NAME"
    sn="${sn#/}"  # 去掉可能的前导 /
    echo "/robot_${rid}/${sn}"
}

# ====================== 启动函数 ======================
cleanup() {
    pkill -f roscore 2>/dev/null || true
    pkill -f roslaunch 2>/dev/null || true
    pkill -f rosbag 2>/dev/null || true
    pkill -f simple_submap_relay 2>/dev/null || true
    pkill -f "${RELAY_PY}" 2>/dev/null || true
    tmux kill-session -t jackal_mrslam 2>/dev/null || true
    sleep 1
}

start_roscore() {
    log_step "启动 roscore..."
    run_in_tmux "jackal_mrslam" "roscore" "roscore"
    sleep 3
}

# bag 播放输出到 /robot_i/Odometry_raw 和 /robot_i/cloud_registered_body
start_rosbags() {
    log_step "播放3个前端bag并重映射topic(输出到 *_raw / cloud_registered_body)..."

    ensure_file "$BAG0" "BAG0"
    ensure_file "$BAG1" "BAG1"
    ensure_file "$BAG2" "BAG2"

    run_in_tmux "jackal_mrslam" "bag0" \
        "rosbag play $BAG0 --clock \
         /robot_0/odom:=/robot_1/Odometry_raw \
         /robot_0/full_cloud:=/robot_1/cloud_registered_body"

    run_in_tmux "jackal_mrslam" "bag1" \
        "rosbag play $BAG1 --clock \
         /robot_1/odom:=/robot_2/Odometry_raw \
         /robot_1/full_cloud:=/robot_2/cloud_registered_body"

    run_in_tmux "jackal_mrslam" "bag2" \
        "rosbag play $BAG2 --clock \
         /robot_2/odom:=/robot_3/Odometry_raw \
         /robot_2/full_cloud:=/robot_3/cloud_registered_body"

    sleep 2
}

start_relays() {
    log_step "启动 relay 节点：生成 /robot_i/Odometry /robot_i/cloud_registered /robot_i/submap ..."
    ensure_file "$RELAY_PY" "RELAY_PY"

    local SRC_CMD="source $MAPPING_WS/devel/setup.bash"

    # robot_1
    local WF1; WF1="$(make_world_frame 1)"
    local BF1; BF1="$(make_body_frame 1)"
    run_in_tmux "jackal_mrslam" "relay1" \
        "$SRC_CMD && python3 $RELAY_PY \
         --odom_topic /robot_1/Odometry_raw \
         --cloud_body_topic /robot_1/cloud_registered_body \
         --cloud_world_topic /robot_1/cloud_registered \
         --odom_relay_topic /robot_1/Odometry \
         --submap_topic /robot_1/submap \
         --world_frame $WF1 \
         --body_frame $BF1 \
         --distance_thresh $DIST_TH \
         --voxel_leaf $VOXEL_LEAF"

    # robot_2
    local WF2; WF2="$(make_world_frame 2)"
    local BF2; BF2="$(make_body_frame 2)"
    run_in_tmux "jackal_mrslam" "relay2" \
        "$SRC_CMD && python3 $RELAY_PY \
         --odom_topic /robot_2/Odometry_raw \
         --cloud_body_topic /robot_2/cloud_registered_body \
         --cloud_world_topic /robot_2/cloud_registered \
         --odom_relay_topic /robot_2/Odometry \
         --submap_topic /robot_2/submap \
         --world_frame $WF2 \
         --body_frame $BF2 \
         --distance_thresh $DIST_TH \
         --voxel_leaf $VOXEL_LEAF"

    # robot_3
    local WF3; WF3="$(make_world_frame 3)"
    local BF3; BF3="$(make_body_frame 3)"
    run_in_tmux "jackal_mrslam" "relay3" \
        "$SRC_CMD && python3 $RELAY_PY \
         --odom_topic /robot_3/Odometry_raw \
         --cloud_body_topic /robot_3/cloud_registered_body \
         --cloud_world_topic /robot_3/cloud_registered \
         --odom_relay_topic /robot_3/Odometry \
         --submap_topic /robot_3/submap \
         --world_frame $WF3 \
         --body_frame $BF3 \
         --distance_thresh $DIST_TH \
         --voxel_leaf $VOXEL_LEAF"

    log_config "Relay frames:"
    log_config "  robot_1: world_frame=$WF1  body_frame=$BF1"
    log_config "  robot_2: world_frame=$WF2  body_frame=$BF2"
    log_config "  robot_3: world_frame=$WF3  body_frame=$BF3"

    sleep 2
}

start_loop_detection() {
    log_step "启动循环检测..."
    source_workspace "$LOOPDETECTION_WS"
    local WORK_DIR="/tmp/loop_detection"
    run_in_tmux "jackal_mrslam" "loop_detect" \
        "source $LOOPDETECTION_WS/devel/setup.bash && mkdir -p $WORK_DIR && cd $WORK_DIR && python3 $LOOPDETECTION_WS/src/RING_ros/main_SC.py"
    sleep 2
}

start_global_manager() {
    log_step "启动 global_manager..."
    source_workspace "$MAPPING_WS"
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
    log_info "    WHU Multi-Robot SLAM 完整模式(前端bag + relay + 后端)"
    log_info "============================================"
    log_config "Bag 文件: $BAG0, $BAG1, $BAG2"
    log_config "机器人数量: $NUM_ROBOTS"
    log_config "回环检测: $LOOP_DETECTION_METHOD"
    log_config "Relay 脚本: $RELAY_PY"
    log_config "Relay 参数: dis_th=$DIST_TH voxel=$VOXEL_LEAF SENSOR_NAME=$SENSOR_NAME"
    log_info "============================================"

    cleanup
    check_ros

    start_roscore
    start_rosbags
    start_relays
    start_loop_detection
    start_global_manager
    start_visualization

    log_info "============================================"
    log_info "         系统启动完成!"
    log_info "============================================"
    log_info ""
    log_info "rosbag 已自动开始播放（输出到 /robot_i/Odometry_raw 和 /robot_i/cloud_registered_body）"
    log_info "relay 会发布：/robot_i/Odometry  /robot_i/cloud_registered  /robot_i/submap"
    log_info "frame 对齐原 C++：world_frame=/robot_i/odom  body_frame=/robot_i/${SENSOR_NAME}"
    log_info "使用 tmux attach -t jackal_mrslam 查看各个终端"
    log_info ""
    log_info "停止运行: bash $0 stop"
}

show_help() {
    echo "WHU Multi-Robot SLAM 启动脚本（前端bag + relay + 后端）"
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  run         运行完整的多机器人 SLAM 系统"
    echo "  stop        停止所有节点"
    echo "  help        显示此帮助信息"
    echo ""
    echo "你需要修改的配置:"
    echo "  BAG0/BAG1/BAG2  - rosbag 文件路径"
    echo "  RELAY_PY        - python relay 脚本路径"
    echo "  SENSOR_NAME     - 对齐原 C++ 的 SensorName（例如 velodyne / livox / ouster）"
    echo ""
    echo "Topic 映射（bag输出）:"
    echo "  /robot_0/odom       -> /robot_1/Odometry_raw"
    echo "  /robot_0/full_cloud -> /robot_1/cloud_registered_body"
    echo "  /robot_1/odom       -> /robot_2/Odometry_raw"
    echo "  /robot_1/full_cloud -> /robot_2/cloud_registered_body"
    echo "  /robot_2/odom       -> /robot_3/Odometry_raw"
    echo "  /robot_2/full_cloud -> /robot_3/cloud_registered_body"
    echo ""
    echo "Relay 输出（后端使用）:"
    echo "  /robot_i/Odometry"
    echo "  /robot_i/cloud_registered"
    echo "  /robot_i/submap"
    echo ""
    echo "Frame 对齐（与原 C++ 一致）:"
    echo "  world_frame = /robot_i/odom"
    echo "  body_frame  = /robot_i/${SENSOR_NAME}"
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
