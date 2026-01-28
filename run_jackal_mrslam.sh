#!/usr/bin/env bash
###############################################################################
# MR_SLAM Jackal 多机器人数据处理脚本
# 
# 该脚本用于处理包含多个 Jackal 机器人数据的单个 rosbag 文件
# 通过 topic 重映射将 Jackal 的 topic 格式转换为 MR_SLAM 期望的格式
#
# 你的 bag topics:
#   /jackal0/livox/lidar  -> /robot_1/pointcloud
#   /jackal0/livox/imu    -> /robot_1/imu
#   /jackal1/livox/lidar  -> /robot_2/pointcloud
#   /jackal1/livox/imu    -> /robot_2/imu
#   /jackal2/livox/lidar  -> /robot_3/pointcloud
#   /jackal2/livox/imu    -> /robot_3/imu
#
# 使用方法:
#   1. 首先启动 Docker 容器: bash run_mrslam_docker.sh
#   2. 在容器内运行此脚本: bash run_jackal_mrslam.sh [模式]
#
# 模式选项:
#   run        - 运行完整的多机器人 SLAM
#   remap      - 仅启动 topic 重映射 (用于调试)
#   stop       - 停止所有节点
#
# 示例:
#   bash run_jackal_mrslam.sh run
###############################################################################

set -e

# ====================== 配置参数 ======================
# 工作空间路径 (在 Docker 内的路径)
MRSLAM_ROOT="/home"
MAPPING_WS="${MRSLAM_ROOT}/Mapping"
LOCALIZATION_WS="${MRSLAM_ROOT}/Localization"
LOOPDETECTION_WS="${MRSLAM_ROOT}/LoopDetection"
COSTMAP_WS="${MRSLAM_ROOT}/Costmap"
TOOLS_DIR="${MRSLAM_ROOT}/Tools"
VIS_DIR="${MRSLAM_ROOT}/Visualization"

# ====================== Jackal Bag 配置 ======================
# 你的 rosbag 文件路径 (根据实际情况修改)
JACKAL_BAG_PATH="/media/cyw/KESU/mapping_data/20250516_rawdata/whu_group5.bag"

# 机器人映射配置: jackal0 -> robot_1, jackal1 -> robot_2, jackal2 -> robot_3
declare -A ROBOT_MAPPING=(
    ["jackal0"]="robot_1"
    ["jackal1"]="robot_2"
    ["jackal2"]="robot_3"
)

# 机器人数量
NUM_ROBOTS=3

# 循环检测方法: disco, scancontext, ring, ringplusplus
# 注意: RTX 30xx/40xx 需要重新安装 PyTorch，建议使用 scancontext
LOOP_DETECTION_METHOD="scancontext"

# 里程计方法: fastlio, aloam
ODOMETRY_METHOD="fastlio"

# LiDAR 类型: livox_CustomMsg (1), PointCloud2 (2), ouster (3)
# 注意: 你的 bag 是 PointCloud2 格式，所以使用 2
LIDAR_TYPE=2

# 是否启用高程建图
ENABLE_ELEVATION_MAPPING=false

# 是否启用 costmap
ENABLE_COSTMAP=false

# ====================== 颜色定义 ======================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# ====================== 辅助函数 ======================
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

log_config() {
    echo -e "${CYAN}[CONFIG]${NC} $1"
}

check_ros() {
    if [ -z "$ROS_DISTRO" ]; then
        log_info "Sourcing ROS setup..."
        source /opt/ros/noetic/setup.bash 2>/dev/null || \
        source /opt/ros/melodic/setup.bash 2>/dev/null || \
        source /opt/ros/kinetic/setup.bash
    fi
    log_info "ROS_DISTRO: $ROS_DISTRO"
    
    # 创建 FAST-LIO 日志目录 (消除 "doesn't exist" 警告)
    mkdir -p /home/Localization/src/FAST_LIO/Log 2>/dev/null || true
    mkdir -p /home/Localization/src/FAST_LIO/PCD 2>/dev/null || true
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

cleanup() {
    log_info "清理运行中的节点..."
    pkill -f roscore 2>/dev/null || true
    pkill -f roslaunch 2>/dev/null || true
    pkill -f rosbag 2>/dev/null || true
    pkill -f main.py 2>/dev/null || true
    pkill -f main_RING.py 2>/dev/null || true
    pkill -f main_SC.py 2>/dev/null || true
    pkill -f main_RINGplusplus.py 2>/dev/null || true
    tmux kill-session -t jackal_mrslam 2>/dev/null || true
    sleep 1
}

# ====================== Topic 重映射函数 ======================
# 注意: Topic 重映射现在通过 rosbag play 的 --remap 参数实现
# 这样可以保持原始消息类型 (livox_ros_driver/CustomMsg)
# 以下函数保留用于调试目的，但在正常运行时不再需要
start_topic_remapping() {
    log_info "Topic 重映射已通过 rosbag play 实现，无需额外节点"
    log_config "  /jackal0/livox/lidar -> /robot_1/pointcloud"
    log_config "  /jackal0/livox/imu   -> /robot_1/imu"
    log_config "  /jackal1/livox/lidar -> /robot_2/pointcloud"
    log_config "  /jackal1/livox/imu   -> /robot_2/imu"
    log_config "  /jackal2/livox/lidar -> /robot_3/pointcloud"
    log_config "  /jackal2/livox/imu   -> /robot_3/imu"
}

# ====================== 启动函数 ======================
start_roscore() {
    log_step "启动 roscore..."
    run_in_tmux "jackal_mrslam" "roscore" "roscore"
    sleep 3
}

start_rosbag() {
    log_step "启动 rosbag (带 topic 重映射)..."
    if [ ! -f "$JACKAL_BAG_PATH" ]; then
        log_error "rosbag 文件不存在: $JACKAL_BAG_PATH"
        log_info "请修改脚本中的 JACKAL_BAG_PATH 变量"
        exit 1
    fi
    
    # 使用 rosbag play 的 --remap 功能进行 topic 重映射
    # 这样可以保持原始消息类型 (livox_ros_driver/CustomMsg)
    run_in_tmux "jackal_mrslam" "rosbag" \
        "rosbag play ${JACKAL_BAG_PATH} --clock \
        /jackal0/livox/lidar:=/robot_1/pointcloud \
        /jackal0/livox/imu:=/robot_1/imu \
        /jackal1/livox/lidar:=/robot_2/pointcloud \
        /jackal1/livox/imu:=/robot_2/imu \
        /jackal2/livox/lidar:=/robot_3/pointcloud \
        /jackal2/livox/imu:=/robot_3/imu"
    sleep 2
}

start_odometry() {
    log_step "启动里程计 (${ODOMETRY_METHOD})..."
    source_workspace "$LOCALIZATION_WS"
    
    # Jackal 自定义配置路径 (挂载到 Docker 内的路径)
    local JACKAL_LAUNCH_DIR="/home/cyw_local/MR_SLAM/Localization/src/FAST_LIO/launch"
    
    for i in $(seq 1 $NUM_ROBOTS); do
        if [ "$ODOMETRY_METHOD" == "fastlio" ]; then
            # 使用绝对路径启动 Jackal 专用 launch 文件
            run_in_tmux "jackal_mrslam" "odom_${i}" \
                "source ${LOCALIZATION_WS}/devel/setup.bash && roslaunch ${JACKAL_LAUNCH_DIR}/jackal_${i}.launch"
        else
            run_in_tmux "jackal_mrslam" "odom_${i}" \
                "source ${LOCALIZATION_WS}/devel/setup.bash && roslaunch aloam robot_${i}.launch"
        fi
    done
    sleep 2
}

start_elevation_mapping() {
    if [ "$ENABLE_ELEVATION_MAPPING" != true ]; then
        log_info "高程建图已禁用，跳过..."
        return
    fi
    
    log_step "启动高程建图..."
    source_workspace "$MAPPING_WS"
    
    for i in $(seq 1 $NUM_ROBOTS); do
        run_in_tmux "jackal_mrslam" "elev_${i}" \
            "source ${MAPPING_WS}/devel/setup.bash && roslaunch elevation_mapping_demos robot_${i}.launch"
    done
    
    log_step "启动 fake image 生成器..."
    for i in $(seq 1 $NUM_ROBOTS); do
        run_in_tmux "jackal_mrslam" "fake_img_${i}" \
            "cd ${TOOLS_DIR}/Fake_img && python3 robot_${i}.py"
    done
    sleep 2
}

start_loop_detection() {
    log_step "启动循环检测 (${LOOP_DETECTION_METHOD})..."
    source_workspace "$LOOPDETECTION_WS"
    
    local WORK_DIR="/tmp/loop_detection"
    
    case "$LOOP_DETECTION_METHOD" in
        "disco")
            run_in_tmux "jackal_mrslam" "loop_detect" \
                "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && rosrun disco_ros main.py"
            ;;
        "scancontext")
            run_in_tmux "jackal_mrslam" "loop_detect" \
                "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && python3 ${LOOPDETECTION_WS}/src/RING_ros/main_SC.py"
            ;;
        "ring")
            run_in_tmux "jackal_mrslam" "loop_detect" \
                "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && python3 ${LOOPDETECTION_WS}/src/RING_ros/main_RING.py"
            ;;
        "ringplusplus")
            run_in_tmux "jackal_mrslam" "loop_detect" \
                "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && python3 ${LOOPDETECTION_WS}/src/RING_ros/main_RINGplusplus.py"
            ;;
        *)
            log_error "未知的循环检测方法: $LOOP_DETECTION_METHOD"
            exit 1
            ;;
    esac
    sleep 2
}

start_global_manager() {
    log_step "启动 global_manager..."
    source_workspace "$MAPPING_WS"
    run_in_tmux "jackal_mrslam" "global_mgr" \
        "source ${MAPPING_WS}/devel/setup.bash && roslaunch global_manager global_manager.launch"
    sleep 2
}

start_costmap() {
    if [ "$ENABLE_COSTMAP" != true ]; then
        log_info "Costmap 已禁用，跳过..."
        return
    fi
    
    log_step "启动 costmap 转换器..."
    source_workspace "$COSTMAP_WS"
    run_in_tmux "jackal_mrslam" "costmap" \
        "source ${COSTMAP_WS}/devel/setup.bash && roslaunch costmap_converter create_costmap.launch"
    sleep 2
}

start_visualization() {
    log_step "启动可视化..."
    # 使用 Jackal 专用的 rviz 配置 (挂载路径)
    local JACKAL_VIS="/home/cyw_local/MR_SLAM/Visualization/vis_jackal.rviz"
    if [ -f "$JACKAL_VIS" ]; then
        run_in_tmux "jackal_mrslam" "rviz" "rviz -d ${JACKAL_VIS}"
    else
        run_in_tmux "jackal_mrslam" "rviz" "rviz -d ${VIS_DIR}/vis.rviz"
    fi
}

# ====================== 模式函数 ======================
run_full_mode() {
    log_info "============================================"
    log_info "    Jackal Multi-Robot SLAM 完整模式"
    log_info "============================================"
    log_config "Bag 文件: $JACKAL_BAG_PATH"
    log_config "机器人数量: $NUM_ROBOTS"
    log_config "里程计: $ODOMETRY_METHOD"
    log_config "回环检测: $LOOP_DETECTION_METHOD"
    log_info "============================================"
    
    cleanup
    check_ros
    
    start_roscore
    start_rosbag
    start_topic_remapping
    start_odometry
    start_elevation_mapping
    start_loop_detection
    start_global_manager
    start_costmap
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

run_remap_only() {
    log_info "============================================"
    log_info "    仅启动 Topic 重映射 (调试模式)"
    log_info "============================================"
    
    cleanup
    check_ros
    
    start_roscore
    start_rosbag
    start_topic_remapping
    
    log_info "============================================"
    log_info "Topic 重映射已启动，可以使用以下命令查看:"
    log_info "  rostopic list"
    log_info "  rostopic echo /robot_1/pointcloud"
    log_info "============================================"
    log_info ""
    log_info "使用 tmux attach -t jackal_mrslam 查看各个终端"
    log_info "停止运行: bash $0 stop"
}

check_status() {
    log_info "============================================"
    log_info "         Jackal MR_SLAM 状态检查"
    log_info "============================================"
    
    check_ros
    
    echo ""
    log_step "检查 tmux 会话..."
    if tmux has-session -t jackal_mrslam 2>/dev/null; then
        log_info "tmux 会话 'jackal_mrslam' 存在"
        echo "  窗口列表:"
        tmux list-windows -t jackal_mrslam 2>/dev/null | sed 's/^/    /'
    else
        log_warn "tmux 会话 'jackal_mrslam' 不存在"
    fi
    
    echo ""
    log_step "检查 ROS Master..."
    if rostopic list &>/dev/null; then
        log_info "ROS Master 可访问"
        
        echo ""
        log_step "检查原始 topics (Jackal):"
        rostopic list 2>/dev/null | grep -E "^/jackal" | head -10 | sed 's/^/    /' || echo "    (无)"
        
        echo ""
        log_step "检查重映射后的 topics (robot_X):"
        rostopic list 2>/dev/null | grep -E "^/robot_" | head -10 | sed 's/^/    /' || echo "    (无)"
    else
        log_error "ROS Master 不可访问!"
    fi
    
    echo ""
    log_info "============================================"
}

show_help() {
    echo "Jackal Multi-Robot SLAM 启动脚本"
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  run         运行完整的多机器人 SLAM 系统"
    echo "  remap       仅启动 topic 重映射 (用于调试)"
    echo "  stop        停止所有节点"
    echo "  status      检查各节点运行状态"
    echo "  help        显示此帮助信息"
    echo ""
    echo "配置说明:"
    echo "  请在脚本中修改以下变量:"
    echo "    JACKAL_BAG_PATH         - 你的 rosbag 文件路径"
    echo "    LOOP_DETECTION_METHOD   - 回环检测方法"
    echo "    ODOMETRY_METHOD         - 里程计方法"
    echo ""
    echo "Topic 映射:"
    echo "  /jackal0/livox/lidar -> /robot_1/pointcloud"
    echo "  /jackal0/livox/imu   -> /robot_1/imu"
    echo "  /jackal1/livox/lidar -> /robot_2/pointcloud"
    echo "  /jackal1/livox/imu   -> /robot_2/imu"
    echo "  /jackal2/livox/lidar -> /robot_3/pointcloud"
    echo "  /jackal2/livox/imu   -> /robot_3/imu"
    echo ""
    echo "示例:"
    echo "  $0 run"
    echo "  $0 remap"
}

# ====================== 主程序 ======================
main() {
    case "${1:-help}" in
        "run")
            run_full_mode
            ;;
        "remap")
            run_remap_only
            ;;
        "stop")
            cleanup
            log_info "所有节点已停止"
            ;;
        "status")
            check_status
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
