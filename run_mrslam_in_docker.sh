#!/usr/bin/env bash
###############################################################################
# MR_SLAM Docker 启动脚本
# 
# 该脚本用于在 Docker 容器内启动整个 MR_SLAM 系统
# 参考 README.md 中的 Quick Demo 和 Full Usage 部分
#
# 使用方法:
#   1. 首先启动 Docker 容器: bash run_mrslam_docker.sh
#   2. 在容器内运行此脚本: bash run_mrslam_in_docker.sh [模式]
#
# 模式选项:
#   demo       - 快速演示模式 (单个 rosbag)
#   full       - 完整多机器人模式 (多个 rosbag)
#   single     - 单机器人模式
#
# 示例:
#   bash run_mrslam_in_docker.sh demo
#   bash run_mrslam_in_docker.sh full
###############################################################################

set -e

# ====================== 配置参数 ======================
# 工作空间路径 (在 Docker 内的路径)
MRSLAM_ROOT="/home"  # Docker 内代码部署路径
MAPPING_WS="${MRSLAM_ROOT}/Mapping"
LOCALIZATION_WS="${MRSLAM_ROOT}/Localization"
LOOPDETECTION_WS="${MRSLAM_ROOT}/LoopDetection"
COSTMAP_WS="${MRSLAM_ROOT}/Costmap"
TOOLS_DIR="${MRSLAM_ROOT}/Tools"
VIS_DIR="${MRSLAM_ROOT}/Visualization"

# rosbag 路径 (根据实际情况修改)
DEMO_BAG_PATH="/home/data/3_dog.bag"
FULL_BAG_DIR="/home/data"

# 循环检测方法: disco, scancontext, ring, ringplusplus
LOOP_DETECTION_METHOD="ringplusplus"

# 里程计方法: fastlio, aloam
ODOMETRY_METHOD="fastlio"

# 机器人数量 (用于 full 模式)
NUM_ROBOTS=3

# 是否启用高程建图
ENABLE_ELEVATION_MAPPING=true

# 是否启用 costmap
ENABLE_COSTMAP=false

# 终端复用工具 (tmux 或 screen)
TERMINAL_MUX="tmux"

# ====================== 颜色定义 ======================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

check_ros() {
    if [ -z "$ROS_DISTRO" ]; then
        log_info "Sourcing ROS setup..."
        source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash 2>/dev/null || source /opt/ros/kinetic/setup.bash
    fi
    log_info "ROS_DISTRO: $ROS_DISTRO"
}

source_workspace() {
    local ws_path=$1
    if [ -f "${ws_path}/devel/setup.bash" ]; then
        source "${ws_path}/devel/setup.bash"
    else
        log_warn "工作空间 ${ws_path}/devel/setup.bash 未找到，尝试使用 ROS 默认环境"
    fi
}

run_in_tmux() {
    local session_name=$1
    local window_name=$2
    local command=$3
    
    # 检查 session 是否存在
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
    tmux kill-session -t mrslam 2>/dev/null || true
    sleep 1
}

# ====================== 启动函数 ======================
start_roscore() {
    log_step "启动 roscore..."
    run_in_tmux "mrslam" "roscore" "roscore"
    sleep 3
}

start_rosbag_demo() {
    log_step "启动 demo rosbag..."
    if [ ! -f "$DEMO_BAG_PATH" ]; then
        log_error "rosbag 文件不存在: $DEMO_BAG_PATH"
        log_info "请从以下地址下载:"
        log_info "  Google Drive: https://drive.google.com/file/d/1KNrzvpfeuiQ8i-zUvlKbsn-iTIPJOQh8/view"
        log_info "  Baidu Pan: https://pan.baidu.com/s/1dcDRyn-G7TilFpc8shLQNA?pwd=gupx"
        exit 1
    fi
    run_in_tmux "mrslam" "rosbag" "rosbag play ${DEMO_BAG_PATH} --clock --pause"
    sleep 2
}

start_rosbag_full() {
    log_step "启动完整模式 rosbags..."
    for i in $(seq 1 $NUM_ROBOTS); do
        local bag_file="${FULL_BAG_DIR}/loop_${i}.bag"
        if [ -f "$bag_file" ]; then
            if [ $i -eq 1 ]; then
                run_in_tmux "mrslam" "bag_${i}" "rosbag play ${bag_file} --clock --pause"
            else
                run_in_tmux "mrslam" "bag_${i}" "rosbag play ${bag_file}"
            fi
        else
            log_warn "rosbag 文件不存在: $bag_file"
        fi
    done
    sleep 2
}

start_odometry() {
    log_step "启动里程计 (${ODOMETRY_METHOD})..."
    source_workspace "$LOCALIZATION_WS"
    
    for i in $(seq 1 $NUM_ROBOTS); do
        if [ "$ODOMETRY_METHOD" == "fastlio" ]; then
            run_in_tmux "mrslam" "odom_${i}" "source ${LOCALIZATION_WS}/devel/setup.bash && roslaunch fast_lio robot_${i}.launch"
        else
            run_in_tmux "mrslam" "odom_${i}" "source ${LOCALIZATION_WS}/devel/setup.bash && roslaunch aloam robot_${i}.launch"
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
        run_in_tmux "mrslam" "elev_${i}" "source ${MAPPING_WS}/devel/setup.bash && roslaunch elevation_mapping_demos robot_${i}.launch"
    done
    
    # 启动 fake image 生成器 (如果机器人没有相机)
    log_step "启动 fake image 生成器..."
    for i in $(seq 1 $NUM_ROBOTS); do
        run_in_tmux "mrslam" "fake_img_${i}" "cd ${TOOLS_DIR}/Fake_img && python robot_${i}.py"
    done
    sleep 2
}

start_loop_detection() {
    log_step "启动循环检测 (${LOOP_DETECTION_METHOD})..."
    source_workspace "$LOOPDETECTION_WS"
    
    case "$LOOP_DETECTION_METHOD" in
        "disco")
            run_in_tmux "mrslam" "loop_detect" "source ${LOOPDETECTION_WS}/devel/setup.bash && rosrun disco_ros main.py"
            ;;
        "scancontext")
            run_in_tmux "mrslam" "loop_detect" "cd ${LOOPDETECTION_WS}/src/RING_ros && python main_SC.py"
            ;;
        "ring")
            run_in_tmux "mrslam" "loop_detect" "cd ${LOOPDETECTION_WS}/src/RING_ros && python main_RING.py"
            ;;
        "ringplusplus")
            run_in_tmux "mrslam" "loop_detect" "cd ${LOOPDETECTION_WS}/src/RING_ros && python main_RINGplusplus.py"
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
    run_in_tmux "mrslam" "global_mgr" "source ${MAPPING_WS}/devel/setup.bash && roslaunch global_manager global_manager.launch"
    sleep 2
}

start_costmap() {
    if [ "$ENABLE_COSTMAP" != true ]; then
        log_info "Costmap 已禁用，跳过..."
        return
    fi
    
    log_step "启动 costmap 转换器..."
    source_workspace "$COSTMAP_WS"
    run_in_tmux "mrslam" "costmap" "source ${COSTMAP_WS}/devel/setup.bash && roslaunch costmap_converter create_costmap.launch"
    sleep 2
}

start_visualization() {
    log_step "启动可视化..."
    run_in_tmux "mrslam" "rviz" "rviz -d ${VIS_DIR}/vis.rviz"
}

# ====================== 模式函数 ======================
run_demo_mode() {
    log_info "============================================"
    log_info "         MR_SLAM Quick Demo 模式"
    log_info "============================================"
    
    NUM_ROBOTS=1  # Demo 模式使用单机器人
    
    cleanup
    check_ros
    
    start_roscore
    start_rosbag_demo
    start_loop_detection
    start_global_manager
    start_visualization
    
    log_info "============================================"
    log_info "         Demo 启动完成!"
    log_info "============================================"
    log_info ""
    log_info "使用 tmux attach -t mrslam 查看各个终端"
    log_info "在 rosbag 窗口按空格键开始播放"
    log_info ""
    log_info "停止运行: bash $0 stop"
}

run_full_mode() {
    log_info "============================================"
    log_info "      MR_SLAM Full Multi-Robot 模式"
    log_info "============================================"
    
    cleanup
    check_ros
    
    start_roscore
    start_rosbag_full
    start_odometry
    start_elevation_mapping
    start_loop_detection
    start_global_manager
    start_costmap
    start_visualization
    
    log_info "============================================"
    log_info "         Full 模式启动完成!"
    log_info "============================================"
    log_info ""
    log_info "使用 tmux attach -t mrslam 查看各个终端"
    log_info "在第一个 rosbag 窗口按空格键开始播放"
    log_info ""
    log_info "停止运行: bash $0 stop"
}

run_single_mode() {
    log_info "============================================"
    log_info "         MR_SLAM Single Robot 模式"
    log_info "============================================"
    
    NUM_ROBOTS=1
    
    cleanup
    check_ros
    
    start_roscore
    start_odometry
    start_elevation_mapping
    start_loop_detection
    start_global_manager
    start_visualization
    
    log_info "============================================"
    log_info "       Single Robot 模式启动完成!"
    log_info "============================================"
    log_info ""
    log_info "使用 tmux attach -t mrslam 查看各个终端"
    log_info "请手动启动您的传感器驱动或 rosbag"
    log_info ""
    log_info "停止运行: bash $0 stop"
}

# ====================== 帮助信息 ======================
show_help() {
    echo "MR_SLAM Docker 启动脚本"
    echo ""
    echo "用法: $0 [命令] [选项]"
    echo ""
    echo "命令:"
    echo "  demo          快速演示模式 (单个 rosbag)"
    echo "  full          完整多机器人模式"
    echo "  single        单机器人模式 (实时传感器)"
    echo "  stop          停止所有节点"
    echo "  help          显示此帮助信息"
    echo ""
    echo "选项 (通过环境变量设置):"
    echo "  LOOP_DETECTION_METHOD   循环检测方法 (disco/scancontext/ring/ringplusplus)"
    echo "  ODOMETRY_METHOD         里程计方法 (fastlio/aloam)"
    echo "  NUM_ROBOTS              机器人数量 (默认: 3)"
    echo "  ENABLE_ELEVATION_MAPPING 是否启用高程建图 (true/false)"
    echo "  ENABLE_COSTMAP          是否启用 costmap (true/false)"
    echo ""
    echo "示例:"
    echo "  $0 demo"
    echo "  $0 full"
    echo "  LOOP_DETECTION_METHOD=ring NUM_ROBOTS=2 $0 full"
}

# ====================== 主程序 ======================
main() {
    case "${1:-help}" in
        "demo")
            run_demo_mode
            ;;
        "full")
            run_full_mode
            ;;
        "single")
            run_single_mode
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

# 执行主程序
main "$@"
