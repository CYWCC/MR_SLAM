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
DEMO_BAG_PATH="/media/cyw/KESU/mapping_data/MR_SLAM_data/3_dog.bag"
FULL_BAG_DIR="/media/cyw/KESU/mapping_data/MR_SLAM_data"

# 循环检测方法: disco, scancontext, ring, ringplusplus
# 注意: ring/ringplusplus 需要与 GPU 兼容的 PyTorch
# RTX 30xx/40xx 系列可能需要重新安装 PyTorch，建议使用 scancontext 或 disco
LOOP_DETECTION_METHOD="disco"

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
    run_in_tmux "mrslam" "rosbag" "rosbag play ${DEMO_BAG_PATH} --clock"
    sleep 2
}

start_rosbag_full() {
    log_step "启动完整模式 rosbags..."
    local first_bag=true
    for i in $(seq 1 $NUM_ROBOTS); do
        local bag_file="${FULL_BAG_DIR}/loop_${i}.bag"
        if [ -f "$bag_file" ]; then
            if [ "$first_bag" = true ]; then
                # 第一个 bag 使用 --clock 发布时钟
                run_in_tmux "mrslam" "bag_${i}" "rosbag play ${bag_file} --clock"
                first_bag=false
            else
                # 其他 bag 不带 --clock，使用第一个 bag 的时钟
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
        run_in_tmux "mrslam" "fake_img_${i}" "cd ${TOOLS_DIR}/Fake_img && python3 robot_${i}.py"
    done
    sleep 2
}

start_loop_detection() {
    log_step "启动循环检测 (${LOOP_DETECTION_METHOD})..."
    source_workspace "$LOOPDETECTION_WS"
    
    # 创建可写的工作目录 (避免权限问题)
    local WORK_DIR="/tmp/loop_detection"
    
    case "$LOOP_DETECTION_METHOD" in
        "disco")
            run_in_tmux "mrslam" "loop_detect" "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && rosrun disco_ros main.py"
            ;;
        "scancontext")
            run_in_tmux "mrslam" "loop_detect" "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && python3 ${LOOPDETECTION_WS}/src/RING_ros/main_SC.py"
            ;;
        "ring")
            run_in_tmux "mrslam" "loop_detect" "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && python3 ${LOOPDETECTION_WS}/src/RING_ros/main_RING.py"
            ;;
        "ringplusplus")
            run_in_tmux "mrslam" "loop_detect" "source ${LOOPDETECTION_WS}/devel/setup.bash && mkdir -p ${WORK_DIR} && cd ${WORK_DIR} && python3 ${LOOPDETECTION_WS}/src/RING_ros/main_RINGplusplus.py"
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
    log_info "rosbag 已自动开始播放"
    log_info "使用 tmux attach -t mrslam 查看各个终端"
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
    log_info "rosbag 已自动开始播放"
    log_info "使用 tmux attach -t mrslam 查看各个终端"
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
    echo "  status        检查各节点运行状态"
    echo "  topics        查看当前 ROS topics"
    echo "  nodes         查看当前 ROS nodes"
    echo "  help          显示此帮助信息"
    echo ""
    echo "选项 (通过环境变量设置):"
    echo "  LOOP_DETECTION_METHOD   循环检测方法 (disco/scancontext/ring/ringplusplus)"
    echo "  ODOMETRY_METHOD         里程计方法 (fastlio/aloam)"
    echo "  NUM_ROBOTS              机器人数量 (默认: 3)"
    echo "  ENABLE_ELEVATION_MAPPING 是否启用高程建图 (true/false)"
    echo "  ENABLE_COSTMAP          是否启用 costmap (true/false)"
    echo ""
    echo "调试步骤:"
    echo "  1. 运行 '$0 status' 检查各节点状态"
    echo "  2. 运行 '$0 topics' 查看 ROS topics"
    echo "  3. 运行 'tmux attach -t mrslam' 进入 tmux 查看各终端输出"
    echo "  4. 在 rosbag 窗口按空格键开始播放"
    echo ""
    echo "示例:"
    echo "  $0 demo"
    echo "  $0 full"
    echo "  LOOP_DETECTION_METHOD=ring NUM_ROBOTS=2 $0 full"
}

# ====================== 状态检查函数 ======================
check_status() {
    log_info "============================================"
    log_info "         MR_SLAM 状态检查"
    log_info "============================================"
    
    check_ros
    
    echo ""
    log_step "检查 tmux 会话..."
    if tmux has-session -t mrslam 2>/dev/null; then
        log_info "tmux 会话 'mrslam' 存在"
        echo "  窗口列表:"
        tmux list-windows -t mrslam 2>/dev/null | sed 's/^/    /'
    else
        log_warn "tmux 会话 'mrslam' 不存在"
    fi
    
    echo ""
    log_step "检查 roscore..."
    if pgrep -f roscore > /dev/null; then
        log_info "roscore 正在运行"
    else
        log_error "roscore 未运行!"
    fi
    
    echo ""
    log_step "检查 ROS Master..."
    if rostopic list &>/dev/null; then
        log_info "ROS Master 可访问"
    else
        log_error "ROS Master 不可访问! 请检查 roscore"
    fi
    
    echo ""
    log_step "检查 rosbag..."
    if pgrep -f "rosbag play" > /dev/null; then
        log_info "rosbag 正在运行"
    else
        log_warn "rosbag 未运行或已暂停 (按空格键开始播放)"
    fi
    
    echo ""
    log_step "检查循环检测节点..."
    if pgrep -f "main_RINGplusplus" > /dev/null || pgrep -f "main_RING.py" > /dev/null || pgrep -f "main_SC" > /dev/null || pgrep -f "disco_ros" > /dev/null; then
        log_info "循环检测节点正在运行"
    else
        # 再次尝试通过 python3 进程检测
        if ps aux | grep -E "python3.*RING" | grep -v grep > /dev/null; then
            log_info "循环检测节点正在运行"
        else
            log_warn "循环检测节点未运行"
            # 尝试显示 loop_detect 窗口的最近输出
            if tmux has-session -t mrslam 2>/dev/null; then
                echo "  loop_detect 窗口最近输出:"
                tmux capture-pane -t mrslam:loop_detect -p 2>/dev/null | tail -5 | sed 's/^/    /' || echo "    (无法获取)"
            fi
        fi
    fi
    
    echo ""
    log_step "检查 global_manager..."
    if rosnode list 2>/dev/null | grep -q "global_manager"; then
        log_info "global_manager 节点正在运行"
    else
        log_warn "global_manager 节点未找到"
    fi
    
    echo ""
    log_step "检查 rviz..."
    if pgrep -f rviz > /dev/null; then
        log_info "rviz 正在运行"
    else
        log_warn "rviz 未运行"
    fi
    
    echo ""
    log_info "============================================"
    log_info "提示: 使用 'tmux attach -t mrslam' 查看详细输出"
    log_info "============================================"
}

show_topics() {
    log_info "============================================"
    log_info "         当前 ROS Topics"
    log_info "============================================"
    
    check_ros
    
    if ! rostopic list &>/dev/null; then
        log_error "ROS Master 不可访问!"
        return 1
    fi
    
    echo ""
    log_step "所有 topics:"
    rostopic list
    
    echo ""
    log_step "关键 topics 检查:"
    
    # 检查点云 topics
    echo "  点云 topics:"
    rostopic list 2>/dev/null | grep -E "cloud|points|velodyne|livox" | sed 's/^/    /' || echo "    (无)"
    
    # 检查 submap topics
    echo "  Submap topics:"
    rostopic list 2>/dev/null | grep -i submap | sed 's/^/    /' || echo "    (无)"
    
    # 检查 descriptor topics
    echo "  Descriptor topics:"
    rostopic list 2>/dev/null | grep -iE "disco|ring|descriptor" | sed 's/^/    /' || echo "    (无)"
    
    # 检查 merged topics
    echo "  Merged topics:"
    rostopic list 2>/dev/null | grep -i merged | sed 's/^/    /' || echo "    (无)"
    
    echo ""
    log_info "使用 'rostopic echo <topic>' 查看具体消息"
    log_info "使用 'rostopic hz <topic>' 查看发布频率"
}

show_nodes() {
    log_info "============================================"
    log_info "         当前 ROS Nodes"
    log_info "============================================"
    
    check_ros
    
    if ! rosnode list &>/dev/null; then
        log_error "ROS Master 不可访问!"
        return 1
    fi
    
    echo ""
    log_step "所有节点:"
    rosnode list
    
    echo ""
    log_info "使用 'rosnode info <node>' 查看节点详情"
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
        "status")
            check_status
            ;;
        "topics")
            show_topics
            ;;
        "nodes")
            show_nodes
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
