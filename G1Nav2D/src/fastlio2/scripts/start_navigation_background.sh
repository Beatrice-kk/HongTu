#!/bin/bash
# 后台启动导航系统的脚本

set -e  # 遇到错误立即退出

# 配置参数
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="/home/unitree/HongTu"
LOG_DIR="${PROJECT_ROOT}/PythonProject/point_nav/logs"
CONFIG_FILE="${SCRIPT_DIR}/../config/reloc_improved.yaml"

# 创建日志目录
mkdir -p "$LOG_DIR"

# 生成时间戳
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="${LOG_DIR}/navigation_startup_${TIMESTAMP}.log"

# 函数：记录日志
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOG_FILE"
}

# 函数：检查进程是否运行
check_process() {
    local process_name="$1"
    pgrep -f "$process_name" > /dev/null
}

# 函数：等待服务启动
wait_for_service() {
    local service_name="$1"
    local timeout="${2:-30}"
    local count=0
    
    log "等待服务启动: $service_name"
    while [ $count -lt $timeout ]; do
        if rostopic list | grep -q "$service_name" || rosservice list | grep -q "$service_name"; then
            log "服务 $service_name 已就绪"
            return 0
        fi
        sleep 1
        ((count++))
    done
    
    log "警告: 服务 $service_name 启动超时"
    return 1
}

# 函数：清理进程
cleanup_processes() {
    log "清理相关进程..."
    
    # 清理ROS相关进程
    pkill -f "fastlio" || true
    pkill -f "move_base" || true
    pkill -f "amcl" || true
    pkill -f "map_server" || true
    pkill -f "one_key_reloc_node" || true
    pkill -f "auto_relocalize" || true
    
    # 等待进程完全退出
    sleep 2
    log "进程清理完成"
}

# 函数：启动导航系统
start_navigation() {
    log "开始启动导航系统..."
    
    # 1. 检查ROS环境
    if [ -z "$ROS_MASTER_URI" ]; then
        log "错误: ROS环境未设置"
        return 1
    fi
    
    # 2. 检查roscore是否运行
    if ! check_process "roscore"; then
        log "错误: roscore未运行"
        return 1
    fi
    
    # 3. 清理旧进程
    cleanup_processes
    
    # 4. 加载配置参数
    log "加载重定位配置..."
    if [ -f "$CONFIG_FILE" ]; then
        rosparam load "$CONFIG_FILE" /one_key_reloc_node
        log "配置加载成功: $CONFIG_FILE"
    else
        log "警告: 配置文件不存在: $CONFIG_FILE"
        log "使用默认配置"
    fi
    
    # 5. 启动导航launch文件
    log "启动fastlio导航..."
    roslaunch fastlio navigation.launch use_rviz:=false > "$LOG_FILE" 2>&1 &
    NAVIGATION_PID=$!
    
    # 6. 等待关键服务启动
    log "等待导航服务启动..."
    sleep 8  # 给系统启动时间
    
    # 检查关键服务
    wait_for_service "slam_reloc" 15
    wait_for_service "slam_reloc_check" 15
    
    # 7. 启动优化的重定位脚本
    log "启动自动重定位..."
    python3 "${SCRIPT_DIR}/auto_relocalize_improved.py" >> "$LOG_FILE" 2>&1 &
    RELOC_PID=$!
    
    # 8. 监控重定位状态
    log "监控重定位状态..."
    local monitor_count=0
    local max_monitor_time=60  # 最多监控60秒
    
    while [ $monitor_count -lt $max_monitor_time ]; do
        # 检查重定位是否成功
        if rosservice call /slam_reloc_check "{}" 2>/dev/null | grep -q "status: True"; then
            log "重定位成功！"
            echo "$NAVIGATION_PID" > "${LOG_DIR}/navigation.pid"
            echo "$RELOC_PID" > "${LOG_DIR}/reloc.pid"
            return 0
        fi
        
        # 检查进程是否还在运行
        if ! kill -0 $NAVIGATION_PID 2>/dev/null; then
            log "错误: 导航进程异常退出"
            return 1
        fi
        
        if ! kill -0 $RELOC_PID 2>/dev/null; then
            log "重定位脚本已完成"
            break
        fi
        
        sleep 2
        ((monitor_count += 2))
    done
    
    # 9. 检查最终状态
    if rosservice call /slam_reloc_check "{}" 2>/dev/null | grep -q "status: True"; then
        log "重定位成功！"
        echo "$NAVIGATION_PID" > "${LOG_DIR}/navigation.pid"
        echo "$RELOC_PID" > "${LOG_DIR}/reloc.pid"
        return 0
    else
        log "错误: 重定位失败"
        cleanup_processes
        return 1
    fi
}

# 函数：停止导航系统
stop_navigation() {
    log "停止导航系统..."
    
    # 读取PID文件
    if [ -f "${LOG_DIR}/navigation.pid" ]; then
        NAV_PID=$(cat "${LOG_DIR}/navigation.pid")
        if kill -0 $NAV_PID 2>/dev/null; then
            kill $NAV_PID
            log "导航进程已停止"
        fi
        rm -f "${LOG_DIR}/navigation.pid"
    fi
    
    if [ -f "${LOG_DIR}/reloc.pid" ]; then
        RELOC_PID=$(cat "${LOG_DIR}/reloc.pid")
        if kill -0 $RELOC_PID 2>/dev/null; then
            kill $RELOC_PID
            log "重定位进程已停止"
        fi
        rm -f "${LOG_DIR}/reloc.pid"
    fi
    
    cleanup_processes
    log "导航系统已停止"
}

# 主函数
main() {
    case "${1:-start}" in
        "start")
            log "=== 启动导航系统 ==="
            if start_navigation; then
                log "导航系统启动成功"
                exit 0
            else
                log "导航系统启动失败"
                exit 1
            fi
            ;;
        "stop")
            log "=== 停止导航系统 ==="
            stop_navigation
            ;;
        "restart")
            log "=== 重启导航系统 ==="
            stop_navigation
            sleep 3
            if start_navigation; then
                log "导航系统重启成功"
                exit 0
            else
                log "导航系统重启失败"
                exit 1
            fi
            ;;
        "status")
            log "=== 检查导航系统状态 ==="
            if [ -f "${LOG_DIR}/navigation.pid" ]; then
                NAV_PID=$(cat "${LOG_DIR}/navigation.pid")
                if kill -0 $NAV_PID 2>/dev/null; then
                    log "导航系统正在运行 (PID: $NAV_PID)"
                else
                    log "导航系统未运行"
                fi
            else
                log "导航系统未运行"
            fi
            ;;
        *)
            echo "用法: $0 {start|stop|restart|status}"
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@"
