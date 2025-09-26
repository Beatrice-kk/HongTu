#!/bin/bash

# 舞蹈服务重启脚本
# 用于解决舞蹈服务在手动停止后无法正常工作的问题

echo "? 重启舞蹈服务..."

# 1. 停止所有相关的ROS节点
echo "?? 停止相关ROS节点..."
rosnode kill /g1_action_player 2>/dev/null || true
rosnode kill /simple_nav_waypoints_player 2>/dev/null || true
rosnode kill /g1_client_original_backup1 2>/dev/null || true

# 2. 等待节点完全停止
echo "? 等待节点停止..."
sleep 3

# 3. 清理可能的僵尸进程
echo "? 清理僵尸进程..."
pkill -f "g1_action_player" 2>/dev/null || true
pkill -f "simple_nav_waypoints_player" 2>/dev/null || true
pkill -f "g1_client_original_backup1" 2>/dev/null || true

# 4. 等待进程完全清理
sleep 2

# 5. 重新启动G1客户端（如果需要）
echo "? 重新启动G1客户端..."
cd /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level/
python3 g1_client_original_backup1.py &
sleep 5

# 6. 检查服务状态
echo "? 检查服务状态..."
rosservice list | grep play_dance

if rosservice list | grep -q "play_dance"; then
    echo "? 舞蹈服务已就绪"
else
    echo "? 舞蹈服务未就绪，请手动启动"
fi

echo "? 服务重启完成，可以重新运行导航程序"
