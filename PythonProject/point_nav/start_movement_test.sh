#!/bin/bash

# 机器人移动测试启动脚本
# 优先确保机器人能够移动，不考虑精准度

echo "=========================================="
echo "机器人移动测试启动脚本"
echo "目标：确保机器人能够移动，不考虑精准度"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "错误: ROS环境未设置，请先source ROS环境"
    echo "请运行: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# 检查必要的服务是否运行
echo "检查必要的ROS服务..."

# 检查里程计是否运行
if ! rostopic list | grep -q "/odom"; then
    echo "警告: 里程计话题 /odom 未找到，位置更新可能不准确"
fi

# 检查cmd_vel是否可用
if ! rostopic list | grep -q "/cmd_vel"; then
    echo "错误: cmd_vel话题不可用，无法控制机器人移动"
    exit 1
fi

echo "ROS环境检查完成"

# 设置Python路径
export PYTHONPATH=$PYTHONPATH:/home/unitree/HongTu/PythonProject/point_nav

# 选择测试模式
echo ""
echo "请选择测试模式："
echo "1. 快速移动测试（推荐）- 测试机器人基本移动能力"
echo "2. 高成功率导航测试 - 使用多种策略导航到后台点"
echo "3. 退出"
echo ""

read -p "请输入选择 (1-3): " choice

case $choice in
    1)
        echo "启动快速移动测试..."
        echo "目标：确保机器人能够移动，不考虑精准度"
        echo ""
        python3 /home/unitree/HongTu/PythonProject/point_nav/quick_move_test.py
        ;;
    2)
        echo "启动高成功率导航测试..."
        echo "目标点: (-0.8, 0, 0)"
        echo "使用多种策略确保导航成功"
        echo ""
        python3 /home/unitree/HongTu/PythonProject/point_nav/robust_navigation_to_backstage.py
        ;;
    3)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择，退出"
        exit 1
        ;;
esac

echo ""
echo "测试脚本执行完成"
