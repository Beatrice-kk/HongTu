#!/bin/bash

# G1机器人移动控制修复脚本
# 用于快速修复机器人"控制不了下肢移动"的问题

echo "🤖 G1机器人移动控制修复工具"
echo "=================================="
echo ""

# 检查Python3是否可用
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 未安装或不可用"
    exit 1
fi

# 检查当前目录
if [ ! -f "fix_robot_movement.py" ]; then
    echo "❌ 请在包含修复工具的目录中运行此脚本"
    exit 1
fi

echo "🔍 检查机器人连接..."
ping -c 1 192.168.123.161 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ 机器人网络连接正常"
else
    echo "❌ 机器人网络连接异常"
    echo "💡 请检查:"
    echo "   • 机器人是否已开机"
    echo "   • 网络连接是否正常"
    echo "   • IP地址是否正确"
    exit 1
fi

echo ""
echo "🔧 开始修复机器人移动控制问题..."
echo ""

# 运行修复工具
python3 fix_robot_movement.py

echo ""
echo "修复完成！"
echo "💡 如果问题仍然存在，请查看 README_robot_fix.md 获取更多帮助"
