#!/bin/bash

# 智能G1客户端启动脚本 - 自动检测和修复环境问题
# 使用方法: ./run_g1_smart.sh

echo "=== 智能G1客户端启动脚本 ==="
echo "时间: $(date)"
echo "用户: $(whoami)"
echo "主机: $(hostname)"
echo "工作目录: $(pwd)"
echo "================================"

# 1. 检查并设置ROS环境
echo "🔍 检查ROS环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS环境未设置，正在初始化..."
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        echo "✅ ROS环境已初始化: $ROS_DISTRO"
    else
        echo "❌ 找不到ROS安装，请检查ROS是否正确安装"
        exit 1
    fi
else
    echo "✅ ROS环境已存在: $ROS_DISTRO"
fi

# 2. 检查ROS Master
echo "🔍 检查ROS Master状态..."
if rostopic list >/dev/null 2>&1; then
    echo "✅ ROS Master正在运行"
    echo "📡 活跃话题数量: $(rostopic list | wc -l)"
else
    echo "⚠️  ROS Master未运行，正在启动..."
    roscore > /dev/null 2>&1 &
    ROS_PID=$!
    echo "🔄 等待ROS Master启动..."
    sleep 5
    
    if rostopic list >/dev/null 2>&1; then
        echo "✅ ROS Master启动成功 (PID: $ROS_PID)"
    else
        echo "❌ ROS Master启动失败"
        echo "💡 请手动运行: roscore"
        exit 1
    fi
fi

# 3. 设置完整的环境变量
echo "🔧 设置环境变量..."

# ROS环境变量
export ROS_DISTRO=noetic
export ROS_ROOT=/opt/ros/noetic/share/ros
export ROS_MASTER_URI=http://localhost:11311

# Python环境变量
export PYTHONPATH=/home/unitree/HongTu/G1Nav2D/devel/lib/python3/dist-packages:/home/unitree/ros_noetic_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
export ROS_PYTHON_VERSION=3

# 路径环境变量
export PATH=/home/unitree/.local/bin:/usr/local/cuda-11.4/bin:/opt/ros/noetic/bin:/home/unitree/cyclonedds/install/bin:/opt/ros/foxy/bin:/home/unitree/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin

# 库路径
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:/home/unitree/HongTu/G1Nav2D/devel/lib:/home/unitree/ros_noetic_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu:/usr/local/cuda-11.4/lib64:/home/unitree/cyclonedds/install/lib:/opt/ros/foxy/lib:/opt/ros/foxy/lib/aarch64-linux-gnu

# 包配置路径
export PKG_CONFIG_PATH=/home/unitree/HongTu/G1Nav2D/devel/lib/pkgconfig:/home/unitree/ros_noetic_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/aarch64-linux-gnu/pkgconfig:/home/unitree/cyclonedds/install/lib/pkgconfig:/opt/ros/foxy/lib/pkgconfig:/opt/ros/foxy/lib/aarch64-linux-gnu/pkgconfig

# ROS包路径
export ROS_PACKAGE_PATH=/home/unitree/HongTu/G1Nav2D/src:/home/unitree/ros_noetic_ws/src:/opt/ros/noetic/share

# CMAKE前缀路径
export CMAKE_PREFIX_PATH=/home/unitree/HongTu/G1Nav2D/devel:/home/unitree/ros_noetic_ws/devel:/opt/ros/noetic:/home/unitree/cyclonedds/install:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy

echo "✅ 环境变量设置完成"

# 4. 检查关键依赖
echo "🔍 检查关键依赖..."

# 检查Python
PYTHON_VERSION=$(python3 --version 2>&1)
echo "🐍 Python版本: $PYTHON_VERSION"

# 检查cyclonedx库
if python3 -c "import cyclonedx" 2>/dev/null; then
    echo "✅ cyclonedx库可用"
else
    echo "⚠️  cyclonedx库不可用，尝试修复..."
    if [ -d "/home/unitree/cyclonedds/install/lib" ]; then
        echo "✅ 找到cyclonedx库路径"
    else
        echo "❌ 找不到cyclonedx库，请检查安装"
    fi
fi

# 5. 检查网络接口
echo "🌐 检查网络接口..."
if ip addr show eth0 >/dev/null 2>&1; then
    echo "✅ eth0接口存在"
    ETH0_STATUS=$(ip link show eth0 | grep -o "state [A-Z]*" | cut -d' ' -f2)
    echo "   eth0状态: $ETH0_STATUS"
else
    echo "⚠️  eth0接口不存在"
fi

# 6. 切换到目标目录
echo "📁 切换到目标目录..."
cd /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level
echo "   当前目录: $(pwd)"

# 7. 显示最终环境信息
echo "📊 最终环境信息:"
echo "   工作目录: $(pwd)"
echo "   Python版本: $(python3 --version)"
echo "   Python路径: $(which python3)"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_MASTER_URI: $ROS_MASTER_URI"
echo "   PYTHONPATH: $PYTHONPATH"

echo ""
echo "🚀 启动G1客户端..."
echo "================================"

# 8. 启动G1客户端
python3 g1_client_now.py "$@"
