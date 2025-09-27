#!/bin/bash

# 完整的G1客户端启动脚本，包含环境检查和自动修复
# 使用方法: ./run_g1_complete.sh

# 设置脚本目录为工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== G1客户端完整启动脚本 ==="
echo "工作目录: $(pwd)"
echo "时间: $(date)"
echo "================================"

# 检查并设置ROS环境
echo "🔍 检查ROS环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS环境未设置，正在初始化..."
    source /opt/ros/noetic/setup.bash
    export ROS_DISTRO=noetic
    export ROS_ROOT=/opt/ros/noetic/share/ros
    export ROS_MASTER_URI=http://localhost:11311
    echo "✅ ROS环境已初始化"
else
    echo "✅ ROS环境已存在: $ROS_DISTRO"
fi

# 检查ROS Master是否运行
echo "🔍 检查ROS Master状态..."
if rostopic list >/dev/null 2>&1; then
    echo "✅ ROS Master正在运行"
    echo "📡 活跃话题数量: $(rostopic list | wc -l)"
else
    echo "⚠️  ROS Master未运行，正在启动..."
    roscore &
    sleep 3
    if rostopic list >/dev/null 2>&1; then
        echo "✅ ROS Master启动成功"
    else
        echo "❌ ROS Master启动失败"
        exit 1
    fi
fi

# 设置完整的环境变量
echo "🔧 设置环境变量..."

# ROS环境变量
export ROS_DISTRO=noetic
export ROS_ROOT=/opt/ros/noetic/share/ros
export ROS_MASTER_URI=http://localhost:11311

# Python环境变量
export PYTHONPATH=/home/unitree/HongTu/G1Nav2D/devel/lib/python3/dist-packages:/home/unitree/ros_noetic_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
export ROS_PYTHON_VERSION=3

# 路径环境变量
export PATH=/home/unitree/.local/bin:/usr/local/cuda-11.4/bin:/opt/ros/noetic/bin:/home/unitree/cyclonedds_ws/install/cyclonedds/bin:/opt/ros/foxy/bin:/home/unitree/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin

# 库路径
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:/home/unitree/HongTu/G1Nav2D/devel/lib:/home/unitree/ros_noetic_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu:/usr/local/cuda-11.4/lib64:/home/unitree/cyclonedds_ws/install/cyclonedds/lib:/opt/ros/foxy/lib:/opt/ros/foxy/lib/aarch64-linux-gnu

# 包配置路径
export PKG_CONFIG_PATH=/home/unitree/HongTu/G1Nav2D/devel/lib/pkgconfig:/home/unitree/ros_noetic_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/aarch64-linux-gnu/pkgconfig:/home/unitree/cyclonedds_ws/install/cyclonedds/lib/pkgconfig:/opt/ros/foxy/lib/pkgconfig:/opt/ros/foxy/lib/aarch64-linux-gnu/pkgconfig

# ROS包路径
export ROS_PACKAGE_PATH=/home/unitree/HongTu/G1Nav2D/src:/home/unitree/ros_noetic_ws/src:/opt/ros/noetic/share

# CMAKE前缀路径
export CMAKE_PREFIX_PATH=/home/unitree/HongTu/G1Nav2D/devel:/home/unitree/ros_noetic_ws/devel:/opt/ros/noetic:/home/unitree/cyclonedds_ws/install/cyclonedds:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy

echo "✅ 环境变量设置完成"

# 检查关键依赖
echo "🔍 检查关键依赖..."

# 检查Python版本
PYTHON_VERSION=$(python3 --version 2>&1)
echo "🐍 Python版本: $PYTHON_VERSION"

# 检查cyclonedx库
if python3 -c "import cyclonedx" 2>/dev/null; then
    echo "✅ cyclonedx库可用"
else
    echo "❌ cyclonedx库不可用"
    echo "💡 尝试安装cyclonedx..."
    pip3 install cyclonedx || echo "⚠️  cyclonedx安装失败，但继续运行"
fi

# 检查网络接口
echo "🌐 检查网络接口..."
if ip addr show eth0 >/dev/null 2>&1; then
    echo "✅ eth0接口存在"
else
    echo "⚠️  eth0接口不存在，使用默认网络"
fi

# 检查机器人连接（可选）
echo "🤖 检查机器人连接..."
if ping -c 1 192.168.123.15 >/dev/null 2>&1; then
    echo "✅ 机器人网络连接正常"
else
    echo "⚠️  机器人网络连接失败，将在模拟模式下运行"
fi

# 显示最终环境信息
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

# 启动G1客户端
python3 /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level/g1_client_now.py "$@"
