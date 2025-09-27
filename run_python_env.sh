#!/bin/bash

# 完全复制当前目录Python环境的启动脚本
# 使用方法: ./run_python_env.sh [python脚本路径] [参数...]

# 设置脚本目录为工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 设置ROS环境变量
export ROS_DISTRO=noetic
export ROS_ROOT=/opt/ros/noetic/share/ros
export ROS_MASTER_URI=http://localhost:11311

# 设置Python环境变量
export PYTHONPATH=/home/unitree/HongTu/G1Nav2D/devel/lib/python3/dist-packages:/home/unitree/ros_noetic_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
export ROS_PYTHON_VERSION=3

# 设置路径环境变量
export PATH=/home/unitree/.local/bin:/usr/local/cuda-11.4/bin:/opt/ros/noetic/bin:/home/unitree/.cursor-server/bin/b753cece5c67c47cb5637199a5a5de2b7100c180/bin/remote-cli:/home/unitree/.local/bin:/usr/local/cuda-11.4/bin:/home/unitree/cyclonedds_ws/install/cyclonedds/bin:/opt/ros/foxy/bin:/home/unitree/.cursor-server/bin/b753cece5c67c47cb5637199a5a5de2b7100c180/bin/remote-cli:/home/unitree/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin

# 设置库路径
export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:/home/unitree/HongTu/G1Nav2D/devel/lib:/home/unitree/ros_noetic_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu:/usr/local/cuda-11.4/lib64:/home/unitree/cyclonedds_ws/install/cyclonedds/lib:/opt/ros/foxy/lib:/opt/ros/foxy/lib/aarch64-linux-gnu

# 设置包配置路径
export PKG_CONFIG_PATH=/home/unitree/HongTu/G1Nav2D/devel/lib/pkgconfig:/home/unitree/ros_noetic_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/aarch64-linux-gnu/pkgconfig:/home/unitree/cyclonedds_ws/install/cyclonedds/lib/pkgconfig:/opt/ros/foxy/lib/pkgconfig:/opt/ros/foxy/lib/aarch64-linux-gnu/pkgconfig

# 设置ROS包路径
export ROS_PACKAGE_PATH=/home/unitree/HongTu/G1Nav2D/src:/home/unitree/ros_noetic_ws/src:/opt/ros/noetic/share

# 设置CMAKE前缀路径
export CMAKE_PREFIX_PATH=/home/unitree/HongTu/G1Nav2D/devel:/home/unitree/ros_noetic_ws/devel:/opt/ros/noetic:/home/unitree/cyclonedds_ws/install/cyclonedds:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy

# 显示环境信息（可选）
echo "=== Python环境信息 ==="
echo "工作目录: $(pwd)"
echo "Python版本: $(python3 --version)"
echo "Python路径: $(which python3)"
echo "PYTHONPATH: $PYTHONPATH"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "========================"

# 直接运行g1_client_now.py
echo "启动G1客户端..."
python3 /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level/g1_client_now.py "$@"
