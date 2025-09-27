#!/bin/bash

# 完全复制在 /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level/ 目录下
# 运行 python g1_client_now.py 的效果

# 切换到目标目录
cd /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level

# 设置完整的环境变量（复制当前环境）
export CMAKE_PREFIX_PATH=/home/unitree/HongTu/G1Nav2D/devel:/home/unitree/ros_noetic_ws/devel:/opt/ros/noetic:/home/unitree/cyclonedds/install:/opt/ros/foxy/lib/aarch64-linux-gnu:/opt/ros/foxy

export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64:/home/unitree/HongTu/G1Nav2D/devel/lib:/home/unitree/ros_noetic_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu:/usr/local/cuda-11.4/lib64:/home/unitree/cyclonedds/install/lib:/opt/ros/foxy/lib:/opt/ros/foxy/lib/aarch64-linux-gnu

export PATH=/home/unitree/.local/bin:/usr/local/cuda-11.4/bin:/opt/ros/noetic/bin:/home/unitree/.cursor-server/bin/b753cece5c67c47cb5637199a5a5de2b7100c180/bin/remote-cli:/home/unitree/.local/bin:/usr/local/cuda-11.4/bin:/home/unitree/cyclonedds/install/bin:/opt/ros/foxy/bin:/home/unitree/.cursor-server/bin/b753cece5c67c47cb5637199a5a5de2b7100c180/bin/remote-cli:/home/unitree/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin

export PKG_CONFIG_PATH=/home/unitree/HongTu/G1Nav2D/devel/lib/pkgconfig:/home/unitree/ros_noetic_ws/devel/lib/pkgconfig:/opt/ros/noetic/lib/pkgconfig:/opt/ros/noetic/lib/aarch64-linux-gnu/pkgconfig:/home/unitree/cyclonedds/install/lib/pkgconfig:/opt/ros/foxy/lib/pkgconfig:/opt/ros/foxy/lib/aarch64-linux-gnu/pkgconfig

export PYTHONPATH=/home/unitree/HongTu/G1Nav2D/devel/lib/python3/dist-packages:/home/unitree/ros_noetic_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages

export ROS_DISTRO=noetic
export ROS_ETC_DIR=/opt/ros/noetic/etc/ros
export ROSLISP_PACKAGE_DIRECTORIES=/home/unitree/HongTu/G1Nav2D/devel/share/common-lisp:/home/unitree/ros_noetic_ws/devel/share/common-lisp
export ROS_MASTER_URI=http://localhost:11311
export ROS_PACKAGE_PATH=/home/unitree/HongTu/G1Nav2D/src:/home/unitree/ros_noetic_ws/src:/opt/ros/noetic/share
export ROS_PYTHON_VERSION=3
export ROS_ROOT=/opt/ros/noetic/share/ros
export ROS_VERSION=1

# 显示环境信息
echo "=== G1客户端启动脚本 ==="
echo "工作目录: $(pwd)"
echo "Python版本: $(python3 --version)"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "================================"

# 直接运行g1_client_now.py
echo "启动G1客户端..."
python3 g1_client_now.py "$@"
