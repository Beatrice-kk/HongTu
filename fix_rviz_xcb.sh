#!/bin/bash
# rviz xcb 修复脚本（适用于 Ubuntu20.04 + ROS Noetic）

echo "=== 检查并修复 rviz Qt xcb 插件问题 ==="

# 1. 更新软件源
sudo apt update

# 2. 安装常见依赖
sudo apt install -y \
  libxcb-xinerama0 libxcb-xinerama0-dev \
  libxcb-util1 libxrender1 libxi6 libsm6 libxrandr2 \
  libxkbcommon-x11-0 \
  qt5-default qtbase5-dev qtbase5-dev-tools

# 3. 检查插件目录
PLUGIN_DIR="/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms"
if [ -f "$PLUGIN_DIR/libqxcb.so" ]; then
    echo "[OK] 找到 libqxcb.so 插件"
else
    echo "[ERR] 没找到 libqxcb.so，可能 Qt 没装全"
fi

# 4. 设置环境变量
export QT_QPA_PLATFORM_PLUGIN_PATH=$PLUGIN_DIR
echo "已设置 QT_QPA_PLATFORM_PLUGIN_PATH=$PLUGIN_DIR"

# 5. 检查 DISPLAY
if [ -z "$DISPLAY" ]; then
    echo "[WARN] \$DISPLAY 为空，可能是 ssh 或无图形界面"
else
    echo "[OK] 当前 DISPLAY=$DISPLAY"
fi

echo "=== 修复完成，试试运行 rviz ==="
