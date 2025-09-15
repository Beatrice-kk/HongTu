#!/bin/bash

# 测试正方形画笔功能

echo "🖌️  测试正方形橡皮擦画笔功能"
echo "========================================"

# 设置环境
source devel/setup.bash

# 检查测试地图是否存在
if [ ! -f "src/ros_map_edit/maps/test.yaml" ]; then
    echo "❌ 测试地图不存在，正在生成..."
    python3 src/py_pkg/scripts/create_test_map.py
fi

echo "✅ 测试地图已准备就绪"

# 启动roscore（如果没有运行）
if ! pgrep -x "roscore" > /dev/null; then
    echo "🚀 启动roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 3
else
    echo "✅ roscore已在运行"
    ROSCORE_PID=""
fi

# 启动地图服务器
echo "🗺️  启动地图服务器..."
rosrun map_server map_server src/ros_map_edit/maps/test.yaml &
MAP_SERVER_PID=$!
sleep 2

echo "📊 正方形画笔测试说明："
echo "========================================"
echo "画笔大小测试："
echo "• 1像素 = 1x1正方形（1个像素）"
echo "• 2像素 = 2x2正方形（4个像素）"
echo "• 3像素 = 3x3正方形（9个像素）"
echo "• 5像素 = 5x5正方形（25个像素）"
echo ""
echo "对齐规则："
echo "• 奇数大小（1,3,5...）：以点击位置为中心"
echo "• 偶数大小（2,4,6...）：向右下偏移"
echo ""
echo "🧪 测试步骤："
echo "1. 启动RViz："
echo "   rviz -d src/ros_map_edit/config/map_edit.rviz"
echo ""
echo "2. 选择橡皮擦工具（MapEraserTool）"
echo ""
echo "3. 测试不同画笔大小："
echo "   • 设置画笔大小为1，测试单像素"
echo "   • 设置画笔大小为2，验证2x2正方形"
echo "   • 设置画笔大小为3，验证3x3正方形"
echo "   • 设置画笔大小为5，验证5x5正方形"
echo ""
echo "4. 验证效果："
echo "   • 左键：画黑色（障碍物）"
echo "   • 右键：画白色（自由空间）"
echo "   • 拖拽：连续绘制"
echo ""
echo "5. 检查状态栏信息："
echo "   应该显示类似：'黑白橡皮擦 - 笔刷: 3x3像素, 左键:黑色 右键:白色'"
echo ""
echo "6. 测试Python版本（可选）："
echo "   python3 src/py_pkg/scripts/map_edit.py"
echo ""

# 创建测试日志
echo "🔍 系统状态检查："
echo "- ROS版本: $(rosversion -d)"
echo "- 工作目录: $(pwd)"
echo "- 地图文件: $(ls -la src/ros_map_edit/maps/test.* | wc -l) 个文件"

echo ""
echo "📈 预期结果："
echo "✓ 画笔大小1：只擦除点击的单个像素"
echo "✓ 画笔大小2：擦除2x2像素正方形区域"
echo "✓ 画笔大小3：擦除3x3像素正方形区域（以点击位置为中心）"
echo "✓ 画笔大小5：擦除5x5像素正方形区域（以点击位置为中心）"
echo "✓ 状态栏正确显示'NxN像素'格式"
echo "✓ 不再出现十字形或圆形效果"

# 清理函数
cleanup() {
    echo ""
    echo "🧹 清理进程..."
    if [ ! -z "$MAP_SERVER_PID" ]; then
        kill $MAP_SERVER_PID 2>/dev/null
    fi
    if [ ! -z "$ROSCORE_PID" ]; then
        kill $ROSCORE_PID 2>/dev/null
    fi
    echo "✅ 清理完成"
}

# 设置清理陷阱
trap cleanup EXIT

echo ""
echo "⏳ 按Ctrl+C退出测试..."

# 保持脚本运行，让用户可以测试
while true; do
    sleep 1
done 