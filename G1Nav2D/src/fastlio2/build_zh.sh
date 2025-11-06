#!/bin/bash

# 定义常量：表示不同的 ROS 版本或标识
readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

# 记录并进入脚本所在目录（先保存当前目录到堆栈，然后切换到脚本目录）
pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

# 初始化变量
ROS_VERSION=""
ROS_HUMBLE=""

# 设置工作 ROS 版本：通过脚本的第一个参数判断
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    # 如果传入 "humble"，也认为是 ROS2，但同时标记 HUMBLE 变量用于 cmake 参数
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    # 参数不合法就退出（没有默认）
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION

# ---------- 清理上一次构建的产物 ----------
# 脚本假定当前目录结构是 scripts/...，这里使用相对路径回到仓库顶层并清理构建产物
# 这里直接强制删除 build/devel/install（危险：不可恢复），谨慎使用
rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/

# 如果 src/ 下有 CMakeLists.txt（可能是替换或临时生成的），就删除它
if [ -f ../CMakeLists.txt ]; then
    rm -f ../CMakeLists.txt
fi

# ---------- 替换 package.xml / launch 等文件，使源码与目标 ROS 版本匹配 ----------
# 根据选择的 ROS 版本，把预制的 package_ROS1.xml 或 package_ROS2.xml 覆盖到 package.xml
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    # 如果已存在 package.xml 先删除，然后复制 ROS1 版本的 package_ROS1.xml
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    # ROS2：替换 package.xml，并把 ROS2 专用的 launch_ROS2/ 文件夹复制为 launch/
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# ---------- 构建步骤 ----------
# 再次 pushd 当前路径（这里是脚本目录）；随后根据 ROS 版本选择不同的构建命令
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    # 对 ROS1，返回仓库顶层并执行 catkin_make
    cd ../../
    catkin_make -DROS_EDITION=${VERSION_ROS1} -j1
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    # 对 ROS2，使用 colcon build，传入 cmake 参数以区分 ROS_EDITION 及 HUMBLE_ROS（如果指定）
    cd ../../
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
fi
popd > /dev/null

# ---------- 构建完成后清理：删除临时复制的 launch/（如果是 ROS2 的替换） ----------
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

# 恢复最初的目录（popd 对应开头的 pushd）
popd > /dev/null
