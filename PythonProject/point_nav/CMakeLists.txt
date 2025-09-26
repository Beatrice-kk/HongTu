cmake_minimum_required(VERSION 3.0.2)
project(point_nav)

## 编译为catkin包
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib_msgs
  std_srvs
)

## 系统依赖
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin特定配置 ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs move_base_msgs actionlib_msgs std_srvs
)

###########
## 构建 ##
###########

## 指定其他头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 声明Python脚本
catkin_install_python(PROGRAMS
  src/dance_navigation_node.py
  scripts/simplified_nav_dance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## 安装配置文件
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

## 安装Python模块
install(DIRECTORY src/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)
