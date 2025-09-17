# bash ~/HongTu/py_contrl.sh &
# roslaunch fastlio navigation.launch map_file:=platform1.yaml pcd_path:=platform1.pcd &
# roslaunch g1_pose lidar_pose.launch &
# python3 ~/HongTu/PythonProject/point_nav/cwk_waypoint_platform1.py
#!/bin/bash

gnome-terminal -- bash -c "bash ~/HongTu/py_contrl.sh; exec bash"
gnome-terminal -- bash -c "roslaunch fastlio navigation.launch map_file:=platform1.yaml pcd_path:=platform1.pcd; exec bash"
gnome-terminal -- bash -c "roslaunch g1_pose lidar_pose.launch; exec bash"
gnome-terminal -- bash -c "python3 ~/HongTu/PythonProject/point_nav/cwk_waypoint_platform1.py; exec bash"
