# bash ~/HongTu/py_contrl.sh &
# roslaunch fastlio navigation.launch map_file:=platform2.yaml pcd_path:=platform2.pcd &
# roslaunch g1_pose lidar_pose.launch &
# python3 ~/HongTu/PythonProject/point_nav/cwk_waypoint_platform2.py
#!/bin/bash

gnome-terminal -- bash -c "bash ~/HongTu/py_contrl.sh; exec bash"
gnome-terminal -- bash -c "roslaunch fastlio navigation.launch map_file:=platform2.yaml pcd_path:=platform2.pcd; exec bash"
gnome-terminal -- bash -c "roslaunch g1_pose lidar_pose.launch; exec bash"
gnome-terminal -- bash -c "python3 ~/HongTu/PythonProject/point_nav/cwk_waypoint_platform2.py; exec bash"
