#!/bin/bash
# tmux 会话名称
SESSION="robot_run"

# 创建新的 tmux 会话
tmux new-session -d -s $SESSION

tmux new-window -t $SESSION:1 -n "g1_ctrl"
tmux send-keys -t $SESSION:1 "source /opt/ros/noetic/setup.bash && python3 /home/unitree/HongTu/unitree_sdk2_python/example/g1/high_level/g1_ctrl_cwk.py" C-m

sleep 3

tmux new-window -t $SESSION:2 -n "navigation"
tmux send-keys -t $SESSION:2 "roslaunch fastlio navigation.launch use_rviz:=false" C-m

sleep 10

tmux new-window -t $SESSION:3 -n "test_update"
tmux send-keys -t $SESSION:3 "source /opt/ros/noetic/setup.bash && python3 /home/unitree/HongTu/PythonProject/point_nav/test_update.py" C-m

echo "tmux 会话 $SESSION 已创建并启动所有任务"
echo "使用 'tmux attach -t $SESSION' 查看运行情况"
