#!/bin/bash

USER=unitree
PASS=123

# 每台机器人 IP 和对应程序路径
# LIDAR Go2
LIDARGO2=143.215.103.206
# CMD1="/home/unitree/Desktop/Research/unitree_sdk2/build/bin/go2_pid"
CMD1="cd ~/Desktop/Research/unitree_sdk2/build/bin && ./go2_pid eth0 > output.log"

# Lu's go2
LUNARGO2=143.215.105.115
# CMD1="/home/unitree/Desktop/Research/unitree_sdk2/build/bin/go2_pid"
CMD2="cd /home/unitree/go2_hardware_exp/build/bin && ./go2_pid eth0 > output.log"

ALIENGO=143.215.96.113
CMD3="cd /home/unitree/Aiengo_hardware_exp/build && ./Alien_waypoint_follower > output.log"

# 启动每台机器人的程序（后台运行）
# robot2 in planner
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$LIDARGO2 "$CMD1" &
# robot1 in planner
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$LUNARGO2 "$CMD2" &
# robot3 in planner
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$ALIENGO "$CMD3" &

wait
echo "All robot programs started."
