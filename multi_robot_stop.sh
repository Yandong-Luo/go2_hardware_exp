#!/bin/bash

USER=unitree
PASS=123

# IPs of robots
LIDARGO2=143.215.103.206   # LIDAR Go2
LUNARGO2=143.215.105.115   # LUNAR Go2
ALIENGO=143.215.96.113   # AlienGo

# 停止每个程序
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$LIDARGO2 "pkill -f go2_pid" &
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$LUNARGO2 "pkill -f go2_pid" &
sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$ALIENGO "pkill -f Alien_waypoint_follower" &
# sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$LUNARGO2 "pkill -f start_motion.sh" &
# sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no $USER@$ROBOT3 "pkill -f start_ai_follow.sh" &

wait
echo "All robot programs stopped."
