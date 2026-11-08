#!/bin/bash
# Démo : ramène le bras de Gazebo en position home (posture de repos, pince ouverte).
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export IGN_IP=127.0.0.1     # meme reglage que demo_gazebo.sh
exec python3 aller_repos_gazebo.py
