#!/bin/bash
# Démo, terminal 1 : ouvre Gazebo avec le bras 50 cm, la réglette et les 3 réceptacles.
# Arguments en plus passés au launch, ex. : bash demo_gazebo.sh gui:=false
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export IGN_IP=127.0.0.1     # communications Gazebo en local (sinon ~3 s par commande sous WSL : balles mal placees)
exec ros2 launch mon_bras_robot_moveit_config gazebo.launch.py robot:=bras_50cm monde:=scene_trois_receptacles "$@"
