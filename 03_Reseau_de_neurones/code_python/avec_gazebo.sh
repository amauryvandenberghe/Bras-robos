#!/bin/bash
# Lance Gazebo sans fenetre (bras 50 cm, trois receptacles), execute un script Python, puis arrete Gazebo.
# Usage : bash avec_gazebo.sh diag_servos.py [options]
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
bash arreter_gazebo.sh > /dev/null 2>&1
ros2 launch mon_bras_robot_moveit_config gazebo.launch.py gui:=false robot:=bras_50cm \
    monde:=scene_trois_receptacles > /tmp/gazebo_bras50.log 2>&1 &
sleep 45
python3 -u -W ignore "$@"
bash arreter_gazebo.sh > /dev/null 2>&1
