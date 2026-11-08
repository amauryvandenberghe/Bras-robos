#!/bin/bash
# Arrete proprement tout Gazebo / ROS encore actif (utile si un ancien lancement tourne en arriere-plan :
# sinon le nouveau lancement affiche "Controller already loaded ... Failed to configure controller").
echo "avant :"; pgrep -fa 'ign gazebo|gz sim|robot_state_publisher|ros2 launch|parameter_bridge|ign-transport-topic' | cut -c1-120
pkill -INT -f 'ros2 launch mon_bras_robot' ; sleep 3
pkill -f 'ign gazebo' ; pkill -f 'robot_state_publisher' ; pkill -f 'ign-transport-topic' ; sleep 2
pkill -9 -f 'ign gazebo' ; pkill -9 -f 'robot_state_publisher'
source /opt/ros/humble/setup.bash && ros2 daemon stop > /dev/null 2>&1
echo "apres :"; pgrep -fa 'ign gazebo|gz sim|robot_state_publisher|ros2 launch' | cut -c1-120 || echo "  plus rien ne tourne"
