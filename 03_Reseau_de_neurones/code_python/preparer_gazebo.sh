#!/bin/bash
# Regenere l'URDF du bras 50 cm et la scene a trois receptacles, puis met a jour l'installation ROS.
cd "$(dirname "$0")"
~/venv_cao/bin/python ../bras_50cm/generer_urdf.py
python3 creer_scene.py
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select mon_bras_robot > /dev/null 2>&1 && echo "installation a jour"
grep -o 'name="joint_base_bras_[1-4]".\{0,260\}' ~/ros2_ws/src/mon_bras_robot/urdf/bras_50cm.urdf | grep -o 'joint_base_bras_[1-4]\|effort="[0-9.]*"'
