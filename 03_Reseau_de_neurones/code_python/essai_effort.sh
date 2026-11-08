#!/bin/bash
# Essai : meme URDF avec un couple de coude multiplie par $1 (urdf/bras_50cm_essai.urdf), puis diag_coude.py.
cd ~/ros2_ws/src/mon_bras_robot
python3 - "$1" <<'FIN'
import re, sys
f = open('urdf/bras_50cm.urdf').read()
def remplace(m):
    return re.sub(r'effort="([0-9.]+)"', lambda e: f'effort="{float(e.group(1)) * float(sys.argv[1]):.3f}"', m.group(0))
f = re.sub(r'<joint name="joint_base_bras_3".*?</joint>', remplace, f, flags=re.S)
open('urdf/bras_50cm_essai.urdf', 'w').write(f)
print(re.search(r'<joint name="joint_base_bras_3".*?</joint>', f, re.S).group(0)[:400])
FIN
source /opt/ros/humble/setup.bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select mon_bras_robot > /dev/null 2>&1
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/mon_bras_robot/ia_50cm
bash arreter_gazebo.sh > /dev/null 2>&1
ros2 launch mon_bras_robot_moveit_config gazebo.launch.py gui:=false robot:=bras_50cm_essai \
    monde:=scene_trois_receptacles > /tmp/gazebo_bras50.log 2>&1 &
sleep 45
python3 -u -W ignore diag_coude.py
bash arreter_gazebo.sh > /dev/null 2>&1
