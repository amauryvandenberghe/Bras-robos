#!/bin/bash
# Diagnostic des servos dans Gazebo (postures les plus dures), puis 3 sequences pilotees par l'expert.
cd "$(dirname "$0")"
rm -f ~/ros2_ws/src/mon_bras_robot/urdf/bras_50cm_essai.urdf
python3 creer_scene.py
bash avec_gazebo.sh diag_servos.py
bash gazebo_auto.sh 3 --expert
