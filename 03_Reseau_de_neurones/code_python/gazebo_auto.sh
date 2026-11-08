#!/bin/bash
# Lance Gazebo SANS fenetre (bras 50 cm, scene a trois receptacles) puis evaluer_receptacles.py avec les memes options.
# Si Gazebo plante ou se fige, il est relance et la serie reprend la ou elle en etait.
# Usage : bash gazebo_auto.sh 20                                (test, 20 sequences pilotees par le reseau)
#         bash gazebo_auto.sh 5 --expert                        (l'expert pilote : verification de la physique)
#         bash gazebo_auto.sh 30 --collecter --graine 50000000  (corrections de l'expert, vraie physique)
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# communications Gazebo en local : sinon (reseau WSL) chaque commande prend ~3 s et les balles sont mal placees
export IGN_IP=127.0.0.1
REPRENDRE=""
for essai in $(seq 1 12); do
  bash arreter_gazebo.sh > /dev/null 2>&1
  echo "[$(date +%H:%M:%S)] lancement de Gazebo sans fenetre (essai $essai)"
  ros2 launch mon_bras_robot_moveit_config gazebo.launch.py gui:=false robot:=bras_50cm \
      monde:=scene_trois_receptacles > /tmp/gazebo_bras50.log 2>&1 &
  sleep 45                      # chargement du monde et des controleurs
  python3 -u -W ignore evaluer_receptacles.py "$@" $REPRENDRE
  code=$?
  REPRENDRE="--reprendre"
  [ $code -ne 3 ] && break
  echo "[$(date +%H:%M:%S)] Gazebo s'est fige ou a plante (fin du journal ci-dessous) : relance"
  tail -3 /tmp/gazebo_bras50.log
done
bash arreter_gazebo.sh > /dev/null 2>&1
echo "[$(date +%H:%M:%S)] termine"
