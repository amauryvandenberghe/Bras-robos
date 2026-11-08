#!/bin/bash
# Affinage Gazebo du reseau complete, comme pour le reseau final (fin_automatique.sh) :
#   1. 20 sequences par scene (officielle, devant) pilotees par politique_mission.pt, l'expert note ses corrections
#      (graines 52 000 000 et 51 000 000 : jamais celles du test) ;
#   2. affinage (affiner_mission.py) -> modeles/mission/politique_mission_affine.pt ;
#   3. test Gazebo du reseau affine sur les memes scenes que le test du reseau complete (graines 40 000 000 et 41 000 000).
# Usage : bash affinage_mission.sh
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# communications Gazebo en local : sinon (reseau WSL) chaque commande prend ~3 s et les balles sont mal placees
export IGN_IP=127.0.0.1

gazebo_serie() {   # gazebo_serie <scene> <arguments d'evaluer_mission_gazebo.py...> (relance Gazebo s'il se fige)
  local SCENE=$1; shift
  local REPRENDRE=""
  for essai in $(seq 1 12); do
    bash arreter_gazebo.sh > /dev/null 2>&1
    echo "[$(date +%H:%M:%S)] scene $SCENE $* : lancement de Gazebo sans fenetre (essai $essai)"
    nice -n 10 ros2 launch mon_bras_robot_moveit_config gazebo.launch.py gui:=false robot:=bras_50cm \
        monde:=scene_trois_receptacles > /tmp/gazebo_bras50.log 2>&1 &
    sleep 45
    nice -n 10 python3 -u -W ignore evaluer_mission_gazebo.py --scene $SCENE "$@" $REPRENDRE
    local code=$?
    REPRENDRE="--reprendre"
    [ $code -ne 3 ] && break
    echo "[$(date +%H:%M:%S)] Gazebo s'est fige ou a plante : relance"
  done
  bash arreter_gazebo.sh > /dev/null 2>&1
}

echo "[$(date +%H:%M)] 1. corrections de l'expert dans Gazebo"
rm -f modeles/mission/donnees_gazebo_mission.npz
gazebo_serie officielle 20 --collecter --graine 52000000
gazebo_serie devant 20 --collecter --graine 51000000

echo "[$(date +%H:%M)] 2. affinage"
OUVRIERS=6 nice -n 10 python3 -u -W ignore affiner_mission.py

echo "[$(date +%H:%M)] 3. test Gazebo du reseau affine"
gazebo_serie officielle 20 --modele modeles/mission/politique_mission_affine.pt
gazebo_serie devant 20 --modele modeles/mission/politique_mission_affine.pt
echo "[$(date +%H:%M)] termine"
