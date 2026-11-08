#!/bin/bash
# Comme valider_mission_gazebo.sh, pour n'importe quel reseau (resultats dans modeles/mission/, suffixe = nom du reseau).
# Usage : bash valider_gazebo_modele.sh <modele> [n sequences par test, defaut 20]
#   ex. : bash valider_gazebo_modele.sh modeles/politique_finale.pt 20   (comparaison avec le reseau final)
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
# communications Gazebo en local : sinon (reseau WSL) chaque commande prend ~3 s et les balles sont mal placees
export IGN_IP=127.0.0.1
MODELE=$1
N=${2:-20}
for SCENE in officielle devant; do
  REPRENDRE=""
  for essai in $(seq 1 12); do
    bash arreter_gazebo.sh > /dev/null 2>&1
    echo "[$(date +%H:%M:%S)] $MODELE, scene $SCENE : lancement de Gazebo sans fenetre (essai $essai)"
    nice -n 10 ros2 launch mon_bras_robot_moveit_config gazebo.launch.py gui:=false robot:=bras_50cm \
        monde:=scene_trois_receptacles > /tmp/gazebo_bras50.log 2>&1 &
    sleep 45
    nice -n 10 python3 -u -W ignore evaluer_mission_gazebo.py "$N" --scene $SCENE --modele "$MODELE" $REPRENDRE
    code=$?
    REPRENDRE="--reprendre"
    [ $code -ne 3 ] && break
    echo "[$(date +%H:%M:%S)] Gazebo s'est fige ou a plante : relance"
    tail -3 /tmp/gazebo_bras50.log
  done
done
bash arreter_gazebo.sh > /dev/null 2>&1
echo "[$(date +%H:%M:%S)] termine"
