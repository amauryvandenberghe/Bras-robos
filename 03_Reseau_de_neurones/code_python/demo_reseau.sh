#!/bin/bash
# Démo, terminal 2 (Gazebo déjà ouvert par demo_gazebo.sh) : le réseau final dépose les balles,
# dispositions tirées au hasard. 1er argument = nombre de séquences de 3 balles (3 par défaut),
# les suivants vont à evaluer_receptacles.py, ex. : bash demo_reseau.sh 5 --trace   ou   bash demo_reseau.sh 3 --expert
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export IGN_IP=127.0.0.1     # meme reglage que demo_gazebo.sh
N=${1:-3}
[ $# -gt 0 ] && shift
exec python3 evaluer_receptacles.py "$N" --modele modeles/politique_finale.pt --graine $RANDOM$RANDOM "$@"
