#!/bin/bash
# Complement d'apprentissage pour le parcours reel (balles devant le robot, depot balle tenue). ~1 h 15, priorite basse.
# Usage : bash entrainer_mission.sh [tours]      (sortie : modeles/mission/politique_mission.pt)
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
rm -rf modeles/mission_rapide
OUVRIERS=${OUVRIERS:-5} nice -n 10 python3 -u -W ignore entrainer_mission.py "${1:-5}"
