#!/bin/bash
# Entrainement du reseau du bras 50 cm (6 coeurs, priorite basse : le PC reste utilisable).
# Usage : bash entrainer.sh                 (premier entrainement, ~1 h 30)
#         bash entrainer.sh --reprendre 3   (tours de correction de plus)
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
OUVRIERS=${OUVRIERS:-6} nice -n 10 python3 -u -W ignore entrainer_imitation.py "$@"
