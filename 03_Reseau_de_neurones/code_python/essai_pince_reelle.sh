#!/bin/bash
# Essai : balle de 2 g avec un couple de pince realiste (EFFORT_PINCE N.m par machoire), puis retour aux reglages normaux.
# Usage : bash essai_pince_reelle.sh 0.2 0.5
cd "$(dirname "$0")"
for e in "$@"; do
  echo "===== balle 2 g, couple de pince $e N.m ====="
  EFFORT_PINCE=$e ~/venv_cao/bin/python ../bras_50cm/generer_urdf.py > /dev/null
  MASSE_BALLE=0.002 python3 creer_scene.py > /dev/null
  bash avec_gazebo.sh essai_prise.py 2 2>&1 | grep -E 'essai|plan'
done
~/venv_cao/bin/python ../bras_50cm/generer_urdf.py > /dev/null
python3 creer_scene.py > /dev/null
echo "reglages normaux remis"
